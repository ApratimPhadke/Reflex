/* Reflex I/O node - ball-and-beam sensor/actuator bridge with a last-line watchdog.
 *
 * Builds for an STM32 "Black Pill" (STM32duino core, USB support = CDC) and for a
 * genuine Arduino Uno/Nano. Library needed: "VL53L0X" by Pololu.
 * Wiring and geometry: docs/HARDWARE.md.
 *
 * Link to the host, protocol v2 (115200 baud, ASCII lines; parser in src/core/rx_link.c):
 *     node -> host   "M <raw_mm>"  raw sensor distance, every 20 ms   ("M !" = no valid reading)
 *                    "I reflex_io 2 <board> <emulated 0|1>"   on reset and on "?"
 *                    "T <theta_mrad>"  emulator only: the true beam angle
 *                    "# ..."           comment
 *     host -> node   "S <pulse_us>"  servo pulse        "L <pulse_us>"  level pulse (kept in flash)
 *                    "?"             say who you are    "E pin <mm>" | "E free" | "E remove"  emulator only
 * The node knows nothing about millimetres along the beam or radians: all calibration lives on the
 * host in reflex.conf, written by the rxcal wizard. Nothing here needs editing to calibrate a rig.
 *
 * Third ring of defence, independent of the host:
 *   - if no "S" arrives for WATCHDOG_MS the node drives the stored level pulse and blinks the LED;
 *   - every pulse is clamped to SERVO_MIN_US..SERVO_MAX_US, which no host command can change;
 *   - the level pulse is written to flash only while the beam is parked by the watchdog (an STM32
 *     flash erase stalls the CPU for a second or two) and only when it changed.
 * The loop never blocks: the sensor is polled for "data ready" and output is only written when
 * the USB/serial buffer has room, so the watchdog runs every pass whatever the host is doing.
 *
 * REFLEX_EMULATE=1 (make fw EMULATE=1): no sensor, no servo. The board simulates the same
 * ball-and-beam as rx_plant_step() in src/core/rx_core.c, behind a deliberately non-default hidden
 * calibration (EMU_* below) that rxcal has to find, and reports it with sensor-like noise.
 * The watchdog and the clamps behave exactly as in the real build.
 *
 * SPDX-License-Identifier: MIT */
#ifndef REFLEX_EMULATE
#define REFLEX_EMULATE 0
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
#if defined(REFLEX_BOARD_ID)
#define RX_BOARD_NAME STR(REFLEX_BOARD_ID)        /* scripts/fw.sh passes BOARD */
#elif defined(BOARD_NAME)
#define RX_BOARD_NAME BOARD_NAME                  /* STM32duino's own name for the variant */
#else
#define RX_BOARD_NAME "UNKNOWN"
#endif
#define PROTO 2

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#if !REFLEX_EMULATE
#include <VL53L0X.h>
#endif

/* ---------------- link to the Raspberry Pi -------------------------------------------------------
 * 0 = USB cable (default). 1 = hardware UART on PA9 (TX) / PA10 (RX), 3.3 V, to the Pi's GPIO UART.
 * Use 1 only if QNX on the Pi does not create a serial device for the Black Pill. */
#define LINK_UART 0
#if LINK_UART
#define LINK Serial1
#else
#define LINK Serial
#endif

#if defined(ARDUINO_ARCH_STM32)
const int SERVO_PIN = PA8;
#else
const int SERVO_PIN = 9;
#endif
/* ---------------- hard limits: not calibration, never loosen to "make it work" ---------------- */
const int   SERVO_MIN_US      = 1000;   /* servo pulse clamp, independent of every host command     */
const int   SERVO_MAX_US      = 2000;   /* narrow these if the linkage binds before the servo's end */
const int   DEFAULT_LEVEL_US  = 1500;   /* level pulse until the host has stored one with "L"       */
const unsigned long WATCHDOG_MS = 150;
const unsigned long REPORT_MS   = 20;   /* 50 Hz */
const int   SENSOR_MAX_MM     = 2000;   /* readings beyond this are reported as "M !"              */
/* --------------------------------------------------------------------------------------------- */

Servo servo;
unsigned long lastCmdMs = 0, lastSendMs = 0;
char line[32];
uint8_t len = 0;
bool overflow = false;
int pulseUs = DEFAULT_LEVEL_US;         /* clamped pulse currently applied */
int levelUs = DEFAULT_LEVEL_US;         /* level pulse used by the watchdog */
int storedLevelUs = -1;                 /* what flash holds (-1 = nothing valid) */

/* ---------------- level pulse in flash (EEPROM emulation on STM32, real EEPROM on AVR) --------- */
const uint8_t EE_MAGIC0 = 'R', EE_MAGIC1 = 'L';
static uint8_t eeRead(int a)
{
#if defined(ARDUINO_ARCH_STM32)
  return eeprom_buffered_read_byte(a);
#else
  return EEPROM.read(a);
#endif
}
static void loadLevel()
{
#if defined(ARDUINO_ARCH_STM32)
  eeprom_buffer_fill();                  /* copy the flash page into RAM once */
#endif
  uint8_t m0 = eeRead(0), m1 = eeRead(1), lo = eeRead(2), hi = eeRead(3), ck = eeRead(4);
  int v = lo | (hi << 8);
  if (m0 == EE_MAGIC0 && m1 == EE_MAGIC1 && ck == (uint8_t)(lo ^ hi ^ 0x5A) && v >= SERVO_MIN_US && v <= SERVO_MAX_US) {
    levelUs = storedLevelUs = v;
  }
}
static void saveLevel(int v)            /* one sector erase + program; call only while parked */
{
  uint8_t lo = v & 0xFF, hi = (v >> 8) & 0xFF, b[5] = { EE_MAGIC0, EE_MAGIC1, lo, hi, (uint8_t)(lo ^ hi ^ 0x5A) };
#if defined(ARDUINO_ARCH_STM32)
  for (int i = 0; i < 5; i++) eeprom_buffered_write_byte(i, b[i]);
  eeprom_buffer_flush();
#else
  for (int i = 0; i < 5; i++) EEPROM.update(i, b[i]);
#endif
  storedLevelUs = v;
}

static int clampPulse(long us) { return us < SERVO_MIN_US ? SERVO_MIN_US : (us > SERVO_MAX_US ? SERVO_MAX_US : (int)us); }

void setPulse(long us)
{
  pulseUs = clampPulse(us);
#if !REFLEX_EMULATE
  servo.writeMicroseconds(pulseUs);
#endif
}

/* write a whole line or nothing: never wait for a host that is not reading */
static void sendLine(const char *s)
{
  size_t n = strlen(s);
  if (LINK.availableForWrite() < (int)(n + 2)) return;
  LINK.print(s);
  LINK.print("\r\n");
}

static void sendInfo()
{
  char b[64];
  snprintf(b, sizeof b, "I reflex_io %d %s %d", PROTO, RX_BOARD_NAME, REFLEX_EMULATE ? 1 : 0);
  sendLine(b);
}

/* strict "<c> <int>" parser */
static bool parseCmd(const char *s, char c, long *out)
{
  if (s[0] != c || s[1] != ' ') return false;
  s += 2;
  while (*s == ' ') s++;
  bool neg = false;
  if (*s == '-' || *s == '+') { neg = *s == '-'; s++; }
  long v = 0; int nd = 0;
  while (*s >= '0' && *s <= '9') { if (++nd > 6) return false; v = v * 10 + (*s - '0'); s++; }
  while (*s == ' ') s++;
  if (nd == 0 || *s) return false;
  *out = neg ? -v : v;
  return true;
}

#if REFLEX_EMULATE
/* ---------------- emulated plant: same model and constants as rx_plant_step() ---------------- */
/* hidden calibration of the emulated rig (what rxcal has to find; deliberately not the defaults) */
const int   EMU_LEVEL_US = 1520, EMU_SERVO_DIR = -1, EMU_CENTER_MM = 255, EMU_SENSOR_DIR = +1, EMU_NOBALL_MM = 560;
const float EMU_US_PER_MRAD = 2.0f;
const float K_ACC = 5.886f, SLEW = 6.0f, ROLL_LOSS = 0.15f, HALF_LEN = 0.20f, NOISE_MM = 3.0f, THETA_MAX_MECH = 0.230f;
float ex = 0.03f, ev = 0.0f, eth = 0.0f;
unsigned long lastStepUs = 0, lostUntilMs = 0;
bool eLost = false, eRemoved = false, ePinned = false;
float ePin = 0.0f;
uint32_t rngState = 0x9E3779B9u;
static float urand() { rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5; return (rngState >> 8) * (1.0f / 16777216.0f); }

static float targetAngle()
{
  float th = EMU_SERVO_DIR * (pulseUs - EMU_LEVEL_US) / EMU_US_PER_MRAD * 0.001f;
  return th > THETA_MAX_MECH ? THETA_MAX_MECH : (th < -THETA_MAX_MECH ? -THETA_MAX_MECH : th);
}

static void emuStep()
{
  unsigned long now = micros();
  unsigned long dus = now - lastStepUs;
  if (dus < 500) return;
  if (dus > 20000) dus = 20000;            /* never integrate a long gap in one go */
  lastStepUs = now;
  float target = targetAngle();
  for (unsigned long t = 0; t + 500 <= dus; t += 500) {
    const float h = 0.0005f;
    float d = target - eth, lim = SLEW * h;
    eth += d > lim ? lim : (d < -lim ? -lim : d);
    if (eLost || eRemoved) continue;
    if (ePinned) { ex = ePin; ev = 0; continue; }
    float a = K_ACC * sinf(eth) - ROLL_LOSS * ev;
    ev += a * h;
    ex += ev * h;
    if (fabsf(ex) > HALF_LEN) { eLost = true; lostUntilMs = millis() + 1000; ex = ex > 0 ? HALF_LEN : -HALF_LEN; ev = 0; }
  }
  if (eLost && (long)(millis() - lostUntilMs) >= 0) { eLost = false; ex = 0; ev = 0; }
}

static void emuCommand(const char *s)    /* "E pin <mm>" | "E free" | "E remove" */
{
  long mm;
  if (!strncmp(s, "E pin ", 6)) { char b[24]; snprintf(b, sizeof b, "P %s", s + 6); if (parseCmd(b, 'P', &mm)) { ePinned = true; eRemoved = false; eLost = false; ePin = mm * 0.001f; ex = ePin; ev = 0; } }
  else if (!strcmp(s, "E free")) { if (eRemoved || eLost) { ex = 0; ev = 0; } ePinned = eRemoved = eLost = false; }
  else if (!strcmp(s, "E remove")) { eRemoved = true; ePinned = false; }
}
#else
VL53L0X tof;
bool sensorOk = false;
int lastMm = -1;                         /* newest valid reading, -1 = none */
unsigned long lastReadingMs = 0;
#endif

static void handleLine(const char *s)
{
  long v;
  if (parseCmd(s, 'S', &v)) { setPulse(v); lastCmdMs = millis(); }
  else if (parseCmd(s, 'L', &v)) { levelUs = clampPulse(v); }     /* saved to flash once parked */
  else if (!strcmp(s, "?")) sendInfo();
#if REFLEX_EMULATE
  else if (s[0] == 'E' && s[1] == ' ') emuCommand(s);
#endif
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  LINK.begin(115200);
  loadLevel();
#if REFLEX_EMULATE
  lastStepUs = micros();
#else
  servo.attach(SERVO_PIN);
#endif
  setPulse(levelUs);
#if !REFLEX_EMULATE
  Wire.begin();
  tof.setTimeout(100);
  sensorOk = tof.init();
  if (sensorOk) {
    tof.setMeasurementTimingBudget(20000);         /* 20 ms per reading -> 50 Hz */
    tof.startContinuous();
  }
#endif
  sendInfo();
}

void loop()
{
  /* 1. commands from the host */
  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\n' || c == '\r') {
      line[len] = 0;
      if (!overflow && len > 0) handleLine(line);
      len = 0; overflow = false;
    } else if (overflow) {
    } else if (len < sizeof(line) - 1) line[len++] = c;
    else overflow = true;
  }

  /* 2. watchdog: nobody is commanding us -> level the beam; persist a new level pulse while parked */
  bool orphan = millis() - lastCmdMs > WATCHDOG_MS;
  if (orphan) {
    setPulse(levelUs);
    if (levelUs != storedLevelUs && millis() - lastCmdMs > WATCHDOG_MS + 500) saveLevel(levelUs);
  }
  digitalWrite(LED_BUILTIN, orphan ? ((millis() / 100) & 1) : LOW);

#if REFLEX_EMULATE
  emuStep();
#else
  /* sensor: only read when a measurement is ready, so this never waits on I2C polling */
  if (sensorOk && (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07)) {
    int mm = (int)tof.readRangeContinuousMillimeters();   /* data is ready: returns at once */
    lastReadingMs = millis();
    lastMm = (tof.timeoutOccurred() || mm <= 0 || mm > SENSOR_MAX_MM) ? -1 : mm;
  }
#endif

  /* 3. raw reading at 50 Hz */
  if (millis() - lastSendMs >= REPORT_MS) {
    lastSendMs = millis();
    char out[24];
#if REFLEX_EMULATE
    if (eLost || eRemoved) snprintf(out, sizeof out, "M %ld", lroundf(EMU_NOBALL_MM + (urand() * 2.0f - 1.0f) * NOISE_MM));
    else {
      long raw = lroundf(EMU_CENTER_MM + EMU_SENSOR_DIR * ex * 1000.0f + (urand() * 2.0f - 1.0f) * NOISE_MM);
      snprintf(out, sizeof out, "M %ld", raw < 0 ? 0L : (raw > SENSOR_MAX_MM ? (long)SENSOR_MAX_MM : raw));
    }
    sendLine(out);
    snprintf(out, sizeof out, "T %ld", lroundf(eth * 1000.0f));
    sendLine(out);
#else
    bool fresh = sensorOk && millis() - lastReadingMs < 100;
    if (!fresh || lastMm < 0) strcpy(out, "M !");
    else snprintf(out, sizeof out, "M %d", lastMm);
    sendLine(out);
#endif
  }

  /* 4. say who we are every 2 s (comment line, ignored by the host's parser) */
  static unsigned long lastHello = 0;
  if (millis() - lastHello >= 2000) {
    lastHello = millis();
#if REFLEX_EMULATE
    sendLine("# reflex_io v2 emulate=1");
#else
    sendLine(sensorOk ? "# reflex_io v2 sensor=ok" : "# reflex_io v2 VL53L0X NOT FOUND - check wiring");
#endif
  }
}
