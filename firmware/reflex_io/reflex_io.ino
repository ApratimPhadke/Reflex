/* Reflex I/O node - ball-and-beam sensor/actuator bridge with a last-line watchdog.
 *
 * Builds for an STM32 "Black Pill" (STM32duino core, USB support = CDC) and for a
 * genuine Arduino Uno/Nano. Library needed: "VL53L0X" by Pololu.
 * Wiring and geometry: docs/HARDWARE.md.
 *
 * Link to the guardian (115200 baud, ASCII lines, parser in src/core/rx_link.c):
 *     node -> guardian   "P <position_mm>"   every 20 ms   ("P !" = no ball seen, "# ..." = comment)
 *     guardian -> node   "U <beam_angle_millirad>"
 *
 * Third ring of defence, independent of the host:
 *   - if no "U" arrives for WATCHDOG_MS the node levels the beam on its own and blinks the LED;
 *   - every command is clamped to +/-MAX_MRAD and the servo pulse to SERVO_MIN_US..SERVO_MAX_US.
 * The loop never blocks: the sensor is polled for "data ready" and output is only written when
 * the USB/serial buffer has room, so the watchdog runs every pass whatever the host is doing.
 *
 * REFLEX_EMULATE=1 (make fw EMULATE=1): no sensor, no servo. The board simulates the same
 * ball-and-beam as rx_plant_step() in src/core/rx_core.c and reports it with sensor-like noise,
 * speaking the identical protocol, so the whole chain Pi -> USB -> board can be tested before
 * the mechanics exist. The watchdog and the clamps behave exactly as in the real build.
 *
 * SPDX-License-Identifier: MIT */
#ifndef REFLEX_EMULATE
#define REFLEX_EMULATE 0
#endif

#include <Wire.h>
#include <Servo.h>
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

/* ---------------- calibrate these for YOUR build (docs/HARDWARE.md, part 7) ---------------- */
#if defined(ARDUINO_ARCH_STM32)
const int SERVO_PIN = PA8;
#else
const int SERVO_PIN = 9;
#endif
const int   SERVO_CENTER_US   = 1500;   /* pulse width at which the beam is level                  */
const float US_PER_MILLIRAD   = 1.9f;   /* servo microseconds per milliradian of BEAM angle        */
const int   SERVO_DIR         = +1;     /* flip to -1 if a positive command tilts the wrong way    */
const int   CENTER_MM         = 260;    /* sensor reading when the ball is at the beam centre      */
const int   SENSOR_DIR        = +1;     /* flip to -1 so that +command makes position INCREASE     */
const int   RANGE_MIN_MM      = 40;     /* readings outside this window mean "no ball"             */
const int   RANGE_MAX_MM      = 495;    /* must be BELOW what the sensor reads with no ball on the beam */
/* ---------------- hard limits: not calibration, never loosen to "make it work" ---------------- */
const long  MAX_MRAD          = 230;    /* beam command clamp, independent of the guardian          */
const int   SERVO_MIN_US      = 1000;   /* servo pulse clamp, independent of every constant above   */
const int   SERVO_MAX_US      = 2000;
const unsigned long WATCHDOG_MS = 150;
const unsigned long REPORT_MS   = 20;   /* 50 Hz */
/* --------------------------------------------------------------------------------------------- */

Servo servo;
unsigned long lastCmdMs = 0, lastSendMs = 0;
char line[24];
uint8_t len = 0;
bool overflow = false;
long beamMrad = 0;                      /* clamped command currently applied */

#if REFLEX_EMULATE
/* ---------------- emulated plant: same model and constants as rx_plant_step() ---------------- */
const float K_ACC = 5.886f, SLEW = 6.0f, ROLL_LOSS = 0.15f, HALF_LEN = 0.20f, NOISE_MM = 3.0f;
float ex = 0.03f, ev = 0.0f, eth = 0.0f;
unsigned long lastStepUs = 0, lostUntilMs = 0;
bool eLost = false;
uint32_t rngState = 0x9E3779B9u;
static float urand() { rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5; return (rngState >> 8) * (1.0f / 16777216.0f); }

static void emuStep()
{
  unsigned long now = micros();
  unsigned long dus = now - lastStepUs;
  if (dus < 500) return;
  if (dus > 20000) dus = 20000;            /* never integrate a long gap in one go */
  lastStepUs = now;
  float target = beamMrad * 0.001f;
  for (unsigned long t = 0; t + 500 <= dus; t += 500) {
    const float h = 0.0005f;
    float d = target - eth, lim = SLEW * h;
    eth += d > lim ? lim : (d < -lim ? -lim : d);
    if (eLost) continue;
    float a = K_ACC * sinf(eth) - ROLL_LOSS * ev;
    ev += a * h;
    ex += ev * h;
    if (fabsf(ex) > HALF_LEN) { eLost = true; lostUntilMs = millis() + 1000; ex = ex > 0 ? HALF_LEN : -HALF_LEN; ev = 0; }
  }
  if (eLost && (long)(millis() - lostUntilMs) >= 0) { eLost = false; ex = 0; ev = 0; }
}
#else
VL53L0X tof;
bool sensorOk = false;
int lastMm = -1;                         /* newest valid reading, -1 = none */
unsigned long lastReadingMs = 0;
#endif

void setBeam(long mrad)
{
  if (mrad > MAX_MRAD) mrad = MAX_MRAD;
  if (mrad < -MAX_MRAD) mrad = -MAX_MRAD;
  beamMrad = mrad;
  long us = SERVO_CENTER_US + SERVO_DIR * (long)(mrad * US_PER_MILLIRAD);
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
#if !REFLEX_EMULATE
  servo.writeMicroseconds((int)us);
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

/* strict "U <int>" parser; anything else is ignored */
static bool parseU(const char *s, long *out)
{
  if (s[0] != 'U' || s[1] != ' ') return false;
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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  LINK.begin(115200);
#if REFLEX_EMULATE
  lastStepUs = micros();
#else
  servo.attach(SERVO_PIN);
#endif
  setBeam(0);
#if !REFLEX_EMULATE
  Wire.begin();
  tof.setTimeout(100);
  sensorOk = tof.init();
  if (sensorOk) {
    tof.setMeasurementTimingBudget(20000);         /* 20 ms per reading -> 50 Hz */
    tof.startContinuous();
  }
#endif
}

void loop()
{
  /* 1. commands from the guardian */
  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\n' || c == '\r') {
      line[len] = 0;
      long m;
      if (!overflow && len > 0 && parseU(line, &m)) { setBeam(m); lastCmdMs = millis(); }
      len = 0; overflow = false;
    } else if (overflow) {
    } else if (len < sizeof(line) - 1) line[len++] = c;
    else overflow = true;
  }

  /* 2. watchdog: nobody is talking to us -> level the beam */
  bool orphan = millis() - lastCmdMs > WATCHDOG_MS;
  if (orphan) setBeam(0);
  digitalWrite(LED_BUILTIN, orphan ? ((millis() / 100) & 1) : LOW);

#if REFLEX_EMULATE
  emuStep();
#else
  /* sensor: only read when a measurement is ready, so this never waits on I2C polling */
  if (sensorOk && (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07)) {
    int mm = (int)tof.readRangeContinuousMillimeters();   /* data is ready: returns at once */
    lastReadingMs = millis();
    lastMm = (tof.timeoutOccurred() || mm < RANGE_MIN_MM || mm > RANGE_MAX_MM) ? -1 : mm;
  }
#endif

  /* 3. position report at 50 Hz */
  if (millis() - lastSendMs >= REPORT_MS) {
    lastSendMs = millis();
    char out[24];
#if REFLEX_EMULATE
    if (eLost) strcpy(out, "P !");
    else snprintf(out, sizeof out, "P %ld", lroundf(ex * 1000.0f + (urand() * 2.0f - 1.0f) * NOISE_MM));
#else
    bool fresh = sensorOk && millis() - lastReadingMs < 100;
    if (!fresh || lastMm < 0) strcpy(out, "P !");
    else snprintf(out, sizeof out, "P %d", SENSOR_DIR * (lastMm - CENTER_MM));
#endif
    sendLine(out);
  }

  /* 4. say who we are every 2 s (comment line, ignored by the guardian's parser) */
  static unsigned long lastHello = 0;
  if (millis() - lastHello >= 2000) {
    lastHello = millis();
#if REFLEX_EMULATE
    sendLine("# reflex_io emulate=1");
#else
    sendLine(sensorOk ? "# reflex_io emulate=0 sensor=ok" : "# reflex_io emulate=0 VL53L0X NOT FOUND - check wiring");
#endif
  }
}
