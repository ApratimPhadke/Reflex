/* Reflex I/O node - ball-and-beam sensor/actuator bridge with a last-line watchdog.
 *
 * Builds for an STM32 "Black Pill" (STM32duino core, USB support = CDC) and for a
 * genuine Arduino Uno/Nano. Library needed: "VL53L0X" by Pololu.
 * Wiring and geometry: docs/HARDWARE.md. Never compiled by the author: expect to fix small things.
 *
 * Link to the guardian (115200 baud, ASCII lines):
 *     node -> guardian   "P <position_mm>"   every 20 ms   ("P !" = no ball seen)
 *     guardian -> node   "U <beam_angle_millirad>"
 *
 * Third ring of defence: if no "U" arrives for WATCHDOG_MS the node levels the
 * beam on its own and blinks the LED, whatever the computer above it is doing.
 *
 * SPDX-License-Identifier: MIT */
#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

/* ---------------- link to the Raspberry Pi -------------------------------------------------------
 * 0 = USB cable (default). 1 = hardware UART on PA9 (TX) / PA10 (RX), 3.3 V, to the Pi's GPIO UART.
 * Use 1 only if QNX on the Pi does not create a /dev/serusb* device for the Black Pill. */
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
const unsigned long WATCHDOG_MS = 150;
/* --------------------------------------------------------------------------------------------- */

VL53L0X tof;
Servo servo;
unsigned long lastCmdMs = 0, lastSendMs = 0;
char line[24];
uint8_t len = 0;
bool sensorOk = false;

void setBeam(long mrad)
{
  if (mrad > 230) mrad = 230;                      /* hard limit, independent of the guardian */
  if (mrad < -230) mrad = -230;
  servo.writeMicroseconds(SERVO_CENTER_US + SERVO_DIR * (int)(mrad * US_PER_MILLIRAD));
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  LINK.begin(115200);
  servo.attach(SERVO_PIN);
  setBeam(0);
  Wire.begin();
  tof.setTimeout(100);
  sensorOk = tof.init();
  if (sensorOk) {
    tof.setMeasurementTimingBudget(20000);         /* 20 ms per reading -> 50 Hz */
    tof.startContinuous();
  }
  LINK.println(sensorOk ? "# reflex_io ready" : "# reflex_io: VL53L0X NOT FOUND - check wiring");   /* '#' lines are ignored by the guardian */
}

void loop()
{
  /* 1. commands from the guardian */
  while (LINK.available()) {
    char c = (char)LINK.read();
    if (c == '\n' || c == '\r') {
      line[len] = 0;
      if (line[0] == 'U') { setBeam(atol(line + 1)); lastCmdMs = millis(); }
      len = 0;
    } else if (len < sizeof(line) - 1) line[len++] = c;
  }

  /* 2. watchdog: nobody is talking to us -> level the beam */
  bool orphan = millis() - lastCmdMs > WATCHDOG_MS;
  if (orphan) setBeam(0);
  digitalWrite(LED_BUILTIN, orphan ? ((millis() / 100) & 1) : LOW);

  /* 3. position report at 50 Hz */
  if (millis() - lastSendMs >= 20) {
    lastSendMs = millis();
    int mm = sensorOk ? (int)tof.readRangeContinuousMillimeters() : -1;
    if (!sensorOk || tof.timeoutOccurred() || mm < RANGE_MIN_MM || mm > RANGE_MAX_MM) LINK.println("P !");
    else { LINK.print("P "); LINK.println(SENSOR_DIR * (mm - CENTER_MM)); }
  }
}
