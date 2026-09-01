#include "Arduino.h"
#include "motor.h"
#include "motion.h"
#include "comm.h"
#include "sensors.h"
#include "servo.h"

static float v = 0;
static float w = 0;
static unsigned long lastMsgTime = 0;
static const unsigned long TIMEOUT_MS = 300;

// Telemetry runs slower than the control loop: an ultrasonic ping can block
// for up to 25ms waiting on the echo, which would otherwise stall steering.
static unsigned long lastTelemTime = 0;
static const unsigned long TELEM_INTERVAL_MS = 100;

void setup() {
  Serial.begin(115200);
  motorInit();
  motionInit();
  sensorsInit();
  servoInit();
  commInit();
}

void loop() {
  if (commRead(v, w)) {
    lastMsgTime = millis();
  }

  if (millis() - lastMsgTime > TIMEOUT_MS) {
    v = 0;
    w = 0;
  }

  motionUpdate(v, w);

  if (millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = millis();
    SensorReading s = sensorsRead();
    String line = "T,";
    line += millis();        line += ",";
    line += s.distance_cm;   line += ",";
    line += s.line_l;        line += ",";
    line += s.line_m;        line += ",";
    line += s.line_r;        line += ",";
    line += v;               line += ",";
    line += w;               line += ",";
    line += servoAngle();
    commWrite(line);
  }

  delay(20);
}
