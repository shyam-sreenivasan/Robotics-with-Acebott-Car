#include "Arduino.h"
#include "motor.h"
#include "motion.h"
#include "comm.h"
#include "ultrasonic.h"
#include "servo.h"
#include "autonomous.h"

#define AUTO_MODE true  // set false for manual keyboard control

static float v = 0;
static float w = 0;
static unsigned long lastMsgTime = 0;
static const unsigned long TIMEOUT_MS = 300;

void setup() {
  Serial.begin(115200);
  ultrasonicInit();
  servoInit();
  motorInit();
  motionInit();
  commInit();

  // Test all motors on startup
  motorMove(2 | 128 | 1 | 32, 200);
  delay(2000);
  motorMove(0, 0);
}

void loop() {
  if (AUTO_MODE) {
    runAutonomous();
    delay(20);
    return;
  }

  // Manual mode
  if (commRead(v, w)) {
    lastMsgTime = millis();
  }

  if (millis() - lastMsgTime > TIMEOUT_MS) {
    v = 0;
    w = 0;
  }

  float distance = readDistance();
  buzzAtDistance(distance);

  if (v > 0 && obstacleDetected(distance)) {
    Serial.println("Obstacle detected! Stopping.");
    motionUpdate(0, 0);
    return;
  }

  motionUpdate(v, w);
  delay(20);
}