#include "Arduino.h"
#include "motor.h"
#include "motion.h"
#include "comm.h"

static float v = 0;
static float w = 0;
static unsigned long lastMsgTime = 0;
static const unsigned long TIMEOUT_MS = 300;

void setup() {
  Serial.begin(115200);
  motorInit();
  motionInit();
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
  delay(20);
}
