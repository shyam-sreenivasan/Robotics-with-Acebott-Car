#include "Arduino.h"
#include "servo.h"

// Standard hobby servos want a 50Hz frame with a 0.5-2.5ms pulse.
// Driven straight off the ESP32 LEDC peripheral so we need no servo library
// (the bundled AVR Servo lib does not build for ESP32).
static const int SERVO_FREQ_HZ  = 50;
static const int SERVO_RES_BITS = 16;
static const int PULSE_MIN_US   = 500;
static const int PULSE_MAX_US   = 2500;

static int currentAngle = SERVO_CENTER;

void servoInit() {
  ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS);
  servoWrite(SERVO_CENTER);
}

void servoWrite(int deg) {
  deg = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  currentAngle = deg;

  int pulse_us = map(deg, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  // Duty as a fraction of the 20ms (20000us) frame, in 16-bit counts.
  uint32_t duty = ((uint32_t)pulse_us * ((1 << SERVO_RES_BITS) - 1)) / 20000;
  ledcWrite(PIN_SERVO, duty);
}

int servoAngle() {
  return currentAngle;
}
