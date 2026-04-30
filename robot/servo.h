#pragma once
#include <ESP32Servo.h>

#define SERVO_PIN 25

static Servo scanServo;

void servoInit() {
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);  // center
  delay(500);
}

void servoWrite(int angle) {
  scanServo.write(angle);
  delay(300);  // wait for servo to reach position
}