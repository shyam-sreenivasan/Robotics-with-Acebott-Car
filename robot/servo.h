#pragma once

// Servo rotating the front ultrasonic sensor assembly.
#define PIN_SERVO   25

// 0 = full right, 90 = straight ahead, 180 = full left.
#define SERVO_MIN_DEG   0
#define SERVO_MAX_DEG   180
#define SERVO_CENTER    90

void servoInit();

// Clamped to [SERVO_MIN_DEG, SERVO_MAX_DEG].
void servoWrite(int deg);

// Last commanded angle, so telemetry can report where the head was pointing.
int servoAngle();
