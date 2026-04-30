#pragma once
#include "motion.h"
#include "ultrasonic.h"
#include "servo.h"

#define SCAN_LEFT   150
#define SCAN_CENTER 90
#define SCAN_RIGHT  30

// Forward declarations
int scanDirection();
void runAutonomous();

int scanDirection() {
  servoWrite(SCAN_LEFT);
  float leftDist = readDistance();
  Serial.print("Left: "); Serial.println(leftDist);

  servoWrite(SCAN_CENTER);

  servoWrite(SCAN_RIGHT);
  float rightDist = readDistance();
  Serial.print("Right: "); Serial.println(rightDist);

  servoWrite(SCAN_CENTER);

  if (leftDist <= 0 && rightDist <= 0) return 0;
  if (leftDist > rightDist) return 1;
  return -1;
}

void runAutonomous() {
  float distance = readDistance();
  buzzAtDistance(distance);

  if (distance > 0 && distance <= 10.0) {
    // Too close — keep backing up
    Serial.print("Too close! Backing up. Distance: ");
    Serial.println(distance);
    motionUpdate(-1.0, 0);

  } else if (distance > 10.0 && distance < STOP_DISTANCE) {
    // In warning zone — stop, back off, scan and turn
    Serial.println("Obstacle in range! Scanning...");
    motionUpdate(0, 0);
    delay(300);

    motionUpdate(-1.0, 0);
    delay(700);
    motionUpdate(0, 0);
    delay(300);

    int dir = scanDirection();

    if (dir == 1) {
      Serial.println("Turning left");
      motionUpdate(0, -1.0);
      delay(600);
    } else if (dir == -1) {
      Serial.println("Turning right");
      motionUpdate(0, 1.0);
      delay(600);
    } else {
      Serial.println("Both blocked — reversing more");
      motionUpdate(-1.0, 0);
      delay(800);
    }

    motionUpdate(0, 0);
    delay(200);

  } else {
    // Path clear — move forward
    motionUpdate(1.0, 0);
  }
}