#pragma once

#define TRIG_PIN 13
#define ECHO_PIN 14
#define BUZZER_PIN 33
#define STOP_DISTANCE 20.0  // stop if obstacle within 20cm

void ultrasonicInit() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2;
}

void buzzAtDistance(float distance) {
  if (distance <= 0 || distance > STOP_DISTANCE) {
    noTone(BUZZER_PIN);  // out of range, no buzz
    return;
  }

  // Map distance to frequency: closer = higher pitch
  // 20cm -> 500Hz, 1cm -> 2000Hz
  int freq = map((int)distance, 1, STOP_DISTANCE, 2000, 500);

  // Map distance to beep interval: closer = faster beeps
  // 20cm -> 600ms interval, 5cm -> 100ms interval
  int beepInterval = map((int)distance, 1, STOP_DISTANCE, 100, 600);

  static unsigned long lastBeep = 0;
  static bool buzzerOn = false;

  if (millis() - lastBeep > beepInterval) {
    lastBeep = millis();
    buzzerOn = !buzzerOn;
    if (buzzerOn) {
      tone(BUZZER_PIN, freq);
    } else {
      noTone(BUZZER_PIN);
    }
  }
}

bool obstacleDetected(float distance) {
  if (distance > 0 && distance < STOP_DISTANCE) {
    Serial.print("Obstacle at: ");
    Serial.print(distance);
    Serial.println(" cm");
    return true;
  }
  return false;
}