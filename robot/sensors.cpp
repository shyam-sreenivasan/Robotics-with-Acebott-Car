#include "Arduino.h"
#include "sensors.h"

// Echo timeout in microseconds. 25ms of round-trip is roughly 4m of range,
// well past anything useful indoors, and caps how long a ping can stall loop().
static const unsigned long ECHO_TIMEOUT_US = 25000;

void sensorsInit() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  // GPIO 34-39 are input-only and have no internal pullups, so a plain
  // pinMode(INPUT) is all they support.
  pinMode(PIN_LINE_L, INPUT);
  pinMode(PIN_LINE_M, INPUT);
  pinMode(PIN_LINE_R, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

SensorReading sensorsRead() {
  SensorReading r;

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  // pulseIn returns 0 on timeout. Report that as -1 rather than 0cm, so the
  // logger can tell "nothing in range" apart from "touching the bumper".
  r.distance_cm = (us == 0) ? -1.0f : us / 58.0f;

  r.line_l = analogRead(PIN_LINE_L);
  r.line_m = analogRead(PIN_LINE_M);
  r.line_r = analogRead(PIN_LINE_R);

  return r;
}
