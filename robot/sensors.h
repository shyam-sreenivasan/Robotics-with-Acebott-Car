#pragma once

// ---- Pin map (Acebott QD001-style ESP32 car) ----
// Verify these against your board before trusting the readings.
#define PIN_TRIG      13   // ultrasonic trigger
#define PIN_ECHO      14   // ultrasonic echo
#define PIN_LINE_L    39   // line tracker, left   (input-only pin)
#define PIN_LINE_M    36   // line tracker, middle (input-only pin)
#define PIN_LINE_R    35   // line tracker, right  (input-only pin)

struct SensorReading {
  float distance_cm;   // -1.0 when the echo times out (nothing in range)
  int   line_l;
  int   line_m;
  int   line_r;
};

void sensorsInit();
SensorReading sensorsRead();
