#include "comm.h"
#include "servo.h"

// Lines starting with "S," aim the sensor servo instead of setting velocity.
// Returns true if the line was a servo command and has been handled.
static bool handleServoCmd(const String &msg) {
  if (!msg.startsWith("S,")) return false;
  servoWrite(msg.substring(2).toInt());
  return true;
}

#ifdef COMM_WIFI

#include <WiFi.h>
#include "secrets.h"

static const char* SSID     = WIFI_SSID;
static const char* PASSWORD = WIFI_PASSWORD;

static WiFiServer server(1234);
static WiFiClient client;

void commInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);   // clear any stored config from a previous mode

  Serial.print("Connecting WiFi");
  WiFi.begin(SSID, PASSWORD);

  // Don't block forever: a wrong credential used to hang here with no
  // feedback, and the car ignores every command until this returns.
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi FAILED. Check credentials in secrets.h.");
    Serial.println("Continuing anyway -- the car will not accept commands.");
    return;
  }

  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());
  server.begin();
}

bool commRead(float &v, float &w) {
  if (!client || !client.connected()) {
    client = server.available();
    return false;
  }

  if (!client.available()) return false;

  String msg = client.readStringUntil('\n');
  if (handleServoCmd(msg)) return false;  // not a velocity update

  int comma = msg.indexOf(',');
  if (comma <= 0) return false;

  v = msg.substring(0, comma).toFloat();
  w = msg.substring(comma + 1).toFloat();
  return true;
}

void commWrite(const String &line) {
  if (client && client.connected()) {
    client.println(line);
  }
}

#else  // Serial

#include "Arduino.h"

void commInit() {
  // Serial already started in robot.ino
}

bool commRead(float &v, float &w) {
  if (!Serial.available()) return false;

  String msg = Serial.readStringUntil('\n');
  if (handleServoCmd(msg)) return false;  // not a velocity update

  int comma = msg.indexOf(',');
  if (comma <= 0) return false;

  v = msg.substring(0, comma).toFloat();
  w = msg.substring(comma + 1).toFloat();
  return true;
}

void commWrite(const String &line) {
  Serial.println(line);
}

#endif
