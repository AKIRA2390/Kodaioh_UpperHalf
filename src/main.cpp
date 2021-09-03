#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "ControlStick.h"

#define Debug
controlstick::ControlStick Sticks;
controlstick::BothHandsData_t BothHandsData;

void RecvCB(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&BothHandsData, incomingData, sizeof(BothHandsData));
#ifdef Debug
  Serial.println("Data Received!");
  Serial.println("Right Hand\tRight Hand\tRight Hand\t");
  Sticks.DumpData(BothHandsData.RightStick);
  Serial.println("Left Hand\tLeft Hand\tLeft Hand\t");
  Sticks.DumpData(BothHandsData.LeftStick);
  Serial.println("\n");
#endif
}

void setup() {
  Sticks.ThisReceives(RecvCB);
  Sticks.SetupConnection();
}

void loop() {}