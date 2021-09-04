#include <Arduino.h>

#include "ControlStick.h"

#define Debug
controlstick::ControlStick Sticks;
controlstick::BothHandsData_t BothHandsData;

uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};

void SendCB(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef Debug
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
#endif
}

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

void RightArmUpdate();
void LeftArmUpdate();

void setup() {
  Sticks.ThisSends2Stick(LeftHalfAddress, SendCB);
  Sticks.ThisReceives(RecvCB);
  Sticks.SetupConnection();
}

void loop() {
  RightArmUpdate();
  LeftArmUpdate();
}

void RightArmUpdate(){

}

void LeftArmUpdate() { Sticks.SendData2Stick(BothHandsData.LeftStick); }