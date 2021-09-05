#include "ControlStick.h"

#include <WiFi.h>
#include <esp_now.h>
namespace controlstick {
AMT102V *RoriconPointer;

void RoriconInterrupter() { RoriconPointer->update(); }

ControlStick::ControlStick() {}

ControlStick::~ControlStick() {}

void ControlStick::SetupStick() {
  Roricon = new AMT102V(PinMap.Roricon_A, PinMap.Roricon_B);
  Roricon->setup(0b0000);
  RoriconPointer = Roricon;

  Adafruit_NeoPixel *StatusLeds =
      new Adafruit_NeoPixel(3, PinMap.StatusLed, NEO_GRBW + NEO_KHZ800);
  StatusLeds->begin();

  this->SetupConnection();

  pinMode(PinMap.Stick_X, INPUT);
  pinMode(PinMap.Stick_Y, INPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(PinMap.Buttons[i], INPUT);
  }
  pinMode(PinMap.Roricon_A, INPUT);
  pinMode(PinMap.Roricon_B, INPUT);
  pinMode(PinMap.SliderLimit, INPUT);
  pinMode(PinMap.StatusLed, OUTPUT);

  attachInterrupt(PinMap.Roricon_A, RoriconInterrupter, RISING);
  attachInterrupt(PinMap.Roricon_B, RoriconInterrupter, RISING);
}

void ControlStick::SetupConnection() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
#ifdef DebugControlStick
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }
  if (IsStickSender | IsRobotSender) {
    esp_now_register_send_cb(sendCB);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, DestinationAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#ifdef DebugControlStick
      Serial.println("Failed to add peer");
#endif
      return;
    }
  } else if (IsReceiver) {
    esp_now_register_recv_cb(RecvCB);
  }
}

void ControlStick::Update() {
  if (digitalRead(PinMap.SliderLimit)) {
    SliderIsInitialized = true;
    Roricon->resetRotation();
  }

  for (int i = 0; i < 5; i++) {
    MyStatus.ButtonState[i] = digitalRead(PinMap.Buttons[i]);
  }

  MyStatus.StickStates[0] =
      (analogRead(PinMap.Stick_X) - (4095 / 2)) / (4095 / 2);
  MyStatus.StickStates[1] =
      (analogRead(PinMap.Stick_Y) - (4095 / 2)) / (4095 / 2);
      
  if(abs(MyStatus.StickStates[0])<StickCullingValue)
    MyStatus.StickStates[0] = 0;
  if(abs(MyStatus.StickStates[1])<StickCullingValue)
    MyStatus.StickStates[1] = 0;

  MyStatus.StickAngleRad =
      atan2(MyStatus.StickStates[0], MyStatus.StickStates[1]);
  MyStatus.StickPower = sqrt(MyStatus.StickStates[0] * MyStatus.StickStates[0] +
                             MyStatus.StickStates[1] * MyStatus.StickStates[1]);

  if (SliderIsInitialized)
    MyStatus.Slider = (Roricon->getRotationsDouble() * RoriconGearRadius * 2) /
                      SliderMaxLength;

  if (WorkingStatus.SymmetoryConnection == ConnectionState::CONNECTED) {
    StatusLeds->setPixelColor(0, StatusLeds->Color(0, 255, 0));
  } else {
    StatusLeds->setPixelColor(0, StatusLeds->Color(255, 0, 0));
  }
  if (WorkingStatus.RobotConnection == ConnectionState::CONNECTED) {
    StatusLeds->setPixelColor(1, StatusLeds->Color(0, 255, 0));
  } else {
    StatusLeds->setPixelColor(1, StatusLeds->Color(255, 0, 0));
  }
  StatusLeds->setPixelColor(2, StatusLeds->Color(0, 0, 255));
  StatusLeds->show();

  if (IsStickSender) {
    SendData2Stick(MyStatus);
  } else if (IsRobotSender) {
    SendData2Robot(BothHands);
  }
}

void ControlStick::SendData2Stick(InputData_t HandsData) {
  esp_err_t result =
      esp_now_send(DestinationAddress, (uint8_t *)&MyStatus, sizeof(MyStatus));

  if (result == ESP_OK) {
    SymmetoryConnectionEstablished(true);
#ifdef DebugControlStick
    Serial.println("symmetory send success");
#endif
  } else {
    SymmetoryConnectionEstablished(false);
#ifdef DebugControlStick
    Serial.println("Error symmetory sending");
#endif
  }
}

void ControlStick::SendData2Robot(BothHandsData_t HandsData) {
  esp_err_t result = esp_now_send(DestinationAddress, (uint8_t *)&BothHands,
                                  sizeof(BothHands));

  if (result == ESP_OK) {
    RobotConnectionEstablished(true);
#ifdef DebugControlStick
    Serial.println("robot send success");
#endif
  } else {
    RobotConnectionEstablished(false);
#ifdef DebugControlStick
    Serial.println("Error robot sending");
#endif
  }
}

void ControlStick::ThisSends2Stick(const uint8_t *MACAddress,
                                   esp_now_send_cb_t SendCallBack) {
  IsStickSender = true;
  sendCB = SendCallBack;
  memcpy(this->DestinationAddress, MACAddress, sizeof(MACAddress));
}

void ControlStick::ThisSends2Robot(const uint8_t *MACAddress,
                                   esp_now_send_cb_t SendCallBack) {
  IsRobotSender = true;
  sendCB = SendCallBack;
  memcpy(this->DestinationAddress, MACAddress, sizeof(MACAddress));
}
void ControlStick::ThisReceives(esp_now_recv_cb_t RecvCallBack) {
  IsReceiver = true;
  RecvCB = RecvCallBack;
}

void ControlStick::SymmetoryConnectionEstablished(bool WetherOrNot) {
  WorkingStatus.SymmetoryConnection =
      WetherOrNot ? ConnectionState::CONNECTED : ConnectionState::CONNECTED;
}
void ControlStick::RobotConnectionEstablished(bool WetherOrNot) {
  WorkingStatus.RobotConnection =
      WetherOrNot ? ConnectionState::CONNECTED : ConnectionState::CONNECTED;
}
void ControlStick::GetMyStatus(InputData_t *Value) {
  memcpy(Value, &MyStatus, sizeof(Value));
}

void ControlStick::DumpData(InputData_t Data) {
  Serial.print("Buttons:\t");
  for (int i = 0; i < 5; i++) {
    Serial.print(Data.ButtonState[i]);
    Serial.print("\t");
  }
  Serial.println();

  for (int i = 0; i < 2; i++) {
    Serial.print("Stick:\t");
    Serial.print(Data.StickStates[i]);
    Serial.print("\t");
  }
  Serial.println();

  Serial.print("Slider: ");
  Serial.println(Data.Slider);
  Serial.println();
};
}  // namespace controlstick