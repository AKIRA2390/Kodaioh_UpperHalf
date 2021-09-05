#pragma once

#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>

#include "AMT102V.h"

#define DebugControlStick
namespace controlstick {
typedef struct PinMap_t {
  const int Stick_X = 36;
  const int Stick_Y = 39;
  const int Buttons[5] = {34, 35, 32, 33,
                          4};  //小指側から、スティックのボタンも含む
  const int Roricon_A = 25;
  const int Roricon_B = 26;
  const int SliderLimit = 23;
  const int StatusLed = 27;
} PinMap_t;  // namespace PinMap

typedef struct InputData_t {
  bool ButtonState[5];    //押されたらtrueが入る
  double StickStates[2];  //-1～1が入る
  double StickAngleRad;
  double StickPower;
  double Slider;          // 0～1が入る
} InputData_t;

typedef struct BothHandsData_t {
  InputData_t LeftStick, RightStick;
} BothHandsData_t;

class ControlStick {
 private:
  bool IsStickSender = false, IsRobotSender = false, IsReceiver = false,
       SliderIsInitialized = false;
  const int RoriconGearRadius = 20;  //ロリコンについた歯車の半径[mm]
  const int SliderMaxLength =
      200;  //スライダを最大まで伸ばしたときの始点からの距離[mm]
  const double StickCullingValue = 0.01;  //スティックの値がこれ未満の時は無視される

  uint8_t DestinationAddress[];
  //   void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  //   void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int
  //   len);

  esp_now_send_cb_t sendCB;
  esp_now_recv_cb_t RecvCB;

  enum class ConnectionState { CONNECTED, DISCONNECTED };
  typedef struct WorkingState {
    ConnectionState SymmetoryConnection = ConnectionState::DISCONNECTED;
    ConnectionState RobotConnection = ConnectionState::DISCONNECTED;
  } WorkingState;

  PinMap_t PinMap;
  InputData_t MyStatus;
  WorkingState WorkingStatus;

  BothHandsData_t BothHands;

  AMT102V* Roricon;
  Adafruit_NeoPixel* StatusLeds;

 public:
  // ControlStickInputData_t MessageData;
  ControlStick();
  ~ControlStick();
  void SetupStick();
  void SetupConnection();
  void Update();
  void SendData2Stick(InputData_t HandsData);
  void SendData2Robot(BothHandsData_t HandsData);
  void ThisSends2Stick(const uint8_t* MACAddress,
                       esp_now_send_cb_t SendCallBack);
  void ThisSends2Robot(const uint8_t* MACAddress,
                       esp_now_send_cb_t SendCallBack);
  void ThisReceives(esp_now_recv_cb_t RecvCallBack);
  void SymmetoryConnectionEstablished(bool WetherOrNot);
  void RobotConnectionEstablished(bool WetherOrNot);
  void GetMyStatus(InputData_t* Value);
  void DumpData(InputData_t Data);
};

}  // namespace controlstick