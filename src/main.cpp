#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"
#include "KodaiohShoulder.h"
#include "MotionDatas.h"
#include "MotionManager.h"

//
#include "PID4arduino.h"
//

#define Debug

typedef struct RightHalfPinmap_t {
  // output pins //
  // plus, minus
  const int ElbowMotors[2] = {27, 14};

  // input pins //
  const int ElbowRoricon[2] = {21, 19};
  // max,min
  const int ElbowLimit[2] = {18, 17};
} RightHalfPinmap_t;

typedef struct RightHalfSensorStates {
  bool ElbowLimit[2];
  double ElbowRotationRad;
} RightHalfSensorStates;

RightHalfPinmap_t Pinmap;
RightHalfSensorStates SensorStates;

controlstick::ControlStick Sticks = {};
controlstick::BothHandsData_t BothHandsData;

AMT102V *ElbowRoricon;

//
motionmanager::MotionManager Motion(true);
motionmanager::AngleDatas_t AngleDatas;
//

HardwareSerial MySerial(2);

// uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};
uint8_t LeftHalfAddress[] = {0x24, 0x0A, 0xC4, 0xF9, 0x40, 0xD0};

double ShoulderManipulateValue = 0, UpperArmManipulateValue = 0,
       ElbowManipulateValue = 0;
const int ElbowMotorPower = 150;

bool ElbowRoriconInitialised = false;

//
PID4Arduino::PID4arduino<int> ElbowPID;
PID4Arduino::PIDGain_t ShoulderPIDGain, UpperArmPIDGain, ElbowPIDGain;

int ElbowTargetDeg = 0;
//

// const double ElbowReductionRatio = 3. / 760;
const double ElbowReductionRatio = 38. / 15;
const double ElbowLimitAngleRad[2] = {90 * DEG_TO_RAD, 0 * DEG_TO_RAD};

void ElbowRoriconInterrupter() { ElbowRoricon->update(); }

void RightArmUpdate();
void LeftArmUpdate();

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue);
void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue);
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue,
                         double *ElbowManipulateValue);
void Update4RightArmReset(double *ShoulderManipulateValue,
                          double *UpperArmManipulateValue,
                          double *ElbowManipulateValue);

void PrintVariousThings();

void PWM_transmit_via_CAN(int16_t pwm1, int16_t pwm2, int16_t pwm3,
                          int16_t pwm4);

void setup() {
  Serial.begin(115200);
  // Sticks.ThisSends2Robot(LeftHalfAddress, );
  Serial.println("started!");

  //
  ShoulderPIDGain.KP =
      kodaioh_shoulder::ShoulderMotorPower /
      (abs(kodaioh_shoulder::ShoulderLimitAngleRad[0] * RAD_TO_DEG) +
       abs(kodaioh_shoulder::ShoulderLimitAngleRad[1] * RAD_TO_DEG));
  ShoulderPIDGain.KI = 0;
  ShoulderPIDGain.KD = 0;

  UpperArmPIDGain.KP =
      kodaioh_shoulder::UpperArmMotorPower /
      (abs(kodaioh_shoulder::UpperArmLimitAngleRad[0] * RAD_TO_DEG) +
       abs(kodaioh_shoulder::UpperArmLimitAngleRad[1] * RAD_TO_DEG));
  UpperArmPIDGain.KI = 0;
  UpperArmPIDGain.KD = 0;

  ElbowPIDGain.KP = ElbowMotorPower / (abs(ElbowLimitAngleRad[0] * RAD_TO_DEG) +
                                       abs(ElbowLimitAngleRad[1] * RAD_TO_DEG));
  ElbowPIDGain.KI = 0;
  ElbowPIDGain.KD = 0;

  MySerial.begin(115200, SERIAL_8N1, 17, 15);

  motion_datas::setup();

  Motion.setup(&kodaioh_shoulder::ShoulderTargetDeg,
               &kodaioh_shoulder::UpperArmTargetDeg, &ElbowTargetDeg);

  kodaioh_shoulder::setPIDGains(ShoulderPIDGain, UpperArmPIDGain);
  ElbowPID.setGains(ElbowPIDGain);
  //

  kodaioh_shoulder::setup(&Sticks, &(BothHandsData.RightStick), true, false);

  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ElbowMotors[i], OUTPUT);

    pinMode(Pinmap.ElbowRoricon[i], INPUT);

    pinMode(Pinmap.ElbowLimit[i], INPUT_PULLUP);
  }

  ElbowRoricon = new AMT102V(Pinmap.ElbowRoricon[0], Pinmap.ElbowRoricon[1]);
  ElbowRoricon->setup(0b1100);
  attachInterrupt(Pinmap.ElbowRoricon[0], ElbowRoriconInterrupter, RISING);
  attachInterrupt(Pinmap.ElbowRoricon[1], ElbowRoriconInterrupter, RISING);
  Serial.println("Setup End");
}

void loop() {
  Serial.println(" ");
  for (int i = 0; i < 2; i++) {
    SensorStates.ElbowLimit[i] = !digitalRead(Pinmap.ElbowLimit[i]);
  }

  if (SensorStates.ElbowLimit[1]) {
    ElbowRoriconInitialised = true;
    ElbowRoricon->resetRotation();
  }

  if (ElbowRoriconInitialised)
    SensorStates.ElbowRotationRad =
        (ElbowRoricon->getRotationsDouble() / ElbowReductionRatio) * 2 * PI;

  //
  AngleDatas.ShoulderRotationDeg =
      kodaioh_shoulder::SensorStates.ShoulderRotationRad * RAD_TO_DEG;
  AngleDatas.UpperArmRotationDeg =
      kodaioh_shoulder::SensorStates.UpperArmRotationRad * RAD_TO_DEG;
  AngleDatas.ElbowRotationDeg = SensorStates.ElbowRotationRad * RAD_TO_DEG;

  // Serial.println("Motion Update");
  Motion.update(AngleDatas);
  // Serial.println("Motion Update Called");

  //

  kodaioh_shoulder::update();

  // Sticks.SendData2Robot(BothHandsData);

#ifdef Debug
  RightArmUpdate();
#endif

  if (kodaioh_shoulder::IsDirty) {
    // Serial.println("is dirty");
    kodaioh_shoulder::GetBothHandsData(&BothHandsData);

#ifndef Debug
    RightArmUpdate();
#endif

    LeftArmUpdate();

    // kodaioh_shoulder::UpdateWhenDirty(ShoulderManipulateValue,
    //                                   UpperArmManipulateValue);

    const int S_MAXPOWER = 32000, U_MAXPOWER = 32000, E_MAXPOWER = 32000;
    const int S_SCALEFACTOR = S_MAXPOWER / 255,
              U_SCALEFACTOR = U_MAXPOWER / 255,
              E_SCALEFACTOR = E_MAXPOWER / 255;

    PWM_transmit_via_CAN(ShoulderManipulateValue * S_SCALEFACTOR,
                         UpperArmManipulateValue * U_SCALEFACTOR,
                         ElbowManipulateValue * E_SCALEFACTOR, 0);

    PrintVariousThings();
    kodaioh_shoulder::IsDirty = false;

#ifndef Debug
    Serial.println("Right Hands Received Data:");
    Sticks.DumpData(BothHandsData.RightStick);
    Serial.println("Left Hands Received Data:");
    Sticks.DumpData(BothHandsData.LeftStick);
#endif
  }

  ElbowPID.update(ElbowTargetDeg, SensorStates.ElbowRotationRad);

  // delay(100);
}

void RightArmUpdate() {
  // Serial.println("right arm update");
  kodaioh_shoulder::IsDirty = true;
#ifdef Debug
  UpdateTestDummy(&ShoulderManipulateValue, &UpperArmManipulateValue,
                  &ElbowManipulateValue);
//   UpdateTaishinMethod(&ShoulderManipulateValue, &UpperArmManipulateValue,
//                       &ElbowManipulateValue);
#else
  UpdateAKIRAMethod(&ShoulderManipulateValue, &UpperArmManipulateValue);
#endif
  // if (ElbowManipulateValue > 0) {
  //   if (!SensorStates.ElbowLimit[0]) {
  //     analogWrite(Pinmap.ElbowMotors[0], ElbowManipulateValue);
  //     analogWrite(Pinmap.ElbowMotors[1], 0);
  //   } else {
  //     analogWrite(Pinmap.ElbowMotors[0], 0);
  //     analogWrite(Pinmap.ElbowMotors[1], 0);
  //   }
  // } else {
  //   if (!SensorStates.ElbowLimit[1]) {
  //     analogWrite(Pinmap.ElbowMotors[0], 0);
  //     analogWrite(Pinmap.ElbowMotors[1], ElbowManipulateValue);
  //   } else {
  //     analogWrite(Pinmap.ElbowMotors[0], 0);
  //     analogWrite(Pinmap.ElbowMotors[1], 0);
  //   }
  // }
}

void LeftArmUpdate() {
  //  Sticks.SendData2Robot(BothHandsData);
}

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue) {
  const bool ShoulderTesting = false, UpperArmTesting = false,
             ElbowTesting = true;
  static bool ElbowDirection = false;

  kodaioh_shoulder::IsDirty = true;

  // Serial.println("right arm test dummy");
  kodaioh_shoulder::UpdateTestDummy(ShoulderManipulateValue,
                                    UpperArmManipulateValue, ShoulderTesting,
                                    UpperArmTesting);

  if (SensorStates.ElbowLimit[0]) {
    ElbowDirection = false;
  } else if (SensorStates.ElbowLimit[1]) {
    ElbowDirection = true;
  }

  if (ElbowRoriconInitialised && ElbowTesting) {
    if (ElbowDirection) {
      *ElbowManipulateValue = ElbowMotorPower;
    } else {
      *ElbowManipulateValue = -ElbowMotorPower;
    }
  }

  // Serial.println("//////////////////////////");
  // Serial.println("Elbow reduction ratio");
  // Serial.println(ElbowReductionRatio);
  // Serial.println("Elbow roricon initialised");
  // Serial.println(ElbowRoriconInitialised);
  // Serial.println("Elbow direction");
  // Serial.println(ElbowDirection);
  // Serial.println("Elbow testing");
  // Serial.println("Elbow manipulation vlaue");
  // Serial.println(*ElbowManipulateValue);
  // Serial.println("//////////////////////////\n");
}

void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue) {
  kodaioh_shoulder::UpdateAKIRAMethod(ShoulderManipulateValue,
                                      UpperArmManipulateValue);

  if (BothHandsData.RightStick.ButtonState[3]) {
    ElbowManipulateValue = ElbowMotorPower;
  } else if (BothHandsData.RightStick.ButtonState[2]) {
    ElbowManipulateValue = -ElbowMotorPower;
  } else {
    ElbowManipulateValue = 0;
  }
}
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue,
                         double *ElbowManipulateValue) {
#ifdef Debug
  // Serial.println("Update Taishin Method!");
  if (!Motion.IsBusy()) {
    // Serial.println("Motion Startmove:Empty");
    Motion.StartMove(motion_datas::Empty);
  }
  kodaioh_shoulder::IsDirty = true;
  kodaioh_shoulder::SensorStates.ShoulderLimit = true;
#else

  if (!Motion.IsBusy()) {
    if (BothHandsData.RightStick.ButtonState[0])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.RightStick.ButtonState[1])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.RightStick.ButtonState[2])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.RightStick.ButtonState[3])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.RightStick.ButtonState[4])
      Motion.StartMove(motion_datas::Empty);

    if (BothHandsData.LeftStick.ButtonState[0])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.LeftStick.ButtonState[1])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.LeftStick.ButtonState[2])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.LeftStick.ButtonState[3])
      Motion.StartMove(motion_datas::Empty);
    if (BothHandsData.LeftStick.ButtonState[4])
      Motion.StartMove(motion_datas::Empty);
  }
#endif

  kodaioh_shoulder::UpdateTaishinMethod(ShoulderManipulateValue,
                                        UpperArmManipulateValue);
  *ElbowManipulateValue = ElbowPID.GetValue();
}
void Update4RightArmReset(double *ShoulderManipulateValue,
                          double *UpperArmManipulateValue,
                          double *ElbowManipulateValue) {
  kodaioh_shoulder::Update4ShoulderUnitReset(ShoulderManipulateValue,
                                             UpperArmManipulateValue);
  if (ElbowRoriconInitialised) {
    *ElbowManipulateValue = -ElbowMotorPower;
  }
}

void PrintVariousThings() {
  // Serial.print("\n");
  // Serial.print("Shoulder Roricon Raw:");
  // Serial.print(kodaioh_shoulder::ShoulderRoricon->getRotationsDouble());
  // Serial.print("Shoulder roricon initialized:");
  // Serial.print(kodaioh_shoulder::ShoulderRoriconInitialised ? "true" :
  // "false");
  // Serial.print(", ");
  Serial.print("Shoulder_MV:");
  Serial.print(ShoulderManipulateValue);
  Serial.print(", ");
  Serial.print("Shoulder_Angle_Deg:");
  Serial.print(kodaioh_shoulder::SensorStates.ShoulderRotationRad * RAD_TO_DEG);
  Serial.print(", ");
  // Serial.print("Shoulder Limit Angle Deg:");
  // Serial.print(kodaioh_shoulder::ShoulderLimitAngleRad[0] * RAD_TO_DEG);
  // Serial.print(":");
  // Serial.print(kodaioh_shoulder::ShoulderLimitAngleRad[1] * RAD_TO_DEG);
  // Serial.print(", ");
  // Serial.print("Shoulder Limit Stats:");
  // Serial.print(kodaioh_shoulder::SensorStates.ShoulderLimit);
  // Serial.print("Shoulder Error Value:");
  // Serial.print((kodaioh_shoulder::ShoulderLimitAngleRad[0] -
  //               kodaioh_shoulder::SensorStates.ShoulderRotationRad) *
  //              RAD_TO_DEG);
  // Serial.print(":");
  // Serial.print(kodaioh_shoulder::MotorPower *
  //              ((kodaioh_shoulder::ShoulderLimitAngleRad[0] -
  //                kodaioh_shoulder::SensorStates.ShoulderRotationRad) /
  //               kodaioh_shoulder::ShoulderLimitAngleRad[0]));
  // Serial.print("\n");
  // Serial.print(", ");

  // Serial.print("UpperArm Roricon Raw:");
  // Serial.print(kodaioh_shoulder::UpperArmRoricon->getRotationsDouble());
  // Serial.print(", ");
  // Serial.print("UpperArm roricon initialized:");
  // Serial.print(kodaioh_shoulder::UpperArmRoriconInitialised ? "true" :
  // "false");
  // Serial.print(", ");
  Serial.print("UpperArm_MV:");
  Serial.print(UpperArmManipulateValue);
  Serial.print(", ");
  Serial.print("UpperArm_Angle_Deg:");
  Serial.print(kodaioh_shoulder::SensorStates.UpperArmRotationRad * RAD_TO_DEG);
  Serial.print(", ");
  // Serial.print("UpperArm Limit Angle Deg:");
  // Serial.print(kodaioh_shoulder::UpperArmLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print(":");
  // Serial.print(kodaioh_shoulder::UpperArmLimitAngleRad[1]*RAD_TO_DEG);
  // Serial.print(", ");

  // Serial.print("UpperArm Limit Stats:");
  // Serial.print(kodaioh_shoulder::SensorStates.UpperArmLimit[0]);
  // Serial.print(":");
  // Serial.print(kodaioh_shoulder::SensorStates.UpperArmLimit[1]);
  // Serial.print(", ");
  // Serial.print("UpperArm Error Value:");
  // Serial.print((kodaioh_shoulder::UpperArmLimitAngleRad[0] -
  // kodaioh_shoulder::SensorStates.UpperArmRotationRad)*RAD_TO_DEG);
  // Serial.print(":");
  //
  // Serial.print(kodaioh_shoulder::MotorPower*((kodaioh_shoulder::UpperArmLimitAngleRad[0]
  // -
  //
  // kodaioh_shoulder::SensorStates.UpperArmRotationRad)/kodaioh_shoulder::UpperArmLimitAngleRad[0]));
  // Serial.print("\n");
  // Serial.print(", ");

  // Serial.print("Elbow Roricon Initialized:");
  // Serial.print(ElbowRoriconInitialised ? "true" : "false");
  // Serial.print(", ");
  // Serial.print("Elbow Roricon Raw:");
  // Serial.print(ElbowRoricon->getRotationsDouble());
  // Serial.print(", ");
  Serial.print("Elbow_MV:");
  Serial.print(ElbowManipulateValue);
  Serial.print(", ");
  Serial.print("Elbow_Angle_Deg:");
  Serial.print(SensorStates.ElbowRotationRad * RAD_TO_DEG);
  Serial.print(", ");
  // Serial.print("Elbow Limit max:");
  // Serial.print(SensorStates.ElbowLimit[0]);
  // Serial.print(", ");
  // Serial.print("Elbow Limit min:");
  // Serial.print(SensorStates.ElbowLimit[1]);
  // Serial.print(", ");
  // Serial.print("Elbow Limit Angle Deg:");
  // Serial.print(ElbowLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print(":");
  // Serial.print(ElbowLimitAngleRad[1]*RAD_TO_DEG);
  // Serial.print(", ");
  // Serial.print("Elbow Error Value:");
  // Serial.print((ElbowLimitAngleRad[0] -
  // SensorStates.ElbowRotationRad)*RAD_TO_DEG); Serial.print(":");
  // Serial.print(kodaioh_shoulder::MotorPower*((ElbowLimitAngleRad[0] -
  // SensorStates.ElbowRotationRad)/ElbowLimitAngleRad[0]));
  // Serial.println("\n");
}

// @param : +-32,767のPWM値（モータードライバ４ポート分）
//          上限+-32000程度（ArduinoのAnalogWrite(port,249);と同じくらいの出力：97.65%）にモータードライバ側で勝手に制限↓
//           （Duty比100%を入力するとブートストラップ回路が動作しなくなってモータドライバが動かないから）
// @note :
// 手元の環境的な理由でMySerialを使っています。ESP32用のポートに書き換えて使ったらOKです。
//          相手側の通信速度は115200で設定しています。もしESP側で同じ速度に設定できなかったら服部に知らせてください。
void PWM_transmit_via_CAN(int16_t pwm1, int16_t pwm2, int16_t pwm3,
                          int16_t pwm4) {
  int16_t pwms[4] = {pwm1, pwm2, pwm3, pwm4};
  uint8_t pwm_highbyte[4], pwm_lowbyte[4], can_data[8], calc_checksum;
  static const uint8_t can_id = 3, can_frame_prefix = 0xff,
                       can_frame_verID = 0xfe;
  for (uint8_t i = 0; i < 4; i++) {
    pwm_highbyte[i] = uint8_t(int16_t(pwms[i] >> 8) & 0xff);
    pwm_lowbyte[i] = uint8_t(pwms[i] & 0xff);
    can_data[i * 2] = pwm_lowbyte[i];
    can_data[(i * 2) + 1] = pwm_highbyte[i];
  }
  calc_checksum = 255 - (uint16_t(can_id + can_data[0] + can_data[1] +
                                  can_data[2] + can_data[3] + can_data[4] +
                                  can_data[5] + can_data[6] + can_data[7]) %
                         256);

  MySerial.write(can_frame_prefix);
  MySerial.write(can_frame_verID);
  MySerial.write(can_id);
  MySerial.write(can_data[0]);
  MySerial.write(can_data[1]);
  MySerial.write(can_data[2]);
  MySerial.write(can_data[3]);
  MySerial.write(can_data[4]);
  MySerial.write(can_data[5]);
  MySerial.write(can_data[6]);
  MySerial.write(can_data[7]);
  MySerial.write(calc_checksum);
}