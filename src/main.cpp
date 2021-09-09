#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"

#define Debug

typedef struct RightHalfPinmap_t {
  // output pins //
  // plus, minus
  const int ShoulderMotors[2] = {32, 33};
  const int UpperArmMotors[2] = {25, 26};

  const int ElbowMotors[2] = {27, 14};
  const int HandMotors[2] = {9, 10};

  // input pins //
  const int ShoulderRoricon[2] = {36, 39};
  const int UpperArmRoricon[2] = {34, 35};
  // max,min
  // const int ShoulderLimits[2] = {23, 22};
  const int ShoulderLimits = 22;
  const int UpperArmLimits[2] = {12, 13};

  const int ElbowRoricon[2] = {21, 19};
  const int ElbowLimits[2] = {18, 17};
  const int HandLimits[2] = {16, 4};
} RightHalfPinmap_t;

typedef struct RightHalfSensorStates {
  bool ShoulderLimits, UpperArmLimits[2], ElbowLimits[2], HandLimits[2];
  double ShoulderRotationRad, UpperArmRotationRad, ElbowRotationRad;
} RightHalfSensorStates;

RightHalfPinmap_t Pinmap;
RightHalfSensorStates SensorStates;

controlstick::ControlStick Sticks;
controlstick::BothHandsData_t BothHandsData;
uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};

bool IsDirty = false, SwordDrawInProgress = false, SwordDrawCompleted = false;
bool ShoulderRoriconInitialised = false, UpperArmRoriconInitialised = false,
     ElbowRoriconInitialised = false;

AMT102V *ShoulderRoricon, *UpperArmRoricon, *ElbowRoricon;

const int MotorPower = 200;
const double ShoulderReductionRatio = 2 / 9;
const double UpperArmReductionRatio = 78 / 194;
const double ElbowReductionRatio = 3 / 760;
const double ShoulderLimitAngleRad[2] = {140 * DEG_TO_RAD, -80 * DEG_TO_RAD};
const double UpperArmLimitAngleRad[2] = {90 * DEG_TO_RAD, 0 * DEG_TO_RAD};
const double ElbowLimitAngleRad[2] = {90 * DEG_TO_RAD, 0 * DEG_TO_RAD};

void ShoulderRoriconInterrupter() { ShoulderRoricon->update(); }
void UpperArmRoriconInterrupter() { UpperArmRoricon->update(); }
void ElbowRoriconInterrupter() { ElbowRoricon->update(); }

void SendCB(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef Debug
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
#endif
}

void RecvCB(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&BothHandsData, incomingData, sizeof(BothHandsData));
  IsDirty = true;
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

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue);
void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue);
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue);

void SwordDrawingProcedure();

void setup() {
  Sticks.ThisSends2Stick(LeftHalfAddress, SendCB);
  Sticks.ThisReceives(RecvCB);
  Sticks.SetupConnection();

  pinMode(Pinmap.ShoulderLimits, INPUT_PULLUP);
  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ShoulderMotors[i], OUTPUT);
    pinMode(Pinmap.UpperArmMotors[i], OUTPUT);
    pinMode(Pinmap.ElbowMotors[i], OUTPUT);

    pinMode(Pinmap.ShoulderRoricon[i], INPUT);
    pinMode(Pinmap.UpperArmRoricon[i], INPUT);
    pinMode(Pinmap.ElbowRoricon[i], INPUT);

    pinMode(Pinmap.UpperArmLimits[i], INPUT_PULLUP);
    pinMode(Pinmap.ElbowLimits[i], INPUT_PULLUP);
    pinMode(Pinmap.HandLimits[i], INPUT_PULLUP);
  }
  ShoulderRoricon =
      new AMT102V(Pinmap.ShoulderRoricon[0], Pinmap.ShoulderRoricon[1]);
  ShoulderRoricon->setup(0b0000);
  attachInterrupt(Pinmap.ShoulderRoricon[0], ShoulderRoriconInterrupter,
                  RISING);
  attachInterrupt(Pinmap.ShoulderRoricon[1], ShoulderRoriconInterrupter,
                  RISING);

  UpperArmRoricon =
      new AMT102V(Pinmap.UpperArmRoricon[0], Pinmap.UpperArmRoricon[1]);
  UpperArmRoricon->setup(0b0000);
  attachInterrupt(Pinmap.UpperArmRoricon[0], UpperArmRoriconInterrupter,
                  RISING);
  attachInterrupt(Pinmap.UpperArmRoricon[1], UpperArmRoriconInterrupter,
                  RISING);

  ElbowRoricon = new AMT102V(Pinmap.ElbowRoricon[0], Pinmap.ElbowRoricon[1]);
  ElbowRoricon->setup(0b0000);
  attachInterrupt(Pinmap.ElbowRoricon[0], ElbowRoriconInterrupter, RISING);
  attachInterrupt(Pinmap.ElbowRoricon[1], ElbowRoriconInterrupter, RISING);
}

void loop() {
  for (int i = 0; i < 2; i++) {
    SensorStates.ShoulderLimits = digitalRead(Pinmap.ShoulderLimits);
    SensorStates.UpperArmLimits[i] = digitalRead(Pinmap.UpperArmLimits[i]);
    SensorStates.ElbowLimits[i] = digitalRead(Pinmap.ElbowLimits[i]);
  }
  if (SensorStates.ShoulderLimits) {
    ShoulderRoriconInitialised = true;
    ShoulderRoricon->resetRotation();
  }
  if (SensorStates.UpperArmLimits[1]) {
    UpperArmRoriconInitialised = true;
    UpperArmRoricon->resetRotation();
  }
  if (SensorStates.ElbowLimits[1]) {
    ElbowRoriconInitialised = true;
    ElbowRoricon->resetRotation();
  }

  if (ShoulderRoriconInitialised)
    SensorStates.ShoulderRotationRad =
        (ShoulderRoricon->getRotationsDouble() / ShoulderReductionRatio) * 2 *
        PI;
  if (UpperArmRoriconInitialised)
    SensorStates.UpperArmRotationRad =
        (UpperArmRoricon->getRotationsDouble() / UpperArmReductionRatio) * 2 *
        PI;
  if (ElbowRoriconInitialised)
    SensorStates.ElbowRotationRad =
        (ElbowRoricon->getRotationsDouble() / ElbowReductionRatio) * 2 * PI;
  Serial.print("Shoulder Rotation:");
  Serial.println(SensorStates.ShoulderRotationRad*RAD_TO_DEG);
  Serial.print("UpperArm Rotation:");
  Serial.println(SensorStates.UpperArmRotationRad*RAD_TO_DEG);
  Serial.print("Elbow Rotation:");
  Serial.println(SensorStates.ElbowRotationRad*RAD_TO_DEG);
  Serial.println("\n");

  RightArmUpdate();
  if (IsDirty) {
    LeftArmUpdate();
    IsDirty = false;
  }
}

void RightArmUpdate() {
  double ShoulderManipulateValue, UpperArmManipulateValue, ElbowManipulateValue;
  if ((BothHandsData.RightStick.ButtonState[4] && !SwordDrawCompleted) ||
      SwordDrawInProgress)
    SwordDrawingProcedure();

  UpdateTestDummy(&UpperArmManipulateValue, &UpperArmManipulateValue,
                  &ElbowManipulateValue);
  // UpdateAKIRAMethod(&UpperArmManipulateValue, &UpperArmManipulateValue);
  // UpdateTaishinMethod(&UpperArmManipulateValue, &UpperArmManipulateValue);

  if (ShoulderManipulateValue > 0) {
    if (SensorStates.ShoulderRotationRad < ShoulderLimitAngleRad[0])
      analogWrite(Pinmap.ShoulderMotors[0], ShoulderManipulateValue);
  } else {
    if (ShoulderLimitAngleRad[1] < SensorStates.ShoulderRotationRad)
      analogWrite(Pinmap.ShoulderMotors[1], -ShoulderManipulateValue);
  }

  if (UpperArmManipulateValue > 0) {
    if (!SensorStates.UpperArmLimits[0])
      analogWrite(Pinmap.UpperArmMotors[0], UpperArmManipulateValue);
  } else {
    if (!SensorStates.UpperArmLimits[1])
      analogWrite(Pinmap.UpperArmMotors[1], -UpperArmManipulateValue);
  }

  if (ElbowManipulateValue > 0) {
    if (!SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], ElbowManipulateValue);
  } else {
    if (!SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -ElbowManipulateValue);
  }
}

void LeftArmUpdate() { Sticks.SendData2Stick(BothHandsData.LeftStick); }

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue) {
  const bool ShoulderTesting = true, UpperArmTesting = false,
             ElbowTesting = false, HandTesting = false;
  static bool ShoulderDirection = true, UpperArmDirection = true,
              ElbowDirection = true, HandDirection = true;

  if (SensorStates.ShoulderRotationRad < ShoulderLimitAngleRad[0]) {
    ShoulderDirection = false;
  } else if (ShoulderLimitAngleRad[1] < SensorStates.ShoulderRotationRad) {
    ShoulderDirection = true;
  }
  if (SensorStates.UpperArmLimits[0]) {
    UpperArmDirection = false;
  } else if (SensorStates.UpperArmLimits[1]) {
    UpperArmDirection = true;
  }
  if (SensorStates.ElbowLimits[0]) {
    ElbowDirection = false;
  } else if (SensorStates.ElbowLimits[1]) {
    ElbowDirection = true;
  }

  if (ShoulderRoriconInitialised && ShoulderTesting) {
    if (ShoulderDirection) {
      *ShoulderManipulateValue =
          MotorPower *
          ((ShoulderLimitAngleRad[0] - SensorStates.ShoulderRotationRad) /
           ShoulderLimitAngleRad[0]);
    } else {
      *ShoulderManipulateValue =
          -MotorPower *
          ((ShoulderLimitAngleRad[1] - SensorStates.ShoulderRotationRad) /
           ShoulderLimitAngleRad[0]);
    }
  }
  if (UpperArmRoriconInitialised && UpperArmTesting) {
    if (UpperArmDirection) {
      *UpperArmManipulateValue =
          MotorPower *
          ((UpperArmLimitAngleRad[0] - SensorStates.UpperArmRotationRad) /
           UpperArmLimitAngleRad[0]);
    } else {
      *UpperArmManipulateValue =
          -MotorPower *
          ((UpperArmLimitAngleRad[1] - SensorStates.UpperArmRotationRad) /
           UpperArmLimitAngleRad[0]);
    }
  }
  if (ElbowRoriconInitialised && ElbowTesting) {
    if (ElbowDirection) {
      *ElbowManipulateValue =
          MotorPower *
          ((ElbowLimitAngleRad[0] - SensorStates.ElbowRotationRad) /
           ElbowLimitAngleRad[0]);
    } else {
      *ElbowManipulateValue =
          -MotorPower *
          ((ElbowLimitAngleRad[1] - SensorStates.ElbowRotationRad) /
           ElbowLimitAngleRad[0]);
    }
  }
  // if (HandRoriconInitialised&&HandTesting) {
  //   if (HandDirection) {
  //     *HandManipulateValue =
  //         MotorPower *
  //         ((HandLimitAngleRad[0] - SensorStates.HandRotationRad) /
  //          HandLimitAngleRad[0]);
  //   } else {
  //     *HandManipulateValue =
  //         -MotorPower *
  //         ((HandLimitAngleRad[1] - SensorStates.HandRotationRad) /
  //          HandLimitAngleRad[0]);
  //   }
  // }
}

void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue) {
  *ShoulderManipulateValue = BothHandsData.RightStick.StickStates[0] *
                             BothHandsData.RightStick.Slider * MotorPower;
  *UpperArmManipulateValue = BothHandsData.RightStick.StickStates[1] *
                             BothHandsData.RightStick.Slider * MotorPower;

  if (BothHandsData.RightStick.ButtonState[3]) {
    if (SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], MotorPower);
  } else if (BothHandsData.RightStick.ButtonState[2]) {
    if (SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -MotorPower);
  }

  if (SwordDrawCompleted) return;
  if (BothHandsData.RightStick.ButtonState[1] ||
      BothHandsData.RightStick.ButtonState[0]) {
    if (SensorStates.HandLimits[0])
      analogWrite(Pinmap.HandMotors[0], MotorPower);
  } else {
    if (SensorStates.HandLimits[1])
      analogWrite(Pinmap.HandMotors[1], -MotorPower);
  }
}
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue) {}

void SwordDrawingProcedure() {
  ///このモーターをこのくらい動かし、そのモーターをあのくらい動かし、
}