#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"
#include "KodaiohShoulder.h"

#define Debug

typedef struct RightHalfPinmap_t {
  // output pins //
  // plus, minus
  const int ElbowMotors[2] = {27, 14};
  const int HandMotors[2] = {9, 10};

  // input pins //
  const int ElbowRoricon[2] = {21, 19};
  // max,min
  const int ElbowLimits[2] = {18, 17};
  const int HandLimits[2] = {16, 4};
} RightHalfPinmap_t;

typedef struct RightHalfSensorStates {
  bool ElbowLimits[2], HandLimits[2];
  double ElbowRotationRad;
} RightHalfSensorStates;

RightHalfPinmap_t Pinmap;
RightHalfSensorStates SensorStates;

controlstick::ControlStick Sticks;
controlstick::BothHandsData_t BothHandsData;
uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};

bool ElbowRoriconInitialised = false;
bool SwordDrawInProgress = false, SwordDrawCompleted = false;

AMT102V *ElbowRoricon;

const double ElbowReductionRatio = 3 / 760;
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
                         double *UpperArmManipulateValue);

void SwordDrawingProcedure();

void setup() {
  Sticks.ThisSends2Robot(LeftHalfAddress, kodaioh_shoulder::SendCB);
  Sticks.ThisReceives(kodaioh_shoulder::RecvCB);
  Sticks.SetupConnection();

  kodaioh_shoulder::setup(&Sticks, &BothHandsData.RightStick);

  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ElbowMotors[i], OUTPUT);

    pinMode(Pinmap.ElbowRoricon[i], INPUT);

    pinMode(Pinmap.ElbowLimits[i], INPUT_PULLUP);
    pinMode(Pinmap.HandLimits[i], INPUT_PULLUP);
  }

  ElbowRoricon = new AMT102V(Pinmap.ElbowRoricon[0], Pinmap.ElbowRoricon[1]);
  ElbowRoricon->setup(0b0000);
  attachInterrupt(Pinmap.ElbowRoricon[0], ElbowRoriconInterrupter, RISING);
  attachInterrupt(Pinmap.ElbowRoricon[1], ElbowRoriconInterrupter, RISING);
}

void loop() {
  if (SensorStates.ElbowLimits[1]) {
    ElbowRoriconInitialised = true;
    ElbowRoricon->resetRotation();
  }

  if (ElbowRoriconInitialised)
    SensorStates.ElbowRotationRad =
        (ElbowRoricon->getRotationsDouble() / ElbowReductionRatio) * 2 * PI;

  kodaioh_shoulder::update();

  RightArmUpdate();
  if (kodaioh_shoulder::IsDirty) {
  kodaioh_shoulder::UpdateWhenDirty();
    LeftArmUpdate();
    kodaioh_shoulder::IsDirty = false;
  }
}

void RightArmUpdate() {
  double ShoulderManipulateValue, UpperArmManipulateValue;
  double ElbowManipulateValue;

  if ((BothHandsData.RightStick.ButtonState[4] && !SwordDrawCompleted) ||
      SwordDrawInProgress)
    SwordDrawingProcedure();

  UpdateTestDummy(&UpperArmManipulateValue, &UpperArmManipulateValue,
                  &ElbowManipulateValue);

  if (ElbowManipulateValue > 0) {
    if (!SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], ElbowManipulateValue);
  } else {
    if (!SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -ElbowManipulateValue);
  }
}

void LeftArmUpdate() { Sticks.SendData2Robot(BothHandsData); }

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue) {
  const bool ElbowTesting = false, HandTesting = false;
  static bool ElbowDirection = true, HandDirection = true;

  kodaioh_shoulder::UpdateTestDummy(UpperArmManipulateValue,
                                    UpperArmManipulateValue);

  if (SensorStates.ElbowLimits[0]) {
    ElbowDirection = false;
  } else if (SensorStates.ElbowLimits[1]) {
    ElbowDirection = true;
  }

  if (ElbowRoriconInitialised && ElbowTesting) {
    if (ElbowDirection) {
      *ElbowManipulateValue =
          kodaioh_shoulder::MotorPower *
          ((ElbowLimitAngleRad[0] - SensorStates.ElbowRotationRad) /
           ElbowLimitAngleRad[0]);
    } else {
      *ElbowManipulateValue =
          -kodaioh_shoulder::MotorPower *
          ((ElbowLimitAngleRad[1] - SensorStates.ElbowRotationRad) /
           ElbowLimitAngleRad[0]);
    }
  }
}

void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue) {
  kodaioh_shoulder::UpdateAKIRAMethod(UpperArmManipulateValue,
                                      UpperArmManipulateValue);

  if (BothHandsData.RightStick.ButtonState[3]) {
    if (SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], kodaioh_shoulder::MotorPower);
  } else if (BothHandsData.RightStick.ButtonState[2]) {
    if (SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -kodaioh_shoulder::MotorPower);
  }

  if (SwordDrawCompleted) return;
  if (BothHandsData.RightStick.ButtonState[1] ||
      BothHandsData.RightStick.ButtonState[0]) {
    if (SensorStates.HandLimits[0])
      analogWrite(Pinmap.HandMotors[0], kodaioh_shoulder::MotorPower);
  } else {
    if (SensorStates.HandLimits[1])
      analogWrite(Pinmap.HandMotors[1], -kodaioh_shoulder::MotorPower);
  }
}
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue) {
  kodaioh_shoulder::UpdateTaishinMethod(UpperArmManipulateValue,
                                      UpperArmManipulateValue);
                         }

void SwordDrawingProcedure() {
  ///このモーターをこのくらい動かし、そのモーターをあのくらい動かし、
}