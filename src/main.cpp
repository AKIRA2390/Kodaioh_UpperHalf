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
  const int ElbowMotors[2] = {14, 27};

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

// uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};
uint8_t LeftHalfAddress[] = {0x24, 0x0A, 0xC4, 0xF9, 0x40, 0xD0};

double ShoulderManipulateValue = 0, UpperArmManipulateValue = 0,
       ElbowManipulateValue = 0;
const int ElbowMotorPower = 150;

bool ElbowRoriconInitialised = false;
bool SwordDrawInProgress = false, SwordDrawCompleted = false;

//
extern PID4Arduino::PID4arduino<int> ElbowPID;
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

void SwordDrawingProcedure(double *ShoulderManipulateValue,
                           double *UpperArmManipulateValue,
                           double *ElbowManipulateValue);
void PrintVariousThings();

void setup() {
  Serial.begin(115200);
  // Sticks.ThisSends2Robot(LeftHalfAddress, );
  Serial.println("started!");

  //
  ShoulderPIDGain.KP =
      kodaioh_shoulder::ShoulderMotorPower /
      (abs(kodaioh_shoulder::ShoulderLimitAngleRad[0] * RAD_TO_DEG) +
       abs(kodaioh_shoulder::ShoulderLimitAngleRad[1] * RAD_TO_DEG));
  ShoulderPIDGain.KI = 30;
  ShoulderPIDGain.KD = 30;

  UpperArmPIDGain.KP =
      kodaioh_shoulder::UpperArmMotorPower /
      (abs(kodaioh_shoulder::UpperArmLimitAngleRad[0] * RAD_TO_DEG) +
       abs(kodaioh_shoulder::UpperArmLimitAngleRad[1] * RAD_TO_DEG));
  UpperArmPIDGain.KI = 30;
  UpperArmPIDGain.KD = 30;

  ElbowPIDGain.KP = ElbowMotorPower / (abs(ElbowLimitAngleRad[0] * RAD_TO_DEG) +
                                       abs(ElbowLimitAngleRad[1] * RAD_TO_DEG));
  ElbowPIDGain.KI = 30;
  ElbowPIDGain.KD = 30;

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
}

void loop() {
  Serial.println("update");

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

  Motion.update(AngleDatas);
  //

  kodaioh_shoulder::update();

  // Sticks.SendData2Robot(BothHandsData);

#ifdef Debug
  RightArmUpdate();
#endif

  if (kodaioh_shoulder::IsDirty) {
    Serial.println("is dirty");
    kodaioh_shoulder::GetBothHandsData(&BothHandsData);

#ifndef Debug
    RightArmUpdate();
#endif

    LeftArmUpdate();

    kodaioh_shoulder::UpdateWhenDirty(ShoulderManipulateValue,
                                      UpperArmManipulateValue);
    PrintVariousThings();
    kodaioh_shoulder::IsDirty = false;

    Serial.println("Right Hands Received Data:");
    Sticks.DumpData(BothHandsData.RightStick);
    Serial.println("Left Hands Received Data:");
    Sticks.DumpData(BothHandsData.LeftStick);
  }

  ElbowPID.update(ElbowTargetDeg, SensorStates.ElbowRotationRad);

  delay(100);
}

void RightArmUpdate() {
  if ((BothHandsData.RightStick.ButtonState[4] && !SwordDrawCompleted) ||
      SwordDrawInProgress)
    SwordDrawingProcedure(&ShoulderManipulateValue, &UpperArmManipulateValue,
                          &ElbowManipulateValue);

  Serial.println("right arm update");

#ifdef Debug
  UpdateTestDummy(&ShoulderManipulateValue, &UpperArmManipulateValue,
                  &ElbowManipulateValue);
#else
  UpdateAKIRAMethod(&ShoulderManipulateValue, &UpperArmManipulateValue);
#endif

  if (ElbowManipulateValue > 0) {
    if (!SensorStates.ElbowLimit[0]) {
      analogWrite(Pinmap.ElbowMotors[0], ElbowManipulateValue);
      analogWrite(Pinmap.ElbowMotors[1], 0);
    } else {
      analogWrite(Pinmap.ElbowMotors[0], 0);
      analogWrite(Pinmap.ElbowMotors[1], 0);
    }
  } else {
    if (!SensorStates.ElbowLimit[1]) {
      analogWrite(Pinmap.ElbowMotors[0], 0);
      analogWrite(Pinmap.ElbowMotors[1], ElbowManipulateValue);
    } else {
      analogWrite(Pinmap.ElbowMotors[0], 0);
      analogWrite(Pinmap.ElbowMotors[1], 0);
    }
  }
}

void LeftArmUpdate() {
  //  Sticks.SendData2Robot(BothHandsData);
}

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue) {
  const bool ShoulderTesting = false, UpperArmTesting = true,
             ElbowTesting = false;
  static bool ElbowDirection = true;

  kodaioh_shoulder::IsDirty = true;

  Serial.println("right arm test dummy");
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

void SwordDrawingProcedure(double *ShoulderManipulateValue,
                           double *UpperArmManipulateValue,
                           double *ElbowManipulateValue) {
  ///このモーターをこのくらい動かし、そのモーターをあのくらい動かし、
  Update4RightArmReset(ShoulderManipulateValue, UpperArmManipulateValue,
                       ElbowManipulateValue);
}

void PrintVariousThings() {
  // Serial.println("\n");
  // Serial.print("Shoulder Roricon Raw:\t\t");
  // Serial.println(kodaioh_shoulder::ShoulderRoricon->getRotationsDouble());
  Serial.print("Shoulder roricon initialized:\t");
  Serial.println(kodaioh_shoulder::ShoulderRoriconInitialised ? "true"
                                                              : "false");
  Serial.print("Shoulder Manipulate Value:\t");
  Serial.println(ShoulderManipulateValue);
  Serial.print("Shoulder Angle Deg:\t\t");
  Serial.println(kodaioh_shoulder::SensorStates.ShoulderRotationRad *
                 RAD_TO_DEG);
  // Serial.print("Shoulder Limit Angle Deg:\t");
  // Serial.print(kodaioh_shoulder::ShoulderLimitAngleRad[0] * RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.println(kodaioh_shoulder::ShoulderLimitAngleRad[1] * RAD_TO_DEG);
  Serial.print("Shoulder Limit Stats:\t");
  Serial.print(kodaioh_shoulder::SensorStates.ShoulderLimit);
  // Serial.print("Shoulder Error Value:\t\t");
  // Serial.print((kodaioh_shoulder::ShoulderLimitAngleRad[0] -
  //               kodaioh_shoulder::SensorStates.ShoulderRotationRad) *
  //              RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.print(kodaioh_shoulder::MotorPower *
  //              ((kodaioh_shoulder::ShoulderLimitAngleRad[0] -
  //                kodaioh_shoulder::SensorStates.ShoulderRotationRad) /
  //               kodaioh_shoulder::ShoulderLimitAngleRad[0]));
  Serial.println("\n");

  // Serial.print("UpperArm Roricon Raw:\t\t");
  // Serial.println(kodaioh_shoulder::UpperArmRoricon->getRotationsDouble());
  Serial.print("UpperArm roricon initialized:\t");
  Serial.println(kodaioh_shoulder::UpperArmRoriconInitialised ? "true"
                                                              : "false");
  Serial.print("UpperArm Manipulate Value:\t");
  Serial.println(UpperArmManipulateValue);
  Serial.print("UpperArm Angle Deg:\t\t");
  Serial.println(kodaioh_shoulder::SensorStates.UpperArmRotationRad *
                 RAD_TO_DEG);
  // Serial.print("UpperArm Limit Angle Deg:\t");
  // Serial.print(kodaioh_shoulder::UpperArmLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.println(kodaioh_shoulder::UpperArmLimitAngleRad[1]*RAD_TO_DEG);

  Serial.print("UpperArm Limit Stats:\t");
  Serial.print(kodaioh_shoulder::SensorStates.UpperArmLimit[0]);
  Serial.print("\t");
  Serial.println(kodaioh_shoulder::SensorStates.UpperArmLimit[1]);
  // Serial.print("UpperArm Error Value:\t\t");
  // Serial.print((kodaioh_shoulder::UpperArmLimitAngleRad[0] -
  // kodaioh_shoulder::SensorStates.UpperArmRotationRad)*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.print(kodaioh_shoulder::MotorPower*((kodaioh_shoulder::UpperArmLimitAngleRad[0]
  // -
  // kodaioh_shoulder::SensorStates.UpperArmRotationRad)/kodaioh_shoulder::UpperArmLimitAngleRad[0]));
  Serial.println("\n");

  Serial.print("Elbow Roricon Initialized:\t");
  Serial.println(ElbowRoriconInitialised ? "true" : "false");
  // Serial.print("Elbow Roricon Raw:\t\t");
  // Serial.println(ElbowRoricon->getRotationsDouble());
  Serial.print("Elbow Manipulate Value:\t\t");
  Serial.println(ElbowManipulateValue);
  Serial.print("Elbow Angle Deg:\t\t");
  Serial.println(SensorStates.ElbowRotationRad * RAD_TO_DEG);
  // Serial.print("Elbow Limit max\t\t");
  // Serial.println(SensorStates.ElbowLimit[0]);
  // Serial.print("Elbow Limit min\t\t");
  // Serial.println(SensorStates.ElbowLimit[1]);
  // Serial.print("Elbow Limit Angle Deg:\t\t");
  // Serial.print(ElbowLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.println(ElbowLimitAngleRad[1]*RAD_TO_DEG);
  // Serial.print("Elbow Error Value:\t\t");
  // Serial.print((ElbowLimitAngleRad[0] -
  // SensorStates.ElbowRotationRad)*RAD_TO_DEG); Serial.print("\t");
  // Serial.print(kodaioh_shoulder::MotorPower*((ElbowLimitAngleRad[0] -
  // SensorStates.ElbowRotationRad)/ElbowLimitAngleRad[0]));
  Serial.println("\n");
}