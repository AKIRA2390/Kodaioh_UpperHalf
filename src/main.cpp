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

  // input pins //
  const int ElbowRoricon[2] = {21, 19};
  // max,min
  const int ElbowLimits[2] = {18, 17};
} RightHalfPinmap_t;

typedef struct RightHalfSensorStates {
  bool ElbowLimits[2];
  double ElbowRotationRad;
} RightHalfSensorStates;

RightHalfPinmap_t Pinmap;
RightHalfSensorStates SensorStates;

controlstick::ControlStick Sticks;
controlstick::BothHandsData_t BothHandsData;
// uint8_t LeftHalfAddress[] = {0xEC, 0x94, 0xCB, 0x6E, 0x29, 0x70};
uint8_t LeftHalfAddress[] = {0x24, 0x0A, 0xC4, 0xF9, 0x40, 0xD0};

double ShoulderManipulateValue=0, UpperArmManipulateValue=0;
double ElbowManipulateValue=0;

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
  Serial.begin(115200);
  Sticks.ThisSends2Robot(LeftHalfAddress, kodaioh_shoulder::SendCB);
  Sticks.ThisReceives(kodaioh_shoulder::RecvCB);
  Serial.println("started!");

  kodaioh_shoulder::setup(&Sticks, &BothHandsData, &BothHandsData.RightStick,true);

  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ElbowMotors[i], OUTPUT);

    pinMode(Pinmap.ElbowRoricon[i], INPUT);

    pinMode(Pinmap.ElbowLimits[i], INPUT_PULLUP);
  }

  ElbowRoricon = new AMT102V(Pinmap.ElbowRoricon[0], Pinmap.ElbowRoricon[1]);
  ElbowRoricon->setup(0b0000);
  attachInterrupt(Pinmap.ElbowRoricon[0], ElbowRoriconInterrupter, RISING);
  attachInterrupt(Pinmap.ElbowRoricon[1], ElbowRoriconInterrupter, RISING);
}

void loop() {
  Serial.println("update");

  for (int i = 0; i < 2; i++) {
    SensorStates.ElbowLimits[i] = !digitalRead(Pinmap.ElbowLimits[i]);
  }

  if (SensorStates.ElbowLimits[1]) {
    ElbowRoriconInitialised = true;
    ElbowRoricon->resetRotation();
  }

  if (ElbowRoriconInitialised)
    SensorStates.ElbowRotationRad =
        (ElbowRoricon->getRotationsDouble() / ElbowReductionRatio) * 2 * PI;

  kodaioh_shoulder::update();

  // Sticks.SendData2Robot(BothHandsData);

  RightArmUpdate();
  if (kodaioh_shoulder::IsDirty) {
    Serial.println("data received");
    kodaioh_shoulder::UpdateWhenDirty(ShoulderManipulateValue,
                                      UpperArmManipulateValue );
    LeftArmUpdate();
    kodaioh_shoulder::IsDirty = false;
  }
}

void RightArmUpdate() {
  if ((BothHandsData.RightStick.ButtonState[4] && !SwordDrawCompleted) ||
      SwordDrawInProgress)
    SwordDrawingProcedure();
  Serial.println("right arm update");
  UpdateTestDummy(&ShoulderManipulateValue, &UpperArmManipulateValue,
                  &ElbowManipulateValue);

  if (ElbowManipulateValue > 0) {
    if (!SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], ElbowManipulateValue);
  } else {
    if (!SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -ElbowManipulateValue);
  }
  
  // Serial.print("Shoulder Roricon Raw:\t\t");
  // Serial.println(kodaioh_shoulder::ShoulderRoricon->getRotationsDouble());
  // Serial.print("Shoulder roricon initialized:\t");
  // Serial.println(kodaioh_shoulder::ShoulderRoriconInitialised?"true":"false");
  // Serial.print("Shoulder Manipulate Value:\t");
  // Serial.println(ShoulderManipulateValue);
  // Serial.print("Shoulder Angle Deg:\t\t");
  // Serial.println(kodaioh_shoulder::SensorStates.ShoulderRotationRad*RAD_TO_DEG);
  // Serial.print("Shoulder Limit Angle Deg:\t");
  // Serial.print(kodaioh_shoulder::ShoulderLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.println(kodaioh_shoulder::ShoulderLimitAngleRad[1]*RAD_TO_DEG);
  // Serial.print("Shoulder Error Value:\t\t");
  // Serial.print((kodaioh_shoulder::ShoulderLimitAngleRad[0] - kodaioh_shoulder::SensorStates.ShoulderRotationRad)*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.print(kodaioh_shoulder::MotorPower*((kodaioh_shoulder::ShoulderLimitAngleRad[0] - kodaioh_shoulder::SensorStates.ShoulderRotationRad)/kodaioh_shoulder::ShoulderLimitAngleRad[0]));
  // Serial.println("\n");

  Serial.print("UpperArm roricon initialized:\t");
  Serial.println(kodaioh_shoulder::UpperArmRoriconInitialised?"true":"false");
  Serial.print("UpperArm Manipulate Value:\t");
  Serial.println(UpperArmManipulateValue);
  Serial.print("UpperArm Angle Deg:\t\t");
  Serial.println(kodaioh_shoulder::SensorStates.UpperArmRotationRad*RAD_TO_DEG);
  Serial.print("UpperArm Limit Angle Deg:\t");
  Serial.print(kodaioh_shoulder::UpperArmLimitAngleRad[0]*RAD_TO_DEG);
  Serial.print("\t");
  Serial.println(kodaioh_shoulder::UpperArmLimitAngleRad[1]*RAD_TO_DEG);
  Serial.print("UpperArm Error Value:\t\t");
  Serial.print((kodaioh_shoulder::UpperArmLimitAngleRad[0] - kodaioh_shoulder::SensorStates.UpperArmRotationRad)*RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(kodaioh_shoulder::MotorPower*((kodaioh_shoulder::UpperArmLimitAngleRad[0] - kodaioh_shoulder::SensorStates.UpperArmRotationRad)/kodaioh_shoulder::UpperArmLimitAngleRad[0]));
  Serial.println("\n");

  // Serial.print("Elbow Roricon Initialized:\t");
  // Serial.println(ElbowRoriconInitialised?"true":"false");
  // Serial.print("Elbow Roricon Raw:\t\t");
  // Serial.println(ElbowRoricon->getRotationsDouble());
  // Serial.print("Elbow Manipulate Value:\t\t");
  // Serial.println(ElbowManipulateValue);
  // Serial.print("Elbow Angle Deg:\t\t");
  // Serial.println(SensorStates.ElbowRotationRad*RAD_TO_DEG);
  // Serial.print("Elbow Limit Angle Deg:\t\t");
  // Serial.print(ElbowLimitAngleRad[0]*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.println(ElbowLimitAngleRad[1]*RAD_TO_DEG);
  // Serial.print("Elbow Error Value:\t\t");
  // Serial.print((ElbowLimitAngleRad[0] - SensorStates.ElbowRotationRad)*RAD_TO_DEG);
  // Serial.print("\t");
  // Serial.print(kodaioh_shoulder::MotorPower*((ElbowLimitAngleRad[0] - SensorStates.ElbowRotationRad)/ElbowLimitAngleRad[0]));
  // Serial.println("\n");
}

void LeftArmUpdate() { Sticks.SendData2Robot(BothHandsData); }

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     double *ElbowManipulateValue) {
  const bool ElbowTesting = false;
  static bool ElbowDirection = true;

  kodaioh_shoulder::IsDirty = true;
  
  Serial.println("right arm test dummy");
  kodaioh_shoulder::UpdateTestDummy(ShoulderManipulateValue,
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
  kodaioh_shoulder::UpdateAKIRAMethod(ShoulderManipulateValue,
                                      UpperArmManipulateValue);

  if (BothHandsData.RightStick.ButtonState[3]) {
    if (SensorStates.ElbowLimits[0])
      analogWrite(Pinmap.ElbowMotors[0], kodaioh_shoulder::MotorPower);
  } else if (BothHandsData.RightStick.ButtonState[2]) {
    if (SensorStates.ElbowLimits[1])
      analogWrite(Pinmap.ElbowMotors[1], -kodaioh_shoulder::MotorPower);
  }
}
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue) {
  kodaioh_shoulder::UpdateTaishinMethod(ShoulderManipulateValue,
                                        UpperArmManipulateValue);
}

void SwordDrawingProcedure() {
  ///このモーターをこのくらい動かし、そのモーターをあのくらい動かし、
}