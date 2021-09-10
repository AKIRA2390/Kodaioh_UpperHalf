#include "KodaiohShoulder.h"

#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"

namespace kodaioh_shoulder {

Pinmap_t Pinmap;
ShoulderSensorStates SensorStates;

controlstick::ControlStick *Stick;
controlstick::BothHandsData_t *BothHandsData;
controlstick::InputData_t *InputData;

const int MotorPower = 200;
const double ShoulderReductionRatio = 2 / 9;
const double UpperArmReductionRatio = 78 / 194;
const double ShoulderLimitAngleRad[2] = {140 * DEG_TO_RAD, -80 * DEG_TO_RAD};
const double UpperArmLimitAngleRad[2] = {90 * DEG_TO_RAD, 0 * DEG_TO_RAD};

bool IsDirty = false;
bool ShoulderRoriconInitialised = false, UpperArmRoriconInitialised = false;

AMT102V *ShoulderRoricon, *UpperArmRoricon;

void ShoulderRoriconInterrupter() { ShoulderRoricon->update(); }
void UpperArmRoriconInterrupter() { UpperArmRoricon->update(); }

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
  stickDumpData(BothHandsData.RightStick);
  Serial.println("Left Hand\tLeft Hand\tLeft Hand\t");
  stickDumpData(BothHandsData.LeftStick);
  Serial.println("\n");
#endif
}

void setup(controlstick::ControlStick *stick,
           controlstick::BothHandsData_t *both_hands_data,
           controlstick::InputData_t *input_data) {
  Stick = stick;
  InputData = input_data;
  BothHandsData = both_hands_data;

  Stick->ThisReceives(RecvCB);
  Stick->SetupConnection();
  pinMode(Pinmap.ShoulderLimits, INPUT_PULLUP);
  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ShoulderMotors[i], OUTPUT);
    pinMode(Pinmap.UpperArmMotors[i], OUTPUT);

    pinMode(Pinmap.ShoulderRoricon[i], INPUT);
    pinMode(Pinmap.UpperArmRoricon[i], INPUT);

    pinMode(Pinmap.UpperArmLimits[i], INPUT_PULLUP);
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
}

void update() {
  for (int i = 0; i < 2; i++) {
    SensorStates.ShoulderLimits = digitalRead(Pinmap.ShoulderLimits);
    SensorStates.UpperArmLimits[i] = digitalRead(Pinmap.UpperArmLimits[i]);
  }

  if (SensorStates.ShoulderLimits) {
    ShoulderRoriconInitialised = true;
    ShoulderRoricon->resetRotation();
  }
  if (SensorStates.UpperArmLimits[1]) {
    UpperArmRoriconInitialised = true;
    UpperArmRoricon->resetRotation();
  }

  if (ShoulderRoriconInitialised)
    SensorStates.ShoulderRotationRad =
        (ShoulderRoricon->getRotationsDouble() / ShoulderReductionRatio) * 2 *
        PI;
  if (UpperArmRoriconInitialised)
    SensorStates.UpperArmRotationRad =
        (UpperArmRoricon->getRotationsDouble() / UpperArmReductionRatio) * 2 *
        PI;
#ifdef Debug
  Serial.print("Shoulder Rotation:");
  Serial.println(SensorStates.ShoulderRotationRad * RAD_TO_DEG);
  Serial.print("UpperArm Rotation:");
  Serial.println(SensorStates.UpperArmRotationRad * RAD_TO_DEG);
  Serial.print("Elbow Rotation:");
  Serial.println(SensorStates.ElbowRotationRad * RAD_TO_DEG);
  Serial.println("\n");
#endif
}
void UpdateWhenDirty() {
  if (IsDirty) {
    double ShoulderManipulateValue, UpperArmManipulateValue;
    UpdateAKIRAMethod(&UpperArmManipulateValue, &UpperArmManipulateValue);
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
  }
}

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue) {
  const bool ShoulderTesting = true, UpperArmTesting = false;
  static bool ShoulderDirection = true, UpperArmDirection = true;

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
}
void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue) {
  *ShoulderManipulateValue =
      InputData->StickStates[0] * InputData->Slider * MotorPower;
  *UpperArmManipulateValue =
      InputData->StickStates[1] * InputData->Slider * MotorPower;
}

void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue) {}
}  // namespace kodaioh_shoulder