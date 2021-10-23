#include "KodaiohShoulder.h"

#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"
#include "PID4arduino.h"

// #define Debug

namespace kodaioh_shoulder {

Pinmap_t Pinmap;
ShoulderSensorStates SensorStates = {};

controlstick::ControlStick *Stick;
controlstick::BothHandsData_t BothHandsData;
controlstick::InputData_t *InputData;

const int MotorPower = 200;
const int ShoulderMotorPower = 100;
const int UpperArmMotorPower = 150;
const double ShoulderReductionRatio = 2. / 9;
const double UpperArmReductionRatio = 97. / 39;
const double ShoulderLimitAngleRad[2] = {90 * DEG_TO_RAD, -60 * DEG_TO_RAD};
const double UpperArmLimitAngleRad[2] = {90 * DEG_TO_RAD, 0 * DEG_TO_RAD};

bool IsDirty = false;
bool ShoulderRoriconInitialised = false, UpperArmRoriconInitialised = false;

AMT102V *ShoulderRoricon, *UpperArmRoricon;

void ShoulderRoriconInterrupter() { ShoulderRoricon->update(); }
void UpperArmRoriconInterrupter() { UpperArmRoricon->update(); }

//
PID4Arduino::PID4arduino<int> ShoulderPID, UpperArmPID;
PID4Arduino::PIDGain_t ShoulderPIDGains = {}, UpperArmPIDGains = {};

int ShoulderTargetDeg = 0, UpperArmTargetDeg = 0;
//

void SendCB(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // #ifdef Debug
  //   Serial.print("\r\nLast Packet Send Status:\t");
  //   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
  //                                                 : "Delivery Fail");
  // #endif
}

void RecvCB(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&BothHandsData, incomingData, sizeof(BothHandsData));
  // BothHandsData = ((controlstick::BothHandsData_t *)incomingData);

  IsDirty = true;
  // #ifdef Debug
  // Serial.println("Data Received!");
  // Serial.println("Right Hand\tRight Hand\tRight Hand\t");
  // Stick->DumpData(((controlstick::BothHandsData_t
  // *)incomingData)->RightStick); Serial.println("Left Hand\tLeft Hand\tLeft
  // Hand\t"); Stick->DumpData(((controlstick::BothHandsData_t
  // *)incomingData)->LeftStick); Serial.println("\n"); #endif
}

//
void setPIDGains(PID4Arduino::PIDGain_t shoulderPIDGains,
                 PID4Arduino::PIDGain_t upperArmPIDGains) {
  memcpy(&ShoulderPIDGains, &shoulderPIDGains, sizeof(ShoulderPIDGains));
  memcpy(&UpperArmPIDGains, &upperArmPIDGains, sizeof(UpperArmPIDGains));
}
//

void setup(controlstick::ControlStick *stick,
           controlstick::InputData_t *input_data, bool ShoulderRoriconInvert,
           bool UpperArmRoriconInvert) {
  Stick = stick;
  InputData = input_data;

  Stick->ThisReceives(RecvCB);
  Stick->SetupConnection();

  pinMode(Pinmap.ShoulderLimit, INPUT_PULLUP);
  for (int i = 0; i < 2; i++) {
    pinMode(Pinmap.ShoulderMotors[i], OUTPUT);
    pinMode(Pinmap.UpperArmMotors[i], OUTPUT);

    pinMode(Pinmap.ShoulderRoricon[i], INPUT);
    pinMode(Pinmap.UpperArmRoricon[i], INPUT);

    pinMode(Pinmap.UpperArmLimit[i], INPUT_PULLUP);
  }

  ShoulderRoricon =
      new AMT102V(Pinmap.ShoulderRoricon[0], Pinmap.ShoulderRoricon[1],
                  ShoulderRoriconInvert);
  ShoulderRoricon->setup(0b0000);

  attachInterrupt(digitalPinToInterrupt(Pinmap.ShoulderRoricon[0]),
                  ShoulderRoriconInterrupter, RISING);
  attachInterrupt(digitalPinToInterrupt(Pinmap.ShoulderRoricon[1]),
                  ShoulderRoriconInterrupter, RISING);

  UpperArmRoricon =
      new AMT102V(Pinmap.UpperArmRoricon[0], Pinmap.UpperArmRoricon[1],
                  UpperArmRoriconInvert);
  UpperArmRoricon->setup(0b1100);

  attachInterrupt(Pinmap.UpperArmRoricon[0], UpperArmRoriconInterrupter,
                  RISING);
  attachInterrupt(Pinmap.UpperArmRoricon[1], UpperArmRoriconInterrupter,
                  RISING);

  //
  ShoulderPID.setGains(ShoulderPIDGains);
  UpperArmPID.setGains(UpperArmPIDGains);
  //
}

void update() {
  for (int i = 0; i < 2; i++) {
    SensorStates.ShoulderLimit = !digitalRead(Pinmap.ShoulderLimit);
    SensorStates.UpperArmLimit[i] = !digitalRead(Pinmap.UpperArmLimit[i]);
  }

  if (SensorStates.ShoulderLimit) {
    ShoulderRoriconInitialised = true;
    ShoulderRoricon->resetRotation();
  }

  if (SensorStates.UpperArmLimit[1]) {
    UpperArmRoriconInitialised = true;
    SensorStates.UpperArmRotationRad = UpperArmLimitAngleRad[1];
  }
  if (SensorStates.UpperArmLimit[0]) {
    UpperArmRoriconInitialised = true;
    SensorStates.UpperArmRotationRad = UpperArmLimitAngleRad[0];
  }

  double ShoulderRoriconRotationDelta, UpperArmRoriconRotationDelta;
  ShoulderRoriconRotationDelta = SensorStates.ShoulderRoriconRotationPrev -
                                 ShoulderRoricon->getRotationsDouble();
  UpperArmRoriconRotationDelta = SensorStates.UpperArmRoriconRotationPrev -
                                 UpperArmRoricon->getRotationsDouble();

  if (ShoulderRoriconInitialised)
    SensorStates.ShoulderRotationRad +=
        (ShoulderRoriconRotationDelta / ShoulderReductionRatio) * 2 * PI;

  if (UpperArmRoriconInitialised)
    SensorStates.UpperArmRotationRad +=
        (UpperArmRoriconRotationDelta / UpperArmReductionRatio) * 2 * PI;

  SensorStates.ShoulderRoriconRotationPrev =
      ShoulderRoricon->getRotationsDouble();
  SensorStates.UpperArmRoriconRotationPrev =
      UpperArmRoricon->getRotationsDouble();

  //
  // ShoulderPID.update(ShoulderTargetDeg, SensorStates.ShoulderRotationRad);
  // UpperArmPID.update(UpperArmTargetDeg, SensorStates.UpperArmRotationRad);
  //
}

void UpdateWhenDirty(double ShoulderManipulateValue,
                     double UpperArmManipulateValue) {
  if (!IsDirty) return;

  // UpdateAKIRAMethod(UpperArmManipulateValue, UpperArmManipulateValue);
  // UpdateTaishinMethod(UpperArmManipulateValue, UpperArmManipulateValue);
  if (ShoulderRoriconInitialised) {
    if (ShoulderManipulateValue >= 0) {
      if (SensorStates.ShoulderRotationRad < ShoulderLimitAngleRad[0]) {
        // Serial.print("Shoulder Output Value\t");
        // Serial.println(ShoulderManipulateValue);

        analogWrite(Pinmap.ShoulderMotors[0], abs(ShoulderManipulateValue));
        analogWrite(Pinmap.ShoulderMotors[1], 0);
      } else {
        // Serial.print("Shoulder Output Value\t");
        // Serial.print(0);
        // Serial.println("\tShoulder Max Limit");

        analogWrite(Pinmap.ShoulderMotors[0], 0);
        analogWrite(Pinmap.ShoulderMotors[1], 0);
      }
    } else if (ShoulderManipulateValue < 0) {
      if (ShoulderLimitAngleRad[1] < SensorStates.ShoulderRotationRad) {
        // Serial.print("Shoulder Output Value\t");
        // Serial.println(-ShoulderManipulateValue);

        analogWrite(Pinmap.ShoulderMotors[0], 0);
        analogWrite(Pinmap.ShoulderMotors[1], abs(ShoulderManipulateValue));
      } else {
        // Serial.print("Shoulder Output Value\t");
        // Serial.print(0);
        // Serial.println("\tShoulder Min Limit");

        analogWrite(Pinmap.ShoulderMotors[0], 0);
        analogWrite(Pinmap.ShoulderMotors[1], 0);
      }
    }
  } else {
    // Serial.print("Shoulder Output Value\t");
    // Serial.print(0);
    // Serial.println("\tShoulder Not Initialised");

    analogWrite(Pinmap.ShoulderMotors[0], 0);
    analogWrite(Pinmap.ShoulderMotors[1], 0);
  }

  if (UpperArmRoriconInitialised) {
    if (UpperArmManipulateValue >= 0) {
      if (SensorStates.UpperArmLimit[0] ||
          SensorStates.UpperArmRotationRad > UpperArmLimitAngleRad[0]) {
        // Serial.print("UpperArm Output Value\t");
        // Serial.print(0);
        // Serial.println("\tUpperArm Max Limit");

        analogWrite(Pinmap.UpperArmMotors[0], 0);
        analogWrite(Pinmap.UpperArmMotors[1], 0);
      } else {
        // Serial.print("UpperArm Output Value\t");
        // Serial.print(UpperArmManipulateValue);
        // Serial.println("\tplus");

        analogWrite(Pinmap.UpperArmMotors[0], abs(UpperArmManipulateValue));
        analogWrite(Pinmap.UpperArmMotors[1], 0);
      }
    } else if (UpperArmManipulateValue < 0) {
      if (SensorStates.UpperArmLimit[1] ||
          UpperArmLimitAngleRad[1] > SensorStates.UpperArmRotationRad) {
        // Serial.print("UpperArm Output Value\t");
        // Serial.println(0);
        // Serial.println("\tUpperArm Min Limit");

        analogWrite(Pinmap.UpperArmMotors[0], 0);
        analogWrite(Pinmap.UpperArmMotors[1], 0);
      } else {
        // Serial.print("UpperArm Output Value\t");
        // Serial.print(-UpperArmManipulateValue);
        // Serial.println("\tminus");

        analogWrite(Pinmap.UpperArmMotors[0], 0);
        analogWrite(Pinmap.UpperArmMotors[1], abs(UpperArmManipulateValue));
      }
    }
  } else {
    // Serial.print("UpperArm Output Value\t");
    // Serial.println(0);
    // Serial.println("\tUpperArm Not Initialised");

    analogWrite(Pinmap.UpperArmMotors[0], 0);
    analogWrite(Pinmap.UpperArmMotors[1], 0);
  }
}

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue, bool ShoulderTesting,
                     bool UpperArmTesting) {
  static bool ShoulderDirection = true, UpperArmDirection = true;

  Serial.println("shoulder unit test dummy");

  // Serial.println("//////////////////////////");
  // Serial.println("UpperArm direction");
  // Serial.println(UpperArmDirection);
  // Serial.println("//////////////////////////");

  if (SensorStates.ShoulderRotationRad > ShoulderLimitAngleRad[0]) {
    ShoulderDirection = false;
  } else if (ShoulderLimitAngleRad[1] > SensorStates.ShoulderRotationRad) {
    ShoulderDirection = true;
  }
  if (SensorStates.UpperArmLimit[0]) {
    UpperArmDirection = false;
  } else if (SensorStates.UpperArmLimit[1]) {
    UpperArmDirection = true;
  }

  if (ShoulderRoriconInitialised && ShoulderTesting) {
    if (ShoulderDirection) {
      *ShoulderManipulateValue = ShoulderMotorPower;
    } else {
      *ShoulderManipulateValue = -ShoulderMotorPower;
    }
  }

  if (UpperArmRoriconInitialised && UpperArmTesting) {
    if (UpperArmDirection) {
      *UpperArmManipulateValue = UpperArmMotorPower;
    } else {
      *UpperArmManipulateValue = -UpperArmMotorPower;
    }
  }

  //    *UpperArmManipulateValue = UpperArmMotorPower *
  //     ((UpperArmLimitAngleRad[0] - SensorStates.UpperArmRotationRad)
  //     /
  //      UpperArmLimitAngleRad[0]);
}

void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue) {
  *ShoulderManipulateValue =
      InputData->StickStates[1] * InputData->Slider * ShoulderMotorPower;
  *UpperArmManipulateValue =
      InputData->StickStates[0] * InputData->Slider * UpperArmMotorPower;
}

void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue) {
  // ShoulderTargetDeg = 20;

  *ShoulderManipulateValue = ShoulderPID.GetValue();
  *UpperArmManipulateValue = UpperArmPID.GetValue();
}

void Update4ShoulderUnitReset(double *ShoulderManipulateValue,
                              double *UpperArmManipulateValue) {
  static bool ShoulderDirection = true;

  if (SensorStates.ShoulderRotationRad > 0) {
    ShoulderDirection = false;
  } else if (0 > SensorStates.ShoulderRotationRad) {
    ShoulderDirection = true;
  }

  if (ShoulderRoriconInitialised) {
    if (ShoulderDirection) {
      *ShoulderManipulateValue = ShoulderMotorPower;
    } else {
      *ShoulderManipulateValue = -ShoulderMotorPower;
    }
  }

  *UpperArmManipulateValue = -UpperArmMotorPower;
}

void GetBothHandsData(controlstick::BothHandsData_t *Value) {
  memcpy(Value, &BothHandsData, sizeof(BothHandsData));
}
}  // namespace kodaioh_shoulder
