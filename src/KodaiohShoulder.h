#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"

//
#include "PID4arduino.h"
//

namespace kodaioh_shoulder {
typedef struct Pinmap_t {
  // output pins //
  // plus, minus
  const int ShoulderMotors[2] = {33, 32};
  const int UpperArmMotors[2] = {26, 25};

  // input pins //
  const int ShoulderRoricon[2] = {36, 39};
  const int UpperArmRoricon[2] = {22, 16};
  // max,min
  // const int ShoulderLimit[2] = {23, 22};
  const int ShoulderLimit = 23;
  const int UpperArmLimit[2] = {13, 4};
} Pinmap_t;

typedef struct ShoulderSensorStates {
  bool ShoulderLimit, UpperArmLimit[2];
  double ShoulderRotationRad, UpperArmRotationRad;
  double ShoulderRoriconRotationPrev, UpperArmRoriconRotationPrev;
} ShoulderSensorStates;

extern Pinmap_t Pinmap;
extern ShoulderSensorStates SensorStates;

extern controlstick::ControlStick *Stick;
extern controlstick::BothHandsData_t BothHandsData;
extern controlstick::InputData_t *InputData;

extern const int MotorPower;
extern const int ShoulderMotorPower;
extern const int UpperArmMotorPower;
extern const double ShoulderReductionRatio;
extern const double UpperArmReductionRatio;
extern const double ShoulderLimitAngleRad[2];
extern const double UpperArmLimitAngleRad[2];

extern AMT102V *ShoulderRoricon, *UpperArmRoricon;

//
extern PID4Arduino::PID4arduino<int> *ShoulderPID, *UpperArmPID;
extern PID4Arduino::PIDGain_t ShoulderPIDGains, UpperArmPIDGains;

extern int ShoulderTargetDeg, UpperArmTargetDeg;
//

extern const int MotorPower;
extern const double ShoulderReductionRatio;
extern const double UpperArmReductionRatio;
extern const double ShoulderLimitAngleRad[2];
extern const double UpperArmLimitAngleRad[2];

extern bool IsDirty;
extern bool ShoulderRoriconInitialised, UpperArmRoriconInitialised;

void SendCB(const uint8_t *mac_addr, esp_now_send_status_t status);
void RecvCB(const uint8_t *mac, const uint8_t *incomingData, int len);

//
void setPIDGains(PID4Arduino::PIDGain_t ShoulderPIDGains,
                 PID4Arduino::PIDGain_t UpperArmPIDGains);
//

void setup(controlstick::ControlStick *stick,
           controlstick::InputData_t *input_data,
           bool ShoulderRoriconInvert = false,
           bool UpperArmRoriconInvert = false);
void update();

void UpdateWhenDirty(double ShoulderManipulateValue,
                     double UpperArmManipulateValue);

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue,
                     bool ShoulderTesting = false,
                     bool UpperArmTesting = false);
void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue);
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue);

void Update4ShoulderUnitReset(double *ShoulderManipulateValue,
                              double *UpperArmManipulateValue);

void GetBothHandsData(controlstick::BothHandsData_t *Value);
}  // namespace kodaioh_shoulder