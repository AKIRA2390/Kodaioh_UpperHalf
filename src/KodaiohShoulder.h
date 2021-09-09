#include <Arduino.h>
#include <ESP32Servo.h>

#include "AMT102V.h"
#include "ControlStick.h"

namespace kodaioh_shoulder {
typedef struct Pinmap_t {
  // output pins //
  // plus, minus
  const int ShoulderMotors[2] = {32, 33};
  const int UpperArmMotors[2] = {25, 26};

  // input pins //
  const int ShoulderRoricon[2] = {36, 39};
  const int UpperArmRoricon[2] = {34, 35};
  // max,min
  // const int ShoulderLimits[2] = {23, 22};
  const int ShoulderLimits = 22;
  const int UpperArmLimits[2] = {12, 13};
} Pinmap_t;

typedef struct ShoulderSensorStates {
  bool ShoulderLimits, UpperArmLimits[2];
  double ShoulderRotationRad, UpperArmRotationRad, ElbowRotationRad;
} ShoulderSensorStates;

extern Pinmap_t Pinmap;
extern ShoulderSensorStates SensorStates;

extern controlstick::ControlStick *Stick;
extern controlstick::BothHandsData_t BothHandsData;
extern controlstick::InputData_t *InputData;

extern AMT102V *ShoulderRoricon, *UpperArmRoricon;

extern const int MotorPower;
extern const double ShoulderReductionRatio;
extern const double UpperArmReductionRatio;
extern const double ShoulderLimitAngleRad[2];
extern const double UpperArmLimitAngleRad[2];

extern bool IsDirty;
extern bool ShoulderRoriconInitialised, UpperArmRoriconInitialised;

void SendCB(const uint8_t *mac_addr, esp_now_send_status_t status);
void RecvCB(const uint8_t *mac, const uint8_t *incomingData, int len);

void UpdateTestDummy(double *ShoulderManipulateValue,
                     double *UpperArmManipulateValue);
void UpdateAKIRAMethod(double *ShoulderManipulateValue,
                       double *UpperArmManipulateValue);
void UpdateTaishinMethod(double *ShoulderManipulateValue,
                         double *UpperArmManipulateValue);

void setup(controlstick::ControlStick *stick,
           controlstick::InputData_t *input_data);
void update();

}  // namespace kodaioh_shoulder