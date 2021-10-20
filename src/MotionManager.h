#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "KodaiohShoulder.h"
#include "PID4arduino.h"

namespace motionmanager {
class MotionManager {
 private:
  PID4Arduino::PID4arduino<int> ShoulderMotionPID, ShoulderMotionPID,
      ElbowMotionPID;
  PID4Arduino::PIDGain_t ShoulderMotionGains, UpperArmMotionGains,
      ElbowMotionGains;

  const bool HasElbow = false;

 public:
  MotionManager(bool hasElbow);
  ~MotionManager(){};
  void setup();
  void update();
};

}  // namespace motionmanager
