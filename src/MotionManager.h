#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "KodaiohShoulder.h"
#include "PID4arduino.h"

namespace motionmanager {

typedef struct Movement_t {
  int MovementStartDeg = 0, MovementTargetDeg = 0, MovementDulationTime = 0;
} Movement_t;

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
  
  void addMove(std::vector<Movement_t> &movement_data, Movement_t move);
};

}  // namespace motionmanager
