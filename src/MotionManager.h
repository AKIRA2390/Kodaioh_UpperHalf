#pragma once

#include <Arduino.h>

#include "AxialMovement.h"
#include "KodaiohShoulder.h"

namespace motionmanager {

typedef struct Movement_t {
  int MovementStartDeg = 0, MovementTargetDeg = 0, MovementDulationTime = 0;
} Movement_t;

typedef struct MovementsData_t {
  std::vector<Movement_t> Shoulder, UpperArm, Elbow;
} MovementsData_t;

typedef struct AngleDatas_t {
  double ShouderRotationDeg, UpperArmRotationDeg, ElbowRotationDeg;
} AngleDatas_t;

class MotionManager {
 private:
  axialmovement::AxialMovement Shoulder, UpperArm, Elbow;
  MovementsData_t MovementsData;
  const bool HasElbow = false;
  bool MovementInProgress = false;
  double *ShoulderMV, *UpperArmMV, *ElbowMV;

 public:
  MotionManager(bool hasElbow);
  ~MotionManager(){};
  void setup(double *shouder_MV, double *upper_arm_MV, double *elbow_MV);
  void update(AngleDatas_t angle_datas);

  void StartMove(MovementsData_t movement_datas);
  void addMove(std::vector<Movement_t> &movement_data, Movement_t move);
};

}  // namespace motionmanager
