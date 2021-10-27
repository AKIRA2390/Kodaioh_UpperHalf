#pragma once

#include <Arduino.h>

#include "AxialMovement.h"
#include "KodaiohShoulder.h"

namespace motionmanager {

typedef struct MovementsData_t {
  std::vector<axialmovement::Movement_t> Shoulder, UpperArm, Elbow;
} MovementsData_t;

typedef struct AngleDatas_t {
  double ShoulderRotationDeg, UpperArmRotationDeg, ElbowRotationDeg;
} AngleDatas_t;

class MotionManager {
 private:
  axialmovement::AxialMovement Shoulder, UpperArm, Elbow;
  MovementsData_t MovementsData;
  const bool HasElbow = false;
  bool MovementInProgress = false;
  int *ShoulderTD, *UpperArmTD, *ElbowTD;

 public:
  MotionManager(bool hasElbow);
  ~MotionManager(){};
  void setup(int *shoulder_TD, int *upper_arm_TD, int *elbow_TD);
  void setup(int *shoulder_TD, int *upper_arm_TD);
  void update(AngleDatas_t angle_datas);

  void StartMove(MovementsData_t movement_datas);
  bool IsBusy();
};

void addMove(std::vector<axialmovement::Movement_t> &movement_data,
             int movement_start_deg, int movement_target_deg,
             int movement_dulation_time);
}  // namespace motionmanager
