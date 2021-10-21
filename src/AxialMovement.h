#pragma once

#include <Arduino.h>

#include <vector>

#include "MotionManager.h"

namespace axialmovement {

class AxialMovement {
 private:
  int NowTargetDeg = 0, DeltaTargetDeg = 0, MovementStepNow = 0;
  double DulationTimeAll = 0, DulationTimeTemp = 0;
  uint64_t BaseTime = 0;
  bool isMoving = false;

  std::vector<motionmanager::Movement_t> MovementTasks;

  void setAsMoving();
  void setAsNotMoving();

 public:
  AxialMovement();
  ~AxialMovement();

  void update(double rotation_deg, int *shoulder_target_deg);

  void loadMove(std::vector<motionmanager::Movement_t> movement_data);
  void startMovement();
  void reset();

  bool isFreeToMove();
};

}  // namespace axialmovement