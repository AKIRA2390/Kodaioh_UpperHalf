#pragma once

#include <Arduino.h>

#include <vector>

namespace axialmovement {

typedef struct Movement_t {
  int MovementStartDeg = 0, MovementTargetDeg = 0, MovementDulationTime = 0;
} Movement_t;

class AxialMovement {
 private:
  int NowTargetDeg = 0, DeltaTargetDeg = 0, MovementStepNow = 0;
  double DulationTimeAll = 0, DulationTimeTemp = 0;
  uint64_t BaseTime = 0;
  bool isMoving = false;

  std::vector<Movement_t> MovementTasks;

  void setAsMoving();
  void setAsNotMoving();

 public:
  AxialMovement();
  ~AxialMovement();

  void update(double rotation_deg, int *shoulder_target_deg);

  void loadMove(std::vector<Movement_t> movement_data);
  void startMovement();
  void reset();

  bool isFreeToMove();
};

}  // namespace axialmovement