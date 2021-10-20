#include "AxialMovement.h"

#include <Arduino.h>
#include <ESP32Servo.h>

#include "KodaiohShoulder.h"
#include "PID4arduino.h"

namespace axialmovement {
AxialMovement::AxialMovement() {}
AxialMovement::~AxialMovement() {}

void AxialMovement::update(double rotation_deg, int *shoulder_target_deg) {
  if (isFreeToMove()) return;

  uint64_t internalClock = millis() - BaseTime;

  if (internalClock > DulationTimeAll) {
    setAsNotMoving();
    return;
  }
  setAsMoving();

  while (internalClock >= DulationTimeTemp) {
    DulationTimeTemp += MovementTasks.at(MovementStepNow).MovementDulationTime;
    MovementStepNow++;
  }

  int MSD = MovementTasks.at(MovementStepNow).MovementStartDeg,
      MTD = MovementTasks.at(MovementStepNow).MovementTargetDeg,
      DT = MovementTasks.at(MovementStepNow).MovementDulationTime;

  DeltaTargetDeg = (MTD - MSD) / DT * internalClock;

  *shoulder_target_deg = DeltaTargetDeg;
}

void AxialMovement::loadMove(std::vector<Movement_t> movement_data) {
  for (int i = 0; i < movement_data.size(); i++) {
    DulationTimeAll += (movement_data.at(i)).MovementDulationTime;
  }
}

void AxialMovement::startMovement() {
  BaseTime = millis();
  setAsMoving();
}
void AxialMovement::reset() {
  NowTargetDeg = 0, DeltaTargetDeg = 0, MovementStepNow = 0,
  DulationTimeAll = 0, DulationTimeTemp = 0, BaseTime = 0, isMoving = false;
}

void AxialMovement::addMove(std::vector<Movement_t> &movement_data,
                            Movement_t move) {
  movement_data.push_back(move);  //アヤシイ...動かなさそうな気がする
}

void AxialMovement::setAsMoving() { isMoving = true; }
void AxialMovement::setAsNotMoving() { isMoving = false; }
bool AxialMovement::isFreeToMove() { return !isMoving; }

}  // namespace axialmovement