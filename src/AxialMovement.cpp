#include "AxialMovement.h"

#include <Arduino.h>

#define DebugAxialMovement

namespace axialmovement {
AxialMovement::AxialMovement() {}
AxialMovement::~AxialMovement() {}

void AxialMovement::update(double rotation_deg, int *target_deg) {
#ifdef DebugAxialMovement
  Serial.println("\nDebug Axial Movement!");
  Serial.println("Update Called!");
#endif
  if (isFreeToMove()) {
    Serial.println("Update Cancelled!");
    return;
  }

  int internalClock = (millis() - BaseTime);

  if (internalClock > DulationTimeAll) {
    Serial.print("Dulation Time All:\t");
    Serial.println(DulationTimeAll);
    setAsNotMoving();
    this->reset();
    Serial.println("Update timeout!");
    return;
  }
  setAsMoving();

  if (internalClock > DulationTimeTemp) {
    Serial.println("\n\n\n///////////////////////////////////////\n\n\n");
    MovementStepNow++;
    Movement_t tempMovement = MovementTasks.at(MovementStepNow);
    DulationTimeTemp += tempMovement.MovementDulationTime;
  }

  Serial.print("movement tasks size: \t");
  Serial.println(MovementTasks.size());
  Serial.print("movement step now: \t");
  Serial.println(MovementStepNow);

  int MSD = MovementTasks.at(MovementStepNow).MovementStartDeg,
      MTD = MovementTasks.at(MovementStepNow).MovementTargetDeg,
      DT = MovementTasks.at(MovementStepNow).MovementDulationTime;

  DeltaTargetDeg =
      (double(MTD) - double(MSD)) / double(DT) * double(internalClock);

  *target_deg = DeltaTargetDeg;
#ifdef DebugAxialMovement
  Serial.println("Debug Axial Movement!");
  Serial.print("movement start deg:\t");
  Serial.println(MSD);
  Serial.print("movement target deg:\t");
  Serial.println(MTD);
  Serial.print("movement duration time:\t");
  Serial.println(DT);
  Serial.print("Delta Target Deg:\t");
  Serial.println(DeltaTargetDeg);
#endif
}

void AxialMovement::loadMove(std::vector<Movement_t> movement_data) {
  Serial.println("\n Loading Move!");
  MovementTasks = movement_data;

  for (int i = 0; i < movement_data.size(); i++) {
    DulationTimeAll += (movement_data.at(i)).MovementDulationTime;
  }
  if (movement_data.size()) {
    DulationTimeTemp += movement_data.at(1).MovementDulationTime;
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

void AxialMovement::setAsMoving() { isMoving = true; }
void AxialMovement::setAsNotMoving() { isMoving = false; }
bool AxialMovement::isFreeToMove() { return !isMoving; }

}  // namespace axialmovement