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

  Serial.print("clock \t");
  Serial.println(millis());
  Serial.print("base time \t");
  Serial.println(int(BaseTime));
  int internalClock = (millis() - BaseTime);
  Serial.print("internal clock \t");
  Serial.println(internalClock);

  if (internalClock > DulationTimeAll) {
    Serial.print("Dulation Time All:\t");
    Serial.println(DulationTimeAll);
    setAsNotMoving();
    this->reset();
    Serial.println("Update timeout!");
    return;
  }
  setAsMoving();

  Serial.println("hi!");
  while (internalClock >= DulationTimeTemp) {
    Serial.print("duration time temp");
    Serial.println(DulationTimeTemp);
    Movement_t tempMovement = MovementTasks.at(MovementStepNow);
    DulationTimeTemp += tempMovement.MovementDulationTime;
    MovementStepNow++;
  }

  Serial.print("movement tasks size: \t");
  Serial.println(MovementTasks.size());
  Serial.print("movement step now: \t");
  Serial.println(MovementStepNow);
    Serial.println("hello!");
  int MSD = MovementTasks.at(MovementStepNow).MovementStartDeg,
      MTD = MovementTasks.at(MovementStepNow).MovementTargetDeg,
      DT = MovementTasks.at(MovementStepNow).MovementDulationTime;
    Serial.println("world!");

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
  Serial.print("hoge");
  for (int i = 0; i < movement_data.size(); i++) {
    Serial.println((movement_data.at(i)).MovementDulationTime);
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

void AxialMovement::setAsMoving() { isMoving = true; }
void AxialMovement::setAsNotMoving() { isMoving = false; }
bool AxialMovement::isFreeToMove() { return !isMoving; }

}  // namespace axialmovement