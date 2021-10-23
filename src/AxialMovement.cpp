#include "AxialMovement.h"

#include <Arduino.h>

#define DebugAxialMovement

namespace axialmovement {
AxialMovement::AxialMovement() {}
AxialMovement::~AxialMovement() {}

void AxialMovement::update(double rotation_deg, int *target_deg) {
#ifdef DebugAxialMovement
  // Serial.println("\nDebug Axial Movement!");
  // Serial.println("Update Called!");
#endif
  if (isFreeToMove()) {
    // Serial.println("Update Cancelled!");
    return;
  }

  int internalClock = (millis() - BaseTime);

  if (internalClock > DulationTimeAll) {
    setAsNotMoving();
    this->reset();
    // Serial.println("Update timeout!");
    return;
  }
  setAsMoving();

  if (internalClock > DulationTimeTemp) {
    MovementStepNow++;
    Movement_t tempMovement = MovementTasks.at(MovementStepNow);
    DulationTimeTemp += tempMovement.MovementDulationTime;
  }

  double MSD = MovementTasks.at(MovementStepNow).MovementStartDeg,
         MTD = MovementTasks.at(MovementStepNow).MovementTargetDeg,
         DT = MovementTasks.at(MovementStepNow).MovementDulationTime,
         DeltaTime = DT - (DulationTimeTemp - internalClock);

  DeltaTargetDeg = (MTD - MSD) / DT * DeltaTime + MSD;

  *target_deg = DeltaTargetDeg;
#ifdef DebugAxialMovement
  // Serial.println("Debug Axial Movement!");
  // Serial.print("movement_tasks_size:");
  // Serial.print(MovementTasks.size());
  // Serial.print(", ");
  // Serial.print("movement_step_now:");
  // Serial.print(MovementStepNow);
  // Serial.print(", ");
  Serial.print("movement_start_deg:");
  Serial.print(MSD);
  Serial.print(", ");
  Serial.print("movement_target_deg:");
  Serial.print(MTD);
  Serial.print(", ");
  // Serial.print("delta_time:");
  // Serial.print(DeltaTime);
  // Serial.print(", ");
  // Serial.print("movement_duration_time_temp:");
  // Serial.print(DulationTimeTemp);
  // Serial.print(", ");
  // Serial.print("internal_clock:");
  // Serial.print(internalClock);
  // Serial.print(", ");
  Serial.print("Delta_Target_Deg:");
  Serial.println(DeltaTargetDeg);
  Serial.println("\n");
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