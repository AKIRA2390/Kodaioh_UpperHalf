#include "MotionDatas.h"

#include <Arduino.h>

#include "MotionManager.h"
namespace motion_datas {
void MakeMove_OperationCheck_Right();
void MakeMove_SwingTheSword();
void MakeMove_SwordBatteryDown();
void MakeMove_ThrowTheSwordAway();
void MakeMove_TakeThePose();
void MakeMove_OperationCheck_Left();
void MakeMove_RocketPunch();
void MakeMove_Empty();

motionmanager::MovementsData_t OperationCheck_Right;
motionmanager::MovementsData_t SwingTheSword;
motionmanager::MovementsData_t SwordBatteryDown;
motionmanager::MovementsData_t ThrowTheSwordAway;
motionmanager::MovementsData_t TakeThePose;
motionmanager::MovementsData_t OperationCheck_Left;
motionmanager::MovementsData_t RocketPunch;
motionmanager::MovementsData_t Empty;

void setup() {
  // Serial.println("Motion Data Setup");

  MakeMove_OperationCheck_Right();
  MakeMove_SwingTheSword();
  MakeMove_SwordBatteryDown();
  MakeMove_ThrowTheSwordAway();
  MakeMove_TakeThePose();
  MakeMove_OperationCheck_Left();
  MakeMove_RocketPunch();
  MakeMove_Empty();
}

void MakeMove_OperationCheck_Right() {
}
void MakeMove_SwingTheSword() {}
void MakeMove_SwordBatteryDown() {}
void MakeMove_ThrowTheSwordAway() {}
void MakeMove_TakeThePose() {}
void MakeMove_OperationCheck_Left() {}
void MakeMove_RocketPunch() {}
void MakeMove_Empty() {
  motionmanager::addMove(Empty.Elbow, 0, 90, 5000);
  motionmanager::addMove(Empty.Elbow, 90, 90, 5000);
  motionmanager::addMove(Empty.Elbow, 90, 0, 5000);
  motionmanager::addMove(Empty.Elbow, 0, 0, 5000);
}
}  // namespace motion_datas