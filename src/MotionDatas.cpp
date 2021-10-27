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
void MakeMove_RocketPunch_Reset();
void MakeMove_Empty();

motionmanager::MovementsData_t OperationCheck_Right;
motionmanager::MovementsData_t SwingTheSword;
motionmanager::MovementsData_t SwordBatteryDown;
motionmanager::MovementsData_t ThrowTheSwordAway;
motionmanager::MovementsData_t TakeThePose;
motionmanager::MovementsData_t OperationCheck_Left;
motionmanager::MovementsData_t RocketPunch;
motionmanager::MovementsData_t RocketPunch_Reset;
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
  MakeMove_RocketPunch_Reset();
  MakeMove_Empty();
}

void MakeMove_OperationCheck_Right() {
  motionmanager::addMove(OperationCheck_Right.Shoulder, 0, 0, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 0, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 0, 0, 1000);

  motionmanager::addMove(OperationCheck_Right.Shoulder, 0, 60, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 30, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 0, 60, 1000);

  motionmanager::addMove(OperationCheck_Right.Shoulder, 60, 60, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 30, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 60, 0, 1000);

  motionmanager::addMove(OperationCheck_Right.Shoulder, 60, 60, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 30, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 0, 60, 1000);

  motionmanager::addMove(OperationCheck_Right.Shoulder, 60, 60, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 30, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 60, 0, 1000);

  motionmanager::addMove(OperationCheck_Right.Shoulder, 60, 60, 1000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 30, 30, 1000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 0, 0, 1000);
}

void MakeMove_SwingTheSword() {
  motionmanager::addMove(SwingTheSword.Shoulder, 0, 0, 500);
  motionmanager::addMove(SwingTheSword.UpperArm, 30, 45, 500);
  motionmanager::addMove(SwingTheSword.Elbow, 0, 0, 500);

  motionmanager::addMove(SwingTheSword.Shoulder, 0, 45, 2000);
  motionmanager::addMove(SwingTheSword.UpperArm, 45, 45, 2000);
  motionmanager::addMove(SwingTheSword.Elbow, 0, 90, 2000);

  motionmanager::addMove(SwingTheSword.Shoulder, 45, 45, 1000);
  motionmanager::addMove(SwingTheSword.UpperArm, 45, 45, 1000);
  motionmanager::addMove(SwingTheSword.Elbow, 90, 90, 1000);

  motionmanager::addMove(SwingTheSword.Shoulder, 45, 0, 1500);
  motionmanager::addMove(SwingTheSword.UpperArm, 45, 45, 1500);
  motionmanager::addMove(SwingTheSword.Elbow, 90, 0, 1500);

  motionmanager::addMove(SwingTheSword.Shoulder, 0, 0, 500);
  motionmanager::addMove(SwingTheSword.UpperArm, 45, 30, 500);
  motionmanager::addMove(SwingTheSword.Elbow, 0, 0, 500);
}

void MakeMove_SwordBatteryDown() {
  motionmanager::addMove(SwordBatteryDown.Shoulder, 0, 0, 1000);
  motionmanager::addMove(SwordBatteryDown.UpperArm, 30, 45, 1000);
  motionmanager::addMove(SwordBatteryDown.Elbow, 0, 90, 1000);

  motionmanager::addMove(SwordBatteryDown.Shoulder, 0, 30, 2000);
  motionmanager::addMove(SwordBatteryDown.UpperArm, 45, 60, 2000);
  motionmanager::addMove(SwordBatteryDown.Elbow, 90, 90, 2000);
}

void MakeMove_ThrowTheSwordAway() {
  motionmanager::addMove(ThrowTheSwordAway.Shoulder, 30, -45, 2000);
  motionmanager::addMove(ThrowTheSwordAway.UpperArm, 60, 30, 2000);
  motionmanager::addMove(ThrowTheSwordAway.Elbow, 90, 0, 2000);

  motionmanager::addMove(ThrowTheSwordAway.Shoulder, -45, -45, 1000);
  motionmanager::addMove(ThrowTheSwordAway.UpperArm, 30, 30, 2000);
  motionmanager::addMove(ThrowTheSwordAway.Elbow, 0, 00, 2000);

  motionmanager::addMove(ThrowTheSwordAway.Shoulder, -45, 0, 1000);
}

void MakeMove_TakeThePose() {
  motionmanager::addMove(TakeThePose.Shoulder, 0, 120, 1000);
  motionmanager::addMove(TakeThePose.UpperArm, 30, 60, 1000);
  motionmanager::addMove(TakeThePose.Elbow, 0, 60, 1000);
}

void MakeMove_OperationCheck_Left() {
  motionmanager::addMove(OperationCheck_Right.Shoulder, 0, 0, 7000);
  motionmanager::addMove(OperationCheck_Right.UpperArm, 0, 0, 7000);
  motionmanager::addMove(OperationCheck_Right.Elbow, 0, 0, 7000);

  motionmanager::addMove(OperationCheck_Left.Shoulder, 0, 0, 500);
  motionmanager::addMove(OperationCheck_Left.UpperArm, 0, 60, 1500);

  motionmanager::addMove(OperationCheck_Left.Shoulder, 0, 120, 1500);
  motionmanager::addMove(OperationCheck_Left.UpperArm, 60, 60, 500);

  motionmanager::addMove(OperationCheck_Left.Shoulder, 120, 120, 3500);
  motionmanager::addMove(OperationCheck_Left.UpperArm, 60, 0, 1500);

  motionmanager::addMove(OperationCheck_Left.UpperArm, 0, 60, 1500);

  motionmanager::addMove(OperationCheck_Left.Shoulder, 120, 0, 1500);
  motionmanager::addMove(OperationCheck_Left.UpperArm, 60, 60, 500);

  motionmanager::addMove(OperationCheck_Left.UpperArm, 60, 0, 1500);
}

void MakeMove_RocketPunch() {
  motionmanager::addMove(RocketPunch.Shoulder, 0, 0, 500);
  motionmanager::addMove(RocketPunch.UpperArm, 0, 60, 1500);

  motionmanager::addMove(RocketPunch.Shoulder, 0, 110, 1500);

  motionmanager::addMove(RocketPunch.UpperArm, 60, 60, 500);

  motionmanager::addMove(RocketPunch.Shoulder, 110, 110, 2000);

  motionmanager::addMove(RocketPunch.UpperArm, 60, 0, 1500);
}

void MakeMove_RocketPunch_Reset() {
  motionmanager::addMove(RocketPunch.Shoulder, 110, 110, 3000);
  motionmanager::addMove(RocketPunch.UpperArm, 0, 0, 2000);

  motionmanager::addMove(RocketPunch.UpperArm, 0, 60, 1000);

  motionmanager::addMove(RocketPunch.Shoulder, 110, 0, 2000);
  motionmanager::addMove(RocketPunch.UpperArm, 60, 60, 1000);

  motionmanager::addMove(RocketPunch.UpperArm, 60, 0, 1500);
}
void MakeMove_Empty() {
  motionmanager::addMove(Empty.Elbow, 0, 90, 5000);
  motionmanager::addMove(Empty.Elbow, 90, 90, 5000);
  motionmanager::addMove(Empty.Elbow, 90, 0, 5000);
  motionmanager::addMove(Empty.Elbow, 0, 0, 5000);
}
}  // namespace motion_datas