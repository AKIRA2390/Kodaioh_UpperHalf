#pragma once

#include <Arduino.h>

#include "MotionManager.h"
namespace motion_datas {
extern motionmanager::MovementsData_t OperationCheck_Right;
extern motionmanager::MovementsData_t SwingTheSword;
extern motionmanager::MovementsData_t SwordBatteryDown;
extern motionmanager::MovementsData_t ThrowTheSwordAway;
extern motionmanager::MovementsData_t TakeThePose;
extern motionmanager::MovementsData_t OperationCheck_Left;
extern motionmanager::MovementsData_t RocketPunch;
extern motionmanager::MovementsData_t Empty;
void setup();
}