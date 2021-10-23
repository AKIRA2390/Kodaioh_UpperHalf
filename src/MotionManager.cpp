#include "MotionManager.h"

#include <Arduino.h>

#include "AxialMovement.h"
#include "KodaiohShoulder.h"

#define DebugMotionManager

namespace motionmanager {
MotionManager::MotionManager(bool hasElbow) : HasElbow(hasElbow){};

void MotionManager::setup(int *shoulder_TD, int *upper_arm_TD, int *elbow_TD) {
  // Serial.println("\nMotion Setup");

  ShoulderTD = shoulder_TD;
  UpperArmTD = upper_arm_TD;
  if (HasElbow) {
    ElbowTD = elbow_TD;
  }
}

void MotionManager::update(AngleDatas_t angle_datas) {
  if (MovementInProgress) {
    Shoulder.update(angle_datas.ShoulderRotationDeg, ShoulderTD);
    UpperArm.update(angle_datas.UpperArmRotationDeg, UpperArmTD);
    if (HasElbow) {
      Elbow.update(angle_datas.ElbowRotationDeg, ElbowTD);
    }

#ifdef DebugMotionManager
    // Serial.println("\nDebug Motion Manager!");
    // Serial.println("Movement In Progress!");
#endif

    if (Shoulder.isFreeToMove() && UpperArm.isFreeToMove()) {
      if (HasElbow && Elbow.isFreeToMove()) {
        MovementInProgress = false;
      } else if (!HasElbow) {
        MovementInProgress = false;
      }
    }

  } else if (!MovementInProgress) {
    Shoulder.reset();
    Elbow.reset();
    if (HasElbow) {
      Shoulder.reset();
    }
#ifdef DebugMotionManager
    // Serial.println("\nDebug Motion Manager!");
    // Serial.println("Movement Not In Progress!");
#endif
  } else {
    // Serial.println("Yo WTF");
  }
}

void MotionManager::StartMove(MovementsData_t movement_datas) {
  if (MovementInProgress) return;
  MovementInProgress = true;
  MovementsData = movement_datas;
  Shoulder.loadMove(MovementsData.Shoulder);
  Shoulder.startMovement();

  UpperArm.loadMove(MovementsData.UpperArm);
  UpperArm.startMovement();

  if (HasElbow) {
    Elbow.loadMove(MovementsData.Elbow);
    Elbow.startMovement();
  }
}
bool MotionManager::IsBusy() { return MovementInProgress; }

void addMove(std::vector<axialmovement::Movement_t> &movement_data,
             int movement_start_deg, int movement_target_deg,
             int movement_dulation_time) {
  axialmovement::Movement_t Move;

  Move.MovementStartDeg = movement_start_deg;
  Move.MovementTargetDeg = movement_target_deg;
  Move.MovementDulationTime = movement_dulation_time;

  movement_data.push_back(Move);  //アヤシイ...動かなさそうな気がする
}
}  // namespace motionmanager

/*
AxialMovementの使い方

Movement_tのvectorを作る
addMoveをつかってぽいぽい追加していく(push_backのがよさそう)
追加し終わったらもっとく

loadMoveでMotionデータを読み込む
startMovementで動かし始める
updateを呼び続ける

動作が終わるとisFreeToMoveがtrueになる
resetでリセットする

次の動作をloadする
*/

/*
MotionManagerでやってほしいこと

別で作ったMotionファイルの読み込み
Shoulder/UpperArm/ElbowのAxialMovementクラスの運用

main kodaiohShoulder MotionManagerで三角形を作ってほしい
AxialMovementはこの下にのみ存在するべきである


*/