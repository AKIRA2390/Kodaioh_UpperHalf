#include "MotionManager.h"

#include <Arduino.h>

#include "AxialMovement.h"
#include "KodaiohShoulder.h"

namespace motionmanager {
MotionManager::MotionManager(bool hasElbow) : HasElbow(hasElbow){};

void MotionManager::setup(int *shoulder_TD, int *upper_arm_TD, int *elbow_TD) {
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

    if (Shoulder.isFreeToMove() && UpperArm.isFreeToMove()) {
      if (HasElbow && Elbow.isFreeToMove()) {
        Shoulder.reset();
        UpperArm.reset();
        Elbow.reset();
        MovementInProgress = false;
      } else {
        Shoulder.reset();
        UpperArm.reset();
        MovementInProgress = false;
      }
    }

  } else if (!MovementInProgress) {
    Shoulder.reset();
    Elbow.reset();
    if (HasElbow) {
      Shoulder.reset();
    }
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
             axialmovement::Movement_t move) {
  movement_data.push_back(move);  //アヤシイ...動かなさそうな気がする
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