#include "MotionManager.h"

#include <Arduino.h>

#include "AxialMovement.h"
#include "KodaiohShoulder.h"

namespace motionmanager {
MotionManager::MotionManager(bool hasElbow) : HasElbow(hasElbow){};

void MotionManager::setup(double *shouder_MV, double *upper_arm_MV,
                          double *elbow_MV) {}
void MotionManager::update(AngleDatas_t angle_datas) {
  if (MovementInProgress) {
    Shoulder.update(angle_datas.ShouderRotationDeg, ShoulderMV);
    UpperArm.update(angle_datas.UpperArmRotationDeg, UpperArmMV);
    if (HasElbow) {
      Elbow.update(angle_datas.ElbowRotationDeg, ElbowMV);
    }

    if (Shoulder.isFreeToMove() && UpperArm.isFreeToMove()) {
      Shoulder.reset();
      UpperArm.reset();
      if (HasElbow && Elbow.isFreeToMove()) {
        Elbow.reset();
      }
    }

  } else if (MovementInProgress) {
    Shoulder.reset();
    Elbow.reset();
    if (HasElbow) {
      Shoulder.reset();
    }
  }
}

void MotionManager::StartMove(MovementsData_t movement_datas) {
  MovementsData = movement_datas;
  MovementInProgress = true;
  Shoulder.loadMove(MovementsData.Shoulder);
  Shoulder.startMovement();

  UpperArm.loadMove(MovementsData.UpperArm);
  UpperArm.startMovement();

  if (HasElbow) {
    Elbow.loadMove(MovementsData.Elbow);
    Elbow.startMovement();
  }
}

void MotionManager::addMove(std::vector<Movement_t> &movement_data,
                            Movement_t move) {
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