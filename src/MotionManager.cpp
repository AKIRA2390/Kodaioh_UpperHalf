#include "MotionManager.h"

#include <Arduino.h>
#include <ESP32Servo.h>

#include "KodaiohShoulder.h"
#include "PID4arduino.h"

namespace motionmanager {
MotionManager::MotionManager(bool hasElbow) : HasElbow(hasElbow){};

void MotionManager::setup() {}
void MotionManager::update() {}

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
腕のPIDゲインの隔離(変更容易のため)

main kodaiohShoulder MotionManagerで三角形を作ってほしい
AxialMovementはこの下にのみ存在するべきである


*/