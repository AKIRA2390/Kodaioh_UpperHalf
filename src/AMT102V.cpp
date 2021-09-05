#include "AMT102V.h"

#include <Arduino.h>

// AMT102V* pointer;
// void interrupt();

AMT102V::AMT102V(int a_pin, int b_pin, bool invert) {
  A_PIN = a_pin;
  B_PIN = b_pin;
  isInverted = invert;
  // pointer = this;
}

AMT102V::~AMT102V() {}

// DIPS:スイッチをオン1,オフ0で左から二進数に並べる
void AMT102V::setup(int DIPS, int timeout) {
  switch (DIPS) {
    case 0b0000:
      resolusion = 2048;
      maxRPM = 7500;
      break;
    case 0b0010:
      resolusion = 1024;
      maxRPM = 7500;
      break;
    case 0b1000:
      resolusion = 1000;
      maxRPM = 7500;
      break;
    case 0b0100:
      resolusion = 800;
      maxRPM = 7500;
      break;
    case 0b0001:
      resolusion = 512;
      maxRPM = 15000;
      break;
    case 0b1010:
      resolusion = 500;
      maxRPM = 7500;
      break;
    case 0b0110:
      resolusion = 400;
      maxRPM = 7500;
      break;
    case 0b1100:
      resolusion = 384;
      maxRPM = 7500;
      break;
    case 0b0011:
      resolusion = 256;
      maxRPM = 15000;
      break;
    case 0b1001:
      resolusion = 250;
      maxRPM = 15000;
      break;
    case 0b0101:
      resolusion = 200;
      maxRPM = 15000;
      break;
    case 0b1110:
      resolusion = 192;
      maxRPM = 7500;
      break;
    case 0b1011:
      resolusion = 125;
      maxRPM = 15000;
      break;
    case 0b0111:
      resolusion = 100;
      maxRPM = 15000;
      break;
    case 0b1101:
      resolusion = 96;
      maxRPM = 15000;
      break;
    case 0b1111:
      resolusion = 48;
      maxRPM = 15000;
      break;
  }
  this->timeOut = timeout;
  // attachInterrupt(A_PIN, interrupt, RISING);
  // attachInterrupt(B_PIN, interrupt, RISING);
}

void AMT102V::update() {
  state = digitalRead(A_PIN) << 1 | digitalRead(B_PIN);
  speed[1] = speed[0];
  if (state == old && state == 3) {
    return;
  }
  if (state == 1) {
    rotations -= double(1) / double(resolusion);
    CW = true;
  }
  if (state == 2) {
    rotations += double(1) / double(resolusion);
    CW = false;
  }
  if ((millis() - timer) < timeOut) {
    speed[0] =
        (double(1) / (double(millis() - timer) * double(resolusion))) * 60000;
    acc = double(speed[1] - speed[0]) / (double(millis() - timer));
  }

  old = state;
}
// interrupt relay
// void interrupt() { pointer->update(); }
void AMT102V::resetRotation() { rotations = 0; }

int AMT102V::getResolusion() { return resolusion; }
int AMT102V::getMaxRPM() { return maxRPM; }

double AMT102V::getRotationsDouble() { return rotations*(isInverted?-1:1); }
int AMT102V::getRotationsInt() { return int(rotations*(isInverted?-1:1)); }

double AMT102V::getSpeed() { return speed[0]; };
double AMT102V::getAcc() { return acc; };