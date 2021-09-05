#pragma once
#include <Arduino.h>

// written in 2020.11.14 ~ 2020.XX.XX by AKIRA2390

class AMT102V {
 private:
  int A_PIN, B_PIN, resolusion, maxRPM, timeOut = 1000;
  bool isInverted = false;

  volatile int state, old;
  volatile uint64_t timer;
  volatile double rotations = 0, speed[2] = {0}, acc = 0;
  volatile bool CW;

 public:
  /**
   * @brief Construct a new AMT102V object
   *
   * @param a_pin the pin of a layer
   * @param b_pin the pin of b layer
   * @param invert you can inverse the direction of positive
   */
  AMT102V(int a_pin, int b_pin, bool invert = false);
  ~AMT102V();

  /**
   * @brief the function to setup
   *
   * @param DIPS the dip switch's position
   * @param timeout you can calculate the speed, so dt must be small so dt
   * longer than this will be ignored
   */
  void setup(int DIPS, int timeout = 1000);

  /**
   * @brief the function to update
   *
   */
  void update();

  /**
   * @brief the function to reset the rotation
   *
   */
  void resetRotation();

  /**
   * @brief Get the Resolusion of this roricon
   *
   * @return int resolution of this roricon
   */
  int getResolusion();

  /**
   * @brief Get the Max RPM
   *
   * @return int maximum rpm of this roricon
   */
  int getMaxRPM();

  /**
   * @brief Get the Rotations with Int
   *
   * @return int rotations
   */
  int getRotationsInt();

  /**
   * @brief Get the Rotations with Double
   *
   * @return double rotations
   */
  double getRotationsDouble();

  /**
   * @brief Get the Speed(m/s) of roricon
   *
   * @return double speed
   */
  double getSpeed();

  /**
   * @brief Get the Acceleration(m/s^s) of roricon
   *
   * @return double Acceleration
   */
  double getAcc();
};
