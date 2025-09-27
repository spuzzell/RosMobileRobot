#pragma once
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/*
 * motors.h
 *
 * Small helper wrapper to control a single motor using an H-bridge
 * or motor driver. Provides speed/direction abstraction and a simple
 * braking helper. Intended for use with PWM channels and digital
 * direction/enable pins on Arduino-compatible boards.
 */

/// Logical direction for motor rotation.
enum class Direction {
  FORWARD, ///< Motor driving forward (positive PWM)
  REVERSE  ///< Motor driving in reverse (negative PWM)
};

class Motor {
public:
  /**
   * Constructor.
   * @param pwmChannel  PWM channel number used to output speed (analogWrite/etc).
   * @param enablePin   Digital pin used to enable/disable the motor driver.
   * @param forwardPin  Digital pin used to set forward direction.
   * @param reversePin  Digital pin used to set reverse direction.
   */
  Motor(int pwmChannel, int enablePin, int forwardPin, int reversePin);

  /**
   * Set motor speed.
   * @param pwm  Signed PWM value (0..max). Positive values drive FORWARD,
   *             negative values drive REVERSE. Value interpretation depends
   *             on the platform's PWM range.
   */
  void setSpeed(int pwm);

  /**
   * Activate motor brake (short the motor or disable outputs depending on driver).
   * Implementation details are in the .cpp file; this is a convenience call.
   */
  void brake();


private:
  /**
   * Commit pending motor commands to the hardware (set pins / write PWM).
   * Kept private because callers should use setSpeed() / brake().
   */
  void sendMotorCommand();

  int pwmChannel_;   ///< PWM channel used for speed control
  int enablePin_;    ///< Pin used to enable/disable the motor driver
  int forwardPin_;   ///< Pin used to select forward output
  int reversePin_;   ///< Pin used to select reverse output

  int pendingPwm_ = 0;                      ///< Last requested PWM value (signed)
  Direction currentDirection_ = Direction::FORWARD; ///< Current logical direction
};

#endif
