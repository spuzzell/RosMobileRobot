#include "motors.h"

// Motor constructor
// pwmChannel  - LEDC (PWM) channel used for speed control
// enablePin   - pin connected to the motor driver's PWM/enable input
// forwardPin  - digital pin that enables forward direction (IN1)
// reversePin  - digital pin that enables reverse direction (IN2)
//
// Initializes pin directions, configures the LEDC PWM channel, and
// leaves the motor stopped (PWM = 0, direction pins low).
Motor::Motor(int pwmChannel, int enablePin, int forwardPin, int reversePin)
  : pwmChannel_(pwmChannel), enablePin_(enablePin), forwardPin_(forwardPin), reversePin_(reversePin), currentDirection_(Direction::FORWARD) {
  // Configure GPIOs
  pinMode(enablePin_, OUTPUT);
  pinMode(forwardPin_, OUTPUT);
  pinMode(reversePin_, OUTPUT);

  // Setup LEDC PWM: 20 kHz frequency, 8-bit resolution
  ledcSetup(pwmChannel_, 20000, 8);
  // Attach the enable pin to the configured LEDC channel
  ledcAttachPin(enablePin_, pwmChannel_);
  
  // Ensure both direction pins are low and PWM is 0 on startup
  digitalWrite(forwardPin_, LOW);
  digitalWrite(reversePin_, LOW);
  ledcWrite(pwmChannel_, 0);
}

// Set motor speed and direction.
// pwm is in the range -255..255. Sign indicates direction:
//   pwm > 0 -> forward
//   pwm < 0 -> reverse
//   pwm == 0 -> brake (both direction pins low, PWM 0)
//
// The value is constrained, absolute value used for PWM duty,
// and direction pins are set accordingly.
void Motor::setSpeed(int pwm) {
    // Clamp input to valid range
    pwm = constrain(pwm, -255, 255);

    // Determine requested direction based on sign
    Direction newDirection = (pwm >= 0) ? Direction::FORWARD : Direction::REVERSE;

    // Use absolute magnitude for PWM duty
    pwm = abs(pwm);

    // If speed is zero, apply brake (coast/stop behavior)
    if (pwm == 0) {
        brake();
        return;
    }

    // Update stored current direction to the requested direction.
    // This ensures future logic or queries reflect actual motor direction.
    currentDirection_ = newDirection;

    // Drive the motor direction pins according to currentDirection_
    // IN1 = HIGH, IN2 = LOW for forward
    // IN1 = LOW,  IN2 = HIGH for reverse
    if (currentDirection_ == Direction::FORWARD) {
        digitalWrite(forwardPin_, HIGH);
        digitalWrite(reversePin_, LOW);
    } else {
        digitalWrite(forwardPin_, LOW);
        digitalWrite(reversePin_, HIGH);
    }

    // Apply PWM duty (0-255 for 8-bit resolution)
    ledcWrite(pwmChannel_, pwm);
}

// Brake the motor: set PWM to 0 and force both direction pins low.
// This corresponds to IN1 = 0, IN2 = 0 on many H-bridge drivers.
void Motor::brake() {
  ledcWrite(pwmChannel_, 0);
  digitalWrite(forwardPin_, LOW);
  digitalWrite(reversePin_, LOW);
}