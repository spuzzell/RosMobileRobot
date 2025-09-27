//##################################################################################################
//                     ESP32 GPIO ASSIGNMENTS — Bridge project
//##################################################################################################
//
// Pin mappings for motor control and UART between ESP32 and Teensy.
//
// Conventions:
//  - PWM_*      : PWM output used for motor speed control (connect to driver EN/PWM input)
//  - IN*_       : Direction control pins for each motor (drive H-bridge IN1 / IN2)
//  - RX2 / TX2  : Serial2 pins (ESP32 <-> Teensy communication)
//
// Note: Verify motor driver input wiring and that chosen GPIOs are suitable for PWM / interrupts
//       on your ESP32 variant before final deployment.
//
//##################################################################################################
//                                             MOTOR CONTROL
//##################################################################################################

// Left front motor
#define PWM_LEFT_FRONT  33   // PWM output for left front motor speed (EN/PWM on motor driver)
#define IN1_LEFT_FRONT  26   // Direction input 1 for left front motor (H-bridge IN1)
#define IN2_LEFT_FRONT  14   // Direction input 2 for left front motor (H-bridge IN2)

// Right front motor
#define PWM_RIGHT_FRONT 22   // PWM output for right front motor speed
#define IN1_RIGHT_FRONT 19   // Direction input 1 for right front motor
#define IN2_RIGHT_FRONT 4    // Direction input 2 for right front motor

// Left rear motor
#define PWM_LEFT_REAR   32   // PWM output for left rear motor speed
#define IN1_LEFT_REAR   25   // Direction input 1 for left rear motor
#define IN2_LEFT_REAR   27   // Direction input 2 for left rear motor

// Right rear motor
#define PWM_RIGHT_REAR  23   // PWM output for right rear motor speed
#define IN1_RIGHT_REAR  21   // Direction input 1 for right rear motor
#define IN2_RIGHT_REAR  18   // Direction input 2 for right rear motor

//##################################################################################################
//                                             UART PINS
//##################################################################################################

// Serial2 between ESP32 and Teensy:
//  - RX2_PIN_ESP_TO_TEENSY: ESP32 RX (receive) pin — wired to Teensy TX
//  - TX2_PIN_ESP_TO_TEENSY: ESP32 TX (transmit) pin — wired to Teensy RX
#define RX2_PIN_ESP_TO_TEENSY       16   // Serial2 RX (ESP receives from Teensy)
#define TX2_PIN_ESP_TO_TEENSY       17   // Serial2 TX (ESP transmits to Teensy)
