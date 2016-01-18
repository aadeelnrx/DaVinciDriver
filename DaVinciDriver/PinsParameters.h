#pragma once
//----------------------( Enable Features )------------------------

#define ECHO_TO_SERIAL   1  // echo data to serial port
#define WAIT_TO_START    0  // Wait for serial input in setup()
#define RADIO_ON         0  // Radio transmission
#define IR_ON            0  // IR transmission
#define SD_CARD_ON       1  // Logging to SD card
#define TEST_LOGGING_ON  0  //
#define BNO055_ON        1  // Sensor
#define BNO055_TEST_ON   0  // Extended Sensor reading only valid with BNO055_ON

//----------------------( Declare Constants )------------------------

#define LOG_INTERVAL     10 // mills between logging
#define PID_INTERVAL     10 // mills between PID updates
#define RACING_INTERVAL  10 // mills between PID updates
#define STOP_AFTER_SECONDS 130

//--------------------( Declare Pin Numbers )-------------------------

// LEDs
#define red_LED_PIN 13
#define green_LED_PIN 13

// Sensors
#define speed_PIN 3              // digital 3, interrupt 0
#define track_volt_PIN A1        // analog 1
#define engine_volt_PIN A2       // analog 2

// The SPI pins for CF and Radio
#define CE_Radio_PIN   8
#define CSN_Radio_PIN 10
#define SCK_PIN 14   // SPI bus uses pin 14 instead of pin 13 (used for LED)
#define CSN_CF_PIN 9

// The IR receiver digital pin
#define IR_receiver 4

// Motor driver interface (PWM on these pins uses timer 0):
#define motor_In1_PIN 23
#define motor_In2_PIN 22
#define motor_PWM_PIN 21
#define motor_STBY_PIN 20

// Motor PWM Frequency
// To calculate you need R and L of the motor.
// First the time constant (Tc) of the motor:
// Tc = L/R
// In 5*Tc the motor current has risen to 98.2%.
// Choose a minimum duty cycle (dc = 1..100).
// Tmin = 100/dc * 5 * Tc
// PWM_FREQUENCY = 1/Tmin
#define PWM_FREQUENCY  100 // in Hz (guess, I haven't measured R and L)

// Learning Lap(s) PID and speed definition  
#define MOTOR_TARGET_SPEED 25
#define MOTOR_M_P 2.5
#define MOTOR_M_I 5
#define MOTOR_M_D 0.00

// Racing Laps PID definition
#define MOTOR_R_P 10 //30 //20
#define MOTOR_R_I 1
#define MOTOR_R_D 0.00

