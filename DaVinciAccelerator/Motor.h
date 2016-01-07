#pragma once
// Motor utility functions
//
// You have to define these values (probably with different pins):
// Motor driver interface (PWM on these pins uses timer 0):
// #define motor_In1_PIN 23
// #define motor_In2_PIN 22
// #define motor_PWM_PIN 21
// #define motor_STBY_PIN 20
// #define PWM_FREQUENCY  400 // in Hz

//------------------------------------------------------------------------
// Initialize motor driver
void motor_init();

//------------------------------------------------------------------------
// Motor on full speed:
void motor_on();

//------------------------------------------------------------------------
// Motor on with specified duty cycle:
// Parameters:
//   int dutycycle  -- PWM setting (0..255): 0 = off, 255 = always on
void motor_on_pwm(int dutycycle);

//------------------------------------------------------------------------
// Motor off, coasting:
void motor_off_coast();

//------------------------------------------------------------------------
// Motor off, hard brake:
void motor_off_brake();

