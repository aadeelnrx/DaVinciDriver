#include "PinsParameters.h"
#include "Motor.h"
#include <Arduino.h>

//------------------------------------------------------------------------
// Initialize motor driver
void motor_init()
{
  pinMode(motor_In1_PIN, OUTPUT);
  pinMode(motor_In2_PIN, OUTPUT);
  pinMode(motor_PWM_PIN, OUTPUT); 
  pinMode(motor_STBY_PIN, OUTPUT); 
  digitalWrite(motor_STBY_PIN, HIGH);
  // Set PWM frequency.  The motor driver can handle 100 kHz max.
  // Setting the frequency on one pin changes it for all other
  // pins on this timer as well (5, 6, 9, 10, 20, 21, 22, 23)
  analogWriteFrequency(motor_In1_PIN, PWM_FREQUENCY);
}

//------------------------------------------------------------------------
// Motor on full speed:
void motor_on()
{
  digitalWrite(motor_In1_PIN, HIGH);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor on with specified duty cycle:
// Parameters:
//   int dutycycle  -- PWM setting (0..255): 0 = off, 255 = always on
void motor_on_pwm(int dutycycle)
{
  analogWrite(motor_In1_PIN, dutycycle);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor off, coasting:
void motor_off_coast()
{
  analogWrite(motor_In1_PIN, 0);
  digitalWrite(motor_In1_PIN, LOW);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor off, hard brake:
void motor_off_brake()
{
  digitalWrite(motor_In1_PIN, HIGH);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, LOW);
}

