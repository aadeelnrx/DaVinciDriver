#include <stdio.h>
#include "Motor.h"

//------------------------------------------------------------------------
// Initialize motor driver
void motor_init()
{
  printf("MOTOR init\n");
}

//------------------------------------------------------------------------
// Motor on full speed:
void motor_on()
{
  printf("MOTOR on\n");
}

//------------------------------------------------------------------------
// Motor on with specified duty cycle:
// Parameters:
//   int dutycycle  -- PWM setting (0..255): 0 = off, 255 = always on
void motor_on_pwm(int dutycycle)
{
  printf("MOTOR on PWM %d\n", dutycycle);
}

//------------------------------------------------------------------------
// Motor off, coasting:
void motor_off_coast()
{
  printf("MOTOR off coast\n");
}

//------------------------------------------------------------------------
// Motor off, hard brake:
void motor_off_brake()
{
  printf("MOTOR off brake\n");
}

