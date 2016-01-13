#include "Logging.h"
#include "Motor.h"
#include <Arduino.h>


//----------------------------------------------------------------
// Fatal error: print message to serial, red LED on, stop program.
// Stop motor.
void error(const char *str)
{
  // Better switch off motor:
  motor_off_brake();

  LOG_SERIAL_LN("error: ");
  LOG_SERIAL_LN(str);
  
  // red LED indicates error
  digitalWrite(red_LED_PIN, HIGH);
  
  while(1);
}

