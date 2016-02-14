#include "BnoSim.h"
#include <stdlib.h>  // atof()


Adafruit_BNO055::Adafruit_BNO055(Arduino* _arduino) : arduino(_arduino)
{
 // empty
}

bool Adafruit_BNO055::getEvent(sensors_event_t* event)
{
  event->orientation.x = atof(arduino->matrix.at(arduino->row).at(arduino->dirReading_column).c_str());
  return true;
}

