#pragma once
// Simulation of a BNO055 sensor for the Adafruit Sensor Library
// This implementation is very minimalistic.  Very. :-)

#include "Adafruit_Sensor.h"
#include "Arduino.h"

class Adafruit_BNO055 : public Adafruit_Sensor
{
  public:
    // Constructor/Destructor
    Adafruit_BNO055(Arduino* _arduino);
    ~Adafruit_BNO055() {};

    void enableAutoRange(bool enabled) {};
    bool getEvent(sensors_event_t* event);
    void getSensor(sensor_t*) {};

  private:
    Arduino* arduino;
};


