#include "PinsParameters.h"
#include "datatypes.h"
#include "Arduino.h"
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <PID_v1.h>

// Initialize the arduino simulation:
Arduino arduino("../doc/Misc/LOGGER16.CSV");
volatile uint32_t speedCounter = 0;
uint32_t dirReading = 0;
double PID_input, PID_setpoint, PID_output;;
PID PIDcontroller(&PID_input, &PID_output, &PID_setpoint, MOTOR_M_P, MOTOR_M_I, MOTOR_M_D, DIRECT);
lap_segment lap[1000];
lap_segment_hist lap_hist[10];
uint32_t seq_cnt;        // Segment sequence number
bool lap_found;             // Indication if Lap recognised
uint32_t max_seq_cnt;    // Maximum number of segments
sensors_event_t accel_orient;
CAR_STATE state;
uint32_t dirReadingOld;
uint32_t speedReading, speedReadingLogged, distance;
double brake_faktor;
uint32_t startTime;
bool bno_on;
uint32_t sp, sp1, sp2, sp3, sp4;  // last 4 values of speedReading
bool measuring;          // don't start measuring until BNO055 gives real values 
int dirOverflow;
Adafruit_BNO055 bno(&arduino);
uint32_t segment_types_speed[100];
uint32_t finish_position;   // Lap length e.g. start/finish point
int trackVoltReading;
straight_type straight[100];// Max 100 straights can be stored
uint32_t straight_cnt;      // Straight counter
Metro measurement_timer = Metro(LOG_INTERVAL);




 
int main(int argc, char **argv)
{

  while (arduino.loop())
  {
    std::cout << arduino.matrix[arduino.row][26] << ", ";
    std::cout << arduino.matrix[arduino.row][27] << ", ";
    std::cout << arduino.matrix[arduino.row][28] << ", ";
    std::cout << arduino.matrix[arduino.row][29] << ", ";
    std::cout << std::endl;
    std::cout << "Speed Counter: " << speedCounter << std::endl;
    std::cout << "Direction: " << dirReading << std::endl;
    analogRead(track_volt_PIN);
    analogRead(engine_volt_PIN);
  }

  return 0;
}


