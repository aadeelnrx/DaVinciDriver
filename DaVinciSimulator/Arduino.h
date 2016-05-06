#pragma once

#include <string>
#include <vector>
#include <sys/time.h>

#define LOW 0
#define HIGH 1

#define uint32_t unsigned long int
#define A1 1
#define A2 2

extern struct timeval millis_starttime;

// Simulated Arduino/Teensy functions:
void digitalWrite(int pin, int val);
uint32_t millis();
int analogRead(int pin);
void interrupts();
void noInterrupts();

//--------------------------------------------------
// Helper functions for the simulated functions
class Arduino
{
public:
  // Constructor & Destructor
  Arduino(const std::string& csv_filename);
  ~Arduino();

  // Execute next loop.  Use the next row from the CSV file.
  bool loop();

  // Data matrix read from CSV file
  std::vector< std::vector<std::string> >   matrix;
  unsigned int row;
  std::vector<int> translate_analog_pin;

  int speedCounter_column; // in CSV file
  int dirReading_column;   // in CSV file

private:
  void loadFromCSV(const std::string& filename);

};


