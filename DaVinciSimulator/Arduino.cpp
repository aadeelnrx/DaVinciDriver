#include "Arduino.h"
#include "PinsParameters.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


extern Arduino arduino;
extern uint32_t speedCounter;
extern uint32_t dirReading;
struct timeval millis_starttime;

//------------------------------------------------------------------------
void digitalWrite(int pin, int val)
{
  printf("PIN %d digitalWrite %d\n", pin, val);
}


//------------------------------------------------------------------------
// You must call init_millis() or init_arduino() once at the
// start of your program before you can use this function
uint32_t millis()
{
  struct timeval endtime;
  gettimeofday(&endtime, 0);
  return (uint32_t)(endtime.tv_sec - millis_starttime.tv_sec 
                    + (endtime.tv_usec - millis_starttime.tv_usec) / 1000);
}


//------------------------------------------------------------------------
int analogRead(int pin)
{
  if ((pin == track_volt_PIN) || (pin == engine_volt_PIN))
  {
    int val = atoi(arduino.matrix.at(arduino.row).at(arduino.translate_analog_pin.at(pin)).c_str());
    printf("analogRead: pin %d:  %d\n", pin, val);
    return val;
  }
  else
  {
    printf("### analogRead: unknown pin %d", pin);
  }
  return 0;
}


//------------------------------------------------------------------------
// Fake interrupt functions that do nothing
void interrupts()
{
  return;
}

void noInterrupts()
{
  return;
}


//------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialize the simulated arduino
Arduino::Arduino(const std::string& csv_filename) : row(0)
{
  // Milliseconds since "startup":
  gettimeofday(&millis_starttime, 0);

  // Translate the analog pin number to the column in the LOGGER16.CSV file
  translate_analog_pin.resize(20, 0);
  translate_analog_pin.at(1) = 28;
  translate_analog_pin.at(2) = 29;

  // Columns in the LOGGER16.CSV file where these variables can be found:
  speedCounter_column = 26;
  dirReading_column   = 27;

  // Read CSV file into matrix
  loadFromCSV(csv_filename);
}

//------------------------------------------------------------------------
// Destructor
Arduino::~Arduino()
{
}

//------------------------------------------------------------------------
// CSV Reader from Jim M. (slightly modified)
// http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
void Arduino::loadFromCSV(const std::string& filename)
{
    std::ifstream file(filename.c_str());
    std::vector<std::string>   words;
    std::string                line;
    std::string                cell;

    while (file)
    {
        std::getline(file,line);
        std::stringstream lineStream(line);
        words.clear();

        while (std::getline(lineStream, cell, ','))
        {
          // Remove leading and trailing spaces
          unsigned int pos;
          for (pos = 0;  cell[pos] == ' ';  pos++);
          cell.erase(0, pos);
          if (cell.length() > 0)
          {
            for (pos = cell.length() - 1;  cell[pos] == ' ';  pos--);
            if (pos != cell.length() - 1) cell.erase(pos);
          }

          words.push_back(cell);
        }

        if (!words.empty())
            matrix.push_back(words);
    }
}

//------------------------------------------------------------------------
// Execute the next loop round.
// Read in the next row of simulated input parameters from the CSV file
// Returns:
//    true  -- if the loop can be executed, i.e. more data available
//    false -- if there is no more data
bool Arduino::loop()
{
  // Advance row index to the next non-empty row:
  do
  {
    row++;
  } while ((row < matrix.size()) && matrix[row][speedCounter_column].empty());

  // End reached?
  if (row == matrix.size())  // yes, end reached
  {
    return false;
  }
  else   // no, more data available
  {
    speedCounter = atoi(matrix.at(row).at(speedCounter_column).c_str());
    dirReading = atoi(matrix.at(row).at(dirReading_column).c_str());
    return true;
  }
}




