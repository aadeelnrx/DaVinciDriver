#pragma once

// Logging functions
//
// It can log to serial and/or to SD card.
// If you want to log to serial, then:
// #define ECHO_TO_SERIAL
//
// If you want to log to SD card, then:
// #define SD_CARD_ON
//
// IMPORTANT!!!  You have to define these values **before** including
// this file!!!

#include "PinsParameters.h"
//---------------------------------------------------------------
// Macros to log data to either serial and/or SD-card.
// They are macros because then we don't have to define overloaded
// functions for all possible datatypes.
//    LOG_SERIAL(x)    and LOG_SDCARD(x)
//    LOG_SERIAL_LN(x) and LOG_SDCARD_LN(X)   with newline
//    LOG(x)   to both, comma-separation will be added
//    LOG_LN(x)  to both
#if ECHO_TO_SERIAL 
#define LOG_SERIAL_LN(x) Serial.println(x);
#define LOG_SERIAL_LN2(x,y) Serial.println(x,y);
#define LOG_SERIAL(x) Serial.print(x);
#else
#define LOG_SERIAL_LN(x)
#define LOG_SERIAL_LN2(x,y)
#define LOG_SERIAL(x)
#endif

#if SD_CARD_ON 
#define LOG_SDCARD_LN(x) logfile.println(x);
#define LOG_SDCARD(x) logfile.print(x);
#else
#define LOG_SDCARD_LN(x)
#define LOG_SDCARD(x)
#endif

#define LOG(x) LOG_SERIAL(x) LOG_SERIAL(", ") LOG_SDCARD(x) LOG_SDCARD(", ")
#define LOG_LN(x) LOG_SERIAL_LN(x) LOG_SDCARD_LN(x)


//----------------------------------------------------------------
// Fatal error: print message to serial, red LED on, stop program.
// Stop motor.
void error(const char *str);

