#pragma once
#include <stdio.h>

#if ECHO_TO_SERIAL
#define LOG_SERIAL_LN(x) POG(x);
#define LOG_SERIAL_LN2(x,y) POG2(x,y);
#define LOG_SERIAL(x) POG(x);
#else
#define LOG_SERIAL_LN(x)
#define LOG_SERIAL_LN2(x,y)
#define LOG_SERIAL(x)
#endif

#if SD_CARD_ON 
#define LOG_SDCARDL_LN(x) puts(x);
#define LOG_SDCARDL(x) printf(x);
#else
#define LOG_SDCARD_LN(x)
#define LOG_SDCARD(x)
#endif

#define LOG(x) LOG_SERIAL(x) LOG_SERIAL(", ") LOG_SDCARD(x) LOG_SDCARD(", ")
#define LOG_LN(x) LOG_SERIAL_LN(x) LOG_SDCARD_LN(x)

// Arduino's "put this string into Flash" macro:
#define F(x) x

//----------------------------------------------------------------
// Fatal error: print message to serial, red LED on, stop program.
// Stop motor.
void error(const char *str);

//------------------------------------------------------------------------
// Logging functions
extern void POG();
extern void POG(const char* str);
extern void POG(long unsigned int val);

extern void POG_LN(const char* str);
extern void POG_LN(long unsigned int val);
