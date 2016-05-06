#include "PinsParameters.h"
#if IR_ON
#include <IRremote.h>
#include "Infrared.h"
#endif
#include "Logging.h"
#include "Motor.h"
#if SD_CARD_ON
#include <SdFat.h>
#endif
#include <Metro.h>
#include <PID_v1.h>
#if SD_CARD_ON || RADIO_ON
#include <SPI.h>
#endif
#if RADIO_ON
#include <RH_NRF24.h>
#include "Telemetry.h"
#endif
#if BNO055_ON
#include <Adafruit_Sensor.h>
#if SIMULATION
#include "BnoSim.h"
#else
#include <Wire.h>
#include <Adafruit_BNO055.h>
#endif
#endif

/*-------------------( Global Variables )-----------------------------------*/
// When to stop the car so that it doesn't drive endlessly during test:
extern uint32_t stoptime;

// Timers that control the update rates of various things in the main loop:
// Units are milliseconds.
extern Metro pid_timer;
extern Metro measurement_timer;
extern Metro racing_timer;

// PID Controller
extern double PID_setpoint, PID_input, PID_output;
extern PID PIDcontroller;
// Variables for ASR defining the maximum PWM change
extern double PID_output_maximized,PID_output_maximized_old;
extern double PID_max_change;

// Acceleration and orientation
extern sensors_event_t accel_orient;

// Flag if the BNO055 sensor can be used. 
// The BNO can only be used when it has power applied (5V).
// This is only the case when the car is on the track and gets
// power from there, but not when only the Teensy is powered 
// by USB.  Then the BNO has no power.  Driving the Teensy 
// output pins to the BNO can (and has) locked up the BNO.
extern bool bno_on;
// BNO055 sensor and the data we read from it:
extern Adafruit_BNO055 bno;
#if ! SIMULATION
extern imu::Vector<3> acceleration;
extern imu::Vector<3> lacceleration;
extern imu::Vector<3> gravity;
extern imu::Vector<3> magnometer;
extern imu::Vector<3> gyroscope;
extern imu::Vector<3> euler;
extern imu::Quaternion quat;
#endif 

// Infrared remote control receiver
#if IR_ON
extern IRrecv irrecv;
extern decode_results ir_results;
#endif

// Track Voltage Reading and Direction Correction, later is needed due to 360->0 overflow
// not properly working yet
extern int trackVoltReading;
extern bool trackPowerPresent;

extern int dirOverflow;
extern uint32_t dirReadingOld;

// SD card
#if SD_CARD_ON
extern File logfile;
// File system object.
extern SdFat SD;
// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
extern const uint8_t spiSpeed;
//const uint8_t spiSpeed = SPI_HALF_SPEED;
//const uint8_t spiSpeed = SPI_QUARTER_SPEED;
#endif

// Telemetry
#if RADIO_ON
  extern message_t message;
  extern measurement_t measurement;
  extern uint32_t seq_no;
  // Set up nRF24L01 radio on SPI bus
  extern Telemetry telemetry;
#endif //RADIO_ON

// Wheel encoder, updated by an interrupt routine
extern volatile uint32_t speedCounter;
extern uint32_t speedReading, speedReadingLogged, distance;
extern uint32_t sp, sp1, sp2, sp3, sp4;  // last 4 values of speedReading

// IR button/action
extern char irButton[30];

#if RADIO_ON
extern message_t message;
extern measurement_t measurement;
extern char text[26];

// Sequence number for radio messages to detect message loss
extern uint32_t seq_no;
#endif

//---------------------------------------------------------------
// Car state machine

enum CAR_STATE {
  STARTING_UP = 0,
  MEASURING,
  MEASURING_FINISHED,
  WAIT_FOR_START,
  RACING,
  FINISH,
};

extern CAR_STATE state;

enum RACE_STATE {
  ACCELERATE = 0,
  BRAKE,
  STEADY,
};

extern RACE_STATE race_state;

extern uint32_t startTime;

//---------------------------------------------------------------
// Records for lap detection and lap segment description
// Different segment types
enum SEGMENT_TYPES {
  SLIDE = 0,   // slide
  CURVE_1,     // tightest
  CURVE_2,     // normal
  CURVE_3,     // wide
  CURVE_4,     // widest
  STRAIGHT,
};

// Wanted speed for the different segment types
extern uint32_t segment_types_speed[];
//uint32_t segment_types_speed[] = {25,30,40,60,75,255};

// brake faktor in m/s^2
// needs to be calculated at every different voltage, track surface, etc
extern double brake_faktor;
 
// Lap segment storage records
struct lap_segment{
  uint32_t position;            // Position on track
  uint32_t length;              // Length of segment
  uint32_t direction;           // Direction of (segment/)straight
  SEGMENT_TYPES segment_type;   // Segment type
  uint32_t wanted_speed;        // Wanted speed for this segment
};

// Lap segment detection support records
struct lap_segment_hist{
  uint32_t position;            // Position on track
  uint32_t direction;           // Direction of (segment/)straight
  SEGMENT_TYPES segment_type;   // Segment type
  uint32_t wanted_speed;        // Wanted speed for this segment
};

extern uint32_t seq_cnt;        // Segment sequence number
extern uint32_t max_seq_cnt;    // Maximum number of segments
extern lap_segment lap[1000];
extern lap_segment_hist lap_hist[10];
extern bool measuring;          // don't start measuring until BNO055 gives real values 
extern uint32_t lap_cnt;        // Lap Counter

// Lap recognition based on straights help records
struct straight_type{
  uint32_t position;            // Starting position of straight
  uint32_t length;              // Length of straight
  uint32_t direction;           // Direction of straight
};

extern uint32_t straight_cnt;      // Straight counter
extern straight_type straight[100];// Max 100 straights can be stored
extern bool lap_found;             // Indication if Lap recognised
extern uint32_t finish_position;   // Lap length e.g. start/finish point
extern uint32_t brake_point;       // brake point before a curve
extern uint32_t brake_segment;     // segment towards which we are breaking


