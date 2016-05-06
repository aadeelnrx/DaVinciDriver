/*
 DaVinciDriver

 This program reads the values from the sensor, stores them on SD and transmits them
 via the radio
 
 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Teensy 3.1/3.2
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */

#include "PinsParameters.h"
#include "datatypes.h"
#include <IRremote.h>
#include "Infrared.h"
#include "Logging.h"
#include "Motor.h"
#include <SdFat.h>
#include <Metro.h>
#include <PID_v1.h>
#include <SPI.h>
#include "Telemetry.h"
#include <RH_NRF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "measuring.h"
#include "racing.h"


/*-------------------( Global Variables )-----------------------------------*/
// When to stop the car so that it doesn't drive endlessly during test:
uint32_t stoptime;

// Timers that control the update rates of various things in the main loop:
// Units are milliseconds.
Metro pid_timer = Metro(PID_INTERVAL);
Metro measurement_timer = Metro(LOG_INTERVAL);
Metro racing_timer = Metro(RACING_INTERVAL);

// PID Controller
double PID_setpoint, PID_input, PID_output;
PID PIDcontroller(&PID_input, &PID_output, &PID_setpoint, MOTOR_M_P, MOTOR_M_I, MOTOR_M_D, DIRECT);
// Variables for ASR defining the maximum PWM change
double PID_output_maximized,PID_output_maximized_old = 0;
double PID_max_change = 25;

// Acceleration and orientation
sensors_event_t accel_orient;

// Flag if the BNO055 sensor can be used. 
// The BNO can only be used when it has power applied (5V).
// This is only the case when the car is on the track and gets
// power from there, but not when only the Teensy is powered 
// by USB.  Then the BNO has no power.  Driving the Teensy 
// output pins to the BNO can (and has) locked up the BNO.
bool bno_on = false;
// BNO055 sensor and the data we read from it:
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> acceleration;
imu::Vector<3> lacceleration;
imu::Vector<3> gravity;
imu::Vector<3> magnometer;
imu::Vector<3> gyroscope;
imu::Vector<3> euler;
imu::Quaternion quat;

// Infrared remote control receiver
#if IR_ON
IRrecv irrecv(IR_receiver);
decode_results ir_results;
#endif

// Track Voltage Reading and Direction Correction, later is needed due to 360->0 overflow
// not properly working yet
int trackVoltReading;
bool trackPowerPresent;

int dirOverflow = 0;
uint32_t dirReadingOld;

// SD card
#if SD_CARD_ON
File logfile;
// File system object.
SdFat SD;
// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_FULL_SPEED;
//const uint8_t spiSpeed = SPI_HALF_SPEED;
//const uint8_t spiSpeed = SPI_QUARTER_SPEED;
#endif

// Telemetry
#if RADIO_ON
  message_t message;
  measurement_t measurement;
  uint32_t seq_no = 0;
  // Set up nRF24L01 radio on SPI bus
  Telemetry telemetry(CE_Radio_PIN, CSN_Radio_PIN);
#endif //RADIO_ON

// Wheel encoder, updated by an interrupt routine
volatile uint32_t speedCounter = 0;
uint32_t speedReading, speedReadingLogged, distance = 0;
uint32_t sp, sp1, sp2, sp3, sp4;  // last 4 values of speedReading

// IR button/action
char irButton[30];

#if RADIO_ON
message_t message;
measurement_t measurement;
char text[26];

// Sequence number for radio messages to detect message loss
uint32_t seq_no;
#endif

//---------------------------------------------------------------
// Car state machine
CAR_STATE state = STARTING_UP;

RACE_STATE race_state = ACCELERATE;

uint32_t startTime;

//---------------------------------------------------------------
// Records for lap detection and lap segment description


// Wanted speed for the different segment types
uint32_t segment_types_speed[] = {20,30,40,60,80,255};
//uint32_t segment_types_speed[] = {25,30,40,60,75,255};

// brake faktor in m/s^2
// needs to be calculated at every different voltage, track surface, etc
double brake_faktor = 3.5;

uint32_t seq_cnt = 0;           // Segment sequence number
uint32_t max_seq_cnt = 1000;    // Maximum number of segments
lap_segment lap[1000];
lap_segment_hist lap_hist[10];
bool measuring = false;         // don't start measuring until BNO055 gives real values 
uint32_t lap_cnt = 0;           // Lap Counter

uint32_t straight_cnt = 0;      // Straight counter
straight_type straight[100];    // Max 100 straights can be stored
bool lap_found = false;         // Indication if Lap recognised
uint32_t finish_position = 0;   // Lap length e.g. start/finish point
uint32_t brake_point = 0;       // brake point before a curve
uint32_t brake_segment = 0;     // segment towards which we are breaking



//------------------------------------------------------------------------
// Interrupt service routine for the wheel encoder:
void countSpeed(){
  speedCounter++;
}

//============================================================================
//============================================================================
// Setup:  Initialize everything
void setup(void)
{
  // Initialise pins
  pinMode(red_LED_PIN,   OUTPUT);
  pinMode(green_LED_PIN, OUTPUT);
  pinMode(speed_PIN,     INPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CSN_CF_PIN,    OUTPUT);
  pinMode(CSN_Radio_PIN, OUTPUT);

  pinMode(CE_Radio_PIN,  OUTPUT);

  // Find out if the car is on the track and gets power from there.
  // Only then drive the BNO, SD-card, and Pololu:
  trackVoltReading = analogRead(track_volt_PIN);
  trackPowerPresent = (trackVoltReading > 100);   // 400 is ca. 10V -> 100 is ca. 2.5V
  
   
  motor_init();
 
  // PID
  PID_input = 0.0;
  PID_setpoint = MOTOR_TARGET_SPEED * PID_INTERVAL / 10 ;
  PIDcontroller.SetSampleTime(PID_INTERVAL);              // set sample time
  PIDcontroller.SetOutputLimits(0, 80);                   // limit the pwm input range for measuring 
  PIDcontroller.SetMode(AUTOMATIC);                       // start PID controller
  sp1 = sp2 = sp3 = sp4 = 0;    
  
  // The default SCK pin is connected to the LED which we use for something else
  // Required for Radio/NRF24L01+ and SD-card
  // !!! Must be before SPI.begin() if that is used !!!
  // In order to make this work, change Line 93 in SdFat/SdSpiTeensy3.cpp from:
  // CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // to:
  // CORE_PIN14_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
  // CORE_PIN13_CONFIG = PORT_PCR_MUX(1);
  SPI.setSCK(SCK_PIN);
  
  digitalWrite(green_LED_PIN, HIGH);

  // initialize the serial communication:
  Serial.begin(115200);

  // Blink to indicate it's going to start:
  for (int i=0; i < 4; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(500);
    digitalWrite(green_LED_PIN, LOW);
    delay(500);
  }
  LOG_SERIAL_LN("DaVinciDriver");


  if (trackPowerPresent == true)  {
    bno_on = true;
    // BNO055 IMU initialisation
    // Done here (4 seconds after serial initialisation) so that
    // we can print to serial here.
    LOG_SERIAL_LN("Before bno begin");
    if (!bno.begin())
    {
      Serial.println("bno error");
      //There was a problem detecting the BNO055 ... check your connections
      error("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    LOG_SERIAL_LN("After bno begin");
  }
  
  Serial.print("BNO is: ");
  Serial.println(bno_on ? "on" : "off (no track voltage?)");
//  while(1);
  
  // last second: fast blinking
  // The BNO055 needs one second to initialise anyway.
  for (int i=0; i < 10; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(50);
    digitalWrite(green_LED_PIN, LOW);
    delay(50);
  }
  
  if (bno_on)
  {
    // This has to be done at least one second after bno.begin():
    bno.setExtCrystalUse(true);  
  }

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

#if SD_CARD_ON
  // initialize the SD card
  LOG_SERIAL("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CSN_CF_PIN,spiSpeed)) {
    LOG_SERIAL_LN("Card failed, or not present");
    // don't do anything more:
    if (SD.card()->errorCode()) {
      LOG_SERIAL_LN(
             "\nSD initialization failed.\n"
             "Do not reformat the card!\n"
             "Is the card correctly inserted?\n"
             "Is chipSelect set to the correct value?\n"
             "Does another SPI device need to be disabled?\n"
             "Is there a wiring/soldering problem?\n");
      LOG_SERIAL("\nerrorCode: ");
      LOG_SERIAL(int(SD.card()->errorCode()));
      LOG_SERIAL(", errorData: ");
      LOG_SERIAL_LN(int(SD.card()->errorData()));
    }
    return;
  }
  LOG_SERIAL_LN("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }

  LOG_SERIAL("Logging to: ");
  LOG_SERIAL_LN(filename);
#endif

#if RADIO_ON
  // Initialize all radio related modules
  LOG_SERIAL_LN("Initializing radio...");
  seq_no = 0;
  
  //radio.begin();

  if (! radio.init())
    LOG_SERIAL_LN("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (! radio.setChannel(1))
    LOG_SERIAL_LN("setChannel failed");
  if (! radio.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    LOG_SERIAL_LN("setRF failed");   
   
  LOG_SERIAL_LN(F("Radio ready."));
#endif //RADIO_ON

#if IR_ON  
  irrecv.enableIRIn(); // Start the IR receiver
#endif

#if SD_CARD_ON
  logfile.println("Millis,Ex Time,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Set,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,Wanted_speed,dirChange");    
#endif
#if TEST_LOGGING_ON
  logfile.println("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Set,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,Wanted_speed,dirChange");    
#endif
#if ECHO_TO_SERIAL
  LOG_SERIAL_LN("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Set,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,Wanted_speed,dirChange");
#endif //ECHO_TO_SERIAL 

  //attempt to write out the header to the file
//  if (logfile.writeError || !logfile.sync()) {
//    error("write header");
//  }
  
// Use different interrupt pin for Teensy & Arduino 
#if defined(CORE_TEENSY)
  attachInterrupt(speed_PIN, countSpeed, CHANGE); 
#else    // Arduino
  //  digitalWrite(speed_PIN, HIGH);
  attachInterrupt(0, countSpeed, CHANGE); 
  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
#endif

  // Stop the car after some time
  stoptime = millis() + STOP_AFTER_SECONDS * 1000;

  // Reset metro timers so we start at 0
  pid_timer.reset();
  measurement_timer.reset();

  // change car state from initialising to measuring
  state = MEASURING;
}


//====================================================================
//====================================================================
// Main Loop
// Dispatch into the functions that handle each state.  These functions
// return only when the state has changed.
void loop(void)
{  
   switch(state)
   {
        case MEASURING:
           measuring_loop();
           break;
       case MEASURING_FINISHED:
           measuring_finished();
           break;
       case WAIT_FOR_START:
           wait_for_start_loop();
           break;
       case RACING:
           racing_loop();
           break;
       case FINISH:
           finish_loop();
           break;
       default:
           error("Unknown State");
   }
}


