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
#include "Logging.h"
#include "Motor.h"
#include "Telemetry.h"
#include <SdFat.h>
#include <IRremote.h>
#include <Metro.h>
#include <PID_v1.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>



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
decode_results results;
#endif

// Track Voltage Reading and Direction Correction, later is needed due to 360->0 overflow
// not properly working yet
int trackVoltReading;
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


message_t message;
measurement_t measurement;
char text[26];

// Sequence number for radio messages to detect message loss
uint32_t seq_no;

//---------------------------------------------------------------
// Car state machine

enum CAR_STATE {
  STARTING_UP = 0,
  MEASURING = 1,
  WAIT_FOR_START = 2,
  RACING = 3,
  FINISH = 4,
};

CAR_STATE state = STARTING_UP;

enum RACE_STATE {
  ACCELERATE = 0,
  BRAKE = 1,
  STEADY = 2,
};

RACE_STATE race_state = ACCELERATE;

uint32_t startTime;

//---------------------------------------------------------------
// Records for lap detection and lap segment description
// Different segment types
enum SEGMENT_TYPES {
  SLIDE = 0,    // slide
  KURVE_1 = 1,  // tightest
  KURVE_2 = 2,  // normal
  KURVE_3 = 3,  // wide
  KURVE_4 = 4,  // widest
  STRAIGHT = 5,
};

// Wanted speed for the different segment types
uint32_t segment_types_speed[] = {20,30,40,60,80,255};
//uint32_t segment_types_speed[] = {25,30,40,60,75,255};

// brake faktor in m/s^2
// needs to be calculated at every different voltage, track surface, etc
double brake_faktor = 3.5;
 
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

uint32_t seq_cnt = 0;           // Segment sequence number
uint32_t max_seq_cnt = 1000;    // Maximum number of segments
lap_segment lap[1000];
lap_segment_hist lap_hist[10];
bool measuring = false;         // don't start measuring until BNO055 gives real values 
uint32_t lap_cnt = 0;           // Lap Counter

// Lap recognition based on straights help records
struct straight_type{
  uint32_t position;            // Starting position of straight
  uint32_t length;              // Length of straight
  uint32_t direction;           // Direction of straight
};

uint32_t straight_cnt = 0;      // Straight counter
straight_type straight[100];    // Max 100 straights can be stored
bool lap_found = false;         // Indication if Lap recognised
uint32_t finish_position = 0;   // Lap length e.g. start/finish point
uint32_t brake_point = 0;       // brake point before a curve
uint32_t brake_segment = 0;     // segment towards which we are breaking


//----------------------------------------------------------------
// Decode and act on IR codes
#if IR_ON
void translateIR() // takes action based on IR code received 
{
  switch(results.value)
  {
  case 0xFFA25D: 
    LOG_SERIAL_LN(" CH-"); 
    strcpy(irButton," CH-");
    break;
  case 0xFF629D:
    // Lights on/off 
    LOG_SERIAL_LN(" CH");   
    strcpy(irButton," CH");
    break;
  case 0xFFE21D: 
    LOG_SERIAL_LN(" CH+");  
    strcpy(irButton," CH+");
    break;
  case 0xFF22DD: 
    LOG_SERIAL_LN(" REVERSE");    
    strcpy(irButton," REVERSE");
    break;
  case 0xFF02FD: 
    LOG_SERIAL_LN(" FORWARD");    
    strcpy(irButton," FORWARD");
    break;
  case 0xFFC23D: 
    // Race Start
    LOG_SERIAL_LN(" PLAY/PAUSE"); 
    strcpy(irButton," PLAY/PAUSE");
    break;
  case 0xFFE01F: 
    LOG_SERIAL_LN(" -");    
    strcpy(irButton," -");
    break;
  case 0xFFA857: 
    LOG_SERIAL_LN(" +");    
    strcpy(irButton," +");
    break;
  case 0xFF906F: 
    // Race Stop
    LOG_SERIAL_LN(" EQ");   
    strcpy(irButton," EQ");
    break;
  case 0xFF6897: 
    LOG_SERIAL_LN(" 0");    
    strcpy(irButton," 0");
    break;
  case 0xFF9867: 
    LOG_SERIAL_LN(" 100+"); 
    strcpy(irButton," 100+");
    break;
  case 0xFFB04F: 
    LOG_SERIAL_LN(" 200+");   
    strcpy(irButton," 200+");
    break;
  case 0xFF30CF: 
    LOG_SERIAL_LN(" 1");    
    strcpy(irButton," 1");
    break;
  case 0xFF18E7: 
    LOG_SERIAL_LN(" 2");    
    strcpy(irButton," 2");
    break;
  case 0xFF7A85: 
    LOG_SERIAL_LN(" 3");    
    strcpy(irButton," 3");
    break;
  case 0xFF10EF: 
    LOG_SERIAL_LN(" 4");    
    strcpy(irButton," 4");
    break;
  case 0xFF38C7: 
    LOG_SERIAL_LN(" 5");    
    strcpy(irButton," 5");
    break;
  case 0xFF5AA5: 
    LOG_SERIAL_LN(" 6");    
    strcpy(irButton," 6");
    break;
  case 0xFF42BD: 
    LOG_SERIAL_LN(" 7");    
    strcpy(irButton," 7");
    break;
  case 0xFF4AB5: 
    LOG_SERIAL_LN(" 8");    
    strcpy(irButton," 8");
    break;
  case 0xFF52AD: 
    LOG_SERIAL_LN(" 9");    
    strcpy(irButton," 9");
    break;
  case 0xFFFFFFFF: 
    LOG_SERIAL_LN(" REPEAT"); 
    strcpy(irButton," REPEAT");
    break;  

  default: 
    LOG_SERIAL(" other button   :");
    LOG_SERIAL_LN2(results.value, HEX);
    strcpy(irButton," other button   :");
  }// End Case
} //END translateIR
#endif


//------------------------------------------------------------------------
// Interrupt service routine for the wheel encoder:
void countSpeed(){
  speedCounter++;
}

//============================================================================
//============================================================================
void setup(void)
{
  // initialise pins
  pinMode(red_LED_PIN, OUTPUT);
  pinMode(green_LED_PIN, OUTPUT);
  pinMode(speed_PIN, INPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CSN_CF_PIN, OUTPUT);
  pinMode(CSN_Radio_PIN, OUTPUT);

  pinMode(CE_Radio_PIN, OUTPUT);
   
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

#if BNO055_ON
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
#endif
  // last second: fast blinking
  // The BNO055 needs one second to initialise anyway.
  for (int i=0; i < 10; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(50);
    digitalWrite(green_LED_PIN, LOW);
    delay(50);
  }
  
#if BNO055_ON
// This has to be done at least one second after bno.begin():
  bno.setExtCrystalUse(true);  
#endif

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
void loop(void)
{  
  // Stop motor after some time
//  if (millis() >= stoptime)
//  {
//    LOG_SERIAL_LN("Stoptime expired")
//    digitalWrite(green_LED_PIN, HIGH);
//    motor_off_brake();
//    while(1);
//  }

//=================== RACE FINISHED ====================================================
  if (state == FINISH)
  {
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    digitalWrite(green_LED_PIN, HIGH);
    // Stop motor
    motor_off_brake();
    LOG(msec_since_start)
    LOG("FINISH")
    LOG(lap_cnt)
    LOG_LN()    
#if SD_CARD_ON
    logfile.flush();
#endif
    // Stop program
    while(1);
  }

//=================== MEASURING FINISHED ====================================================
  // Stop motor at start/finish straight after lap detection has finished
  // Still not 100% working, needs fixing
  if ((state==MEASURING) && (finish_position > 0) && (distance >= (2*finish_position - ((10*MOTOR_TARGET_SPEED*MOTOR_TARGET_SPEED/25)/(2*brake_faktor)))))
  {
    // stop motor
    motor_off_brake();
    
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    // Set PID controller to manual, e.g. switch off
    PIDcontroller.SetMode(MANUAL);

    LOG_SERIAL_LN("MEASURING FINISHED => WAIT_FOR_START")
    digitalWrite(green_LED_PIN, HIGH);
    // Determine which seq_cnt belongs to the end of the start/finish 
    // straight and limit the stored lap to that
    LOG(msec_since_start)
    LOG_LN("Writing Lap Data")
    for (uint32_t i=0; i < seq_cnt ; i++)
    {
      LOG(i)
      LOG(lap[i].position)
      LOG(lap[i].length)
      LOG(lap[i].direction)
      LOG(lap[i].segment_type)
      LOG(lap[i].wanted_speed)
      LOG_LN()    
      if (lap[i].position > finish_position)
      {
        max_seq_cnt = i;
      }
    }

    // consolidate Lap
    bool decrease = true;
    int new_seq_cnt = 0;
    int i = 0;
    int j = 0;
    while (i < max_seq_cnt)
    {
      if (decrease == true) // decrease speed
      {
        while (lap[j].wanted_speed >= lap[1+j].wanted_speed)
        {
          j++;
        }
        lap[new_seq_cnt].length = lap[j].position - lap[new_seq_cnt].position;
        new_seq_cnt++;
        lap[new_seq_cnt].position = lap[j].position;
        lap[new_seq_cnt].length = lap[j].length;
        lap[new_seq_cnt].direction = lap[j].direction;
        lap[new_seq_cnt].segment_type = lap[j].segment_type;
        lap[new_seq_cnt].wanted_speed = lap[j].wanted_speed;
        new_seq_cnt++;
        lap[new_seq_cnt].position = lap[j+1].position;
        decrease = false;
        i = j;
      } else //increase speed
      {
        while (lap[j].wanted_speed <= lap[1+j].wanted_speed)
        {
          j++;
        }
        lap[new_seq_cnt].direction = lap[j].direction;
        lap[new_seq_cnt].segment_type = lap[j].segment_type;
        lap[new_seq_cnt].wanted_speed = lap[j].wanted_speed;
        decrease = true;
        i = j;
      }
    }
    max_seq_cnt = new_seq_cnt;

    LOG(msec_since_start)
    LOG_LN("Writing Consolidated Lap Data")
    for (int i=0; i < max_seq_cnt ; i++)
    {
      LOG(i)
      LOG(lap[i].position)
      LOG(lap[i].length)
      LOG(lap[i].direction)
      LOG(lap[i].segment_type)
      LOG(lap[i].wanted_speed)
      LOG_LN()    
    }    // set current leg segment counter to 0 e.g. start/finish
    seq_cnt = 0;

    // set the wanted speed for this segment
    PID_setpoint = lap[seq_cnt].wanted_speed * PID_INTERVAL / 10 ;

    // clear all speed variables
    sp1 = sp2 = sp3 = sp4 = 0;

    // change state from measuring to wait_for_start
    state = WAIT_FOR_START;

    // start the race 3 seconds from now
    startTime = millis() + 3000;
    
    LOG(msec_since_start)
    LOG("WAIT_FOR_START")
    LOG(seq_cnt)
    LOG(distance)
    LOG(PID_setpoint)
    LOG(startTime)
    LOG_LN()    
#if SD_CARD_ON
    logfile.flush();
#endif
//    while(1);
  }

//=================== WAIT FOR RACE START ====================================================
  if (state == WAIT_FOR_START)
  {
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    if (millis() > (startTime - PID_INTERVAL))
    {
      // Reset metro timers so we start at 0
 //     pid_timer.reset();
      racing_timer.reset(); 

      // set current distance from start/finish line
      LOG(msec_since_start)
      LOG("RACE Preparation")
      LOG(speedCounter)
      LOG(distance)

      // increase distance by distance the call rolled after stopping e.g. speedcounter
      distance += speedCounter;

      LOG(distance)
      LOG(finish_position)

      // calculate position from where starting
      // should not be below 0
      // still needs fixing
      distance -= (2*finish_position);

      LOG(distance)
      LOG_LN()    
#if SD_CARD_ON
      logfile.flush();
#endif

      // check if distance was below 0
      // still needs fixing
      if (distance > 100000)
      {
        distance = 0;
      }

      // reset all PID inputs and outputs
      speedCounter = 0;
      PID_output = 0;
      // set race PID settings
      PIDcontroller.SetTunings(MOTOR_R_P,MOTOR_R_I,MOTOR_R_D);
      PIDcontroller.SetOutputLimits(0, 255);                      // remove previous pwm limits
      // Set PID controller to auto, e.g. switch on
//      PIDcontroller.SetMode(AUTOMATIC);
      
      // change state from wait_for_start to racing
      state = RACING;
      
      LOG(msec_since_start)
      LOG("RACING")
      LOG_LN()    
#if SD_CARD_ON
      logfile.flush();
#endif
    }  
  }
    
//=================== MEASURING ====================================================
  if (measurement_timer.check()&&(state == MEASURING))
  {
    LOG_SERIAL_LN("MEASURING")
    digitalWrite(green_LED_PIN, HIGH);

    // Log milliseconds since starting
    uint32_t msec_since_start = millis();
  
    // Take all measurement values right at the beginning of the
    // main loop to ensure taking them after the same interval
    // all the time
    trackVoltReading = analogRead(track_volt_PIN);    
    int engineVoltReading = analogRead(engine_volt_PIN);    

    noInterrupts();
    speedReading = speedCounter;  //calculate speed based on speedCounter
    speedCounter = 0;                 //reset speedCounter 
    interrupts();

    speedReadingLogged += speedReading;
    distance += speedReading;
    sp = speedReading + sp1 + sp2 + sp3 + sp4;

    // Motor also as early as possible to ensure constant update intervals
    PID_input = (float)sp;
    // reset PID control in case of de-slot
    if ((trackVoltReading < 100)&&(speedReading <= 1))
    {
      LOG(msec_since_start)
      LOG("DESLOT")
      LOG_LN()
      PIDcontroller.SetMode(MANUAL);
      PID_output = 0;
      PIDcontroller.SetMode(AUTOMATIC);
    }
    PIDcontroller.Compute();
    motor_on_pwm(PID_output);

    // for handcontroller running of the car 
//    motor_on_pwm(255);

    int32_t dirReading;
    float dirChange = 0.0;
#if BNO055_ON
    // Read the acceleration and orientation:
    bno.getEvent(&accel_orient);

    dirReading = uint32_t(accel_orient.orientation.x);

    // Start measuring after BNO055 starts giving real values
    if ((dirReading > 0)&&(measuring == false))
    {
      measuring = true;                                           // BNO055 is giving real values so measuring can start
      
      // set-up first segment
      lap[seq_cnt].position = 0;                                  // start/finish is at position 0
      lap[seq_cnt].direction = dirReading;
      lap[seq_cnt].segment_type = STRAIGHT;
      lap[seq_cnt].wanted_speed = segment_types_speed[STRAIGHT];  // assumption that start/finish is on a straight
      seq_cnt++;

      // initialise measurement history variables
      for (int i=0; i < 9; i++)
      {
        lap_hist[i].position = 0;                               // start/finish is at position 0
        lap_hist[i].direction = dirReading;
        lap_hist[i].segment_type = STRAIGHT;          
        lap_hist[i].wanted_speed = segment_types_speed[STRAIGHT];
      }        
    }
    
    // Measuring whenever the direction changes once the BNO has started up
    if (measuring == true)
    {

      // still needs fixing
      dirOverflow = 0;
      if ((lap[seq_cnt-1].direction > 270) && (dirReadingOld >= lap[seq_cnt-1].direction) && (dirReading <90))
      {
        dirOverflow = 360;
      }
      if ((lap[seq_cnt-1].direction < 90) && (dirReadingOld <= lap[seq_cnt-1].direction) && (dirReading >270))
      {
        dirOverflow = -360;
      }
      dirReading += dirOverflow;  
      dirReadingOld = dirReading;

      if ( abs(dirReading - lap[seq_cnt-1].direction) > 1)
      {
        // rotate history of last 10 measurements (FIFO)
        for (int i=8; i >= 0 ; i--)
        {
          lap_hist[i+1].position = lap_hist[i].position;
          lap_hist[i+1].direction = lap_hist[i].direction;
          lap_hist[i+1].segment_type = lap_hist[i].segment_type;     
          lap_hist[i+1].wanted_speed = lap_hist[i].wanted_speed;     
        }        
        // store current measurement
        lap_hist[0].position = distance;
        lap_hist[0].direction = dirReading;
        // determine the relative direction change during the last 10 measurements (to smoothen the measurement result curve)
        dirChange = (((float)abs(dirReading - lap_hist[9].direction))/(float)(distance -lap_hist[9].position));
        if (dirChange < 0.05)
        { 
          lap_hist[0].segment_type = STRAIGHT;
          lap_hist[0].wanted_speed = segment_types_speed[STRAIGHT];
        }else if (dirChange < 0.087)
        { 
          lap_hist[0].segment_type = KURVE_4;
          lap_hist[0].wanted_speed = segment_types_speed[KURVE_4];
        }else if (dirChange < 0.125)
        { 
          lap_hist[0].segment_type = KURVE_3;
          lap_hist[0].wanted_speed = segment_types_speed[KURVE_3];
        }else if (dirChange < 0.25)
        { 
          lap_hist[0].segment_type = KURVE_2;
          lap_hist[0].wanted_speed = segment_types_speed[KURVE_2];
        }else if (dirChange < 0.5)
        { 
          lap_hist[0].segment_type = KURVE_1;
          lap_hist[0].wanted_speed = segment_types_speed[KURVE_1];
        }else
        { 
          lap_hist[0].segment_type = SLIDE;
          lap_hist[0].wanted_speed = segment_types_speed[SLIDE];
        }   

        // Create new lap_segment in case direction has changed in the last 6 measurements (to avoid short peeks/drops)
        if ((lap_hist[0].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[1].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[2].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[3].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[4].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[5].segment_type != lap[seq_cnt-1].segment_type))
        {
          // In case of dirOverflow reverse the dirOverflow change done
          if (dirOverflow != 0)
          {
            for (int i=0; i < 9; i++)
            {
              lap_hist[i].direction -= dirOverflow;
            }        
          }
          lap[seq_cnt].position = lap_hist[5].position;                             // set starting position of next segment
          lap[seq_cnt-1].length = lap[seq_cnt].position - lap[seq_cnt-1].position;  // calculate and set length of last segment
          lap[seq_cnt].direction = lap_hist[5].direction;                           // set other values of next segment
          lap[seq_cnt].segment_type = lap_hist[5].segment_type;
          lap[seq_cnt].wanted_speed = lap_hist[5].wanted_speed;

          // to avoid short segments
          if (lap[seq_cnt-1].length > 20)
          {
            seq_cnt++;           
 
            // If new lap segment is a straight, store a new straight and do lap recognition based on the straights
            if (lap[seq_cnt-2].segment_type == STRAIGHT)
            {
              straight[straight_cnt].position = lap[seq_cnt-2].position;
              straight[straight_cnt].length = lap[seq_cnt-2].length;
              straight[straight_cnt].direction = lap[seq_cnt-2].direction;
            
              // check if new straight has been driven/measured before
              for (uint32_t i=0; i < straight_cnt; i++)
              {
                if ((straight[straight_cnt].length>100)&&(abs(straight[i].direction - straight[straight_cnt].direction) < 8)&&(abs(straight[i].length - straight[straight_cnt].length) < 40))
                {
                  lap_found = true;                                                         // lap has been detected
                  finish_position = straight[straight_cnt].position - straight[i].position; // subtracting found straight from current straight is lap length e.g. finish position
                }
              }
              straight_cnt++;
            }
          }else
          {
            // if last segment was too short and new segments is tighter => change segment_type to tighter type
            if (lap_hist[5].wanted_speed < lap[seq_cnt-1].wanted_speed)
            {
              lap[seq_cnt-1].segment_type = lap_hist[5].segment_type;            
              lap[seq_cnt-1].wanted_speed = lap_hist[5].wanted_speed;
            }
          }        
        }
      }
    }
#endif  
  
#if BNO055_TEST_ON
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    acceleration  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    lacceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    gravity       = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    magnometer    = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    quat          = bno.getQuat();
#endif  
  
#if IR_ON
    if (irrecv.decode(&results)) // have we received an IR signal?
    {
//    LOG_SERIAL_LN(results.value, HEX);  UN Comment to see raw values
      translateIR(); 
      irrecv.resume(); // receive the next value
    }  
#endif

    // Log execution time in milliseconds
    uint32_t execution_time_msec = millis() - msec_since_start;

    // Log measurements to serial and/or flash:
    LOG(msec_since_start)
    LOG(execution_time_msec)
    
#if TEST_LOGGING_ON
    LOG(accel_orient.orientation.x)
    LOG(accel_orient.orientation.y)
    LOG(accel_orient.orientation.z)
    
    LOG(acceleration.x())
    LOG(acceleration.y())
    LOG(acceleration.z())
    
    LOG(lacceleration.x())
    LOG(lacceleration.y())
    LOG(lacceleration.z())
    
    LOG(gravity.x())
    LOG(gravity.y())
    LOG(gravity.z())
    
    LOG(magnometer.x())
    LOG(magnometer.y())
    LOG(magnometer.z())
    
    LOG(gyroscope.x())
    LOG(gyroscope.y())
    LOG(gyroscope.z())
    
    LOG(euler.x())
    LOG(euler.y())
    LOG(euler.z())
    
    LOG(quat.w())
    LOG(quat.x())
    LOG(quat.y())
    LOG(quat.z())
#endif
    
    LOG(speedReadingLogged)
    LOG(distance)
    LOG(dirReading)
    LOG(trackVoltReading)    
    LOG(engineVoltReading)

    LOG(sp1)
    LOG(sp2)
    LOG(sp3)
    LOG(sp4)
    LOG(speedReading)

    LOG(PID_input)
    LOG(PID_setpoint)
    LOG(PID_output)
    LOG(MOTOR_M_P)
    LOG(MOTOR_M_I)
    LOG(MOTOR_M_D)

#if IR_ON
    LOG(irButton)
    strcpy(irButton,"");
#endif

    if (seq_cnt > 1)
    {
      LOG(seq_cnt-1)
      LOG(lap[seq_cnt-1].position)
      LOG(lap[seq_cnt-2].length)
      LOG(lap[seq_cnt-1].direction)
      LOG(lap[seq_cnt-1].segment_type)
      LOG(lap[seq_cnt-1].wanted_speed)
      LOG(dirChange)
      LOG(straight_cnt)
      LOG(lap_found)
      LOG(finish_position)
    }else
    {
      LOG(seq_cnt);
    }
    
    LOG_LN()
    
#if SD_CARD_ON
    logfile.flush();
#endif
    
#if RADIO_ON
    seq_no++;
    // If we check here that the packet was sent from the previous loop,
    // we save time because we could already do lots of other things in the
    // meantime
  //  if (seq_no > 1) radio.waitPacketSent();
 
    // Construct the message we'll send
    message.msg_type = MSG_MEASUREMENT;
    message.msg.measurement = (measurement_t){seq_no, msec_since_start, speedReading, dirReading, sp /*trackVoltReading*/};
  
    LOG_SERIAL_LN(F("Transmitting on radio"));
    telemetry.send_msg_wait((uint8_t *)&message);
 
    // Did we receive something from the PC? 
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    while (telemetry.waitAvailableTimeout(0))
    { 
      // Should be a reply message for us now   
      if (telemetry.recv(buf, &len))
      {
  //      Serial.print("got reply: ");
  //      Serial.println((char*)buf);
      }
      else
      {
  //      Serial.println("recv failed");
      }
   
   }
#endif //RADIO_ON

    // reset speedvariables
    speedReadingLogged = 0;

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

    digitalWrite(green_LED_PIN, LOW);
  } // measurement updates

//=================== RACING ====================================================

  if (racing_timer.check()&&(state == RACING))
  {
    digitalWrite(green_LED_PIN, HIGH);

    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    // Take all measurement values right at the beginning of the
    // main loop to ensure taking them after the same interval
    // all the time
    trackVoltReading = analogRead(track_volt_PIN);    
    int engineVoltReading = analogRead(engine_volt_PIN);    

    noInterrupts();
    speedReading = speedCounter;  //calculate speed based on speedCounter
    speedCounter = 0;                 //reset speedCounter 
    interrupts();

    speedReadingLogged += speedReading;
    distance += speedReading;

    // reset PID control in case of de-slot
    // still needs fixing
    if ((trackVoltReading < 100)&&(speedReading <= 1))
    {
      LOG(msec_since_start)
      LOG("DESLOT")
      LOG_LN()
    }
    
    int32_t dirReading;
#if BNO055_ON
    // Read the acceleration and orientation:
    bno.getEvent(&accel_orient);

    dirReading = uint32_t(accel_orient.orientation.x);
     
#endif  
 
#if IR_ON
    if (irrecv.decode(&results)) // have we received an IR signal?
    {
//    LOG_SERIAL_LN(results.value, HEX);  UN Comment to see raw values
      translateIR(); 
      irrecv.resume(); // receive the next value
    }  
#endif

    // Check if lap has finished
    if (distance > finish_position)
    {
      seq_cnt = 0;
      distance -= finish_position;
      lap_cnt++;
      if (lap_cnt > 3)                // finish the race after 3 laps
      {
        state = FINISH;
      }
    }
    
    sp = speedReading + sp1 + sp2 + sp3 + sp4;
    PID_input = (float)sp;
  
    // check if we changed into next segment
    if ( distance >= lap[seq_cnt+1].position)
    {
      seq_cnt++;   

      // check if next segment is straight or curve
      if (lap[seq_cnt].segment_type == STRAIGHT)
      {
        // switch off PID
        PIDcontroller.SetMode(MANUAL);
        race_state = ACCELERATE;   
      }else
      {
        if (seq_cnt >= brake_segment)
        {
          PID_setpoint = lap[seq_cnt].wanted_speed * PID_INTERVAL / 10 ;
          if (race_state != STEADY)
          {
            race_state = STEADY;   
            // still needs fixing
//            PID_output = 0;
            PID_output = lap[seq_cnt].wanted_speed * 1.6;
            // Set PID controller to auto, e.g. switch on
            PIDcontroller.SetMode(AUTOMATIC);
          }
        }
      }     
    }

    // do action according to segment_type
    if (lap[seq_cnt].segment_type == STRAIGHT)
    {
      // in case race_state=ACCELERATE calculate for next 5 segments the brake_point = (Vstart^2 - Vend^2)/(2 * brake_faktor)
      if (race_state == ACCELERATE)
      {
        // initialise brake_point to start of next segment
        brake_point = lap[seq_cnt+1].position;
        for (int i=1; i < 6 ; i++)
        {     
          if (sp > lap[seq_cnt+i].wanted_speed)
          {
            // add 5 speed intervals to brake distance as it takes about 50ms before car starts slowing down
            // TODO: ??? can bp ever be negative?  I changed "int bp" to "uint32_t bp" -- ALEX
            uint32_t bp = lap[seq_cnt+i].position - (uint)round((sp/5)*(5)) - (uint)round(((sp*sp/2.5) - (lap[seq_cnt+i].wanted_speed*lap[seq_cnt+i].wanted_speed/2.5))/(2*brake_faktor));
            // take closest brake point
            if (bp < brake_point)
            {
              brake_point = bp;
              brake_segment = seq_cnt+i;
            }
          }
        }
      }
      
      // check if brake_point has been reached
      if (distance < brake_point)
      {
        race_state = ACCELERATE;   
        // Calculate maximum increase of pwm
        if (255 > PID_output_maximized_old + PID_max_change)
        {
          PID_output_maximized = PID_output_maximized_old + PID_max_change;
        }else
        {
          PID_output_maximized = 255;
        }

        motor_on_pwm(PID_output_maximized);
        PID_output_maximized_old = PID_output_maximized;
//        motor_on_pwm(255);
//        motor_on();
      } else
      {
        race_state = BRAKE;   
        if (sp > (lap[brake_segment].wanted_speed * 1.0)) // hard brake
        {
//          if ((0 < PID_output_maximized_old - PID_max_change) && (PID_output_maximized_old > PID_max_change))
//          {
//            PID_output_maximized = PID_output_maximized_old - PID_max_change;          
//            motor_on_pwm(PID_output_maximized);
//            PID_output_maximized_old = PID_output_maximized;
//          }else
//          {
            motor_off_brake();
            PID_output_maximized = 0;
            PID_output_maximized_old = 0;
//          }
        }else                                           // soft brake
        {
          motor_off_coast();      
//          motor_off_brake();
        }
      }
    } else // kurve
    {
      // only do curve actions if hard braking towards brake_segment is finished
      if (seq_cnt >= brake_segment)
      {
        PID_input = (float)sp;
        PIDcontroller.Compute();

        // Calculate maximum increase/decrease of pwm
        if (PID_output > PID_output_maximized_old + PID_max_change)
        {
          PID_output_maximized = PID_output_maximized_old + PID_max_change;
        }else if ((PID_output < PID_output_maximized_old - PID_max_change) && (PID_output_maximized_old > PID_max_change))
        {
          PID_output_maximized = PID_output_maximized_old - PID_max_change;          
        }else
        {
          PID_output_maximized = PID_output;
        }

        motor_on_pwm(PID_output_maximized);
        PID_output_maximized_old = PID_output_maximized;
//        motor_on_pwm(lap[seq_cnt].wanted_speed*1.6);
      }
    }
    
    // Log execution time in milliseconds
    uint32_t execution_time_msec = millis() - msec_since_start;

    // Log measurements to serial and/or flash:
    LOG(msec_since_start)
    LOG(execution_time_msec)
    
    LOG(speedReadingLogged)
    LOG(distance)
    LOG(dirReading)
    LOG(trackVoltReading)    
    LOG(engineVoltReading)

    LOG(sp1)
    LOG(sp2)
    LOG(sp3)
    LOG(sp4)
    LOG(speedReading)

    LOG(PID_input)
    if (race_state == ACCELERATE)
    {
      LOG(255)  
    } else if (race_state == BRAKE)
    {
      LOG(0)  
    }else
    {
      LOG(PID_setpoint)  
    }
    LOG(PID_output)
    LOG(MOTOR_R_P)
    LOG(MOTOR_R_I)
    LOG(MOTOR_R_D)

    LOG("")
    LOG(PID_output_maximized)
    LOG(race_state)
    LOG(brake_point)
    LOG(brake_segment)

#if IR_ON
    LOG(irButton)
    strcpy(irButton,"");
#endif

    LOG(seq_cnt)
 
    LOG_LN()
    
#if SD_CARD_ON
    logfile.flush();
#endif
    
#if RADIO_ON
    seq_no++;
    // If we check here that the packet was sent from the previous loop,
    // we save time because we could already do lots of other things in the
    // meantime
  //  if (seq_no > 1) radio.waitPacketSent();
 
    // Construct the message we'll send
    message.msg_type = MSG_MEASUREMENT;
    message.msg.measurement = (measurement_t){seq_no, msec_since_start, speedReading, dirReading, sp /*trackVoltReading*/};
  
    LOG_SERIAL_LN(F("Transmitting on radio"));
    radio.send((uint8_t *)&message, sizeof(message_t));
    radio.waitPacketSent();
 
    // Did we receive something from the PC? 
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    while (radio.waitAvailableTimeout(0))
    { 
      // Should be a reply message for us now   
      if (radio.recv(buf, &len))
      {
  //      Serial.print("got reply: ");
  //      Serial.println((char*)buf);
      }
      else
      {
  //      Serial.println("recv failed");
      }
   
   }
#endif //RADIO_ON

    speedReadingLogged = 0;

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

    digitalWrite(green_LED_PIN, LOW);
  } // racing updates
} 

