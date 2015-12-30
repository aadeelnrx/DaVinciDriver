/*
 DaVinciDriver

 This program reads the values from the sensor, stores them on SD and transmits them
 via the radio
 
 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Teensy 3.1/3.2
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */

//#include <SD.h>
#include <SdFat.h>
#include <IRremote.h>
#include <Metro.h>
#include <PID_v1.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*-----( Declare Constants and Pin Numbers )-----*/

#define LOG_INTERVAL     10 // mills between logging
#define PID_INTERVAL     10 // mills between PID updates
#define ECHO_TO_SERIAL   1  // echo data to serial port
#define WAIT_TO_START    0  // Wait for serial input in setup()
#define RADIO_ON         1  // Radio transmission
#define IR_ON            0  // IR transmission
#define SD_CARD_ON       0  // Logging to SD card
#define TEST_LOGGING_ON  0  //
#define BNO055_ON        1  // Sensor
#define BNO055_TEST_ON   0  // Extended Sensor reading only valid with BNO055_ON

#define STOP_AFTER_SECONDS 130

// the digital pins that connect to the LEDs
#define red_LED_PIN 13
#define green_LED_PIN 13

// The pins that connect to the sensors
#define speed_PIN 3              // digital 3, interrupt 0
#define track_volt_PIN A1        // analog 1
#define engine_volt_PIN A2       // analog 2

// The SPI pins for CF and Radio
#define CE_Radio_PIN   8
#define CSN_Radio_PIN 10
#define SCK_PIN 14   // SPI bus uses pin 14 instead of pin 13 (used for LED)
#define CSN_CF_PIN 9

// IR receiver digital pin
#define IR_receiver 4

// Motor driver interface (PWM on these pins uses timer 0):
#define motor_In1_PIN 23
#define motor_In2_PIN 22
#define motor_PWM_PIN 21
#define motor_STBY_PIN 20

// Motor PWM Frequency
// To calculate you need R and L of the motor.
// First the time constant (Tc) of the motor:
// Tc = L/R
// In 5*Tc the motor current has risen to 98.2%.
// Choose a minimum duty cycle (dc = 1..100).
// Tmin = 100/dc * 5 * Tc
// PWM_FREQUENCY = 1/Tmin
#define PWM_FREQUENCY  400 // in Hz (guess, I haven't measured R and L)

#define MOTOR_TARGET_SPEED 25
#define MOTOR_P 0.8
#define MOTOR_I 2.5
#define MOTOR_D 0.01


/*-------------------( Global Variables )-----------------------------------*/
// When to stop the car so that it doesn't drive endlessly during test:
uint32_t stoptime;

// Timers that control the update rates of various things in the main loop:
// Units are milliseconds.
Metro pid_timer = Metro(PID_INTERVAL);
Metro measurement_timer = Metro(LOG_INTERVAL);

// PID Controller
double PID_setpoint, PID_input, PID_output;
PID PIDcontroller(&PID_input, &PID_output, &PID_setpoint, MOTOR_P, MOTOR_I, MOTOR_D, DIRECT);

//RTC_DS1307 RTC; // define the Real Time Clock object

// Set up nRF24L01 radio on SPI bus
RH_NRF24 radio(CE_Radio_PIN, CSN_Radio_PIN);

// Acceleration and orientation
sensors_event_t accel_orient;

// Infrared remote control receiver
#if IR_ON
IRrecv irrecv(IR_receiver);
decode_results results;
#endif

// BNO055 sensor and the data we read from it:
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> acceleration;
imu::Vector<3> lacceleration;
imu::Vector<3> gravity;
imu::Vector<3> magnometer;
imu::Vector<3> gyroscope;
imu::Vector<3> euler;
imu::Quaternion quat;

int trackVoltReading;
int dirOverflow = 0;
uint32_t dirReadingOld;


#if SD_CARD_ON
File logfile;
// File system object.
SdFat SD;
// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;
#endif

//----------------------------------------------
// Structure of our telemetry messages
// !!! Max. length is 28 bytes !!!
// sizeof(uint32_t) = 4 bytes --> 7x uint32_t
enum MSG_TYPES
{
  MSG_MEASUREMENT = 0,
  MSG_TEXT,
  MSG_TRACK,
};

struct msg_track_t
{
  uint32_t seq_no;
  uint16_t seq;
  uint16_t position;
  uint16_t length;
  uint16_t direction;
  uint16_t segment_type;
  uint16_t dirChange;
  uint16_t straight_cnt;
  uint16_t lap_found;
  uint16_t finish_position;
};

struct measurement_t
{
  uint32_t seq_no;
  uint32_t millis;
  uint32_t speed;
  uint32_t direction;
  uint32_t voltage;
};

struct message_t
{
  MSG_TYPES msg_type;  // 0 = measurement, 1 = text
  union
  {
    measurement_t measurement;
    msg_track_t track;
    char text[20];
  } msg;
};
message_t message;
measurement_t measurement;
char text[26];
//-----------------------------------------------

// Sequence number for radio messages to detect message loss
uint32_t seq_no;

// Wheel encoder, updated by an interrupt routine
volatile uint32_t speedCounter = 0;
uint32_t speedReading, speedReadingLogged, distance = 0;
uint32_t sp, sp1, sp2, sp3, sp4;  // last 4 values of speedReading

// IR button/action
char irButton[30];

//-----------------------------------------------
// Records for lap detection and lap segment description
// Different segment types
enum SEGMENT_TYPES {
  STRAIGHT = 0,
  KURVE_1 = 1,  // tightest
  KURVE_2 = 2,  // normal
  KURVE_3 = 3,  // wide
  KURVE_4 = 4,  // widest
  SLIDE = 5,    // slide
};

// Lap segment storage records
struct lap_segment{
  uint32_t position;
  uint32_t length;
  uint32_t direction;
  SEGMENT_TYPES segment_type;
};

// Lap segment detection help records
struct lap_segment_hist{
  uint32_t position;
  uint32_t direction;
  SEGMENT_TYPES segment_type;
};

lap_segment_hist lap_hist[10];
lap_segment lap[1000];
uint32_t seq_cnt = 0;
bool measuring = false;

// Lap recognition based on straights help records
struct straight_type{
  uint32_t position;
  uint32_t length;
  uint32_t direction;  
};

straight_type straight[100];
uint32_t straight_cnt = 0;
bool lap_found = false;
uint32_t finish_position = 0;

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
void error(const char *str)
{
  // Better switch off motor:
  motor_off_brake();

  Serial.print("error: ");
  LOG_SERIAL_LN(str);
  
  // red LED indicates error
  digitalWrite(red_LED_PIN, HIGH);
  
  while(1);
}

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
// Motor on full speed:
void motor_on()
{
  digitalWrite(motor_In1_PIN, HIGH);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor on with specified duty cycle:
// Parameters:
//   int dutycycle  -- PWM setting (0..255)
void motor_on_pwm(int dutycycle)
{
  analogWrite(motor_In1_PIN, dutycycle);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor off, coasting:
void motor_off_coast()
{
  digitalWrite(motor_In1_PIN, LOW);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, HIGH);
}

//------------------------------------------------------------------------
// Motor off, hard brake:
void motor_off_brake()
{
  digitalWrite(motor_In1_PIN, HIGH);
  digitalWrite(motor_In2_PIN, LOW);
  digitalWrite(motor_PWM_PIN, LOW);
}

//------------------------------------------------------------------------
// Interrupt service routine for the wheel encoder:
void countSpeed(){
  speedCounter++;
}

//============================================================================
//============================================================================
void setup(void)
{
  pinMode(red_LED_PIN, OUTPUT);
  pinMode(green_LED_PIN, OUTPUT);
  pinMode(speed_PIN, INPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CSN_CF_PIN, OUTPUT);
  pinMode(CSN_Radio_PIN, OUTPUT);

  pinMode(CE_Radio_PIN, OUTPUT);
  
  pinMode(motor_In1_PIN, OUTPUT);
  pinMode(motor_In2_PIN, OUTPUT);
  pinMode(motor_PWM_PIN, OUTPUT); 
  pinMode(motor_STBY_PIN, OUTPUT); 
  digitalWrite(motor_STBY_PIN, HIGH);
  // Set PWM frequency.  The motor diver can handle 100 kHz max.
  // Setting the frequency on one pin changes it for all other
  // pins on this timer as well (5, 6, 9, 10, 20, 21, 22, 23)
  analogWriteFrequency(motor_In1_PIN, PWM_FREQUENCY);
  
  // PID
  PID_input = 0.0;
  PID_setpoint = MOTOR_TARGET_SPEED * PID_INTERVAL / 10 ;
  PIDcontroller.SetMode(AUTOMATIC);
  PIDcontroller.SetSampleTime(PID_INTERVAL);
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
  logfile.println("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,dirChange");    
#endif
#if ECHO_TO_SERIAL
  LOG_SERIAL_LN("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,dirChange");
#endif //ECHO_TO_SERIAL 

  //attempt to write out the header to the file
//  if (logfile.writeError || !logfile.sync()) {
//    error("write header");
//  }
  

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
}


//====================================================================
//====================================================================
void loop(void)
{  
  // Stop motor after some time
  if (millis() >= stoptime)
  {
    digitalWrite(green_LED_PIN, HIGH);
    motor_off_brake();
    while(1);
  }

  // Stop motor at start/finish straight after lap detection has finished
  if ((finish_position > 0) && (distance >= (2*finish_position)))
  {
    digitalWrite(green_LED_PIN, HIGH);
    motor_off_brake();
    while(1);
  }
    
  if (measurement_timer.check())
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
    sp = speedReading + sp1 + sp2 + sp3 + sp4;

    // Motor also as early as possible to ensure constant update intervals
    PID_input = (float)sp;
//    if ((trackVoltReading = 0)&&(speedCounter = 0))
//    {
//      PID_output = 0;
//    }
    PIDcontroller.Compute();
    motor_on_pwm(PID_output);

    int32_t dirReading;
    float dirChange = 0.0;
#if BNO055_ON
    // Read the acceleration and orientation:
    bno.getEvent(&accel_orient);

    dirReading = uint32_t(accel_orient.orientation.x);

    // Start measuring after BNO055 starts giving real values
    if ((dirReading > 0)&&(measuring == false))
    {
      measuring = true;
      lap[seq_cnt].position = 0;
      lap[seq_cnt].direction = dirReading;
      lap[seq_cnt].segment_type = STRAIGHT;
      seq_cnt++;
      for (int i=0; i < 9; i++)
      {
        lap_hist[i+1].position = 0;
        lap_hist[i+1].direction = dirReading;
        lap_hist[i+1].segment_type = STRAIGHT;          
      }        
    }
    // Measuring whenever the direction changes
    if (measuring == true)
    {
      if ( (dirReading - lap[seq_cnt-1].direction) > 1)
      {
        // rotate history of last 10 measurements (FIFO)
        for (int i=8; i >= 0 ; i--)
        {
          lap_hist[i+1].position = lap_hist[i].position;
          lap_hist[i+1].direction = lap_hist[i].direction;
          lap_hist[i+1].segment_type = lap_hist[i].segment_type;          
        }        
        // store current measurement
        lap_hist[0].position = distance;
        lap_hist[0].direction = dirReading;
        // determine the relative direction change during the last 10 measurements (to smoothen the measurement result curve)
        dirChange = (((float)abs(dirReading - lap_hist[9].direction))/(float)(distance -lap_hist[9].position));
        if (dirChange < 0.05)
        { 
          lap_hist[0].segment_type = STRAIGHT;
        }else if (dirChange < 0.087)
        { 
          lap_hist[0].segment_type = KURVE_4;
        }else if (dirChange < 0.125)
        { 
          lap_hist[0].segment_type = KURVE_3;
        }else if (dirChange < 0.25)
        { 
          lap_hist[0].segment_type = KURVE_2;
        }else if (dirChange < 0.5)
        { 
          lap_hist[0].segment_type = KURVE_1;
        }else
        { 
          lap_hist[0].segment_type = SLIDE;
        }   

        // Create new lap_segment in case direction has changed in the last 3 measurements (to avoid short peeks/drops)
        if ((lap_hist[0].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[1].segment_type != lap[seq_cnt-1].segment_type)&&(lap_hist[2].segment_type != lap[seq_cnt-1].segment_type))
        {
          lap[seq_cnt].position = lap_hist[2].position;
          lap[seq_cnt-1].length = lap[seq_cnt].position - lap[seq_cnt-1].position;
          lap[seq_cnt].direction = lap_hist[2].direction;
          lap[seq_cnt].segment_type = lap_hist[2].segment_type;
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
                lap_found = true; 
                finish_position = straight[straight_cnt].position - straight[i].position;
              }
            }
            straight_cnt++;
          }        
        }
      }
    }
    if ((dirReadingOld > 350)&& (dirReading <10))
    {
      dirOverflow += 360;
    }
    if ((dirReadingOld < 10)&& (dirReading >350))
    {
      dirOverflow -= 360;
    }
    dirReadingOld = dirReading;
    dirReading += dirOverflow;  
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
    LOG(PID_output)
    LOG(MOTOR_P)
    LOG(MOTOR_I)
    LOG(MOTOR_D)

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

    speedReadingLogged = 0;
    
#if RADIO_ON
    seq_no++;
    // If we check here that the packet was sent from the previous loop,
    // we save time because we could already do lots of other things in the
    // meantime
  //  if (seq_no > 1) radio.waitPacketSent();
 
    // Construct the message we'll send
    //message.msg_type = MSG_MEASUREMENT;
    //message.msg.measurement = (measurement_t){seq_no, msec_since_start, speedReading, dirReading, sp /*trackVoltReading*/};
  
    if (seq_cnt > 1)
    {
      message.msg_type = MSG_TRACK;
      message.msg.track = (msg_track_t){seq_no,
                                              (uint16_t)(seq_cnt-1),
                                              (uint16_t)lap[seq_cnt-1].position,
                                              (uint16_t)lap[seq_cnt-2].length,
                                              (uint16_t)lap[seq_cnt-1].direction,
                                              (uint16_t)lap[seq_cnt-1].segment_type,
                                              (uint16_t)(100 * dirChange),
                                              (uint16_t)straight_cnt,
                                              (uint16_t)lap_found,
                                              (uint16_t)finish_position};
  
      LOG_SERIAL_LN(F("Transmitting on radio"));
      radio.send((uint8_t *)&message, sizeof(message_t));
      radio.waitPacketSent();
    } 
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

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

    digitalWrite(green_LED_PIN, LOW);
  } // measurement updates
} 

