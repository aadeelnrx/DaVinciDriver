/*
 DaVinciAccelerator

 This program tests the acceleration of the car
 
 Tools -> Board: Teensy 3.1/3.2
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */


#include "PinsParameters.h"
#include "Logging.h"
#include "Motor.h"
#include "Telemetry.h"
#include <SdFat.h>
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

// PID Controller
double PID_setpoint, PID_input, PID_output;
PID PIDcontroller(&PID_input, &PID_output, &PID_setpoint, MOTOR_P, MOTOR_I, MOTOR_D, DIRECT);

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

uint16_t trackVoltReading, engineVoltReading;
uint16_t dirOverflow = 0;
uint16_t dirReadingOld;


#if SD_CARD_ON
File logfile;
// File system object.
SdFat SD;
// Test with reduced SPI speed for breadboards.
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;
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
uint16_t speedReading, distance = 0;
uint16_t sp, sp1, sp2, sp3, sp4;  // last 4 values of speedReading


// Measurement struct
struct accel_t
{
  uint32_t millis;
  uint16_t ex_time;
  uint16_t distance;
  uint16_t speed;
  float    acc_x;  // 32 bit
  float    acc_y;  // 32 bit
  uint16_t trackVolt;
};

uint8_t bnostatus, bnoselftest, bnosyserr;

accel_t accel_data[1000];
uint16_t accel_index = 0;

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
  
  motor_init();
  
  // PID
  PID_input = 0.0;
  PID_setpoint = MOTOR_TARGET_SPEED;
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
  for (int i=0; i < 9; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(500);
    digitalWrite(green_LED_PIN, LOW);
    delay(500);
  }
  LOG_SERIAL_LN("DaVinciDriver");


  // BNO055 IMU initialisation
  // Done here (4 seconds after serial initialisation) so that
  // we can print to serial here.
  LOG_SERIAL_LN("Before bno begin");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    Serial.println("bno error");
    //There was a problem detecting the BNO055 ... check your connections
    error("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  LOG_SERIAL_LN("After bno begin");

  // last second: fast blinking
  // The BNO055 needs one second to initialise anyway.
  for (int i=0; i < 10; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(50);
    digitalWrite(green_LED_PIN, LOW);
    delay(50);
  }
  

  // This has to be done at least one second after bno.begin():
  bno.setExtCrystalUse(true);  


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

  logfile.println("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,dirChange");    
#endif

#if ECHO_TO_SERIAL
  LOG_SERIAL_LN("Millis,Ex Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Distance,Direction,VoltageIn,VoltageEngine,sp1,sp2,sp3,sp4,SR,PID_In,PID_Out,P,I,D,IR,Seq,Pos,l,dir,ST,dirChange");
#endif //ECHO_TO_SERIAL 


  attachInterrupt(speed_PIN, countSpeed, CHANGE); 

  // Stop the car after some time
  stoptime = millis() + STOP_AFTER_SECONDS * 1000;

#define TEST_2_PWM_80
#ifdef TEST_2_PWM_80
  motor_on_pwm(80);
#endif
#ifdef TEST_3_PWM_128
  motor_on_pwm(128);
#endif
#ifdef TEST_4_PWM_180
  motor_on_pwm(180);
#endif
#ifdef TEST_5_PWM_255
  motor_on_pwm(255);
#endif
#ifdef TEST_6_ON
  motor_on();
#endif

  // Reset metro timers so we start at 0
  pid_timer.reset();
  measurement_timer.reset();
}


//====================================================================
//====================================================================
void loop(void)
{  

  // Stop motor after some distance, then print measurement data
  if (distance >= 2000)
  {
    motor_off_coast();

    // Wait until motor stops
    if (speedReading == 0)
    {
      // Print all measurements:
      while(1)
      {
        Serial.println("-------------------------------------------------");
        Serial.println("Millis, ExTime, Distance, Speed, Acc_x, Acc_y, TrackVolt");
        for (int i = 0;  i < accel_index;  i++)
        {
          Serial.print(accel_data[i].millis);
          Serial.print(", ");
          Serial.print(accel_data[i].ex_time);
          Serial.print(", ");
          Serial.print(accel_data[i].distance);
          Serial.print(", ");
          Serial.print(accel_data[i].speed);
          Serial.print(", ");
          Serial.print(accel_data[i].acc_x);
          Serial.print(", ");
          Serial.print(accel_data[i].acc_y);
          Serial.print(", ");
          Serial.print(accel_data[i].trackVolt);
          Serial.println();
        }
        delay(10000);
      }
    }
  }
    
  if (measurement_timer.check())
  {
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();
  
    // Take all measurement values right at the beginning of the
    // main loop to ensure taking them after the same interval
    // all the time
    trackVoltReading = analogRead(track_volt_PIN);    
    engineVoltReading = analogRead(engine_volt_PIN);    

    noInterrupts();
    speedReading = speedCounter;  //calculate speed based on speedCounter
    speedCounter = 0;             //reset speedCounter 
    interrupts();

    distance += speedReading;
    sp = speedReading + sp1 + sp2 + sp3 + sp4;

#ifdef TEST_1_PID
    // Motor also as early as possible to ensure constant update intervals
    PID_input = (float)sp;
    PIDcontroller.Compute();
    motor_on_pwm(PID_output);
#endif

    acceleration  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    accel_data[accel_index].millis = msec_since_start;
    accel_data[accel_index].distance = distance;
    accel_data[accel_index].speed = speedReading;
    accel_data[accel_index].acc_x = (float)acceleration.x();
    accel_data[accel_index].acc_y = (float)acceleration.y();
    accel_data[accel_index].trackVolt = trackVoltReading;
    accel_data[accel_index].ex_time = (uint16_t)(millis() - msec_since_start);
    accel_index++;
    // Prevent writing past the array boundary:
    if (accel_index > 3000) accel_index = 3000;

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

  } // measurement updates
} 

