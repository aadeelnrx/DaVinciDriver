/*
 DaVinciDriver

 This program reads the values from the sensor, stores them on SD and transmits them
 via the radio
 
 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Teensy 3.1
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */

#include <SD.h>
#include <IRremote.h>
#include <PID_v1.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*-----( Declare Constants and Pin Numbers )-----*/

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL    100 //75 // mills between entries
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()
#define RADIO_ON         0 // Radio transmission
#define IR_ON            0 // IR transmission
#define SD_CARD_ON       0 // Logging to SD card

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
#define motor_stby_PIN 20

// Motor PWM Frequency
// To calculate you need R and L of the motor.
// First the time constant (Tc) of the motor:
// Tc = L/R
// In 5*Tc the motor current has risen to 98.2%.
// Choose a minimum duty cycle (dc = 1..100).
// Tmin = 100/dc * 5 * Tc
// PWM_FREQUENCY = 1/Tmin
#define PWM_FREQUENCY  400 // in Hz (guess, I haven't measured R and L)

/*-------------------( Global Variables )-----------------------------------*/
uint32_t starttime;

// PID Controller
double PID_setpoint, PID_input, PID_output;
PID PIDcontroller(&PID_input, &PID_output, &PID_setpoint, 1.0, 10, 0, DIRECT);

//RTC_DS1307 RTC; // define the Real Time Clock object

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};

// Set up nRF24L01 radio on SPI bus
RF24 radio(CE_Radio_PIN, CSN_Radio_PIN);

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


#if SD_CARD_ON
File logfile;
#endif

// Structure of our telemetry message
struct message_t {
  uint32_t millis;
  int speed;
  int direction;
  int voltage;
};
message_t message;

// Wheel encoder, updated by an interrupt routine
volatile unsigned int speedCounter = 0;

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
#define LOG_SERIAL(x) Serial.print(x);
#else
#define LOG_SERIAL_LN(x)
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
    break;
  case 0xFF629D:
    // Lights on/off 
    LOG_SERIAL_LN(" CH");   
    break;
  case 0xFFE21D: 
    LOG_SERIAL_LN(" CH+");  
    break;
  case 0xFF22DD: 
    LOG_SERIAL_LN(" REVERSE");    
    break;
  case 0xFF02FD: 
    LOG_SERIAL_LN(" FORWARD");    
    break;
  case 0xFFC23D: 
    // Race Start
    LOG_SERIAL_LN(" PLAY/PAUSE"); 
    break;
  case 0xFFE01F: 
    LOG_SERIAL_LN(" -");    
    break;
  case 0xFFA857: 
    LOG_SERIAL_LN(" +");    
    break;
  case 0xFF906F: 
    // Race Stop
    LOG_SERIAL_LN(" EQ");   
    break;
  case 0xFF6897: 
    LOG_SERIAL_LN(" 0");    
    break;
  case 0xFF9867: 
    LOG_SERIAL_LN(" 100+"); 
    break;
  case 0xFFB04F: 
    LOG_SERIAL_LN(" 200+");   
    break;
  case 0xFF30CF: 
    LOG_SERIAL_LN(" 1");    
    break;
  case 0xFF18E7: 
    LOG_SERIAL_LN(" 2");    
    break;
  case 0xFF7A85: 
    LOG_SERIAL_LN(" 3");    
    break;
  case 0xFF10EF: 
    LOG_SERIAL_LN(" 4");    
    break;
  case 0xFF38C7: 
    LOG_SERIAL_LN(" 5");    
    break;
  case 0xFF5AA5: 
    LOG_SERIAL_LN(" 6");    
    break;
  case 0xFF42BD: 
    LOG_SERIAL_LN(" 7");    
    break;
  case 0xFF4AB5: 
    LOG_SERIAL_LN(" 8");    
    break;
  case 0xFF52AD: 
    LOG_SERIAL_LN(" 9");    
    break;
  case 0xFFFFFFFF: 
    LOG_SERIAL_LN(" REPEAT"); 
    break;  

  default: 
    LOG_SERIAL(" other button   :");
    LOG_SERIAL_LN(results.value, HEX);
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

  pinMode(CE_Radio_PIN, OUTPUT);
  
  pinMode(motor_In1_PIN, OUTPUT);
  pinMode(motor_In2_PIN, OUTPUT);
  pinMode(motor_PWM_PIN, OUTPUT); 
  pinMode(motor_stby_PIN, OUTPUT); 
  
  digitalWrite(motor_stby_PIN, HIGH);
  
  // The default SCK pin is connected to the LED which we use for something else
  // Required for Radio/NRF24L01+ and SD-card
  SPI.setSCK(SCK_PIN);
  
  digitalWrite(green_LED_PIN, HIGH);

  // initialize the serial communication:
  Serial.begin(115200);
  delay(5000); // otherwise first lines may be missing
  Serial.println("DaVinciDriver");
  

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

#if SD_CARD_ON
  // initialize the SD card
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CSN_CF_PIN)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
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

  Serial.print("Logging to: ");
  Serial.println(filename);
#endif

#if RADIO_ON
  // Initialize all radio related modules
  Serial.println("Initializing radio...");
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  Serial.println(F("Setting up channel"));
  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(sizeof(message_t));
  radio.setChannel(0x4c);
  radio.setPALevel(RF24_PA_MAX);

  // Open pipes to other node for communication
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

  // Start listening
  radio.startListening();

  // Dump the configuration of the rf unit for debugging
  //
  //radio.printDetails();
  Serial.println(F("Radio ready."));
#endif //RADIO_ON

#if IR_ON  
  irrecv.enableIRIn(); // Start the IR receiver
#endif

  // Set PWM frequency.  The motor diver can handle 100 kHz max.
  // Setting the frequency on one pin changes it for all other
  // pins on this timer as well (5, 6, 9, 10, 20, 21, 22, 23)
  analogWriteFrequency(motor_In1_PIN, PWM_FREQUENCY);
Serial.println("Before bno begin");
  if (!bno.begin())
  {
Serial.println("bno error");
    //There was a problem detecting the BNO055 ... check your connections
    LOG_SDCARD_LN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!")
    error("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
Serial.println("After bno begin");

  delay(1000);
  bno.setExtCrystalUse(true);  

#if SD_CARD_ON
  logfile.println("Millis,Delay,Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Direction,VoltageIn,VoltageEngine");    
#endif
#if ECHO_TO_SERIAL
  LOG_SERIAL_LN("Millis,Delay,Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Direction,VoltageIn,VoltageEngine");
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

  // Blink to indicate it's going to start:
  for (int i=0; i < 4; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(500);
    digitalWrite(green_LED_PIN, LOW);
    delay(500);
  }
  // last second: fast blinking
  for (int i=0; i < 10; i++)
  {
    digitalWrite(green_LED_PIN, HIGH);
    delay(50);
    digitalWrite(green_LED_PIN, LOW);
    delay(50);
  }
  
  // PID
  PID_input = 0.0;
  PID_setpoint = 40;
  PIDcontroller.SetMode(AUTOMATIC);
  
}


//====================================================================
//====================================================================
void loop(void)
{
  // Stop motor after some time
  if ((millis() - starttime) > 600 * 1000)
  {
    motor_off_brake();
    while(1);
  }
  
  // Delay for the amount of time we want between readings
  uint32_t delayTime = (LOG_INTERVAL -1) - (millis() % LOG_INTERVAL);
  delay(delayTime);

  // Log milliseconds since starting
  uint32_t msec_since_start = millis();
  
  digitalWrite(green_LED_PIN, HIGH);

  // Take all measurement values right at the beginning of the
  // main loop to ensure taking them after the same interval
  // all the time
  noInterrupts();
  int speedReading = speedCounter;  //calculate speed based on speedCounter
  speedCounter = 0;                 //reset speedCounter 
  interrupts();
  
  int trackVoltReading = analogRead(track_volt_PIN);    
  int engineVoltReading = analogRead(engine_volt_PIN);    

  // Read the acceleration and orientation:
  bno.getEvent(&accel_orient);

  int dirReading = int(accel_orient.orientation.x);

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

  // Motor also as early as possible to ensure constant update intervals
  PID_input = (float)speedReading;
  PIDcontroller.Compute();
  motor_on_pwm(PID_output);
  
  
#if IR_ON
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
//    LOG_SERIAL_LN(results.value, HEX);  UN Comment to see raw values
    translateIR(); 
    irrecv.resume(); // receive the next value
  }  
#endif


  // Log measurements to serial and/or flash:
  LOG(msec_since_start)
  LOG(delayTime)
  
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
  
  LOG(speedReading)
  LOG(dirReading)
  LOG(trackVoltReading)    
  LOG(engineVoltReading)
  LOG_LN()
  
#if SD_CARD_ON
  logfile.flush();
#endif

#if RADIO_ON
  // Construct the message we'll send
  message = (message_t){msec_since_start, speedReading, dirReading, trackVoltReading};
  
  // We have to stop/start listening in order to receive ACK packets
  radio.stopListening();

  //LOG_SERIAL_LN(F("Transmitting on radio"));
  radio.write(&message, sizeof(message_t) );
  //bool ok = radio.write(&message, sizeof(message_t) );
  //if (ok)  LOG_SERIAL_LN(F("ok..."));
  //else     LOG_SERIAL_LN(F("failed.\n"));
  radio.startListening();
#endif //RADIO_ON

  digitalWrite(green_LED_PIN, LOW);
} 

