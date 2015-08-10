/*
 DaVinciDriver

 This program reads the values from the sensor, stores them on SD and transmits them
 via the radio
 
 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Teensy 3.1
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */

#include <SPI.h>
#include <SD.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>  // debugging for NRF24L01
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*-----( Declare Constants and Pin Numbers )-----*/

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL    55 //75 // mills between entries
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()
#define RADIO_ON         1 // Radio transmission
#define SD_CARD_ON       0 // Logging to SD card

// the digital pins that connect to the LEDs
#define red_LED_PIN 13
#define green_LED_PIN 13

// The pins that connect to the sensors
#define speed_PIN 3              // digital 3, interrupt 0
#define volt_PIN A1              // analog 1

// The SPI pins for CF and Radio
#define CE_Radio_PIN   8
#define CSN_Radio_PIN 10
#define SCK_PIN 14   // SPI bus uses pin 14 instead of pin 13 (used for LED)
#define CSN_CF_PIN 9

// Motor driver interface (PWM on these pins uses timer 0):
#define motor_In1_PIN 23
#define motor_In2_PIN 22
#define motor_PWM_PIN 21

// Motor PWM Frequency
// To calculate you need R and L of the motor.
// First the time constant (Tc) of the motor:
// Tc = L/R
// In 5*Tc the motor current has risen to 98.2%.
// Choose a minimum duty cycle (dc = 1..100).
// Tmin = 100/dc * 5 * Tc
// PWM_FREQUENCY = 1/Tmin
#define PWM_FREQUENCY  400 // in Hz (guess, I haven't measured R and L)


//RTC_DS1307 RTC; // define the Real Time Clock object

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
byte addresses[][6] = {"1Node","2Node"};

// Set up nRF24L01 radio on SPI bus plus pins 8 & 10
RF24 radio(CE_Radio_PIN, CSN_Radio_PIN);

// Set up BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#if SD_CARD_ON
// the logging file
File logfile;
#endif

// Structure of our message
struct message_t {
  uint32_t millis;
  int speed;
  int direction;
  int voltage;
};

message_t message;

unsigned int speedCounter = 0;

void error(const char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(red_LED_PIN, HIGH);
  
  while(1);
}

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
  
  // The default SCK pin is connected to the LED which we use for something else
  // Required for Radio/NRF24L01+ and SD-card
  SPI.setSCK(SCK_PIN);
  
  // initialize the serial communication:
  Serial.begin(115200);
  printf_begin(); // debugging for NRF24L01
  delay(1000); // otherwise first lines may be missing
  Serial.println();
  

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
  radio.printDetails();
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicAck();
 // radio.setPayloadSize(sizeof(message_t));
 // radio.setAddressWidth(5);

  //radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, addresses[1]);
  radio.openWritingPipe(addresses[0]);
  Serial.print("  is + variant: ");
  Serial.println(radio.isPVariant());
  radio.printDetails();
  Serial.println("Radio initialized");
#endif //RADIO_ON

  // Set PWM frequency.  The motor diver can handle 100 kHz max.
  // Setting the frequency on one pin changes it for all other
  // pins on this timer as well (5, 6, 9, 10, 20, 21, 22, 23)
  analogWriteFrequency(motor_In1_PIN, PWM_FREQUENCY);
  
  if (!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
#if SD_CARD_ON
    logfile.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
#endif
    error("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  delay(1000);
  bno.setExtCrystalUse(true);  

#if SD_CARD_ON
  logfile.println("Millis,Delay,Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Direction,VoltageIn");    
#endif
#if ECHO_TO_SERIAL
  Serial.println("Millis,Delay,Time,Ori.x,Ori.y,Ori.z,Acc.x,Acc.y,Acc.z,LAcc.x,LAcc.y,LAcc.z,Grav.x,Grav.y,Grav.z,Mag.x,Mag.y,Mag.z,Gyro.x,Gyro.y,Gyro.z,Euler.x,Euler.y,Euler.z,Quat.w,Quat.x,Quat.y,Quat.z,Speed,Direction,VoltageIn");
#endif //ECHO_TO_SERIAL 

  //attempt to write out the header to the file
//  if (logfile.writeError || !logfile.sync()) {
//    error("write header");
//  }
  

//  digitalWrite(speed_PIN, HIGH);
  attachInterrupt(0, countSpeed, CHANGE); 
   // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
}

void countSpeed(){
  speedCounter++;
}

void loop(void)
{
  // delay for the amount of time we want between readings
  uint32_t delayTime = (LOG_INTERVAL -1) - (millis() % LOG_INTERVAL);
  delay(delayTime);
  
  digitalWrite(green_LED_PIN, HIGH);

  // log milliseconds since starting
  uint32_t m = millis();
#if SD_CARD_ON
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
  logfile.print(delayTime);   // delay time
  logfile.print(", ");    
#endif
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
  Serial.print(delayTime);   // delay time
  Serial.print(", ");    
#endif


  //Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(event.orientation.x);
  logfile.print(", ");    
  logfile.print(event.orientation.y);
  logfile.print(", ");    
  logfile.print(event.orientation.z);
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(event.orientation.x);
  Serial.print(", ");    
  Serial.print(event.orientation.y);
  Serial.print(", ");   
  Serial.print(event.orientation.z);
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(acceleration.x());
  logfile.print(", ");    
  logfile.print(acceleration.y());
  logfile.print(", ");    
  logfile.print(acceleration.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(acceleration.x());
  Serial.print(", ");    
  Serial.print(acceleration.y());
  Serial.print(", ");   
  Serial.print(acceleration.z());
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> lacceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(lacceleration.x());
  logfile.print(", ");    
  logfile.print(lacceleration.y());
  logfile.print(", ");    
  logfile.print(lacceleration.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(lacceleration.x());
  Serial.print(", ");    
  Serial.print(lacceleration.y());
  Serial.print(", ");   
  Serial.print(lacceleration.z());
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(gravity.x());
  logfile.print(", ");    
  logfile.print(gravity.y());
  logfile.print(", ");    
  logfile.print(gravity.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(gravity.x());
  Serial.print(", ");    
  Serial.print(gravity.y());
  Serial.print(", ");   
  Serial.print(gravity.z());
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> magnometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(magnometer.x());
  logfile.print(", ");    
  logfile.print(magnometer.y());
  logfile.print(", ");    
  logfile.print(magnometer.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(magnometer.x());
  Serial.print(", ");    
  Serial.print(magnometer.y());
  Serial.print(", ");   
  Serial.print(magnometer.z());
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(gyroscope.x());
  logfile.print(", ");    
  logfile.print(gyroscope.y());
  logfile.print(", ");    
  logfile.print(gyroscope.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(gyroscope.x());
  Serial.print(", ");    
  Serial.print(gyroscope.y());
  Serial.print(", ");   
  Serial.print(gyroscope.z());
#endif //ECHO_TO_SERIAL

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(euler.x());
  logfile.print(", ");    
  logfile.print(euler.y());
  logfile.print(", ");    
  logfile.print(euler.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(euler.x());
  Serial.print(", ");    
  Serial.print(euler.y());
  Serial.print(", ");   
  Serial.print(euler.z());
#endif //ECHO_TO_SERIAL

  imu::Quaternion quat = bno.getQuat();
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(quat.w());
  logfile.print(", ");    
  logfile.print(quat.x());
  logfile.print(", ");    
  logfile.print(quat.y());
  logfile.print(", ");    
  logfile.print(quat.z());
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(quat.w());
  Serial.print(", ");   
  Serial.print(quat.x());
  Serial.print(", ");    
  Serial.print(quat.y());
  Serial.print(", ");   
  Serial.print(quat.z());
#endif //ECHO_TO_SERIAL

  noInterrupts();
  int speedReading = speedCounter;  //calculate speed based on speedCounter
  speedCounter = 0;                 //reset speedCounter 
  interrupts();
//  delay(10);
  int dirReading = int(event.orientation.x);
//  delay(10);
  int voltReading = analogRead(volt_PIN);    
  
#if SD_CARD_ON
  logfile.print(", ");    
  logfile.print(speedReading);
  logfile.print(", ");    
  logfile.print(dirReading);
  logfile.print(", ");    
  logfile.println(voltReading);
#endif
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(speedReading);
  Serial.print(", ");    
  Serial.print(dirReading);
  Serial.print(", ");    
  Serial.println(voltReading);
#endif //ECHO_TO_SERIAL

#if SD_CARD_ON
  logfile.flush();
#endif

#if RADIO_ON
  // Construct the message we'll send
  message = (message_t){m, speedReading, dirReading, voltReading};
  radio.stopListening();
  Serial.println("Transmitting on radio");
  radio.write(&message, sizeof(message_t), true);
  Serial.println("...transmission finished");
  radio.startListening();
#endif //RADIO_ON

  digitalWrite(green_LED_PIN, LOW);
} 
