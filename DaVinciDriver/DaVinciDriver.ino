/*
 DaVinciDriver

 This program reads the values from the sensor, stores them on SD and transmits them
 via the radio
 
 https://github.com/eedala/DaVinciDriver

 */

#include <SPI.h>
#include <SD.h>
#include <IRremote.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*-----( Declare Constants and Pin Numbers )-----*/

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL    55 //75 // mills between entries
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()
#define RADIO_ON         0 // Radio transmission
#define IR_ON            1 // IR transmission
#define SD_CARD_ON       1 // Logging to SD card

// the digital pins that connect to the LEDs
#define redLEDpin 13
#define greenLEDpin 13

// The pins that connect to the sensors
#define speedPin 3              // digital 3, interrupt 0
#define voltPin A1              // analog 1

// The SPI pins for CF and Radio
#define CE_PIN   8
#define CSN_PIN_Radio 10
#define CSN_PIN_CF 9

// IR receiver digital pin
#define IR_receiver 4

// Motor driver interface (PWM on these pins uses timer 0):
#define motorIn1Pin 23
#define motorIn2Pin 22
#define motorPwmPin 21

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

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(CE_PIN, CSN_PIN_Radio);

// Define IR variables
IRrecv irrecv(IR_receiver);           // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'

// Set up BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#if SD_CARD_ON
// the logging file
File logfile;
#endif

// Structure of our message
struct message_t {
  float millis;
  float speed;
  float direction;
  float voltage;
};

message_t message;

unsigned int speedCounter = 0;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  
  while(1);
}

void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  switch(results.value)
  {
  case 0xFFA25D: 
#if ECHO_TO_SERIAL 
    Serial.println(" CH-"); 
#endif  
    break;
  case 0xFF629D: 
#if ECHO_TO_SERIAL 
    Serial.println(" CH");   
#endif  
    break;
  case 0xFFE21D: 
#if ECHO_TO_SERIAL 
    Serial.println(" CH+");  
#endif  
    break;
  case 0xFF22DD: 
#if ECHO_TO_SERIAL 
    Serial.println(" REVERSE");    
#endif  
    break;
  case 0xFF02FD: 
#if ECHO_TO_SERIAL 
    Serial.println(" FORWARD");    
#endif  
    break;
  case 0xFFC23D: 
#if ECHO_TO_SERIAL 
    Serial.println(" PLAY/PAUSE"); 
#endif  
    break;
  case 0xFFE01F: 
#if ECHO_TO_SERIAL 
    Serial.println(" -");    
#endif  
    break;
  case 0xFFA857: 
#if ECHO_TO_SERIAL 
    Serial.println(" +");    
#endif  
    break;
  case 0xFF906F: 
#if ECHO_TO_SERIAL 
    Serial.println(" EQ");   
#endif  
    break;
  case 0xFF6897: 
#if ECHO_TO_SERIAL 
    Serial.println(" 0");    
#endif  
    break;
  case 0xFF9867: 
#if ECHO_TO_SERIAL 
    Serial.println(" 100+"); 
#endif  
    break;
  case 0xFFB04F: 
#if ECHO_TO_SERIAL 
    Serial.println(" 200+"); 
#endif  
    break;
  case 0xFF30CF: 
#if ECHO_TO_SERIAL 
    Serial.println(" 1");    
#endif  
    break;
  case 0xFF18E7: 
#if ECHO_TO_SERIAL 
    Serial.println(" 2");    
#endif  
    break;
  case 0xFF7A85: 
#if ECHO_TO_SERIAL 
    Serial.println(" 3");    
#endif  
    break;
  case 0xFF10EF: 
#if ECHO_TO_SERIAL 
    Serial.println(" 4");    
#endif  
    break;
  case 0xFF38C7: 
#if ECHO_TO_SERIAL 
    Serial.println(" 5");    
#endif  
    break;
  case 0xFF5AA5: 
#if ECHO_TO_SERIAL 
    Serial.println(" 6");    
#endif  
    break;
  case 0xFF42BD: 
#if ECHO_TO_SERIAL 
    Serial.println(" 7");    
#endif  
    break;
  case 0xFF4AB5: 
#if ECHO_TO_SERIAL 
    Serial.println(" 8");    
#endif  
    break;
  case 0xFF52AD: 
#if ECHO_TO_SERIAL 
    Serial.println(" 9");    
#endif  
    break;
  case 0xFFFFFFFF: 
#if ECHO_TO_SERIAL 
    Serial.println(" REPEAT"); 
#endif  
    break;  

  default: 
#if ECHO_TO_SERIAL 
    Serial.print(" other button   :");
    Serial.println(results.value, HEX);
#endif  
  }// End Case
} //END translateIR

//------------------------------------------------------------------------
// Motor on full speed:
void motor_on()
{
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, LOW);
  digitalWrite(motorPwmPin, HIGH);
}

//------------------------------------------------------------------------
// Motor on with specified duty cycle:
// Parameters:
//   int dutycycle  -- PWM setting (0..255)
void motor_on_pwm(int dutycycle)
{
  analogWrite(motorIn1Pin, dutycycle);
  digitalWrite(motorIn2Pin, LOW);
  digitalWrite(motorPwmPin, HIGH);
}

//------------------------------------------------------------------------
// Motor off, coasting:
void motor_off_coast()
{
  digitalWrite(motorIn1Pin, LOW);
  digitalWrite(motorIn2Pin, LOW);
  digitalWrite(motorPwmPin, HIGH);
}

//------------------------------------------------------------------------
// Motor off, hard brake:
void motor_off_brake()
{
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, LOW);
  digitalWrite(motorPwmPin, LOW);
}


void setup(void)
{
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(speedPin, INPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CSN_PIN_CF, OUTPUT);

  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);
  pinMode(motorPwmPin, OUTPUT); 
  

  
  // initialize the serial communication:
  Serial.begin(115200);
  Serial.println();
  

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

#if SD_CARD_ON
  // initialize the SD card
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CSN_PIN_CF)) {
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
  radio.begin();
  radio.openWritingPipe(pipe);
#endif //RADIO_ON

#if IR_ON  
  irrecv.enableIRIn(); // Start the IR receiver
#endif

  // Set PWM frequency.  The motor diver can handle 100 kHz max.
  // Setting the frequency on one pin changes it for all other
  // pins on this timer as well (5, 6, 9, 10, 20, 21, 22, 23)
  analogWriteFrequency(motorIn1Pin, PWM_FREQUENCY);
  
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
  

//  digitalWrite(speedPin, HIGH);
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
  
  digitalWrite(greenLEDpin, HIGH);

#if IR_ON
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
//    Serial.println(results.value, HEX);  UN Comment to see raw values
    translateIR(); 
    irrecv.resume(); // receive the next value
  }  
#endif

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

//  int speedReading = analogRead(speedPin); 
  int speedReading = speedCounter;  //calculate speed based on speedCounter
  speedCounter = 0;                 //reset speedCounter 
//  delay(10);
//  int dirReading = analogRead(dirPin); 
  int dirReading = int(event.orientation.x);
//  delay(10);
  int voltReading = analogRead(voltPin);    
  
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
  message = (message_t){ m, speedReading, dirReading, voltReading };
  radio.write( &message, sizeof(message) );
#endif //RADIO_ON

  digitalWrite(greenLEDpin, LOW);
} 
