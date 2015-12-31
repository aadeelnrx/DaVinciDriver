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
    if (seq_cnt > 1)
    {
      message.msg_type = MSG_TRACK;
      message.msg.track = (msg_track_t){seq_no++,
                                        (uint16_t)(seq_cnt-1),
                                        (uint16_t)lap[seq_cnt-1].position,
                                        (uint16_t)lap[seq_cnt-2].length,
                                        (uint16_t)lap[seq_cnt-1].direction,
                                        (uint16_t)lap[seq_cnt-1].segment_type,
                                        (uint16_t)(100 * dirChange),
                                        (uint16_t)straight_cnt,
                                        (uint16_t)lap_found,
                                        (uint16_t)finish_position};
      telemetry.send_msg_wait(message);
    } 
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

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

    digitalWrite(green_LED_PIN, LOW);
  } // measurement updates
} 

