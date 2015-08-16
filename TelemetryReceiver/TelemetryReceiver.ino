/*
 Receiver for Telemetry data sent by the DaVinciDriver via radio
 Prints the received data on the serial port.
 Every time a message is received, the LED on the Teensy flashes.
 
 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Teensy 3.1
 Tools -> USB Type: Serial
 Tools -> CPU Speed: 72 MHz
 
 */

#include <RF24.h>
#include <SPI.h>

// The digital pins that connect to the LEDs
#define red_LED_PIN 13

// The SPI pins for CF and Radio
#define CE_Radio_PIN   8
#define CSN_Radio_PIN 10
#define SCK_PIN 14   // SPI bus uses pin 14 instead of pin 13 (used for LED)

//-------------------------------------------------------------------------
// Structure of our message
struct message_t
{
  uint32_t millis;
  int speed;
  int direction;
  int voltage;
};

//-------------------------------------------------------------------------
RF24 radio(CE_Radio_PIN, CSN_Radio_PIN);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


//-------------------------------------------------------------------------
void setup()
{
  pinMode(red_LED_PIN, OUTPUT);
  digitalWrite(red_LED_PIN, HIGH);  // indicate Power On

  Serial.begin(115200);
  delay(2000);   // Serial over USB needs some time to start
  digitalWrite(red_LED_PIN, LOW);

  Serial.println(F("Telemetry Receiver R1.0"));
  
  // Setup and configure rf radio
  SPI.setSCK(SCK_PIN); // we use an alternate pin for SCK
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  Serial.println(F("Setting up channel"));
  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(sizeof(message_t));
  radio.setChannel(0x4c);
  radio.setPALevel(RF24_PA_MAX);

  // Open pipes to other node for communication
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)
  Serial.println(F("Setting up pipes"));
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);

  // Start listening
  Serial.println(F("Start listening"));
  radio.startListening();

  // Dump the configuration of the rf unit for debugging
  //radio.printDetails(); 
  Serial.println(F("Ready."));
  Serial.println(F("millis, speed, direction, trackVoltage"));
}


//---------------------------------------------------------------------------
void loop()
{
  // Check if there is data ready
  if (radio.available())
  {
    digitalWrite(red_LED_PIN, HIGH);  // indicate reception of a message
    
    // Dump the payloads until we've gotten everything
    message_t data;
    bool done = false;
    
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read(&data, sizeof(message_t));
      
      // Print it
      //Serial.println(F("Got payload:"));
      Serial.print(data.millis);
      Serial.print(F(", "));
      Serial.print(data.speed);
      Serial.print(F(", "));
      Serial.print(data.direction);
      Serial.print(F(", "));
      Serial.println(data.voltage);
    }
//Commented out, this can be used later when the telemetry station (PC)
//wants to send commands to the car      
//    // Delay just a little bit to let the other unit
//    // make the transition to receiver
//    delay(20);  
//
//    // First, stop listening so we can talk
//    radio.stopListening();
//    
//    // Send the final one back.
//    radio.write( &got_time, sizeof(unsigned long) );
//    
//    // Now, resume listening so we catch the next packets.
//    radio.startListening();
 
    digitalWrite(red_LED_PIN, LOW);
  }
}
