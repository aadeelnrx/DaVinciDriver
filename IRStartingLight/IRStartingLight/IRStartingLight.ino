/*
IR Starting Light for DaVinciDriver

An IR LED must be connected to Arduino PWM pin 3.

 https://github.com/eedala/DaVinciDriver

 Tools -> Board: Arduino Nano
 Tools -> USB Type: Serial
 Tools -> CPU Speed: ?? MHz


 */

#include <IRremote.h>

// The pin that connects to the Starting Lights Green Light
#define green_light_PIN 2              // digital 2, interrupt 0

IRsend irsend;

// Interrupt routine for sending the start signal
void sendStart(){
  irsend.sendNEC(0xFFC23D, 24);
  Serial.println("START");
}

void setup() {
  pinMode(green_light_PIN, INPUT);
 
  // initialize the serial communication:
  Serial.begin(115200);
  delay(1000); // otherwise first lines may be missing
  Serial.println();

  //  digitalWrite(green_light_PIN, HIGH);
  attachInterrupt(0, sendStart, CHANGE); 
}

void loop() {

}
