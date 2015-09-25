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
bool stopping=false;
bool driving=false;

// Interrupt routine for sending the start signal
void sendStart(){
  if (digitalRead(green_light_PIN)== LOW) {
    irsend.sendNEC(0xFFC23D, 32);
    Serial.println("START");
    stopping = false;
    driving = true;
  }else{
    irsend.sendNEC(0xFF906F, 32);
    Serial.println("STOP");
    stopping = true;
  }
}

void setup() {
  pinMode(green_light_PIN, INPUT);
 
  // initialize the serial communication:
  Serial.begin(9600);
  delay(1000); // otherwise first lines may be missing
  Serial.println();

  //  digitalWrite(green_light_PIN, HIGH);
  attachInterrupt(0, sendStart, CHANGE); 
}

void loop() {
  if (driving == false){
    irsend.sendNEC(0xFF629D, 32);
    Serial.println("Lights ON/OFF");    
  }
  if (stopping == true){
  }else{
    irsend.sendNEC(0xFF906F, 32);
    Serial.println("STOPPING");    
    irsend.sendNEC(0xFF629D, 32);
    Serial.println("Lights ON/OFF");    
  }
  delay(2000); // otherwise first lines may be missing

}
