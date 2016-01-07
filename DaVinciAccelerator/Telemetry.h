#pragma once

#include <RH_NRF24.h>

// Telemetry-related functions
// Send messages via NRF24L01+ to computer

//----------------------------------------------
// Structure of the telemetry messages
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

//-----------------------------------------------------------------------------------
class Telemetry : public RH_NRF24
{
public:
  Telemetry(uint8_t ce_pin, uint8_t csn_pin,
            int channel = 1,
            RH_NRF24::DataRate data_rate = RH_NRF24::DataRate2Mbps,
            RH_NRF24::TransmitPower tx_power = RH_NRF24::TransmitPower0dBm);
  
  // Send a message and wait until it's sent          
  void send_msg_wait(message_t& message);
  
  // Send a message, but don't wait until it's sent
  void send_msg(message_t& message);

};


