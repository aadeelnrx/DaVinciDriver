// Telemetry-related functions

#include "Telemetry.h"
#include "Logging.h"

Telemetry::Telemetry(uint8_t ce_pin, uint8_t csn_pin, int channel,
  RH_NRF24::DataRate data_rate, RH_NRF24::TransmitPower tx_power)
  : RH_NRF24(ce_pin, csn_pin)
{
  // Initialize all radio related modules
  LOG_SERIAL_LN("Initializing radio...");
 
  if (! init())
    LOG_SERIAL_LN("init failed");
  if (! setChannel(channel))
    LOG_SERIAL_LN("setChannel failed");
  if (! setRF(data_rate, tx_power))
    LOG_SERIAL_LN("setRF failed");   
   
  LOG_SERIAL_LN(F("Radio ready."));
}

//-------------------------------------------------------------------------
// Send a message, but don't wait until it's sent
void Telemetry::send_msg(message_t& message)
{
  LOG_SERIAL_LN(F("Transmitting on radio"));
  send((uint8_t *)&message, sizeof(message_t));
}

//-------------------------------------------------------------------------
// Send a message and wait until it's sent
void Telemetry::send_msg_wait(message_t& message)
{
  send_msg(message);
  waitPacketSent();
}



