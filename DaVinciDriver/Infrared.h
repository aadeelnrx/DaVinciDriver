#pragma once

#include <IRremote.h>

// Decode the data received from the infrared diode
// Parameters:
//   results   -- input to this function, with the received data
//   ir_button -- string into which this function stores the key code
void translateIR(decode_results& results, char *ir_button);

