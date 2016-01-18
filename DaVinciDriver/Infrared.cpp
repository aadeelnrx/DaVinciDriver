#include "Infrared.h"
#include "Logging.h"
#include <string.h>
#include <Arduino.h>

//----------------------------------------------------------------
// Decode and act on IR codes
// Parameters:
//   results   -- input to this function, with the received data
//   ir_button -- string into which this function stores the key code
void translateIR(decode_results& results, char *ir_button)
{
  switch(results.value)
  {
  case 0xFFA25D: 
    LOG_SERIAL_LN(" CH-"); 
    strcpy(ir_button," CH-");
    break;
  case 0xFF629D:
    // Lights on/off 
    LOG_SERIAL_LN(" CH");   
    strcpy(ir_button," CH");
    break;
  case 0xFFE21D: 
    LOG_SERIAL_LN(" CH+");  
    strcpy(ir_button," CH+");
    break;
  case 0xFF22DD: 
    LOG_SERIAL_LN(" REVERSE");    
    strcpy(ir_button," REVERSE");
    break;
  case 0xFF02FD: 
    LOG_SERIAL_LN(" FORWARD");    
    strcpy(ir_button," FORWARD");
    break;
  case 0xFFC23D: 
    // Race Start
    LOG_SERIAL_LN(" PLAY/PAUSE"); 
    strcpy(ir_button," PLAY/PAUSE");
    break;
  case 0xFFE01F: 
    LOG_SERIAL_LN(" -");    
    strcpy(ir_button," -");
    break;
  case 0xFFA857: 
    LOG_SERIAL_LN(" +");    
    strcpy(ir_button," +");
    break;
  case 0xFF906F: 
    // Race Stop
    LOG_SERIAL_LN(" EQ");   
    strcpy(ir_button," EQ");
    break;
  case 0xFF6897: 
    LOG_SERIAL_LN(" 0");    
    strcpy(ir_button," 0");
    break;
  case 0xFF9867: 
    LOG_SERIAL_LN(" 100+"); 
    strcpy(ir_button," 100+");
    break;
  case 0xFFB04F: 
    LOG_SERIAL_LN(" 200+");   
    strcpy(ir_button," 200+");
    break;
  case 0xFF30CF: 
    LOG_SERIAL_LN(" 1");    
    strcpy(ir_button," 1");
    break;
  case 0xFF18E7: 
    LOG_SERIAL_LN(" 2");    
    strcpy(ir_button," 2");
    break;
  case 0xFF7A85: 
    LOG_SERIAL_LN(" 3");    
    strcpy(ir_button," 3");
    break;
  case 0xFF10EF: 
    LOG_SERIAL_LN(" 4");    
    strcpy(ir_button," 4");
    break;
  case 0xFF38C7: 
    LOG_SERIAL_LN(" 5");    
    strcpy(ir_button," 5");
    break;
  case 0xFF5AA5: 
    LOG_SERIAL_LN(" 6");    
    strcpy(ir_button," 6");
    break;
  case 0xFF42BD: 
    LOG_SERIAL_LN(" 7");    
    strcpy(ir_button," 7");
    break;
  case 0xFF4AB5: 
    LOG_SERIAL_LN(" 8");    
    strcpy(ir_button," 8");
    break;
  case 0xFF52AD: 
    LOG_SERIAL_LN(" 9");    
    strcpy(ir_button," 9");
    break;
  case 0xFFFFFFFF: 
    LOG_SERIAL_LN(" REPEAT"); 
    strcpy(ir_button," REPEAT");
    break;  

  default: 
    LOG_SERIAL(" other button   :");
    LOG_SERIAL_LN2(results.value, HEX);
    strcpy(ir_button," other button   :");
  }// End Case
} //END translateIR


