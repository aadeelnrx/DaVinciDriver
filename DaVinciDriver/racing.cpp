#include "datatypes.h"
#include "racing.h"
    

//=================== WAIT FOR RACE START ====================================================
void wait_for_start_loop()
{
  while (1)
  {
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    if (millis() > (startTime - PID_INTERVAL))
    {
      // Reset metro timers so we start at 0
      racing_timer.reset(); 

      // set current distance from start/finish line
      LOG(msec_since_start)
      LOG("RACE Preparation")
      LOG(speedCounter)
      LOG(distance)

      // increase distance by distance the call rolled after stopping e.g. speedcounter
      distance += speedCounter;

      LOG(distance)
      LOG(finish_position)

      // calculate position from where starting
      // should not be below 0
      // still needs fixing
      distance -= (2*finish_position);

      LOG(distance)
      LOG_LN()    
#if SD_CARD_ON
      logfile.flush();
#endif

      // check if distance was below 0
      // still needs fixing
      if (distance > 100000)
      {
        distance = 0;
      }

      // reset all PID inputs and outputs
      speedCounter = 0;
      PID_output = 0;
      // set race PID settings
      PIDcontroller.SetTunings(MOTOR_R_P,MOTOR_R_I,MOTOR_R_D);
      PIDcontroller.SetOutputLimits(0, 255);                      // remove previous pwm limits
      // Set PID controller to auto, e.g. switch on
//      PIDcontroller.SetMode(AUTOMATIC);
      
      // change state from wait_for_start to racing
      state = RACING;
      
      LOG(msec_since_start)
      LOG("RACING")
      LOG_LN()    
#if SD_CARD_ON
      logfile.flush();
#endif
      
      // State has changed, return to main loop
      return;
    }  
  }
}


//=================== RACING ====================================================
void racing_loop()
{
  while(1)
  {
  if (racing_timer.check())
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

    // reset PID control in case of de-slot
    // still needs fixing
    if ((trackVoltReading < 100)&&(speedReading <= 1))
    {
      LOG(msec_since_start)
      LOG("DESLOT")
      LOG_LN()
    }
    
    int32_t dirReading = 0;
    if (bno_on)
    {
      // Read the acceleration and orientation:
      bno.getEvent(&accel_orient);

      dirReading = uint32_t(accel_orient.orientation.x);
    }
 
#if IR_ON
    if (irrecv.decode(&ir_results)) // have we received an IR signal?
    {
//    LOG_SERIAL_LN(results.value, HEX);  UN Comment to see raw values
      translateIR(ir_results, irButton); 
      irrecv.resume(); // receive the next value
    }  
#endif

    // Check if lap has finished
    if (distance > finish_position)
    {
      seq_cnt = 0;
      distance -= finish_position;
      lap_cnt++;
      if (lap_cnt > 3)                // finish the race after 3 laps
      {
        state = FINISH;
      }
    }
    
    sp = speedReading + sp1 + sp2 + sp3 + sp4;
    PID_input = (float)sp;
  
    // check if we changed into next segment
    if ( distance >= lap[seq_cnt+1].position)
    {
      seq_cnt++;   

      // check if next segment is straight or curve
      if (lap[seq_cnt].segment_type == STRAIGHT)
      {
        // switch off PID
        PIDcontroller.SetMode(MANUAL);
        race_state = ACCELERATE;   
      }else
      {
        if (seq_cnt >= brake_segment)
        {
          PID_setpoint = lap[seq_cnt].wanted_speed * PID_INTERVAL / 10 ;
          if (race_state != STEADY)
          {
            race_state = STEADY;   
            // still needs fixing
//            PID_output = 0;
            PID_output = lap[seq_cnt].wanted_speed * 1.6;
            // Set PID controller to auto, e.g. switch on
            PIDcontroller.SetMode(AUTOMATIC);
          }
        }
      }     
    }

    // do action according to segment_type
    if (lap[seq_cnt].segment_type == STRAIGHT)
    {
      // in case race_state=ACCELERATE calculate for next 5 segments the brake_point = (Vstart^2 - Vend^2)/(2 * brake_faktor)
      if (race_state == ACCELERATE)
      {
        // initialise brake_point to start of next segment
        brake_point = lap[seq_cnt+1].position;
        for (int i=1; i < 6 ; i++)
        {     
          if (sp > lap[seq_cnt+i].wanted_speed)
          {
            // add 5 speed intervals to brake distance as it takes about 50ms before car starts slowing down
            // TODO: ??? can bp ever be negative?  I changed "int bp" to "uint32_t bp" -- ALEX
            uint32_t bp = lap[seq_cnt+i].position - (uint)round((sp/5)*(5)) - (uint)round(((sp*sp/2.5) - (lap[seq_cnt+i].wanted_speed*lap[seq_cnt+i].wanted_speed/2.5))/(2*brake_faktor));
            // take closest brake point
            if (bp < brake_point)
            {
              brake_point = bp;
              brake_segment = seq_cnt+i;
            }
          }
        }
      }
      
      // check if brake_point has been reached
      if (distance < brake_point)
      {
        race_state = ACCELERATE;   
        // Calculate maximum increase of pwm
        if (255 > PID_output_maximized_old + PID_max_change)
        {
          PID_output_maximized = PID_output_maximized_old + PID_max_change;
        }else
        {
          PID_output_maximized = 255;
        }

        motor_on_pwm(PID_output_maximized);
        PID_output_maximized_old = PID_output_maximized;
//        motor_on_pwm(255);
//        motor_on();
      } else
      {
        race_state = BRAKE;   
        if (sp > (lap[brake_segment].wanted_speed * 1.0)) // hard brake
        {
//          if ((0 < PID_output_maximized_old - PID_max_change) && (PID_output_maximized_old > PID_max_change))
//          {
//            PID_output_maximized = PID_output_maximized_old - PID_max_change;          
//            motor_on_pwm(PID_output_maximized);
//            PID_output_maximized_old = PID_output_maximized;
//          }else
//          {
            motor_off_brake();
            PID_output_maximized = 0;
            PID_output_maximized_old = 0;
//          }
        }else                                           // soft brake
        {
          motor_off_coast();      
//          motor_off_brake();
        }
      }
    } else // CURVE
    {
      // only do curve actions if hard braking towards brake_segment is finished
      if (seq_cnt >= brake_segment)
      {
        PID_input = (float)sp;
        PIDcontroller.Compute();

        // Calculate maximum increase/decrease of pwm
        if (PID_output > PID_output_maximized_old + PID_max_change)
        {
          PID_output_maximized = PID_output_maximized_old + PID_max_change;
        }else if ((PID_output < PID_output_maximized_old - PID_max_change) && (PID_output_maximized_old > PID_max_change))
        {
          PID_output_maximized = PID_output_maximized_old - PID_max_change;          
        }else
        {
          PID_output_maximized = PID_output;
        }

        motor_on_pwm(PID_output_maximized);
        PID_output_maximized_old = PID_output_maximized;
//        motor_on_pwm(lap[seq_cnt].wanted_speed*1.6);
      }
    }
    
    // Log execution time in milliseconds
    uint32_t execution_time_msec = millis() - msec_since_start;

    // Log measurements to serial and/or flash:
    LOG(msec_since_start)
    LOG(execution_time_msec)
    
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
    if (race_state == ACCELERATE)
    {
      LOG(255)  
    } else if (race_state == BRAKE)
    {
      LOG(0)  
    }else
    {
      LOG(PID_setpoint)  
    }
    LOG(PID_output)
    LOG(MOTOR_R_P)
    LOG(MOTOR_R_I)
    LOG(MOTOR_R_D)

    LOG("")
    LOG(PID_output_maximized)
    LOG(race_state)
    LOG(brake_point)
    LOG(brake_segment)

#if IR_ON
    LOG(irButton)
    strcpy(irButton,"");
#endif

    LOG(seq_cnt)
 
    LOG_LN()
    
#if SD_CARD_ON
    logfile.flush();
#endif
    
#if RADIO_ON
    seq_no++;
    // If we check here that the packet was sent from the previous loop,
    // we save time because we could already do lots of other things in the
    // meantime
  //  if (seq_no > 1) radio.waitPacketSent();
 
    // Construct the message we'll send
    message.msg_type = MSG_MEASUREMENT;
    message.msg.measurement = (measurement_t){seq_no, msec_since_start, speedReading, dirReading, sp /*trackVoltReading*/};
  
    LOG_SERIAL_LN(F("Transmitting on radio"));
    radio.send((uint8_t *)&message, sizeof(message_t));
    radio.waitPacketSent();
 
    // Did we receive something from the PC? 
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    while (radio.waitAvailableTimeout(0))
    { 
      // Should be a reply message for us now   
      if (radio.recv(buf, &len))
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

    speedReadingLogged = 0;

    // update history:
    sp1 = sp2;
    sp2 = sp3;
    sp3 = sp4;
    sp4 = speedReading;

    digitalWrite(green_LED_PIN, LOW);
  } // racing updates
  
  if (state != RACING)
  {
    return;
  }
} 
}


//=================== RACE FINISHED ====================================================
// Race finished:
// - stop motor
// - close logfile
// - go into endless loop
void finish_loop()
{
  // Log milliseconds since starting
  uint32_t msec_since_start = millis();
  
  digitalWrite(green_LED_PIN, HIGH);
  // Stop motor
  motor_off_brake();
  LOG(msec_since_start)
  LOG("FINISH")
  LOG(lap_cnt)
  LOG_LN()    
#if SD_CARD_ON
  logfile.flush();
#endif
  // Stop program
  while(1);
}


