#include "datatypes.h"
#include "measuring.h"



//=================== MEASURING ====================================================
void measuring_loop()
{
  while (1)
  {
    if (measurement_timer.check())
    {
      LOG_SERIAL_LN("MEASURING")
      digitalWrite(green_LED_PIN, HIGH);
  
      // Log milliseconds since starting
      uint32_t msec_since_start = millis();
    
      // Take all measurement values right at the beginning of the
      // main loop to ensure taking them after the same interval
      // all the time
      trackVoltReading = analogRead(track_volt_PIN);    
      int engineVoltReading = analogRead(engine_volt_PIN);    
  
      // Read wheel encoder count
      noInterrupts();
      speedReading = speedCounter;  //calculate speed based on speedCounter
      speedCounter = 0;                 //reset speedCounter
      interrupts();
  
      speedReadingLogged += speedReading;
      distance += speedReading;
      // Add up the last 5 speed readings.  This is because we only
      // get a few counts per loop.  For the PID controller these small
      // numbers didn't work well.
      // TODO (Alex): why did I never try to just multiply the speedReading with 5???
      sp = speedReading + sp1 + sp2 + sp3 + sp4;
  
      // Motor also as early as possible to ensure constant update intervals
      PID_input = (float)sp;
      // Reset PID control in case of de-slot
      if ((trackVoltReading < 100) && (speedReading <= 1))
      {
        LOG(msec_since_start)
        LOG("DESLOT")
        LOG_LN()
        PIDcontroller.SetMode(MANUAL);
        PID_output = 0;
        PIDcontroller.SetMode(AUTOMATIC);
      }
      PIDcontroller.Compute();
      motor_on_pwm(PID_output);
  
      // for handcontroller running of the car 
      //    motor_on_pwm(255);
  
      uint32_t dirReading = 0;
      float dirChange = 0.0;
      if (bno_on)
      {
        // Read the acceleration and orientation:
        bno.getEvent(&accel_orient);
    
        dirReading = (uint32_t)accel_orient.orientation.x;
    
        // Start measuring after BNO055 starts giving real values.
        // Even though we wait several seconds after power on, the
        // first readings are always un-usable.
        // Once the dirReading is > 0 the BNO delivers usable values:
        if (measuring == false)
        {
          if (dirReading > 0)                                           // BNO055 is giving real values so measuring can start
          {
            measuring = true;
            
            // Set-up first segment
            lap[seq_cnt].position = 0;                                  // start/finish is at position 0
            lap[seq_cnt].direction = dirReading;
            lap[seq_cnt].segment_type = STRAIGHT;
            lap[seq_cnt].wanted_speed = segment_types_speed[STRAIGHT];  // assumption that start/finish is on a straight
            seq_cnt++;
      
            // Initialise measurement history variables
            for (int i=0; i < 9; i++)
            {
              lap_hist[i].position = 0;                               // start/finish is at position 0
              lap_hist[i].direction = dirReading;
              lap_hist[i].segment_type = STRAIGHT;          
              lap_hist[i].wanted_speed = segment_types_speed[STRAIGHT];
            }
          }        
        }   
        else  // BNO gives good values
        {
          // TODO (Leon): still needs fixing ---> why?
          dirOverflow = 0;
          // Direction overflowed 360 -> 0
          if ((lap[seq_cnt-1].direction > 270) && (dirReadingOld >= lap[seq_cnt-1].direction) && (dirReading <90))
          {
            dirOverflow = 360;
          }
          // Direction underflowed 0 -> 360
          if ((lap[seq_cnt-1].direction < 90) && (dirReadingOld <= lap[seq_cnt-1].direction) && (dirReading >270))
          {
            dirOverflow = -360;
          }
          dirReading += dirOverflow;  // Apply correction.  dirReading is now -90 ... 450
          dirReadingOld = dirReading;

          // Has direction changed from the previous segment?
          if (abs(dirReading - lap[seq_cnt-1].direction) > 1)
          {
            // rotate history of last 10 measurements (FIFO)
            for (int i=8; i >= 0 ; i--)
            {
              lap_hist[i+1] = lap_hist[i];
// used to be element-by-element copy:
//              lap_hist[i+1].position = lap_hist[i].position;
//              lap_hist[i+1].direction = lap_hist[i].direction;
//              lap_hist[i+1].segment_type = lap_hist[i].segment_type;     
//              lap_hist[i+1].wanted_speed = lap_hist[i].wanted_speed;     
            }        
            // store current measurement
            lap_hist[0].position = distance;
            lap_hist[0].direction = dirReading;
            // determine the relative direction change during the last 10 measurements (to smoothen the measurement result curve)
            // TODO (Leon): the above line is the original, but below is more correct, or?
            // determine the relative direction change during the last 10 direction changes (to smoothen the measurement result curve)
            //
            // TODO (Leon): why [9] here and [5] everywhere else?  Can you explain in a comment why?
            dirChange = (((float)abs(lap_hist[0].direction - lap_hist[9].direction))/(float)(lap_hist[0].position - lap_hist[9].position));
            if (dirChange < 0.05)
            { 
              lap_hist[0].segment_type = STRAIGHT;
            }
            else if (dirChange < 0.087)
            { 
              lap_hist[0].segment_type = CURVE_4;
            }
            else if (dirChange < 0.125)
            { 
              lap_hist[0].segment_type = CURVE_3;
            }
            else if (dirChange < 0.25)
            { 
              lap_hist[0].segment_type = CURVE_2;
            }
            else if (dirChange < 0.5)
            { 
              lap_hist[0].segment_type = CURVE_1;
            }
            else
            { 
              lap_hist[0].segment_type = SLIDE;
            }   
            lap_hist[0].wanted_speed = segment_types_speed[lap_hist[0].segment_type];

            // Create new lap_segment in case direction has changed in the last 6 measurements (to avoid short peaks/drops)
            if  ((lap_hist[0].segment_type != lap[seq_cnt-1].segment_type)
              && (lap_hist[1].segment_type != lap[seq_cnt-1].segment_type)
              && (lap_hist[2].segment_type != lap[seq_cnt-1].segment_type)
              && (lap_hist[3].segment_type != lap[seq_cnt-1].segment_type)
              && (lap_hist[4].segment_type != lap[seq_cnt-1].segment_type)
              && (lap_hist[5].segment_type != lap[seq_cnt-1].segment_type))
            {
              // In case of dirOverflow reverse the dirOverflow change done
              // TODO (Leon): explain why we correct this only when a new lap segment has been found.
              //              The lap_hist[] array is not cleared, and we use it next loop again.
              //              This means we have lap_hist[i].direction with overflow-correction
              //              in one case and without overflow-correction in other cases (when a new
              //              segment started).
              //              What if instead we'd change a few lines down from:
              //              lap[seq_cnt].direction = lap_hist[5].direction;
              //              to:
              //              lap[seq_cnt].direction = lap_hist[5].direction - dirOverflow;
              //
              // TODO (Leon): why [5] here?  You mention "smoothing of measurements" earlier,
              //              but taking [5] does not average the values from lap_hist[].
              //              lap_hist[] provides hysteresis, but not smoothing in my opinion.
              if (dirOverflow != 0)
              {
                for (int i=0; i < 9; i++)
                {
                  lap_hist[i].direction -= dirOverflow;
                }        
              }
              // Create the new lap segment:
              lap[seq_cnt].position = lap_hist[5].position;                             // set starting position of next segment
              lap[seq_cnt-1].length = lap[seq_cnt].position - lap[seq_cnt-1].position;  // calculate and set length of previous segment
              lap[seq_cnt].direction = lap_hist[5].direction;                           // set other values of next segment
              lap[seq_cnt].segment_type = lap_hist[5].segment_type;
              lap[seq_cnt].wanted_speed = lap_hist[5].wanted_speed;
    
              // To avoid short segments only store segments longer than 20
              if (lap[seq_cnt-1].length > 20)
              {
                // TODO (Leon): If we only increment seq_cnt when the segment is long enough,
                //              then we could move the 5 lines above where we create the new
                //              segment also here and not have them before the if-statement.
                seq_cnt++;           
     
                // If new lap segment is a straight, store a new straight and do lap recognition based on the straights
                // TODO (Leon): why "seq_cnt - 2" and not "seq_cnt - 1" if the comment on the previous
                //              line mentions "new lap segment"?  "seq_cnt - 2" is the previous segment.
                if (lap[seq_cnt-2].segment_type == STRAIGHT)
                {
                  straight[straight_cnt].position  = lap[seq_cnt-2].position;
                  straight[straight_cnt].length    = lap[seq_cnt-2].length;
                  straight[straight_cnt].direction = lap[seq_cnt-2].direction;
                
                  // check if new straight has been driven/measured before
                  for (uint32_t i=0; i < straight_cnt; i++)
                  {
                    if ((straight[straight_cnt].length > 100)
                    && (abs(straight[i].direction - straight[straight_cnt].direction) < 8)
                    && (abs(straight[i].length    - straight[straight_cnt].length) < 40))
                    // TODO (Leon): If we use direction and length of straights to detect a lap,
                    //              wouldn't then a track like this trigger the lap detector: ??
                    //              (the two "---" straights at the top)
                    //         +---+   +---+
                    //         |   |   |    \                    .
                    //  >------+    \-/      +-------------->
                    {
                      lap_found = true;  // lap has been detected
                      // Subtract found straight from current straight is lap length e.g. finish position
                      // TODO (Leon): the variable finish_position as it is below is the lap lenght.
                      //              It is only equal to the finish position when "distance" is
                      //              set to zero on the start line.  The code in the wait_for_start
                      //              function sets the distance to zero on start, so that is ok.
                      //              But I think we should also go through all straight[] elements
                      //              and through all lap[] elements and subtract straight[i].position
                      //              with "i" having the value it has here.
                      //              Otherwise the code only works when the car is put onto the
                      //              start line when measurement begins.
                      finish_position = straight[straight_cnt].position - straight[i].position; 
                    }
                  }
                  straight_cnt++;
                }
              }
              else  // Last segment was too short
              {
                // If new segments is tighter => change segment_type to tighter type
                // (If the new segment has a larger radius we don't care because
                //  with the slower wanted_speed we're on the safe side).
                // A gradual change in curve radius is still detected.
                if (lap_hist[5].wanted_speed < lap[seq_cnt-1].wanted_speed)
                {
                  lap[seq_cnt-1].segment_type = lap_hist[5].segment_type;            
                  lap[seq_cnt-1].wanted_speed = lap_hist[5].wanted_speed;
                }
              }        
            }
          }
        }
 
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
      }  // if (bno_on)
      
#if IR_ON
      if (irrecv.decode(&ir_results)) // have we received an IR signal?
      {
  //    LOG_SERIAL_LN(results.value, HEX);  UN Comment to see raw values
        translateIR(ir_results, irButton); 
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
      LOG(PID_setpoint)
      LOG(PID_output)
      LOG(MOTOR_M_P)
      LOG(MOTOR_M_I)
      LOG(MOTOR_M_D)
  
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
        LOG(lap[seq_cnt-1].wanted_speed)
        LOG(dirChange)
        LOG(straight_cnt)
        LOG(lap_found)
        LOG(finish_position)
      }
      else
      {
        LOG(seq_cnt);
      }
      
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
      telemetry.send_msg_wait((uint8_t *)&message);
   
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
  
      // reset speedvariables
      speedReadingLogged = 0;
  
      // Update speed history (FIFO):
      sp1 = sp2;
      sp2 = sp3;
      sp3 = sp4;
      sp4 = speedReading;
  
      digitalWrite(green_LED_PIN, LOW);
  
      // TODO (Leon): better use "lap_found" instead of "finish_position > 0"?  It's the same but I find
      //              "lap_found" easier to understand.
      if  ((finish_position > 0) 
        && (distance >= (2*finish_position - ((10*MOTOR_TARGET_SPEED*MOTOR_TARGET_SPEED/25)/(2*brake_faktor)))))
        // TODO (Leon): this second part of the condition, is it to ensure a minimal lap length?
        //              Or what does it protect against?
      {
        state = MEASURING_FINISHED;
        return;
      }  
    } // measurement updates
  } // while
}


//=================== MEASURING FINISHED ====================================================
// Measuring finished:
// This is a transitional state: we go through it once and then
// start the race.
void measuring_finished()
{
    // Stop motor at start/finish straight after lap detection has finished
    // Still not 100% working, needs fixing
    // stop motor
    motor_off_brake();
    
    // Log milliseconds since starting
    uint32_t msec_since_start = millis();

    // Set PID controller to manual, e.g. switch off
    PIDcontroller.SetMode(MANUAL);

    LOG_SERIAL_LN("MEASURING FINISHED => WAIT_FOR_START")
    digitalWrite(green_LED_PIN, HIGH);
    
    // Determine which seq_cnt belongs to the end of the start/finish 
    // straight and limit the stored lap to that
    LOG(msec_since_start)
    LOG_LN("Writing Lap Data")
    for (uint32_t i=0; i < seq_cnt ; i++)
    {
      LOG(i)
      LOG(lap[i].position)
      LOG(lap[i].length)
      LOG(lap[i].direction)
      LOG(lap[i].segment_type)
      LOG(lap[i].wanted_speed)
      LOG_LN()    
      if (lap[i].position > finish_position)
      {
        max_seq_cnt = i;
        // TODO (Leon): should we add a line with "break;" here?  If not and there
        //              are several segments after the finish position, then max_seq_cnt
        //              would point to the last one, not the first one.
      }
    }

    // Consolidate Lap
    bool decrease = true;
    int new_seq_cnt = 0;
    uint32_t i = 0;
    int j = 0;
    // TODO (Leon): why is there a variable "i"?  I think we could use "j" everywhere.
    while (i < max_seq_cnt)
    {
      if (decrease == true) // decrease speed
      {
        while (lap[j].wanted_speed >= lap[j+1].wanted_speed)
        {
          j++;
        }
        lap[new_seq_cnt].length       = lap[j].position - lap[new_seq_cnt].position;
        new_seq_cnt++;
        lap[new_seq_cnt].position     = lap[j].position;
        lap[new_seq_cnt].length       = lap[j].length;
        lap[new_seq_cnt].direction    = lap[j].direction;
        lap[new_seq_cnt].segment_type = lap[j].segment_type;
        lap[new_seq_cnt].wanted_speed = lap[j].wanted_speed;
        new_seq_cnt++;
        lap[new_seq_cnt].position = lap[j+1].position;
        decrease = false;
        i = j;
      }
      else // increase speed
      {
        while (lap[j].wanted_speed <= lap[j+1].wanted_speed)
        {
          j++;
        }
        lap[new_seq_cnt].direction    = lap[j].direction;
        lap[new_seq_cnt].segment_type = lap[j].segment_type;
        lap[new_seq_cnt].wanted_speed = lap[j].wanted_speed;
        decrease = true;
        i = j;
      }
    }
    max_seq_cnt = new_seq_cnt;

    LOG(msec_since_start)
    LOG_LN("Writing Consolidated Lap Data")
    for (uint32_t i=0; i < max_seq_cnt ; i++)
    {
      LOG(i)
      LOG(lap[i].position)
      LOG(lap[i].length)
      LOG(lap[i].direction)
      LOG(lap[i].segment_type)
      LOG(lap[i].wanted_speed)
      LOG_LN()    
    }    // set current leg segment counter to 0 e.g. start/finish
    seq_cnt = 0;

    // set the wanted speed for this segment
    PID_setpoint = lap[seq_cnt].wanted_speed * PID_INTERVAL / 10 ;

    // clear all speed variables
    sp1 = sp2 = sp3 = sp4 = 0;

    // change state from measuring to wait_for_start
    state = WAIT_FOR_START;

    // start the race 3 seconds from now
    startTime = millis() + 3000;
    
    LOG(msec_since_start)
    LOG("WAIT_FOR_START")
    LOG(seq_cnt)
    LOG(distance)
    LOG(PID_setpoint)
    LOG(startTime)
    LOG_LN()    
#if SD_CARD_ON
    logfile.flush();
#endif
  
}

