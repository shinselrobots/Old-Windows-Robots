
#define TRUE true
#define FALSE false


void HandleCmdFromPC()
{
  // Handle commands received from the PC
  // gPicCmdBuffer contains all the parameters

  int i;
  switch( gPicCmdBuffer.Cmd )
  {

  case HW_RESET_CPU:
    {
      // Arduino does not have a sofware reset!
      gLedEyeMode = LED_EYES_OFF;  // go to sleep, little robot...
      SetLedEyes(0);	// SetLedEyes will handle the bottom 3 bits
      //Initialize();
      //delay_ms(500);
      //reset_cpu();
      break;
    }

    // TODO REMOVE THESE!
  case HW_INITIALIZE:
    PrintDebug("NAK: HW INIT not implemented");
    break;
  case HW_CLEAR_ERROR:
    PrintDebug("NAK: Clear Error NA");
    break;
  case HW_GET_VERSION:
    PrintDebug("NAK: Version NA");
    //sprintf(gResponseMsg, "AK3");
    //gResponseMsg[3] = PIC_CODE_VERSION;
    //SendResponse(4);
    break;
  case HW_RESET_ODOMETER:
    PrintDebug("NAK: Reset ODOM NA");
    //sprintf(gResponseMsg, "AK1:RST ODOM NA!"); - Was not used in PIC either
    // Not Implemented: gStatus_OdometerTickCount = 0;
    break;
  
  case HW_GET_STATUS:
    {
      PrintDebug("NAK: Status NA");
      break;
    }

  case HW_SET_SERVO_POWER: // Servo power
    {
      if( 0 != gPicCmdBuffer.Param2 )
      {
        digitalWrite(SERVO_PWR_18V_PIN, true);
        digitalWrite(SERVO_PWR_12V_PIN, true);
        PrintDebug("ACK: SERVO PWR On");
      }
      else
      {
        digitalWrite(SERVO_PWR_18V_PIN, false);
        digitalWrite(SERVO_PWR_12V_PIN, false);
        PrintDebug("ACK: SERVO PWR Off");
      }
      break;
    }

  case HW_SET_LIGHT_POWER: // Not Connected
    {
      PrintDebug("TODO: LIGHT POWER");
      break;
    }

  case HW_SET_AUX_LIGHT_POWER:
    {
      if( 0 != gPicCmdBuffer.Param2 )
      {
        digitalWrite(AUX_LIGHT_PIN, true);
        PrintDebug("ACK: Aux Lights On");
      }
      else
      {
        digitalWrite(AUX_LIGHT_PIN, false);
        PrintDebug("ACK: Aux Lights Off");
      }
      break;
    }

  case HW_SET_IR_SENSOR_POWER:
    {
      PrintDebug("TODO: IR CLAW LED POWER");
      // IR LED is controlled via I2C expanded ports
      /*if( 0 != gPicCmd.Param2 )
       {
       gClawIRSensorOn = TRUE;
       SetClawIRStateLeft();
       sprintf(gResponseMsg, "AK1:LClawIR On");
       }
       else
       {
       gClawIRSensorOn = FALSE;
       SetClawIRStateLeft();
       sprintf(gResponseMsg, "AK1:LClawIR Off");
       }
       SendResponse(0);
       */
      break;
    }


  case HW_SET_LED_EYES:
    {
      // Set eye mode, to open, closed, or blink
      gLedEyeMode = gPicCmdBuffer.Param1;  

      if( LED_EYES_OPEN == gLedEyeMode )
      {
        // Open Eye in stages
        gEyeState = 6;	// Start closed
      }
      else if( LED_EYES_CLOSE == gLedEyeMode )
      {
        // Close Eye in stages
        gEyeState = 0;	// Start opened
      }
      else if( LED_EYES_ON == gLedEyeMode )
      {
        // Open Eye instantly
        SetLedEyes( 0xFF  );	// SetLedEyes will mask out the bottom 3 bits
      }
      else if( LED_EYES_OFF == gLedEyeMode )
      {
        // Close Eye instantly
        SetLedEyes( 0x00  );	// SetLedEyes will mask out the bottom 3 bits
      }
      else if( LED_EYES_BLINK == gLedEyeMode )
      {
        // Close Eye in stages, then open again
        gEyeState = 0;	// Start opened
      }
      //sprintf(gResponseMsg, "AK1:EYES %02X", gPicCmd.Param1 );
      //SendResponse(0);
      PrintDebugHex("ACK: HW_SET_LED_EYES = ", gLedEyeMode);

      break;
    }

    /*
#define LAPTOP_POWER_MODE_DEFAULT				0x00	// Leave laptop alone.  Do nothing
     #define LAPTOP_POWER_OFF_NO_RESTART				0x01	// Shut down Laptop (using servo).  Requires manual restart.
     #define LAPTOP_POWER_OFF_RESTART_ON_ANY_SENSOR	0x02	// Shut down, but restart when any sensor triggers (preset thresholds)
     #define LAPTOP_POWER_OFF_RESTART_ON_PIR_SENSOR	0x03	// Shut down, restart only with PIR sensor
     #define LAPTOP_POWER_OFF_RESTART_NO_PIR_SENSOR	0x04	// Shut down, restart with any sensor BUT PIR
     #define LAPTOP_POWER_OFF_RESTART_ON_TIMER		0x05	// TODO - not implemented!
     
     #define POWER_STATE_PRESSING_POWER_BUTTON
     #define POWER_STATE_RELEASING_POWER_BUTTON
     #define POWER_STATE_MONITORING_SENSORS
     */
  case HW_SET_POWER_MODE:
    {
      PrintDebug("TODO: HW_SET_POWER_MODE - Laptop");
      /** DISABLED
       * 
       * //output_low( PIN_PWR_RELAY );	// Turn Off
       * gRobotPowerMode = gPicCmd.Param2;
       * if( LAPTOP_POWER_MODE_DEFAULT == gRobotPowerMode )
       * {
       * sprintf(gResponseMsg, "AK1:PWR_DEFAULT");
       * // Leave Servo in "up" position, and turn off servo power
       * LaptopPowerServo = LAPTOP_POWER_SERVO_UP_POSITION;
       * gPowerStateTimer = POWER_BUTTON_SERVO_MOVE_TIME;
       * gRobotPowerState = IDLE;
       * }
       * else
       * {
       * // Must be one of the Laptop Power Off commands
       * // Push the power button the go to next state
       * sprintf(gResponseMsg, "AK1:PWR_STBY");
       * LaptopPowerServo = LAPTOP_POWER_SERVO_DOWN_POSITION;
       * gRobotPowerState = POWER_STATE_PRESSING_POWER_BUTTON;
       * gPowerStateTimer = POWER_BUTTON_SERVO_MOVE_TIME;
       * }
       **/

      //SendResponse(0);
      break;
    }

  default:
    {
      PrintDebugHex("TODO: HW Command Not Implemented! ", gPicCmdBuffer.Cmd);
      //sprintf(gResponseMsg, "NAK:BadCmd:%02X %02X %02X\n", gPicCmd.Cmd, gPicCmd.Param1, gPicCmd.Param2);
    }
  }

}



