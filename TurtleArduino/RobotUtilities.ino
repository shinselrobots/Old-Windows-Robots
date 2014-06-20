
/////////////////////////////////////////////////////////////////////////////////////
//  Robot Subroutines
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
boolean TimeToSendStatus()
{
  static unsigned long LastStatusTime = 0;
  unsigned long CurrentTime = millis();
  if( CurrentTime - LastStatusTime > StatusInterval ) 
  {
    LastStatusTime = CurrentTime;    
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////
boolean TimeToBlinkHeartBeat()
{
  static unsigned long LastBlinkTime = 0;
  unsigned long CurrentTime = millis();
  if( CurrentTime - LastBlinkTime > BlinkInterval ) 
  {
    LastBlinkTime = CurrentTime;    
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////
void ToggleHeartBeatLed()
{
  // Blink the Heartbeat LED 
  static boolean LedState = 0;
  LedState = !LedState;
  digitalWrite(HEARTBEAT_LED_PIN, LedState); // Active High
  digitalWrite(STATUS_LED_PIN, !LedState);   // Active Low
  
  // blink the arms too
  SetArmLED( I2C_PCF8574_LEFT_ARM, LedState );
  SetArmLED( I2C_PCF8574_RIGHT_ARM, LedState );
  
}

void AllLedsOff()
{
  digitalWrite(HEARTBEAT_LED_PIN, LOW); // Active High
  digitalWrite(STATUS_LED_PIN, HIGH);   // Active Low
  //digitalWrite(STATUS_LED_PIN2, HIGH);   // Active Low
  //SetArmLED( I2C_PCF8574_LEFT_ARM, LOW ); // blink the arms too
  //SetArmLED( I2C_PCF8574_RIGHT_ARM, LOW );
}

void AllLedsOn()
{
  digitalWrite(HEARTBEAT_LED_PIN, HIGH); // Active High
  digitalWrite(STATUS_LED_PIN, LOW);   // Active Low
  //digitalWrite(STATUS_LED_PIN2, LOW);   // Active Low
  //SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
  //SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );
}

/////////////////////////////////////////////////////////////////////////////////////
void PrintDebug(char *Message)
{ 
  // Sends debug message to the robot control code, which prints the message into the robot log
  int StrSize = strlen(Message);
  // Write the Header
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+2);              // size of data, plus header: version + message type
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message);  
}

/////////////////////////////////////////////////////////////////////////////////////
void PrintDebugHex(char *Message, int Value)
{
  // Sends debug message plus a HEX value to the robot control code, which prints the message into the robot log
  // Write the Header
  int StrSize = strlen(Message);
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+4);              // size of data, plus header (version + message type), plus 2 HEX characters 
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message); 
  if(Value < 0x10)
  {
    Serial.print("0"); // Kludge - print does not print the leading zero! Doh!
  }
  Serial.print(Value, HEX);
}


/////////////////////////////////////////////////////////////////////////////////////
void PrintDebugDec(char *Message, int Value)
{
  // Sends debug message plus a Decimal value to the robot control code, which prints the message into the robot log
  // Write the Header
  int StrSize = strlen(Message);
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+5);              // size of data, plus header (version + message type), plus 3 DEC characters 
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message); 
  if(Value < 100)
  {
    Serial.print(" "); // Kludge - need to print 3 chars
  }
  if(Value < 10)
  {
    Serial.print(" "); // Kludge - need to print 3 chars
  }
  Serial.print(Value, DEC);
}


/////////////////////////////////////////////////////////////////////////////////////
// Interrupt service routines for monitoring the wheel odometers
/*
void CounterISR0()
{ 
  Counter0.poll();
}
void CounterISR1()
{ 
  Counter1.poll();
}
*/

/////////////////////////////////////////////////////////////////////////////////////
void SendStatusToPC()
{
  // Sends status update to the robot control program with sensor data
  // Serial.print(lowByte(TestInt), BYTE); 
  // Serial.print(highByte(TestInt), BYTE);

  // Write the Header
  Serial.write("ZZ");                    // 2 Sync Characters, using Binary mode write 
  Serial.write(StatusSize+2);            // size of data, plus header: version + message type
  Serial.write(ARDUINO_CODE_VERSION);    // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_STATUS);  // Identify this as a Status message   
  // Write the Data  
  pStatusBuf = (byte*)&Status;
  Serial.write(pStatusBuf, StatusSize); // Finally! Write the actual data!
  
  // Reset some data after sending it to the PC:
  Status.AndroidCmd = 0;
  Status.AndroidUpdatePending = 0; // Set to true by Bluetooth code when new data received
  
}


/////////////////////////////////////////////////////////////////////////////////////
boolean CheckForSerialData()
{
  // See if a command from the PC is pending
  // Returns TRUE if command received

  enum SIO_STATE {
    SIO_WAIT_FOR_SYNC_0,
    SIO_WAIT_FOR_SYNC_1,
    SIO_GET_CMD,
    SIO_GET_PARAM1,
    SIO_GET_PARAM2,
    SIO_GET_PARAM3,
    SIO_GET_PARAM4,
    SIO_GET_TERM_CHAR,
    //  SIO_WAIT_FOR_CMD_COMPLETE,
  };

  static int SerialState = SIO_WAIT_FOR_SYNC_0;

  while( Serial.available() > 0 )
  {
    byte cTemp = Serial.read();
    switch( SerialState )
    {
    case SIO_WAIT_FOR_SYNC_0:
      // Looking for first sync byte
      if( cTemp == SIO_SYNC_0 ) 
      {        
        SerialState++; // First Sync byte found, go to next state
      }
      break;
    case SIO_WAIT_FOR_SYNC_1:
      // Looking for second sync byte
      if( cTemp == SIO_SYNC_1 ) 
      {
        SerialState++; // Second Sync byte found
      }
      else 
      {
        SerialState = SIO_WAIT_FOR_SYNC_0; // failed sync
      }
      break;
    case SIO_GET_CMD:
      // Get CMD byte
      gPicCmdBuffer.Cmd = cTemp;
      SerialState++;
      break;
    case SIO_GET_PARAM1:
      // Get first Parameter
      gPicCmdBuffer.Param1 = cTemp;
      SerialState++;
      break;
    case SIO_GET_PARAM2:
      gPicCmdBuffer.Param2 = cTemp;
      SerialState++;
      break;
    case SIO_GET_PARAM3:
      gPicCmdBuffer.Param3 = cTemp;
      SerialState++;
      break;
    case SIO_GET_PARAM4:
      gPicCmdBuffer.Param4 = cTemp;
      SerialState++;
      break;
    case SIO_GET_TERM_CHAR:
      // Check for terminator char
      if( cTemp == CMD_TERM_CHAR )
      {
        // Got the full command!
        SerialState = SIO_WAIT_FOR_SYNC_0;	// Ready to get the next command
        return true; // Done
      }
      else
      {
        PrintDebugHex("ERROR: Bad Term Char = ", cTemp);
        SerialState = SIO_WAIT_FOR_SYNC_0;
      }
      break;
    default:
      PrintDebugHex("ERROR: CheckForSerialData Bad State =  ", SerialState);
      SerialState = SIO_WAIT_FOR_SYNC_0;	// Abort and get next command
    }
  }

  return false;
}
/*
void BlinkEyes()
{
  static int BlinkTimer = 0;
  
  if( LED_EYES_ON == gLedEyeMode )
  {
    // See if it is time to start the next blink cycle
    if( 0 == BlinkTimer )
    {
      BlinkTimer = ( rand() % 180 ) + 20; // Range 20 to 200 * 20ms
      // Close Eye in stages, then open again
      gLedEyeMode = LED_EYES_BLINK;
      gEyeState = 0;	// Start opened
    }
    else
    {
      BlinkTimer--;
    }
  }

  // Turn LED EYES on or off ( in 20ms steps )
  if( LED_EYES_OPEN == gLedEyeMode )
  {
    // Open Eye in stages ( start shift at 5, count down to 0 )
    SetLedEyes( (0xFC << gEyeState) );	// SetLedEyes will handle the bottom 3 bits
    if( 0 == gEyeState )
    {
      gLedEyeMode = LED_EYES_ON;	// fully on now, stay on (until the next blink time)
    }
    else
    {
      gEyeState--;
    }
  }
  else if( LED_EYES_CLOSE == gLedEyeMode )
  {
    // Close Eye in stages ( start shift at 0, count up to 5 )
    SetLedEyes( (0xFC << gEyeState) );	// SetLedEyes will handle the bottom 3 bits
    if( 6 == gEyeState )
    {
      gLedEyeMode = LED_EYES_OFF;	// fully off now, stay off (until override from the PC)
    }
    else
    {
      gEyeState++;
    }
  }
  else if( LED_EYES_BLINK == gLedEyeMode )
  {
    // Close Eye in stages ( start shift at 0, count up to 5 ), and then open again
    SetLedEyes( (0xFC << gEyeState) );	// SetLedEyes will handle the bottom 3 bits
    if( 6 == gEyeState )
    {
      gLedEyeMode = LED_EYES_OPEN;	// fully off now, turn back on in stages
    }
    else
    {
      gEyeState++;
    }
  }
}
*/





