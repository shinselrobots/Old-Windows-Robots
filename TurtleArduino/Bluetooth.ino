// Bluetooth:  Functions for reading data from Android phone via Bluetooth



void ConnectionEvent(byte flag, byte numOfValues)
{
  int Value = meetAndroid.getInt();
  Status.AndroidConnected = Value;

  if(1 == Value)
  {
    PrintDebug("Android Connected");
  }
  else if(0 == Value)
  {
    PrintDebug("Android Disconnected");
  }
  else
  {
    PrintDebug("Android Connect Error!");
  }

}


void EnableAccelerometerEvent(byte flag, byte numOfValues)
{
  int Value = meetAndroid.getInt();
  Status.AndroidAccEnabled = Value;

  if(1 == Value)
  {
    PrintDebug("Android Acc Enabled");
  }
  else if(0 == Value)
  {
    PrintDebug("Android Acc Disabled");
  }
  else
  {
    PrintDebug("Android Acc Msg Error!");
  }

}

void eXecuteCmdEvent(byte flag, byte numOfValues)
{
  // Command received from the Android device
  /* 
  // if this info is needed, convert to PrintDebug statements
  Serial.print("Received Data: ");
  Serial.print(numOfValues);
  Serial.print(" items.  Value = ");
  */
  int Value = meetAndroid.getInt();
  Status.AndroidCmd = Value;
  
  // Send message back to Android Phone
  unsigned char charMsg[32];
  String strMsg = "Arduino: Received " + String(Value, DEC);
  strMsg.getBytes(charMsg, 31);
  meetAndroid.send((char*)charMsg);
}

void AccelerometerEvent(byte flag, byte numOfValues)
{
  // Received Azimuth (compass), Pitch, Roll
  Status.AndroidUpdatePending = 1; // indicate that new data is available
  int AccelerometerValues[3];

  //Serial.print("Received Acc Az,Pitch,Roll: ");
  //Serial.print(numOfValues);
  //Serial.print(" items.  Values = ");

  meetAndroid.getIntValues(AccelerometerValues);
  
  int AndroidCompass = AccelerometerValues[0];
  Status.AndroidCompassHigh = (byte)(AndroidCompass >>8);	// Hi Byte
  Status.AndroidCompassLow = (byte)( AndroidCompass & 0xFF);	// Low Byte
  
  int AndroidPitch = AccelerometerValues[1];
  Status.AndroidPitchHigh = (byte)(AndroidPitch >>8);	        // Hi Byte
  Status.AndroidPitchLow = (byte)( AndroidPitch & 0xFF);	// Low Byte
     
  int AndroidRoll = AccelerometerValues[2];
  Status.AndroidRollHigh = (byte)(AndroidRoll >>8);	        // Hi Byte
  Status.AndroidRollLow = (byte)( AndroidRoll & 0xFF);		// Low Byte
  
}
