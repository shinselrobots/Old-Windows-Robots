// RobotSensors:  Functions for reading sensor data and copying to Status block for sending to the PC
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////
void ReadSensors()
{
//    unsigned long SensorStartTime = millis();
  
  // Read all Sensors, and update the status block for sending to the PC
  Status.StatusFlags = 0;
  Status.DebugCode = 5;
  //Status.HWBumper = PortC;
  //Status.IRBumper = PortA;
  Status.ArmBumperL = 0;
  Status.ArmBumperR = 0;
  //Status.Battery0 = Battery;
  //Status.CompassHigh = 0;
  //Status.CompassLow = 0;
  //Status.OdometerHighL;
  //Status.OdometerLowL;
  //Status.OdometerHighR;
  //Status.OdometerLowR;
   //Status.nOdometerSamples = 1;	// TODO - remove this
   //Status.LeftHandPressureL = 0;
   //Status.LeftHandPressureR = 0;
  //Status.IR[NUM_AD_IR_SENSORS];   // UP TO N, may be less! - Arduino Board A/D
  //Status.IR3[NUM_IR3_SENSORS]; // UP TO N, may be less! - I2C-IT IR Rangers
  
  //Status.HWBumper = Read_Digital_Port_C();
  //Status.IRBumper = Read_Digital_Port_A();
  //Read_Wheel_Odometers();

  //Status.ArmBumperL = ReadPCF8574(I2C_PCF8574_LEFT_ARM);
  //Status.ArmBumperR = ReadPCF8574(I2C_PCF8574_RIGHT_ARM);
  
  //Read_I2C_Digital_Ports();
  
  //Read_I2C_IT_IR_Rangers();
// SensorTime = millis() - SensorStartTime;
// PrintDebugDec("i2c-it = ", SensorTime);

  //Read_Hand_Pressure();

  //Read_Compass();
// SensorTime = millis() - SensorStartTime;
// PrintDebugDec("compass = ", SensorTime);

 // Read_Analog_Ports();
 //SensorTime = millis() - SensorStartTime;
 //PrintDebugDec("analog = ", SensorTime);
 

}

/////////////////////////////////////////////////////////////////////////////////////
void Read_Wheel_Odometers( )
{
  // WheelCounters are pulse counts since last time we checked.  The counter is reset when "count" is called
/* 
  boolean      RevRight = false;
  boolean      RevLeft = false;
  unsigned int WheelCountLeft = 0;
  unsigned int WheelCountRight = 0;
  unsigned int WheelFreqLeft = 0;
  unsigned int WheelFreqRight = 0;
  
  // Read the Wheel Odometers, if any update ready (if the wheels have moved since we last checked)
  if(Counter0.ready())
  {
    WheelFreqRight = Counter0.hertz();
    WheelCountRight = Counter0.count();
    if( HIGH == digitalRead(DIR_0_PIN) )
    {
      RevRight = true;
    }
  }

  if(Counter1.ready())
  {
    WheelFreqLeft = Counter1.hertz();
    WheelCountLeft = Counter1.count();
    if( LOW == digitalRead(DIR_1_PIN) ) // Left wheel is mounted backward
    {
      RevLeft = true;
    }
  }
  
  // Now, Load WheelCounter Odometer data into the status block for sending to the PC
  // Flag the high bit with the direction (I don't trust negative numbers)
  // Note that "High" may mean forward or reverse, depending upon how the motors are mounted
 
  // Left Wheel
  Status.OdometerHighL = (byte)(WheelCountLeft >>8);	// Hi Byte
  if( RevLeft )
  {
    Status.OdometerHighL |= 0x80; // set the high bit
  }
  Status.OdometerLowL = (byte)( WheelCountLeft & 0xFF);		// Low Byte

  // Right Wheel
  Status.OdometerHighR = (byte)(WheelCountRight >>8);	// Hi Byte
  if( RevRight )
  {
    Status.OdometerHighR |= 0x80; // set the high bit
  }
  Status.OdometerLowR = (byte)( WheelCountRight & 0xFF);		// Low Byte
*/ 
}


/////////////////////////////////////////////////////////////////////////////////////
byte Read_Digital_Port_A()
{  
  // Read digital input port A on the Arduino board.  Mega 2560: Pin22=PA0, Pin29=PA7
  byte PortA = PINA; // PORTA;  // bulk read of the full port
  PortA ^= 0b00111111;	   // Invert Active LOW sensors (IR sensors go low when objected detected, but PIR are active high)
  return PortA;
}


/////////////////////////////////////////////////////////////////////////////////////
byte Read_Digital_Port_C()
{  
  // Read digital input port C on the Arduino board. Mega 256:  Pin30=PC7, Pin37=PC0 (note reverse order!)
  byte PortC = PINC; // PORTC;  // note reversed pin numbers on this one
  PortC ^= 0b11111111;	// Active LOW, so invert all bits (goes low when bumper hit).  Unused bits are pulled high by PORTC command above.
  return PortC;
}




/////////////////////////////////////////////////////////////////////////////////////
void Read_I2C_Digital_Ports()
{  
  // Read PCF8574 Digital IO Ports via I2C
  // TODO - need to write a test for this!
}

/////////////////////////////////////////////////////////////////////////////////////
void Read_I2C_IT_IR_Rangers()
{  
  // Read I2C-IT IR Range Sensors via I2C, up to NUM_IR3_SENSORS
  Status.IR3[0] = I2C_Read_Byte( I2C_IT_SENSOR_ADDR_0, I2C_IT_UNITS_INCHES );  // Left Hand IR sensor
  Status.IR3[1] = HW_SENSOR_NOT_ATTACHED;                // I2C_IT_SENSOR_ADDR_1, Left Arm IR sensor - Removed!
  Status.IR3[2] = I2C_Read_Byte( I2C_IT_SENSOR_ADDR_2, I2C_IT_UNITS_INCHES );  // Right Hand IR sensor
  Status.IR3[3] = HW_SENSOR_NOT_ATTACHED;                // I2C_IT_SENSOR_ADDR_3, Right Arm IR sensor - Removed!

}

/////////////////////////////////////////////////////////////////////////////////////
void Read_Hand_Pressure()
{  
  // Read Hand pressure sensors via I2C Adafruit ADS1015 4 channel A2D
  //int16_t adc0;
/*  
  int adc;

  adc = ads1015.readADC_SingleEnded(0);  
  Status.LeftHandPressureL = (byte)((adc >>3)& 0xFF);
  adc = ads1015.readADC_SingleEnded(1);  
  Status.LeftHandPressureR = (byte)((adc >>3)& 0xFF);
*/
//  adc1 = ads1015.readADC_SingleEnded(1);  
//  adc2 = ads1015.readADC_SingleEnded(2);  
//  adc3 = ads1015.readADC_SingleEnded(3);  
//  Serial.print("AIN0: "); Serial.println(adc0);
//  Serial.print("AIN1: "); Serial.println(adc1);
//  Serial.println(" ");
//  int temp0 = 0;
  //temp0 = adc0 >>3;
  //Status.LeftHandPressureR = (byte)( temp0 & 0xFF);	// Low Byte
}

/////////////////////////////////////////////////////////////////////////////////////
void Read_Next_Analog_Port()
{  
  // Read Analog Ports on the Arduino board
  // this gets called twice each 20ms, allowing all ports to be read within 100-200ms (depending upon the number of A/D ports enabled by NUM_AD_IR_SENSORS)

  static int SensorToRead = 0;
  if( SensorToRead < NUM_AD_IR_SENSORS )
  {
    Status.IR[SensorToRead] = analogRead(SensorToRead)/4; // divide by 4 to make the range 0-255  (ignore extra precision, the sensor is not that precise)
    // delay 10ms to let the ADC recover:
    //delay(10); // this delay removed, because the function is only called every so often
    SensorToRead++;
  }
/*** DISABLED for now.  Voltage is same as we see on the Arduino, and current is way to small, so very inaccurate.
 *** TODO:  Try with larger shunt resistor?

  else if( SensorToRead == (NUM_AD_IR_SENSORS) )
  {
    // Read Battery current from the shunt
    int IRaw = analogRead(6); // port 6
    int IFinal = (IRaw * 100)/149; //45 Amp board - gives Tenth Amps   
    PrintDebugDec("BATT I = ", IFinal);
    SensorToRead++;
  }
  else if( SensorToRead == (NUM_AD_IR_SENSORS+1) )
  {
    // Read Battery voltage from the shunt
    int VRaw = analogRead(7); // port 7
    int VFinal = (VRaw * 100)/494; //45 Amp board - gives Tenth Volts   
    PrintDebugDec("BATT V = ", VFinal);
    SensorToRead++;
  }
  */
  else
  {  
    // Analog port 15 is the battery voltage
    Status.Battery0 = analogRead(15)/4;
    PowerIsOn = CheckPower(Status.Battery0); // set global flag if power is not on
    if( !PowerIsOn )
    {
        AllLedsOff();  // turn everyting off!  No power!
        SetLedEyes( 0x00  );
    }
    
    
    SensorToRead = 0;
  }
}


/////////////////////////////////////////////////////////////////////////////////////
boolean CheckPower(byte BatteryByte)
{
  // See if power is turned on, returns true if on
  if( 0 == BatteryByte )
  {
    // No voltage passed in, so read the port
    BatteryByte = analogRead(15)/4;
  }
    float BatteryVoltage = ( (double)BatteryByte * 0.041 ) + 7.0;
    if( BatteryVoltage > 8.5 ) // USB only returns about 8.3v
    {
      return true; // Power is on
    }
    return false;
}


/////////////////////////////////////////////////////////////////////////////////////
void Read_Compass()
{  
  // Read Devantech CMP03 Compass via I2C
  // Result is in Tenth Degrees, so 3600 = 360.0 degrees
  // Result is returned in Status.CompassHigh/CompassLow.  To get the reading, do this:  
  // CompassReading = CompassHigh <<8;	      // Shift the high byte over
  // CompassReading = ( CompassReading + CompassLow );	
  
  const byte NumberOfBytes = 2; // Get both the High and Low bytes
  Wire.beginTransmission( I2C_COMPASS_ADDR );
  Wire.write(0x02);	// Register 2 = High Byte of 16 bit register for compass reading
  Wire.endTransmission();
  Wire.requestFrom(I2C_COMPASS_ADDR, NumberOfBytes, (byte)true); 
  delay(5);
  if( Wire.available() )
  {
    Status.CompassHigh = Wire.read();
    Status.CompassLow = Wire.read();
  }
  Wire.endTransmission();

 // TODO -DEBUG
 /*
 if( TestAndroidCmd > 1028)
 {
   TestAndroidCmd = 0;
 }
 else
 {
   TestAndroidCmd++;
 }
 */

}

/////////////////////////////////////////////////////////////////////////////////////
byte I2C_Read_Byte( int deviceaddress, int datamode ) 
{
  byte rdata = 0;
  Wire.beginTransmission(deviceaddress);
  Wire.write(datamode); 
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1, true);
  delay(5);
  if (Wire.available()) rdata = Wire.read();
  Wire.endTransmission();
  return rdata;
}

/////////////////////////////////////////////////////////////////////////////////////
void SetLedEyes( byte EyeState )
{
// I2C device #3 (in Loki's Head)
  byte PortCmd = 0;
  PortCmd = EyeState | 0b00000001; // P0 = US Read enable.  High = Enabled (TODO - Toggle as needed to coordinate with other US sensors)
  PortCmd = PortCmd  | 0b00000010; // P1 = N/C I/O port.  Keep high (floating) by default
  PortCmd = PortCmd  & 0b11111011; // P2 = N/C Transistor.  Keep low (transistor off) by default
  
  Wire.beginTransmission(I2C_PCF8574_HEAD);
  Wire.write(PortCmd);  // 
  Wire.endTransmission();
}

/////////////////////////////////////////////////////////////////////////////////////
void SetArmLED( byte DeviceAddress, boolean LedOn)
{
// I2C device in Left Arm
  byte PortCmd = 0;
  if( LedOn )  
    PortCmd = 0xFE;
  else
    PortCmd = 0xFF;
  
  Wire.beginTransmission(DeviceAddress);
  Wire.write(PortCmd); 
  Wire.endTransmission();
}

//***************************************************************************
byte ReadPCF8574(int deviceaddress)
{
  byte data = 0;
    Wire.requestFrom(deviceaddress, 1);
  delay(1);
  if (Wire.available()) {
    data = Wire.read();
  }

  return data;
}


