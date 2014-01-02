// Test for I2C interface to Devantech CMP03 Compass
#include <Wire.h>

const int ledPin = 13;           // LED connected to digital pin 13
const int addr = 0x60;           // 0xC0 on PIC, shifted by one bit for Arduino

void setup()                  
{
  Wire.begin();                  // join I2C network as master
  Serial.begin(19200);           // configure serial monitor at 19200 baud
  pinMode(ledPin, OUTPUT);       // set the LED pin as output
  delay (200);                   // Wait to make sure Compass is ready
  Serial.print ("Finished setup\n");
}

void loop(){
  digitalWrite(13, HIGH);   // set the LED on 

  int CompassReading = 0;
  byte TempCompassHigh = 0;
  byte TempCompassLow = 0;

  Wire.beginTransmission(addr);
  Wire.write(0x02);	// Register 2 = High Byte of 16 bit register for compass reading
  Wire.endTransmission();
  Wire.requestFrom(addr,2, true);
  delay(5);
  if (Wire.available())
  {
    TempCompassHigh = Wire.read();
    TempCompassLow = Wire.read();
  }
  Wire.endTransmission();
  CompassReading = TempCompassHigh <<8;	// Shift the high byte over
  CompassReading = ( CompassReading + TempCompassLow );	// 3600--> 360.0

float CompassFloat = (float)CompassReading / 10.0;
  Serial.print("Compass Degrees: ");
  Serial.println(CompassFloat, 1);               // Report the compass reading back to the terminal 

  digitalWrite(13, LOW);   // set the LED off
  delay(100); 
}


