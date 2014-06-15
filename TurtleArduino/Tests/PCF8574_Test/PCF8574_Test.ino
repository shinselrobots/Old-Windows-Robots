// Tests for verifying Loki Robot Arduino interface board I2C expansion chips
// Blink LEDS in the ARMS and robot EYES

#include <Wire.h>

#define LEFT_ARM_PCF8574   0x21 //PCF8574 
#define HEAD_PCF8574       0x22 //PCF8574 
#define RIGHT_ARM_PCF8574  0x23 //PCF8574 

//#define LED_PIN    13
const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)

void setup(void) 
{ 
  Serial.begin(19200); 
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(ledPin, OUTPUT);      
  Serial.println("Init Complete.");
}

void loop(void) 
{

  // Blink Arm LEDS
  Wire.beginTransmission(LEFT_ARM_PCF8574); 
  Wire.write(byte(0xFE));  // 0xFE = LED ON  
  Wire.endTransmission();

  Wire.beginTransmission(RIGHT_ARM_PCF8574);
  Wire.write(byte(0xFE));  // 0xFE = LED ON
  Wire.endTransmission();
  
  Wire.beginTransmission(HEAD_PCF8574);
  Wire.write(byte(0x00));  // 
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(RIGHT_ARM_PCF8574);
  Wire.write(byte(0xFF));  
  Wire.endTransmission();

  Wire.beginTransmission(LEFT_ARM_PCF8574);
  Wire.write(byte(0xFF)); 
  Wire.endTransmission();
  
  Wire.beginTransmission(HEAD_PCF8574); // HEAD LEDS!
  Wire.write(byte(0xFF));  //
  Wire.endTransmission();
  delay(50);


  // Blink the LED 
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;   
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(ledPin, ledState);
  }
  
  delay(100);
}



