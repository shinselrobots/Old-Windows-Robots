// Tests for verifying Loki Robot Arduino interface board

#include <FreqPeriodCounter.h>
#include <Wire.h>
#include "PCF8574.h" // PCF8574 I2C Digital IO Expander Chip
#define EXPANDER_A B00100000 //0x20 //PCF8574 
#define EXPANDER_B B00100001 //0x21 //PCF8574 

#define LEFT_ARM_PCF8574   0x21 //PCF8574 
#define HEAD_PCF8574       0x22 //PCF8574 
#define RIGHT_ARM_PCF8574  0x23 //PCF8574 

/*
// Third PCF8574A I/O expander chip I2C Address: 0x44 (In Loki's Head) ==> 0x22 for Arduino
// (Address line 3 low, 2 high, 1 low)
#define   I2C_PCF8574_3_WRITE  0b01000100	// 1100 = Address + R/W bit, 0=Write (0x44) - TODO change 4A to allow I2C-IT!
#define   I2C_PCF8574_3_READ   0b01000101	// 1101 = Address + R/W bit, 1=Read  (0x45)
//                                   ^^^	selectable address bits

// Fourth PCF8574A I/O expander chip I2C Address: 0x42 (In Loki's Left Arm) ==> 0x21 for Arduino
// (Address line 3 low, 2 low, 1 high)
#define   I2C_PCF8574_4_WRITE  0b01000010	// 1100 = Address + R/W bit, 0=Write (0x42)
#define   I2C_PCF8574_4_READ   0b01000011	// 1101 = Address + R/W bit, 1=Read  (0x43)
//                                   ^^^	selectable address bits

// Fifth PCF8574A I/O expander chip I2C Address: 0x46 (In Loki's Right Arm) ==> 0x23 for Arduino
// (Address line 3 low, 2 high, 1 high)
#define   I2C_PCF8574_5_WRITE  0b01000110	// 1100 = Address + R/W bit, 0=Write (0x46)
#define   I2C_PCF8574_5_READ   0b01000111	// 1101 = Address + R/W bit, 1=Read  (0x47)
//                                   ^^^	selectable address bits
*/

// Interrupt pins for the Mega 2560
#define INT_0       0
#define INT_1       1
#define INT_0_PIN   2  // Right Wheel Speed
#define INT_1_PIN   3  // Left Wheel Speed
#define DIR_0_PIN   4  // Right Wheel Direction
#define DIR_1_PIN   5  // Left Wheel Direction

//#define LED_PIN    13
const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)
int Direction0 = 0;
int Direction1 = 0;

byte PortA = 0;
byte PortC = 0;
byte LastPortA = 0;
byte LastPortC = 0;

//const byte counterPin = 3; 
const byte counterInterrupt1 = 1; // = pin 3
const byte counterInterrupt2 = 2; // = pin 3
//FreqPeriodCounter counter(counterPin, micros, 0);

// Instantiate the library counter class for correct pin,
// microseconds or miliseconds, and debounce time
FreqPeriodCounter counter0(INT_0_PIN, micros, 0);
FreqPeriodCounter counter1(INT_1_PIN, micros, 0);

void setup(void) 
{ 
  Serial.begin(19200); 
  PCF8574.Set(RIGHT_ARM_PCF8574, 0x7F);  // PCF8574 I2C Digital IO Expander Chip - set all bits high
  Wire.begin(); // join i2c bus (address optional for master)

  attachInterrupt(INT_0, counterISR0, CHANGE);
  attachInterrupt(INT_1, counterISR1, CHANGE);
  pinMode(ledPin, OUTPUT);      
  pinMode(DIR_0_PIN, INPUT);      
  pinMode(DIR_1_PIN, INPUT);      

  DDRA = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  DDRC = 0x00; // same as B00000000;  // sets Arduino Port C [pins7...0], 1=output, 0=input
  PORTA = 0xFF; // Set pullup resistors on to avoid noise on unused pins
  PORTC = 0x0F; // Set pullup resistors on to avoid noise on unused pins
 
//  pinMode(37, INPUT); 
// digitalWrite(37, HIGH);       // turn on pullup resistors 



  PortA = PINA; // PORTA;  // bulk read of the full port
  PortC = PINC; // PORTC;  // note reversed pin numbers on this one
    Serial.print("Port A Start: ");
    Serial.println(PortA, BIN);
    LastPortA = PortA;

    Serial.print("Port C Start: ");
    Serial.println(PortC, BIN);
    LastPortC = PortC;

  Serial.println("Init Complete.");
}

void loop(void) 
{

  // SEE http://arduino.cc/forum/index.php?topic=109340.0 for PCF Libaray
  
  byte PCFpin = 0; // pin 0 - 7
  //WriteI2C_Port(I2C_PCF8574_4_WRITE, 0xFE);		// Right Arm Led On
  //WriteI2C_Port(I2C_PCF8574_5_WRITE, 0xFE);		// Left Arm Led On
  //WriteI2C_Port(I2C_PCF8574_4_WRITE, 0xFF);		// Right Arm Led Off
  //WriteI2C_Port(I2C_PCF8574_5_WRITE, 0xFF);		// Left Arm Led Off
  
  // "IO should be HI before being used as inputs" - from data sheet
/*  
  PCF8574.Prox(RIGHT_ARM_PCF8574, PCFpin, LOW);  //this turns ON bit 1 on your primary expander A 
  PCF8574.Prox(LEFT_ARM_PCF8574, PCFpin, LOW);  //this turns ON bit 1 on your primary expander A 
  delay(250);
  PCF8574.Prox(RIGHT_ARM_PCF8574, PCFpin, HIGH);  //this turns OFF bit 1 on your primary expander A 
  PCF8574.Prox(LEFT_ARM_PCF8574, PCFpin, HIGH);  //this turns OFF bit 1 on your primary expander A 
 */


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
  
  Wire.beginTransmission(HEAD_PCF8574);
  Wire.write(byte(0xFF));  //
  Wire.endTransmission();
  delay(50);



  
  
/*
  if(counter0.ready())
  {
    Direction0 = digitalRead(DIR_0_PIN);    
    Serial.print("Counter 0-R: ");
    Serial.print(counter0.hertz(), DEC);
    Serial.print(" Dir: ");
    Serial.println(Direction0, DEC);
  }

  if(counter1.ready())
  {
    Direction1 = digitalRead(DIR_1_PIN);    
    Serial.print("Counter 1-L: ");
    Serial.print(counter1.hertz(), DEC);
    Serial.print(" Dir: ");
    Serial.println(Direction1, DEC);
  }
*/
  
  // Read input Ports
  PortA = PINA; // PORTA;  // bulk read of the full port
  PortA ^= 0b11111111;	// Active LOW, so invert all bits (goes low when bumper hit)
  PortC = PINC; // PORTC;  // note reversed pin numbers on this one
  PortC ^= 0b11111111;	// Active LOW, so invert all bits (goes low when bumper hit)

  if( PortA != LastPortA  )
  {
    Serial.print("Port A Update: ");
    Serial.print(PortA, HEX);
    Serial.print("  ");
    Serial.println(PortA, BIN);
    LastPortA = PortA;
  }
  
  if( PortC != LastPortC  )
  {
    Serial.print("Port C Update: ");
    Serial.print(PortC, HEX);
    LastPortC = PortC;
    Serial.print("  ");
    Serial.println(PortC, BIN);
  }

/*
  int BatteryInt = analogRead(15);
  delay(20);
  byte BatteryByte = analogRead(15)/4;
  
  float BatteryVoltage = ( (double)BatteryByte * 0.041 ) + 7.0;
    
  //Battery = analogRead(15)/4;
  Serial.print("Battery: Byte: ");
  Serial.print(BatteryByte, DEC);
  Serial.print("  Int: ");
  Serial.print(BatteryInt, DEC);
  Serial.print("  Voltage: ");
  Serial.println(BatteryVoltage, 2);
*/

  /*
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
   delay(100);
   digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
   delay(100);
   */

  // Blink the LED 
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  
  delay(100);
}

void counterISR0()
{ 
  counter0.poll();
}
void counterISR1()
{ 
  counter1.poll();
}



