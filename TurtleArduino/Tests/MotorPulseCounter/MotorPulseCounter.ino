#include <FreqPeriodCounter.h>

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
  attachInterrupt(INT_0, counterISR0, CHANGE);
  attachInterrupt(INT_1, counterISR1, CHANGE);
  pinMode(ledPin, OUTPUT);      
  pinMode(DIR_0_PIN, INPUT);      
  pinMode(DIR_1_PIN, INPUT);      
  Serial.println("Init Complete.");

}

void loop(void) 
{
  boolean RevL = false;
  boolean RevR = false;


  if(counter0.ready())
  {
    Serial.print("Counter 0-R: ");
    Serial.print(counter0.hertz(), DEC);
    Serial.print(" Dir: ");
    if( HIGH == digitalRead(DIR_0_PIN) )
    {
      RevR = true;
      Serial.println("REV");
    }
    else
    {
      Serial.println("FWD");
    }
  }

  if(counter1.ready())
  {
    Serial.print("Counter 1-L: ");
    Serial.print(counter1.hertz(), DEC);
    Serial.print(" Dir: ");
    if( LOW == digitalRead(DIR_1_PIN) )
    {
      RevL = true;
      Serial.println("REV");
    }
    else
    {
      Serial.println("FWD");
    }
  }

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
}

void counterISR0()
{ 
  counter0.poll();
}
void counterISR1()
{ 
  counter1.poll();
}



