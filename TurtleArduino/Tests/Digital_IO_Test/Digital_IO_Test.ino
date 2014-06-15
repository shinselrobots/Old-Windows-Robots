// Tests for verifying Loki Robot Arduino interface board
// Prints out anytime a value changes on Port A or C


const int ledPin =  13;      // the number of the LED pin
const int BlueLedPin = 12;

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 100;           // interval at which to blink (milliseconds)

byte PortA = 0;
byte PortC = 0;
byte LastPortA = 0;
byte LastPortC = 0;

boolean relaystate = false;  

void setup(void) 
{ 
  Serial.begin(19200); 
  pinMode(ledPin, OUTPUT);   
  pinMode(BlueLedPin, OUTPUT);   
  

  DDRA = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  DDRC = 0x00; // same as B00000000;  // sets Arduino Port C [pins7...0], 1=output, 0=input
  PORTA = 0xFF; // Set pullup resistors on to avoid noise on unused pins
  PORTC = 0x0F; // Set pullup resistors on to avoid noise on unused pins
  
  // Setup Relay contol pins
  pinMode(8, OUTPUT);      
  pinMode(9, OUTPUT);      
  pinMode(10, OUTPUT);      
  pinMode(11, OUTPUT);  
  digitalWrite(8, false);
  digitalWrite(9, false);
  digitalWrite(10, false);
  digitalWrite(11, false);
  
 
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


// test Relay control (Note: increase delay below when testing relays, so you dont burn the relay out)
  digitalWrite(8, relaystate);  // black wire
  digitalWrite(9, relaystate);  // white wire
//  digitalWrite(10, relaystate); // 18v Servo 
//  digitalWrite(11, relaystate); // 12v Servo
  relaystate = !relaystate;
  



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
    previousMillis = currentMillis;   
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(ledPin, ledState);
    digitalWrite(BlueLedPin, ledState);
  }
 
  delay(3000);
  delay(100);
}


