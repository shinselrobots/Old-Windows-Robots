// Tests for verifying Loki Robot Arduino interface board

//#include <MeetAndroid.h>
//MeetAndroid meetAndroid;
const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)
int incomingByte = 0;   // for incoming serial data

void setup(void) 
{ 
  Serial.begin(19200); 
  //Serial1.begin(9600); 
  Serial1.begin(115200); // bluetooth connection with Android phone 
 // meetAndroid.registerFunction(testEvent, 'A');
  pinMode(ledPin, OUTPUT);      

  Serial.println("Init Complete.");
  Serial1.println("Init Complete.");
}

void loop(void) 
{
  //Serial.println(".");

        if (Serial1.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial1.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
        }
        
        Serial1.println("test");        


//  meetAndroid.receive(); // you need to keep this in your loop() to receive events
 
//  int BatteryInt = analogRead(15);
//  delay(20);
//  byte BatteryByte = analogRead(15)/4;
  
//  float BatteryVoltage = ( (double)BatteryByte * 0.041 ) + 7.0;
    
  //Battery = analogRead(15)/4;
/*
Serial.print("Battery: Byte: ");
  Serial.print(BatteryByte, DEC);
  Serial.print("  Int: ");
  Serial.print(BatteryInt, DEC);
  Serial.print("  Voltage: ");
  Serial.println(BatteryVoltage, 2);
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

/*
 * This method is called each time msg received from Bluetooth.
 * note: flag is in this case 'A' and numOfValues is 0 (since test event doesn't send any data)
 */
/*
void testEvent(byte flag, byte numOfValues)
{
  Serial.print("Bluetooth Value: ");
  Serial.println(numOfValues, DEC);
}
*/
