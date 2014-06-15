#include <Wire.h> // for I2C
#include <FreqPeriodCounter.h> // For wheel odometer pulse monitoring
#include "RobotConstants.h"
#include "C:\Dev\Robots\Common\HardwareCmds.h"
#include "C:\Dev\Robots\Common\HWInterfaceLokiArduino.h"
#include <MeetAndroid.h>
#include <Adafruit_ADS1015.h>

/////////////////////////////////////////////////////////////////////////////////////
// Global Variables
MeetAndroid      meetAndroid; // For Android Bluetooth connection
int              gLedEyeMode = LED_EYES_CLOSE; // Start with eyes closed
int              gEyeState = 0;
boolean          PowerIsOn = false; // Keeps board from running on USB power; requres external power to be turned on
int              TestAndroidCmd = 0;
// Status and Command buffers for communicating with the PC
ARDUINO_STATUS_T Status;
byte            *pStatusBuf; 
int              StatusSize = 0;
ARDUINO_CMD_T    gPicCmdBuffer;	// Used for incoming characters

// Instantiate the library counter class for the pin, micro/miliseconds, and debounce time
FreqPeriodCounter Counter0(INT_0_PIN, micros, 0);  // Right Wheel
FreqPeriodCounter Counter1(INT_1_PIN, micros, 0);  // Left Wheel

Adafruit_ADS1015 ads1015;

/////////////////////////////////////////////////////////////////////////////////////
//                            SETUP
/////////////////////////////////////////////////////////////////////////////////////
void setup(void) 
{ 
  Serial.begin(19200);   // communication with the PC
  Serial1.begin(115200); // bluetooth connection with Android phone
  Wire.begin();           // For I2C
  
  attachInterrupt(INT_0, CounterISR0, CHANGE);
  attachInterrupt(INT_1, CounterISR1, CHANGE);
  
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);      
  pinMode(STATUS_LED_PIN, OUTPUT);      
  pinMode(DIR_0_PIN, INPUT);      
  pinMode(DIR_1_PIN, INPUT); 

  // Setup Relay contol pins
  pinMode(RELAY_BOARD_PIN_0, OUTPUT);      
  pinMode(AUX_LIGHT_PIN, OUTPUT);      
  pinMode(SERVO_PWR_18V_PIN, OUTPUT);      
  pinMode(SERVO_PWR_12V_PIN, OUTPUT);  
  digitalWrite(RELAY_BOARD_PIN_0, false);  // N/C
  digitalWrite(AUX_LIGHT_PIN, false);      // Blue lights off by default
  digitalWrite(SERVO_PWR_18V_PIN, true);   // Servo Power ON by default, FOR NOW!
  digitalWrite(SERVO_PWR_12V_PIN, true);

  // Bluetooth using Amarilo - register callback functions for Bluetooth
  meetAndroid.registerFunction(ConnectionEvent, 'C');  
  meetAndroid.registerFunction(EnableAccelerometerEvent, 'E');  
  meetAndroid.registerFunction(eXecuteCmdEvent, 'X');  
  meetAndroid.registerFunction(AccelerometerEvent, 'A');  
 

  // Fast access for general I/O pins.  Set Ports A and C as inputs
  // Port A = Pins 22-29.  Port C = Pins 37 - 30 (reverse order)
  // Port C hardware bumpers are pulled low when activated
  // DDRD = DDRD | B11111100; // example for setting selected pins only
  // PORTx = B10101000; // sets digital pins 7,5,3 HIGH.  PORTx can be used for read or write
  // PINx // Read port at once. read only.
  DDRA = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  DDRC = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  PORTA = 0xFF; // Set pullup resistors on to avoid noise on unused pins
  PORTC = 0x0F; // Set pullup resistors on to avoid noise on unused pins
 
  StatusSize = sizeof(Status);
  pStatusBuf = (byte*)&Status;
  // Initialize all Status info to 0 by default
  for(int i=0; i< StatusSize; i++ )  { 
    pStatusBuf[i] = 0;
  }
  
// Blink the lights to show the board is booting up
  for( int i = 0; i < 10; i++ )
  {
    AllLedsOn();
    delay(10);
    AllLedsOff();
    delay(50);
  }  

} // End of setup


/////////////////////////////////////////////////////////////////////////////////////
//                            LOOP
/////////////////////////////////////////////////////////////////////////////////////
void loop(void) 
{

  meetAndroid.receive(); // check for Bluetooth events
  
  // See if the board power is enabled, or just running off the PC
  if( !PowerIsOn )
  {
    if( CheckPower(0) )
    {
      // Power has been restored!
      PowerIsOn = true;
    }
    else
    {
       // Power is still off, don't blink any LEDs, just sit quiet
       delay(500);
      return;
    }
  }
  
  
  //static unsigned long LastLoopTime = 0;
  unsigned long LoopStartTime = millis();
  
  // Turn off LEDs (they just blink on for 20ms)
  AllLedsOff();
  
  // See if a command from the PC is pending
  if( CheckForSerialData() )
  {
     //PrintDebugHex("Got Command: ", gPicCmdBuffer.Cmd); // print the command number
     HandleCmdFromPC();
  }

  // Check to see if it's time to send status (if status updates are enabled by the PC)
  if( TimeToSendStatus() )
  {
    digitalWrite(STATUS_LED_PIN, LOW);   // Active Low
    SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
    SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );
    ReadSensors();
    SendStatusToPC();   
  }

  // Update Robot Eyes state (gradual open or close for blinking)
  BlinkEyes();
  
  // Blink Heartbeat LED if it's time
  if( TimeToBlinkHeartBeat() )
  {
    digitalWrite(HEARTBEAT_LED_PIN, HIGH); // Active High
  }

  // keep loop to a tight 20ms
  int LoopTime = millis() - LoopStartTime;

  if( LoopTime > LOOP_TIME_MS )
  {
    // Enable this to test/assure that the loop completes in less than 20ms
    // Disabled for now becuase it reports a lot of 21/22 - need to fix this
    // PrintDebugDec("LoopTime = ", LoopTime);
  }
  else
  {
     while( LoopTime < (LOOP_TIME_MS - 5) )
     {
      // read sensors instead of sitting idle
      Read_Next_Analog_Port();    // read an analog port (need time between reads)
      delay(2);
      LoopTime = millis() - LoopStartTime; // see how much time is still left       
     }
     delay( LOOP_TIME_MS - LoopTime );
  }

} // End of Looop






