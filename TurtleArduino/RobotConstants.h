/////////////////////////////////////////////////////////////////////////////////////
//  Robot Constants
/////////////////////////////////////////////////////////////////////////////////////

/*  General and Hardware Comments
 *  When using long wires with I2C, strong pull-up resistors required! 1k - 4.7k
 * I2C-IT sensors are available from Solarbotics
 */
 
#define BYTE byte // Needed for the included PC C++ header files
#define SYNC_CHAR        "ZZ"
#define LOOP_TIME_MS      20  // main loop should always take about this long (milliseconds)
//#define BINARY(a,b,c,d,e,f,g,h)  (a<<7|b<<6|c<<5|d<<4|e<<3|f<<2|g<<1|h)

////////////////////////////////////////////////////////////////
// Hardware Pins
const int    INT_0 = 0;  // Interrupt pins for the Mega 2560, used for wheel odometer
const int    INT_1 = 1;
const int    INT_0_PIN =           2;   // Right Wheel Speed
const int    INT_1_PIN =           3;   // Left Wheel Speed
const int    DIR_0_PIN =           4;   // Right Wheel Direction
const int    DIR_1_PIN =           5;   // Left Wheel Direction

const int    AUX_LIGHT_PIN_0 =     8;   // N/C - goes to relay control board
const int    AUX_LIGHT_PIN_1 =     9;   // Blue aux lights
const int    AUX_LIGHT_PIN_2 =    10;   // 18v Servo power for RX64
//const int    AUX_LIGHT_PIN_3 =    11;   // 12v Servo power for all other servos

const int    HEARTBEAT_LED_PIN =  13;  // Red Heartbeat Led
//const int    STATUS_LED_PIN2 =    12;  // Not connected
const int    STATUS_LED_PIN =     11;  // Red Status to PC indicator

const int    NUM_AD_IR_SENSORS =   6;  // Plugged into the Arduino Analog port AD0-7    
////////////////////////////////////////////////////////////////
// I2C Address Space Mapping

// I2C-IT IR Rangers (requires 2 consecutive addresses)
const byte   I2C_IT_SENSOR_ADDR_0 =   0x10; // Left Hand IR Range
const byte   I2C_IT_SENSOR_ADDR_1 =   0x12; // N/C
const byte   I2C_IT_SENSOR_ADDR_2 =   0x14; // Right Hand IR Range
const byte   I2C_IT_SENSOR_ADDR_3 =   0x16; // N/C
const byte   I2C_IT_SENSOR_ADDR_4 =   0x20; // N/C
//const byte I2C_IT_SENSOR_ADDR_5 =   0x22; - In Use!
const byte   I2C_IT_SENSOR_ADDR_6 =   0x24; // N/C
//const byte I2C_IT_SENSOR_ADDR_7 =   0x26; - In Use (blocked)

const byte   I2C_PCF8574_LEFT_ARM =   0x21; // Left Arm Digital I/O
const byte   I2C_PCF8574_HEAD =       0x22; // Head Digital I/O
const byte   I2C_PCF8574_RIGHT_ARM =  0x23; // Right Arm Digital I/O

const byte   I2C_COMPASS_ADDR =       0x60; // Devantech CMPS03 Compass



////////////////////////////////////////////////////////////////
//	Reads HVW "I2C-IT"IR Range Sensor via I2C.
// Tell the I2C-It Sensor what Units to use for return values
const byte   I2C_IT_UNITS_INCHES =        1;
const byte   I2C_IT_UNITS_CENTEMETERS =   2;
const byte   I2C_IT_UNITS_RAW =           3;


////////////////////////////////////////////////////////////////
// Other constants
const long   BlinkInterval =   600; // interval at which to blink the LED (milliseconds)
const long   StatusInterval =  333; // interval at which to send status to the PC (milliseconds)

