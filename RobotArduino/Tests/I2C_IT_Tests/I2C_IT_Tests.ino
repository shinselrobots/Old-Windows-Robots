/* I2C-It for the Arduino / Freeduino SB
 * Sample Code release April 15, 2008
 * 
 * The I2C-It is an I2C / TWI networked distance sensor based on the 
 * popular Sharp GP2D12 sensor. It features:
 * 	- User-selectable sensor address (0x20 to 0x2E)
 * 	- 8-devices per chain (more, if you get additional sensor addresses 
 * 		in firmware when ordering)
 * 	- 10-80cm trange (2.5" to 31.5")
 * 	- Request data measured back in cm, inches, or raw 0-254
 *  Data format 1 returns inches
 *  Data format 2 returns centimeters
 *  Data format 3 returns raw 0-254 from the sensor
 * 
 * See more information at http://www.hvwtech.com!
 *
 * Notes: 
 *	- Arduino analog input 4 is I2C SDA
 *	- Arduino analog input 5 is I2C SCL
 *      - *** When using the 11" I2C-It cables, stronger 4.7k pullup resistors 
 *         (Vcc to A4, Vcc to A5) are required***
 *
 *  Some applications (like the I2C Nunchuck adapter) set A2 as digital Gnd 
 *  and A3 as digital Vcc to power the I2C accessory. The I2C-It draws too much
 *  power to use these lines in this manner. Run appropriate jumpers!
 *
 *  This program reads the distance parameters back from the I2C-It, and reports it
 *  back to the serial terminal (running at 19200 baud).
 *  If the value read is less than nine, blink the "close!" lamp, before reporting back.
 */
#include <Wire.h>

int ledPin = 13;                // LED connected to digital pin 13
int address = 0x10;
int dist = 0;

//	Reads HVW "I2C-IT"IR Range Sensor via I2C.
// Tell the I2C-It Sensor what Units to use for return values
#define I2C_IT_UNITS_INCHES         1
#define I2C_IT_UNITS_CENTEMETERS    2
#define I2C_IT_UNITS_RAW            3


/**********************************
 * // I2C-IT IR Rangers
 * // I2C-IT Address range from 0x20 to 0x2C or 0x40 to 0x4C (every 4 bits) -- PIC
 * // I2C Write = base address.  I2C Read = base address + 1
 * #define I2C_IT_SENSOR_ADDR_0	0x20
 * #define I2C_IT_SENSOR_ADDR_1	0x24
 * #define I2C_IT_SENSOR_ADDR_2	0x28
 * #define I2C_IT_SENSOR_ADDR_3	0x2C
 * #define I2C_IT_SENSOR_ADDR_4	0x40
 * //#define I2C_IT_SENSOR_ADDR_5	0x44 - In Use!  Change chip in Loki's head!
 * #define I2C_IT_SENSOR_ADDR_6	0x48
 * //#define I2C_IT_SENSOR_ADDR_7	0x4C - In Use (blocked)
 **********************************/
#define I2C_IT_SENSOR_ADDR_0	0x10 // Left Hand
#define I2C_IT_SENSOR_ADDR_1	0x12 // N/C
#define I2C_IT_SENSOR_ADDR_2	0x14 // Right Hand
#define I2C_IT_SENSOR_ADDR_3	0x16 // N/C
#define I2C_IT_SENSOR_ADDR_4	0x20 // N/C
//#define I2C_IT_SENSOR_ADDR_5	0x22 - In Use!  Change chip in Loki's head!
#define I2C_IT_SENSOR_ADDR_6	0x24 // N/C
//#define I2C_IT_SENSOR_ADDR_7	0x26 - In Use (blocked)


void setup()
{
  Wire.begin();                 // join I2C network as master
  Serial.begin(19200);           // configure serial monitor at 19200 baud
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  Serial.print ("Finished setup\n");
  delay (200);                  // Wait for 1/5 of a second to make sure I2C-It is booted & ready
}

void loop(){
  digitalWrite(13, HIGH);   // set the LED on 
  delay (50);                  // Wait for 1/5 of a second to make sure I2C-It is booted & ready

Serial.print ("Left Hand: ");
  Read_I2C_IT(I2C_IT_SENSOR_ADDR_0);
  
Serial.print ("NC: ");
  Read_I2C_IT(I2C_IT_SENSOR_ADDR_1);
Serial.print ("Right Hand: ");
  Read_I2C_IT(I2C_IT_SENSOR_ADDR_2);
Serial.print ("NC: ");
  Read_I2C_IT(I2C_IT_SENSOR_ADDR_3);

  Serial.println(); 


  digitalWrite(13, LOW);   // set the LED off
  delay(50);

}



//***************************************************************************
void Read_I2C_IT(int addr)
{

  dist = i2c_eeprom_read_byte( addr, I2C_IT_UNITS_INCHES );    // read input value from address "0x20" (default I2C-It address) with data format 1
  Serial.print ("0x"); 
  Serial.print(addr, HEX);
  Serial.print(" = ");
  Serial.print(dist, DEC);               // Report the distance back to the terminal 
  Serial.print("in    ");

}

//***************************************************************************
byte i2c_eeprom_read_byte( int deviceaddress, int datamode ) {
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

