// HardwareCmds.h
// Command interface between the PC and Microcontroller, and is shared by both compilers
// Make sure syntax works with all compilers needed
#ifndef __ROBOT_HW_COMMANDS_H__
#define __ROBOT_HW_COMMANDS_H__

//#include "../Loki/RobotType.h"   // Change this for different robots used


//////////////////////////////////////////////////////////////////////////////////////////
// Hardware Command IDs to the Arduino			CMD_ID	PARAMETER1		PARAMETER2
#define HW_NO_CMD_PENDING					0x00 // Used host code to track flag that no commands are pending
#define HW_GET_STATUS						0x01 //	-				-  Arduino sends regular updates when this is true, and stops when this is false
#define HW_SET_MOTOR_STOP					0x02 //	-				Optional: Acceleration  (currently Kobuki Only?) 
#define HW_UPDATE_MOTOR_SPEED_CONTROL		0x03 // -				-  To Motor Control: evaluate current wheel speed and send update cmds as needed

#define HW_GET_VERSION						0x05 //	-				-
#define HW_RESET_CPU						0x06 //	-				-
#define HW_KEEP_ALIVE						0x07 //	-				-
#define HW_GET_RADAR_SCAN_DATA				0x08 //	-				-
#define HW_INITIALIZE						0x09 //	-				-
#define HW_SET_SPEED_AND_TURN				0x0A //	Speed			Turn	(in servo values) NOTE: To set Acc, use separate speed and turn commands
#define HW_SET_TURN							0x0B //	Turn value		Optional: Acceleration  (currently Kobuki Only?)
//#define HW_SET_SPEED						0x0C // Speed value		Optional: Acceleration  (currently Kobuki Only?)
#define HW_CLEAR_ERROR						0x0D //	-				-
#define HW_PAUSE_MOTION						0x0E //	True/False		-	// Handle Global Pause mode

#define HW_SET_POWER_MODE					0x10 //	Servo Power		Laptop Power -	Sets power of servos, laptop and Arduino!
#define HW_SET_SERVO_POWER					0x11 //	-				On/Off
#define HW_SET_LIGHT_POWER					0x12 //	Camera Lights	On/Off
#define HW_SET_AUX_LIGHT_POWER				0x13 // Aux/Running Lts On/Off
#define HW_RESET_ODOMETER					0x14 //	-				-
#define HW_SET_LED_EYES						0x15 // Off/On/Blink	Future: brightness
#define HW_RESET_WATCHDOG					0x16 //	-				-
#define HW_SET_MOTOR_BRAKE					0x17 //	Brake Value		Brake Time			-
#define HW_COMPASS_CAL_MODE					0x18 // -				-
#define HW_COMPASS_CAL_POINT				0x19 // -				-
#define HW_INITIALIZE_SERVO					0x1A

#define HW_ENABLE_RADAR_SCAN				0x1B //	-				Off/On
#define HW_ENABLE_SPEED_CONTROL				0x1C // -				Off/On
//#define HW_SET_DYNA_USB						0x1D // -				Off/On
#define HW_SET_IR_SENSOR_POWER				0x1E // L/Right (TODO)	Off/On
#define HW_SET_KINECT_POWER					0x1F // -				On/Off

#define HW_CAMERA_MIN_COMMAND				0x20 // For sending to the right thread!
#define HW_CAMERA_INITIALIZE				0x20 //	Pan/Tilt Dir	Pan/Tilt Speed
#define HW_SET_CAMERA_PAN_TILT				0x21 //	Pan/Tilt Dir	Pan/Tilt Speed
#define HW_SET_CAMERA_PAN_ABS				0x22 //	Servo Position	-
#define HW_SET_CAMERA_TILT_ABS				0x23 //	Servo Position	-
#define HW_SET_CAMERA_PAN_TILT_ABS			0x24 // Pan Position	Tilt Position
#define HW_SET_CAMERA_ZOOM					0x25 //	Zoom In/Out		Zoom Speed
#define HW_SET_CAMERA_POWER					0x26 //	On/Off			-
#define HW_SET_CAMERA_ZOOM_ABS				0x27 // Zoom Level (range 0 to 64)
#define HW_SET_CAMERA_ABS_PAN_TILT_SPEED	0x28 // Speed (0-22)
#define HW_SET_CAMERA_MODE					0x29 // Mode			Value
#define HW_SET_CAMERA_SIDETILT_ABS			0x2A // Position 		-
#define HW_SET_CAMERA_FORWARD_ABS			0x2B
#define HW_GET_CAMERA_BULK_SERVO_STATUS		0x2C //	Just dumps status to the log
#define HW_SET_BULK_HEAD_POSITION			0x2D // Uses global g_DynaServoCmd[] buffer
#define HW_SET_BULK_KINECT_POSITION			0x2E // Uses global g_DynaServoCmd[] buffer
#define HW_SET_CAMERA_STOP					0x2F // Stop current head motion		-

#define HW_CAMERA_MAX_COMMAND				0x2F // For sending to the right thread!

#define HW_OTHER_SERVO_MIN_COMMAND			0x30 // For sending to the right thread!
#define HW_SET_GEAR							0x31 //	-				Servo Position (LOW_GEAR or HIGH_GEAR)
#define HW_SET_SERVO_POS_TICKS				0x32 //	Servo			Positon (SERVO_MIN to SERVO_MAX)
#define HW_SET_SERVO_POS_TENTHDEGREES		0x33 //	Servo			Positon (SERVO_MIN to SERVO_MAX)
#define HW_GET_SERVO_STATUS					0x34 //	Servo			Current Status
#define HW_READ_SERVO_REGISTER				0x35 //	Servo			Current Status
#define HW_WRITE_SERVO_REGISTER_BYTE		0x36 //	Servo			Current Status
#define HW_WRITE_SERVO_REGISTER_WORD		0x37 //	Servo			Current Status

#define HW_SET_SERVO_TORQUE_ENABLE			0x38 //	Which Arm.  Uses global g_DynaServoCmd[] buffer
#define HW_SET_SERVO_TORQUE_LIMIT			0x39 //	Arm/Joint		Value (0 - 1023) see DYNA_TORQUE_LIMIT_MAX
#define HW_SET_BULK_ARM_POSITION			0x3A //	Which Arm.  Uses global g_DynaServoCmd[] buffer
#define HW_GET_SMART_SERVO_STATUS			0x3B //	Servo Group.		Return uses global g_DynaServoCmd[] buffer
#define HW_SET_SERVO_COMPLIANCE				0x3C // Servo			Compliance Slope see DYNA_COMPLIANCE_TIGHT

#define HW_OTHER_SERVO_MAX_COMMAND			0x3D // For sending to the right thread!


#define HW_ARDUINO_MAX_MESSAGE					0x3D// Max message for the Arduino
// Other Hardware messages (such as serial Port messages) go here!

#define HW_SET_LASER_SCANNER_POWER			0x3E //	N/A.			1=On/0=Off
#define HW_LASER_REQUEST_SCAN				0x3F // N/A.			Number of Scans to do

#define HW_IROBOT_UNDOCK					0x40
#define HW_IROBOT_DOCK						0x41

#define HW_MAX_MESSAGE						0x42	// Maximum number a message can be (for error trapping)
//////////////////////////////////////////////////////////////////////////////////////////

// Status Flags from the Arduino (StatusFlags)
#define HW_STATUS_WATCHDOG_EXPIRED			0x01
#define HW_STATUS_POWER_ON_INDICATOR		0x02
#define HW_STATUS_MOVE_DISTANCE_COMPLETE	0x04
#define HW_STATUS_BRAKE_COMPLETE			0x08
#define HW_STATUS_RADAR_SCAN_READY			0x10
#define HW_STATUS_RC_BUTTON_PWR_ENABLE		0x20
#define HW_STATUS_RC_BUTTON_2				0x40
//#define HW_STATUS_REAR_BUMPER				0x80	// KLUDGE- put here since 9 bumpers!


// Error Codes from Arduino (LastError)
#define HW_ARDUINO_NO_ERROR						0x00
#define HW_SERIAL_CMD_BUFFER_OVERFLOW		0x01
#define HW_SERIAL_CMD_NOT_READY				0x02
#define HW_SERIAL_CMD_INVALID				0x03
#define HW_SERIAL_CMD_CHECKSUM_ERROR		0x04
#define HW_BAD_STATE						0x05
#define HW_UNKNOWN_CMD						0x06
#define HW_CMD_NOT_IMPLEMENTED				0x07
#define HW_SERIAL_EXTRA_CHARS				0x08
#define HW_SERIAL_BAD_SYNC					0x09
#define HW_SERIAL_WAIT_PRIOR_CMD			0x0A
#define HW_SERVO_VALUE_ERROR				0x0B
#define HW_RADAR_SCAN_BAD_STATE				0x0C
#define HW_BAD_ISR							0x0D	// This is a CRITICAL ERROR!

// Sensor Error values
#define HW_SENSOR_ERROR						0xFE	// Bad Sensor or other error
#define HW_SENSOR_NOT_ATTACHED				0xFD	// Sensor never started
#define HW_SENSOR_NO_OBJECT					0xFC	// Max Range (no object detected)

// Generic Power commands
#define POWER_OFF							0x00
#define POWER_ON							0x01


// Messages - Messages from the Arduino, with Data
#define HW_MSG_DEBUG_STRING					0x00
//#define HW_MSG_IR_RANGE1					0x01
//#define HW_MSG_IR_RANGE2					0x02

// Other defines
#define RESPONSE_STRING_LEN					52	// Max length of response from the Arduino

// HW_SET_POWER_MODE Modes
// Servo will press Laptop Power button.  Typically, puts laptop in standby.
#define LAPTOP_POWER_MODE_DEFAULT				0x00	// Leave laptop alone.  Do nothing
#define LAPTOP_POWER_OFF_NO_RESTART				0x01	// Shut down Laptop (using servo).  Requires manual restart.
#define LAPTOP_POWER_OFF_RESTART_ON_ANY_SENSOR	0x02	// Shut down, but restart when any sensor triggers (preset thresholds)
#define LAPTOP_POWER_OFF_RESTART_ON_PIR_SENSOR	0x03	// Shut down, restart only with PIR sensor
#define LAPTOP_POWER_OFF_RESTART_NO_PIR_SENSOR	0x04	// Shut down, restart with any sensor BUT PIR
#define LAPTOP_POWER_OFF_RESTART_ON_TIMER		0x05	// TODO - not implemented!

#define LED_EYES_OFF						0x00
#define LED_EYES_ON							0x01
#define LED_EYES_CLOSE						0x02
#define LED_EYES_OPEN						0x03
#define LED_EYES_BLINK						0x04
//#define LED_EYE_BRIGHTNESS_MAX				  19	// On time in 20ms cycle



#endif	//__ROBOT_HW_COMMANDS_H__
