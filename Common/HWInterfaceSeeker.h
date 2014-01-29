// HardwareCmds.h
// Command interface between the PC and Microcontroller, and is shared by both compilers
// Make sure syntax works with all compilers needed
#ifndef __ROBOT_HW_INTERFACE_SEEKER_H__
#define __ROBOT_HW_INTERFACE_SEEKER_H__


////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             SEEKER CAR-ROBOT
////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Seeker CarBot Sensor Configuration:

	#define NUMBER_OF_WHEEL_ODOMETERS	1		// Carbot only has one odometer

	// WARNING! If you change these, you must update SensorOffsetDegrees in Globals.cpp!
	#define NUM_IR_SENSORS							  4
	#define NUM_US_SENSORS							  7

	// Sensor Positions in arrays.  Both IR and US numbered from left to right
	#define	IR_SENSOR_SIDE_LEFT				  0
	#define	IR_SENSOR_FRONT_LEFT			  1
	#define	IR_SENSOR_FRONT_RIGHT			  2
	#define	IR_SENSOR_SIDE_RIGHT			  3
	//#define	IR_SENSOR_ANGLE_RIGHT			  4 // No IR angle sensors on Carbot
	//#define	IR_SENSOR_SIDE_RIGHT			  5

	// US Sensors are arranged in a semi-circle, and
	// Numbered left to right (1-6)
	#define US_SENSOR_CAMERA				  0	// Mounted on Camera
	#define US_SENSOR_SIDE_LEFT				  1
	#define US_SENSOR_ANGLE_LEFT			  2
	#define US_SENSOR_FRONT_LEFT			  3
	#define US_SENSOR_FRONT_RIGHT			  4
	#define US_SENSOR_ANGLE_RIGHT			  5
	#define US_SENSOR_SIDE_RIGHT			  6

	/*
	#define	RESULT_FAILURE		0xFFFF
	#define	WAYPOINT_IN_RANGE		360	// with in Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE	120	// Stop if within a foot of expected Waypoint

	#define OBJECT_COLLISION		  40	// Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET 4	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_THRESHOLD_DEFAULT  5	// CrCb +/- Threshold value
	#define CAMERA_SCAN_DIST_DEFAULT 13;	// Feet where camera starts really looking hard for the cone
	*/

	/*
	#define OBJECT_URGENT			 180	// Tenth_Inches - 1.5 ft - Object really close, need to avoid!
	#define OBJECT_DANGER			 360	// Tenth_Inches - 3 ft - Object close, need to avoid
	#define OBJECT_WARNING			 480	// Tenth_Inches - 4 ft - Object getting close, need to avoid or slow down
	#define OBJECT_AHEAD			 600	// Tenth_Inches - 5 ft - Object in range, consider going around it
	#define OBJECT_FAR				 960	// Tenth_Inches - 8 ft - Object barely in range
	#define	IR_HALL_THRESHOLD		 180	// Tenth_Inches - Range of interest in hall way nav.  Greater than 1/2 width of average hall
	*/

	#define IR_RANGE_FUDGE_AMOUNT	 40	// Tenth_Inches - Amount of error in IR readings (for determining distance to objects)
	#define US_RANGE_FUDGE_AMOUNT	 240 // Tenth_Inches - Amount of error in US readings (for determining distance to objects)


	#define IR_SR_MAX_RANGE			 240 // Tenth_Inches - Max range of Short Range IR sensors
	#define IR_SR_DETECT_RANGE		 160 // Tenth_Inches - 4ft Max reliable range of Long range IR sensors for Object Finding
	#define IR_LR_MAX_RANGE			 600 // Tenth_Inches - 5ft Max range of Long Range IR sensors
	#define IR_LR_DETECT_RANGE		 480 // Tenth_Inches - 4ft Max reliable range of Long range IR sensors for Object Finding
	#define US_MAX_RANGE			 960 // Tenth_Inches - 8ft Max range of US sensors
	#define US_MAX_DETECT_RANGE		 960 // TODO Tenth_Inches - 8ft Max range of US sensors

	#define GRID_MAP_AVOID_OBJECT_DISTANCE	60	// Tenth_Inches - Avoid objects closer then this distance to the robot


	////////////////////////////////////////////////////////////////
	#ifndef __ARDUINO_TARGET	// Arduino Compiler does not understand pack
	#pragma pack( 1 )
	#endif

	// Raw data from the Arduino for CARBOT
	typedef struct
	{
	BYTE StatusFlags;		// 3
	BYTE LastError;			// 4
	BYTE DebugCode;			// 5
	BYTE Bumper;			// 6
//	BYTE AckMotorCommand;	// xx
//	BYTE AckMotorSpeed;		// xx
//	BYTE AckMotorTurn;		// xx
	BYTE Battery0;			// 7
	BYTE Battery1;			// 8
	BYTE CompassHigh;		// 9
	BYTE CompassLow;		//10
	BYTE OdometerHigh;		//11
	BYTE OdometerLow;		//12
	BYTE Tachometer;		//13
	BYTE US[NUM_US_SENSORS];//14-20
	BYTE IR[NUM_IR_SENSORS];//21-26	- UP TO 6, may be less!
	BYTE AccelX;			//27
	BYTE AccelY;			//28
	} ARDUINO_STATUS_T;

	#ifndef __ARDUINO_TARGET
	#pragma pack( 4 )
	#endif


// TODO-MUST-CARBOT - FIX THESE TO MATCH REAL HARDWARE!!!!
#define HW_BUMPER_NOT_USED_MASK1			0x01	// Bit 4 mask
#define HW_BUMPER_NOT_USED_MASK2			0x02	// Bit 5 mask
#define HW_BUMPER_NOT_USED_MASK3			0x04	// Bit 6 mask
#define HW_BUMPER_REAR_MASK					0x08	// Bit 7 mask
#define HW_BUMPER_SIDE_LEFT_MASK			0x10	// Bit 2 mask
#define HW_BUMPER_SIDE_RIGHT_MASK			0x20	// Bit 3 mask
#define HW_BUMPER_FRONT_LEFT_MASK			0x40	// Bit 6 mask	// Set in TimerISR
#define HW_BUMPER_FRONT_RIGHT_MASK			0x80	// Bit 7 mask	// Set in TimerISR

// Used by Arduino only:
#define FRONT_LEFT_BUMPER_BIT				   6	// Bit 6 - used by bit_set function in TimerISR
#define FRONT_RIGHT_BUMPER_BIT				   7	// Bit 7


// IR Bumpers.  Arduino uses: gStatus_IR_Bumper
#define IR_BUMPER_FRONT_LEFT_MASK			0x01	// Bit 0 mask
#define IR_BUMPER_FRONT_RIGHT_MASK			0x02	// Bit 1 mask
#define IR_BUMPER_CLIFF_LEFT_MASK			0x04	// Bit 2 mask
#define IR_BUMPER_CLIFF_RIGHT_MASK			0x08	// Bit 3 mask
#define IR_BUMPER_REAR_LEFT_MASK			0x10	// Bit 4 mask
#define IR_BUMPER_REAR_RIGHT_MASK			0x20	// Bit 5 mask
#define IR_BUMPER_NOT_USED_MASK3			0x40	// Bit 6 mask
#define IR_BUMPER_NOT_USED_MASK4			0x80	// Bit 7 mask


// Bumper Macros
	/*
#define IR_BUMPER_OBJECT_FRONT_LEFT		(g_pFullSensorStatus->IRBumper & IR_BUMPER_FRONT_LEFT_MASK)
#define IR_BUMPER_OBJECT_FRONT_RIGHT	(g_pFullSensorStatus->IRBumper & IR_BUMPER_FRONT_RIGHT_MASK)
#define IR_BUMPER_CLIFF_LEFT			(!(g_pFullSensorStatus->IRBumper & IR_BUMPER_CLIFF_LEFT_MASK))	// Lack of object = cliff
#define IR_BUMPER_CLIFF_RIGHT			(!(g_pFullSensorStatus->IRBumper & IR_BUMPER_CLIFF_RIGHT_MASK))
#define IR_BUMPER_OBJECT_REAR_LEFT		(g_pFullSensorStatus->IRBumper & IR_BUMPER_REAR_LEFT_MASK)
#define IR_BUMPER_OBJECT_REAR_RIGHT		(g_pFullSensorStatus->IRBumper & IR_BUMPER_REAR_RIGHT_MASK)

#define HW_BUMPER_HIT_FRONT_LEFT		(g_pFullSensorStatus->HWBumper & HW_BUMPER_FRONT_LEFT_MASK)
#define HW_BUMPER_HIT_FRONT_RIGHT		(g_pFullSensorStatus->HWBumper & HW_BUMPER_FRONT_RIGHT_MASK)
#define HW_BUMPER_HIT_SIDE_LEFT			(g_pFullSensorStatus->HWBumper & HW_BUMPER_SIDE_LEFT_MASK)
#define HW_BUMPER_HIT_SIDE_RIGHT		(g_pFullSensorStatus->HWBumper & HW_BUMPER_SIDE_RIGHT_MASK)
#define HW_BUMPER_HIT_REAR				(g_pFullSensorStatus->HWBumper & HW_BUMPER_REAR_MASK)

#define HW_BUMPER_HIT_FRONT				(HW_BUMPER_HIT_FRONT_RIGHT || HW_BUMPER_HIT_FRONT_LEFT)
	*/

/*
////////////////////////////////////////////////////////////////
// Packet Structure for Comands to Arduino
#ifndef __ARDUINO_TARGET	// Arduino Compiler does not understand pack
#pragma pack( 1 )
#endif

typedef struct
{
	//BYTE Sync0;
	//BYTE Sync1;
	BYTE Cmd;
	BYTE Param1;
	BYTE Param2;
	BYTE Param3;
	BYTE Param4;
	//BYTE TermChar;
} ARDUINO_CMD_T;
#define ARDUINO_CMD_SIZE 5	// 5 bytes

#ifndef __ARDUINO_TARGET
#pragma pack( 4 )
#endif

*/





#endif	//__ROBOT_HW_INTERFACE_SEEKER_H__
