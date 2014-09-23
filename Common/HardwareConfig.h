// HardwareConfig.h
// Physical hardware configuration and dimensions of robot
// Note: Make sure to include HardwareCmds.h if needed

#ifndef __ROBOT_HARDWARE_CONFIG_DEFINED_H__
#define __ROBOT_HARDWARE_CONFIG_DEFINED_H__

#include "RobotConfig.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE - Also see HARDWARE CONFIG section in Globals.h

// General, applies to all robot types
	#define IR_SR_MAX_RANGE_TENTH_INCHES		240 // Tenth_Inches - Max range of Short Range IR sensors
	#define IR_SR_DETECT_RANGE_TENTH_INCHES		160 // Tenth_Inches - 4ft Max reliable range of Long range IR sensors for Object Finding
	#define IR_LR_MAX_RANGE_TENTH_INCHES		600 // Tenth_Inches - 5ft Max range of Long Range IR sensors
	#define IR_LR_DETECT_RANGE_TENTH_INCHES		480 // Tenth_Inches - 4ft Max reliable range of Long range IR sensors for Object Finding
	#define US_MAX_RANGE_TENTH_INCHES			960 // Tenth_Inches - 8ft Max range of US sensors
	#define US_MAX_DETECT_RANGE_TENTH_INCHES	960 // TODO Tenth_Inches - 8ft Max range of US sensors
	#define IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES	 40	// Tenth_Inches - Amount of error in IR readings (for determining distance to objects)
	#define US_RANGE_FUDGE_AMOUNT_TENTH_INCHES	240 // Tenth_Inches - Amount of error in US readings (for determining distance to objects)

	// Which servos are plugged in where
	#define SERVO_SPEED						0
	#define SERVO_TURN						1
	#define SERVO_GEAR						2
	#define SERVO_CAMERA_PAN				3
	#define SERVO_CAMERA_TILT				4



////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            LOKI
////////////////////////////////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI


	/* Loki Sensor Configuration for Arduino : (also see Globals.h)
	IR0: Left  Side Sensor angled almost 90 Degrees from forward
	IR1: Left  Forward, Low mounted, long range sensor
	IR2: Left  Head mounted, long range sensor.  Depends upon where the head is facing!
	IR3: Right Head mounted, long range sensor.  Depends upon where the head is facing!
	IR4: Right Forward, Low mounted, long range sensor
	IR5: Right Side Sensor angled almost 90 Degrees from forward

	US0: Head mounted UltraSonic Sensor
	US1: Left Forward mounted low UltraSonic Sensor (not currently connected)
	US2: Right Forward mounted low UltraSonic Sensor (not currently connected)

	IRB0-5: IR Bumpers
	I2C Expansion board: IRE2_0:
	*/

	#define OBJECT_EQUAL_DISTANCE		  0
	#define SIDE_SENSOR_ANGLE			 80	// degrees from forward TODO-LOKI-MUST is this right???
	#define ANGLE_SENSOR_ANGLE			 15 // Angled "IR Bumper" sensors - almost directly forward
	#define FORWARD_SENSOR_ANGLE		 10	// Hmm, use US angle, or IR angle?
	#define REAR_BUMPER_ANGLE			180
	#define REAR_IR_ANGLE				170

	// Sensor Field Of View for drawing sensor readings on the Map
	#define SENSOR_HALF_FOV_DEGREES_US							 150	// 1/2 of Sensor Field Of View in tenth degrees
	#define SENSOR_HALF_FOV_DEGREES_IR							 20	// This is actually bigger then sensor, but robot movement provides ambiguity

	//#define VERTICAL_IR_DETECT_RANGE_TENTH_INCHES	 300	// Detect tables, etc that robot might run into

	#define REAR_RIGHT				(-REAR_IR_ANGLE)
	#define SIDE_RIGHT				 SIDE_SENSOR_ANGLE
	#define ANGLE_RIGHT				 ANGLE_SENSOR_ANGLE
	#define FORWARD_RIGHT			 FORWARD_SENSOR_ANGLE
	#define FORWARD_LEFT			(-FORWARD_SENSOR_ANGLE)	// Negative Numbers on Left side
	#define ANGLE_LEFT				(-ANGLE_SENSOR_ANGLE)
	#define SIDE_LEFT				(-SIDE_SENSOR_ANGLE)
	#define REAR_LEFT				(-REAR_IR_ANGLE)

	#define IR_HEAD_TRACKING_RANGE_TENTH_INCHES					410 // Tenth_Inches - Head will track any object closer then this
	#define IR_TRACKING_FUDGE_TENTH_INCHES						 60 // Tenth_Inches
	#define IR_ELBOW_HIT_RANGE_TENTH_INCHES						 20 // Tenth_Inches - distance from front of claw tip (used by elbow sensor to detect impending arm collisions)
	#define CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES			 60	// Tenth_Inches

	// Arm Length measurements defined in ArmControl.cpp
	// Arm Sensor measurements
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_L		 55.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_R		 80.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_L				130.0	// Tenth_Inches
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_R				102.5	// Tenth_Inches

	#define GRID_MAP_AVOID_OBJECT_DISTANCE						  60	// Tenth_Inches - Avoid objects closer then this distance to the robot

	// Note: Arm Length measurements defined in ArmControl.cpp

	#define DISTANCE_BETWEEN_WHEELS_TENTH_INCHES 142.0	// 14.0 inches -Used for calculating amount of turn based upon difference in wheel odometers

	#define ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES		 80.0	// Sensor is 8 inches from center of robot body
	#define ROBOT_BODY_WIDTH_TENTH_INCHES				160	// Width of robot main body
	#define ROBOT_BODY_LENGTH_TENTH_INCHES				160	// Length of robot main body up to tail wheel (actual body is a little longer)
	#define ROBOT_ARM_WIDTH_TENTH_INCHES				 50	// Amount that each Arm sticks out from the sides of the robot main body
	#define ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES		(ROBOT_BODY_WIDTH_TENTH_INCHES + (ROBOT_ARM_WIDTH_TENTH_INCHES * 2))	// Size of robot, for calculating if it can fit between objects!

	#define HALF_ROBOT_BODY_WIDTH_TENTH_INCHES				(ROBOT_BODY_WIDTH_TENTH_INCHES/2)
	#define HALF_ROBOT_BODY_LENGTH_TENTH_INCHES				(ROBOT_BODY_LENGTH_TENTH_INCHES/2)
	#define HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES	(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES + ROBOT_ARM_WIDTH_TENTH_INCHES)
	#define HALF_ROBOT_CLAW_WIDTH_TENTH_INCHES				20

	// Sensor Zones for object avoidance
	#define FRONT_ZONE_THREAT_NORM_THRESHOLD	   180		// Tenth_Inches - Always avoid objects closer then this on the front
	#define FRONT_ZONE_THREAT_MIN_THRESHOLD		   120		// Tenth_Inches - Always avoid objects closer then this on the front
	#define ARM_ZONE_THREAT_MIN_THRESHOLD		    30		// Tenth_Inches - Always avoid objects closer then this on the front
	#define SIDE_THREAT_MIN_THRESHOLD			    50		// Tenth_Inches - avoid objects closer then this on the side
	#define SIDE_THREAT_NORM_THRESHOLD			   100		// Tenth_Inches - avoid objects closer then this on the side
	#define THREAT_PERSISTANCE_TIME				    50		// (1/10 sec) - how long to persist until allowing wider side avoidance behavior
	#define REAR_THREAT_THRESHOLD				    50		// Tenth_Inches - avoid objects closer then this behind the robot

	#define PROTECT_ARMS_FRONT_THREAT_THRESHOLD	   180		// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position
	#define PROTECT_ARMS_SIDE_THREAT_THRESHOLD		80		// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position

	#define FRONT_CENTER_ZONE_EDGE_TENTH_INCHES				HALF_ROBOT_BODY_WIDTH_TENTH_INCHES // 2 zones in front center of robot
	#define FRONT_ARM_ZONE_EDGE_TENTH_INCHES				(FRONT_CENTER_ZONE_EDGE_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES) // zone in front of each arm
	#define FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES				90 // look for objects to avoid this far to either side of the robot
	#define FRONT_SIDE_ZONE_EDGE_TENTH_INCHES				(FRONT_ARM_ZONE_EDGE_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)

	#define AVOIDANCE_SHARP_TURN_RANGE			  40 // Tenth_Inches - if front object closer then this, turn sharply

	// Head measurements (for Camera positioning)
	#define HEAD_IR_OFFSET_FROM_FRONT_TENTH_INCHES	 70	// Distance from Head IR sensors to front of robot
	#define LASER_MS_COMMAND_BASE_LEN			13 // lengh of command NOT including "Number of Scans"

	#define CAMERA_CENTER_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 370.0	// Tenth_Inches (not really correct - was fudged.. todo
	#define CAMERA_CENTER_DIST_FROM_ROBOT_FRONT_TENTH_INCHES	 100.0	// Tenth_Inches - distance to SERVO, not CAMERA! (8.5?)

	#define CAMERA_SERVO_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 345.0	// Tenth_Inches 34.5
	#define CAMERA_SERVO_DISTANCE_FROM_FRONT_TENTH_INCHES		  92.0	// Tenth_Inches 9.2
	#define CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES		  76.0	// Tenth_Inches 7.6
	#define CAMERA_OFFSET_IN_FRONT_OF_SERVO_TENTH_INCHES		  25.0	// Tenth_Inches - Camera lense sits in front of neck servo

	// Laser position measurements
	#define LASER_MIN_OBJECT_HEIGHT_TENTH_INCHES		  		   8	// Tenth_Inches!
	#define LASER_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 274.0	// Tenth_Inches - distance to laser beam
//	#define LASER_HEIGHT_ABOVE_SERVO_TENTH_INCHES				  10.0	// Tenth_Inches 
	#define LASER_DISTANCE_FROM_FRONT_TENTH_INCHES				  34.0	// Tenth_Inches - distance to laser front

	// Kinect position measurements
	#define KINECT_SLICE_MIN_OBJECT_HEIGHT_TENTH_INCHES	  		   4.0	// Tenth_Inches! - min height to be included in a slice (but 3D check is tighter)
	#define KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 265.0	// Tenth_Inches - distance to Kinect sensor center
	#define KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES				  25.0	// Tenth_Inches - distance to Kinect internal sensor

	// Degrees from Forward of each sensor
	// TODO! IR2,IR3 and US0 - Depends upon where the head is facing!  ASSUME forward for now!

	static int SensorOffsetDegrees_IR[NUM_IR_SENSORS] =
		//  0    1    2    3    4    5 		// IR Sensor Number
		{ -70, -10,  -5,   5,  10,  70 };	// Degrees from Forward

	static int SensorOffsetDegrees_US[NUM_US_SENSORS] = 
		//  0    1    2  					// UltraSonic Sensor Number
		{   0, -20,   20 };					// Degrees from Forward



////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           TURTLE
////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_KOBUKI_WITH_ARDUINO
	// Turtle Sensor Configuration:

	#define OBJECT_EQUAL_DISTANCE		  0
	#define SIDE_SENSOR_ANGLE			 80	// degrees from forward TODO-TURTLE-MUST is this right???
	#define ANGLE_SENSOR_ANGLE			 15 // Angled "IR Bumper" sensors - almost directly forward
	#define FORWARD_SENSOR_ANGLE		 10	// Hmm, use US angle, or IR angle?
	#define REAR_BUMPER_ANGLE			180
	#define REAR_IR_ANGLE				170

	// Sensor Field Of View for drawing sensor readings on the Map
	#define SENSOR_HALF_FOV_DEGREES_US							 150	// 1/2 of Sensor Field Of View in tenth degrees
	#define SENSOR_HALF_FOV_DEGREES_IR							 20	// This is actually bigger then sensor, but robot movement provides ambiguity

	//#define VERTICAL_IR_DETECT_RANGE_TENTH_INCHES	 300	// Detect tables, etc that robot might run into


	#define REAR_RIGHT				(-REAR_IR_ANGLE)
	#define SIDE_RIGHT				 SIDE_SENSOR_ANGLE
	#define ANGLE_RIGHT				 ANGLE_SENSOR_ANGLE
	#define FORWARD_RIGHT			 FORWARD_SENSOR_ANGLE
	#define FORWARD_LEFT			(-FORWARD_SENSOR_ANGLE)	// Negative Numbers on Left side
	#define ANGLE_LEFT				(-ANGLE_SENSOR_ANGLE)
	#define SIDE_LEFT				(-SIDE_SENSOR_ANGLE)
	#define REAR_LEFT				(-REAR_IR_ANGLE)

	#define IR_HEAD_TRACKING_RANGE_TENTH_INCHES					410 // Tenth_Inches - Head will track any object closer then this
	#define IR_TRACKING_FUDGE_TENTH_INCHES						 60 // Tenth_Inches
	#define IR_ELBOW_HIT_RANGE_TENTH_INCHES						 20 // Tenth_Inches - distance from front of claw tip (used by elbow sensor to detect impending arm collisions)
	#define CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES			 60	// Tenth_Inches

	// Arm Length measurements defined in ArmControl.cpp
	// Arm Sensor measurements
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_L		 55.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_R		 80.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_L				130.0	// Tenth_Inches
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_R				102.5	// Tenth_Inches

	#define GRID_MAP_AVOID_OBJECT_DISTANCE						  60	// Tenth_Inches - Avoid objects closer then this distance to the robot


	#define DISTANCE_BETWEEN_WHEELS_TENTH_INCHES 142.0	// 14.0 inches -Used for calculating amount of turn based upon difference in wheel odometers

	#define ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES		75.0	// Sensor is 7.5 inches from center of robot body
	#define ROBOT_BODY_DRAW_RADIUS_TENTH_INCHES			75.0	// Size of robot to draw on the map
	#define ROBOT_BODY_WIDTH_TENTH_INCHES				75		// Width of robot main body
	#define ROBOT_BODY_LENGTH_TENTH_INCHES				75		// Length of robot main body
	#define ROBOT_ARM_WIDTH_TENTH_INCHES				 1		// Amount that each Arm sticks out from the sides of the robot main body
	#define ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES		(ROBOT_BODY_WIDTH_TENTH_INCHES + (ROBOT_ARM_WIDTH_TENTH_INCHES * 2))	// Size of robot, for calculating if it can fit between objects!

	#define HALF_ROBOT_BODY_WIDTH_TENTH_INCHES				(ROBOT_BODY_WIDTH_TENTH_INCHES/2)
	#define HALF_ROBOT_BODY_LENGTH_TENTH_INCHES				(ROBOT_BODY_LENGTH_TENTH_INCHES/2)
	#define HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES	(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES + ROBOT_ARM_WIDTH_TENTH_INCHES)
	#define HALF_ROBOT_CLAW_WIDTH_TENTH_INCHES				20

	// Sensor Zones for object avoidance
	#define FRONT_ZONE_THREAT_NORM_THRESHOLD	   180		// Tenth_Inches - Always avoid objects closer then this on the front
	#define FRONT_ZONE_THREAT_MIN_THRESHOLD		 120	// Tenth_Inches - Always avoid objects closer then this on the front
	#define ARM_ZONE_THREAT_MIN_THRESHOLD		  30	// Tenth_Inches - Always avoid objects closer then this on the front
	#define SIDE_THREAT_MIN_THRESHOLD			  40	// Tenth_Inches - avoid objects closer then this on the side
	#define SIDE_THREAT_NORM_THRESHOLD			 100	// Tenth_Inches - avoid objects closer then this on the side
	#define THREAT_PERSISTANCE_TIME				 50	// (1/10 sec) - how long to persist until allowing wider side avoidance behavior

	#define THREAT_PERSISTANCE_TIME				    50		// (1/10 sec) - how long to persist until allowing wider side avoidance behavior
	#define REAR_THREAT_THRESHOLD				    50		// Tenth_Inches - avoid objects closer then this behind the robot

	#define PROTECT_ARMS_FRONT_THREAT_THRESHOLD	   360		// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position
	#define PROTECT_ARMS_SIDE_THREAT_THRESHOLD		80		// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position


	#define FRONT_CENTER_ZONE_EDGE_TENTH_INCHES				HALF_ROBOT_BODY_WIDTH_TENTH_INCHES // 2 zones in front center of robot
	#define FRONT_ARM_ZONE_EDGE_TENTH_INCHES				(FRONT_CENTER_ZONE_EDGE_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES) // zone in front of each arm
	#define FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES				90 // look for objects to avoid this far to either side of the robot
	#define FRONT_SIDE_ZONE_EDGE_TENTH_INCHES				(FRONT_ARM_ZONE_EDGE_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)

	#define AVOIDANCE_SHARP_TURN_RANGE			  40 // Tenth_Inches - if front object closer then this, turn sharply

	// Head measurements (for Camera positioning)
	#define HEAD_IR_OFFSET_FROM_FRONT_TENTH_INCHES	 70	// Distance from Head IR sensors to front of robot
	#define LASER_MS_COMMAND_BASE_LEN			13 // lengh of command NOT including "Number of Scans"

	#define CAMERA_CENTER_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 370.0	// Tenth_Inches (not really correct - was fudged.. todo
	#define CAMERA_CENTER_DIST_FROM_ROBOT_FRONT_TENTH_INCHES	  80.0	// Tenth_Inches - distance to SERVO, not CAMERA! (8.5?)

	#define CAMERA_SERVO_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 300.0	// Tenth_Inches 30.0
	#define CAMERA_SERVO_DISTANCE_FROM_FRONT_TENTH_INCHES		  80.0	// Tenth_Inches 9.2
	#define CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES		  20.0	// Tenth_Inches 7.6
	#define CAMERA_OFFSET_IN_FRONT_OF_SERVO_TENTH_INCHES		  10.0	// Tenth_Inches - Camera lense sits in front of servo

	// Laser position measurements
	#define LASER_MIN_OBJECT_HEIGHT_TENTH_INCHES		  		   8	// Tenth_Inches!
	#define LASER_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 274.0	// Tenth_Inches - distance to laser beam
//	#define LASER_HEIGHT_ABOVE_SERVO_TENTH_INCHES				  10.0	// Tenth_Inches 
	#define LASER_DISTANCE_FROM_FRONT_TENTH_INCHES				  34.0	// Tenth_Inches - distance to laser front

	// Kinect position measurements
	#define KINECT_SLICE_MIN_OBJECT_HEIGHT_TENTH_INCHES	  		   4.0	// Tenth_Inches! - min height to be included in a slice (but 3D check is tighter)
	#define KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 320.0	// Tenth_Inches - distance to Kinect sensor center
	#define KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES				  75.0	// Tenth_Inches - distance to Kinect internal sensor

	// Degrees from Forward of each sensor
	// TODO! IR2,IR3 and US0 - Depends upon where the head is facing!  ASSUME forward for now!

	static int SensorOffsetDegrees_IR[NUM_IR_SENSORS] =
		//  0    1    2    3    4    5 		// IR Sensor Number
		{ -70, -10,  -5,   5,  10,  70 };	// Degrees from Forward

	static int SensorOffsetDegrees_US[NUM_US_SENSORS] = 
		//  0    1    2  					// UltraSonic Sensor Number
		{   0, -20,   20 };					// Degrees from Forward




////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           TURTLE
////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP_KOBUKI
	// Teleoperated Robot on Kobuki Base Sensor Configuration:

	#define OBJECT_EQUAL_DISTANCE		  0
	#define SIDE_SENSOR_ANGLE			 80	// degrees from forward TODO-TURTLE-MUST is this right???
	#define ANGLE_SENSOR_ANGLE			 15 // Angled "IR Bumper" sensors - almost directly forward
	#define FORWARD_SENSOR_ANGLE		 10	// Hmm, use US angle, or IR angle?
	#define REAR_BUMPER_ANGLE			180
	#define REAR_IR_ANGLE				170

	// Sensor Field Of View for drawing sensor readings on the Map
	#define SENSOR_HALF_FOV_DEGREES_US							 150	// 1/2 of Sensor Field Of View in tenth degrees
	#define SENSOR_HALF_FOV_DEGREES_IR							 20	// This is actually bigger then sensor, but robot movement provides ambiguity

	//#define VERTICAL_IR_DETECT_RANGE_TENTH_INCHES	 300	// Detect tables, etc that robot might run into


	#define REAR_RIGHT				(-REAR_IR_ANGLE)
	#define SIDE_RIGHT				 SIDE_SENSOR_ANGLE
	#define ANGLE_RIGHT				 ANGLE_SENSOR_ANGLE
	#define FORWARD_RIGHT			 FORWARD_SENSOR_ANGLE
	#define FORWARD_LEFT			(-FORWARD_SENSOR_ANGLE)	// Negative Numbers on Left side
	#define ANGLE_LEFT				(-ANGLE_SENSOR_ANGLE)
	#define SIDE_LEFT				(-SIDE_SENSOR_ANGLE)
	#define REAR_LEFT				(-REAR_IR_ANGLE)

	#define IR_HEAD_TRACKING_RANGE_TENTH_INCHES					410 // Tenth_Inches - Head will track any object closer then this
	#define IR_TRACKING_FUDGE_TENTH_INCHES						 60 // Tenth_Inches
	#define CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES			 60	// Tenth_Inches

#if( ROBOT_HAS_LEFT_ARM || ROBOT_HAS_RIGHT_ARM )

	#define IR_ELBOW_HIT_RANGE_TENTH_INCHES						 20 // Tenth_Inches - distance from front of claw tip (used by elbow sensor to detect impending arm collisions)

	// Arm Length measurements defined in ArmControl.cpp
	// Arm Sensor measurements
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_L		 55.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_R		 80.0	// Tenth_Inches - clear space even when wrist rotated
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_L				130.0	// Tenth_Inches
	#define FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_R				102.5	// Tenth_Inches
#endif

	const int GRID_MAP_AVOID_OBJECT_DISTANCE =					  60;	// Tenth_Inches - Avoid objects closer then this distance to the robot


	const int DISTANCE_BETWEEN_WHEELS_TENTH_INCHES =			142;	// 14.0 inches -Used for calculating amount of turn based upon difference in wheel odometers

	const int ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES	=			 75;	// Sensor is 7.5 inches from center of robot body
	const int ROBOT_BODY_DRAW_RADIUS_TENTH_INCHES =				 75;	// Size of robot to draw on the map
	const int ROBOT_BODY_WIDTH_TENTH_INCHES	=					140;	// Width of robot main body - Kobuki
	const int ROBOT_BODY_LENGTH_TENTH_INCHES =					140;	// Length of robot main body
	const int ROBOT_ARM_WIDTH_TENTH_INCHES =					10;		// Amount that each Arm sticks out from the sides of the robot main body
	const int ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES =			(ROBOT_BODY_WIDTH_TENTH_INCHES + (ROBOT_ARM_WIDTH_TENTH_INCHES * 2));	// Size of robot, for calculating if it can fit between objects!

	const int HALF_ROBOT_BODY_WIDTH_TENTH_INCHES =				(ROBOT_BODY_WIDTH_TENTH_INCHES/2);
	const int HALF_ROBOT_BODY_LENGTH_TENTH_INCHES =				(ROBOT_BODY_LENGTH_TENTH_INCHES/2);
	const int HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES =	(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES + ROBOT_ARM_WIDTH_TENTH_INCHES);
	const int HALF_ROBOT_CLAW_WIDTH_TENTH_INCHES =				20;

	// Sensor Zones for object avoidance
	const int FRONT_ZONE_THREAT_NORM_THRESHOLD =				180;	// Tenth_Inches - Avoid objects closer then this on the front
	const int FRONT_ZONE_THREAT_MIN_THRESHOLD =					120;	// Tenth_Inches - Always avoid objects closer then this on the front
	const int ARM_ZONE_THREAT_MIN_THRESHOLD	=					 30;	// Tenth_Inches - Always avoid objects closer then this on the front
	const int SIDE_THREAT_MIN_THRESHOLD	=						 40;	// Tenth_Inches - avoid objects closer then this on the side
	const int SIDE_THREAT_NORM_THRESHOLD =						100;	// Tenth_Inches - avoid objects closer then this on the side
	const int THREAT_PERSISTANCE_TIME =							 50;	// (1/10 sec) - how long to persist until allowing wider side avoidance behavior

	const int REAR_THREAT_THRESHOLD =						     50;	// Tenth_Inches - avoid objects closer then this behind the robot

	const int PROTECT_ARMS_FRONT_THREAT_THRESHOLD =				360;	// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position
	const int PROTECT_ARMS_SIDE_THREAT_THRESHOLD =				 80;	// Tenth_Inches - if object closer than this, raise the robot's arms into a safe position


	const int FRONT_CENTER_ZONE_EDGE_TENTH_INCHES =				HALF_ROBOT_BODY_WIDTH_TENTH_INCHES; // 2 zones in front center of robot
	const int FRONT_ARM_ZONE_EDGE_TENTH_INCHES =				(FRONT_CENTER_ZONE_EDGE_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES); // zone in front of each arm
	const int FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES =				90; // look for objects to avoid this far to either side of the robot
	const int FRONT_SIDE_ZONE_EDGE_TENTH_INCHES	=				(FRONT_ARM_ZONE_EDGE_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES);

	const int AVOIDANCE_SHARP_TURN_RANGE =					   40; // Tenth_Inches - if front object closer then this, turn sharply

	// Head measurements (for Camera positioning)
	#define HEAD_IR_OFFSET_FROM_FRONT_TENTH_INCHES	 70	// Distance from Head IR sensors to front of robot
	#define LASER_MS_COMMAND_BASE_LEN			13 // lengh of command NOT including "Number of Scans"

	#define CAMERA_CENTER_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 370.0	// Tenth_Inches (not really correct - was fudged.. todo
	#define CAMERA_CENTER_DIST_FROM_ROBOT_FRONT_TENTH_INCHES	  80.0	// Tenth_Inches - distance to SERVO, not CAMERA! (8.5?)

	#define CAMERA_SERVO_HEIGHT_ABOVE_GROUND_TENTH_INCHES		 300.0	// Tenth_Inches 30.0
	#define CAMERA_SERVO_DISTANCE_FROM_FRONT_TENTH_INCHES		  80.0	// Tenth_Inches 9.2
	#define CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES		  20.0	// Tenth_Inches 7.6
	#define CAMERA_OFFSET_IN_FRONT_OF_SERVO_TENTH_INCHES		  10.0	// Tenth_Inches - Camera lense sits in front of servo

	// Laser position measurements
	#define LASER_MIN_OBJECT_HEIGHT_TENTH_INCHES		  		   8	// Tenth_Inches!
	#define LASER_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 274.0	// Tenth_Inches - distance to laser beam
//	#define LASER_HEIGHT_ABOVE_SERVO_TENTH_INCHES				  10.0	// Tenth_Inches 
	#define LASER_DISTANCE_FROM_FRONT_TENTH_INCHES				  34.0	// Tenth_Inches - distance to laser front

	// Kinect position measurements
	#define KINECT_SLICE_MIN_OBJECT_HEIGHT_TENTH_INCHES	  		   4.0	// Tenth_Inches! - min height to be included in a slice (but 3D check is tighter)
	#define KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES				 383.0	// Tenth_Inches - distance to Kinect sensor center - adjusted for TeleOp
	#define KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES				  60.0	// Tenth_Inches - distance to Kinect internal sensor 

	// IR Sensor offsets (Teleop sensors are set back from front of robot)
	#define BASE_IR_OFFSET_FROM_FRONT_TENTH_INCHES	 70	// Distance from Base IR sensors to front of robot



	// Degrees from Forward of each sensor
	// TODO! IR2,IR3 and US0 - Depends upon where the head is facing!  ASSUME forward for now!

	static int SensorOffsetDegrees_IR[NUM_IR_SENSORS] =
		//  0    1    2    3    4    5 		// IR Sensor Number
		{ -70, -10,  -5,   5,  10,  70 };	// Degrees from Forward

	static int SensorOffsetDegrees_US[NUM_US_SENSORS] = 
		//  0    1    2  					// UltraSonic Sensor Number
		{   0, -20,   20 };					// Degrees from Forward


////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             SEEKER CAR-ROBOT
////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT
	// Seeker CarBot Sensor Configuration:

	// Angle from forward of objects
	#define OBJECT_EQUAL_DISTANCE	  0
	#define SIDE_SENSOR_ANGLE		 30	// degrees from forward TODO-CAR-MUST is this right???
	#define ANGLE_SENSOR_ANGLE		 20
	#define FORWARD_SENSOR_ANGLE	 10

	#define SIDE_RIGHT				 SIDE_SENSOR_ANGLE
	#define ANGLE_RIGHT				 ANGLE_SENSOR_ANGLE
	#define FORWARD_RIGHT			 FORWARD_SENSOR_ANGLE
	#define FORWARD_LEFT			(-FORWARD_SENSOR_ANGLE)	// Negative Numbers on Left side
	#define ANGLE_LEFT				(-ANGLE_SENSOR_ANGLE)
	#define SIDE_LEFT				(-SIDE_SENSOR_ANGLE)


	// Sensor Field Of View for drawing sensor readings on the Map
	#define SENSOR_HALF_FOV_DEGREES_US		 15	// 1/2 of Sensor Field Of View in degrees
	#define SENSOR_HALF_FOV_DEGREES_IR		  2	// This is actually bigger then sensor, but robot movement provides ambiguity
	#define SIDE_THREAT_THRESHOLD			 120	// Tenth_Inches - avoid objects closer then this on the side






	#define ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES	  40.0	// Tenth Inches. Most sensors are about 3-4 inches from center of robot body
	#define ROBOT_BODY_DRAW_RADIUS_TENTH_INCHES		  80.0	// Radius of robot to draw on the map


	// TODO-CAR-MUST!
	static int SensorOffsetDegrees_IR[NUM_IR_SENSORS] =
		//  0    1    2    3        		// IR Sensor Number
		{ -80,  -5,   5,  80};				// Degrees from Forward

	static int SensorOffsetDegrees_US[NUM_US_SENSORS] = 
		//  0      1    2    3    4    5    6		// UltraSonic Sensor Number (0=Camera mount)
		{   0,   -70, -42, -14,  14,  42,  70 };	// Degrees from Forward


#else
	#error SENSOR_CONFIG_TYPE
#endif // SENSOR_CONFIG_TYPE


#endif // __ROBOT_HARDWARE_CONFIG_DEFINED_H__