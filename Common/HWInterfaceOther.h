// HWInterfaceOther.h
// Interface parameters for Motor, Servo, etc. Controllers

#ifndef __ROBOT_HW_INTERFACE_OTHER_H__
#define __ROBOT_HW_INTERFACE_OTHER_H__
//#include "stdafx.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generic Servo Constants
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if( SERVO_CONTROL_TYPE == PIC_SERVO_CONTROL )

	#define SERVO_OFF					  0
	#define SERVO_CENTER				 63
	#define SERVO_MAX					127		// Servo values: 0=off, 1=bottom, 64=center 128=top
	#define SERVO_MIN					  1
	#define SERVO_MODE_180				  0		// Not used for Arduino

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( SERVO_CONTROL_TYPE == EXTERN_SERVO_CONTROL )
// Use this if controlling servos from the Pololu Servo Controller

	#define SERVO_OFF					  0
	#define SERVO_CENTER				  0
	#define SERVO_MAX				    127
	#define SERVO_MIN				   -127
	#define SERVO_MODE_180				  8 // +8 Puts servo in 180 degree travel mode!

#elif( SERVO_CONTROL_TYPE == DYNA_SERVO_CONTROL )

	#define SERVO_OFF					  0
	#define SERVO_CENTER				  0
	#define SERVO_MAX					127		// Servo values: 0=off, 1=bottom, 64=center 128=top
	#define SERVO_MIN				   -127
	#define SERVO_MODE_180				  0		// Not used for DYNA

#else
	#error BAD SERVO_CONTROL_TYPE!
#endif



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor Control
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ACCELERATION_SLOW
	#define ACCELERATION_SLOW		0
	#define ACCELERATION_MEDIUM		1
	#define ACCELERATION_FAST		2
	#define ACCELERATION_INSTANT	3
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if( MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL )

	#define TICKS_PER_TENTH_INCH			    429.50	// MrRoboto 2 ER1 Maybe 409.6?

	// Turn Constants
	#define TURN_SERVO_CENTER			  0		// TRIM: + = LEFT - Use Only just before sending to the Arduino!
	#define TURN_CENTER				 	  0
	#define TURN_LEFT_SLOW			   (-5)
	#define TURN_LEFT_MED_SLOW		   (-10)
	#define TURN_LEFT_MED			   (-20)
	#define TURN_LEFT_MED_FAST		   (-40)
	#define TURN_LEFT_FAST			  (-127)
	#define TURN_LEFT_MAX			  (-127)	// For LIMIT CHECKS

	#define TURN_RIGHT_SLOW			      5
	#define TURN_RIGHT_MED_SLOW		     10
	#define TURN_RIGHT_MED			     20
	#define TURN_RIGHT_MED_FAST		     40
	#define TURN_RIGHT_FAST			    127
	#define TURN_RIGHT_MAX			    127 	// For LIMIT CHECKS


/* OLD --------------
	#define TURN_5_DEG_LEFT			   (-20)
	#define TURN_10_DEG_LEFT		   (-40)
	#define TURN_20_DEG_LEFT		   (-60)
	TURN_30_DEG_LEFT
	#define TURN_FULL_LEFT			  (-120)	// 25 Degrees = max turn
	#define TURN_MAX_LEFT			  (-125)	// For LIMIT CHECKS

	#define TURN_5_DEG_RIGHT			 10
	#define TURN_10_DEG_RIGHT			 20
	#define TURN_20_DEG_RIGHT		  	 30
	#define TURN_30_DEG_RIGHT		  	 50
	#define TURN_FULL_RIGHT				127
	#define TURN_MAX_RIGHT				127		// For LIMIT CHECKS
	#define TURN_MAX_ABS				127		// Unsigned numberic limit for turns
OLD -----------------
*/


	// Turn constants.  WARNING! if you swap Left/Right, look for limit checks in code!!!
	// Absolute turn Servo values.  Don't use unless really needed!
	//#define TURN_SERVO_MAX_LEFT				(TURN_SERVO_CENTER + TURN_MAX_LEFT)
	//#define TURN_SERVO_MAX_RIGHT			(TURN_SERVO_CENTER + TURN_MAX_RIGHT)

	// Speed Constants
	#define SPEED_STOP				 	  0		// 60
	#define SPEED_FULL_FWD				127		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV			   -127

		// ER1 TODO!  Set these to correct values!
	#define SPEED_SERVO_CENTER			  0		// Same as SPEED_STOP
	#define SPEED_FWD_SLOW				 20
	#define SPEED_FWD_MED_SLOW			 40
	#define SPEED_FWD_MED				 60
	#define SPEED_FWD_MED_FAST			 80
	#define SPEED_FWD_FAST				127

	#define SPEED_REV_SLOW			   (-20)
	#define SPEED_REV_MED_SLOW		   (-40)
	#define SPEED_REV_MED			   (-60)
	#define SPEED_REV_MED_FAST		   (-80)
	#define SPEED_REV_FAST			  (-127)

	#define LOW_GEAR				     35		// Absolute servo value (no need for dist from zero/center)
	#define HIGH_GEAR					 91

	#define TURN_MULTIPLIER				2.0		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER		  4		// Amount of wheel to apply to head for Radar Object
	#define TURN_ROTATION_COMPLETE_COMPENSATION 2.0	// degrees we assume the robot will wiggle after completing set turn amount

	// Braking and Automatic Speed Control
	// TODO - Are these correct for Mr Roboto2?
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 20	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME		0	// 1/10 sec Delays to allow robot to coast after completing move distance


	#define NEW_SPEED_SET_TIME				 16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME				  2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION			  2	// max speed servo adjustment allowed

	#define MOVE_DISTANCE_SHORT				  40	// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT			  60	// Tenth_Inches
	#define MOVE_DISTANCE_MED				 120	// Tenth_Inches
	#define MOVE_DISTANCE_LONG				 240	// Tenth_Inches
	#define MOVE_DISTANCE_EXTRA				  10 // extra distance to move while turning, so we don't run into the same object!


	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	  60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		  10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE				 360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE			 120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION				  10 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET		  2	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT		 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE			  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
	// Use this if controlling servos from the Pololu Servo Controller

	#define TICKS_PER_TENTH_INCH	   6.35	// Seeker CarBot: about 1.5 inchs per tick

	// Turn Constants
	#define TURN_SERVO_CENTER			127		// TRIM: + = LEFT - Use Only just before sending to the Servo Control!
	#define TURN_CENTER				 	  0

	#define TURN_LEFT_SLOW			   (-20)
	#define TURN_LEFT_MED_SLOW		   (-40)
	#define TURN_LEFT_MED			   (-60)
	#define TURN_LEFT_MED_FAST		   (-80)
	#define TURN_LEFT_FAST			  (-120)	// 25 Degrees = max turn
	#define TURN_LEFT_MAX			  (-125)	// For LIMIT CHECKS

	#define TURN_RIGHT_SLOW			     20
	#define TURN_RIGHT_MED_SLOW		     40
	#define TURN_RIGHT_MED			     60
	#define TURN_RIGHT_MED_FAST		     80
	#define TURN_RIGHT_FAST			    120 	// 25 Degrees = max turn
	#define TURN_RIGHT_MAX			    125 	// For LIMIT CHECKS


	// Turn constants.  WARNING! if you swap Left/Right, look for limit checks in code!!!
	// Absolute turn Servo values.  Don't use unless really needed!
	//#define TURN_SERVO_MAX_LEFT				(TURN_SERVO_CENTER + TURN_MAX_LEFT)
	//#define TURN_SERVO_MAX_RIGHT			(TURN_SERVO_CENTER + TURN_MAX_RIGHT)

	// Logical Speed constants
	#define SPEED_STOP				 	  0		//
	#define SPEED_FULL_FWD				125		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV			  (-125)

	#define SPEED_FWD_SLOW				 13		// Can be below minimum!
	#define SPEED_FWD_MED_SLOW			 14		// 14 is Minimum that the NOVAK speed control will obay
	#define SPEED_FWD_MED				 16
	#define SPEED_FWD_MED_FAST			 40
	#define SPEED_FWD_FAST				100

	#define SPEED_REV_SLOW			   (-16)		// -16 is not reliable!
	#define SPEED_REV_MED_SLOW		   (-17)		// -17 is Minimum that the NOVAK speed control will obay
	#define SPEED_REV_MED			   (-20)
	#define SPEED_REV_MED_FAST		   (-30)
	#define SPEED_REV_FAST			  (-80)

	// Servo Speed Constants (actual servo settings)
	#define SPEED_SERVO_CENTER			105		// TRIM: + = ? - Use Only just before sending to the Arduino!
	#define LOW_GEAR				    22
	#define HIGH_GEAR					250


	#define TURN_MULTIPLIER					0.5		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER			  4		// Amount of wheel to apply to head for Radar Object
	#define TURN_ROTATION_COMPLETE_COMPENSATION 0	// degrees we assume the robot will wiggle after completing set turn amount


	// Braking and Automatic Speed Control
	// TODO-CAR - Check that these values are correct for Carbot
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 60	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME		0	// 1/10 sec Delays to allow robot to coast after completing move distance

	#define NEW_SPEED_SET_TIME			    16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME			     2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION		     2	// max speed servo adjustment allowed


	#define MOVE_DISTANCE_SHORT				 180	// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT			 240	// Tenth_Inches
	#define MOVE_DISTANCE_MED				 360	// Tenth_Inches
	#define MOVE_DISTANCE_LONG				 480	// Tenth_Inches

	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	  60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		 10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE				 360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE			 120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION				  40 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET		  4	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT		 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE			  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( (MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL) )
	// Use this if controlling the motors from the Pololu TReX Motor Controller, or indirectly through the Arduino/Arduino!

// TODO-TREX - Need to set all these values to correct numbers!!!

	#define TICKS_PER_TENTH_INCH		   3.14	// TODO-TREX Seeker (fix Simulator OdometerTickCount after adjusting this value!)

	// Turn Constants
	#define TURN_SERVO_CENTER			  0		// TRIM: + = LEFT - Use Only just before sending to the Arduino!
	#define TURN_CENTER				 	  0
	#define TURN_LEFT_VERY_SLOW		   (-5)	// Only use if moving, otherwise will stall
	#define TURN_LEFT_SLOW			   (-15)
	#define TURN_LEFT_MED_SLOW		   (-25)
	#define TURN_LEFT_MED			   (-40)
	#define TURN_LEFT_MED_FAST		   (-60)
	#define TURN_LEFT_FAST			  (-127)
	#define TURN_LEFT_MAX			  (-127)	// For LIMIT CHECKS

	#define TURN_RIGHT_VERY_SLOW	      5	// Only use if moving, otherwise will stall
	#define TURN_RIGHT_SLOW			     15
	#define TURN_RIGHT_MED_SLOW		     25
	#define TURN_RIGHT_MED			     40
	#define TURN_RIGHT_MED_FAST		     60
	#define TURN_RIGHT_FAST			    127
	#define TURN_RIGHT_MAX			    127 	// For LIMIT CHECKS

	// Speed Constants
	#define SPEED_STOP				 	  0		// 60
	#define SPEED_FULL_FWD				127		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV			   -127

	#define SPEED_SERVO_CENTER			  0		// Same as SPEED_STOP
	#define SPEED_FWD_VERY_SLOW			 15		// Almost stop, but no jerking (used for object avoidance)
	#define SPEED_FWD_SLOW				 25
	#define SPEED_FWD_MED_SLOW			 40
	#define SPEED_FWD_MED				 60
	#define SPEED_FWD_MED_FAST			 80
	#define SPEED_FWD_FAST				127

	#define SPEED_REV_SLOW			   (-25)
	#define SPEED_REV_MED_SLOW		   (-40)
	#define SPEED_REV_MED			   (-60)
	#define SPEED_REV_MED_FAST		   (-80)
	#define SPEED_REV_FAST			  (-127)

	#define LOW_GEAR				     35		// Absolute servo value (no need for dist from zero/center)
	#define HIGH_GEAR					 91

	#define TURN_MULTIPLIER				2.0		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER		  4		// Amount of wheel to apply to head for Radar Object
	#define TURN_ROTATION_COMPLETE_COMPENSATION 2.0	// degrees we assume the robot will wiggle after completing set turn amount

	// Braking and Automatic Speed Control
	// TODO - Are these correct for Mr Roboto2?
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 10	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME		0	// 1/10 sec Delays to allow robot to coast after completing move distance


	#define NEW_SPEED_SET_TIME				 16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME				  2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION			 40	// max speed adjustment allowed

	#define MOVE_DISTANCE_SHORT				  40	// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT			  60	// Tenth_Inches
	#define MOVE_DISTANCE_MED				 120	// Tenth_Inches
	#define MOVE_DISTANCE_LONG				 240	// Tenth_Inches
	#define MOVE_DISTANCE_EXTRA				  10 // extra distance to move while turning, so we don't run into the same object!


	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	  60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		 10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE				 360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE			 120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION				  10 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET		  2	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT		 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE			  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL )
	// Use this if the motors are in the iRobot Create base

// TODO-TURTLE - Need to set all these values to correct numbers!!!

	#define TICKS_PER_TENTH_INCH		   3.14	// TODO-TURTLE  (fix Simulator OdometerTickCount after adjusting this value!)

	// Turn Constants
	#define TURN_SERVO_CENTER			  0		// TRIM: + = LEFT - Use Only just before sending to the Arduino!
	#define TURN_CENTER				 	  0
	#define TURN_LEFT_VERY_SLOW		   (-5)	// Only use if moving, otherwise will stall
	#define TURN_LEFT_SLOW			   (-15)
	#define TURN_LEFT_MED_SLOW		   (-25)
	#define TURN_LEFT_MED			   (-40)
	#define TURN_LEFT_MED_FAST		   (-60)
	#define TURN_LEFT_FAST			  (-127)
	#define TURN_LEFT_MAX			  (-127)	// For LIMIT CHECKS

	#define TURN_RIGHT_VERY_SLOW	      5	// Only use if moving, otherwise will stall
	#define TURN_RIGHT_SLOW			     15
	#define TURN_RIGHT_MED_SLOW		     25
	#define TURN_RIGHT_MED			     40
	#define TURN_RIGHT_MED_FAST		     60
	#define TURN_RIGHT_FAST			    127
	#define TURN_RIGHT_MAX			    127 	// For LIMIT CHECKS

	// Speed Constants
	#define SPEED_STOP				 	  0		// 60
	#define SPEED_FULL_FWD				127		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV			   -127

	#define SPEED_SERVO_CENTER			  0		// Same as SPEED_STOP
	#define SPEED_FWD_VERY_SLOW			 15		// Almost stop, but no jerking (used for object avoidance)
	#define SPEED_FWD_SLOW				 25
	#define SPEED_FWD_MED_SLOW			 40
	#define SPEED_FWD_MED				 60
	#define SPEED_FWD_MED_FAST			 80
	#define SPEED_FWD_FAST				127

	#define SPEED_REV_SLOW			   (-25)
	#define SPEED_REV_MED_SLOW		   (-40)
	#define SPEED_REV_MED			   (-60)
	#define SPEED_REV_MED_FAST		   (-80)
	#define SPEED_REV_FAST			  (-127)

	#define LOW_GEAR				     35		// Absolute servo value (no need for dist from zero/center)
	#define HIGH_GEAR					 91

	#define TURN_MULTIPLIER				2.0		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER		  4		// Amount of wheel to apply to head for Radar Object
	#define TURN_ROTATION_COMPLETE_COMPENSATION 2.0	// degrees we assume the robot will wiggle after completing set turn amount

	// Braking and Automatic Speed Control
	// TODO - Are these correct?
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 20	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME		0	// 1/10 sec Delays to allow robot to coast after completing move distance


	#define NEW_SPEED_SET_TIME				 16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME				  2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION			  2	// max speed servo adjustment allowed

	#define MOVE_DISTANCE_SHORT				  40	// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT			  60	// Tenth_Inches
	#define MOVE_DISTANCE_MED				 120	// Tenth_Inches
	#define MOVE_DISTANCE_LONG				 240	// Tenth_Inches
	#define MOVE_DISTANCE_EXTRA				  10 // extra distance to move while turning, so we don't run into the same object!


	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	  60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		 10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE				 360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE			 120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION				  10 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET		  2	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT		 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE			  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
	// Use this if the motors are in the Kobuki base

// TODO-TURTLE - Need to set all these values to correct numbers!!!

	#define TICKS_PER_TENTH_INCH		   3.14	// TODO-TURTLE  (fix Simulator OdometerTickCount after adjusting this value!)

	// Turn Constants
	#define TURN_SERVO_CENTER			  0		// TRIM: + = LEFT - Use Only just before sending to the Arduino!
	#define TURN_CENTER				 	  0
	#define TURN_LEFT_VERY_SLOW		   (-5)	// Only use if moving, otherwise will stall
	#define TURN_LEFT_SLOW			   (-15)
	#define TURN_LEFT_MED_SLOW		   (-25)
	#define TURN_LEFT_MED			   (-40)
	#define TURN_LEFT_MED_FAST		   (-60)
	#define TURN_LEFT_FAST			  (-127)
	#define TURN_LEFT_MAX			  (-127)	// For LIMIT CHECKS

	#define TURN_RIGHT_VERY_SLOW	      5	// Only use if moving, otherwise will stall
	#define TURN_RIGHT_SLOW			     15
	#define TURN_RIGHT_MED_SLOW		     25
	#define TURN_RIGHT_MED			     40
	#define TURN_RIGHT_MED_FAST		     60
	#define TURN_RIGHT_FAST			    127
	#define TURN_RIGHT_MAX			    127 	// For LIMIT CHECKS

	// Speed Constants
	#define SPEED_STOP				 	  0		// 60
	#define SPEED_FULL_FWD				127		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV			   -127

	#define SPEED_SERVO_CENTER			  0		// Same as SPEED_STOP
	#define SPEED_FWD_VERY_SLOW			 15		// Almost stop, but no jerking (used for object avoidance)
	#define SPEED_FWD_SLOW				 25
	#define SPEED_FWD_MED_SLOW			 40
	#define SPEED_FWD_MED				 60
	#define SPEED_FWD_MED_FAST			 80
	#define SPEED_FWD_FAST				127

	#define SPEED_REV_SLOW			   (-25)
	#define SPEED_REV_MED_SLOW		   (-40)
	#define SPEED_REV_MED			   (-60)
	#define SPEED_REV_MED_FAST		   (-80)
	#define SPEED_REV_FAST			  (-127)

	#define LOW_GEAR				     35		// Absolute servo value (no need for dist from zero/center)
	#define HIGH_GEAR					 91

	#define TURN_MULTIPLIER				2.0		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER		  4		// Amount of wheel to apply to head for Radar Object
	#define TURN_ROTATION_COMPLETE_COMPENSATION 2.0	// degrees we assume the robot will wiggle after completing set turn amount

	// Braking and Automatic Speed Control
	// TODO - Are these correct?
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 20	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME		0	// 1/10 sec Delays to allow robot to coast after completing move distance


	#define NEW_SPEED_SET_TIME				 16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME				  2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION			  2	// max speed servo adjustment allowed

	#define MOVE_DISTANCE_SHORT				  40	// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT			  60	// Tenth_Inches
	#define MOVE_DISTANCE_MED				 120	// Tenth_Inches
	#define MOVE_DISTANCE_LONG				 240	// Tenth_Inches
	#define MOVE_DISTANCE_EXTRA				  10 // extra distance to move while turning, so we don't run into the same object!


	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	  60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		 10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE				 360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE			 120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION				  10 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET		  2	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT		 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE			  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( MOTOR_CONTROL_TYPE == EXTERN_SERVO_CONTROL )

	#define TICKS_PER_TENTH_INCH		  6.35	// Seeker CarBot: about 1.5 inchs per tick

	// Turn Constants
	#define TURN_SERVO_CENTER			 61		// TRIM: + = LEFT - Use Only just before sending to the Arduino!
	#define TURN_CENTER				 	  0

	#define TURN_LEFT_SLOW			    (-5)
	#define TURN_LEFT_MED_SLOW		   (-10)
	#define TURN_LEFT_MED			   (-20)
	#define TURN_LEFT_MED_FAST		   (-25)
	#define TURN_LEFT_FAST			   (-30)	// 25 Degrees = max turn
	#define TURN_LEFT_MAX			   (-32)	// For LIMIT CHECKS

	#define TURN_RIGHT_SLOW			      5
	#define TURN_RIGHT_MED_SLOW		     10
	#define TURN_RIGHT_MED			     20
	#define TURN_RIGHT_MED_FAST		     25
	#define TURN_RIGHT_FAST			     30	// 25 Degrees = max turn
	#define TURN_RIGHT_MAX			     32	// For LIMIT CHECKS


	// Turn constants.  WARNING! if you swap Left/Right, look for limit checks in code!!!
	// Absolute turn Servo values.  Don't use unless really needed!
	//#define TURN_SERVO_MAX_LEFT				(TURN_SERVO_CENTER + TURN_MAX_LEFT)
	//#define TURN_SERVO_MAX_RIGHT			(TURN_SERVO_CENTER + TURN_MAX_RIGHT)

	// Speed Constants
	#define SPEED_STOP				 	 60		//
	#define SPEED_FULL_FWD				 95		// WARNING! if you swap these, update TempSpeedCmd in Module.cpp!!!
	#define SPEED_FULL_REV				 31

	#define SPEED_SERVO_CENTER			 60		// Same as SPEED_STOP
	#define SPEED_FWD_SLOW				  3		// Can be below minimum!
	#define SPEED_FWD_MED_SLOW			  4		// 4 is Minimum that the NOVAK speed control will obay
	#define SPEED_FWD_MED				  5
	#define SPEED_FWD_MED_FAST			  8
	#define SPEED_FWD_FAST				 30

	#define SPEED_REV_SLOW			   (-3)		// -4 is not reliable!
	#define SPEED_REV_MED_SLOW		   (-4)		// -5 is Minimum that the NOVAK speed control will obay
	#define SPEED_REV_MED			   (-5)
	#define SPEED_REV_MED_FAST		   (-8)
	#define SPEED_REV_FAST			  (-20)

	#define LOW_GEAR				     35		// Absolute servo value (no need for dist from zero/center)
	#define HIGH_GEAR					 91

	#define TURN_MULTIPLIER					0.5		// Amount of wheel turn to apply to get back on heading
	#define RADAR_TURN_MULTIPLIER			  4		// Amount of wheel to apply to head for Radar Object

	// Braking and Automatic Speed Control
	// May not be correct for Arduino, particularly BRAKE_TIME, which is in 20ms ticks for Arduino.
	// Not used anyway for Carbot, which uses an external servo controller.
	#define SPEED_BRAKE_AMOUNT			 (-20)	// 8 if not using tachometer to know when wheel stopped
	#define SPEED_BRAKE_TIME			   30	// (* 100ms) Max amount of time to apply brake
	#define MOVE_DISTANCE_COASTING_COMPENSATION 60	// Tenth_Inches we assume the robot will coast after completing move distance
	#define MOVE_DISTANCE_COASTING_TIME	   10	// 100ms Delays to allow robot to coast after completing move distance

	#define NEW_SPEED_SET_TIME			   16	// number of 100ms intervals to wait for motor to come to speed after new command
	#define SPEED_CHANGE_TIME			    2	// number of 100ms intervals to wait for motor after speed correction
	#define MAX_SPEED_CORRECTION		    2	// max speed servo adjustment allowed

	#define MOVE_DISTANCE_SHORT			  180		// Tenth_Inches
	#define MOVE_DISTANCE_MED_SHORT		  240		// Tenth_Inches
	#define MOVE_DISTANCE_MED			  360		// Tenth_Inches
	#define MOVE_DISTANCE_LONG			  480		// Tenth_Inches

	#define SEGMENT_FOLLOW_DISTANCE_FUDGE	60	//Tenth_Inches - tolerance when tracking wall/curb
	#define SEGMENT_FOLLOW_TURN_AMOUNT		10	// amount of turn servo to apply to track wall/curb

	#define	WAYPOINT_IN_RANGE			360	// within x Tenth_Inches, Look for Landmarks
	#define	WAYPOINT_STOP_DISTANCE		120	// Stop if within a foot of expected Waypoint
	#define OBJECT_COLLISION			  40 // Tenth_Inches - (IR Only) Anything closer than this is a potential collision
	#define OBJECT_AVOID_DEFAULT_FEET	  4	// Feet! - used to initialize g_GlobalAvoidObjectRangeFeet, which is the actual value used in tests
	#define CAMERA_SCAN_DIST_DEFAULT	 13	// Feet where camera starts really looking hard for the cone
	#define CAMERA_TO_WHEEL_SCALE		  2	// multiplier between Camera Servo ticks and Wheel Servo ticks - more = more oversteer


#else
	#error BAD MOTOR_CONTROL_TYPE!
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CAMERA CONTROL TYPE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if( ROBOT_TYPE == CARBOT )
	#if( CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA )

		// Camera motion range in Tenth Degrees
		// Sony camera can do 0 to 300 degrees (or +/- 149 degrees to be safe) max
		#define CAMERA_PAN_CENTER					0		// TenthDegrees from center
		#define CAMERA_TILT_CENTER					0		// TenthDegrees from center

		#define CAMERA_PAN_TENTHDEGREES_MAX_RIGHT	( 89 * 10)	// +89 degrees
		#define CAMERA_PAN_TENTHDEGREES_MAX_LEFT	(-89 * 10)	// -89 degrees
		#define CAMERA_TILT_TENTHDEGREES_MAX_UP		( 19 * 10)	// +19 degrees
		#define CAMERA_TILT_TENTHDEGREES_MAX_DOWN	(-19 * 10)	// -19 degrees

		#define CAMERA_SENSE_RANGE				240		// Max distance Camera can spot a cone reliably
		#define SONY_CAMERA_MAX_RIGHT			870
		#define SONY_CAMERA_MAX_LEFT		  (-870)
		#define SONY_CAMERA_MAX_UP				290
		#define SONY_CAMERA_MAX_DOWN		  (-270)
		#define SONY_CAMERA_PAN_PER_DEGREE		7.8		// 870/90 degrees then experiment
		#define SONY_CAMERA_TILT_PER_DEGREE	    7.0		// 280/20 degrees then experiment
		#define SONY_CAMERA_MIN_ZOOM_ABS	 0x0000
		#define SONY_CAMERA_MAX_ZOOM_ABS	 0x03FF		// As specified in Sony docs


		#define CAMERA_PAN_PER_DEGREE			SONY_CAMERA_PAN_PER_DEGREE
		#define CAMERA_TILT_PER_DEGREE			SONY_CAMERA_TILT_PER_DEGREE

		#define CAMERA_PAN_SERVO_RATIO			10	// Pixel value to camera movement divisor
		#define CAMERA_TILT_SERVO_RATIO			10


		#define CAMERA_PAN_SERVO_INCREMENT		 10		// ticks per reading
		#define CAMERA_PAN_SERVO_CENTER			  0		// Trim Right = larger number
		#define CAMERA_PAN_SCAN_START_RIGHT		(CAMERA_PAN_SERVO_CENTER + (8*CAMERA_PAN_SERVO_INCREMENT))
		#define CAMERA_PAN_SCAN_START_LEFT		(CAMERA_PAN_SERVO_CENTER - (8*CAMERA_PAN_SERVO_INCREMENT))
		#define CAMERA_PAN_SERVO_MAX_RIGHT		SONY_CAMERA_MAX_RIGHT		// Max Right
		#define CAMERA_PAN_SERVO_MAX_LEFT		SONY_CAMERA_MAX_LEFT		// Max Left
		#define CAMERA_PAN_TILT_ABS_UNDEFINED	0xFE	// Servo positions are in an unknown state

		#define CAMERA_TILT_SERVO_CENTER		  0 	// Trim Up = larger number
		#define CAMERA_TILT_SERVO_MAX_UP		SONY_CAMERA_MAX_UP		// Max Up
		#define CAMERA_TILT_SERVO_MAX_DOWN		SONY_CAMERA_MAX_DOWN	// Max Down

		#define CAMERA_TILT_CURB				(CAMERA_TILT_SERVO_CENTER - 15)	// Point slightly down to see curbs
		#define CAMERA_PAN_CURB_RIGHT			(CAMERA_PAN_SERVO_MAX_RIGHT - 15)
		#define CAMERA_PAN_CURB_LEFT			(CAMERA_PAN_SERVO_MAX_LEFT + 15)

		#define CAMERA_TILT_FACE				(CAMERA_TILT_SERVO_CENTER + 200)	// Point upward to see faces. Max up is 290

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#elif( CAMERA_CONTROL_TYPE == PIC_SERVO_CONTROL )

		#define CAMERA_PAN_CENTER					0		// TenthDegrees from center
		#define CAMERA_TILT_CENTER					0		// TenthDegrees from center

		// Camera motion range in Tenth Degrees
		// TODO - Need to set these to correct values!!!
		// Carbot camera can do ??? degrees (or +/- ??? degrees to be safe) max
		#define CAMERA_PAN_TENTHDEGREES_MAX_RIGHT	( 59 * 10)	// +59 degrees
		#define CAMERA_PAN_TENTHDEGREES_MAX_LEFT	(-59 * 10)	// -59 degrees
		#define CAMERA_TILT_TENTHDEGREES_MAX_UP		( 19 * 10)	// +19 degrees
		#define CAMERA_TILT_TENTHDEGREES_MAX_DOWN	(-10 * 10)	// -10 degrees

		#define CAMERA_SENSE_RANGE				240		// Max distance Camera can spot a cone reliably
		// Camera Pan/Tilt Constants
		#define CAMERA_PAN_SERVO_INCREMENT		  3		// ticks per reading
		#define CAMERA_PAN_SERVO_CENTER			 73		// Trim Right = larger number (63 default)
		#define CAMERA_PAN_SCAN_START_RIGHT	(CAMERA_PAN_SERVO_CENTER + (8*CAMERA_PAN_SERVO_INCREMENT))
		#define CAMERA_PAN_SCAN_START_LEFT	(CAMERA_PAN_SERVO_CENTER - (8*CAMERA_PAN_SERVO_INCREMENT))
		#define CAMERA_PAN_SERVO_MAX_RIGHT		120		// Max Right
		#define CAMERA_PAN_SERVO_MAX_LEFT		 30		// Max Left
		#define CAMERA_PAN_TILT_ABS_UNDEFINED	0xFE	// Servo positions are in an unknown state

		#define CAMERA_TILT_SERVO_CENTER		 63		// Trim Up = larger number (63 default)
		#define CAMERA_TILT_SERVO_MAX_UP		 72		// Max Up
		#define CAMERA_TILT_SERVO_MAX_DOWN		 48		// Max Down

		#define CAMERA_TILT_CURB				(CAMERA_TILT_SERVO_CENTER - 15)	// Point slightly down to see curbs
		#define CAMERA_PAN_CURB_RIGHT			(CAMERA_PAN_SERVO_MAX_RIGHT - 15)
		#define CAMERA_PAN_CURB_LEFT			(CAMERA_PAN_SERVO_MAX_LEFT + 15)

		#define CAMERA_TILT_FACE				(CAMERA_TILT_SERVO_MAX_UP)	// Point upward to see faces.  Not really used for Carbot

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#elif( CAMERA_CONTROL_TYPE == EXTERN_SERVO_CONTROL )

		// Camera Pan/Tilt Constants
		#define CAMERA_PAN_TILT_ABS_UNDEFINED	0xFE	// Servo positions are in an unknown state

		#define CAMERA_PAN_SERVO_MAX_LEFT		 55		// Max Left
		#define CAMERA_PAN_SERVO_CENTER			150		// Trim Right = larger number
		#define CAMERA_PAN_SERVO_MAX_RIGHT		245		// Max Right
		#define CAMERA_PAN_SERVO_INCREMENT		 10		// ticks per reading
		#define CAMERA_PAN_SCAN_START_RIGHT	(CAMERA_PAN_SERVO_CENTER + (8*CAMERA_PAN_SERVO_INCREMENT))
		#define CAMERA_PAN_SCAN_START_LEFT	(CAMERA_PAN_SERVO_CENTER - (8*CAMERA_PAN_SERVO_INCREMENT))

		#define CAMERA_TILT_SERVO_MAX_UP		240 	// Max Up
		#define CAMERA_TILT_SERVO_CENTER		110		// Trim Up = larger number
		#define CAMERA_TILT_SERVO_MAX_DOWN		 48 	// Max Down

		#define CAMERA_TILT_CURB				(CAMERA_TILT_SERVO_CENTER - 30)	// Point slightly down to see curbs
		#define CAMERA_PAN_CURB_RIGHT			(CAMERA_PAN_SERVO_MAX_RIGHT + 30)
		#define CAMERA_PAN_CURB_LEFT			(CAMERA_PAN_SERVO_MAX_LEFT - 30)

		#define CAMERA_TILT_FACE				(CAMERA_TILT_SERVO_MAX_UP - 30)	// Point upward to see faces.  Not really used for Carbot

	#else
		#error BAD CAMERA_CONTROL_TYPE
	#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( ROBOT_TYPE == LOKI )


	// Servo motion range in Tenth Degrees
	// Dynamixel AX12 and RX64 servos can do 0 to 300 degrees (or +/- 149 degrees to be safe) max
	// Dynamixel MX28 servos can do 0 to 359 degrees max, and have higher resolution, so more "ticks" per degree

	//	AX-12+ range is 0 - 0x3FF;  1023 = 300degrees
	//	TicksPerDegree = 1023ticks / 300 degrees = 3.41 ticks
	//	DegreesPerTick = 300degrees / 1023 ticks = 0.293 degrees

	#define DYNA_SERVO_TICKS_PER_TENTHDEGREE			0.341
	#define DYNA_SERVO_TENTHDEGREES_PER_TICK			2.932

	// MX-28 range is 0 - 0xFFF; 4095 = 359 degrees.  2048 = Centered
	#define DYNA_SERVO_MX_TICKS_PER_TENTHDEGREE		1.136	// MX28 has much higher precision
	#define DYNA_SERVO_MX_TENTHDEGREES_PER_TICK		0.880

	//#define KERR_SERVO_TICKS_PER_TENTHDEGREE	  0.341 // not used - see KerrControl.h
	//#define KERR_SERVO_TENTHDEGREES_PER_TICK	  2.932 //

	//////////////////////////////////////////////////////////////
	#define DYNA_MULTI_SERVO_ID						 0	// Identify Broadcast servo messages
	// Reserve ServoID 1 for new servos!
	#define DYNA_CAMERA_PAN_SERVO_ID				 2
	#define DYNA_CAMERA_TILT_SERVO_ID				 3
	#define DYNA_CAMERA_SIDETILT_SERVO_ID			 4

	#define NUMBER_OF_DYNA_SERVOS_IN_HEAD			 3

	//////////////////////////////////////////////////////////////
	// ARM SERVOS - Includes Shoulder Joint (even though it's not a Dyna Servo)

	#define ROBOT_HAS_RIGHT_ARM			 			TRUE

	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID	   5
	#define DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID		   6	// RX64
	#define DYNA_KINECT_SCANNER_SERVO_ID			   7	// Replaced arm servo with Kinect servo
	#define DYNA_RIGHT_ARM_WRIST_SERVO_ID			   8
	#define DYNA_RIGHT_ARM_CLAW_SERVO_ID			   9

	#define ROBOT_HAS_LEFT_ARM			 			TRUE

	#define DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID		  10
	#define DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID		  11	// RX64
	//#define DYNA_LEFT_ARM_ELBOW_BEND_SERVO2_ID		    12 -- Replaced by RX64 (servo 11) only!
	#define DYNA_LEFT_ARM_WRIST_SERVO_ID			  13
	#define DYNA_LEFT_ARM_CLAW_SERVO_ID				  14

	#define KERR_RIGHT_ARM_SHOULDER_SERVO_ID	 	  15	// Used for Command but not Status!
	#define KERR_LEFT_ARM_SHOULDER_SERVO_ID			  16	// Used for Command but not Status!

	#define NUMBER_OF_SMART_SERVOS					  16	// Total number of Smart Servos (Dyna+Kerr) + 1. Used for Bulk servo array, etc.
	#define NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM		   4

	#define KERR_ARM_MOTOR_ID_RIGHT					   1	// Right Arm motor.  Use in KerrControl Only!
	#define KERR_ARM_MOTOR_ID_LEFT					   2	// Left Arm motor.  Use in KerrControl Only!


	//////////////////////////////////////////////////////////////
	// Common Values in TICKS
/*
	// Right Arm
	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_MAX			4090	// MX28: Rotate Max Clockwise (180 degrees)
	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_MIN			   0	// Rotate Max Counter-clockwise (-180 degrees)

	#define DYNA_RIGHT_ARM_ELBOW_BEND_MAX			 984	// Max Curl - Same as Home2 position
	#define DYNA_RIGHT_ARM_ELBOW_BEND_MIN			 150	// Back-Bend (ouch!)

	#define DYNA_RIGHT_ARM_WRIST_ROTATE_MAX			3970	// Rotate Max Clockwise (150 degrees)
	#define DYNA_RIGHT_ARM_WRIST_ROTATE_MIN			   0	// Rotate Max Counter-Clockwise (-180 degrees) - allows for pouring drinks

	#define DYNA_RIGHT_ARM_CLAW_OPEN_MAX			3500	// Fully Open (90+ degrees)
	#define DYNA_RIGHT_ARM_CLAW_CLOSED_MIN			2225	// Closed Tight! (0 degrees)

	// Left Arm
	#define DYNA_LEFT_ARM_ELBOW_ROTATE_MAX			 930	// Rotate Max Clockwise (140 degrees)
	#define DYNA_LEFT_ARM_ELBOW_ROTATE_MIN			   0	// Rotate Max Counter-clockwise (-140 degrees)

	#define DYNA_LEFT_ARM_ELBOW_BEND_MIN			 95		// Min = Curl - Same as Home position 2
	#define DYNA_LEFT_ARM_ELBOW_BEND_MAX			793		// Back-Bend (ouch!) (-40 degrees)

	#define DYNA_LEFT_ARM_WRIST_ROTATE_MAX			1023	// Rotate Max Clockwise (150 degrees)
	#define DYNA_LEFT_ARM_WRIST_ROTATE_MIN			   0	// Rotoate Max Counter Clockwise (-150 degrees)

	#define DYNA_LEFT_ARM_CLAW_OPEN_MAX				955		// Fully Open (130 degrees)
	#define DYNA_LEFT_ARM_CLAW_CLOSED_MIN			 409	// Closed Tight!


	// Values in Servo Ticks:
	#define DYNA_CAMERA_SERVO_MAX_RIGHT			  	  0
	#define DYNA_CAMERA_SERVO_MAX_LEFT			   1021
	#define DYNA_CAMERA_SERVO_MAX_UP				348
	#define DYNA_CAMERA_SERVO_MAX_DOWN				750
//	#define DYNA_CAMERA_PAN_PER_TENTHDEGREE		  3.41		// 1023/300 degrees then experiment
//	#define DYNA_CAMERA_TILT_PER_TENTHDEGREE	  3.41		// Same?


//	#define CAMERA_PAN_PER_DEGREE			DYNA_CAMERA_PAN_PER_DEGREE
//	#define CAMERA_TILT_PER_DEGREE			DYNA_CAMERA_TILT_PER_DEGREE

//	#define CAMERA_PAN_SERVO_RATIO			10	// TODO-MUST Pixel value to camera movement divisor
//	#define CAMERA_TILT_SERVO_RATIO			10


	#define CAMERA_PAN_SERVO_INCREMENT		 10		// ticks per reading
	#define CAMERA_PAN_SERVO_CENTER			511		// Trim: Left = larger number
	#define CAMERA_PAN_SCAN_START_RIGHT		(CAMERA_PAN_SERVO_CENTER + (8*CAMERA_PAN_SERVO_INCREMENT))
	#define CAMERA_PAN_SCAN_START_LEFT		(CAMERA_PAN_SERVO_CENTER - (8*CAMERA_PAN_SERVO_INCREMENT))

	#define CAMERA_PAN_SERVO_MAX_RIGHT		DYNA_CAMERA_SERVO_MAX_RIGHT		// Max Right
	#define CAMERA_PAN_SERVO_MAX_LEFT		DYNA_CAMERA_SERVO_MAX_LEFT		// Max Left
	#define CAMERA_PAN_TILT_ABS_UNDEFINED	0xFE	// Servo positions are in an unknown state

	#define CAMERA_TILT_SERVO_CENTER		511 	// Trim: Down = larger number
	#define CAMERA_TILT_SERVO_MAX_UP		DYNA_CAMERA_SERVO_MAX_UP	// Max Up
	#define CAMERA_TILT_SERVO_MAX_DOWN		DYNA_CAMERA_SERVO_MAX_DOWN	// Max Down

	#define CAMERA_TILT_CURB				(CAMERA_TILT_SERVO_CENTER - 15)	// Point slightly down to see curbs
	#define CAMERA_PAN_CURB_RIGHT			(CAMERA_PAN_SERVO_MAX_RIGHT - 15)
	#define CAMERA_PAN_CURB_LEFT			(CAMERA_PAN_SERVO_MAX_LEFT + 15)


	// Loki Neck:
	#define CAMERA_NECK_SERVO_MAX_BACK				823		// Resting position
	#define CAMERA_NECK_SERVO_MAX_FORWARD			256		// Full forward
//	#define CAMERA_NECK_SERVO_PAN_ENABLE_POSITION	700		// Don't allow PAN if head further back then this!

	#define CAMERA_NECK_SERVO_BACK_POSITION			658		// Move back when startled
	#define CAMERA_NECK_SERVO_UP_POSITION			558		// Standard Up Position
	#define CAMERA_NECK_SERVO_INTEREST_POSITION		453		// Looking at something Interesting
	#define CAMERA_NECK_SERVO_FORWARD_POSITION		(CAMERA_NECK_SERVO_MAX_FORWARD - 20)	// Full Forward Position

	// Loki Curious Tilt:
	#define CAMERA_SIDETILT_SERVO_CENTER			511
	#define CAMERA_SIDETILT_SERVO_MAX_LEFT			389
	#define CAMERA_SIDETILT_SERVO_MAX_RIGHT			645

	#define CAMERA_SIDETILT_SERVO_TILT_LEFT			(CAMERA_SIDETILT_SERVO_CENTER - 100)
	#define CAMERA_SIDETILT_SERVO_TILT_RIGHT		(CAMERA_SIDETILT_SERVO_CENTER + 100)

	// "Rest" Servo Positions:
	#define CAMERA_NECK_SERVO_REST_POSITION			(CAMERA_NECK_SERVO_MAX_BACK - 5)
	#define CAMERA_PAN_SERVO_REST_POSITION			CAMERA_PAN_SERVO_CENTER
	#define CAMERA_TILT_SERVO_REST_POSITION			CAMERA_TILT_SERVO_CENTER		// Standard tilt when in "Rest" position
	#define CAMERA_SIDETILT_SERVO_REST_POSITION		CAMERA_SIDETILT_SERVO_CENTER	// 3 degree offset to Compensate for HW

//	#define CAMERA_TILT_SERVO_REST_LIMIT			350	// Max tilt when in Rest position

	// Laser Scanner:
	#define LASER_TILT_SERVO_CENTER					511
	#define LASER_TILT_SERVO_MAX_UP					0
	#define LASER_TILT_SERVO_MAX_DOWN				1021

*/

	//////////////////////////////////////////////////////////////
	// Common Values in TENTH DEGREES

	// compliance = 0.3degrees per step
	#define DYNA_COMPLIANCE_LIMIT_MIN			     1	//  1 ( 0.3deg) - VERY Tight compliance!
	#define DYNA_COMPLIANCE_VERY_TIGHT			     2	//  2 ( 0.6deg) - Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_TIGHT				     4	//  4 ( 1.2deg) - Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_MED_TIGHT			     8	//  8 ( 2.4deg) - Med Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_MED					    16	// 16 ( 4.8deg) - Medium Compliance
	#define DYNA_COMPLIANCE_NORMAL				    32	// 32 ( 9.6deg) - Default Compliance
	#define DYNA_COMPLIANCE_LIMIT_MAX			    64	// 64 (19.2deg) - Loose Compliance


	// Tolerance for Joint positioning.  Joint within this will be considered at commanded position
	#define DYNA_SERVO_DELTA_DEFAULT_TENTHDEGREES				60	// = 6 degrees.
	#define ARM_JOINT_DELTA_MAX_TENTHDEGREES					60	// = 6 degrees.
	#define HEAD_JOINT_DELTA_MAX_TENTHDEGREES					30	// = 3 degrees.
	#define HEAD_JOINT_DELTA_HIGH_PRECISION_TENTHDEGREES		15	// = 1.5 degrees.
	#define KINECT_JOINT_DELTA_MAX_TENTHDEGREES					30	// = 3.0 degrees.
	#define KINECT_SERVO_TOLERANCE_NORMAL_TENTHDEGREES			30	// = 3 degrees.
	#define KINECT_SERVO_TOLERANCE_HIGH_PRECISION_TENTHDEGREES	 8	// = 0.8 degrees.


	// Laser Scanner
	// Position for scanning with laser for objects on the floor
	#define LASER_FLOOR_SCAN_THRESHOLD_TENTH_DEGREES		(-200)	// Don't detect objects if laser pointed higher than this.
																		// Obsolete?: If camera pointing down more then 20 degrees, compensate for small mechanical alignment changes
	#define LASER_FLOOR_SCAN_FAR_TENTH_DEGREES			(-160)	// 16.0 degrees down for looking for objects on the floor.  (~7 feet from robot)
	//        ^---- Warning!  too far to scan - max 99 scan lines!
	#define LASER_FLOOR_SCAN_5_FEET_TENTH_DEGREES		(-240)	// 24.0 degrees down for looking for objects on the floor.  (~5 feet from robot)
	#define LASER_FLOOR_SCAN_MEDIUM_TENTH_DEGREES		(-350)	// 35.0 degrees down for looking for objects on the floor.  (~? feet from robot)
	#define LASER_FLOOR_SCAN_NEAR_TENTH_DEGREES			(-600)	// 60.0 degrees down for objects near grasping range of robot (~ 12" from robot)
	#define LASER_FLOOR_SCAN_MIN_TENTH_DEGREES			(-760)	// Closest object detection (~ 3.5" from robot)

	// Position for Laser Scan Positioning (level laser) for SLAM
	#define LASER_TILT_TENTHDEGREES_SLAM_POSITION			0	// Head tilted up, so laser is level


	#define CAMERA_PAN_CENTER									0		// TenthDegrees from center  See CAMERA_PAN_TENTH_DEGREES_ZERO
	#define CAMERA_TILT_CENTER									0		// TenthDegrees from center See CAMERA_TILT_TENTH_DEGREES_ZERO
	#define CAMERA_SIDETILT_CENTER								0		// 3 deg offset to compensate for HW?
	#define CAMERA_HUMAN_DETECT_START_POSITION				(  25 * 10)	// +25 degrees - normal position for tracking humans
	#define KINECT_TILT_CENTER									0		// TenthDegrees from center See KINECT_TILT_TENTH_DEGREES_ZERO
	#define KINECT_HUMAN_DETECT_START_POSITION				(  25 * 10)	// +25 degrees - normal position for tracking humans
	#define KINECT_OBJECT_AVOIDANCE_POSITION				( -20 * 10)	// degrees - normal position for avoiding objects - Don't tilt too low, or Voice reco does not work!
	#define KINECT_SLEEP_POSITION							( -45 * 10)	// -45 degrees - keep dust off the lense

	#define KINECT_TILT_TENTHDEGREES_MAX_UP					(  65 * 10)	//  +65 degrees
	#define KINECT_TILT_TENTHDEGREES_MAX_DOWN				( -65 * 10)	//  -65 degrees
	//#define KINECT_TILT_TENTHDEGREES_NAVIGATE				( 20 * 10)	//  degrees - when looking for obstacles while navigating
	//#define KINECT_TILT_TENTHDEGREES_PEOPLE				( 20 * 10)	//  degrees - when looking for People

	#define CAMERA_PAN_TENTHDEGREES_MAX_RIGHT				( 149 * 10)	// +149 degrees
	#define CAMERA_PAN_TENTHDEGREES_MAX_LEFT				(-149 * 10)	// -149 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_UP					(  34 * 10)	//  +34 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_DOWN				( -72 * 10)	//  -72 degrees

	// Limits for how far head will move while tracking people
	#define CAMERA_PAN_TENTHDEGREES_MAX_TRACK_RIGHT			(  80 * 10)	// +80 degrees
	#define CAMERA_PAN_TENTHDEGREES_MAX_TRACK_LEFT			( -80 * 10)	// -80 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_TRACK_UP			(  38 * 10)	//  +40 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_TRACK_DOWN			( -20 * 10)	//  -20 degrees

	#define CAMERA_TILT_TENTHDEGREES_TRACK_LASER_MAX_UP		( -33 * 10)	// While tracking laser spot, keep head lower then this

	#define CAMERA_TILT_SLEEP_POSITION						( -68 * 10)	// Point down to "sleep"
	#define CAMERA_TILT_FACE_POSITION						(  15 * 10)	// Point upward to see faces.
	#define CAMERA_TILT_LASER_LEVEL							(  13 * 10)	// Laser is level - used for finding walls
	#define CAMERA_TILT_ROBOT_MOVING						( -25 * 10)	// Point down to look for obstacles while moving (37" out)

	#define CAMERA_PAN_PIR_RIGHT							(  30 * 10)	// Look to the right if PIR sensor detects motion
	#define CAMERA_PAN_PIR_LEFT								( -30 * 10)	// Look to the left if PIR sensor detects motion
	#define CAMERA_PAN_PIR_TENTHDEGREES						( 5 * 10)	// amount to move if PIR sensor detects thermal object



	// Loki Curious Tilt:
	#define CAMERA_SIDETILT_TENTHDEGREES_MAX_LEFT			(  21 * 10)	// +19 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_MAX_RIGHT			( -21 * 10)	// -19 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT			(  17 * 10)	// +15 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_TILT_RIGHT			( -17 * 10)	// -15 degrees

	#define CAMERA_NECK_TENTHDEGREES_MAX_FORWARD			(  69 * 10)	// +69 degrees
	#define CAMERA_NECK_TENTHDEGREES_MAX_BACK				( -89 * 10)	// -89 degrees

	#define CAMERA_NECK_TENTHDEGREES_BACK_POSITION			( -30 * 10)	// Move back when startled
	#define CAMERA_NECK_TENTHDEGREES_UP_POSITION			(  -8 * 10)	// Standard Up Position
	#define CAMERA_NECK_TENTHDEGREES_INTEREST_POSITION		(  20 * 10)	// Looking at something Interesting
	#define CAMERA_NECK_TENTHDEGREES_FORWARD_POSITION		(CAMERA_NECK_TENTHDEGREES_MAX_FORWARD - 20)	// Full Forward Position

	// "Rest" Positions:
	#define CAMERA_NECK_TENTHDEGREES_REST_POSITION			( -85 * 10)
	#define CAMERA_PAN_TENTHDEGREES_REST_POSITION			CAMERA_PAN_CENTER
	#define CAMERA_TILT_TENTHDEGREES_REST_POSITION			CAMERA_TILT_CENTER
	#define CAMERA_SIDETILT_TENTHDEGREES_REST_POSITION		CAMERA_SIDETILT_CENTER

//	#define CAMERA_TILT_TENTHDEGREES_REST_LIMIT				(  30 * 10)		// Max tilt when in Rest position
//	#define CAMERA_NECK_TENTHDEGREES_PAN_ENABLE_POSITION	( -60 * 10)		// Don't allow PAN if head further back then this!


// Misc stuff

	#define DYNA_CAMERA_MIN_ZOOM_ABS			0x0000
	#define DYNA_CAMERA_MAX_ZOOM_ABS			0x0001		// TODO - NOT SUPPORTED

	#define CAMERA_SENSE_RANGE				240		// Max distance Camera can spot a cone reliably

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif( ROBOT_TYPE == TURTLE )

	// Servo motion range in Tenth Degrees
	// Dynamixel AX12 and RX64 servos can do 0 to 300 degrees (or +/- 149 degrees to be safe) max
	// Dynamixel MX28 servos can do 0 to 359 degrees max, and have higher resolution, so more "ticks" per degree

	//	AX-12+ range is 0 - 0x3FF;  1023 = 300degrees
	//	TicksPerDegree = 1023ticks / 300 degrees = 3.41 ticks
	//	DegreesPerTick = 300degrees / 1023 ticks = 0.293 degrees

	#define DYNA_SERVO_TICKS_PER_TENTHDEGREE			0.341
	#define DYNA_SERVO_TENTHDEGREES_PER_TICK			2.932

	// MX-28 range is 0 - 0xFFF; 4095 = 359 degrees.  2048 = Centered
	#define DYNA_SERVO_MX_TICKS_PER_TENTHDEGREE		1.136	// MX28 has much higher precision
	#define DYNA_SERVO_MX_TENTHDEGREES_PER_TICK		0.880


	//////////////////////////////////////////////////////////////
	#define DYNA_MULTI_SERVO_ID						 0	// Identify Broadcast servo messages
	// Reserve ServoID 1 for new servos!
	#define DYNA_CAMERA_PAN_SERVO_ID				 2
	#define DYNA_CAMERA_TILT_SERVO_ID				 3
	#define DYNA_CAMERA_SIDETILT_SERVO_ID			 4

	#define NUMBER_OF_DYNA_SERVOS_IN_HEAD			 2 // 3 // TODO-MUST - Kludge for telepresence robot

	//////////////////////////////////////////////////////////////
	// ARM SERVOS - Includes Shoulder Joint (even though it's not a Dyna Servo)

	#define ROBOT_HAS_RIGHT_ARM			 			FALSE

	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID	   5
	#define DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID		   6

	#define DYNA_KINECT_SCANNER_SERVO_ID			   7	// Replaced arm servo with Kinect servo

	#define DYNA_RIGHT_ARM_WRIST_SERVO_ID			   8
	#define DYNA_RIGHT_ARM_CLAW_SERVO_ID			   9

	#define ROBOT_HAS_LEFT_ARM			 			FALSE

	#define DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID		  10
	#define DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID		  11
	//#define DYNA_LEFT_ARM_ELBOW_BEND_SERVO2_ID		  12 -- Replaced by RX64 (servo 11) only!
	#define DYNA_LEFT_ARM_WRIST_SERVO_ID			  13
	#define DYNA_LEFT_ARM_CLAW_SERVO_ID				  14

	#define KERR_RIGHT_ARM_SHOULDER_SERVO_ID	 	  15	// Used for Command but not Status!
	#define KERR_LEFT_ARM_SHOULDER_SERVO_ID			  16	// Used for Command but not Status!

	#define NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM		   4

	#define KERR_ARM_MOTOR_ID_RIGHT					   1	// Right Arm motor.  Use in KerrControl Only!
	#define KERR_ARM_MOTOR_ID_LEFT					   2	// Left Arm motor.  Use in KerrControl Only!


	#define NUMBER_OF_SMART_SERVOS					  16	// Total number of Smart Servos (Dyna+Kerr) + 1. Used for Bulk servo array, etc.


	//////////////////////////////////////////////////////////////
	// Common Values in TICKS
	// Right Arm
/*** NO LONGER USED ***
	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_MAX			 930	// Rotate Max Clockwise (140 degrees)
	#define DYNA_RIGHT_ARM_ELBOW_ROTATE_MIN			   0	// Rotate Max Counter-clockwise (-140 degrees)

	#define DYNA_RIGHT_ARM_ELBOW_BEND_MAX			 984	// Max Curl - Same as Home2 position
	#define DYNA_RIGHT_ARM_ELBOW_BEND_MIN			 150	// Back-Bend (ouch!)

	#define DYNA_RIGHT_ARM_WRIST_ROTATE_MAX			1023	// Rotate Max Clockwise (150 degrees)
	#define DYNA_RIGHT_ARM_WRIST_ROTATE_MIN			   0	// Rotoate Max Counter Clockwise (-150 degrees)

	#define DYNA_RIGHT_ARM_CLAW_OPEN_MAX			 205	// Fully Open (90 degrees)
	#define DYNA_RIGHT_ARM_CLAW_CLOSED_MIN			 512	// Closed Tight! (0 degrees)

	// Left Arm
	#define DYNA_LEFT_ARM_ELBOW_ROTATE_MAX			 930	// Rotate Max Clockwise (140 degrees)
	#define DYNA_LEFT_ARM_ELBOW_ROTATE_MIN			   0	// Rotate Max Counter-clockwise (-140 degrees)

	#define DYNA_LEFT_ARM_ELBOW_BEND_MIN			 95		// Min = Curl - Same as Home position 2
	#define DYNA_LEFT_ARM_ELBOW_BEND_MAX			793		// Back-Bend (ouch!) (-40 degrees)

	#define DYNA_LEFT_ARM_WRIST_ROTATE_MAX			1023	// Rotate Max Clockwise (150 degrees)
	#define DYNA_LEFT_ARM_WRIST_ROTATE_MIN			   0	// Rotoate Max Counter Clockwise (-150 degrees)

	#define DYNA_LEFT_ARM_CLAW_OPEN_MAX				955		// Fully Open (130 degrees)
	#define DYNA_LEFT_ARM_CLAW_CLOSED_MIN			 409	// Closed Tight!


	// Values in Servo Ticks:
	#define DYNA_CAMERA_SERVO_MAX_RIGHT			  	  0
	#define DYNA_CAMERA_SERVO_MAX_LEFT			   1021
	#define DYNA_CAMERA_SERVO_MAX_UP				348
	#define DYNA_CAMERA_SERVO_MAX_DOWN				750
//	#define DYNA_CAMERA_PAN_PER_TENTHDEGREE		  3.41		// 1023/300 degrees then experiment
//	#define DYNA_CAMERA_TILT_PER_TENTHDEGREE	  3.41		// Same?


//	#define CAMERA_PAN_PER_DEGREE			DYNA_CAMERA_PAN_PER_DEGREE
//	#define CAMERA_TILT_PER_DEGREE			DYNA_CAMERA_TILT_PER_DEGREE

//	#define CAMERA_PAN_SERVO_RATIO			10	// TODO-MUST Pixel value to camera movement divisor
//	#define CAMERA_TILT_SERVO_RATIO			10
***/

	#define CAMERA_PAN_SERVO_INCREMENT		 10		// ticks per reading
	#define CAMERA_PAN_SERVO_CENTER			511		// Trim: Left = larger number
	#define CAMERA_PAN_SCAN_START_RIGHT		(CAMERA_PAN_SERVO_CENTER + (8*CAMERA_PAN_SERVO_INCREMENT))
	#define CAMERA_PAN_SCAN_START_LEFT		(CAMERA_PAN_SERVO_CENTER - (8*CAMERA_PAN_SERVO_INCREMENT))

	#define CAMERA_PAN_SERVO_MAX_RIGHT		DYNA_CAMERA_SERVO_MAX_RIGHT		// Max Right
	#define CAMERA_PAN_SERVO_MAX_LEFT		DYNA_CAMERA_SERVO_MAX_LEFT		// Max Left
	#define CAMERA_PAN_TILT_ABS_UNDEFINED	0xFE	// Servo positions are in an unknown state

	#define CAMERA_TILT_SERVO_CENTER		511 	// Trim: Down = larger number
	#define CAMERA_TILT_SERVO_MAX_UP		DYNA_CAMERA_SERVO_MAX_UP	// Max Up
	#define CAMERA_TILT_SERVO_MAX_DOWN		DYNA_CAMERA_SERVO_MAX_DOWN	// Max Down

	#define CAMERA_TILT_CURB				(CAMERA_TILT_SERVO_CENTER - 15)	// Point slightly down to see curbs
	#define CAMERA_PAN_CURB_RIGHT			(CAMERA_PAN_SERVO_MAX_RIGHT - 15)
	#define CAMERA_PAN_CURB_LEFT			(CAMERA_PAN_SERVO_MAX_LEFT + 15)


	// Loki Neck:
	#define CAMERA_NECK_SERVO_MAX_BACK				823		// Resting position
	#define CAMERA_NECK_SERVO_MAX_FORWARD			256		// Full forward
//	#define CAMERA_NECK_SERVO_PAN_ENABLE_POSITION	700		// Don't allow PAN if head further back then this!

	#define CAMERA_NECK_SERVO_BACK_POSITION			658		// Move back when startled
	#define CAMERA_NECK_SERVO_UP_POSITION			558		// Standard Up Position
	#define CAMERA_NECK_SERVO_INTEREST_POSITION		453		// Looking at something Interesting
	#define CAMERA_NECK_SERVO_FORWARD_POSITION		(CAMERA_NECK_SERVO_MAX_FORWARD - 20)	// Full Forward Position

	// Loki Curious Tilt:
	#define CAMERA_SIDETILT_SERVO_CENTER			511
	#define CAMERA_SIDETILT_SERVO_MAX_LEFT			389
	#define CAMERA_SIDETILT_SERVO_MAX_RIGHT			645

	#define CAMERA_SIDETILT_SERVO_TILT_LEFT			(CAMERA_SIDETILT_SERVO_CENTER - 100)
	#define CAMERA_SIDETILT_SERVO_TILT_RIGHT		(CAMERA_SIDETILT_SERVO_CENTER + 100)

	// "Rest" Servo Positions:
	#define CAMERA_NECK_SERVO_REST_POSITION			(CAMERA_NECK_SERVO_MAX_BACK - 5)
	#define CAMERA_PAN_SERVO_REST_POSITION			CAMERA_PAN_SERVO_CENTER
	#define CAMERA_TILT_SERVO_REST_POSITION			CAMERA_TILT_SERVO_CENTER		// Standard tilt when in "Rest" position
	#define CAMERA_SIDETILT_SERVO_REST_POSITION		CAMERA_SIDETILT_SERVO_CENTER	// 3 degree offset to Compensate for HW

//	#define CAMERA_TILT_SERVO_REST_LIMIT			350	// Max tilt when in Rest position

	// Laser Scanner:
	#define LASER_TILT_SERVO_CENTER					511
	#define LASER_TILT_SERVO_MAX_UP					0
	#define LASER_TILT_SERVO_MAX_DOWN				1021

	//////////////////////////////////////////////////////////////
	// Common Values in TENTH DEGREES

	#define DYNA_TORQUE_LIMIT_MAX				  0x03FF	// 1023

	// compliance = 0.3degrees per step
	#define DYNA_COMPLIANCE_LIMIT_MIN			     1	//  1 ( 0.3deg) - VERY Tight compliance!
	#define DYNA_COMPLIANCE_VERY_TIGHT			     2	//  2 ( 0.6deg) - Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_TIGHT				     4	//  4 ( 1.2deg) - Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_MED_TIGHT			     8	//  8 ( 2.4deg) - Med Tight Compliance, Head/Kinect servos only?
	#define DYNA_COMPLIANCE_MED					    16	// 16 ( 4.8deg) - Medium Compliance
	#define DYNA_COMPLIANCE_NORMAL				    32	// 32 ( 9.6deg) - Default Compliance
	#define DYNA_COMPLIANCE_LIMIT_MAX			    64	// 64 (19.2deg) - Loose Compliance


	// Tolerance for Joint positioning.  Joint within this will be considered at commanded position
	#define DYNA_SERVO_DELTA_DEFAULT_TENTHDEGREES				60	// = 6 degrees.
	#define ARM_JOINT_DELTA_MAX_TENTHDEGREES					60	// = 6 degrees.
	#define HEAD_JOINT_DELTA_MAX_TENTHDEGREES					30	// = 3 degrees.
	#define HEAD_JOINT_DELTA_HIGH_PRECISION_TENTHDEGREES		15	// = 1.5 degrees.
	#define KINECT_JOINT_DELTA_MAX_TENTHDEGREES					30	// = 3.0 degrees.
	#define KINECT_SERVO_TOLERANCE_NORMAL_TENTHDEGREES			30	// = 3 degrees.
	#define KINECT_SERVO_TOLERANCE_HIGH_PRECISION_TENTHDEGREES	 8	// = 0.8 degrees.


	// Laser Scanner
	// Position for scanning with laser for objects on the floor
	#define LASER_FLOOR_SCAN_THRESHOLD_TENTH_DEGREES		(-200)	// Don't detect objects if laser pointed higher than this.
																		// Obsolete?: If camera pointing down more then 20 degrees, compensate for small mechanical alignment changes
	#define LASER_FLOOR_SCAN_FAR_TENTH_DEGREES			(-160)	// 16.0 degrees down for looking for objects on the floor.  (~7 feet from robot)
	//        ^---- Warning!  too far to scan - max 99 scan lines!
	#define LASER_FLOOR_SCAN_5_FEET_TENTH_DEGREES		(-240)	// 24.0 degrees down for looking for objects on the floor.  (~5 feet from robot)
	#define LASER_FLOOR_SCAN_MEDIUM_TENTH_DEGREES		(-350)	// 35.0 degrees down for looking for objects on the floor.  (~? feet from robot)
	#define LASER_FLOOR_SCAN_NEAR_TENTH_DEGREES			(-600)	// 60.0 degrees down for objects near grasping range of robot (~ 12" from robot)
	#define LASER_FLOOR_SCAN_MIN_TENTH_DEGREES			(-760)	// Closest object detection (~ 3.5" from robot)

	// Position for Laser Scan Positioning (level laser) for SLAM
	#define LASER_TILT_TENTHDEGREES_SLAM_POSITION			0	// Head tilted up, so laser is level


	#define CAMERA_PAN_CENTER									0		// TenthDegrees from center  See CAMERA_PAN_TENTH_DEGREES_ZERO
	#define CAMERA_TILT_CENTER									0		// TenthDegrees from center See CAMERA_TILT_TENTH_DEGREES_ZERO
	#define CAMERA_SIDETILT_CENTER								0		// 3 deg offset to compensate for HW?
	#define CAMERA_HUMAN_DETECT_START_POSITION				(  15 * 10)	// +25 degrees - normal position for tracking humans
	#define KINECT_TILT_CENTER									0		// TenthDegrees from center See LASER_TILT_TENTH_DEGREES_ZERO
	#define KINECT_HUMAN_DETECT_START_POSITION				( 15 * 10)	// +25 degrees - normal position for tracking humans
	#define KINECT_OBJECT_AVOIDANCE_POSITION				(-60 * 10)	// degrees - normal position for avoiding objects - for TeleOp!
	#define KINECT_SLEEP_POSITION							(0 * 10)	// degrees - Just leave neutral for Teleop

	#define KINECT_TILT_TENTHDEGREES_MAX_UP					(  65 * 10)	//  +65 degrees
	#define KINECT_TILT_TENTHDEGREES_MAX_DOWN				( -90 * 10)	//  -90 degrees for Teleop Robot
	#define KINECT_TILT_TENTHDEGREES_LONG_RANGE				( -25 * 10)	//  24" - Max range (~20 feet?)
	#define KINECT_TILT_TENTHDEGREES_MID_RANGE				( -40 * 10)	//  6" - 60" (almost 6 feet)
	#define KINECT_TILT_TENTHDEGREES_CLOSE_RANGE			( -55 * 10)	//  6" - 28" (including 6 in on front-sides)
	//#define KINECT_TILT_TENTHDEGREES_NAVIGATE				( -50 * 10)	//  degrees - when looking for obstacles while navigating
	//#define KINECT_TILT_TENTHDEGREES_PEOPLE				( 20 * 10)	//  degrees - when looking for People

	#define CAMERA_PAN_TENTHDEGREES_MAX_RIGHT				( 149 * 10)	// +149 degrees
	#define CAMERA_PAN_TENTHDEGREES_MAX_LEFT				(-149 * 10)	// -149 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_UP					(  34 * 10)	//  +34 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_DOWN				( -72 * 10)	//  -72 degrees

	// Limits for how far head will move while tracking people
	#define CAMERA_PAN_TENTHDEGREES_MAX_TRACK_RIGHT			(  80 * 10)	// +80 degrees
	#define CAMERA_PAN_TENTHDEGREES_MAX_TRACK_LEFT			( -80 * 10)	// -80 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_TRACK_UP			(  38 * 10)	//  +40 degrees
	#define CAMERA_TILT_TENTHDEGREES_MAX_TRACK_DOWN			( -20 * 10)	//  -20 degrees

	#define CAMERA_TILT_TENTHDEGREES_TRACK_LASER_MAX_UP		( -33 * 10)	// While tracking laser spot, keep head lower then this

	#define CAMERA_TILT_SLEEP_POSITION						( -68 * 10)	// Point down to "sleep"
	#define CAMERA_TILT_FACE_POSITION						(  15 * 10)	// Point upward to see faces.
	#define CAMERA_TILT_LASER_LEVEL							(  13 * 10)	// Laser is level - used for finding walls
	#define CAMERA_TILT_ROBOT_MOVING						( -50 * 10)	// Point down to look for obstacles while moving

	#define CAMERA_PAN_PIR_RIGHT							(  30 * 10)	// Look to the right if PIR sensor detects motion
	#define CAMERA_PAN_PIR_LEFT								( -30 * 10)	// Look to the left if PIR sensor detects motion
	#define CAMERA_PAN_PIR_TENTHDEGREES						( 5 * 10)	// amount to move if PIR sensor detects thermal object



	// Loki Curious Tilt:
	#define CAMERA_SIDETILT_TENTHDEGREES_MAX_LEFT			(  21 * 10)	// +19 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_MAX_RIGHT			( -21 * 10)	// -19 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT			(  17 * 10)	// +15 degrees
	#define CAMERA_SIDETILT_TENTHDEGREES_TILT_RIGHT			( -17 * 10)	// -15 degrees

	#define CAMERA_NECK_TENTHDEGREES_MAX_FORWARD			(  69 * 10)	// +69 degrees
	#define CAMERA_NECK_TENTHDEGREES_MAX_BACK				( -89 * 10)	// -89 degrees

	#define CAMERA_NECK_TENTHDEGREES_BACK_POSITION			( -30 * 10)	// Move back when startled
	#define CAMERA_NECK_TENTHDEGREES_UP_POSITION			(  -8 * 10)	// Standard Up Position
	#define CAMERA_NECK_TENTHDEGREES_INTEREST_POSITION		(  20 * 10)	// Looking at something Interesting
	#define CAMERA_NECK_TENTHDEGREES_FORWARD_POSITION		(CAMERA_NECK_TENTHDEGREES_MAX_FORWARD - 20)	// Full Forward Position

	// "Rest" Positions:
	#define CAMERA_NECK_TENTHDEGREES_REST_POSITION			( -85 * 10)
	#define CAMERA_PAN_TENTHDEGREES_REST_POSITION			CAMERA_PAN_CENTER
	#define CAMERA_TILT_TENTHDEGREES_REST_POSITION			CAMERA_TILT_CENTER
	#define CAMERA_SIDETILT_TENTHDEGREES_REST_POSITION		CAMERA_SIDETILT_CENTER

//	#define CAMERA_TILT_TENTHDEGREES_REST_LIMIT				(  30 * 10)		// Max tilt when in Rest position
//	#define CAMERA_NECK_TENTHDEGREES_PAN_ENABLE_POSITION	( -60 * 10)		// Don't allow PAN if head further back then this!


// Misc stuff

	#define DYNA_CAMERA_MIN_ZOOM_ABS			0x0000
	#define DYNA_CAMERA_MAX_ZOOM_ABS			0x0001		// TODO - NOT SUPPORTED

	#define CAMERA_SENSE_RANGE				240		// Max distance Camera can spot a cone reliably





#else
	#error BAD ROBOT_TYPE
#endif



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Non-Specific settings
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define WAYPOINT_SENSOR_RANGE			 960		// Distance (Tenth_Inches) from waypoint to start looking for landmarks
#define WAYPOINT_STOP_RANGE				 480		// Distance (inches) from waypoint landmark to stop, and head to next landmark
#define WAYPOINT_BACKUP_RANGE			 360		// If closer than this, backup before heading to next landmark

// Camera Pan/Tilt control
#define US_SCAN_MAX_SAMPLES			     16		// See "US1_SAMPLES"

// Camera Control parameters (used in WM_ROBOT_CAMERA_PAN_CMD, and by Arduino)
#define CAMERA_PAN_STOP						0x00
#define CAMERA_PAN_UP						0x01
#define CAMERA_PAN_DOWN						0x02
#define CAMERA_PAN_LEFT						0x03
#define CAMERA_PAN_RIGHT					0x04
#define CAMERA_PAN_UP_LEFT					0x05
#define CAMERA_PAN_UP_RIGHT					0x06
#define CAMERA_PAN_DOWN_LEFT				0x07
#define CAMERA_PAN_DOWN_RIGHT				0x08
#define CAMERA_PAN_ABS_CENTER				0x09	// Absolute Center Command

// ABS Camera Speed settings
//#define CAMERA_SPEED_VERY_SLOW				1		// Range 1 to SERVO_SPEED_MAX (15)
//#define CAMERA_SPEED_SLOW					4
//#define CAMERA_SPEED_MED					8
//#define CAMERA_SPEED_FAST					12
//#define CAMERA_SPEED_MAX					SERVO_SPEED_MAX

/*
#define SIO_SYNC_0							0xE5	// Serial Command Sync characters
#define SIO_SYNC_1							0x5F
#define CMD_TERM_CHAR						0xC4	// Serial Command termination character (use for check)
#define SERIAL_CMD_SIZE 					   8	// Fixed length of a command from the Host to the Arduino

#define LED_EYES_OFF						0x00
#define LED_EYES_ON							0x01
#define LED_EYES_CLOSE						0x02
#define LED_EYES_OPEN						0x03
#define LED_EYES_BLINK						0x04
#define LED_EYE_BRIGHTNESS_MAX				  19	// On time in 20ms cycle
*/



#endif	//__ROBOT_HW_INTERFACE_OTHER_H__
