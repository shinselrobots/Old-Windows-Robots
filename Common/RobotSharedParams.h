#ifndef __ROBOT_SHARED_PARAMS_H__
#define __ROBOT_SHARED_PARAMS_H__

#include "RobotType.h"

// Common defines used by Robot Client and Server
// But not required by specific hardware, such as Arduino

#define SERVER_PORT    50000
#define CLIENT_PORT    50001

#define DEFAULT_PATH_FILE			ROBOT_DATA_PATH "\\DefaultPath.pat"
#define DEFAULT_MAP_FILE			ROBOT_DATA_PATH "\\BlankMap.rmp"
#define DEFAULT_UPSTAIRS_MAP_FILE	ROBOT_DATA_PATH "\\Upstairs.rmp"
#define DEFAULT_DOWNSTAIRS_MAP_FILE	ROBOT_DATA_PATH "\\Downstairs.rmp"
#define DEFAULT_KNOWN_OBJECTS_FILE	ROBOT_DATA_PATH "\\KnownObjects\\KnownObjects.txt"
#define DEFAULT_KNOWN_OBJECTS_DIR	ROBOT_DATA_PATH "\\KnownObjects\\"
#define DEFAULT_STEREO_CALIB_FILE	ROBOT_DATA_PATH "\\StereoData\\CalibrationList.txt"

#define GPS_TENTH_INCHES_PER_DEGREE_LAT		43859650	// West Coast

//#define GPS_INCHES_PER_DEGREE_LONG		29505780	// Seattle, WA:
#define GPS_TENTH_INCHES_PER_DEGREE_LONG	30673280	// Portland, OR:
//#define GPS_INCHES_PER_DEGREE_LONG		34592100	// San Francisco:

// General State defines
#define OFF									0x00
#define ON									0x01
#define AUTO								0x02

#define REVERSE								0x00
#define FORWARD								0x01
#define LEFT								0x00
#define RIGHT								0x01

#define SUPPRESS							0x00
#define UNSUPPRESS							0x01

// Parameter for WM_ROBOT_SET_USER_PRIORITY
//#define SET_USER_REMOTE					0x01
//#define SET_USER_LOCAL					0x02
//#define SET_USER_LOCAL_AND_STOP			0x03

// Fine turn turn amounts here
#define TURN_AMOUNT_45_DEGREES				  40
#define TURN_AMOUNT_90_DEGREES				  80
#define TURN_AMOUNT_180_DEGREES			 	 170


/////////////////////////////////////////////////////////////////////////////
enum WM_ROBOT_MESSAGES { 

	// Commands To Module (included Arduino responses)
	// No ACK Required
		WM_ROBOT_MESSAGE_BASE = WM_APP,				// Start at App Base message
		WM_ROBOT_REQUEST_STATUS_CMD,				// WM_ROBOT_MESSAGE_BASE + 0x01	// Usually Sent directly to HW thread, except simulation
		WM_ROBOT_REQUEST_VERSION_CMD,				// Request version from Arduino
		WM_ROBOT_STOP_CMD,							// Will force a stop.
		WM_ROBOT_DRIVE_LOCAL_CMD,					// User has local, direct control of the robot, wParam = Speed, lParam = Turn 
		WM_ROBOT_DRIVE_REMOTE_CMD,					// User is remote (can't see robot directly), wParam = Speed, lParam = Turn
		WM_ROBOT_GET_SERVO_STATUS,				

		// HW Responses
		WM_ROBOT_SENSOR_STATUS_READY,				
		//WM_ROBOT_PIC_VERSION_READY,				
		WM_ROBOT_GPS_DATA_READY,					
		WM_ROBOT_ER1_ODOMETER_READY,				
		WM_ROBOT_SERVO_STATUS_READY,				

		WM_ROBOT_MOVE_SET_DISTANCE_CMD,				// REMOTE_USER_MODULE is always used as owner! wParam = distance in tenth inches, lParam = direction
		WM_ROBOT_TURN_SET_DISTANCE_CMD,				// REMOTE_USER_MODULE is always used as owner! wParam = distance in degrees, lParam = Direction and speed
		WM_ROBOT_RESET_WATCHDOG_CMD,			
		WM_ROBOT_SERVO_POWER_CMD,				
		WM_ROBOT_CLIENT_KEEP_ALIVE_CMD,		

		WM_ROBOT_SERVO_CMD,					
		WM_ROBOT_LIGHT_POWER_CMD,			
		WM_ROBOT_AUX_LIGHT_POWER_CMD,		
		WM_ROBOT_ENABLE_OBJECT_NAV_UPDATE,	
		WM_ROBOT_GOTO_WAYPOINT_CMD,			
		WM_ROBOT_SPEAK_TEXT,					
		WM_ROBOT_EXECUTE_PATH,				
		WM_ROBOT_RESET_ODOMETER,				
		WM_ROBOT_BRAKE_CMD,					
		WM_ROBOT_OPEN_DATA_FILE,					// wParem = Path type, LParam = Map type.  Request Server to open the files
		WM_ROBOT_SET_GEAR_CMD,				
		WM_ROBOT_SENSOR_DATA,				
		WM_ROBOT_PLAY_SOUND,				
		WM_ROBOT_PLAY_MUSIC,				
		WM_ROBOT_TEXT_TO_AI,			
		//WM_ROBOT_ENABLE_DYNA_CONTROLLER_CMD,	

		WM_ROBOT_THREAD_EXIT_CMD,					// Tell all threads to exit
		WM_ROBOT_IR_SENSOR_POWER_CMD,	
		WM_ROBOT_POWER_MODE_CMD,					// wParam = Arm/Head Power, lParam = Laptop mode. See LAPTOP_POWER_MODE_DEFAULT
		WM_ROBOT_SET_KINECT_POSITION,				// BULK servo command.  wParam not used. lParam = Set Speed too (True/False) 
		WM_ROBOT_KINECT_POWER_ENABLE_CMD,			// wParam not used, lParam = Enable/Disable
		// add more here...

		WM_ROBOT_CAMERA_POWER_CMD,					// User Pan/Tilt commands
		WM_ROBOT_CAMERA_INITIALIZE_CMD,			
		WM_ROBOT_USER_CAMERA_PAN_CMD,				// wParam = Pan/Tilt Enum, lParam = movement speed		
		WM_ROBOT_USER_CAMERA_ZOOM_CMD,		
		WM_ROBOT_SET_HEAD_POSITION,					// BULK Head command.  wParam not used. lParam = Set Speed too (True/False) 
		WM_ROBOT_USER_CAMERA_PAN_TILT_SPEED_CMD,	// wParam = speed

		// Absolute Pan/Tilt Commands
		WM_ROBOT_CAMERA_PAN_ABS_CMD,				// wParam = value
		WM_ROBOT_CAMERA_TILT_ABS_CMD,				// wParam = value
		// Relative Pan/Tilt Commands
		WM_ROBOT_CAMERA_PAN_REL_CMD,				// wParam = value
		WM_ROBOT_CAMERA_TILT_REL_CMD,				// wParam = value
		// Other Camera commands
		WM_ROBOT_CAMERA_ZOOM_ABS_CMD,				// wParam = zoom level.  Range 0 to 16.
		WM_ROBOT_CAMERA_MODE_CMD,					// wParam = mode, lParam = value
		WM_ROBOT_CAMERA_SIDETILT_ABS_CMD,			// wParam = value
		WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD,			// wParam = X,Z, lParam = Y -- point to XYZ position in space (Y is dist from robot)
		WM_ROBOT_CAMERA_LOOK_AT_SOUND_CMD,			// wParam = Direction TenthDegrees, lParam = speed -- Turn head to point to sound
		WM_ROBOT_CAMERA_LOOK_AT_PLAYER_CMD,			// wParam = Player Number, lParam = speed == Point robot eyes at player's head
		WM_ROBOT_CAMERA_NOD_HEAD_CMD,				// wParam, lParam, not used.  When set will nod the head once

		WM_ROBOT_KINECT_SEARCH_FLOOR_CMD,			// wParam:  TRUE = Find Close Objects Only
		WM_ROBOT_KINECT_TRACK_OBJECT_CMD,			// 
		WM_ROBOT_KINECT_CANCEL_CMD,					// wParam:  N/A - Cancel any pending search / track work


		WM_ROBOT_FIND_OBJECT_AT_XYZ_CMD,			// wParam = X,Z, lParam = Y -- point to XYZ position in space (Y is dist from robot)
		WM_ROBOT_SLAM_CHECK_CMD,					// wParam n/a, lParam n/a
		WM_ROBOT_DO_LASER_SCANS,					// wParam = number of scans
		WM_ROBOT_ENABLE_COLLISION_MODULE,	
		WM_ROBOT_ENABLE_AVOIDANCE_MODULE,	
		WM_ROBOT_SET_AVOID_OBJ_RANGE,		
		WM_ROBOT_SET_SCAN_DISTANCE_CMD,		
		WM_ROBOT_SET_COMPASS_CAL_MODE,		
		WM_ROBOT_SET_COMPASS_CAL_POINT,		
		WM_ROBOT_ENABLE_RADAR_SCAN_CMD,		
		WM_ROBOT_ENABLE_CLIFF_SENSORS,		
		WM_ROBOT_ENABLE_GPS_PATH,			

		WM_ROBOT_GOTO_GRID_LOCATION_CMD,				
		WM_ROBOT_SET_BEHAVIOR_CMD,					// Select behavior mode for robot, lparam = behavior
		WM_ROBOT_SET_ACTION_CMD,					// Select Action mode.  lparam = mode
		WM_ROBOT_SET_LED_EYES_CMD,					// wParam = mode, lParam = brightness
		WM_ROBOT_SET_CURRENT_LOCATION,				// wParem = X, lParam = Y

		WM_ROBOT_SPEECH_RECO_SET_CMD_MODE,			// wParam = bEnable.  Commands to enable Speech Reco Command mode (vs AI)

		WM_ROBOT_SET_ARM_POSITION,					// wParam = Right/Left Arm, lParam = Set Speed too (True/False)
		WM_ROBOT_SET_ARM_DEFAULT_SPEED,				// wParam = Arm, lParam = Value
		WM_ROBOT_GET_SMART_SERVO_STATUS,				// wParam = not used, lParam = Get Speed too (True/False)
		WM_ROBOT_SET_SERVO_TORQUE_ENABLE,			// Uses bitfields: wParam = Operation, lParam = Enable/Disable
		WM_ROBOT_SET_SERVO_TORQUE_LIMIT,				// wParam = ServoID, lParam = Torque Limit
		WM_ROBOT_SET_ARM_MOVEMENT,					// Complex Arm movements.  wParam = Operation, lParam = Left/Right

		WM_ROBOT_ENABLE_RECO,					
		WM_ROBOT_DICTATION_RECO_EVENT,			
		WM_ROBOT_COMMAND_RECO_EVENT,				
		WM_ROBOT_SPEAKING_COMPLETE_EVENT,		

		WM_ROBOT_CAMERA_ENABLE_FEATURE,			
		WM_ROBOT_COLOR_BLOB_AUTO_CAL_CMD,		
		WM_ROBOT_COLOR_BLOB_MANUAL_CAL_CMD,		
		WM_ROBOT_COLOR_BLOB_LOOK_AHEAD_CMD,		
		WM_ROBOT_COLOR_BLOB_SEARCH_CMD,			
		WM_ROBOT_SET_CAMERA_COLOR_THRESHOLD,		
		WM_ROBOT_CAMERA_LOOK_AHEAD_CMD,			
		WM_ROBOT_CAMERA_BEHAVIOR_MODE,			

		WM_ROBOT_USER_CAMERA_FOWARD,				
		WM_ROBOT_USER_CAMERA_SIDETILT,			
		WM_ROBOT_CAMERA_TAKE_SNAPSHOT,				// wParam, lParam not used
		WM_ROBOT_CAMERA_RECORD_VIDEO,				// wParam: True = start, False = Stop.  lParam not used
		WM_ROBOT_CAMERA_CAPTURE_FACE,				// wParam not used.  lParam not used - Captures person's face for face recognition
		WM_ROBOT_CAMERA_MATCH_OBJECT,				// Match objects on file to the current frame.  Respond with WM_ROBOT_CAMERA_MATCH_COMPLETE
		WM_ROBOT_CAMERA_MATCH_COMPLETE,				// Response to WM_ROBOT_CAMERA_MATCH_OBJECT request.  wParam: ObjectID, lParam: Location in Frame.  

		WM_ROBOT_LASER_SCAN_DATA_READY,				// from LaserScannerParser
		WM_ROBOT_KINECT_SEARCH_COMPLETE,			// wParam = ObjectFound (T/F)

		// Bulk Commands from Remote GUI
		WM_ROBOT_TEXT_MESSAGE_TO_SERVER,			
		WM_ROBOT_BULK_DATA_TO_SERVER,			


		//////// GUI Display messages Only! Commands >= to this are ignored by Module!!!
		// See HW_MAX_MESSAGE in HardwareCmds.h!
		WM_GUI_MESSAGES,								
		WM_ROBOT_SEND_TEXT_MESSAGES,				
		WM_ROBOT_DISPLAY_STATUS_MESSAGES,		
		WM_ROBOT_DISPLAY_SINGLE_ITEM,				// Display a single item on the remote client GUI
		WM_ROBOT_DISPLAY_BULK_ITEMS,				// Display lots of data on the remote client GUI
		WM_ROBOT_REMOTE_GUI_CMD,					// Commands from Remote GUI, for camera or other things that need a window
		WM_ROBOT_DISPLAY_TCP_TIME,				
		WM_ROBOT_DISPLAY_OPEN_DATA_FILE,			// Does the REAL file open, in the display thread!
		WM_ROBOT_CLIENT_CONNECT,				
		WM_ROBOT_CLIENT_DISCONNECT,			
		WM_ROBOT_OPEN_MAP_FILE,						// wParam: 0=Create New file, lParam =
		WM_ROBOT_ADD_WAYPOINT_LOCATION,				// Message from MapView to PathView to add a new Waypoint at given location
		WM_ROBOT_UPDATE_VIEW,						// Tell view to update ( server changed some data)
		WM_ROBOT_GET_CAMERA_SETTINGS,				// Tell GUI to send current camera settings to the Camera Module

	////////////////////////////////////////////////
	// Commands from Remote GUI to Server GUI
		ROBOT_REMOTE_ENABLE_CAMERA,						
		ROBOT_REMOTE_ENABLE_MOTION_VIEW,					
		WM_ROBOT_MAX_ROBOT_MESSAGE					// Max message ID used by this program (for noise filter on send messages)
};   // WM_ROBOT_MESSAGES


	// Server Response Messages
enum SERVER_RESPONSE_MESSAGES {
		ROBOT_RESPONSE_TEXT_MESSAGE = 0,
		ROBOT_RESPONSE_CONNECTED,	
		ROBOT_RESPONSE_RADAR_SAMPLE,	
		ROBOT_RESPONSE_IR_SAMPLE,		
		ROBOT_RESPONSE_PIC_DEBUG1,					// For debugging Arduino code
		ROBOT_RESPONSE_PIC_DEBUG2,					// used for debugging new message code
		ROBOT_RESPONSE_PIC_WATCHDOG,				// Indicates watchdog expired or reset
		ROBOT_RESPONSE_PIC_VERSION,			
		ROBOT_RESPONSE_PIC_12V_POWER,		
		ROBOT_RESPONSE_PIC_BUMPER,					// state of bumper switches
		ROBOT_RESPONSE_PIC_LAST_ERROR,		
		ROBOT_RESPONSE_PIC_STATUS,
		ROBOT_RESPONSE_GPS_DATA,			
		ROBOT_RESPONSE_COLOR_BLOB_CAL_RESULT,
		ROBOT_RESPONSE_DRIVE_MODULE_OWNER,	
		ROBOT_RESPONSE_MOTOR_SPEED,					// From the ER1 Pilot controller
		ROBOT_RESPONSE_LASER_SCANNER_DATA,		
		ROBOT_RESPONSE_KINECT_DATA		
}; // SERVER_RESPONSE_MESSAGES


	
// AutoNavigateMode for WM_ROBOT_AUTO_NAVIGATE_CMD
//#define MODE_NAVIGATE_PATH					0x00	// Fully autonomous mode
//#define MODE_MANUAL_AVOID_OBJECTS			0x01	// Manual control, but avoid objects in path
//#define MODE_MANUAL_NO_COLLISION			0x02	// Stop on collisions, but ignore anyting else
//#define MODE_MANUAL_FULL_OVERRIDE			0x03	// Direct control - ignore sensors

/////////////////////////////////////////////////////////////////////////////
// Behavior modes - enabled by WM_ROBOT_SET_BEHAVIOR_CMD


#define BEHAVIOR_FRIENDLY					0x00	// Random movements, tracks motion and faces, listening for commands
#define BEHAVIOR_MANUAL_CONTROL				0x01	// Don't do any automatic behaviors, voice recog turned off
#define BEHAVIOR_WAIT_FOR_VERBAL_CMD		0x02	// No motion, waiting for verbal command
#define BEHAVIOR_GUARD						0x03	// No motion, monitoring for movement, "Intruder Alert!"
#define BEHAVIOR_PERSONAL_SPACE				0x04	// Move if someone violated your personal space
#define BEHAVIOR_MAX						0x05	// Last valid mode

// Action Modes - Random ideas for now, of things robot could do
// Set by WM_ROBOT_SET_ACTION_CMD
// WARNING - order of the first of these must match the CMD GUI!
// RobotCmdView.cpp:  Ready;Pickup Close;Pickup Any;Photo;Follow;Danger;Karate Demo;
enum SET_ACTION_CMD { 
	ACTION_MODE_NONE = 0,						// No Action mode pending
	ACTION_MODE_PICKUP_CLOSE_OBJECT,			// Look for and pickup nearest object that is within reach of the robot
	ACTION_MODE_PICKUP_OBJECTS,					// Look for and pickup object anywhere in front of robot
	ACTION_MODE_FOLLOW_PERSON,					//
	ACTION_MODE_EXPLORE,						//
	ACTION_MODE_FIND_DOCK,						//  Find the dock, and recharge

	//Ready;Pickup Close;Pickup Any;Follow;Explore;Dock;

	// ---------- GUI cut off --------------------
	ACTION_MODE_TAKE_PHOTO,						//
	ACTION_MODE_FREAK_OUT,						//
	ACTION_MODE_KARATE_DEMO,					//
	ACTION_MODE_COME_HERE,						//
	ACTION_MODE_TURN_TOWARDS_ME,				//
	ACTION_MODE_BAD_ROBOT,						//
	ACTION_MODE_LIGHT_SABER,					// demonstrate use of light saber
	ACTION_MODE_MOVE_WHILE_TALKING,				//
	ACTION_MODE_RUN_SCRIPT,						//
	ACTION_MODE_GO_TO_LOCATION,					// Go to named location on map (example "master bedroom")?
	ACTION_MODE_FOLLOW_OBJECT,					//
	ACTION_MODE_FIND_OBJECT, 					//
	ACTION_MODE_DELIVER_RECORDED_MSG,			//
	ACTION_MODE_RETREIVE_INTERNET_DATA,			//
	ACTION_MODE_START_PHONE_CALL,				//	Call someone using Skype?
	ACTION_MODE_GET_BEER,						//	Every robot should be able to get Beer from the Fridge!
	ACTION_MODE_OPEN_DOOR,						//	Just to demo door opening ability
	ACTION_MODE_DARTH_VADER,					//  Do a Darth impression
	ACTION_MODE_TURN_TO_COMPASS_DIR,			//  Param = COMPASS_ROSE (eg. NORTH_WEST)
	ACTION_MODE_POINT_TO_COMPASS_DIR,			//  COMPASS_ROSE
	ACTION_MODE_MOVE_IN_COMPASS_DIR,			//  COMPASS_ROSE
	ACTION_MODE_TURN_TO_COMPASS_DEGREES,		//  Param = Compass Degrees (0-360)
	ACTION_MODE_WHAT_TIME_IS_IT,				//
	ACTION_MODE_WAKE_UP,						//
	ACTION_MODE_GO_TO_SLEEP,					//
	ACTION_MODE_TELL_JOKES,						//
	ACTION_MODE_CHAT_DEMO_WITH_ADULT,			//
	ACTION_MODE_CHAT_DEMO_WITH_CHILD,			//
	ACTION_MODE_YES_NO_RESPONSE_RECEIVED,		// Heard Yes or No from person.  Used for interactive dialog

};

// Tasks needed to accomplish actions
#define TASK_NONE							0x00	// No Tasks Pending
#define TASK_GO_TO_LOCATION					0x01	//
#define TASK_OPEN_DOOR						0x02	//
#define TASK_CLOSE_DOOR						0x03	//
#define TASK_OPEN_FRIDGE_DOOR				0x04	//
#define TASK_CLOSE_FRIDGE_DOOR				0x05	//
#define TASK_GET_BEER						0x06	//
#define TASK_RETURN_TO_START				0x07	//
#define TASK_FIND_HUMAN						0x08	//
#define TASK_PERFORM_KARATE					0x09	//

// Tasks for picking up objects
#define OBJECT_TASK_NONE					0x00	// No Tasks Pending
#define OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT	0x01	//
#define OBJECT_TASK_PICK_UP_OBJECT			0x02
#define OBJECT_TASK_LOOK_WHILE_MOVING		0x03	//
#define OBJECT_TASK_CLOSE_SCAN_FOR_OBJECT	0x04	//
#define OBJECT_TASK_MOVE_TO_OBJECT			0x05	//
#define OBJECT_TASK_DONE					0x06	//

// Kinect Module Task states
#define OBJECT_TASK_NONE					0x00	// No Tasks Pending
#define KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT	0x01	//
#define KINECT_TASK_TRACK_CLOSEST_OBJECT	0x02	//
#define KINECT_TASK_HUMAN_DETECTION			0x03	// Kinect can be in either Object or Human search/track mode

// Script Task states
#define OBJECT_TASK_NONE					0x00	// No Tasks Pending
#define SCRIPT_TASK_OPEN_FILE				0x01	// Open the script file
#define SCRIPT_TASK_RUN_SCRIPT				0x02	// Run the script
#define SCRIPT_TASK_CLEAN_UP				0x03	// Close file, cleanup, and exit task


// Turn Handle States
#define TURN_HANDLE_STATE_NONE				0x00
#define ARM_TURN_HANDLE_STATE_IN_PROGRESS	0x01
#define TURN_HANDLE_STATE_SUCCESS			0x02
#define TURN_HANDLE_STATE_FAIL				0x03



// Arm Movements - may be complex movements.  wParam for WM_ROBOT_SET_ARM_MOVEMENT
enum ARM_MOVEMENT_CMD { 
	ARM_MOVEMENT_NONE,						// No Action mode pending
	ARM_MOVEMENT_HOME1,						// lparam - Which Arm or BOTH_ARMS (Applies for all ARM commands)
	ARM_MOVEMENT_HOME2,						// 
	ARM_MOVEMENT_TAKE_OBJECT,				// 
	ARM_MOVEMENT_GIVE_OBJECT,				// 
	ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW,		// Code figures out if claw open or closed
	ARM_MOVEMENT_OPEN_CLAW,					// 
	ARM_MOVEMENT_CLOSE_CLAW_FULL,			// 
	ARM_MOVEMENT_EXTEND_FULL,				// 
	ARM_MOVEMENT_SHAKE_READY,				// 
	ARM_MOVEMENT_SHAKE_MOVE,				// 

	ARM_MOVEMENT_PICKUP_OBJECT_BLIND,		// Blindly try to pick up object at set position
	ARM_MOVEMENT_PUT_DOWN_OBJECT,			// 
	ARM_MOVEMENT_THROW_OBJECT_FRONT	,		// 
	ARM_MOVEMENT_PUT_IN_BASKET,				// 
	ARM_MOVEMENT_LOOK_AT_HAND,				// 
	ARM_MOVEMENT_SCRATCH_HEAD,				// 
	ARM_MOVEMENT_ARM_UP_FULL,				// 
	ARM_MOVEMENT_SCRATCH_BACK,				// 
	ARM_MOVEMENT_WAVE,						// 
	ARM_MOVEMENT_GRAB_COKE,					// 
	ARM_MOVEMENT_LIFT_OBJECT,				// 
	ARM_MOVEMENT_PICKUP_OBJECT_XYZ,			// Try to pickup object at XYZ found by Kinect

	// Complex moves, and moves for two hands
	ARM_MOVEMENT_IDENTIFY_OBJECT,			//
	ARM_MOVEMENT_KARATE,					//

	ARM_MOVEMENT_TURN_DOOR_HANDLE,			// Assumes horizontal door knob, used in our home
	ARM_MOVEMENT_OPEN_HATCH,				// 
	ARM_MOVEMENT_CLOSE_HATCH,				// 
	ARM_MOVEMENT_SALUTE,					// 
	ARM_MOVEMENT_RANDOM,					// 
	ARM_MOVEMENT_TURN_DOOR_KNOB,			// Round Door knobs
	ARM_MOVEMENT_GRAB_FRIDGE_HANDLE,		// Refridgerator doors!
	ARM_MOVEMENT_ROBOT_MOVING,				//
	ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW,		//
	ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW,		//
	ARM_MOVEMENT_ENABLE_IDLE_MOVEMENT,		// lparam = True/False
	ARM_MOVEMENT_90_DEGREE,					// Mechanical Calibration Mode

};

// Head Movements - may be complex (multi-step) movements.  wParam for WM_ROBOT_SET_HEAD_MOVEMENT
#define HEAD_MOVEMENT_NONE					0x00	// No Action mode pending
#define HEAD_MOVEMENT_HOME					0x01	// Look straight ahead
#define HEAD_MOVEMENT_SLEEP					0x02	// Power down position

// Vision System updates.  wParam for WM_ROBOT_VISION_STATUS_READY
// #define CAMERA_MATCH_OBJECT_UPDATE			0x00	// lparam = object ID.  Response to WM_ROBOT_CAMERA_MATCH_OBJECT command


// Sound Commands
#define SOUND_STOP					 0x00
#define SOUND_DARTH_VADER			 0x10
// put sound effects here...

// Text To Speech Commands
#define SPEAK_ARDUINO_CONNECTED		 0x01 
#define SPEAK_ARDUINO_VERSION_ERROR	 0x02 
#define SPEAK_SIMULATION_MODE		 0x03
#define SPEAK_COLLISION				 0x04
#define SPEAK_AVOID					 0x05
#define SPEAK_TRAPPED				 0x06
#define SPEAK_EXECUTING_PATH		 0x07
#define SPEAK_WAYPOINT_REACHED		 0x08
#define SPEAK_LOOKING_FOR_LANDMARK	 0x09
#define SPEAK_HEADING_FOR_OBJECT	 0x0A
#define SPEAK_INTRO					 0x0B

#define SPEAK_TEXT_FROM_BUFFER		 0xFF	// get text from golbal buffer and speak it


// Music Files
// #define MUSIC_STOP					 0
#define MUSIC_SILENCE_STR				_T( ROBOT_DATA_PATH "\\RobotSounds\\silence.wav" )
#define MUSIC_MR_ROBOTO				 1 
#define MUSIC_MR_ROBOTO_STR				_T( ROBOT_DATA_PATH "\\RobotSounds\\MrRoboto.wav" )
#define MUSIC_HAWAII50				 2
#define MUSIC_HAWAII50_STR				_T( ROBOT_DATA_PATH "\\RobotSounds\\Hawaii50.wav" )
#define CAMERA_CLICK_STR				_T( ROBOT_DATA_PATH "\\RobotSounds\\CameraClick.wav" )


//////////// HEAD / CAMERA /////////////////////////////////

// Used to group servos for commands and status updates
enum SERVO_GROUP {
	MOVING_SERVOS = 0,		// only applies to servos that are moving. See IsDynaServoMoving()
	RIGHT_ARM,
	LEFT_ARM,
	BOTH_ARMS,
	HEAD_SERVOS,
	KINECT_SERVO,
	ALL_SERVOS
};

// Generic Servo Speeds
#define SERVO_SPEED_STOP					0x00	// Steps by 16 each, to allow for variable speeds if needed
#define SERVO_SPEED_EXTREMELY_SLOW			0x03	// 1/2 step for this one
#define SERVO_SPEED_VERY_SLOW				0x10	
#define SERVO_SPEED_SLOW					0x20
#define SERVO_SPEED_MED_SLOW				0x30
#define SERVO_SPEED_MED						0x40
#define SERVO_SPEED_MED_FAST				0x50
#define SERVO_SPEED_FAST					0x60
#define SERVO_SPEED_MAX						0x70

#define CLAW_TORQUE_DETECT_OBJECT			  100	// Min torque reported to decide robot is holding something

//////////// RIGHT ARM /////////////////////////////////

// Arm Limits - In TENTH DEGREES!
#define RIGHT_ARM_ELBOW_ROTATE_MAX				 (140*10)	// Rotate Max Clockwise (140 degrees)
#define RIGHT_ARM_ELBOW_ROTATE_MIN				(-140*10)	// Rotate Max Counter-clockwise (-140 degrees)

#define RIGHT_ARM_ELBOW_BEND_MAX				 (186*10)	// Max Curl - Same as Home position
#define RIGHT_ARM_ELBOW_BEND_MIN				 (-55*10)//(-40*10)	// Back-Bend (ouch!) (-55 degrees)

#define RIGHT_ARM_WRIST_ROTATE_MAX				 (150*10)	// Rotate Max Clockwise (150 degrees)
#define RIGHT_ARM_WRIST_ROTATE_MIN				(-150*10)	// Rotoate Max Counter Clockwise (-150 degrees)

#define RIGHT_ARM_CLAW_OPEN_MAX					 (90*10)	// Fully Open (90 degrees)
#define RIGHT_ARM_CLAW_OPEN_NORMAL				 (50*10)	// Mostly Open (100 degrees)
#define RIGHT_ARM_CLAW_CLOSED_MIN				     (0)	// Closed Tight!

#define RIGHT_SHOULDER_ROTATE_MAX				 (330*10)	// Rotoate forward
#define RIGHT_SHOULDER_ROTATE_MIN				(-330*10)	// Rotate Backward

//Standard Arm Positions - in DEGREES!
#define RIGHT_ARM_CLAW_OPEN_FULL					 105	// Degrees
#define RIGHT_ARM_CLAW_OPEN_HALF					  50	// Degrees
#define RIGHT_ARM_CLAW_CLOSED_COKE					  30	// Degrees
#define RIGHT_ARM_CLAW_CLOSED_LOOSE					   0	// Degrees
#define RIGHT_ARM_CLAW_CLOSED_TIGHT					  -5	// Degrees

// Home Position1 - Normal
#define RIGHT_ARM_SHOULDER_HOME1					 -15	// Degrees
#define RIGHT_ARM_ELBOW_ROTATE_HOME1				   0    // Degrees
#define RIGHT_ARM_ELBOW_BEND_HOME1					 116	// Degrees - Moved to avoid side IR sensor!
#define RIGHT_ARM_WRIST_ROTATE_HOME1				   0	// Degrees
#define RIGHT_ARM_CLAW_HOME1						 RIGHT_ARM_CLAW_CLOSED_LOOSE

// Home Position2 - Claws up
#define RIGHT_ARM_SHOULDER_HOME2					   7	// Degrees - so elbow stays up when powered off
#define RIGHT_ARM_ELBOW_ROTATE_HOME2				   0	// Degrees
#define RIGHT_ARM_ELBOW_BEND_HOME2					 167	// Degrees
#define RIGHT_ARM_WRIST_ROTATE_HOME2				   0	// Degrees
#define RIGHT_ARM_CLAW_HOME2						 RIGHT_ARM_CLAW_CLOSED_LOOSE

#define RIGHT_ARM_ELBOW_BEND_HOME2_LOCK				 175	// Degrees - locks into position with latch

// 90 Degree Calibration Position (for adjusting hardware)
#define RIGHT_ARM_SHOULDER_90_DEGREE				   0	// Degrees - so elbow stays up when powered off
#define RIGHT_ARM_ELBOW_ROTATE_90_DEGREE			   0	// Degrees
#define RIGHT_ARM_ELBOW_BEND_90_DEGREE				  90	// Degrees
#define RIGHT_ARM_WRIST_ROTATE_90_DEGREE			   0	// Degrees
#define RIGHT_ARM_CLAW_90_DEGREE					 RIGHT_ARM_CLAW_CLOSED_LOOSE

// Elbows In for moving


// Shake Hands Position
#define RIGHT_ARM_SHOULDER_ARM_SHAKE				  RIGHT_ARM_SHOULDER_ARM_SHAKE_UP	// Degrees
#define RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE			  -5	// Degrees
#define RIGHT_ARM_ELBOW_BEND_ARM_SHAKE				  65	// Degrees
#define RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE			RIGHT_ARM_WRIST_ROTATE_HOME1
#define RIGHT_ARM_CLAW_ARM_SHAKE					RIGHT_ARM_CLAW_OPEN_HALF

// Shake UP
#define RIGHT_ARM_SHOULDER_ARM_SHAKE_UP				  80	// Degrees
#define RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_UP			  60	// Degrees

// Shake Down
#define RIGHT_ARM_SHOULDER_ARM_SHAKE_DOWN			  70	// Degrees
#define RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN			  60	// Degrees

#define RIGHT_ARM_SHOULDER_ARM_STRAIGHT_UP			 180	// Degrees


//////////// LEFT ARM /////////////////////////////////

// Arm Limits - In TENTH DEGREES!
#define LEFT_ARM_ELBOW_ROTATE_MAX	  				  (140*10)	// Rotate Max Clockwise (140 degrees)
#define LEFT_ARM_ELBOW_ROTATE_MIN					  (-140*10)	// Rotate Max Counter-clockwise (-140 degrees)

#define LEFT_ARM_ELBOW_BEND_MAX						  (164*10)	// Max Curl - Same as Home position
#define LEFT_ARM_ELBOW_BEND_MIN						  (-40*10)	// Back-Bend (ouch!) (-40 degrees)

#define LEFT_ARM_WRIST_ROTATE_MAX					  (150*10)	// Rotate Max Clockwise (150 degrees)
#define LEFT_ARM_WRIST_ROTATE_MIN					  (-150*10)	// Rotoate Max Counter Clockwise (-150 degrees)

#define LEFT_ARM_CLAW_OPEN_MAX						  (130*10)	// Fully Open (130 degrees)
#define LEFT_ARM_CLAW_OPEN_NORMAL					  (100*10)	// Mostly Open (100 degrees)
#define LEFT_ARM_CLAW_CLOSED_MIN					  (-15*10)		// Closed Tight!

#define LEFT_SHOULDER_ROTATE_MAX				  (330*10)	// Rotoate forward
#define LEFT_SHOULDER_ROTATE_MIN				  (-330*10)	// Rotate Backward

//Standard Arm Positions - in DEGREES!
#define LEFT_ARM_CLAW_OPEN_FULL							120		// Degrees
#define LEFT_ARM_CLAW_OPEN_HALF							 60		// Degrees
#define LEFT_ARM_CLAW_CLOSED_COKE						 40		// Degrees
#define LEFT_ARM_CLAW_CLOSED_LOOSE						  4		// Degrees
#define LEFT_ARM_CLAW_CLOSED_SNUG						-10		// Degrees  Fingertips touch at zero torque (not good for paper, but fine for other objects)
#define LEFT_ARM_CLAW_CLOSED_TIGHT						-14		// Degrees  MUST NOT EXCEED LEFT_ARM_CLAW_CLOSED_MIN

// Home Position 1
#define LEFT_ARM_SHOULDER_HOME1						-15	// Degrees
#define LEFT_ARM_ELBOW_ROTATE_HOME1					 -4	// Degrees // Negative means to the Robot's Left (away from the body)
#define LEFT_ARM_ELBOW_BEND_HOME1					109 // Degrees - Moved to avoid side IR sensor!
#define LEFT_ARM_WRIST_ROTATE_HOME1					  0	// Degrees
#define LEFT_ARM_CLAW_HOME1							LEFT_ARM_CLAW_CLOSED_LOOSE

// Home Position 2 - Claws Up
#define LEFT_ARM_SHOULDER_HOME2						  7	// Degrees
#define LEFT_ARM_ELBOW_ROTATE_HOME2					  0	// Degrees
#define LEFT_ARM_ELBOW_BEND_HOME2					154	// Degrees
#define LEFT_ARM_WRIST_ROTATE_HOME2					  0 // Degrees
#define LEFT_ARM_CLAW_HOME2							LEFT_ARM_CLAW_CLOSED_LOOSE

#define LEFT_ARM_ELBOW_BEND_HOME2_LOCK				170	// Degrees
#define LEFT_ARM_WRIST_ROTATE_HOME2_LOCK			-90 // Degrees - get cable out of the way

// 90 Degree Calibration Position (for adjusting hardware)
#define LEFT_ARM_SHOULDER_90_DEGREE					  0	// Degrees - so elbow stays up when powered off
#define LEFT_ARM_ELBOW_ROTATE_90_DEGREE				  0	// Degrees
#define LEFT_ARM_ELBOW_BEND_90_DEGREE				 90	// Degrees
#define LEFT_ARM_WRIST_ROTATE_90_DEGREE				  0 // Degrees
#define LEFT_ARM_CLAW_90_DEGREE						LEFT_ARM_CLAW_CLOSED_LOOSE

// Shake Hands Position
#define LEFT_ARM_SHOULDER_ARM_SHAKE					 65	// Degrees
#define LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE				 -5	// Degrees
#define LEFT_ARM_ELBOW_BEND_ARM_SHAKE				 65	// Degrees
#define LEFT_ARM_WRIST_ROTATE_ARM_SHAKE				LEFT_ARM_WRIST_ROTATE_HOME2
#define LEFT_ARM_CLAW_ARM_SHAKE						LEFT_ARM_CLAW_CLOSED_LOOSE

// Shake UP
#define LEFT_ARM_SHOULDER_ARM_SHAKE_UP				 70	// Degrees
#define LEFT_ARM_ELBOW_BEND_ARM_SHAKE_UP			 60	// Degrees

// Shake Down
#define LEFT_ARM_SHOULDER_ARM_SHAKE_DOWN			 50	// Degrees
#define LEFT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN			 60	// Degrees

#define LEFT_ARM_SHOULDER_ARM_STRAIGHT_UP			180	// Degrees

#define DYNA_TORQUE_LIMIT_MAX					0x03FF	// 1023
#define STRONG_TORQUE							767 // 3/4 max
#define MED_TORQUE								511	// 1/2 max
#define GENTLE_TORQUE							250
#define DYNA_TORQUE_LIMIT_NORMAL				STRONG_TORQUE


// TODO - for outdoor, look at fixed high-speed Shutter speed, to avoid blur

/////////////////////////////////////////////////////////////////////////////
// Camera Absolute or Relative parameters (used in WM_ROBOT_CAMERA_PAN_ABS_CMD and TILT)
//#define CAMERA_VALUE_ABSOLUTE				0x00
//#define CAMERA_VALUE_RELATIVE				0x01	// Value is relative to current position

// Camera Control parameters (used in WM_ROBOT_USER_CAMERA_PAN_CMD)
#define CAMERA_ZOOM_STOP					0x00
#define CAMERA_ZOOM_IN						0x01
#define CAMERA_ZOOM_OUT						0x02

#define CAMERA_PAN_SLOW						0x00
#define CAMERA_PAN_MEDIUM					0x01
#define CAMERA_PAN_FAST						0x03

// Sony Camera White Balance
#define CAMERA_WB_AUTO						0x00
#define CAMERA_WB_INDOOR					0x01
#define CAMERA_WB_OUTDOOR					0x02

// Camera Arm parameters (not used)
#define CAMERA_ARM_UP						0x00
#define CAMERA_ARM_DOWN						0x01
#define CAMERA_ARM_OFF						0x01
#define CAMERA_ARM_ON						0x00


// Camera modes passed in WM_ROBOT_CAMERA_MODE_CMD
// Loki Camera Modes (using VideoInput via OpenCV )
#define CAMERA_MODE_SHOW_FORMAT_DIALOG		0x00	// lParam = TRUE/FALSE
#define CAMERA_MODE_SHOW_PROP_DIALOG		0x01	// lParam = TRUE/FALSE
// Sony Camera mode commands
#define CAMERA_MODE_DISPLAY_ENABLE			0x02	// lParam = TRUE/FALSE
#define CAMERA_MODE_TRACK_FACE				0x03


/////////////////////////////////////////////////////////////////////////////
// Loki Head Positions

// WM_ROBOT_USER_CAMERA_FOWARD common positions
//#define HEAD_REST_POSITION					0x00
//#define HEAD_BACK_POSITION					0x01
//#define HEAD_UP_POSITION					0x02
//#define HEAD_INTEREST_POSITION				0x03
//#define HEAD_FORWARD_POSITION				0x04

// Head tilt parameters
//#define HEAD_SIDETILT_CENTER				0x00
//#define HEAD_SIDETILT_LEFT					0x01
//#define HEAD_SIDETILT_RIGHT					0x02


/////////////////////////////////////////////////////////////////////////////
// Features controlled by WM_ROBOT_CAMERA_ENABLE_FEATURE
#define CAMERA_ENABLE_VIDCAP_PROCESSING		0x00	// Used to disable ALL vidcap processing!
#define CAMERA_ENABLE_TRACKING_FACE			0x01
#define CAMERA_ENABLE_TRACKING_COLORS		0x02
#define CAMERA_ENABLE_TRACKING_CAMSHIFT		0x03
#define CAMERA_ENABLE_TRACKING_OBJECTS		0x04
#define CAMERA_ENABLE_TRACKING_CONES		0x05
#define CAMERA_ENABLE_TRACKING_MOTION		0x06
#define CAMERA_ENABLE_TRACKING_IR			0x07
#define CAMERA_ENABLE_SHOW_MOTION_VIEW		0x08
#define CAMERA_ENABLE_FACE_IDENTIFICATION	0x09
#define CAMERA_ENABLE_TRACKING_HAND			0x0A
#define CAMERA_ENABLE_STEREO_VISION			0x0B
#define CAMERA_ENABLE_MATCHING_OBJECTS		0x0C


#define SET_COLOR_THRESHOLD_CR				0x01
#define SET_COLOR_THRESHOLD_CB				0x02
#define NO_MATCH							(-1)	// No match found by MatchObjectDetector 

// 12V and Sony Camera Power parameters
#define POWER_OFF							0x00
#define POWER_ON							0x01
#define CAMERA_RESET						0x02
#define SYSTEM_SLEEP						0x04
#define SYSTEM_SHUT_DOWN					0x08

/*
// LED Eyes parameters
#define LED_EYE_OFF							0x00
#define LED_EYE_ON							0x01
#define LED_EYE_CLOSE						0x02
#define LED_EYE_OPEN						0x03
#define	EYES_BLINK							0x04
#define LED_EYE_BRIGHTNESS_MAX				  19	// On time in 20ms cycle
*/

// Misc parameters
#define MUSIC_STOP							0x00
#define MUSIC_START							0x01

#define PATH_EXECUTE_CANCEL					0x00
#define NAV_PATH_EXECUTE_START				0x01
#define GRID_PATH_EXECUTE_START				0x02
#define PATH_EXECUTE_PAUSE					0x03
#define PATH_EXECUTE_RESUME					0x04


// File Open Parameters for Path and Map types
#define PATH_TYPE_NONE						0x00	// Don't load a Path
#define PATH_TYPE_DEFAULT					0x01	// Load the default Path file
#define PATH_TYPE_BLANK						0x02	// Load a blank Path file


#define MAP_TYPE_NONE						0x00	// Load a blank Map file
#define MAP_TYPE_BLANK						0x01	// Load a blank Map file
#define MAP_TYPE_ROBOTHON					0x02	// Not used at this time
#define MAP_TYPE_UPSTAIRS					0x03	// Load the Upstairs Map
#define MAP_TYPE_DOWNSTAIRS					0x04	// Load the Downstairs Map


// Sensor IDs for enable/disable via WM_ROBOT_ENABLE_SENSOR
#define SENSOR_IR0							0x00
#define SENSOR_IR1							0x01
#define SENSOR_IR2							0x02
#define SENSOR_IR3							0x03
#define SENSOR_IR4							0x04
#define SENSOR_IR5							0x05

#define SENSOR_US0							0x10
#define SENSOR_US1							0x11
#define SENSOR_US2							0x12

#define BULK_DATA_TYPE_UNKNOWN				0x00	// Parameter 1 for WM_ROBOT_BULK_DATA_TO_SERVER
#define BULK_DATA_TYPE_MAP_STROKE			0x01


// Radar parameters - see globals.h for buffer declaration
#define MAX_SENSORS					4	// Total number of sensor scans tracked (not counting Laser Scanner)
#define FIXED_IR_ARRAY				0	// Array of fixed (non moving) IR sensors
#define FIXED_US_ARRAY				1	// Array of fixed (non moving) Ultrasonic sensors
#define IR_ARRAY1					2
#define US_ARRAY1					3

#define FIXED_IR_SAMPLES			 6
#define FIXED_US_SAMPLES			 3
#define ULTRASONIC1_SAMPLES			16	// and number of samples for each type
#define ULTRASONIC2_SAMPLES			32


#define US1_SAMPLES					16	// number of samples.
/**
#define US_1_HARD_LEFT				 1	//  0/1
#define US_1_MID_LEFT				 4	//  2-4
#define US_1_SLIGHT_LEFT			 6	//  5/6
#define US_1_CENTER					 8	//  7/8 CHANGE if number of samples change!
#define US_1_SLIGHT_RIGHT			10	//  9/10
#define US_1_MID_RIGHT				13	// 11-13
#define US_1_HARD_RIGHT				15	// 14/15

#define US_1_CENTER_LEFT	 		 7	//  7/8 CHANGE if number of samples change!
#define US_1_CENTER_RIGHT	 		 8	//  7/8 CHANGE if number of samples change!
**/

#define IR1_SAMPLES					16
#define IR2_SAMPLES					16


#define ULTRASONIC_RAW_MAX						  255		// Max value for each type
//#define ULTRASONIC_TENTH_INCHES_MAX			  840		// 1080	// For Carbot, we use 9ft(108 inches)
#define ULTRASONIC_TENTH_INCHES_MAX				 1440		// Max EZ1 works pretty good to 12 feet! // For Carbot, we use 9ft(108 inches)
#define LASER_RANGEFINDER_MM_MAX				 4096		// URG-04LX max range = 4096mm = 13.4 feet 
//#define LASER_RANGEFINDER_TENTH_INCHES_MAX	1560.0		// = 13.0 feet, just under sensor limit
//#define LASER_RANGEFINDER_TENTH_INCHES_ERROR	1570.0		// Bad Reading, ignore
#define LASER_RANGEFINDER_TENTH_INCHES_MAX		 1560		// = 13.0 feet, just under sensor limit
#define LASER_RANGEFINDER_TENTH_INCHES_ERROR	 1570		// Bad Reading, ignore
#define LASER_RANGEFINDER_DEGREES_PER_STEP	0.3515625
#define LASER_RANGEFINDER_MAX_SAMPLES			 2047		// Max number of samples per scan from URG-04LX Laser Scanner
#define LASER_RANGEFINDER_MAX_SCAN_LINES		    4		// Max number of scan lines to store for analysis
#define LASER_RANGEFINDER_MAX_OBJECT_LINES		  100		// Max number of object scan lines to store for analysis (up to one sample per 1/2 inch for 4 feet)


#define KINECT_MM_MAX							 4096 		// Kinect max range = 4096 mm, or 13.4 feet (at least the PC version,  XBox version might be more?) 
#define KINECT_RANGE_TENTH_INCHES_MAX			 1560		// = 13.0 feet, just under sensor limit of 161 inches (13.4 feet)
#define KINECT_RANGE_TENTH_INCHES_ERROR			 1570		// Bad Reading, ignore
#define KINECT_DEGREES_PER_STEP_X			0.3515625		// TODO Horizontal degrees per pixel step calculation: ????
#define KINECT_DEGREES_PER_STEP_Y			0.3515625		// TODO Horizontal degrees per pixel step calculation: ????
#define KINECT_MAX_SAMPLES						 2047		// Max number of samples per scan from URG-04LX Laser Scanner
#define KINECT_MAX_SCAN_LINES					   16		// Max number of scan lines to store for analysis
#define KINECT_MAX_OBJECT_LINES					  100		// Max number of object scan lines to store for analysis (up to one sample per 1/2 inch for 4 feet)


//#define IR_RAW_MAX							 255
#define IR_GUI_TENTH_INCHES_MAX					 840				// 7 feet - Just used to draw circle in Radar GUI
#define IR_MAX_TENTH_INCHES						 240
#define IR_LR_TENTH_INCHES_MAX					 360				// Long Range Sensors
#define IR_SR_TENTH_INCHES_MAX					 240				// Short Range Sensors
#define PIC_NO_OBJECT_IN_RANGE					0xFC			// Same value as used in Arduino
#define PIC_OBJECT_DETECTED_RANGE_UNKNOWN		0xFE
#define NO_OBJECT_IN_RANGE						LASER_RANGEFINDER_TENTH_INCHES_MAX		// used to be 0xFC, Same value as used in Arduino
#define OBJECT_DETECTED_RANGE_UNKNOWN			LASER_RANGEFINDER_TENTH_INCHES_ERROR	// used to be 0xFE
#define SENSOR_DISABLED							2048	// disable any computations that use this sensor

#define STATUS_BUF_LEN							4096
#define STATUS_TEXT_BUF_LEN					(STATUS_BUF_LEN-6)
#define STATUS_DATA_BUF_LEN					(STATUS_BUF_LEN-16)	// 12 plus padding for paranoia
#define BULK_DATA_SIZE						STATUS_DATA_BUF_LEN	// Size of transfer buffer
#define	WIDE_BULK_DATA_SIZE					(BULK_DATA_SIZE*2)	// Room to store Wide Bytes for sound routines


typedef struct
{
	DWORD	Cmd;
	DWORD	Param1;
	DWORD	Param2;
} SOCK_DATA_T;

typedef struct
{
	DWORD	MessageType;	// WM_ROBOT_RESPONSE_...
	DWORD	Length;
	CHAR	Message[STATUS_TEXT_BUF_LEN];
} SOCKET_STRING_MESSAGE_T;

typedef struct
{
	DWORD	MessageType;	// WM_ROBOT_RESPONSE_...
	DWORD	Param1;
	DWORD	Param2;
} SOCKET_STATUS_MESSAGE_T;


typedef struct
{
	DWORD	MessageType;	// WM_ROBOT_RESPONSE_...
	DWORD	Param1;
	DWORD	Param2;
	WORD	Length;
} SOCKET_DATABUF_HEADER_T;

typedef struct
{
	DWORD	MessageType;	// Same as HEADER_T, plus the Data buffer
	DWORD	Param1;
	DWORD	Param2;
	WORD	Length;
	CHAR	Data[STATUS_DATA_BUF_LEN];
} SOCKET_DATABUF_MESSAGE_T;


#define GPS_MAX_SAT	12	// 12 Satellites max
typedef struct
{
	WORD	wPRN;
	WORD	wSignalQuality;
	WORD	wAzimuth;
	WORD	wElevation;
	BOOL	bUsedInSolution;
} SAT_INFO_T;


typedef struct
{
	DWORD		dwCommandCount;				// number of NMEA commands received (processed or not processed)
	double		dGGALatitude;				// < 0 = South, > 0 = North
	double		dGGALongitude;				// < 0 = West, > 0 = East
	double		dGGAAltitude;				// Altitude: mean-sea-level (geoid) meters
	double		dGSAVDOP;					//
	double		dGSAHDOP;					//
	double		dGSAPDOP;					//
	//double	dRMCLatitude;				// current latitude
	//double	dRMCLongitude;				// current longitude
	//double	dRMCGroundSpeed;			// speed over ground, knots
	//double	dRMCCourse;				// course over ground, degrees true
	//double	dRMCMagVar;				// magnitic variation, degrees East(+)/West(-)
	WORD		wGSVTotalNumSatsInView;		//
	WORD		wGSASatsInSolution[GPS_MAX_SAT]; // ID of sats in solution
	SAT_INFO_T	GSVSatInfo[GPS_MAX_SAT];	//
	//BYTE		btRMCDataValid;				// A = Data valid, V = navigation rx warning
	BYTE		btGGAGPSQuality;				// 0 = fix not available, 1 = GPS sps mode, 2 = Differential GPS, SPS mode, fix valid, 3 = GPS PPS mode, fix valid
	BYTE		btGSAMode;					// M = manual, A = automatic 2D/3D
	BYTE		btGSAFixMode;				// 1 = fix not available, 2 = 2D, 3 = 3D
} GPS_MESSAGE_T;
	





#endif