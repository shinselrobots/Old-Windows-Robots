#ifndef __ROBOT_GLOBALS_H__
#define __ROBOT_GLOBALS_H__

// Turn off all the annoying security warnings! TODO - fix all these some day...
//#define _CRT_SECURE_NO_WARNINGS  
//#define _AFX_SECURE_NO_WARNINGS
//#define _ATL_SECURE_NO_WARNINGS

//#include "..\Common\HardwareCmds.h"
#include "RobotType.h"
#include "RobotSharedParams.h"
#include "HardwareCmds.h"
#include "HWInterfaceParams.h"

#include "PathStruct.h"
#include "GridMap.h"
//#include "highgui.h"
//#include "ObjectKnowledge.h"
#include "Config.h"

#include <list>
#include <iostream>
#include <queue>

#define GPA_TRACE_ENABLED 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Enable or disable Intel GPA for performance analysis (requres GPA to be installed) www.intel.com/software/gpa

#if GPA_TRACE_ENABLED

// Uncomment the following line to just turn off Intel GPA ITT Tracing 
//#define INTEL_NO_ITTNOTIFY_API 
#include <ittnotify.h>

// Common strings used by multiple modules
extern __itt_string_handle* psh_csDisplayLock;
extern __itt_string_handle* psh_csServoLock;
extern __itt_string_handle* psh_csKinectPointCloudLock;
extern __itt_string_handle* psh_csLaserSummaryDataLock;
extern __itt_string_handle* psh_csLaserDataLock;
extern __itt_string_handle* psh_csKinectHumanTrackingLock;
extern __itt_string_handle* psh_csKinectSummaryDataLock;
extern __itt_string_handle* psh_Sleep;

#else
// this macro will "comment out" references to GPA ITT tracing (so GPA does not need to be installed)
#define INTEL_NO_ITTNOTIFY_API 
#define __itt_domain char
#define __itt_domain_create( foo ) { "" }
#define __itt_string_handle char
#define __itt_string_handle_create( foo ) { "" } 
#define __itt_thread_set_name( foo ) {  } 
#define __itt_task_begin( foo1, foo2, foo3, foo4 ) { }
#define __itt_marker( foo1, foo2, foo3, foo4 ) { }
#define __itt_metadata_add( foo1, foo2, foo3, foo4, foo5, foo6 ) { }
#define __itt_task_end( foo ) { }

#endif


// Domains for ITT instrumentation
extern __itt_domain* pDomainGlobalThread;
extern __itt_domain* pDomainControlThread;
extern __itt_domain* pDomainVidCapThread;
extern __itt_domain* pDomainArduinoThread;
extern __itt_domain* pDomainGPSThread;
extern __itt_domain* pDomainPololuServoThread;
extern __itt_domain* pDomainMotorThread;
extern __itt_domain* pDomainSmartServoThread;
extern __itt_domain* pDomainLaserThread;
extern __itt_domain* pDomainKinectThread;
extern __itt_domain* pDomainSocketThread;
extern __itt_domain* pDomainGUIThread;
extern __itt_domain* pDomainSpeakThread;
extern __itt_domain* pDomainSpeechRecoThread;
extern __itt_domain* pDomainModuleThread;
extern __itt_domain* pDomainKinectAppSharedMemoryIPCThread;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Trace Logging

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRACE_ENABLED	1 // If enabled, Log output goes to Visual Studio Output, as well as the log file. (Slower!)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void g_PostStatus();	// forward declaration, so we can use it here
//#define ROBOT_LOG(__test1, __test2, __format, ...) if( __test1 && __test2 ) {  \       // use this if 2 tests are needed

#define ROBOT_LOG( __test1, __format, ...) if( __test1 ) {  \
	CString LogString; \
	LogString.Format( __format, ## __VA_ARGS__ );   \
	g_PostStatus( (LPCTSTR)LogString, __FUNCTION__, FALSE );    } // Does not display on GUI

#define ROBOT_DISPLAY( __test1, __format, ...) if( __test1 ) {  \
	CString LogString; \
	LogString.Format( __format, ## __VA_ARGS__ );   \
	g_PostStatus( (LPCTSTR)LogString, __FUNCTION__, TRUE );    } // Does display on GUI


// Path to config data
/// MOVED TO RobotType.h
//#define ROBOT_DATA_PATH	"C:\\Dev\\_Robot\\RobotData"
//#define ROBOT_COMMON_PATH L"C:\\Dev\\_Robot\\Common"

///////////////////////////////////////////////////////////////////
// Kludge for when OpenCV not linked
	#ifndef CvPoint
		typedef struct CvPoint
	{
		int x;
		int y;
	}
	CvPoint;


	typedef struct
	{
		int width;
		int height;
	}
	CvSize;

	//#define IplImage void

#endif
///////////////////////////////////////////////////////////////////

//#define BUMPER_HIT (g_pFullSensorStatus->BumperFront | g_pFullSensorStatus->BumperRight | g_pFullSensorStatus->BumperLeft)
//#define CLIFF_DETECTED (g_pFullSensorStatus->CliffFront | g_pFullSensorStatus->CliffRight | g_pFullSensorStatus->CliffLeft | g_pFullSensorStatus->WheelDropRight | g_pFullSensorStatus->WheelDropLeft  )


// Delete data and set pointer to Null
#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
#define SAFE_DELETE_CS(p)  { if(p) { DeleteCriticalSection(&p);     (p)=NULL; } }

// Stop warnings for parameters that are not used.  
// Include this in any function that has an unused parameter on purpose (such as threads).
#define IGNORE_UNUSED_PARAM (void)	

// Must match setting of Pwr Enable Invert Jumper!
#define INVERT_RC_BUTTON_PWR_ENABLE		TRUE 
enum COMM_DEVICES { 
		ARDUINO_COMM_DEVICE = 0,
		GPS_COMM_DEVICE,					
		SERVO_COMM_DEVICE,				
		MOTOR_COMM_DEVICE,				
		CAMERA_COMM_DEVICE,				
		DYNA_SERVO_AX12_COMM_DEVICE,		
		DYNA_SERVO_RX64_COMM_DEVICE,		
		KERR_SERVO_COMM_DEVICE,			
		LASER_SCANNER_COMM_DEVICE		
};

enum COMPASS_ROSE { 
		NORTH = 0,
		NORTH_EAST,					
		EAST,				
		SOUTH_EAST,				
		SOUTH,				
		SOUTH_WEST,		
		WEST,		
		NORTH_WEST,			
};

// WARNING!  THIS MUST MATCH C# CODE!  - TODO - put this in the .cs file?
#define KINECT_SPEECH_INTERFACE_SHARED_FILE_NAME	"RobotKinectSpeechMappingFile"
#define KINECT_DEPTH_INTERFACE_SHARED_FILE_NAME		"RobotKinectDepthMappingFile"
//#define KINECT_INTERFACE_SHARED_FILE_NAME	"Global\\MyKinectMappingObject"


#define DEGREES_TO_RADIANS	0.017452778	// Used in Sin and Cos calculations
//  or use this if it's easier (such as in the Arduino?):
//	double DirectionRadians = (((double)m_SegmentDirection / 360.0) * TWO_PI);

#define PI						((double)3.14159265358979)
#define TWO_PI					((double)6.28318530717958)
#define RADIANS_TO_DEGREES		((double)57.297468632)		// (1 / DEGREES_TO_RADIANS)

#define MIN_INT 			(-0x7FFFFFFF)	// Min Negative Int for initializing variables
#define MAX_INT				0x7FFFFFFF		// Max Positive Int for initializing variables
#define	RESULT_FAILURE		0xFFFF
#define NO_HEADING			RESULT_FAILURE

// Magic number to "No Op" a servo in a bulk servo command
#define SERVO_NO_CHANGE					MAX_INT
#define NOP								SERVO_NO_CHANGE	// shorter way to say it
#define TIMER_NOT_SET					MIN_INT
#define TIMER_SERVO_RESET				(-1)	// After timer times out, and message sent, it goes to RESET until set to TIMER_NOT_SET again


#define DEFAULT_ROBOT_START_POSITION_X	(50 * 12 * 10)	// TenthInches - 50 feet from boarder
#define DEFAULT_ROBOT_START_POSITION_Y	(50 * 12 * 10)

enum SUBSYSTEM_STATUS { 
		SUBSYSTEM_DISABLED = 0,	// Yellow
		SUBSYSTEM_WAITING,		// Yellow
		SUBSYSTEM_CONNECTED,	// Green
		SUBSYSTEM_FAILED		// Red
};

enum SPEECH_ARM_MOVEMENT_TYPE {
		SPEECH_ARM_MOVEMENT_HOME = 1,
		SPEECH_ARM_MOVEMENT_RANDOM_OFF,
		SPEECH_ARM_MOVEMENT_RANDOM_ON,
		SPEECH_ARM_MOVEMENT_SMALL_UP,
		SPEECH_ARM_MOVEMENT_SMALL_DOWN,
		SPEECH_ARM_MOVEMENT_BOXING1,
		SPEECH_ARM_MOVEMENT_BOXING2,
};



//////////////////////////////////////////////////////////////////////////////////////
// Camera constants

enum CAMERA_NUMBER { 
		LEFT_CAMERA = 0, 
		RIGHT_CAMERA,
		KINECT_DEPTH,
		KINECT_VIDEO
};
#define ALL_CAMERAS					0xFF

enum CAMERA_STATE {
		CAMERA_STATE_NOT_ENABLED = 0,		// Not enabled, or error occured trying to enable
		CAMERA_STATE_INIT_REQUESTED,		// Request to initialize camera
		CAMERA_STATE_INITIALIZED,			// Initialized for normal capture
		CAMERA_STATE_STEREO_INITIALIZED,	// Initialized for Stereo capture
		CAMERA_STATE_SHUTDOWN_REQUESTED		// Request to shut down the camera
};

#define CAMERA_THRESHOLD_DEFAULT	  5	// CrCb +/- Threshold value

#define CAMERA_WINDOW_NAME_RAW					"Robot Camera Raw"
//#define CAMERA_WINDOW_NAME_PROCESSED			"Robot Camera Processed"
#define CAMERA_WINDOW_NAME_LEFT					"Left Camera"
#define CAMERA_WINDOW_NAME_RIGHT				"Right Camera"
#define CAMERA_WINDOW_NAME_KINECT_DEPTH			"Kinect Depth"
#define CAMERA_WINDOW_NAME_KINECT_VIDEO			"Kinect Video"
#define CAMERA_WINDOW_NAME_MOTION				"Motion Detector"
#define CAMERA_WINDOW_NAME_COLOR_BLOB			"Color Blob Debug"


#define CAMERA_WINDOW_DISPLAY_SIZE_LARGE			FALSE
#if (CAMERA_WINDOW_DISPLAY_SIZE_LARGE == TRUE)
	#define CAMERA_WINDOW_DISPLAY_SIZE_X			640
	#define CAMERA_WINDOW_DISPLAY_SIZE_Y			480
	#define KINECT_WINDOW_DISPLAY_SIZE_X			640
	#define KINECT_WINDOW_DISPLAY_SIZE_Y			480
#else
	#define CAMERA_WINDOW_DISPLAY_SIZE_X			320
	#define CAMERA_WINDOW_DISPLAY_SIZE_Y			240
	#define KINECT_WINDOW_DISPLAY_SIZE_X			320
	#define KINECT_WINDOW_DISPLAY_SIZE_Y			240
#endif

#define KINECT_CAPTURE_SIZE_MAX_X					640
#define KINECT_CAPTURE_SIZE_MAX_Y					480



#define	CAMERA_CAPTURE_FRAME_RATE		30.0	// FPS default = 30.0

#define MOTION_DETECTED_NONE		0x00
#define MOTION_DETECTED_LEFT		0x01
#define MOTION_DETECTED_RIGHT		0x02
#define MOTION_DETECTED_BOTH		0x04


#define ACCEL_OFFSET_X	131// Center Value - used to set center at 0,0
#define ACCEL_OFFSET_Y	131

// Kobuki Base IR Beacon
#define KOBUKI_BASE_NEAR_RIGHT				0x01	// Backward from docs. From the perspective of the Robot, not the base
#define KOBUKI_BASE_NEAR_CENTER				0x02	//
#define KOBUKI_BASE_NEAR_LEFT				0x04	// Backward from docs. From the perspective of the Robot, not the base

#define KOBUKI_BASE_FAR_CENTER				0x08	// (Yes, Center and left really are swapped for far ranges.  Doh)
#define KOBUKI_BASE_FAR_RIGHT				0x10	// Backward from docs. From the perspective of the Robot, not the base
#define KOBUKI_BASE_FAR_LEFT				0x20	// Backward from docs. From the perspective of the Robot, not the base



////////////////////////////////////////////////////////////////////////////////
// Module Identifiers, in priority order (bigger number = higher priority)
// But, lower priority modules may suppress higher priority modules (brain planning suppresses reflex)
//#define	OVERRIDE_MODULE			0x80	// Emergency Override, used to stop robot
#define	LOCAL_USER_MODULE		0x40	// Local direct control
#define	COLLISION_MODULE		0x20
#define	AVOID_OBJECT_MODULE		0x10
#define	REMOTE_USER_MODULE		0x08
#define BEHAVIOR_GOAL_MODULE	0x04
#define	WAY_POINT_NAV_MODULE	0x02
#define	GRID_NAV_MODULE			0x01
#define	NO_MODULE				0x00

////////////////////////////////////////////////////////////////////////////////
// Results of Owner Priority Check
enum MODULE_OWNER_TEST_RESULT_T {
		MODULE_SUPPRESSED = 0,				// Module Suppressed.  No action
		MODULE_HIGHER_PRIORITY_HAS_CONTROL,	// Possible to negotiate speed and turn	
		MODULE_OWNERSHIP_REQUEST_SUCCESS,		 
};
// SUPPRESSED - do nothing
// HIGHER_OWNER - negotiate speed and turn
// SUCCESS - use value
////////////////////////////////////////////////////////////////////////////////
enum HEAD_SERVO_OWNERS {
		HEAD_OWNER_NONE = 0,		// Lowest Priority
		HEAD_OWNER_RANDOM_MOVEMENT,
		HEAD_OWNER_PIR_TRACKER,
		HEAD_OWNER_MOTION_TRACKER,
		HEAD_OWNER_BEHAVIOR_P2,
		HEAD_OWNER_FACE_TRACKER,
		HEAD_OWNER_KINECT_HUMAN,
		HEAD_OWNER_HEAD_NOD,		// nod head when talking
		HEAD_OWNER_BEHAVIOR_P1,
		HEAD_OWNER_TRACK_OBJECT,
		HEAD_OWNER_USER_CONTROL		// Highest Priority
};



////////////////////////////////////////////////////////////////////////////////
// Kinect Tilt Owner Identifiers, in priority order (bigger number = higher priority)
enum KINECT_SERVO_OWNERS {
		KINECT_TILT_OWNER_NONE = 0,		// Lowest Priority
		KINECT_TILT_OWNER_COLLISION_AVOIDANCE,		
		KINECT_TILT_OWNER_TRACK_HUMAN,		
		KINECT_TILT_OWNER_TRACK_OBJECT,		
		KINECT_TILT_OWNER_USER_CONTROL		// Highest Priority
};




// All motor modules except for User:
#define ALL_AUTO_MOTOR_MODULES (COLLISION_MODULE | AVOID_OBJECT_MODULE | WAY_POINT_NAV_MODULE | GRID_NAV_MODULE)
#define AVOID_AND_COLLISION_MODULES (COLLISION_MODULE | AVOID_OBJECT_MODULE )


////////////////////////////////////////////////////////////////////////////////
// Structures




typedef struct
{
	int		X;
	int		Y;
} POINT2D_T;

typedef struct
{
	int		X;
	int		Y;
	int		Z;
} POINT3D_T;

typedef struct
{
	double		X;
	double		Y;
	double		Z;
} FPOINT3D_T;


// Structure to hold real or projected servo positions, in DEGREES
typedef struct
{
	double		ShoulderAngle;
	double		ElbowBendAngle;
	double		ElbowRotateAngle;
	double		WristRotateAngle;
} ARM_SERVOS_POSITION_T;

typedef struct
{
	BYTE Cmd;
	BYTE Param1;
	BYTE Param2;
	BYTE Param3;
	BYTE Param4;
} SIMULATED_HW_CMD_T;

/////////////////////////////////////////////////////////////////////////////
// FullSensorStatus
// Detailed Global Sensor data from each sensor.  Use NavSensorSummary instead when possible.
class FullSensorStatus
{
public:
				FullSensorStatus();		// Constructor automatically initializes defaults
	void		InitializeDefaults();	// Set all sensor readings to "no object detected"
										// If sensor not installed, override with SENSOR_DISABLED after calling this function!
	// Basic Data
	BYTE	StatusFlags;				// Typically From Arduino
	int 	LastError;					// Typically From Arduino
	int 	DebugCode;					// Typically From Arduino
	int 	Battery0;					// Typically From Arduino
	int 	Battery1;					// Typically From Arduino

	// Cliff and Wheel drops
	bool	CliffFront;
	bool	CliffRight;
	bool	CliffLeft;
	bool	WheelDropRight;
	bool	WheelDropLeft;

	// TODO remove this?
	//int		Cliff;
	//int		WheelDrop;


	// Hardware Bumpers, IR range switches, and pressure sensors
	//	int 	HWBumper;					// Typically From Arduino
	//	int 	IRBumper;					// Typically From Arduino
	//	int 	IRBumper2;					// Calculated from Vertical IR detector range
	bool	HWBumperFront;
	bool	HWBumperRear;
	bool	HWBumperSideLeft;
	bool	HWBumperSideRight;

	bool	IRBumperFrontLeft;
	bool	IRBumperFrontRight;
	bool	IRBumperRearLeft;
	bool	IRBumperRearRight;
	bool	IRBumperSideLeft;
	bool	IRBumperSideRight;

	bool	ArmRightBumperElbow;
	bool	ArmLeftBumperElbow;
	bool	ArmLeftBumperFingerLeft;
	bool	ArmLeftBumperFingerRight;
	bool	ArmLeftBumperInsideClaw;

	// TODO - remove this?
	//int 	ArmBumperL;					// Typically From Arduino
	//int 	ArmBumperR;					// Typically From Arduino

	int		LeftHandRawPressureL;		// Pressure values are only useable if CalibratePressureSensors 
	int		LeftHandRawPressureR;		// is called before each use.  Use GetPressureLoadPercent to get final value

	// Heading and Odometry
	int		CompassHeading;				// Typically From Arduino
	//int		CompassError;				// Typically From Arduino
	double	OdometerTenthInches;		// Typically From Arduino or ER1 Pilot
	double	OdometerUpdateTenthInches;	// Calculated from Arduino or ER1 data
	int 	Tachometer;					// Typically From Arduino or ER1 Pilot
	int 	TachometerTicksL;			// Typically From Arduino, provided feedback for Motor Speed Control
	int 	TachometerTicksR;			// Typically From Arduino, provided feedback for Motor Speed Control
//	double	TurnAngleUpdate;			// Calculated from ER1 motor positions
	double	DistanceToWaypoint;			// Calculated
	double	CalculatedMotorHeading;		// Calculated from motor movements (ER1)
	FPOINT	CurrentLocation;			// Calculated from compass and odometer
	FPOINT	CurrentLocationMotor;		// Calculated from motor movements (ER1)
	FPOINT	CurrentLocationGPS;			// Current location of robot as indicated by GPS

	// Kobuki Dock 
	int		DockSensorRight;			
	int		DockSensorCenter;			// IR sensors for the Kobuki Dock
	int		DockSensorLeft;

	// From Android Phone
	BOOL	AndroidConnected;
	BOOL	AndroidAccEnabled;
	int		AndroidCommand;				// Commands received from Android phone over Bluetooth
	int		AndroidCompass;				// X,Y,Z data received from Android phone over Bluetooth
	int		AndroidRoll;
	int		AndroidPitch;

	// Other Sensors and state
	bool	PIRMotionLeft;
	bool	PIRMotionRight;
	int 	ThermalArray[9];			// Value picked up by TPS Thermal sensor
	int		ThermalPosition;			// Position of thermal object detected.  Negative = left of center
	int		TiltAccelX;					// Typically From Arduino.  zero = level
	int		TiltAccelY; 				// Typically From Arduino.  zero = level
//	double	VideoFPS;					// How quickly video frames are processed
	BOOL	AuxLightsOn;				// Track if the Aux lights are on or off

	//		Analog Sensors
	int 	US[NUM_US_SENSORS];			// Typically From Arduino
	int 	IR[NUM_IR_SENSORS];			// Typically From Arduino
	int 	IR3[NUM_IR3_SENSORS];		// Typically From Arduino


};	// FullSensorStatus
/////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////
// NavSensorSummary
// Processed and summarized sensor data
class NavSensorSummary
{
public:
				NavSensorSummary();		// Constructor automatically initializes defaults
	void		InitializeDefaults();	// Set all sensor readings to "no object detected"
										// If sensor not installed, override with SENSOR_DISABLED after calling this function!
	bool		CliffDetected();		// Some cliff detected
	bool		CliffFront();			// Cliff dead ahead
	bool		BumperHitFront();		// Hit something with the front bumper

	// summary
	int 		nFrontObjectDistance;	// Any object in front of and within width of robot, including Arms
	int			nFrontObjectDirection;	// Right or Left of center.  Negative = Left
	int 		nSideObjectDistance;	// Closest object on either side
	int			nSideObjectDirection;	// Which side has the closest object
	int 		nRearObjectDistance;	// Closest object in the rear
	int			nRearObjectDirection;	// Which side has the closest object

	int 		nClosestObjectFrontLeft;
	int 		nClosestObjectFrontRight;

	bool		bCliffLeft;
	bool		bCliffFront;
	bool		bCliffRight;
	bool		bWheelDropLeft;
	bool		bWheelDropRight;
	bool		bHWBumperFront;

	// zones
	int 		nLeftRearZone;
	int 		nObjectClawLeft;
	int 		nObjectArmLeft;		// compensated for distance in front of robot - REMOVE
	int 		nLeftSideZone;	
	int 		nLeftFrontSideZone;	
	int 		nLeftArmZone;
	int 		nLeftFrontZone;		

	int 		MotionDetectedDirection; ////// Robot Center

	int 		nRightFrontZone;
	int 		nRightArmZone;	
	int 		nRightFrontSideZone;	
	int 		nRightSideZone;	
	int 		nObjectArmRight;	// compensated for distance in front of robot - REMOVE
	int 		nObjectClawRight;
	int 		nRightRearZone;

};



enum IROBOT_BATTERY_CHARGING_STATE {
	 IROBOT_BATTERY_NOT_CHARGING = 0,
	 IROBOT_BATTERY_RECONDITIONING,
	 IROBOT_BATTERY_FULL_CHARGING,
	 IROBOT_BATTERY_TRICKLE_CHARGING,
	 IROBOT_BATTERY_WAITING,
	 IROBOT_BATTERY_FAULT
};


typedef struct
{
	BOOL	WheelDropCaster;
	BOOL	WheelDropLeft;
	BOOL	WheelDropRight;
	BOOL	BumperLeft;
	BOOL	BumperRight;
	BOOL	WallDetected;
	BOOL	CliffLeft;
	BOOL	CliffFrontLeft;
	BOOL	CliffFrontRight;
	BOOL	CliffRight;
	BOOL	VirtualWall;
	BOOL	OverCurrent;			// Actually 4 separate bits, but we just care if it's non-zero
	int		IRcode;					// code received from remote control, beacons, home, etc.
	int		Distance;				// in mm, since last update
	int		Angle;					// in degrees since last update
	int		ChargingState;			// See IROBOT_BATTERY_CHARGING_STATE
	int		BatteryVoltage;			// in miliVolts
	int		BatteryCurrent;			// in milliAmps (mA)
	int		BatteryTemperature;		// degrees Celsius
	int		BatteryCharge;			// current charge state in milliAmp Hours (mAh)
	int		BatteryCapacity;		// estimated capacity in milliAmp Hours (mAh)
	int		ExpansionDigitalInputs;	// bottom 4 bits are digital inputs
	int		ExpansionAnalogInput;	// Accelerometer!
	BOOL	InHomeBase;				// True when charging on the home base available
	BOOL	ChargingPlugInserted;	// True when charging plug inserted
	int		OIMode;					// Current Mode: Off/Passive/Safe/Full
} IROBOT_STATUS_T;					// Status from iRobot Create base




typedef struct
{
	unsigned int	TimeStamp;				// when sensor data was reported
	double			GyroDegrees;			// From direction robot was facing when Kobuki Base was powered up
	double			TurnRate;				//
	int				OdometerTicksLeft;		// Raw Encoder ticks
	int				OdometerTicksRight;		// Raw Encoder ticks
	int				OdometerLeft;			// tenth inches - distance since power on
	int				OdometerRight;			// tenth inches - distance since power on
	double			BatteryVoltage;			// 
	double			BatteryPercent;			//
	unsigned int	BatteryLevelEnum;		// See enum BatteryLevel
	unsigned int	BatteryChargeSourceEnum;// See enum BatteryChargeSource
	unsigned int	BatteryChargeStateEnum;	// See enum BatteryChargeState
	bool			BumperLeft;
	bool			BumperFront;
	bool			BumperRight;
	bool			CliffLeft;
	bool			CliffFront;
	bool			CliffRight;
	bool			WheelDropLeft;
	bool			WheelDropRight;
	bool			MotorStallErrorLeft;
	bool			MotorStallErrorRight;
	unsigned int	ButtonPress;			// bit values for button 1,2,3// 
	int				MotorPWMLeft;			// 
	int				MotorPWMRight;			// 
	unsigned int	MotorCurrentLeft;		// in 10mA
	unsigned int	MotorCurrentRight;		// in 10mA
	unsigned int	DockLeftSignal;			//
	unsigned int	DockCenterSignal;		//
	unsigned int	DockRightSignal;		//
	unsigned int	RightCliffA2D;			//
	unsigned int	CenterCliffA2D;			//
	unsigned int	LeftCliffA2D;			//

} KOBUKI_STATUS_T;

/* EACH SENSOR'S DOCK Flags will be set when signal is detected
0x01 for NEAR_LEFT state
0x02 for NEAR_CENTER state
0x04 for NEAR_RIGHT state
0x08 for FAR_CENTER state
ox10 for FAR_LEFT state
0x20 for FAR_RIGHT state	
*/



typedef struct
{
	int			PositionTenthDegrees;
	int 		Speed; // Generic Servo Speed.  Range = 1-15
	int			Load;	// Dyna Load value - range = 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)
	int 		TemperatureFahrenheit;
	int 		StatusFlags;
	int			StallTimer;
	BOOL		Enabled;
} BULK_SERVO_STATUS_T; // used by g_BulkServoStatus


typedef struct
{
	int			PositionTenthDegrees;
	int 		Speed;	// Generic Servo Speed 1-15.  0 = use current
	int 		Delay;	// Wait this many MS before moving this servo
	BOOL		Enable;	// Enable / Disable Torque
	BOOL		Update;	// "Dirty Bit" Flag to indicate servo needs updating
} BULK_SERVO_CMD_T; // used by g_BulkServoCmd


// Shared data between writer and reader threads for 
// Hokuyo URG-04LX-UG01 Laser Scanner, using SCIP 2.0 Protocol
//#define LASER_SCANNER_MAX_CMD_SIZE		  32	// Max command size in characters
#define LASER_SCANNER_READ_BUF_SIZE			2239	// Max size of SIO buffer (can handle multiple reads)
#define LASER_SCANNER_CMD_BUF_SIZE			  63
#define LASER_SCAN_MAX_2D_OBJECTS			 255	// max 2D object slices that can be found by the laser scanner LookForObjects function
#define LASER_SCAN_MAX_3D_OBJECTS			  32  


// Kinect Object detection limits 
#define KINECT_SCAN_MAX_2D_OBJECTS			 255	// max 2D object slices that can be found by the laser scanner LookForObjects function
#define KINECT_SCAN_MAX_3D_OBJECTS			  32  


// Laser Object 2D data
// This is a "slice" of an object.  Height and width but not depth
typedef struct
{
	int		X;		// Summary
	int		Y;
	int		PeakHeight;
	int		Width;
	int		StartX;	// Details
	int		StartY;
	int		StartZ;
	int		EndX;
	int		EndY;
	int		EndZ;
	int		LeftPixel;
	int		RightPixel;
	int		ScanLine;
} DETECTED_OBJECT_2D_T;	// All values in TenthInches
typedef struct
{
	int						nObjectsDetected; // Number of objects detected so far
	DETECTED_OBJECT_2D_T	Object[LASER_SCAN_MAX_2D_OBJECTS];
} OBJECT_2D_ARRAY_T;


// Laser Object 3D data
// This is full object, including height, width, and length
typedef struct
{
	int		CenterX;	// current average center
	int		CenterY;
	int		Height;
	int		Width;
	int		Length;
	int		LeftPixel;	// for GUI bounding box
	int		RightPixel;
	int		StartScanLine;
	int		EndScanLine;
} DETECTED_OBJECT_3D_T;	// All values in TenthInches

typedef struct
{
	int						nObjectsDetected;
	int						nClosestObjectIndex;
	int						nClosestObjectDistance;
	DETECTED_OBJECT_3D_T	Object[LASER_SCAN_MAX_3D_OBJECTS];
} OBJECT_3D_ARRAY_T;


typedef struct
{
	BOOL		bReaderInitialized;	// Make sure reader is initialized before sending commands
	char		LastCmd[LASER_SCANNER_CMD_BUF_SIZE];	// Last command sent to the scanner
} LASER_SCANNER_STATE_T; // used by g_LaserScannerState


typedef struct
{
	int					NumberOfSamples;
	FPOINT				RobotLocation;								// Location of robot at the time of the laser scan
	int					CompassHeading;								// Heading of robot at the time of the laser scan
	int 				ScanData[LASER_RANGEFINDER_MAX_SAMPLES];	// Data in TENTH INCHES from the scanner
	POINT2D_T			ScanPoints[LASER_RANGEFINDER_MAX_SAMPLES];	// Data translated to X,Y from front center of Robot
} LASER_SCANNER_DATA_T; // used by g_pLaserScannerData


typedef struct
{
	DWORD		dwTimeOfSample;
	FPOINT		RobotLocation;							// Location of robot at the time of the laser scan
	int			CompassHeading;							// Heading of robot at the time of the laser scanscanner
	int			FrameSizeX;
	int			FrameSizeY;
	POINT2D_T	MapPoints2D[KINECT_CAPTURE_SIZE_MAX_X];		// 2D map of objects in X,Y from front center of Robot
	POINT2D_T	WallPoints[KINECT_CAPTURE_SIZE_MAX_X];		// Wall distance X,Y from front center of Robot (ignores low objects)
	POINT3D_T	Point3dArray[KINECT_CAPTURE_SIZE_MAX_Y][KINECT_CAPTURE_SIZE_MAX_X];	// 3D values in TenthInches

} KINECT_3D_CLOUD_T;


// TODO - REMOVE THIS - DEBUG ONLY!!!
#define MAX_DEBUG_SLICES	2047
typedef struct
{
	POINT		Pt1;
	POINT		Pt2;
} DEBUG_SLICE_T;

typedef struct
{
	int						nSlices;
	DEBUG_SLICE_T			Slice[MAX_DEBUG_SLICES+1];
} DEBUG_SLICE_ARRAY_T;


typedef struct
{
	BOOL		bLeftCliff;
	int 		nLeftRearZone;
	int 		nLeftSideZone;		
	int 		nLeftFrontSideZone;		
	int 		nLeftArmZone;			
	int 		nLeftFrontZone;		//Left Center
	int 		nRightFrontZone;	//Right Center
	int 		nRightArmZone;	
	int 		nRightFrontSideZone;		
	int 		nRightSideZone;		
	int 		nRightRearZone;
	BOOL		bRightCliff;
	//int			LaserAngleTenthDegrees;	// Up/down angle of laser at last scan
	FPOINT		RobotLocation;		// Location of robot at the time of the laser scan
	int			CompassHeading;		// Heading of robot at the time of the laser scan
	DWORD		SampleTimeStamp;	// Time laser sample was done
} SCANNER_SUMMARY_T; 


// from RobotClientSock
typedef struct
{
	SOCKET	sock;
	char	szIPAddress[100];
	HWND	hDlgWnd;
	HANDLE	hClientSockSendThread;
	HANDLE	hClientSockReceiveThread;
} CLIENT_SOCKET_STRUCT;


// from RobotServerSock
typedef struct
{
	SOCKET	sockConnected;
	HANDLE	hReceiveThread;
	HANDLE	hSendThread;
} SERVER_SOCKET_STRUCT;


// Camera Structure


typedef struct
{
	int 			State;		// Current state of the camera ( see enum CAMERA_STATE )
//	CvCapture*		pCapture;	// Capture structure for the camera
	SIZE			FrameSize;
	SIZE			DisplaySize;
	BOOL			Flip;		// Flip video horizontally
}	CAMERA_T;


#define KINECT_MAX_HUMANS_TO_TRACK 16 // TODO - change to 8?
typedef struct
{
	BOOL			Found;	// inicate if this Human Player number is being tracked
	POINT3D_T		HeadLocation;
	POINT2D_T		AngleTenthDegrees;
} KINECT_HUMAN_TRACKING_T;

typedef struct
{
	BOOL			Found;	// inicate if this Human Player number was found in this frame
	BOOL			TopOfHeadVisable; // if visable, Y can be used to calculate player's height
	POINT2D_T		Pixel;
	int				HeadLocationY;
	int				HeadLocationZ;
	int				HeadLocation_LeftX;			// Left edge of head
	int				HeadLocation_RightX;		// Right edge of head
	int				AngleTenthDegreesY;			// Angle up from the Robot's view
	int				AngleTenthDegrees_LeftX;	// Left edge of head
	int				AngleTenthDegrees_RightX;	// Right edge of head
	int				ScanLine;
} KINECT_HUMAN_FINDING_T;


// Frequency at which Servo status is updated.  Used with g_nServoStatusRequestsPerSecond
#define SERVO_STATUS_REQUEST_FREQ_NORMAL	1	// times per second.  
//#define SERVO_STATUS_REQUEST_FREQ_FAST	    2 	// times per second


// For Launching other processes
#define FT64(filetime) (*((LONGLONG*)&(filetime)))
typedef struct
{
	DWORD					dwProcessID;
	PROCESS_INFORMATION		ProcessInfo;
	HWND 					hWndFound;
} FIND_WINDOW_HANDLE_STRUCT_T;

 
/****
// Pre-defined Arm positions

//									   Shoulder, El Rot,   El Bend,     Wrist,      Grip
const int ArmPosHomeR[]				= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosPointAtSelfR[]		= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosWave1R[]			= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosWave2R[]			= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosPointAhead[]		= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosDance1[]			= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees
const int ArmPosDance2[]			= {  0000,	  0000,	     0000,		 0000,		0000 };	// Degrees

***/

//-----------------------------------------------------------------------------
// Name: FastTimer Class
// Desc: High resolution timer for fast events
//-----------------------------------------------------------------------------
typedef struct {
    LARGE_INTEGER start;
    LARGE_INTEGER last;
} TIMER_STATE_T;

class FastTimer {

private:
	TIMER_STATE_T timer;
	LARGE_INTEGER frequency;
	double LargeIntToSeconds( LARGE_INTEGER & L);
	double LargeIntToMS( LARGE_INTEGER & L);
public:
	FastTimer();
	void startTimer();		// Start or Restart timer at zero
	void stopTimer();		// Reset timer to zero and don't start counting
	BOOL Running();			// See if Timer is running (has been started)
	double getTime();		// Get time since timer started
	double getElapsedTime();// Get elapsed time since last status request
};

//-----------------------------------------------------------------------------
// Name: IsDynaServoMoving
// Desc: See if any of the Dynamixel servos are moving
// Used to set the frequency of servo updates dynamically
//-----------------------------------------------------------------------------
BOOL IsDynaServoMoving();

//-----------------------------------------------------------------------------
// Name: IsKerrServoMoving
// Desc: See if one of the Kerr servos (shoulder motors) are moving
// Used to set the frequency of servo updates dynamically
//-----------------------------------------------------------------------------

BOOL IsKerrServoMoving();


//-----------------------------------------------------------------------------
// Name: SpeakText
// Desc: queues text to speak and signals text to speech thread
//-----------------------------------------------------------------------------
void SpeakText( CString &TextToSpeak );
void SpeakText( const char *TextToSpeak );

//-----------------------------------------------------------------------------
// Name: LaunchKinectApp
// Desc: If enabled, auto-launch the C# applicaiton that handles the 
// Kinect audio and video+depth cameras
// Note: started at different times for Loki or Turtle, since iRobot base enables power only on startup
//-----------------------------------------------------------------------------
void LaunchKinectApp();
void TerminateKinectApp();

//-----------------------------------------------------------------------------
// Name: LaunchCameraApp
// Desc: If enabled, auto-launch the applicaiton that handles camera input
// Done as a separate application, so core app can be run debug, but camera runs optimized release code
// OLD Note: started at different times for Loki or Turtle, since iRobot base enables power only on startup
//-----------------------------------------------------------------------------
void LaunchCameraApp();
void TerminateCameraApp();

//-----------------------------------------------------------------------------
// Name: LaunchKobukiApp
// Desc: If enabled, auto-launch the applicaiton that handles Kobuki Base control
// Done as a separate application, so core app can be run debug, while Kobuki only can run in release code (no debug dlls available)
// Note that this must start before access Kinect, since Kobuki Base enables Kinect power only on startup
//-----------------------------------------------------------------------------
void LaunchKobukiApp();
void TerminateKobukiApp();


//-----------------------------------------------------------------------------
void ReportCommError( LPTSTR lpszMessage, DWORD dwCommError );

void InitScannerSummaryData( SCANNER_SUMMARY_T* Summary );

//void InitSensorSummaryData( SENSOR_SUMMARY_T* Summary );

#if GPA_TRACE_ENABLED
	void RobotSleep( DWORD msSleepTime, __itt_domain *ThreadDomain );
#else
	void RobotSleep( DWORD msSleepTime, char *ThreadDomain );
#endif


////////////////////////////////////////////////////////////////////////////////
// Global Variables
//

extern ARDUINO_STATUS_T		g_RawArduinoStatus;

extern FullSensorStatus*	g_pFullSensorStatus;			// Current status of all sensors

extern NavSensorSummary*	g_pNavSensorSummary;
extern SCANNER_SUMMARY_T*	g_pLaserSummary;
extern SCANNER_SUMMARY_T*	g_pKinectSummary;

extern HWND					g_RobotMainFrameHWND;			// Handle to main frame, for Joystick
extern HWND					g_RobotCmdViewHWND;				// Handle for posting messages to Command window
extern HWND					g_RobotPathViewHWND;			// Handle for posting messages to Path window
extern HWND					g_RobotMapViewHWND;				// Handle for posting messages to Map window
extern HWND					g_RobotSetupViewHWND;			// Handle for posting messages to Setup window

extern BOOL					g_bRunThread;					// When FALSE, tells all threads to exit
extern BOOL					g_bRunVidCapThread;				// When FALSE, tells Vidcap thread to exit
extern BOOL					g_bRunKinectThread;				// When FALSE, tells Kinect thread to exit

// Global Pause and Power Control - Allows pausing or powering on/off subsystems instantly
extern BOOL					g_SleepMode;					// Power on by default, unless in lowpower "sleep mode"
extern BOOL					g_DynaPowerEnabled;
extern BOOL					g_KinectPowerEnabled;
extern BOOL					g_GlobalPause;					// freeze all servos and motors until unpaused

extern FILE*				g_LogFile;						// Log file for non-debug mode
extern BOOL					g_CriticalSectionsInitialized;	// Flag when CS are valid


//maker
extern BOOL					g_StopBehavior;					// When enabled, blocks all speech recognition by the C++ code, including "Stop". 

extern HANDLE				g_hSpeechRecoEvent;				// Synchronization between C# app and C++ for Speech recognition
extern HANDLE				g_hKinectDepthReadyEvent;		// Synchronization between C# app and C++ for when depth data is ready
extern BOOL					g_SpeechRecoBlocked;			// When enabled, blocks all speech recognition by the C++ code, including "Stop". 
															// Used when arm motors are moving due to noise picked by Kinect

// ROBOT_SERVER globals

// Thread Handles
extern HANDLE				g_hControlThread;
extern HANDLE				g_hSoundThread;
extern HANDLE				g_hSpeakThread;
extern HANDLE				g_hCameraVidCapThread;
extern HANDLE				g_hKinectThread;
extern HANDLE				g_hTimerThread;
extern HANDLE				g_hSmartServoThread;

extern HANDLE				g_hServoThread;
extern HANDLE				g_hArduinoWriteThread;
extern HANDLE				g_hArduinoReadThread;
extern HANDLE				g_hCameraThread;
extern HANDLE				g_hMotorWriteThread;
extern HANDLE				g_hMotorReadThread;
//extern HANDLE				g_hTrexWriteThread;
extern HANDLE				g_hLaserScannerReadThread;
extern HANDLE				g_hLaserScannerWriteThread;
extern HANDLE				g_hGPSThread;
extern HANDLE				g_hiRobotReadThread;	// Read thread for iRobot Base.  The write thread is g_dwMotorCommThreadId
extern HANDLE				g_hKinectNuiThread;
//extern HANDLE				g_hEvNuiProcessStop;
extern HANDLE				g_hKinectAppSharedMemoryIPCThread;
extern HANDLE				g_hCameraAppSharedMemoryIPCThread;
extern HANDLE				g_hKobukiAppSharedMemoryIPCThread;

extern DWORD				g_dwControlThreadId;	// Control Thread for communicating to hardware
extern DWORD				g_dwSoundThreadId;		// Thread for playing sounds
extern DWORD				g_dwSpeakThreadId;			// Thread for processing AI input
extern DWORD				g_dwCameraVidCapThreadId;	// Thread for camera object recognition
extern DWORD				g_dwServerSendThreadId;	// For sending messages to the client Socket thread
extern DWORD				g_dwKinectAppSharedMemoryIPCThreadId;	// Thread for shared memory access between managed C# and C++ processes
extern DWORD				g_dwCameraAppSharedMemoryIPCThreadId;	// Thread for shared memory access between managed C# and C++ processes
extern DWORD				g_dwKobukiAppSharedMemoryIPCThreadId;	// Thread for shared memory access between managed C# and C++ processes


// System Status
extern SUBSYSTEM_STATUS		g_ArduinoSubSystemStatus;   
extern SUBSYSTEM_STATUS		g_DynaSubSystemStatus;   
extern SUBSYSTEM_STATUS		g_RX64SubSystemStatus;
extern SUBSYSTEM_STATUS		g_KerrSubSystemStatus;
extern SUBSYSTEM_STATUS		g_MotorSubSystemStatus;
extern SUBSYSTEM_STATUS		g_GPSSubSystemStatus; 

extern SUBSYSTEM_STATUS		g_KinectSubSystemStatus;
extern SUBSYSTEM_STATUS		g_CameraSubSystemStatus;
extern SUBSYSTEM_STATUS		g_LaserSubSystemStatus;
extern SUBSYSTEM_STATUS		g_LeftArmSubSystemStatus;
extern SUBSYSTEM_STATUS		g_RightArmSubSystemStatus;



// for Arduino serial communication
extern HANDLE				g_hArduinoCommPort;
extern DWORD				g_dwArduinoCommWriteThreadId;	// Comm Thread for sending to Arduino

// For External Servo Controller
extern HANDLE				g_hServoCommPort;
extern DWORD				g_dwServoCommWriteThreadId;	// Comm Thread for sending to Servo Controller

// For Sony Camera
extern HANDLE				g_hCameraCommPort;
extern DWORD				g_dwCameraCommWriteThreadId;	// Comm Thread for sending to Sony Camera

// For Cameras and Kinect
extern void*				g_pCameraModule;				// make pointer available to Camera Video Thread
extern void*				g_pKinectModule;				// make pointer available to Kinect Video Thread
extern void*				g_pKinectNui;
extern DWORD				g_LastCameraMoveTime;			// indicate when to begin IDLE behavior


// For any Motor Controller (ER1, TREX, IROBOT, etc.)
extern int 					g_nMotorStatusTimer;
extern HANDLE				g_hMotorCommPort;
extern DWORD				g_dwMotorCommThreadId;	// Comm Thread for sending to Motor Controller

extern HANDLE				g_hTrexCommPort;
extern DWORD				g_dwTrexMotorCommWriteThreadId;	// Comm Thread for sending to Servo Controller


// For iRobot Create Base
extern IROBOT_STATUS_T*		g_pIRobotStatus;		// Status data updated every 15ms

// For Kobuki Base
extern KOBUKI_STATUS_T*		g_pKobukiStatus;			// Status data updated frequently


// For Dynamixel and Kerr Servos
extern CRITICAL_SECTION		g_csServoLock;			// Initialized in Robot.cpp 
extern DWORD				g_dwSmartServoCommThreadId;	// Comm Thread for sending to Dynamixel and Kerr Servos
extern HANDLE				g_hDynaServoCommPort_AX12;
extern HANDLE				g_hDynaServoCommPort_RX64;
extern HANDLE				g_hKerrServoCommPort;
extern int 					g_nServoStatusTimer;
extern int 					g_nServoStatusRequestsPerSecond;
extern int					gServoOverheatError;		// SERVO NUMBER THAT IS OVERHEATING. IF NONZERO, DISABLES ALL ARM MOVEMENTS to prevent servo damage

// For Laser Scanner
extern CRITICAL_SECTION		g_csLaserSummaryDataLock;			// Initialized in Robot.cpp 
extern CRITICAL_SECTION		g_csLaserDataLock;					// Initialized in Robot.cpp 
extern HANDLE				g_hLaserScannerCommPort;
extern DWORD				g_dwLaserScannerCommWriteThreadId;	// Comm Thread for sending to Hokuyo URG-04LX-UG01 Laser Scanner
//extern DWORD				g_dwLaserScannerCommReadThreadId;	// Comm Thread for reading data from the Laser Scanner
extern BOOL					g_bLaserScanEnabled;				// When TRUE, short term laser scan is running.  get frequent servo updates
extern BOOL					g_bLaserContinuousScanEnabled;		// When TRUE, Laser runs on timer
extern int 					g_LaserScansRemaining;				// Number of scans remaining when laser scanner is in multi scan mode
extern LASER_SCANNER_STATE_T g_LaserScannerState;
extern LASER_SCANNER_DATA_T* g_pLaserScannerData;


// For Kinect
extern CRITICAL_SECTION		g_csKinectSummaryDataLock;			// Initialized in Robot.cpp 
extern CRITICAL_SECTION		g_csKinectPointCloudLock;			// Initialized in Robot.cpp 
extern CRITICAL_SECTION		g_csKinectHumanTrackingLock;		// Initialized in Robot.cpp 

extern OBJECT_2D_ARRAY_T*	g_pKinectObjects2D;					// Array of 2D object slices from one scan line
extern OBJECT_3D_ARRAY_T*	g_pKinectObjects3D;					// Array of 3D complete objects detected in the complete capture
extern KINECT_3D_CLOUD_T*	g_KinectPointCloud;					// Array of 3D points detected by Kinect in TenthInches
extern KINECT_HUMAN_TRACKING_T g_HumanLocationTracking[KINECT_MAX_HUMANS_TO_TRACK]; // Locations of humans being tracked
extern int					g_CurrentHumanTracked;				// Current player number of the human currently being tracked (zero if none)
extern int					g_LastHumanCompassDirection;		// Direction last human was at
extern float				g_LastHumanAudioBeamDirection;		// Direction last command came from

//#if (DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1)
//	extern DEBUG_SLICE_ARRAY_T* g_pKinectDebugSliceArray;			// temp array for debugging slices
//#endif


// Status and Command blocks for all Dyna Servos and Kerr Motors
extern BULK_SERVO_STATUS_T	g_BulkServoStatus[NUMBER_OF_SMART_SERVOS+1];	// +1 because servo numbers start at 1, not 0!
extern BULK_SERVO_CMD_T		g_BulkServoCmd[NUMBER_OF_SMART_SERVOS+1];		// (0 is "wasted space, but avoids common index error)

extern CAMERA_T				g_Camera[4];	// State and Capture structure for each camera attached (LEFT_CAMERA or RIGHT_CAMERA)


// For GPS device
extern HANDLE				g_hGPSCommPort;
extern BOOL					g_GPSGotFirstFix;		// If false, indicates no GPS Lock yet
extern double				gGPSOriginLat;			// Latitude of the Map Origin
extern double				gGPSOriginLong;			// Longitue of the Map Origin

// Other Stuff
extern BOOL					g_bConnectedToClient;
extern int					g_nClientKeepAliveCount;
extern int					gStartTime;				// Time that the robot started executing.  Used as baseline for time numbers
extern int					gCurrentTime;			// Convenient tenth second count-up timer
extern int					gPicWatchDogTimer;
extern int					gMotorSpeedTimer;
extern int					gBrakeTimer;
extern int					gCollisionTimer;
extern int					gAvoidanceTimer;
extern int					gNavigationTimer;
extern int					gServoOverHeatTimer;
extern int					gArmTimerRight;
extern int					gArmTimerLeft;
extern int					gBehaviorTimer;
extern int					gHeadNodTimer;

extern int					gKinectMoveTimeout;
extern int					gKinectDelayTimer;
extern int					gKinectOwnerTimer;
extern int					gKinectCurrentOwner;

extern int					gHeadIdleTimer;
extern int					gHeadMoveTimeout;
extern int					gHeadOwnerTimer;
extern int					gDriveControlOwnerTimer;
extern int					gHeadCurrentOwner;
extern BOOL					gWaitingForLaserSingleLineScan;	// Indicates a Laser Scan line has been requested, and HeadBehavior waiting for it to complete
//extern BOOL					gWaitingForLaserMultiScan;	// Indicates that a multi line Laser Scan has been requested, and HeadBehavior waiting for it to complete

extern BOOL					gEnableIdleMovements;	// Indicates that idle movements can be enabled with flags below
extern BOOL					gHeadIdle;				// Indicates that head can execute idle behaviors
extern BOOL					gArmIdleLeft;			// Indicates that arm can execute idle behaviors
extern BOOL					gArmIdleRight;			// Indicates that arm can execute idle behaviors
extern BOOL					gArmInHomePositionLeft;	// Arm currently in home position
extern BOOL					gArmInHomePositionRight;// Arm currently in home position

extern BOOL					gKerrControlInitialized; // When true, indicates Kerr control has completed calibration of shoulder motors

extern BOOL					g_ColorBlobDetected;	// Indicates that Color Blob has been found and camera servo is tracking it

extern CString				g_FaceCaptureName;
extern HANDLE				g_hCameraRequestEvent;
extern LPCTSTR				g_pCameraRequestSharedMemory;
extern HANDLE				g_hCameraUpdateEvent;			// Synchronization wtih the Camera OpenCV app, when camera app has new info
extern CString				g_ObjectName;					// Text name of object found by Camera

extern HANDLE				g_hKobukiCommandEvent;
extern LPCTSTR				g_pKobukiCommandSharedMemory;
extern HANDLE				g_hKobukiDataEvent;
extern LPCTSTR				g_pKobukiDataSharedMemory;


extern SERVER_SOCKET_STRUCT g_ServerSockStruct;
extern CString				g_ScriptToRun;			// For sending the name of a script from GUI to Behavior Module


// ROBOTCLIENT globals
extern DWORD				g_dwClientSendThreadId;	// For sending messages to the client Socket thread
extern BOOL					g_bConnectedToServer;
extern DWORD				g_LastPingTime;
extern CString				g_ClientTextToSend;
extern char					g_ClientBulkData[BULK_DATA_SIZE];
extern int					g_ClientBulkDataLength;	// Current length of data in buffer

extern std::queue<CString>	g_SpeakQueue;
extern CRITICAL_SECTION		g_csSpeakQueue;			// Initialized in Robot.cpp 
extern BOOL					g_MoveArmsWhileSpeaking;
extern BOOL					g_CurrentlySpeaking;	// Indicate to other threads if robot is talking
extern BOOL					g_PhraseDoneTokenProcessed; // Threads can queue a token, so they know when speaking has reached a certain point



extern CLIENT_SOCKET_STRUCT	g_ClientSockStruct;

// endif

//extern BOOL					g_IRDA_Socket;
extern HANDLE				g_hCameraCommThread;
extern BOOL					g_DownloadingFirmware;
extern BOOL					g_PicFirstStatusReceived;

extern int					g_LastKey;				// For manual control in Cmd or Map view
extern int					g_SpeedSetByKeyboard;
extern int					g_LastSpeedSetByKeyboard;

extern int					g_MotorCurrentSpeedCmd;
extern int					g_MotorCurrentTurnCmd;
extern int 					g_GlobalMaxAvoidObjectDetectionFeet;	// Max range of objects to detect for Avoidance behavior
extern int 					g_SegmentAvoidObjectRangeTenthInches;	// Max range for Avoidance behavior, for CURRENT SEGMENT!

extern BOOL					g_MotorKludgeRevL; 
extern BOOL					g_MotorKludgeRevR; 
	
extern CString				g_CurrentUserName;				// Person Loki is talking to
extern CString				g_StatusMessagesToDisplay;
extern CString				g_StatusMessagesToSend;
extern char					g_BulkSensorData[MAX_SENSORS][BULK_DATA_SIZE];	// for RAW sensor data
extern int 					g_ScaledSensorData[MAX_SENSORS][BULK_DATA_SIZE];	// Structure to hold processed sensor data
extern BOOL					g_bResetWatchdog;		// If true, resets Watchdog with each USB status request
extern CRect				g_CameraWindowRect;		// location of the camera window on the GUI

//extern int					g_CameraPanPos;			// Track Camera state and make available to all modules
//extern int					g_CameraTiltPos;
//extern int					g_CameraSideTiltPos;
//extern int					g_CameraNeckPos;
extern int					g_CameraZoomPos;
extern BOOL					g_CameraServoTorqueEnabled;


extern BOOL					g_bCmdRecognized;		// Global to all modules

extern BOOL					g_MotorControlDebug;	// debug ERI motor control - dump commands

extern CRITICAL_SECTION		g_csDisplayLock;
extern int					g_ConnectionMonitorTimer;
extern CSegmentStructList*	g_pSegmentList;		// Share the current PATH with other modules that need access to it!
extern CWaypointStructList* g_pWaypointList;
extern CGridMap*			g_pGridMap;

//extern ObjectKnowledge*	g_pObjectKnowledge;		// Database of Object info

extern GPS_MESSAGE_T*		g_pGPSData;

// Degrees from Forward of each sensor
// TODO! IR2,IR3 and US0 - Depends upon where the head is facing!  ASSUME forward for now!
//extern int SensorOffsetDegrees_IR[NUM_IR_SENSORS];
//extern int SensorOffsetDegrees_US[NUM_US_SENSORS];


////////////////////////////////////////////////////////////////////////////////
// Global Functions

DWORD WINAPI DeviceThreadProc( LPVOID lpParameter );
DWORD WINAPI TimerThreadProc( LPVOID lpParameter );


//-----------------------------------------------------------------------------
// Name: CalculateTurn
// Desc: Given current heading and desired heading, returns amount in degrees
//       to turn.  Positive is right turn, Negative is Left turn
//-----------------------------------------------------------------------------
int CalculateTurn( int CurrentHeading, int DesiredHeading);

//-----------------------------------------------------------------------------
// Name: DegreesToCompassRoseString
// Desc: Given current or desired heading in degrees, convert to "Compass Rose" points
//       returns a string
//-----------------------------------------------------------------------------
char* DegreesToCompassRoseString( int Degrees );

//-----------------------------------------------------------------------------
// Name: CompassRoseToDegrees
// Desc: Given "COMPASS_ROSE" points (eg. "NW"), convert to degrees
// Returns 0-360, or -1 if error
//-----------------------------------------------------------------------------
int CompassRoseToDegrees( int nCompassRose );

//-----------------------------------------------------------------------------
// Name: CompassRoseToString
// Desc: Given "Compass Rose" points (eg. NORTH), convert to string "north"
//-----------------------------------------------------------------------------
char* CompassRoseToString( int nCompassRose );

//-----------------------------------------------------------------------------
// Name: SendCommand
// Desc: For Client, sends command to Socket
//		 For Server, sends command to USB device
//-----------------------------------------------------------------------------
void SendCommand( DWORD Cmd, DWORD Param1, DWORD Param2 );

//-----------------------------------------------------------------------------
// Name: g_PostStatus 
// Desc: Puts status message in global memory, then posts message to have dialog display it
//       We do it this way, so threads don't get blocked waiting to display on the GUI
//-----------------------------------------------------------------------------
void g_PostStatus( LPCTSTR lpszStatus, char *FunctionName = 0, BOOL bDisplayOnGui = TRUE, BOOL bRemote = FALSE );


//-----------------------------------------------------------------------------
// Name: SendCommand
// Desc: For Client, sends command to Socket
//		 For Server, sends command to Arduino device
void SendCommand( DWORD Cmd, DWORD Param1, DWORD Param2 );

//-----------------------------------------------------------------------------
// Name: GPSDegreesToRWTenthInches
// Desc: Given Latitude and Longitude, calculates X,Y of current 
//       Real World map position (in inches)
//-----------------------------------------------------------------------------
POINT GPSDegreesToRWTenthInches( double Lat, double Long );

//-----------------------------------------------------------------------------
// Name: ModuleNumberToName
// Desc: Given a module number, return a string with the module name for display
//-----------------------------------------------------------------------------
void ModuleNumberToName(int  Module, CString &ModuleString);

//-----------------------------------------------------------------------------
// Name: HeadOwnerNumberToName
// Desc: Given a Head Owner number, return a string with the Owner name for display
//		 This function used by client and server
//-----------------------------------------------------------------------------
void HeadOwnerNumberToName(int  nOwner, CString &strOwner);

//-----------------------------------------------------------------------------
// Name: KinectOwnerNumberToName
// Desc: Given a Kinect Tilt Owner number, return a string with the Owner name for display
//		 This function used by client and server
//-----------------------------------------------------------------------------
void KinectOwnerNumberToName(int  nOwner, CString &strOwner);

//-----------------------------------------------------------------------------
// Name: FPointToPoint
// Desc: Converts high precision FPOINT (double POINTs) to POINT
//-----------------------------------------------------------------------------
POINT FPointToPoint( FPOINT From);

//-----------------------------------------------------------------------------
// Name: CalculateAngle
// Desc: Given 2 points, returns direction angle
//-----------------------------------------------------------------------------
int  CalculateAngle( FPOINT From, FPOINT To );	// For High Precision double Points
int  CalculateAngle( FPOINT From,  POINT To );
int  CalculateAngle(  POINT From, FPOINT To );
int  CalculateAngle(  POINT From,  POINT To );
double CalculateAnglePlusMinus( FPOINT From, FPOINT To );	// positive or negative from current heading




//-----------------------------------------------------------------------------
// Name: CalculateDistance
// Desc: Given 2 points, returns distance between them
//-----------------------------------------------------------------------------
int  CalculateDistance( FPOINT From, POINT To);	// For High Precision double Points
int  CalculateDistance( POINT From, POINT To);

//-----------------------------------------------------------------------------
// Name: ConvertPolarToRectangular
// Desc: Given starting point, angle, and distance, find new point
//-----------------------------------------------------------------------------
POINT ConvertPolarToRectangular( POINT Origin, double AngleDegrees, double Distance );


// ROBOT_SERVER Functions

//-----------------------------------------------------------------------------
// Name: OpenCommPort
//		 For Server. Opens COMM port to Arduino
HANDLE OpenCommPort(LPCTSTR strPort, DWORD nBaudRate, DWORD Device);

//-----------------------------------------------------------------------------
// Name: CloseCommPort
//		 For Server. Closes COMM port to Arduino
void CloseCommPort( LPCTSTR strPort, HANDLE &hPort );

//-----------------------------------------------------------------------------
// Name: SendResponse
//		 For Server, sends command to Client
void SendResponse( DWORD Cmd, DWORD Param1, DWORD Param2 );

// endif	// ROBOT_SERVER Functions
//////////////////////////////////////////////////////////////////////




#endif  //__ROBOT_GLOBALS_H__