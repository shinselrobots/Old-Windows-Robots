#include "stdafx.h"
//#include <vld.h>	// Visual Leak Detector utility!
#include "resource.h"
///#include "Joystick.h"
#include "ClientOrServer.h"
#include "Globals.h"
#include "thread.h"
#include <math.h>

//#include "cv.h"
//#include "cvcam.h"
//#include "highgui.h"
#include <mmsystem.h>	// for timeGetTime

#if ( ROBOT_SERVER == 1 )	// These modules used for Robot Server only

#include "HardwareConfig.h"
#include "headcontrol.h"
#include "armcontrol.h"
#include "..\Common\WiiMoteCommon.h"
#include "WiiControl.h"
#include "CameraCommon.h"
#include "KobukiCommon.h"


#include <vector>
#include <algorithm>

#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Auto launch KobukiControl Application
#if (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL)
	#define ENABLE_KOBUKI_APP			1	// set value to 1 to allow app communication
	#define AUTO_LAUNCH_KOBUKI_APP		1	// if 0, start this process first, then start Kobuki
	#if (ENABLE_KOBUKI_APP == 1)
		FIND_WINDOW_HANDLE_STRUCT_T KobukiFWHS; // global to this file
	#endif
#endif

// Auto launch Camera OpenCV Application
#define ENABLE_CAMERA_APP			1	// set value to 1 to allow app communication
#define AUTO_LAUNCH_CAMERA_APP		1	// set value to 1 to auto launch
#if (ENABLE_CAMERA_APP == 1)
	FIND_WINDOW_HANDLE_STRUCT_T CameraFWHS; // global to this file
#endif


//#include <windows.h>
//#include <stdio.h>
//#include <conio.h>
//#include <tchar.h>
//#pragma comment(lib, "user32.lib")


// Domains for ITT instrumentation
// Make sure to declare in Globals.h!
__itt_domain* pDomainGlobalThread = __itt_domain_create("Loki.Global.Thread");
__itt_domain* pDomainControlThread = __itt_domain_create("Loki.Control.Thread");
__itt_domain* pDomainVidCapThread = __itt_domain_create("Loki.VidCap.Thread");
__itt_domain* pDomainArduinoThread = __itt_domain_create("Loki.Arduino.Thread");
__itt_domain* pDomainGPSThread = __itt_domain_create("Loki.GPS.Thread");
__itt_domain* pDomainPololuServoThread = __itt_domain_create("Loki.PololuServo.Thread");
__itt_domain* pDomainMotorThread = __itt_domain_create("Loki.Motor.Thread");
__itt_domain* pDomainSmartServoThread = __itt_domain_create("Loki.DynaServo.Thread");
__itt_domain* pDomainLaserThread = __itt_domain_create("Loki.Laser.Thread");
__itt_domain* pDomainKinectThread = __itt_domain_create("Loki.Kinect.Thread");
__itt_domain* pDomainSocketThread = __itt_domain_create("Loki.Socket.Thread");
__itt_domain* pDomainGUIThread = __itt_domain_create("Loki.GUI.Thread");
__itt_domain* pDomainSpeakThread = __itt_domain_create("Loki.Speak.Thread");
__itt_domain* pDomainSpeechRecoThread = __itt_domain_create("Loki.SpeechReco.Thread");
__itt_domain* pDomainModuleThread = __itt_domain_create("Loki.Module.Thread");
__itt_domain* pDomainKinectAppSharedMemoryIPCThread = __itt_domain_create("Loki.SharedMemory.Thread");

// Common strings used by multiple modules
//__itt_string_handle* pshCriticalSection = __itt_string_handle_create("CS Lock");

__itt_string_handle* psh_csDisplayLock = __itt_string_handle_create("CS DisplayLock");
__itt_string_handle* psh_csServoLock = __itt_string_handle_create("CS ServoLock");
__itt_string_handle* psh_csKinectPointCloudLock = __itt_string_handle_create("CS PtCloudLock");
__itt_string_handle* psh_csLaserSummaryDataLock = __itt_string_handle_create("CS LaserSummaryDataLock");
__itt_string_handle* psh_csLaserDataLock = __itt_string_handle_create("CS LaserDataLock");
__itt_string_handle* psh_csKinectHumanTrackingLock = __itt_string_handle_create("CS KinectHumanTrackingLock");
__itt_string_handle* psh_csKinectSummaryDataLock = __itt_string_handle_create("CS KinectSummaryDataLock");
__itt_string_handle* psh_Sleep = __itt_string_handle_create("Sleep");

/* examples:
	__itt_marker(pDomainGlobalThread, __itt_null, psh_Sleep, __itt_marker_scope_task);
	__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, psh_Sleep);
	__itt_task_end(pDomainGlobalThread);
*/

#define LOOP_TIMES_PER_SECOND		10		// 100ms - all timers must be evenly divisible by this!

//#define DEBUG_TIMER_LOOP

// Global Variables
// Note: values initialized set here or in Robot.cpp

////////////////////////////////////////////////////////////////
//                    HARDWARE CONFIG
//	        HW Config is selected in HardwareCmds.h
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

	#define SENSOR_SAMPLE_RATE		(LOOP_TIMES_PER_SECOND/5)	// 200 ms!


////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE_IROBOT
	#define SENSOR_SAMPLE_RATE		(LOOP_TIMES_PER_SECOND/5)	// 200 ms!


////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP_KOBUKI
	#define SENSOR_SAMPLE_RATE		(LOOP_TIMES_PER_SECOND/5)	// 200 ms!


////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_KOBUKI_WITH_ARDUINO
	#define SENSOR_SAMPLE_RATE		(LOOP_TIMES_PER_SECOND/5)	// 200 ms!


////////////////////////////////////////////////////////////////
#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

	#define SENSOR_SAMPLE_RATE		(LOOP_TIMES_PER_SECOND/3)	// 6 times per second for Carbot?  with 6 US, try 3

////////////////////////////////////////////////////////////////
#else
	#error BAD SENSOR_CONFIG_TYPE!  
#endif


//-----------------------------------------------------------------------------

HWND				g_RobotMainFrameHWND   = NULL;		// Handle to main frame (joystick only)
HWND				g_RobotCmdViewHWND   = NULL;		// for posting messages to main window
HWND				g_RobotSetupViewHWND = NULL;	
HWND				g_RobotPathViewHWND  = NULL;	
HWND				g_RobotMapViewHWND   = NULL;	
FILE*				g_LogFile = NULL;					// Log file for non-debug mode
BOOL				g_CriticalSectionsInitialized = FALSE; // Flag when CS are valid


BOOL				g_bRunThread = TRUE;				// When FALSE, tells all threads to exit
BOOL				g_bRunVidCapThread = FALSE;			// When FALSE, tells Vidcap thread to exit
BOOL				g_bRunKinectThread = FALSE;			// When FALSE, tells Vidcap thread to exit

// Global Pause and Power Control - Allows pausing or powering on/off subsystems instantly
BOOL				g_SleepMode = FALSE;				// Power on by default, unless in lowpower "sleep mode"
BOOL				g_DynaPowerEnabled = FALSE;
BOOL				g_KinectPowerEnabled = TRUE;
BOOL				g_GlobalPause = FALSE;				// freeze all servos and motors until unpaused

//maker
BOOL	g_StopBehavior = FALSE;

HANDLE				g_hSpeechRecoEvent = NULL;			// Synchronization between C# app and C++ for Speech recognition
HANDLE				g_hKinectDepthReadyEvent = NULL;	// Synchronization between C# app and C++ for when depth data is ready
BOOL				g_SpeechRecoBlocked = FALSE;		// When enabled, blocks all speech recognition by the C++ code, including "Stop". 
														// Used when arm motors are moving due to noise picked by Kinect



// ROBOT_SERVER globals
// Thread Handles
HANDLE				g_hControlThread = INVALID_HANDLE_VALUE;
HANDLE				g_hSoundThread = INVALID_HANDLE_VALUE;
HANDLE				g_hSpeakThread = INVALID_HANDLE_VALUE;
HANDLE				g_hCameraVidCapThread = INVALID_HANDLE_VALUE;
HANDLE				g_hKinectThread = INVALID_HANDLE_VALUE;
HANDLE				g_hTimerThread = INVALID_HANDLE_VALUE;
HANDLE				g_hSmartServoThread = INVALID_HANDLE_VALUE;
HANDLE				g_hServoThread = INVALID_HANDLE_VALUE;
HANDLE				g_hArduinoWriteThread = INVALID_HANDLE_VALUE;
HANDLE				g_hArduinoReadThread = INVALID_HANDLE_VALUE;
HANDLE				g_hCameraThread = INVALID_HANDLE_VALUE;
HANDLE				g_hMotorWriteThread = INVALID_HANDLE_VALUE;
HANDLE				g_hMotorReadThread = INVALID_HANDLE_VALUE;
//HANDLE				g_hTrexWriteThread = INVALID_HANDLE_VALUE;
HANDLE				g_hLaserScannerReadThread = INVALID_HANDLE_VALUE;
HANDLE				g_hLaserScannerWriteThread = INVALID_HANDLE_VALUE;
HANDLE				g_hGPSThread = INVALID_HANDLE_VALUE;
HANDLE				g_hiRobotReadThread = INVALID_HANDLE_VALUE;	// Read thread for iRobot Base.  The write thread is g_dwMotorCommThreadId
HANDLE				g_hKinectNuiThread = INVALID_HANDLE_VALUE;
//HANDLE				g_hEvNuiProcessStop = INVALID_HANDLE_VALUE;
HANDLE				g_hKinectAppSharedMemoryIPCThread = INVALID_HANDLE_VALUE;
HANDLE				g_hCameraAppSharedMemoryIPCThread = INVALID_HANDLE_VALUE;
HANDLE				g_hKobukiAppSharedMemoryIPCThread = INVALID_HANDLE_VALUE;

DWORD				g_dwControlThreadId = 0;			// Control Thread for communicating to hardware
DWORD				g_dwSoundThreadId = 0;				// Thread for playing sounds
DWORD				g_dwSpeakThreadId = 0;				// Thread for speech output
DWORD				g_dwCameraVidCapThreadId = 0;		// Thread for visual object recognition
DWORD				g_dwServerSendThreadId =0;			// For sending messages to the client Socket thread
DWORD				g_dwKinectAppSharedMemoryIPCThreadId = 0;	// Control Thread for communicating to hardware
DWORD				g_dwCameraAppSharedMemoryIPCThreadId = 0;	// Control Thread for communicating to hardware
DWORD				g_dwKobukiAppSharedMemoryIPCThreadId = 0;	// Control Thread for communicating to hardware


// System Status
/*
		SUBSYSTEM_DISABLED = 0,	// Yellow
		SUBSYSTEM_WAITING,		// Yellow
		SUBSYSTEM_CONNECTED,	// Green
		SUBSYSTEM_FAILED		// Red

*/
SUBSYSTEM_STATUS	g_ArduinoSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToPIC = FALSE;		// If false, indicates Arduino not found yet
SUBSYSTEM_STATUS	g_DynaSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToDyna = FALSE;		// If false, indicates Dyna controller not found yet
SUBSYSTEM_STATUS	g_RX64SubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToRX64 = FALSE;		// If false, indicates Dyna RX64 controller not found yet
SUBSYSTEM_STATUS	g_KerrSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToKerr = FALSE;		// If false, indicates Kerro controller not found yet
SUBSYSTEM_STATUS	g_MotorSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToMotor = FALSE;		// If false, indicates Motor controller not found yet
SUBSYSTEM_STATUS	g_GPSSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToGPSDevice = FALSE;	// If false, indicates GPS device not connected yet

SUBSYSTEM_STATUS	g_KinectSubSystemStatus = SUBSYSTEM_WAITING;   //g_KinectReady = FALSE;			// If false, indicates Kinect not ready
SUBSYSTEM_STATUS	g_CameraSubSystemStatus = SUBSYSTEM_WAITING;   //g_CameraReady = FALSE;	
SUBSYSTEM_STATUS	g_LaserSubSystemStatus = SUBSYSTEM_WAITING;		//g_LaserReady = FALSE;	
SUBSYSTEM_STATUS	g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING;   //g_ArmReady_L = FALSE;	
SUBSYSTEM_STATUS	g_RightArmSubSystemStatus = SUBSYSTEM_WAITING;	//g_ArmReady_R = FALSE;	


// for Arduino serial communication
HANDLE				g_hArduinoCommPort = INVALID_HANDLE_VALUE;
DWORD				g_dwArduinoCommWriteThreadId = 0;				// Comm Thread for sending to Arduino

// For External Servo Controller
HANDLE				g_hServoCommPort = INVALID_HANDLE_VALUE;
DWORD				g_dwServoCommWriteThreadId = 0;				// Comm Thread for sending to Servo Controller

// For Sony Camera
HANDLE				g_hCameraCommPort = INVALID_HANDLE_VALUE;
DWORD				g_dwCameraCommWriteThreadId = 0;			// Comm Thread for sending to Sony Camera

// For Cameras and Kinect
void*				g_pCameraModule = 0;						// make pointer available to Camera Video Thread
void*				g_pKinectModule = 0;						// make pointer available to Kinect Video Thread
void*				g_pKinectNui = 0;
DWORD				g_LastCameraMoveTime = GetTickCount();		// indicate when to begin IDLE behavior

// For ER1 Controller
HANDLE				g_hMotorCommPort = INVALID_HANDLE_VALUE;
int					g_nMotorStatusTimer = 0;
DWORD				g_dwMotorCommThreadId = 0;					// Comm Thread for sending to ER1 or Pololu TReX Motor Controller

// For iRobot Create Base
IROBOT_STATUS_T*	g_pIRobotStatus = NULL;						// Status data updated every 15ms

// For Kobuki Base
KOBUKI_STATUS_T*	g_pKobukiStatus  = NULL;			// Status data updated frequently


// For Dynamixel and Kerr Servos
CRITICAL_SECTION	g_csServoLock;								// Initialized in Robot.cpp 
DWORD				g_dwSmartServoCommThreadId = 0;				// Comm Thread for sending to Dynamixel and Kerr Servos
HANDLE				g_hDynaServoCommPort_AX12 = INVALID_HANDLE_VALUE;
HANDLE				g_hDynaServoCommPort_RX64 = INVALID_HANDLE_VALUE;
HANDLE				g_hKerrServoCommPort = INVALID_HANDLE_VALUE;

int 				g_nServoStatusTimer = 0;
int 				g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_NORMAL;
int					gServoOverheatError = 0;					//  SERVO NUMBER THAT IS OVERHEATING. IF NONZERO, DISABLES ALL ARM MOVEMENTS to prevent servo damage

// For Laser Scanner
CRITICAL_SECTION	g_csLaserSummaryDataLock;					// Initialized in Robot.cpp 
CRITICAL_SECTION	g_csLaserDataLock;							// Initialized in Robot.cpp 
HANDLE				g_hLaserScannerCommPort = 0;

DWORD				g_dwLaserScannerCommWriteThreadId = 0;		// Comm Thread for sending to Hokuyo URG-04LX-UG01 Laser Scanner
//DWORD				g_dwLaserScannerCommReadThreadId;			// Comm Thread for reading data from the Laser Scanner
BOOL				g_bLaserScanEnabled = FALSE;				// When TRUE, short term laser scan is running.  get frequent servo updates
BOOL				g_bLaserContinuousScanEnabled = TRUE;		// When TRUE, laser runs continuously
int 				g_LaserScansRemaining = 0;					// Number of scans remaining when laser scanner is in multi scan mode

LASER_SCANNER_STATE_T g_LaserScannerState;
LASER_SCANNER_DATA_T* g_pLaserScannerData = NULL;

// For Kinect
CRITICAL_SECTION	g_csKinectSummaryDataLock;					// Initialized in Robot.cpp 
CRITICAL_SECTION	g_csKinectPointCloudLock;					// Initialized in Robot.cpp 
CRITICAL_SECTION	g_csKinectHumanTrackingLock;				// Initialized in Robot.cpp 

KINECT_3D_CLOUD_T*	g_KinectPointCloud = NULL;					// Array of 3D points detected by Kinect in TenthInches
OBJECT_2D_ARRAY_T*	g_pKinectObjects2D = NULL;					// Array of 2D object slices from one scan line
OBJECT_3D_ARRAY_T*	g_pKinectObjects3D = NULL;					// Array of 3D complete objects detected in the complete capture
KINECT_HUMAN_TRACKING_T g_HumanLocationTracking[KINECT_MAX_HUMANS_TO_TRACK]; // Location of humans being tracked
int					g_CurrentHumanTracked = 0;					// Current player number of the human currently being tracked (zero if none)
int					g_LastHumanCompassDirection = -1;			// Direction last human was at
float				g_LastHumanAudioBeamDirection = -1;

//#if (DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1)
//	DEBUG_SLICE_ARRAY_T* g_pKinectDebugSliceArray = NULL;				// temp array for debugging slices
//#endif

// Status and Command blocks for all Dyna Servos and Kerr Motors
BULK_SERVO_STATUS_T	g_BulkServoStatus[NUMBER_OF_SMART_SERVOS+1];	// +1 because servo numbers start at 1, not 0!
BULK_SERVO_CMD_T	g_BulkServoCmd[NUMBER_OF_SMART_SERVOS+1];		// (0 is "wasted space, but avoids common index error)

// Camera Capture
CAMERA_T			g_Camera[4];	// State and Capture structure for each camera attached (LEFT_CAMERA or RIGHT_CAMERA)


// For GPS device
HANDLE				g_hGPSCommPort = INVALID_HANDLE_VALUE;
BOOL				g_GPSGotFirstFix = FALSE;		// If false, indicates no GPS Lock yet
double				gGPSOriginLat = -1.0;			// Latitude of the Map Origin
double				gGPSOriginLong = -.0;			// Longitue of the Map Origin

//Other stuff
BOOL				g_bConnectedToClient = FALSE;
int					g_nClientKeepAliveCount = 0;
int					gStartTime = 0;				// Used to determine time since Robot started.  Warning, only good for 45 days! :-)
int					gCurrentTime = 0;			// Convenient tenth second count-up timer
int					gPicWatchDogTimer = 0;
int					gKeyPressTimer = 0;
int					gLocalUserCmdTimer = 0;
int					gRemoteUserCmdTimer = 0;
int					gMotorSpeedTimer = 0;
int					gBrakeTimer = 0;
int					gCollisionTimer = 0;
int					gAvoidanceTimer = 0;
int					gNavigationTimer = 0;
int					gServoOverHeatTimer = 0;
int					gArmTimerRight = 0;
int					gArmTimerLeft = 0;
int					gBehaviorTimer = 0;
int					gHeadNodTimer = 0;

int					gKinectMoveTimeout = 0;
int					gKinectDelayTimer = 0;
int					gKinectOwnerTimer = 0;
int					gKinectCurrentOwner = KINECT_TILT_OWNER_NONE;

int					gHeadIdleTimer = 0;
int					gHeadMoveTimeout = 0;
int					gHeadOwnerTimer = 0;
int					gDriveControlOwnerTimer = 0;
int					gHeadCurrentOwner = HEAD_OWNER_NONE;

BOOL				gWaitingForLaserSingleLineScan = FALSE;	// Indicates a Laser Scan line has been requested, and HeadBehavior waiting for it to complete
//BOOL				gWaitingForLaserMultiScan = FALSE;		// Indicates that a multi line Laser Scan has been requested, and HeadBehavior waiting for it to complete

BOOL				gEnableIdleMovements = TRUE;		// Indicates that idle movements can be enabled with flags below
BOOL				gHeadIdle = TRUE;					// Indicates that head can execute idle behaviors
BOOL				gArmIdleLeft = TRUE;				// Indicates that arm can execute idle behaviors
BOOL				gArmIdleRight = TRUE;				// Indicates that arm can execute idle behaviors
BOOL				gArmInHomePositionLeft = TRUE;		// Arm currently in home position
BOOL				gArmInHomePositionRight = TRUE;		// Arm currently in home position

BOOL				gKerrControlInitialized = FALSE;	// When true, indicates Kerr control has completed calibration of shoulder motors
BOOL				g_ColorBlobDetected = FALSE;		// Indicates that Color Blob has been found and camera servo is tracking it
CString				g_FaceCaptureName;
HANDLE				g_hCameraRequestEvent = NULL;
LPCTSTR				g_pCameraRequestSharedMemory = NULL;
HANDLE				g_hCameraUpdateEvent;				// Synchronization wtih the Camera OpenCV app, when camera app has new info
CString				g_ObjectName;						// Text name of object found by Camera

HANDLE				g_hKobukiCommandEvent = NULL;
LPCTSTR				g_pKobukiCommandSharedMemory = NULL;
HANDLE				g_hKobukiDataEvent;
LPCTSTR				g_pKobukiDataSharedMemory;



SERVER_SOCKET_STRUCT g_ServerSockStruct;
CString				g_ScriptToRun;


// ROBOTCLIENT globals
DWORD				g_dwClientSendThreadId = 0;	// For sending messages to the client Socket thread
BOOL				g_bConnectedToServer = FALSE;
DWORD				g_LastPingTime = 0;
CString				g_ClientTextToSend;
char				g_ClientBulkData[BULK_DATA_SIZE];
int					g_ClientBulkDataLength;	// Current length of data in buffer
CLIENT_SOCKET_STRUCT	g_ClientSockStruct;		// Initialized in ?
// endif

std::queue<CString>	g_SpeakQueue;
CRITICAL_SECTION	g_csSpeakQueue;			// Initialized in Robot.cpp
BOOL				g_MoveArmsWhileSpeaking;
BOOL				g_CurrentlySpeaking;	// Indicate to other threads if robot is talking
BOOL				g_PhraseDoneTokenProcessed; // Threads can queue a token, so they know when speaking has reached a certain point


ARDUINO_STATUS_T	g_RawArduinoStatus;				// Initialized in ?
KOBUKI_STATUS_T		g_RawKobukiStatus;				// Initialized in ?

BOOL				g_IRDA_Socket = FALSE;
FullSensorStatus*	g_pFullSensorStatus;			// Initialized in ?		// Current status of all sensors
HANDLE				g_hCameraCommThread = INVALID_HANDLE_VALUE;
BOOL				g_DownloadingFirmware = FALSE;
BOOL				g_PicFirstStatusReceived = FALSE;

BOOL				g_GUILocalUser = FALSE;	// indicate if the GUI is running locally or remote.  Used by Cmd and Map views.
int					g_GUICurrentSpeed = 0;	//  Keep Cmd and Map views in sync
int					g_GUICurrentTurn = 0;

int					g_LastKey = 0;		// For manual control in Cmd or Map view
int					g_SpeedSetByKeyboard = SPEED_FWD_MED_SLOW;
int					g_LastSpeedSetByKeyboard = SPEED_FWD_MED_SLOW;
//int					g_MotorCurrentSpeedCmd = 0;
//int					g_MotorCurrentTurnCmd = 0;
int 				g_GlobalMaxAvoidObjectDetectionFeet = OBJECT_AVOID_DEFAULT_FEET;	// Max range of objects to detect for Avoidance behavior
int 				g_SegmentAvoidObjectRangeTenthInches = 0xFFFFF;	// Max range for Avoidance behavior, for CURRENT SEGMENT!

BOOL				g_MotorKludgeRevL = FALSE; 
BOOL				g_MotorKludgeRevR = FALSE; 

CString				g_CurrentUserName;				// Person Loki is talking to
CString				g_StatusMessagesToDisplay;
CString				g_StatusMessagesToSend;
char				g_BulkSensorData[MAX_SENSORS][BULK_DATA_SIZE];		// Initialized in ? // for RAW sensor data
int 				g_ScaledSensorData[MAX_SENSORS][BULK_DATA_SIZE];	// Initialized in ? // Structure to hold processed sensor data
BOOL				g_bResetWatchdog = TRUE;			// By default, reset EZ-USB Watchdog with each status request
CRect				g_CameraWindowRect;		// location of the camera window on the GUI

int					g_CameraZoomPos	= 0;
BOOL				g_CameraServoTorqueEnabled = FALSE;
BOOL				g_bCmdRecognized = FALSE;		// Global to all Modules

BOOL				g_MotorControlDebug = FALSE;	// debug ERI motor control - dump commands

CRITICAL_SECTION	g_csDisplayLock;		// Initialized in Robot.cpp 
int					g_ConnectionMonitorTimer = 0;
CSegmentStructList*	 g_pSegmentList = NULL;		// Initialized in ? // Share the current PATH with other modules that need access to it!
CWaypointStructList* g_pWaypointList = NULL;		// Initialized in ? 
CGridMap*			g_pGridMap;				// Initialized in ? 

//ObjectKnowledge*	g_pObjectKnowledge;		// Database of Object info


// GPS Structure
GPS_MESSAGE_T*		g_pGPSData;				// Initialized in ? 

NavSensorSummary*	g_pNavSensorSummary;	// created in Robot.cpp and initialized in DoSensorFusion().  Values in TENTH INCHES!
SCANNER_SUMMARY_T*	g_pLaserSummary;		// created in Robot.cpp and initialized in DoSensorFusion()
SCANNER_SUMMARY_T*	g_pKinectSummary;		// created in Robot.cpp and initialized in DoSensorFusion()


// Global Utility Functions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Name: TimerThreadProc
// Desc: dedicated thread for timing functions
//-----------------------------------------------------------------------------
//#define BUF_SIZE 256
TCHAR szName[]=TEXT("Global\\MyFileMappingObject");

__itt_string_handle* pshGlobalThreadLoop = __itt_string_handle_create("GlobalThreadLoop");
DWORD WINAPI TimerThreadProc( LPVOID NotUsed )
{
	IGNORE_UNUSED_PARAM (NotUsed);
	__itt_thread_set_name( "Global Timer Thread" );

	// Fairly precise timer, not subject to the windows message posting
	// issues that Windows Timers have

	int		nTenthSecondTimer = 0;	// fires 10 times per second
	int		nOneSecondTimer = 0;	// fires once per second
	int		nStatusTimer = 0;
	int		nServoUpdateTimer = 3;	// offset, so servo updates finish when Arduino update begins
	int		nLaserScanTimer = 0;
	int		nMotorCmdTimer = 0;
	int		nSendStatusCount = 0;
	int		nSendKeepAliveCount = 0;
	int		nMaxClientKeepAliveCount = 0;
	int		nLastSpeedCmd = 0;
	int		nLastTurnCmd = 0;
	BOOL	AndroidHasMotorControl = FALSE;


#if ( ROBOT_SERVER == 1 )
	//WiiControl		*pWiiControl = new WiiControl;				// Mapped file IPC with Wii Control app
	int  Message;	// Used by Server code
#endif	

	gStartTime = GetTickCount();	// Keep track of time since system startup
//	HeadControl	*m_pHeadControl = new HeadControl();	// For controlling head servos

	// Initialize Wii Control and open shared memory
#if ( ROBOT_SERVER == 1 )
	//pWiiControl->Initialize();
#endif
/*	unsigned long RunTime = GetTickCount() - gStartTime;
	unsigned long Delta = RunTime - LastReportTime;
	LastReportTime = RunTime;
    unsigned int days = RunTime / (24 * 60 * 60 * 1000);
    RunTime %= (24 * 60 * 60 * 1000);
    unsigned int hours = RunTime / (60 * 60 * 1000);
    RunTime %= (60 * 60 * 1000);
    unsigned int minutes = RunTime / (60 * 1000);
    RunTime %= (60 * 1000);
    unsigned int seconds = RunTime / (1000);
    RunTime %= (1000);
    unsigned int miliseconds = RunTime;

	ROBOT_LOG( TRUE,"GLOBAL TIMER STARTING: %4d delta, %d days, %d hours, %d minutes, %d seconds, %d miliseconds\n",
		Delta, days, hours, minutes, seconds, miliseconds )
*/

	RobotSleep(100, pDomainGlobalThread); // Allow other initialization to complete


	// DEBUG TEST



//	TRACE( "\n\n\nRANDOM NUMBER TEST \n\n");

/*
	//int RandomNumber = ((4 * rand()) / RAND_MAX);
#define SORT_RAND_MIN 0
#define SORT_RAND_MAX 16
	int MyArray[SORT_RAND_MAX+1];
	memset( MyArray, -1, SORT_RAND_MAX );

	int i;
	while( TRUE )
	{
		i = rand() / (RAND_MAX + 1) * SORT_RAND_MAX;
		TRACE( "Index:  %d\n", i);



	}




// Shuffle the questions once, so we go through them all
#define NUMBER_OF_QUESTIONS 16
	int data[NUMBER_OF_QUESTIONS];
	for( int q=0; q<NUMBER_OF_QUESTIONS; q++ )
	{
		data[q] = q;
	}

    int size   =  NUMBER_OF_QUESTIONS;

    std::random_shuffle(data, data + size);

    for(int loop = 0; loop < size; ++loop)
    {
        TRACE(" %d\n", data[loop]) ;
    }

*/
/*
	int data[] =  { 0,1,2,3,4,5,6,7,8,9,10,11};
    int size   =  sizeof(data)/sizeof(data[0]);

    std::random_shuffle(data, data + size);

    for(int loop = 0; loop < size; ++loop)
    {
        TRACE(" %d\n", data[loop]) ;
    }
*/

//	TRACE( "\n\nEND OF RANDOM NUMBER TEST \n\n\n");


	// Request Arduino Version first thing.  Optimized to send directly to the Arduino COMM Port Write Thread
#if ( ROBOT_SERVER == 1 )
	Message = WM_ROBOT_MESSAGE_BASE+HW_GET_VERSION;
	PostThreadMessage( g_dwArduinoCommWriteThreadId, Message, 0, 0 );
#endif



	//////////////////////////////////////////////////////////////////////////////////////////////
	while( g_bRunThread )
	{
		{
			__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, pshGlobalThreadLoop);
			///TAL_SCOPED_TASK_NAMED("Global Timer Loop");

			/*
			LoopStartTime = GetTickCount();

			unsigned long RunTime = GetTickCount() - gStartTime;
			unsigned long Delta = RunTime - LastReportTime;
			LastReportTime = RunTime;
			//unsigned int days = RunTime / (24 * 60 * 60 * 1000);
			RunTime %= (24 * 60 * 60 * 1000);
			//unsigned int hours = RunTime / (60 * 60 * 1000);
			RunTime %= (60 * 60 * 1000);
			unsigned int minutes = RunTime / (60 * 1000);
			RunTime %= (60 * 1000);
			unsigned int seconds = RunTime / (1000);
			RunTime %= (1000);
			unsigned int miliseconds = RunTime;
			*/

	#ifdef DEBUG_TIMER_LOOP
			ROBOT_LOG( TRUE,"TIMER LOOP: %4d delta, %d min, %d sec, %d ms\n",
				Delta, minutes, seconds, miliseconds )
	#endif

			// One Second Timer
			if( nOneSecondTimer++ >= (LOOP_TIMES_PER_SECOND) ) // 1 time per second
			{
				nOneSecondTimer = 0;
				// Once per second, post message to keep screen awake, if not in SleepMode
				if( !g_SleepMode )
				{
					SetThreadExecutionState(ES_DISPLAY_REQUIRED);
				}
			}

			// Limit number of motor command messages per second that can be sent from the GUI
/*** 
			if( nMotorCmdTimer++ >= (LOOP_TIMES_PER_SECOND/5) ) // Send 5 per second
			{
				///TAL_SCOPED_TASK_NAMED("WII Ctrl");

   				// Handle joystick input here, if desired
				// Replaced by Wii controller instead
				
#if ( ROBOT_SERVER == 1 )
				//pWiiControl->Update();
#endif
				// Now send update to the Robot Control module

				if( (g_MotorCurrentSpeedCmd != nLastSpeedCmd) ||
					(g_MotorCurrentTurnCmd != nLastTurnCmd) )
				{
					// GUI or WII/joystick has requested change.  Send it to the motor control
					nLastSpeedCmd = g_MotorCurrentSpeedCmd;
					nLastTurnCmd = g_MotorCurrentTurnCmd;
					ROBOT_LOG( DEBUG_MOTOR_COMMANDS, "Send User Command Joystick: Speed=%d, Turn=%d  ",  nLastSpeedCmd, nLastTurnCmd )

					SendCommand( WM_ROBOT_JOYSTICK_DRIVE_CMD, (DWORD)g_MotorCurrentSpeedCmd, (DWORD)g_MotorCurrentTurnCmd );
				}
				nMotorCmdTimer = 0;
			}
***/

	#if ( ROBOT_SERVER == 1 )  ///////////////////////////////////////////////////////////////////


			// Request a servo update on a regular cadance
			// Faster, if a servo is in motion
			// Hard coded to every 10th second - if( nTenthSecondTimer++ >= LOOP_TIMES_PER_SECOND/10 ) // every 10th second
			BOOL DynaServoMoving = IsDynaServoMoving();
			BOOL KerrServoMoving = IsKerrServoMoving();

			if( DynaServoMoving || KerrServoMoving )
			{
				//ROBOT_LOG( TRUE,"Fast Servo updates\n")
				if( nServoUpdateTimer++ >= LOOP_TIMES_PER_SECOND / 2 ) // n times per second
				{
					if( DynaServoMoving )
					{	// Yes, servo moving.  Request frequent updates, but only for the servos that are moving
						PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_GET_SMART_SERVO_STATUS), MOVING_SERVOS, FALSE );
					}
					if( KerrServoMoving )
					{	// Yes, servo moving.  Request frequent updates, but only for the servos that are moving
						PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_GET_SMART_SERVO_STATUS), MOVING_SERVOS, FALSE );
					}
					nServoUpdateTimer = 0; // Reset
				}
			}
			else
			{	// No servos moving, just track for normal updates
				if( nServoUpdateTimer++ >= (LOOP_TIMES_PER_SECOND * 2) ) // every 2 seconds
				{
					// Request servo postions
					PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_GET_SMART_SERVO_STATUS), ALL_SERVOS, FALSE );
					//PostThreadMessage( g_dwKerrServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_GET_SMART_SERVO_STATUS), ALL_SERVOS, FALSE );
					nServoUpdateTimer = 0;	// Reset
				}
			}


			// Request a laser scan on a regular cadance if enabled
			#if ( ROBOT_TYPE == LOKI )
				if( g_bLaserContinuousScanEnabled )
				{
					///TAL_Event("Laser Scan Request");
					if( nLaserScanTimer++ >= (SENSOR_SAMPLE_RATE) ) // n times per second
					{
						nLaserScanTimer = 0;
						PostThreadMessage( g_dwLaserScannerCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_LASER_REQUEST_SCAN), 0, 1 );	// Just one scan
					}
				}
			#endif
			// Handle various timers
			// Hard coded to every 10th second - if( nTenthSecondTimer++ >= LOOP_TIMES_PER_SECOND/10 ) // every 10th second
			{
				nTenthSecondTimer = 0;
				gCurrentTime++;	// Update global timer


				// Timers used by Modules:	// Assumes 1/10 second count!
				
				if( gKeyPressTimer != 0 ) 
				{
					gKeyPressTimer--;
				}
				if( gLocalUserCmdTimer != 0 ) 
				{
					gLocalUserCmdTimer--;
				}
				if( gRemoteUserCmdTimer != 0 ) 
				{
					gRemoteUserCmdTimer--;
				}
				if( gMotorSpeedTimer != 0 ) 
				{
					gMotorSpeedTimer--;
				}
				if( gBrakeTimer != 0 ) 
				{
					gBrakeTimer--;
				}
				if( gCollisionTimer != 0 ) 
				{
					gCollisionTimer--;
				}
				if( gAvoidanceTimer != 0 ) 
				{
					gAvoidanceTimer--;
				}
				if( gNavigationTimer != 0 ) 
				{
					gNavigationTimer--;
				}
				if( gServoOverHeatTimer != 0 ) 
				{
					gServoOverHeatTimer--;
				}
				if( gHeadNodTimer != 0 ) 
				{
					gHeadNodTimer--;
				}
				if( gHeadIdleTimer != 0 ) 
				{
					//ROBOT_LOG( TRUE,"GLOBAL gHeadIdleTimer = %ld\n", gHeadIdleTimer)
					gHeadIdleTimer--;
				}
				if( gBehaviorTimer != 0 ) 
				{
					gBehaviorTimer--;
				}

				#if ( ROBOT_HAS_RIGHT_ARM == 1 )
					if( gArmTimerRight != 0 ) 
					{
						gArmTimerRight--;
					}
					// Arm Servo count-down Stall Timers (initialized to TIMER_NOT_SET)
					if( g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].StallTimer >= 0 )		g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer >= 0 )	g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].StallTimer >= 0 )		g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].StallTimer >= 0 )			g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].StallTimer >= 0 )			g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].StallTimer--;
				#endif // ROBOT_HAS_RIGHT_ARM

				#if ( ROBOT_HAS_LEFT_ARM == 1 )
					if( gArmTimerLeft != 0 ) 
					{
						//ROBOT_LOG( TRUE,"GLOBAL gArmTimerLeft = %ld\n", gArmTimerLeft)
						gArmTimerLeft--;
					}
					// Arm Servo count-down Stall Timers (initialized to TIMER_NOT_SET)
					if( g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].StallTimer >= 0 )		g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer >= 0 )	g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].StallTimer >= 0 )		g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].StallTimer >= 0 )			g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].StallTimer--;
					if( g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].StallTimer >= 0 )			g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].StallTimer--;
				#endif // ROBOT_HAS_LEFT_ARM


				if( gHeadMoveTimeout != 0 ) 
				{
					//ROBOT_LOG( TRUE,"GLOBAL gHeadMoveTimeout = %ld\n", gHeadMoveTimeout)
					gHeadMoveTimeout--;
				}
				if( gHeadOwnerTimer != 0 ) // For tracking how long owner has control
				{
					//ROBOT_LOG( TRUE,"GLOBAL gHeadOwnerTimer = %ld\n", gHeadOwnerTimer)
					gHeadOwnerTimer--;
				}

				if( gDriveControlOwnerTimer != 0 ) // For tracking how long owner has control
				{
					//ROBOT_LOG( TRUE,"GLOBAL gDriveControlOwnerTimer = %ld\n", gDriveControlOwnerTimer)
					gDriveControlOwnerTimer--;
				}

				if( gKinectMoveTimeout != 0 ) 
				{
					//ROBOT_LOG( TRUE,"GLOBAL gKinectMoveTimeout = %ld\n", gLaserMoveTimeout)
					gKinectMoveTimeout--;
				}
				if( gKinectDelayTimer != 0 ) 
				{
					//ROBOT_LOG( TRUE,"GLOBAL gKinectDelayTimer = %ld\n", gKinectDelayTimer)
					gKinectDelayTimer--;
				}				

			}


			// Send status to remote client (laptop) in blocks, every N seconds
			if( nSendStatusCount++ >= LOOP_TIMES_PER_SECOND ) // Send 1 per seconds
			{
				///TAL_Event("Send To Client");
				nSendStatusCount = 0;
				SendResponse( WM_ROBOT_SEND_TEXT_MESSAGES, 0, 0 );
			}


			#if( (MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL) )
			// Request status from the controller
			if( g_nMotorStatusTimer++ >= (LOOP_TIMES_PER_SECOND/2) ) // Send n per second
			{
				///TAL_Event("Motor Status Request");
				if( g_hMotorCommPort != INVALID_HANDLE_VALUE )
				{
					// Send directly to the Servo COMM Port Thread
					Message = WM_ROBOT_MESSAGE_BASE+HW_GET_STATUS;
					PostThreadMessage( g_dwMotorCommThreadId, Message, 0, 0 );	// Send to the ER1 Pilot or Pololu TReX Controller too!
				}

				g_nMotorStatusTimer = 0;	
			}
			#endif

			// Check "smart servo" status, such as arm servos
			#if OTHER_SERVO_CONTROL_TYPE == DYNA_SERVO_CONTROL	
			if( (INVALID_HANDLE_VALUE != g_hKerrServoCommPort) && 
				#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
					(INVALID_HANDLE_VALUE != g_hDynaServoCommPort_RX64) &&
				#endif
				(INVALID_HANDLE_VALUE != g_hDynaServoCommPort_AX12) )
			{
				if( g_nServoStatusTimer++ >= (LOOP_TIMES_PER_SECOND / g_nServoStatusRequestsPerSecond) ) // Send n per second
				{
					///TAL_Event("Dyna Status Request");

					// Send command to read current status of all joints in the arms and head, etc.  Uses global g_BulkServoCmd to receive the data
					// Note that when a servo is moving, frequency of servo status updates is increased (g_nServoStatusRequestsPerSecond)

					// Send this command to an Dynamixel and Kerr servos via the SmartServoComm Thread
					PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_GET_SMART_SERVO_STATUS), ALL_SERVOS, FALSE );
					g_nServoStatusTimer = 0;	
				}
			}
			#endif


			// Automatically request a version from the Arduino, after the COMM port is opened
			#if ( ROBOT_TYPE == LOKI )
				if( 1 == ROBOT_SIMULATION_MODE )
				{
					// Arduino is not connected, simulate as if it were
					if( nStatusTimer++ >= (LOOP_TIMES_PER_SECOND/2) ) // Send n times per second
					{
						SendCommand( WM_ROBOT_REQUEST_STATUS_CMD, 0, 0 );
						nStatusTimer = 0;	
					}
				}
				else // Real robot
				{ 
					if( INVALID_HANDLE_VALUE != g_hArduinoCommPort)
					{
						if( SUBSYSTEM_CONNECTED != g_ArduinoSubSystemStatus )
						{
							// Ask for the status to start until we get some!
							// Receipt of first status packet sets SUBSYSTEM_CONNECTED for g_ArduinoSubSystemStatus
/*							if( nStatusTimer++ >= LOOP_TIMES_PER_SECOND ) // Send 1 per second
							{
								// Optimized to send directly to the Arduino COMM Port Write Thread
								Message = WM_ROBOT_MESSAGE_BASE+HW_GET_STATUS;
								PostThreadMessage( g_dwArduinoCommWriteThreadId, Message, 0, TRUE ); // TRUE = Enable Status updates
								nStatusTimer = 0;	
							}
*/
						}
/* TODO-MUST
						if( gPicWatchDogTimer++ > LOOP_TIMES_PER_SECOND )	// 1 second time out
						{
							g_ArduinoSubSystemStatus = SUBSYSTEM_FAILED;
							// Stop Motors!
							g_MotorCurrentSpeedCmd = 0;	// Stop
							g_MotorCurrentTurnCmd = 0;	// Center
							SendCommand( WM_ROBOT_JOYSTICK_DRIVE_CMD, 0, 0 );
							// Notify User!
							ROBOT_DISPLAY( TRUE, "Arduino ERROR!  Arduino WATCHDOG TIME OUT!" )
							SpeakText( "I have a Arduino Error. Motors disabled." );
							// Update the GUI
							SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
								ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
								0 );									// Param2 = not used
						}
*/
					}
				}
			#else
				// TURTLE
				if( 1 == ROBOT_SIMULATION_MODE )
				{
					// Motor base is not connected, simulate as if it were
					g_hArduinoCommPort = SIMULATED_SIO_HANDLE;
					if( nStatusTimer++ >= (LOOP_TIMES_PER_SECOND/2) ) // Send n times per second
					{
						if( SUBSYSTEM_CONNECTED != g_ArduinoSubSystemStatus )
						{
							SendCommand( WM_ROBOT_REQUEST_VERSION_CMD, 0, 0 );
						}
						else
						{
							SendCommand( WM_ROBOT_REQUEST_STATUS_CMD, 0, 0 );
						}
						nStatusTimer = 0;	
					}					
				}
				else
				{
					// Not in Simulation mode
					// Turtle does not use Arduino for sensors, so just request status as a heartbeat
					if( nStatusTimer++ >= SENSOR_SAMPLE_RATE ) // Send n per second
					{
						///TAL_Event("Heartbeat Status Request");
						SendCommand( WM_ROBOT_SENSOR_STATUS_READY, 0, 0 );
						nStatusTimer = 0;	
					}
				}
			#endif

			// See if we've received a keep alive from the client
			if( g_bConnectedToClient )
			{
				// ROBOT_LOG( TRUE, "TIME: Server Timer = %d\n", g_nClientKeepAliveCount )
				if( g_nClientKeepAliveCount > (LOOP_TIMES_PER_SECOND * 3) ) // every n seconds
				{
					// client must be dead, it should have reset our count!
/*					if( g_bResetWatchdog ) 
					{
						// Resetting of watchdog timer currently enabled, so disable it and display a message
						g_bResetWatchdog = FALSE;	// Tell Watchdog to kill 12v
						//SendCommand( WM_ROBOT_12V_POWER_CMD, POWER_OFF, 0 ); // and kill the power immediately
						//SendCommand( WM_ROBOT_WALK_STOP_CMD, 0, 0 );
						ROBOT_DISPLAY( TRUE, "Client Timed out. Stopped ." );
						//ROBOT_LOG( TRUE, "Client Timed out, Killing 12v Power!" )
					}
*/
				}
				else
				{
					g_nClientKeepAliveCount++;
					if( g_nClientKeepAliveCount > nMaxClientKeepAliveCount )
					{
						nMaxClientKeepAliveCount = g_nClientKeepAliveCount;
						// Send message to the GUI to display the time
						// TODO?: PostMessage( g_RobotCmdViewHWND, WM_ROBOT_DISPLAY_TCP_TIME, 0, nMaxClientKeepAliveCount );
					}

					/* For Now, disable this automatic reset.  Require user to manually reset the watchdog timer!
					if( !g_bResetWatchdog )
					{
						// Client died, but has come back.  Reenable keep alives
						g_bResetWatchdog = TRUE;	// Tell Watchdog to enable 12v
						SendCommand( WM_ROBOT_DRIVE_SPEED_CMD, 0, 0 );  // Set motors to Stop
						SendCommand( WM_ROBOT_12V_POWER_CMD, POWER_ON, 0 ); // and turn on the power immediately
						ROBOT_DISPLAY( TRUE, "Client back, reenabling 12v power" );
						//ROBOT_LOG( TRUE, "Client back, reenabling 12v power\n" )
					}
					*/
				}
			}
	#else	// ROBOTCLIENT //////////////////////////////////////////////////////////


			// Check to see if we are connected to the server
			if(	g_bConnectedToServer )
			{
				if( g_ConnectionMonitorTimer++ >= (LOOP_TIMES_PER_SECOND * 10) ) // Timeout value for connecton
				{
					ROBOT_DISPLAY( TRUE, "Connection to Robot Timed Out!  Connection Lost!");
					g_bConnectedToServer = FALSE;
					g_ConnectionMonitorTimer = 0;
				}
			}			
	 

			// See if it is time to send a KeepAlive ping to the robot server
			if(	g_bConnectedToServer )
			{
				if( nSendKeepAliveCount++ >= LOOP_TIMES_PER_SECOND/3 ) // Send 3 per second
				{
					nSendKeepAliveCount = 0;
					SendCommand( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD, 0, 0 );
				}
				// Everytime we receive a Keep Alive that got echoed back from the server
				// The receiving thread resets the g_LastPingTime to the current time.
				DWORD RoundTripTime = GetTickCount() - g_LastPingTime;
				PostMessage( g_RobotCmdViewHWND, (WM_ROBOT_DISPLAY_TCP_TIME), 0, RoundTripTime );
			}
	#endif	// ROBOTCLIENT

			__itt_task_end(pDomainGlobalThread);

		}
		RobotSleep( ((1000/LOOP_TIMES_PER_SECOND) - 10), pDomainGlobalThread );	// Assume loop takes about 10ms to process
	}

	ROBOT_LOG( TRUE,"Timer Thread exiting.\n")

#if ( ROBOT_SERVER == 1 )
	//SAFE_DELETE(pWiiControl);
#endif
	return 0;
}

//-----------------------------------------------------------------------------
// Name: CalculateTurn
// Desc: Given current heading and desired heading, returns amount in degrees
//       to turn.  Positive is right turn, Negative is Left turn
//-----------------------------------------------------------------------------
int CalculateTurn( int CurrentHeading, int DesiredHeading)
{
	if( (CurrentHeading > 360) || (DesiredHeading > 360) )
	{
		return RESULT_FAILURE;	// Bad value
	}

	int Delta = DesiredHeading - CurrentHeading;
	if( Delta > 180 )
	{
		// Don't go the long way around the circle!
		Delta = Delta-360;
	}
	else if( Delta < -180 )
	{
		// Don't go the long way around the circle!
		Delta = Delta+360;
	}
	return Delta;	// Success

}



//-----------------------------------------------------------------------------
// Name: DegreesToCompassRoseString
// Desc: Given current or desired heading in degrees, convert to "Compass Rose" points
//       returns a string
//-----------------------------------------------------------------------------
char* DegreesToCompassRoseString( int Degrees )
{
	// Return approx heading direction for display
	if(Degrees < 23) return("N");
	else if(Degrees < 68) return("NE");
	else if(Degrees < 113) return("E");
	else if(Degrees < 158) return("SE");
	else if(Degrees < 203) return("S");
	else if(Degrees < 248) return("SW");
	else if(Degrees < 293) return("W");
	else if(Degrees < 338) return("NW");
	else if(Degrees < 360) return("N");
	else return("---");
}

//-----------------------------------------------------------------------------
// Name: CompassRoseToDegrees
// Desc: Given "Compass Rose" points (eg. "NW"), convert to degrees
// Returns 0-360, or -1 if bad text
//-----------------------------------------------------------------------------
int CompassRoseToDegrees( int nCompassRose )
{
	switch( nCompassRose )
	{		
		case NORTH:			return   0;
		case NORTH_EAST:	return  45;
		case EAST:			return  90;
		case SOUTH_EAST:	return 135;
		case SOUTH:			return 180;
		case SOUTH_WEST:	return 225;
		case WEST:			return 270;
		case NORTH_WEST:	return 315;
	}
	ROBOT_ASSERT(0);
	return -1;
}

//-----------------------------------------------------------------------------
// Name: CompassRoseToString
// Desc: Given "Compass Rose" points (eg. NORTH), convert to string "north"
//-----------------------------------------------------------------------------
char* CompassRoseToString( int nCompassRose )
{
	switch( nCompassRose )
	{		
		case NORTH:			return	"north";
		case NORTH_EAST:	return	"north east";
		case EAST:			return	"east";
		case SOUTH_EAST:	return	"south east";
		case SOUTH:			return	"south";
		case SOUTH_WEST:	return	"south west";
		case WEST:			return	"west";
		case NORTH_WEST:	return	"north west";
	}
	ROBOT_ASSERT(0);
	return "error";
}

//-----------------------------------------------------------------------------
// Name: SendCommand
// Desc: For Client, sends command to Socket
//		 For Server, sends command to Control Modules
//-----------------------------------------------------------------------------
void SendCommand( DWORD Cmd, DWORD Param1, DWORD Param2 )
{

#if ( ROBOT_SERVER == 1 )
	// Send the command to the motor control

	while( 0 == g_dwControlThreadId )
	{
		// Wait for the thread to start
		ROBOT_LOG( TRUE,"Waiting for ControlThread to Start\n")
		Sleep(20);
	}

	if( WM_ROBOT_TEXT_MESSAGE_TO_SERVER == Cmd )
	{
		// Sending a text message to the robot from the client. Usually used for text to speech
		// In this instance, the GUI is running on the server, so no need to send to socket.
		// Convert to a local command
		Cmd = Param1;
		Param1 = Param2;
	}

	PostThreadMessage( g_dwControlThreadId, (Cmd), Param1, Param2 ); // Post as Windows message.  First command starts at WM_ROBOT_MESSAGE_BASE

#else	// ROBOTCLIENT
	// Send the command via Sockets to the Robot
	if( 0 != g_dwClientSendThreadId )
	{
		PostThreadMessage( g_dwClientSendThreadId, (Cmd), Param1, Param2 ); // Post as Windows message.  First command starts at WM_ROBOT_MESSAGE_BASE
	}
#endif


}


//-----------------------------------------------------------------------------
// Name: g_PostStatus
// Desc: Puts status message in global memory, then posts message to have dialog display it
//       We do it this way, so threads don't get blocked waiting to display on the GUI
//		 bRemote is a Optional Parameter (aka Default Parameter), that indicates message came from remote robot
//-----------------------------------------------------------------------------
void g_PostStatus( LPCTSTR lpszStatus, char *FunctionName, BOOL bDisplayOnGui, BOOL bRemote )
{
	IGNORE_UNUSED_PARAM (bRemote);

	CString strFormattedMessage;
	CString strTime;
	static int nLineNum = 0;
	DWORD CurrentTime = GetTickCount() - gStartTime;
	// TODO - Look at COleDateTime::GetCurrentTime().Format()
	if( !g_CriticalSectionsInitialized )
	{
		TRACE( "g_PostStatus: Ignoring ROBOT_DISPLAY()  because g_CriticalSectionsInitialized = FALSE\n" );
		if( NULL != g_LogFile )	fprintf(g_LogFile, " Ignoring ROBOT_DISPLAY() because g_CriticalSectionsInitialized = FALSE\n" );
		return;
	}

	// Format the current time from miliseconds since startup
	//nLineNum++;
	strTime.Format( _T("[%02d:%02d.%d] "), CurrentTime/60000, (CurrentTime /1000)%60, CurrentTime%100 ); //  min, sec, tenthsec, miliseconds (accurate to 10-16 ms)   
	//strTime.Format( _T("[%02d:%02d.%d] "), CurrentTime/600, (CurrentTime %600)/10, CurrentTime%10 ); //     ,hundredth-seconds

#if ( ROBOT_SERVER == 1 )
	
	// Format output for the Log.

	// Format the message with current time
	strFormattedMessage = strTime;

	// Add the function name, if available
	if( 0 != FunctionName )
	{
		strFormattedMessage += FunctionName;
		strFormattedMessage += ": ";
	}

	// Now, the user message
	strFormattedMessage += lpszStatus;
	strFormattedMessage += "\n"; // NOTE: automatically add a Newline


	//#ifdef _DEBUG
		#if TRACE_ENABLED == 1
			// Echo message to the debug window
			TRACE( (LPCTSTR)strFormattedMessage );
		#endif

		// Write to the log file
		if( NULL != g_LogFile )
		{
			fprintf(g_LogFile, "%s", strFormattedMessage );
		}


	if( bDisplayOnGui )
	{
		// GUI format is simpler to save GUI space

		strFormattedMessage = strTime;
		strFormattedMessage += lpszStatus;
		strFormattedMessage += "\n"; // NOTE: automatically add a Newline

		// Save the message to display on local GUI
	 	if( !g_CriticalSectionsInitialized ) return;
		__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, psh_csDisplayLock);
		EnterCriticalSection(&g_csDisplayLock);
			g_StatusMessagesToDisplay += strFormattedMessage;

			if( g_bConnectedToClient )
			{
				// Save message to send to Client (Handled in RobotServerSock.cpp)
				g_StatusMessagesToSend += strFormattedMessage;
			}
		LeaveCriticalSection(&g_csDisplayLock);
		__itt_task_end(pDomainGlobalThread);
		// Now, post a message to tell the GUI to display this when it gets a chance
		PostMessage( g_RobotCmdViewHWND, (WM_ROBOT_DISPLAY_STATUS_MESSAGES), 0, 0 );
	}
	

#else

	// Save the message to display
	{
	 	if( !g_CriticalSectionsInitialized ) return;
		__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, psh_csDisplayLock);
		EnterCriticalSection(&g_csDisplayLock);

		if( bRemote )  
		{
			// Message came from the robot server, not local code
			g_StatusMessagesToDisplay += lpszStatus;
		}
		else
		{
			nLineNum++;
			strFormattedMessage.Format( _T("%04d [%02d:%02d.%d] "),
				nLineNum, CurrentTime/600, (CurrentTime %600)/10, CurrentTime%10 );	// gCurrentTime = 10th seconds
		
			strFormattedMessage += lpszStatus;
			if( 0 != FunctionName )
			{
				strFormattedMessage += "   ";
				strFormattedMessage += FunctionName;
			}
			strFormattedMessage += "\n";

			// Echo message to the debug window
			TRACE( strFormattedMessage );

		}
		LeaveCriticalSection(&g_csDisplayLock);
		__itt_task_end(pDomainGlobalThread);
	}

	// Now, post a message to tell the GUI to display this when it gets a chance
	{
		PostMessage( g_RobotCmdViewHWND, (WM_ROBOT_DISPLAY_STATUS_MESSAGES), 0, 0 );
	}

#endif
}

//-----------------------------------------------------------------------------
// Name: SendResponse
//		 For Server, sends response to Client
//-----------------------------------------------------------------------------
void SendResponse( DWORD Cmd, DWORD Param1, DWORD Param2 )
{
#if ( ROBOT_SERVER == 1 )
	ROBOT_ASSERT( 1 != Cmd );
	// Display message locally
	// Post as Windows message.  First command starts at WM_ROBOT_MESSAGE_BASE
	PostMessage( g_RobotCmdViewHWND, (Cmd), Param1, Param2 );
	PostMessage( g_RobotPathViewHWND, (Cmd), Param1, Param2 );
	PostMessage( g_RobotMapViewHWND, (Cmd), Param1, Param2 );
	PostMessage( g_RobotSetupViewHWND, (Cmd), Param1, Param2 );

	// And send it to the client
	if( g_bConnectedToClient )	
	{
		PostThreadMessage( g_dwServerSendThreadId, (Cmd), Param1, Param2 );
	}
#endif	// ROBOT_SERVER
}

//-----------------------------------------------------------------------------
// Name: OpenCommPort
// Desc: Generic function to open any COMM port.  If sucessful, returns Handle to the port
//-----------------------------------------------------------------------------

/*************************
For ER1:
			memset(&dcb,0,sizeof(DCB));

			dcb.DCBlength=sizeof(DCB);
			dcb.BaudRate = 250000;
			dcb.fBinary  = TRUE;
			dcb.fParity  = FALSE;
			dcb.fOutxCtsFlow = FALSE;
			dcb.fOutxDsrFlow = FALSE;
			//dcb.fDtrControl = DTR_CONTROL_ENABLE;
			dcb.fDsrSensitivity = FALSE;
			//dcb.fTXContinueOnXoff = TRUE;
			//dcb.fOutX = TRUE;
			//dcb.fInX = TRUE;
			dcb.fErrorChar = FALSE;
			dcb.fNull = FALSE;
			//dcb.fRtsControl = RTS_CONTROL_ENABLE;
			//dcb.fAbortOnError = TRUE;
			//dcb.fDummy2 =;
			//dcb.wReserved =;
			//dcb.XonLim = 2048;
			//dcb.XoffLim = 512;
			dcb.ByteSize = 8;
			dcb.Parity = NOPARITY;
			dcb.StopBits = ONESTOPBIT;
			//dcb.XonChar = 17;
			//dcb.XoffChar = 19;
			//dcb.ErrorChar = ;
			//dcb.EofChar = EOF;
			//dcb.EvtChar = ;
			//dcb.wReserved1 =;

			dwErr=SetCommState(hRCMCom,&dcb);
		}

		
		{
			CommTimeout.ReadIntervalTimeout=0;
			CommTimeout.ReadTotalTimeoutMultiplier=1;
			CommTimeout.ReadTotalTimeoutConstant=1;
			CommTimeout.WriteTotalTimeoutMultiplier=0;
			CommTimeout.WriteTotalTimeoutConstant=0;

			SetCommTimeouts(hRCMCom,&CommTimeout);
		}



*************************/





HANDLE OpenCommPort(LPCTSTR strPort, DWORD nBaudRate, DWORD Device)
{
	HANDLE hCommPort = INVALID_HANDLE_VALUE;

	if( 1 == ROBOT_SIMULATION_MODE )
	{
		CString strStatus;
		strStatus.Format(_T("COM Port %s SIMULATED due to ROBOT_SIMULATION_MODE"), strPort );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)strStatus )
		return SIMULATED_SIO_HANDLE;	// For simulation, return bogus handle 
	}

#if ( ROBOT_SERVER == 1 )


	COMMTIMEOUTS ct;	// = {0};
	DCB dcb;			//  = {0};
	SecureZeroMemory(&ct, sizeof(COMMTIMEOUTS));
	SecureZeroMemory(&dcb, sizeof(DCB));


	if( (MOTOR_COMM_DEVICE == Device) && (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) )
	{
		// TO TRY: For flushing output: FILE_FLAG_NO_BUFFERING and FILE_FLAG_WRITE_THROUGH or use FlushFileBuffers in the write routine
		hCommPort = CreateFile ( strPort,		// Port Name (Unicode compatible)
			GENERIC_READ | GENERIC_WRITE,		// Open for Read-Write
            0,									// COM port cannot be shared
            NULL,								// Always NULL for Windows CE
            OPEN_EXISTING,						// For communication resource
            FILE_FLAG_NO_BUFFERING | FILE_FLAG_WRITE_THROUGH,  // dwFlagsAndAttributes
            NULL );								// Always NULL for Windows CE

		if(hCommPort == INVALID_HANDLE_VALUE)
		{
			ROBOT_LOG( TRUE, "Error Opening COMM Port %s\n" , strPort)
			return INVALID_HANDLE_VALUE;
		}

		//ct.ReadIntervalTimeout = MAXDWORD;		// Maximum time between read chars. 
		//ct.ReadTotalTimeoutMultiplier = 0;		// Timout that is multiplied by the number of characters requested.        
		//ct.ReadTotalTimeoutConstant = 0;		// Constant in milliseconds.        
		//ct.WriteTotalTimeoutMultiplier = 1;//10;	// Multiplier of characters.        
		//ct.WriteTotalTimeoutConstant = 1;//1000;	// Constant in milliseconds.        

		ct.ReadIntervalTimeout = 10;			// Max time between read chars.  iRobot reports status every 15ms.  Will this break between lines? 
		ct.ReadTotalTimeoutMultiplier = 0;		// Timeout = n MS * Number of Characters        
		ct.ReadTotalTimeoutConstant = 300;		// Add to ReadTotalTimeoutMultiplier number above      
		ct.WriteTotalTimeoutMultiplier = 5;		// Multiplier of characters. (add 10MS for each char to write)        
		ct.WriteTotalTimeoutConstant = 10;		// Constant in milliseconds.  (1 second time-out)        

		// get the current communications parameters, and configure baud rate
		dcb.DCBlength = sizeof(DCB);
		if(!GetCommState(hCommPort, &dcb))
		{
			ROBOT_LOG( TRUE, "Error Getting Comms. State." )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}
		//PrintCommState(dcb);       //  Output to console

		dcb.BaudRate		= nBaudRate;
		dcb.fBinary         = TRUE;	// Windows only supports binary transfer mode
//		dcb.fOutxCtsFlow	= FALSE;	// Don't wait for the device to say OK to send
//		dcb.fRtsControl     = RTS_CONTROL_DISABLE; // DWS CHANGE  RTS_CONTROL_DISABLE; //RTS_CONTROL_ENABLE; // Set RTS on TODO:  try RTS_CONTROL_HANDSHAKE;
//		dcb.fDtrControl     = DTR_CONTROL_ENABLE; //ENABLE;	// Set DTR line to true and leave it on
//		dcb.fOutxCtsFlow	= FALSE;
//		dcb.fOutxDsrFlow    = FALSE;
//		dcb.fOutX           = FALSE; // no XON/XOFF control
//		dcb.fInX            = FALSE;
		dcb.ByteSize        = 8;
		dcb.Parity          = NOPARITY;
		dcb.StopBits        = ONESTOPBIT; // ONE5STOPBITS
		dcb.fAbortOnError	= FALSE; // TRUE; // DWS CHANGE


		if(!SetCommTimeouts(hCommPort, &ct))
		{
			ROBOT_LOG( TRUE, "Error Setting comm. timeouts.\n" )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}

		if(!SetCommState(hCommPort, &dcb))
		{
			ROBOT_LOG( TRUE, "Error Setting Comms. State.\n" )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}	
		else
		{
			ROBOT_LOG( TRUE, "COM Port State set\n" )
		}

	}
	else // NOT IROBOT_MOTOR_CONTROL
	{
		hCommPort = CreateFile ( strPort, // Port Name (Unicode compatible)
			GENERIC_READ | GENERIC_WRITE, // Open for Read-Write
            0,             // COM port cannot be shared
            NULL,          // Always NULL for Windows CE
            OPEN_EXISTING, // For communication resource
            0,             // dwFlagsAndAttributes
            NULL );        // Always NULL for Windows CE
	
		if(hCommPort == INVALID_HANDLE_VALUE)
		{
			ROBOT_LOG( TRUE, "Error Opening COMM Port %s\n", strPort)
			return INVALID_HANDLE_VALUE;
		}


		// TODO: set the timeouts to specify the behavor of reads and writes.
		ct.ReadIntervalTimeout = MAXDWORD;		// Maximum time between read chars. 
		ct.ReadTotalTimeoutMultiplier = 0;		// Timout that is multiplied by the number of characters requested.        
		ct.ReadTotalTimeoutConstant = 0;		// Constant in milliseconds.        
		ct.WriteTotalTimeoutMultiplier = 1;//10;	// Multiplier of characters.        
		ct.WriteTotalTimeoutConstant = 1;//1000;	// Constant in milliseconds.        


		// get the current communications parameters, and configure baud rate
		dcb.DCBlength = sizeof(DCB);
		if(!GetCommState(hCommPort, &dcb))
		{
			ROBOT_LOG( TRUE, "Error Getting Comms. State." )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}
		dcb.BaudRate		= nBaudRate;
		dcb.fOutxCtsFlow	= FALSE;	// Don't wait for the device to say OK to send
		dcb.fRtsControl     = RTS_CONTROL_DISABLE; // DWS CHANGE  RTS_CONTROL_DISABLE; //RTS_CONTROL_ENABLE; // Set RTS on TODO:  try RTS_CONTROL_HANDSHAKE;
		dcb.fDtrControl     = DTR_CONTROL_ENABLE; //ENABLE;	// Set DTR line to true and leave it on
		dcb.fOutxCtsFlow	= FALSE;
		dcb.fOutxDsrFlow    = FALSE;
		dcb.fOutX           = FALSE; // no XON/XOFF control
		dcb.fInX            = FALSE;
		dcb.ByteSize        = 8;
		dcb.Parity          = NOPARITY;
		dcb.StopBits        = ONESTOPBIT;
		dcb.fAbortOnError	= FALSE; // TRUE; // DWS CHANGE



		// Special Overrides
		if( ARDUINO_COMM_DEVICE == Device )
		{
			dcb.fAbortOnError	= FALSE;		// TODO-MUST!  DOES THIS WORK BETTER?  Arduino DEBUG
			ct.WriteTotalTimeoutMultiplier = 0;	// Multiplier of characters.        
			ct.WriteTotalTimeoutConstant = 0;	// Constant in milliseconds.  

			// NOTE! CHANGE THIS TO "DTR_CONTROL_ENABLE" to CAUSE ARDUINO TO RESET EVERYTIME PC CODE STARTS
			//dcb.fDtrControl     = DTR_CONTROL_DISABLE;   // if DTR_CONTROL_ENABLE, set DTR line to true and leave it on
			dcb.fDtrControl     = DTR_CONTROL_ENABLE;   // if DTR_CONTROL_ENABLE, set DTR line to true and leave it on

		}
		else if( LASER_SCANNER_COMM_DEVICE == Device )
		{
			//ct.ReadIntervalTimeout = 0;				// Max time between each char. Seems like a good indication that the message is done
			//ct.ReadTotalTimeoutMultiplier = 2;		// Timeout = 20MS * Number of Characters        
			//ct.ReadTotalTimeoutConstant = 10;		// Add to the number above      
			//ct.WriteTotalTimeoutMultiplier = 10;	// Multiplier of characters. (add 10MS for each char to write)        
			//ct.WriteTotalTimeoutConstant = 1000;	// Constant in milliseconds.  (1 second time-out)        
		}
		else if( DYNA_SERVO_AX12_COMM_DEVICE == Device )
		{
			ct.ReadIntervalTimeout = 32;			// Max time between read chars 
			ct.ReadTotalTimeoutMultiplier = 20;		// Timeout = x MS * Number of Characters        
			ct.ReadTotalTimeoutConstant = 400;		// Add to the number above      
			//ct.WriteTotalTimeoutMultiplier = 10;	// Multiplier of characters. (add 10MS for each char to write)        
			//ct.WriteTotalTimeoutConstant = 1000;	// Constant in milliseconds.  (1 second time-out)        
		}
		else if( DYNA_SERVO_RX64_COMM_DEVICE == Device )
		{
			ct.ReadIntervalTimeout = 32;			// Max time between read chars. 
			ct.ReadTotalTimeoutMultiplier = 20;		// Timeout = 10MS * Number of Characters        
			ct.ReadTotalTimeoutConstant = 400;		// Add to the number above      
			//ct.WriteTotalTimeoutMultiplier = 10;	// Multiplier of characters. (add 10MS for each char to write)        
			//ct.WriteTotalTimeoutConstant = 1000;	// Constant in milliseconds.  (1 second time-out)        
		}
		else if( KERR_SERVO_COMM_DEVICE == Device )
		{
			ct.ReadIntervalTimeout = 32;			// Max time between read chars.  It can take a while to get next USB buffer... 
			ct.ReadTotalTimeoutMultiplier = 20;		// Timeout = 10MS * Number of Characters        
			ct.ReadTotalTimeoutConstant = 200;		// Add to the number above      
			//ct.WriteTotalTimeoutMultiplier = 10;	// Multiplier of characters. (add 10MS for each char to write)        
			//ct.WriteTotalTimeoutConstant = 1000;	// Constant in milliseconds.  (1 second time-out)        
		}
		else if( SERVO_COMM_DEVICE == Device )
		{
		}
		else if( MOTOR_COMM_DEVICE == Device )
		{
			// ER1 Control
	/*
			ct.ReadIntervalTimeout = 0; 
			ct.ReadTotalTimeoutMultiplier = 0; 
			ct.ReadTotalTimeoutConstant = 0; 
			ct.WriteTotalTimeoutMultiplier = 10; 
			ct.WriteTotalTimeoutConstant = 1000; 
	*/
			#if MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL
				ct.ReadIntervalTimeout = 10;				// Max time between read chars.  iRobot reports status every 15ms.  Will this break between lines? 
			#else
				ct.ReadIntervalTimeout = 0;				// Max time between read chars.  Interval Timer not used 
			#endif
			ct.ReadTotalTimeoutMultiplier = 10;		// Timeout = 10MS * Number of Characters        
			ct.ReadTotalTimeoutConstant = 100;		// Add 100MS to tWriteTotalTimeoutConstanthe number above      
			ct.WriteTotalTimeoutMultiplier = 0;//10;	// Multiplier of characters. (add 10MS for each char to write)        
			ct.WriteTotalTimeoutConstant = 0; //1000;	// Constant in milliseconds.  (1 second time-out)        


	/*
			dcb.fOutxCtsFlow	= FALSE;	// Don't wait for the device to say OK to send
			dcb.fRtsControl     = RTS_CONTROL_DISABLE; //RTS_CONTROL_ENABLE; // Set RTS on TODO:  try RTS_CONTROL_HANDSHAKE;
			dcb.fDtrControl     = DTR_CONTROL_DISABLE;// Set DTR line to false and leave it
			dcb.fOutxDsrFlow    = FALSE;
			dcb.fOutX           = FALSE; // no XON/XOFF control
			dcb.fInX            = FALSE;
			dcb.ByteSize        = 8;
			dcb.Parity          = NOPARITY;
			dcb.StopBits        = ONESTOPBIT;
			dcb.fAbortOnError	= TRUE;
	*/

	//		dcb.BaudRate; (already set)				// Baudrate at which running       
			dcb.fBinary  = TRUE;					// *Binary Mode (skip EOF check)    
			dcb.fParity  = FALSE;					// *Enable parity checking          
			dcb.fOutxCtsFlow = FALSE;				// CTS handshaking on output       
			dcb.fOutxDsrFlow = FALSE;				// *DSR handshaking on output       
			dcb.fDtrControl = DTR_CONTROL_DISABLE;	// DTR Flow control                
			dcb.fDsrSensitivity = FALSE;			// *DSR Sensitivity              
	//		dcb.fTXContinueOnXoff;					// Continue TX when Xoff sent 
			dcb.fOutX = FALSE;						// Enable output X-ON/X-OFF        
			dcb.fInX= FALSE;						// Enable input X-ON/X-OFF         
			dcb.fErrorChar = FALSE;					// *Enable Err Replacement          
			dcb.fNull = FALSE;						// *Enable Null stripping           
			dcb.fRtsControl = RTS_CONTROL_DISABLE;	// Rts Flow control                
			dcb.fAbortOnError = FALSE;				// DONT Abort all reads and writes on Error 
	//		dcb.XonLim;								// Transmit X-ON threshold         
	//		dcb.XoffLim;							// Transmit X-OFF threshold        
			dcb.ByteSize = 8;						// Number of bits/dcb., 4-8        
			dcb.Parity = NOPARITY;					// 0-4=None,Odd,Even,Mark,Space    
			dcb.StopBits = ONESTOPBIT;				// 0,1,2 = 1, 1.5, 2               
	//		dcb.Xondcb.;							// Tx and Rx X-ON dcb.acter        
	//		dcb.Xoffdcb.;							// Tx and Rx X-OFF dcb.acter       
	//		dcb.Errordcb.;							// Error replacement dcb.         
	//		dcb.Eofdcb.;							// End of Input dcb.acter          
	//		dcb.Evtdcb.;							// Received Event dcb.acter        

		}
		else if ( GPS_COMM_DEVICE == Device )
		{
			dcb.fBinary			= TRUE;				// binary mode, no EOF check 
			dcb.fDtrControl = DTR_CONTROL_ENABLE;	// DTR flow control type 
			dcb.fDsrSensitivity = FALSE;			// DSR sensitivity 
			dcb.fNull = FALSE;						// enable null stripping 
			dcb.fRtsControl=RTS_CONTROL_ENABLE;		// RTS flow control 
			dcb.fAbortOnError = FALSE;				// abort reads/writes on error 
		}

		if(!SetCommTimeouts(hCommPort, &ct))
		{
			ROBOT_LOG( TRUE, "Error Setting comm. timeouts." )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}

		if(!SetCommState(hCommPort, &dcb))
		{
			ROBOT_LOG( TRUE, "Error Setting Comms. State." )
			CloseCommPort( strPort, hCommPort );
			return INVALID_HANDLE_VALUE;
		}	
		else
		{
			ROBOT_LOG( TRUE, "COM Port dcb set\n" )
		}

	}
#endif // ROBOT_SERVER
	return hCommPort;	// If everything went OK, return handle to the port

}
void CloseCommPort( LPCTSTR strPort, HANDLE &hPort )
{
	if( SIMULATED_SIO_HANDLE == hPort)
	{
		ROBOT_LOG( TRUE,"%s COM Port SIMULATED closed\n", strPort)
		return;
	}

#if ( ROBOT_SERVER == 1 )

	if( (INVALID_HANDLE_VALUE != hPort)  && (SIMULATED_SIO_HANDLE != hPort) )
	{
		// Close the Serial Port
		ROBOT_LOG( TRUE,"Waiting for %s COM Port to close\n", strPort)

		PurgeComm( hPort, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR );
		// TODO!  Will it lock up the COM port if we don't do this?
		// Try If Connected To Arduino { ... (or check Arduino Version flag?)

		if( (g_hMotorCommPort == hPort) && (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) )
		{
			CloseHandle( hPort );	// This hangs for Arduino unless Arduino sends a byte to the port!
		}
		hPort = INVALID_HANDLE_VALUE;
		ROBOT_LOG( TRUE,"%s COM Port closed\n", strPort)
	}


#endif // ROBOT_SERVER
}

//-----------------------------------------------------------------------------
// Name: GPSDegreesToRWTenthInches
// Desc: Given Latitude and Longitude, calculates X,Y of current 
//       Real World map position (in tenth inches)
//		 This function used by client and server
//-----------------------------------------------------------------------------

POINT GPSDegreesToRWTenthInches( double Lat, double Long )
{
	POINT RW;	// Real World Tenth Inches

	if( (-1 == gGPSOriginLat) && (-1 == gGPSOriginLong) )
	{
		// GPS Origin not initialized!
		ROBOT_LOG( TRUE,"ERROR: GPS ORIGIN NOT INITIALIZED!\n")
		RW.x = 0;
		RW.y = 0;
	}
	else
	{
		double DeltaLat = Lat - gGPSOriginLat;
		if( DeltaLat < 0 ) DeltaLat = DeltaLat*(-1);	// absolute value
		RW.y = (long)(DeltaLat * GPS_TENTH_INCHES_PER_DEGREE_LAT);
		
		double DeltaLong = Long - gGPSOriginLong;
		if( DeltaLong < 0 ) DeltaLong = DeltaLong*(-1);	// absolute value
		RW.x = (long)(DeltaLong * GPS_TENTH_INCHES_PER_DEGREE_LONG);
	}
	return RW;

}


//-----------------------------------------------------------------------------
// Name: ModuleNumberToName
// Desc: Given a module number, return a string with the module name for display
//		 This function used by client and server
//-----------------------------------------------------------------------------
void ModuleNumberToName(int  Module, CString &ModuleString)
{
	switch( Module )
	{		
		case LOCAL_USER_MODULE:
			ModuleString = "LOCAL_USER_MODULE";
			break;
		case COLLISION_MODULE:
			ModuleString = "COLLISION_MODULE";
			break;
		case AVOID_OBJECT_MODULE:
			ModuleString = "AVOID_OBJECT_MODULE";
			break;
		case REMOTE_USER_MODULE:
			ModuleString = "REMOTE_USER_MODULE";
			break;
		case BEHAVIOR_GOAL_MODULE:
			ModuleString = "BEHAVIOR_GOAL_MODULE";
			break;
		case WAY_POINT_NAV_MODULE:
			ModuleString = "WAY_POINT_NAV_MODULE";
			break;
		case GRID_NAV_MODULE:
			ModuleString = "GRID_NAV_MODULE";
			break;
		case NO_MODULE:
			ModuleString = "NO_MODULE";
			break;
		default:
			ModuleString = "UNKNOWN MODULE";
			ROBOT_ASSERT(0);
			break;
	}

}

//-----------------------------------------------------------------------------
// Name: HeadOwnerNumberToName
// Desc: Given a Head Owner number, return a string with the Owner name for display
//		 This function used by client and server
//-----------------------------------------------------------------------------
void HeadOwnerNumberToName(int  nOwner, CString &strOwner)
{
	switch( nOwner )
	{		
		case HEAD_OWNER_USER_CONTROL:
			strOwner = "HEAD_OWNER_USER_CONTROL";
			break;
		case HEAD_OWNER_TRACK_OBJECT:
			strOwner = "HEAD_OWNER_TRACK_OBJECT";
			break;
		case HEAD_OWNER_HEAD_NOD:
			strOwner = "HEAD_OWNER_HEAD_NOD";
			break;
		case HEAD_OWNER_BEHAVIOR_P1:
			strOwner = "HEAD_OWNER_BEHAVIOR_P1";
			break;
		case HEAD_OWNER_KINECT_HUMAN:
			strOwner = "HEAD_OWNER_KINECT_HUMAN";
			break;
		case HEAD_OWNER_FACE_TRACKER:
			strOwner = "HEAD_OWNER_FACE_TRACKER";
			break;
		case HEAD_OWNER_BEHAVIOR_P2:
			strOwner = "HEAD_OWNER_BEHAVIOR_P2";
			break;
		case HEAD_OWNER_MOTION_TRACKER:
			strOwner = "HEAD_OWNER_MOTION_TRACKER";
			break;
		case HEAD_OWNER_PIR_TRACKER:
			strOwner = "HEAD_OWNER_PIR_TRACKER";
			break;
		case HEAD_OWNER_RANDOM_MOVEMENT:
			strOwner = "HEAD_OWNER_RANDOM_MOVEMENT";
			break;
		case HEAD_OWNER_NONE:
			strOwner = "HEAD_OWNER_NONE";
			break;
		default:
			strOwner = "*** ERROR UNKNOWN OWNER ***";
			break;
	}

}

//-----------------------------------------------------------------------------
// Name: KinectOwnerNumberToName
// Desc: Given a Laser Owner number, return a string with the Owner name for display
//		 This function used by client and server
//-----------------------------------------------------------------------------
void KinectOwnerNumberToName(int  nOwner, CString &strOwner)
{
	switch( nOwner )
	{		
		case KINECT_TILT_OWNER_USER_CONTROL:
			strOwner = "KINECT_TILT_OWNER_USER_CONTROL";
			break;
		case KINECT_TILT_OWNER_TRACK_HUMAN:
			strOwner = "KINECT_TILT_OWNER_BEHAVIOR_TRACK_OBJECT";
			break;
		case KINECT_TILT_OWNER_TRACK_OBJECT:
			strOwner = "KINECT_TILT_OWNER_BEHAVIOR_TRACK_OBJECT";
			break;
		case KINECT_TILT_OWNER_COLLISION_AVOIDANCE:
			strOwner = "KINECT_TILT_OWNER_BEHAVIOR_COLLISION_AVOIDANCE";
			break;
		case KINECT_TILT_OWNER_NONE:
			strOwner = "KINECT_TILT_OWNER_NONE";
			break;
		default:
			strOwner = "UNKNOWN OWNER";
			ROBOT_ASSERT(0);
			break;
	}

}

//-----------------------------------------------------------------------------
// Name: FPointToPoint
// Desc: Converts high precision FPOINT (double POINTs) to POINT
//-----------------------------------------------------------------------------
POINT FPointToPoint( FPOINT fPt)
{
	POINT Pt;
	Pt.x = (long)fPt.x;
	Pt.y = (long)fPt.y;
	return Pt;
}


//-----------------------------------------------------------------------------
// Name: CalculateAngle
// Desc: Given 2 points, returns direction angle
//-----------------------------------------------------------------------------

int  CalculateAngle( POINT From, POINT To )
{
	// Convert Both to FPOINT
	FPOINT FptFrom = {(double)From.x, (double)From.y};
	FPOINT FptTo = {(double)To.x, (double)To.y};
	return CalculateAngle( FptFrom, FptTo );
}

int  CalculateAngle( FPOINT From, POINT To )
{
	// Convert To to FPOINT
	FPOINT FptTo = {(double)To.x, (double)To.y};
	return CalculateAngle( From, FptTo );
}

int  CalculateAngle( POINT From, FPOINT To )
{
	// Convert From to FPOINT
	FPOINT FptFrom = {(double)From.x, (double)From.y};
	return CalculateAngle( FptFrom, To );
}

int  CalculateAngle( FPOINT From, FPOINT To )
{
	// atan2 returns a value in the range -n to n radians, using the signs of 
	// both parameters to determine the quadrant of the return value
	double DeltaY = To.y - From.y;
	double DeltaX = To.x - From.x;
	double AngleRadians = atan2( DeltaX, DeltaY );	// X,Y backward from classical math
	double AngleDegrees = AngleRadians * RADIANS_TO_DEGREES;
	if( AngleDegrees < 0 )
	{
		// ArcTan return negatives for 180-360 degrees
		AngleDegrees = 360 + AngleDegrees;
	}
	AngleDegrees += 0.5;	// Cast will truncate, this will round instead

	return (int )AngleDegrees;
}

double CalculateAnglePlusMinus( FPOINT From, FPOINT To )
{
	// atan2 returns a value in the range -n to n radians, using the signs of 
	// both parameters to determine the quadrant of the return value
	double DeltaY = To.y - From.y;
	double DeltaX = To.x - From.x;
	double AngleRadians = atan2( DeltaX, DeltaY );	// X,Y backward from classical math
	double AngleDegrees = AngleRadians * RADIANS_TO_DEGREES;
	return AngleDegrees;
}


//-----------------------------------------------------------------------------
// Name: CalculateDistance
// Desc: Given 2 points, returns distance between them
//-----------------------------------------------------------------------------
int  CalculateDistance( POINT From, POINT To )
{
	FPOINT FptFrom = {(double)From.x, (double)From.y};
	return CalculateDistance( FptFrom, To );
}


int  CalculateDistance( FPOINT From, POINT To)
{

	// Use basic Pythagorean theorem: C**2 = a**2 + b**2

	double x = To.x - From.x;	// don't worry about the sign
	double y = To.y - From.y;

	int  Distance = (int )sqrt((x*x) + (y*y) );

	return Distance;

}

//-----------------------------------------------------------------------------
// Name: ConvertPolarToRectangular
// Desc: Given starting point, angle, and distance, find new point
//-----------------------------------------------------------------------------
POINT ConvertPolarToRectangular( POINT Origin, double AngleDegrees, double Distance )
{

	double AngleRadians = AngleDegrees * DEGREES_TO_RADIANS;
	double dX = Distance * sin( AngleRadians );	// Returns negative numbers as needed
	double dY = Distance * cos( AngleRadians );
	
	if(dX>=0) dX += 0.5; else dX -= 0.5;	// Cast will truncate, this will round instead
	if(dY>=0) dY += 0.5; else dY -= 0.5;
	POINT NewPosition;
	NewPosition.x = (int)Origin.x + (int)dX;		// Get new Map absolute X,Y
	NewPosition.y = (int)Origin.y + (int)dY;


	if( NewPosition.x < 0) 
	{
		ROBOT_LOG( TRUE,"ERROR! ConvertPolarToRectangular: NewPosition.x = %d\n", NewPosition.x )
		NewPosition.x = 0;	// trap bad values
	}
	if( NewPosition.y < 0)
	{
		ROBOT_LOG( TRUE,"ERROR! ConvertPolarToRectangular: NewPosition.y = %d\n", NewPosition.y )
		NewPosition.y = 0;
	}

	return NewPosition;
}

//-----------------------------------------------------------------------------
// Name: FastTimer Class
// Desc: High resolution timer for fast events
//-----------------------------------------------------------------------------

double FastTimer::LargeIntToSeconds( LARGE_INTEGER & L) 
{
	return ((double)L.QuadPart /(double)frequency.QuadPart);
}

double FastTimer::LargeIntToMS( LARGE_INTEGER & L) 
{
	return ((double)L.QuadPart * 1000 /(double)frequency.QuadPart);
}

FastTimer::FastTimer()
{
	timer.start.QuadPart=0;
	timer.last.QuadPart=0;	
	QueryPerformanceFrequency( &frequency );
}

void FastTimer::startTimer( ) 
{
    QueryPerformanceCounter(&timer.start);
	timer.last.QuadPart = timer.start.QuadPart;
}

void FastTimer::stopTimer( ) 
{
	timer.start.QuadPart=0;
	timer.last.QuadPart=0;	
}

BOOL FastTimer::Running( ) 
{	// Is the timer running?
    if( 0 != timer.start.QuadPart )
		return TRUE;
	else
		return FALSE;
}
double FastTimer::getTime() 
{
	LARGE_INTEGER time;
	LARGE_INTEGER result;
	if( 0 == timer.start.QuadPart )
	{	// Timer was never started
		return 0.0;
	}	
    QueryPerformanceCounter(&time);
	result.QuadPart = time.QuadPart - timer.start.QuadPart;
    return LargeIntToSeconds( result ) ;
}

double FastTimer::getElapsedTime() 
{
	LARGE_INTEGER time;
	LARGE_INTEGER result;
	if( 0 == timer.start.QuadPart )
	{	// Timer was never started
		return 0.0;
	}	
    QueryPerformanceCounter(&time);
	result.QuadPart = time.QuadPart - timer.last.QuadPart;
	timer.last.QuadPart = time.QuadPart;
    return LargeIntToMS( result ) ;
}


//-----------------------------------------------------------------------------
// Name: IsDynaServoMoving
// Desc: See if any of the Dynamixel servos are moving
// Used to set the frequency of servo updates dynamically
//-----------------------------------------------------------------------------

BOOL IsDynaServoMoving()
{
	// Laser Scanner
	// TODO? if( ROBOT_HAS_KINECT_SERVO )
	{
		if( abs( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees ) > KINECT_SERVO_TOLERANCE_NORMAL_TENTHDEGREES ) return TRUE;
	}
	// Camera/Head servos
	if( abs( g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;

	// Right Arm
	if( abs( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees )> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees )	> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;

	// Left Arm
	if( abs( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees )	> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;
	if( abs( g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees )			> ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;

	return FALSE;
}

//-----------------------------------------------------------------------------
// Name: IsKerrServoMoving
// Desc: See if one of the Kerr servos (shoulder motors) are moving
// Used to set the frequency of servo updates dynamically
//-----------------------------------------------------------------------------

BOOL IsKerrServoMoving()
{
	// Shoulder Servos
	if( abs( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;

	if( abs( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees - 
		  g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES ) return TRUE;

	return FALSE;
}


//-----------------------------------------------------------------------------
// Name: FindDoors
// Desc: Using data from depth finder (Kinect, LaserScanner or other),
// Find doorways in front of me, so I can shoot for the center of the doorway
// RETURNS: Number of doorways found, and array of doorway locations
//-----------------------------------------------------------------------------

// TODO-MUST KLUDGE! remove from Module.cpp!!!!
const int DOOR_SPOTTING_DISTANCE_TENTH_INCHES	=		480; //360;	// Start looking for doors if object closer than this
const int DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES	=	250;	// Minimum clear distance beyond the door
const int DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES	=	(ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES + 20); // Width of robot plus clearance

int FindDoors( int nSamples, int nMaxDoorways, POINT2D_T *pPointArray, DOORWAY_T *pDoorWaysFound )
{
	// just in case another thread happens to check while the function is running (acts as a non-blocking semiphore)
	g_pNavSensorSummary->nDoorWaysFound = 0; 

	int nDoorwaysFound = 0;
	if( nSamples <= 0 )
	{
		return 0; // No doorways found
	}

	int X, Y;
	POINT2D_T RightEdge = {0,0};
	POINT2D_T LeftEdge = {0,0};
	int LastX = 0;
	int LastY = 0;
	bool DebugFindDoors = false;  // enable this for debug

	//int TargetClearDistance = DOOR_SPOTTING_DISTANCE_TENTH_INCHES + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES;
	// int TargetClearDistance = (__max(g_pNavSensorSummary->nRightFrontSideZone, g_pNavSensorSummary->nLeftFrontSideZone) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) ; // tenth inches

	//DOORWAY_T DoorwayFound[MAX_DOORWAYS];	// array of potential multiple doors 
	for( int i=0; i<nMaxDoorways; i++ )
	{
		pDoorWaysFound[i].CenterX = 0;
		pDoorWaysFound[i].Width = 0;
		pDoorWaysFound[i].RightEdge.X = 0;
		pDoorWaysFound[i].RightEdge.Y = 0;
		pDoorWaysFound[i].LeftEdge.X = 0;
		pDoorWaysFound[i].LeftEdge.Y = 0;
	}

	// Track last door edge found (default to first sample, in case door extends off sensor range to the right)
	// Look for the open path/door from right to left
	bool FirstValidSampleFound = FALSE;
	for( int i = 0; i < nSamples; i++ ) 
	{
		X = pPointArray[i].X; // tenth inches  // see WallPoints  MapPoints2D
		Y = pPointArray[i].Y ;

		// Skip any bad values
		if( (X >= LASER_RANGEFINDER_TENTH_INCHES_ERROR) ||
			(Y >= LASER_RANGEFINDER_TENTH_INCHES_ERROR) )
		{
			continue;
		}

		// DEBUG ONLY!!!
		ROBOT_LOG( DebugFindDoors, "DOOR SCAN: x = %d:  y = %d, ", X,Y )

		if( !FirstValidSampleFound )
		{
			if( X < LASER_RANGEFINDER_TENTH_INCHES_MAX )
			{
				LastX = X;
				LastY = Y;
				FirstValidSampleFound = true;
			}
			continue;
		}

		if( 0 == RightEdge.X )
		{ 
			// looking for right edge
			if( LastY < DOOR_SPOTTING_DISTANCE_TENTH_INCHES ) 	// something was in range
			{
				if( (Y > LastY+DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) || // Sudden change in Y distance, at least big enough for robot		
					( (LastX - X) > 120 )  )  // TenthInches - Sudden change in X, indicates that there is a gap that extends beyond sensor range
				{	
					// Found inside of potential door edge! (or may be that the door extends off sensor range to the left)
					RightEdge.X = LastX;	// Use the last valid data
					RightEdge.Y = __min(LastY, DOOR_SPOTTING_DISTANCE_TENTH_INCHES); // Use the last valid data
					ROBOT_LOG( DebugFindDoors, "DOOR SCAN: RIGHT Edge at: x = %d:  y = %d, ", LastX, LastY )
				}
			}
		}
		else
		{
			// Right edge found, looking for Left edge
			if( Y < RightEdge.Y + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES )
			{
				int DoorwayWidth = RightEdge.X - X;
				if( (DoorwayWidth >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES) &&
					(DoorwayWidth <= 420) )
				{
					// Found it!
					LeftEdge.X = X;
					LeftEdge.Y = Y;
					ROBOT_LOG( DebugFindDoors, "DOOR SCAN: LEFT Edge at: x = %d:  y = %d, ", X, Y )

					// Found a Doorway, so save the results
					pDoorWaysFound[nDoorwaysFound].Width = RightEdge.X - LeftEdge.X;
					pDoorWaysFound[nDoorwaysFound].CenterX = RightEdge.X - (DoorwayWidth/2);
					pDoorWaysFound[nDoorwaysFound].LeftEdge.X = LeftEdge.X;
					pDoorWaysFound[nDoorwaysFound].LeftEdge.Y = LeftEdge.Y;
					pDoorWaysFound[nDoorwaysFound].RightEdge.X = RightEdge.X;
					pDoorWaysFound[nDoorwaysFound].RightEdge.Y = RightEdge.Y;

					ROBOT_LOG( TRUE, "FOUND DOOR: Left = %d, %d  Right = %d, %d  Width = %d  Center = %d", 
						LeftEdge.X, LeftEdge.Y, RightEdge.X, RightEdge.Y, 
						pDoorWaysFound[nDoorwaysFound].Width, pDoorWaysFound[nDoorwaysFound].CenterX )

					// clear values to search for next doorway
					RightEdge.X = 0; LeftEdge.X = 0;
					RightEdge.Y = 0; LeftEdge.Y = 0;

					if( ++nDoorwaysFound >= nMaxDoorways)
					{
						ROBOT_LOG( TRUE, "ERROR!  Too Many Doorways Found!" )
						break;
					}
				}
				else
				{
					// Doorway too narrow, restart search
					RightEdge.X = 0;
				}
			}
		}
		LastX = X;
		LastY = Y;

	}	// for loop

	g_pNavSensorSummary->nDoorWaysFound = nDoorwaysFound; // update for other threads 
	return nDoorwaysFound;
}




//-----------------------------------------------------------------------------
// Name: SpeakText
// Desc: queues text to speak and signals text to speech thread
//-----------------------------------------------------------------------------

void SpeakText( CString &TextToSpeak )
{
	#if ( ROBOT_SERVER == 1 ) // Queue only works locally!

	EnterCriticalSection(&g_csSpeakQueue); // Note: only have to guard WRITES, not READS
	//CString *s = new CString(TextToSpeak);

	    g_SpeakQueue.push( TextToSpeak );

	LeaveCriticalSection(&g_csSpeakQueue);

	PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, SPEAK_TEXT_FROM_BUFFER, 0 );

#else
	//	TODO: For Client, somehow copy the text and then do this
	ROBOT_ASSERT( 0 );
#endif

}

void SpeakText( const char *TextToSpeak )
{
	CString s = TextToSpeak;
	SpeakText( s ); // Call the base version
}



//-----------------------------------------------------------------------------
// Name: LaunchCameraApp
// Desc: If enabled, auto-launch the applicaiton that handles camera input
// Done as a separate application, so core app can be run debug, but camera runs optimized release code
// OLD Note: started at different times for Loki or Turtle, since iRobot base enables power only on startup
//-----------------------------------------------------------------------------

// Shut down the OpenCV process
void TerminateCameraApp()
{
	#if (AUTO_LAUNCH_CAMERA_APP == 1)
		TerminateProcess( CameraFWHS.ProcessInfo.hProcess, 0 );
		CloseHandle( CameraFWHS.ProcessInfo.hProcess ); 
		CloseHandle( CameraFWHS.ProcessInfo.hThread ); 
	#endif
}

void LaunchCameraApp()
{
#if (ENABLE_CAMERA_APP == 1)
	int SecondsToWait = 240;
    //size_t iMyCounter = 0, iReturnVal = 0, iPos = 0; 
    DWORD dwExitCode = 0; 
    //std::wstring sTempStr = L""; 
	// If need to pass parameters, see example at: http://www.goffconcepts.com/techarticles/development/cpp/createprocess.html
	// http://msdn.microsoft.com/en-us/library/ms682425(VS.85).aspx


	// Initialize Shared Memory and Event for communicating user requests to the Camera App
	// LPCTSTR g_pCameraRequestSharedMemory = NULL;
	HANDLE hMapFile;
	g_pCameraRequestSharedMemory = NULL;

	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size high
		(sizeof(CAMERA_REQUEST_T)),	// buffer size  
		_T(CAMERA_REQUEST_SHARED_FILE_NAME) );// name of mapping object

	if (hMapFile == NULL) 
	{ 
		ROBOT_LOG( TRUE, "ERROR!  Shared Memory init failed for communicating with Camera App!  Error: %d\n", GetLastError() )
		return;
	}

	g_pCameraRequestSharedMemory = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		(sizeof(CAMERA_REQUEST_T)) );           

	if (g_pCameraRequestSharedMemory == NULL) 
	{ 
		ROBOT_LOG( TRUE, "Could not map view of file (%d).\n", GetLastError() )
		CloseHandle(hMapFile);
		return;
	}

	// Now, create the event for signaling when a request is sent to the Camera app
	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hCameraRequestEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(CAMERA_REQUEST_EVENT_NAME) );
	if ( !g_hCameraRequestEvent ) 
	{ 
		ROBOT_LOG( TRUE, "ERROR!  Camera Request Event creation failed!\n" )
	}
	SetEvent( g_hCameraRequestEvent );  // Indicate to child process that file mapping is setup

	#if (AUTO_LAUNCH_CAMERA_APP == 1)
		///////////////////////////////////////////////////////////////////////////////////////////
		// Launch app if autolaunch enabled.  Disable this, and manually launch the camera app when debugging it
		// CreateProcess API initialization 
		STARTUPINFO StartupInfo; 
		PROCESS_INFORMATION ProcessInfo; 
		memset(&StartupInfo, 0, sizeof(StartupInfo)); 
		memset(&ProcessInfo, 0, sizeof(ProcessInfo)); 
		StartupInfo.cb = sizeof(StartupInfo); 
		memset(&CameraFWHS, 0, sizeof(CameraFWHS)); 

		ROBOT_LOG( TRUE, "\n ==================== Starting Camera App ====================\n" )

		if( !CreateProcess(
			//"C:\\Dev\\Robots\\RobotCamera\\Release\\RobotCamera.exe",	//  __in_opt     LPCTSTR lpApplicationName,
			"C:\\Dev\\Robots\\RobotCamera\\Debug\\RobotCamera.exe",	//  __in_opt     LPCTSTR lpApplicationName,
			"",											//  __inout_opt  LPTSTR lpCommandLine,
			0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpProcessAttributes,
			0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpThreadAttributes,
			false,										//  __in         BOOL bInheritHandles,
			CREATE_DEFAULT_ERROR_MODE,					//  __in         DWORD dwCreationFlags,
			0,											//  __in_opt     LPVOID lpEnvironment,
			//"C:\\Dev\\Robots\\RobotCamera\\Release",	//  __in_opt     LPCTSTR lpCurrentDirectory,
			"C:\\Dev\\Robots\\RobotCamera\\Debug",		//  __in_opt     LPCTSTR lpCurrentDirectory,
			&StartupInfo,								// __in         LPSTARTUPINFO lpStartupInfo,
			&(CameraFWHS.ProcessInfo) )					//  __out        LPPROCESS_INFORMATION lpProcessInformation
		)
		{ 
			/* CreateProcess failed */ 
			ROBOT_DISPLAY( TRUE, "ERROR: CAMERA PROCESS LAUNCH FAILED!  Return Code = %04X", GetLastError() )
		} 
	#endif // (AUTO_LAUNCH_CAMERA_APP == 1)

	// Allow the camera process to get started a bit.  this is not critical...
	Sleep(20);

	// Create Camera App shared memory thread
	g_hCameraAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, CameraAppSharedMemoryIPCThreadProc, (LPVOID)0, 0, &g_dwCameraAppSharedMemoryIPCThreadId );
	ROBOT_LOG( TRUE,  "Created Camera App IPC Thread. ID = (0x%x) (CameraAppSharedMemoryIPCThreadProc)", g_dwCameraAppSharedMemoryIPCThreadId )


/*
	// Wait until child process has created a window
	ROBOT_LOG( TRUE,  "WAITING FOR PROCESS WINDOW...\n" )
	DWORD Result = WaitForInputIdle(
		ProcessInfo.hProcess,	// __in  HANDLE hProcess,
		(DWORD)30000 );					// __in  DWORD dwMilliseconds

	if( 0 == Result )
	{
		ROBOT_LOG( TRUE,  "DONE\n" )
	}
	else
	{
		ROBOT_LOG( TRUE,  "TIMED OUT!\n" )
	}

	// Try PostThreadMessage or PostMessage
	CameraFWHS.hWndFound  = NULL;

	// Enumerate all top level windows on the desktop, and find the itunes one
	EnumWindows ( EnumWindowCallBack, (LPARAM)&CameraFWHS ) ;
	*/

//	SendMessage ( CameraFWHS.hWndFound, Msg, wParam, lParam );
#else
	g_CameraSubSystemStatus = SUBSYSTEM_DISABLED;
#endif  // #if (ENABLE_CAMERA_APP == 1)


}

//-----------------------------------------------------------------------------
// Name: LaunchKobukiApp
// Desc: If enabled, auto-launch the applicaiton that handles Kobuki Base control
// Done as a separate application, so core app can be run debug, while Kobuki only can run in release code (no debug dlls available)
// Note that this must start before access Kinect, since Kobuki Base enables Kinect power only on startup
//-----------------------------------------------------------------------------


// Shut down the KobukiControl process
void TerminateKobukiApp()
{
	#if (AUTO_LAUNCH_KOBUKI_APP == 1)
		TerminateProcess( KobukiFWHS.ProcessInfo.hProcess, 0 );
		CloseHandle( KobukiFWHS.ProcessInfo.hProcess ); 
		CloseHandle( KobukiFWHS.ProcessInfo.hThread ); 
	#endif
}

void LaunchKobukiApp()
{
#if ( (ENABLE_KOBUKI_APP == 1) && ( ROBOT_SERVER == 1 )  && (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL) )

	int SecondsToWait = 240;
    //size_t iMyCounter = 0, iReturnVal = 0, iPos = 0; 
    DWORD dwExitCode = 0; 
    //std::wstring sTempStr = L""; 
	// If need to pass parameters, see example at: http://www.goffconcepts.com/techarticles/development/cpp/createprocess.html
	// http://msdn.microsoft.com/en-us/library/ms682425(VS.85).aspx


	// Initialize Shared Memory and Event for communicating commands to the App
	HANDLE hMapFile;
	g_pKobukiCommandSharedMemory = NULL;

	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size high
		(sizeof(KOBUKI_COMMAND_T)),	// buffer size  
		_T(KOBUKI_COMMAND_SHARED_FILE_NAME) );// name of mapping object

	if (hMapFile == NULL) 
	{ 
		ROBOT_LOG( TRUE, "ERROR!  Shared Memory init failed for communicating with Kobuki App!  Error: %d\n", GetLastError() )
		return;
	}

	g_pKobukiCommandSharedMemory = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		(sizeof(KOBUKI_COMMAND_T)) );           

	if (g_pKobukiCommandSharedMemory == NULL) 
	{ 
		ROBOT_LOG( TRUE, "Could not map view of file (%d).\n", GetLastError() )
		CloseHandle(hMapFile);
		return;
	}

	// Now, create the event for signaling when a command is sent to the Kobuki app
	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hKobukiCommandEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(KOBUKI_COMMAND_EVENT_NAME) );
	if ( !g_hKobukiCommandEvent ) 
	{ 
		ROBOT_LOG( TRUE, "ERROR!  Kobuki Request Event creation failed!\n" )
	}
	SetEvent( g_hKobukiCommandEvent );  // Indicate to child process that file mapping is setup

	#if (AUTO_LAUNCH_KOBUKI_APP == 1)
		///////////////////////////////////////////////////////////////////////////////////////////
		// Launch app if autolaunch enabled.  Disable this, and manually launch the kobuki app when debugging it
		// CreateProcess API initialization 
		STARTUPINFO StartupInfo; 
		PROCESS_INFORMATION ProcessInfo; 
		memset(&StartupInfo, 0, sizeof(StartupInfo)); 
		memset(&ProcessInfo, 0, sizeof(ProcessInfo)); 
		StartupInfo.cb = sizeof(StartupInfo); 
		memset(&KobukiFWHS, 0, sizeof(KobukiFWHS)); 

		ROBOT_LOG( TRUE, "\n ==================== Starting Kobuki Control App ====================\n" )

		if( !CreateProcess(
			"C:\\Dev\\Robots\\KobukiControl\\Release\\KobukiControl.exe",	//  __in_opt     LPCTSTR lpApplicationName,
			"",											//  __inout_opt  LPTSTR lpCommandLine,
			0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpProcessAttributes,
			0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpThreadAttributes,
			false,										//  __in         BOOL bInheritHandles,
			CREATE_DEFAULT_ERROR_MODE,					//  __in         DWORD dwCreationFlags,
			0,											//  __in_opt     LPVOID lpEnvironment,
			"C:\\Dev\\Robots\\KobukiControl\\Release",	//  __in_opt     LPCTSTR lpCurrentDirectory,
			&StartupInfo,								// __in         LPSTARTUPINFO lpStartupInfo,
			&(KobukiFWHS.ProcessInfo) )					//  __out        LPPROCESS_INFORMATION lpProcessInformation
		)
		{ 
			/* CreateProcess failed */ 
			ROBOT_DISPLAY( TRUE, "ERROR: KOBUKI CONTROL PROCESS LAUNCH FAILED!  Return Code = %04X", GetLastError() )
		} 
	#endif // (AUTO_LAUNCH_KOBUKI_APP == 1)

	// Allow the KobukiControl process to get started a bit.  this is not critical...
	Sleep(20); 

	g_hKobukiAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, KobukiAppSharedMemoryIPCThreadProc, (LPVOID)0, 0, &g_dwKobukiAppSharedMemoryIPCThreadId );
	ROBOT_LOG( TRUE,  "Created Kobuki App IPC Thread. ID = (0x%x) (KobukiAppSharedMemoryIPCThreadProc)", g_dwKobukiAppSharedMemoryIPCThreadId )


/*
	// Wait until child process has created a window
	ROBOT_LOG( TRUE,  "WAITING FOR PROCESS WINDOW...\n" )
	DWORD Result = WaitForInputIdle(
		ProcessInfo.hProcess,	// __in  HANDLE hProcess,
		(DWORD)30000 );					// __in  DWORD dwMilliseconds

	if( 0 == Result )
	{
		ROBOT_LOG( TRUE,  "DONE\n" )
	}
	else
	{
		ROBOT_LOG( TRUE,  "TIMED OUT!\n" )
	}

	// Try PostThreadMessage or PostMessage
	KobukiFWHS.hWndFound  = NULL;

	// Enumerate all top level windows on the desktop, and find the itunes one
	EnumWindows ( EnumWindowCallBack, (LPARAM)&KobukiFWHS ) ;
	*/

//	SendMessage ( KobukiFWHS.hWndFound, Msg, wParam, lParam );

#endif  // #if ( (ENABLE_KOBUKI_APP == 1) && ( ROBOT_SERVER == 1 ) )


}

/////////////////////////////////////////////////////////////////////////////
void ReportCommError(LPTSTR lpszMessage, DWORD dwCommError)
{
	CString StrText;
	//DWORD dwLastError = GetLastError();

	char strLastError[1024];
	FormatMessage(
	FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
	NULL,
	dwCommError,
	MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	strLastError,
	1024,
	NULL);

	ROBOT_DISPLAY( TRUE, "********************************************")
	StrText.Format( "Comm Error %04X: %s : %s",  dwCommError, lpszMessage, strLastError);
	ROBOT_DISPLAY( TRUE, StrText )
	ROBOT_DISPLAY( TRUE, "********************************************")

//	StrText.Format( "Comm Error %d \n%s", GetLastError(), lpszMessage);

}

/////////////////////////////////////////////////////////////////////////////
void InitScannerSummaryData( SCANNER_SUMMARY_T* Summary )
{
	Summary->nLeftRearZone =			NO_OBJECT_IN_RANGE;	
	Summary->bLeftCliff =				FALSE;
	Summary->nLeftSideZone =			NO_OBJECT_IN_RANGE;	// 
	Summary->nLeftFrontSideZone =		NO_OBJECT_IN_RANGE;	// 
	Summary->nLeftArmZone =				NO_OBJECT_IN_RANGE;	// Object in front of Arm
	Summary->nLeftFrontZone =			NO_OBJECT_IN_RANGE;
	Summary->nRightFrontZone =			NO_OBJECT_IN_RANGE;
	Summary->nRightArmZone =			NO_OBJECT_IN_RANGE; // Object in front of Arm
	Summary->nRightFrontSideZone =		NO_OBJECT_IN_RANGE;
	Summary->nRightSideZone =			NO_OBJECT_IN_RANGE;
	Summary->bRightCliff =				FALSE;
	Summary->nRightRearZone =			NO_OBJECT_IN_RANGE;	
	//Summary->KinectAngleTenthDegrees =	0;
	Summary->RobotLocation.x =			0;
	Summary->RobotLocation.y =			0;
	Summary->CompassHeading =			0;
	Summary->SampleTimeStamp =			0;

}

/////////////////////////////////////////////////////////////////////////////
// NavSensorSummary
// Processed and summarized sensor data

NavSensorSummary::NavSensorSummary()
{
	InitializeDefaults();
}

bool NavSensorSummary::CliffDetected()
{
	if( bCliffLeft || bCliffFront || bCliffRight || bWheelDropLeft || bWheelDropRight )
	{
		return true;
	}
	return false;
}

bool NavSensorSummary::CliffFront()
{
	if( bCliffFront || (bCliffLeft &&  bCliffRight) )
	{
		return true;
	}
	return false;
}

bool NavSensorSummary::BumperHitFront()
{
	if( bHWBumperFront )
	{
		return true;
	}
	return false;
}


void NavSensorSummary::InitializeDefaults()
{
	// Note, if sensor not installed, override with SENSOR_DISABLED after calling this function!
	// summary
	nFrontObjectDistance =			NO_OBJECT_IN_RANGE;	// Any object in front of and within width of robot, including Arms
	nFrontObjectDirection =			NO_OBJECT_IN_RANGE;	// Right or Left of center.  Negative = Left
	nSideObjectDistance =			NO_OBJECT_IN_RANGE;
	nSideObjectDirection =			NO_OBJECT_IN_RANGE;
	nRearObjectDistance =			NO_OBJECT_IN_RANGE;	
	nRearObjectDirection =			NO_OBJECT_IN_RANGE;	
	nClosestObjectFrontLeft =		NO_OBJECT_IN_RANGE;
	nClosestObjectFrontRight =		NO_OBJECT_IN_RANGE;

	bCliffLeft =					false;
	bCliffFront =					false;
	bCliffRight =					false;
	bWheelDropLeft =				false;
	bWheelDropRight =				false;
	bHWBumperFront =				false;

	// zones
	nLeftRearZone =					NO_OBJECT_IN_RANGE;
	nObjectClawLeft =				NO_OBJECT_IN_RANGE;
	nObjectArmLeft =				NO_OBJECT_IN_RANGE;		// compensated for distance in front of robot - REMOVE
	nLeftSideZone =					NO_OBJECT_IN_RANGE;	
	nLeftFrontSideZone =			NO_OBJECT_IN_RANGE;	
	nLeftArmZone =					NO_OBJECT_IN_RANGE;
	nLeftFrontZone =				NO_OBJECT_IN_RANGE;

	MotionDetectedDirection =		MOTION_DETECTED_NONE; ////// Robot Center

	nRightFrontZone =				NO_OBJECT_IN_RANGE;
	nRightArmZone =					NO_OBJECT_IN_RANGE;	
	nRightFrontSideZone =			NO_OBJECT_IN_RANGE;	
	nRightSideZone =				NO_OBJECT_IN_RANGE;	
	nObjectArmRight =				NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot - REMOVE
	nObjectClawRight =				NO_OBJECT_IN_RANGE;
	nRightRearZone =				NO_OBJECT_IN_RANGE;

	nDoorWaysFound = 0;			// number of doorways spotted somewhere in front of robot
	nBestCenterIndex = 0;		// doorway in the array that is closest to front of robot
	for( int i=0; i<MAX_DOORWAYS; i++ )
	{
		DoorWaysFound[i].CenterX = 0;
		DoorWaysFound[i].Width = 0;
		DoorWaysFound[i].RightEdge.X = 0;
		DoorWaysFound[i].RightEdge.Y = 0;
		DoorWaysFound[i].LeftEdge.X = 0;
		DoorWaysFound[i].LeftEdge.Y = 0;
	}

}

/////////////////////////////////////////////////////////////////////////////
// NavSensorSummary
// Processed and summarized sensor data
FullSensorStatus::FullSensorStatus()
{
	InitializeDefaults();
}


void FullSensorStatus::InitializeDefaults()
{
	// Note, if sensor not installed, override with SENSOR_DISABLED after calling this function!


	// Basic Data
	StatusFlags =				0;			// Typically From Arduino
	LastError =					0;			// Typically From Arduino
	DebugCode =					0;			// Typically From Arduino
	Battery0 =					0;			// Typically From Arduino
	Battery1 =					0;			// Typically From Arduino

	// Cliff and Wheel drops
	CliffFront =				false;
	CliffRight =				false;
	CliffLeft =					false;
	WheelDropRight =			false;
	WheelDropLeft =				false;

	// TODO remove this?
	//Cliff=						0;
	//WheelDrop=					0;


	// Hardware Bumpers, IR range switches, and pressure sensors
	//	int 	HWBumper;					// Typically From Arduino
	//	int 	IRBumper;					// Typically From Arduino
	//	int 	IRBumper2;					// Calculated from Vertical IR detector range
	HWBumperFront =				false;
	HWBumperRear =				false;
	HWBumperSideLeft =			false;
	HWBumperSideRight =			false;

	IRBumperFrontLeft =			false;
	IRBumperFrontRight =		false;
	IRBumperRearLeft =			false;
	IRBumperRearRight =			false;
	IRBumperSideLeft =			false;
	IRBumperSideRight =			false;

	ArmRightBumperElbow =		false;
	ArmLeftBumperElbow =		false;
	ArmLeftBumperFingerLeft =	false;
	ArmLeftBumperFingerRight =	false;
	ArmLeftBumperInsideClaw =	false;

	// TODO - remove this?
	//ArmBumperL =				0;			// Typically From Arduino
	//ArmBumperR =				0;			// Typically From Arduino

	LeftHandRawPressureL =		0;			// Pressure values are only useable if CalibratePressureSensors 
	LeftHandRawPressureR =		0;			// is called before each use.  Use GetPressureLoadPercent to get final value

	// Heading and Odometry
	CompassHeading =			0;			// Typically From Arduino
	//CompassError =				0;			// Typically From Arduino
	OdometerTenthInches =		0.0;		// Typically From Arduino or ER1 Pilot
	OdometerUpdateTenthInches =	0.0;		// Calculated from Arduino or ER1 data
	Tachometer =				0;			// Typically From Arduino or ER1 Pilot
	TachometerTicksL =			0;			// Typically From Arduino, provided feedback for Motor Speed Control
	TachometerTicksR =			0;			// Typically From Arduino, provided feedback for Motor Speed Control
//	TurnAngleUpdate =			0.0;		// Calculated from ER1 motor positions
	DistanceToWaypoint =		0.0;		// Calculated
	CalculatedMotorHeading =	0.0;		// Calculated from motor movements (ER1)
	CurrentLocation.x =			0.0;		// Calculated from compass and odometer
	CurrentLocation.y =			0.0;		// Calculated from compass and odometer
	CurrentLocationMotor.x =	0.0;		// Calculated from motor movements (ER1)
	CurrentLocationMotor.y =	0.0;		// Calculated from motor movements (ER1)
	CurrentLocationGPS.x =		0.0;		// Current location of robot as indicated by GPS
	CurrentLocationGPS.y =		0.0;		// Current location of robot as indicated by GPS

	// Kobuki Dock 
	DockSensorRight =			0;				
	DockSensorCenter =			0;			// IR sensors for the Kobuki Dock
	DockSensorLeft =			0;	

	// From Android Phone
	AndroidConnected =			false;
	AndroidAccEnabled =			false;
	AndroidCommand =			0;			// Commands received from Android phone over Bluetooth
	AndroidUpdatePending =		false;
	AndroidCompass =			0;			// X,Y,Z data received from Android phone over Bluetooth
	AndroidRoll =				0;	
	AndroidPitch =				0;	

	// Other Sensors and state
	PIRMotionLeft =				false;
	PIRMotionRight =			false;
	ThermalPosition =			0;			// TPS - Position of thermal object detected.  Negative = left of center
	TiltAccelX =				0;			// Typically From Arduino.  zero = level
	TiltAccelY =				0;	 		// Typically From Arduino.  zero = level
//	VideoFPS;								// How quickly video frames are processed
	AuxLightsOn =				false;		// Track if the Aux lights are on or off


	// Analog Sensors,  Typically From Arduino
	for( int i=0; i < NUM_US_SENSORS; i++ ) 
		US[i] = NO_OBJECT_IN_RANGE;

	for( int i=0; i < NUM_IR3_SENSORS; i++ ) 
		IR3[i] = NO_OBJECT_IN_RANGE;

	for( int i=0; i < NUM_IR_SENSORS; i++ ) 
		IR[i] = NO_OBJECT_IN_RANGE;

	for( int i=0; i < 9; i++ )							// TPS Thermal sensor - 9 elements
		ThermalArray[i] = NO_OBJECT_IN_RANGE;

}



/////////////////////////////////////////////////////////////////////////////
// Do a sleep, but track when sleeping in Intel Platform Analyzer
#if GPA_TRACE_ENABLED
	void RobotSleep( DWORD msSleepTime, __itt_domain *ThreadDomain )
#else
	void RobotSleep( DWORD msSleepTime, char *ThreadDomain )
#endif
{
	UINT16 SleepTime = (UINT16)msSleepTime;
	__itt_marker(ThreadDomain, __itt_null, psh_Sleep, __itt_marker_scope_task);
	__itt_metadata_add(ThreadDomain, __itt_null, psh_Sleep, __itt_metadata_u16, 1, (void*)SleepTime);

//	__itt_task_begin(ThreadDomain, __itt_null, __itt_null, psh_Sleep);

	Sleep( msSleepTime );

//	__itt_task_end(ThreadDomain);

}
/////////////////////////////////////////////////////////////////////////////
