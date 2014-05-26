#ifndef __MODULE_H__
#define __MODULE_H__

#include "stdafx.h"
#include "RobotType.h"
//#include "../Common/HardwareCmds.h"
//#include "HWInterfaceParams.h"

#include "PathStruct.h"
#include "Globals.h"

//#include "cv.h"
//#include "cvcam.h"

//#include "highgui.h"
//#include "CameraDetectors.h"

#include "Kinect.h"
#include "ArmControl.h"
#include "HeadControl.h"
///#include "LaserControl.h"
#include "LaserScanner.h"
//#include "LaserScannerBehavior.h"
#include "CameraCommon.h"	// For communicating with the RobotCamera OpenCV App

#include "micropather.h"		// required here since Map Doc inherits from MicroPanther class "Graph"
using namespace micropather;

void SimulateHardware( DWORD Cmd, DWORD Param1, DWORD Param2 ); // Implemented in Simulator.cpp

#define ULTRASONIC_TENTH_INCH_PER_STEP			06.74	// .674 inches per step (measured)
#define LASER_FIND_AREA_TENTHDEGREES			   60	// 6 degrees
#define LASER_TRACK_AREA_TENTHDEGREES			   50	// 5 degrees
#define LASER_TRACK_BIAS_TENTHDEGREES			   25	// 2.5 degrees - bias the scan a little closer to the robot
#define LASER_SCAN_DEFAULT_TENTHDEGREES_PER_STEP    5	// tenth degree per vertical scan step
#define LASER_OFFSET_FROM_CAMERA_TENTHDEGREES	  160	// TODO - Get real value for this!!!

#define HUMAN_FACE_TO_TOP_OF_HEAD_OFFSET	70	//TenthInches

#define KINECT_FIND_OBJECTS_REQUEST_CONTINUOUS	 1024	
#define KINECT_FIND_OBJECTS_REQUEST_RETRIES		    3	// Number of frames to view trying to find an object

// Generic strings to mark ITT brackets on state machine case statements
extern __itt_string_handle* pshCase1;
extern __itt_string_handle* pshCase2;
extern __itt_string_handle* pshCase3;
extern __itt_string_handle* pshCase4;
extern __itt_string_handle* pshCase5;
extern __itt_string_handle* pshCase6;
extern __itt_string_handle* pshCase7;
extern __itt_string_handle* pshCase8;
extern __itt_string_handle* pshCase9;


///////////////////////////////////////////////////////////////////////////////
// KINECT STUFF
enum KINECT_POSITIONS { 
		KINECT_SERVO_POSITION_CLOSE_SCAN = 0, 
		KINECT_SERVO_POSITION_MID_SCAN,
		KINECT_SERVO_POSITION_FAR_SCAN,
		KINECT_SERVO_POSITION_CENTER,
		KINECT_SERVO_POSITION_LOOK_UP
};
enum KINECT_SERVO_MOVE_STATUS { 
		KINECT_SERVO_MOVING = 0, 
		KINECT_SERVO_SUCCESS,
		KINECT_SERVO_TIMED_OUT
};
const int KINECT_FLOOR_SCAN_POSITION[] = {
   (  -60 * 10),	//  -58 degrees -  KINECT_SERVO_POSITION_CLOSE_SCAN (closest possible to Robot body)
   (  -40 * 10),	//  -40 degrees -  KINECT_SERVO_POSITION_MID_SCAN - Floor Far Scan
   (  -22 * 10),	//  -22 degrees -  KINECT_SERVO_POSITION_FAR_SCAN - Floor Far Scan
   (  0),			//    0 degrees	- KINECT_SERVO_POSITION_CENTER - looking straight forward
   (   30 * 10)	//  +30 degrees -  KINECT_SERVO_POSITION_LOOK_UP - Looking up at people (not used?)
};


enum KINECT_FLOOR_SCAN_STATES {
		KINECT_FLOOR_SCAN_MOVE_ARMS_STATE	= 1,
		KINECT_FLOOR_SCAN_START_SEARCH_STATE,
		KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE,
		KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE,
		KINECT_FLOOR_SCAN_PICKING_UP_OBJECT_STATE,
};



typedef struct
{
	int		CenterX;	// current average center
	int		CenterY;
	int		Height;
	int		Width;
	int		Length;
	int		NumberOfSlices;
	int		CenterXSum;	// for calculating average of the object
	int		HeightSum;
	int		WidthSum;
	int		StartY;		// for calculating length of the object
	int		EndY;
	int		LastX;		// for aligning slices
	int		LeftPixel;	// For GUI Bounding Box
	int		RightPixel;
	int		StartScanLine;
	int		EndScanLine;

} TEMP_OBJECT_3D_T;	// All values in TenthInches

typedef struct
{
	int						nObjectsDetected;
	TEMP_OBJECT_3D_T		Object[KINECT_SCAN_MAX_3D_OBJECTS];
} TEMP_OBJECT_3D_ARRAY_T;

//#define KINECT_FLOOR_SCAN_POSITION_1		(  -65 * 10)	//  -65 degrees



// Set following to 0 to turn off some debug info
#define	TRACE_DRIVE_CMDS		 0
#define	TRACE_PIC_STATUS		 0


#define ONE_SECOND_DELAY		10	// Assumes 1/10 second count down timer defined in Globals.cpp

#define ACC_MAX_SAMPLES			10	// Number of samples to average for acceleration readings
#define GPS_HISTORY_MAX_SAMPLES  5	// Number of samples to compare to see if traveling in a straight line
#define GPS_ERROR_AMOUNT		(5*12)	// number of inches for GPS Error tolerance
#define GPS_TRAIL_DEGREE_TOLERANCE 10	// Number of degrees that "N" consecutive GPS samples can vary from straight line
#define DONT_STOP_AFTER			 0		// Same as FALSE, tell move to not stop after SetMoveDistance, or SetTurnRotation
#define STOP_AFTER				 1		// Stop after SetMoveDistance (the default)

//#define BACKUP_DIST_HIGH		 01	// Ticks - pre-calculated from TICK_PER_INCH (11.13)
//#define BACKUP_DIST_LOW		 11	// Ticks - 12"
//#define SHORT_DIST_LOW		126	// Ticks - assumes High=0.  About 6"


//#define FWD_SPEED					200	// Set hi bit (0x80) for Forward
//#define REV_SPEED					 85	// 127 = full speed, HighBit 0 = Reverse.
//#define FWD_TURN_SPEED			200	// Set hi bit (0x80) for Forward
//#define REV_TURN_SPEED			 85	

// CAR-TODO Recalculate all these values!
// Assumes forward or reverse slow speed with wheel set at some angle?
//#define TURN_TICKS_180		139	// Ticks for pre-defined turns (number of DEGREES)
//#define TURN_TICKS_90		 70	// pre-calculated from TICK_PER_INCH (11.13)
//#define TURN_TICKS_45		 34	// TICKS_PER_TENTH_INCH defined in HardwareCmds.h
//#define TURN_TICKS_22		 17
//#define TURN_TICKS_10		  8
//#define TICS_PER_DEGREE	   0.77	// For navigation turns


#define MIN_ZOOM_LEVEL		2	// Camera Min Zoom


// Speed to Pulse lookup table
const int  SpeedTable[] = 
//0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15		// Speed
{ 0,  5, 25, 35, 45, 50, 55, 60, 60, 60, 60, 60, 60, 60, 64,100 };	// Pulses

// IR Distance lookup table (non-linear response)  255 = undefined
const int  IRDistanceTable[] = 
//  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15		// Inches
{ 255,255,144,119, 99, 85, 76, 67, 60, 55, 51, 47, 44, 40, 38, 36, 		// Reading
// 16  17  18  19  20  21  22  23  24  25 		// Inches
   34, 32, 31, 30, 30, 29, 28, 27, 25, 24 }; 	// Reading

const int  DistanceTable_IRWideAngle[] = 
//  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24    // Inches
{ 156,156,128,108, 89, 76, 69, 61, 55, 52, 47, 44, 41, 38, 35, 34, 32, 31, 29, 28, 27, 26, 25, 24, 23 }; // Reading

// Less then 31 not reliable - may go up when farther away!
// if value read between 18 and 30, something there just not sure how far away it is
// however, relative values seem to be ok, so should still work for finding way down hallway
// need to test this!

// LD 160,137, 96, 82, 74, 68, 60, 56, 51, 48, 45, 43, 41, 39, 37, 35, 		// Reading
// 16  17  18  19  20  21  22  23  24 		// Inches
//   33, 32, 31, 30, 29, 28, 27, 26, 25}; 	// Reading
// Value > 15 = possible object detected.

const int  DistanceTable_IRLongRange[] =	
//feet 0		                                                   1		                                                   2
//in   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22	 23   24
   { 255, 255, 255, 128, 127, 126, 120, 115, 108, 103,  98,  93,  88,  84,  78,  74,  70,  67,  64,  61,  58,  56,  53,  51,  50,

//feet 		                                                       3		                                                   4
//in       25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44   45   46   47   48
           48,  47,  45,  45,  43,  40,  40,  39,  38,  37,  36,  35,  35,  34,  33,  32,  32,  31,  31,  31,  30,  29,  28,  27 };




//Head133  113   91        61        47        37        31        27 
// Loki Note: Left side generally reads 1-2 higher then right, so ADD 1 to right side to compensate.
// Value > 13 = possible object detected.
// for head, add 4 inches to reading (head set back by 4 inches)

extern  BOOL g_bCmdRecognized;


#if ( ROBOT_SERVER == 1 )	// for Robot Server only
#ifdef USINGOPENCV
void	OnMouseEvent( int event, int x, int y, int flags, void* param );
void	VideoCallBack(IplImage* image);
int		InitCameraVidCap(HWND hwnd, int width, int height, double framerate);
DWORD WINAPI VidCapThreadProc( LPVOID lpParameter );
#endif
DWORD WINAPI KinectDepthThreadProc( LPVOID lpParameter );




#endif


/////////////////////////////////////////////////////////////////////////////
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

class KinectServoControl
{

public:
	int		GetTiltPosition();
	void	SetTiltPosition(  int  NewOwner, int TiltTenthDegrees, int Speed = 0 ); // Speed optional.  0 = don't set
	int		CheckServoPosition( BOOL verbose );
	BOOL	CheckAndSetOwner( int  NewOwner );
	BOOL	IsOwner( int  NewOwner );
	BOOL	ReleaseOwner( int  NewOwner );
	void	SetOwnerTimer( int TimerValue ) { gKinectOwnerTimer = TimerValue; }

protected:
//	int		m_LastObjectDirection;
};
#endif

/////////////////////////////////////////////////////////////////////////////
// SequenceOrder - creates a non-repeating ordered list (zero based)
// for example: SequenceOrder(5) may return: 3,0,2,1,4
// automatically shuffles upon initialization
// returns -1 when list runs out

class CSequenceOrder
{
	public:

		CSequenceOrder( int NumberOfItems );
		~CSequenceOrder();

		int		Next();
		void	Shuffle();

	private:
		int		 m_NumberOfItems;
		int		 m_NextItem;
		int		*m_Items;


};


/////////////////////////////////////////////////////////////////////////////
class CRobotModule
{
public:
	CRobotModule();
	CRobotModule( HANDLE hDevice );

	virtual ~CRobotModule(){};
	void SetSpeedCmd(int NewSpeed);
	void SetTurnCmd(int NewTurn);
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	void DisplayStatus( LPCTSTR lpszError );
	void SpeakCannedPhrase( int PhraseToSpeak );
	void ConvertTenthInchesToTicks( int TenthInches, BYTE &nDistHigh, BYTE &nDistLow );
	//void SendHardwareCmd(BYTE Request, BYTE Index, BYTE Value, BYTE Option1=0, BYTE Option2=0);
	void SendHardwareCmd(WORD Request, DWORD Index, DWORD Value);

	void SendServoCmd(BYTE Request, BYTE Index, BYTE Value, BYTE Option1, BYTE Option2);
	signed int CalculateCollisionDirection();
//	signed int CalculateThreatDirection();

	BOOL ObjectAhead();
//	BOOL ObjectInRange();
//	BOOL ObjectAheadFront();
//	BOOL ObjectAheadAngle();

	CSegmentStruct* m_pCurrentSegment;
	CWaypointStruct* m_pCurrentWaypoint;

};


/////////////////////////////////////////////////////////////////////////////
class CDriveControlModule : public CRobotModule
{
public:
	CDriveControlModule();
	virtual ~CDriveControlModule(){};
//	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );


	int  GetCurrentCommand() { return m_Command; }	// Inline
	int GetCurrentSpeed() { return m_Speed; }	// Inline
	int GetCurrentTurn() { return m_Turn; }	// Inline
	BOOL MoveDistanceCompleted() { return (m_MoveDistanceRemaining == 0); }	// Inline
	BOOL TurnRotationCompleted() { return (m_TurnRotationRemaining == 0); }	// Inline, used for both compass and odometry turns

	void CDriveControlModule::BeginSensorUpdate();
	void CDriveControlModule::EndSensorUpdate();
	BOOL CDriveControlModule::MovementCommandPending();
	void SetCommandPending() { m_CommandPending = TRUE; }	// Inline

	void Brake( int  Module, int Acceleration = ACCELERATION_MEDIUM );
	void Stop( int  Module, int Acceleration = ACCELERATION_MEDIUM );
	void SetSpeed( int  Module, int Speed, int Acceleration = ACCELERATION_MEDIUM );
	void SetTurn( int  Module, int Turn, int Acceleration = ACCELERATION_MEDIUM );
	void SetSpeedAndTurn( int  Module, int Speed, int Turn, int Acceleration = ACCELERATION_MEDIUM );
	BOOL SetMoveDistance( int  Module, int Speed, int Turn, int  DistanceTenthInches, BOOL StopAfterMove = TRUE, int Acceleration = ACCELERATION_MEDIUM );
	void SetMoveDistanceCompleted( int  Module );
	BOOL SetTurnRotation( int  Module, int Speed, int Turn, int TurnAmountDegrees, BOOL StopAfterTurn = TRUE );
	BOOL SetTurnToCompassDirection( int  Module, int Speed, int Turn, int DesiredCompassHeading, BOOL StopAfterTurn );
	void UpdateMoveDistance( double OdometerUpdate );
	void UpdateTurnRotation( double RotationAngleAmount );
	//void SpeedControl();
	int  SetTachometerTarget( int TargetSpeed );
	void BrakeControl();

	int CheckAndSetOwner( int  Module );	// returns MODULE_OWNER_TEST_RESULT_T
	BOOL IsOwner( int  Module );
	BOOL ReleaseOwner( int  Module );
	void SuppressModule( int  Module );
	void EnableModule( int  Module );
	void ExecuteCommand();
	void SetPicAck( int  Command, int  Speed, int  Turn );
	BOOL RobotStopped();

//public:
	//int		m_AutoNavigateMode;

protected:
//	PilotCmd_T	m_PilotCmd;
	BOOL	m_MotorsPaused;
	int 	m_Command;
	BOOL	m_CommandPending;	// a Command is pending execution
	int 	m_LastMotorCommand;
	int		m_LastMotorSpeed;
	int		m_LastMotorTurn;
	int		m_Speed;
	int		m_Turn;
	int		m_Acceleration;
	BYTE	m_DistHigh;
	BYTE	m_DistLow;
	int 	m_DriveOwner;			// Which Module is currently in charge
	int 	m_ModulesSuppressed;	// For Behavior suppression
//	BOOL	m_MoveDistanceCompleted;
	double	m_MoveDistanceRemaining; //TenthInches
	double	m_TurnRotationRemaining;
	int		m_LastCompassHeading;
	BOOL	m_StopAfterMoveDistance;
	BOOL	m_StopAfterTurnRotation;
	int 	m_SpeedRequested;		// For External Servo Speed control
	int 	m_SpeedCurrentSetting;	// For External Servo Speed control
	int 	m_TachometerTarget;		// For External Servo Speed control
	int 	m_BrakeLastTachometer;
	BOOL	m_MotorForward;			// Direction of travel (Fwd/Rev)
	int		m_UserOverrideMode;
	BOOL	m_TrackCompassHeading;
	int		m_TargetCompassHeading;

};



/////////////////////////////////////////////////////////////////////////////
// Modules


// Collision and Object Avoidance State Machine States
#define	IDLE			0x00
#define	BRAKING			0x01
#define	BACKING1		0x02
#define	BACKING2		0x03
#define	TURN_BACKUP1	0x04
#define	TURN_BACKUP2	0x05
#define	TURN_FORWARD1	0x06
#define FORWARD1		0x07
#define	TURNING1		0x08
#define FINAL_MOVE		0x09
#define	AVOIDING		0x0A
#define DISABLED		0xFF	// Module Disabled



/////////////////////////////////////////////////////////////////////////////
class CSensorModule : public CRobotModule
{
public:
	CSensorModule( CDriveControlModule *pDriveControlModule );
	~CSensorModule();


	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	void ProcessSensorStatus(); // different implementations for each robot type
	void SetCompassCorrection( int CompassCorrection );
	void UpdateFromTwoOdometers( double OdometerUpdateTenthInchesL, double OdometerUpdateTenthInchesR);

	int 	ScaleIR( int  nReading );
	int 	ScaleWideIR( int  nReading );
	int 	ScaleWideIRKobuki( int  nReading );
	int 	ScaleLongRangeIRKobuki( int  nReading, int Compensation = 0 );	
	int 	ScaleLongRangeIR( int  nReading, int Compensation = 0 );
	int 	ScaleSRF04UltraSonic( int  nReading );
	int 	ScaleMaxEZ1UltraSonic( int  nReading );
	int 	ScaleMaxEZ1UltraSonicAnalog(int  nReading );

	void	ScaleUltraSonicData( USHORT Length, BYTE*  InBuffer, int * OutBuffer );
	void	ScaleIRArrayData( USHORT Length, BYTE*  InBuffer, int * OutBuffer );

	int		GetObjectDirection( int  SummaryLeft, int  SummaryRight, int  IR_NumberLeft, int  IR_NumberRight, int  IR_SensorRange, int SensorAngle );
	int 	ReadElbowSensorsLeft();
	int 	ReadElbowSensorsRight();
	void	DoSensorFusion();
	void	FuseLaserAndKinectData();
	void	UpdateOdometer();

	#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI
		void	HandleAndroidPhone();
		void	HandleThermalSensor();
		void	HandleAnalogSensors();
	#endif

	void	UpdateLocation();
	void	CalculateSensorPositionOffset( 
				int  SearchDistance, double SensorOffsetDegrees, int  ObjectDistance,	// IN Parameters
				POINT &PositionOffset, int  &OffsetDistance, int  &ObjectType );		// OUT Parameters

	void	FindNearestObject( int y, int x, int  SearchDistance,	// IN Parameters
				FPOINT &NearestObjectDelta, int  &ObjectType );		// OUT Parameters

	void	FindNearestRadarObject( USHORT Length, int * ScaledBuffer );
	int 	CompensateForTilt( int  CompassIn, int  AccelX, int  AccelY );
	void	ConvertAccToTilt( int AccelX, int AccelY, int  &SlopeDir, int  &SlopeAmt);

protected:
	CDriveControlModule *m_pDriveCtrl;
	signed int	m_LastCompassHeading;
	long		m_LastOdometerReadingR;
	long		m_LastOdometerReadingL;
	int			m_MotorPowerEnabled;
	int			m_AccelHistoryX[ACC_MAX_SAMPLES];
	int			m_AccelHistoryY[ACC_MAX_SAMPLES];
	int			m_AccelHistorySample;	// Current Sample in the array

	int			m_GPSHistory[GPS_HISTORY_MAX_SAMPLES];
	int			m_GPSHistorySample;	// Current Sample in the array
	POINT		m_GPSLastSample;
	BOOL		m_GPSNavigationEnabled;
	BOOL		m_LandmarkUpdatesEnabled;
	BOOL		m_CliffSensorsEnabled;
	int			m_CompassCorrection;


//////////////// SERVER ONLY //////////////////
#if ( ROBOT_SERVER == 1 )

	#if ( ROBOT_HAS_RIGHT_ARM == 1 )
		ArmControl *m_pArmControlRight;
	#endif
	#if ( ROBOT_HAS_LEFT_ARM == 1 )
		ArmControl *m_pArmControlLeft;
	#endif

	SCANNER_SUMMARY_T m_LaserSummary;
#endif

};

/////////////////////////////////////////////////////////////////////////////
class CSystemModule : public CRobotModule
{
public:
	CSystemModule( CDriveControlModule *pDriveControlModule );
	~CSystemModule();
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );

protected:
	CDriveControlModule *m_pDriveCtrl;

};

/////////////////////////////////////////////////////////////////////////////
class CCollisionModule : public CRobotModule
{

public:
	CCollisionModule( CDriveControlModule *pDriveControlModule );
	~CCollisionModule();
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );

protected:
	CDriveControlModule *m_pDriveCtrl;
	int 				 m_CollisionState;
	int					 m_CollisionDirection;
	int					 m_RandomTurn;
};

/////////////////////////////////////////////////////////////////////////////
class CUserCmdModule : public CRobotModule
{
public:
	CUserCmdModule( CDriveControlModule *pDriveControlModule );
	~CUserCmdModule();
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	void HandleAndroidInput( );


protected:
	CDriveControlModule	   *m_pDriveCtrl;
	int						m_LocalSpeed;
	int						m_LocalTurn;
	int						m_RemoteSpeed;
	int						m_RemoteTurn;
	//BOOL					m_UserOwnerRequested;
	BOOL					m_AndroidHasMotorControl; // for Android phone contol of motors
	BOOL					m_VidCapProcessingEnabled;
	BOOL					m_IRTrackingEnabled;
	BOOL					m_FaceTrackingEnabled;
	BOOL					m_FaceIdentificationEnabled;
	BOOL					m_ObjectIdentificationEnabled;
	int						m_NextChatPhrase;

	LPCTSTR				 m_pSharedMemory;		// Shared memory for communicating with the Camera App
	HANDLE				 m_hCameraRequestEvent;	// Event for communicating with the Camera App
	CAMERA_REQUEST_T	 m_CameraRequest;
	//UINT				 m_UserOwner;			// switches between LOCAL_USER_MODULE or REMOTE_USER_MODULE if needed.





	// TODO - THIS NEEDED FOR TEMP DEBUG ONLY:
#if ( ROBOT_SERVER == 1 )

	#if ( ROBOT_HAS_RIGHT_ARM == 1 )
		ArmControl *m_pArmControlRight;
	#endif
	#if ( ROBOT_HAS_LEFT_ARM == 1 )
		ArmControl *m_pArmControlLeft;
	#endif

#endif

};


/////////////////////////////////////////////////////////////////////////////
class CBehaviorModule : public CRobotModule
{
public:
	CBehaviorModule( CDriveControlModule *pDriveControlModule );
	~CBehaviorModule();
		
	void ProcessMessage( UINT  uMsg, WPARAM wParam, LPARAM lParam );

protected:
	// Action Modes
	void	ActionFollowPerson();
	void	ActionComeHere(); 
	void	ActionTakePhoto();
	void	ActionMoveWhileTalking();
	void	ActionKarate();
	void	ActionLightSaber();
	void	ActionRunScript();
	void	ActionPickupObjects();
	void	ActionPickupCloseObject();
	void	ActionOpenDoor();
	void	ActionGetBeer();
	void	ActionFindDock();
	int		StartTurnToFaceDock();
	void	ActionTurnToCompassDir( int nCompassRose );
	void	ActionPointToCompassDir( int nCompassRose );
	void	ActionTurnToCompassDegrees( int DesiredCompassHeading );
	void	ActionWhatTimeIsIt();
	void	ActionBadRobot();
	void	ActionFreakOut();
	void	ActionWakeUp();
	void	ActionGoToSleep();
	void	ActionTellJokes( BOOL TellMultipleJokes );
	void	ActionDemoChat();


	// Task Subroutines
	void	TaskRunScript();
	void	TaskDoKarate();	
	void	TaskOpenDoor();
	void	TaskDoLightSaberDemo();
	void	TaskGetLightSaber();

	void	DoHeadNod();	// runs async to tasks

	// Utility Functions
	void	RightArmHome();	
	void	LeftArmHome();	
	void	HeadCenter();
	void	EndActionMode();

	BOOL	ParseScriptFile( char *TextLine );

	void	HandleArmMovementRequest( WPARAM wParam, LPARAM lParam );
	void	HandleArmMovementRequestLeft( WPARAM wParam, LPARAM lParam );
	void	HandleArmMovementRequestRight( WPARAM wParam, LPARAM lParam );
	void	HandleCameraMatchComplete( WPARAM wParam, LPARAM lParam );
	void	HandleArmServoStatusUpdate( WPARAM wParam, LPARAM lParam );
	void	HandleArmServoStatusUpdateLeft( WPARAM wParam, LPARAM lParam );
	void	HandleArmServoStatusUpdateRight( WPARAM wParam, LPARAM lParam );

	void	HandleUserHeadMovementCmd( WPARAM wParam, LPARAM lParam );
	void	CalculatePathToObjectPickup( int ObjectX, int ObjectY, int &DistanceTenthInches, double &DirectionDegrees );


	CDriveControlModule		*m_pDriveCtrl;
	int 					 m_CurrentActionMode;
	DWORD					 m_ActionParam;
	int 					 m_CurrentTask;
	int 					 m_TaskState;
	int 					 m_TurnHandleState;
	int 					 m_NextShoulderPosition;
	int						 m_nObjectsPickedUp;
	int						 m_LedEyeMode;
	int						 m_LedEyeBlinkTimer;
	//int						 m_LedEyeBrightness;
	int 					 m_ArmMovementRight;
	int 					 m_ArmMovementLeft;
	int 					 m_ArmMovementBoth;
	int 					 m_ArmMovementStateRight;
	int 					 m_ArmMovementStateLeft;
	int 					 m_ArmSpeedRight;
	int 					 m_ArmSpeedLeft;
	int						 m_ObjectDetectCount;
	int 					 nKinectObjectsDetected;
	BOOL					 m_ArmWaitForMoveToCompleteRight;
	BOOL					 m_ArmWaitForMoveToCompleteLeft;
	BOOL					 m_KinectSearchComplete;
	int						 m_KinectRetries;
	BOOL					 m_ObjectPickupComplete;
	BOOL					 m_PutObjectInCarryBasket;
	int						 m_nHumanIDToTrack;
	int						 m_RepeatCount; // used when a behavior needs to repeat some action more than once
	int						 m_ResponseReceived; // used to track Yes/No response from person
	BOOL					 m_ResponsePending;
	int						 m_HeadNodState;					 
	int						 m_PhraseToSpeak;
	int						 m_RandomPhrase;
	BOOL					 m_bSpeakingToChild;
	BOOL					 m_SubTaskComplete;
	BOOL					 m_HeadNodEnabled;


	// Scripts
	FILE					*m_ScriptFileHandle;	// File handle for script to execute in Behavior Module
	int						 m_ScriptLineNumber;
	BOOL					 m_HeadExecutePending;
	BOOL					 m_HeadSpeedPending;
	BOOL					 m_LeftArmExecutePending;
	BOOL					 m_LeftArmSpeedPending;
	BOOL					 m_RightArmExecutePending;
	BOOL					 m_RightArmSpeedPending;
	BOOL					 m_WheelExecutePending;
	BOOL					 m_WheelSpeedPending;

#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
	ArmControl				*m_pArmControlRight;
	ArmControl				*m_pArmControlLeft;
	HeadControl				*m_pHeadControl;
	KinectServoControl		*m_pKinectServoControl;
#endif

	DWORD					 m_MaxMoveTimeRight;	// Movement should be completed by this time
	DWORD					 m_MaxMoveTimeLeft;

	int 					 m_HeadMovement;
	int 					 m_HeadMovementState;
	int 					 m_HeadSpeed;
	BOOL					 m_HeadWaitForMoveToComplete;
	//BOOL					 m_PickupObjectsEnabled;
	int						 m_ObjectDistanceTenthInches;
	double					 m_ObjectDirectionDegrees;
	double					 m_ObjectDirectionDegreesPeak; // max turn that was used to get to this object. Used to turn back to find more objects
	int						 m_ObjectX; 
	int						 m_ObjectY; 
	int						 m_PriorObjectX;
	int						 m_PriorObjectY;
//	TEMP_OBJECT_3D_ARRAY_T  *m_pKinectTempObjects3D;
	CSequenceOrder			*m_JokeOrder;



};

/////////////////////////////////////////////////////////////////////////////
class CSpeechRecoModule : public CRobotModule
{
public:
	CSpeechRecoModule( CDriveControlModule *pDriveControlModule );
	~CSpeechRecoModule();
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	void Initialize();


protected:
	CDriveControlModule		*m_pDriveCtrl;
    BOOL fActivateCFG;
    BOOL fActivateDictation;
    BOOL fLoadDictation;
    BOOL fLoadCFG;
    BOOL fActivateSpelling;
    BOOL fLoadSpelling;
//    SPSTATEHANDLE   hDynRule = NULL;

};



/////////////////////////////////////////////////////////////////////////////
class CGridNavModule : public CRobotModule
{
public:
	CGridNavModule( CDriveControlModule *pDriveControlModule, CSensorModule *pSensorModule );
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );


protected:
	CDriveControlModule		*m_pDriveCtrl;
	POINT					m_NextCellLocationRW;
	void					InitVariables();
	void					CancelPath();


	int		m_NavState;			// WayPoint State Machine
	POINT	m_Goal;				// Current goal locaiton (end point of current path)
	BOOL	m_GridPathStarted;
	BOOL	m_PausePath;
	int		m_PauseSpeed;
	int		m_PauseTurn;
	int		m_GoalWayPoint;
	int		m_CurrentWayPoint;
	POINT	m_CurrentLocation;
	int		m_CurrentDirection;
	int		m_NextWayPoint;
	POINT	m_NextLocation;
	int		m_NextDirection;
	int		m_NextDistance;
	


};

/////////////////////////////////////////////////////////////////////////////
class CWayPointNavModule : public CRobotModule
{
	// Path State Machine (for NavModule.cpp)
	// IDLE already define as			0x00
	#define END_OF_PATH_REACHED			0x01
	#define START_NEW_PATH				0x02
	#define GET_NEXT_SEGMENT			0x03
	#define	GET_NEXT_WAYPOINT			0x04
	#define TURN_TO_NEXT_WAYPOINT		0x05	// Default after collision
	#define TURNING_TOWARD_WAYPOINT		0x06
	#define CHECK_COMPASS_ERROR			0x07
	#define SEEKING_LANDMARK_WAYPOINT	0x08
	#define	DRIVING_TO_WAYPOINT			0x09
	#define	DRIVING_TO_DOORWAY			0x0A
	#define	DRIVING_TO_DROPOFF			0x0B
	#define BACKING_UP_FROM_WAYPOINT	0x0C
	#define SCANNING_FOR_CONE1			0x0D
	#define SCANNING_FOR_CONE2			0x0E
	#define ROBOT_BRAKING				0x0F


	// WayPoint constants
	#define NO_WAYPOINT		0
	#define NO_DIRECTION	0
	#define NO_DISTANCE		0


	// Results of LineIntersectTest
	#define	DONT_INTERSECT    0
	#define	DO_INTERSECT      1
	#define COLLINEAR         2


	#define WAYPOINT_TABLE_LEN 20
	#define DIRECT_MOVE			0xFFFF	// Special flag for moving to waypoint

	typedef struct
	{
		BYTE	From;
		BYTE	To;
		DWORD	Next;
	} WAYPOINT_ENTRY_T;


public:
	CWayPointNavModule( CDriveControlModule *pDriveControlModule, CSensorModule *pSensorModule );
	~CWayPointNavModule();
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );

	void	CancelPath();
	int		CheckForObject();
	void	LandmarkUpdateLocation();
	void	InitVariables();
	void	SetSpeedAndTurn( int Speed, int Turn );
	void	SetSpeed( int Speed );
	void	SetTurn(  int Turn );
	POINT	GetWayPointLocation( int TargetWayPoint );
	int		CalculateCameraDirection( int CurrentHeading, int CameraOffset );
	int		CalculateDistanceToWaypoint( FPOINT CurrentLocation, CWaypointStruct* m_pCurrentWaypoint );
	int 	CalculateHeadingToWaypoint( FPOINT CurrentLocation, CWaypointStruct* pTargetWaypoint );
	int		SetTargetWayPoint( int TargetWayPoint );
	int		FindNearestWayPoint();
	int		DegreeToRadian( int degrees );
	int		RadianToDegree( int Radians );
	int		Norm180( int RawAngle );
	int		Norm360( int RawAngle );
	void	CalculateTurnAndDistance( POINT TargetLocation, int Turn, int Distance );
	int		GoToNextWayPoint();
	int		GotoWayPoint( int WayPoint );
	int		GetNextWayPoint( int Current, int Destination );

	int 	LineIntersectTest( 
				long x1, long y1, long x2, long y2,	// First Line
				long x3, long y3, long x4, long y4, // Second Line
				long *Ix, long *Iy );				// Intersection Point (in case calling function cares)


	CWaypointStruct*	GetNextWaypointStruct( CSegmentStruct* pSegmentStruct );
	CSegmentStruct*		GetFirstSegment();
	CSegmentStruct*		GetNextSegment();

	CDriveControlModule *m_pDriveCtrl;
	CSensorModule		*m_pSensorModule;
	HeadControl			*m_pHeadControl;	// For controlling Camera Head servos


protected:
	int		m_NavState;	// WayPoint State Machine
	BOOL	m_NavPathStarted;
	BOOL	m_PausePath;
	int		m_PauseSpeed;
	int		m_PauseTurn;
	int		m_GoalWayPoint;
	int		m_CurrentWayPoint;
	POINT	m_CurrentLocation;
	int		m_CurrentDirection;
	int		m_NextWayPoint;
	POINT	m_NextLocation;
	int		m_NextDirection;
	int		m_NextDistance;
	BOOL	m_WaitForStartSwitch;
//	int		m_NearestRadarObjectDirecton;
//	int		m_NearestRadarObjectRange; 
	int		m_ScanForConeDistance;
	BOOL	m_TrackingCone;
	int		m_Last_Cone_directon;


	POSITION m_NextSegmentPos;
	WAYPOINT_ENTRY_T m_WayPtTbl[WAYPOINT_TABLE_LEN];

};



/////////////////////////////////////////////////////////////////////////////
#define OCCUPANCY_GRID_RESOLUTION		60 // tenth inches
#define OCCUPANCY_GRID_SIZE			LASER_RANGEFINDER_TENTH_INCHES_MAX / OCCUPANCY_GRID_RESOLUTION

class CAvoidObjectModule : public CRobotModule
{

public:
	CAvoidObjectModule( CDriveControlModule *pDriveControlModule );
	~CAvoidObjectModule();

	void					 ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	BOOL					 DetectAndHandleCliff( int &Turn, int &Speed );
	void					 CheckArmSafePosition();
	BOOL					 DetectAndHandleDoorway( int &DoorwayWidth, int &DoorwayCenter);
	int						 FindDoors( int nSamples, int nMaxDoorways, POINT2D_T *pPointArray, DOORWAY_T *pDoorWaysFound );
	void					 HandleKinectPosition();
	BOOL					 BuildWeightedOccupancyGrid();
	int						 RecommendClearestDirection();

#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
	KinectServoControl		*m_pKinectServoControl;
#endif

protected:
	CDriveControlModule *m_pDriveCtrl;
	int		 	m_AvoidanceState;
	int			m_WarningDirection;
	int			m_PriorSpeed;
	int			m_SideAvoidDistanceTenthInches;
	int			m_LastObjectDirection;
	BOOL		m_ArmsInSafePosition;
	BOOL		m_KinectInObjectSpottingPosition;
	BOOL		m_WaitingToMoveKinect;
	int			m_OccupancyGrid[OCCUPANCY_GRID_SIZE][OCCUPANCY_GRID_SIZE];


#if ( ROBOT_SERVER == 1 )

	#if ( ROBOT_HAS_RIGHT_ARM == 1 )
		ArmControl *m_pArmControlRight;
	#endif
	#if ( ROBOT_HAS_LEFT_ARM == 1 )
		ArmControl *m_pArmControlLeft;
	#endif
#endif

};


/////////////////////////////////////////////////////////////////////////////
// class CKinectModule

#define MAX_DEPTH_DATA_SIZE	((640*480)+4)	// 4 bytes per 32 bit value

const int ControlFlag_None = 0;
const int ControlFlag_DisplayBoundingBox = 1;
const int ControlFlag_HidePlayers = 2;
const int ControlFlag_All = 3;				// enables all flags

typedef struct
{
	int	ControlFlags;			// OUT - flags to control behavior of the C# code
	int	BoundingBoxTop;			// OUT
	int	BoundingBoxBottom;		// OUT
	int	BoundingBoxLeft;		// OUT
	int	BoundingBoxRight;		// OUT
	int	FrameNumber;
	int	Height;
	int	Width;
	int	tooFarDepth;	// max depth as reported by Kinect SDK
	int MouseDown;
	int MouseX;
	int MouseY;
	short DepthData[MAX_DEPTH_DATA_SIZE];

} KINECT_DATA_T; // Must match struct in Managed code!


class CKinectModule : public CRobotModule
{

public:

	CKinectModule( CDriveControlModule *pDriveControlModule );
	~CKinectModule();

public:
	void	ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );
	void	FindObjectsOnFloorRequest( int NumberOfTries );		// Queue up requests to find objects on the floor
	void	FindObjectsOnFloor();							// finds 3D objects.  Assumes Kinect pointing down at the floor
	void	FindObjectsInSingleScanLine( int  ScanLine, int NumberOfSamples, OBJECT_2D_ARRAY_T* pKinectObjects2D );
	void	FindWallsAnd2dMaps();
	void	UpdateKinectObjectSummary();
	BOOL	OpenMemoryMappedFile();

	// Helper functions.  Left camera is used for most operations
//	int		GetFrameWidth(IplImage*	pVideoFrame) { return pVideoFrame->width; }
//	int		GetFrameHeight() { return m_pVideoFrameLeft->height; }
//	CvSize	GetFrameSize()	 { return cvGetSize(m_pVideoFrameLeft); }

//	CvPoint	GetFrameCenter(IplImage* pVideoFrame)	// Need to specify Left or Right Frame
//		{ return cvPoint( (pVideoFrame->width /2), (pVideoFrame->height /2) ); }

//	double	GetFrameAspectRatio(IplImage* pVideoFrame)
//		{ return ( (double)pVideoFrame->height/(double)pVideoFrame->width ); }


	// Depth Frame
	void	ShowDepthFrame( );
	void	GetDepthImage();
	RGBQUAD ZValueToColor( int Zvalue ); // for debugging Z Depth on floor


///	void	ProcessVideoFrames( );
	void	GetControlCommand();
	void	MoveDepthWindow();
	void	MoveVideoWindow();
///	void	PositionCamera( int nOwner, CvPoint CameraPanTiltPoint );
///	int		WindowPosX( IplImage* pInputFrame );
///	void	CopyImage( IplImage* pInputFrame, IplImage* pOutputFrame );


public:

	// Mouse Target State
	CvPoint					m_OpenCVDepthMousePoint;
	BOOL					m_OpenCVDepthMousePointSelected;	// OpenCV window in the C++ app
	BOOL					m_KinectWindowMousePointSelected;	// Kinect window in the C# app
	// Image info
	//CvSize					m_CaptureSize; USE: m_FrameInfo->Height
	CvSize					m_DisplaySize;

	// Working Images
	int 					m_FrameNumber;
//	IplImage*				m_pDepthFrame;	// Working copy of current Kinect Depth frame
//	IplImage*				m_pVideoFrame;	// Working copy of current Kinect Video frame
//	IplImage*				m_pDepthDisplayFrame;
//	IplImage*				m_pVideoDisplayFrame;
#if (SHOW_XYZ_WINDOW)

	IplImage*				m_pDebugZFrame;	// Z values frame for debugging
#endif

	// Memory Mapped File
	HANDLE					m_hMapFile;
	int					   *m_pDepthInfoSharedMemory;	// Shared Buffer space LPCTSTR
	KINECT_DATA_T			m_KinectData;
	BOOL					m_bKinectSharedMemoryOpened;
	KINECT_DATA_T		   *m_FrameInfo;

protected:

	HeadControl				*m_pHeadControl; // For telling head to look at humans found
	int 					 m_CurrentTask;
	int 					 m_TaskState;
	int						 m_TrackObjectX;
	int						 m_TrackObjectY;
	int						 m_FindObjectsOnFloorTrys;
	int						 m_nKinect3DObjectsFound;
	UINT					 m_KinectScanPosition;
	BOOL					 m_FindCloseObjectsOnly;

	BOOL					m_ServoMoving;			// Indicate that a head move is in progress
	DWORD					m_BlurSettleTime;		// non-zero indicates the camera is moving

	// Camera VidCap object detection and tracking flags
	BOOL					m_VidCapProcessingEnabled;
	BOOL					m_ObjectTrackingEnabled;
	BOOL					m_ObjectTrackingActive;
	BOOL					m_ObjectToTrackFound;
	CvSize					m_ObjectTrackSize;
	CvPoint					m_ObjectTrackingCenter;
	//DWORD					m_LastCameraMoveTime;	
	SCANNER_SUMMARY_T		m_KinectSummary;		// Summary of objects to avoid detected by Kinect

	// Pointers to Detector Objects
//	CKinectTrackObjectDetector*	m_pTrackObjectDetector;

	CDriveControlModule *m_pDriveCtrl;
//	HeadControl			*m_pHeadControl;	// For controlling Head servos
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
	KinectServoControl		*m_pKinectServoControl;
#endif
	TEMP_OBJECT_3D_ARRAY_T  *m_pKinectTempObjects3D;


///////////////////////////////////////////////////////////////////////
};


/////////////////////////////////////////////////////////////////////////////
/****
class CCameraModule : public CRobotModule
{

public:

	CCameraModule( CDriveControlModule *pDriveControlModule );
	~CCameraModule();

protected:
	CDriveControlModule *m_pDriveCtrl;
	HeadControl			*m_pHeadControl;	// For controlling Head servos

//	int m_nCameraPanTiltCmd;	// User controled Pan/Tilt state
//	int m_nUserCameraPanTiltSpeed;

public:
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );

	// Working Images
	int 					m_FrameNumber;
	IplImage*				m_pVideoFrameLeft;		// Working copy of current LEFT video frame
	IplImage*				m_pVideoFrameRight;		// Working copy of current RIGHT video frame
	IplImage*				m_pKinectDepthFrame;	// Working copy of current Kinect Depth frame
	IplImage*				m_pKinectVideoFrame;	// Working copy of current Kinect Video frame
	IplImage*				m_pMotionFrame;			// for displaying motion view
	IplImage*				m_pColorBlobFrame;		// for displaying color blob view

	IplImage*				m_pDisplayFrameLeft;
	IplImage*				m_pDisplayFrameRight;
	IplImage*				m_pDisplayFrameKinectDepth;
	IplImage*				m_pDisplayFrameKinectVideo;

	// Helper functions  Left camera is used for most operations
//	int		GetFrameWidth(IplImage*	pVideoFrame) { return pVideoFrame->width; }
//	int		GetFrameHeight() { return m_pVideoFrameLeft->height; }
//	CvSize	GetFrameSize()	 { return cvGetSize(m_pVideoFrameLeft); }

	CvPoint	GetFrameCenter(IplImage* pVideoFrame)	// Need to specify Left or Right Frame
		{ return cvPoint( (pVideoFrame->width /2), (pVideoFrame->height /2) ); }

	double	GetFrameAspectRatio(IplImage* pVideoFrame)
		{ return ( (double)pVideoFrame->height/(double)pVideoFrame->width ); }



	// Camera VidCap object detection and tracking flags
	BOOL					m_VidCapProcessingEnabled;
	BOOL					m_FaceTrackingEnabled;
	BOOL					m_ColorTrackingEnabled;
	BOOL					m_ColorTrackingActive;
	BOOL					m_LaserSpotTrackingEnabled;
	BOOL					m_LaserSpotTrackingActive;
	BOOL					m_LaserLineTrackingEnabled;
	BOOL					m_LaserLineTrackingActive;
	BOOL					m_LaserLineFound;
	BOOL					m_ObjectTrackingEnabled;
	BOOL					m_ObjectTrackingActive;
	BOOL					m_ObjectToTrackFound;
	BOOL					m_StereoEnabled;
	BOOL					m_StereoActive;
	BOOL					m_CamShiftTrackingEnabled;
	BOOL					m_ConeTrackingEnabled;	// For Cone SHAPE tracking
	BOOL					m_MotionTrackingEnabled;
	BOOL					m_IRTrackingEnabled;
	BOOL					m_MotionTrackingViewEnabled;
	BOOL					m_HandTrackingEnabled;
	BOOL					m_TakeSnapshot;
	BOOL					m_MotionTrackingViewStarted;

	BOOL					m_MatchObjectEnabled;
	BOOL					m_MatchObjectInFrame;
	BOOL					m_MatchObjectTracking;

	BOOL					m_ShowPropDialog;
	BOOL					m_ShowFormatDialog;

	// Video Recording mode
	int 					m_VideoRecordMode;
	CvVideoWriter			*m_pVideoWriter;


	// Color Blob Tracking
	int						m_ColorSearchingState;
	int						m_ScanMode;
	int						m_FrameRetryCount;
	int						m_nPScanPosLeft;
	int						m_nPScanPosRight;
	int						m_ColorScanningEnabled;
	DWORD					m_ServoDelayTime;		// target system tic count, in ms


	// Mouse Target State
	CvPoint					m_VideoMousePoint;
	BOOL					m_VideoMousePointSelected;

	// Pointers to Detector Objects
	CFaceDetector*			m_pFaceDetector;
	CMotionDetector*		m_pMotionDetector;
	CCamShiftDetector*		m_pCamShiftDetector;
	CColorDetector*			m_pColorDetector;
	CLaserSpotDetector*		m_pLaserSpotDetector;
	CLaserLineDetector*		m_pLaserLineDetector;
	CTrackObjectDetector*	m_pTrackObjectDetector;
	CStereoDetector*		m_pStereoDetector;
	CMatchObjectDetector*	m_pMatchObjectDetector;


public:
	BOOL					m_HeadMoving;			// Indicate that a head move is in progress
	DWORD					m_BlurSettleTime;		// non-zero indicates the camera is moving
	DWORD					m_MotionDetectorTime;	// non-zero indicates detector should delay for settle time
	DWORD					m_StereoDetectorTime;	// Time to wait between stereo frames
	DWORD					m_FaceLostTime;			// last time we saw a face - how long has it been?
	int						m_CurrentZoomLevel;		// Current zoom level.  g_CameraZoomPos is desired zoom level.

	// VidCap Data
//	CvSize					m_FrameSize;
//	CvPoint					m_CameraFrameCenter;	// Center of frame (used as target to position camera)
//	double					m_FrameAspectRatio;
//	int						m_WindowPosX;			// Horizontal position for most VidCap display windows
	
	int 					m_FaceTrackingState;
	int 					m_CamShiftState;
	int 					m_IdleBehavior;			// What to do when not tracking an object
	int 					m_FramesSinceMotionLost;
	// Face Detector Data			
	int						m_FaceTrackingFrameCount;
	CvPoint					m_FaceTrackCenter;		// current face-location estimate
	CvSize					m_FaceTrackSize;
	CvPoint					m_LastFaceCenter;		// Locaton of where the last face was found
	CvPoint					m_LastMotionCenter;
	int						m_RandomHeadCounter;	// timer for determing when to center head after random movements

	// Object Tracker			
	CvPoint					m_ObjectTrackingCenter;			// Center of motion in the image

	// Face Detector Data			
	CvPoint					m_MotionCenter;			// Center of motion in the image

	// IR Head tracking
	CvPoint					m_IRTrackCenter;		// Use IR sensors to track objects

	CvPoint					m_PIR_Position;			// Position PIR Motion sensors indicate (body mounted)
	
	// CamShift Data
//	CvRect					m_prevFaceRect;			// location of face in previous frame
	CvPoint					m_CamShiftCenter;		// current face-location estimate from CamShift
//	int						nFrames;
	CvBox2D					m_CamShiftBox;			// current face-location estimate
	CvPoint					m_ColorBlobCenter;		// center of ColorBlob in frame

	CvPoint					m_LaserSpot;			// Location of curr

	// Video Capture Functions
	void	InitVidCap();
//	void	ReleaseVidCap();
	void	InitWorkingImagesLeft( IplImage* pInputFrame );
	void	ReleaseWorkingImagesLeft();

	void	InitWorkingImagesRight( IplImage* pInputFrame );
	void	ReleaseWorkingImagesRight();

	void	InitKinectDepthWorkingImages( IplImage* pInputFrame );
	void	ReleaseKinectDepthWorkingImages();

	void	InitKinectVideoWorkingImages( IplImage* pInputFrame );
	void	ReleaseKinectVideoWorkingImages();

	void	InitNewDetectors( );
//	void	GetVideoFrame( IplImage* pVideoFrame );
	void	ProcessVideoFrames( );	// Process Left and Right Frames
	void	GetControlCommand();
	void	SaveSnapshot();


	void	CameraIdleBehavior();
	void	TrackMotion();

	void	ScanForColorBlob();
	void	CenterCamera();
	void	BeginScanningMode();
	void	CheckPositionLimits( int &NewPan, int &NewTilt);
	void	CheckTrackingPositionLimits( int &NewPan, int &NewTilt);
	void	PanTiltRelative( int TenthDegreeMoveX, int TenthDegreeMoveY );



	void	MoveWindows();
	void	ReleaseAllDetectors();
	void	PositionCamera( int nOwner, CvPoint CameraPanTiltPoint );
	void	ZoomCameraToWindowSize( CvSize WindowSize );
	void	ZoomIn( int  ZoomDelta );	// Increment Zoom level by amount specified
	void	ZoomOut( int  ZoomDelta );	// Decrement Zoom level by amount specified
	void	ZoomAbs( int  nZoomLevel );	// Zoom to level specified



///////////////////////////////////////////////////////////////////////
};
***/

/*
class CServoModule : public CRobotModule
{
public:
	CServoModule( HANDLE hServoCommPort );
	void SendServoCmd(char *CameraBytes, int nByteCount);
	void ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam );

protected:
	HANDLE m_hServoCommPort;

};
*/

#endif
