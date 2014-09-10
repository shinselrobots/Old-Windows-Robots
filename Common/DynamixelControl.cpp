// DynamixelControl.cpp
// Interface for Dynamixel Digital Robot Servos for LOKI robot
// written by Dave Shinsel (2008/9) - feel free to use any way you want


// This class is used by the thread loop that reads commands from the queue
// in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include "thread.h"
//#include "module.h"
#include "Globals.h"
#include "DynamixelControl.h"
//#include "../Common/HardwareCmds.h" // for TICS_PER_INCH
//#if ( ROBOT_SERVER == 1 )  // Nothing in this file used for client!

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define TEST_SIDETILT	0

//------------------------------------------------------------------------------
//                DEBUG SWITCHES
//------------------------------------------------------------------------------
//#define DEBUG_DYNA_COMMAND_DUMP   // Dump all commands
//#define DEBUG_DYNA_SHOW_ACK_MESSAGES
#define DEBUG_DYNA_SHOW_STATUS_MESSAGES		0
#define DEBUG_DYNA_SHOW_STATUS_HEAD			0
#define DEBUG_DYNA_SHOW_STATUS_ARM_RIGHT	0
#define DEBUG_DYNA_SHOW_STATUS_ARM_LEFT		0
#define DEBUG_DYNA_SHOW_STATUS_KINECT		0
//#define DEBUG_DYNA_TIMING	// Show how long each step of processing is taking
//#define DEBUG_DYNA_READWRITE_TIME  // Log Read/Write SIO latency


// Servo Offset Compensation

#define CAMERA_PAN_TENTH_DEGREES_ZERO			   (0) //(-15)		// TenthDegrees!
#define CAMERA_TILT_TENTH_DEGREES_ZERO				(0) // (35)		// TenthDegrees!
#define CAMERA_SIDETILT_TENTH_DEGREES_ZERO			 (0)		// TenthDegrees!

#if (ROBOT_TYPE == LOKI)
	#define KINECT_TILT_TENTH_DEGREES_ZERO				(-140)		// TenthDegrees! - Trim this to make Kinect level!
#elif (ROBOT_TYPE == TURTLE)
	#define KINECT_TILT_TENTH_DEGREES_ZERO				(-110)		// TenthDegrees! - Trim this to make Kinect level!
#else
	#error BAD ROBOT_TYPE!  
#endif

// Calibrate at elbow - 90 degrees, all others at zero
//#define RIGHT_ARM_SHOULDER_TENTH_DEGREES_ZERO					// Handled in KerControl.cpp
#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
	#define RIGHT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-62*10)	// TenthDegrees! RX64 Elbow offset to allow Home2 bend
#else
//	#define RIGHT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-20*10)	// TenthDegrees! MX64 Elbow offset to allow Home2 bend
	#define RIGHT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-69*10)	// TenthDegrees! MX64 Elbow offset to allow Home2 bend
#endif
#define RIGHT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO	 ( -5*10)	// TenthDegrees!
#define RIGHT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO	 ( 0*10)	// TenthDegrees!
#define RIGHT_ARM_CLAW_TENTH_DEGREES_ZERO			 ( 16*10)	// TenthDegrees!

//#define LEFT_ARM_SHOULDER_TENTH_DEGREES_ZERO					// Handled in KerControl.cpp
#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
	#define LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-42*10)	// TenthDegrees! RX64 Elbow offset to allow Home2 bend
#else
	#define LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-68*10)	// TenthDegrees! MX64 Elbow offset to allow Home2 bend
//	#define LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO		 (-52*10)	// TenthDegrees! MX64 Elbow offset to allow Home2 bend
#endif
#define LEFT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO	 (-5*10)	// TenthDegrees!
#define LEFT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO	 (0)//(  0 )		// TenthDegrees!
#define LEFT_ARM_CLAW_TENTH_DEGREES_ZERO			 (-84*10)	// TenthDegrees!



/***************** FROM THE MANUAL:
 However, unlike the Alarm LED, after returning to a normal condition, it maintains the torque off status.
To recover, the Torque Enable (Address0X18) needs to be reset to 1.
********************/

/***********************************************************
// command packet sent to Dynamixel servo:
// 0      1      2    3        4            4 + data-length
// [0xFF] [0xFF] [id] [length] [...data...] [checksum]
typedef struct DynaCmd_struct {
	BYTE Header1;
	BYTE Header2;
	BYTE ID;
	BYTE Length;
	BYTE Instruction
	BYTE Data[MAX_DATA_BYTES]; // (includes CheckSum after the data)
} DynaCmd_T;

// status packet returned from Dynamixel servo:
// 0      1      2    3        4       5            5 + data-length
// [0xFF] [0xFF] [id] [length] [error] [...data...] [checksum]
typedef struct DynaReply_struct {
	BYTE Header1;
	BYTE Header2;
	BYTE ID;
	BYTE Length;
	BYTE Error;
	BYTE Data[MAX_DATA_BYTES]; // (includes CheckSum after the data)
} DynaReply_T;
***********************************************************/

// 		MsgString.Format( "%s: New Brake Command Requested",ModuleName );
//		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )



///////////////////////////////////////////////////////////////////////////////
// Dynamixel Servo Control Class
///////////////////////////////////////////////////////////////////////////////


CDynaControl::CDynaControl()
{
	m_DynaCmd.Header1 = 0xFF;
	m_DynaCmd.Header2 = 0xFF;
	m_DynaCmd.ID = 0;			// Address of Servo to access
	m_DynaCmd.Length = 0;		
	m_DynaCmd.Instruction = 0;		
	memset(m_DynaCmd.Data,0,DYNA_MAX_DATA_BYTES);

	m_DynaReply.ID = 0;
	m_DynaReply.Length = 0;
	m_DynaReply.Error = 0;
	memset(m_DynaReply.Data,0,DYNA_MAX_REPLY_BYTES);
	memset(m_BulkServoCmdCopy, 0, (sizeof(BULK_SERVO_CMD_T))* NUMBER_OF_SMART_SERVOS+1);

	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
	m_pHeadControl = new HeadControl();
	DynaCmdStartTime = 0;
	m_PowerEnabled = TRUE; //enable power on start up

//	memset(m_ReadBuf,0,READ_BUF_SIZE);
}

CDynaControl::~CDynaControl()
{
	SAFE_DELETE(m_pArmControlRight);
	SAFE_DELETE(m_pArmControlLeft);
	SAFE_DELETE(m_pHeadControl);
}


///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////

void CDynaControl::Init()
{
	DynaCmdStartTime = GetTickCount();
	// just to make sure, reset everything to defaul values
	m_DynaCmd.Header1 = 0xFF;
	m_DynaCmd.Header2 = 0xFF;
	m_DynaCmd.ID = 0;			// Address of Servo to access
	m_DynaCmd.Length = 0;		
	m_DynaCmd.Instruction = 0;		
	memset(m_DynaCmd.Data,0,DYNA_MAX_DATA_BYTES);

	m_DynaReply.ID = 0;
	m_DynaReply.Length = 0;
	m_DynaReply.Error = 0;
	memset(m_DynaReply.Data,0,DYNA_MAX_REPLY_BYTES);

	//************************************************
	//To change a servo's ID, uncomment out this code and run one time.
	// Use 0xFE (Broadcast ID) to just do whatever servo is connected.
	
	/*int  OldServoID = 17;	// 0xFE; // <<< set to old ID or 0xFE if only one servo connected!
	int  NewServoID = 14; // <<< Put new ID here!
	ChangeServoID( OldServoID, NewServoID );
	Sleep(3000);
	GetServoStatus( NewServoID );	
	ROBOT_LOG( TRUE,"Done Changing Servo ID.  STOPPING DYNA THREAD!\n")
	while(1) { Sleep(1000); }
	*/
	//************************************************

//	SetAllCameraServosMaxTorque(	3 * (DYNA_TORQUE_LIMIT_MAX/16) );	// Set to value between 1 and 16
//	SetAllCameraServosSpeed(		3 * (DYNA_MOVING_SPEED_MAX /16) );

	// Set Compliance of Laser Tilt servo to "med", for accurate laser scanner control
	// SetServoComplianceSlope( DYNA_KINECT_SCANNER_SERVO_ID, DYNA_COMPLIANCE_MED_TIGHT );

	// Put Camera Head and arms "startup" Position, with Torque applied 
	EnableCameraServosTorque( TRUE );
	EnableRightArmTorque( TRUE );
	EnableLeftArmTorque( TRUE );

	// Set torque limits for all servos
	ROBOT_DISPLAY( TRUE, "NOTE: DYNA Servos SET TO NORMAL TORQUE! (%d)", DYNA_TORQUE_LIMIT_NORMAL )

	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
	{
		SetServoMaxTorque( DYNA_CAMERA_SIDETILT_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
	{
		SetServoMaxTorque( DYNA_CAMERA_TILT_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
	{
		SetServoMaxTorque( DYNA_CAMERA_PAN_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	
	}

	if( LOKI == ROBOT_TYPE )
	{
		SetServoMaxTorque( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	// 0 - 1023	
		SetServoMaxTorque( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	
		SetServoMaxTorque( DYNA_RIGHT_ARM_WRIST_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);
		SetServoMaxTorque( DYNA_RIGHT_ARM_CLAW_SERVO_ID,  MED_TORQUE);	// 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)	

		SetServoMaxTorque( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID,  DYNA_TORQUE_LIMIT_NORMAL);
		SetServoMaxTorque( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);	
		SetServoMaxTorque( DYNA_LEFT_ARM_WRIST_SERVO_ID, DYNA_TORQUE_LIMIT_NORMAL);
		SetServoMaxTorque( DYNA_LEFT_ARM_CLAW_SERVO_ID,  MED_TORQUE);	// 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)	
	}

	GotoStartupPosition();
	RobotSleep(500, pDomainSmartServoThread);
	GetAllCameraServosStatus();

	// Set shutdown conditions for all servos
	/* 
	// NORMAL FOR LOKI IS 0x04 - Torque off for Overheating only!
	Bit 7 0
	Bit 6 If set to 1, torque off when an Instruction Error occurs
	Bit 5 If set to 1, torque off when an Overload Error occurs
	Bit 4 If set to 1, torque off when a Checksum Error occurs
	Bit 3 If set to 1, torque off when a Range Error occurs
	Bit 2 If set to 1, torque off when an Overheating Error occurs
	Bit 1 If set to 1, torque off when an Angle Limit Error occurs
	Bit 0 If set to 1, torque off when an Input Voltage Error occurs
	*/
	BYTE ShutdownFlags = 0;	// Never shut servo down.  Handle in code instead.
/*
	THIS IS AN ERROR.  ONLY DO THESE ONCE TO WRITE THE SERVO ROM!
	// USE Dynamixel Configurator INSTEAD!
	SetShutdownConditions( DYNA_CAMERA_PAN_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_CAMERA_TILT_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_CAMERA_SIDETILT_SERVO_ID, ShutdownFlags );

	SetShutdownConditions( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_RIGHT_ARM_WRIST_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_RIGHT_ARM_CLAW_SERVO_ID, ShutdownFlags );

	SetShutdownConditions( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_LEFT_ARM_WRIST_SERVO_ID, ShutdownFlags );
	SetShutdownConditions( DYNA_LEFT_ARM_CLAW_SERVO_ID, ShutdownFlags );
*/

	// Set the torque of the hand grip
	//SetServoMaxTorque( DYNA_RIGHT_ARM_CLAW_SERVO_ID, 200 );	// 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)	
	//SetServoMaxTorque( DYNA_LEFT_ARM_CLAW_SERVO_ID, 200 );	// 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)	



	ROBOT_DISPLAY( TRUE, "Dyna Servos initialized" )

}

void CDynaControl::EnableCameraServosTorque( BOOL bEnable )
{
	// Enable or disable torque for all Camera Head servos
	// Also enables Kinect servo

	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
	{
		EnableServoTorque( DYNA_CAMERA_SIDETILT_SERVO_ID, bEnable );	
		g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Speed = SERVO_SPEED_SLOW;
		g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Enable = bEnable;
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
	{
		EnableServoTorque( DYNA_CAMERA_TILT_SERVO_ID, bEnable );	
		g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Speed = SERVO_SPEED_SLOW;
		g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Enable = bEnable;
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
	{
		EnableServoTorque( DYNA_CAMERA_PAN_SERVO_ID, bEnable );	
		g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Speed = SERVO_SPEED_SLOW;
		g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Enable = bEnable;
		g_CameraServoTorqueEnabled = bEnable;
	}

	// Now do Kinect Scanner
	if( ROBOT_HAS_KINECT_SERVO )
	{
		EnableServoTorque( DYNA_KINECT_SCANNER_SERVO_ID, bEnable );	
		g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Speed = SERVO_SPEED_SLOW;
		g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Enable = bEnable;
	}
}

void CDynaControl::EnableRightArmTorque( BOOL bEnable )
{
	// Enable or disable torque for all Right Arm Servos

	if( LOKI == ROBOT_TYPE )
	{
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_RIGHT_ARM_WRIST_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_RIGHT_ARM_CLAW_SERVO_ID, bEnable );
	}
}

void CDynaControl::EnableLeftArmTorque( BOOL bEnable )
{
	// Enable or disable torque for all Left Arm Servos

	if( LOKI == ROBOT_TYPE )
	{
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_LEFT_ARM_WRIST_SERVO_ID, bEnable );

		g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Speed = SERVO_SPEED_MED_SLOW;
		g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable = bEnable;
		EnableServoTorque( DYNA_LEFT_ARM_CLAW_SERVO_ID, bEnable );
	}
}


void CDynaControl::SetAllCameraServosSpeed( int  Speed )
{
	// Set the speed for all Camera Head servos

	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
		SetServoSpeed( DYNA_CAMERA_SIDETILT_SERVO_ID, Speed );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
		SetServoSpeed( DYNA_CAMERA_TILT_SERVO_ID, Speed );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
		SetServoSpeed( DYNA_CAMERA_PAN_SERVO_ID, Speed );

	if( 1 == ROBOT_HAS_KINECT_SERVO )
		SetServoSpeed( DYNA_KINECT_SCANNER_SERVO_ID, Speed );	

}

void CDynaControl::SetAllCameraServosMaxTorque( int  MaxTorque )
{
	// Set the Max Torque for all Camera Head servos
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
		SetServoMaxTorque( DYNA_CAMERA_SIDETILT_SERVO_ID, MaxTorque );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
		SetServoMaxTorque( DYNA_CAMERA_TILT_SERVO_ID, MaxTorque );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
		SetServoMaxTorque( DYNA_CAMERA_PAN_SERVO_ID, MaxTorque );
	if( 1 == ROBOT_HAS_KINECT_SERVO )
		SetServoMaxTorque( DYNA_KINECT_SCANNER_SERVO_ID, MaxTorque );	

}

void CDynaControl::GetAllCameraServosStatus( )
{
	// Get the status for all Camera Head servos
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
		GetServoStatus( DYNA_CAMERA_SIDETILT_SERVO_ID );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
		GetServoStatus( DYNA_CAMERA_TILT_SERVO_ID );	
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
	{
		GetServoStatus( DYNA_CAMERA_PAN_SERVO_ID );	
		ROBOT_LOG( TRUE, "\n============= Dumping Status of all Camera Servos =============\n" )
	}
	if( 1 == ROBOT_HAS_KINECT_SERVO )
		GetServoStatus( DYNA_KINECT_SCANNER_SERVO_ID );	

}


void CDynaControl::PauseMotion( BOOL bPause )
{
	// Stop or resume motion for all servos
	if( bPause )
	{
		ROBOT_LOG( TRUE, "\n============= Pausing all Servo motion =============\n" )

		// send speed command to all servos using BroadcastID
		int Speed = DYNA_VELOCITY_STOP;

		int  SendParams = 3;
		m_DynaCmd.ID = DYNA_ID_BROADCAST;
		m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
		m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
		m_DynaCmd.Data[0] = DYNA_REG_MOVING_SPEED;
		m_DynaCmd.Data[1] = (BYTE)(Speed & 0xFF);	
		m_DynaCmd.Data[2] = (BYTE)(Speed >> 8);

		// Send comamnd to the servo
		SendCmd( SendParams, 0, DYNA_MULTI_SERVO_ID );	// number of send params, receive optional params, ServoID

	}
	else
	{
		ROBOT_LOG( TRUE, "\n============= Resuming all Servo motion =============\n" )

		// Restore speed and position values to all servos - resume normal movement
		HandleCommand( HW_SET_BULK_ARM_POSITION, RIGHT_ARM, TRUE, 0, 0 );

		//SetAllCameraServosSpeed (DYNA_VELOCITY_SLOW);
		//SetServoSpeed( DYNA_KINECT_SCANNER_SERVO_ID, 150 );	// Max = 0x03FF = 1023

	}
}

void CDynaControl::CameraStop( )
{
	// Stop motion for camera head Pan/Tilt servos
	ROBOT_LOG( TRUE, "\n============= Stopping Camera Head Servo motion =============\n" )

	SetAllCameraServosSpeed( DYNA_VELOCITY_STOP );
	Sleep(10);

	// Now, set target position to current position
	if( (DYNA_CAMERA_PAN_SERVO_ID < 100) && g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Enable )
	{
		GetServoStatus( DYNA_CAMERA_PAN_SERVO_ID );
		g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees = g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees;
		int PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees ); 
		SetServoPosition( DYNA_CAMERA_PAN_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}

	if( (DYNA_CAMERA_TILT_SERVO_ID < 100) && g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Enable )
	{
		GetServoStatus( DYNA_CAMERA_TILT_SERVO_ID );
		g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees = g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees;
		int PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees ); 
		SetServoPosition( DYNA_CAMERA_PAN_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}

}



void CDynaControl::AddCheckSum(int  nParamBytes)
{
	// Calculate Checksum, and insert it into the packet
	// Usage: 	DoCheckSum(nParamBytes);

	int  CheckSum = m_DynaCmd.ID;
	CheckSum += m_DynaCmd.Length;
	CheckSum += m_DynaCmd.Instruction;

	for ( int  i=0 ; i<nParamBytes ; i++ )
	{
		CheckSum += m_DynaCmd.Data[i];
	}
	CheckSum = (~CheckSum);
	CheckSum = CheckSum & 0x00FF;	// save lower 8 bits only
	m_DynaCmd.Data[nParamBytes] = (BYTE)CheckSum;	// Notice this is lower 8 bits only
}


void CDynaControl::GotoSleepPosition() // Sleeping Position
{
	// Put all servos in the "Rest" Position
	int  PositionTicks = 0;
	ROBOT_DISPLAY( TRUE, "Moving Arms to sleep position" )
	if( ROBOT_TYPE != LOKI )
	{
		if( 1 == ROBOT_HAS_KINECT_SERVO )
		{
			SetServoSpeed( DYNA_KINECT_SCANNER_SERVO_ID, 100 );	// Max = 0x03FF = 1023
			//SetAllCameraServosSpeed(DYNA_VELOCITY_MED_SLOW); // Max = 0x03FF = 1023
			PositionTicks = TenthDegreeToTicks( DYNA_KINECT_SCANNER_SERVO_ID, KINECT_SLEEP_POSITION ); // NOT: KINECT_TILT_CENTER
			SetServoPosition( DYNA_KINECT_SCANNER_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
		}
	}
	else
	{
		SetAllCameraServosSpeed(DYNA_VELOCITY_MED_SLOW); // Max = 0x03FF = 1023

		// Point head forward before tilting down to sleep
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, CAMERA_PAN_CENTER );
		SetServoPosition( DYNA_CAMERA_PAN_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_SIDETILT_SERVO_ID, CAMERA_SIDETILT_CENTER );
		SetServoPosition( DYNA_CAMERA_SIDETILT_SERVO_ID, PositionTicks );	// ID, position (0-03FF)

		if( 1 == ROBOT_HAS_KINECT_SERVO )
		{
			PositionTicks = TenthDegreeToTicks( DYNA_KINECT_SCANNER_SERVO_ID, KINECT_SLEEP_POSITION ); // NOT: KINECT_TILT_CENTER	
			SetServoPosition( DYNA_KINECT_SCANNER_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
		}

		/////////////////////////////////////////////////////////////////////////////
		// Set the arms to sleep position
		// Make sure g_BulkServoCmd block stays in sync with where the hardware is
		int  ServoIDArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// IDs Servos
		int  ServoValueArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Values to set
		int  ServoSpeedArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Speed to set

		int  DynaServoSpeed = ServoSpeedToDynaServoSpeed( SERVO_SPEED_MED );

		///////////////////////////////
		// Right Arm
		// Only move if all servos are enabled

		if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable )
		{

			// Handle possible RX64 Servo Separately
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_ELBOW_BEND_HOME2_LOCK * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, RIGHT_ARM_ELBOW_BEND_HOME2_LOCK * 10 ); 
			SetServoPositionAndSpeed( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, PositionTicks, DynaServoSpeed );

			// Now handle AX12 Servos
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_ELBOW_ROTATE_HOME2 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, RIGHT_ARM_ELBOW_ROTATE_HOME2 * 10 ); 
			ServoIDArray[0] = DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID;															// ID
			ServoValueArray[0] = PositionTicks;																				// Position (0-03FF)
			ServoSpeedArray[0] = DYNA_VELOCITY_MED;	// Speed

			g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_WRIST_ROTATE_HOME2 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_WRIST_SERVO_ID, RIGHT_ARM_WRIST_ROTATE_HOME2 * 10 ); 
			ServoIDArray[1] = DYNA_RIGHT_ARM_WRIST_SERVO_ID;	// ID
			ServoValueArray[1] = PositionTicks;
			ServoSpeedArray[1] = DYNA_VELOCITY_MED;

			g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_CLAW_HOME2 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_CLAW_SERVO_ID, RIGHT_ARM_CLAW_HOME2  * 10 ); 
			ServoIDArray[2] = DYNA_RIGHT_ARM_CLAW_SERVO_ID;	// ID
			ServoValueArray[2] = PositionTicks;
			ServoSpeedArray[2] = DYNA_VELOCITY_MED;

			// OK, send the command to move the servos, including speed
			SetMultiServoPositionAndSpeed( NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
		}

		///////////////////////////////
		// Left Arm
		// Only move if all servos are enabled

		if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable )
		{

			// Handle RX64 Servo Separately
			// TODO: #if ( DYNA_SERVO_RX64_INSTALLED == 1 )
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = LEFT_ARM_ELBOW_BEND_HOME2_LOCK * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, LEFT_ARM_ELBOW_BEND_HOME2_LOCK * 10 ); 
			SetServoPositionAndSpeed( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, PositionTicks, DynaServoSpeed );

			// Now handle AX12 Servos
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = LEFT_ARM_ELBOW_ROTATE_HOME2 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, LEFT_ARM_ELBOW_ROTATE_HOME2 * 10 ); 
			ServoIDArray[0] = DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID;															// ID
			ServoValueArray[0] = PositionTicks;																				// Position (0-03FF)
			ServoSpeedArray[0] = DYNA_VELOCITY_MED;	// Speed

			g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = LEFT_ARM_WRIST_ROTATE_HOME2_LOCK * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_WRIST_SERVO_ID, LEFT_ARM_WRIST_ROTATE_HOME2_LOCK * 10 ); 
			ServoIDArray[1] = DYNA_LEFT_ARM_WRIST_SERVO_ID;	// ID
			ServoValueArray[1] = PositionTicks;
			ServoSpeedArray[1] = DYNA_VELOCITY_MED_FAST; // Move wrist fast to get claw out of the way for shutdown

			g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = LEFT_ARM_CLAW_HOME2 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_CLAW_SERVO_ID, LEFT_ARM_CLAW_HOME2  * 10 ); 
			ServoIDArray[2] = DYNA_LEFT_ARM_CLAW_SERVO_ID;	// ID
			ServoValueArray[2] = PositionTicks;
			ServoSpeedArray[2] = DYNA_VELOCITY_MED;

			// OK, send the command to move the servos, including speed
			SetMultiServoPositionAndSpeed( NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
		}
		// Now, put the Head down to "sleep"
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_TILT_SERVO_ID, CAMERA_TILT_SLEEP_POSITION );
		SetServoPosition( DYNA_CAMERA_TILT_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	} // LOKI ROBOT
}

void CDynaControl::GotoStartupPosition() // Home Position
{
	int  PositionTicks = 0;
	
	ROBOT_LOG( TRUE, "Moving servos to startup Positions\n" )

	SetAllCameraServosSpeed(DYNA_VELOCITY_MED); // Max = 0x03FF = 1023

	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
	{
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_SIDETILT_SERVO_ID, CAMERA_SIDETILT_CENTER );
		SetServoPosition( DYNA_CAMERA_SIDETILT_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
	{
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_TILT_SERVO_ID, CAMERA_TILT_CENTER );
		SetServoPosition( DYNA_CAMERA_TILT_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
	{
		PositionTicks = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, CAMERA_PAN_CENTER );
		SetServoPosition( DYNA_CAMERA_PAN_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}
	if( 1 == ROBOT_HAS_KINECT_SERVO )
	{
		PositionTicks = TenthDegreeToTicks( DYNA_KINECT_SCANNER_SERVO_ID, KINECT_HUMAN_DETECT_START_POSITION ); // Start in position to detect people
		SetServoPosition( DYNA_KINECT_SCANNER_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
	}

	if( ROBOT_TYPE == LOKI )
	{
		/////////////////////////////////////////////////////////////////////////////
		// Set the arms to home position
		// Make sure g_BulkServoCmd block stays in sync with where the hardware is
		int  ServoIDArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// IDs Servos
		int  ServoValueArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Values to set
		int  ServoSpeedArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Speed to set

		int  DynaServoSpeed = ServoSpeedToDynaServoSpeed( SERVO_SPEED_MED_SLOW );

		///////////////////////////////
		// Right Arm
		// Only move if all servos are enabled

		if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable )
		{	
			// Handle RX64 Servo Separately
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_ELBOW_BEND_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, RIGHT_ARM_ELBOW_BEND_HOME1 * 10 ); 
			SetServoPositionAndSpeed( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, PositionTicks, DynaServoSpeed );

			// Now handle AX12 Servos
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_ELBOW_ROTATE_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, RIGHT_ARM_ELBOW_ROTATE_HOME1 * 10 ); 
			ServoIDArray[0] = DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID;															// ID
			ServoValueArray[0] = PositionTicks;																				// Position (0-03FF)
			ServoSpeedArray[0] = DYNA_VELOCITY_MED_SLOW;	// Speed

			g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_WRIST_ROTATE_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_WRIST_SERVO_ID, RIGHT_ARM_WRIST_ROTATE_HOME1 * 10 ); 
			ServoIDArray[1] = DYNA_RIGHT_ARM_WRIST_SERVO_ID;	// ID
			ServoValueArray[1] = PositionTicks;
			ServoSpeedArray[1] = DYNA_VELOCITY_MED_SLOW;

			g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_CLAW_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_RIGHT_ARM_CLAW_SERVO_ID, RIGHT_ARM_CLAW_HOME1  * 10 ); 
			ServoIDArray[2] = DYNA_RIGHT_ARM_CLAW_SERVO_ID;	// ID
			ServoValueArray[2] = PositionTicks;
			ServoSpeedArray[2] = DYNA_VELOCITY_MED_SLOW;

			// OK, send the command to move the servos, including speed
			SetMultiServoPositionAndSpeed( NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
		}	

		///////////////////////////////
		// Left Arm
		// Only move if all servos are enabled

		if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable &&
			g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable )
		{

			// Handle RX64 Servo Separately
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = LEFT_ARM_ELBOW_BEND_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, LEFT_ARM_ELBOW_BEND_HOME1 * 10 ); 
			SetServoPositionAndSpeed( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, PositionTicks, DynaServoSpeed );

			// Now handle AX12 Servos
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = LEFT_ARM_ELBOW_ROTATE_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, LEFT_ARM_ELBOW_ROTATE_HOME1 * 10 ); 
			ServoIDArray[0] = DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID;															// ID
			ServoValueArray[0] = PositionTicks;																				// Position (0-03FF)
			ServoSpeedArray[0] = DYNA_VELOCITY_MED_SLOW;	// Speed

			g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = LEFT_ARM_WRIST_ROTATE_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_WRIST_SERVO_ID, LEFT_ARM_WRIST_ROTATE_HOME1 * 10 ); 
			ServoIDArray[1] = DYNA_LEFT_ARM_WRIST_SERVO_ID;	// ID
			ServoValueArray[1] = PositionTicks;
			ServoSpeedArray[1] = DYNA_VELOCITY_MED_FAST;

			g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = LEFT_ARM_CLAW_HOME1 * 10;
			PositionTicks = TenthDegreeToTicks( DYNA_LEFT_ARM_CLAW_SERVO_ID, LEFT_ARM_CLAW_HOME1  * 10 ); 
			ServoIDArray[2] = DYNA_LEFT_ARM_CLAW_SERVO_ID;	// ID
			ServoValueArray[2] = PositionTicks;
			ServoSpeedArray[2] = DYNA_VELOCITY_MED_SLOW;

			// OK, send the command to move the servos, including speed
			SetMultiServoPositionAndSpeed( NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
		}
	} // LOKI ROBOT
}

int  CDynaControl::GetResponseCheckSum(int  nResponseBytes)
{
	// Calculate Checksum of the returned data
	int  CheckSum = m_DynaReply.ID;
	CheckSum += m_DynaReply.Length;
	CheckSum += m_DynaReply.Error;

	for ( int  i=0 ; i<nResponseBytes ; i++ )
	{
		CheckSum += m_DynaReply.Data[i];
	}
	CheckSum = (~CheckSum);
	CheckSum = CheckSum & 0x00FF;	// save lower 8 bits only
	return CheckSum;	// Notice this is lower 8 bits only
}

char* CDynaControl::ServoIDToName( int  ServoID )
{
	switch( ServoID )
	{
		case DYNA_CAMERA_PAN_SERVO_ID:				return "DYNA_CAMERA_PAN_SERVO";
		case DYNA_CAMERA_TILT_SERVO_ID:				return "DYNA_CAMERA_TILT_SERVO";
		case DYNA_CAMERA_SIDETILT_SERVO_ID:			return "DYNA_CAMERA_SIDETILT_SERVO";

		case DYNA_KINECT_SCANNER_SERVO_ID:			return "DYNA_KINECT_SCANNER_SERVO_ID";
	
		case DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID:	return "DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO";
		case DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID:	return "DYNA_RIGHT_ARM_ELBOW_BEND_SERVO";
		case DYNA_RIGHT_ARM_WRIST_SERVO_ID:			return "DYNA_RIGHT_ARM_WRIST_SERVO";
		case DYNA_RIGHT_ARM_CLAW_SERVO_ID:			return "DYNA_RIGHT_ARM_CLAW_SERVO";

		case DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID:	return "DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO";
		case DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID:		return "DYNA_LEFT_ARM_ELBOW_BEND_SERVO";
		case DYNA_LEFT_ARM_WRIST_SERVO_ID:			return "DYNA_LEFT_ARM_WRIST_SERVO";
		case DYNA_LEFT_ARM_CLAW_SERVO_ID:			return "DYNA_LEFT_ARM_CLAW_SERVO";
		default:
		{
			ROBOT_ASSERT(0); // unhandled servo
			return "UNKNOWN";
		}
	}

}

///////////////////////////////////////////////////////////////////////////////
// Returns TRUE if FATAL ERROR detected (currently only Overheat)
//
BOOL CDynaControl::DisplayServoError( int  ServoID, int  nError)
{
	CString MsgString;
	BOOL NewFatalError = FALSE;

	if( 0 == nError )
	{
		if ( ServoID == gServoOverheatError )
		{
			// We had an overheat error on this servo, but the error has cleared now.  Reenable Arms
			gServoOverheatError = 0;
			gServoOverHeatTimer = 0;
			ROBOT_DISPLAY( TRUE, "\n=================================================")
			ROBOT_DISPLAY( TRUE, "========> %s (%d): OVERHEAT ERROR CLEARED! <=======", ServoIDToName(ServoID), ServoID )
			ROBOT_DISPLAY( TRUE, "          All Arm Movements Re-Enabled!")
			ROBOT_DISPLAY( TRUE, "====================================================\n" )
			SpeakText( "My Servo Over heat error has cleared.  My arm movements are now enabled again.");
		}
		return NewFatalError;
	}

	if( nError & 0x01 )
	{
		MsgString.Format( "========> %s (%d): Input Voltage Error!", ServoIDToName(ServoID), ServoID );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	if( nError & 0x02 )
	{
		MsgString.Format( "========> %s (%d): Angle Limit Error!", ServoIDToName(ServoID), ServoID );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	if( nError & 0x04 )
	{
		if ( 0 != gServoOverheatError )
		{
			// Only report once when the error is detected
			gServoOverheatError = ServoID; // track the error for first servo that overheats
			NewFatalError = TRUE; // flag that the servo needs to be disabled
			gServoOverHeatTimer = 100; // tenth seconds to wait for servo to cool off
			ROBOT_DISPLAY( TRUE, "\n================ FATAL ERROR =================")
			MsgString.Format( "========> %s (%d): OVERHEAT ERROR! <=======", ServoIDToName(ServoID), ServoID );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
			ROBOT_DISPLAY( TRUE, "          All Arm Movements Disabled!")
			ROBOT_DISPLAY( TRUE, "================ FATAL ERROR =================\n" )
			SpeakText( "I have a fatal servo over heat error. My arms are disabled to prevent damange." );
		}
	}
	if( nError & 0x08 )
	{
		MsgString.Format( "========> %s (%d): Range Error!", ServoIDToName(ServoID), ServoID );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	if( nError & 0x10 )
	{
		MsgString.Format( "========> %s (%d): CheckSum Error!", ServoIDToName(ServoID), ServoID );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	if( nError & 0x20 )
	{
		if( DYNA_LEFT_ARM_CLAW_SERVO_ID != ServoID ) // Ignore Left Claw, which is has torque set low to overload on purpose
		{
			MsgString.Format( "========> %s (%d): Overload Error!", ServoIDToName(ServoID), ServoID );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
	}
	if( nError & 0x40 )
	{
		MsgString.Format( "========> %s (%d): Instruction Error!", ServoIDToName(ServoID), ServoID );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	return NewFatalError;
}



///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::ChangeServoID( int  OldServoID, int  NewServoID )
{
	// Utility function for changing the ID of a servo
	// NOTE: it's easier to use the Dynamixel Utility instead!
		ROBOT_LOG( TRUE, "DYNA - Changing Servo ID from %d to %d!\n", OldServoID, NewServoID )

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)OldServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_ROM_ID;
	m_DynaCmd.Data[1] = (BYTE)NewServoID;
	// Use this to send command out the RX64 port
	//SendCmd( SendParams, 0, OldServoID );	// send params, 0 receive optional params, servo to write to 
}

void CDynaControl::EnableServoTorque( int  ServoID, BOOL bEnable )
{
	// send command to enable servo torque (not sync'd with other commands
	if( bEnable )
		ROBOT_LOG( TRUE,"DYNA - Enabling Torque on Servo ID %d\n", ServoID)
	else
		ROBOT_LOG( TRUE,"DYNA - Disabling Torque on Servo ID %d\n", ServoID)

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_TORQUE_ENABLE;
	m_DynaCmd.Data[1] = (BYTE)bEnable;	// Enable/Disable Servo

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID
	
}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetShutdownConditions( int  ServoID, BYTE ShutdownFlags )
{
	// send command to enable servo torque (not sync'd with other commands
	// Only do this once for new servos, to write into Servo's ROM.
	ROBOT_ASSERT(0); // CAREFUL HOW YOU USE THIS FUNCTION!  ONLY FOR NEW SERVOS!
	ROBOT_LOG( TRUE, "DYNA - Setting Shutdown Flags for Servo ID %d to %02X\n", ServoID, ShutdownFlags )

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_ROM_ALARM_SHUTDOWN;
	m_DynaCmd.Data[1] = ShutdownFlags;	// Flags that indicate shutdown events

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}



///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoSpeed( int  ServoID, int  Speed )
{
	// send command to set servo speed (not sync'd with other commands
	if( Speed > DYNA_MOVING_SPEED_MAX )
	{
		ROBOT_LOG( TRUE,"DYNA - Error! Requested Servo Speed (%d) > DYNA_MOVING_SPEED_MAX for Servo ID %d\n", Speed, ServoID )
		Speed = DYNA_MOVING_SPEED_MAX;
		ROBOT_ASSERT(0);
	}
	ROBOT_LOG( TRUE,"DYNA - Setting Servo ID %d Speed to %d\n", ServoID, Speed)

	int  SendParams = 3;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_MOVING_SPEED;
	m_DynaCmd.Data[1] = (BYTE)(Speed & 0xFF);	
	m_DynaCmd.Data[2] = (BYTE)(Speed >> 8);

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoComplianceSlope( int  ServoID, int  ComplianceSlope )
{
	return; // DAVES - TODO-MUST - REPLACE THIS?
	ROBOT_ASSERT(0); // BE VERY CAREFUL IF SETTING COMPLIANCE ON RX64!  DONT WANT TO BURN IT OUT!

	// Send command to set servo Compliance Slope, the amount of torque the servo will exert
	// to move to exact position specified. Used for Head Tilt, for precise Laser Scanner control. 
	// But, too tight will put a lot of stress on the servo, so move slow if this is set to tight!
	if( ComplianceSlope > DYNA_COMPLIANCE_LIMIT_MAX )
	{
		ROBOT_LOG( TRUE,"DYNA - Error! Requested Servo Compliance Slope (%d) > DYNA_COMPLIANCE_LIMIT_MAX for Servo ID %d\n", ComplianceSlope, ServoID )
		ComplianceSlope = DYNA_COMPLIANCE_LIMIT_MAX;
	}
	else if( ComplianceSlope < DYNA_COMPLIANCE_LIMIT_MIN )
	{
		ROBOT_LOG( TRUE,"DYNA - Error! Requested Servo Compliance Slope (%d) < DYNA_COMPLIANCE_LIMIT_MIN for Servo ID %d\n", ComplianceSlope, ServoID )
		ComplianceSlope = DYNA_COMPLIANCE_LIMIT_MIN;
	}
	ROBOT_LOG( TRUE,"DYNA - Setting Servo ID %d Compliance Slope to %02X\n", ServoID, ComplianceSlope)


/*
	#define DYNA_COMPLIANCE_LIMIT_MIN			    0x01	//  1 ( 0.3deg) - VERY Tight compliance!
	#define DYNA_COMPLIANCE_VERY_TIGHT			    0x02	//  2 ( 0.6deg) - Tight Compliance, Head servos only?
	#define DYNA_COMPLIANCE_TIGHT				    0x08	//  8 ( 2.3deg) - Tight Compliance, Head servos only?
	#define DYNA_COMPLIANCE_NORMAL				    0x20	// 32 ( 9.4deg) - Default Compliance
	#define DYNA_COMPLIANCE_LIMIT_MAX			    0x64	// 64 (18.8deg) - Loose Compliance
*/
	int  SendParams = 3;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_CW_COMPLIANCE_SLOPE;
	m_DynaCmd.Data[1] = (BYTE)(ComplianceSlope);	// CW Compliance Slope
	m_DynaCmd.Data[2] = (BYTE)(ComplianceSlope);	// CCW Compliance Slope

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

/*
		// For tight slope, need to crank up the Punch too, and restore it, but only on AX12 servos
		int  CompliancePunch = 0x20;
		if( ComplianceSlope <= DYNA_COMPLIANCE_TIGHT )
		{
			CompliancePunch = 0x40; // Max = 0x3FF;
		}

		int  SendParams = 3;
		m_DynaCmd.ID = (BYTE)ServoID;
		m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
		m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
		m_DynaCmd.Data[0] = DYNA_REG_PUNCH;
		m_DynaCmd.Data[1] = (BYTE)(CompliancePunch & 0xFF);	
		m_DynaCmd.Data[2] = (BYTE)(CompliancePunch >> 8);

		{	// Send comamnd out the AX12 port
			SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, servoID
		}
*/


}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoMaxTorque( int  ServoID, int  MaxTorque )
{
//	if( (DYNA_LEFT_ARM_CLAW_SERVO_ID == ServoID)	|| (DYNA_RIGHT_ARM_CLAW_SERVO_ID == ServoID) )
//	{
		// DEBUG TEST - disable Claw torque setting!
		//ROBOT_LOG( TRUE,"DYNA - WARNING - IGNORING TORQUE SETTING FOR CLAWS!!!\n")
//		return;
//	}

	// send command to set servo torque
	if( MaxTorque > DYNA_TORQUE_LIMIT_MAX )
	{
		ROBOT_LOG( TRUE,"DYNA - Error! Requested Servo MaxTorque (%d) > DYNA_TORQUE_LIMIT_MAX for Servo ID %d\n", MaxTorque, ServoID )
		MaxTorque = DYNA_TORQUE_LIMIT_MAX;
	}
	ROBOT_LOG( TRUE,"DYNA - Setting Servo ID %d MaxTorque to %d\n", ServoID, MaxTorque)

	int  SendParams = 3;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_TORQUE_LIMIT;
	m_DynaCmd.Data[1] = (BYTE)(MaxTorque & 0xFF);	
	m_DynaCmd.Data[2] = (BYTE)(MaxTorque >> 8);

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoPosition( int  ServoID, int  Position ) // in SERVO TICKS
{
	// send command to move servo immediately (not sync'd with other commands
	//ROBOT_LOG( TRUE,"DYNA - SET POSITION ID %d = %08lX\n", ServoID, Position)

	int  SendParams = 3;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_GOAL_POSTION;	
	m_DynaCmd.Data[1] = (BYTE)(Position & 0xFF);	
	m_DynaCmd.Data[2] = (BYTE)(Position >> 8);

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}


///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoPositionAndSpeed( int  ServoID, int  Position, int  Speed )

{
	// send command to move servo immediately (not sync'd with other commands
	//ROBOT_LOG( TRUE,"DYNA - SET POSITION AND SPEED: ID %d Position = %d, speed = %d\n", ServoID, Position, Speed)

/*	if( Position > DYNA_ANGLE_LIMIT_MAX )
	{
		ROBOT_LOG( TRUE,"DYNA - ERROR! Position Out Of Range: ID %d Position = %08lX\n", ServoID, Position)
		return;
	}
*/
	//int  RegistersToWrite = 4;	// Number of Registers to write for each servo (Position and Speed words)
	memset(m_DynaCmd.Data,0,DYNA_MAX_DATA_BYTES);	// make it easier to debug

	int  SendParams = 5;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_GOAL_POSTION;	
	m_DynaCmd.Data[1] = (BYTE)(Position & 0xFF);	
	m_DynaCmd.Data[2] = (BYTE)(Position >> 8);
	m_DynaCmd.Data[3] = (BYTE)(Speed & 0xFF);	
	m_DynaCmd.Data[4] = (BYTE)(Speed >> 8);

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID
	
}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetCameraHeadPosition(int  HeadPanServo, int  HeadTiltServo, int  SideTiltServo )
{
	// send command to move servo immediately
	ROBOT_LOG( TRUE,"DYNA - SET HEAD POSITION \n")

	//***************************************************************************
	// KLUDE!  NECK DISABLED FOR NOW!  SERVO 1 = PAN, not NECK!  SERVO2 NOT USED!
	//***************************************************************************

	int  RegistersToWrite = 2;	// Number of Registers to write for each servo
	int  NumberOfServos = 3;

	m_DynaCmd.ID = DYNA_ID_BROADCAST;
	m_DynaCmd.Length = (BYTE)(((RegistersToWrite+1) * NumberOfServos) + 4);
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_SYNC_WRITE;
	m_DynaCmd.Data[0] = DYNA_REG_GOAL_POSTION;	// First Register to write
	m_DynaCmd.Data[1] = (BYTE)RegistersToWrite;	// Number of Registers to write for each instruction

	m_DynaCmd.Data[2] = 1;	// Servo ID of Data - HeadPanServo
	m_DynaCmd.Data[3] = (BYTE)(HeadPanServo & 0xFF);	
	m_DynaCmd.Data[4] = (BYTE)(HeadPanServo >> 8);

	m_DynaCmd.Data[5] = 3;	// Servo ID of Data - HeadTiltServo
	m_DynaCmd.Data[6] = (BYTE)(HeadTiltServo & 0xFF);	
	m_DynaCmd.Data[7] = (BYTE)(HeadTiltServo >> 8);

	m_DynaCmd.Data[8] = 4;	// Servo ID of Data - SideTiltServo
	m_DynaCmd.Data[9] = (BYTE)(SideTiltServo & 0xFF);	
	m_DynaCmd.Data[10] = (BYTE)(SideTiltServo >> 8);

	SendCmd( ((RegistersToWrite+1) * NumberOfServos) + 2, 0, DYNA_MULTI_SERVO_ID, FALSE);	
	// 3 send params for each servo, 0 receive optional params.  NO RESPONSE expected, since this is a Broadcast command
}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetMultiServoPosition(int  NumberOfServos, int  *ServoIDArray, int  *ServoValueArray )
{
//	ROBOT_LOG( TRUE,"DYNA - SetMultiServoPosition \n")
#ifdef DEBUG_DYNA_TIMING
	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	ROBOT_LOG( TRUE, "   Dyna SetMultiServoPosition Start: %4d ms\n", ProcessingTime )
#endif

	BOOL bRXServo = FALSE;
	BYTE RXServoID = 0;
	BYTE RXValueLow = 0;
	BYTE RXValueHigh = 0;

	int  RegistersToWrite = 2;	// Number of Registers to write for each servo
	memset(m_DynaCmd.Data,0,DYNA_MAX_DATA_BYTES);	// make it easier to debug

	m_DynaCmd.ID = DYNA_ID_BROADCAST;
	m_DynaCmd.Length = (BYTE)(((RegistersToWrite+1) * NumberOfServos) + 4);
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_SYNC_WRITE;
	m_DynaCmd.Data[0] = DYNA_REG_GOAL_POSTION;	// First Register to write
	m_DynaCmd.Data[1] = (BYTE)RegistersToWrite;	// Number of Registers to write for each instruction

	BYTE *pData = &m_DynaCmd.Data[2];

	for( int  i=0; i<NumberOfServos; i++ )
	{
		*pData++ = (BYTE)ServoIDArray[i];	// Servo ID of Data
		*pData++ = (BYTE)(ServoValueArray[i] & 0xFF);	// Data	Low
		*pData++ = (BYTE)(ServoValueArray[i] >> 8);		// Data High
	}
	SendCmd( ((RegistersToWrite+1) * NumberOfServos) + 2, 0, DYNA_MULTI_SERVO_ID, FALSE);	
	// 3 send params for each servo, 0 receive optional params.  NO RESPONSE expected, since this is a AX12 Broadcast command


}

void CDynaControl::SetMultiServoPositionAndSpeed(int  NumberOfServos, int  *ServoIDArray, int  *ServoValueArray,int  *ServoSpeedArray )
{
//	ROBOT_LOG( TRUE,"DYNA - SetMultiServoPosition \n")
#ifdef DEBUG_DYNA_TIMING
	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	ROBOT_LOG( TRUE, "   Dyna SetMultiServoPositionAndSpeed Start: %4d ms\n", ProcessingTime )
#endif

	if( 0 != gServoOverheatError )
	{
		ROBOT_DISPLAY( TRUE, "DYNA - GLOBAL OVERHEAT ERROR set! All moves ignored!\n" )
		return;
	}

	int  RegistersToWrite = 4;	// Number of Registers to write for each servo (Position and Speed words)
	memset(m_DynaCmd.Data,0,DYNA_MAX_DATA_BYTES);	// make it easier to debug

	m_DynaCmd.ID = DYNA_ID_BROADCAST;
	m_DynaCmd.Length = (BYTE)(((RegistersToWrite+1) * NumberOfServos) + 4);
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_SYNC_WRITE;
	m_DynaCmd.Data[0] = DYNA_REG_GOAL_POSTION;	// First Register to write
	m_DynaCmd.Data[1] = (BYTE)RegistersToWrite;	// Number of Registers to write for each instruction

	BYTE *pData = &m_DynaCmd.Data[2];

	for( int  i=0; i<NumberOfServos; i++ )
	{
		*pData++ = (BYTE) ServoIDArray[i];	// Servo ID of Data
		*pData++ = (BYTE)(ServoValueArray[i] & 0xFF);	// Data	Low
		*pData++ = (BYTE)(ServoValueArray[i] >> 8);		// Data High
		*pData++ = (BYTE)(ServoSpeedArray[i] & 0xFF);	// Data	Low
		*pData++ = (BYTE)(ServoSpeedArray[i] >> 8);		// Data High
	}
	SendCmd( ((RegistersToWrite+1) * NumberOfServos) + 2, 0, DYNA_MULTI_SERVO_ID, FALSE);	
	// 3 send params for each servo, 0 receive optional params.  NO RESPONSE expected, since this is a AX12 Broadcast command

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::PingServo(int  ServoID)
{
	// send command to move servo immediately (not sync'd with other commands
	ROBOT_LOG( TRUE,"DYNA - Ping Servo ID %d\n", ServoID)

	int  SendParams = 0;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_PING;

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::SetServoLED(int  ServoID, BOOL State)
{
	// send command to move servo immediately (not sync'd with other commands
	ROBOT_LOG( TRUE,"DYNA - SET LED ID %d = %d\n", ServoID, State)

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_LED;	
	m_DynaCmd.Data[1] = (BYTE)(State);	
	
	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}


///////////////////////////////////////////////////////////////////////////////
void CDynaControl::GetServoStatus(int  ServoID)
{
	//TAL_SCOPED_TASK();
	//TAL_SCOPED_TASK_NAMED("GetServoStatus");
	// Skip any servo that is disabled
	if( !g_BulkServoCmd[ServoID].Enable )
	{
		//ROBOT_LOG( TRUE, "Skipping disabled servo %d", ServoID )
		return;
	}

	// Request current servo Status
#ifdef DEBUG_DYNA_TIMING
	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	ROBOT_LOG( TRUE, "   Dyna GetServoStatus (ID%d Start: %4d ms\n", ServoID, ProcessingTime )
#endif

	int  SendParams = 2;
	BOOL Success = FALSE;
	BOOL NewFatalError = FALSE;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_READ_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_CURRENT_POSITION;	// First Register to read	
	m_DynaCmd.Data[1] = 8;	// 8 Registers to read

	// Send comamnd out the servo port
	Success = SendCmd( SendParams, 8, ServoID );	// number of send params, receive optional params, Response Expected, ServoID
	
	if( Success )
	{
		// Process status and display any errors
		// If NewFatalError, disable the servo to prevent damage
		NewFatalError = DisplayServoError( ServoID, m_DynaReply.Error );
	}
	if( NewFatalError )
	{
		EnableServoTorque( ServoID, FALSE );
		// Update Status to indicate system failure
		#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
			if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == ServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == ServoID) )
				g_RX64SubSystemStatus = SUBSYSTEM_FAILED;
			else
				g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
		#else
			g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
		#endif
	}
	else if( Success )
	{
		// Dump status of the servo
		int  Position = MAKEWORD(m_DynaReply.Data[0], m_DynaReply.Data[1]);
		int  Speed = MAKEWORD(m_DynaReply.Data[2], m_DynaReply.Data[3]);
		int  Load = MAKEWORD(m_DynaReply.Data[4], m_DynaReply.Data[5]);
		int  VoltageRaw = m_DynaReply.Data[6];
		int  Centigrade = m_DynaReply.Data[7];
		double Voltage = (double)VoltageRaw / 10.0;
		double Fahrenheit = ((double)Centigrade * 1.8) + 32.0;
		signed int SignedLoad = Load;
		if( Load >= 0x400 )
		{
			SignedLoad = Load -0x400;
			SignedLoad *= -1;
		}
		// Special case backward servos for load only.  Position is handled by ticks<-->tenthdegree conversion functions
		if( DYNA_RIGHT_ARM_CLAW_SERVO_ID == ServoID )
		{
			SignedLoad *= -1;
		}

		int PositionTenthDegrees = TicksToTenthDegree( ServoID, Position );

		if( DYNA_KINECT_SCANNER_SERVO_ID == ServoID )
		{
			#if DEBUG_DYNA_SHOW_STATUS_KINECT == 1
				ROBOT_LOG( TRUE,"Got Kinect Servo Position Update = %04.1f\n", (double)PositionTenthDegrees / 10.0 )
			#endif
		}
		// Indicate that communication is working OK with the servo controller
			// Update Status to indicate system failure
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == ServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == ServoID) )
					g_RX64SubSystemStatus = SUBSYSTEM_CONNECTED;
				else
					g_DynaSubSystemStatus = SUBSYSTEM_CONNECTED;
			#else
				g_DynaSubSystemStatus = SUBSYSTEM_CONNECTED;
			#endif


		#if (DEBUG_DYNA_SHOW_STATUS_MESSAGES == 1)
		if( (DEBUG_DYNA_SHOW_STATUS_KINECT && (DYNA_KINECT_SCANNER_SERVO_ID == ServoID) ) ||
			(DEBUG_DYNA_SHOW_STATUS_HEAD && ((DYNA_CAMERA_PAN_SERVO_ID == ServoID) || (DYNA_CAMERA_TILT_SERVO_ID == ServoID) || (DYNA_CAMERA_SIDETILT_SERVO_ID == ServoID)) ) ||
			(DEBUG_DYNA_SHOW_STATUS_ARM_RIGHT && ((DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID == ServoID) || (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == ServoID) || 
				(DYNA_RIGHT_ARM_WRIST_SERVO_ID == ServoID) || (DYNA_RIGHT_ARM_CLAW_SERVO_ID == ServoID)) ) ||
			(DEBUG_DYNA_SHOW_STATUS_ARM_LEFT && ((DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID == ServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == ServoID) || 
				(DYNA_LEFT_ARM_WRIST_SERVO_ID == ServoID) || (DYNA_LEFT_ARM_CLAW_SERVO_ID == ServoID)) )   )
		{
			// TODO: DYNA_KINECT_SCANNER_SERVO_ID
	
		// FILTER FOR WHAT EVER SERVO NEEDS DEBUG HERE
		//	if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == ServoID) ||
		//		(DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == ServoID) )
		//	{
	
				ROBOT_LOG( TRUE, "DYNA - Status for Servo %d:   \t(raw hex values)", ServoID )
					ROBOT_LOG( TRUE, "       Position =     %4d       (%04Xh)", Position, Position )
					ROBOT_LOG( TRUE, "       Speed =        %4d       (%04Xh)", Speed, Speed )
					ROBOT_LOG( TRUE, "       Load =         %4d       (%04Xh)  (Max= +/- 1023)", SignedLoad, Load )
					ROBOT_LOG( TRUE, "       Voltage =        %4.1f v     (%02Xh)", Voltage, VoltageRaw )
					ROBOT_LOG( TRUE, "       Temperature =   %4.1f deg   (%02Xh)\n", Fahrenheit, Centigrade )
		//	}
		}
		#endif

		// DAVES-TODO
		// TODO-MUST REMOVE THIS TEMP DEBUG:
/*
		if(0 != Speed)
		{
				ROBOT_LOG( TRUE, "DYNA - Status for Servo %d:   \t(raw hex values)", ServoID )
					ROBOT_LOG( TRUE, "       Position =     %4d       (%04Xh)", Position, Position )
					ROBOT_LOG( TRUE, "   *** Speed =        %4d       (%04Xh)", Speed, Speed )
					ROBOT_LOG( TRUE, "       Load =         %4d       (%04Xh)  (Max= +/- 1023)", SignedLoad, Load )
					ROBOT_LOG( TRUE, "       Voltage =        %4.1f v     (%02Xh)", Voltage, VoltageRaw )
					ROBOT_LOG( TRUE, "       Temperature =   %4.1f deg   (%02Xh)\n", Fahrenheit, Centigrade )
		}
*/
		// Now update the global status block with this servo's info. (see BULK_SERVO_STATUS_T)
		g_BulkServoStatus[ServoID].PositionTenthDegrees = PositionTenthDegrees;
		g_BulkServoStatus[ServoID].Speed = DynaServoSpeedToServoSpeed(Speed);
		g_BulkServoStatus[ServoID].Load = SignedLoad;
		g_BulkServoStatus[ServoID].StatusFlags = m_DynaReply.Error;		
		g_BulkServoStatus[ServoID].TemperatureFahrenheit = (int )Fahrenheit;
	}
	else
	{
		if( 1 != ROBOT_SIMULATION_MODE )
		{
			//if( (ROBOT_TYPE == LOKI) || 
			//	((ROBOT_TYPE == TURTLE) && (DYNA_KINECT_SCANNER_SERVO_ID == ServoID)) )
			//{
				ROBOT_LOG( TRUE, "DYNA - Unable to dump Status for Servo %d:\n", ServoID )
			//}
		}
	}

	// Check for Overheat status and recovery
	if( 0 != gServoOverheatError )
	{
		if ( 0 == gServoOverHeatTimer )
		{
			// timer completed. Try re-enabling the servo
			EnableServoTorque( ServoID, FALSE );
		}
	}

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::GetStatusOfMovingServos()
{
	///TAL_SCOPED_TASK_NAMED("Get Status of Moving Servos");
	// Request the status of any servos that are not close to or at commanded position

	// Laser Scanner
	
	if( 1 == ROBOT_HAS_KINECT_SERVO )
	{

		if( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees - 
				g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees ) > KINECT_SERVO_TOLERANCE_NORMAL_TENTHDEGREES ))
		{
			GetServoStatus( DYNA_KINECT_SCANNER_SERVO_ID );
		}
	}

	// Camera/Head servos
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=3 )
	{
		if( g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_CAMERA_SIDETILT_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			///TAL_SCOPED_TASK_NAMED("CAM_SIDETILT");
				GetServoStatus( DYNA_CAMERA_SIDETILT_SERVO_ID );
		}
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=2 )
	{
				if( g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
				GetServoStatus( DYNA_CAMERA_TILT_SERVO_ID );
		}
	}
	if( NUMBER_OF_DYNA_SERVOS_IN_HEAD >=1 )
	{
		if( g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees )	> HEAD_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_CAMERA_PAN_SERVO_ID );
		}
	}
		
	if( ROBOT_TYPE == LOKI )
	{
		// Right Arm
		if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees )> ARM_JOINT_DELTA_MAX_TENTHDEGREES ))
		{
			GetServoStatus( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees )	> ARM_JOINT_DELTA_MAX_TENTHDEGREES ))
		{
			GetServoStatus( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_RIGHT_ARM_WRIST_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_RIGHT_ARM_CLAW_SERVO_ID );
		}

		// Left Arm
		if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees )	> ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees )		> ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_LEFT_ARM_WRIST_SERVO_ID );
		}
		if( g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable && (abs( g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees - 
			  g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees )			> ARM_JOINT_DELTA_MAX_TENTHDEGREES )) 
		{
			GetServoStatus( DYNA_LEFT_ARM_CLAW_SERVO_ID );
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::ReadServoRegisters(int  ServoID, int  Register, int  NumberOfRegisters)
{
	// Request current servo Status - Dump ALL the registers!

	int  SendParams = 2;
	BOOL Success = FALSE;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_READ_DATA;
	m_DynaCmd.Data[0] = (BYTE)Register;	// First Register to read	
	m_DynaCmd.Data[1] = (BYTE)NumberOfRegisters;	// Registers to read


	// Send comamnd out the servo port
	Success = SendCmd( SendParams, NumberOfRegisters, ServoID );	// Number of data BYTEs to Write and Read, ServoID

	if( Success )
	{
		// Dump status of the Registers read

		ROBOT_LOG( TRUE, "DYNA - Register Dump for Servo %d:   \t(raw hex values)\n", ServoID )
		for( int  RegNum=0; RegNum < NumberOfRegisters; RegNum++ )
		{
			ROBOT_LOG( TRUE, "       Register %d (%02Xh) = %4d (%02Xh)\n", 
				(Register+RegNum), (Register+RegNum), m_DynaReply.Data[RegNum], m_DynaReply.Data[RegNum] )
		}

	}
	else
	{
		ROBOT_LOG( TRUE, "DYNA - Unable to Dump Registers for Servo %d:\n", ServoID )
	}

	ROBOT_LOG( TRUE,"\n")
}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::WriteServoRegisterByte( int  ServoID, int  Register, int  Value )
{
	// send command to write a given register (single byte values)
	ROBOT_LOG( TRUE, "DYNA - Write Register BYTE: ServoID %d, Register %d (%02X) = %d (%02X)\n", 
		ServoID, (BYTE)Register, (BYTE)Register, (BYTE)(Value), (BYTE)(Value) )

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = (BYTE)Register;	
	m_DynaCmd.Data[1] = (BYTE)(Value & 0xFF);

	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}

///////////////////////////////////////////////////////////////////////////////
void CDynaControl::WriteServoRegisterWord( int  ServoID, int  Register, int  Value )
{
	// send command to write a given register (single byte values)
	ROBOT_LOG( TRUE,"DYNA - Write Register WORD: ID %d = %d (%04X)\n", ServoID, (WORD)Value, (WORD)Value)

	int  SendParams = 3;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_WRITE_DATA;
	m_DynaCmd.Data[0] = (BYTE)Register;	
	m_DynaCmd.Data[1] = (BYTE)(Value & 0xFF);	
	m_DynaCmd.Data[2] = (BYTE)(Value >> 8);
	
	// Send comamnd to the servo
	SendCmd( SendParams, 0, ServoID );	// number of send params, receive optional params, ServoID

}


///////////////////////////////////////////////////////////////////////////////
int  CDynaControl::GetServoLoad(int  ServoID)
{
	// Request current load on servo
	int  Load;

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_READ_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_CURRENT_LOAD;	// Register to read	
	m_DynaCmd.Data[1] = DYNA_DOUBLE_REGISTER;	// 2 Registers to read

	// Send comamnd to the servo
	SendCmd( SendParams, 2, ServoID );	// number of send params, receive optional params, ServoID
	
	Load = MAKEWORD(m_DynaReply.Data[1], m_DynaReply.Data[0]);

	ROBOT_LOG( TRUE,"DYNA - Load for Servo %d = %08lX\n", ServoID, Load)
	return Load;
}

///////////////////////////////////////////////////////////////////////////////
int  CDynaControl::GetServoTemperature(int  ServoID)
{
	// Request current temperature of servo
	int  Temperature;

	int  SendParams = 2;
	m_DynaCmd.ID = (BYTE)ServoID;
	m_DynaCmd.Length = (BYTE)(2+SendParams);	// Length is always 2 + the amount of data
	m_DynaCmd.Instruction = DYNA_INSTRUCTION_READ_DATA;
	m_DynaCmd.Data[0] = DYNA_REG_CURRENT_TEMPERATURE;	// Register to read	
	m_DynaCmd.Data[1] = DYNA_SINGLE_REGISTER;	// Just one Register to read

	// Send comamnd to the servo
	SendCmd( SendParams, 1, ServoID );	// number of send params, receive optional params, ServoID

	Temperature = m_DynaReply.Data[0];

	ROBOT_LOG( TRUE,"DYNA - Temperature of Servo %d = %02X\n", ServoID, Temperature)
	return Temperature;
}



///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void CDynaControl::GetStatus( int  ID )
{
	
	// Get current Status from the Dyna Servo
	CString MsgString;
#define DEBUG_Dyna 0

#if DEBUG_Dyna == 1
	ROBOT_LOG( TRUE,"------------------- DYNA SERVO STATUS --------------\n")
#endif

/*** TODO

		// Update the Robot Control thread with the Servo Status
		// this in turn will update the GUI with latest status
		Sample code:
		PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_ER1_ODOMETER_READY),
			WheelDistanceL, WheelDistanceR );
***/
	
#if DEBUG_Dyna == 1
	ROBOT_LOG( TRUE,"------------------------------------------------\n")
#endif
}

///////////////////////////////////////////////////////////////////////////////
BOOL CDynaControl::SendCmd(int  nParamBytes, int  nResponseBytes, int  nServoID, BOOL ResponseExpected )
{

	// Send command to Dynamixel Servo.  Returns 1 for good status, 0 if error occured.
	// Defaults: ResponseExpected = TRUE
	// nServoID may be DYNA_MULTI_SERVO_ID ==> 0

	CString MsgString;
	DWORD dwBytesWritten=0;
//	int  Status = 0;
//	int  ReadCheckSum = 0;
	unsigned char DbgBuf[16];
	memset(DbgBuf,0, 16);
//	int  q;
	DWORD dwBytesReceived = 0;
	//DWORD dwWriteStartTime = 0;
	DWORD dwReadStartTime = 0;
	DWORD dwWriteTime = 0;
	DWORD dwReadTime = 0;
	HANDLE hSerialPort = INVALID_HANDLE_VALUE;

	// Skip any servos that are not enabled
	/* DAVES - TODO-MUST!
	if( !g_BulkServoCmd[nServoID].Enable )
	{
		//ROBOT_LOG( TRUE, "Skipping disabled servo %d", ServoID )
		return 0;
	}
	*/

	//if( (ROBOT_TYPE == TURTLE) && (DYNA_KINECT_SCANNER_SERVO_ID != nServoID) )
	//{	// For Turtle, ignore any servos not installed - KLUDGE, fix this better someday
		//return 0;
	//}

	// Calculate CheckSum and insert it into the command buffer
	AddCheckSum( nParamBytes );
	int  nBytesToWrite = nParamBytes + 6;	// Commands are  5 BYTEs long plus nParamBytes + Checksum
	int  nBytesToRead = nResponseBytes + 6;	// Responses are Header + 3 BYTEs plus nResponseBytes + Checksum

	// Debug:
//	MsgString.Format( "Dyna SendCmd:  Sending cmd %02X to Dyna controller.\n", m_DynaCmd.Code );
//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

	// Determine if this command is targeted for AX12/MX28 serial port or RX64 serial port (if RX servos are used)
	hSerialPort = g_hDynaServoCommPort_AX12;

	#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
		if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == nServoID)	|| (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == nServoID) )
		{	// Send command out the RX64 port instead
			hSerialPort = g_hDynaServoCommPort_RX64;
		}
	#endif


	
#ifdef DEBUG_DYNA_COMMAND_DUMP

	if( 7 == m_DynaReply.ID ) // TODO REMOVE THIS HACK FOR SERVO 7 KINECT
	{
		ROBOT_LOG( TRUE,"Dyna CMD DUMP (%2d BytesToWrite): ", nBytesToWrite )
		ROBOT_LOG( TRUE,"Header: %02X %02X, ServoID: %02X, Length: %02X, Command: %02X  Data = ", m_DynaCmd.Header1, m_DynaCmd.Header2, m_DynaCmd.ID, m_DynaCmd.Length, m_DynaCmd.Instruction )
		for( int  q=0; q < nParamBytes; q++ )
		{
			ROBOT_LOG( TRUE,"%02X ",	m_DynaCmd.Data[q] )
		}
		if( ResponseExpected )
			ROBOT_LOG( TRUE," Response Expected\n")
		else
			ROBOT_LOG( TRUE," No Response Needed\n")

		if( 3 == m_DynaCmd.Instruction )
		{
			ROBOT_LOG( TRUE,"sssssssssssssssssssssssssssss\n")
		}
	}
#endif

	if( INVALID_HANDLE_VALUE == hSerialPort )	// skip COM Read/Write if no serial hooked up
	{
		return 0;
	}


#ifdef DEBUG_DYNA_TIMING
	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	ROBOT_LOG( TRUE, "   Dyna WriteFile: %4d ms\n", ProcessingTime )
#endif

	// SEND COMMAND /////////////////////////////////////////////////////////////
#ifdef DEBUG_DYNA_TIMING
	dwWriteStartTime = GetTickCount();
#endif
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainSmartServoThread);
	}
	else
	{
		if(!WriteFile(hSerialPort,			// where to write to (the open comm port)
				&m_DynaCmd,					// what to write
				nBytesToWrite,				// number of BYTEs to be written to port
				&dwBytesWritten,			// number of BYTEs that were actually written
				NULL))						// overlapped I/O not supported			
		{

			ROBOT_DISPLAY( TRUE, "DYNA SERIAL ERROR Sending Command!  (Is Power On? DOH!)\n" )
			PurgeComm(hSerialPort, PURGE_RXABORT|PURGE_RXCLEAR);
			//PurgeComm(hCommPort, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
			DWORD dwCommError = 0;
			ClearCommError(g_hMotorCommPort, &dwCommError, NULL);
			ReportCommError("DYNA SIO Write Error", dwCommError );

			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if( hSerialPort == g_hDynaServoCommPort_RX64 )
				{
					if( SUBSYSTEM_FAILED != g_RX64SubSystemStatus )
					{
						g_RX64SubSystemStatus = SUBSYSTEM_FAILED;
						SpeakText( "Warning, Dyna RX 64 controller has failed");
						ROBOT_DISPLAY( TRUE, "No Response from Dyna RX64 Controller!" )
					}
				}
				else
			#endif

			if( hSerialPort == g_hDynaServoCommPort_AX12 )
			{
				if( SUBSYSTEM_FAILED != g_DynaSubSystemStatus )
				{
					g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
					SpeakText( "Warning, Dyna Servo controller has failed");
					ROBOT_DISPLAY( TRUE, "No Response from Dyna AX12 Controller!" )
				}
			}			
			return 0;
		}
	}

	#ifdef  DEBUG_DYNA_READWRITE_TIME
		dwWriteTime = GetTickCount() - dwWriteStartTime;
		ROBOT_LOG( TRUE,"Dyna Write Time: %d ms\n", dwWriteTime )
	#endif


	if( ResponseExpected )
	{

		RobotSleep(10, pDomainSmartServoThread);	

RETRY_READ:

#ifdef DEBUG_DYNA_TIMING
	//	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	//	ROBOT_LOG( TRUE, "   Dyna ReadFile Start: %4d ms\n", ProcessingTime )
#endif
		// READ RESPONSE /////////////////////////////////////////////////////////////
		// Sent the command, now look for response
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainSmartServoThread);
			return 0;
		}

		SetCommMask (hSerialPort, EV_RXCHAR | EV_BREAK);
	//		memset(m_ReadBuf,0,READ_BUF_SIZE);
		m_DynaReply.ID = 0;
		m_DynaReply.Length = 0;
		m_DynaReply.Error = 0;
		memset(m_DynaReply.Data,0,DYNA_MAX_REPLY_BYTES);

		// Read ANSI characters
		dwReadStartTime = GetTickCount();
		if( INVALID_HANDLE_VALUE == hSerialPort )
		{
			// Update Status to indicate system failure
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == nServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == nServoID) )
					g_RX64SubSystemStatus = SUBSYSTEM_FAILED;
				else
					g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#else
				g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#endif

			return 0;
		}

		if(!ReadFile(hSerialPort,			// where to read from (the open comm port)
				 &m_DynaReply,				// where to put the characters
				 nBytesToRead,				// number of BYTEs to read
				 &dwBytesReceived,			// how many BYTEs were actually read
				 NULL))						// no overlapped I/O
		{

			ROBOT_DISPLAY( TRUE, "DYNA SERIAL READ ERROR (Is Power On? DOH!)\n" )
			PurgeComm(hSerialPort, PURGE_RXABORT|PURGE_RXCLEAR);
			//PurgeComm(hCommPort, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
			DWORD dwCommError = 0;
			ClearCommError(g_hMotorCommPort, &dwCommError, NULL);
			ReportCommError("DYNA SIO Read Error", dwCommError );

			// Update Status to indicate system failure
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == nServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == nServoID) )
					g_RX64SubSystemStatus = SUBSYSTEM_FAILED;
				else
					g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#else
				g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#endif
			return 0;	// TODO: terminate thread on error???
		}
	dwReadTime = GetTickCount() - dwReadStartTime;

#ifdef DEBUG_DYNA_TIMING
		unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
		ROBOT_LOG( TRUE, "   Dyna ReadFile Done: %4d ms\n", ProcessingTime )
#endif
		#ifdef  DEBUG_DYNA_READWRITE_TIME
		ROBOT_LOG( TRUE,"Dyna Write + Read Time: %d ms Bytes Rcd: %d\n", (GetTickCount() - DynaCmdStartTime), dwBytesReceived )
		#endif

		if( 0 == dwBytesReceived )
		{
			// Error, no ACK received
			//ROBOT_LOG( TRUE,"\nDyna WriteTime=%d ms, ReadTime=%d ms,Total= %dms  Bytes Rcd: %d\n", dwWriteTime, dwReadTime, (GetTickCount() - dwWriteStartTime), dwBytesReceived )
			MsgString.Format("Dyna SendCmd:  No Response from Dyna Servo %d!", m_DynaCmd.ID );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			// Update Status to indicate system failure
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if( (DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == nServoID) || (DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == nServoID) )
					g_RX64SubSystemStatus = SUBSYSTEM_FAILED;
				else
					g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#else
				g_DynaSubSystemStatus = SUBSYSTEM_FAILED;
			#endif
			return 0;
		}

		// Got a response
		#ifdef  DEBUG_DYNA_SHOW_ACK_MESSAGES

		if( 7 == m_DynaReply.ID ) // TODO REMOVE THIS HACK FOR SERVO 7 KINECT
		{

			ROBOT_LOG( TRUE,"Dyna RECEIVED (%2d Data Bytes):   Header: %02X %02X, ServoID: %02X, Length: %02X, Error: %02X Data:",
				nResponseBytes, m_DynaReply.Header1, m_DynaReply.Header2, m_DynaReply.ID, m_DynaReply.Length, m_DynaReply.Error )

			if( dwBytesReceived > 6)	// 3 bytes plus checksum
			{
				for( UINT  q=3; q < (dwBytesReceived - 2); q++ )
				{
					ROBOT_LOG( TRUE,"%02X ",	m_DynaReply.Data[q] )
				}
			}
			ROBOT_LOG( TRUE,"\n")

			// DEBUG - TODO KUDGE
			if( 3 == m_DynaCmd.Instruction )
			{
				ROBOT_LOG( TRUE,"sssssssssssssssssssssssssssss\n")
			}

		}

		#endif

		if( dwBytesReceived != (DWORD)nBytesToRead )
		{
			//ROBOT_LOG( TRUE,"\nDyna WriteTime=%d ms, ReadTime=%d ms,Total= %dms  Bytes Rcd: %d\n", dwWriteTime, dwReadTime, (GetTickCount() - dwWriteStartTime), dwBytesReceived )
			ROBOT_LOG( TRUE,"ERROR! Dyna Servo %d: BytesToRead = %d, BytesReceived = %d\n", nServoID, nBytesToRead, dwBytesReceived )
			return 0;
		}

		// Calculate Expected Checksum
		int  CalculatedCheckSum = GetResponseCheckSum( nResponseBytes );
		BYTE ReportedCheckSum = 0;
		ReportedCheckSum = m_DynaReply.Data[nResponseBytes];

		if( CalculatedCheckSum != ReportedCheckSum )
		{
			//ROBOT_LOG( TRUE,"\nDyna WriteTime=%d ms, ReadTime=%d ms,Total= %dms  Bytes Rcd: %d\n", dwWriteTime, dwReadTime, (GetTickCount() - dwWriteStartTime), dwBytesReceived )
			MsgString.Format("Dyna SendCmd: Bad Checksum in Response from Dyna! CalculatedCheckSum = (%04X), Packet CheckSum = (%04X)\n", 
				CalculatedCheckSum, ReportedCheckSum );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			//if( ROBOT_TYPE == TURTLE )
			//{
				//ROBOT_ASSERT(0); // Keep an eye on any errors we are seeing on Turtle!
			//}
			return 0;
		}


		if( m_DynaCmd.ID != m_DynaReply.ID )
		{
			//ROBOT_LOG( TRUE,"\nDyna WriteTime=%d ms, ReadTime=%d ms,Total= %dms  Bytes Rcd: %d\n", dwWriteTime, dwReadTime, (GetTickCount() - dwWriteStartTime), dwBytesReceived )
			MsgString.Format("Dyna SendCmd: Response Servo ID (%d) does not match Cmd Servo ID! (%d)\n", 
				m_DynaReply.ID, m_DynaCmd.ID );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			RobotSleep(5, pDomainSmartServoThread);
			ROBOT_LOG( TRUE,"Dyna SendCmd: RETRYING READ\n")
			goto RETRY_READ;	// usually, got the prior servo, so we are off by one.  Try again.
			//return 0;
		}
	}

	return 1;

}

///////////////////////////////////////////////////////////////////////////////
int  CDynaControl::TenthDegreeToTicks( int  ServoID, int TenthDegrees )
{
	// Convert from "Tenth Degrees" (+/- 1500) to Servo "ticks" (0 - 1023)
	int CompensatedTenthDegrees = TenthDegrees;
	bool HighResMXServo = TRUE;

	// First, compensate for servo offset and rotation direction
	switch( ServoID )
	{
		// Head Servos
		case DYNA_CAMERA_PAN_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees + CAMERA_PAN_TENTH_DEGREES_ZERO);
			if( ROBOT_TYPE == LOKI )
				HighResMXServo = TRUE;
			else
				HighResMXServo = FALSE;
		}
		break;

		case DYNA_CAMERA_TILT_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees + CAMERA_TILT_TENTH_DEGREES_ZERO) * -1;
			if( ROBOT_TYPE == LOKI )
				HighResMXServo = TRUE;
			else
				HighResMXServo = FALSE;
		}
		break;

		case DYNA_CAMERA_SIDETILT_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees + CAMERA_SIDETILT_TENTH_DEGREES_ZERO);
				HighResMXServo = FALSE;  // LOKI Still has an AX12 for the Side-Tilt
		}
		break;

		// Kinect Servo
		case DYNA_KINECT_SCANNER_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees + KINECT_TILT_TENTH_DEGREES_ZERO);
			HighResMXServo = FALSE;
		}
		break;
 
		// Right Arm TenthDegreeToTicks
		case DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID:
			// Both Elbow Servos are backward (mounted upside down), so use negative position.
			// Note - Positive means move to the Robot's Right, Negative means move to the Robot's Left
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO) * -1;
			HighResMXServo = TRUE;
		break;

		case DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID:
			// Servo is backward, so use negative position.
			//CompensatedTenthDegrees = (TenthDegrees + (-620)) * -1;
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO) * -1;
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				HighResMXServo = FALSE;	// RX64 Servo;
			#else
				HighResMXServo = TRUE;	// MX64 Servo;
			#endif
		break;

		case DYNA_RIGHT_ARM_WRIST_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees +  RIGHT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO) * -1;
			HighResMXServo = TRUE;
		break;
		case DYNA_RIGHT_ARM_CLAW_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_CLAW_TENTH_DEGREES_ZERO) * -1;
			HighResMXServo = TRUE;
		break;

		// Left Arm
		case DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID:
			// Both Elbow Servos are backward (mounted upside down), so use negative position.
			// Note - Positive means move to the Robot's Right, Negative means move to the Robot's Left
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO) * -1 ;
			HighResMXServo = TRUE;
		break;
		case DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID:
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO) ;
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				HighResMXServo = FALSE;	// RX64 Servo;
			#else
				HighResMXServo = TRUE;	// MX64 Servo;
			#endif
		break;
		case DYNA_LEFT_ARM_WRIST_SERVO_ID:
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO) * -1 ;
			HighResMXServo = TRUE;
		break;
		case DYNA_LEFT_ARM_CLAW_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_CLAW_TENTH_DEGREES_ZERO)  * -1;
			HighResMXServo = TRUE;
		break;

		default:
			ROBOT_ASSERT(0);
	}

	// Dynamixel servos go from 0 - 300 degrees, or 0-360 degrees (for MX series)
	// We want zero to be at the mid-point, and go +/- from there
	int PositiveTenthDegrees = 0;
	if( HighResMXServo )
	{
		// Convert from +/- 180 degrees to 0-360 degrees.  Note that Dynamixel defines 360 as COUNTER-Clockwise!
		PositiveTenthDegrees = 1800 - CompensatedTenthDegrees;
	}
	else
	{
		// Convert from +/- 150 degrees to 0-300 degrees.  Note that Dynamixel defines 300 as COUNTER-Clockwise!
		PositiveTenthDegrees = 1500 - CompensatedTenthDegrees;
	}

	// None of our servos should ever go the full range
	if( PositiveTenthDegrees > 3000 )
	{
		ROBOT_LOG( TRUE," CDynaControl Error: PositiveTenthDegrees > 3000!\n")
		PositiveTenthDegrees = 3000;
		return 0x3FF; // return max value
	}
	if( PositiveTenthDegrees < 0 )
	{
		ROBOT_LOG( TRUE," CDynaControl Error: PositiveTenthDegrees < 0!\n")
		PositiveTenthDegrees = 0;
		return 0; // return min value
	}

	// Convert to ticks.  Add 0.5 to compensate for truncation error
	double fTicks = 0.0;
	if(	HighResMXServo )
	{
		// MX28 has high resolution
		fTicks  = ((double)PositiveTenthDegrees * (double)DYNA_SERVO_MX_TICKS_PER_TENTHDEGREE) + 0.5;
	}
	else
	{
		// AX12 and RX64 have the same resolution
		fTicks  = ((double)PositiveTenthDegrees * (double)DYNA_SERVO_TICKS_PER_TENTHDEGREE) + 0.5;
	}
	return (int )fTicks;
}

///////////////////////////////////////////////////////////////////////////////
int CDynaControl::TicksToTenthDegree( int  ServoID, int  Ticks )
{
	// Convert from "ticks" (0 - 1023) to "Tenth Degrees" (+/- 1500) 

	// Convert to "raw" tenth degrees (0-3000).  Add 0.5 to compensate for truncation error
	double fRawTenthDegrees = 0.0;
	BOOL HighResMXServo = TRUE;

	#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
		if(	(DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID == ServoID)				||
			(DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID == ServoID)				)
		{
			HighResMXServo = FALSE;	// RX64 Servo; AX12 and RX64 have the same (lower) resolution
		}
	#endif


	if(	( DYNA_KINECT_SCANNER_SERVO_ID == ServoID)							||
		( DYNA_CAMERA_SIDETILT_SERVO_ID == ServoID)							||
		((DYNA_CAMERA_TILT_SERVO_ID == ServoID)	&& (ROBOT_TYPE != LOKI))	||
		((DYNA_CAMERA_PAN_SERVO_ID == ServoID)	&& (ROBOT_TYPE != LOKI))	 )
	{
		HighResMXServo = FALSE; // AX12 and RX64 have the same (lower) resolution
	}


	if( HighResMXServo )
	{
		// MX series has high resolution
		HighResMXServo = TRUE;
		fRawTenthDegrees  = ((double)Ticks * (double)DYNA_SERVO_MX_TENTHDEGREES_PER_TICK);
	}
	else
	{
		fRawTenthDegrees  = ((double)Ticks * (double)DYNA_SERVO_TENTHDEGREES_PER_TICK);
	}

	int RawTenthDegrees = (int)(fRawTenthDegrees + 0.5);


	// Convert from raw to normalized tenth degrees
	// Dynamixel servos go from 0 - 300 degrees, or 0-360 degrees (for MX series)
	// We want zero to be at the mid-point, and go +/- from there

	int TenthDegrees = 0;
	if( HighResMXServo )
	{
		// convert from 0-360 degrees to +/- 180 degrees.  Note that Dynamixel defines 359 as COUNTER-Clockwise!
		TenthDegrees = 1800 - RawTenthDegrees;
	}
	else
	{
		// convert from 0-300 degrees to +/- 150 degrees.  Note that Dynamixel defines 300 as COUNTER-Clockwise!
		TenthDegrees = 1500 - RawTenthDegrees;
	}

	// None of the servos should ever get past +/- 179 degrees 
	if( TenthDegrees > 1790 )
	{
		ROBOT_LOG( TRUE," Error: Reported Degrees > 179! (%d TenthDegrees)\n", TenthDegrees )
	}
	if( TenthDegrees < -1790 )
	{
		ROBOT_LOG( TRUE," Error: Reported Degrees < -179!(%d TenthDegrees)\n", TenthDegrees )
	}

	int CompensatedTenthDegrees = TenthDegrees;
	// Now, compensate for servo offset and rotation direction
	switch( ServoID )
	{
		// Head Servos
		case DYNA_CAMERA_PAN_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees - CAMERA_PAN_TENTH_DEGREES_ZERO);
		}
		break;

		case DYNA_CAMERA_TILT_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees - CAMERA_TILT_TENTH_DEGREES_ZERO) * -1;
		}
		break;

		case DYNA_CAMERA_SIDETILT_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees - CAMERA_SIDETILT_TENTH_DEGREES_ZERO);
		}
		break;

		// Kinect Servo		
		case DYNA_KINECT_SCANNER_SERVO_ID:
		{
			CompensatedTenthDegrees = (TenthDegrees - KINECT_TILT_TENTH_DEGREES_ZERO);
		}
		break;


		// Right Arm TicksToTenthDegree
		case DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO) * -1;
		break;

		case DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID:
			// Servo is backward, so use negative position.
			//CompensatedTenthDegrees = (TenthDegrees + (-620)) * -1;
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO) * -1;
		break;

		case DYNA_RIGHT_ARM_WRIST_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO) * -1;
		break;
		case DYNA_RIGHT_ARM_CLAW_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + RIGHT_ARM_CLAW_TENTH_DEGREES_ZERO) * -1;
		break;


		// Left Arm TicksToTenthDegree
		case DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID:
			// Both Elbow Servos are backward (mounted upside down), so use negative position.
			// Note - Positive means move to the Robot's Right, Negative means move to the Robot's Left
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_ELBOW_ROTATE_TENTH_DEGREES_ZERO) * -1;
		break;

		case DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID:
			// Servo is backward, so use negative position.
//			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO) * -1;
			CompensatedTenthDegrees = (TenthDegrees - LEFT_ARM_ELBOW_BEND_TENTH_DEGREES_ZERO) ;
		break;

		case DYNA_LEFT_ARM_WRIST_SERVO_ID:
			CompensatedTenthDegrees = (TenthDegrees - LEFT_ARM_WRIST_ROTATE_TENTH_DEGREES_ZERO) * -1 ;
		break;
		case DYNA_LEFT_ARM_CLAW_SERVO_ID:
			// Servo is backward, so use negative position.
			CompensatedTenthDegrees = (TenthDegrees + LEFT_ARM_CLAW_TENTH_DEGREES_ZERO)  * -1;
		break;

		default:
			ROBOT_ASSERT(0);
	}


	return (int )CompensatedTenthDegrees;
}

///////////////////////////////////////////////////////////////////////////////
int  CDynaControl::ServoSpeedToDynaServoSpeed( int  ServoSpeed )
{
	int  Velocity;
//	OLD: DynaServoSpeed = ServoSpeed * (DYNA_MOVING_SPEED_MAX / SERVO_SPEED_MAX);	// 	DYNA_MOVING_SPEED_MAX = 0x03FF or 1023

	if( ServoSpeed == SERVO_SPEED_STOP )
	{
		Velocity =		DYNA_VELOCITY_STOP;
	}
	else if( ServoSpeed <= SERVO_SPEED_EXTREMELY_SLOW )
	{
		Velocity =		DYNA_VELOCITY_EXTREMELY_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_VERY_SLOW )
	{
		Velocity =		DYNA_VELOCITY_VERY_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_SLOW )
	{
		Velocity =		DYNA_VELOCITY_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED_SLOW )
	{
		Velocity =		DYNA_VELOCITY_MED_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED )
	{
		Velocity =		DYNA_VELOCITY_MED;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED_FAST )
	{
		Velocity =		DYNA_VELOCITY_MED_FAST;
	}
	else if( ServoSpeed <= SERVO_SPEED_FAST )
	{
		Velocity =		DYNA_VELOCITY_FAST;
	}
	else //if( ServoSpeed <= SERVO_SPEED_MAX )
	{
		Velocity =		DYNA_VELOCITY_MAX;
	}
	
	return Velocity;
}

int  CDynaControl::DynaServoSpeedToServoSpeed( int  DynaServoSpeed )
{
	int  ServoSpeed;
	// Convert from generic Servo Speed to DynaServoSpeed units
	if( DynaServoSpeed >= SERVO_SPEED_MAX )
	{
		ServoSpeed = DYNA_MOVING_SPEED_MAX;
	}
	else
	{
		// convert from Dyna Speed units to Generic Speed units, and multiply by selected speed
		ServoSpeed = DynaServoSpeed * ( SERVO_SPEED_MAX / DYNA_MOVING_SPEED_MAX );	// 	DYNA_MOVING_SPEED_MAX = 0x03FF or 1023
	}
	return ServoSpeed;

}

double CDynaControl::CentigradeToFahrenheit( double Centigrade )
{
	return( ((double)Centigrade * 1.8) + 32.0);
}


///////////////////////////////////////////////////////////////////////////////
void CDynaControl::HandleCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 )
{
	IGNORE_UNUSED_PARAM (Option1);
	IGNORE_UNUSED_PARAM (Option2);
	//TAL_Event("Sending Cmd");
	//-TAL_SCOPED_TASK_NAMED("HandleDynaCommand");

	#ifdef DEBUG_DYNA_TIMING
		DynaCmdStartTime = GetTickCount();	// for measuring how long this pass is taking
		unsigned long RunTime = DynaCmdStartTime - gStartTime;
		RunTime %= (24 * 60 * 60 * 1000);
		RunTime %= (60 * 60 * 1000);
		unsigned int minutes = RunTime / (60 * 1000);
		RunTime %= (60 * 1000);
		unsigned int seconds = RunTime / (1000);
		RunTime %= (1000);
		unsigned int miliseconds = RunTime;
		ROBOT_LOG( TRUE,"--------------------------------------------------\n")
		ROBOT_LOG( TRUE,"Dyna Command Start: %d min, %d sec, %d ms\n", minutes, seconds, miliseconds )
	#endif

	if( !m_PowerEnabled && (HW_SET_CAMERA_POWER != Request) && (HW_SET_POWER_MODE != Request) )
	{
		// Power disabled, ignore all commands but power enable
		return;
	}

	switch( Request )
	{
		// Status Requests
		case HW_GET_STATUS:
		case HW_GET_SERVO_STATUS:
		//case HW_GET_CAMERA_BULK_SERVO_STATUS:	 -- TODO
		{
			if( 0 == Param1 )
			{
				// Kludge - there is no servo zero, so if request for zero comes in dump all Head servos!
				GetAllCameraServosStatus();
			}
			else
			{
				GetServoStatus( Param1 );
			}
			break;
		}

		case HW_GET_SMART_SERVO_STATUS:
		{
			// Get status of all "smart" servos
			// Param1  = Which servos to get
			// Param2  = Get Speed too? (Not implemented)
			//TAL_Event("Get Servo Status");

			int  ServoToPoll = Param1;
			DWORD dwStartTime = GetTickCount();

			if( MOVING_SERVOS == ServoToPoll )
			{
				// Just get status for the servos that are moving
				GetStatusOfMovingServos();
			}
			if( (HEAD_SERVOS == ServoToPoll) || (ALL_SERVOS == ServoToPoll) )
			{
				// Get the status for head servos
				#if (DEBUG_DYNA_SHOW_STATUS_MESSAGES == 1 )
				//ROBOT_LOG( TRUE, "\n============= Dumping Status of all Head Servos =============\n" )
				#endif
				GetServoStatus( DYNA_CAMERA_PAN_SERVO_ID );	
				//GetServoStatus( DYNA_CAMERA_NECK_SERVO_ID );	
				GetServoStatus( DYNA_CAMERA_TILT_SERVO_ID );	
				GetServoStatus( DYNA_CAMERA_SIDETILT_SERVO_ID );	
			}

			if( ROBOT_HAS_KINECT_SERVO && ( (KINECT_SERVO == ServoToPoll) || (ALL_SERVOS == ServoToPoll) ) )
			{
				// Get the status for Kinect servo			
				GetServoStatus( DYNA_KINECT_SCANNER_SERVO_ID );
			}
			if( ROBOT_HAS_RIGHT_ARM && ((RIGHT_ARM == ServoToPoll) || (BOTH_ARMS == ServoToPoll) || (ALL_SERVOS == ServoToPoll)) )
			{
				// Get the status for all right arm servos
				#if (DEBUG_DYNA_SHOW_STATUS_MESSAGES == 1 )
				//ROBOT_LOG( TRUE, "\n============= Dumping Status of all Right Arm Servos =============\n" )
				#endif
				GetServoStatus( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID );	
				GetServoStatus( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID );	
				GetServoStatus( DYNA_RIGHT_ARM_WRIST_SERVO_ID );	
				GetServoStatus( DYNA_RIGHT_ARM_CLAW_SERVO_ID );

			}
			if( ROBOT_HAS_LEFT_ARM && ((LEFT_ARM == ServoToPoll) || (BOTH_ARMS == ServoToPoll) || (ALL_SERVOS == ServoToPoll)) )
			{
				// Get the status for all left arm servos

				#if (DEBUG_DYNA_SHOW_STATUS_MESSAGES == 1 )
				//ROBOT_LOG( TRUE, "\n============= Dumping Status of all Left Arm Servos =============\n" )
				#endif
				GetServoStatus( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID );	
				GetServoStatus( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID );	
				GetServoStatus( DYNA_LEFT_ARM_WRIST_SERVO_ID );	
				GetServoStatus( DYNA_LEFT_ARM_CLAW_SERVO_ID );	
			}

			/////////////////////////////////////
			// DEBUG!
			//}
//			ROBOT_LOG( TRUE,"DEBUG TEST: DYNA Total Status Time (all servos) = %d ms\n", (GetTickCount() - dwStartTime) )

			// OK, now tell the GUI and arm control module that there is a new status update.
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_SERVO_STATUS_READY, 0, 0 );
			//PostThreadMessage( g_dwControlThreadId, WM_ROBOT_SERVO_STATUS_READY, 0, 0);
			SendCommand(WM_ROBOT_SERVO_STATUS_READY, 0, 0);

			/**
			if( SERVO_STATUS_REQUEST_FREQ_FAST == g_nServoStatusRequestsPerSecond )
			{
				if( m_pArmControlRight->CheckArmPosition(FALSE) &&	//	TRUE = Verbose	
					m_pArmControlLeft->CheckArmPosition(FALSE) &&	
					m_pHeadControl->CheckHeadPosition(FALSE)  )	
				{
					// Last movement completed.
					// Request less frequent status updates since arm and head are done moving
					g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_NORMAL;
				}
			}
			**/
			break;
		}

		case HW_INITIALIZE:
		case HW_CAMERA_INITIALIZE:
		{
			Init();
			break;
		}


		case HW_SET_CAMERA_STOP:
		{
			CameraStop();
			break;
		}


		case HW_SET_SERVO_POS_TICKS:
		{	
			if( (0 == Param1) || (Param1 > DYNA_ID_MAX) )
			{
				ROBOT_LOG( TRUE, "DYNA - ERROR: HW_SET_SERVO_POS_TICKS - Bad Servo ID (%d)\n", Param1 )
			}
			else
			{
				PingServo( Param1 );
				SetServoLED( Param1, TRUE );	// Turn on LED
				EnableServoTorque( Param1, TRUE );		// Param1 = ID
				SetServoPosition( Param1, Param2 );	// Param1 = ID, Param2 = position (0-03FF)
				SetServoLED( Param1, FALSE );	// Turn off LED
			}
			break;
		}


		case HW_SET_CAMERA_ABS_PAN_TILT_SPEED:
		{
			// We just set all head servos to the same speed
			// Value passed in is 1 to SERVO_SPEED_MAX.  Calculate Dyna value.
			// Param1 = speed
			// Param2 = Not Used
			static int DebugCmdCount = 0;

			DebugCmdCount++;
			ROBOT_LOG( TRUE,"***************> HEAD MOVE DYNA EXECUTE %d\n", DebugCmdCount)
			if( 0 != Param2 )
			{
				ROBOT_ASSERT(0); // Trap bad usage
			}

			// convert from Camera Speed units to Dyna Speed units
			int  Speed = ServoSpeedToDynaServoSpeed( Param1 );

			SetAllCameraServosSpeed( Speed );
			ROBOT_LOG( TRUE," Servo Speed set to %d (%02Xh)\n", Speed, Speed )

			break;
		}

		case HW_SET_CAMERA_PAN_ABS:
		{
			// Param1 = Requested Pan Position
			// Param2 = not used
			if( !g_CameraServoTorqueEnabled )
			{
				ROBOT_DISPLAY( TRUE, "DYNA: PAN Command ignored, Torque not Enabled!" )
				return;
			}
			int NewPanPositionTenthDegrees = (int)Param1;	// Must cast to int


			/********** NOT USED, SINCE WE SHORTENED THE NECK! ************
			// Don't allow Camera Pan while close to "rest" position. (But do allow centering of Camera Pan)
			if( g_CameraNeckPos < CAMERA_NECK_TENTHDEGREES_PAN_ENABLE_POSITION )
			{
				if( (NewPanPositionTenthDegrees > (CAMERA_PAN_CENTER+5)) || 
					(NewPanPositionTenthDegrees < (CAMERA_PAN_CENTER-5)) )
				{
					ROBOT_DISPLAY( TRUE, "DYNA: PAN Command ignored, Neck too far back!" )
					return;
				}
			}
			**********/

			// Make sure Pan command is within Camera limits
			if( NewPanPositionTenthDegrees < CAMERA_PAN_TENTHDEGREES_MAX_RIGHT )
				NewPanPositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_RIGHT;
			if( NewPanPositionTenthDegrees > CAMERA_PAN_TENTHDEGREES_MAX_LEFT )
				NewPanPositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_LEFT;


			// Convert from +/- Tenth Degrees to Servo Ticks
			int  NewCameraPanPosition = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, NewPanPositionTenthDegrees );


			SetServoLED( DYNA_CAMERA_PAN_SERVO_ID, TRUE );			// Turn on LED
			SetServoPosition( DYNA_CAMERA_PAN_SERVO_ID, NewCameraPanPosition );	// ID, position (0-03FF)
			SetServoLED( DYNA_CAMERA_PAN_SERVO_ID, FALSE );			// Turn off LED

//			g_CameraPanPos = NewPanPositionTenthDegrees;
			
			ROBOT_LOG( TRUE,"Debug Camera - sent Pan Command, Position = %d\n", NewPanPositionTenthDegrees )
			break;
		}

		case HW_SET_CAMERA_TILT_ABS:
		{
			// Param1 = Requested Pan Position
			// Param2 = not used

			int NewTiltPositionTenthDegrees = (int)Param1;	// Must cast to int

			if( !g_CameraServoTorqueEnabled )
			{
				ROBOT_DISPLAY( TRUE, "DYNA: TILT Command ignored, Torque not Enabled!" )
				return;
			}

			/********** NOT USED, SINCE WE SHORTENED THE NECK! ************
			if( g_CameraNeckPos < CAMERA_NECK_TENTHDEGREES_PAN_ENABLE_POSITION )
			{
				// Camera close to rest position.  Limit tilt so it does not hit.
				if( NewTiltPositionTenthDegrees > CAMERA_TILT_TENTHDEGREES_REST_LIMIT )
				{
					if( g_CameraTiltPos < CAMERA_TILT_TENTHDEGREES_REST_LIMIT )
					{
						// Move as far as we can
						NewTiltPositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_REST_LIMIT;
						ROBOT_LOG( TRUE,"DYNA: Setting Tilt to REST LIMIT/n")
					}
					else
					{
						// Can't move any more
						ROBOT_DISPLAY( TRUE, "DYNA: TILT Command ignored, Neck too far back!" )
						return;
					}
				}
			}
			***********/

			// Make sure Tilt command is within Camera limits
			if( NewTiltPositionTenthDegrees < CAMERA_TILT_TENTHDEGREES_MAX_UP )
				NewTiltPositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_UP;
			if( NewTiltPositionTenthDegrees > CAMERA_TILT_TENTHDEGREES_MAX_DOWN )
				NewTiltPositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_DOWN;


			// Convert from +/- Tenth Degrees to Servo Ticks
			int  NewCameraTiltPosition = TenthDegreeToTicks( DYNA_CAMERA_TILT_SERVO_ID, NewTiltPositionTenthDegrees );

			SetServoLED( DYNA_CAMERA_TILT_SERVO_ID, TRUE );			// Turn on LED
			SetServoPosition( DYNA_CAMERA_TILT_SERVO_ID, NewCameraTiltPosition );	// ID, position (0-03FF)
			SetServoLED( DYNA_CAMERA_TILT_SERVO_ID, FALSE );		// Turn off LED

			// g_CameraTiltPos = NewTiltPositionTenthDegrees;
			ROBOT_LOG( TRUE,"Debug Camera - sent Tilt Command, Position = %d\n", NewTiltPositionTenthDegrees )
			break;
		}


		case HW_SET_CAMERA_PAN_TILT_ABS:
		{
			// Param1 = Pan
			// Param2 = Tilt
			int NewPanPositionTenthDegrees = (int)Param1;
			int NewTiltPositionTenthDegrees = (int)Param2;

			// Make sure Pan command is within Camera limits
			if( NewPanPositionTenthDegrees > CAMERA_PAN_TENTHDEGREES_MAX_RIGHT)
				NewPanPositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_RIGHT;
			if( NewPanPositionTenthDegrees < CAMERA_PAN_TENTHDEGREES_MAX_LEFT)
				NewPanPositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_LEFT;

			// Make sure Tilt command is within Camera limits
			if( NewTiltPositionTenthDegrees > CAMERA_TILT_TENTHDEGREES_MAX_UP )
				NewTiltPositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_UP;
			if( NewTiltPositionTenthDegrees < CAMERA_TILT_TENTHDEGREES_MAX_DOWN )
				NewTiltPositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_DOWN;

			if( !g_CameraServoTorqueEnabled )
			{
				ROBOT_DISPLAY( TRUE, "DYNA: CAMERA HW_SET_CAMERA_PAN_TILT_ABS Command ignored, Torque not Enabled!" )
				return;
			}

			int  ServoIDArray[2];		// IDs Servos
			int  ServoValueArray[2];	// Values to set
//			int  ServoSpeedArray[2];	// Speed to set

			// PAN
			int  NewPanPositionTicks = TenthDegreeToTicks( DYNA_CAMERA_PAN_SERVO_ID, NewPanPositionTenthDegrees );
			ServoIDArray[0] = DYNA_CAMERA_PAN_SERVO_ID;		// ID
			ServoValueArray[0] = NewPanPositionTicks;		// position (0-03FF)

			// TILT
			int  NewTiltPositionTicks = TenthDegreeToTicks( DYNA_CAMERA_TILT_SERVO_ID, NewTiltPositionTenthDegrees);
			ServoIDArray[1] = DYNA_CAMERA_TILT_SERVO_ID;	// ID
			ServoValueArray[1] = NewTiltPositionTicks;		// position (0-03FF)

			
//			ServoSpeedArray[0] = 150;
//			ServoSpeedArray[1] = 150;

			// OK, send the command to move the pan and tilt servos
//			SetMultiServoPositionAndSpeed(2, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
			SetMultiServoPosition(2, ServoIDArray, ServoValueArray );	// Number of Servos and command info

			
//			g_CameraPanPos = NewPanPositionTenthDegrees;
//			g_CameraTiltPos = NewTiltPositionTenthDegrees;

/*			if( g_CameraPanPos != NewPanPositionTenthDegrees )
			{
				ROBOT_LOG( TRUE,"ERROR - NewPanPositionTenthDegrees \n")
				//ROBOT_ASSERT(0);
			}
			if( g_CameraTiltPos != NewTiltPositionTenthDegrees )
			{
				ROBOT_LOG( TRUE,"ERROR - NewTiltPositionTenthDegrees \n")
				//ROBOT_ASSERT(0);
			}
*/
			ROBOT_LOG( TRUE,"Debug Camera - got Pan Command, Position = %d\n", NewPanPositionTenthDegrees )
			ROBOT_LOG( TRUE,"Debug Camera - got Tilt Command, Position = %d\n", NewTiltPositionTenthDegrees )
	
			break;
		}

		case HW_SET_CAMERA_FORWARD_ABS:
		{
			// NECK NOT INSTALLED IN THIS VERSION OF LOKI!!!
			ASSERT(0); 
/***
			// Param1 = Requested Forward Neck Position
			// Param2 = not used
			if( !g_CameraServoTorqueEnabled )
			{
				ROBOT_DISPLAY( TRUE, "DYNA: CAMERA FORWARD Command ignored, Torque not Enabled!" )
				return;
			}

			int NewNeckPositionTenthDegrees = (int)Param1;
			int NeckPositionChangeTenthDegrees = NewNeckPositionTenthDegrees - g_CameraNeckPos;
			int  NewNeckPositionTicks = TenthDegreeToTicks( DYNA_CAMERA_NECK_SERVO_ID, NewNeckPositionTenthDegrees );

			int  ServoIDArray[2];		// IDs Servos
			int  ServoValueArray[2];	// Values to set
//			int  ServoSpeedArray[2];	// Speed to set

			// Make sure Forward command is within Camera limits
			if( NewNeckPositionTicks < CAMERA_NECK_SERVO_MAX_FORWARD ) 
				NewNeckPositionTicks = CAMERA_NECK_SERVO_MAX_FORWARD;
			if( NewNeckPositionTicks > CAMERA_NECK_SERVO_MAX_BACK ) 
				NewNeckPositionTicks = CAMERA_NECK_SERVO_MAX_BACK;
		
			ServoIDArray[0] = DYNA_CAMERA_NECK_SERVO_ID;		// ID
			ServoValueArray[0] = NewNeckPositionTicks;		// position (0-03FF)


			// Tilt the head to compensate for neck movement
			int NewTiltPositionTenthDegrees = g_CameraTiltPos + NeckPositionChangeTenthDegrees;

			int  NewTiltPositionTicks = TenthDegreeToTicks( DYNA_CAMERA_TILT_SERVO_ID, (int)NewTiltPositionTenthDegrees);

			// Make sure Tilt command is within Camera limits
			if( NewTiltPositionTicks < CAMERA_TILT_SERVO_MAX_UP )
				NewTiltPositionTicks = CAMERA_TILT_SERVO_MAX_UP;
			if( NewTiltPositionTicks > CAMERA_TILT_SERVO_MAX_DOWN )
				NewTiltPositionTicks = CAMERA_TILT_SERVO_MAX_DOWN;

			ServoIDArray[1] = DYNA_CAMERA_TILT_SERVO_ID;	// ID
			ServoValueArray[1] = NewTiltPositionTicks;		// position (0-03FF)

//			ServoSpeedArray[0] = 150;
//			ServoSpeedArray[1] = 150;

			// OK, send the command to move the neck and tilt
//			SetMultiServoPositionAndSpeed(2, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
			SetMultiServoPosition(2, ServoIDArray, ServoValueArray );	// Number of Servos and command info
			g_CameraNeckPos = NewNeckPositionTenthDegrees;
			g_CameraTiltPos = NewTiltPositionTenthDegrees;
	
			ROBOT_LOG( TRUE,"Debug Camera - sent Forward Command, Position = %04x\n", g_CameraNeckPos )
***/
			break;
		}

		case HW_SET_CAMERA_SIDETILT_ABS:
		{
			// Param1 = Requested Side Tilt Head Position
			// Param2 = not used
			int NewSideTiltPositionTenthDegrees = (int)Param1;

			if( !g_CameraServoTorqueEnabled )
			{
				ROBOT_DISPLAY( TRUE, "DYNA: CAMERA SIDETILT Command ignored, Torque not Enabled!" )
				return;
			}

			// SideTilt servo is backward, so pass in negative position.
			int  NewSideTiltPositionTicks = TenthDegreeToTicks( DYNA_CAMERA_SIDETILT_SERVO_ID, NewSideTiltPositionTenthDegrees );

/***
// Limit values to max camera can handle
			if( NewSideTiltPositionTicks < CAMERA_SIDETILT_SERVO_MAX_LEFT )
			{
				NewSideTiltPositionTicks = CAMERA_SIDETILT_SERVO_MAX_LEFT;
			}
			if( NewSideTiltPositionTicks > CAMERA_SIDETILT_SERVO_MAX_RIGHT )
			{
				NewSideTiltPositionTicks = CAMERA_SIDETILT_SERVO_MAX_RIGHT;
			}
***/
			SetServoLED( DYNA_CAMERA_SIDETILT_SERVO_ID, TRUE );	// Turn on LED
			SetServoPosition( DYNA_CAMERA_SIDETILT_SERVO_ID, NewSideTiltPositionTicks );	// ID, position (0-03FF)
			SetServoLED( DYNA_CAMERA_SIDETILT_SERVO_ID, FALSE );	// Turn off LED
//			g_CameraSideTiltPos = NewSideTiltPositionTenthDegrees;

			ROBOT_LOG( TRUE,"Debug Camera - sent SideTilt Command, Position = %04x\n", NewSideTiltPositionTenthDegrees )
			break;
		}

		case HW_SET_CAMERA_ZOOM:
		{
			// NOT USED FOR DYNA
//				int ZoomCmd = wParam;
//				// TODO: int ZoomSpeed = lParam;
//				if( CAMERA_ZOOM_STOP == ZoomCmd )
//				else if( CAMERA_ZOOM_IN == ZoomCmd )
//				else if( CAMERA_ZOOM_OUT == ZoomCmd )
			break;
		}

		case HW_SET_CAMERA_ZOOM_ABS:
		{
			//TODO - need to figure out how to do digital zoom with new stereo cameras
			//int ZoomLevel = wParam;		// Standard Range: 0 - 16
			// standard Zoom range is 0 to 16.  Map this to Sony range of 0 to 03FF (1023)
			//ROBOT_LOG( TRUE,"Debug Camera - sent Abs Zoom Command, Position = %04x\n", (DWORD)ZoomLevel )

			break;
		}


		case HW_SET_CAMERA_POWER:
		case HW_SET_POWER_MODE:
		{
			if( (POWER_ON == Param1) || (CAMERA_RESET == Param1) )
			{
				// Turn Power On - For Dynamixel Servos, that means enable power and Torque
				if( ROBOT_TYPE == LOKI )
				{
					ROBOT_LOG( TRUE,"Requesting Arduino to Enable Servo Power\n")
					::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_SERVO_POWER, 0, 1 ); // Turn ON Dynamixel Servo Power for Loki
					RobotSleep(5, pDomainSmartServoThread);
				}
				ROBOT_LOG( TRUE,"Moving to Wake Up Position\n")
				m_PowerEnabled = TRUE;
				EnableCameraServosTorque( TRUE );
				if( LOKI == ROBOT_TYPE )
				{
					EnableRightArmTorque( TRUE );
					EnableLeftArmTorque( TRUE );
				}
				GotoStartupPosition();
			}
			else
			{
				// Shut Down - Return to "Rest" position, and turn torque off
				if( SYSTEM_SHUT_DOWN == Param1 )
				{
					ROBOT_LOG( TRUE,"DYNA - Moving to Sleep Position\n")
					m_PowerEnabled = FALSE; // prevent further servo commands from other modules while shutting down
					GotoSleepPosition();
					if( ROBOT_TYPE == LOKI )
					{
						ROBOT_LOG( TRUE,"DYNA - Waiting for Move to complete...\n")
						RobotSleep(2000, pDomainSmartServoThread); // Give time for head and arms to return to home position
						ROBOT_LOG( TRUE,"DYNA - Disabling Right Arm Torque\n")
						EnableRightArmTorque( FALSE );
						ROBOT_LOG( TRUE,"DYNA - Disabling Left Arm Torque\n")
						EnableLeftArmTorque( FALSE );
						ROBOT_LOG( TRUE,"DYNA - Disabling Head Torque\n")
						EnableCameraServosTorque( FALSE );
						// Remove Servo Power
						::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_SERVO_POWER, 0, 0 ); // Turn off Dynamixel Servo Power for Loki
					}
					else
					{
						ROBOT_LOG( TRUE,"DYNA - Waiting for Move to complete...\n")
						RobotSleep(1000, pDomainSmartServoThread); // Give time for Kinect to move to sleep position
						ROBOT_LOG( TRUE,"DYNA - Disabling Head Torque\n")
						EnableCameraServosTorque( FALSE );
						// Tell iRobot Controller to turn off power to the servos
						//::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_DYNA_USB, 0, 0 ); // Turn off Dynamixel Servo Power
						::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_SERVO_POWER, 0, 0 ); // Turn off Dynamixel Servo Power for Turtle
						
					}
				}
				else
				{
					GotoSleepPosition();
					RobotSleep(2500, pDomainSmartServoThread); // Give time for head and arms to return to home position
					EnableRightArmTorque( FALSE );
					EnableLeftArmTorque( FALSE );
					EnableCameraServosTorque( FALSE );
				}
			}
			break;
		}

		case HW_SET_CAMERA_PAN_TILT:
		{
			// Slow Pan or tilt - TODO figure out how to emulate this...
			ROBOT_DISPLAY( TRUE, "ANALOG PAN/TILT NOT CURRENTLY SUPPORTED" )

			//int Direction = wParam;	// Pan/Tilt direction
			//BYTE nPanTiltSpeed = lParam;
			/***
			switch( Direction )  // Which direction to Pan/Tilt
			{

				case CAMERA_PAN_STOP:
				{
					break;
				}
				case CAMERA_PAN_UP:
				{
					break;
				}
				case CAMERA_PAN_DOWN:
				{
					break;
				}

				case CAMERA_PAN_LEFT:
				{
					break;
				}

				case CAMERA_PAN_RIGHT:
				{
					break;
				}

				// Combo Pan/Tilt commands
				case CAMERA_PAN_UP_LEFT:
				{
					break;
				}
				case CAMERA_PAN_UP_RIGHT:
				{
					break;
				}
				case CAMERA_PAN_DOWN_LEFT:
				{
					break;
				}
				case CAMERA_PAN_DOWN_RIGHT:
				{
					break;
				}
				case CAMERA_PAN_ABS_CENTER:
				{
					break;
				}
				default:
				{
					ROBOT_LOG( TRUE, "ERROR! Invalid Camera Pan Direction, wParam = 0x%08lX\n", wParam )
				}
				break;
			}
			***/
			break;
		}	

		////////////////////////////////////////////////////////////////////
		// Arm Servos

		case HW_SET_SERVO_TORQUE_ENABLE:
		{
			// Update torque enable of all servos with "Update" flagged in the global g_BulkServoCmd buffer
			ROBOT_LOG( TRUE,"DYNA - Setting Servo Torque\n")

			// Head servos
			if( g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Update )
			{
				EnableServoTorque( DYNA_CAMERA_PAN_SERVO_ID, g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Enable  ); // ID, Enable
			}
			if( g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Update )
			{
				EnableServoTorque( DYNA_CAMERA_TILT_SERVO_ID, g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Enable  ); // ID, Enable
			}
			if( g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Update )
			{
				EnableServoTorque( DYNA_CAMERA_SIDETILT_SERVO_ID, g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Enable  ); // ID, Enable
			}

			// Right Arm Servos
			#if ROBOT_HAS_RIGHT_ARM
				if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable  ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_RIGHT_ARM_WRIST_SERVO_ID, g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_RIGHT_ARM_CLAW_SERVO_ID, g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable ); // ID, Enable
				}
			#endif

			// Left Arm Servos
			#if ROBOT_HAS_LEFT_ARM
				if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable  ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_LEFT_ARM_WRIST_SERVO_ID, g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable ); // ID, Enable
				}
				if( g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update )
				{
					EnableServoTorque( DYNA_LEFT_ARM_CLAW_SERVO_ID, g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable ); // ID, Enable
				}
			#endif

			// Indicate to the system that the servo enable command has been handled

			g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Update = FALSE;

			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update = FALSE;

			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update = FALSE;
			g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update = FALSE;

			break;
		}


		case HW_SET_SERVO_COMPLIANCE:
		{
			// Set compliance slope of the specified servo
			// This is how accurate the servo will track to requested
			// but may also put extra stress on the servo.  See DYNA_COMPLIANCE_TIGHT
			int  ServoNo = Param1;
			int  Compliance = Param2;
			//ROBOT_LOG( TRUE,"DYNA - Setting Servo %d Compliance to %d\n", ServoNo, Compliance)
			SetServoComplianceSlope( ServoNo, Compliance );
			break;
		}

		case HW_SET_SERVO_TORQUE_LIMIT:
		{
			// Set torque limit of the specified servo
			int  ServoNo = Param1;
			int  TorqueValue = Param2;
			if( TorqueValue > DYNA_TORQUE_LIMIT_MAX )
			{
				ROBOT_LOG( TRUE,"ERROR: DYNA HW_SET_SERVO_TORQUE_LIMIT exceeds DYNA_TORQUE_LIMIT_MAX\n")
				TorqueValue = DYNA_TORQUE_LIMIT_MAX;
			}			
			ROBOT_LOG( TRUE,"DYNA - Setting Servo Torque Limit\n")
			SetServoMaxTorque( ServoNo, TorqueValue );	// 0 - 1023 (DYNA_TORQUE_LIMIT_MAX)	
			break;
		}

		case HW_SET_BULK_HEAD_POSITION:
		{
			// Update all head servos flagged in the global g_BulkServoCmd buffer
			// Param1  = not used
			// Param2  = Set Speed of each servo (True/False)

			int FirstServo = DYNA_CAMERA_PAN_SERVO_ID;
			int LastServo = DYNA_CAMERA_SIDETILT_SERVO_ID;

			if( 0 == NUMBER_OF_DYNA_SERVOS_IN_HEAD )
			{
				// This robot has no head
				break;
			}
			else if( 1 == NUMBER_OF_DYNA_SERVOS_IN_HEAD )
			{
				// This robot has no tilt
				LastServo = DYNA_CAMERA_PAN_SERVO_ID;
			}
			else if( 2 == NUMBER_OF_DYNA_SERVOS_IN_HEAD )
			{
				// This robot has no side-tilt
				LastServo = DYNA_CAMERA_TILT_SERVO_ID;
			}

			int  PositionTicks = 0;
			int  ServoIDArray[NUMBER_OF_DYNA_SERVOS_IN_HEAD+1];		// IDs Servos
			int  ServoValueArray[NUMBER_OF_DYNA_SERVOS_IN_HEAD+1];	// Values to set
			int  ServoSpeedArray[NUMBER_OF_DYNA_SERVOS_IN_HEAD+1];	// Speed to set
			int  nServoToSet = 0;
			//ROBOT_LOG( TRUE,"\nDYNA DEBUG: HW_SET_BULK_HEAD_POSITION\n")


			for( int  ServoID= FirstServo; ServoID <= LastServo; ServoID++ )
			{
				//if( 2 == ServoID) continue;	// skip the neck servo (not installed)

				// See if this servo command has been updated
				if( g_BulkServoCmd[ServoID].Update)
				{
					// Yes, send a command to this servo
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: UPDATE Servo %d\n", ServoID)
					// Convert each value from TenthDegrees to Servo Ticks	
					PositionTicks = TenthDegreeToTicks( ServoID, g_BulkServoCmd[ServoID].PositionTenthDegrees ); 
					// Place data into servo array to send via broadcast to the servos
					ServoIDArray[nServoToSet] = ServoID;														// ID
					ServoValueArray[nServoToSet] = PositionTicks;												// Position (0-03FF)
					ServoSpeedArray[nServoToSet] = ServoSpeedToDynaServoSpeed( g_BulkServoCmd[ServoID].Speed );	// Speed
					// Indicate to the system that the servo position has been handled (reset for next time)
					g_BulkServoCmd[ServoID].Update = FALSE;
					nServoToSet++;
				}
			}
		
			// OK, send the command to move the servos
			if( 0 == nServoToSet )
			{
				//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending commands to no Servos\n")
			}
			else
			{
				//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending commands to %d Servos\n", nServoToSet)

				if( Param2 )
				{
					// Param2 true = set the speed of each servo too
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending SPEED (%d) and commands to %d Servos\n", g_BulkServoCmd[1].Speed, nServoToSet)
					SetMultiServoPositionAndSpeed( nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
				}
				else
				{
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending commands to %d Servos\n", nServoToSet)
					SetMultiServoPosition( nServoToSet, ServoIDArray, ServoValueArray );	// Number of Servos and command info
				}

				// Request more frequent status updates while the head is moving
				//g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_FAST;
			}
			//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Done\n\n")

			break;
		}

		case HW_SET_BULK_KINECT_POSITION:
		{
			// Update Kinect servo if flagged in the global g_BulkServoCmd buffer
			// Param1  = not used
			// Param2  = Set Speed of servo (True/False)
			int  PositionTicks = 0;
			//ROBOT_LOG( TRUE, "DEBUG: HW_SET_BULK_KINECT_POSITION called\n")
			if( !ROBOT_HAS_KINECT_SERVO )
			{
				break;
			}

			// See if this servo command has been updated
			if( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update)
			{	
				// Yes, send a command to this servo
				ROBOT_LOG( TRUE, "KINECT Moving to %d Degrees\n", (g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees) / 10)

				// See if command is within limits
				if( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees > KINECT_TILT_TENTHDEGREES_MAX_UP )
				{
					CString MsgString;
					MsgString.Format( "ERROR! DYNA: HW_SET_BULK_KINECT_POSITION exceeds MAX UP degree limit (%d, %d)", 
						g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees, KINECT_TILT_TENTHDEGREES_MAX_UP);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees = KINECT_TILT_TENTHDEGREES_MAX_UP;
					PositionTicks = KINECT_TILT_TENTHDEGREES_MAX_UP;
				}
				else if( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees < KINECT_TILT_TENTHDEGREES_MAX_DOWN )
				{
					CString MsgString;
					MsgString.Format( "ERROR! DYNA: HW_SET_BULK_KINECT_POSITION exceeds MAX DOWN degree limit (%d, %d)", 
						g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees, KINECT_TILT_TENTHDEGREES_MAX_DOWN);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees = KINECT_TILT_TENTHDEGREES_MAX_DOWN;
					PositionTicks = KINECT_TILT_TENTHDEGREES_MAX_DOWN;
				}
	

				PositionTicks = TenthDegreeToTicks( DYNA_KINECT_SCANNER_SERVO_ID, g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees ); 
				// Indicate to the system that the servo position has been handled (reset for next time)
				g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update = FALSE;

				if( Param2 )
				{	// Param2 true = set the speed of servo too
					int  ServoSpeed = ServoSpeedToDynaServoSpeed( g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Speed );	// Speed
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending Speed %d and Position %d to Kinect Servo\n",
					//	ServoSpeed,	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees )
					SetServoPositionAndSpeed( DYNA_KINECT_SCANNER_SERVO_ID, PositionTicks, ServoSpeed);	// ID, position (0-03FF), speed
				}
				else
				{
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending Position %d to Kinect Servo\n",
//						g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees )
					SetServoPosition( DYNA_KINECT_SCANNER_SERVO_ID, PositionTicks );	// ID, position (0-03FF)
				}

				// Request more frequent status updates while the laser is moving
				//g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_FAST;
			}
			else
			{
				ROBOT_LOG( TRUE,"**** ERROR?  HW_SET_BULK_KINECT_POSITION called, but g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update not set?\n")
			}

			break;
		}

		case HW_SET_BULK_ARM_POSITION:
		{
			// Update all arm servos flagged in the global g_BulkServoCmd buffer
			// Param1  = Right Arm or Left Arm
			// Param2  = Set Speed of each servo (True/False)
			//ROBOT_DISPLAY( TRUE, "=========> DYNA: Processing HW_SET_BULK_ARM_POSITION  <=========\n" )

//			int  PositionTicks = 0;
			int  nServoToSet = 0;
			BOOL WaitForDelay = TRUE;
			DWORD DebugTime = GetTickCount();
			int  ServoIDArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// IDs Servos
			int  ServoValueArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Values to set
			int  ServoSpeedArray[NUMBER_OF_DYNA_AX12_SERVOS_PER_ARM];		// Speed to set

			// Make sure shoulder motors have finished initialization before allowing other commands
/*
			if( !gKerrControlInitialized )
			{
				ROBOT_DISPLAY( TRUE, "Arm servo command ignored.  Shoulders not yet calibrated")
				return;
			}
			*/

			// Save a copy of the global command buffer, in case it changes while we are processing the command
			{

					__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, psh_csServoLock);
						EnterCriticalSection(&g_csServoLock);
							memcpy( m_BulkServoCmdCopy, g_BulkServoCmd, (sizeof(BULK_SERVO_CMD_T))* NUMBER_OF_SMART_SERVOS+1 );
						LeaveCriticalSection(&g_csServoLock);
					__itt_task_end(pDomainSmartServoThread);

					// DEBUG!!!
					/*	if( m_BulkServoCmdCopy[ServoID].Delay != 0 )
						{
							ROBOT_LOG( TRUE,"STOP!\n")
						}
					*/
					// Indicate to the system that the servo position has been handled (reset for next time)

					if( ROBOT_HAS_RIGHT_ARM && (RIGHT_ARM == Param1) )
					{
						g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update = FALSE;
					}
					if( ROBOT_HAS_LEFT_ARM && (LEFT_ARM == Param1) )
					{
						g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update = FALSE;
						g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update = FALSE;
					}
					else
					{
						return; // if no arms installed
					}

			}
			while( WaitForDelay )
			{
				// Loop as many times as needed to handle delayed servo moves
				// Alway TRUE first time through
  
				//ROBOT_LOG( TRUE,"\nDYNA DEBUG: HW_SET_BULK_ARM_POSITION Time = %ld\n", (GetTickCount()-DebugTime) )
				if( RIGHT_ARM == Param1 )
				{

					#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
						HandleRX64Servo( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, (BOOL)Param2 );	// Handle Elbow separately (RX64 on separate COM port)
					#endif
					PrepMultiServoData( DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_RIGHT_ARM_WRIST_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_RIGHT_ARM_CLAW_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
				}
				else // Left Arm
				{
					#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
						HandleRX64Servo( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, (BOOL)Param2 ); 	// Handle Elbow separately (RX64 on separate COM port)
					#endif

					PrepMultiServoData( DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_LEFT_ARM_WRIST_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
					PrepMultiServoData( DYNA_LEFT_ARM_CLAW_SERVO_ID, nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );
				}

				// OK, send the broadcast command to move all of the AX-12 servos
				if( 0 == nServoToSet )
				{
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending commands to no Servos\n")
				}
				else
				{
					// OK DO IT!  Send comamnds to the servos!
					//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Sending commands to %d Servos, time = %ld\n", nServoToSet, (GetTickCount()-DebugTime))
					if( Param2 )
					{
						// Param2 true = set the speed of each servo too
						SetMultiServoPositionAndSpeed( nServoToSet, ServoIDArray, ServoValueArray, ServoSpeedArray );	// Number of Servos and command info
					}
					else
					{
						SetMultiServoPosition( nServoToSet, ServoIDArray, ServoValueArray );	// Number of Servos and command info
					}

					// Request more frequent status updates while the arm is moving
					//g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_FAST;
				}
				//ROBOT_LOG( TRUE,"\nDYNA DEBUG: Done\n\n")

				///////////////////////////////////////////////////////////////////////////////
				// OK, now check to see if we have any delayed servo movements
				// Note that this thread blocks until all servo delays have been satisfied,
				// so only use for short delays
				int  count = 0;
				int  NextDelayTime = 0xFFFF;
				if( RIGHT_ARM == Param1 )
				{
					// Find the shortest delay time
					for( int  ServoID= DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID; ServoID <= DYNA_RIGHT_ARM_CLAW_SERVO_ID; ServoID++ )
					{
						// See if this servo command has been updated
						if( (m_BulkServoCmdCopy[ServoID].Update) && (m_BulkServoCmdCopy[ServoID].Delay != 0) )
						{
							// Need to wait for delay on for this servo. Save the lowest value time delay so we know which to do next
							if ( m_BulkServoCmdCopy[ServoID].Delay < NextDelayTime )
							{
								NextDelayTime = m_BulkServoCmdCopy[ServoID].Delay;
							}
						}
					}
					// Now, subtract the delay time from each remaining servo.
					// Servos with zero delay time + Update=True will be executed at the end of the delay
					if( 0xFFFF != NextDelayTime )
					{
						///TAL_SCOPED_TASK_NAMED("R Delay Servo!");

						for( int  ServoID= DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID; ServoID <= DYNA_RIGHT_ARM_CLAW_SERVO_ID; ServoID++ )
						{
							if( (m_BulkServoCmdCopy[ServoID].Update) && (m_BulkServoCmdCopy[ServoID].Delay >= NextDelayTime) )
							{
								m_BulkServoCmdCopy[ServoID].Delay -= NextDelayTime;
							}
						}
						// Now, wait the delay time before moving the next set of servos!
						ROBOT_LOG( TRUE,"Dynamixel Sleeping for Delay = %d\n", NextDelayTime )
						RobotSleep( NextDelayTime, pDomainSmartServoThread );
					}
					else
					{
						WaitForDelay = FALSE; // Done
					}
				}
				else
				{	//LEFT ARM
					// Find the shortest delay time
					for( int  ServoID= DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID; ServoID <= DYNA_LEFT_ARM_CLAW_SERVO_ID; ServoID++ )
					{
						// See if this servo command has been updated
						if( (m_BulkServoCmdCopy[ServoID].Update) && (m_BulkServoCmdCopy[ServoID].Delay != 0) )
						{
							// Need to wait for delay on for this servo
							// Save the lowest value time delay so we know which to do next
							if ( m_BulkServoCmdCopy[ServoID].Delay < NextDelayTime )
							{
								NextDelayTime = m_BulkServoCmdCopy[ServoID].Delay;
							}
						}
					}
					// Now, subtract the delay time from each remaining servo.
					// Servos with zero delay time + Update=True will be executed at the end of the delay
					if( 0xFFFF != NextDelayTime )
					{
						///TAL_SCOPED_TASK_NAMED("L Delay Servo!");
						for( int  ServoID= DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID; ServoID <= DYNA_LEFT_ARM_CLAW_SERVO_ID; ServoID++ )
						{
							if( (m_BulkServoCmdCopy[ServoID].Update) && (m_BulkServoCmdCopy[ServoID].Delay >= NextDelayTime) )
							{
								m_BulkServoCmdCopy[ServoID].Delay -= NextDelayTime;
							}
						}
						// Now, wait the delay time before moving the next set of servos!
						ROBOT_LOG( TRUE,"Dynamixel Sleeping for Delay = %d\n", NextDelayTime )
						RobotSleep( NextDelayTime, pDomainSmartServoThread );
					}
					else
					{
						WaitForDelay = FALSE; // Done
					}
				}

			}	// end while(WaitForDelay)
			break;
		}
		case HW_READ_SERVO_REGISTER:
		{
			// Unpack Servo ID and Register Number
			//MAKEWORD(m_nDynaServoTestID,m_nDynaServoRegNum ); // Servo number and Register Number packed together
			int  ServoID = (BYTE)(Param1 & 0xFF);
			int  Register = (BYTE)(Param1 >> 8);
			int  NumberOfRegisters = Param2;
			ReadServoRegisters( ServoID, Register, NumberOfRegisters);
			break;

		}
		case HW_WRITE_SERVO_REGISTER_BYTE:
		{
			// Unpack Servo ID and Register Number
			int  ServoID = (BYTE)(Param1 & 0xFF);
			int  Register = (BYTE)(Param1 >> 8);
			int  Value = Param2;
			WriteServoRegisterByte( ServoID, Register, Value);
			break;

		}
		case HW_WRITE_SERVO_REGISTER_WORD:
		{
			// Unpack Servo ID and Register Number
			int  ServoID = (BYTE)(Param1 & 0xFF);
			int  Register = (BYTE)(Param1 >> 8);
			int  Value = Param2;
			WriteServoRegisterWord( ServoID, Register, Value);
			break;

		}
		default:
		{
			CString StrText;
			StrText.Format( "ERROR! DynaServoCommThread: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, StrText )
		}
	}

#ifdef DEBUG_DYNA_TIMING
	unsigned long ProcessingTime = GetTickCount() - DynaCmdStartTime;
	ROBOT_LOG( TRUE, "Dyna Command End: %4d ms\n", ProcessingTime )
	ROBOT_LOG( TRUE,"--------------------------------------------------\n")
#endif

}


void CDynaControl::PrepMultiServoData( int  ServoID, int  &nServoToSet, int  *ServoIDArray, int  *ServoValueArray, int  *ServoSpeedArray )
{
	// Pack servo data into buffer for broadcast send
	int  PositionTicks = 0;

	// First, reset the servo if torque was exceeded.
	// TODO-LOKI - remove this later, after arm is debugged
	if( g_BulkServoStatus[ServoID].StatusFlags & 0x20 )
	{	// Torque was exceeded, reset it
		ROBOT_LOG( TRUE,"DYNA: RE-ENABLING SERVO %d TORQUE!\n", ServoID )
		EnableServoTorque( ServoID, TRUE );		// ID, Enable
	}

	// See if this servo command has been updated, and the delay is zero
	if( (m_BulkServoCmdCopy[ServoID].Update) && (0 == m_BulkServoCmdCopy[ServoID].Delay) )
	{
		// Yes, send a command to this servo
		//ROBOT_LOG( TRUE,"\nDYNA DEBUG: UPDATE Servo %d\n", ServoID)
		// Convert each value from TenthDegrees to Servo Ticks	
		PositionTicks = TenthDegreeToTicks( ServoID, m_BulkServoCmdCopy[ServoID].PositionTenthDegrees ); 
		// Place data into servo array to send via broadcast to the servos
		ServoIDArray[nServoToSet] = ServoID;														// ID
		ServoValueArray[nServoToSet] = PositionTicks;												// Position (0-03FF)
		ServoSpeedArray[nServoToSet] = ServoSpeedToDynaServoSpeed( m_BulkServoCmdCopy[ServoID].Speed );	// Speed
		// Indicate that the servo position has been handled
		m_BulkServoCmdCopy[ServoID].Update = FALSE;
		nServoToSet++;
		//ROBOT_LOG( TRUE,"Dynamixel: Handling Servo %d\n", ServoID)
	}
}

void CDynaControl::HandleRX64Servo( int  ServoID, BOOL SetSpeed )

//, int  &nServoToSet, int  *ServoIDArray, int  *ServoValueArray, int  *ServoSpeedArray )
{
	#if ( DYNA_SERVO_RX64_INSTALLED != 1 )
		// NO LONGER USED
		ROBOT_ASSERT(0);
	#endif

	// RX64 servos (in elbows) are controlled through a separate COM port
	int  PositionTicks = 0;

	// First, reset the servo if torque was exceeded.
	// TODO-LOKI - remove this later, after arm is debugged
	if( g_BulkServoStatus[ServoID].StatusFlags & 0x20 )
	{	// Torque was exceeded, reset it
		ROBOT_LOG( TRUE,"DYNA: RESETTING RX64 SERVO %d TORQUE!\n", ServoID )
		EnableServoTorque( ServoID, TRUE );		// ID, Enable
	}

	// See if this servo command has been updated, and the delay is zero
	if( (m_BulkServoCmdCopy[ServoID].Update) && (0 == m_BulkServoCmdCopy[ServoID].Delay) )
	{
		// Yes, send a command to this servo
		//ROBOT_LOG( TRUE,"\nDYNA DEBUG: UPDATE Servo %d\n", ServoID)

		// Convert the value from TenthDegrees to Servo Ticks	
		PositionTicks = TenthDegreeToTicks( ServoID, m_BulkServoCmdCopy[ServoID].PositionTenthDegrees );

		// Format command and send to the servo
		if( SetSpeed )
		{
			//  set the speed of the servo too
			int  DynaServoSpeed = ServoSpeedToDynaServoSpeed( m_BulkServoCmdCopy[ServoID].Speed );
			SetServoPositionAndSpeed( ServoID, PositionTicks, DynaServoSpeed );
		}
		else
		{
			SetServoPosition( ServoID, PositionTicks );
		}

		// Indicate that the servo position has been handled
		m_BulkServoCmdCopy[ServoID].Update = FALSE;

		//ROBOT_LOG( TRUE,"Dynamixel: Handling RX64 Servo %d\n", ServoID)

		//SetPositionAndSpeed
	}
}

#endif // ROBOT_SERVER - This module used for Robot Server only

