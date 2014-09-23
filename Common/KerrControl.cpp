// KerrControl.cpp
// Interface for Kerr Servo controller


// This class is used by the thread loop that reads commands from the queue
// in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include "Globals.h"
#include "KerrControl.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_KERR_COMMAND_DUMP   // Dump all commands
//#define DEBUG_KERR_SHOW_ACK_MESSAGES
//#define DEBUG_KERR_SHOW_STATUS_MESSAGES
//#define DEBUG_KERR_TIMING
//#define DEBUG_KERR_READWRITE_TIME

// Servo Offset Compensation - use if Clean Up the Floor does not work!
#define RIGHT_ARM_SHOULDER_TENTH_DEGREES_ZERO	     (-10)	// TenthDegrees!
#define LEFT_ARM_SHOULDER_TENTH_DEGREES_ZERO		 (-40)	// TenthDegrees!  more negative moves arm further back


/***********************************************************
// command packet sent to Kerr controller
// 0      1         2                         3...         3 + data-length
// [0xAA] [address] [Length Nib][Command Nib] [...data...] [checksum]
typedef struct KerrCmd_struct {
	BYTE Header;	KERR_SIO_HEADER_BYTE (0xAA)
	BYTE ID;		// Command + Length!!!
	BYTE Command;
	BYTE Data[KERR_MAX_DATA_BYTES]; // (includes CheckSum after the data)
} KerrCmd_T;

// status packet returned from Kerr Servo:
// 0      1      2    3        4       5            5 + data-length
// [0xFF] [0xFF] [id] [length] [error] [...data...] [checksum]
typedef struct KerrReply_struct {
	BYTE Status;
	BYTE Data[KERR_MAX_REPLY_BYTES]; // (includes CheckSum after the data)
} KerrReply_T;
***********************************************************/



///////////////////////////////////////////////////////////////////////////////
// Kerr Control Class
///////////////////////////////////////////////////////////////////////////////



CKerrControl::CKerrControl()
{
	m_KerrCmd.Header = KERR_SIO_HEADER_BYTE;	// 0xAA
	m_KerrCmd.ID = 0;			// Address of Servo to access
	m_KerrCmd.Command = 0;		
	memset(m_KerrCmd.Data,0,KERR_MAX_DATA_BYTES);

	memset(m_KerrReplyBuf,0,KERR_READ_BUF_SIZE);

	m_HomePositionInitializedRight = FALSE;
	m_HomePositionInitializedLeft = FALSE;
	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
	KerrCmdStartTime = 0;
	m_ControllerIDsInitialized = FALSE;
	m_PowerEnabled = TRUE; //enable power on start up


//	memset(m_ReadBuf,0,READ_BUF_SIZE);
}

CKerrControl::~CKerrControl()
{
	SAFE_DELETE(m_pArmControlRight);
	SAFE_DELETE(m_pArmControlLeft);
}

///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////

void CKerrControl::AddCheckSum(int  nParamBytes)
{
	// Calculate Checksum, and insert it into the packet
	// Usage: 	DoCheckSum(nParamBytes);

	int  CheckSum = m_KerrCmd.ID;
	CheckSum += m_KerrCmd.Command;

	for ( int  i=0 ; i<nParamBytes ; i++ )
	{
		CheckSum += m_KerrCmd.Data[i];
	}
// Not used for Kerr:	CheckSum = (~CheckSum);
	CheckSum = CheckSum & 0x00FF;	// save lower 8 bits only
	m_KerrCmd.Data[nParamBytes] = (BYTE)CheckSum;	// Notice this is lower 8 bits only
}

int  CKerrControl::GetResponseCheckSum(int  nResponseBytes)
{
	// Calculate Checksum of the returned data
	int  CheckSum = 0;
	for ( int  i=0 ; i<nResponseBytes ; i++ )
	{
		CheckSum += m_KerrReplyBuf[i];
	}
	// Not used for Kerr - CheckSum = (~CheckSum);
	CheckSum = CheckSum & 0x00FF;	// save lower 8 bits only
	return CheckSum;	// Notice this is lower 8 bits only
}


///////////////////////////////////////////////////////////////////////////////
void CKerrControl::InitControllers()
{
	// Kerr controller require the ID of each Servo sub-controller to be initialized at startup
	// All controllers start with ID=0, but only the first will respond.
	// So we change first one to 1, and the second to 2.

	/* 
	From the Kerr "Arduino-Servo SC" user's manual: PicSrvSC.pdf (page 6):
	1. On power-up, all modules assume a default address of 0x00, and each will set its ADDR_OUT
	signal HIGH. Furthermore, a module’s communications will be disabled completely until its
	ADDR_IN signal goes LOW. If the ADDR_OUT and ADDR_IN signals are daisy-chained as
	described above, all modules will be disabled except for the module furthest from the host.
	2. The host starts by sending a Set Address command to module 0, changing its address to a value of
	1. A side affect of the Set Address command is that the module will lower the its ADDR_OUT
	signal.
	3. At this point, the next module in line is enabled with an address of 0. The host then sends a
	command to module 0 to change its address to a value of 2.
	4. This process is continued until all modules have been assigned unique addresses.
	*/

	if( m_ControllerIDsInitialized )
	{
		return; // only do once!
	}
	m_ControllerIDsInitialized = TRUE;

	// Send a bunch of zeros to get the Kerr control synchronized
	m_KerrCmd.ID = 0;			// Address of Servo to access use 0 for initial start-up
	m_KerrCmd.Command = 0;			
	m_KerrCmd.Data[0] = 0;
	m_KerrCmd.Data[1] = 0;
	m_KerrCmd.Data[2] = 0;
	m_KerrCmd.Data[3] = 0;
	m_KerrCmd.Data[4] = 0;
	m_KerrCmd.Data[5] = 0;
	m_KerrCmd.Data[6] = 0;
	m_KerrCmd.Data[7] = 0;
	m_KerrCmd.Data[8] = 0;
	m_KerrCmd.Data[9] = 0;
	m_KerrCmd.Data[10] = 0;
	m_KerrCmd.Data[11] = 0;
	m_KerrCmd.Data[12] = 0;
	m_KerrCmd.Data[13] = 0;
	m_KerrCmd.Data[14] = 0;
	m_KerrCmd.Data[15] = 0;
	SendCmd(16,0, FALSE);	// no response expected, just flushing the port


	m_KerrCmd.ID = 0;			// Address of Servo to access use 0 for initial start-up
	m_KerrCmd.Command = 0x0E;	// Send NOP to just get a status back (which is ignored)		
	SendCmd(0, 0, FALSE);


	m_KerrCmd.ID = 0xFF;			// Address of Servo - FF = all servos (default group)
	m_KerrCmd.Command = KERR_CMD_HARD_RESET;	// High Nibble contains number of Data Bytes		
	SendCmd(0,0, FALSE);	// No response to Reset command

	Sleep(100);

	// Purge the comm port input and output of junk
	if( (INVALID_HANDLE_VALUE != g_hKerrServoCommPort)  && (SIMULATED_SIO_HANDLE != g_hKerrServoCommPort) )
	{
		if( !PurgeComm(g_hKerrServoCommPort, (PURGE_TXCLEAR | PURGE_RXCLEAR)) )
		{
			ROBOT_ASSERT(0);	// Purge failed.  Need to call GetLastError().
		}
	}

	// Set the first servo controller ID
	m_KerrCmd.ID = 0;			// Address of Servo to access use 0 for initial start-up
	m_KerrCmd.Command = KERR_CMD_SET_ADDRESS + 0x20;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = KERR_ARM_MOTOR_ID_RIGHT;		// Individual Address = 1
	m_KerrCmd.Data[1] = 0xFF;	// No Group Address
	SendCmd(2);

	Sleep(100);
	
	// Set the second servo controller ID
	m_KerrCmd.ID = 0;			// Address of Servo to access use 0 for initial start-up
	m_KerrCmd.Command = KERR_CMD_SET_ADDRESS + 0x20;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = KERR_ARM_MOTOR_ID_LEFT;		// Individual Address = 2
	m_KerrCmd.Data[1] = 0xFF;	// No Group Address
	SendCmd(2);

	
// Set the Amplifier Gain

	m_KerrCmd.ID = KERR_ARM_MOTOR_ID_RIGHT;			// Address of Servo
	m_KerrCmd.Command  = 0xF6;			
	m_KerrCmd.Data[0]  = 0x64;
	m_KerrCmd.Data[1]  = 0x00;
	m_KerrCmd.Data[2]  = 0xE8;
	m_KerrCmd.Data[3]  = 0x03;
	m_KerrCmd.Data[4]  = 0x00;
	m_KerrCmd.Data[5]  = 0x00;
	m_KerrCmd.Data[6]  = 0x00;
	m_KerrCmd.Data[7]  = 0x00;
	m_KerrCmd.Data[8]  = 0xFF;
	m_KerrCmd.Data[9]  = 0x00;
	m_KerrCmd.Data[10] = 0xA0;
	m_KerrCmd.Data[11] = 0x0F;
	m_KerrCmd.Data[12] = 0x01;
	m_KerrCmd.Data[13] = 0x01;
	m_KerrCmd.Data[14] = 0x01;
	SendCmd(15);

	m_KerrCmd.ID = KERR_ARM_MOTOR_ID_LEFT;			// Address of Servo
	m_KerrCmd.Command  = 0xF6;			
	m_KerrCmd.Data[0]  = 0x64;
	m_KerrCmd.Data[1]  = 0x00;
	m_KerrCmd.Data[2]  = 0xE8;
	m_KerrCmd.Data[3]  = 0x03;
	m_KerrCmd.Data[4]  = 0x00;
	m_KerrCmd.Data[5]  = 0x00;
	m_KerrCmd.Data[6]  = 0x00;
	m_KerrCmd.Data[7]  = 0x00;
	m_KerrCmd.Data[8]  = 0xFF;
	m_KerrCmd.Data[9]  = 0x00;
	m_KerrCmd.Data[10] = 0xA0;
	m_KerrCmd.Data[11] = 0x0F;
	m_KerrCmd.Data[12] = 0x01;
	m_KerrCmd.Data[13] = 0x01;
	m_KerrCmd.Data[14] = 0x01;
	SendCmd(15);


	// In case the arm was up when reset, the counter is now not at zero
	// so, reset it to get it close enough to allow home calibration to work right
	m_KerrCmd.ID = KERR_ARM_MOTOR_ID_RIGHT;	// First Arm
	m_KerrCmd.Command = KERR_CMD_RESET_POS;	// High Nibble contains number of Data Bytes (none)		
	SendCmd(0);

	m_KerrCmd.ID = KERR_ARM_MOTOR_ID_LEFT;	// Second Arm
	m_KerrCmd.Command = KERR_CMD_RESET_POS;	// High Nibble contains number of Data Bytes (none)		
	SendCmd(0);

}


///////////////////////////////////////////////////////////////////////////////
void CKerrControl::EnableServoTorque( int  MotorNumber, BOOL bEnable )
{

	// send command to enable servo Amplifier
	m_KerrCmd.ID = (BYTE)MotorNumber;			// Address of Servo to access use 0 for initial start-up
	m_KerrCmd.Command = KERR_CMD_STOP_SERVO + 0x10;	// High Nibble contains number of Data Bytes		

	if(bEnable)
	{
		ROBOT_LOG( TRUE,"KERR - Enabling Torque on Shoulder Motor %d\n", MotorNumber)
		m_KerrCmd.Data[0] = KERR_AMP_ENABLE;	// Enable/Disable bit 0
		SendCmd(1);

		// Now, if home position not set, do it!
		if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
		{
			if( !m_HomePositionInitializedRight )
			{
				CalibrateHomePosition( MotorNumber );
				// Set motor speed and move home
				SetArmPositionAndSpeed( MotorNumber, (RIGHT_ARM_SHOULDER_HOME1 * 10), SERVO_SPEED_MED_SLOW );
				g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = RIGHT_ARM_SHOULDER_HOME1 * 10; // make sure g_BulkServoCmd matches
			}
		}
		else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
		{
			if( !m_HomePositionInitializedLeft )
			{
				CalibrateHomePosition( MotorNumber );
				// Set motor speed and move home
				SetArmPositionAndSpeed( MotorNumber, (LEFT_ARM_SHOULDER_HOME1 * 10), SERVO_SPEED_MED_SLOW );
				g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = LEFT_ARM_SHOULDER_HOME1 * 10; // make sure g_BulkServoCmd matches
			}
		}
		else
		{
			ROBOT_ASSERT(0);
		}
	}
	else
	{
		// Disable Torque
		m_KerrCmd.Data[0] = KERR_SERVO_OFF;		// Enable/Disable
		SendCmd(1);

		if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
		{
			m_HomePositionInitializedRight = FALSE;
			ROBOT_LOG( TRUE,"KERR - Right Shoulder Torque Disabled\n")
		}
		else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
		{
			m_HomePositionInitializedLeft = FALSE;
			ROBOT_LOG( TRUE,"KERR - Left Shoulder Torque Disabled\n")
		}
		else
		{
			ROBOT_ASSERT(0);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////
void CKerrControl::DisplayServoStatus(int  MotorNumber, int  nResponseBytes)
{
	CString MsgString;
	KerrReplyLong_T	*pKerrReply = (KerrReplyLong_T*)m_KerrReplyBuf;
	int  ServoID = MotorNumberToServoID( MotorNumber );

	// Display Basic Status 
	if( !(pKerrReply->Status & KERR_MOVE_DONE) )
	{
		#ifdef DEBUG_KERR_SHOW_STATUS_MESSAGES
		ROBOT_LOG( TRUE, "========> Kerr Servo %d: Move In Progress\n", ServoID )
		#endif
	}
	if( pKerrReply->Status & KERR_CKSUM_ERROR )
	{
		MsgString.Format( "========> Kerr Servo %d: CheckSum Error!", ServoID );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
	}
	if( pKerrReply->Status & KERR_OVERCURRENT )
	{
		MsgString.Format( "========> Kerr Servo %d: OverCurrent Error!\n", ServoID );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
	}
	if( !(pKerrReply->Status & KERR_POWER_ON) )
	{
		MsgString.Format( "========> Kerr Servo %d: Power is Off!\n", ServoID );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
	}
	if( pKerrReply->Status & KERR_POS_ERR )
	{
		MsgString.Format( "========> Kerr Servo %d: Position Error!", ServoID );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
	}
	if( !(pKerrReply->Status & KERR_LIMIT1) )	// Active Low
	{
		#ifdef DEBUG_KERR_SHOW_STATUS_MESSAGES
		MsgString.Format( "========> Kerr Servo %d: Detect on Limit Switch 1\n", ServoID );
		ROBOT_LOG( TRUE, (LPCTSTR)MsgString )
		#endif
	}
	if( !(pKerrReply->Status & KERR_LIMIT2) )	// Active Low
	{
		MsgString.Format( "========> Kerr Servo %d: Detect on Limit Switch 2\n", ServoID );
		ROBOT_LOG( TRUE, (LPCTSTR)MsgString )
	}
	if( pKerrReply->Status & KERR_HOME_IN_PROG )
	{
		TRACE("." );
	}

	
	// Now update the global status block with this Servo's BASIC info. (see BULK_SERVO_STATUS_T)
	g_BulkServoStatus[ServoID].StatusFlags = pKerrReply->Status;

	if( nResponseBytes > 1 )
	{
		// Display extended status
		int PositionTicks = pKerrReply->Position;

		int  Load = pKerrReply->Load;	// Motor current (0 - 255)
		int PositionTenthDegrees = TicksToTenthDegree( ServoID, PositionTicks );
//		if( KERR_RIGHT_ARM_SHOULDER_SERVO_ID == ServoID )
//		{
//			PositionTicks = PositionTicks * -1;// Need to reverse +/- for Right Arm!
//		}

		#ifdef DEBUG_KERR_SHOW_STATUS_MESSAGES

		ROBOT_LOG( TRUE, "KERR - Status for Servo %d:   \t(raw hex values)\n", ServoID )
		ROBOT_LOG( TRUE, "       Position =     %4d       (%04Xh ticks)\n", PositionTenthDegrees, PositionTicks )
		ROBOT_LOG( TRUE, "       Load =         %4d       (%04Xh)\n", Load, Load)
		#endif

		// Update the global status block with this Servo's extended info. (see KERR_SERVO_STATUS_T)
		g_BulkServoStatus[ServoID].PositionTenthDegrees = PositionTenthDegrees;
		g_BulkServoStatus[ServoID].Load = Load;
	}
	// and Notify the GUI
//	PostMessage( g_RobotSetupViewHWND, WM_ROBOT_SERVO_STATUS_READY, 0, 0 );

}

///////////////////////////////////////////////////////////////////////////////
void CKerrControl::ServoSpeedToKerrSpeed(int  ServoSpeed, int  &Velocity, int  &Acceleration )
{
	// Covert the given arm speed to Kerr values

	if( ServoSpeed == SERVO_SPEED_STOP )
	{
		Velocity =		KERR_VELOCITY_STOP;
		Acceleration =	KERR_ACCELERATION_STOP;
	}
	else if( ServoSpeed <= SERVO_SPEED_VERY_SLOW )
	{
		Velocity =		KERR_VELOCITY_VERY_SLOW;
		Acceleration =	KERR_ACCELERATION_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_SLOW )
	{
		Velocity =		KERR_VELOCITY_SLOW;
		Acceleration =	KERR_ACCELERATION_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED_SLOW )
	{
		Velocity =		KERR_VELOCITY_MED_SLOW;
		Acceleration =	KERR_ACCELERATION_MED_SLOW;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED )
	{
		Velocity =		KERR_VELOCITY_MED;
		Acceleration =	KERR_ACCELERATION_MED;
	}
	else if( ServoSpeed <= SERVO_SPEED_MED_FAST )
	{
		Velocity =		KERR_VELOCITY_MED_FAST;
		Acceleration =	KERR_ACCELERATION_MED_FAST;
	}
	else if( ServoSpeed <= SERVO_SPEED_FAST )
	{
		Velocity =		KERR_VELOCITY_FAST;
		Acceleration =	KERR_ACCELERATION_FAST;
	}
	else if( ServoSpeed <= SERVO_SPEED_MAX )
	{
		Velocity =		KERR_VELOCITY_MAX;
		Acceleration =	KERR_ACCELERATION_MAX;
	}
	else
	{
		ROBOT_ASSERT( 0 );
	}
}

///////////////////////////////////////////////////////////////////////////////
void CKerrControl::SetSpeed(int  MotorNumber, int  Speed)
{

	// Set the speed and Trapezoidal profile for motor commands

	int  Velocity, Acceleration;
	ServoSpeedToKerrSpeed( Speed, Velocity, Acceleration );	// Get Vel and Acc from Speed

	m_KerrCmd.ID = (BYTE)MotorNumber;
	m_KerrCmd.Command = KERR_CMD_LOAD_TRAJECTORY + 0x90;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = (
		//KERR_LOAD_POS			// Load Position Data - add 4 bytes
		KERR_LOAD_VELOCITY	|	// Load Velocity Data - add 4 bytes
		KERR_LOAD_ACC		|	// Load Acceleration Data - add 4 bytes
		//KERR_LOAD_PWM			// Load PWM Data - add 1 byte
		KERR_ENABLE_SERVO		// Enable PID Servo 
		//KERR_VEL_MODE			// Profile Mode: 0 = Trapezoidal, 1 = Velocity
		//KERR_MOVE_REL			// Move Relative to current Position
		//KERR_START_NOW		|	// Start motion immediately
		);	// Bit Flags for desired fields to update

	// Set Velocity for movement:
	m_KerrCmd.Data[1] = LOBYTE(LOWORD(Velocity));
	m_KerrCmd.Data[2] = HIBYTE(LOWORD(Velocity));
	m_KerrCmd.Data[3] = LOBYTE(HIWORD(Velocity));
	m_KerrCmd.Data[4] = HIBYTE(HIWORD(Velocity));

	// Set Acceleration for movement (ramp up and ramp down)
	m_KerrCmd.Data[5] = LOBYTE(LOWORD(Acceleration));
	m_KerrCmd.Data[6] = HIBYTE(LOWORD(Acceleration));
	m_KerrCmd.Data[7] = LOBYTE(HIWORD(Acceleration));
	m_KerrCmd.Data[8] = HIBYTE(HIWORD(Acceleration));
	SendCmd(9);	// (always 1 more then the last Data[] value)

}


///////////////////////////////////////////////////////////////////////////////
void CKerrControl::Init()
{
	// Reset everything to default values
	m_KerrCmd.Header = KERR_SIO_HEADER_BYTE;	// 0xAA
	m_KerrCmd.ID = 0;			// Address of Servo to access
	m_KerrCmd.Command = 0;		
	memset(m_KerrCmd.Data,0,KERR_MAX_DATA_BYTES);
	memset(m_KerrReplyBuf,0,KERR_READ_BUF_SIZE);

	KerrCmdStartTime = GetTickCount();

//	m_nStatusBytes = 1;	// Initially, only one status byte is returned

	// Kerr controller require the ID of each Servo sub-controller to be initialized at startup
	// All controllers start with ID=0, but only the first will respond.
	// So we change first one to 1, and the second to 2.
	InitControllers();	// Assign a unique address to each module, and initialize for use

	// Make sure init is totally done before changing how status is returned (to avoid checksum errors)
	Sleep(500);
	if( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable )
	{
		// Arm movement is enabled by GUI.
		// Enable the servo and find Home position
		EnableServoTorque( KERR_ARM_MOTOR_ID_RIGHT, TRUE );
	}

	if( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable )
	{
		// Arm movement is enabled by GUI.
		// Enable the servo and find Home position
		EnableServoTorque( KERR_ARM_MOTOR_ID_LEFT, TRUE );
	}

	// TODO-LOKI
	// Set shutdown conditions for Servos
	//	SetShutdownConditions( ID, ShutdownFlags );

	ROBOT_DISPLAY( TRUE, "Kerr Controller initialized" )
	gKerrControlInitialized = TRUE;	// Allows arm commands to be processed

}

void CKerrControl::GotoSleepPosition()  // for Shutdown?
{
	// Put both arms "Rest" Position
	// Only move if servos are enabled
	ROBOT_DISPLAY( TRUE, "Moving Arms to sleep position" )
	int SleepOffset = 100; // TenthDegrees to keep arm from falling after power off

	if( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable )
	{
		g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = (RIGHT_ARM_SHOULDER_HOME2 * 10) + SleepOffset; // make sure g_BulkServoCmd matches
		SetArmPositionAndSpeed( KERR_ARM_MOTOR_ID_RIGHT, (RIGHT_ARM_SHOULDER_HOME2 * 10) + SleepOffset, SERVO_SPEED_MED_SLOW );
	}

	if( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable )
	{
		g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = (LEFT_ARM_SHOULDER_HOME2 * 10) + SleepOffset; // make sure g_BulkServoCmd matches
		SetArmPositionAndSpeed( KERR_ARM_MOTOR_ID_LEFT, (LEFT_ARM_SHOULDER_HOME2 * 10) + SleepOffset, SERVO_SPEED_MED_SLOW );
	}
}


void CKerrControl::CalibrateHomePosition( int  MotorNumber )
{
	if( (KERR_ARM_MOTOR_ID_RIGHT == MotorNumber) &&
		( !g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable) )
	{
		ROBOT_LOG( TRUE,"\nRight Arm CalibrateHomePosition Aborted: Arm not enabled \n\n")
		g_RightArmSubSystemStatus = SUBSYSTEM_DISABLED;
		return;
	}

	if( (KERR_ARM_MOTOR_ID_LEFT == MotorNumber) &&
		( !g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable) )
	{
		ROBOT_LOG( TRUE,"\nLeft Arm CalibrateHomePosition Aborted: Arm not enabled \n\n")
		g_LeftArmSubSystemStatus = SUBSYSTEM_DISABLED;
		return;
	}

	int i;
	KerrReplyLong_T	*pKerrReply = (KerrReplyLong_T*)m_KerrReplyBuf;

	// Move to position 1 (arm ahead of body)
	int  PositionTenthDegrees = 300;	//30.0 degrees forward
	SetArmPositionAndSpeed( MotorNumber, PositionTenthDegrees, SERVO_SPEED_MED );

	Sleep(1000);	// Need to wait for the move to start before checking the flag
	if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
	{
		ROBOT_DISPLAY( TRUE, "Right Arm Moving to Home Position, Step 1" )
	}
	else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
	{
		ROBOT_DISPLAY( TRUE, "Left Arm Moving to Home Position, Step 1" )
	}
	else
	{
		ROBOT_ASSERT(0); // Bad Motor Number
	}

	BOOL fSuccess = FALSE;
	for(i=0; i<20; i++)	// seconds / 5
	{
		Sleep(200);	// ms

		GetServoStatus( MotorNumber );

		if( pKerrReply->Status & KERR_MOVE_DONE )
		{
			fSuccess = TRUE;
			ROBOT_LOG( TRUE, "========> Arm Homing Step 1 Complete\n" )
			break;
		}
		else
		{
			ROBOT_LOG( TRUE, "Arm Homing Step 1 in progress\n" )
		}
	}
	if( !fSuccess )
	{
		ROBOT_DISPLAY( TRUE, "*****************************************" )
		ROBOT_DISPLAY( TRUE, "ERROR: ARM MOVE FORWARD TIMED OUT!" )
		ROBOT_DISPLAY( TRUE, "*****************************************" )
		g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
	}


	// Now set Homing Mode
	m_KerrCmd.ID = (BYTE)MotorNumber;			// Kerr Address of Motor
	m_KerrCmd.Command = KERR_CMD_SET_HOMING + 0x10;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = KERR_ON_LIMIT1 | KERR_HOME_STOP_SMOOTH; //KERR_HOME_STOP_ABRUPT; // Stop when Limit 1 found
	SendCmd(1);
	
//	Sleep(200);

	// And move to position 2
	ROBOT_DISPLAY( TRUE, "Arm Moving to Home Position, Step 2" )
	PositionTenthDegrees = -450;	//45.0 degrees back
	SetArmPositionAndSpeed( MotorNumber, PositionTenthDegrees, SERVO_SPEED_SLOW );

	ROBOT_DISPLAY( TRUE, "Arm Moving to Home Position, Step 3" )
	if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
	{
		ROBOT_LOG( TRUE,"Right Arm Home in progress")
	}
	else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
	{
		ROBOT_LOG( TRUE,"Left Arm Home in progress")
	}

	for(i=0; i<50; i++)	// seconds / 5
	{
		Sleep(200);	// ms

		GetServoStatus( MotorNumber );

		if( !(pKerrReply->Status & KERR_HOME_IN_PROG) )
		{
			ROBOT_LOG( TRUE, " Home DONE\n" )
			// if homing sucessful, reset positon relative to home
			m_KerrCmd.ID = (BYTE)MotorNumber;			// Address of Servo to access use 0 for initial start-up
			m_KerrCmd.Command = KERR_CMD_RESET_POS + 0x10;	// High Nibble contains number of Data Bytes		
			m_KerrCmd.Data[0] = KERR_REL_HOME; // Home will now be position 0000
			SendCmd(1);
			if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
			{
				m_HomePositionInitializedRight = TRUE;
				g_RightArmSubSystemStatus = SUBSYSTEM_CONNECTED;
			}
			else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
			{
				m_HomePositionInitializedLeft = TRUE;
				g_LeftArmSubSystemStatus = SUBSYSTEM_CONNECTED;
			}
			else
			{
				ROBOT_ASSERT(0);
			}
			return;
		}
	}
	ROBOT_DISPLAY( TRUE, "*****************************************" )
	ROBOT_DISPLAY( TRUE, "ERROR! Arm not able to find Home Position!!!" )
	ROBOT_DISPLAY( TRUE, "*****************************************" )
	if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
		g_RightArmSubSystemStatus = SUBSYSTEM_FAILED;
	if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
		g_LeftArmSubSystemStatus = SUBSYSTEM_FAILED;
}



///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
void CKerrControl::SetArmPosition(int  MotorNumber, int PositionTenthDegrees)
{
	// send command to move arm

#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "   Kerr SetArmPosition Start: %4d ms\n", ProcessingTime )
#endif

	if( 0 != gServoOverheatError )
	{
		ROBOT_DISPLAY( TRUE, "KERR - GLOBAL OVERHEAT ERROR set! All moves ignored!")
		return;
	}


//	ROBOT_LOG( TRUE,"KERR - Set Shoulder Motor %d to Position %3.2f degrees\n", MotorNumber,  ((double)PositionTenthDegrees / 10.0) )
		DWORD CurrentTime = GetTickCount() - gStartTime;
		CString strStatus;
		strStatus.Format("Kerr SetArmPosition [%04d.%d]\n",
			(CurrentTime /1000), CurrentTime%1000 );	//
		ROBOT_LOG( TRUE, strStatus )

	if( PositionTenthDegrees > 3500)
	{
		ROBOT_LOG( TRUE,"KERR - ERROR! Position > Max!  Ignored!\n")
		return;
	}
	if( PositionTenthDegrees < -3500)
	{
		ROBOT_LOG( TRUE,"KERR - ERROR! Position > Max!  Ignored!\n")
		return;
	}

	// Convert TenthDegrees to Kerr units
/*	if( KERR_ARM_MOTOR_ID_RIGHT == ServoID )
	{
		PositionTenthDegrees = PositionTenthDegrees * -1;// Need to reverse +/- for Right Arm!
	}
*/
	int PositionTicks = TenthDegreeToTicks( MotorNumber, PositionTenthDegrees );



	// Set the speed and Trapezoidal profile for motor commands
	// So we get all the status we need with any command
	m_KerrCmd.ID = (BYTE)MotorNumber;
	m_KerrCmd.Command = KERR_CMD_LOAD_TRAJECTORY + 0x50;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = (
		KERR_LOAD_POS		|	// Load Position Data - add 4 bytes
		//KERR_LOAD_VELOCITY	|	// Load Velocity Data - add 4 bytes
		//KERR_LOAD_ACC		|	// Load Acceleration Data - add 4 bytes
		//KERR_LOAD_PWM			// Load PWM Data - add 1 byte
		KERR_ENABLE_SERVO	|	// Enable PID Servo 
		//KERR_VEL_MODE			// Profile Mode: 0 = Trapezoidal, 1 = Velocity
		//KERR_MOVE_REL			// Move Relative to current Position
		KERR_START_NOW			// Start motion immediately
		);	// Bit Flags for desired fields to update

	// Set Position:
	m_KerrCmd.Data[1] = LOBYTE(LOWORD(PositionTicks));
	m_KerrCmd.Data[2] = HIBYTE(LOWORD(PositionTicks));
	m_KerrCmd.Data[3] = LOBYTE(HIWORD(PositionTicks));
	m_KerrCmd.Data[4] = HIBYTE(HIWORD(PositionTicks));

	SendCmd(5);	// (always 1 more then the last Data[] value)
}

///////////////////////////////////////////////////////////////////////////////
void CKerrControl::SetArmPositionAndSpeed(int  MotorNumber, int PositionTenthDegrees, int  Speed)
{
	// send command to move arm
#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "   Kerr SetArmPositionAndSpeed Start: %4d ms\n", ProcessingTime )
#endif


//	ROBOT_LOG( TRUE,"KERR - Set Shoulder Motor %d to Position %3.2f degrees\n", MotorNumber,  ((double)PositionTenthDegrees / 10.0) )
	/*
		DWORD CurrentTime = GetTickCount() - gStartTime;
		CString strStatus;
		strStatus.Format("Kerr SetArmPositionAndSpeed [%04d.%d]\n",
			(CurrentTime /1000), CurrentTime%1000 );	//
		ROBOT_LOG( TRUE, strStatus )
	*/

	if( PositionTenthDegrees > 3500)
	{
		ROBOT_LOG( TRUE,"KERR - ERROR! Position > Max!  Ignored!\n")
		return;
	}
	if( PositionTenthDegrees < -3500)
	{
		ROBOT_LOG( TRUE,"KERR - ERROR! Position > Max!  Ignored!\n")
		return;
	}

/*	// Convert TenthDegrees to Kerr units
	if( KERR_ARM_MOTOR_ID_RIGHT == ServoID )
	{
		PositionTenthDegrees = PositionTenthDegrees * -1;// Need to reverse +/- for Right Arm!
	}
*/
	int PositionTicks = TenthDegreeToTicks( MotorNumber, PositionTenthDegrees );

	int  Velocity, Acceleration;
	ServoSpeedToKerrSpeed( Speed, Velocity, Acceleration );	// Get Vel and Acc from Speed


	// Set the speed and Trapezoidal profile for motor commands
	// So we get all the status we need with any command
	m_KerrCmd.ID = (BYTE)MotorNumber;
	m_KerrCmd.Command = KERR_CMD_LOAD_TRAJECTORY + 0xD0;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = (
		KERR_LOAD_POS		|	// Load Position Data - add 4 bytes
		KERR_LOAD_VELOCITY	|	// Load Velocity Data - add 4 bytes
		KERR_LOAD_ACC		|	// Load Acceleration Data - add 4 bytes
		//KERR_LOAD_PWM			// Load PWM Data - add 1 byte
		KERR_ENABLE_SERVO	|	// Enable PID Servo 
		//KERR_VEL_MODE			// Profile Mode: 0 = Trapezoidal, 1 = Velocity
		//KERR_MOVE_REL			// Move Relative to current Position
		KERR_START_NOW			// Start motion immediately
		);	// Bit Flags for desired fields to update

	// Set Position:
	m_KerrCmd.Data[1] = LOBYTE(LOWORD(PositionTicks));
	m_KerrCmd.Data[2] = HIBYTE(LOWORD(PositionTicks));
	m_KerrCmd.Data[3] = LOBYTE(HIWORD(PositionTicks));
	m_KerrCmd.Data[4] = HIBYTE(HIWORD(PositionTicks));

	// Set Velocity for movement:
	m_KerrCmd.Data[5] = LOBYTE(LOWORD(Velocity));
	m_KerrCmd.Data[6] = HIBYTE(LOWORD(Velocity));
	m_KerrCmd.Data[7] = LOBYTE(HIWORD(Velocity));
	m_KerrCmd.Data[8] = HIBYTE(HIWORD(Velocity));

	// Set Acceleration for movement (ramp up and ramp down)
	m_KerrCmd.Data[9] = LOBYTE(LOWORD(Acceleration));
	m_KerrCmd.Data[10] = HIBYTE(LOWORD(Acceleration));
	m_KerrCmd.Data[11] = LOBYTE(HIWORD(Acceleration));
	m_KerrCmd.Data[12] = HIBYTE(HIWORD(Acceleration));
	SendCmd(13);	// (always 1 more then the last Data[] value)
}




///////////////////////////////////////////////////////////////////////////////
void CKerrControl::GetServoStatus(int  MotorNumber)
{
	// Request Servo Status
#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "   Kerr GetServoStatus Start: %4d ms\n", ProcessingTime )
#endif

	/* a simple NoOp will return basic status	
	m_KerrCmd.ID = MotorNumber;
	m_KerrCmd.Command = KERR_CMD_NOP; // NoOp - simply gets status
	SendCmd(0);	// Number of extra data BYTEs to Write and Read
	*/

	// But to get real status, call Read Status
	m_KerrCmd.ID = (BYTE)MotorNumber;
	m_KerrCmd.Command = KERR_CMD_READ_STATUS + 0x10;	// High Nibble contains number of Data Bytes		
	m_KerrCmd.Data[0] = (KERR_GET_POSITION | KERR_GET_LOAD_CURRENT);	// Bit Flags for desired status info
	SendCmd(1,6);	// Normal call returns 1 byte by default.  we add 5 more for extended status (4+1)
	

}


///////////////////////////////////////////////////////////////////////////////
BOOL CKerrControl::SendCmd(int  nParamBytes, int  nResponseBytes, BOOL ResponseExpected )
{
	// Send command to Kerr Servo.  Returns 1 for good status, 0 if error occured.
	// By default, nResponseBytes = 1 byte, and ResponseExpected = TRUE
	/* 
	Command Packet format:
	1. Header Byte (0xAA)
	2. Servo ID
	3. Command
	   Data bytes (nParamBytes; 0 to 15 bytes)
	4. CheckSum (8 bit sum, not including Header Byte)

	Status Packet format:
	Basic Status (before Status has been configured)
	- Status Byte
	- CheckSum (8 bit sum of all bytes)

	Status Request
	- Status Byte
	- Data Bytes ( nResponseBytes+1 ) )
	- CheckSum (8 bit sum of all bytes)
	*/
	if( ROBOT_TYPE == TURTLE )
	{
		return 0; 
	}

	CString MsgString;
	DWORD dwBytesWritten=0;
//	int  Status = 0;
//	int  ReadCheckSum = 0;
	unsigned char DbgBuf[16];
	memset(DbgBuf,0, 16);
//	int  q;
	DWORD dwBytesReceived = 0;
	DWORD dwStartTime = 0;

	// Calculate CheckSum and insert it into the command buffer
	AddCheckSum( nParamBytes );

	int  nBytesToWrite = nParamBytes + 4;	// Commands are  3 bytes long + nParamBytes + Checksum
	int  nBytesToRead = nResponseBytes + 1;	// Responses are Status Byte + (optional: AdditonalStatusBytes) + Checksum
	

	// Debug:
//	MsgString.Format( "Kerr SendCmd:  Sending cmd %02X to Kerr controller.\n", m_KerrCmd.Code );
//	ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )

	
#ifdef DEBUG_KERR_COMMAND_DUMP

	if( KERR_CMD_NOP != m_KerrCmd.Command )
	{
		memcpy( DbgBuf, &m_KerrCmd, KERR_MAX_DATA_BYTES);
		ROBOT_LOG( TRUE,"Kerr CMD DUMP (%d nBytesToWrite): ", nBytesToWrite )

		for( int  q=0; q < nBytesToWrite; q++ )
		{
			ROBOT_LOG( TRUE,"%02X ",	DbgBuf[q] );
		}
		ROBOT_LOG( TRUE,"\n" );
	}
#endif

	if( INVALID_HANDLE_VALUE == g_hKerrServoCommPort )	// skip COM Read/Write if no serial hooked up
	{
		return 0;
	}


#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "   Kerr WriteFile Start: %4d ms\n", ProcessingTime )
#endif

	// SEND COMMAND /////////////////////////////////////////////////////////////
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		Sleep(SIMULATED_SIO_SLEEP_TIME);
	}
	else
	{
		dwStartTime = GetTickCount();
		if(!WriteFile(g_hKerrServoCommPort,	// where to write to (the open comm port)
				&m_KerrCmd,					// what to write
				nBytesToWrite,				// number of BYTEs to be written to port
				&dwBytesWritten,			// number of BYTEs that were actually written
				NULL))						// overlapped I/O not supported			
		{
			ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to KerrServoCommPort!\n" )
			DWORD ErrorCode = GetLastError();	// See Help: " System Errors "
			g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
			return 0;
		}

		#ifdef  DEBUG_KERR_READWRITE_TIME
			ROBOT_LOG( TRUE,"Kerr Write Time: %d ms\n", (GetTickCount() - dwStartTime) )
		#endif
	}

	if( ResponseExpected )
	{
		// READ RESPONSE /////////////////////////////////////////////////////////////
		// Sent the command, now look for response
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			Sleep(SIMULATED_SIO_SLEEP_TIME);
			return 0;
		}
		SetCommMask (g_hKerrServoCommPort, EV_RXCHAR | EV_BREAK);
	//		memset(m_ReadBuf,0,READ_BUF_SIZE);
		memset( m_KerrReplyBuf,0,KERR_MAX_REPLY_BYTES );

		Sleep(5); // This is REQUIRED for reliable Kerr communication!
		// Read ANSI characters
		dwStartTime = GetTickCount();
		if(!ReadFile(g_hKerrServoCommPort,		// where to read from (the open comm port)
				 &m_KerrReplyBuf,			// where to put the characters
				 (nBytesToRead),			// number of BYTEs to read, including Checksum
				 &dwBytesReceived,			// how many BYTEs were actually read
				 NULL))						// no overlapped I/O
		{
			// Has the comm port been closed?
			if(GetLastError() != ERROR_INVALID_HANDLE)
			{
				ROBOT_DISPLAY( TRUE, "Kerr Servo SendCmd:  Error reading response from KerrServo!\n" )
				g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
			}
			return 0;	// TODO: terminate thread on error???
		}

		#ifdef  DEBUG_KERR_READWRITE_TIME
			//ROBOT_LOG( TRUE,"Kerr Read Time: %d ms\n", (GetTickCount() - dwStartTime) )
			ROBOT_LOG( TRUE,"Kerr Read Time: %d ms Bytes Rcd: %d\n", (GetTickCount() - dwStartTime), dwBytesReceived )
		#endif


		if( 0 == dwBytesReceived )
		{
			// Error, no ACK received
			if( SUBSYSTEM_FAILED != g_KerrSubSystemStatus )
			{
				g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
				SpeakText( "Warning, shoulder motor controller has failed");
				ROBOT_DISPLAY( TRUE, "Kerr SendCmd:  No Response from Kerr Controller!" )
			}
			return 0;
		}

#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "   Kerr ReadFile Done: %4d ms\n", ProcessingTime )
#endif

		// Got a response
		#ifdef  DEBUG_KERR_SHOW_ACK_MESSAGES

			ROBOT_LOG( TRUE,"Kerr RECEIVED (%d Bytes): Status= %02X\n", dwBytesReceived, (BYTE)(m_KerrReplyBuf[0]) )

		#endif

		if( dwBytesReceived != (DWORD)nBytesToRead )
		{
			ROBOT_LOG( TRUE,"ERROR! Kerr Servo BytesToRead = %d, BytesReceived = %d\n", nBytesToRead, dwBytesReceived )
			g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
			return 0;
		}

		// Calculate Expected Checksum
		int  CalculatedCheckSum = GetResponseCheckSum( nResponseBytes );
		BYTE ReportedCheckSum = 0;

		if( 1 == nResponseBytes )
		{
			ReportedCheckSum = ((KerrReplyShort_T*)m_KerrReplyBuf)->Checksum;	// cast to short response type
		}
		else
		{
			ReportedCheckSum = ((KerrReplyLong_T*)m_KerrReplyBuf)->Checksum;	// cast to long response type
		}

		if( CalculatedCheckSum != ReportedCheckSum )
		{
			MsgString.Format("Kerr SendCmd: Bad Checksum in Response from Kerr! CalculatedCheckSum = (%04X), Packet CheckSum = (%04X)\n", 
				CalculatedCheckSum, ReportedCheckSum );
			g_KerrSubSystemStatus = SUBSYSTEM_FAILED;
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
//			ROBOT_ASSERT(0);
//			return 0;
		}
		else
		{
			g_KerrSubSystemStatus = SUBSYSTEM_CONNECTED; // Indicate that communication is working OK with the Kerr controller
		}

		// Handle startup case, where the servo IDs are initially zero
		if( 0 != m_KerrCmd.ID )
		{
			// Display and post status
			DisplayServoStatus( m_KerrCmd.ID, nResponseBytes );
		}
	}

	return 1;

}

///////////////////////////////////////////////////////////////////////////////
int CKerrControl::TenthDegreeToTicks( int  MotorNumber, int TenthDegrees )
{
	// Convert from "Tenth Degrees" (+/- 3600) to Servo motor encoder "ticks"

#define TENTHDEGREES360					3600.0	// Number of TenthDegrees in a circle
#define KERR_TICKS_PER_360_DEGREES		302500.0	// As measured on LOKI: 302,500
#define	KERR_TICKS_PER_TENTHDEGREE		(KERR_TICKS_PER_360_DEGREES / TENTHDEGREES360)	// 302,500ticks / 3600 tenth degrees
#define	KERR_TENTHDEGREE_PER_TICK		(TENTHDEGREES360 / KERR_TICKS_PER_360_DEGREES)	// 3600 tenth degrees / 302,500ticks

	if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
	{
		TenthDegrees += RIGHT_ARM_SHOULDER_TENTH_DEGREES_ZERO;	// distance from "true" vertical to "0" degrees
		TenthDegrees = TenthDegrees * -1;// Need to reverse +/- for Right Arm!
		//ROBOT_LOG( TRUE,"DEBUG Right TenthDegreeToTicks = %d\n", TenthDegrees)
	}
	else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
	{
		TenthDegrees += LEFT_ARM_SHOULDER_TENTH_DEGREES_ZERO;	// distance from "true" vertical to "0" degrees
		//TenthDegrees = TenthDegrees * -1;// Need to reverse +/- for Right Arm!
		//ROBOT_LOG( TRUE,"DEBUG Left TenthDegreeToTicks = %d\n", TenthDegrees)

	}
	else
	{
		ROBOT_ASSERT(0); // Logic Error
	}

	double fTicks = (double)TenthDegrees * KERR_TICKS_PER_TENTHDEGREE;


	return (int)fTicks;
}

///////////////////////////////////////////////////////////////////////////////
int CKerrControl::TicksToTenthDegree( int  ServoID, int Ticks )
{
	// Convert from Servo motor encoder "ticks" to "Tenth Degrees" (+/- 3600)

	double fTenthDegrees = (double)Ticks * KERR_TENTHDEGREE_PER_TICK;

	if( KERR_RIGHT_ARM_SHOULDER_SERVO_ID == ServoID )
	{
		fTenthDegrees = fTenthDegrees * -1;// Need to reverse +/- for Right Arm!
		fTenthDegrees -= RIGHT_ARM_SHOULDER_TENTH_DEGREES_ZERO;	// distance from "true" vertical to "0" degrees
		//ROBOT_LOG( TRUE,"DEBUG Right TicksToTenthDegree = %3.2f\n", fTenthDegrees)
	}
	else if( KERR_LEFT_ARM_SHOULDER_SERVO_ID == ServoID )
	{
		fTenthDegrees -= LEFT_ARM_SHOULDER_TENTH_DEGREES_ZERO;	// distance from "true" vertical to "0" degrees
		//fTenthDegrees = fTenthDegrees * -1;// Need to reverse +/- for Right Arm!
		//ROBOT_LOG( TRUE,"DEBUG Left TicksToTenthDegree = %3.2f\n", fTenthDegrees)
	}
	else
	{
		ROBOT_ASSERT(0); // Logic Error
	}


	return (int)fTenthDegrees;

}

int  CKerrControl::ArmNumberToMotorNumber( int  ArmNumber )
{
	if( RIGHT_ARM == ArmNumber )
	{
		return KERR_ARM_MOTOR_ID_RIGHT;
	}
	else if( LEFT_ARM == ArmNumber )
	{
		return KERR_ARM_MOTOR_ID_LEFT;
	}
	else
	{
		ROBOT_ASSERT(0);
		return KERR_ARM_MOTOR_ID_RIGHT;	// Return something reasonable
	}

}

int  CKerrControl::ServoIDToMotorNumber( int  ServoID )
{
	if( KERR_RIGHT_ARM_SHOULDER_SERVO_ID == ServoID )
	{
		return KERR_ARM_MOTOR_ID_RIGHT;
	}
	else if( KERR_LEFT_ARM_SHOULDER_SERVO_ID == ServoID )
	{
		return KERR_ARM_MOTOR_ID_LEFT;
	}
	else
	{
		ROBOT_ASSERT(0);
		return KERR_ARM_MOTOR_ID_RIGHT;	// Return something reasonable
	}

}

int  CKerrControl::MotorNumberToServoID( int  MotorNumber )
{
	// Convert Kerr Servo number (1 or 2) to the correct position in the Bulk Array
	if( KERR_ARM_MOTOR_ID_RIGHT == MotorNumber )
	{
		return KERR_RIGHT_ARM_SHOULDER_SERVO_ID;
	}
	else if( KERR_ARM_MOTOR_ID_LEFT == MotorNumber )
	{
		return KERR_LEFT_ARM_SHOULDER_SERVO_ID;
	}
	else
	{
		ROBOT_ASSERT(0);
		return KERR_RIGHT_ARM_SHOULDER_SERVO_ID;	// Don't trash memory by returning a bad value
	}

}


///////////////////////////////////////////////////////////////////////////////
void CKerrControl::HandleCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 )
{
	IGNORE_UNUSED_PARAM (Option1);
	IGNORE_UNUSED_PARAM (Option2);
	//TAL_Event("Sending Cmd");
	//-TAL_SCOPED_TASK_NAMED("Handling Kerr Command");


#ifdef DEBUG_KERR_TIMING
		KerrCmdStartTime = GetTickCount();	// for measuring how long this pass is taking
		unsigned long RunTime = KerrCmdStartTime - gStartTime;
		RunTime %= (24 * 60 * 60 * 1000);
		RunTime %= (60 * 60 * 1000);
		unsigned int minutes = RunTime / (60 * 1000);
		RunTime %= (60 * 1000);
		unsigned int seconds = RunTime / (1000);
		RunTime %= (1000);
		unsigned int miliseconds = RunTime;
		ROBOT_LOG( TRUE,"--------------------------------------------------\n")
		ROBOT_LOG( TRUE,"Kerr Command Start: %d min, %d sec, %d ms\n", minutes, seconds, miliseconds )
#endif

	if( !m_PowerEnabled && (HW_SET_POWER_MODE != Request) )
	{
		// Power disabled, ignore all commands but power enable
		return;
	}

	switch( Request )
	{
		// Status Requests
		case HW_GET_STATUS:
		case HW_GET_SERVO_STATUS:
		{
			if( Param1 < KERR_RIGHT_ARM_SHOULDER_SERVO_ID )
			{
				// Ignore requests for non-Kerr servos
				//ROBOT_LOG( TRUE,"KERR: HW_GET_SERVO_STATUS Ignoring command to servo # %d\n", Param1)
				return;	// Nothing to do - Not for Kerr Controller
			}
			int  MotorNumber = ServoIDToMotorNumber( Param1 );
			GetServoStatus( MotorNumber );
			break;
		}
		case HW_GET_SMART_SERVO_STATUS:
		{
			// This message requests status of all arm servos and shoulder Servos
			// Param1  = Not Used
			// Param2  = Not used

			DWORD dwStartTime = GetTickCount();
			// DEBUG TODO-LOKI Status Speed Test!
//			for(int t=0; t=10; t++)
//			{
				// Get the status for all right and left arm Servos
				if( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable && 
					(abs( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees - 
					  g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES ))
				GetServoStatus( KERR_ARM_MOTOR_ID_RIGHT );

				if( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable && 
					(abs( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees - 
					  g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees ) > ARM_JOINT_DELTA_MAX_TENTHDEGREES ))
				GetServoStatus( KERR_ARM_MOTOR_ID_LEFT );

				/////////////////////////////////////
				// DEBUG!
//			}
//			ROBOT_LOG( TRUE,"DEBUG TEST: KERR Total Status Time (2 Servos) = %d ms\n", (GetTickCount() - dwStartTime) )


			// OK, now tell the GUI and arm control module that there is a new status update.
			// (happens twice, once for Dyna Servos and once for Kerr servos)
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_SERVO_STATUS_READY, 0, 0 );
			PostThreadMessage( g_dwControlThreadId, WM_ROBOT_SERVO_STATUS_READY, 0, 0);

			/***
			if( SERVO_STATUS_REQUEST_FREQ_FAST == g_nServoStatusRequestsPerSecond )
			{
				if( m_pArmControlRight->CheckArmPosition(FALSE)  &&	//	TRUE = Verbose
					m_pArmControlLeft->CheckArmPosition(FALSE) )	
				{
					// Last movement completed.
					// Request less frequent status updates since arm is done moving
					g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_NORMAL;
				}
			}
			***/
			break;
		}

		case HW_INITIALIZE:
		{
			Init();
			break;
		}

		case HW_SET_SERVO_POS_TICKS:	// We treat the arm like one big honking servo
		{	
			ROBOT_LOG( TRUE,"KerrControl: HW_SET_SERVO_POS_TICKS Not Implemented\n")
			break;
		}

		case HW_SET_POWER_MODE:
		{
			if( (POWER_ON == Param1) )
			{
				// Turn Power On - For Kerr Servos, that means enable Torque
				// EnableTorque TRUE will automatically also do a "home" move
				m_PowerEnabled = TRUE;
				EnableServoTorque( KERR_ARM_MOTOR_ID_RIGHT, TRUE );
				EnableServoTorque( KERR_ARM_MOTOR_ID_LEFT, TRUE );
			}
			else
			{
				// Option1 indicates shut down step
				if(0 == Option1)
				{
					// Option1 == 0 means Return to "Rest" position
					m_PowerEnabled = FALSE; // prevent further commands other than power on
					GotoSleepPosition();
				}
				else 
				{
					// Option1 == 1 means turn torque off
					EnableServoTorque( KERR_ARM_MOTOR_ID_RIGHT, FALSE );
					EnableServoTorque( KERR_ARM_MOTOR_ID_LEFT, FALSE );
				}
			}
			break;
		}

		case HW_SET_SERVO_TORQUE_ENABLE:
		{
			// Update torque enable of all shoulder servos with "Update" flagged in the global g_BulkServoCmd buffer

			if( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update )
			{
				int  MotorNumber = ServoIDToMotorNumber( KERR_RIGHT_ARM_SHOULDER_SERVO_ID );
				EnableServoTorque( KERR_ARM_MOTOR_ID_RIGHT, g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable  ); // ID, Enable
				// Indicate to the system that the servo enable command has been handled
				g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = FALSE;
			}

			if( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update )
			{
				int  MotorNumber = ServoIDToMotorNumber( KERR_LEFT_ARM_SHOULDER_SERVO_ID );
				EnableServoTorque( KERR_ARM_MOTOR_ID_LEFT, g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable  ); // ID, Enable
				// Indicate to the system that the servo enable command has been handled
				g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = FALSE;
			}

			break;
		}

		case HW_SET_BULK_ARM_POSITION:
		{
			// Update all arm Servos flagged in the global g_ServoCmd buffer ( for both Dyna and Kerr servos )
			// Param1  = LEFT_ARM or RIGHT_ARM
			// Param2  = Set Speed of each Servo (True/False)
			//ROBOT_DISPLAY( TRUE, "=========> KERR: GOT HW_SET_BULK_ARM_POSITION, Sleeping for Dyna  <=========\n" )
			//Sleep(10);
			//ROBOT_DISPLAY( TRUE, "=========> KERR: Processing HW_SET_BULK_ARM_POSITION  <=========\n" )


			if( RIGHT_ARM == Param1 )
			{
				if( g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update)
				{
					int  MotorNumber = ArmNumberToMotorNumber( Param1 );

					int NewPositionTenthDegrees = g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees; 
					//CheckServoLimit(KERR_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID, PositionTicks);

					// Make sure command is within motion limits
					if( NewPositionTenthDegrees < -3500 )	// 350 degrees max
						NewPositionTenthDegrees = -3500;
					if( NewPositionTenthDegrees > 3500)
						NewPositionTenthDegrees = 3500;


					if( Param2 )
					{
						// Param2 true = set the speed of Servo too
						int  NewSpeed = g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Speed;
						SetArmPositionAndSpeed( MotorNumber, NewPositionTenthDegrees, NewSpeed );
					}
					else
					{
						SetArmPosition( MotorNumber, NewPositionTenthDegrees );
					}

					// Indicate to the system that the Servo positions have been handled
					g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = FALSE;
				}
			}
			else if( LEFT_ARM == Param1 )
			{
				if( g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update)
				{
					int  MotorNumber = ArmNumberToMotorNumber( Param1 );

					int NewPositionTenthDegrees = g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees; 
					//CheckServoLimit(KERR_LEFT_ARM_SHOULDER_SERVO_ID, PositionTicks);

					// Make sure command is within motion limits
					if( NewPositionTenthDegrees < -3500 )	// 350 degrees max
						NewPositionTenthDegrees = -3500;
					if( NewPositionTenthDegrees > 3500)
						NewPositionTenthDegrees = 3500;


					if( Param2 )
					{
						// Param2 true = set the speed of Servo too
						int  NewSpeed = g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Speed;
						SetArmPositionAndSpeed( MotorNumber, NewPositionTenthDegrees, NewSpeed );
					}
					else
					{
						SetArmPosition( MotorNumber, NewPositionTenthDegrees );
					}

					// Indicate to the system that the Servo positions have been handled
					g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = FALSE;
				}
			}
			else
			{
				ROBOT_ASSERT(0);
			}

			// Request more frequent status updates while the arm is moving
			//g_nServoStatusRequestsPerSecond = SERVO_STATUS_REQUEST_FREQ_FAST;


			break;
		}
/*		case HW_SET_ARM_JOINT_POSITION:
		{
			// Param1 = Which Arm and Joint to move
			// Param2 = New Position
			if( Param1 < KERR_RIGHT_ARM_SHOULDER_SERVO_ID )
			{
				Return;	// Nothing to do - Not for Kerr Controller
			}

			int  ServoID = ServoIDToMotorNumber( Param1 );

			int NewPositionTenthDegrees = (int)Param2;
			// Make sure command is within motion limits
			if( NewPositionTenthDegrees < -3500 )	// 350 degrees max
				NewPositionTenthDegrees = -3500;
			if( NewPositionTenthDegrees > 3500)
				NewPositionTenthDegrees = 3500;

			SetArmPosition( ServoID, NewPositionTenthDegrees );

			break;
		}	// Case HW_SET_ARM_JOINT_POSITION
*/

		case HW_SET_SERVO_TORQUE_LIMIT:
		{
			// No Torque limit on Kerr, only used for Dynamixel.
			// But, all Servo commands go to both controllers.  Just silently ignore it.
			break;
		}

		/*
		Silently ignore other commands (since Kerr gets same commands as DYNA, and some don't apply)
		default:
		{
			CString StrText;
			StrText.Format( "ERROR! KerrServoCommThread: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, (StrText) )
		}
		*/

	}
#ifdef DEBUG_KERR_TIMING
	unsigned long ProcessingTime = GetTickCount() - KerrCmdStartTime;
	ROBOT_LOG( TRUE, "Kerr Command End: %4d ms\n", ProcessingTime )
	ROBOT_LOG( TRUE,"--------------------------------------------------\n")
#endif

}

#endif // ROBOT_SERVER - This module used for Robot Server only

