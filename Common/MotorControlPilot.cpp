// MotorControlPilot.cpp
// Interface for the PMD Pilot MC3410 Motor Control use for the ER1 Robot
// NOTE! See Pilot Users Guide.pdf, section 10.3 "serial communications"


// This class is used by the thread loop that reads commands from the queue
// in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include "thread.h"
//#include "module.h"
#include "Globals.h"
#include "MotorControl.h"
#include "../Common/HardwareCmds.h" // for TICS_PER_INCH

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#if ( ROBOT_SERVER == 1 )  // Nothing in this file used for client!

/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_PILOT_COMMAND_DUMP   // Dump all ER1 Pilot commands
//#define DEBUG_PILOT_READWRITE_TIME  // Log Read/Write SIO latency
//#define DEBUG_PILOT_SHOW_ACK_MESSAGES

/////////////////////////////////////////////////////

#define LEFT_MOTOR			1
#define RIGHT_MOTOR			0


// CString strStatus;

/////////////////////////////////////////////////////////////////////////////


/***********************************************************
typedef struct PWDCmd_struct {
	byte Address;
	byte Checksum;
	byte Axis;	// initialized by constructor, always 0
	byte Code;
	byte Data[MAX_DATA_BYTES];
	byte Size;	// meta data, should be not be send.
} PWDCmd;
***********************************************************/



///////////////////////////////////////////////////////////////////////////////
// PMD Pilot Motor Control Class
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////

CPilotControl::CPilotControl()
{
	m_PilotCmd.Address = 0;	// Which Motor
	m_PilotCmd.Axis = 0;		// Always zero
	m_PilotCmd.Code = 0;		// Insert command code here!
	memset(m_PilotCmd.Data,0,MAX_DATA_BYTES);

	m_PilotReply.CommandStatus = 0;
	m_PilotReply.Checksum = 0;
	memset(m_PilotReply.Data,0,MAX_REPLY_BYTES);

//	memset(m_ReadBuf,0,READ_BUF_SIZE);
	m_Speed = 0;
	m_Turn = 0;
	m_LeftWheelStopped = TRUE;
	m_RightWheelStopped = TRUE;
}

void CPilotControl::Init()
{
	m_PilotCmd.Address = 0;		// Which Motor
	m_PilotCmd.Axis = 0;		// Always zero
	m_PilotCmd.Code = 0;		// Insert command code here!
	memset(m_PilotCmd.Data,0,MAX_DATA_BYTES);
//	memset(m_ReadBuf,0,READ_BUF_SIZE);

	m_PilotReply.CommandStatus = 0;
	m_PilotReply.Checksum = 0;
	memset(m_PilotReply.Data,0,MAX_REPLY_BYTES);

	m_Speed = 0;
	m_Turn = 0;
	m_LeftWheelStopped = TRUE;
	m_RightWheelStopped = TRUE;

	DWORD dwStartTime = GetTickCount();

	ROBOT_LOG( TRUE,  "\n\n======================== PILOT INITIALIZATION  ============================= \n" )
	ROBOT_LOG( TRUE,  "\n\nPILOT INITIALIZING MOTOR 1\n" )
	InitWheelMotor( 1 );		// Right Weel?
	ROBOT_LOG( TRUE,  "\n\nPILOT INITIALIZING MOTOR 0\n" )
	InitWheelMotor( 0 );		// Left Weel?
	ROBOT_LOG( TRUE,  "\n\n=================== PILOT INITIALIZATION COMPLETE ========================== \n\n\n" )

	ROBOT_LOG( TRUE,  "Pilot Total Init Time: %d ms\n", (GetTickCount() - dwStartTime) )

	g_MotorSubSystemStatus = SUBSYSTEM_CONNECTED;

	// DEBUG!!
	GetStatus();

}

void CPilotControl::DoCheckSum(int  nParamBytes)
{
	// Calculate Checksum
	// Usage: 	DoCheckSum(nParamBytes);

	int  CheckSum = m_PilotCmd.Address;
	CheckSum += m_PilotCmd.Axis;
	CheckSum += m_PilotCmd.Code;

	for ( int  i=0 ; i<nParamBytes ; i++ )
	{
		CheckSum += m_PilotCmd.Data[i];
	}
	// See Pilot Users Guide.pdf, page 63, 10.3.5 (Serial Checksum)
	CheckSum = (~CheckSum) + 1;
	CheckSum = CheckSum & 0x00FF;	// save lower 8 bits only
	m_PilotCmd.Checksum = (BYTE)CheckSum;	// Notice this is lower 8 bits only
}

DWORD CPilotControl::SpeedToVelocity(int nSpeed)
{
	// Scale speed sent by main control to Pilot Velocity 
	// dwSpeed = +/- 127
	// ER1 code uses 0001 E1B4 for full speed motor 1,
	// and           FFFE 49DF for full speed motor 0

	// Step size = 0x0001 E1B4 / 127 = 970 (0x3CA)
	DWORD dwVelocity;

	// Special case "Stop".  For some reason, the ER1 wants 0x0007 for stop instead of 0x0000.
	// (maybe it's just round off?)
	if( 0 == nSpeed )
	{
		dwVelocity = 0x07; // TODO is this necessary?
	}
	else
	{
		dwVelocity = (DWORD)(int)(nSpeed * PILOT_SPEED_TO_VELOCITY_FACTOR);
	}

/*	if( dwVelocity > 0x1E1B4 )
	{
		dwVelocity = 0x1E1B4;	// TODO is this limit necessary?
	}
*/
	return dwVelocity;
	
}

///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////

void CPilotControl::DoMotorUpdate()
{
	// Make pending commands active
	m_PilotCmd.Address = 1;		// Which Motor
	m_PilotCmd.Axis = 0;		// Always zero
	m_PilotCmd.Code = PilotUpdate;
	SendWriteCmd(0);

	m_PilotCmd.Address = 0;		// Which Motor
	SendWriteCmd(0);
}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::InitWheelMotor(int  MotorNumber)
{

	CString MsgString;

	m_PilotCmd.Address = (BYTE)MotorNumber;	// Motor
	m_PilotCmd.Axis = 0;

	// Initializaton reverse engineered from ER1
	// Get Version
	m_PilotCmd.Code = GetPilotVersion;
	if (SendReadCmd(4) )
	{
		MsgString.Format( "Pilot Version: Family/Motor %02X, Axes/Attribs %02X, Custom Code %02X, FW Vers: %02X\n",
			m_PilotReply.Data[0], m_PilotReply.Data[1], m_PilotReply.Data[2], m_PilotReply.Data[3] );
		ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
	}	
	// Reset
	ROBOT_DISPLAY( TRUE, "Pilot: Ignore Error after Reset -->" )
	m_PilotCmd.Code = PilotReset;
	SendWriteCmd(0);
	ROBOT_DISPLAY( TRUE, "<-- Pilot Reset Complete" )

	// Note: GetHostIOError happens automatically in SendWriteCmd

	// SetLimitSwitchMode - disable limit switches (continuous wheel rotation)
	ROBOT_LOG( TRUE,  "Pilot: SetLimitSwitchMode\n" )
	m_PilotCmd.Code = SetLimitSwitchMode;
	m_PilotCmd.Data[0] = 0;
	m_PilotCmd.Data[1] = 0;
	SendWriteCmd(2);

	// SetMotorCommand - Turn motor off
	ROBOT_LOG( TRUE,  "Pilot: SetMotorCommand = 0\n" )
	m_PilotCmd.Code = SetMotorCommand;
	m_PilotCmd.Data[0] = 0;
	m_PilotCmd.Data[1] = 0;
	SendWriteCmd(2);

	// GetSignalStatus - no clue what this does! (expect: 00 9C 04 60 ?)
	ROBOT_LOG( TRUE,  "Pilot: GetSignalStatus\n" )
	m_PilotCmd.Code = GetSignalStatus;
	if( SendReadCmd(2) )
	{
		ROBOT_LOG( TRUE,  "Pilot GetSignalStatus returned:  %02X, %02X\n",	m_PilotReply.Data[0], m_PilotReply.Data[1] )
	}

	// Here the ER1 does Wheel 0 first then wheel 1... is this an issue?

	ROBOT_LOG( TRUE,  "Pilot: GetCommandedVelocity\n" )
	m_PilotCmd.Code = GetCommandedVelocity;
	SendReadCmd(4);

	ROBOT_LOG( TRUE,  "Pilot: SetMotorCommand To Speed 7h\n" )
	m_PilotCmd.Code = SetMotorCommand;
	m_PilotCmd.Data[0] = 00;
	m_PilotCmd.Data[1] = 0x07;
	SendWriteCmd(2);

	ROBOT_LOG( TRUE,  "Pilot: Update\n" )
	m_PilotCmd.Code = PilotUpdate;
	SendWriteCmd(0);



}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SendWheelStopCommand(int  Wheel)
{
	ROBOT_LOG( TRUE,  "PILOT - SET STOP Wheel %d\n", Wheel)

	m_PilotCmd.Address = (BYTE)Wheel;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;

	m_PilotCmd.Code = SetMotorCommand;
	m_PilotCmd.Data[0] = 0x00;	
	m_PilotCmd.Data[1] = 0x07;	// Turn Motor OFF!
	SendWriteCmd(2);

	m_PilotCmd.Code = SetProfileMode;
	m_PilotCmd.Data[0] = 0x00;
	m_PilotCmd.Data[1] = 0x01;	// 0x0001 = Velocity Contouring mode
	SendWriteCmd(2);

	m_PilotCmd.Code = SetAcceleration;
	m_PilotCmd.Data[0] = 0x00;
	m_PilotCmd.Data[1] = 0x00;
	m_PilotCmd.Data[2] = 0x00;
	m_PilotCmd.Data[3] = PILOT_ACCELERATION_PROFILE; // 0x0000 00B0 = default acceleration profile for ER
	SendWriteCmd(4);

	m_PilotCmd.Code = SetDeceleration;
	m_PilotCmd.Data[0] = 0x00;
	m_PilotCmd.Data[1] = 0x00;
	m_PilotCmd.Data[2] = 0x00;
	m_PilotCmd.Data[3] = PILOT_DECELERATION_PROFILE; // 0x0000 00B0 = default deceleration profile for ER
	SendWriteCmd(4);

	m_PilotCmd.Code = SetVelocity;	
	m_PilotCmd.Data[0] = 0x00;
	m_PilotCmd.Data[1] = 0x00;
	m_PilotCmd.Data[2] = 0x00;
	m_PilotCmd.Data[3] = 0x00;
	SendWriteCmd(4);

}


///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SendWheelCommand(int  Wheel, DWORD dwMotorSpeed)
{
	ROBOT_LOG( TRUE,  "PILOT - SET SPEED Wheel %d = %08lX\n", Wheel, dwMotorSpeed)

	m_PilotCmd.Address = (BYTE)Wheel;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;

	m_PilotCmd.Code = SetMotorCommand;
	m_PilotCmd.Data[0] = 0x37;	
	m_PilotCmd.Data[1] = 0x14;	// 0x3714 = Go 43% (I think this refers to %servo motor motion)
	SendWriteCmd(2);			// OR, Should this be 0x4C C0, as shwn in er1_rcm.pdf Hack Doc?

	m_PilotCmd.Code = SetProfileMode;
	m_PilotCmd.Data[0] = 0;
	m_PilotCmd.Data[1] = 1;	// 0x0001 = Velocity Contouring mode: Needed for Continuous Wheel Motion!
	SendWriteCmd(2);					// See Pilot User's guide, page 18

	m_PilotCmd.Code = SetAcceleration;
	m_PilotCmd.Data[0] = 0;
	m_PilotCmd.Data[1] = 0;
	m_PilotCmd.Data[2] = 0;
	m_PilotCmd.Data[3] = PILOT_ACCELERATION_PROFILE;	// 0x0000 00B0 = default acceleration profile for ER1
	SendWriteCmd(4);

	m_PilotCmd.Code = SetDeceleration;
	m_PilotCmd.Data[0] = 0;
	m_PilotCmd.Data[1] = 0;
	m_PilotCmd.Data[2] = 0;
	m_PilotCmd.Data[3] = PILOT_DECELERATION_PROFILE;	// 0x0000 00B0 = default deceleration profile for ER1
	SendWriteCmd(4);

// TODO - SET JERK 03 29 0A BB - ? For S curve only?


	// ER1 code uses 0001 E1B4 for full speed motor 1,
	// and           FFFE 49DF for full speed motor 0

	m_PilotCmd.Code = SetVelocity;	
	m_PilotCmd.Data[0] = HIBYTE(HIWORD(dwMotorSpeed));
	m_PilotCmd.Data[1] = LOBYTE(HIWORD(dwMotorSpeed));
	m_PilotCmd.Data[2] = HIBYTE(LOWORD(dwMotorSpeed));
	m_PilotCmd.Data[3] = LOBYTE(LOWORD(dwMotorSpeed));
	SendWriteCmd(4);

}


///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SendQuickWheelCommand(int  Wheel, DWORD dwMotorSpeed)
{
	// Useful for speed changes while moving.  Assume full speed command already happened.

#if DEBUG_PILOT == 1
	ROBOT_LOG( TRUE,  "PILOT - QUICK SET SPEED Wheel %d = %08lX\n", Wheel, dwMotorSpeed)
#endif
	m_PilotCmd.Address = (BYTE)Wheel;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;


	// ER1 code uses 0001 E1B4 for full speed motor 1,
	// and           FFFE 49DF for full speed motor 0

	m_PilotCmd.Code = SetVelocity;	
	m_PilotCmd.Data[0] = HIBYTE(HIWORD(dwMotorSpeed));
	m_PilotCmd.Data[1] = LOBYTE(HIWORD(dwMotorSpeed));
	m_PilotCmd.Data[2] = HIBYTE(LOWORD(dwMotorSpeed));
	m_PilotCmd.Data[3] = LOBYTE(LOWORD(dwMotorSpeed));
	SendWriteCmd(4);

}


///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SetWheelSpeedAndTurn()
{

	// "Mixes" Speed and Turn to set speed of the motors
	// since the wheels are oposite each other (one mounted backward)
	// We drive one "forward" and one "reverse" to make the robot go forward!
	// m_Speed and m_Turn are set prior to calling this function

	int LeftMotorSpeed = m_Speed + m_Turn;
	int RightMotorSpeed = m_Speed - m_Turn;
	

	if( (m_Speed == 0) && (m_Turn == 0) )
	{
		// Stop request
		SetWheelAllStop();
		m_LeftWheelStopped = TRUE;
		m_RightWheelStopped = TRUE;
	}
	else
	{	
		DWORD LeftMotorVelocity = SpeedToVelocity(LeftMotorSpeed);
		DWORD RightMotorVelocity = SpeedToVelocity(RightMotorSpeed);
		LeftMotorVelocity = ~LeftMotorVelocity +1;	// invert left motor

		DWORD dwStartTime = GetTickCount();

		if( m_LeftWheelStopped )
		{
			// Wheel was stopped.  Use full command to get it going
			SendWheelCommand(LEFT_MOTOR, LeftMotorVelocity);
		}
		else
		{
			// Wheel currently moving.  Send quick command to change speed.
			SendQuickWheelCommand(LEFT_MOTOR, LeftMotorVelocity);
		}

		if( m_RightWheelStopped )
		{
			SendWheelCommand(RIGHT_MOTOR, RightMotorVelocity);
		}
		else
		{
			SendQuickWheelCommand(RIGHT_MOTOR, RightMotorVelocity);
		}

		DoMotorUpdate();	// Tell Motor Controller to execute the commands

		#ifdef  DEBUG_PILOT_READWRITE_TIME
			ROBOT_LOG( TRUE,  "Pilot Total SetWheelSpeedAndTurn Time: %d ms\n", (GetTickCount() - dwStartTime) )
		#endif

		if( 0 != LeftMotorSpeed )
		{
			m_LeftWheelStopped = FALSE;
		}
		if( 0 != RightMotorSpeed )
		{
			m_RightWheelStopped = FALSE;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SetWheelAllStop()
{

	// Turns off wheels.  If currently moving, motor will coast to stop.

	CString MsgString;

	for( int MotorNumber = 0; MotorNumber < 2; MotorNumber++ )
	{
		m_PilotCmd.Address = (BYTE)MotorNumber;	// Motor
		m_PilotCmd.Axis = 0;

		m_PilotCmd.Code = SetVelocity;	
		m_PilotCmd.Data[0] = 0;
		m_PilotCmd.Data[1] = 0;
		m_PilotCmd.Data[2] = 0;
		m_PilotCmd.Data[3] = 0;
		SendWriteCmd(4);

	//	ROBOT_DISPLAY( TRUE, "Pilot: Stop - SetMotorCommand To Speed 7h - Should this be Zero?" )
		m_PilotCmd.Code = SetMotorCommand;
		m_PilotCmd.Data[0] = 00;
		m_PilotCmd.Data[1] = 0x07;
		SendWriteCmd(2);
	}

	DoMotorUpdate();	// Tell Motor Controller to execute the commands

}


///////////////////////////////////////////////////////////////////////////////
void CPilotControl::DisplayIOError()
{
	CString MsgString;
	CString strText;
	
	switch(m_PilotReply.CommandStatus)
	{
		case 0x00 : strText = ("No Error"); break;
		case 0x01 : strText = ("Processor Reset"); break;
		case 0x02 : strText = ("Invalid Instruction"); break;
		case 0x03 : strText = ("Invalid Axis"); break;
		case 0x04 : strText = ("Invalid Parameter"); break;
		case 0x05 : strText = ("Trace Running"); break;
		case 0x06 : strText = ("Reserved Error 06"); break;
		case 0x07 : strText = ("Block Out of Bounds"); break;
		case 0x08 : strText = ("Trace Buffer Zero"); break;
		case 0x09 : strText = ("Bad Serial Checksum"); break;
		case 0x0A : strText = ("Reserved Error 0A"); break;
		case 0x0B : strText = ("Invalid Negative Velocity"); break;
		case 0x0C : strText = ("Invalid Parameter Change"); break;
		case 0x0D : strText = ("Invalid Move After Limit"); break;
		case 0x0E : strText = ("Invalid Move Into Limit"); break;
		default : strText = ("Unknown Error"); break;
	}
	

	MsgString.Format( "========> Pilot Error %02X: %s!", 
		m_PilotReply.CommandStatus, strText );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SendWriteCmd(int  nParamBytes )
{
	// send a write command with no response data expected

	CString MsgString;

	SendCmd( nParamBytes, 0 );	// Send the Command via COM port

	// Check for errors
	if( 0 != m_PilotReply.CommandStatus )
	{
		// Error already displayed.  Need to reset the error

		// GetHostIOError - This clears the prior error.
		// Response data should be same as the error previously reported
		ROBOT_DISPLAY( TRUE, "Clearing Reset Error Bit" )
		m_PilotCmd.Code = GetHostIOError;
		SendCmd(0,2);

		if( 0 != m_PilotReply.CommandStatus )
		{
			MsgString.Format( "Error %02X while trying to Reset %02X", 
				m_PilotReply.CommandStatus, m_PilotReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
		else
		{
			MsgString.Format( "Error %02X has been Reset", m_PilotReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
	}

}

///////////////////////////////////////////////////////////////////////////////
BOOL CPilotControl::SendReadCmd( int  nResponseBytes )
{

	// send a read command with no parameters

	CString MsgString;
	BOOL SendOK = TRUE;

	SendCmd( 0, nResponseBytes );	// Send the Command via COM port

	// Check for errors
	if( 0 != m_PilotReply.CommandStatus )
	{
		// Error already displayed.  Need to reset the error
		SendOK = FALSE;

		// GetHostIOError - This clears the prior error.
		// Response data should be same as the error previously reported
		ROBOT_DISPLAY( TRUE, "Clearing Reset Error Bit" )
		m_PilotCmd.Code = GetHostIOError;
		SendCmd(0,2);

		if( 0 != m_PilotReply.CommandStatus )
		{
			MsgString.Format( "Error %02X while trying to Reset %02X", 
				m_PilotReply.CommandStatus, m_PilotReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
		else
		{
			MsgString.Format( "Error %02X has been Reset", m_PilotReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
	}
	return SendOK;

}
///////////////////////////////////////////////////////////////////////////////
void CPilotControl::GetStatus()
{
	
	// Get current Status from the Pilot controller
	CString MsgString;
#define DEBUG_PILOT 0

#if DEBUG_PILOT == 1
	ROBOT_LOG( TRUE,  "------------------- STATUS UPDATE --------------\n" )
#endif
	////////////////////////////////////////////////////////
	// Speed / Tach
	////////////////////////////////////////////////////////
	m_PilotCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;
	m_PilotCmd.Code = GetCommandedVelocity;		// see how fast the motor is ACTUALLY going
	if( SendReadCmd(4) )
	{
#if DEBUG_PILOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_PilotReply.Data[0], m_PilotReply.Data[1], m_PilotReply.Data[2], m_PilotReply.Data[3] )
#endif
	}
	long WheelSpeedL = htonl( *(long*)( &m_PilotReply.Data[0] ) );
	WheelSpeedL = (~WheelSpeedL) + 1;	// Wheel pointing backward, negate the value

	m_PilotCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;
	m_PilotCmd.Code = GetCommandedVelocity;	
	if( SendReadCmd(4) )
	{
#if DEBUG_PILOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_PilotReply.Data[0], m_PilotReply.Data[1], m_PilotReply.Data[2], m_PilotReply.Data[3] )
#endif
	}
	long WheelSpeedR = htonl( *(long*)( &m_PilotReply.Data[0] ) );

	if( (WheelSpeedR != 0) || (WheelSpeedR != 0) )
	{
#if DEBUG_PILOT == 1
		MsgString.Format( "Pilot Speed: LEFT_MOTOR: %ld, RIGHT_MOTOR: %ld",
			WheelSpeedL/SPEED_DIVIDER, WheelSpeedR/SPEED_DIVIDER );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

#endif
		// Update the GUI with the new Speed traveled

		// Average the two wheels
		int WheelSpeedAverage = (WheelSpeedL + WheelSpeedR) / (2*SPEED_DIVIDER);
		SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_MOTOR_SPEED, (LPARAM)WheelSpeedAverage );
	}


	////////////////////////////////////////////////////////
	// Distance
	////////////////////////////////////////////////////////
	m_PilotCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;
	m_PilotCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_PILOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_PilotReply.Data[0], m_PilotReply.Data[1], m_PilotReply.Data[2], m_PilotReply.Data[3] )
#endif
	}
	long WheelDistanceL = htonl( *(long*)( &m_PilotReply.Data[0] ) );
	WheelDistanceL = (~WheelDistanceL) + 1;	// Wheel opposite, negate the value

	m_PilotCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_PilotCmd.Axis = 0;
	m_PilotCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_PILOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_PilotReply.Data[0], m_PilotReply.Data[1], m_PilotReply.Data[2], m_PilotReply.Data[3] )
#endif
	}
	long WheelDistanceR = htonl( *(long*)( &m_PilotReply.Data[0] ) );

	if( (WheelDistanceR != 0) || (WheelDistanceL != 0) )
	{
//		ROBOT_LOG( TRUE,  "MotorControl: Pilot Distance Moved (Inches): LEFT_MOTOR: %0.2f  RIGHT_MOTOR:%0.2f",
//			WheelDistanceL/TICKS_PER_TENTH_INCH, WheelDistanceR/TICKS_PER_TENTH_INCH )

		// Each wheel is sent, so turns can be computed
		// We assume that WPARAM and LPARAM are both 32 bits (see ROBOT_ASSERT in SetupView.cpp)

		// Update the Robot Control thread with the new distance traveled (in "ticks")
		// this in turn will update the GUI with Odometer and current location
		PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_ER1_ODOMETER_READY),
			WheelDistanceL, WheelDistanceR );

	}
#if DEBUG_PILOT == 1
	ROBOT_LOG( TRUE,  "------------------------------------------------\n" )
#endif
}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::SendCmd(int  nParamBytes, int  nResponseBytes )
{
	
	CString MsgString;
	DWORD dwBytesWritten=0;
//	int  Status = 0;
//	int  ReadCheckSum = 0;
	unsigned char DbgBuf[16];
	memset(DbgBuf,0, 16);
	int  q;
	DWORD dwBytesReceived = 0;
	DWORD dwStartTime = 0;

	int  BytesToWrite = nParamBytes + 4;	// Commands are always 4 bytes long plus nParamBytes
	int  nBytesToRead = nResponseBytes + 2;	// Add 2 for the CommandStatus and Checksum

	// Calculate CheckSum and insert it into the command buffer
	DoCheckSum( nParamBytes );

	// Debug:
//	MsgString.Format( "Pilot SendWriteCmd:  Sending cmd %02X to Pilot controller.\n", m_PilotCmd.Code );
//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

/*	Format of commands:
	byte Address;
	byte Checksum;
	byte Axis;	// initialized by constructor, always 0
	byte Code;
	byte Data[6];
*/

	
#ifdef DEBUG_PILOT_COMMAND_DUMP

	memcpy( DbgBuf, &m_PilotCmd, 10);
	ROBOT_LOG( TRUE,  "PILOT CMD DUMP (%d Data Bytes): %02X   %02X   %02X    %02X     ",
		nParamBytes, DbgBuf[0],DbgBuf[1],DbgBuf[2],DbgBuf[3] )

	for( q=0; q < nParamBytes; q++ )
	{
		ROBOT_LOG( TRUE,  "%02X ",	DbgBuf[(q+4)] );
	}
	ROBOT_LOG( TRUE,  "\n" );
	ROBOT_LOG( TRUE,  "                               Addr Cksm Axis  Cmd    Data\n" );

#endif

	if( !g_MotorControlDebug )	// skip COM Read/Write if no serial hooked up
	{
		dwStartTime = GetTickCount();
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainMotorThread);
		}
		else
		{
			if(!WriteFile(g_hMotorCommPort,	// where to write to (the open comm port)
					&m_PilotCmd,				// what to write
					BytesToWrite,				// number of bytes to be written to port
					&dwBytesWritten,		// number of bytes that were actually written
					NULL))					// overlapped I/O not supported			
			{
				ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to Pilot Controller!\n" )
				return;
			}
		}
		#ifdef  DEBUG_PILOT_READWRITE_TIME
			ROBOT_LOG( TRUE,  "Pilot Write Time: %d ms\n", (GetTickCount() - dwStartTime) );
		#endif

		// Sent the command, now look for ACK
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainMotorThread);
			return;
		}
		SetCommMask (g_hMotorCommPort, EV_RXCHAR | EV_BREAK);
//		memset(m_ReadBuf,0,READ_BUF_SIZE);
		m_PilotReply.CommandStatus = 0;
		m_PilotReply.Checksum = 0;
		memset(m_PilotReply.Data,0,MAX_REPLY_BYTES);

		// Read ANSI characters
		dwStartTime = GetTickCount();
		if(!ReadFile(g_hMotorCommPort,	// where to read from (the open comm port)
				 &m_PilotReply,				// where to put the characters
				 nBytesToRead,			// number of bytes to read
				 &dwBytesReceived,		// how many bytes were actually read
				 NULL))					// no overlapped I/O
		{
			// Has the comm port been closed?
			if(GetLastError() != ERROR_INVALID_HANDLE)
			{
				ROBOT_DISPLAY( TRUE, "Pilot SendWriteCmd:  Error writing to Pilot Controller!\n" )
			}
			return;	// TODO: terminate thread on error???
		}

		#ifdef  DEBUG_PILOT_READWRITE_TIME
			ROBOT_LOG( TRUE,  "Pilot Read Time: %d ms\n", (GetTickCount() - dwStartTime) )
		#endif

	}
	else
	{
		// For offline debugging / testing
		dwBytesReceived = nResponseBytes + 2;	// DEBUG ONLY!!!!
	}


	if( 0 == dwBytesReceived )
	{
		// Error, no ACK received
		ROBOT_DISPLAY( TRUE, "Pilot SendWriteCmd:  No Response from Pilot Controller!" )
		return;
	}

	// Got a response
	#ifdef  DEBUG_PILOT_SHOW_ACK_MESSAGES

		ROBOT_LOG( TRUE,  "PILOT RECEIVED (%d Data Bytes): %02X   %02X     ",
			nResponseBytes, m_PilotReply.CommandStatus, m_PilotReply.Checksum )

		if( dwBytesReceived > 2)
		{
			for( q=0; q < (dwBytesReceived - 2); q++ )
			{
				ROBOT_LOG( TRUE,  "%02X ",	m_PilotReply.Data[q] );
			}
		}
		ROBOT_LOG( TRUE,  "\n" );
		ROBOT_LOG( TRUE,  "                           Status   Cksm   Data\n" );
	#endif

	// Calculate Checksum
	// Bottom 8 bits of ResponseSum + CheckSum should be zero for valid packet

	int  ResponseSum = 0;	// Total sum of all bytes except for the CheckSum
	for( q=0; q < nResponseBytes; q++)
	{
		ResponseSum += m_PilotReply.Data[q];	// Sum all the data bytes
	}
	ResponseSum += m_PilotReply.CommandStatus;

	int  FullCheckSum = ResponseSum + m_PilotReply.Checksum;
	BYTE TrimCheckSum = (BYTE)FullCheckSum;	// Chop off anything over 8 bits  Should be zero

	if( TrimCheckSum != 0)	// Should be zero
	{
		MsgString.Format("Pilot SendWriteCmd: Bad Checksum in Response from Pilot! ResponseSum(%04X) + Checksum)(%02Xd) =  FullCheckSum(%04X), TrimCheckSum = %04X\n", 
			ResponseSum, m_PilotReply.Checksum, FullCheckSum, TrimCheckSum );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		// TODO!  If multiple checksum errors, figure out if we can reset the Pilot controller without confusing the map!

	}
	else if( 0 != m_PilotReply.CommandStatus )
	{
		MsgString.Format( "Pilot NCK received %d Bytes: SendWriteCmd ERROR - Cmd %02x Error Code = %d, CheckSum = %2x, Data = %2x %2x", 
			dwBytesReceived, m_PilotCmd.Code, m_PilotReply.CommandStatus, m_PilotReply.Checksum, m_PilotReply.Data[0], m_PilotReply.Data[1] );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		DisplayIOError();
	}
	else
	{
		#ifdef  DEBUG_PILOT_SHOW_ACK_MESSAGES
			ROBOT_LOG( TRUE,  "Pilot ACK received %d Bytes: SendWriteCmd GOOD  - Cmd %02x Error Code = %d, CheckSum = %2x, Data = %2x %2x", 
				dwBytesReceived, m_PilotCmd.Code, m_PilotReply.CommandStatus, m_PilotReply.Checksum, m_PilotReply.Data[0], m_PilotReply.Data[1] )
		#endif
	}
}

///////////////////////////////////////////////////////////////////////////////
void CPilotControl::HandlePilotCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 )
{
	IGNORE_UNUSED_PARAM (Option1);
	IGNORE_UNUSED_PARAM (Option2);

	switch( Request )
	{		
		case HW_GET_STATUS:
		{
			GetStatus();
			break;
		}
		case HW_INITIALIZE:
		{
			Init();
			break;
		}
		case HW_SET_MOTOR_STOP:
		{
			m_Speed = 0;
			m_Turn = 0;
			SetWheelSpeedAndTurn();
			break;
		}
		case HW_SET_TURN:
		{
			// Param1 = New Turn (+/- 127)
			m_Turn = Param1;
			SetWheelSpeedAndTurn();
			break;
		}
/*
		case HW_SET_SPEED:
		{
			// Param1 = New Speed (+/- 127)
			m_Speed = Param1;
			SetWheelSpeedAndTurn();
			break;

		}
*/
		case HW_SET_SPEED_AND_TURN:
		{
			// Param1 is Speed, Param2 is Turn
			// ROBOT_DISPLAY( TRUE, "Server ACK: JOYSTICK DRIVE CMD" )
			// Speed and Turn commands from GUI are +/- 127, with zero = Center/Stop 
			// Convert to HW command values, for reverse, we add a negative number
			m_Speed = (signed short)LOWORD(Param1);  // does not use Acceleration 
			m_Turn = (signed char)Param2;
			SetWheelSpeedAndTurn();
			break;
		}
		default:
		{
			CString StrText;
			StrText.Format( "ERROR! MotorCommThread: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, StrText)
		}

	}

}


#endif // ROBOT_SERVER  // Nothing in this file used for client!
