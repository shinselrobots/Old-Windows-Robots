// MotorControlTrex.cpp
// Interface for the Pololu TReX Motor Controller

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
//#define DEBUG_TREX_COMMAND_DUMP   // Dump all Trex commands
//#define DEBUG_TREX_READWRITE_TIME  // Log Read/Write SIO latency
//#define DEBUG_TREX_SHOW_ACK_MESSAGES

/////////////////////////////////////////////////////

#define LEFT_MOTOR			1
#define RIGHT_MOTOR			0

#define TREX_MOTOR_BRAKE_LOW_1		0x00
#define TREX_MOTOR_REVERSE			0x01
#define TREX_MOTOR_FORWARD			0x02
#define TREX_MOTOR_BRAKE_LOW_2		0x03

#define TREX_MOTOR_ACCELERATE_CMD	0xE0


static char TrexCmdBuf[40];
const char* LEFT_WHEEL_STR = "Left";
const char* RIGHT_WHEEL_STR	= "Right";

#define DEBUG_SPEED_CONTROL		0 // if true, print debug messages

// CString strStatus;

/////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// PMD Trex Motor Control Class
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////

CTrexControl::CTrexControl()
{
	m_TrexCmd = 0;		// Insert command code here!

//	memset(m_ReadBuf,0,TREX_READ_BUF_SIZE);
	m_Speed = 0;
	m_Turn = 0;

	m_MotorSpeedCmdLeft = 0;
	m_MotorSpeedRequestLeft = 0;
	m_TachometerTargetLeft = 0;

	m_MotorSpeedCmdRight = 0;
	m_MotorSpeedRequestRight = 0;
	m_TachometerTargetRight = 0;
	m_StraightTracking = 0;

}

void CTrexControl::Init()
{
	memset(m_CmdBuf,0,TREX_CMD_BUF_SIZE);
	g_MotorSubSystemStatus = SUBSYSTEM_CONNECTED;

	// DEBUG!!
//	GetStatus();

}

int CTrexControl::SetTachometerTarget( int TargetSpeed )
{
	// Given a target Speed, return the corresponding target Tachometer value
	// For Loki, this is a flat ratio

	double Tach = (double)TargetSpeed * 0.92;
	return (int)Tach;
}

///////////////////////////////////////////////////////////////////////////////
BOOL CTrexControl::SingleWheelSpeedControl( int &MotorSpeedCmd, int MotorSpeedRequest, int TachometerTicks, int TachometerTarget, const char* WhichWheel )
{
	// Does Speed Control for one wheel.  Modifies MotorSpeedCmd and returns TRUE if MotorSpeedCmd was updated.
	// Assumes wheel is forward-mounted.  If reversed, switch signs before calling this function
	int OldMotorSpeedCmd = MotorSpeedCmd;

	//KLUDGE! for some reason (HW BUG?) Tach Ticks sometimes report wrong sign
	// we see this only when one wheel moving forward, and one moving backward.  Weird.
	// So, ignore what the HW reports, and use the commanded direction instead
	int absTachometerTicks = abs(TachometerTicks);
	int absTachometerTarget = abs(TachometerTarget);

	if( absTachometerTicks < absTachometerTarget-1 )
	{
		// Going too slow
		if( abs(MotorSpeedCmd) < (abs(MotorSpeedRequest) + MAX_SPEED_CORRECTION) ) // don't exceed max correction
		{
			if( MotorSpeedRequest > 0 )
			{
				// going forward
				MotorSpeedCmd++;	// Speed up forward
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SPEED UP FORWARD\n", WhichWheel )
			}
			else if( MotorSpeedRequest < 0 )
			{
				// going reverse
				MotorSpeedCmd--;	// Speed up reverse
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SPEED UP REVERSE\n", WhichWheel )
			}
		}
	}
	else if( absTachometerTicks > absTachometerTarget+1 )
	{
		// Going too fast
		if( abs(MotorSpeedCmd) > 0 ) // don't slow down below Stop (don't go negative)
		{
			if( MotorSpeedRequest > 0 )
			{
				// going forward
				MotorSpeedCmd--;	// Slow down forward
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SLOW DOWN FORWARD\n", WhichWheel )
			}
			else if( MotorSpeedRequest < 0 )
			{
				// going reverse
				MotorSpeedCmd++;	// Slow down reverse
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SLOW DOWN REVERSE\n", WhichWheel )
			}
		}
	}




/***
	if( MotorSpeedRequest > 0 )
	{
		// going forward
		if( absTachometerTicks < TachometerTarget-1 )
		{
			// Going too slow
			if( MotorSpeedCmd < (MotorSpeedRequest + MAX_SPEED_CORRECTION) )
			{
				MotorSpeedCmd++;	// Speed up forward
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SPEED UP FORWARD\n", WhichWheel )
			}
		}
		else if( absTachometerTicks > TachometerTarget+1 )
		{
			// Going too fast
			if( MotorSpeedCmd > (MotorSpeedRequest - MAX_SPEED_CORRECTION) )
			{
				MotorSpeedCmd--;	// Slow down forward
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SLOW DOWN FORWARD\n", WhichWheel )
			}
		}
		if( MotorSpeedCmd < 0 ) 
			MotorSpeedCmd = 0; // Clamp at zero - just stop the wheel if it's overshooting

	}
	else if( MotorSpeedRequest < 0 )
	{
		// going reverse
		if( absTachometerTicks > TachometerTarget+1 )
		{
			// Going too slow
			if( MotorSpeedCmd > (MotorSpeedRequest - MAX_SPEED_CORRECTION) )
			{
				MotorSpeedCmd--;	// Speed up reverse
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SPEED UP REVERSE\n", WhichWheel )
			}
		}
		else if( absTachometerTicks < TachometerTarget-1 )
		{
			// Going too fast
			if( MotorSpeedCmd < (MotorSpeedRequest + MAX_SPEED_CORRECTION) )
			{
				MotorSpeedCmd++;	// Slow down reverse
				ROBOT_LOG( DEBUG_SPEED_CONTROL, "SPEEDCTRL: %s Wheel SLOW DOWN REVERSE\n", WhichWheel )
			}
		}
		if( MotorSpeedCmd > 0 ) 
			MotorSpeedCmd = 0; // Clamp at zero - safety check

	}
***/

	// Clamp max and min motor speeds
	if( MotorSpeedCmd > 127 ) MotorSpeedCmd = 127;
	if( MotorSpeedCmd < -127 ) MotorSpeedCmd = -127;

	return (MotorSpeedCmd != OldMotorSpeedCmd);

}

///////////////////////////////////////////////////////////////////////////////
#define STRAIGHT_TRACKING_THRESHOLD 40
#define STRAIGHT_TRACKING_RATIO		10

void CTrexControl::SpeedControl()
{
	// Does Speed Control for each wheel independently
	// since the wheels are oposite each other (one mounted backward)
	// We drive one "forward" and one "reverse" to make the robot go forward!

	// Note: Amazingly, the Command (0-127) and the Motor Ticks map almost exactly! But they are not quite the same.
	// Motor ticks trend slightly less than command, making matching a great target for speed control.

	// if( 0 != gMotorSpeedTimer ) NOT USED!

	// Tracking control to make robot go straight
	int TrackingOffset = 0;

	if( (0 == m_Turn) && (0 != m_Speed) )
	{
		// Robot going straight, update the tracking counter
		// This number shows how much cumlative error has occured
		m_StraightTracking += g_pFullSensorStatus->TachometerTicksL - g_pFullSensorStatus->TachometerTicksR;
		TrackingOffset = m_StraightTracking / STRAIGHT_TRACKING_RATIO;
	}


	BOOL RightMotorChanged = SingleWheelSpeedControl( m_MotorSpeedCmdRight, m_MotorSpeedRequestRight, 
							g_pFullSensorStatus->TachometerTicksR, (m_TachometerTargetRight+TrackingOffset), RIGHT_WHEEL_STR );


	BOOL LeftMotorChanged = SingleWheelSpeedControl( m_MotorSpeedCmdLeft, m_MotorSpeedRequestLeft, 
							g_pFullSensorStatus->TachometerTicksL, (m_TachometerTargetLeft-TrackingOffset), LEFT_WHEEL_STR );

	if( (0 != m_MotorSpeedRequestLeft) || (0 != m_MotorSpeedRequestRight) )
	{
		ROBOT_LOG( DEBUG_SPEED_CONTROL, "Right: Request: %3d  Current Cmd: %3d  Target Tach: %3d, Current Tach: %3d", 
			m_MotorSpeedRequestRight, m_MotorSpeedCmdRight, m_TachometerTargetRight, g_pFullSensorStatus->TachometerTicksR )

		ROBOT_LOG( DEBUG_SPEED_CONTROL, "Left:  Request: %3d  Current Cmd: %3d  Target Tach: %3d, Current Tach: %3d, Straight Tracking: %3d", 
			m_MotorSpeedRequestLeft, m_MotorSpeedCmdLeft, m_TachometerTargetLeft, g_pFullSensorStatus->TachometerTicksL, m_StraightTracking )
	}


	if( LeftMotorChanged || RightMotorChanged )
	{
		// Update needed
		ROBOT_LOG( DEBUG_SPEED_CONTROL, "New Speed Delta: L=%d, R=%d   <==================\n", 
			(m_MotorSpeedCmdLeft - m_MotorSpeedRequestLeft), (m_MotorSpeedCmdRight - m_MotorSpeedRequestRight) )

//		gMotorSpeedTimer = SPEED_CHANGE_TIME;	// give time for motors to respond before checking again
		TrexSendAccelerateCmd( m_MotorSpeedCmdLeft, m_MotorSpeedCmdRight );
	}

}


///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////


void CTrexControl::TrexSendAccelerateCmd( int MotorSpeedL, int MotorSpeedR )
{

	// Record current command for speed control
	m_MotorSpeedCmdLeft = MotorSpeedL;
	m_MotorSpeedCmdRight = MotorSpeedR;


	BYTE DirL = TREX_MOTOR_FORWARD;
	BYTE DirR = TREX_MOTOR_FORWARD;
	// KLUDGE!! TODO-MUST FIX THIS! (hardware error work around)
	g_MotorKludgeRevR = FALSE;
	g_MotorKludgeRevL = FALSE;

	if( MotorSpeedL < 0 )
	{
		DirL = TREX_MOTOR_REVERSE;
		g_MotorKludgeRevL = TRUE;
	}
	if( MotorSpeedR < 0 )
	{
		DirR = TREX_MOTOR_REVERSE;
		g_MotorKludgeRevR = TRUE;
	}

	int  Command = TREX_MOTOR_ACCELERATE_CMD | (DirR * 4) | ( DirL );
	int  AbsSpeedL = abs( MotorSpeedL );
	int  AbsSpeedR = abs( MotorSpeedR );

//	ROBOT_LOG( DEBUG_SPEED_CONTROL,  "*********> TREX SPEED = %d, %d\n", AbsSpeedL,AbsSpeedR)
	ROBOT_LOG( DEBUG_SPEED_CONTROL,  "*********> TREX SPEED = %d, %d\n", MotorSpeedL, MotorSpeedR)

	m_CmdBuf[0] = (char)Command;
	m_CmdBuf[1] = (char)AbsSpeedL;
	m_CmdBuf[2] = (char)AbsSpeedR;
	int  nBytesToSend = 3;

	SendWriteCmd( nBytesToSend );
}


///////////////////////////////////////////////////////////////////////////////
void CTrexControl::SetWheelSpeedAndTurn()
{
	// "Mixes" Speed and Turn to set speed of the motors
	// since the wheels are oposite each other (one mounted backward)
	// We drive one "forward" and one "reverse" to make the robot go forward!
	// TODO if making a class: m_Speed and m_Turn are set prior to calling this function

	m_MotorSpeedRequestLeft = m_Speed + m_Turn;
	m_MotorSpeedRequestRight = m_Speed - m_Turn;
//		m_LeftMotorSpeedRequest = ~m_LeftMotorSpeedRequest +1;	// invert left motor

	if( m_MotorSpeedRequestLeft > 127 )
	{
		m_MotorSpeedRequestLeft = 127;
	}
	if( m_MotorSpeedRequestLeft < -127 )
	{
		m_MotorSpeedRequestLeft = -127;
	}

	if( m_MotorSpeedRequestRight > 127 )
	{
		m_MotorSpeedRequestRight = 127;
	}
	if( m_MotorSpeedRequestRight < -127 )
	{
		m_MotorSpeedRequestRight = -127;
	}


	// Set the Tachometer Target reading for the speed control.
	m_TachometerTargetLeft = SetTachometerTarget( m_MotorSpeedRequestLeft );
	m_TachometerTargetRight = SetTachometerTarget( m_MotorSpeedRequestRight );
	m_StraightTracking = 0; // Goes +/- if drifting to left or right

	gMotorSpeedTimer = NEW_SPEED_SET_TIME;		// give time for motors to respond before checking again

	TrexSendAccelerateCmd( m_MotorSpeedRequestLeft, m_MotorSpeedRequestRight );
}


///////////////////////////////////////////////////////////////////////////////
void CTrexControl::DisplayIOError()
{
	/****
	CString MsgString;
	CString strText;
	
	switch(m_TrexReply.CommandStatus)
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
	

	MsgString.Format( "========> Trex Error %02X: %s!", 
		m_TrexReply.CommandStatus, strText );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	***/

}

///////////////////////////////////////////////////////////////////////////////
void CTrexControl::SendWriteCmd(int  nBytesToSend )
{
	// send a write command with no response data expected
	SendCmd( nBytesToSend, 0 );	// Send the Command via COM port
}

///////////////////////////////////////////////////////////////////////////////
//BOOL CTrexControl::SendReadCmd( int  nResponseBytes )
//{
	// send a read command with no parameters
//	BOOL SendOK = TRUE;

/***

	CString MsgString;

	SendCmd( 0, nResponseBytes );	// Send the Command via COM port

	// Check for errors
	if( 0 != m_TrexReply.CommandStatus )
	{
		// Error already displayed.  Need to reset the error
		SendOK = FALSE;

		// GetHostIOError - This clears the prior error.
		// Response data should be same as the error previously reported
		ROBOT_DISPLAY( TRUE, "Clearing Reset Error Bit" )
		m_TrexCmd.Code = GetHostIOError;
		SendCmd(0,2);

		if( 0 != m_TrexReply.CommandStatus )
		{
			MsgString.Format( "Error %02X while trying to Reset %02X", 
				m_TrexReply.CommandStatus, m_TrexReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
		else
		{
			MsgString.Format( "Error %02X has been Reset", m_TrexReply.Data[1] ); // Ignore Data[0]
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}
	}
***/
//	return SendOK;
//}
///////////////////////////////////////////////////////////////////////////////
void CTrexControl::GetStatus()
{
	//ROBOT_LOG( DEBUG_SPEED_CONTROL,  "STATUS Requested\n" )


/***	
	// Get current Status from the Trex controller
	CString MsgString;
#define DEBUG_TREX 0

#if DEBUG_TREX == 1
	ROBOT_LOG( DEBUG_SPEED_CONTROL,  "------------------- STATUS UPDATE --------------\n" )
#endif
	////////////////////////////////////////////////////////
	// Speed / Tach
	////////////////////////////////////////////////////////
	m_TrexCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_TrexCmd.Axis = 0;
	m_TrexCmd.Code = GetCommandedVelocity;		// see how fast the motor is ACTUALLY going
	if( SendReadCmd(4) )
	{
#if DEBUG_TREX == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_TrexReply.Data[0], m_TrexReply.Data[1], m_TrexReply.Data[2], m_TrexReply.Data[3] )
#endif
	}
	long WheelSpeedL = htonl( *(long*)( &m_TrexReply.Data[0] ) );
	WheelSpeedL = (~WheelSpeedL) + 1;	// Wheel pointing backward, negate the value

	m_TrexCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_TrexCmd.Axis = 0;
	m_TrexCmd.Code = GetCommandedVelocity;	
	if( SendReadCmd(4) )
	{
#if DEBUG_TREX == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_TrexReply.Data[0], m_TrexReply.Data[1], m_TrexReply.Data[2], m_TrexReply.Data[3] )
#endif
	}
	long WheelSpeedR = htonl( *(long*)( &m_TrexReply.Data[0] ) );

	if( (WheelSpeedR != 0) || (WheelSpeedR != 0) )
	{
#if DEBUG_TREX == 1
		MsgString.Format( "Trex Speed: LEFT_MOTOR: %ld, RIGHT_MOTOR: %ld",
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
	m_TrexCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_TrexCmd.Axis = 0;
	m_TrexCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_TREX == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_TrexReply.Data[0], m_TrexReply.Data[1], m_TrexReply.Data[2], m_TrexReply.Data[3] )
#endif
	}
	long WheelDistanceL = htonl( *(long*)( &m_TrexReply.Data[0] ) );
	WheelDistanceL = (~WheelDistanceL) + 1;	// Wheel opposite, negate the value

	m_TrexCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_TrexCmd.Axis = 0;
	m_TrexCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_TREX == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_TrexReply.Data[0], m_TrexReply.Data[1], m_TrexReply.Data[2], m_TrexReply.Data[3] )
#endif
	}
	long WheelDistanceR = htonl( *(long*)( &m_TrexReply.Data[0] ) );

	if( (WheelDistanceR != 0) || (WheelDistanceL != 0) )
	{
//		ROBOT_LOG( TRUE,  "MotorControl: Trex Distance Moved (Inches): LEFT_MOTOR: %0.2f  RIGHT_MOTOR:%0.2f",
//			WheelDistanceL/TICKS_PER_TENTH_INCH, WheelDistanceR/TICKS_PER_TENTH_INCH )

		// Each wheel is sent, so turns can be computed
		// We assume that WPARAM and LPARAM are both 32 bits (see ROBOT_ASSERT in SetupView.cpp)

		// Update the Robot Control thread with the new distance traveled (in "ticks")
		// this in turn will update the GUI with Odometer and current location
		PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_ER1_ODOMETER_READY),
			WheelDistanceL, WheelDistanceR );

	}
#if DEBUG_TREX == 1
	ROBOT_LOG( TRUE,  "------------------------------------------------\n" )
#endif
	***/
}

///////////////////////////////////////////////////////////////////////////////
void CTrexControl::SendCmd(int  nCmdBytes, int  nResponseBytes )
{
	
	CString MsgString;
	DWORD dwBytesWritten=0;
//	int  Status = 0;
//	int  ReadCheckSum = 0;
//	unsigned char DbgBuf[16];
//	memset(DbgBuf,0, 16);
//	int  q;
	DWORD dwBytesReceived = 0;

	int  nBytesToRead = nResponseBytes + 2;	// Add 2 for the CommandStatus and Checksum

	// Debug:
	//	MsgString.Format( "Trex SendWriteCmd:  Sending cmd %02X to Trex controller.\n", m_TrexCmd.Code );
	//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )


	
#ifdef DEBUG_TREX_COMMAND_DUMP

	ROBOT_LOG( TRUE,  "TREX CMD DUMP (%d Data Bytes): ", nCmdBytes );
	for( int q=0; q < nCmdBytes; q++ )
	{
		ROBOT_LOG( TRUE,  "%02X ",	(unsigned char)m_CmdBuf[(q)] );
	}
	ROBOT_LOG( TRUE,  "\n" );
#endif

	if( 1 == ROBOT_SIMULATION_MODE )
	{
		RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainMotorThread);
		return;
	}

	if(!WriteFile(g_hMotorCommPort,	// where to write to (the open comm port)
			&m_CmdBuf,				// what to write
			nCmdBytes,				// number of bytes to be written to port
			&dwBytesWritten,		// number of bytes that were actually written
			NULL))					// overlapped I/O not supported			
	{
		ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to Trex Controller!\n" )
		return;
	}
	else
	{
		if( dwBytesWritten != (DWORD)nCmdBytes )
		{
			MsgString.Format( "TREX SERIAL ERROR Bytes Written = %d!\n", dwBytesWritten );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			//continue;
		}
	}


/****
		// Sent the command, now look for ACK
		SetCommMask (g_hMotorCommPort, EV_RXCHAR | EV_BREAK);
//		memset(m_ReadBuf,0,TREX_READ_BUF_SIZE);
		m_TrexReply.CommandStatus = 0;
		m_TrexReply.Checksum = 0;
		memset(m_TrexReply.Data,0,TREX_MAX_REPLY_BYTES);

		// Read ANSI characters
		dwStartTime = GetTickCount();
		if(!ReadFile(g_hMotorCommPort,	// where to read from (the open comm port)
				 &m_TrexReply,				// where to put the characters
				 nBytesToRead,			// number of bytes to read
				 &dwBytesReceived,		// how many bytes were actually read
				 NULL))					// no overlapped I/O
		{
			// Has the comm port been closed?
			if(GetLastError() != ERROR_INVALID_HANDLE)
			{
				ROBOT_DISPLAY( TRUE, "Trex SendWriteCmd:  Error writing to Trex Controller!\n" )
			}
			return;	// TODO: terminate thread on error???
		}

		#ifdef  DEBUG_TREX_READWRITE_TIME
			ROBOT_LOG( TRUE,  "Trex Read Time: %d ms\n", (GetTickCount() - dwStartTime) )
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
		ROBOT_DISPLAY( TRUE, "Trex SendWriteCmd:  No Response from Trex Controller!" )
		return;
	}

	// Got a response
	#ifdef  DEBUG_TREX_SHOW_ACK_MESSAGES

		ROBOT_LOG( TRUE,  "TREX RECEIVED (%d Data Bytes): %02X   %02X     ",
			nResponseBytes, m_TrexReply.CommandStatus, m_TrexReply.Checksum )

		if( dwBytesReceived > 2)
		{
			for( q=0; q < (dwBytesReceived - 2); q++ )
			{
				ROBOT_LOG( TRUE,  "%02X ",	m_TrexReply.Data[q] );
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
		ResponseSum += m_TrexReply.Data[q];	// Sum all the data bytes
	}
	ResponseSum += m_TrexReply.CommandStatus;

	int  FullCheckSum = ResponseSum + m_TrexReply.Checksum;
	BYTE TrimCheckSum = (BYTE)FullCheckSum;	// Chop off anything over 8 bits  Should be zero

	if( TrimCheckSum != 0)	// Should be zero
	{
		MsgString.Format("Trex SendWriteCmd: Bad Checksum in Response from Trex! ResponseSum(%04X) + Checksum)(%02Xd) =  FullCheckSum(%04X), TrimCheckSum = %04X\n", 
			ResponseSum, m_TrexReply.Checksum, FullCheckSum, TrimCheckSum );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		// TODO!  If multiple checksum errors, figure out if we can reset the Trex controller without confusing the map!

	}
	else if( 0 != m_TrexReply.CommandStatus )
	{
		MsgString.Format( "Trex NCK received %d Bytes: SendWriteCmd ERROR - Cmd %02x Error Code = %d, CheckSum = %2x, Data = %2x %2x", 
			dwBytesReceived, m_TrexCmd.Code, m_TrexReply.CommandStatus, m_TrexReply.Checksum, m_TrexReply.Data[0], m_TrexReply.Data[1] );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		DisplayIOError();
	}
	else
	{
		#ifdef  DEBUG_TREX_SHOW_ACK_MESSAGES
			ROBOT_LOG( TRUE,  "Trex ACK received %d Bytes: SendWriteCmd GOOD  - Cmd %02x Error Code = %d, CheckSum = %2x, Data = %2x %2x", 
				dwBytesReceived, m_TrexCmd.Code, m_TrexReply.CommandStatus, m_TrexReply.Checksum, m_TrexReply.Data[0], m_TrexReply.Data[1] )
		#endif
	}
****/
}

///////////////////////////////////////////////////////////////////////////////
void CTrexControl::HandleCommand( int  Request, int  Param1, int  Param2 )
{

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

		case HW_UPDATE_MOTOR_SPEED_CONTROL:
		{
			// Use this to disable speed control
			//ROBOT_DISPLAY( TRUE, "WARNING SpeedControl DISABLED!!! TODO!!! ")
			SpeedControl();
			break;
		}

		case HW_SET_TURN:
		{
			/*
			if(  SUBSYSTEM_CONNECTED != g_ArduinoSubSystemStatus )
			{
				m_Speed = 0;
				m_Turn = 0;
				SetWheelSpeedAndTurn();
				ROBOT_DISPLAY( TRUE, "TREX: ERROR - Arduino NOT CONNECTED! MOTOR DISABLED!")
				break;
			}
			else
			*/
			{
				// Param1 = New Turn (+/- 127)
				m_Turn = Param1;
				SetWheelSpeedAndTurn();
				break;
			}
		}
/*
		case HW_SET_SPEED:
		{
			//
			//if(  SUBSYSTEM_CONNECTED != g_ArduinoSubSystemStatus )
			//{
			//	m_Speed = 0;
			//	m_Turn = 0;
			//	SetWheelSpeedAndTurn();
			//	ROBOT_DISPLAY( TRUE, "TREX: ERROR - Arduino NOT CONNECTED! MOTOR DISABLED!")
			//	break;
			//}
			//else
			//
			{
				// Param1 = New Speed (+/- 127)
				m_Speed = Param1;
				SetWheelSpeedAndTurn();
				break;
			}
		}
*/
		case HW_SET_SPEED_AND_TURN:
		{
			/*
			if(  SUBSYSTEM_CONNECTED != g_ArduinoSubSystemStatus )
			{
				m_Speed = 0;
				m_Turn = 0;
				SetWheelSpeedAndTurn();
				ROBOT_DISPLAY( TRUE, "TREX: ERROR - Arduino NOT CONNECTED! MOTOR DISABLED!")
				break;
			}
			else
			*/
			{
				// Param1 is Speed, Param2 is Turn
				// ROBOT_DISPLAY( TRUE, "Server ACK: JOYSTICK DRIVE CMD" )
				// Speed and Turn commands from GUI are +/- 127, with zero = Center/Stop 
				// Convert to HW command values, for reverse, we add a negative number
				m_Speed = (signed short)LOWORD(Param1);  // does not use Acceleration
				m_Turn = (signed short)Param2;
				SetWheelSpeedAndTurn();
				break;
			}
		}
		default:
		{
			CString StrText;
			StrText.Format( "ERROR! MotorCommThread: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, (StrText) )
		}

	}

}


#endif // ROBOT_SERVER  // Nothing in this file used for client!
