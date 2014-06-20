// MotorControlKobuki.cpp - DEPRICATED!!!  DONT USE!

// Interface for the Kobuki Base


// This class is used by the thread loop that reads commands from the queue in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only



#include "Globals.h"
#include "MotorControl.h"
#include "../Common/HardwareCmds.h" // for TICS_PER_INCH

#include "KobukiCommon.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_KOBUKI_COMMAND_DUMP   // Dump all Kobuki commands
#define DEBUG_KOBUKI_SHOW_ACK_MESSAGES
#define KOBUKI_DUMP_RECEIVED_BYTES 0

/////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Kobuki Motor Control Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CKobukiControl::CKobukiControl()
{
	//m_pReadBuf[Kobuki_READ_BUF_SIZE]
	//m_pCmdBuf = new BYTE[Kobuki_CMD_BUF_SIZE];
	memset(&m_pCmdBuf,0,KOBUKI_CMD_BUF_SIZE);
	m_pCmdBuf[0] = 0xAA; // first sync character
	m_pCmdBuf[1] = 0x55; // first sync character
	// m_pCmdBuf[2] = length of command in bytes
	memset(&m_pCmdMsg,0,KOBUKI_CMD_BUF_SIZE);
	//m_StartCmdSent = FALSE;
	m_Speed = 0;
	m_Turn = 0;
	m_Acceleration = ACCELERATION_MEDIUM;
	m_LeftWheelStopped = TRUE;
	m_RightWheelStopped = TRUE;
	m_Docked = FALSE;
	m_LEDPower = 0;

	memset( &m_KobukiCommand, 0, sizeof(KOBUKI_COMMAND_T) );


	g_MotorSubSystemStatus = SUBSYSTEM_WAITING;

	ROBOT_LOG( TRUE, "==============================>>>>>>> CONSTRUCTOR CALLED <<<<<<<<<<<==========================" )

}

CKobukiControl::~CKobukiControl()
{
	// Power down during the destructor
	g_DynaPowerEnabled = FALSE;
	g_KinectPowerEnabled = FALSE;
	m_LEDPower = FALSE;
	//SetExternalPower(); - not needed
	Sleep(1);
	ShutDown();
	Sleep(1);
}


///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////
#define KOBUKI_SPEED_MAX		100	// Doc says this is mm/s, but I wonder if it's 100%?

// TODO: http://yujinrobot.github.io/kobuki/doxygen/enAppendixProtocolSpecification.html
// Commands from "Create Open Interface v2.pdf"
#define KOBUKI_MOTOR_DRIVE_CMD				 1	// Use speend and turn radius to drive the wheel motors
#define KOBUKI_REQUEST_EXTRA_CMD			 9   // Hardware Version, Firmware Version and Unique Device IDentifier(UDID)
#define KOBUKI_DIGITAL_OUTPUT_CMD			12 // Control state of I/O port bits 0, 1, 2, 4, and External (Kinect) Power, and LEDs
//#define KOBUKI_START_SENSOR_STREAM_CMD		148 // Start streaming all sensor data every 15ms




void CKobukiControl::Init()
{
	// Init at robot startup
	ROBOT_LOG( TRUE, "==============================>>>>>>> INIT CALLED <<<<<<<<<<<==========================" )

	/* NOT use for Kobuki!!  Kinect power is on by default!
		SetPort(); // Initialize power for Kinect and Dynamixel Servos

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Launch the C# Kinect Capture Application here for iRobot or Kobuki bases, after turning on power to the sensor
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		#if (ROBOT_TYPE == TURTLE) 
			RobotSleep(2000, pDomainMotorThread); // allow kinect to power up
			LaunchKinectApp(); // defined in globals.cpp
		#endif
	*/

	///g_MotorControlInitialized = TRUE;
	ROBOT_LOG( TRUE, "MotorControlKobuki: Init Complete\n" )
}



void CKobukiControl::SetDigitalIO( unsigned int Pins)
{

	// Enable or disable each pin.

	m_KobukiCommand.DigitalIO = Pins;

	// Send command to the Kobuki control app via shared memory
	if( (NULL != g_pKobukiCommandSharedMemory) && (NULL != g_hKobukiCommandEvent) )
	{
		CopyMemory((PVOID)g_pKobukiCommandSharedMemory, &m_KobukiCommand, (sizeof(KOBUKI_COMMAND_T)));
		SetEvent( g_hKobukiCommandEvent );  // Tell Kobuki app that a new command is pending
		ROBOT_LOG( TRUE, "Sending message to KobukiControl app - Digital IO\n" )
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR: Cant send command to Kobuki!  Did you have AUTO_LAUNCH_KOBUKI_APP enabled?\n" )
	}

}

void CKobukiControl::SetExternalPower( )
{
	// Manage power for Kinect and Servos
	// 0x01 for external power 3.3V		- NOT USED
	// 0x02 for external power 5V		- NOT USED
	// 0x04 for external power 12V/5A	- Servos (Led 1)
	// 0x08 for external power 12V/1.5A	- Kinect (Led 2)

	// Just for fun, set LEDs at the same time to indicate state of power
	// 0x01 for red color of LED1
	// 0x02 for green color of LED1
	// 0x04 for red color of LED2
	// 0x08 for green color of LED2

	if( g_DynaPowerEnabled )
	{
		m_KobukiCommand.ExternPower |= 0x04;	// Turn the Dynamixel Servo Power bit ON
		m_KobukiCommand.LEDState |= 0x02;		// Led 1 = Green
	}
	else
	{
		m_KobukiCommand.ExternPower &= 0xFB;	// Turn the Dynamixel Servo Power bit OFF
		m_KobukiCommand.LEDState |= 0xFD;		// Led 1 = Off
	}


	if( g_KinectPowerEnabled )
	{
		m_KobukiCommand.ExternPower |= 0x08;	// Turn the Kinect Power bit ON
		m_KobukiCommand.LEDState |= 0x08;		// Led 2 = Green
	}
	else
	{
		m_KobukiCommand.ExternPower &= 0xF7;	// Turn the Kinect Power bit OFF
		m_KobukiCommand.LEDState |= 0xF7;		// Led 2 = Off
	}


	// Send command to the Kobuki control app via shared memory
	if( (NULL != g_pKobukiCommandSharedMemory) && (NULL != g_hKobukiCommandEvent) )
	{
		CopyMemory((PVOID)g_pKobukiCommandSharedMemory, &m_KobukiCommand, (sizeof(KOBUKI_COMMAND_T)));
		SetEvent( g_hKobukiCommandEvent );  // Tell Kobuki app that a new command is pending
		ROBOT_LOG( TRUE, "Sending message to KobukiControl app - External Power\n" )
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR: Cant send command to Kobuki!  Did you have AUTO_LAUNCH_KOBUKI_APP enabled?\n" )
	}

}


///////////////////////////////////////////////////////////////////////////////
/*
#ifndef ACCELERATION_SLOW
	#define ACCELERATION_SLOW		0
	#define ACCELERATION_MEDIUM		1
	#define ACCELERATION_FAST		2
	#define ACCELERATION_INSTANT	3
#endif
*/


void CKobukiControl::SetWheelSpeedAndTurn()
{
#define WHEEL_BASE 230 // mm between wheels

	// Convert from standard +/- 127 values to Kobuki values in mm per second 
	// mm/s = 500 * value/127

	int Command = KOBUKI_MOTOR_DRIVE_CMD;
	int Turn = 0;  // Radians per second / 100.  So 6.28 *100 = 628 = 1 full turn per second 
	int Speed = 0;

	Speed = (m_Speed * KOBUKI_SPEED_MAX) / SPEED_FULL_FWD;
	if( m_Turn != 0)
	{
		Turn = (m_Turn * (-600)) / TURN_RIGHT_MAX; //800
			// (TURN_RIGHT_MAX / m_Turn); // TODO - Tune this!     //  KOBUKI_SPEED_MAX_MM_PER_SEC) / TURN_RIGHT_MAX;
	}

	ROBOT_LOG( TRUE, "*********> Kobuki SPEED, RADIUS = %d, %d\n", Speed, Turn )


	// Send command to the Kobuki control app via shared memory
	if( (NULL != g_pKobukiCommandSharedMemory) && (NULL != g_hKobukiCommandEvent) )
	{
		m_KobukiCommand.Speed = Speed;
		m_KobukiCommand.Turn = Turn;
		m_KobukiCommand.Acceleration =  m_Acceleration;

		CopyMemory((PVOID)g_pKobukiCommandSharedMemory, &m_KobukiCommand, (sizeof(KOBUKI_COMMAND_T)));
		SetEvent( g_hKobukiCommandEvent );  // Tell Kobuki app that a new command is pending
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR: Cant send command to Kobuki!  Did you have AUTO_LAUNCH_KOBUKI_APP enabled?\n" )
	}

}

void CKobukiControl::ShutDown()
{
	// Send command to the Kobuki control app via shared memory
	if( (NULL != g_pKobukiCommandSharedMemory) && (NULL != g_hKobukiCommandEvent) )
	{
		m_KobukiCommand.bShutDown = true;
		CopyMemory((PVOID)g_pKobukiCommandSharedMemory, &m_KobukiCommand, (sizeof(KOBUKI_COMMAND_T)));
		SetEvent( g_hKobukiCommandEvent );  // Tell Kobuki app that a new command is pending
		ROBOT_LOG( TRUE, "Sending message to KobukiControl app - Shut Down!\n" )
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR: Cant send command to Kobuki!  Did you have AUTO_LAUNCH_KOBUKI_APP enabled?\n" )
	}

}

///////////////////////////////////////////////////////////////////////////////

void CKobukiControl::GetStatus()
{
#if 0

// Get current Status from the Kobuki controller
	CString MsgString;
#define DEBUG_Kobuki 0

#if DEBUG_Kobuki == 1
	ROBOT_LOG( TRUE,  "------------------- STATUS UPDATE --------------\n" )
#endif
	////////////////////////////////////////////////////////
	// Speed / Tach
	////////////////////////////////////////////////////////
	m_KobukiCommand.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_KobukiCommand.Axis = 0;
	m_KobukiCommand.Code = GetCommandedVelocity;		// see how fast the motor is ACTUALLY going
	if( SendReadCmd(4) )
	{
#if DEBUG_Kobuki == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_KobukiReply.Data[0], m_KobukiReply.Data[1], m_KobukiReply.Data[2], m_KobukiReply.Data[3] )
#endif
	}
	long WheelSpeedL = htonl( *(long*)( &m_KobukiReply.Data[0] ) );
	WheelSpeedL = (~WheelSpeedL) + 1;	// Wheel pointing backward, negate the value

	m_KobukiCommand.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_KobukiCommand.Axis = 0;
	m_KobukiCommand.Code = GetCommandedVelocity;	
	if( SendReadCmd(4) )
	{
#if DEBUG_Kobuki == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_KobukiReply.Data[0], m_KobukiReply.Data[1], m_KobukiReply.Data[2], m_KobukiReply.Data[3] )
#endif
	}
	long WheelSpeedR = htonl( *(long*)( &m_KobukiReply.Data[0] ) );

	if( (WheelSpeedR != 0) || (WheelSpeedR != 0) )
	{
#if DEBUG_Kobuki == 1
		MsgString.Format( "Kobuki Speed: LEFT_MOTOR: %ld, RIGHT_MOTOR: %ld",
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
	m_KobukiCommand.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_KobukiCommand.Axis = 0;
	m_KobukiCommand.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_Kobuki == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_KobukiReply.Data[0], m_KobukiReply.Data[1], m_KobukiReply.Data[2], m_KobukiReply.Data[3] )
#endif
	}
	long WheelDistanceL = htonl( *(long*)( &m_KobukiReply.Data[0] ) );
	WheelDistanceL = (~WheelDistanceL) + 1;	// Wheel opposite, negate the value

	m_KobukiCommand.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_KobukiCommand.Axis = 0;
	m_KobukiCommand.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_Kobuki == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_KobukiReply.Data[0], m_KobukiReply.Data[1], m_KobukiReply.Data[2], m_KobukiReply.Data[3] )
#endif
	}
	long WheelDistanceR = htonl( *(long*)( &m_KobukiReply.Data[0] ) );

	if( (WheelDistanceR != 0) || (WheelDistanceL != 0) )
	{
//		ROBOT_LOG( TRUE,  "MotorControl: Kobuki Distance Moved (Inches): LEFT_MOTOR: %0.2f  RIGHT_MOTOR:%0.2f",
//			WheelDistanceL/TICKS_PER_TENTH_INCH, WheelDistanceR/TICKS_PER_TENTH_INCH )

		// Each wheel is sent, so turns can be computed
		// We assume that WPARAM and LPARAM are both 32 bits (see ROBOT_ASSERT in SetupView.cpp)

		// Update the Robot Control thread with the new distance traveled (in "ticks")
		// this in turn will update the GUI with Odometer and current location
		PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_ER1_ODOMETER_READY),
			WheelDistanceL, WheelDistanceR );

	}
#if DEBUG_Kobuki == 1
	ROBOT_LOG( TRUE,  "------------------------------------------------\n" )
#endif

#endif
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SendCmd
// Send command to Kobuki base.
// Format of data stream:
//   Two header bytes 0xAA, 0x55
//   Length of payload
//   Payload, pointed to by m_pCmdMsg
//   Checksum; = XOR of every byte except headers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CKobukiControl::SendCmd( int nCmdBytes )
{
	CString MsgString;
	DWORD dwBytesWritten=0;
	unsigned int nTotalBytes = nCmdBytes +  4; // number of bytes in full message, including headers, etc.
	int CheckSumBytes = 0;

	m_pCmdBuf[0] = 0xAA;		// Header
	m_pCmdBuf[1] = 0x55;		// Header
	m_pCmdBuf[2] = nCmdBytes;	// size of payload

	for( int i=0; i<nCmdBytes; i++ )
	{
		m_pCmdBuf[i+3] = m_pCmdMsg[i];
	}


	for(int i=0; i<5; i++ )
	{
		ROBOT_LOG( TRUE, "Kobuki Test %d...\n\n", i )
		if(0 == i)
		{
			m_pCmdBuf[0] =  0xAA;			// Header
			m_pCmdBuf[1] =  0x55;			// Header
			m_pCmdBuf[2] = 	0x04;			// Payload size
			m_pCmdBuf[3] =	0x0C;			// Payload ID
			m_pCmdBuf[4] =	0x02;			// Data Bytes
			m_pCmdBuf[5] =	0x04;			// Data - Red LED2
			m_pCmdBuf[6] =	0x00;			// Data

			byte Checksum = 0;
			for( int i = 2; i < 7; i++) // 
			{
				Checksum ^= m_pCmdBuf[i];
			}
			m_pCmdBuf[7] = Checksum;
			nTotalBytes = 8;
		}
		else if(1 == i)
		{
			m_pCmdBuf[0] =  0xAA;			// Header
			m_pCmdBuf[1] =  0x55;			// Header
			m_pCmdBuf[2] = 	0x05;			// Payload size  include checksum?
			m_pCmdBuf[3] =	0x0C;			// Payload ID
			m_pCmdBuf[4] =	0x02;			// Data Bytes
			m_pCmdBuf[5] =	0x04;			// Data - Red LED2
			m_pCmdBuf[6] =	0x00;			// Data

			byte Checksum = 0;
			for( int i = 2; i < 7; i++) // 
			{
				Checksum ^= m_pCmdBuf[i];
			}
			m_pCmdBuf[7] = Checksum;
			nTotalBytes = 8;

		}
		else if(2 == i)
		{
			m_pCmdBuf[0] =  0xAA;			// Header
			m_pCmdBuf[1] =  0x55;			// Header
			m_pCmdBuf[2] = 	0x04;			// Payload size
			m_pCmdBuf[3] =	0x0C;			// Payload ID
			m_pCmdBuf[4] =	0x02;			// Data Bytes
			m_pCmdBuf[5] =	0x05;			// Data
			m_pCmdBuf[6] =	0x05;			// Data

			byte Checksum = 0;
			for( int i = 3; i < 7; i++) // ** dont include size
			{
				Checksum ^= m_pCmdBuf[i];
			}
			m_pCmdBuf[7] = Checksum;
			nTotalBytes = 8;

		}
		else if(3 == i)
		{
			m_pCmdBuf[0] =  0xAA;			// Header
			m_pCmdBuf[1] =  0x55;			// Header
			m_pCmdBuf[2] = 	0x04;			// Payload size
			m_pCmdBuf[3] =	0x0C;			// Payload ID
			m_pCmdBuf[4] =	0x05;			// Data  ** no data byte size
			m_pCmdBuf[5] =	0x05;			// Data

			byte Checksum = 0;
			for( int i = 2; i < 6; i++) // 
			{
				Checksum ^= m_pCmdBuf[i];
			}
			m_pCmdBuf[6] = Checksum;
			nTotalBytes = 7;

		}
		else if(4 == i)
		{
			m_pCmdBuf[0] =  0xAA;			// Header
			m_pCmdBuf[1] =  0x55;			// Header
			m_pCmdBuf[2] = 	0x03;			// Payload size **
			m_pCmdBuf[3] =	0x0C;			// Payload ID
			m_pCmdBuf[4] =	0x05;			// Data ** no data byte size
			m_pCmdBuf[5] =	0x05;			// Data

			byte Checksum = 0;
			for( int i = 2; i < 6; i++) // 
			{
				Checksum ^= m_pCmdBuf[i];
			}
			m_pCmdBuf[6] = Checksum;
			nTotalBytes = 7;

		}

/*
BYTE TestBuf[32];

		TRACE( "Kobuki CMD DUMP (%d Bytes):  ", nTotalBytes ) ;
		for(int q=0; q<nTotalBytes; q++)
		{
			TestBuf[q] = m_pCmdBuf[( nTotalBytes - (q+1) )];
			TRACE( "%02X ",	TestBuf[q] );
		}
*/
		// now, add the Checksum for all bytes from Length --> end of Payload (nCmdBytes+1 in length)
/*
		byte Checksum = 0;
		for( int i = 2; i < (CheckSumBytes); i++) // 
		{
			Checksum ^= m_pCmdBuf[i];
		}
		m_pCmdBuf[CheckSumBytes+2] = Checksum;
		*/

	//	#ifdef DEBUG_KOBUKI_COMMAND_DUMP

			TRACE( "Kobuki CMD DUMP (%d Data Bytes): %02X %02X : len:%02X : ", 
				nCmdBytes, m_pCmdBuf[0], m_pCmdBuf[1], m_pCmdBuf[2] );
			for( unsigned int q=3; q < nTotalBytes; q++ )
			{
				TRACE( "%02X ",	m_pCmdBuf[q] );
			}
			TRACE( "\n" );
	
	//	#endif

		if( 1 == ROBOT_SIMULATION_MODE )
		{
			RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainMotorThread);
			ROBOT_LOG( TRUE, "\n" )
			return;
		}

				

		if(!WriteFile(g_hMotorCommPort,	// where to write to (the open comm port)
				&m_pCmdBuf,				// what to write
			//	&TestBuf,				// what to write
				nTotalBytes,			// number of bytes to be written to port
				&dwBytesWritten,		// number of bytes that were actually written
				NULL))					// overlapped I/O not supported			
		{
			ROBOT_LOG( TRUE, "\n" )
			ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to Kobuki Controller!\n" )
			return;
		}
		else
		{
			if( dwBytesWritten != (DWORD)nTotalBytes )
			{
				ROBOT_LOG( TRUE, "\n" )
				MsgString.Format( "Kobuki SERIAL ERROR Bytes Written = %d!\n", dwBytesWritten );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				//continue;
			}
		}
		FlushFileBuffers( g_hMotorCommPort );	// force serial write
		ROBOT_LOG( TRUE, "Kobuki WRITE DONE\n\n" )

		Sleep( 1000);//deubg

	}
	
	ROBOT_LOG( TRUE, "LOOP DONE!!! \n")
}

///////////////////////////////////////////////////////////////////////////////
void CKobukiControl::HandleCommand( int  Request, int  Param1, int  Param2 )
{

	ROBOT_LOG( DEBUG_MOTOR_COMMANDS, "Got Command Message %02X ",  Request)

	switch( Request )
	{		
		case HW_GET_STATUS:
		{
			ROBOT_LOG( TRUE, "*************** CKobukiControl::HandleCommand: HW_GET_STATUS not implemented! **********\n" )
			//GetStatus();
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
			m_Acceleration = Param2;
			SetWheelSpeedAndTurn();
			break;
		}
		case HW_SET_TURN:
		{
			// Param1 = New Turn (+/- 127)
			m_Turn = Param1;
			//m_Acceleration = Param2;  no Acceleration for turns
			SetWheelSpeedAndTurn();
			break;
		}
/*
		case HW_SET_SPEED:
		{
			// Param1 = New Speed (+/- 127)
			m_Speed = (signed int)LOWORD(Param1);
			m_Acceleration = HIWORD(Param1);
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
			m_Speed = (signed short)LOWORD(Param1);
			m_Acceleration = (unsigned short)HIWORD(Param1);
			m_Turn = (signed short)Param2;
			ROBOT_LOG( TRUE, "MotorControlKobuki: HW_SET_SPEED_AND_TURN: Speed = %d, Accel = %d, Turn = %d\n", m_Speed, m_Acceleration, m_Turn )

			SetWheelSpeedAndTurn();
			break;
		}

		//////////////////////////////////////////////
		// Non-Motor commands to the Kobuki Base
		
		case HW_SET_SERVO_POWER:
		{
			g_DynaPowerEnabled = (BOOL)Param2;
			ROBOT_LOG( TRUE, "MotorControlKobuki: DynaPower = %d\n", g_DynaPowerEnabled )
			SetExternalPower();	// Sets all output ports at once
			break;
		}

		case HW_SET_KINECT_POWER:
		{
			g_KinectPowerEnabled = (BOOL)Param2;
			// m_DynaPower = (BOOL)Param2;	// for now, turn on at the same time?
			ROBOT_LOG( TRUE, "MotorControlKobuki: KinectPower = %d\n", g_KinectPowerEnabled )
			SetExternalPower();	// Sets all output ports at once
			break;
		}

		case HW_SET_AUX_LIGHT_POWER:
		{
			m_LEDPower = (BOOL)Param2;
			ROBOT_LOG( TRUE, "MotorControlKobuki: Aux Lights = %d\n", m_LEDPower )
			SetExternalPower();	// Sets all output ports at once
			break;
		}

		//////////////////////////////////////////////

		default:
		{
			CString StrText;
			StrText.Format( "ERROR! Kobuki Handle Command: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, StrText )
		}

	}

}


#endif // ROBOT_SERVER  // Nothing in this file used for client!
