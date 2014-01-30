// MotorControliRobot.cpp
// Interface for the iRobot Create Base
/*
	iRobot Create startup info: 
		Default Baud: 57600
		When first powered up, iRobot Create starts in “Off” Mode.

	Modes: 
		Off: Initial mode at power-up.  Waits for a  “Start” command. 
		Passive:  Can received sensor data only, will ignore commands
		Safe: Full control, except when danger: Cliff, Wheel Drop, or Docked to Charger
		When any of these conditions, Create will revert to Passive mode.
		Full:  Complete control.  Cliff sensors, etc. are up to the PC to handle

	Notes:
		Charging:  Charging terminates when entering Safe or Full mode!
		Buttons:  Play and Advance buttons are disabled when in Safe or Full mode.
		Commands do not time out.  Once a command starts, Create will wait for the rest of the command.

		“At a baud rate of 115200, there must be at least 200µs between the onset of each character, or some
		characters may not be received.”
*/

// This class is used by the thread loop that reads commands from the queue in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include "Globals.h"
#include "MotorControl.h"
#include "../Common/HardwareCmds.h" // for TICS_PER_INCH

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_IROBOT_COMMAND_DUMP   // Dump all iRobot commands
#define DEBUG_IROBOT_SHOW_ACK_MESSAGES
#define IROBOT_DUMP_RECEIVED_BYTES 0

/////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iRobot Motor Control Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CiRobotControl::CiRobotControl()
{
	//m_pReadBuf[IROBOT_READ_BUF_SIZE]
	//m_pCmdBuf = new BYTE[IROBOT_CMD_BUF_SIZE];
	memset(&m_pCmdBuf,0,IROBOT_CMD_BUF_SIZE);
	m_StartCmdSent = FALSE;
	m_Speed = 0;
	m_Turn = 0;
	m_LeftWheelStopped = TRUE;
	m_RightWheelStopped = TRUE;
	m_Docked = FALSE;	// TODO-TURTLE change this!
	m_LEDPower = 0;

	g_MotorSubSystemStatus = SUBSYSTEM_WAITING;

	ROBOT_LOG( TRUE, "==============================>>>>>>> CONSTRUCTOR CALLED <<<<<<<<<<<==========================" )

}

CiRobotControl::~CiRobotControl()
{
	// Power down during the destructor
	g_DynaPowerEnabled = FALSE;
	g_KinectPowerEnabled = FALSE;
	m_LEDPower = FALSE;
	SetPort();	// Sets all output ports at once
}


///////////////////////////////////////////////////////////////////////////////
// Commands
///////////////////////////////////////////////////////////////////////////////
#define IROBOT_SPEED_MAX_MM_PER_SEC		500	// 500 mm/s max

// Commands from "Create Open Interface v2.pdf"
#define IROBOT_START_CMD					128	// Must always be the first command send after power on
#define IROBOT_SAFE_MODE_CMD				131 // Cliff sensors etc. override PC
#define IROBOT_FULL_MODE_CMD				132	// PC has full control
#define IROBOT_MOTOR_DRIVE_INDIRECT_CMD		137	// Use speend and turn radius to drive the wheel motors (not used)
#define IROBOT_MOTOR_DRIVE_DIRECT_CMD		145	// Direct Drive the wheel motors
#define IROBOT_SET_IO_CMD					147 // Control state of I/O port bits 0, 1, 2
#define IROBOT_START_SENSOR_STREAM_CMD		148 // Start streaming all sensor data every 15ms

enum IROBOT_MODES { 
		IROBOT_MODE_PASSIVE = 0, // Allows charging
		IROBOT_MODE_SAFE,
		IROBOT_MODE_FULL,
};


void CiRobotControl::SendStartCmd()
{
	m_pCmdBuf[0] = 0x80; // IROBOT_START_CMD;
	int  nBytesToSend = 1;
	SendCmd( nBytesToSend );
	RobotSleep(100, pDomainMotorThread);
	m_StartCmdSent = TRUE;
}

void CiRobotControl::StartSensorStreamCmd()
{
	// Tells iRobot to send a continuous stream of sensor data (every 15ms)
	// Should we use "Group 0", the most common / interesting sensors or
	// Should we use "Group 6", all the sensors. ?

	if( (!m_StartCmdSent) )
	{
		// iRobot has not yet received the Start command
		SendStartCmd();
	}

	m_pCmdBuf[0] = IROBOT_START_SENSOR_STREAM_CMD;
	m_pCmdBuf[1] = 1; // number of packet groups requested
	m_pCmdBuf[2] = 6; // Requested Group - (unsigned char)
	int  nBytesToSend = 3;
	SendCmd( nBytesToSend );
	RobotSleep(100, pDomainMotorThread);
}

void CiRobotControl::SetMode( int mode )
{
	// Set to one of: Off, Passive, Safe, Full
	// Note: must be in Passive Mode to Charge Batteries!

	if( IROBOT_MODE_PASSIVE == mode )
	{
		// Just resend the start command to put iRobot back into passive mode (which allows charging)
		SendStartCmd();
	}
	else
	{
		if( (!m_StartCmdSent) )
		{
			// iRobot has not yet received the Start command
			SendStartCmd();
		}

		if( IROBOT_MODE_SAFE == mode )
		{
			m_pCmdBuf[0] = IROBOT_SAFE_MODE_CMD;
			int  nBytesToSend = 1;
			SendCmd( nBytesToSend );
		}
		else if( IROBOT_MODE_FULL == mode )
		{
			m_pCmdBuf[0] = IROBOT_FULL_MODE_CMD;
			int  nBytesToSend = 1;
			SendCmd( nBytesToSend );
		}
		else
		{
			ROBOT_ASSERT(0); // bad mode
		}
	}

	ROBOT_LOG( TRUE, "*********> IROBOT MODE = %d\n", mode )
}

void CiRobotControl::Init()
{
	// Init at robot startup
	ROBOT_LOG( TRUE, "==============================>>>>>>> INIT CALLED <<<<<<<<<<<==========================" )

	#if MOTOR_CONTROL_TYPE != IROBOT_MOTOR_CONTROL
		ROBOT_ASSERT(0);
	#endif

	StartSensorStreamCmd();
	SetPort(); // Initialize power for Kinect and Dynamixel Servos

	////////////////////////////////////////////////////////////////////////////
	// Launch the C# Kinect Capture Application here for Turtle
	////////////////////////////////////////////////////////////////////////////
	#if (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) 
		RobotSleep(2000, pDomainMotorThread); // allow kinect to power up
		LaunchKinectApp(); // defined in globals.cpp
	#endif

	///g_MotorControlInitialized = TRUE;
	ROBOT_LOG( TRUE, "MotorControliRobot: Init Complete\n" )
}

void CiRobotControl::UnDock()
{
	// Undock from charging base and spin in place, ready for next command
	SetMode( IROBOT_MODE_FULL );
	// TODO-TURTLE

}

void CiRobotControl::Dock()
{
	// Search for charging base and dock
	// TODO-TURTLE

}


void CiRobotControl::SetPort()
{
	// Enable or disable each port.  Must be in Full or Safe mode for this to work!
	SetMode( IROBOT_MODE_FULL );
	
	// Tracks state of Port 0,1,2, and masks bits as needed 
	BYTE PortCmd = 0;

	if( g_KinectPowerEnabled )
	{
		PortCmd |= 0x01;	// Turn on the Kinect Power bit
	}

	if( g_DynaPowerEnabled )
	{
		PortCmd |= 0x02;	// Turn on the Dynamixel Servo Power bit
	}

	if( m_LEDPower )
	{
		PortCmd |= 0x04;	// Turn on the LED lights bit
	}

	ROBOT_LOG( TRUE, "Sending IO Port command to iRobot Base: %02X hex\n", PortCmd )
	m_pCmdBuf[0] = IROBOT_SET_IO_CMD;
	m_pCmdBuf[1] = PortCmd; // Set all 3 I/O Port bits simultaneously
	int  nBytesToSend = 2;
	SendCmd( nBytesToSend );

}

void CiRobotControl::SendMotorCmd( int MotorSpeedL, int MotorSpeedR )
{
	// Convert from standard +/- 127 values to iRobot values in mm per second
	// mm/s = 500 * value/127

	int Command = IROBOT_MOTOR_DRIVE_DIRECT_CMD;
	int SpeedL = (MotorSpeedL * IROBOT_SPEED_MAX_MM_PER_SEC) / SPEED_FULL_FWD;
	int SpeedR = (MotorSpeedR * IROBOT_SPEED_MAX_MM_PER_SEC) / SPEED_FULL_FWD;


	ROBOT_LOG( TRUE, "*********> IROBOT SPEED = %d, %d\n", SpeedL,SpeedR )

// TODO!!! DETERMINE WHICH OF THESE WORK BEST!!!!
	/**
	IROBOT_MOTOR_CMD_T MotorCmd;
	MotorCmd.Cmd = IROBOT_MOTOR_DRIVE_DIRECT_CMD;
	MotorCmd.MotorSpeedL = SpeedL;
	MotorCmd.MotorSpeedR = SpeedR;
	**/

	m_pCmdBuf[0] = Command;
	m_pCmdBuf[1] = HIBYTE(SpeedR);
	m_pCmdBuf[2] = LOBYTE(SpeedR);
	m_pCmdBuf[3] = HIBYTE(SpeedL);
	m_pCmdBuf[4] = LOBYTE(SpeedL);
	int  nBytesToSend = 5;

	SendCmd( nBytesToSend );
}

///////////////////////////////////////////////////////////////////////////////
void CiRobotControl::SetWheelSpeedAndTurn()
{

	// "Mixes" Speed and Turn to set speed of the motors
	// since the wheels are oposite each other (one mounted backward)
	// We drive one "forward" and one "reverse" to make the robot go forward!
	// TODO if making a class: m_Speed and m_Turn are set prior to calling this function

	int LeftMotorSpeed = m_Speed + m_Turn;
	int RightMotorSpeed = m_Speed - m_Turn;
//		LeftMotorSpeed = ~LeftMotorSpeed +1;	// invert left motor


	if( RightMotorSpeed > 127 )
	{
		RightMotorSpeed = 127;
	}
	if( RightMotorSpeed < -127 )
	{
		RightMotorSpeed = -127;
	}
	if( LeftMotorSpeed > 127 )
	{
		LeftMotorSpeed = 127;
	}
	if( LeftMotorSpeed < -127 )
	{
		LeftMotorSpeed = -127;
	}

	if( 0 != LeftMotorSpeed )
	{
		m_LeftWheelStopped = FALSE;
	}
	if( 0 != RightMotorSpeed )
	{
		m_RightWheelStopped = FALSE;
	}

	SendMotorCmd( LeftMotorSpeed, RightMotorSpeed );
}



///////////////////////////////////////////////////////////////////////////////
/***	
void CiRobotControl::GetStatus()
{
	// Get current Status from the iRobot controller
	CString MsgString;
#define DEBUG_IROBOT 0

#if DEBUG_IROBOT == 1
	ROBOT_LOG( TRUE,  "------------------- STATUS UPDATE --------------\n" )
#endif
	////////////////////////////////////////////////////////
	// Speed / Tach
	////////////////////////////////////////////////////////
	m_iRobotCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_iRobotCmd.Axis = 0;
	m_iRobotCmd.Code = GetCommandedVelocity;		// see how fast the motor is ACTUALLY going
	if( SendReadCmd(4) )
	{
#if DEBUG_IROBOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_iRobotReply.Data[0], m_iRobotReply.Data[1], m_iRobotReply.Data[2], m_iRobotReply.Data[3] )
#endif
	}
	long WheelSpeedL = htonl( *(long*)( &m_iRobotReply.Data[0] ) );
	WheelSpeedL = (~WheelSpeedL) + 1;	// Wheel pointing backward, negate the value

	m_iRobotCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_iRobotCmd.Axis = 0;
	m_iRobotCmd.Code = GetCommandedVelocity;	
	if( SendReadCmd(4) )
	{
#if DEBUG_IROBOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedVelocity RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Speed = %02X%02X %02X%02X \n", 
			m_iRobotReply.Data[0], m_iRobotReply.Data[1], m_iRobotReply.Data[2], m_iRobotReply.Data[3] )
#endif
	}
	long WheelSpeedR = htonl( *(long*)( &m_iRobotReply.Data[0] ) );

	if( (WheelSpeedR != 0) || (WheelSpeedR != 0) )
	{
#if DEBUG_IROBOT == 1
		MsgString.Format( "iRobot Speed: LEFT_MOTOR: %ld, RIGHT_MOTOR: %ld",
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
	m_iRobotCmd.Address = LEFT_MOTOR;	// Wheel 0 or 1
	m_iRobotCmd.Axis = 0;
	m_iRobotCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_IROBOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition LEFT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW LEFT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_iRobotReply.Data[0], m_iRobotReply.Data[1], m_iRobotReply.Data[2], m_iRobotReply.Data[3] )
#endif
	}
	long WheelDistanceL = htonl( *(long*)( &m_iRobotReply.Data[0] ) );
	WheelDistanceL = (~WheelDistanceL) + 1;	// Wheel opposite, negate the value

	m_iRobotCmd.Address = RIGHT_MOTOR;	// Wheel 0 or 1
	m_iRobotCmd.Axis = 0;
	m_iRobotCmd.Code = GetCommandedPosition;		// for tracking distance moved
	if( SendReadCmd(4) )
	{
#if DEBUG_IROBOT == 1
		ROBOT_LOG( TRUE,  "GetCommandedPosition RIGHT_MOTOR read OK\n" )
		ROBOT_LOG( TRUE,  "----------> RAW RIGHT_MOTOR Distance = %02X%02X %02X%02X \n", 
			m_iRobotReply.Data[0], m_iRobotReply.Data[1], m_iRobotReply.Data[2], m_iRobotReply.Data[3] )
#endif
	}
	long WheelDistanceR = htonl( *(long*)( &m_iRobotReply.Data[0] ) );

	if( (WheelDistanceR != 0) || (WheelDistanceL != 0) )
	{
//		ROBOT_LOG( TRUE,  "MotorControl: iRobot Distance Moved (Inches): LEFT_MOTOR: %0.2f  RIGHT_MOTOR:%0.2f",
//			WheelDistanceL/TICKS_PER_TENTH_INCH, WheelDistanceR/TICKS_PER_TENTH_INCH )

		// Each wheel is sent, so turns can be computed
		// We assume that WPARAM and LPARAM are both 32 bits (see ROBOT_ASSERT in SetupView.cpp)

		// Update the Robot Control thread with the new distance traveled (in "ticks")
		// this in turn will update the GUI with Odometer and current location
		PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_ER1_ODOMETER_READY),
			WheelDistanceL, WheelDistanceR );

	}
#if DEBUG_IROBOT == 1
	ROBOT_LOG( TRUE,  "------------------------------------------------\n" )
#endif
}

***/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SendCmd
// Send command to iRobot base.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CiRobotControl::SendCmd( int nCmdBytes )
{
	CString MsgString;
	DWORD dwBytesWritten=0;

//	#ifdef DEBUG_IROBOT_COMMAND_DUMP

		ROBOT_LOG( TRUE, "IROBOT CMD DUMP (%d Data Bytes): ", nCmdBytes )
		for( int q=0; q < nCmdBytes; q++ )
		{
			ROBOT_LOG( TRUE, "%02X ",	m_pCmdBuf[q] )
		}
		//ROBOT_LOG( TRUE, "\n" )
//	#endif

	if( 1 == ROBOT_SIMULATION_MODE )
	{
		RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainMotorThread);
		ROBOT_LOG( TRUE, "\n" )
		return;
	}

	if(!WriteFile(g_hMotorCommPort,	// where to write to (the open comm port)
			&m_pCmdBuf,				// what to write
			nCmdBytes,				// number of bytes to be written to port
			&dwBytesWritten,		// number of bytes that were actually written
			NULL))					// overlapped I/O not supported			
	{
		ROBOT_LOG( TRUE, "\n" )
		ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to iRobot Controller!\n" )
		return;
	}
	else
	{
		if( dwBytesWritten != (DWORD)nCmdBytes )
		{
			ROBOT_LOG( TRUE, "\n" )
			MsgString.Format( "IROBOT SERIAL ERROR Bytes Written = %d!\n", dwBytesWritten );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			//continue;
		}
	}
	FlushFileBuffers( g_hMotorCommPort );	// force serial write
	ROBOT_LOG( TRUE, "IROBOT WRITE DONE\n\n" )

}

///////////////////////////////////////////////////////////////////////////////
void CiRobotControl::HandleCommand( int  Request, int  Param1, int  Param2 )
{

	switch( Request )
	{		
		case HW_GET_STATUS:
		{
			ROBOT_LOG( TRUE, "*************** CiRobotControl::HandleCommand: HW_GET_STATUS not implemented! **********\n" )
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
			m_Turn = (signed short)Param2;
			SetWheelSpeedAndTurn();
			break;
		}

		case HW_IROBOT_UNDOCK:
		{
			UnDock();	
			break;
		}

		case HW_IROBOT_DOCK:
		{
			Dock();
			break;
		}

		//////////////////////////////////////////////
		// Non-Motor commands to the iRobot Base
		
		case HW_SET_SERVO_POWER:
		{
			g_DynaPowerEnabled = (BOOL)Param2;
			ROBOT_LOG( TRUE, "MotorControliRobot: DynaPower = %d\n", g_DynaPowerEnabled )
			SetPort();	// Sets all output ports at once
			break;
		}

		case HW_SET_KINECT_POWER:
		{
			g_KinectPowerEnabled = (BOOL)Param2;
			// m_DynaPower = (BOOL)Param2;	// for now, turn on at the same time?
			ROBOT_LOG( TRUE, "MotorControliRobot: KinectPower = %d\n", g_KinectPowerEnabled )
			SetPort();	// Sets all output ports at once
			break;
		}

		case HW_SET_AUX_LIGHT_POWER:
		{
			m_LEDPower = (BOOL)Param2;
			ROBOT_LOG( TRUE, "MotorControliRobot: Aux Lights = %d\n", m_LEDPower )
			SetPort();	// Sets all output ports at once
			break;
		}

		//////////////////////////////////////////////

		default:
		{
			CString StrText;
			StrText.Format( "ERROR! iRobot Handle Command: Unknown Cmd:%02X \n", Request);
			ROBOT_DISPLAY( TRUE, StrText )
		}

	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iRobot Status Parser Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define START_BYTE				19
#define ASCII_CARRIAGE_RTN		13
#define ASCII_LINE_FEED			10
#define ASCII_SPACE				32
#define ASCII_Z				   122
#define FULL_SENSOR_PACKET_SIZE		53 // The only thing we ever ask for is the full sensor packet
#define CHECKSUM_OFFSET		(FULL_SENSOR_PACKET_SIZE+2)

CiRobotParser::CiRobotParser()
{
	//m_pReadBuf[IROBOT_READ_BUF_SIZE]
}

CiRobotParser::~CiRobotParser()
{
//	SAFE_DELETE(m_pCmdBuf);

}

BOOL CiRobotParser::ParseBuffer( char* iRobotSIOBuf, int dwSIOBytesReceived )
{
	// Returns TRUE if sucessful communication with the iRobot Create base established

	// First, check to see if we are getting ASCII debug messages
	// These happen upon startup and battery charge, until the Start Code is sent
	BOOL AsciiMsg = TRUE;
	for( int i=0; i<dwSIOBytesReceived; i++ )
	{
		if(  ((iRobotSIOBuf[i] < ASCII_SPACE) && (iRobotSIOBuf[i] != ASCII_LINE_FEED) &&  (iRobotSIOBuf[i] != ASCII_CARRIAGE_RTN))  || (iRobotSIOBuf[i] > ASCII_Z) )
		{
			AsciiMsg = FALSE;
			break;
		}
	}
	if( AsciiMsg )
	{
		iRobotSIOBuf[dwSIOBytesReceived] = 0;
		ROBOT_LOG( TRUE, "iRobot Msg: %s", iRobotSIOBuf )
		return 0;
	}

	// NON ASCII, so should be a Sensor message.
	// We only subscribe to a singel message type, Message #6, which sends ALL sensor data every 15ms.
	// Since we don't need it that frequently (and there is no way to tell Create to send less often), we just
	// get the latest sample, and ignore extra samples

	ROBOT_LOG( IROBOT_DUMP_RECEIVED_BYTES, "iRobot BINARY Msg (%d bytes)\n", dwSIOBytesReceived )
	
	// Start at the back of the buffer (most recent bytes), and look for Start message and Size byte.  
	// These two are unique enough to identify the packet (usually), and confirmed by the Checksum
	int index = 0;
	BOOL bOkSoFar = FALSE;
	for( index = dwSIOBytesReceived-(FULL_SENSOR_PACKET_SIZE+2); index >= 0; index-- ) // Start at first point that a full packet might be
	{
		if( START_BYTE == iRobotSIOBuf[index] )
		{
			// found start byte. Check size byte.
			if( FULL_SENSOR_PACKET_SIZE == iRobotSIOBuf[index+1] )
			{
				bOkSoFar = TRUE;
				break;
			}
		}
	}
	if( !bOkSoFar )
	{
		// No start of message found anywhere in the buffer. clear it out and return
		//// TODO-MUST restore this:  ROBOT_LOG( TRUE, "iRobot Status Parse - No Start Found\n" )
		return 0;
	}

	// Check Checksum
	int CheckSumLocation = index + CHECKSUM_OFFSET;
	BYTE ReportedCheckSum = (BYTE)iRobotSIOBuf[CheckSumLocation];
	ROBOT_LOG( IROBOT_DUMP_RECEIVED_BYTES, "iRobot BINARY DEBUG (%d bytes):", (CheckSumLocation - index) )
	WORD Sum = 0;
	int SumIndex = 0;
	for( SumIndex = index; SumIndex < CheckSumLocation; SumIndex++ )
	{
		Sum += (BYTE)iRobotSIOBuf[SumIndex];
		#if( IROBOT_DUMP_RECEIVED_BYTES == 1 )
			TRACE( " %02X", (BYTE)(iRobotSIOBuf[SumIndex]) );
		#endif
	}
	#if( IROBOT_DUMP_RECEIVED_BYTES == 1 )
		TRACE( "\n" );
	#endif

	BYTE CalculatedCheckSum = Sum + ReportedCheckSum;
	BYTE CheckSumResult = CalculatedCheckSum & 0xFF;
	if( 0 != CheckSumResult )
	{
		//TODO-MUST - why does this happen a lot?: ROBOT_LOG( TRUE, "iRobot Status Parse - Bad Checksum!\n")
		return 0;
	}
	//ROBOT_LOG( TRUE, "iRobot Status Parse - GOOD Checksum\n" )


	// OK, good checksum, update global with values found
	index += 2;
	//ROBOT_LOG( TRUE, "PacketID = %02X\n", (BYTE)(iRobotSIOBuf[index]) )
	index++;

	g_pIRobotStatus->WheelDropCaster = ( (iRobotSIOBuf[index] & 0x10) != 0 ? 1 : 0 );
	g_pIRobotStatus->WheelDropLeft = ( (iRobotSIOBuf[index] & 0x08) != 0 ? 1 : 0 );
	g_pIRobotStatus->WheelDropRight = ( (iRobotSIOBuf[index] & 0x04) != 0 ? 1 : 0 );
	g_pIRobotStatus->BumperLeft = ( (iRobotSIOBuf[index] & 0x02) != 0 ? 1 : 0 );
	g_pIRobotStatus->BumperRight = ( (iRobotSIOBuf[index] & 0x01) != 0 ? 1 : 0 );
	//ROBOT_LOG( TRUE, "WDrop+Bmpr = %02X\n", (BYTE)(iRobotSIOBuf[index]) )
	index++;

	g_pIRobotStatus->WallDetected = iRobotSIOBuf[index++];
	g_pIRobotStatus->CliffLeft = iRobotSIOBuf[index++];
	g_pIRobotStatus->CliffFrontLeft = iRobotSIOBuf[index++];
	g_pIRobotStatus->CliffFrontRight = iRobotSIOBuf[index++];
	g_pIRobotStatus->CliffRight = iRobotSIOBuf[index++];
	g_pIRobotStatus->VirtualWall = iRobotSIOBuf[index++];
	g_pIRobotStatus->OverCurrent = iRobotSIOBuf[index++] & 0x1F;	// Actually 4 separate bits, but we just care if it's non-zero
	index += 2; // skip unused packets.  On Roomba, these are dirt detector left, right

	g_pIRobotStatus->IRcode = iRobotSIOBuf[index++]; // code received from remote control, beacons, home, etc.
	index++; // skip reading the Create buttons


	int  Distance = 0;
	Distance = iRobotSIOBuf[index++] <<8;
	Distance += iRobotSIOBuf[index++];
	g_pIRobotStatus->Distance = Distance; // in mm, since last request
	// TODO-TURTLE - keep running total here!!!

	int  Angle = 0;
	Angle = iRobotSIOBuf[index++] <<8;
	Angle += iRobotSIOBuf[index++];
	g_pIRobotStatus->Angle = Angle; // in degrees, since last request
	// TODO-TURTLE - keep running total here!!!

	g_pIRobotStatus->ChargingState = iRobotSIOBuf[index++];

	int  Voltage = 0;
	Voltage = iRobotSIOBuf[index++] <<8;
	Voltage += iRobotSIOBuf[index++];
	g_pIRobotStatus->BatteryVoltage = Voltage; // in milivolts

	int  Current = 0;
	Current = iRobotSIOBuf[index++] <<8;
	Current += iRobotSIOBuf[index++];
	g_pIRobotStatus->BatteryCurrent = Current; // in miliamps, negative = discharging (which is normal)

	g_pIRobotStatus->BatteryTemperature = iRobotSIOBuf[index++];

	int  Charge = 0;
	Charge = iRobotSIOBuf[index++] <<8;
	Charge += iRobotSIOBuf[index++];
	g_pIRobotStatus->BatteryCharge = Charge; // in miliamp Hours, negative = discharging (which is normal)

	int  Capacity = 0;
	Capacity = iRobotSIOBuf[index++] <<8;
	Capacity += iRobotSIOBuf[index++];
	g_pIRobotStatus->BatteryCapacity = Capacity; // in miliamp Hours

	index += 10;	// Skip Signal Strength of sensors (packet ID 27-31, 2 bytes each)

	g_pIRobotStatus->ExpansionDigitalInputs = iRobotSIOBuf[index++] & 0x1F; // mask out top 3 bits, not used

	int  Analog = 0;
	Analog = iRobotSIOBuf[index++] <<8;
	Analog += iRobotSIOBuf[index++];
	g_pIRobotStatus->ExpansionAnalogInput = Analog; // Cargo Bay Analog Input, 0=0v, 1023 = 5v

	g_pIRobotStatus->InHomeBase = ( (iRobotSIOBuf[index] &0x02) != 0 ? 1 : 0 );
	g_pIRobotStatus->ChargingPlugInserted = ( (iRobotSIOBuf[index++] &0x01) != 0 ? 1 : 0 );

	g_pIRobotStatus->OIMode = iRobotSIOBuf[index++];
	// 0 = Off, 1=Passive, 2=Safe, 3=Full

	//index += 7;	// Skip feedback info (packet ID 36-42)

	// TODO - Lock and unlock the global status?
	return TRUE; // assume communicaton established

}


#endif // ROBOT_SERVER  // Nothing in this file used for client!
