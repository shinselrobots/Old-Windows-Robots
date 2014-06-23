// Module.cpp
// This version modified for Loki.
// Removed most Seeker/Car specific functionality, such as speed control


#include "stdafx.h"
#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "thread.h"
#include "HardwareConfig.h"
#include "module.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//#define AVOIDANCE_MOVES_DISABLED_FOR_TESTING

// Global to this module (defined as extern in Module.h)
//int					gAutoNavigateMode = MODE_MANUAL_OVERRIDE;

///__itt_string_handle* pshMotorCommand = __itt_string_handle_create("Motor Command, Speed, Turn = ");
__itt_string_handle* pshCase1 = __itt_string_handle_create("1 Case");
__itt_string_handle* pshCase2 = __itt_string_handle_create("2 Case");
__itt_string_handle* pshCase3 = __itt_string_handle_create("3 Case");
__itt_string_handle* pshCase4 = __itt_string_handle_create("4 Case");
__itt_string_handle* pshCase5 = __itt_string_handle_create("5 Case");
__itt_string_handle* pshCase6 = __itt_string_handle_create("6 Case");
__itt_string_handle* pshCase7 = __itt_string_handle_create("7 Case");
__itt_string_handle* pshCase8 = __itt_string_handle_create("8 Case");
__itt_string_handle* pshCase9 = __itt_string_handle_create("9 Case");


/////////////////////////////////////////////////////////////////////////////
// class CSequenceOrder

CSequenceOrder::CSequenceOrder( const int NumberOfItems )
{
	m_NumberOfItems = NumberOfItems;
	m_Items = new int[m_NumberOfItems];
	// create an ordered list
	for( int q=0; q<NumberOfItems; q++ )
		m_Items[q] = q;

	// now randomize it
	DWORD dwTime = GetTickCount();
	std::srand(dwTime); // Seed with random number (current time in ticks).  Only call this once

	#if ( PUBLIC_DEMO == 0) // For public demos, play jokes in order every time
		Shuffle();
	#endif

	m_NextItem = 0;
	//ROBOT_LOG( TRUE,"CSequenceOrder initialized\n")
}

CSequenceOrder::~CSequenceOrder()
{
	// Release resources
	SAFE_DELETE( m_Items );
	//ROBOT_LOG( TRUE,"~CSequenceOrder done\n")
}

int CSequenceOrder::Next()
{
	if( m_NextItem > m_NumberOfItems )
	{
		ROBOT_LOG( TRUE,"Ran out of Items!\n")
		m_NextItem = 0; // next request will start at beginning again
		return -1; // flag so calling routine knows we ran out of items
	}
	return m_Items[m_NextItem++];
}

void CSequenceOrder::Shuffle()
{
	// Shuffle the questions once, so we go through them all

	for( int q=0; q<m_NumberOfItems; q++ )
	{
		m_Items[q] = q;
	}
	std::random_shuffle(m_Items, m_Items + m_NumberOfItems);

	ROBOT_LOG( TRUE,"Shuffled Order: ")
	for( int q=0; q<m_NumberOfItems; q++ )
	{
		TRACE("%d, ", m_Items[q]);
	}
	

}


///////////////////////////////////////////////////////////////////////////////
// Module Base Class
CRobotModule::CRobotModule()
{
	m_pCurrentSegment = NULL;
	m_pCurrentWaypoint = NULL;

}

void CRobotModule::DisplayStatus( LPCTSTR lpszError )
{

	CString strStatus;
		strStatus.Format( "Module: %s" , lpszError );
	ROBOT_DISPLAY( TRUE, (LPCTSTR)strStatus )

	ROBOT_LOG( TRUE, "Module: DisplayStatus: %s\n",lpszError )
}

void CRobotModule::SendHardwareCmd(WORD Request, DWORD wParam, DWORD lParam)
{
	// Send a command to the right control hardware:
	//   ER1 Motor Control, Servo controller, or Arduino

	// See if this is a Motor command
	if( (HW_SET_MOTOR_STOP == Request)		||
		(HW_SET_MOTOR_BRAKE == Request)		||
		(HW_SET_SPEED_AND_TURN == Request)	||
		(HW_SET_TURN == Request)			||
//		(HW_SET_SPEED == Request)			||
		(HW_UPDATE_MOTOR_SPEED_CONTROL == Request)	)	
	{
		// Motor command.  wParam has Speed and Turn packed together.  lParam has other options, such as acceleration

//#if ( (MOTOR_CONTROL_TYPE != IROBOT_MOTOR_CONTROL) && (MOTOR_CONTROL_TYPE != KOBUKI_MOTOR_CONTROL) )
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			// Arduino is not connected, simulate as if it were
			// This allows offline debug, regardless of actual hardware type
			#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL
				// Special case ER1 to allow motor control testing, even with no Arduino!
				if( INVALID_HANDLE_VALUE == g_hMotorCommPort )
				{
					// Arduino is not connected, simulate as if it were
					SimulateHardware( Request, wParam, lParam );
				}
			#else
				// all other hardware configurations
					// Arduino is not connected, simulate as if it were
					SimulateHardware( Request, wParam, lParam );
			#endif
			return;
		}
//#endif
		

		// Not Simulated.  Send to the hardware that is controlling the motors
		#if MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL
		
			// Send the command to the Arduino Comm Write Thread
			PostThreadMessage( g_dwArduinoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			return;

		#elif MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL	

			if( HW_SET_MOTOR_STOP == Request )
			{
				// Send the STOP command.
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
				return;
			}
			else if( HW_SET_MOTOR_BRAKE == Request )
			{
				// Handle the BRAKE command.
				// wParam = How hard to brake
				// lParam = How long to brake
				// Set a timer to time how long to Brake for
				gBrakeTimer = lParam;

				// Send the Brake command to the Servo controller.
				wParam += SPEED_SERVO_CENTER;
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_SPEED), wParam, 0 );
				return;
			}
			else if( HW_SET_SPEED_AND_TURN == Request )
			{
				ROBOT_ASSERT(0); // HW_SET_SPEED_AND_TURN NOT USED!
				// Break into two separate commands:
				// Handle Turn - Convert from Logical Turn to Hardware Turn value
				wParam -= TURN_SERVO_CENTER;	// Turn servo is reverse of Speed servo
				// Send this command to an external servo controller via the ServoComm Write Thread
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_TURN), wParam, 0 );

				// Handle Speed - Convert from Logical Speed to Hardware Speed value
				lParam += SPEED_SERVO_CENTER;
				// Send this command to an external servo controller via the ServoComm Write Thread
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_SPEED), lParam, 0 ); // lParam is correct in this case
			}
			else if( HW_SET_TURN == Request )
			{
				// Handle Turn - Convert from Logical Turn to Hardware Turn value
				wParam -= TURN_SERVO_CENTER;	// Turn servo is reverse of Speed servo
				ROBOT_LOG( MOTOR_DBG, "Execute: New Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )
				// Send this command to an external servo controller via the ServoComm Write Thread
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			}
			else if( HW_SET_SPEED == Request )
			{
				// Handle Speed - Convert from Logical Speed to Hardware Speed value
				wParam += SPEED_SERVO_CENTER;
				// Send this command to an external servo controller via the ServoComm Write Thread
				PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			}
			else
			{
				ROBOT_ASSERT(0); // BAD MOTOR COMMAND
			}
			return;

		#elif MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL

			// Send this command to an ER1 controller via the MotorCommThreadFunc
			if( HW_SET_MOTOR_BRAKE == Request )
			{
				// ER1 does not support the BRAKE command
				PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP), 0, 0 );
			}
			else
			{
				PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			}
			return;

		#elif MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL
			// Send the command to the Motor Comm Write Thread
			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			return;

		///////////////////////////////////////////////////////////////////////////////
		#elif MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL

			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			return;

		///////////////////////////////////////////////////////////////////////////////
		#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL
			ROBOT_LOG( DEBUG_MOTOR_COMMANDS, "KOBUKI_MOTOR_CONTROL: PostThreadMessage %02X ",  Request)
			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			return;

		#else
			#error BAD MOTOR_CONTROL_TYPE
		#endif

		return;
	}  // (if Motor command)


	// See if this is a Camera command
	if( (Request >= HW_CAMERA_MIN_COMMAND) && (Request <= HW_CAMERA_MAX_COMMAND) )
	{
		// Camera command.
		#if CAMERA_CONTROL_TYPE == PIC_SERVO_CONTROL	
			// Send the command to the Arduino Comm Write Thread
			PostThreadMessage( g_dwArduinoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

		#elif CAMERA_CONTROL_TYPE == EXTERN_SERVO_CONTROL	
			// Send this command to an external servo controller via the ServoComm Write Thread
			PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

		#elif CAMERA_CONTROL_TYPE == DYNA_SERVO_CONTROL	
			// Send this command to Dynamixel Servos via the SmartServoCommThreadFunc Thread
			PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

		#elif CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA
			// Send this command to a Sony serial interface camera via the CameraCommThreadFunc
			// NOTE - this will change if using SONY PRO camera!
			PostThreadMessage( g_dwCameraCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
		#else
			#error BAD CAMERA_CONTROL_TYPE
		#endif	

		return;
	}

	// See if this is a Power command
	if( HW_SET_POWER_MODE == Request )
	{
		if( LAPTOP_POWER_MODE_DEFAULT == lParam )
		{
			// Leave laptop alone.  Do nothing
			// (Ignore POWER_MODE commands to Dynamixel servos, passed in wParam)
			return;
		}
	}

	if( HW_SET_KINECT_POWER == Request )
	{
		#if ( (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL)  )
		// On iRobot Create Base, Kinect power is controled through the iRobot
			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
		#else
			ROBOT_LOG( TRUE, "ERROR!! : HW_SET_KINECT_POWER not supported on this Robot Type!\n" )
		#endif
		return;
	}

	if( HW_SET_SERVO_POWER == Request )
	{
		#if ( (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL)  )
		// On iRobot Create Base, Dynamixel power is controled through the iRobot
			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
		#endif
		return;
	}

	#if ( (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL)  )
		if( HW_SET_AUX_LIGHT_POWER == Request )
		{
			#if (TURTLE_TYPE == TURTLE_KOBUKI_WITH_ARDUINO )
				// When arduino used, control lights with the Arduino
				// Send the command to the Arduino Comm Write Thread
				PostThreadMessage( g_dwArduinoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			#else
				// On Turtle, LED Lights are usually controled through the iRobot base
				PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
			#endif
			return;
		}
	#endif

	// See if this is some other SERVO command
	if( (Request >= HW_OTHER_SERVO_MIN_COMMAND) && (Request <= HW_OTHER_SERVO_MAX_COMMAND) )
	{
		// Some other Servo command (not motor or camera related)
		// Camera command.
		#if OTHER_SERVO_CONTROL_TYPE == PIC_CONTROLLER	
			// Send the command to the Arduino Comm Write Thread
			PostThreadMessage( g_dwArduinoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

		#elif OTHER_SERVO_CONTROL_TYPE == EXTERN_SERVO_CONTROL	
			// Send this command to an external servo controller via the ServoComm Write Thread
			PostThreadMessage( g_dwServoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

		#elif OTHER_SERVO_CONTROL_TYPE == DYNA_SERVO_CONTROL	
			// Send this command to an Dynamixel and Kerr servos via the SmartServoComm Thread
			PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
		#else
			#error BAD OTHER_SERVO_CONTROL_TYPE
		#endif

		return;
	}

	// Other command.  Send the command to the Arduino Comm Write Thread
	// See if the command needs to be simulated
//	#if ( (MOTOR_CONTROL_TYPE != IROBOT_MOTOR_CONTROL) && (MOTOR_CONTROL_TYPE != KOBUKI_MOTOR_CONTROL)  )
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			// Arduino is not connected, simulate as if it were
			SimulateHardware( Request, wParam, lParam );
			return;
		}
//	#endif

	// Nope, real command to send to the Arduino!
	PostThreadMessage( g_dwArduinoCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );

}



/****
signed int CRobotModule::CalculateThreatDirection()
{
	// Calculate "average potential" of threats

	// Bumper collisions override other sensors
	if( CENTER_HIT == g_pFullSensorStatus->Bumper  )
	{		
		return( 0 );	// Center Hit on Both bumpers
	}
	else if( LEFT_HIT == g_pFullSensorStatus->Bumper )
	{		
		return( -10 );	// Hit on Left bumper (mostly in front, but slightly to one side)
	}
	else if( RIGHT_HIT == g_pFullSensorStatus->Bumper )
	{		
		return( 10 );	// Hit on Right bumper
	}

	// New version, attempts to weight sensor based upon direction of robot
	// Front Sensors get priority

	int nClosestObjectLeft = 255;
	if( g_pNavSensorSummary->nClosestObjectFrontLeft < nClosestObjectLeft )
		nClosestObjectLeft = g_pNavSensorSummary->nClosestObjectFrontLeft;
	if( g_pNavSensorSummary->nClosestObjectSideLeft < nClosestObjectLeft )
		nClosestObjectLeft = g_pNavSensorSummary->nClosestObjectSideLeft;

	int nClosestObjectRight = 255;
	if( g_pNavSensorSummary->nClosestObjectFrontRight < nClosestObjectRight )
		nClosestObjectRight = g_pNavSensorSummary->nClosestObjectFrontRight;
	if( g_pNavSensorSummary->nClosestObjectSideRight < nClosestObjectRight )
		nClosestObjectRight = g_pNavSensorSummary->nClosestObjectSideRight;

	// Return the result.  Negative means *threat* to the Left
	return( nClosestObjectLeft - nClosestObjectRight );

}
****/

void CRobotModule::SpeakCannedPhrase( int PhraseToSpeak )
{
		PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, PhraseToSpeak, 0 );

}



///////////////////////////////////////////////////////////////////////////////
//  MODULES
//  Message are sent to each module in order, highest priority first
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
//	MODULE: SYSTEM MODULE

CSystemModule::CSystemModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
}

CSystemModule::~CSystemModule()
{
		ROBOT_LOG( TRUE, "~CSystemModule()\n" )
}

void CSystemModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (wParam);

	switch( uMsg )  
	{
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			// Check for any errors
			switch( g_pFullSensorStatus->LastError )
			{
				case HW_ARDUINO_NO_ERROR:
				{
					// Nothing to do
				}
				break;

				case HW_SERIAL_CMD_BUFFER_OVERFLOW:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_CMD_BUFFER_OVERFLOW\n" )
				}
				break;

				case HW_SERIAL_CMD_NOT_READY:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_CMD_NOT_READY\n" )
				}
				break;

				case HW_SERIAL_CMD_INVALID:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_CMD_INVALID\n" )
				}
				break;

				case HW_SERIAL_CMD_CHECKSUM_ERROR:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_CMD_CHECKSUM_ERROR\n" )
				}
				break;

				case HW_SERIAL_EXTRA_CHARS:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_EXTRA_CHARS\n" )
				}
				break;

				case HW_BAD_STATE:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_BAD_STATE\n" )
				}
				break;

				case HW_UNKNOWN_CMD:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_UNKNOWN_CMD\n" )
				}
				break;

				case HW_CMD_NOT_IMPLEMENTED:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_CMD_NOT_IMPLEMENTED\n" )
				}
				break;

				case HW_SERIAL_BAD_SYNC:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERIAL_BAD_SYNC\n" )
				}
				break;

				case HW_SERIAL_WAIT_PRIOR_CMD:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Warning: HW_SERIAL_WAIT_PRIOR_CMD\n" )
				}
				break;

				case HW_SERVO_VALUE_ERROR:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: HW_SERVO_VALUE_ERROR\n" )
				}
				break;

				case HW_RADAR_SCAN_BAD_STATE:
				{
					ROBOT_DISPLAY( TRUE, ">>>  Arduino Error: IR_SCAN_BAD_STATE\n" )
				}
				break;

				case HW_BAD_ISR:
				{
					// This is a CRITICAL ERROR!
					ROBOT_DISPLAY( TRUE, ">>>  CRITICAL Arduino Error!! : HW_BAD_ISR\n" )
				}
				break;

				default:
				{
					CString MsgString;
					MsgString.Format( ">>>  Arduino Error: UNKNOWN ERROR: %02Xh", g_pFullSensorStatus->LastError );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );
				}
			}	// end switch( g_pFullSensorStatus->LastError )

 

			////////////////////////////////////////////////////////////////////////////
			// All status sent as one big bulk structure
			// Post message to the GUI display (local or remote)
			SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
				ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
				0 );									// Param2 = not used
			return;
		}
		case WM_ROBOT_REQUEST_VERSION_CMD:
		{
			g_bCmdRecognized = TRUE;
			SendHardwareCmd(HW_GET_VERSION, 0, 0);
			return;
		}

		// Used only in Simulation.  During real run, this command is sent
		// directly to the hardware by the TimerThreadProc in Globls.cpp
		case WM_ROBOT_REQUEST_STATUS_CMD:
		{
			g_bCmdRecognized = TRUE;
			SendHardwareCmd(HW_GET_STATUS, 0, 0);
			return;
		}

		/*
		case WM_ROBOT_PIC_VERSION_READY:
		{
			g_bCmdRecognized = TRUE;
			CString VersionInfo;
			if( lParam != ARDUINO_CODE_VERSION )
			{
				if( 99 == lParam )
				{
					VersionInfo.Format( "Arduino Version 99 - SIMULATION" );
				}
				else
				{
					ROBOT_LOG( TRUE, "\n===========================================\n")
					ROBOT_LOG( TRUE, "               VERSION ERROR!\n")
					ROBOT_LOG( TRUE, "===========================================\n")
					VersionInfo.Format( "ARDUINO VERSION ERROR!  Expected: %d, Received: %d", ARDUINO_CODE_VERSION, lParam );
					SpeakCannedPhrase( SPEAK_ARDUINO_VERSION_ERROR );
					AfxMessageBox( _T("ARDUINO VERSION ERROR!") );
				}
			}
			else
			{
				VersionInfo.Format( "Arduino S/W Version: %d", lParam );
				SpeakCannedPhrase( SPEAK_ARDUINO_CONNECTED );
			}
			ROBOT_DISPLAY( TRUE, (LPCTSTR)VersionInfo )
			SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_PIC_VERSION, lParam );
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 ); // Turn on eyes after version received. (gives time for Arduino sync)

			return;
		}
		*/
	}
}


///////////////////////////////////////////////////////////////////////////////
//	MODULE: USER COMMAND MODULE

CUserCmdModule::CUserCmdModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_LocalSpeed = SPEED_STOP;
	m_LocalTurn = TURN_CENTER;
	m_RemoteSpeed = SPEED_STOP;
	m_RemoteTurn = TURN_CENTER;
	m_VidCapProcessingEnabled = FALSE;
	m_IRTrackingEnabled = FALSE;
	m_FaceTrackingEnabled = FALSE;
	m_FaceIdentificationEnabled = FALSE;
	m_ObjectIdentificationEnabled = FALSE;
	m_pSharedMemory = NULL;
	m_CameraRequest.RequestData.FaceRequest.PersonID = 0;
	m_CameraRequest.RequestData.FaceRequest.PersonName[0] = 0;
	m_NextChatPhrase = 0;

	#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
		#if( ROBOT_HAS_RIGHT_ARM )
			m_pArmControlRight = new ArmControl( RIGHT_ARM );	// For arm position information
		#endif

		#if( ROBOT_HAS_LEFT_ARM )
			m_pArmControlLeft = new ArmControl( LEFT_ARM );	// For arm position information
		#endif
	#endif
//	m_UserOwner = REMOTE_USER_MODULE;	// Normal mode.  Can be switched to LOCAL_USER_MODULE if needed

}

CUserCmdModule::~CUserCmdModule()
{
	ROBOT_LOG( TRUE, "~CUserCmdModule()\n" )
	#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
		#if( ROBOT_HAS_RIGHT_ARM )
			SAFE_DELETE(m_pArmControlRight);
		#endif
		#if( ROBOT_HAS_LEFT_ARM )
			SAFE_DELETE(m_pArmControlLeft);
		#endif
	#endif
}


void CUserCmdModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )  
	{
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;
			CString MsgString;

			///////////////////////////////////////////////////////////////////////////////////////////
			// Handle Android Phone / bluetooth commands
			// and accelerometer
			// These commands come from the Arduino board (which is connected via BT to the Android phone)

			//if( !g_pFullSensorStatus->AndroidConnected )
			{
				// reissue last drive command continuously to keep control of wheels
				if( gLocalUserCmdTimer > 0 )
				{
					// Received a local user command recently
					m_pDriveCtrl->SetSpeedAndTurn( LOCAL_USER_MODULE, m_LocalSpeed, m_LocalTurn );
				}
				else if( gRemoteUserCmdTimer > 0 )
				{
					// Received a remote user command recently
					m_pDriveCtrl->SetSpeedAndTurn( REMOTE_USER_MODULE, m_RemoteSpeed, m_RemoteTurn );
				}
				else
				{
					// Have not received a local user command recently
					if( !g_pFullSensorStatus->AndroidAccEnabled )
					{
						m_RemoteSpeed = SPEED_STOP;
						m_RemoteTurn = TURN_CENTER;
						m_LocalSpeed = SPEED_STOP;
						m_LocalTurn = TURN_CENTER;
					}
				}
			}

		} // WM_ROBOT_SENSOR_STATUS_READY:
		break;

		case WM_ROBOT_OPEN_DATA_FILE:
		{
			g_bCmdRecognized = TRUE;
			// Post a Windows message to the Server GUI thread to open the Path file.
			// wParam = Path Type (Outdoor, Gridmap, etc.)
			// lParam = not used
			PostMessage( g_RobotCmdViewHWND, (WM_ROBOT_DISPLAY_OPEN_DATA_FILE), wParam, lParam );
			//m_NavPathStarted = FALSE;
		}		
		break;

		case WM_ROBOT_MOVE_SET_DISTANCE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// wParam = Distance in TENTH INCHES, lParam = direction
			// Owner is always assumed to be REMOTE_USER_MODULE
			
			// DEBUG
			int nTempTicks = (int)(wParam * TICKS_PER_TENTH_INCH);

			CString MsgString;
			MsgString.Format( "MOVE_SET_DISTANCE: %d Inches", 
				wParam/10 );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			// VALIDATE Arduino CODE HERE - THEN REMOVE THIS!  TODO-CAR
			//WORD gDesiredMoveTicks = (((BYTE) nDistHigh) << 8);	// Get High Byte
			//gDesiredMoveTicks += ((BYTE) nDistLow);		// Add in Low Byte	
			//ROBOT_LOG( TRUE, "DEBUG: gDesiredMoveTicks = %u\n", gDesiredMoveTicks)

			if( FORWARD == lParam )
			{
				m_pDriveCtrl->SetMoveDistance( REMOTE_USER_MODULE, SPEED_FWD_MED_SLOW, TURN_CENTER, wParam);
			}
			else
			{
				m_pDriveCtrl->SetMoveDistance( REMOTE_USER_MODULE, SPEED_REV_MED_SLOW, TURN_CENTER, wParam);
			}
		}
		break;


		case WM_ROBOT_TURN_SET_DISTANCE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// wParam = Turn Amount in degrees, 
			// lParam = direction (Left/Right) and speed
			// Owner is always assumed to be REMOTE_USER_MODULE
			
			CString MsgString;
			MsgString.Format( "TURN_SET_DISTANCE: %d Degrees", wParam );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			if( LEFT == lParam )
			{
				m_pDriveCtrl->SetTurnRotation( REMOTE_USER_MODULE, SPEED_STOP, (int)lParam, wParam); // Speed of turn, turn amount
			}
			else
			{
				m_pDriveCtrl->SetTurnRotation( REMOTE_USER_MODULE, SPEED_STOP, (int)lParam, wParam);
			}
		}
		break;

		case WM_ROBOT_SET_AVOID_OBJ_RANGE:
		{
			g_bCmdRecognized = TRUE;
			CString MsgString;
			MsgString.Format("Setting Avoid Object Range to %u feet", lParam);
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			g_GlobalMaxAvoidObjectDetectionFeet = (int )lParam;
		}
		break;

		case WM_ROBOT_BRAKE_CMD:
		{
			g_bCmdRecognized = TRUE;
			if( 0 != lParam )
			{
				ROBOT_DISPLAY( TRUE, "TEST: SENDING BRAKE COMMAND!")
				SendHardwareCmd(HW_SET_MOTOR_BRAKE, (DWORD)SPEED_BRAKE_AMOUNT, (DWORD)SPEED_BRAKE_TIME);
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "SENDING BRAKE COMMAND!")
				m_pDriveCtrl->Brake( LOCAL_USER_MODULE );
				// m_pDriveCtrl->ExecuteCommand();	// Don't wait for next status update
			}
		}
		break;

		case WM_ROBOT_STOP_CMD:
		{
			// Take control from all other modules and Stop 
			// Assumes user is always with local user authority
			ROBOT_LOG( TRUE, " WM_ROBOT_STOP_CMD: Stop\n" )
			// ?? this will force the other modules to abort and go back to IDLE state:

//			m_UserOwner = LOCAL_USER_MODULE;	// Super user mode
			//m_pDriveCtrl->SuppressModule( COLLISION_MODULE );
			//m_pDriveCtrl->SuppressModule( AVOID_OBJECT_MODULE );
			//m_pDriveCtrl->SuppressModule( WAY_POINT_NAV_MODULE );
			//m_pDriveCtrl->SuppressModule( GRID_NAV_MODULE );
			gLocalUserCmdTimer = 5; // TenthSeconds, so robot stays stopped for at least 1/2 second
			m_LocalSpeed = SPEED_STOP;
			m_LocalTurn = TURN_CENTER;
			m_RemoteSpeed = SPEED_STOP;
			m_RemoteTurn = TURN_CENTER;
			m_pDriveCtrl->SetSpeedAndTurn( LOCAL_USER_MODULE, m_LocalSpeed, m_LocalTurn );
		}
		break;

		case WM_ROBOT_DRIVE_LOCAL_CMD:
		{
			// wParam is Speed, lParam is Turn
			g_bCmdRecognized = TRUE;

			gLocalUserCmdTimer = 10; // TenthSeconds, so robot will run for 1 second without a command, then stop
			int NewSpeed = wParam;
			int NewTurn = lParam;
			int Owner = LOCAL_USER_MODULE;
			ROBOT_LOG( DEBUG_MOTOR_COMMANDS, "Received WM_ROBOT_DRIVE_LOCAL_CMD: Speed=%d, Turn=%d  ",  NewSpeed, NewTurn )

			// Speed and Turn commands from GUI are +/- 127, with zero = Center/Stop 
			// Convert to HW command values, for reverse, we add a negative number
			#if( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
				// RC Car with external servo control: Servo max values are 0-255

				m_LocalSpeed = SPEED_STOP + (NewSpeed);	 // convert to servo range
				if( m_LocalSpeed > SPEED_FULL_FWD )			// Limit to valid range
				{
					m_LocalSpeed = SPEED_FULL_FWD;	
				}
				else if( m_LocalSpeed < SPEED_FULL_REV )
				{
					m_LocalSpeed = SPEED_FULL_REV;	
				}

				m_LocalTurn = (NewTurn);	// convert to servo range
				if( m_LocalTurn < TURN_LEFT_MAX )			// Limit to valid range
				{
					m_LocalTurn = TURN_LEFT_MAX;	
				}
				else if( m_LocalTurn > TURN_RIGHT_MAX )
				{
					m_LocalTurn = TURN_RIGHT_MAX;	
				}

				//int test = (TURN_CENTER - m_LocalTurn);
				m_pDriveCtrl->SetSpeedAndTurn( Owner, m_LocalSpeed, m_LocalTurn );

			////////////////////////////////////////////////////////////////////////////////////
			#else // Not Servo motor control
				m_LocalSpeed = NewSpeed;
				m_LocalTurn = NewTurn;
				m_pDriveCtrl->SetSpeedAndTurn( Owner, m_LocalSpeed, m_LocalTurn, ACCELERATION_FAST ); // Local control snappy, remote a little slower
			#endif

			return;
		}

		case WM_ROBOT_DRIVE_REMOTE_CMD:
		{
			// wParam is Speed, lParam is Turn
			g_bCmdRecognized = TRUE;

			gRemoteUserCmdTimer = 10; // TenthSeconds, so robot will run for 1 second without a command, then stop
			int NewSpeed = wParam;
			int NewTurn = lParam;
			int Owner = REMOTE_USER_MODULE;
			ROBOT_LOG( DEBUG_MOTOR_COMMANDS, "Received WM_ROBOT_DRIVE_REMOTE_CMD: Speed=%d, Turn=%d  ",  NewSpeed, NewTurn )

			// Speed and Turn commands from GUI are +/- 127, with zero = Center/Stop 
			// Convert to HW command values, for reverse, we add a negative number
			#if( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
				// RC Car with external servo control: Servo max values are 0-255

				m_RemoteSpeed = SPEED_STOP + (NewSpeed);	 // convert to servo range
				if( m_RemoteSpeed > SPEED_FULL_FWD )			// Limit to valid range
				{
					m_RemoteSpeed = SPEED_FULL_FWD;	
				}
				else if( m_RemoteSpeed < SPEED_FULL_REV )
				{
					m_RemoteSpeed = SPEED_FULL_REV;	
				}

				m_RemoteTurn = (NewTurn);	// convert to servo range
				if( m_RemoteTurn < TURN_LEFT_MAX )			// Limit to valid range
				{
					m_RemoteTurn = TURN_LEFT_MAX;	
				}
				else if( m_RemoteTurn > TURN_RIGHT_MAX )
				{
					m_RemoteTurn = TURN_RIGHT_MAX;	
				}

				//int test = (TURN_CENTER - m_RemoteTurn);
				m_pDriveCtrl->SetSpeedAndTurn( Owner, m_RemoteSpeed, m_RemoteTurn );

			////////////////////////////////////////////////////////////////////////////////////
			#else // Not Servo motor control
				m_RemoteSpeed = NewSpeed;
				m_RemoteTurn = NewTurn;
				m_pDriveCtrl->SetSpeedAndTurn( Owner, m_RemoteSpeed, m_RemoteTurn );
			#endif

			return;
		}

		case WM_ROBOT_RELEASE_OWNER_CMD:
		{
			m_pDriveCtrl->ReleaseOwner( wParam ); // wParam is the owner ID
			return;
		}

		case WM_ROBOT_RESET_WATCHDOG_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Process Watchdog Reset command to Arduino
			// 12v Power ON also does similar things, but this function will not turn power on/of
			// Tell the PC to resume sending Keep Alive pings
			g_bResetWatchdog = TRUE;
			// Send Command to reset the watchdog timer, and clear Watchdog error state
			// BOOL bResult = VendorCommandWrite( HW_RESET_WATCHDOG, 0, 0 );
			return;
		}

		case WM_ROBOT_RESET_ODOMETER:
		{
			g_bCmdRecognized = TRUE;
			// Process Odometer Reset command to Arduino
			g_pFullSensorStatus->OdometerTenthInches = 0;
			g_pFullSensorStatus->OdometerUpdateTenthInches = 0;
			SendHardwareCmd( HW_RESET_ODOMETER, 0, (BYTE)lParam );	// 0=off, non-zero = on
			return;
		}


		case WM_ROBOT_SERVO_POWER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Process Servo Power On/Off command from the GUI
			// (Allows user to manually turn servo power On/Off. 
			
			// Send Command to turn the power on or off
			SendHardwareCmd( HW_SET_SERVO_POWER, 0, (BYTE)lParam );	// 0=off, non-zero = on
			return;
		}

		case WM_ROBOT_ENABLE_RADAR_SCAN_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Process Command to enable a Radar Scanning
			// The Arduino will indicate when each scan is complete, and run until disabled

			// First, set the Tilt Angle (adjust for minimum height object detected)
			// Note: CENTER=63, MAX_UP=75, MAX_DOWN=45
			/// TODO: SendHardwareCmd( HW_SET_CAMERA_TILT_ABS, (CAMERA_TILT_SERVO_CENTER-2), 0 );
			ROBOT_ASSERT(0); // need to fix this

			// Now, tell the Radar Scan to start
			SendHardwareCmd( HW_ENABLE_RADAR_SCAN, (BYTE)wParam, (BYTE)lParam );
			return;
		}

		case WM_ROBOT_SERVO_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Process Servo position command from the GUI
			// Servos numbered above 127 are controlled via PC Serial Port (handled by CServoModule)
			// Servos below 127 are controlled via Arduino (handled here)
			//ROBOT_DISPLAY( TRUE, "Servo Command received" )

			WORD	nServoNumber = (WORD)wParam;
			WORD	nPosition = (WORD)lParam;
			if( nServoNumber <= 127 )
			{
				SendHardwareCmd( HW_SET_SERVO_POS_TICKS, nServoNumber, nPosition );
			}
			return;
		}

/*		case WM_ROBOT_SET_ARM_JOINT_POSITION:
		{
			g_bCmdRecognized = TRUE;
			// Process Arm Joint position command from the GUI
			ROBOT_DISPLAY( TRUE, "Arm Servo Single Joint Command received" )
			SendHardwareCmd( HW_SET_ARM_JOINT_POSITION, wParam, lParam );
			return;
		}
*/
		case WM_ROBOT_SET_ARM_POSITION:
		{
			g_bCmdRecognized = TRUE;
			// Process command to move all joints in the arm.  Uses global g_BulkServoCmd to pass the data
			//ROBOT_DISPLAY( TRUE, "Arm Servo Bulk Positon Command received" )


			// Get current X,Y,Z position for debug
			#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////

				if( LEFT_ARM == wParam )
				{
					#if( ROBOT_HAS_LEFT_ARM )
						FPOINT3D_T LeftArmXYZ;
						m_pArmControlLeft->GetTargetArmXYZ( LeftArmXYZ );
					#endif
					//ROBOT_LOG( TRUE, "DEBUG: LEFT ARM X = %4.1f, Y = %4.1f, Z = %4.1f\n", ArmX, ArmY, ArmZ )
				}
				else
				{
					#if( ROBOT_HAS_RIGHT_ARM )
						FPOINT3D_T RightArmXYZ;
						m_pArmControlRight->GetTargetArmXYZ( RightArmXYZ );
					#endif
					//ROBOT_LOG( TRUE, "DEBUG: RIGHT ARM X = %4.1f, Y = %4.1f, Z = %4.1f\n", ArmX, ArmY, ArmZ )
				}

				//ROBOT_DISPLAY( TRUE, "MODULE: Sending HW_SET_BULK_ARM_POSITION\n" )
				SendHardwareCmd( HW_SET_BULK_ARM_POSITION, wParam, lParam );
			#endif
			return;
		}

		case WM_ROBOT_SET_HEAD_POSITION:
		{
			g_bCmdRecognized = TRUE;
			// Process command to move all joints in the head.  Uses global g_BulkServoCmd to pass the data
			//ROBOT_DISPLAY( TRUE, "Head Servo Bulk Positon Command received" )
			SendHardwareCmd( HW_SET_BULK_HEAD_POSITION, wParam, lParam );
			return;
		}

		case WM_ROBOT_SET_KINECT_POSITION:
		{
			g_bCmdRecognized = TRUE;
			// Process command to move Kinect servo.  Uses global g_BulkServoCmd to pass the data
			//ROBOT_DISPLAY( TRUE, "Kinect Bulk Positon Command received" )
			SendHardwareCmd( HW_SET_BULK_KINECT_POSITION, wParam, lParam );
			return;
		}

		case WM_ROBOT_SET_SERVO_TORQUE_ENABLE:
		{
			g_bCmdRecognized = TRUE;
			// Process command to enable/disable servo torque.
			// Uses bitfields: wParam = Operation, lParam = Enable/Disable
			//ROBOT_DISPLAY( TRUE, "Arm Servo Bulk Torque Enable Command received" )
			SendHardwareCmd( HW_SET_SERVO_TORQUE_ENABLE, wParam, lParam );
			return;
		}

		case WM_ROBOT_SET_SERVO_TORQUE_LIMIT:
		{
			g_bCmdRecognized = TRUE;
			// Process command to set servo torque.
			//ROBOT_DISPLAY( TRUE, "Servo Set Torque Command received" )
			SendHardwareCmd( HW_SET_SERVO_TORQUE_LIMIT, wParam, lParam );
			return;
		}

		case WM_ROBOT_GET_SMART_SERVO_STATUS:
		{
			g_bCmdRecognized = TRUE;
			// Process command to read current status of all joints in the arm.  Uses global g_BulkServoCmd to receive the data
			//ROBOT_DISPLAY( TRUE, "Arm Get Bulk Status received" )
			SendHardwareCmd( HW_GET_SMART_SERVO_STATUS, wParam, lParam );
			return;
		}

		case WM_ROBOT_GET_SERVO_STATUS:
		{
			g_bCmdRecognized = TRUE;
			// Process Servo status request from the GUI
			//ROBOT_DISPLAY( TRUE, "Servo Status Request received" )

			BYTE	nServoNumber = (BYTE)wParam;
			if( nServoNumber <= 127 )
			{
				SendHardwareCmd( HW_GET_SERVO_STATUS, nServoNumber, 0 );
			}
			return;
		}


		case WM_ROBOT_SET_GEAR_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Set the Car Gear speed.
			int Gear = lParam;		// 0 = Low Gear, 1 = High Gear
			if( 0 == lParam )
			{
				// Low Gear
				SendHardwareCmd( HW_SET_GEAR, 0, LOW_GEAR );	// Servo position for Low
			}
			else
			{
				// High Gear
				SendHardwareCmd( HW_SET_GEAR, 0, HIGH_GEAR );	// Servo position for High
			}
			return;
		}

		/* Handled by Behavioral Module
		case WM_ROBOT_SET_LED_EYES_CMD:
		*/

		case WM_ROBOT_LIGHT_POWER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to set the driving Lights On/Off
			SendHardwareCmd( HW_SET_LIGHT_POWER, 0, (BYTE)lParam );	// 0=off, 1=on
			return;
		}

		case WM_ROBOT_AUX_LIGHT_POWER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to set the Aux Lights On/Off
			SendHardwareCmd( HW_SET_AUX_LIGHT_POWER, 0, (BYTE)lParam );	// 0=off, 1=on
			g_pFullSensorStatus->AuxLightsOn = (BOOL)lParam; // keep track of current state
			return;
		}

/*		case WM_ROBOT_ENABLE_DYNA_CONTROLLER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to enable Dyna Servos.  For Turtle, this enables power to the servos
			SendHardwareCmd( HW_SET_SERVO_POWER, 0, (BYTE)lParam );	// 0=off, 1=on
			return;
		}
*/		
		case WM_ROBOT_KINECT_POWER_ENABLE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to set the Kinect On/Off
			SendHardwareCmd( HW_SET_KINECT_POWER, 0, (BYTE)lParam );	// 0=off, 1=on
			return;
		}

		case WM_ROBOT_POWER_MODE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to turn servos and laptop on/off
			//                                   SERVO PWR,    LAPTOP PWR
			SendHardwareCmd( HW_SET_POWER_MODE, (BYTE)wParam, (BYTE)lParam );	// see LAPTOP_POWER_MODE_DEFAULT
			return;
		}


		case WM_ROBOT_IR_SENSOR_POWER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to enable/disable power to the IR sensor in the RIGHT claw
			SendHardwareCmd( HW_SET_IR_SENSOR_POWER, /*TODO - Right/L*/ 0, (BYTE)lParam );	// 0=off, 1=on
			return;
		}


		case WM_ROBOT_TEXT_TO_AI:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to play a sound
			BOOL bResult = PostThreadMessage( g_dwSpeakThreadId, (WM_ROBOT_TEXT_TO_AI), wParam, lParam);
			return;
		}

		case WM_ROBOT_SPEAK_TEXT:
		{
			// send command to speak some text. TODO-MUST: For System generated speech, call the below directly!!!
			//PostMessage( g_RobotSetupViewHWND, WM_ROBOT_SPEAK_TEXT, wParam, lParam );
			PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, SPEAK_TEXT_FROM_BUFFER, 0 );

		}

		case WM_ROBOT_PLAY_SOUND:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to play a sound wav file (NOT Text to speach!)
			//	DONT DO THIS:	PostMessage( g_RobotSetupViewHWND, WM_ROBOT_SPEAK_TEXT, wParam, lParam );
			BOOL bResult = PostThreadMessage( g_dwSoundThreadId, (WM_ROBOT_PLAY_SOUND), wParam, lParam);
			return;
		}

		case WM_ROBOT_PLAY_MUSIC:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to play a music file
			BOOL bResult = PostThreadMessage( g_dwSoundThreadId, (WM_ROBOT_PLAY_MUSIC), wParam, lParam);
			return;
		}

		case WM_ROBOT_SET_COMPASS_CAL_MODE:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to enter Compass Calibration Mode
			SendHardwareCmd( HW_COMPASS_CAL_MODE, 0, (BYTE)lParam );	// 0=off, 1=on
			return;
		}

		case WM_ROBOT_SET_COMPASS_CAL_POINT:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to enter Compass Calibration Mode
			SendHardwareCmd( HW_COMPASS_CAL_POINT, 0, 0 );
			return;
		}

		case WM_ROBOT_CAMERA_ENABLE_FEATURE:
		{
			// Handle Camera feature enabling / disabling
			//g_bCmdRecognized = TRUE;
			BOOL EnableFeature = (BOOL)lParam;
			switch( wParam )
			{
				case CAMERA_ENABLE_TRACKING_IR:
				{
					m_IRTrackingEnabled = EnableFeature;
					return; // Special case IR, not handled by the camera app
				}
				case CAMERA_ENABLE_VIDCAP_PROCESSING:
				{
					m_VidCapProcessingEnabled = EnableFeature;
					m_CameraRequest.RequestData.EnableFeatures.VideoEnable = (int)EnableFeature;
					break;
				}
				case CAMERA_ENABLE_TRACKING_FACE:
				{
					m_FaceTrackingEnabled = EnableFeature;
					m_CameraRequest.RequestData.EnableFeatures.FaceTracking = (int)EnableFeature;
					break;
				}
				case CAMERA_ENABLE_FACE_IDENTIFICATION:
				{
					m_FaceIdentificationEnabled = EnableFeature;
					m_CameraRequest.RequestData.EnableFeatures.FaceRecognition = (int)EnableFeature;
					break;
				}
				case CAMERA_ENABLE_MATCHING_OBJECTS:
				{
					m_ObjectIdentificationEnabled = EnableFeature;
					m_CameraRequest.RequestData.EnableFeatures.ObjectMatch = (int)EnableFeature;
					break;
				}				
				default:
				{
					// ignore for now, lots of these are not enabled yet in the new design
					//ROBOT_LOG( TRUE, "ERROR: Unhandled Camera feature\n" )
				}
			}

			// Send data to Camera App:
			if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
			{
				//g_FaceCaptureName = "Fred"; // DEBUG
				m_CameraRequest.RequestType = CAMERA_REQUEST_ENABLE_FEATURES;

				CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
				SetEvent( g_hCameraRequestEvent );  // Send request to remember current person in view
			}
			else
			{
				ROBOT_LOG( TRUE, "Not enabling camera features.  AUTO_LAUNCH_CAMERA_APP probably not enabled\n" )
			}
			break;
		}

		case WM_ROBOT_CAMERA_TAKE_SNAPSHOT: 
		{
			// Signal the Camera OpenCL app to capture the current frame.
			g_bCmdRecognized = TRUE;

			// Send data to Camera App:
			if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
			{
				// TODO - supply an name or number for the picture?
				//m_CameraRequest.RequestData.FaceRequest.PersonID = -1; // Add new user
				//strncpy_s( m_CameraRequest.RequestData.FaceRequest.PersonName, (LPCTSTR)g_FaceCaptureName, CAMERA_MAX_NAME_LEN );
				m_CameraRequest.RequestType = CAMERA_REQUEST_TAKE_SNAPSHOT;
				CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
				SetEvent( g_hCameraRequestEvent );  // Send request
				ROBOT_LOG( TRUE, "WM_ROBOT_CAMERA_TAKE_SNAPSHOT - Requested Snapshot" )
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR: Can't take Snapshot!  Did you have AUTO_LAUNCH_CAMERA_APP enabled?\n" )
			}
			break;
		}

		case WM_ROBOT_CAMERA_CAPTURE_FACE:
		{
			// Signal the Camera OpenCV app to capture the current person's face, and assigned the selected name.
			// Person's name is passed in g_FaceCaptureName. wParam and lParam not used.
			g_bCmdRecognized = TRUE;

			// Send data to Camera App:
			if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
			{
				//g_FaceCaptureName = "Fred"; // DEBUG
				m_CameraRequest.RequestType = CAMERA_REQUEST_FACE_RECO;
				m_CameraRequest.RequestData.FaceRequest.PersonID = -1; // Add new user
				strncpy_s( m_CameraRequest.RequestData.FaceRequest.PersonName, (LPCTSTR)g_FaceCaptureName, CAMERA_MAX_NAME_LEN );
				CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
				SetEvent( g_hCameraRequestEvent );  // Send request
				ROBOT_LOG( TRUE, "WM_ROBOT_CAMERA_CAPTURE_FACE - Requested Face capture" )
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR: Can't capture Face!  Did you have AUTO_LAUNCH_CAMERA_APP enabled?\n" )
			}
			break;
		}

		case WM_ROBOT_CAMERA_MATCH_OBJECT:
		{
			// Signal the Camera OpenCV app to attempt to match an object in the database to current scene.
			// Need to identify how to pass objet info back. wParam and lParam not used.
			g_bCmdRecognized = TRUE;

			// Send data to Camera App:
			if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
			{
				//g_FaceCaptureName = "Fred"; // DEBUG
				m_CameraRequest.RequestType = CAMERA_REQUEST_OBJECT_RECO;
				//m_CameraRequest.RequestData.FaceRequest.PersonID = -1; // Add new user
				//strncpy_s( m_CameraRequest.RequestData.FaceRequest.PersonName, (LPCTSTR)g_FaceCaptureName, CAMERA_MAX_NAME_LEN );
				CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
				SetEvent( g_hCameraRequestEvent );  // Send request
				ROBOT_LOG( TRUE, "WM_ROBOT_CAMERA_MATCH_OBJECT - Requested Find Object" )
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR: Can't request find object!  Did you have AUTO_LAUNCH_CAMERA_APP enabled?\n" )
			}
			break;
		}

	}
}




