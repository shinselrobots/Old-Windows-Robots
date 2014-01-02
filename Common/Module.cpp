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
	Shuffle();

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
					ARDUINO_CMD_T	SimulatedPicCmd;
					SimulatedPicCmd.Cmd = Request;
					SimulatedPicCmd.Param1 = wParam;
					SimulatedPicCmd.Param2 = lParam;
					SimulatePic( SimulatedPicCmd );
				}
			#else
				// all other hardware configurations
					// Arduino is not connected, simulate as if it were
					ARDUINO_CMD_T	SimulatedPicCmd;
					SimulatedPicCmd.Cmd = (BYTE)Request;
					SimulatedPicCmd.Param1 = (BYTE)wParam;
					SimulatedPicCmd.Param2 = (BYTE)lParam;
					SimulatePic( SimulatedPicCmd );
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

	#if ROBOT_TYPE == TURTLE
		// On Turtle, LED Lights are controled through the iRobot base
		if( HW_SET_AUX_LIGHT_POWER == Request )
		{
			PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+Request), wParam, lParam );
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
			ARDUINO_CMD_T	SimulatedPicCmd;
			SimulatedPicCmd.Cmd = (BYTE)Request;
			SimulatedPicCmd.Param1 = (BYTE)wParam;
			SimulatedPicCmd.Param2 = (BYTE)lParam;
			SimulatePic( SimulatedPicCmd );
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
	if( CENTER_HIT == g_SensorStatus.Bumper  )
	{		
		return( 0 );	// Center Hit on Both bumpers
	}
	else if( LEFT_HIT == g_SensorStatus.Bumper )
	{		
		return( -10 );	// Hit on Left bumper (mostly in front, but slightly to one side)
	}
	else if( RIGHT_HIT == g_SensorStatus.Bumper )
	{		
		return( 10 );	// Hit on Right bumper
	}

	// New version, attempts to weight sensor based upon direction of robot
	// Front Sensors get priority

	int nClosestObjectLeft = 255;
	if( g_pSensorSummary->nClosestObjectFrontLeft < nClosestObjectLeft )
		nClosestObjectLeft = g_pSensorSummary->nClosestObjectFrontLeft;
	if( g_pSensorSummary->nClosestObjectSideLeft < nClosestObjectLeft )
		nClosestObjectLeft = g_pSensorSummary->nClosestObjectSideLeft;

	int nClosestObjectRight = 255;
	if( g_pSensorSummary->nClosestObjectFrontRight < nClosestObjectRight )
		nClosestObjectRight = g_pSensorSummary->nClosestObjectFrontRight;
	if( g_pSensorSummary->nClosestObjectSideRight < nClosestObjectRight )
		nClosestObjectRight = g_pSensorSummary->nClosestObjectSideRight;

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
			switch( g_SensorStatus.LastError )
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
					MsgString.Format( ">>>  Arduino Error: UNKNOWN ERROR: %02Xh", g_SensorStatus.LastError );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );
				}
			}	// end switch( g_SensorStatus.LastError )

 

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
	m_CurrentSpeed = SPEED_STOP;
	m_CurrentTurn = TURN_CENTER;
	m_VidCapProcessingEnabled = FALSE;
	m_IRTrackingEnabled = FALSE;
	m_FaceTrackingEnabled = FALSE;
	m_FaceIdentificationEnabled = FALSE;
	m_ObjectIdentificationEnabled = FALSE;
	m_UserOwnerRequested = FALSE;
//	HANDLE m_hCameraRequestEvent = NULL;
	m_pSharedMemory = NULL;
	m_CameraRequest.RequestData.FaceRequest.PersonID = 0;
	m_CameraRequest.RequestData.FaceRequest.PersonName[0] = 0;
	m_AndroidHasMotorControl = FALSE;
	m_NextChatPhrase = 0;

	#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
		m_pArmControlRight = new ArmControl( RIGHT_ARM );
		m_pArmControlLeft = new ArmControl( LEFT_ARM );
	#endif
	m_UserOwner = LOCAL_USER_MODULE;	// Normal mode.  Can be switched to OVERRIDE_MODULE if needed

}

CUserCmdModule::~CUserCmdModule()
{
	ROBOT_LOG( TRUE, "~CUserCmdModule()\n" )
	#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
		SAFE_DELETE( m_pArmControlRight );
		SAFE_DELETE( m_pArmControlLeft );
	#endif
}


void CUserCmdModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )  
	{
		//case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;
			CString MsgString;

			// DEBUG
			//m_pArmControlLeft->GetPressureLoadPercent();


			///////////////////////////////////////////////////////////////////////////////////////////
			// Handle Android Phone / bluetooth commands
			// and accelerometer

			// Handle Bluetooth Phone control
			if( g_SensorStatus.AndroidConnected )
			{
				// Handle button commands
/*
		SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 0, 0 );	// Disable
		SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable
		PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, FALSE, 0 );	// Turn Off
		PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, TRUE, 0 );	// Turn On
		// resets for next introduction
		SpeakText( "I like meeting people" );

*/
				switch( g_SensorStatus.AndroidCommand )
				{				
					case 0: // No new command
					{
						break;
					}
					case 1: // Wave and say hello
					{
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );
						ROBOT_LOG( TRUE,  "Got Android command: Wave\n")
						break;
					}
					case 2: // Head Center
					{
						SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 );
						SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_CENTER, 5 );
						ROBOT_LOG( TRUE,  "Got Android command: Head Center\n")
						break;
					}
					case 3: //  "New Chat" - chat with adults  // OLD: New Child Conversation
					{
						ROBOT_LOG( TRUE,  "Got Android command: New Adult Chat\n")
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_CHAT_DEMO_WITH_ADULT, 1 );	// TRUE = start mode
						break;
					}
					case 4: // "Quick Chat" - say a quick phrase   // OLD: New Child Conversation
					{
						ROBOT_LOG( TRUE,  "Got Android command: New Quick Chat\n")
						CString TextToSpeak;
						switch( m_NextChatPhrase++ )
						{
							case 0:   TextToSpeak =  "My name is Low Key" ;break;
							case 1:   TextToSpeak =  "What is your name?" ;break;
							case 2:   TextToSpeak =  "Do you like robots?" ;break;
							case 3:   TextToSpeak =  "want to hear some jokes?" ;break;
							default:  
								m_NextChatPhrase = 0;
								TextToSpeak =  "Do you live here?" ;
								break;
						}
						SpeakText( TextToSpeak );
						//g_MoveArmsWhileSpeaking = FALSE;
						break;
					}
					case 5: // Follow Me
					{
						ROBOT_LOG( TRUE,  "Got Android command: Follow Me\n")
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_FOLLOW_PERSON, (DWORD)TRUE );	// TRUE = start mode
						SpeakText( "I will follow you" );
						break;
					}
					case 6: // Tell Joke
					{
						ROBOT_LOG( TRUE,  "Got Android command: Tell Joke\n")
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TELL_JOKES, (DWORD)0 ); // single joke only
						break;
					}
					// TODO - Make this "come here"?
					case 7: // Take Picture // OLD: Pan/Tilt Head by Accelerometer
					{

						ROBOT_DISPLAY( TRUE, "android  Command Recognized: Arms/Hands UP")
				SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)ARM_MOVEMENT_ARM_UP_FULL );	// Right/Left arm, Movement to execute, 
					RobotSleep(100, pDomainSpeakThread); // give time for arm to raise 
					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0: SpeakText( "Ok, Don't shoot" ); break;
						default:  SpeakText( "I am not the droid you are looking for" );// If const is larger number, this gets called more often
					}
			
						//ROBOT_LOG( TRUE,  "Got Android command: Take Picture \n")
						//SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TAKE_PHOTO, (DWORD)TRUE );	// TRUE = start mode

						/* Set Speed
						m_pHeadControl->SetHeadSpeed(HEAD_OWNER_USER_CONTROL, WiiRollPanServoSpeed, WiiPitchTiltServoSpeed, NOP );
						// Set position
						m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL,
							WiiRollPanChangeTenthDegrees, WiiPitchTiltChangeTenthDegrees, NOP, FALSE ); // TRUE = Limit to front of robot
						m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );
						*/
						
						break;
					}
					case 8: // Shake
					{
						ROBOT_LOG( TRUE,  "Got Android command: Shake Hands\n")
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );
						break;
					}

					case 9: // Have Something
					{
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW );
						break;
					}
					case 10: // Throw Away
					{
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PUT_IN_BASKET );
						// Respond with random phrases
						int RandomNumber = ((4 * rand()) / RAND_MAX);
						ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
						switch( RandomNumber )
						{
							case 0:  SpeakText( "easy come, easy go" );break;
							case 1:  SpeakText( "I will take care of this later" );break;
							case 3:  SpeakText( "OK, if you say so" );break;
							default: SpeakText( "Good idea" ); // If const is larger number, this gets called more often
						}
						break;
					}
					case 11: // Clean Up
					{
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_PICKUP_OBJECTS, (DWORD)1 );
						SpeakText( "A robots work is never done" );break;
						break;
					}
					case 12: // Intro
					{
						ROBOT_LOG( TRUE,  "Got Android command: 12 Intro \n")
						PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, SPEAK_INTRO, 0 );
						break;
					}

					case 13: // EMERGENCY STOP!
					{
						m_pDriveCtrl->SetSpeedAndTurn( OVERRIDE_MODULE, SPEED_STOP, TURN_CENTER );
						m_AndroidHasMotorControl = TRUE;
						ROBOT_LOG( TRUE,  "Got Android command: STOP\n")
						SendCommand( WM_ROBOT_USER_OVERRIDE_CMD, SET_USER_OVERRIDE_AND_STOP, 0 ); // Tell all modules to reset (cancel current behavior)
						break;
					}
					case 14: // What Time
					{
						ROBOT_LOG( TRUE,  "Got Android command: 17 What Time \n")
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_WHAT_TIME_IS_IT, (DWORD)0 );
						break;
					}
					case 15: // Danger
					{
						ROBOT_LOG( TRUE,  "Got Android command: 18 Danger\n")
						SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_FREAK_OUT, (DWORD)0 );
						break;
					}

					default:
						ROBOT_LOG( TRUE,  "ERROR! Unhandled Android Button: %d\n", g_SensorStatus.AndroidCommand)
				}
					
			}

			// Handle Accelerometer
			if( g_SensorStatus.AndroidAccEnabled )
			{
				int AndoidCmdSpeed = 0;
				int AndoidCmdTurn = 0;

				// Pitch can go to over 90 degrees, but then roll does not work well, so max at pitch = 60 degrees
				// Convert Speed from +/- 60 to +/- 127, but leave guard band at center				
				int Pitch = g_SensorStatus.AndroidPitch - 10; //  To make it comfortable to hold the phone, "neutral" is 10 degrees up
				if( (Pitch > -15) && (Pitch< 10) )
				{
					AndoidCmdSpeed = 0;
				}
				else
				{					
					AndoidCmdSpeed = (int)( ((double)Pitch * -127.0) / 60.0 );	// Invert, down = forward.
					if( AndoidCmdSpeed > 127 ) AndoidCmdSpeed = 127;
					if( AndoidCmdSpeed < -127 ) AndoidCmdSpeed = -127;
				}

				// Convert Turn from +/- 90 to +/- 64, but leave guard band at center
				if( (g_SensorStatus.AndroidRoll > -10) && (g_SensorStatus.AndroidRoll < 10) )
				{
					AndoidCmdTurn = 0;
				}
				else
				{					
					AndoidCmdTurn = (int)( ((double)g_SensorStatus.AndroidRoll * 64.0) / 80.0 ); // slightly less then 90 to ease wrist motion
					if( AndoidCmdTurn > 64 ) AndoidCmdTurn = 64;
					if( AndoidCmdTurn < -64 ) AndoidCmdTurn = -64;
				}

				ROBOT_LOG( TRUE,  "AndoidCmdSpeed = %d, AndoidCmdTurn = %d\n", AndoidCmdSpeed, AndoidCmdTurn)

				m_pDriveCtrl->SetSpeedAndTurn( OVERRIDE_MODULE, AndoidCmdSpeed, AndoidCmdTurn );
				m_AndroidHasMotorControl = TRUE;
			}	
			else
			{
				if( m_AndroidHasMotorControl )
				{
					// Android had control, but now released (or dropped connection!). Force a stop
					m_pDriveCtrl->SetSpeedAndTurn( OVERRIDE_MODULE, SPEED_STOP, TURN_CENTER );
					m_AndroidHasMotorControl = FALSE;
				}
				else if( m_pDriveCtrl->IsOwner(OVERRIDE_MODULE) )
				{
					// On a previous pass, we took over control via OVERRIDE
					// Now, reset back to normal operation
					m_pDriveCtrl->ReleaseOwner( OVERRIDE_MODULE );
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
				m_pDriveCtrl->SetMoveDistance( m_UserOwner, SPEED_FWD_MED_SLOW, TURN_CENTER, wParam);
			}
			else
			{
				m_pDriveCtrl->SetMoveDistance( m_UserOwner, SPEED_REV_MED_SLOW, TURN_CENTER, wParam);
			}
		}
		break;


		case WM_ROBOT_TURN_SET_DISTANCE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// wParam = Turn Amount in degrees, 
			// lParam = direction (Left/Right) and speed
			
			CString MsgString;
			MsgString.Format( "TURN_SET_DISTANCE: %d Degrees", wParam );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			if( LEFT == lParam )
			{
				m_pDriveCtrl->SetTurnRotation( m_UserOwner, SPEED_STOP, (int)lParam, wParam); // Speed of turn, turn amount
			}
			else
			{
				m_pDriveCtrl->SetTurnRotation( m_UserOwner, SPEED_STOP, (int)lParam, wParam);
			}
		}
		break;

		case WM_ROBOT_SET_PATH_SPEED_INCREASE:
		{
			g_bCmdRecognized = TRUE;
			CString MsgString;
			MsgString.Format("Setting Path Speed Increase to %u", lParam);
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			g_SpeedIncrease = (int )lParam;
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
				m_pDriveCtrl->Brake( m_UserOwner );
				// m_pDriveCtrl->ExecuteCommand();	// Don't wait for next status update
			}
		}
		break;

		case WM_ROBOT_USER_OVERRIDE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// This command only comes from a user doing manual control
			// Allows user to override higher priority modules, or release control when done.
			// Example: SendCommand( WM_ROBOT_USER_OVERRIDE_CMD, SET_USER_OVERRIDE_AND_STOP, 0 ); // FORCE STOP, no matter what!
			// wParam = action to take

			/*if( SET_USER_RELEASED == wParam )
			{
				// Release user control
				m_pDriveCtrl->ReleaseOwner( LOCAL_USER_MODULE );
				m_UserOwner = LOCAL_USER_MODULE;	// back to normal user mode
				m_pDriveCtrl->EnableModule( COLLISION_MODULE );
				m_pDriveCtrl->EnableModule( AVOID_OBJECT_MODULE );
				m_pDriveCtrl->EnableModule( WAY_POINT_NAV_MODULE );
				m_pDriveCtrl->EnableModule( GRID_NAV_MODULE );
				m_pDriveCtrl->ReleaseOwner( OVERRIDE_MODULE );
			}
			else
			*/
			if( SET_USER_NORMAL == wParam )
			{
				// Normal mode.  User can override lesser modules, but Avoid and Collision will act
				ROBOT_DISPLAY( TRUE, " USER_OVERRIDE_CMD: Mode set to USER_NORMAL\n" )

					
				m_pDriveCtrl->ReleaseOwner( OVERRIDE_MODULE );
				m_UserOwner = LOCAL_USER_MODULE;	// back to normal user mode
				//m_pDriveCtrl->EnableModule( COLLISION_MODULE );
				//m_pDriveCtrl->EnableModule( AVOID_OBJECT_MODULE );
				//m_pDriveCtrl->SuppressModule( WAY_POINT_NAV_MODULE );
				//m_pDriveCtrl->SuppressModule( GRID_NAV_MODULE );
			}
			else if( SET_USER_OVERRIDE == wParam )
			{
				// furture user commands will override all other modules
				ROBOT_DISPLAY( TRUE, " USER_OVERRIDE_CMD: Mode set to USER_OVERRIDE MODE!\n" )
				m_UserOwner = OVERRIDE_MODULE;	// Super user mode

				// ??? this will force the other modules to abort and go back to IDLE state:
				//m_pDriveCtrl->SuppressModule( COLLISION_MODULE );
				//m_pDriveCtrl->SuppressModule( AVOID_OBJECT_MODULE );
				//m_pDriveCtrl->SuppressModule( WAY_POINT_NAV_MODULE );
				//m_pDriveCtrl->SuppressModule( GRID_NAV_MODULE );
			}
			else if( SET_USER_OVERRIDE_AND_STOP == wParam )
			{
				// Take control from all other modules and Stop (User already higher priority then Nav modules, so no need to supress them)
				ROBOT_LOG( TRUE, " USER_OVERRIDE_CMD: Mode set to USER_OVERRIDE_AND_STOP!\n" )
				// ?? this will force the other modules to abort and go back to IDLE state:

				m_UserOwner = OVERRIDE_MODULE;	// Super user mode
				//m_pDriveCtrl->SuppressModule( COLLISION_MODULE );
				//m_pDriveCtrl->SuppressModule( AVOID_OBJECT_MODULE );
				//m_pDriveCtrl->SuppressModule( WAY_POINT_NAV_MODULE );
				//m_pDriveCtrl->SuppressModule( GRID_NAV_MODULE );
				m_CurrentSpeed = SPEED_STOP;
				m_CurrentTurn = TURN_CENTER;
				m_pDriveCtrl->SetSpeedAndTurn( m_UserOwner, m_CurrentSpeed, m_CurrentTurn );


			}
			else
			{
				ROBOT_ASSERT(0);	// Logic Error
			}
		}
		break;

		case WM_ROBOT_JOYSTICK_DRIVE_CMD:
		{
			// This command only comes from a user doing manual control.
			// lParam is Speed, wParam is Turn
			// ROBOT_DISPLAY( TRUE, "Server ACK: JOYSTICK DRIVE CMD" )
			g_bCmdRecognized = TRUE;
			int NewSpeed = wParam;
			int NewTurn = lParam;


			// Speed and Turn commands from GUI are +/- 127, with zero = Center/Stop 
			// Convert to HW command values, for reverse, we add a negative number

			//-------------------------------------------------------------------------------
			#if( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
				// RC Car with external servo control: Servo max values are 0-255

				m_CurrentSpeed = SPEED_STOP + (NewSpeed);	 // convert to servo range
				if( m_CurrentSpeed > SPEED_FULL_FWD )			// Limit to valid range
				{
					m_CurrentSpeed = SPEED_FULL_FWD;	
				}
				else if( m_CurrentSpeed < SPEED_FULL_REV )
				{
					m_CurrentSpeed = SPEED_FULL_REV;	
				}

				m_CurrentTurn = (NewTurn);	// convert to servo range
				if( m_CurrentTurn < TURN_LEFT_MAX )			// Limit to valid range
				{
					m_CurrentTurn = TURN_LEFT_MAX;	
				}
				else if( m_CurrentTurn > TURN_RIGHT_MAX )
				{
					m_CurrentTurn = TURN_RIGHT_MAX;	
				}

				//int test = (TURN_CENTER - m_CurrentTurn);
				m_pDriveCtrl->SetSpeedAndTurn( m_UserOwner, m_CurrentSpeed, m_CurrentTurn );

			#else
				m_CurrentSpeed = NewSpeed;
				m_CurrentTurn = NewTurn;
				m_pDriveCtrl->SetSpeedAndTurn( m_UserOwner, m_CurrentSpeed, m_CurrentTurn );
			#endif

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
			g_SensorStatus.OdometerTenthInches = 0;
			g_SensorStatus.OdometerUpdateTenthInches = 0;
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

				FPOINT3D_T ArmXYZ;
				if( LEFT_ARM == wParam )
				{
					m_pArmControlLeft->GetTargetArmXYZ( ArmXYZ );
					//ROBOT_LOG( TRUE, "DEBUG: RIGHT ARM X = %4.1f, Y = %4.1f, Z = %4.1f\n", ArmX, ArmY, ArmZ )
				}
				else
				{
					m_pArmControlRight->GetTargetArmXYZ( ArmXYZ );
					//ROBOT_LOG( TRUE, "DEBUG: LEFT ARM X = %4.1f, Y = %4.1f, Z = %4.1f\n", ArmX, ArmY, ArmZ )
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
			g_SensorStatus.AuxLightsOn = (BOOL)lParam; // keep track of current state
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



