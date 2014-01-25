// SpeechReco.cpp
// Handle speech phrases detected by the Kinect C# App
// This class is used by the thread loop that reads commands from the Mapped Memory queue in thread.cpp
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include "thread.h"
//#include "module.h"
#include "Globals.h"
#include "SpeechEnums.cs"
#include "SpeechReco.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// GPA TRACE Macros
__itt_string_handle* pshInitSpeechReco = __itt_string_handle_create("InitSpeechReco");
//__itt_string_handle* pshSpeaking = __itt_string_handle_create("Speaking");
__itt_string_handle* pshCmdProcess = __itt_string_handle_create("CmdProcess");
__itt_string_handle* pshCmdRecoEvent = __itt_string_handle_create("CmdRecoEvent");
__itt_string_handle* pshDictRecoEvent = __itt_string_handle_create("DictRecoEvent");
__itt_string_handle* pshSwitchRecoMode = __itt_string_handle_create("SwitchRecoMode");

__itt_string_handle* pshSendToAI = __itt_string_handle_create("SendToAI"); // marker
__itt_string_handle* pshRecoEnabledByGUI = __itt_string_handle_create("RecoEnabledByGUI"); // marker
__itt_string_handle* pshRecoDisabledByGUI = __itt_string_handle_create("RecoDisabledByGUI"); // marker
__itt_string_handle* pshRecoEnabled = __itt_string_handle_create("RecoEnabled"); // marker
__itt_string_handle* pshRecoDisabled = __itt_string_handle_create("RecoDisabled"); // marker
__itt_string_handle* pshRecoCmdMode = __itt_string_handle_create("RecoCmdMode"); // marker
__itt_string_handle* pshRecoDictMode = __itt_string_handle_create("RecoDictMode"); // marker

#define MIN_ALL        419
#define MIN_ALL_UNDO   416


/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_TREX_READWRITE_TIME  // Log Read/Write SIO latency

/////////////////////////////////////////////////////

// Pre-defined Waypoint Locations to go to
#define WAYPOINT_OFFICE_X				341
#define WAYPOINT_OFFICE_Y				248

#define WAYPOINT_MASTER_BEDROOM_X		282
#define WAYPOINT_MASTER_BEDROOM_Y		580

#define WAYPOINT_HEATHER_BEDROOM_X		291
#define WAYPOINT_HEATHER_BEDROOM_Y		448

#define WAYPOINT_AMBER_BEDROOM_X		466
#define WAYPOINT_AMBER_BEDROOM_Y		236

#define WAYPOINT_MASTER_BATHROOM_X		368
#define WAYPOINT_MASTER_BATHROOM_Y		655

#define SPEECH_MIC_TIMEOUT				15000 // MS

/////////////////////////////////////////////////////////////////////////////
// For Debugging Speech, enable / disable actual command execution here
//#define DISABLE_SPEECH_ACTIONS		1
void SpeechSendCommand( DWORD Cmd, DWORD Param1, DWORD Param2 )
{
	#if ( 1 != DISABLE_SPEECH_ACTIONS )
		SendCommand( Cmd, Param1, Param2 );
	#else
		ROBOT_DISPLAY( TRUE, "WARNING: DISABLE_SPEECH_ACTIONS == 1")
	#endif
}


///////////////////////////////////////////////////////////////////////////////
// Robot Speech Recognition Class
///////////////////////////////////////////////////////////////////////////////
CRobotSpeechReco::CRobotSpeechReco()
{
	m_ProcessMicOnOnly = FALSE;
	m_bSendRecoToAI = FALSE;
	m_bEnableMotorsForSpeechCommands = FALSE;
	m_bPlaySimonSays = FALSE;
	m_bSimonSays = FALSE;
	m_PhraseNumber = 0;

	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
//	m_pHeadControl = new HeadControl();

}

CRobotSpeechReco::~CRobotSpeechReco()
{
	SAFE_DELETE( m_pArmControlRight );
	SAFE_DELETE( m_pArmControlLeft );
//	SAFE_DELETE( m_pHeadControl );

}

void CRobotSpeechReco::Init()
{
	__itt_task_begin(pDomainSpeechRecoThread, __itt_null, __itt_null, pshInitSpeechReco);

	ROBOT_LOG( TRUE, "\n\n======================== SPEECH RECO INITIALIZATION  ============================= \n" )
	HRESULT hr = S_OK;
	RobotSleep(200, pDomainSpeechRecoThread);

	ROBOT_LOG( TRUE, "\n\n=================== SPEECH RECO RECOGNITION INITIALIZATION COMPLETE ========================== \n\n\n" )

	__itt_task_end(pDomainSpeechRecoThread);
}


void CRobotSpeechReco::CmdToString( UINT Command, CString &CmdString )
{
	switch( Command )
	{
        case SpeechCmd_None:			CmdString = "None";				break;
        case SpeechCmd_RobotName:		CmdString = "RobotName";		break;
        case SpeechCmd_Stop:			CmdString = "Stop";				break;
        case SpeechCmd_Yes:				CmdString = "Yes";				break;
        case SpeechCmd_No:				CmdString = "No";				break;
        case SpeechCmd_Wave:			CmdString = "Wave";				break;
        case SpeechCmd_SayHi:			CmdString = "SayHi";			break;
        case SpeechCmd_SayBye:			CmdString = "SayBye";			break;
        case SpeechCmd_SendEmail:		CmdString = "SendEmail";		break;
        case SpeechCmd_TakePhoto:		CmdString = "TakePhoto";		break;
        case SpeechCmd_DoIntro:			CmdString = "DoIntro";			break;
        case SpeechCmd_LookForward:		CmdString = "LookForward";		break;
        case SpeechCmd_Follow:			CmdString = "Follow";			break;
        case SpeechCmd_ComeHere:		CmdString = "ComeHere";			break;
        case SpeechCmd_TurnTowardsMe:	CmdString = "TurnTowardsMe";	break;
        case SpeechCmd_DoWhatTime:		CmdString = "DoWhatTime";		break;
        case SpeechCmd_FaceMe:			CmdString = "FaceMe";			break;
        case SpeechCmd_CleanUp:			CmdString = "CleanUp";			break;
        case SpeechCmd_Shake:			CmdString = "Shake";			break;
        case SpeechCmd_MeetName:		CmdString = "MeetName";			break;
        case SpeechCmd_Wake:			CmdString = "Wake";				break;
        case SpeechCmd_Mic:				CmdString = "Mic";				break;
        case SpeechCmd_Lights:			CmdString = "Lights";			break;
        case SpeechCmd_Move:			CmdString = "Move";				break;
		case SpeechCmd_Explore:			CmdString = "Explore";			break;
        case SpeechCmd_Spin:			CmdString = "Spin";				break;
        case SpeechCmd_Turn:			CmdString = "Turn";				break;
        case SpeechCmd_SmallTurn:		CmdString = "SmallTurn";		break;
        case SpeechCmd_EnableMotors:	CmdString = "EnableMotors";		break;
        case SpeechCmd_EnableAvoidance:	CmdString = "EnableAvoidance";	break;
        case SpeechCmd_ArmHome:			CmdString = "ArmHome";			break;
        case SpeechCmd_ArmUp:			CmdString = "ArmUp";			break;
        case SpeechCmd_ExtendArm:		CmdString = "ExtendArm";		break;
        case SpeechCmd_Claw:			CmdString = "Claw";				break;
        case SpeechCmd_TakeObject:		CmdString = "TakeObject";		break;
        case SpeechCmd_GiveObject:		CmdString = "GiveObject";		break;
        case SpeechCmd_PickUpObject:	CmdString = "PickUpObject";		break;
        case SpeechCmd_PutObjectDown:	CmdString = "PutObjectDown";	break;
        case SpeechCmd_PutInBasket:		CmdString = "PutInBasket";		break;
        case SpeechCmd_ThrowObject:		CmdString = "ThrowObject";		break;
        case SpeechCmd_ScratchHead:		CmdString = "ScratchHead";		break;
        case SpeechCmd_ScratchBack:		CmdString = "ScratchBack";		break;
        case SpeechCmd_MeaningOfLife:	CmdString = "MeaningOfLife";	break; // TODO-MUST Not implemented???
        case SpeechCmd_Karate:			CmdString = "Karate";			break;
        case SpeechCmd_DoDanger:		CmdString = "DoDanger";			break;
        case SpeechCmd_LightSaber:		CmdString = "LightSaber";		break;
        case SpeechCmd_FaceCompass:		CmdString = "FaceCompass";		break;
        case SpeechCmd_PointCompass:	CmdString = "PointCompass";		break;

		case SpeechCmd_IdentifyObj:		CmdString = "IdentifyObj";		break;
        case SpeechCmd_GotoLocation:	CmdString = "GotoLocation";		break;
        case SpeechCmd_HaveNotHeardName: CmdString = "HaveNotHeardName";break;
        case SpeechCmd_TellJoke:		CmdString = "TellJoke";			break;
        case SpeechCmd_BadRobot:		CmdString = "BadRobot";			break;

		default:
			CmdString.Format("UNKNOWN COMMAND (%d, %04Xh)", Command, Command);

	}
}

////////////////////////////////////////////////////////////////////////////////////////////////
#define USER_NAME_LEN 132

void CRobotSpeechReco::Speak( const char* TextToSay )
{
	SpeakText( TextToSay );
}

BOOL CRobotSpeechReco::MovementEnabled()
{
	if( !m_bEnableMotorsForSpeechCommands )
	{
		ROBOT_DISPLAY( TRUE, "SpeechReco Command Ignored: Movement Disabled\n" )
		Speak( "Sorry, my motors are disabled" );
		return FALSE;
	}

	if( g_GlobalPause )
	{
		ROBOT_DISPLAY( TRUE, "SpeechReco Command Ignored: Pause Active\n" )
		Speak( "Sorry, I am Frozen" );
		return FALSE;
	}

	return TRUE;
}

void CRobotSpeechReco::HandleRecognition( 
		RECO_TYPE RecoType, // Command, Question, Statement, ...
		int Param1,	int Param2, int Param3, int Param4, float SpeechRecoConfidence )
{
	if( g_SpeechRecoBlocked )
	{
		// All speech reco, including Stop, Blocked!  Used when arm motors in motion, due to noise picked up by Kinect Mics
		ROBOT_DISPLAY( TRUE, "SpeechReco Command Ignored: Reco Blocked (Arm Motors?)\n" )
		return;
	}

	__itt_task_begin(pDomainSpeechRecoThread, __itt_null, __itt_null, pshCmdProcess);

	switch( (int)RecoType )
	{
		case RecoType_Command:
			HandleCommandRecognition( (SPEECH_CMD)Param1, // Param1 is the actual command
				Param2, Param3, Param4, SpeechRecoConfidence );
		break;

		case RecoType_Question:
			ROBOT_LOG( TRUE, "RecoType_Question NOT IMPLEMENTED!/n" )
			HandleQuestionRecognition( Param1, Param2, Param3, Param4, SpeechRecoConfidence );
		break;

		case RecoType_Statement:
			ROBOT_LOG( TRUE, "RecoType_Statement NOT IMPLEMENTED!/n" )
		break;

		case RecoType_None:
			ROBOT_LOG( TRUE, "RecoType_None passed in!/n" )
		break;

		default:
			ROBOT_LOG( TRUE, "RecoType_ BAD VALUE!/n" )
		break;
	}

	__itt_task_end(pDomainSpeechRecoThread);
}

// NEW /////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CRobotSpeechReco::HandleQuestionRecognition( 
		int Param1,	int Param2, int Param3, int Param4, float SpeechRecoConfidence )
{

//	__itt_task_begin(pDomainSpeechRecoThread, __itt_null, __itt_null, pshCmdProcess);
	CString strText;
	int RandomNumber = ((3 * rand()) / RAND_MAX);

	CString CmdString;
	strText.Format(_T("Speech Reco: Got Speech Question: %d, %d, %d, %d \n"), Param1, Param2, Param3, Param4 );
	ROBOT_LOG( TRUE, (LPCTSTR)strText )

	switch( Param1 )
	{
		case SpeechCmd_TellJoke:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Question Recognized: Tell Jokes")
			BOOL MultipleJokes = FALSE;
			if( SpeechParam_Multiple == Param2 )
			{
				MultipleJokes = TRUE;
			}
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TELL_JOKES, (DWORD)MultipleJokes );	
			break;
		}
		default:
		{
			CString strErrorMsg;
			strErrorMsg.Format("SpeechReco Question Ignored: Unknown Question %d (%04X)!\n", Param1, Param1 );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)strErrorMsg )
			//ROBOT_ASSERT(0);
		}
	}
	
}
// NEW /////////////////////////////////////////////////////////////////////////////////////////////////////////////


void CRobotSpeechReco::HandleCommandRecognition( 
		SPEECH_CMD Command,	int Param2, int Param3, int Param4, float SpeechRecoConfidence )
//	( DWORD wParam, DWORD lParam )
{

	__itt_task_begin(pDomainSpeechRecoThread, __itt_null, __itt_null, pshCmdProcess);
	CString strText;

	if( SpeechCmd_Stop == Command )
	{
		// Send immediate command to stop the robot!
		PostThreadMessage( g_dwMotorCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP), 0, 0 );
		RobotSleep(5, pDomainSpeakThread); // allow motor thread to run
	}

	CString CmdString;
	CmdToString( Command, CmdString );
	strText.Format(_T("Speech Reco: GOT Speech COMMAND: %d: %s\n"), Command, CmdString );
	ROBOT_LOG( TRUE, (LPCTSTR)strText )

	if( (Command != SpeechCmd_Mic)  && 
		(Command != SpeechCmd_RobotName) && 
		(Command != SpeechCmd_Stop) )
	{
		// Some command other than "Loki", "Mic On", "Mic Off" "Pause" or "Stop"
		if( m_ProcessMicOnOnly )
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Ignored: Microphone Disabled\n" )
			__itt_task_end(pDomainSpeechRecoThread);
			return;
		}
	}

/*	if( Command == SpeechCmd_SimonSays )
	{
		ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Simon Says...")
		m_bSimonSays = TRUE;	// got the "Simon says" key phrase
		__itt_task_end(pDomainSpeechRecoThread);
		return;
	}
*/
	// Play Simon Says (except for critical commands)
/*** TODO_MUST. DISABLED FOR NOW!

if( m_bPlaySimonSays && !m_bSimonSays )
	{
		// We are playing, but user forgot to say "Simon Says"
		if( (Command != SpeechCmd_MicrophoneOn)		&& (Command != SpeechCmd_MicrophoneOff)	&&
			(Command != SpeechCmd_EnableMovement)		&& (Command != SpeechCmd_DisableMovement)	&&
			(Command != SpeechCmd_PlaySimonSaysOff)	&& (Command != SpeechCmd_ExploreStop)		&&
			(Command != SpeechCmd_Loki)				&& (Command != SpeechCmd_Stop)			&& 
			(Command != SpeechCmd_SmallTurnLeft)			&& (Command != SpeechCmd_SmallTurnRight)		&& 
			(Command != SpeechCmd_Pause)				&& (Command != SpeechCmd_Resume)			&& 
			(Command != SpeechCmd_GoStraight)			&& (Command != SpeechCmd_TalkingEnabled)  )
		{
			ROBOT_DISPLAY( TRUE, "You didn't say Simon Says!\n" )
			Speak( "You didn't say Simon Says!" );	
			__itt_task_end(pDomainSpeechRecoThread);
			return;
		}
	}
***/	

	switch( Command )
	{
		case SpeechCmd_RobotName:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Robot's name called")
			m_ProcessMicOnOnly = FALSE;

			// Respond with random phrases
			int RandomNumber = ((6 * rand()) / RAND_MAX);
			ROBOT_LOG( TRUE, "DEBUG: Loki Name Called RAND = %d\n", RandomNumber)
			switch( RandomNumber )
			{
				case 0: Speak( "Yes?" );break;
				case 1: Speak( "I am all ears" );break;
				case 2: Speak( "What?" );break;
				default: Speak( "Yes?" ); // If const is larger number, this gets called more often
			}
			break;
		}

		case SpeechCmd_Mic:
		{
			if( SpeechParam_True == Param2 )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Microphone On")
				m_ProcessMicOnOnly = FALSE;

				// Respond with random phrases
				int RandomNumber = ((6 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: Mic On RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0: Speak( "Microphone On" );break;
					case 1: Speak( "I am all ears" );break;
					case 2: Speak( "Microphone Enabled" );break;
					default: Speak( "OK, I am listening" ); // If const is larger number, this gets called more often
				}
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Microphone Off")
				m_ProcessMicOnOnly = TRUE;
				int RandomNumber = ((6 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: Mic Off RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0: Speak( "Microphone Off" );break;
					case 1: Speak( "OK, I will mind my own business" );break;
					case 2: Speak( "Microphone Disabled" );break;
					default: Speak( "OK, I won't listen" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}

		case SpeechCmd_EnableMotors:
		{
			if( SpeechParam_True == Param2 )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Enable Movement")
				m_bEnableMotorsForSpeechCommands = TRUE;
				Speak( "Ok, I have enabled my wheel motors" );	
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Disable Movement")
				m_bEnableMotorsForSpeechCommands = FALSE;
				g_MotorCurrentSpeedCmd = SPEED_STOP;
				g_MotorCurrentTurnCmd = 0;	// Center
				SpeechSendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 ); // FORCE STOP, no matter what!
				RobotSleep(5, pDomainSpeakThread); // Let the command run first
				Speak( "Ok, I have disabled my wheel motors" );	
			}
			break;
		}

		case SpeechCmd_Stop:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Stop")
			BOOL bCurrentlyMoving = (SPEED_STOP != g_MotorCurrentSpeedCmd) || (0 != g_MotorCurrentTurnCmd);
			// Note that since this is the STOP command, we don't care about m_bEnableMotorsForSpeechCommands
			g_MotorCurrentSpeedCmd = SPEED_STOP;
			g_MotorCurrentTurnCmd = 0;	// Center
			SpeechSendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 ); // FORCE STOP, no matter what!
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, ACTION_MODE_NONE, (DWORD)0 );	// Right/Left arm, Movement to execute, TODO-MUST This is not what Behavior code says!!
			RobotSleep(5, pDomainSpeakThread); // Let the command run first

			// Tell head to stop moving if it was
			// SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_STOP, 5 );

			if( bCurrentlyMoving )
			{
				Speak( "Stopping" );	// Acknowledge if we were moving
			}
			else
			{
				Speak( "I have stopped" );	// Acknowledge if we were not moving
			}

			// Tell arms to stop and go to home postion too
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute, 


			SpeechSendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_REMOTE, 0 ); // Rest owner back
			break;
		}

/*		case SpeechCmd_Pause:
		{
			g_GlobalPause = TRUE;
			RobotSleep(20, pDomainSpeakThread); // allow other threads to run, for fast response
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Pause / Freeze")
			break;
		}

		case SpeechCmd_Resume:
		{
			g_GlobalPause = FALSE;
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Resume")
			Speak( "Resuming" );	
			break;
		}
*/


		case SpeechCmd_Yes: // Continue previous command, if one was pending
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Yes")
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_YES_NO_RESPONSE_RECEIVED, TRUE );
			break;
		}

		case SpeechCmd_No: 
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: No")
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_YES_NO_RESPONSE_RECEIVED, FALSE );
			break;
		}


		case SpeechCmd_Explore:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Explore/Go")
			if( MovementEnabled() )
			{
				BOOL CurrentlyMoving = FALSE;
				if( (TURN_CENTER != g_MotorCurrentTurnCmd) || (SPEED_STOP != g_MotorCurrentSpeedCmd) )
				{
					CurrentlyMoving = TRUE;
				}

				// Tell robot to go straight forward
				g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
				g_MotorCurrentTurnCmd = 0;	// Center

				if( CurrentlyMoving )
				{
					// Currently turning, so assume user wants to go straight now
					Speak( "Going Straight" );
				}
				else
				{

					// TODO - Enable avoidance automatically?
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Explore: Avoid Objects Enabled!")
					SpeechSendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable

					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "Here I go!" );break;
						case 1:  Speak( "look out world, here I come" );break;
						case 2:  Speak( "I am off to see the wizard" );break;
						default: Speak( "I like exploring" ); // If const is larger number, this gets called more often
					}
				}
			}
			break;
		}

		case SpeechCmd_Move:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Move Forward")
			if( MovementEnabled() )
			{
				if( SpeechParam_Forward == Param2 )
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Move Forward")
					// Respond to the command with action, then talk
					const DWORD DistTenthInches = 120; // move set distance forward
					SpeechSendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, DistTenthInches, FORWARD );	// wParam = distance in TENTH INCHES, lParam = direction
					//RobotSleep(500, pDomainSpeakThread);
					// Respond with random phrases
					int RandomNumber = ((4 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "OK" );break;
						case 1:  Speak( "all righty" );break;
						case 3:  Speak( "how's this" );break;
						default: Speak( "look out" ); // If const is larger number, this gets called more often
					}
				}
				else if( SpeechParam_Reverse == Param2 )
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Move Back")
					// Respond to the command with action, then talk
					const DWORD DistTenthInches = 90;
					SpeechSendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, DistTenthInches, REVERSE );	// wParam = distance in TENTH INCHES, lParam = direction
					RobotSleep(1000, pDomainSpeakThread); // Allow action to start before talking

					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "Look out behind me" );break;
						case 1:  Speak( "Backing up" );break;
						default: Speak( "How's this" ); // If const is larger number, this gets called more often
					}
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "ERROR!  BAD PARAM for MOVE command")
				}
			}
			break;
		}

		case SpeechCmd_FaceMe:
		{
			if( MovementEnabled() )
			{
				if( g_LastHumanCompassDirection < 0 )
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command: Face Me - ERROR, g_LastHumanCompassDirection < 0") // no human previously found
					Speak( "I am having a problem detecting your location" );
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Face Me")
					SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TURN_TO_COMPASS_DEGREES, g_LastHumanCompassDirection );
				}
			}
			break;
		}

		case SpeechCmd_PointCompass:
		case SpeechCmd_FaceCompass:
		{
			int Direction = -1; // invalid direction
			switch( Param2 )
			{
				case SpeechParam_North:
					Direction = NORTH;
					break;
				case SpeechParam_East:
					Direction = EAST;
					break;
				case SpeechParam_South:
					Direction = SOUTH;
					break;
				case SpeechParam_West:
					Direction = WEST;
					break;
			}
			if( -1 == Direction )
			{
				ROBOT_DISPLAY( TRUE, "ERROR Point or Face Compass Direction, BAD PARAM2 =  %d", Param2)
			}
			else if( MovementEnabled() )
			{
				if( SpeechCmd_PointCompass == Command )
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Point Compass Direction %d", Direction)
					SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_POINT_TO_COMPASS_DIR, Direction );
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Face Compass Direction %d", Direction)
					SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TURN_TO_COMPASS_DIR, Direction );
				}
			}
			break;
		} // case Point/FaceCompass

		case SpeechCmd_Spin: // 180 degree turn
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Spin Right/Left")
			if( MovementEnabled() )
			{
				// Respond to the command with action, then talk
				// Spin in place
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 180, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
					RobotSleep(500, pDomainSpeakThread);
					// Respond with random phrases
					int RandomNumber = ((5 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "You spin me round round baby" );break;
						case 1:  Speak( "I think I am getting dizzy" );break;
						case 2:  Speak( "Check this out" );break;
						case 3:  Speak( "This is my good side" );break;
						default: Speak( "This is fun" ); // If const is larger number, this gets called more often
					}
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 180, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
					RobotSleep(500, pDomainSpeakThread);
					// Respond with random phrases
					int RandomNumber = ((5 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "Spinning Right" );break;
						case 1:  Speak( "I like this" );break;
						case 2:  Speak( "You spin me round round baby" );break;
						case 3:  Speak( "This is NOT my good side" );break;
						default: Speak( "How do I look" ); // If const is larger number, this gets called more often
					}
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "ERROR Bad Direction for Spin Command")
				}
			}
			break;
		}

		case SpeechCmd_Turn: // 90 degree turn
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Turn Right/Left")
			if( MovementEnabled() )
			{
				// Respond to the command with action, then talk
				if( SpeechParam_Left == Param2 )
				{
					// Respond to the command with action, then talk
					if( SPEED_STOP == g_MotorCurrentSpeedCmd )
					{
						// Not currently moving, so assume user wants to turn in place
						SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 90, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
						RobotSleep(500, pDomainSpeakThread);
						// Respond with random phrases
						int RandomNumber = ((3 * rand()) / RAND_MAX);
						ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
						switch( RandomNumber )
						{
							case 0:  Speak( "Turning Left" );break;
							case 1:  Speak( "How is this" );break;
							default: Speak( "Now what?" ); // If const is larger number, this gets called more often
						}
					}
					else
					{
						// Currently moving, so assume use wants to curve turn.
						g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
						g_MotorCurrentTurnCmd = -(g_SpeedSetByKeyboard/2);
						if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
						{
							g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
						}
						RobotSleep(200, pDomainSpeakThread);
						Speak( "Turn Left" );
					}
				}
				else if( SpeechParam_Right == Param2 )
				{
					// Respond to the command with action, then talk
					if( SPEED_STOP == g_MotorCurrentSpeedCmd )
					{
						// Not currently moving, so assume user wants to turn in place
						SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 90, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
						RobotSleep(500, pDomainSpeakThread);
						// Respond with random phrases
						int RandomNumber = ((3 * rand()) / RAND_MAX);
						ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
						switch( RandomNumber )
						{
							case 0:  Speak( "Turning Right" );break;
							case 1:  Speak( "How is this" );break;
							default: Speak( "Now what?" ); // If const is larger number, this gets called more often
						}
					}
					else
					{
						// Currently moving, so assume use wants to curve turn.
						g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
						g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard/2);
						if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
						{
							g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
						}
						RobotSleep(200, pDomainSpeakThread);
						Speak( "Turn Right" );
					}
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "ERROR Bad Direction for Turn Command")
				}
			}
			break;
		}


		case SpeechCmd_SmallTurn: // ~45 degree turn, or curve if moving
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Small Turn Right/Left")
			if( MovementEnabled() )
			{
				// Respond to the command with action, then talk
				if( SpeechParam_Left == Param2 )
				{
					// Respond to the command with action, then talk
					if( SPEED_STOP == g_MotorCurrentSpeedCmd )
					{
						// Not currently moving, so assume user wants to turn in place
						SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 45, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
						RobotSleep(500, pDomainSpeakThread);
						// Respond with random phrases
						int RandomNumber = ((3 * rand()) / RAND_MAX);
						ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
						switch( RandomNumber )
						{
							case 0:  Speak( "OK" );break;
							case 1:  Speak( "How is this" );break;
							default: Speak( "Now what?" ); // If const is larger number, this gets called more often
						}
					}
					else
					{
						// Currently moving, so assume use wants to curve turn.
						g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
						g_MotorCurrentTurnCmd = -(g_SpeedSetByKeyboard/4);
						if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
						{
							g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
						}
						RobotSleep(200, pDomainSpeakThread);
						Speak( "Bear Left" );
					}
				}
				else if( SpeechParam_Right == Param2 )
				{
					// Respond to the command with action, then talk
					if( SPEED_STOP == g_MotorCurrentSpeedCmd )
					{
						// Not currently moving, so assume user wants to turn in place
						SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 45, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
						RobotSleep(500, pDomainSpeakThread);
						// Respond with random phrases
						int RandomNumber = ((3 * rand()) / RAND_MAX);
						ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
						switch( RandomNumber )
						{
							case 0:  Speak( "OK" );break;
							case 1:  Speak( "How is this" );break;
							default: Speak( "Now what?" ); // If const is larger number, this gets called more often
						}
					}
					else
					{
						// Currently moving, so assume use wants to curve turn.
						g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
						g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard/4);
						if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
						{
							g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
						}
						RobotSleep(200, pDomainSpeakThread);
						Speak( "Bear Right" );
					}
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "ERROR Bad Direction for Small Turn Command")
				}
			}
			break;
		}
		
		

/*
		case SpeechCmd_LookUp:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Look Up")
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_UP, 5 );
			break;
		}

		case SpeechCmd_LookDown:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Look Down")
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_DOWN, 5 );
			break;
		}

		case SpeechCmd_LookLeft:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Look Left")
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_LEFT, 5 );
			break;
		}

		case SpeechCmd_LookRight:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Look Right")
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_RIGHT, 5 );
			break;
		}
*/

		case SpeechCmd_LookForward:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Look Forward")
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 );
			SpeechSendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_CENTER, 5 );
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Nice" );break;
					case 1:  Speak( "That's better" );break;
					//default: // say nothing							// If const is larger number, this gets called more often
				}

			break;
		}

		case SpeechCmd_EnableAvoidance:
		{
			if( SpeechParam_True == Param2 )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Avoid Objects On")
				SpeechSendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable
				Speak( "OK, I will avoid objects" );	
			}
			else if( SpeechParam_False == Param2 )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Avoid Objects Off")
				SpeechSendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 0, 0 );	// Disable
				Speak( "Disabling Object avoidance" );	
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param for Avoid Objects")
			}
			break;
		}
/*
		case SpeechCmd_CameraTrackFaceOn:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: CameraTrack Face On")
			SpeechSendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_FACE, (DWORD)TRUE );
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "I am looking now" );break;
					case 1:  Speak( "ok, I'm looking" );break;
					case 2:  Speak( "face tracking enabled" );break;
					default: Speak( "looking" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_CameraTrackFaceOff:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Camera Track Face Off")
			SpeechSendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_FACE, (DWORD)FALSE );
			Speak( "Face tracking off" );	
			break;
		}
*/
		case SpeechCmd_ArmHome:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Home")

			if( SpeechParam_Left == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute
			}
			else if( SpeechParam_Right == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param for Arm Home = %d", Param2)
			}

			RobotSleep(10, pDomainSpeakThread);
			// Respond with random phrases
			int RandomNumber = ((5 * rand()) / RAND_MAX);
			ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
			switch( RandomNumber )
			{
				case 0:  Speak( "My arm was getting tired" );break;
				case 1:  Speak( "Will comply" );break;
				case 2:  Speak( "That's better" );break;
				default: Speak( "Arm Home" ); // If const is larger number, this gets called more often
			}
			break;
		}

		case SpeechCmd_TakeObject:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Take Object")
			if( SpeechParam_Left == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_TAKE_OBJECT );	// Right/Left arm, Movement to execute
			}
			else if( SpeechParam_Right == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_TAKE_OBJECT );	// Right/Left arm, Movement to execute
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param for Take Object = %d", Param2)
			}
			break;
		}

		case SpeechCmd_GiveObject:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Give Object")
			if( SpeechParam_Left == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_GIVE_OBJECT );	// Right/Left arm, Movement to execute
			}
			else if( SpeechParam_Right == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_GIVE_OBJECT );	// Right/Left arm, Movement to execute
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param for Give Object = %d", Param2)
			}
			break;
		}


		case SpeechCmd_ExtendArm:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Extend Arm")
			if( SpeechParam_Full != Param3 )
			{

				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW );	// Right/Left arm, Movement to execute
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW );	// Right/Left arm, Movement to execute
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param for Extend Arm = %d", Param2)
				}
			}
			else
			{
				// Extend arm fully
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_FULL );	// Right/Left arm, Movement to execute
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_FULL );	// Right/Left arm, Movement to execute
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param for Extend Arm Full = %d", Param2)
				}

				RobotSleep(10, pDomainSpeakThread);
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "reach out and touch someone" );break;
					case 1:  Speak( "I have long arms" );break;
					case 2:  Speak( "this makes me tired" );break;
					default: Speak( "this is a reach" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}
/*
		case SpeechCmd_ArmExtendClosedRight:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Extend Close Claw Right")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW );	// Right/Left arm, Movement to execute, 
			break;
		}
*/

		case SpeechCmd_Shake:	//implies Right hand
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Shake Hands")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );	// Right/Left arm, Movement to execute, 
			break;
		}

		case SpeechCmd_MeetName:	// Meet some new person - TODO add Name parsing!
		{
			// Currently ignores the name passed in by the recognizer
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Meet person")
			SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_CHAT_DEMO_WITH_ADULT, 1 ); 
			break;
		}
				
		case SpeechCmd_Claw:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Open/Close Claw")
			if( SpeechParam_Close == Param3 )
			{
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_CLOSE_CLAW_FULL );	// Right/Left arm, Movement to execute, 
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_CLOSE_CLAW_FULL );	// Right/Left arm, Movement to execute, 
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Claw = %d", Param2)
				}

				// Respond with random phrases
				int RandomNumber = ((2 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Ok, Now what?" );break;
					default: Speak( "Ok" ); // If const is larger number, this gets called more often
				}
			}
			else if( SpeechParam_Open == Param3 )
			{
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_OPEN_CLAW );	// Right/Left arm, Movement to execute, 
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_OPEN_CLAW );	// Right/Left arm, Movement to execute, 
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Claw = %d", Param2)
				}

				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "All right" );break;
					case 1:  Speak( "OK" );break;
					case 2:  Speak( "I am opening my hand" );break;
					default: Speak( "I am letting go" ); // If const is larger number, this gets called more often
				}
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param3 for Claw = %d", Param3)
			}
			break;
		}

		// *************** TODO:  DISABLED AUTO STUFF - LEFT ARM ONLY!!! *******************************
		case SpeechCmd_PutObjectDown:	// NOT: Behavior Module will try to auto detect which arm
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Put Object Down (ASSUME LEFT)")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PUT_DOWN_OBJECT );	// Right/Left arm, Movement to execute, 
				// Respond with random phrases
				int RandomNumber = ((5 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "All right" );break;
					case 1:  Speak( "Seems like a good idea" );break;
					case 2:  Speak( "I'll keep it close to me" );break;
					case 4:  Speak( "This should be a good place to put it" );break;
					default: Speak( "That is a good idea" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_CleanUp:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: CleanUp")
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_PICKUP_OBJECTS, (DWORD)1 );
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "A robots work is never done" );break;
					case 1:  Speak( "hi ho hi ho.  its off to work I go" );break;
					default: Speak( "Ok, I will look for things to pick up" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_PickUpObject:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Pick Up Object")
			if( SpeechParam_Left == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_PICKUP_CLOSE_OBJECT, (DWORD)1 ); 
			}
			else if( SpeechParam_Right == Param2 )
			{	// WARNING - USES LEFT not RIGHT
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_PICKUP_CLOSE_OBJECT, (DWORD)1 ); // TODO-MUST Behavior module only uses left hand
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Pickup Object = %d", Param2)
			}

			// Respond with random phrases
			int RandomNumber = ((5 * rand()) / RAND_MAX);
			ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				RandomNumber = 1;
			switch( RandomNumber )
			{
				case 0:  Speak( "All right" );break;
				case 1:  Speak( "Okie dokie" );break;
				case 2:  Speak( "sounds like fun" );break;
				case 4:  Speak( "Your wish is my command" );break;
				default: Speak( "I was just going to pick it up anyway" ); // If const is larger number, this gets called more often
			}
			break;
		}

		// *************** TODO:  DISABLED AUTO STUFF - LEFT ARM ONLY!!! *******************************
		case SpeechCmd_ThrowObject: 	// NOT: Behavior Module will try to auto detect which arm
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Throw Object")


			if( SpeechParam_Forward == Param2 )
			{
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_THROW_OBJECT_FRONT ); 
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Look out, here it comes" );break;
					case 1:  Speak( "I hope this does not break" );break;
					case 2:  Speak( "I'm not very good at this" );break;
					default: Speak( "Get ready to catch" ); // If const is larger number, this gets called more often
				}
			}
			else if( SpeechParam_Reverse == Param2 )
			{	
				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PUT_IN_BASKET );
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "easy come, easy go" );break;
					case 1:  Speak( "I will take care of this later" );break;
					case 3:  Speak( "OK, if you say so" );break;
					default: Speak( "Good idea" ); // If const is larger number, this gets called more often
				}
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Throw Object = %d", Param2)
			}
			break;
		}

		case SpeechCmd_PutInBasket:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Put in Basket")

				SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PUT_IN_BASKET );
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "easy come, easy go" );break;
					case 1:  Speak( "I will take care of this later" );break;
					case 3:  Speak( "OK, if you say so" );break;
					default: Speak( "Good idea" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_ArmUp: // one or Both Arms
		{
			if( SpeechParam_True == Param3 )
			{	// Arm(s) *UP*
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arms/Hands UP")
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_ARM_UP_FULL );	// Right/Left arm, Movement to execute, 
					RobotSleep(1000, pDomainSpeakThread); // give time for arm to raise 
					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0: Speak( "I come in peace" ); break;
						default:  Speak( "pick me.  Pick me" );// If const is larger number, this gets called more often
					}
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_ARM_UP_FULL );	// Right/Left arm, Movement to execute, 
					RobotSleep(1000, pDomainSpeakThread); // give time for arm to raise 
					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0: Speak( "I swear to tell the truth, the whole truth, and nothing but the truth" ); break;
						default:  Speak( "live long and prosper" );// If const is larger number, this gets called more often
					}
				}
				else if( SpeechParam_Both == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)ARM_MOVEMENT_ARM_UP_FULL );	// Right/Left arm, Movement to execute, 
					RobotSleep(1000, pDomainSpeakThread); // give time for arm to raise 
					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0: Speak( "Ok, Don't shoot" ); break;
						default:  Speak( "I am not the droid you are looking for" );// If const is larger number, this gets called more often
					}
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Arm Up = %d", Param2)
				}
			}
			else if( SpeechParam_False == Param3 )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Hand down")
				if( SpeechParam_Left == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute, 
				}
				else if( SpeechParam_Right == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute, 
				}
				else if( SpeechParam_Both == Param2 )
				{
					SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)ARM_MOVEMENT_HOME1 );	// Right/Left arm, Movement to execute, 
				}
				else
				{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param2 for Arm Down = %d", Param2)
				}
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "all right" ); break;
					default:  Speak( "that's better" );// If const is larger number, this gets called more often
				}
			}
			else
			{
					ROBOT_LOG( TRUE, "ERROR:  Bad Param3 for ArmUp = %d", Param3)
			}
			break;
		}

		case SpeechCmd_Wave:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Arm Wave")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute, 
			// Don't talk, the behavior module will add some voice comments to go with the wave
			break;
		}

		case SpeechCmd_ScratchBack:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Scratch Back")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SCRATCH_BACK );	// Right/Left arm, Movement to execute, 
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "good idea, I have an itch" );break;
					case 1:  Speak( "Ok, big stretch!" );break;
					default: Speak( "I think I have a loose screw back there" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_ScratchHead:
		case SpeechCmd_MeaningOfLife:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Scratch Head")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SCRATCH_HEAD );	// Right/Left arm, Movement to execute, 
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "let me think" );break;
					case 1:  Speak( "I need to think about that" );break;
					default: Speak( "interesting question" ); // If const is larger number, this gets called more often
				}
			break;
		}	

		case SpeechCmd_Follow:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Follow Me")
			if( MovementEnabled() )
			{
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_FOLLOW_PERSON, (DWORD)TRUE );	// TRUE = start mode
				// Respond with random phrases
				int RandomNumber = ((2 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Where are we going?" );break;
					default: Speak( "Ok, I will follow you" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}

		case SpeechCmd_TakePhoto:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Take Photo")
				// Don't speak here.  Handle in behavior module, so head nod can be suppressed
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TAKE_PHOTO, (DWORD)TRUE );	// TRUE = start mode
			break;
		}

		case SpeechCmd_ComeHere:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Come Here")
			if( MovementEnabled() )
			{
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_COME_HERE, (DWORD)TRUE );	// TRUE = start mode
				// Respond with random phrases
				int RandomNumber = ((2 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Ok" );break;
					default: Speak( "Sure" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}

		case SpeechCmd_TurnTowardsMe:
		{
			// Turn in the direction of the speaker's voice!
			//g_LastHumanAudioBeamDirection //Audio Angle in TenthDegrees
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Turn Towards Me NOT IMPLEMENTED!")
			if( MovementEnabled() )
			{
				// Respond to the command with action, then talk
				int TurnAmt = abs( (int)(g_LastHumanAudioBeamDirection * (float)2.0) );		//Audio Angle in Degrees
				if( g_LastHumanAudioBeamDirection < 0 )
				{
					SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, TurnAmt, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
				}
				if( g_LastHumanAudioBeamDirection > 0 )
				{
					SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, TurnAmt, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
				}

				RobotSleep(500, pDomainSpeakThread);
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "How is this" );break;
					default: Speak( "Now what?" ); // If const is larger number, this gets called more often
				}
					
			}
			break;
		}

		case SpeechCmd_GotoLocation:
		{
			if( MovementEnabled() )
			{
				switch( Param2 )
				{
					case SpeechParam_Office:
					{
						ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to Office")
						// Tell robot to find a path to desired location!
						DWORD Param = (DWORD)MAKELONG(WAYPOINT_OFFICE_X, WAYPOINT_OFFICE_Y); //LOWORD, HIWORD
						SpeechSendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
						SpeechSendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START, 0 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
						Speak( "Ok, I will go to the office" );	
					}
					break;
					case SpeechParam_MasterBedroom:
					{
						ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to master bedroom")
						// Tell robot to find a path to desired location!
						DWORD Param = (DWORD)MAKELONG(WAYPOINT_MASTER_BEDROOM_X, WAYPOINT_MASTER_BEDROOM_Y); //LOWORD, HIWORD
						SpeechSendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
						SpeechSendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START, 0 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
						Speak( "Ok, I will go to your bed room" );	
					}
					break;
					case SpeechParam_HeatherRoom:
					{
						ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to Heathers room")
						// Tell robot to find a path to desired location!
						DWORD Param = (DWORD)MAKELONG(WAYPOINT_HEATHER_BEDROOM_X, WAYPOINT_HEATHER_BEDROOM_Y); //LOWORD, HIWORD
						SpeechSendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
						SpeechSendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START, 0 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
						Speak( "OK, I will go to Heathers room" );	
					}
					break;
					case SpeechParam_AmberRoom:
					{
						ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to Ambers room")
						// Tell robot to find a path to desired location!
						DWORD Param = (DWORD)MAKELONG(WAYPOINT_AMBER_BEDROOM_X, WAYPOINT_AMBER_BEDROOM_Y); //LOWORD, HIWORD
						SpeechSendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
						SpeechSendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START, 0 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
						Speak( "Ok, I will go to Ambers room" );	
					}
					break;
					case SpeechParam_MasterBath:
					{
						ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to my bathroom")
						// Tell robot to find a path to desired location!
						DWORD Param = (DWORD)MAKELONG(WAYPOINT_MASTER_BATHROOM_X, WAYPOINT_MASTER_BATHROOM_Y); //LOWORD, HIWORD
						SpeechSendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
						SpeechSendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START, 0 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
						Speak( "Ok, I will go into your bath room" );	
					}
					break;
					default:
					{
						ROBOT_DISPLAY( TRUE, "ERROR: SpeechCmd_GotoLocation - Location Unknown!")
						Speak( "Sorry, I dont know how to get to that location" );	
					}
					break;
				}
			}
			break;
		}

		case SpeechCmd_Lights:
		{
			// "On"/"off" is hard to hear, so we just always toggle!
			// request to toggle light state
			if( g_SensorStatus.AuxLightsOn )
			{
				// lights are on, turn them off
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Lights Off")
				SpeechSendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)FALSE );
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Ok, I guess i should save my batteries" );break;
					case 1:  Speak( "Entering Stealth mode.  Are we playing secret agent?" );break;
					case 2:  Speak( "Ok, this will save power" );break;
					default:  Speak( "lights off" );break;
				}
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Lights On")
				SpeechSendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE );
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Doesn't this look nice?" );break;
					case 1:  Speak( "Blue is my favorite color" );break;
					case 3:  Speak( "I love my lights" );break;
					default:  Speak( "the lights are on, but nobody's home" );break;
				}
			}
			break;
		}

		case SpeechCmd_Karate:
		{
			if( MovementEnabled() )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Karate")
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_KARATE_DEMO, (DWORD)0 );	// Right/Left arm, Movement to execute, 
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "I am not exactly Bruce Lee" );break;
					case 1:  Speak( "This always make me feel centered" );break;
					default: Speak( "Look Out! Make sure there is plenty of space around me!" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}

		case SpeechCmd_LightSaber:
		{
			if( MovementEnabled() )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: LightSaber")
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_LIGHT_SABER, (DWORD)0 );	 
				// Respond with random phrases
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "We must beware of the dark side" );break;
					case 1:  Speak( "I will use the force to guide me" );break;
					default: Speak( "Look Out! Make sure there is plenty of space around me!" ); // If const is larger number, this gets called more often
				}
			}
			break;
		}

		case SpeechCmd_DoWhatTime:
		{
			if( MovementEnabled() )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: WhatTimeIsIt")
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_WHAT_TIME_IS_IT, (DWORD)0 );	
			}
			break;
		}

		case SpeechCmd_DoDanger:
		{
			if( MovementEnabled() )
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Freak Out")
				SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_FREAK_OUT, (DWORD)0 );
			}
			break;
		}

		case SpeechCmd_BadRobot:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Bad Robot")
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_BAD_ROBOT, (DWORD)0 );
			break;
		}

		case SpeechCmd_Wake:
		{
			if( SpeechParam_True == Param2 )
			{

				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Wake Up")
				//SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_WAKE_UP, (DWORD)0 );
				//Speak( "Wake up Not implemented" );	

				// Tell display not to blank (assumes PC is set to blank the display quickly, if this has not been called)
				SetThreadExecutionState(ES_DISPLAY_REQUIRED);
				g_SleepMode = FALSE;
				HWND lHwnd = FindWindow("Shell_TrayWnd",NULL);
				// Restore all windoes to appear to be waking up
				SendMessage(lHwnd,WM_COMMAND,MIN_ALL_UNDO,0);

				//SetThreadExecutionState(ES_CONTINUOUS | ES_DISPLAY_REQUIRED);
				// for Win7, use this instead?
				// HANDLE hPowerRequest = PowerCreateRequest(  __in  PREASON_CONTEXT Context ); // just a T String _T("Keep Alive")
				// BOOL PowerSetRequest(  __in  HANDLE PowerRequest,  __in  POWER_REQUEST_TYPE RequestType);
				// BOOL PowerClearRequest(  __in  HANDLE PowerRequest,  __in  POWER_REQUEST_TYPE RequestType);
				// CloseHandle( hPowerRequest );

				int RandomNumber = ((3 * rand()) / RAND_MAX);
				switch( RandomNumber )
				{
					case 0:  Speak( "I feel refreshed" );break;
					case 1:  Speak( "Recalibrating systems" );break;
					default: Speak( "I need to stretch" ); // If const is larger number, this gets called more often
				}

				// Move Head and arms to sleep position, and turn off servos
				ROBOT_LOG( TRUE,  "WakeUp: Enabling Servos...\n" )
					::PostThreadMessage( g_dwSmartServoCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_POWER_MODE, POWER_ON, 0 );
					::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_LED_EYES, LED_EYES_ON, 0 ); 	// Turn on Eyes

					/*
					g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING; 
					//g_ArduinoSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToPIC = FALSE;		// If false, indicates Arduino not found yet
					g_DynaSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToDyna = FALSE;		// If false, indicates Dyna controller not found yet
					g_RX64SubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToRX64 = FALSE;		// If false, indicates Dyna RX64 controller not found yet
					g_KerrSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToKerr = FALSE;		// If false, indicates Kerro controller not found yet
					g_MotorSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToMotor = FALSE;		// If false, indicates Motor controller not found yet
					//g_GPSSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToGPSDevice = FALSE;	// If false, indicates GPS device not connected yet

					//g_KinectSubSystemStatus = SUBSYSTEM_DISABLED;   //g_KinectReady = FALSE;			// If false, indicates Kinect not ready
					//g_CameraSubSystemStatus = SUBSYSTEM_DISABLED;   //g_CameraReady = FALSE;	
					g_LaserSubSystemStatus = SUBSYSTEM_WAITING;		//g_LaserReady = FALSE;	
					g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING;   //g_ArmReady_L = FALSE;	
					g_RightArmSubSystemStatus = SUBSYSTEM_WAITING;	//g_ArmReady_R = FALSE;	
					*/

					// enable laser scanning
					g_bLaserContinuousScanEnabled = TRUE;
					SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE ); // Turn lights on
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Go to Sleep")
				//SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_GO_TO_SLEEP, (DWORD)TRUE );
				//Speak( "Go to sleep not implemented" );	

				// Tell display it can blank (assumes PC is set to blank the display quickly, once this been called)
				/// DAVES DOES NOT WORK: SetThreadExecutionState(ES_CONTINUOUS); // No extra flags means "stop requiring the display"
				g_SleepMode = TRUE; // stop keeping the display turned on
				HWND lHwnd = FindWindow("Shell_TrayWnd",NULL);
				// Minimize all windoes to appear to be shutting down
				SendMessage(lHwnd,WM_COMMAND,MIN_ALL,0);
				//Sleep(2000);
				//SendMessage(lHwnd,WM_COMMAND,MIN_ALL_UNDO,0);

				int RandomNumber = ((3 * rand()) / RAND_MAX);
				switch( RandomNumber )
				{
					case 0:  Speak( "All right, shutting down" );break;
					case 1:  Speak( "This will save my batteries" );break;
					default: Speak( "Ok, powering down" ); // If const is larger number, this gets called more often
				}

				// Move Head and arms to sleep position, and turn off servos
				ROBOT_LOG( TRUE,  "GoToSleep: Moving Head and Arms to Sleep Position...\n" )
				::PostThreadMessage( g_dwSmartServoCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_POWER_MODE, SYSTEM_SHUT_DOWN, 0 );
				::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_LED_EYES, LED_EYES_OFF, 0 ); 	// Turn off Eyes

				ROBOT_LOG( TRUE,  "GoToSleep: Stopping Wheel Motors...\n" )
				#if( MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL )
					::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );

				#elif( MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL )
					::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );

				#elif( MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL )
					::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );

				#elif( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
					// Move Servos to STOP position!
					::PostThreadMessage( g_dwServoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );

				#elif ( (MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL)  )
					// Move Servos to STOP position!
					::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );
					::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_KINECT_POWER, 0, 0 ); // Turn off Kinect Power
				#else
					ROBOT_ASSERT(0);
				#endif

				g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING; 
				//g_ArduinoSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToPIC = FALSE;		// If false, indicates Arduino not found yet
				g_DynaSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToDyna = FALSE;		// If false, indicates Dyna controller not found yet
				g_RX64SubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToRX64 = FALSE;		// If false, indicates Dyna RX64 controller not found yet
				g_KerrSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToKerr = FALSE;		// If false, indicates Kerro controller not found yet
				g_MotorSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToMotor = FALSE;		// If false, indicates Motor controller not found yet
				//g_GPSSubSystemStatus = SUBSYSTEM_WAITING;		//g_ConnectedToGPSDevice = FALSE;	// If false, indicates GPS device not connected yet

				//g_KinectSubSystemStatus = SUBSYSTEM_DISABLED;   //g_KinectReady = FALSE;			// If false, indicates Kinect not ready
				//g_CameraSubSystemStatus = SUBSYSTEM_DISABLED;   //g_CameraReady = FALSE;	
				g_LaserSubSystemStatus = SUBSYSTEM_WAITING;		//g_LaserReady = FALSE;	
				g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING;   //g_ArmReady_L = FALSE;	
				g_RightArmSubSystemStatus = SUBSYSTEM_WAITING;	//g_ArmReady_R = FALSE;	

				//turn off laser scanning, so the COM port input can drain out
				g_bLaserContinuousScanEnabled = FALSE;
				SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)FALSE ); // Turn lights off
			}
			break;
		}
/*
		case SpeechCmd_PlaySimonSaysOn:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Simon Says On")
			m_bPlaySimonSays = TRUE;
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
				switch( RandomNumber )
				{
					case 0:  Speak( "Ok, you lead" );break;
					case 1:  Speak( "I love Simon Says.  Ready when you are" );break;
					case 2:  Speak( "That is my favorite game.  You be Simon" );break;
					case 3:  Speak( "Ok, let's play.  I'm ready." );break;
					default: Speak( "OK" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_PlaySimonSaysOff:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Play Stop / Simon Says Off")
			m_bPlaySimonSays = FALSE;
				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber);
				switch( RandomNumber )
				{
					case 0:  Speak( "Gee, that was fun" );break;
					case 1:  Speak( "Ok, Let me know when you want to play again" );break;
					case 2:  Speak( "that was the most fun I have had in thirty billion nano seconds" );break;
					case 3:  Speak( "Good game!  That was fun" );break;
					default: Speak( "OK" ); // If const is larger number, this gets called more often
				}
			break;
		}
*/
		case SpeechCmd_IdentifyObj:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Identify Object")
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_IDENTIFY_OBJECT);	// Right/Left arm, Movement to execute, 
				// Respond with random phrases
				int RandomNumber = ((5 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber);
				switch( RandomNumber )
				{
					case 0:  Speak( "All right" );break;
					case 1:  Speak( "Thanks" );break;
					case 2:  Speak( "Nice" );break;
					case 3:  Speak( "Cool" );break;
					default: Speak( "Ok" ); // If const is larger number, this gets called more often
				}
			break;
		}

		case SpeechCmd_TellJoke:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Tell Jokes")
			BOOL MultipleJokes = FALSE;
			if( SpeechParam_Multiple == Param2 )
			{
				MultipleJokes = TRUE;
			}
			SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_TELL_JOKES, (DWORD)MultipleJokes );	
			break;
		}

		case SpeechCmd_DoIntro:
		{
			ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Introduction")
			//SendCommand( WM_ROBOT_TEXT_MESSAGE_TO_SERVER, WM_ROBOT_SPEAK_TEXT, SPEAK_INTRO );
			SpeechSendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 120, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 ); // face forward
			RobotSleep(1000, pDomainSpeakThread);
			SpeechSendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 ); // face forward
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute, 
			Sleep(3000);
			SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute, 
			Sleep(3000);
			PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, SPEAK_INTRO, 0 );
			//Speak( "Ready");

			// Note: arm and body movements handled by the speaking routine
			//SpeechSendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_MOVE_WHILE_TALKING, (DWORD)0 );
			break;
		}

		default:
		{
			CString strErrorMsg;
			strErrorMsg.Format("SpeechReco Command Ignored: Unknown Command %d (%04X)!\n", Command, Command );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)strErrorMsg )
			//ROBOT_ASSERT(0);
		}
	}

	m_bSimonSays = FALSE;	// wait for next "Simon says" key phrase
	__itt_task_end(pDomainSpeechRecoThread);

}


void CRobotSpeechReco::SendRecoToAI( BOOL bEnable )
{
	m_bSendRecoToAI = bEnable;
	__itt_marker(pDomainSpeechRecoThread, __itt_null, pshSendToAI, __itt_marker_scope_task);
}



#endif	// ROBOT_SERVER


/* 
EXAMPLE FOR USER ID
            // Figure out where dictation starts and get that text
            // Since we know how many words are in our rule ( 5 ) we can get the text
            // after them, which will be the dictation.
			if( SpeechCmd_SayHello == SpeechRecoCmd )
			{
				nWordsInPhrase = 3;
			}
			else if( SpeechCmd_Meet6 == SpeechRecoCmd )
			{
				nWordsInPhrase = 6;
			}
			else if( SpeechCmd_Meet5 == SpeechRecoCmd )
			{
				nWordsInPhrase = 5;
			}
			else if( SpeechCmd_Meet2 == SpeechRecoCmd )
			{
				nWordsInPhrase = 2;
			}

			ROBOT_LOG( TRUE, "PID = %d, words expected in phrase = %d\n", SpeechRecoCmd, nWordsInPhrase)
            if ( nWordsInPhrase <= pElements->Rule.ulCountOfElements )
            {
                if ( SUCCEEDED( rResult.GetText( nWordsInPhrase, pElements->Rule.ulCountOfElements - nWordsInPhrase, FALSE,
                     &wszCoMemNameText, NULL ) ) )
                {
                    int ilen = wcslen( pElements->pProperties->pszName );
                    ilen = (ilen + wcslen( wszCoMemNameText ) + 2) * sizeof(WCHAR);
                    wszCoMemValueText = (WCHAR *) CoTaskMemAlloc( ilen );
                    if ( wszCoMemValueText )
                    {
                        wcscpy( wszCoMemValueText, pElements->pProperties->pszName );
                        wcscat( wszCoMemValueText, L" " );
                        wcscat( wszCoMemValueText, wszCoMemNameText );
                        
                        // Copy new name to global user name
                        _tcsncpy_s( UserName, W2T(wszCoMemNameText), USER_NAME_LEN - 1 );
 						ROBOT_LOG( TRUE, "NAME = %s\n", UserName)
						g_CurrentUserName = UserName;

						// Now respond to the user's introduction

						//ROBOT_DISPLAY( TRUE, "SpeechReco Command Recognized: Attention (LOKI's name called)")

						if( SpeechCmd_SayHello == SpeechRecoCmd )
						{
							swprintf_s(SpeakBuf, L"Hi, %s, How's it going?", UserName );
							// Respond with random phrases
							CString strResponse;
							int RandomNumber = ( (8 * rand()) / RAND_MAX);
							ROBOT_LOG( TRUE, "DEBUG: SpeechCmd_SayHello RAND = %d\n", RandomNumber)
							switch( RandomNumber )
							{
								case 0: swprintf_s(SpeakBuf, L"Hi %s, how are you?", T2CW(UserName) ); 
								case 1: swprintf_s(SpeakBuf, L"Hey %s, what's up?", T2CW(UserName) ); 
								case 2: swprintf_s(SpeakBuf, L"Hi %s, how's it going", T2CW(UserName) ); 
								default: swprintf_s(SpeakBuf, L"Hello %s", T2CW(UserName) ); 
							}
							Speak( SpeakBuf );
							//SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );	// Right/Left arm, Movement to execute, 
						}
						else if( (SpeechCmd_Meet6 == SpeechRecoCmd) || (SpeechCmd_Meet5 == SpeechRecoCmd) || (SpeechCmd_Meet2 == SpeechRecoCmd) )
						{
							// Say hello.  Rest of the greeting will be done by the hand shake routine
							swprintf_s(SpeakBuf, L"Hello %s", T2CW(UserName) );
							Speak( SpeakBuf );
							SpeechSendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );	// Right/Left arm, Movement to execute, 
						}
						else
						{
							ROBOT_ASSERT(0);
						}
	                    CoTaskMemFree( wszCoMemNameText );
                    }
                }
            }
        }
        break;
*/