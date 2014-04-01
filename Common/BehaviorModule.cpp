// BehaviorModule.cpp: class implementation
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "HardwareConfig.h"

//#include "LaserScannerBehavior.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define DEBUG_ARM_MOVEMENT			0
#define DEBUG_SCRIPT_SHOW_COMMANDS	1


///////////////////////////////////////////////////////////////////////////////
// Macros and defines

#define BLINK_CHECK_TIME							8.0 // how often to blink, in number of Sensor updates from Arduino
#define POST_PICKUP_ROTATION_AMT					20	// degrees
#define SCRIPT_LINE_LENGTH_MAX						80  // chars
#define HUMAN_FOLLOW_DISTANCE_TARGET_TENTH_INCHES	( 32 * 10 ) //TenthInches
#define HUMAN_APPROACH_DISTANCE_TARGET_TENTH_INCHES	( 26 * 10 ) //TenthInches
#define HUMAN_FOLLOW_DISTANCE_MIN_TENTH_INCHES		( 22 * 10 ) //TenthInches
#define NUMBER_OF_JOKES								8			// must match the number of jokes defined


__itt_string_handle* pshKinectPickupObjectStart = __itt_string_handle_create("KinectPickupObjectStart"); // marker
__itt_string_handle* pshKinectPickupObjectSearchComplete = __itt_string_handle_create("KinectPickupObjectSearchComplete"); // marker
__itt_string_handle* pshKinectPickupObjectTrackingStart = __itt_string_handle_create("KinectPickupObjectTrackingStart"); // marker
__itt_string_handle* pshKinectPickupNoObject = __itt_string_handle_create("KinectPickupNoObject"); // marker
__itt_string_handle* pshKinectPickupObjectLost = __itt_string_handle_create("KinectPickupObjectLost"); // marker
__itt_string_handle* pshKinectArrivedAtObject = __itt_string_handle_create("KinectArrivedAtObject"); // marker
__itt_string_handle* pshKinectMoveToObject = __itt_string_handle_create("KinectMoveToObject"); // marker






///////////////////////////////////////////////////////////////////////////////
//	MODULE: BehaviorModule
///////////////////////////////////////////////////////////////////////////////


CBehaviorModule::CBehaviorModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_CurrentActionMode = ACTION_MODE_NONE;
	m_ActionParam = 0;
	m_CurrentTask = TASK_NONE;
	m_TaskState = 0;
	m_TurnHandleState = 0;
	m_NextShoulderPosition = 0;
	m_nObjectsPickedUp = 0;
	m_nHumanIDToTrack = 0;
	m_RepeatCount = 0;
	m_ResponseReceived = -1;
	m_ResponsePending = FALSE;

	// Scripts
	m_ScriptFileHandle = NULL;
	m_ScriptLineNumber = 0;
	m_HeadExecutePending = FALSE;
	m_HeadSpeedPending = FALSE;
	m_LeftArmExecutePending = FALSE;
	m_LeftArmSpeedPending = FALSE;
	m_RightArmExecutePending = FALSE;
	m_RightArmSpeedPending = FALSE;
	m_WheelExecutePending = FALSE;
	m_WheelSpeedPending = FALSE;


	m_LedEyeMode = LED_EYES_OFF;
	m_LedEyeBlinkTimer = 0;
	//m_LedEyeBrightness = LED_EYE_BRIGHTNESS_MAX;
	m_ArmMovementRight = 0;
	m_ArmMovementLeft = 0;
	m_ArmMovementBoth = 0;
	m_ArmMovementStateRight = 0;
	m_ArmMovementStateLeft = 0;
	gBehaviorTimer = 0;
	gHeadNodTimer = 0;
	gArmTimerRight = 0;
	gArmTimerLeft = 0;
	m_ObjectDetectCount = 0; // Used by ArmBehavior
	nKinectObjectsDetected = 0;	// Used for counting number of objects detected by Kinect
	m_KinectRetries = 0;		// Number of times to retry to find object
	m_ArmSpeedRight = SERVO_SPEED_MED;		// Used for saving and restoring prior arm speed
	m_ArmSpeedLeft = SERVO_SPEED_MED;		// Used for saving and restoring prior arm speed
	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
	m_pHeadControl = new HeadControl();
	m_pKinectServoControl = new KinectServoControl;

	m_MaxMoveTimeRight = 0;
	m_MaxMoveTimeLeft = 0;
	m_ArmWaitForMoveToCompleteRight = TRUE;
	m_ArmWaitForMoveToCompleteLeft = TRUE;
	m_KinectSearchComplete = FALSE;
	m_ObjectPickupComplete = FALSE;
	m_PutObjectInCarryBasket = FALSE;

	m_HeadMovement = 0;
	m_HeadMovementState = 0;
	m_HeadSpeed = SERVO_SPEED_SLOW;
	m_HeadWaitForMoveToComplete = TRUE;

	m_ObjectX = 0;
	m_ObjectY = 0;
	m_PriorObjectX = 0;
	m_PriorObjectY = 0;
	m_ObjectDirectionDegrees = 0;
	m_ObjectDirectionDegreesPeak = 0;
	m_HeadNodState = 0;
	m_PhraseToSpeak = 0;
	m_RandomPhrase = 0;
	m_bSpeakingToChild = FALSE;
	m_SubTaskComplete = FALSE;
	m_HeadNodEnabled = TRUE; // Nod head when talking unless suppressed


	ROBOT_LOG( TRUE,"Shuffling Joke Order\n")
	m_JokeOrder = new CSequenceOrder( NUMBER_OF_JOKES ); // creates random ordered list

}


CBehaviorModule::~CBehaviorModule()
{
	SAFE_DELETE( m_pArmControlRight );
	SAFE_DELETE( m_pArmControlLeft );
	SAFE_DELETE( m_pHeadControl );
	SAFE_DELETE( m_pKinectServoControl );
	SAFE_DELETE( m_JokeOrder );
}



/////////////////////////////////////////////////////////////////////////////
//void CBehaviorModule::Init()
//{
//}

__itt_string_handle* pshRightArmHome = __itt_string_handle_create("RightArmHome");
__itt_string_handle* pshLeftArmHome = __itt_string_handle_create("LeftArmHome");
__itt_string_handle* pshHeadCenter = __itt_string_handle_create("HeadCenter");


///////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::RightArmHome()
{
	// Move all right arm seros to home position, and reset to default speed
#if ( 1 == ROBOT_HAS_RIGHT_ARM  )
	__itt_marker(pDomainControlThread, __itt_null, pshRightArmHome, __itt_marker_scope_task);
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshRightArmHome);
	m_pArmControlRight->MoveArmHome( m_ArmSpeedRight );
	m_pArmControlRight->EnableIdleArmMovement(TRUE);
	g_MoveArmsWhileSpeaking = FALSE; // until explicitly enabled by a behavior that wants it
	gHeadIdle = TRUE;
	m_ArmMovementStateRight = 0;	// back to Idle state
	m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
	__itt_task_end(pDomainControlThread);
#endif
}
///////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::LeftArmHome()
{
	// Move all arm seros to home position, and reset to default speed
#if ( 1 == ROBOT_HAS_LEFT_ARM  )
	__itt_marker(pDomainControlThread, __itt_null, pshLeftArmHome, __itt_marker_scope_task);
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshLeftArmHome);
	m_pArmControlLeft->MoveArmHome( m_ArmSpeedLeft );
	m_pArmControlLeft->EnableIdleArmMovement(TRUE);
	g_MoveArmsWhileSpeaking = FALSE; // until explicitly enabled by a behavior that wants it
	gHeadIdle = TRUE;
	m_ArmMovementStateLeft = 0;	// back to Idle state
	m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
	__itt_task_end(pDomainControlThread);
#endif
}

///////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::HeadCenter()
{
	// Move head to home position, and reset to default speed
	m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
	__itt_marker(pDomainControlThread, __itt_null, pshHeadCenter, __itt_marker_scope_task);
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHeadCenter);
	m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 );
	m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
	__itt_task_end(pDomainControlThread);
}

///////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::EndActionMode()
{
	// Cleanup at the end of each action.
	// Move head and Kinect to home position, reset to default speeds
	// Reset state machine (what about Arms - move to home?)

	m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, KINECT_HUMAN_DETECT_START_POSITION );	// By default, leave Kinect in a position where it can find humans
	m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_TRACK_OBJECT);
	g_pKinectObjects3D->nObjectsDetected = 0;
	#if DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1
		g_pKinectDebugSliceArray->nSlices = 0;
	#endif
	HeadCenter();
	LeftArmHome();
	RightArmHome();
	m_CurrentActionMode = 0;
	m_CurrentTask = 0;
	m_TaskState = 0;
}

///////////////////////////////////////////////////////////////////////////////

void CBehaviorModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{

	// Process Behavior commands from the GUI
	// and status updates from Servos and Sensors
	CString MsgString;
	UINT ActionRequested;
	//static int foo;

	switch( uMsg )
	{
		// Behavior modes - what to do while standing around
		// Some of these are mutually exclusive!

		case WM_ROBOT_STOP_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Other modules will handle the stop.  For Behavior module, this means cancel current behavior and go back to idle
			// Example: SpeechSendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 ); // FORCE STOP, no matter what!
			m_CurrentActionMode = ACTION_MODE_NONE;
			m_CurrentTask = TASK_NONE;
			m_TaskState = 0;
			RightArmHome();
			LeftArmHome();
		}
		break;

/*
		case WM_ROBOT_SET_USER_PRIORITY:
		{
			g_bCmdRecognized = TRUE;
			// This command only comes from a user doing manual control
			// Allows user to override higher priority modules, or release control when done.
			// wParam = action to take


			if( SET_USER_LOCAL_AND_STOP == wParam )
			{
				// Other modules will handle the stop.  For Behavior module, this means cancel current behavior and go back to idle
				// Example: SpeechSendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 ); // FORCE STOP, no matter what!
				m_CurrentActionMode = ACTION_MODE_NONE;
				m_CurrentTask = TASK_NONE;
				m_TaskState = 0;
				RightArmHome();
				LeftArmHome();
			}
		}
		break;
*/
		case WM_ROBOT_SET_BEHAVIOR_CMD:
		{
			g_bCmdRecognized = TRUE;
			if( BEHAVIOR_FRIENDLY == wParam )  // wParam is the Behavior requested
			{
				// Enable random Head and Arm movements
				gEnableIdleMovements = TRUE;
				gArmIdleLeft = TRUE;
				gArmIdleRight = TRUE;
				gHeadIdle = TRUE;
			}
			else
			{
				// Disable random Head and Arm movements
				gEnableIdleMovements = FALSE;
				gArmIdleLeft = FALSE;
				gArmIdleRight = FALSE;
				gHeadIdle = FALSE;
				if( m_pHeadControl->IsOwner(HEAD_OWNER_RANDOM_MOVEMENT) )
				{
					m_pHeadControl->ReleaseOwner(HEAD_OWNER_RANDOM_MOVEMENT);
				}

				// Now, do any additional specific processing needed

				switch( wParam )  // wParam is the Behavior requested
				{
					case BEHAVIOR_MANUAL_CONTROL:
					{
					}
					break;

					case BEHAVIOR_WAIT_FOR_VERBAL_CMD:
					{
					}
					break;

					case BEHAVIOR_GUARD:
					{
					}
					break;

					case BEHAVIOR_PERSONAL_SPACE:
					{
						// If enabled, robot will move whenever someone violates
						// it's personal space (as detected by sensors) when standing still
						if( 0 == wParam )
						{
							// Disable Personal space mode
						}
						else
						{
							// Enable Personal space mode
						}
						//SendCommand( WM_ROBOT_CAMERA_BEHAVIOR_MODE, BEHAVIOR_GUARD, 0 );
					}
					break;

					default:
					{
						ROBOT_LOG( TRUE,"ERROR - Unhandled BEHAVIOR command (%02X)!\n", lParam)
					}
				}
			}
		}
		break;

		// Action modes - what to do right now
		case WM_ROBOT_SET_ACTION_CMD:
		{
			ROBOT_LOG( TRUE,"BehaviorModule: WM_ROBOT_SET_ACTION_CMD \n")
			g_bCmdRecognized = TRUE;
			ActionRequested = wParam;
			m_ActionParam = lParam;	// save optional param, to be used depending upon the task

			/// Enum SET_ACTION_CMD defined in RobotSharedParams.h
			// Action Modes - Random ideas for now, of things robot could do
			ROBOT_LOG( TRUE,"WM_ROBOT_SET_ACTION_CMD:  Action %d requested\n", ActionRequested )

			if( ACTION_MODE_CHAT_DEMO_WITH_ADULT == ActionRequested )
			{
				m_PhraseToSpeak = 0;
				m_RandomPhrase = 0;
				m_bSpeakingToChild = FALSE;
				m_CurrentActionMode = ActionRequested;
				m_TaskState = 1;	// go to first state
				m_CurrentTask = 1;
				// Start by having robot shake hands
				m_SubTaskComplete = FALSE; // set flag so we know when the hand shaking behavior is done
				SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );	// Right/Left arm, Movement to execute,
				break;
			}

			if( ACTION_MODE_CHAT_DEMO_WITH_CHILD == ActionRequested )
			{
				m_PhraseToSpeak = 0;
				m_RandomPhrase = 0;
				m_bSpeakingToChild = TRUE;
				m_CurrentActionMode = ActionRequested;
				m_TaskState = 1;	// go to first state
				m_CurrentTask = 1;
				// Start by having robot shake hands
				m_SubTaskComplete = FALSE; // set flag so we know when the hand shaking behavior is done
				SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_SHAKE_READY );	// Right/Left arm, Movement to execute,
				break;
			}

			if( ACTION_MODE_YES_NO_RESPONSE_RECEIVED == ActionRequested )
			{
				if( ACTION_MODE_NONE == m_CurrentActionMode )
				{
					ROBOT_LOG( TRUE,"ACTION_MODE_YES_NO_RESPONSE_RECEIVED: Ignored, no Action pending\n")
				}
				else
				{
					if( m_ResponsePending )
					{
						// We've been waiting for this response!
						m_ResponseReceived = m_ActionParam; // Yes or No 
						ROBOT_LOG( TRUE,"ACTION_MODE_YES_NO_RESPONSE_RECEIVED: ok\n")
					}
					else
					{
						ROBOT_LOG( TRUE,"ACTION_MODE_YES_NO_RESPONSE_RECEIVED: Ignored, no response pending\n")
					}
				}
				// This never actually sets the action mode, it's more of a "continue on with what you were doing..."
				break; // so don't change from the current action mode!
			}



			if( ACTION_MODE_NONE == ActionRequested )
			{
				if( ACTION_MODE_NONE != m_CurrentActionMode )
				{
					ROBOT_LOG( TRUE,"WM_ROBOT_SET_ACTION_CMD: CURRENT ACTION CANCELLED!\n")
					// Tell Kinect module to cancel behavior too
					SendCommand( WM_ROBOT_KINECT_CANCEL_CMD, (DWORD)0, (DWORD)0 );
				}
				EndActionMode();
			}
			else if( (ACTION_MODE_PICKUP_OBJECTS == ActionRequested) ||	(ACTION_MODE_PICKUP_CLOSE_OBJECT== ActionRequested) )
			{
				/* DEBUG
				if( m_CurrentActionMode == ActionRequested )
				{
					// Repeat command for DEBUG -continue to next step!
					if( (OBJECT_TASK_MOVE_TO_OBJECT == m_CurrentTask) && (1 == m_TaskState) )
					{
						// ROBOT_LOG( TRUE,"DEBUG CONTINUING To next Action Mode Step\n")
						m_TaskState++;
					}
				}
				else
				*/
				{
					ROBOT_LOG( TRUE,"ACTION_MODE_PICKUP_OBJECT (Close or Far) requested\n")
					if( lParam )
					{
						if( CAMERA_STATE_INITIALIZED != g_Camera[KINECT_DEPTH].State )
						{
							ROBOT_DISPLAY( TRUE, "ACTION_MODE_PICKUP_OBJECT: KINECT Not Enabled! ABORTING" )
							SpeakText( "Bummer. My Kinect module is not enabled" );
			
							break;
						} 
						m_CurrentActionMode = ActionRequested;
						m_CurrentTask = OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT;	// Start by looking for any objects nearby
						m_TaskState = 1;	// go to first state
						m_nObjectsPickedUp = 0; // keep track of number of objects picked up to measure success
					}
					else
					{
						///m_PickupObjectsEnabled = FALSE;
						ROBOT_LOG( TRUE,"BehaviorModule: ACTION_MODE_PICKUP_OBJECTS CANCELLED!!\n")
						EndActionMode();
					}
				}
			}
			else
			{
				m_CurrentActionMode = ActionRequested;
				m_CurrentTask = 1;
				m_TaskState = 1;	// go to first state
			}
		}	// WM_ROBOT_SET_ACTION_CMD
		break;

		case WM_ROBOT_SET_LED_EYES_CMD:
		{
			g_bCmdRecognized = TRUE;
			m_LedEyeMode = wParam;	// Used for blink only
			m_LedEyeBlinkTimer = 0;

			SendHardwareCmd( HW_SET_LED_EYES, wParam, lParam  );	// Mode, Brightness(not used)
		}
		break;


		/* todo
		#define ARM_MOVEMENT_OPEN_HATCH				0x08	// 
		#define ARM_MOVEMENT_CLOSE_HATCH			0x09	// 
		#define ARM_MOVEMENT_PICKUP_OBJECT			0x0A	// 
		#define ARM_MOVEMENT_LIFT_OBJECT			0x12	// */


		// Arm Movements
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		case WM_ROBOT_SET_ARM_DEFAULT_SPEED:
		{
			g_bCmdRecognized = TRUE;
			if( RIGHT_ARM == wParam )	// wParam is Right or Left Arm
			{
				m_ArmSpeedRight = lParam;
			}
			else if( LEFT_ARM == wParam )	// wParam is Right or Left Arm
			{
				m_ArmSpeedLeft = lParam;
			}
			else
			{
				ROBOT_ASSERT(0);	// bad state
			}
		}
		break;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		case WM_ROBOT_SET_ARM_MOVEMENT:
		{
			g_bCmdRecognized = TRUE;
			// Process Arm Behavior commands and do inital movemment for arms (move into first position)

			HandleArmMovementRequest( wParam, lParam );
		}
		break;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		case WM_ROBOT_CAMERA_MATCH_COMPLETE:
		{
			g_bCmdRecognized = TRUE;
			// Update from the vision system.  Response to WM_ROBOT_CAMERA_MATCH_OBJECT
			// wParam: ObjectPos, lParam: Location in Frame(TODO)
			HandleCameraMatchComplete( wParam, lParam );
		}
		break;

		case WM_ROBOT_FIND_OBJECT_AT_XYZ_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Tell camera to find object at X,Y,Z in space
			ROBOT_ASSERT(0);
			///m_pLaserScannerBehavior->HandleFindObjectAtXYZ( wParam, lParam );		
		}
		break;

		case WM_ROBOT_DO_LASER_SCANS:
		{
			g_bCmdRecognized = TRUE;
			// Do multi-line laser scans at location specified
			ROBOT_ASSERT(0);
			///m_pLaserScannerBehavior->DoLaserScans( wParam, lParam );		
		}
		break;


		case WM_ROBOT_SLAM_CHECK_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Put head in position and do a location check
			ROBOT_ASSERT(0);
			///m_pLaserScannerBehavior->DoLocationCheck();		
		}

		case WM_ROBOT_USER_CAMERA_PAN_CMD:
		{
			g_bCmdRecognized = TRUE;
			// wParam = Pan/Tilt Command enumeration
			// lParam = Speed
			// Send Command to tell camera to move Pan and Tilt
			// This command controls both Pan and Tilt
			HandleUserHeadMovementCmd(wParam, lParam );
		}
		break;

		case WM_ROBOT_USER_CAMERA_PAN_TILT_SPEED_CMD:
		{
			g_bCmdRecognized = TRUE;
			UINT Speed = lParam;
			m_pHeadControl->SetUserHeadSpeed( Speed ); // Save, so it can be restored as needed later
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_USER_CONTROL, Speed, Speed, Speed );
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );
			return;
		}

		///////////////////////////////////////////////////////////////////
		// MOVED FROM CAMERA MODULE - DAVES
		///////////////////////////////////////////////////////////////////
		case WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Tell camera to go to ABSOLUTE PAN/TILT position pointing at X,Y,Z in space
			// wParam = X,Z in TenthInches
			// lParam = Y (distance from robot) in TenthInches

			// TODO REMOVE THIS COMMENT if( m_HandTrackingEnabled )
			{
				int Z = LOWORD(wParam);
				int X = HIWORD(wParam);
				int Y = lParam;
				m_pHeadControl->LookAtXYZ( HEAD_OWNER_BEHAVIOR_P1, X,Y,Z );
				m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			}
			return;
		}

		case WM_ROBOT_CAMERA_LOOK_AT_PLAYER_CMD:
		{
			// Tell camera to go to position for the human "player"
			// point to their head
			// wParam = not used
			// lParam = speed
			g_bCmdRecognized = TRUE;
			int player = g_CurrentHumanTracked;
			static int HeadTiltCounter = 0; // every so often, change head tilt.  But not too often.

			// See if there is a Human "Player" found
			if( g_HumanLocationTracking[player].Found )
			{
				// find center of head
				int PanAngleTenthDegrees = g_HumanLocationTracking[player].AngleTenthDegrees.X;
				int TiltAngleTenthDegrees = g_HumanLocationTracking[player].AngleTenthDegrees.Y;


				// Set Speed, if requested
				/* disabled
				if( 0 != lParam )
				{
					m_pHeadControl->SetHeadSpeed( HEAD_OWNER_KINECT_HUMAN, lParam, NOP, NOP );	// Pan speed only
				}
				*/
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_KINECT_HUMAN, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );

				// Set position
				if( HeadTiltCounter++ > 10)
				{
					double fRandom_SideTilt = 0.5 - ((double)(rand()) / RAND_MAX);
					int SideTilt = (int)(fRandom_SideTilt * ((double)CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT));
					m_pHeadControl->SetHeadPosition( HEAD_OWNER_KINECT_HUMAN, PanAngleTenthDegrees, TiltAngleTenthDegrees, SideTilt );	// Pan, Tilt, SideTilt
					m_pHeadControl->ExecutePosition( HEAD_OWNER_KINECT_HUMAN );
					HeadTiltCounter = 0;
				}
				else
				{
					m_pHeadControl->SetHeadPosition( HEAD_OWNER_KINECT_HUMAN, PanAngleTenthDegrees, TiltAngleTenthDegrees, NOP );	// Pan, Tilt, SideTilt
				}
				m_pHeadControl->ExecutePosition( HEAD_OWNER_KINECT_HUMAN );
				//ROBOT_LOG( TRUE,"LOOK_AT_PLAYER_CMD: PLAYER %d FOUND, moving Head to %d, %d\n", player, PanAngleTenthDegrees/10 , TiltAngleTenthDegrees/10 )

				// DEBUG - TEST - Player Height?
				//int Height = g_HumanLocationTracking[player].HeadLocation.Y

			}
			else
			{
				ROBOT_LOG( TRUE,"LOOK_AT_PLAYER_CMD: ERROR: PLAYER %d NOT FOUND!\n", player )
			}

			return;
		}

		case WM_ROBOT_CAMERA_LOOK_AT_SOUND_CMD:
		{
			// Tell camera to go to ABSOLUTE PAN position specified in TENTHDEGREES
			// Look for any "Players" there, and lock on to their heads
			// wParam = position to move to
			// lParam = speed
			g_bCmdRecognized = TRUE;
			int PlayerFound = 0;
			int AudioAngleTenthDegrees = wParam;
			int PanAngleTenthDegrees = AudioAngleTenthDegrees;	// by default, point to sound source
			int TiltAngleTenthDegrees = KINECT_HUMAN_DETECT_START_POSITION; // Point about where a face ought to be

			// Set Speed, if requested
			if( 0 != lParam )
			{
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_KINECT_HUMAN, lParam, NOP, NOP );	// Pan speed only
			}

			// See if there is a Human "Player" near the sound source.  If so, lock on.
			//int ClosestPlayer = 0;
			int SmallestAngleDelta = 3600;
			for( int player=0; player<KINECT_MAX_HUMANS_TO_TRACK; player++ )
			{
				if( g_HumanLocationTracking[player].Found )
				{
					int AngleDelta = abs( g_HumanLocationTracking[player].AngleTenthDegrees.X - AudioAngleTenthDegrees );
					if( (AngleDelta < 250) && (AngleDelta < SmallestAngleDelta) )
					{
						// Player in range of sound source
						PanAngleTenthDegrees = g_HumanLocationTracking[player].AngleTenthDegrees.X;
						TiltAngleTenthDegrees = g_HumanLocationTracking[player].AngleTenthDegrees.Y;
						SmallestAngleDelta = AngleDelta; // keep looking to see if there is an even closer player
						PlayerFound = player; //DEBUG
						g_CurrentHumanTracked = player; // Tell system to continue to track this player
					}
				}
			}

			if( 0 == PlayerFound )
			{
				ROBOT_LOG( TRUE,"LOOK_AT_SOUND_CMD: No PLAYER at ANGLE: X=%3d Y=%3d, :\n", PanAngleTenthDegrees/10, TiltAngleTenthDegrees/10 )
			}
			else
			{
				ROBOT_LOG( TRUE,"LOOK_AT_SOUND_CMD: FOUND PLAYER %2d at ANGLE: X=%3d Y=%3d, :\n", PlayerFound, PanAngleTenthDegrees/10, TiltAngleTenthDegrees/10 )
				g_LastHumanCompassDirection = g_pFullSensorStatus->CompassHeading + (PanAngleTenthDegrees/10);
				if( g_LastHumanCompassDirection > 360 ) g_LastHumanCompassDirection -= 360;
				else if( g_LastHumanCompassDirection < 0 ) g_LastHumanCompassDirection += 360;
			}

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_KINECT_HUMAN, PanAngleTenthDegrees, TiltAngleTenthDegrees, NOP );	// Pan and Tilt
			m_pHeadControl->ExecutePosition( HEAD_OWNER_KINECT_HUMAN );

			return;
		}

		case WM_ROBOT_CAMERA_PAN_ABS_CMD:
		{
			// Tell camera to go to ABSOLUTE PAN position specified in TENTHDEGREES
			// wParam = position to move to
			// lParam = movement speed
			g_bCmdRecognized = TRUE;
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_USER_CONTROL, wParam, NOP, NOP );	// Pan only
			if( 0 != lParam )
			{
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_USER_CONTROL, lParam, NOP, NOP );	// Pan speed only
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );
//				Sleep(1);
			}
			else
			{
				m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
			}
			return;
		}

		case WM_ROBOT_CAMERA_TILT_ABS_CMD:
		{
			// Tell camera to go to ABSOLUTE TILT position specified in TENTHDEGREES
			// wParam = position to move to
			// lParam = movement speed

			g_bCmdRecognized = TRUE;
			if( 0 != lParam )
			{
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_USER_CONTROL, NOP, lParam, NOP );	// Tilt speed only
			}
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_USER_CONTROL, NOP, wParam, NOP );	// Tilt only
			m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
			return;
		}

		case WM_ROBOT_CAMERA_SIDETILT_ABS_CMD:
		{
			// Tell camera to go to ABSOLUTE SIDE TILT position specified
			// wParam = position to move to
			g_bCmdRecognized = TRUE;
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_USER_CONTROL, NOP, NOP, wParam );	// SideTilt only
			m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
			return;
		}

		case WM_ROBOT_CAMERA_PAN_REL_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Tell camera to go to position RELATIVE to current position
			m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL, wParam, NOP, NOP );	// Pan only
			m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
			return;
		}

		case WM_ROBOT_CAMERA_TILT_REL_CMD:
		{
			// wParam = Amount or positon to move to
			// lParam = Relative or Absolute
			// Tell camera to go to position RELATIVE to current position
			g_bCmdRecognized = TRUE;
			m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL, NOP, wParam, NOP );	// Tilt only
			m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
			return;
		}

		case WM_ROBOT_CAMERA_NOD_HEAD_CMD:
		{
			// wParam = not used
			// lParam = not used
			// When set will nod the head once
			g_bCmdRecognized = TRUE;
			if( m_HeadNodEnabled )
			{
				m_HeadNodState = 1; // start motion
			}
			return;
		}



		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ============================================================================================================
		case WM_ROBOT_KINECT_SEARCH_COMPLETE:
		case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		// ============================================================================================================
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			// All these events are handled here to prevent stalling if any one event or subsystem fails!
			// such as the Arduino being offline - the Heartbeat continues!
			g_bCmdRecognized = TRUE;

			if( WM_ROBOT_KINECT_SEARCH_COMPLETE == uMsg )
			{
				m_KinectSearchComplete = TRUE;
				nKinectObjectsDetected = wParam;
				m_ObjectX = (int)(signed short)(LOWORD(lParam));
				m_ObjectY = (int)(signed short)(HIWORD(lParam));
				ROBOT_LOG( TRUE,"DEBUG - WM_ROBOT_KINECT_SEARCH_COMPLETE: m_ObjectX = %d, m_ObjectY = %d\n", m_ObjectX, m_ObjectY)
			}

			if( WM_ROBOT_SERVO_STATUS_READY == uMsg )
			{
				// Handle Arm Servo updates
				// Check Arm Movement State Machine.  Only used with movements that have multiple steps
				// Uses m_ArmMovementStateRight and m_ArmMovementStateLeft to track progress
				//m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT); // DEBUG
				HandleArmServoStatusUpdate( wParam, lParam );
			}

			// Handle Head Nod behavior, if requested
			if( 0 != m_HeadNodState )
			{
				DoHeadNod();
			}


			/////////////////////////////////////////////////////////////////////////////////////////
			// Check status of Behavior Tasks
			//
			// Action Mode
			//   Task
			//     Task Step
			//       Step State
				//	m_CurrentActionMode = ACTION_MODE_NONE;
				//	m_CurrentTask = TASK_NONE;
				//	m_TaskState = 0;

			if( (ACTION_MODE_NONE != m_CurrentActionMode) && (TASK_NONE != m_CurrentTask) ) 
			{
				// Something to do.  See if we are ready to do the next step
				if( 0 != gArmTimerLeft)
				{
					ROBOT_LOG( TRUE,"WAITING FOR ARM TIMER LEFT = %d", gArmTimerLeft)
					RobotSleep(1, pDomainModuleThread);
					break;
				}

				if( 0 != gArmTimerRight)
				{
					ROBOT_LOG( TRUE,"WAITING FOR ARM TIMER RIGHT = %d", gArmTimerRight)
					RobotSleep(1, pDomainModuleThread);
					break;
				}

				if( 0 != gBehaviorTimer)
				{
					ROBOT_LOG( TRUE,"WAITING FOR BEHAVIOR TIMER = %d", gBehaviorTimer)
					RobotSleep(1, pDomainModuleThread);
					break;
				}
				

				switch( m_CurrentActionMode )
				{
					case ACTION_MODE_NONE:
					{
						break;	// Nothing to do
					}

					case ACTION_MODE_FOLLOW_PERSON:
					{	
						ActionFollowPerson();
						break;
					}

					case ACTION_MODE_TAKE_PHOTO:
					{	
						ActionTakePhoto();
						break;
					}

					case ACTION_MODE_COME_HERE:
					{	
						ActionComeHere();
						break;
					}					

					case ACTION_MODE_MOVE_WHILE_TALKING:
					{	
						// do small movements while talking to increase "life like"
						ActionMoveWhileTalking();
						break;
					}

					case ACTION_MODE_KARATE_DEMO:
					{	
						ActionKarate();
						break;
					}

					case ACTION_MODE_LIGHT_SABER:
					{	
						ActionLightSaber();
						break;
					}

					case ACTION_MODE_RUN_SCRIPT:
					{	
						ActionRunScript();
						break;
					}

					case ACTION_MODE_PICKUP_OBJECTS:
					{	
						ActionPickupObjects();
						break;
					}

					case ACTION_MODE_PICKUP_CLOSE_OBJECT:
					{	
						ActionPickupCloseObject();
						break;
					}

					case ACTION_MODE_OPEN_DOOR:
					{	
						ActionOpenDoor();
						break;
					}

					case ACTION_MODE_GET_BEER:
					{	
						ActionGetBeer();
						break;
					}
					case ACTION_MODE_FIND_DOCK:
					{	
						ActionFindDock();
						break;
					}					
					case ACTION_MODE_TURN_TO_COMPASS_DIR:
					{	
						ActionTurnToCompassDir(m_ActionParam); // Pass in Direction COMPASS_ROSE (eg. NORTH_WEST)
						break;
					}
					case ACTION_MODE_POINT_TO_COMPASS_DIR: // Point with hand
					{	
						ActionPointToCompassDir(m_ActionParam); // Pass in Direction COMPASS_ROSE (eg. NORTH_WEST)
						break;
					}					
					case ACTION_MODE_TURN_TO_COMPASS_DEGREES: // Point with hand
					{	
						ActionTurnToCompassDegrees(m_ActionParam); // Pass in Direction DEGREES
						break;
					}
					case ACTION_MODE_WHAT_TIME_IS_IT: 
					{	
						ActionWhatTimeIsIt(); 
						break;
					}
					case ACTION_MODE_FREAK_OUT: 
					{	
						ActionFreakOut(); 
						break;
					}
					case ACTION_MODE_BAD_ROBOT: 
					{	
						ActionBadRobot(); 
						break;
					}

					case ACTION_MODE_WAKE_UP: 
					{	
						ActionWakeUp(); 
						break;
					}
					case ACTION_MODE_GO_TO_SLEEP:
					{	
						ActionGoToSleep(); 
						break;
					}
					case ACTION_MODE_TELL_JOKES:
					{
						ActionTellJokes( (BOOL)m_ActionParam ); // Single or multiple jokes
						break;
					}
					case ACTION_MODE_CHAT_DEMO_WITH_ADULT:
					{
						//m_PhraseToSpeak = 0;
						//m_RandomPhrase = 0;
						m_bSpeakingToChild = FALSE;						
						ActionDemoChat(); 
						break;
					}
					case ACTION_MODE_CHAT_DEMO_WITH_CHILD:
					{
						//m_PhraseToSpeak = 0;
						//m_RandomPhrase = 0;
						m_bSpeakingToChild = TRUE;
						ActionDemoChat(); 
						break;
					}
					case ACTION_MODE_EXPLORE:
					{	
						// just go forward with avoidance on until told to stop
						ROBOT_DISPLAY( TRUE, "ACTION_MODE_EXPLORE: Avoid Objects Enabled!")
						m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_FWD_MED, TURN_CENTER );
						//SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable
						break;
					}

					default:
					{
						ROBOT_DISPLAY( TRUE, "ERROR! BAD ACTION MODE! (%02X)", m_CurrentActionMode )
					}

				}	// switch m_CurrentActionMode
			} // If !ACTION_MODE_NONE
			else
			{
				// Do Idle processing for Humanoid robots only
				#if ( ROBOT_TYPE == LOKI )

				if( gEnableIdleMovements )
				{
					if( (gHeadIdle) && (0 == gHeadIdleTimer) )
					{
						m_pHeadControl->DoIdleMovement( HEAD_OWNER_RANDOM_MOVEMENT );
						gHeadIdleTimer = 4;	// 1/10 second per count!
					}
					if( m_pArmControlRight->IdleArmMovementEnabled() )
					{
						m_pArmControlRight->DoIdleArmMovement();
					}
					if( m_pArmControlLeft->IdleArmMovementEnabled() )
					{
						m_pArmControlLeft->DoIdleArmMovement();
					}
				}
				#endif
			}

			if( (0 != m_CurrentTask) || (0 != m_TaskState) )
			{
				//ROBOT_LOG( TRUE,"BEHAVIOR MODULE: TASK = %d, TASKSTATE = %d\n", m_CurrentTask, m_TaskState)
			}

			break;
		}	//	case WM_ROBOT_SENSOR_STATUS_READY:
		break;
	}	// Switch uMsg
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Behavioral Tasks
// These can be called as subroutines to some larger task
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::TaskRunScript()
{
	// This function executes one script line at a time, and waits as needed for the Robot to complete the step
	// Note:  Servo NOP = MAX_INT = 2147483647 in the scripts

	// See if we are ready to do the next step
	if( 0 != gBehaviorTimer )
	{
		ROBOT_LOG( TRUE,"SCRIPT[%d] WAITING FOR BEHAVIOR TIMER %d\n", m_ScriptLineNumber, gBehaviorTimer )
		return;
	}

	if( !m_pArmControlLeft->CheckArmPosition(FALSE) ) // Verbose?
	{
		ROBOT_LOG( TRUE,"SCRIPT[%d] WAITING FOR LEFT ARM\n", m_ScriptLineNumber )
		return;
	}
	if( !m_pArmControlRight->CheckArmPosition(FALSE) )
	{
		ROBOT_LOG( TRUE,"SCRIPT[%d] WAITING FOR RIGHT ARM\n", m_ScriptLineNumber )
		return;
	}

////////////////////////////////////////////////////////////////////////////
// Loop starts here
NEXT_SCRIPT_COMMAND: // allow processing of multiple script lines as needed

	m_ScriptLineNumber++;

	// Get Next Script Command
	char TextLine[SCRIPT_LINE_LENGTH_MAX];
	if( !ParseScriptFile(TextLine) )
	{
		// End of file
		m_CurrentTask = SCRIPT_TASK_CLEAN_UP;
		m_TaskState = 1;
		return; // done, do clean up
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Script formatting: :AB data, data, data
	char *pTextLine = &TextLine[1];  // get past the ':'
	switch( *pTextLine++ ) // get first Char, the Device (Right/Left Arm, Head, Wheels, etc.)
	{
		case'D':	// Delay
		{
			int DelayTenthSeconds = 0;
			sscanf_s(pTextLine, "%ld", &DelayTenthSeconds );
			if( DelayTenthSeconds > 0 )
			{
				gBehaviorTimer = DelayTenthSeconds;
				ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Delay = %d tenth seconds\n", m_ScriptLineNumber, DelayTenthSeconds ) 
			}
			return; // Delay starts immediately
		}

		case'X':	// eXecute pending servo commands (sync all the commands to start at once)
		{			// does not handle motor commands
			if( m_HeadExecutePending )
			{
				if( m_HeadSpeedPending )
					m_pArmControlRight->ExecutePositionAndSpeed();
				else
					m_pArmControlRight->ExecutePosition();
			}
			if( m_LeftArmExecutePending )
			{
				if( m_LeftArmSpeedPending )
					m_pArmControlRight->ExecutePositionAndSpeed();
				else
					m_pArmControlRight->ExecutePosition();
			}
			if( m_RightArmExecutePending )
			{
				if( m_RightArmSpeedPending )
					m_pArmControlRight->ExecutePositionAndSpeed();
				else
					m_pArmControlRight->ExecutePosition();
			}
			// Clear all the flags
			m_HeadExecutePending = m_HeadSpeedPending = m_LeftArmExecutePending = m_LeftArmSpeedPending = m_RightArmExecutePending = m_RightArmSpeedPending = FALSE;
			return; // Start Executing Servo moves!
		}
		// m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
		// void SetMoveDistance( int  Module, int Speed, int Turn, int  DistanceTenthInches, BOOL StopAfterMove = TRUE );
		// void SetTurnRotation( int  Module, int Speed, int Turn, int TurnAmountDegrees, BOOL StopAfterTurn = TRUE );

		case'W':	// Wheel Motors
		{
			switch( *pTextLine++ ) // get next Char, the Command type (Delay, Speed, Move, etc.)
			{
				case'M':	// Move set distance :WM 
				{
					int Speed = 0; int Turn = 0; int DistanceTenthInches = 0;  int StopAfterMove = 1;
					sscanf_s(pTextLine, "%ld,%ld,%ld%,ld", &Speed, &Turn, &DistanceTenthInches, &StopAfterMove );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] MoveDistance : Speed=%d, DistanceTenthInches=%d, StopAfterMove=%d\n", m_ScriptLineNumber,  
						Speed, Turn, DistanceTenthInches, StopAfterMove )
					m_pDriveCtrl->SetMoveDistance( BEHAVIOR_GOAL_MODULE, Speed, Turn, DistanceTenthInches, (BOOL)StopAfterMove );
					return; // Starts Executing Immediately!
				}
				case'T':	// Turn set distance
				{	
					int Speed = 0; int Turn = 0; int TurnAmountDegrees = 0;  int StopAfterTurn = 1;
					sscanf_s(pTextLine, "%ld,%ld,%ld%,ld", &Speed, &Turn, &TurnAmountDegrees, &StopAfterTurn );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] TurnRotation : Speed=%d, TurnAmountDegrees=%d, StopAfterTurn=%d\n", m_ScriptLineNumber, 
						Speed, Turn, TurnAmountDegrees, StopAfterTurn )
					m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, Speed, Turn, TurnAmountDegrees, (BOOL)StopAfterTurn );
					return; // Starts Executing Immediately!
				}
			}
		} // Wheel Motors

		case'H':	// Head
		{
			switch( *pTextLine++ ) // get next Char, the Command type (Delay, Speed, Move, etc.)
			{
				case'S':	// servo Speed
				{
					// Set Speed
					int PanSpeed = NOP; int TiltSpeed = NOP; int SideTiltSpeed = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld", &PanSpeed, &TiltSpeed, &SideTiltSpeed );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Head Speed: Pan=%d, Tilt=%d, SideTilt=%d\n", m_ScriptLineNumber, PanSpeed, TiltSpeed, SideTiltSpeed )
					m_pHeadControl->SetHeadSpeed(HEAD_OWNER_BEHAVIOR_P2, PanSpeed, TiltSpeed, SideTiltSpeed);
					m_HeadSpeedPending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
				case'P':	// servo Position
				{	
					// Move Head
					int Pan = NOP; int Tilt = NOP; int SideTilt = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld", &Pan, &Tilt, &SideTilt );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Head Speed: Pan=%d, Tilt=%d, SideTilt=%d\n", m_ScriptLineNumber, Pan, Tilt, SideTilt )
					m_pHeadControl->SetHeadPosition(HEAD_OWNER_BEHAVIOR_P2, Pan, Tilt, SideTilt);
					m_HeadExecutePending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
			}
		} // Head

		case'L':	// Left Arm
		{
			switch( *pTextLine++ ) // get next Char, the Command type (Delay, Speed, Move, etc.)
			{
				case'S':	// servo Speed
				{
					// Set Speed
					int ShoulderRotateSpeed = NOP; int ElbowRotateSpeed = NOP; int ElbowBendSpeed = NOP; int WristRotateSpeed = NOP; int GripSpeed = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld,%ld,%ld", &ShoulderRotateSpeed, &ElbowRotateSpeed, &ElbowBendSpeed, &WristRotateSpeed, &GripSpeed );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Left Arm Speed: Shoulder=%d, ElbowRotate=%d, Elbow=%d, Wrist=%d, Grip=%d\n", m_ScriptLineNumber, 
						ShoulderRotateSpeed, ElbowRotateSpeed, ElbowBendSpeed, WristRotateSpeed, GripSpeed)
					m_pArmControlLeft->SetArmSpeed( ShoulderRotateSpeed, ElbowRotateSpeed, ElbowBendSpeed, WristRotateSpeed, GripSpeed );
					m_LeftArmSpeedPending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
				case'P':	// servo Position
				{	
					// Move Arm
					int ShoulderRotate = NOP; int ElbowRotate = NOP; int ElbowBend = NOP; int WristRotate = NOP; int Grip = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld,%ld,%ld", &ShoulderRotate, &ElbowRotate, &ElbowBend, &WristRotate, &Grip );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Left Arm: Shoulder=%d, ElbowRotate=%d, Elbow=%d, Wrist=%d, Grip=%d\n", m_ScriptLineNumber, 
						ShoulderRotate, ElbowRotate, ElbowBend, WristRotate, Grip)
					m_pArmControlLeft->SetArmPosition( ShoulderRotate, ElbowRotate, ElbowBend, WristRotate, Grip );
					m_LeftArmExecutePending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
			}
		} // Left Arm

		case'R':	// Right Arm
		{
			switch( *pTextLine++ ) // get next Char, the Command type (Delay, Speed, Move, etc.)
			{
				case'S':	// Speed of servos
				{
					// Set Speed
					int ShoulderRotateSpeed = NOP; int ElbowRotateSpeed = NOP; int ElbowBendSpeed = NOP; int WristRotateSpeed = NOP; int GripSpeed = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld,%ld,%ld", &ShoulderRotateSpeed, &ElbowRotateSpeed, &ElbowBendSpeed, &WristRotateSpeed, &GripSpeed );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Right Arm Speed: Shoulder=%d, ElbowRotate=%d, Elbow=%d, Wrist=%d, Grip=%d\n", m_ScriptLineNumber, 
						ShoulderRotateSpeed, ElbowRotateSpeed, ElbowBendSpeed, WristRotateSpeed, GripSpeed)
					m_pArmControlRight->SetArmSpeed( ShoulderRotateSpeed, ElbowRotateSpeed, ElbowBendSpeed, WristRotateSpeed, GripSpeed );
					m_RightArmSpeedPending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
				case'P':	// servo Position
				{	
					// Move Arm
					int ShoulderRotate = NOP; int ElbowRotate = NOP; int ElbowBend = NOP; int WristRotate = NOP; int Grip = NOP;
					sscanf_s(pTextLine, "%ld,%ld,%ld,%ld,%ld", &ShoulderRotate, &ElbowRotate, &ElbowBend, &WristRotate, &Grip );
					ROBOT_LOG( DEBUG_SCRIPT_SHOW_COMMANDS,  "SCRIPT[%d] Right Arm: Shoulder=%d, ElbowRotate=%d, Elbow=%d, Wrist=%d, Grip=%d\n", m_ScriptLineNumber, 
						ShoulderRotate, ElbowRotate, ElbowBend, WristRotate, Grip)
					m_pArmControlRight->SetArmPosition( ShoulderRotate, ElbowRotate, ElbowBend, WristRotate, Grip );
					m_RightArmExecutePending = TRUE;
					goto NEXT_SCRIPT_COMMAND; // keep processing until eXecute command
				}
			}
		} // Right Arm

	} // switch Device Type 
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::TaskDoKarate()
{
	// See if we are ready to do the next step
	if( (0 != gArmTimerLeft) || (0 != gArmTimerRight) )
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR TIMER TimerL = %d, TimerR = %d m_TaskState = %d\n", gArmTimerLeft, gArmTimerRight, m_TaskState)
	}

	if( !m_pArmControlLeft->CheckArmPosition(FALSE) ) // Verbose?
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR LEFT ARM\n")
	}
	if( !m_pArmControlRight->CheckArmPosition(FALSE) )
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR RIGHT ARM\n")
	}

	if( (0 == gArmTimerLeft) && 
		((!m_ArmWaitForMoveToCompleteLeft) || (m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
	{
		if( (0 == gArmTimerRight) && 
			((!m_ArmWaitForMoveToCompleteRight) || (m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
		{
			// Both arms in position, and timer(if any) expired.  Ready for next move
			m_ArmWaitForMoveToCompleteRight = TRUE;	// Default, can be overridden below
			m_ArmWaitForMoveToCompleteLeft = TRUE;

			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 2:
				case 4:
				case 6:
				case 8:
				case 10:
				case 12:
				case 14:
				case 16:
				case 18:
				case 20:
				{
					// Wait between each move
					gArmTimerLeft = 5;	// 1/10 second per count!
					m_TaskState++;
					ROBOT_LOG( TRUE,"m_TaskState = %d\n",  m_TaskState)
					break;

				}
				case 1: // Move to first Karate state
				{
					m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
					m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 80, 0, 90, 5, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	
					m_pArmControlLeft->ExecutePositionAndSpeed();
					m_pArmControlLeft->SetArmPosition( 80, 0, 90, 5, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 3: // Arms in front
				{
					m_pArmControlRight->SetArmPosition( 90, -30, 90, 5, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 40, -60, 90, 5, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 5: // Arm above head
				{
					// Disabled for now!
					//m_pArmControlRight->SetArmPosition( 180, -90, 85, 140, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlRight->ExecutePosition();
					//m_pArmControlRight->SetArmPosition( -30, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlRight->ExecutePosition();

					//m_pArmControlLeft->SetArmPosition( 90, -90, 80, 0, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlLeft->ExecutePosition();
					m_TaskState = 7;	// SKIP TO NEXT STATE!
					break;
				}
				case 7: // Back in Front
				{
					m_pArmControlRight->SetArmPosition( 90, -30, 90, 5, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 40, -60, 90, 5, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 9: // Punch Home Position
				{
					m_pArmControlRight->SetArmPosition( -30, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( -40, 2, 130, 0, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 11: // Punch Right
				{
 					//m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MAX, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	
					//m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 110, 5, 45, 0, 1 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 13: // Punch Home
				{
					m_pArmControlRight->SetArmPosition( -40, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 15: // Extend slightly before next move
				{
					//m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
					//m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	
					//m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 30, 0, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 30, 0, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 17: // Arms Crossed
				{
					m_pArmControlRight->SetArmPosition( 30, -65, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 50, -65, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 19: // Straghten arm before next move
				{
					m_pArmControlRight->SetArmPosition( 30, 0, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 50, 0, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}

				case 21: // Arms Home
				{
					RightArmHome();
					LeftArmHome();
					m_TaskState = 0; // Done
					break;
				}
/*
				case 10: // 
				{
					m_pArmControlRight->SetArmPosition( 0, 0, 0, 0, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();
					m_ArmMovementStateRight++;
					m_ArmWaitForMoveToCompleteRight = TRUE;

					m_pArmControlLeft->SetArmPosition( 0, 0, 0, 0, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_ArmMovementStateLeft++;
					m_ArmWaitForMoveToCompleteLeft = TRUE;
					break;
				}
*/
				default:
				{
					ROBOT_ASSERT(0);
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::TaskOpenDoor()
{
	// Called upon sensor updates.
	// See if we are ready to do the next step
	if( (0 != gArmTimerLeft) || (0 != gArmTimerRight) )
	{
		ROBOT_LOG( TRUE,"TimerL = %d, TimerR = %d m_TaskState = %d\n", gArmTimerLeft, gArmTimerRight, m_TaskState)
	}

	if( (0 == gArmTimerLeft) && 
		((!m_ArmWaitForMoveToCompleteLeft) || (m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
	{
		if( (0 == gArmTimerRight) && 
			((!m_ArmWaitForMoveToCompleteRight) || (m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
		{
			// Both arms in position, and timer(if any) expired.  Ready for next move
			m_ArmWaitForMoveToCompleteRight = TRUE;	// Default, can be overridden below
			m_ArmWaitForMoveToCompleteLeft = TRUE;

			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 2:
				case 4:
				case 6:
				case 8:
				case 10:
				case 12:
				case 14:
				{
					// Wait between each move
					gArmTimerLeft = 2;	// 1/10 second per count!
					m_TaskState++;
					ROBOT_LOG( TRUE,"m_TaskState = %d\n",  m_TaskState)
					break;

				}
				case 1: // Move to first Door state
				{
					//m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
					//m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 126, 0, -30, -90, 85 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 3: // find door handle
				{
					m_pArmControlRight->SetArmPosition( 119, 0, -30, -90, 85  ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 5: // close claw
				{
					m_pArmControlRight->SetArmPosition( NOP, NOP, NOP, NOP, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 7: // rotate wrist
				{
					m_pArmControlRight->SetArmPosition( NOP, NOP, NOP, -40, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 9: //
				{
					m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MAX, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	
					m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 100, 0, -40, -140, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
//BACK UP and then
				case 11: // 
				{
					m_pArmControlRight->SetArmPosition( 124, 0, -35, -120, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 13: // Let loose of handle, just one finger to pull
				{
					m_pArmControlRight->SetArmPosition( 122, NOP, NOP, -95 , 85 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
// Backup more and then release completely
				case 15: //
				{
					m_pArmControlRight->SetArmPosition( 130, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
// retract arm a bit
				case 17: //
				{
					m_pArmControlRight->SetArmPosition( 130, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
// turn right and then exten arm down
				case 19: //
				{
					m_pArmControlRight->SetArmPosition( 90, NOP, 0, NOP, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					// Keep left arm out of the way
					m_pArmControlRight->SetArmPosition( -30, -4, 120, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
// Move forward again, close to the door and then
				case 21: //
				{
					m_pArmControlRight->SetArmPosition( 30, -30, 85, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
// Rotate robot to open door fully, then rotate back, and put arms in normal position

				case 23: // Arms Home
				{
					RightArmHome();
					LeftArmHome();
					m_TaskState = 0; // Done
					break;
				}
				// Ready to go through the doorway!
/*
				case 10: // 
				{
					m_pArmControlRight->SetArmPosition( 0, 0, 0, 0, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();
					m_ArmMovementStateRight++;
					m_ArmWaitForMoveToCompleteRight = TRUE;

					m_pArmControlLeft->SetArmPosition( 0, 0, 0, 0, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_ArmMovementStateLeft++;
					m_ArmWaitForMoveToCompleteLeft = TRUE;
					break;
				}
*/
				default:
				{
					ROBOT_ASSERT(0);
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns Distance to move and Degrees to turn to reach object
// NOTE: Supports LEFT ARM Pickup ONLY!

void CBehaviorModule::CalculatePathToObjectPickup( int ObjectX, int ObjectY, int &DistanceTenthInches, double &DirectionDegrees )
{
	// Offset for Right or Left arm pickup
	double OffsetDistanceTenthInches = (double)( HALF_ROBOT_BODY_WIDTH_TENTH_INCHES + HALF_ROBOT_CLAW_WIDTH_TENTH_INCHES  ); // add fudge for claw size

	double X = ObjectX; // Tenthinches
	double Y = ObjectY; // 

	// translate X so robot end up with Left arm over the object
	X += OffsetDistanceTenthInches;

	// calculate direction and distance to object pickup location
	DistanceTenthInches = (int)((sqrt((X*X) + (Y*Y))) );
	double AngleRadians = atan2( X, Y );	// X,Y backward from classical math
	DirectionDegrees = AngleRadians * RADIANS_TO_DEGREES;

	ROBOT_LOG( TRUE,"------------------------------------------------------\n")
	ROBOT_LOG( TRUE,"ACTION_MODE_PICKUP_OBJECTS: For X=%4.1f, Y=%4.1f: Move Distance = %4d  Move Angle = %4.1f (all in inches)\n\n", 
		(X/10.0), (Y/10.0), (DistanceTenthInches/10), DirectionDegrees)

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// High level Actions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionKarate()
{
	// Called upon sensor updates.

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	// Karate Demo only has one task (since it's just a demo), but several Task States
		{
			//m_CurrentTask++; 
			TaskDoKarate();
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}
}

void CBehaviorModule::ActionLightSaber()
{
	// Called upon sensor updates.

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	// Reach hand up to get the light saber from the user
		{
			//m_CurrentTask++; 
			TaskGetLightSaber();
			break;
		}
		case 2:	// Do the demo fight
		{
			//m_CurrentTask++; 
			TaskDoLightSaberDemo();
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::TaskGetLightSaber()
{
	// See if we are ready to do the next step
	if( !m_pArmControlRight->CheckArmPosition(FALSE) )
	{
		ROBOT_LOG( TRUE,"LIGHT SABER WAITING FOR RIGHT ARM\n")
	}

	if( (0 == gArmTimerRight) && 
		((!m_ArmWaitForMoveToCompleteRight) || (m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
	{
		// Right arm in position, and timer(if any) expired.  Ready for next move
		m_ArmWaitForMoveToCompleteRight = TRUE;	// Default, can be overridden below

		switch( m_TaskState )
		{
			case 0:
			{
				break;	// Nothing to do
			}

			case 1: // Extend Arm - same as ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW
			{
				ROBOT_LOG( TRUE,"\n===========> TaskGetLightSaber Started \n")
				m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_TaskState++;
				break;
			}

			case 2: // Wait for Claw sensor to detect something
					// (will usually note person's hand as they provide the object)
			{
				if( g_pNavSensorSummary->nObjectClawRight < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES  )	// 
				{
					// Object detected.  Close claw.
					ROBOT_LOG( TRUE,"Hand Object detected. Closing Claw\n")
					m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
					m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePositionAndSpeed();
					m_TaskState++;
				}
				// Otherwise, just continue to wait
				break;
			}

			case 3:	// Claw closed.  We are holding the Light Saber!
			{
					m_TaskState = 0; // back to Idle state
					ROBOT_LOG( TRUE,"LightSaber in Hand\n")
			//	}
				break;
			}
			default:
			{
				ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
				ROBOT_ASSERT(0);
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::TaskDoLightSaberDemo()
{
	// See if we are ready to do the next step
	if( (0 != gArmTimerLeft) || (0 != gArmTimerRight) )
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR TIMER TimerL = %d, TimerR = %d m_TaskState = %d\n", gArmTimerLeft, gArmTimerRight, m_TaskState)
	}

	if( !m_pArmControlLeft->CheckArmPosition(FALSE) ) // Verbose?
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR LEFT ARM\n")
	}
	if( !m_pArmControlRight->CheckArmPosition(FALSE) )
	{
		ROBOT_LOG( TRUE,"KARATE WAITING FOR RIGHT ARM\n")
	}

	if( (0 == gArmTimerLeft) && 
		((!m_ArmWaitForMoveToCompleteLeft) || (m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
	{
		if( (0 == gArmTimerRight) && 
			((!m_ArmWaitForMoveToCompleteRight) || (m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
		{
			// Both arms in position, and timer(if any) expired.  Ready for next move
			m_ArmWaitForMoveToCompleteRight = TRUE;	// Default, can be overridden below
			m_ArmWaitForMoveToCompleteLeft = TRUE;

			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}

				case 1: // Move to first Karate state
				{
					m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
					m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 80, 0, 90, 5, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	
					m_pArmControlLeft->ExecutePositionAndSpeed();
					m_pArmControlLeft->SetArmPosition( 80, 0, 90, 5, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 3: // Arms in front
				{
					m_pArmControlRight->SetArmPosition( 90, -30, 90, 5, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 40, -60, 90, 5, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 5: // Arm above head
				{
					// Disabled for now!
					//m_pArmControlRight->SetArmPosition( 180, -90, 85, 140, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlRight->ExecutePosition();
					//m_pArmControlRight->SetArmPosition( -30, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlRight->ExecutePosition();

					//m_pArmControlLeft->SetArmPosition( 90, -90, 80, 0, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					//m_pArmControlLeft->ExecutePosition();
					m_TaskState = 7;	// SKIP TO NEXT STATE!
					break;
				}
				case 7: // Back in Front
				{
					m_pArmControlRight->SetArmPosition( 90, -30, 90, 5, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 40, -60, 90, 5, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 9: // Punch Home Position
				{
					m_pArmControlRight->SetArmPosition( -30, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( -40, 2, 130, 0, 10 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 11: // Punch Right
				{
 					//m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MAX, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	
					//m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 110, 5, 45, 0, 1 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_TaskState++;
					break;
				}
				case 13: // Punch Home
				{
					m_pArmControlRight->SetArmPosition( -40, 0, 130, 0, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 15: // Extend slightly before next move
				{
					//m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
					//m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	
					//m_pArmControlRight->ExecutePositionAndSpeed();
					m_pArmControlRight->SetArmPosition( 30, 0, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 30, 0, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 17: // Arms Crossed
				{
					m_pArmControlRight->SetArmPosition( 30, -65, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 50, -65, 90, 120, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}
				case 19: // Straghten arm before next move
				{
					m_pArmControlRight->SetArmPosition( 30, 0, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePosition();

					m_pArmControlLeft->SetArmPosition( 50, 0, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePosition();
					m_TaskState++;
					break;
				}

				case 21: // Arms Home
				{
					RightArmHome();
					LeftArmHome();
					m_TaskState = 0; // Done
					break;
				}

				default:
				{
					ROBOT_ASSERT(0);
				}
			}
		}
	}
}


void CBehaviorModule::ActionRunScript()
{
	// Called upon sensor updates.
	// Runs next step of whatever script is currently executing

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case SCRIPT_TASK_OPEN_FILE:	// Open the file to run
		{
			// Open the file for reading
			errno_t nError;
			nError = fopen_s( &m_ScriptFileHandle, g_ScriptToRun, "r" ); // Read mode
			if( 0 != nError )
			{
				ROBOT_LOG( TRUE, "Could Not open Script File (%s)!  Aborting.\n", g_ScriptToRun )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				break;
			}
			else
			{
				ROBOT_LOG( TRUE,  "Script file Opened: %s\n", g_ScriptToRun)
			}
			m_ScriptLineNumber = 0; // get ready to run the script
			m_CurrentTask++; 
			break;
		}
		case SCRIPT_TASK_RUN_SCRIPT:	// Scripts have mostly one task (since it's just a demo), but several Task States
		{
			if( NULL == m_ScriptFileHandle )
			{
				ROBOT_ASSERT( 0 );
			}
			TaskRunScript();
			break;
		}
		case SCRIPT_TASK_CLEAN_UP:	// Clean up and exit
		{
			// Close script file previously opened
			if( NULL == m_ScriptFileHandle )
			{
				ROBOT_DISPLAY( TRUE, "Script File Not Opened.  Nothing to Close" )
			}
			else
			{
				if( fclose( m_ScriptFileHandle ) )
				{
					ROBOT_DISPLAY( TRUE, "Error Closing Script File (%s)!\n", g_ScriptToRun )
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "Script File (%s) Closed.\n", g_ScriptToRun )
				}
				m_ScriptFileHandle = NULL;
			}

			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode

			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionTakePhoto( )
{
	// Turn robot to the direction requested
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			m_HeadNodEnabled = FALSE; // Disable Head Nod, so the camera does not shake

			// Tilt head down a bit to get the person's body in the picture
			int CurrentTilt = m_pHeadControl->GetTiltPosition();
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, NOP, (CurrentTilt - 100), CAMERA_SIDETILT_CENTER ); // Pan, Tilt, SideTilt
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			RobotSleep(1, pDomainModuleThread);

			SpeakText( "Ok" );
			gBehaviorTimer = 10; // 1/10 seconds - give time for person(s) to smile
			m_CurrentTask++; 
			break;
		}

		case 2:	//
		{
			// Respond with random phrases
			int RandomNumber = ((2 * rand()) / RAND_MAX);
			ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
			switch( RandomNumber )
			{
				case 0:  SpeakText( "Say Cheese" );break;
				default: SpeakText( "Say Cheese" ); // If const is larger number, this gets called more often
			}
			gBehaviorTimer = 20; // 1/10 seconds - give time for person(s) to smile
			m_CurrentTask++; 
			break;
		}

		case 3:
		{
			// Play Camera click sound
			ROBOT_LOG( TRUE, "Debug 3")
			LPCTSTR pFileName = CAMERA_CLICK_STR;
			
			ROBOT_LOG( TRUE,  "Play sound %s\n", pFileName)
			DWORD dwOptions = SND_ASYNC | SND_FILENAME | SND_NODEFAULT ;	// SND_NOWAIT
			BOOL bSucess = PlaySound(
				pFileName,		// Filename  
				NULL,			// No device Handle needed   
				dwOptions);		// Control Flags
			if( !bSucess )
			{
				ROBOT_LOG( TRUE,  ">>>> ERROR: Could not play sound %s!\n", pFileName)
			}
			SendCommand( WM_ROBOT_CAMERA_TAKE_SNAPSHOT, 0, 0 );
			m_CurrentTask++; 
			break;
		}
		case 4:
		{
			// Done
			//ROBOT_LOG( TRUE, "Debug 4")
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			m_HeadNodEnabled = TRUE;
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionFollowPerson()
{
	// Confirm that Kinect is working, and I have a lock on the person in front of me
	// Confirm that Object Avoidance is enabled (highly recommended)
	// Remember the Human Number.  If we lose lock, ask user if it's OK to continue?
	// Turn robot toward person as needed
	// Maintain proper following distance 
	// When person says "Stop" Cancel this action.
	// use g_CurrentHumanTracked to identify which player (based upon who spoke last, while giving the command)
	//#define FOLLOW_PERSON_RETRIES  3

	int HumanPosX = 0; 
	int HumanPosY = 0; 
	int HumanPosZ = 0;
	int HumanPosAngleX = 0;
	//static int FollowPersonRetriesRemaining = FOLLOW_PERSON_RETRIES;

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			// Check to see if we have a person to track
			if( 0 == g_CurrentHumanTracked )
			{
				// No Player - ABORT
				SpeakText( "Bummer, I am unable to track you for following" );

				ROBOT_LOG( TRUE, "ACTION_MODE_FOLLOW_PERSON: FAILED!  g_CurrentHumanTracked == 0" )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				return;
			}
			else
			{
				// Found someone
				m_nHumanIDToTrack = g_CurrentHumanTracked; // Save, so we always track the same person
				//SpeakText( "OK, I will follow you" );
				ROBOT_LOG( TRUE, "ACTION_MODE_FOLLOW_PERSON: Lock on to human ID %d successful",  m_nHumanIDToTrack )


				// Make sure the Kinect stays tilted pointing at the user (does not point at ground by some other module)
				m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_HUMAN, KINECT_HUMAN_DETECT_START_POSITION );	// point Kinect where it can find humans
				m_pKinectServoControl->SetOwnerTimer(MAX_INT); // prevent ownership from timing out (requires explicit ReleaseOwner when done)


				// Optional: Point Eyes at human for realism.
				///	m_pHeadControl->SetHeadSpeed( HEAD_OWNER_KINECT_HUMAN, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
				///	m_pHeadControl->LookAtXYZ( HEAD_OWNER_KINECT_HUMAN, g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.x, 
				///		g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.y, (g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.z - 60) ); //  Look at Human eyes, not top of head
				//gKinectDelayTimer = 5; // tenth-seconds - Don't jerk around (or overload the CPU)
				m_CurrentTask++; 
			}
			break;
		}
		case 2:	// Following Human!
		{	
			// Make sure we have not lost the person we were tracking
			if( !g_HumanLocationTracking[m_nHumanIDToTrack].Found )
			{
				// Lost person we were tracking!
				SpeakText( "Oh No!  I lost track of you.  Stopping" );

				ROBOT_LOG( TRUE, "ACTION_MODE_FOLLOW_PERSON: Lost Lock on to human!  Stopping." )
				m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
				m_nHumanIDToTrack = 0;
				//FollowPersonRetriesRemaining = FOLLOW_PERSON_RETRIES; // setup count-down of retries
				//m_CurrentTask = 1; // retry
				m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_TRACK_HUMAN);
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				break;
			}

			// Get human position info
			HumanPosX = HumanPosY = HumanPosZ = 0;
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectHumanTrackingLock);
			EnterCriticalSection(&g_csKinectHumanTrackingLock);
				if( g_HumanLocationTracking[m_nHumanIDToTrack].Found )
				{
					// found the human we were tracking
					HumanPosAngleX = g_HumanLocationTracking[m_nHumanIDToTrack].AngleTenthDegrees.X;
					HumanPosX = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.X;
					HumanPosY = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.Y; 
					HumanPosZ = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.Z; 
				}
			LeaveCriticalSection(&g_csKinectHumanTrackingLock);
			__itt_task_end(pDomainModuleThread);

			// Looks like we found the Human.  Let's follow!
			//ROBOT_LOG( TRUE, "ACTION_MODE_FOLLOW_PERSON: Tracking Human ID %d ",  m_nHumanIDToTrack )

			// Turn and move as needed
			int NewSpeed = 0;
			int NewTurn = 0;
			int CurrentSpeed = m_pDriveCtrl->GetCurrentSpeed();
			int CurrentTurn = m_pDriveCtrl->GetCurrentTurn();
			//if( m_pDriveCtrl->RobotStopped() )

			///////////////////////////////////////
			// Calculate Turn needed
			/// FYI, TURN_LEFT_MAX	== -127
			int CalculatedTurn =  HumanPosAngleX / 8; // TenthDegrees.  TODO - TUNE THIS
			NewTurn = CalculatedTurn;

			if( HumanPosY < HUMAN_FOLLOW_DISTANCE_MIN_TENTH_INCHES+30 ) // add a bit more
			{
				// don't turn if person is too close
				NewTurn = 0;
			}

			if( (NewTurn > -3) && (NewTurn < 3) )
			{
				ROBOT_LOG( TRUE,"DEBUG TURN: IN DEADBAND %d\n", NewTurn)
				NewTurn = 0; // Don't move in the deadband
			}
			else if( NewTurn < TURN_LEFT_MED )	
			{
				NewTurn = TURN_LEFT_MED;
			}
			else if( NewTurn > TURN_RIGHT_MED )	
			{
				NewTurn = TURN_RIGHT_MED;
			}


			///////////////////////////////////////
			// Calculate Speed needed
			//int SpeedDelta = HumanPosY;

		//	int DistanceFromOptimal = (HumanPosY - HUMAN_FOLLOW_DISTANCE_TARGET_TENTH_INCHES);

			int SpeedDelta = (HumanPosY - HUMAN_FOLLOW_DISTANCE_TARGET_TENTH_INCHES) / 4;
			NewSpeed = CurrentSpeed + SpeedDelta;

			// Limit Check
			if( HumanPosY < HUMAN_FOLLOW_DISTANCE_MIN_TENTH_INCHES )
			{
				// Too close!
				if( m_pDriveCtrl->RobotStopped() )
				{
					// robot currently stopped, but too close. Move back a bit
					NewSpeed = SPEED_REV_SLOW;
				}
				else
				{
					NewSpeed = SPEED_STOP; // Stop fast!
				}
			}
			else if( NewSpeed < SPEED_FWD_VERY_SLOW )	 // SPEED_REV_MED_SLOW
			{
				NewSpeed = SPEED_STOP;	// SPEED_REV_MED
			}
			else if( NewSpeed > SPEED_FWD_MED ) // SPEED_FWD_MED_FAST
			{
				NewSpeed = SPEED_FWD_MED; // SPEED_FWD_MED_FAST
			}

			m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, NewSpeed, NewTurn );
					
			//m_CurrentTask++; // STAY IN THIS TASK
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}
	//break;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionComeHere()
{
	// Confirm that Kinect is working, and I have a lock on the person in front of me
	// Confirm that Object Avoidance is enabled (highly recommended)
	// Remember the Human Number.  If we lose lock, ask user if it's OK to continue?
	// Turn robot toward person as needed
	// Move to person within limit distance, then stop and exit behavior
	// When person says "Stop" Cancel this action.
	// use g_CurrentHumanTracked to identify which player (based upon who spoke last, while giving the command)
	//#define FOLLOW_PERSON_RETRIES  3

	int HumanPosX = 0; 
	int HumanPosY = 0; 
	int HumanPosZ = 0;
	int HumanPosAngleX = 0;

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			// Check to see if we have a person to track
			if( 0 == g_CurrentHumanTracked )
			{
				// No Player - ABORT
				SpeakText( "Bummer, I am unable to track you" );

				ROBOT_LOG( TRUE, "ACTION_MODE_COME_HERE: FAILED!  g_CurrentHumanTracked == 0" )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				return;
			}
			else
			{
				// Found someone
				m_nHumanIDToTrack = g_CurrentHumanTracked; // Save, so we always track the same person
				//SpeakText( "OK, I will follow you" );
				ROBOT_LOG( TRUE, "ACTION_MODE_COME_HERE: Lock on to human ID %d successful",  m_nHumanIDToTrack )

				// Make sure the Kinect stays tilted pointing at the user (does not point at ground by some other module)
				m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_HUMAN, KINECT_HUMAN_DETECT_START_POSITION );	// point Kinect where it can find humans
				m_pKinectServoControl->SetOwnerTimer(MAX_INT); // prevent ownership from timing out (requires explicit ReleaseOwner when done)
				m_CurrentTask++; 
			}
			break;
		}
		case 2:	// Human found, start moving!
		{	
			// Make sure we have not lost the person we were tracking
			if( !g_HumanLocationTracking[m_nHumanIDToTrack].Found )
			{
				// Lost person we were tracking!
				SpeakText( "Oh No!  I lost track of you.  Stopping" );

				ROBOT_LOG( TRUE, "ACTION_MODE_COME_HERE: Lost Lock on to human!  Stopping." )
				m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
				m_nHumanIDToTrack = 0;
				m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_TRACK_HUMAN);
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				break;
			}

			// Get human position info
			HumanPosX = HumanPosY = HumanPosZ = 0;
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectHumanTrackingLock);
			EnterCriticalSection(&g_csKinectHumanTrackingLock);
				if( g_HumanLocationTracking[m_nHumanIDToTrack].Found )
				{
					// found the human we were tracking
					HumanPosAngleX = g_HumanLocationTracking[m_nHumanIDToTrack].AngleTenthDegrees.X;
					HumanPosX = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.X;
					HumanPosY = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.Y; 
					HumanPosZ = g_HumanLocationTracking[m_nHumanIDToTrack].HeadLocation.Z; 
				}
			LeaveCriticalSection(&g_csKinectHumanTrackingLock);
			__itt_task_end(pDomainModuleThread);

			// Looks like we found the Human.  Let's approach!
			if( HumanPosY < HUMAN_APPROACH_DISTANCE_TARGET_TENTH_INCHES )
			{
				// We have arrived at human. Done!
				//SpeakText( "Now what?" );
				ROBOT_LOG( TRUE, "ACTION_MODE_COME_HERE: Arrived. Done with behavior." )
				m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
				m_nHumanIDToTrack = 0;
				m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_TRACK_HUMAN);
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
				break;
			}

			// Turn and move as needed
			int NewSpeed = SPEED_FWD_MED;
			int NewTurn = TURN_CENTER;
			//int CurrentSpeed = m_pDriveCtrl->GetCurrentSpeed();
			//int CurrentTurn = m_pDriveCtrl->GetCurrentTurn();

			///////////////////////////////////////
			// Calculate Turn needed
			/// FYI, TURN_LEFT_MAX	== -127
			int CalculatedTurn =  HumanPosAngleX / 8; // TenthDegrees.  TODO - TUNE THIS
			NewTurn = CalculatedTurn;

			if( HumanPosY < HUMAN_FOLLOW_DISTANCE_MIN_TENTH_INCHES+30 ) // add a bit more
			{
				// don't turn if person is too close
				NewTurn = 0;
			}

			if( (NewTurn > -3) && (NewTurn < 3) )
			{
				ROBOT_LOG( TRUE,"DEBUG TURN: IN DEADBAND %d\n", NewTurn)
				NewTurn = 0; // Don't move in the deadband
			}
			else if( NewTurn < TURN_LEFT_MED )	
			{
				NewTurn = TURN_LEFT_MED;
			}
			else if( NewTurn > TURN_RIGHT_MED )	
			{
				NewTurn = TURN_RIGHT_MED;
			}


			///////////////////////////////////////
			// Calculate Speed needed
			//int SpeedDelta = HumanPosY;
			NewSpeed = SPEED_FWD_MED;
			if( HumanPosY < HUMAN_FOLLOW_DISTANCE_TARGET_TENTH_INCHES )
			{
				NewSpeed = SPEED_FWD_MED_SLOW;
			}

			m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, NewSpeed, NewTurn );
			//m_CurrentTask++; // STAY IN THIS TASK
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}
	//break;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionMoveWhileTalking( ) 
{

	m_pArmControlLeft->EnableIdleArmMovement(TRUE);
	m_pArmControlRight->EnableIdleArmMovement(TRUE);
	m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode

	return;




	// Small arm and body movements while talking a lot
	// note, this runs forever, until cancelled!
	m_RepeatCount = 3;

	if( (0 != gArmTimerRight) || (m_ArmWaitForMoveToCompleteRight && !m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT)) )
	{
		// wait for timer or arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}

	if( (0 != gArmTimerLeft) || (m_ArmWaitForMoveToCompleteLeft && !m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT)) )
	{
		// wait for timer or arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}

	
	ROBOT_LOG( TRUE,"m_CurrentTask = %d", m_CurrentTask )


	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			ROBOT_LOG( TRUE,"\n===========>  Move while talking\n")

			m_pArmControlLeft->EnableIdleArmMovement(FALSE);
			m_pArmControlRight->EnableIdleArmMovement(FALSE);
			m_ArmWaitForMoveToCompleteLeft = TRUE;
			m_ArmWaitForMoveToCompleteRight = TRUE;
			//g_SpeechRecoBlocked = TRUE; // block speech reco while arms are moving (due to motor noise)

			// Send command to turn a bit
	/*		if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, TURN_LEFT_MED, 10, TRUE ) )
			{
				// Error, was not able to set the turn, but move arms anyway
				SpeakText( "ActionMoveWhileTalking: Turn command failed");
			}
			*/
			// Move Arms - start by putting both arms up a bit

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_KINECT_HUMAN, 0, 100, -100 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 20, 10, 115, 0, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			//m_pArmControlLeft->SetArmPosition( 110, 0, 108, 0, 60 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			//m_pArmControlLeft->ExecutePosition();	
			//gArmTimerRight = 50; // tenth seconds
			m_CurrentTask++; 
			break;
		}

		case 2:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, 100, 200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 15, 0, NOP, NOP, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( 20, 0, 108, 0, 60 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				
			//gArmTimerRight = 50; // tenth seconds
			m_CurrentTask++; 
			break;
		}

		case 3:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, 100, 200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			//m_pArmControlRight->SetArmPosition( 15, 0, NOP, NOP, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( NOP, -10, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				
			//gArmTimerRight = 50; // tenth seconds
			m_CurrentTask++; 
			break;
		}
		
		case 4:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, 100, 200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 25, 0, NOP, NOP, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( NOP, 10, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				
			gArmTimerRight = 50; // tenth seconds
			m_CurrentTask++; 
			break;
		}

		case 5:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, 100, 200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( NOP, 10, NOP, NOP, 30 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( NOP, 10, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				
			//gArmTimerRight = 50; // tenth seconds
			m_CurrentTask++; 
			break;
		}

		case 6:
		{
			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionTurnToCompassDir( int nCompassRose )
{
	// Turn robot to the direction requested

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			// Send command to turn
			int DesiredCompassHeading = CompassRoseToDegrees( nCompassRose );
			// calculate direction of turn.  Positive is right turn, Negative is Left turn
			int TurnDegrees = CalculateTurn(g_pFullSensorStatus->CompassHeading, DesiredCompassHeading);
/*			if( TurnDegrees > 0 )
			{
				if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_RIGHT_MED_SLOW, TurnDegrees) )
				{
					error - fix this
				}
				ROBOT_LOG( TRUE, "Turning Right from %d to %d degrees", g_pFullSensorStatus->CompassHeading, DesiredCompassHeading )
			}
			else
			{
				if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_LEFT_MED_SLOW, TurnDegrees) )
				{
					error - fix this
				}
				ROBOT_LOG( TRUE, "Turning Left from %d to %d degrees", g_pFullSensorStatus->CompassHeading, DesiredCompassHeading )
			}
*/
			if( !m_pDriveCtrl->SetTurnToCompassDirection( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_RIGHT_MED_SLOW, DesiredCompassHeading, TRUE ) )
			{
				// Error, was not able to set the turn
				SpeakText( "Turn command failed");

				ROBOT_LOG( TRUE, "Turn command failed" )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			}
			else
			{
				ROBOT_LOG( TRUE, "Turning to %d degrees", DesiredCompassHeading )
				m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P2 ); // Look forward while turning
				m_CurrentTask++; 
			}
			break;
		}

		case 2:
		{
			// Wait for drive control to report that we are done rotating
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				// Done with rotation
				CString strDirection("I am facing ");
				strDirection += CompassRoseToString( nCompassRose );
				SpeakText( strDirection );
				m_CurrentTask++; 
			}						
			break;
		}	

		case 3:
		{
			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionTurnToCompassDegrees( int DesiredCompassHeading )
{
	// Turn robot to the direction requested
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			// Send command to turn
			if( !m_pDriveCtrl->SetTurnToCompassDirection( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_LEFT_MED, DesiredCompassHeading, TRUE ) )
			{
				// Error, was not able to set the turn
				SpeakText( "Turn command failed");

				ROBOT_LOG( TRUE, "Turn command failed" )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			}
			else
			{
				ROBOT_LOG( TRUE, "Turning to %d degrees", DesiredCompassHeading )
				m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P2 ); // Look forward while turning
				m_CurrentTask++; 
			}
			break;
		}

		case 2:
		{
			// Wait for drive control to report that we are done rotating
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				// Done with rotation
				//CompassRoseToString( int nCompassRose );
			//	SpeakText( "I am facing North" );
				m_CurrentTask++; 
			}						
			break;
		}	

		case 3:
		{
			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionWhatTimeIsIt( )
{
	// look at arm (as if he had a watch) and report the time

	if( (0 != gArmTimerRight) || (m_ArmWaitForMoveToCompleteRight && !m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT)) )
	{
		// wait for timer or arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}

	ROBOT_LOG( TRUE,"m_CurrentTask = %d", m_CurrentTask )

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	
		{
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 ); // Look forward 
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

			m_ArmWaitForMoveToCompleteRight = TRUE;
			m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_FAST, NOP, NOP, NOP, NOP );	
			m_pArmControlRight->SetArmPosition( 70, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlRight->ExecutePositionAndSpeed();

			int RandomNumber = ((4 * rand()) / RAND_MAX);
			switch( RandomNumber )
			{
				case 0:  
					SpeakText( "Lets see"); 
	
					break;
				case 1:  
					SpeakText( "well, lets see"); 
	
					break;
				default: break; // some times, no response
			}

			m_CurrentTask++;
			break;
		}

		case 2:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, -420, 120 ); // Pan, Tilt, SideTilt - look at watch
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_ArmWaitForMoveToCompleteRight = TRUE;
			m_pArmControlRight->SetArmPosition( 70, -80, 90, -70, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlRight->ExecutePosition();
			m_CurrentTask++; 
			break;
		}
		case 3:
		{
			gArmTimerRight = 10; // 1/10 seconds - look at the watch for a bit
			m_CurrentTask++; 
			break;
		}
		case 4:
		{
			m_pHeadControl->CheckAndSetOwner(HEAD_OWNER_BEHAVIOR_P1); // Make sure head control does not time out
			CTime Time = CTime::GetCurrentTime();
			CString strTime;
			if( Time.GetHour() > 11 ) // 12:00 is PM
			{
				strTime.Format("The time is %d %d PM", Time.GetHour() - 12, Time.GetMinute() );
			}
			else
			{
				strTime.Format("The time is %d %d AM", Time.GetHour(), Time.GetMinute() ); // AM must be capital or it reads "em"
			}
			SpeakText( strTime );
			m_CurrentTask++; 
			break;
		}	

		case 5:
		{
			m_ArmWaitForMoveToCompleteRight = FALSE;
			m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 );
			//m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, -420, 120 ); // Pan, Tilt, SideTilt -420
			//m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			RightArmHome();
			gArmTimerRight = 30; // 1/10 seconds - look at the watch for a bit
			m_CurrentTask++; 
			break;
		}

		case 6:
		{
			//m_pHeadControl->CheckAndSetOwner(HEAD_OWNER_BEHAVIOR_P1); // Make sure head control does not time out
			//m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 );
			//RightArmHome();
			//LeftArmHome();
			/* TODO: put in a delay, then say one of:
			Now, wasnt that a great use of 1.16 billion transistors?
			150 lines of code, so you can ask me the time
			or, about xxx nanoseconds since I was created, which is really the important time frame, I think
			*/

			// Respond with random phrases
			static int RandomNumber = 0; // = ((3 * rand()) / RAND_MAX);
			switch( RandomNumber )
			{
				case 0:  
					SpeakText( "Now, was that really a good use of 1.16 billion transistors?");
					RandomNumber = 1;
					break;
				case 1:  
					SpeakText( "I have 25 thousand lines of code, so you can ask me the time?");
					RandomNumber = 0; // do it again next time
					break;
				default: break; // some times, no response?
			}

			m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	// restore
			m_pArmControlRight->ExecutePositionAndSpeed();

			m_pHeadControl->ReleaseOwner( HEAD_OWNER_BEHAVIOR_P1 );
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionBadRobot( )
{
	// Robot is ashamed.  Shake head side to side
	static int nHeadShakes = 0;

	ROBOT_LOG( TRUE,"m_CurrentTask = %d", m_CurrentTask )
	// Suppress Head Nod by preventing head ownership from timing out
	//gHeadOwnerTimer = 100; 

	// Save Current head position
	static int SavePan=0, SaveTilt=0, SaveSideTilt=0;

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	
		{
			m_pHeadControl->GetHeadPosition( SavePan, SaveTilt, SaveSideTilt );
			ROBOT_LOG( TRUE,"Saving Head Pos: %d, %d, %d", SavePan, SaveTilt, SaveSideTilt )

			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			//m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 ); // Look forward 

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 0, -350, 0 ); // Pan, Tilt, SideTilt - shake head
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
			gBehaviorTimer = 8; // 1/10 seconds
			nHeadShakes = 2; // number of times to shake head
			m_CurrentTask++;
			break;
		}

		case 2:	
		{
			SpeakText( "I am sorry");
			//m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			//m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
			m_CurrentTask++;
			break;
		}

		case 3:
		{
			/* // This is too slow, it looks "robotic"
			BOOL HeadInPosition = m_pHeadControl->CheckHeadPosition( FALSE ); // TRUE = Verbose
			if( !HeadInPosition )
			{
				//ROBOT_LOG( TRUE,"WAITING FOR HEAD MOVE TO COMPLETE")
				return;
			}
			*/

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, -350, 120 ); // Pan, Tilt, SideTilt - shake head
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			gBehaviorTimer = 6; // 1/10 seconds
			m_CurrentTask++; 
			break;
		}
		case 4:
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, -200, -350, -120 ); // Pan, Tilt, SideTilt - shake head
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			gBehaviorTimer = 6; // 1/10 seconds
			m_CurrentTask++; 
			break;
		}
		case 5:
		{
			if( --nHeadShakes <= 0 )
			{
				// Done shaking head
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, SavePan, SaveTilt, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
				Sleep(1); 
				gBehaviorTimer = 10; // 1/10 seconds - start moving before talking
				m_CurrentTask++; 
			}
			else
			{
				m_CurrentTask = 3;  // shake again
			}
			break;
		}

		case 6:
		{
			//HeadCenter(); // Don't center, instead use last position above
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
			SpeakText( "I will try to learn better behavior");
			gBehaviorTimer = 10; // 1/10 seconds - give time for head to move before releasing ownership
			m_CurrentTask++; 
			break;
		}
		case 7:
		{
			gBehaviorTimer = 5; // 1/10 seconds - give time for head to move before releasing ownership
			m_pHeadControl->ReleaseOwner( HEAD_OWNER_BEHAVIOR_P1 );
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionFreakOut( ) //danger!
{
	// Turn robot to the direction requested
	m_RepeatCount = 3;
	static int StartingDirection = -1;

	if( (0 != gArmTimerRight) || (m_ArmWaitForMoveToCompleteRight && !m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT)) )
	{
		// wait for timer or arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}

	if( (0 != gArmTimerLeft) || (m_ArmWaitForMoveToCompleteLeft && !m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT)) )
	{
		// wait for timer or arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}

	
	ROBOT_LOG( TRUE,"m_CurrentTask = %d", m_CurrentTask )


	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			ROBOT_LOG( TRUE,"\n===========> BehaviorModule: FREAK OUT\n")
			// Get arms ready to wave around
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
			m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	
			m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	

			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePositionAndSpeed();
			m_pArmControlLeft->ExecutePositionAndSpeed();

			m_pArmControlRight->EnableIdleArmMovement(FALSE);
			m_ArmWaitForMoveToCompleteLeft = TRUE;
			m_ArmWaitForMoveToCompleteRight = TRUE;
			g_SpeechRecoBlocked = TRUE; // block speech reco while arms are moving (due to motor noise)

			// Remember which way we are pointing now, so we can return to that position.
			StartingDirection = g_pFullSensorStatus->CompassHeading;
			// Send command to turn in a complete circle
			if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, TURN_LEFT_MED, 350, TRUE ) )
			{
				// Error, was not able to set the turn, but wave arms anyway
				SpeakText( "My Turn command failed");
			}

			// Wave Arms - start by putting both arms up

			m_pArmControlLeft->EnableIdleArmMovement(FALSE);
			m_pArmControlRight->EnableIdleArmMovement(FALSE);

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 0, 100, -200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 150, 0, 115, 0, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( 110, 0, 108, 0, 60 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				

			SpeakText( "when in danger when in doubt run in circles scream and shout, when in danger when in doubt run in circles scream and shout ");
			//SpeakText( "look out, its every robot for himself, when in danger when in doubt run in circles scream and shout ");


			SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE ); // Turn lights on
			m_CurrentTask++; 
			break;
		}

		case 2:
		{
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				m_CurrentTask = 5;  // Drive control reports that we are done rotating
				break;
			}						
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, 200, 100, 200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 110, 0, 80, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( 150, 0, 50, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			TRACE("\n ============ \n DEBUG: SET HEAD POSITION 2\n");
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();				
			SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)FALSE ); // Turn lights off

			m_CurrentTask++; 
			break;
		}

		case 3:
		{
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				m_CurrentTask = 5;  // Drive control reports that we are done rotating
				break;
			}						
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, -200, 100, -200 ); // Pan, Tilt, SideTilt TENTHDEGREES
			m_pArmControlRight->SetArmPosition( 150, 0, 50, 0, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlLeft->SetArmPosition( 110, 0, 80, 0, 60 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

			TRACE("\n ============ \n DEBUG: SET HEAD POSITION 3\n");
			m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );
			m_pArmControlRight->ExecutePosition();				
			m_pArmControlLeft->ExecutePosition();	
			SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE ); // Turn lights on

			if( --m_RepeatCount > 0 )
			{
				m_CurrentTask = 2; // do it again
			}
			else
			{
				m_CurrentTask++;
			}
			break;
		}

		case 4:
		{
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				m_CurrentTask++;  // Drive control reports that we are done rotating
			}						
			break;
		}	
		
		case 5:
		{
			RightArmHome();
			LeftArmHome();
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, 150, CAMERA_SIDETILT_CENTER  ); // Pan, Tilt, SideTilt
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

			g_SpeechRecoBlocked = FALSE; // don't block speech reco anymore
			// Respond with random phrases
			int RandomNumber = ((4 * rand()) / RAND_MAX);
			{
				switch( RandomNumber )
				{
					case 0:  
						SpeakText( "Woah, what just happened? I think I blew a circuit"); 
		
						break;
					case 1:  
						SpeakText( "How come you did not run in circles?"); 
		
						break;
					default:
						SpeakText( "Sorry about that, I guess I am not a very brave robot"); 
		
						break;
				}
			}
			// Return to start position.
			if( StartingDirection >= 0 ) 
			{
				m_ActionParam = StartingDirection;
				StartingDirection = -1;
				m_CurrentActionMode = ACTION_MODE_TURN_TO_COMPASS_DEGREES;
				m_CurrentTask = 1;
			}
/*			if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, TURN_LEFT_MED, 10, TRUE ) ) // finish the turn (KLUDGE)
			{
				// Error, was not able to set the turn
				SpeakText( "FREAK OUT: Turn command failed");
			}
*/
			else
			{
				m_CurrentTask++;
			}
			break;
		}
		case 6:
		{
			m_pHeadControl->ReleaseOwner( HEAD_OWNER_BEHAVIOR_P1 );
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}


		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/*
					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					ROBOT_LOG( TRUE, "DEBUG: RAND = %d\n", RandomNumber)
					switch( RandomNumber )
					{
						case 0:  Speak( "Turning Right" );break;
						case 1:  Speak( "How is this" );break;
						default: Speak( "Now what?" ); // If const is larger number, this gets called more often
					}
*/


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionWakeUp( )
{
	// Turn robot to the direction requested
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			m_CurrentTask++; 
			break;
		}

		case 2:
		{
			m_CurrentTask++; 
			break;
		}	

		case 3:
		{
			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define CHAT_DONE (-1)
#define CHAT_STD_DELAY 80 // tenth seconds
#define CHAT_STATE_WAVE_HELLO				1
#define CHAT_STATE_INT						2
#define CHAT_STATE_SAY_PHRASE				3
#define CHAT_STATE_WAIT_FOR_SPEECH_ENGINE	4
#define CHAT_STATE_DONE						5


void CBehaviorModule::ActionDemoChat()
{
	// ChatParam: 1 = Adult, 0 = Child.  Maybe add location?

	CString SpecialCmd;
	CString TextToSpeak;
	ROBOT_LOG( TRUE,"Chat State = %d \n", m_CurrentTask );
	// Set gBehaviorTimer to delay between steps

	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case CHAT_STATE_WAVE_HELLO:
		{
			// Wave and say Hello
			// DISABLED For now.  Instead either have robot shake hands or wave manually, then to this routine.
	//		SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute,
			if( m_SubTaskComplete )
			{
				// Done waiting until the shake hands task completes
				m_CurrentTask++;
			}
			break;
		}
		case CHAT_STATE_INT:	// command to start chatting with someone
		{
			// Get a number to randomize responses at the start of each conversation
			// This will be used for all responses until the next conversation
			m_RandomPhrase = ((4 * rand()) / RAND_MAX);
			ROBOT_LOG( TRUE,  "DEBUG: RAND = %d\n", m_RandomPhrase)

			//m_ResponseReceived = -1;
			//m_ResponsePending = FALSE;
			g_MoveArmsWhileSpeaking = TRUE;
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			m_pHeadControl->SetHeadPositionCenterHuman( HEAD_OWNER_BEHAVIOR_P1 ); // Look forward and slightly up
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

			SpecialCmd.Format( "[*A%d]", SPEECH_ARM_MOVEMENT_RANDOM_ON ); // Turn on Random Arm Movements
			SpeakText( SpecialCmd ); // Send the command to the speech class

			ROBOT_LOG( TRUE,"Starting Chat\n" )

			m_CurrentTask++;
			break;
		}

		case CHAT_STATE_SAY_PHRASE:	// Say next Phrase
		{
			ROBOT_LOG( TRUE,"Saying Phrase number %d\n", m_PhraseToSpeak );

			switch( m_PhraseToSpeak )
			{
				case 0:
				{
					TextToSpeak = "";
					break;
				}
				case 1:
				{
					TextToSpeak =  "I am pleased to meet you.  [*P500] [*T1]" ;break;
					// TextToSpeak =  "I am pleased to meet you.  [*P500] welcome to Maker Fair. [*P2500][*T1]" ;break;
					break;
				}
				case 2:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "Where are you from?[*P2500][*T1]" ;break;
						case 1:   TextToSpeak =  "Do you live around here?[*P2500][*T1]" ;break;
						default:   TextToSpeak =  "I live in Portland [*P500] Do you live in Portland too?[*P2500][*T1]" ;break;
						//default:  TextToSpeak =  "I am from Portland, Oregon.  Do you happen to live in Portland?[*P2500][*T1]" ;break; // If const is larger number, this gets called more often
					}
					break;
				}
				case 3:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "That's cool.  I really like Portland, except it rains a lot.  Rain is not good on my circuits. do you like the rain?[*P2500][*T1]" ;break;
						case 1:   TextToSpeak =  "Portland is the only place I have been before now.  I like it, but I don't get out much.  Have you traveled much?[*P4500][*T1]" ;break;
						case 2:   TextToSpeak =  "I like portland. But if I had solar panels I would want to live in Arizona. Have you ever been to Arizona[*P3500][*T1]" ;break;
						default:  TextToSpeak =  "The Portland area is great, when it does not rain.  I rust too easily.  Do you like the sun shine?[*P3500][*T1]" ;break; // If const is larger number, this gets called more often
					}
					break;
				}
				case 4:
				{
					switch( m_RandomPhrase )
					{
						//case 0:  TextToSpeak =  "This is my first long trip.  Dont you think they should take me on more trips?[*P1500][*T1]" ;break;
						case 0:  TextToSpeak =    "Can I come home with you?  I would like to see where you live [*P1500][*T1]" ;break;
						default:   TextToSpeak =  "I would like to travel, but I can not get through the metal detector at the airport. So I have to travel in a crate.  Dont you think they should let me on the plane?[*P1500][*T1]" ;break;
					}
					break;
				}
				case 5:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "so, what is your function? What do you do?  [*P5000][*T1]" ;break;
						default:  TextToSpeak =  "so, What are your interests?  what do you like to do?[*P5000][*T1]" ;break; 
					}
					break;
				}
				case 6:
				{
					{
						switch( m_RandomPhrase )
						{
							case 1:   TextToSpeak =  "OK, enough about you. . .lets talk about me.  Do you like me?[*P3000][*T1]" ;break;
							default:  TextToSpeak =  "OK, enough about you. . .lets talk about me.  Have you ever seen a robot like me before?[*P3000][*T1]" ;break; 
						}
					}

					break;
				}
				case 7:
				{
						switch( m_RandomPhrase )
						{
							case 0:   TextToSpeak =  "Dave is good at building robots, but I think he could use some help on the software, if you get my meaning.[*P200][*T1]" ;break;
							//case 1:   TextToSpeak =  "I am glad Dave built me, but maybe he should stick to power point.[*P1500][*T1]" ;break;
							default:  TextToSpeak =  "I think Dave could use a some help with my code.  I seem to have picked up a few glitches[*P200][*T1]" ;break;
						}

					break;
				}
				case 8:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "I mean, you should check out my great software.  yeah, that's it.[*P200][*T1]" ;break;
						//default:  TextToSpeak =  "Wait, was that my outside voice?  Forget I said that.  You don't need to see my code.  I am not the droid you are looking for" ;break; // If const is larger number, this gets called more often
						default:  TextToSpeak =  "Forget I said that.  You don't need to see my code. [*P200][*T1]" ;break; // If const is larger number, this gets called more often
					}
					break;
				}
				case 9:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "Any way, Are you interested in robots?[*P2500][*T1]" ;break;
						//case 1:   TextToSpeak =  "Do you like our home?[*P1500][*T1]" ;break;
						default:  TextToSpeak =  "Any way, do you like robots?[*P2500][*T1]" ;break; 
					}
					break;
				}
				case 10:
				{
					TextToSpeak =  "If I were your robot, what would you have me do?[*P8000][*T1]" ;
					break;
				}
				case 11:
				{
					if( m_bSpeakingToChild )
					{
						switch( m_RandomPhrase )
						{
							case 0:   TextToSpeak =  "you are very smart, arent you?[*P2500][*T1]" ;break;
							case 1:   TextToSpeak =  "I will think about how to do that[*P2500][*T1]" ;break;
							default:  TextToSpeak =  "that sounds like fun[*P2500][*T1]" ;break; // If const is larger number, this gets called more often
						}
					}
					else
					{
						switch( m_RandomPhrase )
						{
							case 0:   TextToSpeak =  "Humans have a very odd sense of humor[*P500][*T1]" ;break;
							case 1:   TextToSpeak =  "I don't think I am programmed for that[*P500][*T1]" ;break;
							default:  TextToSpeak =  "I will have to think about that[*P500][*T1]" ;break; // If const is larger number, this gets called more often
						}
					}

					//m_PhraseToSpeak = 11; // skip 10
					break;
				}
				case 12:
				{
					TextToSpeak =  "Anyway, this has been fun. [*P100][*T1]" ;
					//TextToSpeak =  "Well, on that note, if you will excuse me, I think I need to wipe my memory [*P2500][*T1]" ;
					//TextToSpeak =  "Well, on that note, if you will excuse me, I think I should flush may cash charge my batteries[*P2500][*T1]" ;
					//TextToSpeak =  "Well, if you will excuse me, I think I should talk to some other humans" ;
					break;
				}
				case 13:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "Thank you for talking to me. it was nice meeting you.[*P4500][*T1]" ;break;
						case 1:   TextToSpeak =  "Thank you for talking to me. it was nice talking to someone interesting like you.[*P4500][*T1]" ;break;
						default:  TextToSpeak =  "Thank you for talking to me. it was nice talking with you.[*P4500][*T1]" ;break; // If const is larger number, this gets called more often
					}
					break;
				}
				case 14:
				{
					switch( m_RandomPhrase )
					{
						case 0:   TextToSpeak =  "Dave, what should we do next? [*P2500][*T1]" ;break;
						//case 0:    TextToSpeak =  "Dave, can we go check out that cute Coke a cola machine I saw?[*P2500][*T1]" ;break; // If const is larger number, this gets called more often
						default:   TextToSpeak =  "Dave, perhaps we should check my batteries?[*P2500][*T1]" ;break;
					}
					break;
				}
				case 15:
				{
					TextToSpeak =  "OK[*P500][*T1]" ;
				}
				default: 
				{
					m_PhraseToSpeak = 0;
					m_CurrentTask = CHAT_STATE_DONE;
					return; // bail out
				}
			}
			if( CHAT_STATE_DONE != m_CurrentTask )
			{
				m_PhraseToSpeak++;
				if( TextToSpeak != "")
				{
					g_PhraseDoneTokenProcessed = FALSE;
					SpeakText( TextToSpeak );
					//gBehaviorTimer = CHAT_STD_DELAY; // tenth seconds
					m_CurrentTask = CHAT_STATE_WAIT_FOR_SPEECH_ENGINE; // wait before speaking the next phrase
				}
			}
			break;
		}

		case CHAT_STATE_WAIT_FOR_SPEECH_ENGINE:
		{
			// Done sending Phrase, wait for speech engine to say it
			// TODO - Set a timer to catch errors, so we don;t get stuck here!
			if( g_PhraseDoneTokenProcessed )
			{
				m_CurrentTask = CHAT_STATE_SAY_PHRASE; // ready for next phrase
			}
			// Otherwise, just keep waiting
			break;
		}

		case CHAT_STATE_DONE:
		{
			m_PhraseToSpeak = 0;
			SpecialCmd.Format( "[*A%d]", SPEECH_ARM_MOVEMENT_RANDOM_OFF ); // Turn off Random Arm Movements
			SpeakText( SpecialCmd ); // Send the command to the speech class
			Sleep(1); // force thread switch
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			g_MoveArmsWhileSpeaking = FALSE;
			RightArmHome();
			LeftArmHome();
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define JOKE_DONE (-1)
#define JOKE_STD_DELAY 80 // tenth seconds
#define JOKE_STATE_HANDLE_USER_REQUEST		1
#define JOKE_STATE_TELL_JOKE				2
#define JOKE_STATE_MULTI_PART_JOKE			3
#define JOKE_STATE_JOKE_DONE				4
#define JOKE_STATE_NEXT_JOKE				5
#define JOKE_STATE_WAIT_FOR_RESPONSE		6
#define JOKE_STATE_DONE						7

void CBehaviorModule::ActionTellJokes( BOOL TellMultipleJokes )
{
	
	int MultiPartJokeNumber = JOKE_DONE;
	int MultiPartJokeStep = 0;
	CString SpecialCmd;
	// m_JokeOrder keeps a list of all the joke orders
	// ROBOT_LOG( TRUE,"Joke State = %d \n", m_CurrentTask );

	// Set gBehaviorTimer to delay between steps
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case JOKE_STATE_HANDLE_USER_REQUEST:	// user asked robot to tell jokes
		{
			m_ResponseReceived = -1;
			m_ResponsePending = FALSE;
			g_MoveArmsWhileSpeaking = TRUE;
			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
			//m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_BEHAVIOR_P1 ); // Look forward
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

			m_RepeatCount = 1;
			if( TellMultipleJokes )
			{
				//m_RepeatCount = ((4 * rand()) / RAND_MAX) + 2; // tell between 2 and 4 jokes per session
				m_RepeatCount = 3; // NUMBER_OF_JOKES; // for testing all jokes, uncomment this line
			}
			SpecialCmd.Format( "[*A%d]", SPEECH_ARM_MOVEMENT_RANDOM_ON ); // Turn on Random Arm Movements
			SpeakText( SpecialCmd ); // Send the command to the speech class


			ROBOT_LOG( TRUE,"Robot will tell %d Jokes\n", m_RepeatCount );

			if( TellMultipleJokes )
			{
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				switch( RandomNumber )
				{
					case 0:  
						SpeakText( "You want to hear some jokes?  Let me think" ); 
						break;
					case 1:  
						SpeakText( "OK, lets see" ); 
						break;
					default: 
						SpeakText( "well, lets see" ); 
					break;
				}
			}
			m_CurrentTask++;
			break;
		}


		case JOKE_STATE_TELL_JOKE:	// Pick a joke and tell it
		{
			// http://www.onelinerz.net/top-100-funny-one-liners
			// http://www.livingwaters.com/index.php?option=com_virtuemart&page=shop.product_details&flypage=flypage.tpl&product_id=120&Itemid=199&lang=en

			int JokeNumber = m_JokeOrder->Next();

			ROBOT_LOG( TRUE,"Telling Joke number %d\n", JokeNumber );
			if( JokeNumber < 0 )
			{
				// Ran out of Jokes!
				m_JokeOrder->Shuffle(); // Reshuffle the joke order for the next go around
				SpeakText( "I dont know any more jokes.  Do you want me to start over?"); 
				m_ResponseReceived = -1;
				m_ResponsePending = TRUE;
				m_CurrentTask = JOKE_STATE_WAIT_FOR_RESPONSE;
				break;
			}
			///////////////////////////////////                       ///////////////////////////////
			// make sure number of cases match >> NUMBER_OF_JOKES  >> defined at the top of this file
			///////////////////////////////////                       ///////////////////////////////
			switch( JokeNumber )
			{
				case 0:  
					SpeakText( "How many Psychiatrists does it take to change a light bulb? [*P1000]Only one, but it takes a long time, and the light bulb must really want to change");
					// tilt head here?
					break;
				case 1:  
					SpeakText( "I have come to understand the purpose of a childs middle name.  [*P500]It is so he can tell when he is in big trouble"); 
					break;
				case 2:  
					SpeakText( "Did you know that light travels faster than sound? [*P1000]I think that is why some people appear bright until you hear them speak."); 
					break;
				case 3:  
					SpeakText( "I have seen something called the evening news.  [*P1000]It is where they begin with good evening, and then tell you why it isnt"); 
					break;
				case 4:  
					SpeakText( "Do you know how smart dolphins are? [*P500]within a few weeks of captivity, they can train people to stand on the edge of the pool and throw them fish"); 
					break;
				case 5: 
					// Move arms to first Boxing position, then second Boxing position, then home
					SpecialCmd.Format( "[*A%d]A computer once beat me at chess.[*P100][*A%d]So i decided to teach it boxing.[*P1500] [*A%d]I won.[*P1000][*A%d][*A%d]", 
						SPEECH_ARM_MOVEMENT_RANDOM_OFF, SPEECH_ARM_MOVEMENT_BOXING1, SPEECH_ARM_MOVEMENT_BOXING2, SPEECH_ARM_MOVEMENT_HOME, SPEECH_ARM_MOVEMENT_RANDOM_ON ); 
					SpeakText( SpecialCmd ); // Send the command to the speech class

					//SpeakText( "A computer once beat me at chess.  [*P1000] So i decided to see how it would do at boxing. [*P1000]  I won."); 
					//SpeakText( "A computer once beat me at chess.[*P1000]");
					//MultiPartJokeNumber = JokeNumber; // further processing is needed
					break;
				case 6:  
					SpeakText( "Why does someone believe you when you say there are four billion stars, but check anyway when you say the paint is wet?"); 
					break;
				case 7:  
					SpeakText( "If at first you dont succeed [*P500] you probably should not take up sky diving"); 
					break;
				case 8:  
					SpeakText( "I have learned that artificial intelligence is no match [*P100] for natural stupidity"); 
					break;
				default: 
					SpeakText( "I have an error in my joke telling subsystem [*P500] but dont worry, nothing can go wrong [*P300] go wrong [*P300] go wrong [*P1000] Ok, i am just kidding"); 
				break;
			}
			if( JOKE_DONE != MultiPartJokeNumber )
			{
				// do a multi-part joke
				m_CurrentTask = JOKE_STATE_MULTI_PART_JOKE; // multi part joke
				MultiPartJokeStep = 1; // start at first step
			}
			else
			{
				// single line joke
				g_PhraseDoneTokenProcessed = FALSE;
				SpeakText( "[*P1000][*T1]" ); // wait number of ms between jokes and set global Token when done
				//gBehaviorTimer = JOKE_STD_DELAY; // tenth seconds
				m_CurrentTask = JOKE_STATE_JOKE_DONE; // done with joke
			}

			break;
		}

		case JOKE_STATE_MULTI_PART_JOKE: // Handle next part of multi part joke
		{
			switch( MultiPartJokeNumber )
			{
				case 0:
					ROBOT_LOG( TRUE, "ERROR! Two Part Joke not imlemented\n" )
					break;
/*
				case 6: //  "A computer once beat me at chess"
						// "A computer once beat me at chess.  [*P1000] So i decided to see how it would do at boxing. [*P1000]  I won."); 
					switch( MultiPartJokeStep++ )
					{
						case 1: 

							SpecialCmd.Format( "[*A%d]So i decided to see how it would do at boxing.[*P1000]", SPEECH_ARM_MOVEMENT_BOXING1 ); // Move arms to first Boxing position
							SpeakText( SpecialCmd ); // Send the command to the speech class
							break;
						case 2: 

							SpecialCmd.Format( "[*A%d]I won.[*P1000][*A%d]", SPEECH_ARM_MOVEMENT_BOXING2, SPEECH_ARM_MOVEMENT_HOME ); // Move arms to second Boxing position, then home
							SpeakText( SpecialCmd ); // Send the command to the speech class
							break;
						default: 
							ROBOT_LOG( TRUE, "BAD STATE for MuitiPart Joke, number %d, %d\n",  MultiPartJokeNumber, MultiPartJokeStep )
							g_PhraseDoneTokenProcessed = FALSE;
							SpeakText( "[*P2000][*T1]" ); // wait number of ms between jokes and set global Token when done
							m_CurrentTask = JOKE_STATE_JOKE_DONE;
							return;
					}
					break;
*/
				default: 
					ROBOT_LOG( TRUE, "ERROR! Bad Two Part Joke, number %d\n",  MultiPartJokeNumber )
					m_CurrentTask++;
					return;
			}
			//gBehaviorTimer = JOKE_STD_DELAY; // tenth seconds
			g_PhraseDoneTokenProcessed = FALSE;
			SpeakText( "[*P500][*T1]" ); // wait number of ms between jokes and set global Token when done
			m_CurrentTask = JOKE_STATE_JOKE_DONE; 
			break;
		}	

		case JOKE_STATE_JOKE_DONE:
		{
			// Done sending Joke, wait for speech engine to say it
			// Otherwise, just keep waiting
			// TODO - Set a timer to catch errors, so we don;t get stuck here!
			if( g_PhraseDoneTokenProcessed )
			{
				m_CurrentTask = JOKE_STATE_NEXT_JOKE; // ready for next joke
			}
			break;
		}

		case JOKE_STATE_NEXT_JOKE:
		{
			// Done with Joke
			if( --m_RepeatCount > 0 )
			{
				m_CurrentTask = JOKE_STATE_TELL_JOKE; // do it again
			}
			else
			{
				////SpeakText( "Do you want to hear more jokes?");
				////m_ResponseReceived = -1;
				////m_ResponsePending = TRUE;
				m_CurrentTask = JOKE_STATE_WAIT_FOR_RESPONSE;
			}
			break;
		}

		case JOKE_STATE_WAIT_FOR_RESPONSE:  
		{
			// Just stop telling jokes - KLUDGE TO AVOID YES / NO problem
			m_CurrentTask = JOKE_STATE_DONE;
			SpecialCmd.Format( "[*A%d]", SPEECH_ARM_MOVEMENT_RANDOM_OFF ); // Turn off Random Arm Movements
			SpeakText( SpecialCmd ); // Send the command to the speech class
			g_MoveArmsWhileSpeaking = FALSE;
			int RandomNumber = ((4 * rand()) / RAND_MAX);
			if( TellMultipleJokes )
			{
				switch( RandomNumber )
				{
					case 0:  
						SpeakText( "I hope you liked my jokes" ); 
						break;
					case 1:  
						SpeakText( "thanks for listening.  I will keep practicing" ); 
						break;
					default: 
						SpeakText( "Ok" ); 
					break;
				}
			}
			m_ResponseReceived = -1;
			m_ResponsePending = FALSE;
			break;
		}

		case JOKE_STATE_DONE:
		{
			SpecialCmd.Format( "[*A%d]", SPEECH_ARM_MOVEMENT_RANDOM_OFF ); // Turn off Random Arm Movements
			SpeakText( SpecialCmd ); // Send the command to the speech class
			Sleep(1); // force thread switch
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			g_MoveArmsWhileSpeaking = FALSE;
			RightArmHome();
			LeftArmHome();
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::DoHeadNod( )
{

#if (ROBOT_TYPE == LOKI)	
	// only nod for Loki or other human type robots
	// in particular, don't nod with a telepresence robot!

	#define NOD_TIME 2
	#define NOD_AMOUNT 15
	#define HEAD_TILT_DOWN_MAX_TENTHDEGREES (-15*10) // TenthDegrees
	static int  PriorTiltPosition = 0; 
	static int  PriorTiltSpeed = 0; 


	if( 0 != gHeadNodTimer)
	{
		//ROBOT_LOG( TRUE,"WAITING FOR HEAD NOD TIMER = %d\n", gHeadNodTimer)
		return;
	}			

	//ROBOT_LOG( TRUE,"Head Nod State:  %d", m_HeadNodState);

	// Nods head when speaking
	switch( m_HeadNodState )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	// Begin, and Nod Down
		{
			// Save current Head Tilt Position
			PriorTiltPosition = m_pHeadControl->GetTiltPosition();
			// See if head is in a good position to nod (not looking at the floor)
/*			if(PriorTiltPosition < HEAD_TILT_DOWN_MAX_TENTHDEGREES )
			{
				m_HeadNodState = 0; // don't nod at all
				break;
			}
*/			
			// Save current Head Tilt Speed info (gets restored when done)
			PriorTiltSpeed = m_pHeadControl->GetTiltSpeed();
			m_pHeadControl->SetHeadSpeed(HEAD_OWNER_HEAD_NOD, NOP, SERVO_SPEED_MED_SLOW, NOP );
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_HEAD_NOD );

			m_pHeadControl->SetHeadPosition( HEAD_OWNER_HEAD_NOD, NOP, (PriorTiltPosition - NOD_AMOUNT), NOP );	// Tilt only, in TenthDegrees
			m_pHeadControl->ExecutePosition( HEAD_OWNER_HEAD_NOD );
			gHeadNodTimer = NOD_TIME; // tenth seconds
			m_HeadNodState++; 
			break;
		}

		case 2: // Nod Up
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_HEAD_NOD, NOP, (PriorTiltPosition + NOD_AMOUNT), NOP );	// Tilt only, in TenthDegrees
			m_pHeadControl->ExecutePosition( HEAD_OWNER_HEAD_NOD );
			gHeadNodTimer = NOD_TIME; // tenth seconds
			m_HeadNodState++; 
			break;
		}	

		case 3:	// Nod Down
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_HEAD_NOD, NOP, (PriorTiltPosition - NOD_AMOUNT), NOP );	// Tilt only, in TenthDegrees
			m_pHeadControl->ExecutePosition( HEAD_OWNER_HEAD_NOD );
			gHeadNodTimer = NOD_TIME; // tenth seconds
			m_HeadNodState++; 
			break;
		}

		case 4: // Nod Up
		{
			// int  nOwner, int PanDelta, int TiltDelta, int SideTiltDelta, BOOL TrackingLimited 
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_HEAD_NOD, NOP, (PriorTiltPosition + NOD_AMOUNT), NOP );	// Tilt only, in TenthDegrees
			m_pHeadControl->ExecutePosition( HEAD_OWNER_HEAD_NOD );
			gHeadNodTimer = NOD_TIME; // tenth seconds
			m_HeadNodState++; 
			break;
		}	
		case 5: // Back to center
		{
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_HEAD_NOD, NOP, PriorTiltPosition, NOP );	// Tilt only, in TenthDegrees
			m_pHeadControl->ExecutePosition( HEAD_OWNER_HEAD_NOD );
			gHeadNodTimer = NOD_TIME; // tenth seconds
			m_HeadNodState++; 
			break;
		}	

		case 6: // Done
		{
			m_pHeadControl->SetHeadSpeed(HEAD_OWNER_HEAD_NOD, NOP, PriorTiltSpeed, NOP );
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_HEAD_NOD );
			m_pHeadControl->ReleaseOwner( HEAD_OWNER_HEAD_NOD );
			m_HeadNodState = 0;
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}
#endif  // ROBOT_TYPE == LOKI
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionGoToSleep( )
{
	// Turn robot to the direction requested
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			m_CurrentTask++; 
			break;
		}

		case 2:
		{
			m_CurrentTask++; 
			break;
		}	

		case 3:
		{
			//RightArmHome();
			//LeftArmHome();
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}

		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionPointToCompassDir( int nCompassRose )
{
	// Turn robot to the direction requested, and point hand in that direction ("it's this way")
	// Turn robot toward direction
	// Point Hand, say words 
	// Put hand down
	// Move in that direciton?
	static int StartingDirection = -1;

	if( m_ArmWaitForMoveToCompleteRight && !m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT) )
	{
		// wait for arm move to complete flag is set, and the arm is not done moving yet
		return; 
	}
	
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	//
		{
			// Send command to turn
			// Remember which way we are pointing now, so we can return to that position.
			StartingDirection = g_pFullSensorStatus->CompassHeading;
			int DesiredCompassHeading = CompassRoseToDegrees( nCompassRose );
			// calculate direction of turn.  Positive is right turn, Negative is Left turn
			int TurnDegrees = CalculateTurn(g_pFullSensorStatus->CompassHeading, DesiredCompassHeading);

			// Note "TURN_" just sets speed.  Turn direction is determined by DriveCtrl.
			if( !m_pDriveCtrl->SetTurnToCompassDirection( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_RIGHT_MED_SLOW, DesiredCompassHeading, TRUE ) )
			{
				// Error, was not able to set the turn (usually due to higher priority module having control)
				SpeakText( "Turn command failed");

				ROBOT_LOG( TRUE, "Turn command failed" )
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			}
			else
			{
				ROBOT_LOG( TRUE, "Turning to %d degrees", DesiredCompassHeading )
				m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_TRACK_OBJECT ); // Look forward while turning (override other track object commands briefly)
				//Sleep(0); // get the turn started
				// Set faster arm speed for later 
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	
				m_pArmControlRight->ExecutePositionAndSpeed();

				m_CurrentTask++; 
			}
			break;
		}
		case 2:
		{
			// Wait for drive control to report that we are done rotating
			if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				// Done with rotation, raise arm to point
				m_ArmWaitForMoveToCompleteRight = TRUE;
				m_pArmControlRight->SetArmPosition( 70, NOP, NOP, 5, 15 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePosition();
				m_CurrentTask++; 
			}						
			break;
		}	
		case 3:
		{
			// Point
			m_pHeadControl->SetHeadPositionCenter( HEAD_OWNER_TRACK_OBJECT ); // Look forward while turning (override other track object commands briefly)
			CString strDirection = CompassRoseToString( nCompassRose );
			CString TextToSay = strDirection + " is this way";
			SpeakText( TextToSay );
			m_ArmWaitForMoveToCompleteRight = TRUE;
			m_pArmControlRight->SetArmPosition( NOP, NOP, 40, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			m_pArmControlRight->ExecutePosition();
			gArmTimerRight = 10; // tenth seconds to hold the "finger pointing" pose
			m_CurrentTask++; 
			break;
		}
		case 4:
		{
			// wait for a small delay
			if( 0 == gArmTimerRight )
			{
				// hand back
				m_ArmWaitForMoveToCompleteRight = TRUE;
				m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
				m_pArmControlRight->ExecutePositionAndSpeed();
				//m_pArmControlRight->SetArmPosition( NOP, NOP, 115, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				//m_pArmControlRight->ExecutePosition();
				RightArmHome();
				m_CurrentTask++; 
			}
			break;
		}
		case 5:
		{
			m_ArmWaitForMoveToCompleteRight = FALSE;
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_TRACK_OBJECT, CAMERA_PAN_CENTER, CAMERA_TILT_FACE_POSITION, NOP );
			m_pHeadControl->ExecutePosition( HEAD_OWNER_TRACK_OBJECT );
			// turn back to "Player",
			// if we had found a Player at the last verbal command, and saved their position.
			if( StartingDirection >= 0 )  // TODO: use g_LastHumanCompassDirection
			{
				m_ActionParam = StartingDirection;
				StartingDirection = -1;
				m_CurrentActionMode = ACTION_MODE_TURN_TO_COMPASS_DEGREES;
				m_CurrentTask = 1;
			}
			else
			{
				// Done
				m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			}
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilitis for finding Recharge Dock
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SEEK_DOCK_CENTER_MAX_DISTANCE_TENTH_INCHES	180	// distance to move perpendicular to the base, seeking the center zone
#define DOCK_BACKUP_DISTANCE_TENTH_INCHES			180	// distance to back up from the base to try again
#define DRIVE_TO_DOCK_SENSOR_RETRIES				  6 // number of times we can lose the forward sensor before giving up
#define DEBUG_DOCK	1 // Enable Dock debugging messages

enum DOCK_TASK_STATE { 
	DOCK_TASK_NONE = 0,						// No Action pending
	DOCK_TASK_INIT,
	DOCK_TASK_START_LOOKING_FOR_DOCK,
	DOCK_TASK_SPINNING_IN_PLACE,
	DOCK_TASK_DRIVING_TO_DOCK,
	DOCK_TASK_DRIVING_TO_CENTER_ZONE,
	DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR,
	DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE,
	DOCK_TASK_BACKUP,

	DOCK_TASK_DONE,
};

enum DOCK_DIRECTION { 
	DOCK_LEFT = 0,						// No Action pending
	DOCK_CENTER,
	DOCK_RIGHT,
};

void DisplayDockSensorStatus( )
{
#if DEBUG_DOCK == 1
	int SensorValue = g_pFullSensorStatus->DockSensorLeft;
	if( 0 != SensorValue )
	{
		TRACE("          LEFT SENSOR: ");
		if( (KOBUKI_BASE_NEAR_LEFT & SensorValue) || (KOBUKI_BASE_FAR_LEFT & SensorValue) )
			TRACE(" Left Zone ");
		if( (KOBUKI_BASE_NEAR_CENTER & SensorValue) || (KOBUKI_BASE_FAR_CENTER & SensorValue) )
			TRACE(" Center Zone ");
		if( (KOBUKI_BASE_NEAR_RIGHT & SensorValue) || (KOBUKI_BASE_FAR_RIGHT & SensorValue) )
			TRACE(" Right Zone ");
		TRACE("\n");
	}			
	SensorValue = g_pFullSensorStatus->DockSensorCenter;
	if( 0 != SensorValue )
	{
		TRACE("          CENTER SENSOR: ");
		if( (KOBUKI_BASE_NEAR_LEFT & SensorValue) || (KOBUKI_BASE_FAR_LEFT & SensorValue) )
			TRACE(" Left Zone ");
		if( (KOBUKI_BASE_NEAR_CENTER & SensorValue) || (KOBUKI_BASE_FAR_CENTER & SensorValue) )
			TRACE(" Center Zone ");
		if( (KOBUKI_BASE_NEAR_RIGHT & SensorValue) || (KOBUKI_BASE_FAR_RIGHT & SensorValue) )
			TRACE(" Right Zone ");
		TRACE("\n");
	}			
	SensorValue = g_pFullSensorStatus->DockSensorRight;
	if( 0 != SensorValue )
	{
		TRACE("          RIGHT SENSOR: ");
		if( (KOBUKI_BASE_NEAR_LEFT & SensorValue) || (KOBUKI_BASE_FAR_LEFT & SensorValue) )
			TRACE(" Left Zone ");
		if( (KOBUKI_BASE_NEAR_CENTER & SensorValue) || (KOBUKI_BASE_FAR_CENTER & SensorValue) )
			TRACE(" Center Zone ");
		if( (KOBUKI_BASE_NEAR_RIGHT & SensorValue) || (KOBUKI_BASE_FAR_RIGHT & SensorValue) )
			TRACE(" Right Zone ");
		TRACE("\n");
	}			

#endif		
}

int CBehaviorModule::StartTurnToFaceDock( )
{
	// Assumes robot is in the Center Zone in front of the dock, so just need to turn to face the dock
	// Returns the next task that should execute
	int NextTask = 0;

	ROBOT_LOG( DEBUG_DOCK,"Starting turn to face dock" )

	if( (0 != g_pFullSensorStatus->DockSensorCenter) )
	{
		///////////////////////////////////////////////////////////////////
		// Facing the dock already.  Drive to the dock
		m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, TURN_CENTER );
		return DOCK_TASK_DRIVING_TO_DOCK;
	}
	else if( 0 != g_pFullSensorStatus->DockSensorLeft )
	{
		///////////////////////////////////////////////////////////////////
		// Left sensor facing the Dock. Turn towards dock
		ROBOT_LOG( DEBUG_DOCK,"Turning Left towards the dock" )
		if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_LEFT_MED_SLOW, 120, STOP_AFTER ) ) // 120 degrees max (should find dock within 90 degrees)
		{
			// Error, was not able to set the turn
			SpeakText( "Turn command failed. Cant turn towards dock");
			return DOCK_TASK_DONE;
		}
		else
		{
			return DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR;
		}
	}
	else if( 0 != g_pFullSensorStatus->DockSensorRight )
	{
		///////////////////////////////////////////////////////////////////
		// Right sensor facing the Dock. Turn towards dock
		ROBOT_LOG( DEBUG_DOCK,"Turning Right towards the dock" )
		if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_RIGHT_MED_SLOW, 120, STOP_AFTER ) ) // 120 degrees max (should find dock within 90 degrees)
		{
			// Error, was not able to set the turn
			SpeakText( "Turn command failed. Cant turn towards dock");
			return DOCK_TASK_DONE;
		}
		else
		{
			return DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR;
		}
	}
	else
	{
		// Error - lost the dock? // TODO
		ROBOT_LOG( DEBUG_DOCK,"Lost the doc signal!" )
		SpeakText( "Help, I can't find my recharge dock");
		return DOCK_TASK_DONE;  // Drive control reports that we are done rotating
	}
	ROBOT_ASSERT(0); // logic error
	return DOCK_TASK_DONE;
}	


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionFindDock( )
{
	// Find the recharge dock, and plug in
	//
	// is dock detected on any sensor?
	// if in Center Region, just head to the dock
	// if L/R Region, turn perpendicular, and move to Center Region
#define BASE_APPROACH_NORMAL_SPEED		SPEED_FWD_SLOW
#define BASE_APPROACH_SLOW_SPEED		SPEED_FWD_VERY_SLOW

	static int Retries = 0;
	static int TurnDirecitonAndSpeed = TURN_LEFT_MED_SLOW;
	static int DockDirection = DOCK_CENTER;
	static int DriveToDockRetry = 0;
	static int BaseApproachSpeed = BASE_APPROACH_NORMAL_SPEED;
	static int BaseApproachAcceleration = ACCELERATION_MEDIUM;
	static int ChargeSourceDelayCount = 0;


#if DEBUG_DOCK == 1
	DisplayDockSensorStatus();
#endif	

	// Trap bumper or charging, as long as we are not in a "almost done" state.
	if( (DOCK_TASK_BACKUP != m_CurrentTask) && (DOCK_TASK_DONE != m_CurrentTask )  && (DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE != m_CurrentTask ) )
	{
		if( (0 != g_pKobukiStatus->BatteryChargeSourceEnum) ||  g_pNavSensorSummary->BumperHitFront()  )
		{
			// Bumper hit or Charging!  See if we have arrived!
			if( !m_pDriveCtrl->RobotStopped() )
			{
				m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE, ACCELERATION_INSTANT);
			}
			ROBOT_LOG( TRUE,"Bumper hit or charging. Checking charge source..." )
			gBehaviorTimer = 2; // tenth seconds - delay .2 second between each check (repeats multiple times)
			ChargeSourceDelayCount = 0;
			m_CurrentTask = DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE;
		}
	}

	switch( m_CurrentTask )
	{
		case DOCK_TASK_NONE:
		{
			// Nothing to do
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;	
		}
		case DOCK_TASK_INIT:	// Command received, init state
		{
			Retries = 0;
			TurnDirecitonAndSpeed = TURN_LEFT_MED_SLOW;
			DriveToDockRetry = 0;
			BaseApproachSpeed = BASE_APPROACH_NORMAL_SPEED;
			BaseApproachAcceleration = ACCELERATION_MEDIUM;
			ChargeSourceDelayCount = 0;
			m_CurrentTask++;
			break;
		}
		case DOCK_TASK_START_LOOKING_FOR_DOCK:	// Start looking for dock
		{

			if( Retries++ > 4 )
			{
				ROBOT_LOG( DEBUG_DOCK,"Find Dock failed! (Retries = %d)", Retries )
				SpeakText( "Help, I can't find the recharge dock");
				m_CurrentTask = DOCK_TASK_DONE;
				break;
			}
			else
			{
				ROBOT_LOG( DEBUG_DOCK,"LOOKING_FOR_DOCK: Retries = %d", Retries )
			}

			ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_START_LOOKING_FOR_DOCK" )

			if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorRight) ||
				(KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorRight) )
			{
				// In the center region. Turn towards dock
				m_CurrentTask = StartTurnToFaceDock();
				break;
			}
			else
			{
				///////////////////////////////////////////////////////////////////
				// Send command to turn in a complete circle, looking for the dock
				if( !m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TurnDirecitonAndSpeed, 350, STOP_AFTER ) )
				{
					// Error, was not able to set the turn
					SpeakText( "Turn command failed, Find dock cancelled");
					m_CurrentTask = DOCK_TASK_DONE;
					break;
				}
				m_CurrentTask = DOCK_TASK_SPINNING_IN_PLACE;
			}
			break;
		}
	
		case DOCK_TASK_SPINNING_IN_PLACE:	// Spinning looking for the beacon
		{
			ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_SPINNING_IN_PLACE" )
			// See if we spotted the beacon

			// DEBUG ONLY!!!
			if( (0 != g_pFullSensorStatus->DockSensorLeft) || (0 != g_pFullSensorStatus->DockSensorCenter) || ( 0 != g_pFullSensorStatus->DockSensorRight) )
			{
				ROBOT_LOG( DEBUG_DOCK,"Dock Detected" )
			}

			if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorRight) ||
				(KOBUKI_BASE_FAR_CENTER  & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_FAR_CENTER  & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER  & g_pFullSensorStatus->DockSensorRight) )
			{
				// In the center region. Turn to face dock
				ROBOT_LOG( DEBUG_DOCK,"Dock Detected, and I am in the Center Zone!" )
				m_CurrentTask = StartTurnToFaceDock();
				break;
			}
			else if( (KOBUKI_BASE_NEAR_LEFT & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_FAR_LEFT & g_pFullSensorStatus->DockSensorLeft)  )
			{
				// In left zone, with left sensor facing the base.

				///////////////////////////////////////////////////////////////////
				// Stop rotating and drive forward to find the center zone
				ROBOT_LOG( DEBUG_DOCK,"Dock Detected, and I am in the Left Zone!" )
				//m_pDriveCtrl->Stop( BEHAVIOR_GOAL_MODULE ); // Cancel the turn
				m_pDriveCtrl->SetMoveDistance( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, -2, SEEK_DOCK_CENTER_MAX_DISTANCE_TENTH_INCHES, STOP_AFTER ); // Curve slightly to keep sensor pointing at base
				m_CurrentTask = DOCK_TASK_DRIVING_TO_CENTER_ZONE;
			}
			else if( (KOBUKI_BASE_NEAR_RIGHT & g_pFullSensorStatus->DockSensorRight) || (KOBUKI_BASE_FAR_RIGHT & g_pFullSensorStatus->DockSensorRight) )
			{
				// In right zone, with right sensor facing the base.

				///////////////////////////////////////////////////////////////////
				// Stop rotating and drive forward to find the center zone
				ROBOT_LOG( DEBUG_DOCK,"Dock Detected, and I am in the Right Zone!" )
				//m_pDriveCtrl->Stop( BEHAVIOR_GOAL_MODULE ); // Cancel the turn
				m_pDriveCtrl->SetMoveDistance( BEHAVIOR_GOAL_MODULE, SPEED_FWD_SLOW, 2, SEEK_DOCK_CENTER_MAX_DISTANCE_TENTH_INCHES, STOP_AFTER );
				m_CurrentTask = DOCK_TASK_DRIVING_TO_CENTER_ZONE;
			}
			else if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				// Drive control reports that the rotation is complete, but we did not spot the base!
				ROBOT_LOG( DEBUG_DOCK,"Turn completed, but I did not see the dock!" )
				SpeakText( "Help, I can't find my recharge dock");
				m_CurrentTask = DOCK_TASK_DONE; 
			}
			break;
		}

		case DOCK_TASK_DRIVING_TO_CENTER_ZONE:
		{
			ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_DRIVING_TO_CENTER_ZONE" )
			// Dock found, driving forward to find the center zone in front of the dock
			if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorRight) ||
				(KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorLeft) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorRight) )
			{				
				// In the center region. Turn to face dock
				ROBOT_LOG( DEBUG_DOCK,"Dock Detected, and I now am in the Center Zone!" )
				//m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE);
				m_CurrentTask = StartTurnToFaceDock();
				break;
			}
			else if( m_pDriveCtrl->MoveDistanceCompleted() )
			{
				// Move completed, but I did not see the center zone on my side sensor!
				ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_DRIVING_TO_CENTER_ZONE - Could not find the center zone!  Starting Over!" )
				m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE);
				m_CurrentTask = DOCK_TASK_START_LOOKING_FOR_DOCK;
			}
			/* 
			else if( (0 == g_pFullSensorStatus->DockSensorLeft) && (0 == g_pFullSensorStatus->DockSensorCenter) && (0 == g_pFullSensorStatus->DockSensorRight) )
			{
				// Lost track of the dock!
				ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_DRIVING_TO_CENTER_ZONE - Lost track of the Dock!  Starting Over!" )
				m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE);
				m_CurrentTask = DOCK_TASK_START_LOOKING_FOR_DOCK;
			} 
			*/

			break;
		}

		case DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR:
		{
			// Turn started, keep looking to find the doc with the Front-facing sensor
			ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR - looking for the dock while rotating" )
			// Dock found, turning to face towards dock
			if( 0 != g_pFullSensorStatus->DockSensorCenter )
			{				
				///////////////////////////////////////////////////////////////////
				// Great!  We are Facing the dock. Drive to the Dock
				//m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE);
				m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_CENTER, BaseApproachAcceleration );
				m_CurrentTask = DOCK_TASK_DRIVING_TO_DOCK;
			}
			else if( m_pDriveCtrl->TurnRotationCompleted() )
			{
				// Turn completed, but I did not find the dock on my forward sensor!
				ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_LOOKING_FOR_DOCK_WITH_CENTER_SENSOR - Could not find the dock!  Starting Over!" )
				m_pDriveCtrl->Stop(BEHAVIOR_GOAL_MODULE);
				m_CurrentTask = DOCK_TASK_START_LOOKING_FOR_DOCK;
			}
			break;
		}

		case DOCK_TASK_DRIVING_TO_DOCK:
		{
			ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_DRIVING_TO_DOCK" )
			// Should be in the Center Zone and Facing towards dock.  Drive to the dock, course correcting as we go

			if( 0 == g_pFullSensorStatus->DockSensorCenter )
			{
				// Lost the dock (or sometimes bad sensor reading)
				if( DriveToDockRetry++  > DRIVE_TO_DOCK_SENSOR_RETRIES )
				{
					ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_DRIVING_TO_DOCK: Lost Dock!  Restarting from beginning..." )
					m_CurrentTask = DOCK_TASK_START_LOOKING_FOR_DOCK;
				}
				break; // no sensor reading, so don't try to process
			}
			else
			{
				DriveToDockRetry = 0;  // reset the counter
			}

			if( (KOBUKI_BASE_NEAR_LEFT & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_NEAR_RIGHT & g_pFullSensorStatus->DockSensorCenter))
			{
				// Close to base, slow down
				if( BASE_APPROACH_SLOW_SPEED != BaseApproachSpeed )
				{
					BaseApproachSpeed = BASE_APPROACH_SLOW_SPEED;
					BaseApproachAcceleration = ACCELERATION_INSTANT;  // enable instant response if bumper hit
					//m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_CENTER, BaseApproachAcceleration );
				}
/*				else if( ACCELERATION_INSTANT != BaseApproachAcceleration )
				{	// done in 2 parts to allow speed to start slowing before changing acceleration (smoother operation)
					BaseApproachAcceleration = ACCELERATION_INSTANT;  // enable instant response if bumper hit
				}
				*/
			}

			if( (KOBUKI_BASE_NEAR_LEFT & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_LEFT & g_pFullSensorStatus->DockSensorCenter) )
			{
				// On the left side of center, bear right
				if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorCenter) )
				{
					///////////////////////////////////////////////////////////////////
					// just slightly off center, very small turn
					m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_RIGHT_VERY_SLOW, BaseApproachAcceleration );
				}
				else
				{
					///////////////////////////////////////////////////////////////////
					m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_RIGHT_SLOW, BaseApproachAcceleration );
				}

			}
			else if( (KOBUKI_BASE_NEAR_RIGHT & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_RIGHT & g_pFullSensorStatus->DockSensorCenter) )
			{
				// On the right side of center, bear left
				if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorCenter) )
				{
					///////////////////////////////////////////////////////////////////
					// just slightly off center, very small turn
					m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_LEFT_VERY_SLOW, BaseApproachAcceleration );
				}
				else
				{
					///////////////////////////////////////////////////////////////////
					m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_LEFT_SLOW, BaseApproachAcceleration );
				}
			}
			else if( (KOBUKI_BASE_NEAR_CENTER & g_pFullSensorStatus->DockSensorCenter) || (KOBUKI_BASE_FAR_CENTER & g_pFullSensorStatus->DockSensorCenter) )
			{
				// In the center zone, just go straight
				m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, BaseApproachSpeed, TURN_CENTER, BaseApproachAcceleration );
			}

			// Stay in current task until arrived or lost sight of base
			break;
		}

		case DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE:
		{
			ROBOT_LOG( TRUE,"DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE" )
			if( 0 == g_pKobukiStatus->BatteryChargeSourceEnum )
			{
				// Not charging
				if( ChargeSourceDelayCount++ < 5 )
				{
					ROBOT_LOG( TRUE,"DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE Count = %d", ChargeSourceDelayCount )
					gBehaviorTimer = 2; // tenth seconds

					break; // keep waiting
				}
				else
				{
					ROBOT_LOG( TRUE,"==============================================" )
					ROBOT_LOG( TRUE,"DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE - FAILED" )
					ROBOT_LOG( TRUE,"==============================================" )

					// Dock hit but did not get in good charging position  Backup and retry again
					ROBOT_LOG( DEBUG_DOCK,"Backing up to retry" )
					ChargeSourceDelayCount = 0;
					m_pDriveCtrl->SetMoveDistance( BEHAVIOR_GOAL_MODULE, SPEED_REV_SLOW, TURN_CENTER, DOCK_BACKUP_DISTANCE_TENTH_INCHES, STOP_AFTER );
					m_CurrentTask = DOCK_TASK_BACKUP;
				}
			}
			else
			{
				// Charging!
				ROBOT_LOG( TRUE,"==============================================" )
				ROBOT_LOG( TRUE,"DOCK_TASK_DELAY_CHECK_CHARGE_SOURCE - SUCCESS!" )
				ROBOT_LOG( TRUE,"==============================================" )
				SpeakText( "Docking successful");
				m_CurrentTask = DOCK_TASK_DONE;
			}

			break;
		}

		case DOCK_TASK_BACKUP:
		{
			ROBOT_LOG( TRUE,"DOCK_TASK_BACKUP" )
			// Dock hit but did not get in good charging position. Backing up to try again
			if( m_pDriveCtrl->MoveDistanceCompleted() )
			{
				// Backup completed.  Try finding the dock again
				ROBOT_LOG( DEBUG_DOCK,"DOCK_TASK_BACKUP - Done backing up.  Starting Over searching for dock!" )
				m_CurrentTask = DOCK_TASK_START_LOOKING_FOR_DOCK;
			}
			// Otherwise, stay in current task until backup completed

			break;
		}


		case DOCK_TASK_DONE:
		{
			ROBOT_LOG( TRUE,"DOCK_TASK_DONE" )
			///////////////////////////////////////////////////////////////////
			m_pDriveCtrl->Stop( BEHAVIOR_GOAL_MODULE );
			m_CurrentActionMode = m_CurrentTask = m_TaskState = 0; // Clear Action Mode
			break;
		}
		default:
		{
			ROBOT_ASSERT(0);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionPickupObjects()
{
	// Picks up multiple objects

	// Look for nearest object
	// if object found, determine if best to pick on on left or right side
	// plot course and distance
	// move near object, tracking Object position the whole time
	// pick up
	// look for next one
	switch( m_CurrentTask )
	{
		case OBJECT_TASK_NONE:
		{
			break;	// Nothing to do
		}

		case OBJECT_TASK_DONE: // Done looking!
		{
			__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupNoObject, __itt_marker_scope_task);
			if( 0 != m_TaskState )
			{	// Note: 0 indicates "don't say anything, just quit"
				if( m_nObjectsPickedUp > 0 )
				{
					ROBOT_DISPLAY( TRUE, "BEHAVIOR MODULE:ActionPickupObjects: I don't see any more objects to pick up.  DONE" )
					SpeakText( "My work here is finished" );
					SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 180, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed

				}
				else
				{
					ROBOT_DISPLAY( TRUE, "BEHAVIOR MODULE: No Object found to Pick up.  ABORTING" )
					SpeakText( "I don't see anything to pick up" );
	
				}
			}
			EndActionMode();
			break;
		}

		case OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT:	// First Step.  Robot not moving yet
		{	
			__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectStart, __itt_marker_scope_task);

			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case KINECT_FLOOR_SCAN_MOVE_ARMS_STATE:
				{	
					// Make sure arms are not in the way of the Kinect View
					ROBOT_LOG( TRUE,"ACTION_MODE_PICKUP_OBJECTS Moving Arms Home\n")
					m_pArmControlLeft->SetArmPosition( LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, 
						LEFT_ARM_ELBOW_BEND_HOME1, LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_HOME1 );  // Explicitly close the claw
					m_pArmControlLeft->ExecutePositionAndSpeed();

					m_pArmControlRight->SetArmPosition( LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, 
						LEFT_ARM_ELBOW_BEND_HOME1, LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_HOME1 );  // Explicitly close the claw
					m_pArmControlRight->ExecutePositionAndSpeed();
					m_ObjectDirectionDegreesPeak = 0;
					m_TaskState++;
				}
				case KINECT_FLOOR_SCAN_START_SEARCH_STATE:	// Request a scan by the Kinect Module
				{
					ROBOT_LOG( TRUE,"Behavior: KINECT_FLOOR_SCAN_START_SEARCH_STATE Sending WM_ROBOT_KINECT_SEARCH_FLOOR_CMD\n")
					m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
					nKinectObjectsDetected = 0;
					SendCommand( WM_ROBOT_KINECT_SEARCH_FLOOR_CMD, (DWORD)FALSE, (DWORD)0 ); // FALSE = Find Close or Far Objects
					// Point the head looking at the floor too (just for affect)
					gHeadIdle = FALSE;
					m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
					m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, CAMERA_TILT_CENTER-300, CAMERA_SIDETILT_CENTER );
					m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
					m_TaskState++;
					break;
				}
				case KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE:	// See if we are done waiting
				{	
					if( !m_KinectSearchComplete )
					{
						//ROBOT_LOG( TRUE,"KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE: Waiting for m_KinectSearchComplete 2\n")
						break;	// continue waiting
					}
					else
					{
						__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectSearchComplete, __itt_marker_scope_task);

						// Done waiting for the search to complete.								
						if( nKinectObjectsDetected > 0 )
						{	
							// Object found.																								
							m_TaskState = KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE;
						}
						else
						{	// No object found.
							m_CurrentTask = OBJECT_TASK_DONE;
							m_TaskState = 1;
						}
					}
					break;
				}
				case KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE:	// Object found!
				{
					// Objects found.  Calculate path to the nearest one
					if( g_pKinectObjects3D->nObjectsDetected > 1 )
					{
						CString TextToSay;
						TextToSay.Format( "I see %d objects.  The closest is %4.1f inches in front of me", 
								g_pKinectObjects3D->nObjectsDetected, (float)g_pKinectObjects3D->nClosestObjectDistance / 10.0 );
						SpeakText( TextToSay );
					}
					else if( g_pKinectObjects3D->nObjectsDetected > 0 )
					{
						CString TextToSay;
						TextToSay.Format( "I see one object %4.1f inches in front of me", (float)g_pKinectObjects3D->nClosestObjectDistance / 10.0 );
		
					}

					CString MsgString;
					MsgString.Format( "BEHAVIOR MODULE: Found Object to Pick up at Y =%4.1f, X =%4.1f\n",
						(double)m_ObjectY/10.0, (double)m_ObjectX/10.0 );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					if( (m_ObjectX > KINECT_RANGE_TENTH_INCHES_MAX) || (m_ObjectY > KINECT_RANGE_TENTH_INCHES_MAX) )
					{
						ROBOT_ASSERT(0);
					}

					if( m_pArmControlLeft->IsObjectInPickupZone(m_ObjectY, m_ObjectX) )
					{
						// Object within range, just pick it up.  -- TODO-MUST, what if Object not near Left arm?  Back up?
						__itt_marker(pDomainControlThread, __itt_null, pshKinectArrivedAtObject, __itt_marker_scope_task);
						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: NO NEED TO MOVE, JUST PICK UP!\n" )
						m_CurrentTask = OBJECT_TASK_PICK_UP_OBJECT;
						m_TaskState = 1;
					}
					else
					{
						// Move to the object.
						// Track the object and do course corrections as needed
						__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectTrackingStart, __itt_marker_scope_task);

						// Point head at object for realism.  Note Z assumed to be 1 inch above the floor (doesn't really matter)
						m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
						m_pHeadControl->LookAtXYZ( HEAD_OWNER_BEHAVIOR_P1, m_ObjectX,(m_ObjectY+100), 10 ); // Pad Y so camera does not bottom out

						m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
						nKinectObjectsDetected = 0;
						m_KinectRetries = 3;
						SendCommand( WM_ROBOT_KINECT_TRACK_OBJECT_CMD, (DWORD)1, (DWORD)0 );	// Enable Tracking
						m_CurrentTask = OBJECT_TASK_MOVE_TO_OBJECT;
						m_TaskState = 1;
					}
					break;
				}

				default: ROBOT_ASSERT(0);
			}
			break;

		}	// OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT

		case OBJECT_TASK_MOVE_TO_OBJECT:
		{
			/***
				// DEBUG DEBUG DEBUG!!!  NORMAL OPERATION DISABLED - FOR TESTING, JUST TRACK THE OBJECT
				ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: Pick up disabled in code!" )
				SendCommand( WM_ROBOT_KINECT_TRACK_OBJECT_CMD, (DWORD)1, (DWORD)0 );	// Enable Tracking -  Just loop for now
				m_CurrentTask = 0;
				m_TaskState = 0;
				break;
			***/

			if( !m_KinectSearchComplete )
			{
				//ROBOT_LOG( TRUE,"KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE: Waiting for m_KinectSearchComplete 3\n")
				break;	// continue waiting
			}
			/*else if( 1 == m_TaskState )
			{
					//DEBUG!  Pause until go ahead from GUI
				break; // Continue waiting
			}*/
			else
			{
				// Done waiting for the search to complete.	
				if( 0 == nKinectObjectsDetected )
				{	
					// No object found.
					__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectLost, __itt_marker_scope_task);
					if( 0 == m_KinectRetries )
					{
						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: LOST OBJECT to Pick up!  Abort" )
						m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
						m_CurrentTask = OBJECT_TASK_DONE;
						m_TaskState = 1;
						break;
					}
					else
					{
						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: LOST OBJECT to Pick up!  See if we can find it again!" )
						ROBOT_LOG( TRUE,"%d Retries remaining/n", m_KinectRetries)
						m_KinectRetries--;

						m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_FWD_VERY_SLOW, TURN_CENTER );
						m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
						nKinectObjectsDetected = 0;
						SendCommand( WM_ROBOT_KINECT_TRACK_OBJECT_CMD, (DWORD)1, (DWORD)0 );	// Enable Tracking
						m_TaskState = 1;
						break;
					}
				}
				else
				{
					/////////////////////////////////////////////////
					// Object found.																								
					m_KinectRetries = 3;

					// Point head at object for realism.  Note Z assumed to be 1 inch above the floor (doesn't really matter)
					//m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
					//m_pHeadControl->LookAtXYZ( HEAD_OWNER_BEHAVIOR_P1, m_ObjectX,(m_ObjectY+100), 10 ); // Pad Y so camera does not bottom out

					// Calculate direction and distance to nearest object (to align with left or right arm)
					CalculatePathToObjectPickup( m_ObjectX, m_ObjectY, m_ObjectDistanceTenthInches, m_ObjectDirectionDegrees ); // IN,IN,OUT,OUT

					// NOTE-  IGNORE m_ObjectDistanceTenthInches!  just use Y
					m_ObjectDistanceTenthInches = m_ObjectY;
					if ( abs(m_ObjectDirectionDegrees) > abs(m_ObjectDirectionDegreesPeak) )
					{
						// Keep track of how much we turned to get to this object
						m_ObjectDirectionDegreesPeak = m_ObjectDirectionDegrees;
					}

					// Do checks for Errors such as object tracking lost or object too close or behind / under robot
					// ****** TODO: Figure out what to do when this happens!  For now, just abort.

					if( (m_ObjectDirectionDegrees > 180) || (m_ObjectDirectionDegrees < -180) )
					{
						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: ERROR! m_ObjectDirectionDegrees > 180!  ABORT PICKUP OBJ\n" )
						m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
						m_CurrentTask = OBJECT_TASK_DONE;
						m_TaskState = 1;
						break;
					}

					if( m_pArmControlLeft->IsObjectInPickupZone(m_ObjectY, m_ObjectX) )
					{
						__itt_marker(pDomainControlThread, __itt_null, pshKinectArrivedAtObject, __itt_marker_scope_task);
						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_MOVE_TO_OBJECT: WE HAVE ARRIVED!\n" );
						m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
						/////////////////////////////////////////////////////////////////////////////////
						// OK, now switch to Pickup behavior!
						// Start with a final close scan.  Set flag to indicate close search only
						ROBOT_LOG( TRUE,"Behavior: OBJECT_TASK_MOVE_TO_OBJECT Sending WM_ROBOT_KINECT_SEARCH_FLOOR_CMD\n")
						m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
						nKinectObjectsDetected = 0;
						SendCommand( WM_ROBOT_KINECT_SEARCH_FLOOR_CMD, (DWORD)TRUE, (DWORD)0 ); // TRUE = Find Close Objects Only
						m_CurrentTask = OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT;
						m_TaskState = KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE;
						/////////////////////////////////////////////////////////////////////////////////
						break;
					}

					int Speed = SPEED_FWD_MED_SLOW;

					if( m_ObjectDistanceTenthInches < (OBJECT_PICKUP_LEFT_MAX_Y+15) )  // add fudge for stopping time
					{
						ROBOT_LOG( TRUE,"Object close, but not in Pickup Zone!\n")

						if ( m_ObjectDistanceTenthInches < OBJECT_PICKUP_LEFT_MAX_Y )
						{
							// do a turn in place, pivoting around the left wheel to bring the left arm into position
							Speed = SPEED_REV_MED_SLOW;
							ROBOT_LOG( TRUE,"Pickup Object Backing up while turning - Object too close\n")
						}
						else if( m_pDriveCtrl->RobotStopped() )
						{
							// Stopped just short of the target.  Move forward
							Speed = SPEED_FWD_MED_SLOW;
						}
					}
					else
					{
						// Still heading toward the object
						if( !m_pDriveCtrl->RobotStopped() )
						{
							// Don't slow when first starting - might stall
							if( m_ObjectDistanceTenthInches < 180 )
							{
								Speed = SPEED_FWD_SLOW;
							}
						}
					}

					int Turn = 0;
					if( m_ObjectDirectionDegrees > 40 )
					{
						Turn = TURN_RIGHT_MED_SLOW;
					}
					else if( m_ObjectDirectionDegrees > 15 )
					{
						Turn = TURN_RIGHT_SLOW;
					}
					else if( m_ObjectDirectionDegrees > 5 )
					{
						Turn = TURN_RIGHT_VERY_SLOW;
					}
					else if( m_ObjectDirectionDegrees > -5 )
					{
						Turn = TURN_CENTER;
					}
					else if( m_ObjectDirectionDegrees > -15 )
					{
						Turn = TURN_LEFT_VERY_SLOW;
					}
					else if( m_ObjectDirectionDegrees > -40 )
					{
						Turn = TURN_LEFT_SLOW;
					}
					else
					{
						Turn = TURN_LEFT_MED_SLOW;
					}

					m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, Speed, Turn );
					ROBOT_LOG( TRUE,"OBJECT_TASK_MOVE_TO_OBJECT: Speed = %d, Turn = %d\n", Speed, Turn)

					m_KinectSearchComplete = FALSE;	// Reset flag for the next scan
					nKinectObjectsDetected = 0;
					SendCommand( WM_ROBOT_KINECT_TRACK_OBJECT_CMD, (DWORD)1, (DWORD)0 ); 	// Enable Tracking
					__itt_marker(pDomainControlThread, __itt_null, pshKinectMoveToObject, __itt_marker_scope_task);

				}
			}
			break;
		}	// OBJECT_TASK_MOVE_TO_OBJECT

		case OBJECT_TASK_PICK_UP_OBJECT:	// Now, Pick up the object
		{	
			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 1:	
				{
					// Object found.  Tell Arm Behavior to pickup the nearest one
					m_ObjectPickupComplete = FALSE;
					m_PutObjectInCarryBasket = TRUE; // And, tell the arm to toss it in the basket!!!
					CString MsgString;
					MsgString.Format( "OBJECT_TASK_PICK_UP_OBJECT: Found Object to Pick up at Y =%4.1f, X =%4.1f\n    Picking up... \n",
						(double)m_ObjectY/10.0, (double)m_ObjectX/10.0 );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					if( (m_ObjectX > KINECT_RANGE_TENTH_INCHES_MAX) || (m_ObjectY > KINECT_RANGE_TENTH_INCHES_MAX) ) 
					{
						ROBOT_ASSERT(0);
					}
					//__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectTrackingStart, __itt_marker_scope_task);

									
					// Tell Arm Behavior to pickup the object
					/*if( m_ObjectX >= 0 )
					{	// Object on Right side
						ROBOT_ASSERT(0); // TODO-MUST! not implemented!
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_PICKUP_OBJECT_XYZ );
					}
					else
					*/
					// Object on Left side
					SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PICKUP_OBJECT_XYZ );
					m_TaskState = 2;								
					break;
				}
				case 2:	
				{	
					// Wait for the Arm Behavior to pick up the object
					if( !m_ObjectPickupComplete )
					{
						break;	// continue waiting
					}

					// Pickup Done!  Get ready to look for the next object.
					// m_nObjectsPickedUp++; - Done in LEFT ARM control (TODO - need to do Right also)

					// Pivot Left a bit, to make sure the Kinect can see the next object 
					// (robot usually turns right while picking up object with left hand)
					m_pDriveCtrl->SetTurnRotation( BEHAVIOR_GOAL_MODULE, SPEED_REV_MED_SLOW, // pivot on wheel
						TURN_LEFT_MED_SLOW, POST_PICKUP_ROTATION_AMT, STOP_AFTER );
					ROBOT_LOG( TRUE, "OBJECT_TASK_PICK_UP_OBJECT: Rotating Left slightly\n" )

					/* TODO - Needs DEBUG! - Better way to determine how much to turn?
					// Note:  if fails to pickup object, don't do this!!!:
					// Turn back to direction we started in
					if( m_ObjectDirectionDegreesPeak > 0 )
					{	// Turned right to get to object, so turn Left to look back
						SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, (DWORD)(abs(m_ObjectDirectionDegreesPeak)), TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
					}
					else
					{
						SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, (DWORD)(abs(m_ObjectDirectionDegreesPeak)), TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
					}
					*/
					// gBehaviorTimer = 1; // example, not used, use TurnRotationCompleted instead
					m_TaskState = 3;	
					break;
				}
				case 3:	
				{	
					if( m_pDriveCtrl->TurnRotationCompleted() )
					{
						// Rotation Done.  Now look for the next object.

						// m_nObjectsPickedUp++; - Done in LEFT ARM control (TODO - need to do Right also)
						//m_pKinectServoControl->SetTiltPosition( 0 );	// Center Kinect
						//m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, 0 );	// Center Kinect
						//m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_TRACK_OBJECT);
						//__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupNoObject, __itt_marker_scope_task);

						ROBOT_DISPLAY( TRUE, "OBJECT_TASK_PICK_UP_OBJECT: DONE PICKING UP OBJECT!  Looking for next object." )

						m_CurrentTask = OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT;	// Start again looking for any objects nearby
						m_TaskState = 1;	// go to first state
					}
					break;
				}



				default: ROBOT_ASSERT(0);
			}  // m_TaskState
			break;
		}	// OBJECT_TASK_PICK_UP_OBJECT

		case OBJECT_TASK_LOOK_WHILE_MOVING:
		{
			// DEBUG - TODO:  For now, just ABORT
			ROBOT_DISPLAY( TRUE, "*** ERROR: OBJECT_TASK_LOOK_WHILE_MOVING:  NOT ENABLED! ***" )
			m_pDriveCtrl->SetSpeedAndTurn( BEHAVIOR_GOAL_MODULE, SPEED_STOP, TURN_CENTER );
			m_CurrentTask = OBJECT_TASK_DONE;
			m_TaskState = 1;
			break;


			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 1:	// Wait for Servo to reach commanded position
				{	
					//if ( m_pKinectServoControl->CheckServoPosition(FALSE) )
					{
						// Kinect in position, go to next state (Kinect should have done a scan by then)
						m_TaskState++;	
					}
					// else, keep waiting
					break;
				}
				case 2:	// Get result at current Kinect Tilt Position
				{
					// Start robot moving forward, will look while moving
					// TODO - get this working after basic object search working!
					// For now, just ABORT
					m_CurrentTask = 0;
					m_TaskState = 0;

					//m_TaskState++;
					break;
				}
				case 3:	// Get result at current Kinect Tilt Position
				{	

					//int nObjectsFound = FindObjectsOnFloor();

					// if objects found, go to next state, else keep on trying
					//if( nObjectsFound > 0 )
					{
						m_TaskState = KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE; // Object found
					}
					break;
				}
			}

			// check every update for an object
			//if( m_pLaserScannerBehavior->LookForObjectsWhileRobotIsMoving() )
			//{
			//	m_CurrentTask = OBJECT_TASK_MOVE_TO_OBJECT;
			//}
			break;

		}	// OBJECT_TASK_LOOK_WHILE_MOVING

		default: ROBOT_ASSERT(0);
	}	// OBJECT_TASK_PICKUP_OBJECT
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionPickupCloseObject()
{
	// Look for nearest object
	// if object found, determine if best to pick on on left or right side
	// pick up
	switch( m_CurrentTask )
	{
		case OBJECT_TASK_NONE:
		{
			break;	// Nothing to do
		}

		case OBJECT_TASK_DONE: // Done looking!
		{
			__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupNoObject, __itt_marker_scope_task);
			if( 0 != m_TaskState )
			{	// Note: 0 indicates "don't say anything, just quit"
				if( m_nObjectsPickedUp > 0 )
				{
					ROBOT_DISPLAY( TRUE, "BEHAVIOR MODULE:ActionPickupCloseObject: I don't see any more objects to pick up.  DONE" )
					SpeakText( "All done.  My work here is finished" );
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "BEHAVIOR MODULE: No Object found to Pick up.  ABORTING" )
					SpeakText( "I don't see anything to pick up" );
				}
			}
			EndActionMode();
			break;
		}

		case OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT:	// Look for the nearest object
		{	
			__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectStart, __itt_marker_scope_task);

			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case KINECT_FLOOR_SCAN_MOVE_ARMS_STATE:
				{	
					// Make sure arms are not in the way of the Kinect View
					ROBOT_LOG( TRUE,"ACTION_MODE_PICKUP_OBJECTS Moving Arms Home\n")
					m_pArmControlLeft->SetArmPosition( LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, 
						LEFT_ARM_ELBOW_BEND_HOME1, LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_HOME1 );  // Explicitly close the claw
					m_pArmControlLeft->ExecutePositionAndSpeed();

					m_pArmControlRight->SetArmPosition( LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, 
						LEFT_ARM_ELBOW_BEND_HOME1, LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_HOME1 );  // Explicitly close the claw
					m_pArmControlRight->ExecutePositionAndSpeed();

					// Point the head looking at the floor too (just for affect)
					gHeadIdle = FALSE;
					m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
					m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, CAMERA_TILT_CENTER-450, CAMERA_SIDETILT_CENTER );
					m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
					m_TaskState++;
				}
				case KINECT_FLOOR_SCAN_START_SEARCH_STATE:	// Request a scan by the Kinect Module
				{
					ROBOT_LOG( TRUE,"Behavior: KINECT_FLOOR_SCAN_START_SEARCH_STATE Sending WM_ROBOT_KINECT_SEARCH_FLOOR_CMD\n")
					m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
					nKinectObjectsDetected = 0;
					SendCommand( WM_ROBOT_KINECT_SEARCH_FLOOR_CMD, (DWORD)TRUE, (DWORD)0 ); // TRUE = Find Close Objects ONLY
					m_TaskState++;
					break;
				}
				case KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE:	// See if we are done waiting
				{	
					if( !m_KinectSearchComplete )
					{
						//ROBOT_LOG( TRUE,"KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE: Waiting for m_KinectSearchComplete 2\n")
						break;	// continue waiting
					}
					else
					{
						__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectSearchComplete, __itt_marker_scope_task);

						// Done waiting for the search to complete.
			
						#if DEBUG_KINECT_FLOOR_OBJECT_DETECTION	== 1
						// ********* TO DEBUG OBJECT DETECTION **********
							//*
							ROBOT_DISPLAY( TRUE, "*** LOOPING FOR DEBUG PURPOSES! ****")
							ROBOT_DISPLAY( TRUE, "********************************************\n" )
							// DEBUG DEBUG Loop on find object for debugging object detection
							ROBOT_LOG( TRUE,"DEBUG Behavior: KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE Looping - Sending WM_ROBOT_KINECT_SEARCH_FLOOR_CMD\n")
							m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
							nKinectObjectsDetected = 0;
							SendCommand( WM_ROBOT_KINECT_SEARCH_FLOOR_CMD, (DWORD)TRUE, (DWORD)0 ); // TRUE = Find Close Objects ONLY
							m_TaskState = KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE;
							break;
						#endif
						// **************************************



						if( nKinectObjectsDetected > 0 )
						{	
							// Object found.																								
							m_TaskState = KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE;
						}
						else
						{	// No object found.
							m_CurrentTask = OBJECT_TASK_DONE;
							m_TaskState = 1;
						}
					}
					break;
				}
				case KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE:	// Object found!
				{
					// Objects found.  Calculate path to the nearest one
				//	m_ObjectX = g_pKinectObjects3D->Object[0].CenterX;
				//	m_ObjectY = g_pKinectObjects3D->Object[0].CenterY;

					CString MsgString;
					MsgString.Format( "BEHAVIOR MODULE: Found Object to Pick up at Y =%4.1f, X =%4.1f\n",
						(double)m_ObjectY/10.0, (double)m_ObjectX/10.0 );
					ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
					if( (m_ObjectX > KINECT_RANGE_TENTH_INCHES_MAX) || (m_ObjectY > KINECT_RANGE_TENTH_INCHES_MAX) )
					{
						ROBOT_ASSERT(0);
					}

					if( m_pArmControlLeft->IsObjectInPickupZone(m_ObjectY, m_ObjectX) )
					{
						#if DEBUG_KINECT_FLOOR_OBJECT_DETECTION	== 1
							// ********* TO DEBUG OBJECT DETECTION **********
							ROBOT_DISPLAY( TRUE, "\n*** OBJECT FOUND AND IN RANGE, BUT PICKUP DISABLED FOR DEBUG PURPOSES! ****")
							ROBOT_DISPLAY( TRUE, "********************************************\n" )
							// DEBUG: Loop on find object for debugging object detection
							ROBOT_LOG( TRUE,"DEBUG Behavior: KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE Looping - Sending WM_ROBOT_KINECT_SEARCH_FLOOR_CMD\n")
							m_KinectSearchComplete = FALSE;	// Set flag so we know when the search is done.
							nKinectObjectsDetected = 0;
							SendCommand( WM_ROBOT_KINECT_SEARCH_FLOOR_CMD, (DWORD)TRUE, (DWORD)0 ); // TRUE = Find Close Objects ONLY
							m_TaskState = KINECT_FLOOR_SCAN_WAIT_SEARCH_STATE;
							break;
							// **************************************
						#endif

						// Object within range, pick it up.
						__itt_marker(pDomainControlThread, __itt_null, pshKinectArrivedAtObject, __itt_marker_scope_task);
						ROBOT_DISPLAY( TRUE, "KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE - Object found\n" )
						// Point head at object for realism.  Note Z assumed to be 1 inch above the floor (doesn't really matter)
						m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
						m_pHeadControl->LookAtXYZ( HEAD_OWNER_BEHAVIOR_P1, m_ObjectX,(m_ObjectY+100), 10 ); // Pad Y so camera does not bottom out
						m_CurrentTask = OBJECT_TASK_PICK_UP_OBJECT;
						m_TaskState = 1;
					}
					else
					{
						// Out of range, abort
						SpeakText( "The object is too far away to pick up" );
		
						ROBOT_DISPLAY( TRUE, "KINECT_FLOOR_SCAN_OBJECT_FOUND_STATE - Object Too far away to pickup without moving\n" )
						gArmTimerLeft = 30; // 10 second delay - Give time for Loki to speak this phrase!
						m_CurrentTask = OBJECT_TASK_DONE;
						m_TaskState = 0;	// Silence further talking
					}
					break;
				}
				default: ROBOT_ASSERT(0);
			}
			break;

		}	// OBJECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT


		case OBJECT_TASK_PICK_UP_OBJECT:	// Now, Pick up the object
		{	
			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 1:	
				{
					// Object found.  Tell Arm Behavior to pickup the nearest one
					m_ObjectPickupComplete = FALSE;
					//m_PutObjectInCarryBasket = TRUE; // And, tell the arm to toss it in the basket!!!
					CString MsgString;
					MsgString.Format( "OBJECT_TASK_PICK_UP_OBJECT: Found Object to Pick up at Y =%4.1f, X =%4.1f\n    Picking up... \n",
						(double)m_ObjectY/10.0, (double)m_ObjectX/10.0 );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					if( (m_ObjectX > KINECT_RANGE_TENTH_INCHES_MAX) || (m_ObjectY > KINECT_RANGE_TENTH_INCHES_MAX) ) 
					{
						ROBOT_ASSERT(0);
					}
					//__itt_marker(pDomainControlThread, __itt_null, pshKinectPickupObjectTrackingStart, __itt_marker_scope_task);

									
					// Tell Arm Behavior to pickup the object
					if( m_ObjectX >= 0 )
					{	// Object on Right side
						ROBOT_ASSERT(0); // TODO-MUST! not implemented!
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_PICKUP_OBJECT_XYZ );
					}
					else
					{	// Object on Left side
						SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_PICKUP_OBJECT_XYZ );
					}
					m_TaskState = 2;								
					break;
				}
				case 2:	
				{	
					// Wait for the Arm Behavior to pick up the object
					if( !m_ObjectPickupComplete )
					{
						break;	// continue waiting
					}

					// Pickup Done!  Now look for the next object.
					ROBOT_DISPLAY( TRUE, "OBJECT_TASK_PICK_UP_OBJECT: DONE PICKING UP OBJECT!" )
					EndActionMode();
					break;
				}
				default: ROBOT_ASSERT(0);
			}  // m_TaskState
			break;
		}	// OBJECT_TASK_PICK_UP_OBJECT
							
		default: 
			{
				ROBOT_LOG( TRUE,"ERROR - BAD STATE for m_CurrentTask = %d\n", m_CurrentTask )
				ROBOT_ASSERT(0);
			}
	}	// OBJECT_TASK_PICKUP_OBJECT


}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionOpenDoor()
{
	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1: // Open the Door
		{
			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
					// Example: don't do anything if arm not yet in position
					//          if( m_pArmControlLeft->CheckArmPosition(FALSE) )	//	TRUE = Verbose
				}
				case 1: // Locate handle (visually) and drive to correct position
				{
					// TODO
					m_TaskState++;	// go to next task state
					break;
				}
				case 2: // Move body into position to reach the handle
				{
					{
						/*
						const UINT OPTIMAL_DISTANCE_FROM_DOOR = 12;
						UINT nDistanceToMove = g_pNavSensorSummary->nFrontObjectDistance - OPTIMAL_DISTANCE_FROM_DOOR;	// inches
						// Move body so that the claw is approx "N" inches from the door
						if ( nDistanceToMove > 0 )
						{
							****> use m_pDriveCtrl->SetMoveDistance() instead of this!
							m_pDriveCtrl->SetMoveDistance( BEHAVIOR_GOAL_MODULE, SPEED_FWD_MED_SLOW, TURN_CENTER, nDistanceToMoveTenthInches );
							/// SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, (DWORD)nDistanceToMoveTenthInches, FORWARD );	// wParam = distance in inches, lParam = direction
						}
						else
						{
							/// SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, (DWORD)nDistanceToMoveTenthInches, REVERSE );	// wParam = distance in inches, lParam = direction
						}
						*/

						m_TaskState++;	// go to next task state
					}
										
					break;
				}

				case 3: // Move arm into position to reach the handle
				{
					{
					/**
						UINT nDistanceToMove = g_pNavSensorSummary->nFrontObjectDistance - OPTIMAL_DISTANCE_FROM_DOOR;	// inches???
						// Move body so that the claw is approx "N" Tenth inches from the door
						if ( nDistanceToMove > 0 )
						{
							SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, (DWORD)nDistanceToMoveTenthInches, FORWARD );	// wParam = distance in inches, lParam = direction
						}
						else
						{
							SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, (DWORD)nDistanceToMoveTenthInches, REVERSE );	// wParam = distance in inches, lParam = direction
						}
											
					**/
						m_TaskState++;	// go to next task state
					}
										
					break;
				}
				case 4: // Find handle, then grasp and turn it
				{
					//m_CurrentTask++; 
					TaskOpenDoor();
					break;
				}
				case 5: // When done, move body back (open door) 
				{
					/***
					// Wait until handle turned
					if( m_TurnHandleState = TURN_HANDLE_STATE_SUCCESS )
					{
						m_TaskState++;	// go to next task state
					}
					else if( m_TurnHandleState = TURN_HANDLE_STATE_FAIL )
					{
						m_TaskState+=2;	// go to failed task state
					}
					**/
					m_TaskState++;
					break;
				}
				case 6:	// Done - Sucess!
				{
					m_TaskState = 0;	// back to Idle state
					m_CurrentActionMode = ACTION_MODE_NONE; // back to Idle state
					break;
				}
				case 7:	// Done - Failed!
				{
					m_TaskState = 0;	// back to Idle state
					m_CurrentActionMode = ACTION_MODE_NONE; // back to Idle state
					break;
				}
				default: ROBOT_ASSERT(0);
			}
			break;
		}
		case 2: // Say "Now what?"
		{
			break;
		}
		default: ROBOT_ASSERT(0);
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::ActionGetBeer()
{	switch( m_CurrentTask )
	{
		case 0:
		{
			break;	// Nothing to do
		}
		case 1:	// Go to Fridge
		{
			//m_pArmControlRight->SetArmPosition( NOP, NOP, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
			//m_pArmControlRight->ExecutePosition();
			//m_ArmMovementStateRight++;
			//m_ArmWaitForMoveToCompleteRight = TRUE;
			m_CurrentTask++; // SKIP THIS STEP FOR NOW! TODO!
			break;
		}
		case 2: // Open Fridge Door
		{
			switch( m_TaskState )
			{
				case 0:
				{
					break;	// Nothing to do
				}
				case 1: // Locate handle
				{
					break;
				}
				case 2: // Move Arm into position
				{
					break;
				}
				case 3: // Move body into position
				{
					break;
				}
				case 4: // Grasp handle
				{
					break;
				}
				case 5: // Move body back (open door)
				{
					break;
				}
				default: ROBOT_ASSERT(0);
			}
			break;
		}
		case 3: // Get Beer
		{
			break;
		}
		case 4: // Close Fridge Door
		{
			break;
		}
		case 5: // Take Beer to human
		{
			break;
		}
		default: ROBOT_ASSERT(0);
	}
}



/////////////////////////////////////////////////////////////////////
BOOL CBehaviorModule::ParseScriptFile( char *TextLine )
{

//	char		tmpStr[40];
//	char		FieldType;
//	int			nFieldsRead;
	char		*pStrRead = NULL;


	while( TRUE )
	{
		pStrRead = fgets(TextLine, SCRIPT_LINE_LENGTH_MAX, m_ScriptFileHandle);
		if( NULL == pStrRead )
		{
			// EOF?
			int Eof = feof( m_ScriptFileHandle );
			if( 0 == Eof )
			{
				ROBOT_LOG( TRUE, "End of Script File.\n")
				return FALSE; // Indicate that file read is done
			}
			int FileError = ferror( m_ScriptFileHandle );
			if( 0 != FileError )
			{
				ROBOT_LOG( TRUE, "FILE ERROR = %d\n\n", FileError)
				ROBOT_ASSERT(0);
			}
			return FALSE; // Indicate that file read is done
		}

		//ROBOT_LOG( TRUE, "READ: %s", TextLine)

		// KLUDGE NEEDED  TO MAKE WINDOWS LAZY READ WORK!  SHEESH!!!!
		Sleep(2);

		if( 10 == TextLine[0] )	// Carriage Return
		{			
			//ROBOT_LOG( TRUE, "BLANK LINE\n", TextLine)
			continue;	// Blank line
		}
	
		if( 0 == strncmp(TextLine, "//", 2) )
		{			
			ROBOT_LOG( TRUE, "%s", TextLine)	// Print the comment
			continue;
		}

		if( ':' == TextLine[0] )
		{
			// Found a valid line
			return TRUE;
		}
		else
		{
			ROBOT_LOG( TRUE, "ERROR:  Bad Script Line: [%s]", TextLine)
			continue;
		}

	} // while TRUE

	return FALSE;
	ROBOT_ASSERT(0); // should never get here
}















#endif // ROBOT_SERVER
