// WiiControl.cpp: Wii Remote Control class
//
//////////////////////////////////////////////////////////////////////


#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 ) 	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"

#include <mmsystem.h>	// for timeGetTime

#include "..\Common\WiiMoteCommon.h"
#include "headcontrol.h"
#include "armcontrol.h"
#include "WiiControl.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define NUMBER_BUTTONS_CONTROL		1	// 1 = Adult/Child conversation, 0 = manual arm control

__itt_string_handle* pshWiiUpdate = __itt_string_handle_create("pshWiiUpdate");


///////////////////////////////////////////////////////////////////////////////
// WiiControl
/***
BUTTONS
A = HeadControl by tilt / twist

1 = Adult
2 = Child
+ = Face Tracking Enable/Disable
- = Disable Listening Toggle!

Home = Wave and say hello
Up = Not used
Down = reset Intoduciton
Left = Next Phrase
Right = First Phrase



**/



WiiControl::WiiControl()
{

	m_hMapFile = INVALID_HANDLE_VALUE;
	m_pBuf = NULL;	// Shared Buffer space
	m_pHeadControl = NULL;

	m_bWiiMoteSharedMemoryOpened = FALSE;
	m_bWiiHasMotorControl = FALSE;	// Keep track if Wii is controlling or something else
	m_bWiiHasHeadControl = FALSE;
	m_SpeechRecoPausedByWii = FALSE;
	m_bSpeakingToChild = FALSE;
	m_PhraseToSpeak = 0;
	m_RandomPhrase = 0;
	m_ObjectAvoidanceEnabledByWii = FALSE;
	m_OldPanSpeed = 0;
	m_OldTiltSpeed = 0;
	m_OldSideTiltSpeed = 0;
	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
	m_LastButtonState = 0;

	m_ButtonDownWiiMote_A = FALSE; // remember pior button state for debouncing
	m_ButtonDownWiiMote_B = FALSE;
	m_ButtonDownWiiMote_1 = FALSE;
	m_ButtonDownWiiMote_2 = FALSE;
	m_ButtonDownWiiMote_MINUS = FALSE;
	m_ButtonDownWiiMote_PLUS = FALSE;
	m_ButtonDownWiiMote_UP = FALSE;
	m_ButtonDownWiiMote_DOWN = FALSE;
	m_ButtonDownWiiMote_RIGHT = FALSE;
	m_ButtonDownWiiMote_LEFT = FALSE;
	m_ButtonDownWiiMote_HOME = FALSE;


	ROBOT_LOG( TRUE,  "Wii Control constructor complete" )
}

WiiControl::~WiiControl()
{
	if( NULL != m_pBuf )
	{
		UnmapViewOfFile(m_pBuf);
	}
	if( INVALID_HANDLE_VALUE != m_hMapFile)
	{
		CloseHandle(m_hMapFile);
	}
	SAFE_DELETE(m_pArmControlRight);
	SAFE_DELETE(m_pArmControlLeft);
	SAFE_DELETE(m_pHeadControl);
	ROBOT_LOG( TRUE,  "~WiiControl done\n" )
}


//-----------------------------------------------------------------------------
// Name: Initialize
// Desc: Initialize Wii Control
//-----------------------------------------------------------------------------
void WiiControl::Initialize()
{
	// Initialize shared memory for getting input from WiiMote
	TCHAR szWiiMoteSharedFileName[]=TEXT(WIIMOTE_SHARED_FILE_NAME);

	m_hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		szWiiMoteSharedFileName);	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == m_hMapFile) || (NULL == m_hMapFile)  )
	{ 
		ROBOT_LOG( TRUE,  "WiiMote: Could not open file mapping object (%d).\n", GetLastError())
	}
	else
	{
		m_pBuf = (LPCTSTR)MapViewOfFile(m_hMapFile, // handle to map object
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,                    
			0,                    
			(sizeof(WIIMOTE_STATUS_T)) );                   

		if (m_pBuf == NULL) 
		{ 
			ROBOT_LOG( TRUE,  "WiiMote: Could not map view of file (%d).\n", GetLastError());
			CloseHandle(m_hMapFile);
		}
		else
		{
			m_bWiiMoteSharedMemoryOpened = TRUE;
			m_pHeadControl = new HeadControl();	// For controlling head servos
			ROBOT_DISPLAY( TRUE, "WiiMote: Shared Memory Sucess!" )
		}
	}

}


// ------------------------------------------------------------------------------------
// Name: ButtonDownEvent
// Desc: Returns true if button down transition
//-------------------------------------------------------------------------------------
BOOL WiiControl::ButtonDownEvent( UINT ButtonBit, BOOL &PriorButtonDownState ) 
{
	BOOL DownEvent = FALSE;
	if( PriorButtonDownState )
	{
		// button already down
		if( !(m_WiiMoteStatus.Buttons & ButtonBit) )
		{
			PriorButtonDownState = FALSE; // Button now up
		}
	}
	else if( m_WiiMoteStatus.Buttons & ButtonBit )
	{
		DownEvent = TRUE;
		PriorButtonDownState = TRUE;  // now down
	}
	return DownEvent; // tell caller if this is a button-down Event

}

// ------------------------------------------------------------------------------------
// Name: Update
// Desc: Get updated status from the Wii Control, and sends commands as needed
//-------------------------------------------------------------------------------------
void WiiControl::Update()
{

	__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, pshWiiUpdate);

	if( !m_bWiiMoteSharedMemoryOpened || (NULL == m_pHeadControl) )
	{
		__itt_task_end(pDomainGlobalThread);
		return;
	}

	// Get updated data from the WiiMote
//				ROBOT_LOG( TRUE,  "Message from WiiMote\n" )
	int WiiSpeed = 0;
	int WiiTurn = 0;
	
	CopyMemory(&m_WiiMoteStatus, m_pBuf, (sizeof(WIIMOTE_STATUS_T)));
//				ROBOT_LOG( TRUE,  "Pitch = %04d, Roll = %04d, Buttons = %04X, Acc = %3.2f, %3.2f, %3.2f \n", 
//					m_WiiMoteStatus.Pitch, m_WiiMoteStatus.Roll, m_WiiMoteStatus.Buttons,
//					m_WiiMoteStatus.Accel_X, m_WiiMoteStatus.Accel_Y, m_WiiMoteStatus.Accel_Z )

	if( m_WiiMoteStatus.Buttons != 0x40  )
	{
		//ROBOT_LOG( TRUE,  "WiiMote: Button pressed!\n" )
		//ROBOT_LOG( TRUE,  "WiiMote BUTTON DEBUG: %02X\n",  m_WiiMoteStatus.Buttons)
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// B Trigger button - Wheel Motor Control
	if( !(m_WiiMoteStatus.Buttons & WIIMOTE_A) )	// If A pressed, ignore B state (allows turning head while moving)
	{
		if( m_WiiMoteStatus.Buttons & WIIMOTE_B )
		{
			ROBOT_LOG( TRUE,  "WiiMote: Button B pressed!\n" )
			// B = Enable Wheel Motors.  Check for Speed / Turn updates
			// WiiMote allows pitch to 90 degrees, but then roll does not work, so max at pitch = 60 degrees
			// Convert Speed from +/- 60 to +/- 127, but leave guard band at center
			if( (m_WiiMoteStatus.Pitch > -10) && (m_WiiMoteStatus.Pitch < 10) )
			{
				WiiSpeed = 0;
			}
			else
			{					
				WiiSpeed = (int)( ((double)m_WiiMoteStatus.Pitch * -127.0) / 60.0 );	// Invert, down = forward.
				if( WiiSpeed > 127 ) WiiSpeed = 127;
				if( WiiSpeed < -127 ) WiiSpeed = -127;
			}

			// Convert Turn from +/- 90 to +/- 64, but leave guard band at center
			if( (m_WiiMoteStatus.Roll > -10) && (m_WiiMoteStatus.Roll < 10) )
			{
				WiiTurn = 0;
			}
			else
			{					
				WiiTurn = (int)( ((double)m_WiiMoteStatus.Roll * 64.0) / 80.0 ); // slightly less then 90 to ease wrist motion
				if( WiiTurn > 64 ) WiiTurn = 64;
				if( WiiTurn < -64 ) WiiTurn = -64;
			}

			ROBOT_LOG( TRUE,  "WiiSpeed = %d, WiiTurn = %d\n", WiiSpeed, WiiTurn)
			if( !m_bWiiHasMotorControl )
			{
				// Wii does not currently have control, force override
				SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL, 0 );
			}
			g_MotorCurrentSpeedCmd = WiiSpeed;
			g_MotorCurrentTurnCmd = WiiTurn;
			m_bWiiHasMotorControl = TRUE;

		}	// WiiMote "B"
		else
		{
			// "B" not pressed.
			if( m_bWiiHasMotorControl )
			{
				// WiiMote Had control, but now button released. Force a stop
				g_MotorCurrentSpeedCmd = 0;
				g_MotorCurrentTurnCmd = 0;
				SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );
				m_bWiiHasMotorControl = FALSE;
			}
		} // B pressed
	}	// A not pressed too

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// "A" button - Head Control
	if(m_WiiMoteStatus.Buttons & WIIMOTE_A)
	{
		ROBOT_LOG( TRUE,  "WiiMote: Button A pressed!\n" )
		if( !m_bWiiHasHeadControl )
		{
			// Just got control.  Save old speed, so it can be restored when we are done
			m_pHeadControl->GetHeadSpeed( m_OldPanSpeed, m_OldTiltSpeed, m_OldSideTiltSpeed );
		}
		m_bWiiHasHeadControl= TRUE;

		int WiiPitchTiltChangeTenthDegrees = 0;
		int WiiPitchTiltServoSpeed = 0;

		int WiiRollPanChangeTenthDegrees = 0;
		int WiiRollPanServoSpeed = 0;

		// A = Enable Head Control.
		// Tilt/Roll control head position 
		// To avoid "jerk", Wii position above threshold sets a position delta.
		// Pitch/Roll set the speed of servo movement.
		// WiiMote allows pitch to 90 degrees, but then roll does not work, so max at pitch = 60 degrees
		// leave guard band at center

		// Up/Down buttons control shoulder joint
		// Right/Left buttons control wrist rotate
		// +/- buttons control claw open/close

		// WII PITCH / HEAD TILT
		if( m_WiiMoteStatus.Pitch < -10 )
		{
			WiiPitchTiltChangeTenthDegrees = -SERVO_CHANGE_TENTH_DEGREES;
		}
		else if( m_WiiMoteStatus.Pitch > 10 )
		{
			WiiPitchTiltChangeTenthDegrees = SERVO_CHANGE_TENTH_DEGREES;
		}
		else
		{
			WiiPitchTiltChangeTenthDegrees = 0;
			WiiPitchTiltServoSpeed = 0;
		}

		// WII ROLL / HEAD PAN
		if( m_WiiMoteStatus.Roll < -10 )
		{
			WiiRollPanChangeTenthDegrees = -SERVO_CHANGE_TENTH_DEGREES;
		}
		else if( m_WiiMoteStatus.Roll > 10 )
		{
			WiiRollPanChangeTenthDegrees = SERVO_CHANGE_TENTH_DEGREES;
		}
		else
		{
			WiiRollPanChangeTenthDegrees = 0;
			WiiRollPanServoSpeed = 0;
		}

		if( (0 != WiiRollPanChangeTenthDegrees) || (0 != WiiPitchTiltChangeTenthDegrees) )
		{
			// Some head movement requested

			if( 0 != WiiPitchTiltChangeTenthDegrees )
			{	
				// Calculate Speed for Tilt servo
				WiiPitchTiltServoSpeed = SERVO_SPEED_SLOW; // abs( (int)( ((double)m_WiiMoteStatus.Pitch * (double)(SERVO_SPEED_MAX)) / 60.0 ) );
				//if( WiiPitchTiltServoSpeed > SERVO_SPEED_MAX ) WiiPitchTiltServoSpeed = SERVO_SPEED_MAX;
			}
			if( 0 != WiiRollPanChangeTenthDegrees )
			{	
				// Calculate Speed for Tilt servo
				WiiRollPanServoSpeed = SERVO_SPEED_SLOW; // abs( (int)( ((double)m_WiiMoteStatus.Roll * (double)(SERVO_SPEED_MAX)) / 60.0 ) );
				//if( WiiRollPanServoSpeed > SERVO_SPEED_MAX ) WiiRollPanServoSpeed = SERVO_SPEED_MAX;
			}

			// Set Speed
			m_pHeadControl->SetHeadSpeed(HEAD_OWNER_USER_CONTROL, WiiRollPanServoSpeed, WiiPitchTiltServoSpeed, NOP );
			// Set position
			m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL,
				WiiRollPanChangeTenthDegrees, WiiPitchTiltChangeTenthDegrees, NOP, FALSE ); // TRUE = Limit to front of robot
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );
		}

	}	// Button "A" pressed (Head Pan/Tilt)
	else
	{
		// "A" button Up
		if( m_bWiiHasHeadControl )
		{
			// WiiMote Had control, but now button released. Force all servos stop
			m_bWiiHasHeadControl = FALSE;

			// Set Speed
			//m_pHeadControl->SetHeadSpeed( HEAD_OWNER_USER_CONTROL, m_OldPanSpeed, m_OldTiltSpeed, m_OldSideTiltSpeed );
			// Set position
			m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL, 0, 0, NOP, FALSE ); // TRUE = Limit to front of robot
			m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL ); // ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );

			/*******
			// Restore Speed and stop position change for Tilt/Pan servos
			g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Speed = SERVO_SPEED_MED; // TODO - Set to some default?
			g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees = 
				g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees;
			g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Update = TRUE;

			g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Speed = SERVO_SPEED_MED; // TODO - Set to some default?
			g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees = 
				g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees;
			g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Update = TRUE;
			*****/
		}
	} // Button "A"


#if (NUMBER_BUTTONS_CONTROL	== 1)
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// "1" button - Adult
	// "2" button - Child


	if( ButtonDownEvent(WIIMOTE_1, m_ButtonDownWiiMote_1) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Button 1 pressed\n" )
		// Talking to an Adult (MODIFY THIS CODE FOR GROUP DEMOS!)
		m_bSpeakingToChild = FALSE;
		SpeakText( "Are you an Adult" );
	}

	if( ButtonDownEvent(WIIMOTE_2, m_ButtonDownWiiMote_2) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Button 2 pressed\n" )
		// Talking to a Child (MODIFY THIS CODE FOR GROUP DEMOS!)
		m_bSpeakingToChild = TRUE;
		SpeakText( "Are you a Child" );
	}


#else

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// "1" button - Right Arm Control
	// "2" button - Left Arm Control
	// Pitch = Shoulder
	// tilt = Elbow

#define SHOULDER_MULTIPLYER 0.8
#define ELBOW_MULTIPLYER	0.5

	if( (m_WiiMoteStatus.Buttons & WIIMOTE_1) || (m_WiiMoteStatus.Buttons & WIIMOTE_2) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Button 1 or 2 pressed!  Right/Left Arm\n" )
		double WiiArmPitchChangeDegrees = 0.0;
		double WiiArmRollChangeDegrees = 0.0;

		// To avoid "jerk", Wii position above threshold sets a position delta.
		// Pitch/Roll set the speed of servo movement.
		// WiiMote allows pitch to 90 degrees, but then roll does not work, so max at pitch = 60 degrees
		// leave guard band at center

		// WII PITCH = Shoulder
		if( (m_WiiMoteStatus.Pitch < -2) || (m_WiiMoteStatus.Pitch > 2) )
		{
			WiiArmPitchChangeDegrees = (double)m_WiiMoteStatus.Pitch * SHOULDER_MULTIPLYER;
		}

		// WII ROLL = Elbow
		if( (m_WiiMoteStatus.Roll < -2) || (m_WiiMoteStatus.Roll > 2) )
		{
			WiiArmRollChangeDegrees = (double)m_WiiMoteStatus.Roll * ELBOW_MULTIPLYER;
		}

		if( (0 != WiiArmRollChangeDegrees) || (0 != WiiArmPitchChangeDegrees) )
		{
			// Some arm movement requested
			ROBOT_LOG( TRUE,  "**** WII ARM CONTROL: Shoulder = %3.2f, Elbow = %3.2f\n", WiiArmPitchChangeDegrees, WiiArmRollChangeDegrees)

			if( m_WiiMoteStatus.Buttons & WIIMOTE_1 ) 
			{
				// RIGHT ARM ////////////////////////////////////////////////
				// Get current position of arm
				int Shoulder = 0; int ElbowRotate = 0; int ElbowBend = 0; int Wrist = 0; int Claw = 0;
				int NewShoulderDegrees = 0; int NewElbowDegrees = 0;

				m_pArmControlRight->GetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
				ROBOT_LOG( TRUE,  "Current Right Position = %d, %d\n", Shoulder, ElbowBend)

				if( (0 != WiiArmPitchChangeDegrees) && (0 != WiiArmRollChangeDegrees) )
				{	
					// Both changed
					NewShoulderDegrees = Shoulder + (int)WiiArmPitchChangeDegrees;
					NewElbowDegrees = ElbowBend + (int)WiiArmRollChangeDegrees;

					m_pArmControlRight->SetArmPosition( NewShoulderDegrees, NOP, NewElbowDegrees, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				else if( 0 != WiiArmPitchChangeDegrees )
				{	
					// Shoulder changed
					NewShoulderDegrees = Shoulder + (int)WiiArmPitchChangeDegrees;
					m_pArmControlRight->SetArmPosition( NewShoulderDegrees, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				else if( 0 != WiiArmRollChangeDegrees )
				{	
					// Elbow changed
					NewElbowDegrees = ElbowBend + (int)WiiArmRollChangeDegrees;
					m_pArmControlRight->SetArmPosition( NOP, NOP, NewElbowDegrees, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				// Do it!
				ROBOT_LOG( TRUE,  "**** WII ARM CONTROL: Sending Shoulder = %d, Elbow = %d\n", NewShoulderDegrees, NewElbowDegrees)
				m_pArmControlRight->ExecutePosition();
			}

			if( m_WiiMoteStatus.Buttons & WIIMOTE_2 )
			{
				// LEFT ARM ////////////////////////////////////////////////
				// Get current position of arm
				int Shoulder = 0; int ElbowRotate = 0; int ElbowBend = 0; int Wrist = 0; int Claw = 0;
				int NewShoulderDegrees = 0; int NewElbowDegrees = 0;

				m_pArmControlLeft->GetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
				ROBOT_LOG( TRUE,  "Current Left Position = %d, %d\n", Shoulder, ElbowBend)

				if( (0 != WiiArmPitchChangeDegrees) && (0 != WiiArmRollChangeDegrees) )
				{	
					// Both changed
					NewShoulderDegrees = Shoulder + (int)WiiArmPitchChangeDegrees;
					NewElbowDegrees = ElbowBend + (int)WiiArmRollChangeDegrees;
					m_pArmControlLeft->SetArmPosition( NewShoulderDegrees, NOP, NewElbowDegrees, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				else if( 0 != WiiArmPitchChangeDegrees )
				{	
					// Shoulder changed
					NewShoulderDegrees = Shoulder + (int)WiiArmPitchChangeDegrees;
					m_pArmControlLeft->SetArmPosition( NewShoulderDegrees, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				else if( 0 != WiiArmRollChangeDegrees )
				{	
					// Elbow changed
					NewElbowDegrees = ElbowBend + (int)WiiArmRollChangeDegrees;
					m_pArmControlLeft->SetArmPosition( NOP, NOP, NewElbowDegrees, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				}
				// Do it!
				ROBOT_LOG( TRUE,  "**** WII ARM CONTROL: Sending Shoulder = %d, Elbow = %d\n", NewShoulderDegrees, NewElbowDegrees)
				m_pArmControlLeft->ExecutePosition();
			}
		}

	}	// Button "1" or "2" pressed (Arm Pan/Tilt)
#endif

	////////////////////////////////////////////////////////////////////////////////////////////
	// OTHER BUTTONS
	////////////////////////////////////////////////////////////////////////////////////////////

	if( ButtonDownEvent(WIIMOTE_PLUS, m_ButtonDownWiiMote_PLUS) ) // "+"
	{
		ROBOT_LOG( TRUE,  "WiiMote: Button PLUS pressed\n" )
		// Toggle object avoidance each time button is pressed
		if( m_ObjectAvoidanceEnabledByWii )
		{
			// was on, so turn off
			SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 0, 0 );	// Disable
			SpeakText( "warning, object avoidance disabled" );
			m_ObjectAvoidanceEnabledByWii = FALSE;
		}
		else
		{
			// was off, so turn on
			SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable
			SpeakText( "object avoidance enabled" );
			m_ObjectAvoidanceEnabledByWii = TRUE;
		}
	}

	/**
	if( ButtonDownEvent(WIIMOTE_MINUS, m_ButtonDownWiiMote_MINUS) ) // "-"
	{
		// MINUS = Disable Listening on Voice Recognition
		if( !m_SpeechRecoPausedByWii ) // Just send once on down
		{
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, FALSE, 0 );	// Turn Off
			m_SpeechRecoPausedByWii = TRUE;
		}
	}
	else 
	{
		if( m_SpeechRecoPausedByWii )
		{
			// Was paused, so unpause it
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, TRUE, 0 );	// Turn On
			m_SpeechRecoPausedByWii = FALSE;
		}
	}
	**/


	if( ButtonDownEvent(WIIMOTE_HOME, m_ButtonDownWiiMote_HOME) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Home pressed\n" )
		// Home = "Wave and say Hello"
		SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute, 
	}

	if( ButtonDownEvent(WIIMOTE_UP, m_ButtonDownWiiMote_UP) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Up pressed\n" )
		// not used
	}

	if( ButtonDownEvent(WIIMOTE_DOWN, m_ButtonDownWiiMote_DOWN) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Down pressed\n" )
		// resets for next introduction
		SpeakText( "I like meeting people" );

		m_PhraseToSpeak=0;
		g_CurrentUserName.Empty();

	}

	if( ButtonDownEvent(WIIMOTE_LEFT, m_ButtonDownWiiMote_LEFT) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Left Pressed.  State = %d\n", m_PhraseToSpeak)
		// Say next phrase

		/*	g_CurrentUserName not supported with C#/Kinect version of speech reco api	
		if( !(g_CurrentUserName.IsEmpty()) && (m_PhraseToSpeak < 3) )
		{
			// Loki has already been introduced to the person, so jump to step 3
			m_PhraseToSpeak = 3;
		}
		*/

		switch(m_PhraseToSpeak)
		{

/*	// Example of direct voice control, watching star wars
			case 0:
			{
				SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)600, SERVO_SPEED_MED );
				Sleep(1000);
				SpeakText( "I am watching Star Wars" );

				m_PhraseToSpeak++;
				break;
			}
*/
			case 0:
			{
				// Home = "Wave and say Hello"
				// Get a number to randomize responses at the start of each conversation
				// This will be used for all responses until the next conversation
				m_RandomPhrase = ((4 * rand()) / RAND_MAX);
				ROBOT_LOG( TRUE,  "DEBUG: RAND = %d\n", m_RandomPhrase)
				SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_WAVE );	// Right/Left arm, Movement to execute, 
				m_PhraseToSpeak++;
				break;
			}
			case 1:
			{
				SpeakText( "What is your name?" );

				m_PhraseToSpeak++;
				break;
			}
			case 2:
			{
				switch( m_RandomPhrase )
				{
					case 0:   SpeakText( "That is a cool name.  Where are you from?" );break;
					case 1:   SpeakText( "That is a good name for a human. Do you live around here?" );break;
					case 2:   SpeakText( "I am pleased to meet you.  I live in Portland, do you?" );break;
					default:  SpeakText( "I am pleased to meet you.  Do you live in Portland" );break; // If const is larger number, this gets called more often
				}

				m_PhraseToSpeak++;
				break;
			}
			case 3:
			{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "That's cool.  I really like Portland.  Except for the rain.  Rain is not good on my circuits. do you like the rain?" );break;
						case 1:   SpeakText( "Portland is the only place I have been.  I like it, but I don't get out much.  Have you gone anywhere but portland?" );break;
						case 2:   SpeakText( "I like portland. But if I had solar panels I would want to live in Arizona. Have you ever been to Arizona" );break;
						default:  SpeakText( "The Portland area is great, when it does not rain.  I rust too easily.  Do you like the sun shine?" );break; // If const is larger number, this gets called more often
					}

				m_PhraseToSpeak++;
				break;
			}
			case 4:
			{
				if( m_bSpeakingToChild )
				{
					switch( m_RandomPhrase )
					{
						//case 0:   SpeakText( "why are you in this class?" );break;
						//case 1:   SpeakText( "do you like robotics" );break;
						case 1:   SpeakText( "Do you like our home?" );break;
						default:  SpeakText( "Are you interested in robotics?" );break; // If const is larger number, this gets called more often
					}
				}
				else
				{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "that is nice. . . So, do you work?  what is your function?" );break;
						default:  SpeakText( "that is nice. . . What are your interests?  what do you like to do?" );break; // If const is larger number, this gets called more often
					}
				}

				m_PhraseToSpeak++;
				break;
			}
			case 5:
			{
				{
					switch( m_RandomPhrase )
					{
						case 1:   SpeakText( "OK, enough about you. . .lets talk about me.  Do you like me?" );break;
						default:  SpeakText( "Have you ever seen a robot like me before?" );break; // If const is larger number, this gets called more often
					}
				}

				m_PhraseToSpeak++;
				break;
			}
			case 6:
			{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "Dave is good at building robots, but I think he could use some help on the software, if you get my meaning." );break;
						//case 1:   SpeakText( "I am glad Dave built me, but maybe he should stick to power point." );break;
						default:  SpeakText( "I think I could use a some help with my code.  I seem to have picked up a few glitches" );break;
					}

				m_PhraseToSpeak++;
				break;
			}
			case 7:
			{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "I mean, you should check out my great software.  yeah, that's it." );break;
						//default:  SpeakText( "Wait, was that my outside voice?  Forget I said that.  You don't need to see my code.  I am not the droid you are looking for" );break; // If const is larger number, this gets called more often
						default:  SpeakText( "Forget I said that.  You don't need to see my code.  I am not the droid you are looking for" );break; // If const is larger number, this gets called more often
					}

				m_PhraseToSpeak++;
				break;
			}
			case 8:
			{
				SpeakText( "If I were your robot, what would you have me do?" );

				m_PhraseToSpeak++;
				break;
			}
			case 9:
			{
				if( m_bSpeakingToChild )
				{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "you are very smart, arent you? " );break;
						case 1:   SpeakText( "I will think about how to do that" );break;
						default:  SpeakText( "that sounds like fun" );break; // If const is larger number, this gets called more often
					}
				}
				else
				{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "Humans have a very odd sense of humor" );break;
						case 1:   SpeakText( "I am not programmed for that" );break;
						default:  SpeakText( "I will have to think about that" );break; // If const is larger number, this gets called more often
					}
				}

				m_PhraseToSpeak = 11; // skip 10
				break;
			}
			case 10:
			{
				SpeakText( "Well, if you will excuse me, I think I should charge my batteries" );
				SpeakText( "Well, if you will excuse me, I think I should talk to some other humans" );

				m_PhraseToSpeak++;
				break;
			}
			case 11:
			{
				{
					switch( m_RandomPhrase )
					{
						case 0:   SpeakText( "Well, Thank you for talking to me. it was nice meeting you." );break;
						case 1:   SpeakText( "Well, Thank you for talking to me. it was nice talking to someone interesting like you." );break;
						default:  SpeakText( "Well, Thank you for talking to me. it was nice talking with you." );break; // If const is larger number, this gets called more often
					}
				}

				m_PhraseToSpeak++;
				break;
			}
			case 12:
			{
				SpeakText( "Goodbye" );

				m_PhraseToSpeak = 0;
				g_CurrentUserName.Empty();
				break;
			}
			default:
			{
				m_PhraseToSpeak = 0;
				break;
			}
		}
	}

	if( ButtonDownEvent(WIIMOTE_RIGHT, m_ButtonDownWiiMote_RIGHT) )
	{
		ROBOT_LOG( TRUE,  "WiiMote: Right pressed\n" )
		// Go back to first phrase
		m_PhraseToSpeak = 3;
	}

	m_LastButtonState = m_WiiMoteStatus.Buttons;
	__itt_task_end(pDomainGlobalThread);

}

#endif // ROBOT_SERVER - This module used for Robot Server only
