// WiiControl.h: Wii Remote Control class
//////////////////////////////////////////////////////////////////////
#pragma once	// Only include this header once

#include "RobotType.h"

#define SERVO_CHANGE_TENTH_DEGREES	150		// Tenth degrees per update

/////////////////////////////////////////////////////////////////////////////
class WiiControl
{

public:


	WiiControl::WiiControl();
	WiiControl::~WiiControl();

	//-----------------------------------------------------------------------------
	// Name: Initialize
	// Desc: Initialize Wii Control
	//-----------------------------------------------------------------------------
	void Initialize();

	// ------------------------------------------------------------------------------------
	// Name: Update
	// Desc: Get updated status from the Wii Control, and sends commands as needed
	//-------------------------------------------------------------------------------------
	void Update();

	// ------------------------------------------------------------------------------------
	// Name: ButtonDownEvent
	// Desc: Returns true if button down transition
	//-------------------------------------------------------------------------------------
	BOOL WiiControl::ButtonDownEvent( UINT ButtonBit, BOOL &PriorButtonDownState );

	// ------------------------------------------------------------------------------------
	// Name: SendDriveCommand
	// Desc: Sends drive commands from Wii to robot motor control
	//-------------------------------------------------------------------------------------
	void SendDriveCommand( int Speed, int Turn );

private:

	HANDLE			m_hMapFile;
	LPCTSTR			m_pBuf;	// Shared Buffer space
	WIIMOTE_STATUS_T m_WiiMoteStatus;

	BOOL			m_bWiiMoteSharedMemoryOpened;
	BOOL			m_bWiiHasMotorControl;		// Keep track if Wii is controlling or something else
	BOOL			m_bWiiHasHeadControl;
	BOOL			m_SpeechRecoPausedByWii;
	BOOL			m_bSpeakingToChild;
	int 			m_PhraseToSpeak;
	int				m_RandomPhrase;
	BOOL			m_ObjectAvoidanceEnabledByWii;
	int 			m_OldPanSpeed;
	int 			m_OldTiltSpeed;
	int 			m_OldSideTiltSpeed;
	int 			m_ManualArmControlRight;
	int 			m_ManualArmControlLeft;
	UINT			m_LastButtonState;   // Track button up to prevent duplicate commands
	
	BOOL			m_ButtonDownWiiMote_A; // remember pior button state for debouncing
	BOOL			m_ButtonDownWiiMote_B;
	BOOL			m_ButtonDownWiiMote_1;
	BOOL			m_ButtonDownWiiMote_2;
	BOOL			m_ButtonDownWiiMote_MINUS;
	BOOL			m_ButtonDownWiiMote_PLUS;
	BOOL			m_ButtonDownWiiMote_UP;
	BOOL			m_ButtonDownWiiMote_DOWN;
	BOOL			m_ButtonDownWiiMote_RIGHT;
	BOOL			m_ButtonDownWiiMote_LEFT;
	BOOL			m_ButtonDownWiiMote_HOME;

	int				m_CurrentSpeed;
	int				m_CurrentTurn;

	ArmControl	   *m_pArmControlRight;
	ArmControl	   *m_pArmControlLeft;
	HeadControl	   *m_pHeadControl;	// For controlling head servos

};

