// Speak.h
// Based upon sample code from the Samples in the Microsoft Speech SDK (SAPI 5.1)
//////////////////////////////////////////////////////////////////////
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RobotType.h"
// Turn off all the annoying security warnings! TODO - fix all these some day...
//#define _CRT_SECURE_NO_WARNINGS  
//#define _AFX_SECURE_NO_WARNINGS
//#define _ATL_SECURE_NO_WARNINGS

// SAPI Header Files
#include <sphelper.h>
#include <spddkhlp.h>

#include "ArmControl.h"
//#include "HeadControl.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Defines and Enums

#define MAX_EDIT_TEXT   1000

/////////////////////////////////////////////////////////////////////////////////////////////////////
class CRobotSpeak
{

public:
	CRobotSpeak();
	~CRobotSpeak();

	BOOL		Init();
	void		Speak( WCHAR  *TextToSpeak );
	void		SpeakFromGlobalBuffer();
	void		SpeakIntro();
	void		RobotDoneSpeaking();
	void		HandleSpecialCommands( char TokenType, int Param );
	void		DoRandomArmMovements();
	
protected:
	BOOL                         m_bInSound;
	LCID                         m_lcid;
	BOOL						 m_bEnableSpeaking;
	BOOL						 m_SpeechErrorMsgDisplayed;
	ISpVoice					*m_pVoice;
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
	ArmControl					*m_pArmControlRight;
	ArmControl					*m_pArmControlLeft;
#endif 
	int							 m_ArmPosition;


};

