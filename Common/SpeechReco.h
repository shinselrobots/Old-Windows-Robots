// SpeechReco.h
// Handle speech phrases detected by the Kinect C# App
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

/*
#define MAX_EDIT_TEXT   1000

#define PID_DictMode					  1
#define PID_DictCommand					901
#define PID_Loki						902
#define PID_Stop						903
#define PID_AI							904


#define PID_CmdMenu						 1
#define VID_UserName					 2
#define PID_SayHello					 3
#define PID_Meet6						 4
#define PID_Meet5						 5
#define PID_Meet2						 6

#define PID_CmdMicrophoneOn				10		 
#define PID_CmdMicrophoneOff			11		 
#define PID_CmdAIMode					12
#define PID_CmdDictTestMode				13
#define PID_CmdEnableMovement			14
#define PID_CmdDisableMovement			15
#define PID_CmdMoveStop					16
#define PID_CmdMoveForward				17		 
#define PID_CmdMoveBack					18
#define PID_CmdTurnLeft					19
#define PID_CmdTurnRight				20
#define PID_CmdLookUp					21		 
#define PID_CmdLookDown					22
#define PID_CmdLookLeft					23
#define PID_CmdLookRight				24
#define PID_CmdLookForward				25

#define PID_CmdAvoidObjectsOn			26
#define PID_CmdAvoidObjectsOff			27
#define PID_CmdTalkAboutObstaclesOn		28
#define PID_CmdTalkAboutObstaclesOff	29
#define PID_CmdTalkingEnabled			30
#define PID_CmdTalkingDisabled			31
#define PID_CmdCameraTrackFaceOn		32		 
#define PID_CmdCameraTrackFaceOff		33

// Generic Arm commands (robot should figure out which arm you mean)
#define PID_CmdArmPutDown				35
#define PID_CmdArmDropIt				36
#define PID_CmdArmTossFront				37
#define PID_CmdArmTossBack				38
#define PID_CmdArmHandsUp				39
#define PID_CmdArmHandsDown				40
#define PID_CmdArmWave					41
#define PID_CmdArmScratchBack			42
#define PID_CmdArmScratchHead			43

// Left / Right Arm commands
#define PID_CmdArmHomeLeft				44
#define PID_CmdArmExtendLeft			45
#define PID_CmdArmExtendFullLeft		46
#define PID_CmdArmTakeObjectLeft		47
#define PID_CmdArmPickUpLeft			48
#define PID_CmdArmCloseClawLeft			49
#define PID_CmdArmOpenClawLeft			50

#define PID_CmdFindObjectsAndPickup		51

#define PID_CmdArmHomeRight				52
#define PID_CmdArmExtendRight			53
#define PID_CmdArmExtendFullRight		54
#define PID_CmdArmTakeObjectRight		55
#define PID_CmdArmPickUpRight			56
#define PID_CmdArmCloseClawRight		57
#define PID_CmdArmOpenClawRight			58

#define PID_CmdArmExtendClosedRight		59

#define PID_CmdFollowMeStart			60
#define PID_CmdExploreStart				61
#define PID_CmdExploreStop				62
#define PID_CmdGotoOffice				63
#define PID_CmdGotoBedroom				64
#define PID_CmdGotoHeatherRoom			65
#define PID_CmdGotoAmberRoom			66
#define PID_CmdGotoMyBathroom			67

#define PID_CmdBlueLightsOn				68
#define PID_CmdBlueLightsOff			69
#define PID_CmdKarate					70
#define PID_CmdWakeUp					71
#define PID_CmdGoToSleep				72
#define PID_CmdPlaySimonSaysOn			73
#define PID_CmdPlaySimonSaysOff			74
#define PID_CmdSimonSays				75

#define PID_CmdIdentifyObject			76
#define PID_CmdPlayStart				77
#define PID_CmdPlayStop					78

#define PID_CmdArmShakeHands			79
#define PID_CmdAttention				80


//#define GID_DICTATION   0           // Dictation grammar has grammar ID 0
// There are three grammars loaded
typedef enum GRAMMARIDS
{
    GID_DICTATION,      // ID for the dictation grammar
    GID_DICTATIONCC,    // ID for the C&C grammar that's active during dictation
    GID_CC              // ID for the C&C grammar that's active when dictation is not
};

typedef enum RECO_MODE
{
    RECO_OFF,				// Recognition disabled
    RECO_DICTATION_MODE,	// Dictation Mode
    RECO_COMMAND_MODE		// Command Mode
};

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////
class CRobotSpeechReco
{

public:
	CRobotSpeechReco();
	~CRobotSpeechReco();

	void		Init();
	void		HandleRecognition( 
					RECO_TYPE RecoType, // Command, Question, Statement, ...
					int Param1,			// If command type, this is the actual command
					int Param2, int Param3, int Param4, float SpeechRecoConfidence );

	void		HandleCommandRecognition( 
					SPEECH_CMD Command,	int Param2, int Param3, int Param4, float SpeechRecoConfidence );

	void		HandleQuestionRecognition( 
					int Param1,	int Param2, int Param3, int Param4, float SpeechRecoConfidence );

	void		SendRecoToAI( BOOL bEnable );
	BOOL		MovementEnabled();
	void		PIDToString( UINT nPID, CString &String );
	void		CmdToString( UINT Command, CString &CmdString );
	void		Speak( const char* TextToSay ); // send to other thread to speak text

	
protected:
	BOOL						m_ProcessMicOnOnly;	// Reco ignored for all commands except "Mic On"
	BOOL						m_bSendRecoToAI;
//	BOOL                        m_bInSound;
//	BOOL                        m_bGotReco;
//	LCID                        m_lcid;
	BOOL						m_bEnableMotorsForSpeechCommands;
//	BOOL						m_bEnableSpeaking;
	BOOL						m_bPlaySimonSays;
	BOOL						m_bSimonSays;
//	ISpVoice					*m_pVoice;
	UINT						m_PhraseNumber;	// rotate through various phrases to speak, avoid the same one twice
	ArmControl					*m_pArmControlRight;
	ArmControl					*m_pArmControlLeft;

};

