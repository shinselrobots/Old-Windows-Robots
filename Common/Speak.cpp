// Speak.cpp
// Based upon sample code from the Samples in the Microsoft Speech SDK (SAPI 5.1)


#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include "thread.h"
//#include "module.h"
#include "Globals.h"
//#include "SpeechEnums.cs"
#include "Speak.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


// GPA TRACE Macros
__itt_string_handle* pshInitSpeech = __itt_string_handle_create("InitSpeech");
__itt_string_handle* pshSpeaking = __itt_string_handle_create("Speaking");





///////////////////////////////////////////////////////////////////////////////
// Robot Speaking Capability Class
///////////////////////////////////////////////////////////////////////////////
CRobotSpeak::CRobotSpeak()
{
	m_bInSound = FALSE;
//	m_bGotReco = FALSE;
	m_lcid = 0;
//	m_bSendRecoToAI = FALSE;
//	m_bEnableMotorsForSpeechCommands = FALSE;
	m_bEnableSpeaking = FALSE;
//	m_bPlaySimonSays = FALSE;
//	m_bSimonSays = FALSE;
//	m_PhraseNumber = 0;
	m_ArmPosition = 0;
	m_pVoice = NULL;
	m_pArmControlRight = new ArmControl( RIGHT_ARM );
	m_pArmControlLeft = new ArmControl( LEFT_ARM );
//	m_pHeadControl = new HeadControl();
	g_MoveArmsWhileSpeaking = FALSE; // off until explicitly turned on

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
CRobotSpeak::~CRobotSpeak()
{
	if( m_pVoice != NULL )
	{
		m_pVoice->Release();
		m_pVoice = NULL;
		::CoUninitialize();
	}
	SAFE_DELETE( m_pArmControlRight );
	SAFE_DELETE( m_pArmControlLeft );
//	SAFE_DELETE( m_pHeadControl );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL CRobotSpeak::Init()
{
	__itt_task_begin(pDomainSpeakThread, __itt_null, __itt_null, pshInitSpeech);

	ROBOT_LOG( TRUE, "\n\n======================== SPEAKING VOICE INITIALIZATION  ============================= \n" )
	m_bEnableSpeaking = FALSE; // Gets turned on if Init completes sucessfullly
	HRESULT hr = S_OK;
	RobotSleep(200, pDomainSpeakThread);


	///////////////////////////////////////////////////////////
	// Initialize Speech Engine for text to speech:
	if( FAILED( CoInitializeEx(NULL, COINIT_MULTITHREADED) )  )
//	if (FAILED(::CoInitialize(NULL)))
	{
		m_pVoice = NULL;
		m_bEnableSpeaking = FALSE;
		//ASSERT(0);
		ROBOT_LOG( TRUE, "\n\n" )
		ROBOT_DISPLAY( TRUE, "=========================================" )
		ROBOT_DISPLAY( TRUE, "ERROR ERROR ERROR - Speech Engine failed!" )
		ROBOT_DISPLAY( TRUE, "=========================================" )
		ROBOT_LOG( TRUE, "\n\n" )
		__itt_task_end(pDomainSpeakThread); // init
		return 0;
	}

	hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&m_pVoice);
	if( FAILED( hr ) )
	{
		ROBOT_LOG( TRUE, "ERROR! Could not initialize Voice!\n" )
		m_pVoice->Release();
		m_pVoice = NULL;
		::CoUninitialize();
		//ASSERT(0);
		__itt_task_end(pDomainSpeakThread); // init
		return 0;
	}

	if( NULL == m_pVoice )
	{
		__itt_task_end(pDomainSpeakThread); // init
		return 0;
	}

    hr = m_pVoice->SetNotifyWindowMessage( g_RobotSetupViewHWND, WM_ROBOT_SPEAKING_COMPLETE_EVENT, 0, 0 );

	const ULONGLONG ullInterest = SPFEI(SPEI_END_INPUT_STREAM);
	hr = m_pVoice->SetInterest(ullInterest, ullInterest);
	if( FAILED( hr ) )
	{
		ROBOT_LOG( TRUE, "ERROR Setting Event Interest!\n" )
		m_pVoice->Release();
		m_pVoice = NULL;
		::CoUninitialize();
		//ASSERT(0);
		__itt_task_end(pDomainSpeakThread); //init
		return 0;
	}

	m_bEnableSpeaking = TRUE;	// ON by default, change if you want it different (required for "Speak" function)
	Speak( L"Initializing System");
	if( FAILED( hr ) )
	{
		ROBOT_LOG( TRUE, "ERROR Speaking!\n" )
		m_bEnableSpeaking = FALSE;
		m_pVoice->Release();
		m_pVoice = NULL;
		::CoUninitialize();
		//ASSERT(0);
		__itt_task_end(pDomainSpeakThread); // init
		return 0;
	}

	ROBOT_LOG( TRUE, "\n\n=================== SPEAKING VOICE INITIALIZATION COMPLETE ========================== \n\n\n" )
	SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 ); // Make sure eyes are enabled

	__itt_task_end(pDomainSpeakThread); // init
	return 1;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobotSpeak::RobotDoneSpeaking()
{
	///TAL_Event("Speaking Done");
	// when robot done speaking, re-enable speech recognition

	ROBOT_LOG( TRUE, "Done Speaking Event (done with all queued work?)\n" )

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobotSpeak::Speak( WCHAR *TextToSpeak )
{
#if( SPEECH_ENGINE_ENABLED == 0 )
	return;
#endif

	if( NULL == m_pVoice )
	{
		// Not initialized yet, or Init failed
		g_CurrentlySpeaking = FALSE;
		return;
	}

	if( !m_bEnableSpeaking )
	{
		ROBOT_LOG( TRUE, "**** SPEAKING DISABLED!  Ignored: %s\n",  TextToSpeak )
		g_CurrentlySpeaking = FALSE;
		return;
	}

	//ROBOT_LOG( TRUE, "**** ROBOT SAYS %s\n",  TextToSpeak )

	// turn off mic while speaking! Mic will be turned back on when speaking is completed (WM_ROBOT_SPEAKING_COMPLETE_EVENT) 
	//	ROBOT_LOG( TRUE, "**** SPEAKING!\n" )

	__itt_task_begin(pDomainSpeakThread, __itt_null, __itt_null, pshSpeaking); // Robot Speaking

	g_CurrentlySpeaking = TRUE;
	HRESULT hr = m_pVoice->Speak( TextToSpeak, 0, NULL );
	if (FAILED(hr))
	{
		ROBOT_ASSERT(0);
	}
	g_CurrentlySpeaking = FALSE;
	__itt_task_end( pDomainSpeakThread ); // Robot Speaking

}


/////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobotSpeak::SpeakIntro()
{
	#if( SPEECH_ENGINE_ENABLED == 0 )
		return;
	#endif


	m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
	//m_pArmControlLeft->SetArmPosition( RIGHT_ARM_SHOULDER_HOME1, RIGHT_ARM_ELBOW_ROTATE_HOME1, RIGHT_ARM_ELBOW_BEND_HOME1, RIGHT_ARM_WRIST_ROTATE_HOME1, RIGHT_ARM_CLAW_HOME1 ); // in DEGREES


	DoRandomArmMovements(); // move arms while speaking
	Speak( L"My name is Low key, and I am 4 years old" );
//	Speak( L"I am 4 years old, and I was built by Dave, who builds robots as his hobby");
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"My brain is a laptop computer, which has an Intel core processor with 1.1 billion transistors" );
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"My actions are controlled by over 25,000 lines of custom C plus plus code" );
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"I have 12 servos, 4 motors, 13 infrared sensors, a compass, two cameras, a laser scanner, and a Kenect sensor" );
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"I am able to understand human commands, navigate my home with a built-in map, and avoid obstacles" );
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"I love humans, other robots, and dogs, but unfortunately dogs dont seem to like me" );
	DoRandomArmMovements(); // move arms while speaking
	Speak( L"the only thing I dont like is darth vaider.  He scares me." );
	DoRandomArmMovements(); // move arms while speaking
	
	m_pArmControlLeft->MoveArmHome( SERVO_SPEED_MED );
	m_pArmControlLeft->EnableIdleArmMovement(TRUE);
	m_pArmControlRight->MoveArmHome( SERVO_SPEED_MED );
	m_pArmControlRight->EnableIdleArmMovement(TRUE);
	gHeadIdle = TRUE;

	SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, 120, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 ); // face forward
	RobotSleep(3000, pDomainSpeakThread);
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 5 ); // face forward
	Speak( L"what should we do next?" );

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobotSpeak::SpeakFromGlobalBuffer()
{

	#if( SPEECH_ENGINE_ENABLED == 0 )
		return;
	#endif
	CString TextToSpeak;

	if( g_SpeakQueue.empty() )
	{
		// *** OLD GLOBAL METHOD - TODO REMOVE THIS ***
		// Get text to speak from the global text buffer
		// Convert from string to wide character format needed by Speak.
		// I'm sure there is some conversion function to do this in Microsoft ATL land, but this works...

		// Nod the head when speaking
		SendCommand( WM_ROBOT_CAMERA_NOD_HEAD_CMD, (DWORD)(0), (DWORD)0 );
		Sleep(1);

		char WStrBuf[WIDE_BULK_DATA_SIZE];
		char * pWStrBuf = WStrBuf;

		ROBOT_LOG( TRUE, "ROBOT SPEAKS: \"%s\"\n", g_ClientTextToSend )
		memset(WStrBuf, 0, WIDE_BULK_DATA_SIZE);
		int StrLen = g_ClientTextToSend.GetLength();
		if( 0 == StrLen )
		{
			ROBOT_LOG( TRUE, "ERROR - Empty text, nothing to say\n" )
			return;
		}
		if( StrLen > BULK_DATA_SIZE ) StrLen = BULK_DATA_SIZE;
		for(int i = 0; i < g_ClientTextToSend.GetLength(); i++)
		{
			*pWStrBuf = g_ClientTextToSend[i];
			pWStrBuf+=2;
		}

		// turn off mic while speaking! Mic will be turned back on when speaking is completed (WM_ROBOT_SPEAKING_COMPLETE_EVENT) 
		//	ROBOT_LOG( TRUE, "**** SPEAKING FROM GLOBAL BUFFER!\n" )

		Speak((WCHAR *)WStrBuf);
		g_ClientTextToSend.Empty();	// Reset the buffer to empty
	}
	else
	{
		// Something in the queue to say
		//m_ArmPosition = 0; // first position
		while (!g_SpeakQueue.empty())
		{
			// Get text to speak from the global queue
			CString NextPhrase = g_SpeakQueue.front();

			// Nod the head when speaking
			SendCommand( WM_ROBOT_CAMERA_NOD_HEAD_CMD, (DWORD)(0), (DWORD)0 );
			Sleep(1);

			// parse the text to look for pauses or other commands.  When we see a pause, we wait before saying the text
			int curPos = 0;

			while( TRUE )
			{
				CAtlString TokenString;
				TokenString = NextPhrase.Tokenize("[]", curPos);
				if( _T("") == TokenString  )
				{
					break;
				}

				if( _T(" ") == TokenString  )
				{
					continue;
				}

				// Check for special characters in the token string
				// '*P' = Pause,   '*A' = Arm Behavior, '*H' = head behavior
				//char FirstChar = TokenString.Left(1);
				if( '*' == TokenString.Left(1) )
				{
					// Handle special command
					CString tempStr = TokenString.Mid(1,1);
					char TokenType = tempStr[0];
					//char TokenType = TokenString[1];
					CString ParamString = TokenString.Mid(2); // get string without the special characters
					int Param = atoi( ParamString );
					if( Param <= 0 )
					{
						// Token was not a number
						ROBOT_LOG( TRUE, "ERROR - Bad Param! [%s]\n", ParamString )
					}
					else
					{
						HandleSpecialCommands( TokenType, Param );
					}
					continue; // process next token
				} // Special Characters
				else
				{
					// normal string to speak

					if( g_MoveArmsWhileSpeaking )
					{
						DoRandomArmMovements(); // move arms while speaking
					}

					TextToSpeak = TokenString;
					// TODO? turn off mic while speaking! Mic will be turned back on when speaking is completed (WM_ROBOT_SPEAKING_COMPLETE_EVENT) 

					int StrLen = TextToSpeak.GetLength();
					if( 0 == StrLen )
					{
						ROBOT_LOG( TRUE, "ERROR - Empty text in queue, nothing to say\n" )
						continue; // process next token
					}

					// Convert from string to wide character format needed by Speak.
					// I'm sure there is some conversion function to do this in Microsoft ATL land, but this works...
					if( StrLen > BULK_DATA_SIZE ) StrLen = BULK_DATA_SIZE;
					char WStrBuf[WIDE_BULK_DATA_SIZE];
					char * pWStrBuf = WStrBuf;
					memset(WStrBuf, 0, WIDE_BULK_DATA_SIZE);
					for(int i = 0; i < TextToSpeak.GetLength(); i++)
					{
						*pWStrBuf = TextToSpeak[i];
						pWStrBuf+=2;
					}

					// Finally, Say it!
					ROBOT_LOG( TRUE, "ROBOT SPEAKS: \"%s\"\n", TextToSpeak )
					Speak( (WCHAR *)WStrBuf );

				}

			}	// while SubString not empty

			g_SpeakQueue.pop(); // Remove text from the queue

		} // while g_SpeakQueue not empty

	}

} // SpeakFromGlobalBuffer

/////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleSpecialCommands()
// Special commands embedded in the text sent to this speech class
// Formatting is "[*tnHello]"  where t = TokenType, and n = Param number, and "Hello" is an example string
//
void CRobotSpeak::HandleSpecialCommands( char TokenType, int Param )
{
	if( 'P' == TokenType )
	{
		// Pause command
		if( g_MoveArmsWhileSpeaking )
		{
			DoRandomArmMovements(); // move even while pausing in speech
		}
		ROBOT_LOG( TRUE, "Sleeping %d ms before speaking\n", Param )
		Sleep( Param );

	}
	else if( 'T' == TokenType )
	{
		// Set Token to indicate speech completed to this point
		// Parm is ignored for now
			g_PhraseDoneTokenProcessed = TRUE; // tell calling thread that speech reached this point
		ROBOT_LOG( TRUE, "Speech complete Token processed\n" )

	}
	else if( 'A' == TokenType )
	{	
		// 'A' = Arm command
		if ( SPEECH_ARM_MOVEMENT_RANDOM_OFF == Param )
		{				
			g_MoveArmsWhileSpeaking = FALSE; // stop moving arms while talking
		}
		else if( SPEECH_ARM_MOVEMENT_RANDOM_ON == Param )
		{
			g_MoveArmsWhileSpeaking = TRUE; // move arms while talking
		}
		else if( SPEECH_ARM_MOVEMENT_HOME == Param )
		{
			// Move arms to "Talking Home", AKA First Position
			m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MED, SERVO_SPEED_MED );
			m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MED, SERVO_SPEED_MED );
			m_pArmControlRight->SetArmPosition( 20,  0, 125, 0, 10 ); // Start talking position
			m_pArmControlLeft->SetArmPosition( 15,   0, 105, 0, 10 ); // Start talking position
			m_pArmControlLeft->ExecutePositionAndSpeed();
			m_pArmControlRight->ExecutePositionAndSpeed();
		}
		else if( SPEECH_ARM_MOVEMENT_SMALL_UP == Param )
		{
		}
		else if( SPEECH_ARM_MOVEMENT_SMALL_DOWN == Param )
		{
		}
		else if( SPEECH_ARM_MOVEMENT_SMALL_DOWN == Param )
		{
		}
		else if( SPEECH_ARM_MOVEMENT_BOXING1 == Param )
		{
			m_pArmControlRight->SetArmPosition( 50, -30, 130, NOP, 1 ); 
			m_pArmControlLeft->SetArmPosition(  50,  25, 130, NOP, 10 ); 
			m_pArmControlRight->ExecutePosition();
			m_pArmControlLeft->ExecutePosition();
		}
		else if( SPEECH_ARM_MOVEMENT_BOXING2 == Param )
		{
			m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED, SERVO_SPEED_MED );
			m_pArmControlRight->SetArmPosition( 60, NOP, 100, NOP, NOP ); 
			//m_pArmControlLeft->SetArmPosition(  50, NOP, 135, NOP, NOP ); 
			m_pArmControlRight->ExecutePositionAndSpeed();
			//m_pArmControlLeft->ExecutePositionAndSpeed();
		}
		else
		{
			ROBOT_LOG( TRUE, "ERROR - Bad Arm Movement type! [%d]\n", TokenType )
		}
	}
	else if( 'H' == TokenType )
	{
		ROBOT_LOG( TRUE, "ERROR - HEAD Special Character not implemented! [%d]\n", TokenType )
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR - Bad Special Character! [%d]\n", TokenType )
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobotSpeak::DoRandomArmMovements()
{
	// Tell arms to move while talking

	//int RandomNumber = ((4 * rand()) / RAND_MAX);
	ROBOT_LOG( TRUE, "Moving Arms to position %d\n", m_ArmPosition )
	// Set speed everytime, incase some other funtion took over the arms for a bit
	m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MED, SERVO_SPEED_MED );
	m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MED, SERVO_SPEED_MED );

	// make sure arm is out far enough for random movements without hitting the body
	if( (m_pArmControlRight->GetShoulderPosition() < 15) || //some fudge for servo not reaching commanded position
		(m_pArmControlLeft->GetShoulderPosition() < 5) )
	{
		m_ArmPosition = 0;
	}

	switch( m_ArmPosition++ )
	{
		case 0:

			ROBOT_LOG( TRUE, "Moving Arms to first position \n" )
			m_pArmControlRight->SetArmPosition( 20,  0, 125, 0, 10 ); // Start talking position
			m_pArmControlLeft->SetArmPosition( 15,   0, 105, 0, 10 ); // Start talking position
			break;
		case 1:  
			m_pArmControlRight->SetArmPosition( 20,  -5, 125, NOP, 10 ); // Start talking position
			//m_pArmControlLeft->SetArmPosition( 20,   0, 105, NOP, 10 ); // Start talking position
			break;
		case 2:  
			m_pArmControlRight->SetArmPosition( 20, -10, 115, NOP, 10 ); // Hand in
			m_pArmControlLeft->SetArmPosition( 10,   10, 115, NOP, 10 ); // Hand in
			break;
		case 3:  
			m_pArmControlRight->SetArmPosition( 20,  5, 105, NOP, 30 ); // Hand out
			//m_pArmControlLeft->SetArmPosition( 20,  -5, 105, NOP, 30 ); // Hand out
			break;
		case 4:  
			m_pArmControlRight->SetArmPosition( 25, -5, 125, NOP, 10 ); // Hand in small
			//m_pArmControlLeft->SetArmPosition( 20,   5, 115, NOP, 10 ); // Hand in small
			break;
		case 5:  
			m_pArmControlRight->SetArmPosition( 20,  10, 125, NOP, 10 ); // Hand out small
			//m_pArmControlLeft->SetArmPosition( 20,  -10, NOP, NOP, 10 ); // Hand out small
			m_ArmPosition = 1; // restart
			break;
		case 6:
			m_ArmPosition = 1; // restart
			break;
		default: 
			m_ArmPosition = 1; // restart
		break;
	}

	m_pArmControlLeft->ExecutePositionAndSpeed();
	m_pArmControlRight->ExecutePositionAndSpeed();

}
#endif	// ROBOT_SERVER

