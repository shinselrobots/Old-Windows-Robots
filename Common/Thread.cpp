// Thread.cpp

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include "HardwareCmds.h"
#include "HWInterfaceParams.h"
#include "thread.h"
#include "module.h"

#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
//#include <sapi.h>
//#include <stdio.h>
//#include <string.h>
//#include <atlbase.h>
#include <sphelper.h>
#include "Kinect.h"
#include "SpeechEnums.cs"
#include "SpeechReco.h"
#include "Speak.h"
#include "CameraCommon.h"
#include "KobukiCommon.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Auto launch Kinect C# Application
#define AUTO_LAUNCH_KINECT_APP		1	// set value to 1 to auto launch
#if (AUTO_LAUNCH_KINECT_APP == 1)
	FIND_WINDOW_HANDLE_STRUCT_T KinectFWHS; // global to this file
#endif


// ITT
//const wchar_t threadLoopsCounterName[] = L"Thread loops";
//const wchar_t taskOpsCounterName[] = L"Task ops";

#define DEBUG_KOBUKI_SHARED_MEMORY 0
#define DEBUG_KINECT_SHARED_MEMORY 0
//-----------------------------------------------------------------------------
//	Name: ControlThreadProc
//	This thread is the heart of the robot control system.
//	The tread gets status and requests from the message loop
//	and sends them to various modules for processing.
//	The order of dispatch is important!  Highest priority modules
//	are processed first.  When a module wishes to suppress futher
//	processing, it sets the CmdDisabled flag.
//-----------------------------------------------------------------------------
///
__itt_string_handle* pshControlThreadLoop = __itt_string_handle_create("ControlThreadLoop");
__itt_string_handle* pshSensorModule = __itt_string_handle_create("SensorModule");
__itt_string_handle* pshCameraModule = __itt_string_handle_create("CameraModule");
__itt_string_handle* pshDepthCameraModule = __itt_string_handle_create("DepthCameraModule");
__itt_string_handle* pshKinectModule = __itt_string_handle_create("KinectModule");
__itt_string_handle* pshSystemModule = __itt_string_handle_create("SystemModule");
__itt_string_handle* pshCollisionModule = __itt_string_handle_create("CollisionModule");
__itt_string_handle* pshAvoidModule = __itt_string_handle_create("AvoidModule");
__itt_string_handle* pshUserCmdModule = __itt_string_handle_create("UserCmdModule");
__itt_string_handle* pshWayPointModule = __itt_string_handle_create("WayPointModuleX");
__itt_string_handle* pshGridNavModule = __itt_string_handle_create("GridNavModule");
__itt_string_handle* pshBehaviorModule = __itt_string_handle_create("BehaviorModule");
__itt_string_handle* pshControlThreadLoopTop = __itt_string_handle_create("Top of Loop"); //marker

__itt_string_handle* pSH2 = __itt_string_handle_create("index");


DWORD WINAPI ControlThreadProc( LPVOID NotUsed )
{
	IGNORE_UNUSED_PARAM (NotUsed);
	__itt_thread_set_name( "Control Thread" );

	RobotSleep(4000, pDomainControlThread); // allow other threads to start first


	MSG	msg;
	UINT UserMessage = 0;
	UINT LoopIndex = 0;
	BOOL MotorsPaused = FALSE;

	// Allocate robot modules
	CDriveControlModule	*pDriveControlModule = new CDriveControlModule();
	CSensorModule		*pSensorModule = new CSensorModule(pDriveControlModule);
////	CCameraModule		*pCameraModule = new CCameraModule(pDriveControlModule);  
////						 g_pCameraModule = (void*)pCameraModule; // Make pointer to class available to video callback
	CDepthCameraModule	*pDepthCameraModule = new CDepthCameraModule(pDriveControlModule); 
	CKinectModule		*pKinectModule = new CKinectModule(pDriveControlModule); // Kinect Module creates its own video thread 
						 g_pKinectModule = (void*)pKinectModule; // Make pointer to class available to video callback
	CSystemModule		*pSystemModule = new CSystemModule(pDriveControlModule);
	CCollisionModule	*pCollisionModule = new CCollisionModule(pDriveControlModule);
	CAvoidObjectModule	*pAvoidObjectModule = new CAvoidObjectModule(pDriveControlModule);
	CUserCmdModule		*pUserCmdModule = new CUserCmdModule(pDriveControlModule);
	CWayPointNavModule	*pWayPointNavModule = new CWayPointNavModule(pDriveControlModule, pSensorModule);
	CGridNavModule		*pGridNavModule = new CGridNavModule(pDriveControlModule, pSensorModule);
	CBehaviorModule		*pBehaviorModule = new CBehaviorModule(pDriveControlModule);


	RobotSleep(1500, pDomainControlThread);	// Let other init run
	


	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MAX_ROBOT_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		{
			if( WM_ROBOT_THREAD_EXIT_CMD == (msg.message) )
			{
				break;	// Quit command received!
			}

			if( (msg.message < WM_ROBOT_MESSAGE_BASE) || 
				((msg.message) >= WM_GUI_MESSAGES) ||		// Ignore messages intended for the GUI
				((msg.message) > WM_ROBOT_MAX_ROBOT_MESSAGE) )	// Or messages out of range
			{
				// Ignore any messages not intended for user (where do these come from?)
				continue;
			}


			if( g_GlobalPause )
			{
				// Global Pause requested!  Stop drive motors.
				if( !MotorsPaused )
				{
					// Not yet paused, so stop motors now (issue one-time command)
					pDriveControlModule->Stop( LOCAL_USER_MODULE );
					MotorsPaused = TRUE;
				}

				// Now, Block all commands until pause reset
				// This allows easy debug of state when something bad happens, while protecting hardware from damage
				// in the meantime, just throw away new messages
				continue;
			}
			else
			{
					MotorsPaused = FALSE;
			}
		

			// Process the message in the queue.

			UserMessage = msg.message;
			g_bCmdRecognized = FALSE;

			__itt_marker(pDomainControlThread, __itt_null, pshControlThreadLoopTop, __itt_marker_scope_task);
			__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshControlThreadLoop);
			//__itt_metadata_add(pDomainControlThread, __itt_null, pshControlThreadLoop, __itt_metadata_s32, 1, (void*)&LoopIndex);

			__itt_metadata_add(pDomainControlThread, __itt_null, pSH2, __itt_metadata_s32, 1, (void*)&LoopIndex);
			LoopIndex++;

			// Dispatch the message to each module
			// Sensor modules are called first, so all sensor data gets processed and fused to sensor summary other modues can use.
			if( WM_ROBOT_SENSOR_STATUS_READY == msg.message )
			{
				pDriveControlModule->BeginSensorUpdate();
			}

			////////////////////////////////////////
			// Sensory Modules
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshSensorModule);
				pSensorModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			/**
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCameraModule);
				pCameraModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			**/
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshKinectModule);
				pKinectModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshDepthCameraModule);
				pDepthCameraModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshSystemModule);
				pSystemModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}

			////////////////////////////////////////
			// Behavioral modules
			// Lowest to hightest priority, so higher priority modules can get hints from lower priority modules

			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshBehaviorModule);
				pBehaviorModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGridNavModule);
				pGridNavModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshWayPointModule);
				pWayPointNavModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshUserCmdModule);
				pUserCmdModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshAvoidModule);
				pAvoidObjectModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCollisionModule);
				pCollisionModule->ProcessMessage( UserMessage, msg.wParam, msg.lParam );
				__itt_task_end(pDomainControlThread);
			}

			if( !g_bCmdRecognized )
			{
				ROBOT_LOG( TRUE, _T( "Thread.cpp: ERROR unknown message (probably new) Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n"), 
					UserMessage, msg.wParam, msg.lParam )
			}
			else
			{
				// DriveControlModule arbitrates requests from other modules and issues
				// the command to the motor control.  Send motor commands after each status update
				if( WM_ROBOT_SENSOR_STATUS_READY == msg.message )
				{
					///TAL_SCOPED_TASK_NAMED("Execute Drive Control");
	//				ROBOT_LOG( TRUE,  "DBG: Thread Execute Cmd %02X\n",(msg.message) )
					// 
					//pDriveControlModule->ExecuteCommand();
					pDriveControlModule->EndSensorUpdate();

				}
			}
			__itt_task_end(pDomainControlThread);  // pshControlThreadLoop
		}
		Sleep(0); // force thread switch after each command
	}

	// We received a WM_ROBOT_THREAD_EXIT_CMD or WM_QUIT message, exit
	ROBOT_LOG( TRUE,  "ControlThread exiting.\n" )
	// Release modules (in reverse order)
	SAFE_DELETE( pBehaviorModule );
	SAFE_DELETE( pGridNavModule );
	SAFE_DELETE( pWayPointNavModule );
	SAFE_DELETE( pUserCmdModule );
	SAFE_DELETE( pAvoidObjectModule );
	SAFE_DELETE( pCollisionModule );
	SAFE_DELETE( pSystemModule );
////	SAFE_DELETE( pCameraModule );
	SAFE_DELETE( pDepthCameraModule );
	SAFE_DELETE( pKinectModule );
	SAFE_DELETE( pSensorModule );
	SAFE_DELETE( pDriveControlModule );

	return 0;
}

//-----------------------------------------------------------------------------
// Name: SoundThreadProc
// This thread plays sounds to avoid the latency of startup for ASYNC sounds
// messages are sent to g_dwSoundThreadId
//-----------------------------------------------------------------------------
DWORD WINAPI SoundThreadProc( LPVOID NotUsed ) // g_dwSoundThreadId
{
	IGNORE_UNUSED_PARAM (NotUsed);
	__itt_thread_set_name( "Sound Thread" );

	MSG	msg;
    HRESULT hr = E_FAIL;

	RobotSleep(5000, pDomainControlThread); // allow other threads to start first


// TODO!  Does this thread still do speach, or just music?  Is it needed?  Can we play iTunes?

	////////////////////////////////////////////
	////////////////////////////////////////////
	// WHEN NEXT LINE IS UNCOMMENTED, ALL VOICE DISABLED!!!
	//	return 0;
	////////////////////////////////////////////
	////////////////////////////////////////////


	// for Wav playback:
	DWORD dwOptions = SND_ASYNC | SND_FILENAME | SND_NODEFAULT ;	// SND_NOWAIT

	///////////////////////////////////////////////////////////////////////////////////////
	// Initialize Speech Engine For Text to speech:
/****
	ISpVoice * pVoice = NULL;


//	IStream   *pStream, 
//	DWORD      dwFlags, 
//	ULONG     *pulStreamNumber


	if( FAILED( CoInitializeEx(NULL, COINIT_MULTITHREADED) )  )
//	if (FAILED(::CoInitialize(NULL)))
	{
		pVoice = NULL;
		ASSERT(0);
		return 0;
	}

	hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
	if( FAILED( hr ) )
	{
		ROBOT_LOG( TRUE,  "ERROR! Could not initialize Voice!\n" )
		pVoice->Release();
		pVoice = NULL;
		::CoUninitialize();
		ASSERT(0);
		return 0;
	}

	// Disable Speech recogntion while talking
	PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, FALSE, 0 );
	Sleep(10);	// Allow time to turn off reco engine

	hr = pVoice->Speak(L"Initializing System", 0, NULL);
	if( FAILED( hr ) )
	{
		ROBOT_LOG( TRUE,  "ERROR Speaking!\n" )
	}
	PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, TRUE, 0 );
***/
	// test wave file
/*** TODO
   hr = cpSpStream->BindToFile(WAVE_FILENAME, SPFM_OPEN_READONLY, NULL, NULL, NULL);
	hr = SpeakStream(
	pStream, 
	dwFlags, 
	pulStreamNumber );
***/

	///////////////////////////////////////////////////////////////////////////////////////
	// Loop forever


	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MAX_ROBOT_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		if( WM_ROBOT_THREAD_EXIT_CMD == (msg.message) )
		{
			break;	// Quit command received!
		}

		// 

		// Get message from the message queue and process it.
		if( (WM_ROBOT_PLAY_MUSIC) == msg.message )
		{
			// Start playing Music
			// for Wav files, options are: 	DWORD dwOptions = SND_SYNC | SND_FILENAME | SND_NODEFAULT ;	// SND_NOWAIT

			LPCTSTR pFileName;
			switch( msg.wParam )  // wParam is the number of the music requested
			{
				case MUSIC_STOP:
				{
					ROBOT_LOG( TRUE,  "Music Stop Requested\n" )
					pFileName = MUSIC_SILENCE_STR; // HACK - play silent wav to kill prior.
					break;
				}
				case MUSIC_MR_ROBOTO:
				{
					ROBOT_LOG( TRUE,  "Mr Roboto Requested\n" )
					pFileName = MUSIC_MR_ROBOTO_STR;
					break;
				}
				case MUSIC_HAWAII50:
				{
					ROBOT_LOG( TRUE,  "Hawaii 50 Requested\n" )
					pFileName = MUSIC_HAWAII50_STR;
					break;
				}
				default:
				{
					ROBOT_LOG( TRUE,  ">>>> ERROR: Unmapped Music Request!\n" )
					continue;
				}
			}
			
			// Disable Speech recogntion while playing music - TODO - change this to allow STOP voice command!  Instead, just stop text from getting to the AI...
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, FALSE, FALSE );
			Sleep(10);	// Allow time to turn off reco engine

			BOOL bSucess = PlaySound(
				pFileName,		// Filename  
				NULL,			// No device Handle needed   
				dwOptions);		// Control Flags
			if( !bSucess )
			{
				ROBOT_LOG( TRUE,  ">>>> ERROR: Could not play sound %s!\n", pFileName)
			}
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, TRUE, FALSE );
		}
		else if( (WM_ROBOT_PLAY_SOUND) == msg.message )
		{
/***
			// Play "sound bytes" from Wav files

  // TODO-LOKI!!!

			// Disable Speech recogntion while talking
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, FALSE, FALSE );
			Sleep(10);	// Allow time to turn off reco engine

			LPCTSTR pFileName;

			switch( msg.wParam )  // wParam is the Sound requested
			{
				case SPEAK_TEXT_FROM_BUFFER:
				{
					ROBOT_ASSERT(0);

				// Convert from string to wide character format needed by Speak.
					// I'm sure there is somen conversion function to do this in Microsoft ATL land, but this works...
					char WStrBuf[WIDE_BULK_DATA_SIZE];
					char * pWStrBuf = WStrBuf;
					memset(WStrBuf, 0, WIDE_BULK_DATA_SIZE);
					int StrLen = g_ClientTextToSend.GetLength();
					if( StrLen > BULK_DATA_SIZE ) StrLen = BULK_DATA_SIZE;
					for(int i = 0; i < g_ClientTextToSend.GetLength(); i++)
					{
						*pWStrBuf = g_ClientTextToSend[i];
						pWStrBuf+=2;
					}
					hr = pVoice->Speak((const unsigned short *)WStrBuf, 0, NULL);
					g_ClientTextToSend.Empty();	// Reset the buffer to empty				

				}
				break;
				case SOUND_PIC_CONNECTED:
				{
					hr = pVoice->Speak(L"Arduino Connected!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\PicConnectedSound.wav");
				}
				break;
				case SOUND_PIC_VERSION_ERROR:
				{
					Beep(1000,500);
					Beep(200,500);
					Beep(1000,500);
					Beep(200,500);
					hr = pVoice->Speak(L"WARNING! WARNING! Arduino Version Error!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\PicConnectedSound.wav");
				}
				break;				
				case SOUND_SIMULATION_MODE:
				{
					hr = pVoice->Speak(L"Simulation Mode!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\PicSimulationMode.wav");
				}
				break;
				case SOUND_COLLISION:
				{
					hr = pVoice->Speak(L"Ouch", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\CollisionSound.wav");
				}
				break;
				case SOUND_AVOID:
				{
					hr = pVoice->Speak(L"Avoiding!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\AvoidSound.wav");
				}
				break;
				case SOUND_TRAPPED:
				{
					hr = pVoice->Speak(L"Help, I am trapped!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\TrappedSound.wav");
				}
				break;
				case SOUND_EXECUTING_PATH:
				{
					hr = pVoice->Speak(L"Executing Path!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\ExecutingPath.wav");
				}
				break;
				case SOUND_WAYPOINT_REACHED:
				{
					hr = pVoice->Speak(L"Waypoint Reached!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\WayPointReached.wav");
				}
				break;
				case SOUND_LOOKING_FOR_LANDMARK:
				{
					hr = pVoice->Speak(L"Looking for Landmark!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\LookingForLandmark.wav");
				}
				break;
				case SOUND_HEADING_FOR_OBJECT:
				{
					hr = pVoice->Speak(L"Heading For Object!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\HeadingForObject.wav");
				}
				case SOUND_DARTH_VADER:
				{
					ROBOT_LOG( TRUE,  "===============  Doing Darth!=================\n" )

					hr = pVoice->Speak(L"Would you like to hear my impression of Darth Vaider?", 0, NULL);
					Sleep(1000);

					// Position Camera slightly to left

					SendCommand( WM_ROBOT_CAMERA_PAN_TILT_SPEED_CMD, (DWORD)16, (DWORD)16 );	// Fairly fast
					SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)(CAMERA_PAN_TENTHDEGREES_MAX_LEFT/2), 0 );
					SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)(CAMERA_PAN_CENTER), 0 );

					//Sleep(500);
					hr = pVoice->Speak(L"OK, here goes,", 0, NULL);

					Sleep(1000);

					// Slow Pan camera while talking like Darth
					SendCommand( WM_ROBOT_CAMERA_PAN_TILT_SPEED_CMD, (DWORD)6, (DWORD)6 );	// Slow pan
					SendCommand( WM_ROBOT_LIGHT_POWER_CMD, 0, (DWORD)TRUE );	// Lights On
					Sleep(300);
					SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)(CAMERA_PAN_TENTHDEGREES_MAX_RIGHT/2), 0 );

					Sleep(300);
					pFileName = _T("..\\RobotData\\RobotSounds\\vader_bidding.wav");
					BOOL bSucess = PlaySound(
						pFileName,		// Filename  
						NULL,			// No device Handle needed   
						dwOptions);		// Control Flags
					if( !bSucess )
					{
						ROBOT_LOG( TRUE,  ">>>> ERROR: Could not play sound vader_bidding.wav!\n" )
					}
					Sleep(500);
					SendCommand( WM_ROBOT_LIGHT_POWER_CMD, 0, (DWORD)FALSE );	// Lights Off
					Sleep(1000);
					SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 7 );
					Sleep(2000);
					hr = pVoice->Speak(L"So, Did you like my Darth Vaider impression?", 0, NULL);
					Sleep(1000);
					hr = pVoice->Speak(L"Good.. Well, that's all I can do so far.", 0, NULL);
					SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)(CAMERA_PAN_CENTER+20), 0 );
					hr = pVoice->Speak(L"Dave, Do really need to make me smarter when you get time! ", 0, NULL);
					hr = pVoice->Speak(L"Perhaps more memory would help?", 0, NULL);
					Sleep(500);
					SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)(CAMERA_PAN_CENTER-20), 0 );
					hr = pVoice->Speak(L"Well, I am tired, I think I will power down for a while", 0, NULL);
					SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 7 );
					SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)(CAMERA_TILT_CENTER+20), 0 );
					SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)FALSE, (DWORD)FALSE );	// Eyes Off

					ROBOT_LOG( TRUE,  "===============  Darth Done!=================\n" )
				}
				break;
				default:
				{
					hr = pVoice->Speak(L"Unknown Speak Command!", 0, NULL);
					//pFileName = _T("..\\RobotData\\RobotSounds\\UndefinedSound.wav");
					ROBOT_LOG( TRUE,  ">>>> ERROR: Unknown sound!\n" )
				}
			}	// end switch
			// ReEnable Speech recogntion when done
			PostMessage( g_RobotSetupViewHWND, WM_ROBOT_ENABLE_RECO, TRUE, FALSE );
***/
		}
		else
		{
			// Ignore any messages not intended for this function
			UINT temp = WM_ROBOT_MESSAGE_BASE;	// for debug
			ROBOT_LOG( TRUE,  "SOUND THREAD: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				temp, msg.message, msg.wParam, msg.lParam )
			continue;
		}
		Sleep(0); // force thread switch after each command
	}

	// We received a WM_ROBOT_THREAD_EXIT_CMD or WM_QUIT message, exit
	ROBOT_LOG( TRUE,  "SoundThread exiting.\n" )

	// Release voice resources
/*	if( pVoice != NULL )
	{
		pVoice->Release();
		pVoice = NULL;
		::CoUninitialize();
	}
	*/
	return 0;
}


//-----------------------------------------------------------------------------
// Name: SpeakThreadProc
// This thread outputs words - the Robot's Voice!
//-----------------------------------------------------------------------------

//#if SPEECH_ENGINE_ENABLED == 1

DWORD WINAPI SpeakThreadProc( LPVOID NotUsed ) // g_dwSpeakThreadId
{
	__itt_thread_set_name( "Speak Thread" );
	#if ( ROBOT_SERVER == 1 )

	MSG	msg;
//    HRESULT hr = E_FAIL;
//	CString input, ResponseString, MsgString;


	// Speech Engine is one of the first modules to start, so the robot can report startup progress
	// Enable Speech Recognition by default on server.  Disabled for client.
	CRobotSpeak *pRobotSpeak = new CRobotSpeak;

	///////////////////////////////////////////////////////////////////////////////////////
	// Initialize Speech Engine
	ROBOT_LOG( TRUE,  "\n=============================================================\n" )
	ROBOT_LOG( TRUE,  "              INITIALIZING SPEAK\n" )
	ROBOT_LOG( TRUE,  "=============================================================\n\n" )

	if( !pRobotSpeak->Init() )
	{
		ROBOT_DISPLAY( TRUE, "Error Initializing Speak Engine.  SpeakThreadProc Exiting!")
		return 0;
	}

	///////////////////////////////////////////////////////////////////////////////////////
	// Loop forever

	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MAX_ROBOT_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		///TAL_SCOPED_TASK_NAMED("Speak MSG Loop");

		if( WM_ROBOT_THREAD_EXIT_CMD == msg.message )
		{
			break;	// Quit command received!
		}

		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || 
			((msg.message) >= WM_GUI_MESSAGES) ||		// Ignore messages intended for the GUI
			((msg.message) > WM_ROBOT_MAX_ROBOT_MESSAGE) )	// Or messages out of range
		{
			// Ignore any messages not intended for this thread
			continue;
		}

		// Get message from the message queue and process it.
		if( (WM_ROBOT_SPEAK_TEXT) == msg.message )
		{
			switch( msg.wParam )  // wParam is the Sound requested
			{
				case SPEAK_TEXT_FROM_BUFFER:
				{
					pRobotSpeak->SpeakFromGlobalBuffer();
				}
				break;
				case SPEAK_INTRO:
				{
					// allow time for the wave to finish
					//Sleep(2000);
					//SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_MOVE_WHILE_TALKING, (DWORD)0 );
					//Sleep(500);
					pRobotSpeak->SpeakIntro();
					//SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)ACTION_MODE_NONE, (DWORD)0 ); // cancel arm motion
				}
				break;
				case SPEAK_ARDUINO_CONNECTED:
				{
					pRobotSpeak->Speak(L"Sensors On Line");
					//pFileName = _T("..\\RobotData\\RobotSounds\\PicConnectedSound.wav");
				}
				break;
				case SPEAK_ARDUINO_VERSION_ERROR:
				{
					Beep(1000,500);
					Beep(200,500);
					Beep(1000,500);
					Beep(200,500);
					pRobotSpeak->Speak(L"WARNING! WARNING! Arduino Version Error!");
				}
				break;				
				case SPEAK_SIMULATION_MODE:
				{
					pRobotSpeak->Speak(L"Simulation Mode!");
				}
				break;
				case SPEAK_COLLISION:
				{
					pRobotSpeak->Speak(L"Ouch");
					//pFileName = _T("..\\RobotData\\RobotSounds\\CollisionSound.wav");
				}
				break;
				case SPEAK_AVOID:
				{
					pRobotSpeak->Speak(L"Avoiding!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\AvoidSound.wav");
				}
				break;
				case SPEAK_TRAPPED:
				{
					pRobotSpeak->Speak(L"Help, I am trapped!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\TrappedSound.wav");
				}
				break;
				case SPEAK_EXECUTING_PATH:
				{
					pRobotSpeak->Speak(L"Executing Path!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\ExecutingPath.wav");
				}
				break;
				case SPEAK_WAYPOINT_REACHED:
				{
					pRobotSpeak->Speak(L"Waypoint Reached!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\WayPointReached.wav");
				}
				break;
				case SPEAK_LOOKING_FOR_LANDMARK:
				{
					pRobotSpeak->Speak(L"Looking for Landmark!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\LookingForLandmark.wav");
				}
				break;
				case SPEAK_HEADING_FOR_OBJECT:
				{
					pRobotSpeak->Speak(L"Heading For Object!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\HeadingForObject.wav");
				}
				break;
				default:
				{
					pRobotSpeak->Speak(L"Unknown Speak Command!");
					//pFileName = _T("..\\RobotData\\RobotSounds\\UndefinedSound.wav");
					ROBOT_LOG( TRUE,  ">>>> ERROR: Unknown Speak Command! %04X\n",(UINT)msg.wParam )
				}
			}	// end switch
		}
		else
		{
			// Ignore any messages not intended for this function
			UINT temp = WM_ROBOT_MESSAGE_BASE;	// for debug
			ROBOT_LOG( TRUE,  "Message out of Range, WM_ROBOT_MESSAGE_BASE = %08X, Msg = 0x%08lX, wParam = %08lX, lParam = %08lX\n", 
				temp, msg.message, msg.wParam, msg.lParam )
			continue;
		}
		Sleep(0); // force thread switch after each command
	}

	// We received a WM_ROBOT_THREAD_EXIT_CMD or WM_QUIT message, exit
	ROBOT_LOG( TRUE,  "SpeakThread exiting.\n" )

	// Release resources
	SAFE_DELETE( pRobotSpeak );
	return 0;

	#endif	// ROBOT_SERVER
}	// End of Speak Thread

//#endif //if (SPEECH_ENGINE_ENABLED == 1)


//-----------------------------------------------------------------------------
// Name: LaunchKinectApp
// Desc: If enabled, auto-launch the C# applicaiton that handles the 
// Kinect audio and video+depth cameras
// Note: started at different times for Loki or Turtle, since iRobot base enables power only on startup
//-----------------------------------------------------------------------------

// Shut down the Kinect C# process
void TerminateKinectApp()
{
	#if (AUTO_LAUNCH_KINECT_APP == 1)
		TerminateProcess( KinectFWHS.ProcessInfo.hProcess, 0 );
		CloseHandle( KinectFWHS.ProcessInfo.hProcess ); 
		CloseHandle( KinectFWHS.ProcessInfo.hThread ); 
	#endif
}

bool LaunchKinectApp()
{
#if (AUTO_LAUNCH_KINECT_APP == 1)
	int SecondsToWait = 240;
    //size_t iMyCounter = 0, iReturnVal = 0, iPos = 0; 
    //DWORD dwExitCode = 0; 
    //std::wstring sTempStr = L""; 
	// If need to pass parameters, see example at: http://www.goffconcepts.com/techarticles/development/cpp/createprocess.html
	// http://msdn.microsoft.com/en-us/library/ms682425(VS.85).aspx

    /* CreateProcess API initialization */ 
    STARTUPINFO StartupInfo; 
    PROCESS_INFORMATION ProcessInfo; 
    memset(&StartupInfo, 0, sizeof(StartupInfo)); 
    memset(&ProcessInfo, 0, sizeof(ProcessInfo)); 
    StartupInfo.cb = sizeof(StartupInfo); 
    memset(&KinectFWHS, 0, sizeof(KinectFWHS)); 

	ROBOT_LOG( TRUE, "\n ==================== Starting Kinect App ====================\n" )

	if( !CreateProcess(
		//"C:\\Dev\\Robots\\RobotKinectViewer\\bin\\Debug\\RobotKinectViewer.exe",	//  __in_opt     LPCTSTR lpApplicationName,
		"C:\\Dev\\Robots\\RobotKinectViewer\\bin\\Release\\RobotKinectViewer.exe",	//  __in_opt     LPCTSTR lpApplicationName,
		"",											//  __inout_opt  LPTSTR lpCommandLine,
		0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpProcessAttributes,
		0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpThreadAttributes,
		false,										//  __in         BOOL bInheritHandles,
		CREATE_DEFAULT_ERROR_MODE,					//  __in         DWORD dwCreationFlags,
		0,											//  __in_opt     LPVOID lpEnvironment,
		//"C:\\Dev\\Robots\\RobotKinectViewer\\bin\\Debug",	 //  __in_opt     LPCTSTR lpCurrentDirectory,
		"C:\\Dev\\Robots\\RobotKinectViewer\\bin\\Release",	 //  __in_opt     LPCTSTR lpCurrentDirectory,
		&StartupInfo,								// __in         LPSTARTUPINFO lpStartupInfo,
		&(KinectFWHS.ProcessInfo) )		//  __out        LPPROCESS_INFORMATION lpProcessInformation
	)
    { 
        /* CreateProcess failed */ 
		ROBOT_DISPLAY( TRUE, "ERROR: KINECT PROCESS LAUNCH FAILED!  Return Code = %04X", GetLastError() )
		AfxMessageBox( _T("ERROR: KINECT PROCESS LAUNCH FAILED! (Thread.cpp)") );
		return false;
    } 

	// Allow the C# process to get started a bit.  this is not critical...
	Sleep(100); 

	// Now, Create Thread for talking to the C# app
	//g_hKinectAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, KinectSpeechThreadProc, (LPVOID)0, 0, &g_dwKinectAppSharedMemoryIPCThreadId );
	//ROBOT_LOG( TRUE,  "Created Kinect App IPC Thread. ID = (0x%x) (KinectSpeechThreadProc)", g_dwKinectAppSharedMemoryIPCThreadId )



/*
	// Wait until child process has created a window
	ROBOT_LOG( TRUE,  "WAITING FOR PROCESS WINDOW...\n" )
	DWORD Result = WaitForInputIdle(
		ProcessInfo.hProcess,	// __in  HANDLE hProcess,
		(DWORD)30000 );					// __in  DWORD dwMilliseconds

	if( 0 == Result )
	{
		ROBOT_LOG( TRUE,  "DONE\n" )
	}
	else
	{
		ROBOT_LOG( TRUE,  "TIMED OUT!\n" )
	}

	// Try PostThreadMessage or PostMessage
	KinectFWHS.hWndFound  = NULL;

	// Enumerate all top level windows on the desktop, and find the itunes one
	EnumWindows ( EnumWindowCallBack, (LPARAM)&KinectFWHS ) ;
	*/

//	SendMessage ( KinectFWHS.hWndFound, Msg, wParam, lParam );

#else
	g_KinectSubSystemStatus = SUBSYSTEM_DISABLED;
#endif  // #if (AUTO_LAUNCH_KINECT_APP == 1)

	return true;

}


//-----------------------------------------------------------------------------
// Name: KinectSpeechThreadProc
// This thread reads data from the Kinect C# application via shared memory, and sends it to handler threads 
//-----------------------------------------------------------------------------
#define DEBUG_KINECT_IPC_SHARED_MEMORY 1

#define MESSAGE_QUEUE_MAX_MESSAGES			4
//#define SPEECH_INPUT_PHRASE_MAX_LENGTH	   128
//#define SPEECH_INPUT_QUEUE_MAX_MESSAGES		16

enum MESSAGE_QUEUE_FLAGS
{
    MESSAGE_QUEUE_OPEN_SLOT = 0,
    MESSAGE_QUEUE_WRITE_IN_PROGRESS,
    MESSAGE_QUEUE_MESSAGE_READY,
};


typedef struct
{
	int	Flag;	// Flag if there is a message ready in this slot of the queue
	int	RecoType; //SpeechRecoItem;	// WM_
	int	Param1;
	int	Param2;
	int	Param3;
	int	Param4;
	float	SpeechRecoConfidence;
	float	AudioBeamDirection;
} KINECT_IPC_MESSAGE_QUEUE_ITEM_T; // Must match struct in Managed code!

typedef struct
{
	int	  TrackPlayerDisableRequestFlag; // OUT:  Tell C# code to disable display of players
	int	  SpeechRecoDisableRequestFlag; // OUT: Tell C# code to disable speech recognition
	KINECT_IPC_MESSAGE_QUEUE_ITEM_T MessageQueueFromApp[MESSAGE_QUEUE_MAX_MESSAGES];	// IN - From Managed App
} KINECT_IPC_MAPPED_DATA_T; // Must match struct in Managed code!



DWORD WINAPI KinectSpeechThreadProc( LPVOID NotUsed ) // g_dwKinectAppSharedMemoryIPCThreadId
{
	__itt_thread_set_name( "SharedMemoryIPC Thread" );

	//	MSG	msg;
	//    HRESULT hr = E_FAIL;
	//	CString input, ResponseString, MsgString;
	ROBOT_LOG( TRUE,  "KinectSpeechThreadProc started.\n" )

	#if (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL) 
		ROBOT_LOG( TRUE,  "Waiting for Kinect hardware power up...\n" )
		Sleep(5000); // wait for Kinect to power up
		Sleep(5000); // wait for Kinect to power up
	#endif
	ROBOT_LOG( TRUE,  "Launching Kinect C# App...\n" )
	LaunchKinectApp();


	// Give a chance for the rest of the Robot threads to start up
	while( SUBSYSTEM_CONNECTED != g_KinectSubSystemStatus )
	{
		// wait for the Kinect to finish initializing
		RobotSleep(1000, pDomainControlThread); // allow other threads to start first
	}

	CRobotSpeechReco SpeechReco;	// Robot Speech Recognition Class
	SpeechReco.Init();				// Send Initialization commands

    // Create an event for synchronizing between the C# and C++ apps
	ROBOT_LOG( TRUE,  "Waiting for C# App to start...\n" )
	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hSpeechRecoEvent = CreateEvent ( NULL, bManualReset, bInitialState, "LokiRobotSpeechRecoEvent" );
	if ( !g_hSpeechRecoEvent ) 
	{ 
		ROBOT_LOG( TRUE,  "Event creation failed!:  LokiRobotSpeechRecoEvent\n" )
		return 0; // exit thread
	}


	// Now wait for the event to be signaled by the C# process, indicating that it's ok to proceed
	const DWORD msTimeOut = 10000;
	DWORD dwResult = WaitForSingleObject(g_hSpeechRecoEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		ROBOT_LOG( TRUE,  "Event Timed out or failed!:  LokiRobotSpeechRecoEvent, Error: %08X\n", dwResult )
		return 0; // exit thread
	}
	ROBOT_LOG( TRUE,  "C# App start Success!\n" )


	// Open Memory Mapped File
	TCHAR szKinectSpeechInterfaceSharedFileName[]=TEXT(KINECT_SPEECH_INTERFACE_SHARED_FILE_NAME);
	char *pBuf;
//	int NextQueuedMessageFromApp = 0;	// Keep track of queue position

	HANDLE hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		szKinectSpeechInterfaceSharedFileName);	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		//ROBOT_ASSERT(0);
		TRACE("/n/n");
		ROBOT_LOG( TRUE, "***********************************************************************" )
		ROBOT_LOG( TRUE,  "Could not open file mapping object (%d).\n", GetLastError())
		ROBOT_LOG( TRUE, "******** FAILED TO OPEN IPC SHARED MEMORY FROM C# APPLICATION! ********" )
		ROBOT_LOG( TRUE, "***********************************************************************\n" )
		return 0; // no sense wasting cycles, the MMF did not init correctly!
	}
	else
	{
		pBuf = (char*)MapViewOfFile(hMapFile, // handle to map object (LPCTSTR)
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,                    
			0,                    
			(sizeof(KINECT_IPC_MAPPED_DATA_T)) );                   

		if (pBuf == NULL) 
		{ 
			ROBOT_LOG( TRUE,  "Could not map view of file (%d).\n", GetLastError());
			CloseHandle(hMapFile);
			return 0; // exit thread
		}
		else
		{
			ROBOT_DISPLAY( TRUE, "Kinect Speech Shared Memory File Opened Sucessfully!" )
		} 
	}


	///////////////////////////////////////////////////////////////////////////////////////
	// Loop forever
//	int RecoType = 0; int Param1 = 0; int Param2 = 0; int Param3 = 0; int Param4 = 0;
//	float SpeechRecoConfidence = 0.0;


	ROBOT_LOG( TRUE,"Thread Loop starting.\n")
	while( g_bRunThread && (WAIT_OBJECT_0 == WaitForSingleObject(g_hSpeechRecoEvent, INFINITE)) )
	{
		ROBOT_LOG( DEBUG_KINECT_SHARED_MEMORY,  "---------------------- g_hSpeechRecoEvent Signaled ---------------------\n");

		// Read from Shared Memory
		KINECT_IPC_MAPPED_DATA_T *pIpcData = (KINECT_IPC_MAPPED_DATA_T*)pBuf;

		for( int NextQueuedMessageFromApp = 0; NextQueuedMessageFromApp <MESSAGE_QUEUE_MAX_MESSAGES; NextQueuedMessageFromApp++ )
		{
			if( MESSAGE_QUEUE_MESSAGE_READY == pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Flag )
			{
				RECO_TYPE RecoType = (RECO_TYPE)(pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].RecoType);
				int Param1 = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Param1;
				int Param2 = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Param2;
				int Param3 = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Param3;
				int Param4 = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Param4;
				float SpeechRecoConfidence = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].SpeechRecoConfidence;
				g_LastHumanAudioBeamDirection = pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].AudioBeamDirection;

				// DEBUG
				ROBOT_LOG( DEBUG_KINECT_SHARED_MEMORY, " MESSAGE_QUEUE_MESSAGE_READY in Slot %d: RecoType=%4d, Param1=%4d, SpRecoConf=%3.2f, AudioBeamDir=%3.2f\n", 
						NextQueuedMessageFromApp, (int)RecoType, Param1, SpeechRecoConfidence, g_LastHumanAudioBeamDirection );

				// Message ready for processing
				SpeechReco.HandleRecognition( RecoType, Param1, Param2, Param3, Param4, SpeechRecoConfidence );

	/*** TODO-MUST

				// Now, tell the robot to look at whoever is speaking, unless the command was a "move head" command
				if( (RecoType != Command) )
				{
					TRACE("");
				}				

				if( SpeechCmd_LookLeft != SpeechRecoItem )
				{
					int PanPositionTenthDegrees = (int)(AudioBeamDirection * -10.0); // convert to tenth degrees (and negative to swap Left/Right)
					//SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)(PanPositionTenthDegrees), (DWORD)SERVO_SPEED_MED );
					SendCommand( WM_ROBOT_CAMERA_LOOK_AT_SOUND_CMD, (DWORD)(PanPositionTenthDegrees), (DWORD)SERVO_SPEED_MED );
					
					//SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)(KINECT_HUMAN_DETECT_START_POSITION), 0 ); // TODO - change this to use info from Kinect on person's head location
				}
	***/
				// Clear the message slot
				pIpcData->MessageQueueFromApp[NextQueuedMessageFromApp].Flag = MESSAGE_QUEUE_OPEN_SLOT;
			}
		}
	}

	// exit when g_bRunThread flag cleared
	ROBOT_LOG( TRUE,"KinectSpeechThreadProc g_bRunThread shutdown. Terminating Kinect App...\n")
	TerminateKinectApp();
	ROBOT_LOG( TRUE,"KinectSpeechThreadProc exiting.\n")

	return 0;

}	// End of Kinect App IPC Thread



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread to communicate with Camera App, which runs OpenCV in non-debug mode for speed
#define DEBUG_CAMERA_SHARED_MEMORY 1
DWORD WINAPI CameraAppSharedMemoryIPCThreadProc( LPVOID NotUsed )
{
	__itt_thread_set_name( "CameraSharedMemoryIPC Thread" );

	//	MSG	msg;
	//    HRESULT hr = E_FAIL;
	//	CString input, ResponseString, MsgString;

	// Give a chance for the rest of the Robot threads to start up
//	while( SUBSYSTEM_CONNECTED != g_CameraSubSystemStatus )
	{
		// wait for the Kinect to finish initializing
		RobotSleep(1000, pDomainControlThread); // allow other threads to start first
	}


//	CRobotSpeechReco SpeechReco;	// Robot Speech Recognition Class
//	SpeechReco.Init();				// Send Initialization commands

    // Create an event for synchronizing with the Camera App
	ROBOT_LOG( TRUE,  "Waiting for Camera App to start...\n" )
	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hCameraUpdateEvent = CreateEvent ( NULL, bManualReset, bInitialState, CAMERA_UPDATE_EVENT_NAME );
	if ( !g_hCameraUpdateEvent ) 
	{ 
		ROBOT_LOG( TRUE,  "Event creation failed!:  %s\n", CAMERA_UPDATE_EVENT_NAME )
		return 0; // exit thread
	}


	// Now wait for the event to be signaled by the Camera process, indicating that it's ok to proceed
	const DWORD msTimeOut = 10000;
	DWORD dwResult = WaitForSingleObject(g_hCameraUpdateEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		if ( NULL == g_pCameraRequestSharedMemory )
		{
			ROBOT_LOG( TRUE,  "Camera App probably not running (g_pCameraRequestSharedMemory = NULL).  exiting IPC Thread\n" )
		}
		else
		{
			ROBOT_LOG( TRUE,  "Event Timed out or failed!:  %s, Error: %08X\n", CAMERA_UPDATE_EVENT_NAME, dwResult )
		}
		g_CameraSubSystemStatus = SUBSYSTEM_DISABLED;
		return 0; // exit thread
	}
	ROBOT_LOG( TRUE,  "Camera App start Success!\n" )


	// Open Memory Mapped File
	//TCHAR szCameraInterfaceSharedFileName[]=TEXT(CAMERA_SHARED_FILE_NAME);
	char *pBuf;
//	int NextQueuedMessageFromApp = 0;	// Keep track of queue position

	HANDLE hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		_T(CAMERA_DATA_SHARED_FILE_NAME) );	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		//ROBOT_ASSERT(0);
		TRACE("/n/n");
		ROBOT_LOG( TRUE, "***********************************************************************" )
		ROBOT_LOG( TRUE,  "Could not open Camera file mapping object (%d).\n", GetLastError())
		ROBOT_LOG( TRUE, "******** FAILED TO OPEN IPC SHARED MEMORY FROM C# APPLICATION! ********" )
		ROBOT_LOG( TRUE, "***********************************************************************\n" )
		return 0; // no sense wasting cycles, the MMF did not init correctly!
	}
	else
	{
		pBuf = (char*)MapViewOfFile(hMapFile, // handle to map object (LPCTSTR)
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,                    
			0,                    
			(sizeof(CAMERA_UPDATE_T)) );                   

		if (pBuf == NULL) 
		{ 
			ROBOT_LOG( TRUE,  "Could not map view of file (%d).\n", GetLastError());
			CloseHandle(hMapFile);
			return 0; // exit thread
		}
		else
		{
			ROBOT_DISPLAY( TRUE, "IPC Shared Memory File Opened Sucessfully!" )
		} 
	}


	///////////////////////////////////////////////////////////////////////////////////////
	// Loop forever
	ROBOT_LOG( TRUE,"Thread Loop starting.\n")
	while( g_bRunThread && (WAIT_OBJECT_0 == WaitForSingleObject(g_hCameraUpdateEvent, INFINITE)) )
	{
		ROBOT_LOG( DEBUG_CAMERA_SHARED_MEMORY,  "---------------------- g_hCameraUpdateEvent Signaled ---------------------\n");

		// Read from Shared Memory
		CAMERA_UPDATE_T CameraUpdate;
		CopyMemory(&CameraUpdate, pBuf, (sizeof(CAMERA_UPDATE_T)));

		if( CAMERA_UPDATE_FACE_RECO == CameraUpdate.UpdateType )
		{

			if( 0 != CameraUpdate.UpdateData.FaceUpdate.FaceDetected )
			{
				ROBOT_LOG( DEBUG_CAMERA_SHARED_MEMORY,  "Face Detected\n");
			}
			if( 0 != CameraUpdate.UpdateData.FaceUpdate.PersonRecognized )
			{
				ROBOT_LOG( DEBUG_CAMERA_SHARED_MEMORY,  "Person Recognized: ID: %d, Name: %s\n", 
					CameraUpdate.UpdateData.FaceUpdate.PersonID, CameraUpdate.UpdateData.FaceUpdate.PersonName );
			}

		}
		else if( CAMERA_UPDATE_OBJECT_RECO == CameraUpdate.UpdateType )
		{
			// TODO - unpack parameters and pass to handler

			DWORD bObjectRecognized = (DWORD)CameraUpdate.UpdateData.ObjectUpdate.bObjectRecognized;
			DWORD ObjectID = (DWORD)CameraUpdate.UpdateData.ObjectUpdate.ObjectID;
			g_ObjectName = CameraUpdate.UpdateData.ObjectUpdate.ObjectName; // copy name to global string

			if( bObjectRecognized )
			{
				ROBOT_LOG( DEBUG_CAMERA_SHARED_MEMORY,  "Object not Recognized\n" )
			}
			else
			{
				ROBOT_LOG( DEBUG_CAMERA_SHARED_MEMORY,  "Object Recognized: ID: %d, Name: %s\n", ObjectID, g_ObjectName )
			}

			SendCommand( WM_ROBOT_CAMERA_MATCH_COMPLETE, bObjectRecognized, ObjectID );
		}
		else
		{
			ROBOT_LOG( TRUE,  "ERROR! Unhandled UpdateType!\n");
			ROBOT_ASSERT(0);
		}
	}
	// exit when g_bRunThread flag cleared
	ROBOT_LOG( TRUE,"SharedMemoryIPC exiting.\n")

	return 0;

}	// End of Camera OpenCV App IPC Thread





#endif	// ROBOT_SERVER