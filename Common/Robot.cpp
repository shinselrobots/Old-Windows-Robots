// Robot.cpp : Defines the class behaviors for the application.
//
#include "stdafx.h"
#include "ClientOrServer.h"
#include "RobotType.h"


#if ( ROBOT_SERVER == 1 )
	#include "HardwareCmds.h"
	#include "HWInterfaceParams.h"
	#include "HWInterface.h"
#else
	#include "RobotClientSock.h"
#endif

#include "Globals.h"
#include "Module.h"

#include "resource.h"
#include "Robot.h"

//#include <winsock.h>
//#include <rapi.h>
//#include <basetsd.h>
#include "Thread.h"
///#include "Joystick.h"
#include "RadarDisplayWnd.h"
#include "LaserDisplayWnd.h"
#include "HtmlCtrl.h"

#if ( ROBOT_SERVER != 1 )
#include "RobotClientSock.h"
#endif

#include "RobotCmdDoc.h"
#include "RobotCmdView.h"

#include "MainFrm.h"
#include "ChildFrm.h"
#include "MapDoc.h"
#include "PathDoc.h"
#include "RobotCmdDoc.h"
#include "SetupDoc.h"

#include "MapView.h"
#include "PathView.h"
#include "RobotCmdView.h"
#include "SetupView.h"
#include "SetupDoc.h"

#include <iostream>

using namespace std;


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

static char *LogFileName = "c:\\temp\\RobotLog.txt";
///static char *LogFileName = "RobotLog.txt";

// Use the following to debug memory leaks
#define DEBUG_MEMORY_LEAKS			1

#if (DEBUG_MEMORY_LEAKS == 1)
	#ifndef _CRTDBG_MAP_ALLOC 
		#define _CRTDBG_MAP_ALLOC_NEW
		#define _CRTDBG_MAP_ALLOC
	#endif
	#include <stdlib.h>
	#include <crtdbg.h>
#endif


/////////////////////////////////////////////////////////////////////////////
// CRobotApp

BEGIN_MESSAGE_MAP(CRobotApp, CWinApp)
	//{{AFX_MSG_MAP(CRobotApp)
	ON_COMMAND(ID_APP_ABOUT, OnAppAbout)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
	// Standard file based document commands
	ON_COMMAND(ID_FILE_NEW, CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, CWinApp::OnFileOpen)
	// Standard print setup command
	ON_COMMAND(ID_FILE_PRINT_SETUP, CWinApp::OnFilePrintSetup)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRobotApp construction

CRobotApp::CRobotApp()
{
	// Place all significant initialization in InitInstance

}

/////////////////////////////////////////////////////////////////////////////
// The one and only CRobotApp object

CRobotApp theApp;

/////////////////////////////////////////////////////////////////////////////
// CRobotApp initialization

BOOL CRobotApp::InitInstance()
{
	// //TAL_SCOPED_TASK();	
	TRACE( "\n\nRobot.cpp: Robot InitInstance: ============= ORDER  ==========\n" );

	#if (DEBUG_MEMORY_LEAKS == 1 )
		_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	// If non-debug, open log file
	//#ifndef _DEBUG
		errno_t nError = fopen_s( &g_LogFile, LogFileName, "w" );
		if( 0 == nError )
		{
			fprintf(g_LogFile, "STARTING LOG FILE.  Start Time: %s\n\n", COleDateTime::GetCurrentTime().Format() );
		}
		else
		{
			g_LogFile = NULL;
			CString ErrorStr;
			ErrorStr.Format( "Can't open Log File\n %s Error = %d", LogFileName, nError );
			AfxMessageBox( ErrorStr );
		}
	//#endif


	__itt_thread_set_name( "GUI Thread" );


	// Use to detect memory leaks!
	// _CrtSetBreakAlloc( Put allocator number here);
	// _CrtSetBreakAlloc( 377 );
	// _CrtSetBreakAlloc( 0x724b0000 );




	// MOVED FROM SetupView.cpp to here, needed to open files MRU
  // PUT THIS IN WINMAIN???

	//Windows::Foundation::Initialize
    ::CoInitializeEx(NULL,COINIT_MULTITHREADED);

	/////////////////////////////////////////////////////////////////////////
	// ALLOCATE AND INITIALIZE ALL GLOBALS HERE!
	/////////////////////////////////////////////////////////////////////////
	g_CriticalSectionsInitialized = TRUE;
	InitializeCriticalSection( &g_csSpeakQueue );				// for sync access to Speech queue
	InitializeCriticalSection( &g_csDisplayLock );				// for sync access to Status buffers
	InitializeCriticalSection( &g_csServoLock );				// for sync access to Servo Control buffer
	InitializeCriticalSection( &g_csKinectPointCloudLock );		// for sync access to Kinect Point Cloud buffer
	InitializeCriticalSection( &g_csKinectSummaryDataLock );	// for sync access to Kinect summary data buffer
	InitializeCriticalSection( &g_csLaserSummaryDataLock );		// for sync access to Laser summary data buffer
	InitializeCriticalSection( &g_csLaserDataLock );			// for sync access to Laser data buffer array
	InitializeCriticalSection( &g_csKinectHumanTrackingLock );	// for sync access to array of detected Humans
	g_CriticalSectionsInitialized = TRUE;
	g_MoveArmsWhileSpeaking = FALSE;
	g_CurrentlySpeaking = FALSE;
	g_PhraseDoneTokenProcessed = FALSE;


	// Tell display not to blank (assumes PC is set to blank the display quickly, if this has not been called)
///	SetThreadExecutionState(ES_CONTINUOUS | ES_DISPLAY_REQUIRED);
	// for Win7, use this instead?
	// HANDLE hPowerRequest = PowerCreateRequest(  __in  PREASON_CONTEXT Context ); // just a T String _T("Keep Alive")
	// BOOL PowerSetRequest(  __in  HANDLE PowerRequest,  __in  POWER_REQUEST_TYPE RequestType);
	// BOOL PowerClearRequest(  __in  HANDLE PowerRequest,  __in  POWER_REQUEST_TYPE RequestType);
	// CloseHandle( hPowerRequest );


	///////////////////////////////////////////////////////////
	// Memory Allocations 
	// Must have matching delete in ExitInstance - see "Release allocated robot data" at the end of ExitInstance

	g_pGPSData = new GPS_MESSAGE_T;
	memset( g_pGPSData, 0x00, sizeof( GPS_MESSAGE_T ) );

	g_pFullSensorStatus = new FullSensorStatus;	
	// Class automatically initializes

	g_pNavSensorSummary = new NavSensorSummary;
	// Class automatically initializes

	g_pKinectObjects2D = new OBJECT_2D_ARRAY_T;
	memset( g_pKinectObjects2D, 0x00, sizeof( OBJECT_2D_ARRAY_T ) );

	g_pKinectObjects3D = new OBJECT_3D_ARRAY_T;
	memset( g_pKinectObjects3D, 0x00, sizeof( OBJECT_3D_ARRAY_T ) );

	g_pLaserScannerData = new LASER_SCANNER_DATA_T;
	memset( g_pLaserScannerData, 0x00, sizeof( LASER_SCANNER_DATA_T ) );

	g_pIRobotStatus = new IROBOT_STATUS_T;
	memset( g_pIRobotStatus, 0x00, sizeof( IROBOT_STATUS_T ) );

	g_pKobukiStatus = new KOBUKI_STATUS_T;
	memset( g_pKobukiStatus, 0x00, sizeof( KOBUKI_STATUS_T ) );

	g_pGridMap = NULL; // Initialized in MapDoc: new CGridMap;
//	g_pObjectKnowledge = NULL; // Initialized by camera detector

	g_pLaserSummary = new SCANNER_SUMMARY_T;
	memset( g_pLaserSummary, 0x00, sizeof( SCANNER_SUMMARY_T ) );
	InitScannerSummaryData( g_pLaserSummary );

	g_pKinectSummary = new SCANNER_SUMMARY_T;
	memset( g_pKinectSummary, 0x00, sizeof( SCANNER_SUMMARY_T ) );
	InitScannerSummaryData( g_pKinectSummary );

	///////////////////////////////////////////////////////////
	//Intialize Gobal Structures (that are not automatically initialized in globals.cpp)
	if (!AfxSocketInit())
	{
		AfxMessageBox(IDP_SOCKETS_INIT_FAILED);
		return FALSE;
	}

	g_StatusMessagesToDisplay.Empty();	// CString buffer used for display of Status
	g_StatusMessagesToSend.Empty();		// CString buffer used for Sending Status to Client

	// Initialize human location tracking
	memset( g_HumanLocationTracking, 0x00, (sizeof(KINECT_HUMAN_TRACKING_T) * KINECT_MAX_HUMANS_TO_TRACK) );


#if ( ROBOT_SERVER == 1 )
	g_pFullSensorStatus->CurrentLocation.x = DEFAULT_ROBOT_START_POSITION_X;
	g_pFullSensorStatus->CurrentLocation.y = DEFAULT_ROBOT_START_POSITION_Y;
	g_pFullSensorStatus->CurrentLocationMotor.x = DEFAULT_ROBOT_START_POSITION_X;
	g_pFullSensorStatus->CurrentLocationMotor.y = DEFAULT_ROBOT_START_POSITION_Y;
#else
	g_pFullSensorStatus->CurrentLocation.x = 0;	// Don't show the robot on client until connected
	g_pFullSensorStatus->CurrentLocation.y = 0;
	g_pFullSensorStatus->CurrentLocationMotor.x = 0;
	g_pFullSensorStatus->CurrentLocationMotor.y = 0;
#endif

	// Initialize g_ServerSockStruct: SERVER_SOCKET_STRUCT
	g_ServerSockStruct.hReceiveThread = INVALID_HANDLE_VALUE;
	g_ServerSockStruct.hSendThread = INVALID_HANDLE_VALUE;
	g_ServerSockStruct.sockConnected = INVALID_SOCKET;

	// Initialize Socket structure
	g_ClientSockStruct.sock = INVALID_SOCKET;
	g_ClientSockStruct.szIPAddress[0] = 0;
	g_ClientSockStruct.hDlgWnd = NULL;
	g_ClientSockStruct.hClientSockSendThread = INVALID_HANDLE_VALUE;
	g_ClientSockStruct.hClientSockReceiveThread = INVALID_HANDLE_VALUE;

	// There is some code that assumes that an WPARAM is 32 bits.  Make sure.
	ASSERT(4 == sizeof(WPARAM) );
	ASSERT(4 == sizeof(LPARAM) );
	ASSERT(4 == sizeof(int) );
	ASSERT(4 == sizeof(long) );


	////////////////////////////////////////////////////////////////////////////
	// Launch the KobukiControl Application here for Kobuki robots
	// Launch the C# Kinect Capture Application here for Kobuki robots
	////////////////////////////////////////////////////////////////////////////
	#if (MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL) 
		LaunchKobukiApp();	// Will only launch if enabled in globals.cpp
		//Sleep(1000);		// Allow Kobuki to power up Kinect!
		// DELAY THIS TO ALLOW KINECT TO POWER UP FROM KOBUKI : LaunchKinectApp();	// Will only launch if enabled in globals.cpp.  
	#endif

	////////////////////////////////////////////////////////////////////////////
	// Launch the C# Kinect Capture Application here for Loki
	// Launch the OpenCV Camera Capture Application here for Loki
	////////////////////////////////////////////////////////////////////////////
	#if (ROBOT_TYPE == LOKI) 
		//LaunchKinectApp(); // Will only launch if enabled in globals.cpp.  - MOVED TO auto start with thread
		LaunchCameraApp(); // Will only launch if enabled in globals.cpp
	#endif



	///////////////////////////////////////////////////////////
	// UNCOMMENT THIS TO WAIT FOR REMOTE DEBUGGER TO ATTACH!
//	ROBOT_ASSERT(0); // Wait for Debugger to attach
	///////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////
	// Create Threads
	DWORD dwTempThreadId;


#if ( ROBOT_SERVER == 1 )

	// Create sound thread
	g_hSoundThread = ::CreateThread( NULL, 0, SoundThreadProc, (LPVOID)0, 0, &g_dwSoundThreadId );
	ROBOT_LOG( TRUE,  "Created Robot Sound Thread. ID = (0x%x)", g_dwSoundThreadId )

	// Create Speech thread
	g_hSpeakThread = ::CreateThread( NULL, 0, SpeakThreadProc, (LPVOID)0, 0, &g_dwSpeakThreadId );
	ROBOT_LOG( TRUE,  "Created Robot Speak Thread. ID = (0x%x)", g_dwSpeakThreadId )

	// Create robot control thread
	g_hControlThread = ::CreateThread( NULL, 0, ControlThreadProc, (LPVOID)0, 0, &g_dwControlThreadId );
	ROBOT_LOG( TRUE,  "Created Robot Control Thread. ID = (0x%x) (ControlThreadProc)", g_dwControlThreadId )

	// Create C# managed shared memory thread
	g_hKinectAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, KinectSpeechThreadProc, (LPVOID)0, 0, &g_dwKinectAppSharedMemoryIPCThreadId );
	ROBOT_LOG( TRUE,  "Created Kinect App IPC Thread. ID = (0x%x) (KinectSpeechThreadProc)", g_dwKinectAppSharedMemoryIPCThreadId )


	// Create Camera App shared memory thread
	/* MOVED TO LaunchCameraApp
	g_hCameraAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, CameraAppSharedMemoryIPCThreadProc, (LPVOID)0, 0, &g_dwCameraAppSharedMemoryIPCThreadId );
	ROBOT_LOG( TRUE,  "Created Camera App IPC Thread. ID = (0x%x) (CameraAppSharedMemoryIPCThreadProc)", g_dwCameraAppSharedMemoryIPCThreadId )
	*/

	// Create Kobuki App shared memory thread
	/* MOVED to LaunchKobukiApp
	#if( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
		g_hKobukiAppSharedMemoryIPCThread = ::CreateThread( NULL, 0, KobukiAppSharedMemoryIPCThreadProc, (LPVOID)0, 0, &g_dwKobukiAppSharedMemoryIPCThreadId );
		ROBOT_LOG( TRUE,  "Created Kobuki App IPC Thread. ID = (0x%x) (KobukiAppSharedMemoryIPCThreadProc)", g_dwKobukiAppSharedMemoryIPCThreadId )
	#endif
	*/

	// Create the video capture thread - MOVED TO CAMERA MODULE CLASS
//	g_bRunVidCapThread = TRUE; // When FALSE, tells Vidcap thread to exit
//	g_hCameraVidCapThread = ::CreateThread( NULL, 0, VidCapThreadProc, (LPVOID)0, 0, &dwTempThreadId );
//	ROBOT_LOG( TRUE,  "Created VidCap Thread. ID = (0x%x)", dwTempThreadId )

	// Create the Kinect video capture thread - MOVED TO KINECT CLASS
//	g_bRunKinectThread = TRUE; // When FALSE, tells Vidcap thread to exit
//	g_hKinectThread = ::CreateThread( NULL, 0, KinectDepthThreadProc, (LPVOID)0, 0, &dwTempThreadId );
//	ROBOT_LOG( TRUE,  "Created Kinect Thread. ID = (0x%x)", dwTempThreadId )


#endif

	// Create robot Timer thread
	g_hTimerThread = ::CreateThread( NULL, 0, TimerThreadProc, (LPVOID)0, 0, &dwTempThreadId );
	ROBOT_LOG( TRUE,  "Created Timer Thread. ID = (0x%x)", dwTempThreadId )



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// DOC/VIEW TEMPLATES 
	{	// BLOCK: doc template registration
		// Register the document template.  Document templates serve
		// as the connection between documents, frame windows and views.
		// Attach this form to another document or frame window by changing
		// the document or frame class in the constructor below.
		CMultiDocTemplate* pNewDocTemplate = new CMultiDocTemplate(
			IDR_ROBOTCMDVIEW_TMPL,
			RUNTIME_CLASS(CRobotCmdDoc),		// document class
			RUNTIME_CLASS(CChildFrame),		// custom MDI child frame
//			RUNTIME_CLASS(CMDIChildWnd),		// frame class
			RUNTIME_CLASS(CRobotCmdView));		// view class
		AddDocTemplate(pNewDocTemplate);
	}

	{	// BLOCK: doc template registration
		// Register the document template.  Document templates serve
		// as the connection between documents, frame windows and views.
		// Attach this form to another document or frame window by changing
		// the document or frame class in the constructor below.
		CMultiDocTemplate* pNewDocTemplate = new CMultiDocTemplate(
			IDR_SETUP_TMPL,
			RUNTIME_CLASS(SetupDoc),		// document class
			RUNTIME_CLASS(CMDIChildWnd),		// frame class
			RUNTIME_CLASS(Setup));		// view class
		AddDocTemplate(pNewDocTemplate);
	}

	{	// BLOCK: doc template registration
		// Register the document template.  Document templates serve
		// as the connection between documents, frame windows and views.
		// Attach this form to another document or frame window by changing
		// the document or frame class in the constructor below.
		CMultiDocTemplate* pNewDocTemplate = new CMultiDocTemplate(
			IDR_PATHVIEW_TMPL,
			RUNTIME_CLASS(CPathDoc),		// document class
			RUNTIME_CLASS(CChildFrame),		// custom MDI child frame
//			RUNTIME_CLASS(CMDIChildWnd),		// basic frame class
			RUNTIME_CLASS(CPathView));		// view class
		AddDocTemplate(pNewDocTemplate);
	}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


	AfxEnableControlContainer();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	//  of your final executable, you should remove from the following
	//  the specific initialization routines you do not need.

	/** - Removed per VS2008 warning
#ifdef _AFXDLL
	Enable3dControls();			// Call this when using MFC in a shared DLL
#else
	Enable3dControlsStatic();	// Call this when linking to MFC statically 
#endif
**/
	// Change the registry key under which our settings are stored.
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization.
	SetRegistryKey(_T("Shinsel Robot Applications"));

	LoadStdProfileSettings(8);  // Load standard INI file options (including MRU)

	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views.

	CMultiDocTemplate* pDocTemplate;
	pDocTemplate = new CMultiDocTemplate(
		IDR_MAPTYPE,
		RUNTIME_CLASS(CMapDoc),
		RUNTIME_CLASS(CChildFrame), // custom MDI child frame
		RUNTIME_CLASS(CMapView));
	AddDocTemplate(pDocTemplate);

	// create main MDI Frame window
	CMainFrame* pMainFrame = new CMainFrame;
	if (!pMainFrame->LoadFrame(IDR_MAINFRAME))
		return FALSE;
	m_pMainWnd = pMainFrame;

	// Enable drag/drop open
	m_pMainWnd->DragAcceptFiles();

	// Enable DDE Execute open
	EnableShellOpen();
	RegisterShellFileTypes(TRUE);

	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);

	// Cool Startup Behaviors!  Pick the one you want
	//cmdInfo.m_nShellCommand = CCommandLineInfo::FileNew;		// Default behavior, prompts for file type
	cmdInfo.m_nShellCommand = CCommandLineInfo::FileOpen;		// Open file upon invocation
	//cmdInfo.m_nShellCommand = CCommandLineInfo::FilePrint;	// Send straight to print dialog
	//cmdInfo.m_nShellCommand = CCommandLineInfo::FileNothing;	// Surpresses doc open at startup


	// Dispatch commands specified on the command line

	// Use fully qualified pathname, so we don't care where the exe is executed from
	// cmdInfo.m_strFileName = "..\\RobotData/Setup.rsd";		// File to open at startup!
	// cmdInfo.m_strFileName = "..\\RobotData\\Map.rmp";

	//cmdInfo.m_strFileName = "C:\\Dev\\Robots\\RobotData\\Setup.rsd";		// File to open at startup!

	cmdInfo.m_strFileName = ROBOT_DATA_PATH "\\Setup.rsd";
	if (!ProcessShellCommand(cmdInfo))
	{
		ROBOT_LOG( TRUE,  "\n\n========================================\n" )
		ROBOT_LOG( TRUE,  "ERROR! ERROR! Could not open Setup.rsd\n" )
		ROBOT_LOG( TRUE,  "========================================\n\n" )
		ROBOT_ASSERT(0);
		//return FALSE;
	}

	cmdInfo.m_strFileName = ROBOT_DATA_PATH "\\Command.rcd";		// File to open at startup!
	if (!ProcessShellCommand(cmdInfo))
	{
		ROBOT_LOG( TRUE,  "\n\n========================================\n" )
		ROBOT_LOG( TRUE,  "Could not open Command.rcd\n" )
		ROBOT_LOG( TRUE,  "========================================\n\n" )
		ROBOT_ASSERT(0);
		//return FALSE;
	}

	// m_nCmdShow = SW_SHOWMAXIMIZED;  // Launch App Maximized!



	// The main window has been initialized, so show and update it.
	pMainFrame->ShowWindow(m_nCmdShow);
	pMainFrame->UpdateWindow();

	return TRUE;
}


/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
		// No message handlers
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

// App command to run the dialog
void CRobotApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

/////////////////////////////////////////////////////////////////////////////
// CRobotApp message handlers


int CRobotApp::ExitInstance() 
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )
	::CoUninitialize();

	//TAL_SendTraces();

////////////////////////////////////////////////////////////////////
#if ( ROBOT_SERVER == 1 )
/*
	// TODO: These things cause a fault when attempted at shutdown
	// Turn off the camera
	//SendCommand( WM_ROBOT_CAMERA_POWER_CMD, (DWORD)POWER_ON, 0 );

	// Turn XP Wireless Zero Config back on
	//StartWirelessZeroConfig();

*/

	Sleep(1000);  // Make sure Servos get their move commands started ( see ~CRobotCmdView )

	// Shut down other threads while servos moving to final position.

	ROBOT_LOG( TRUE,  "Shutting down threads\n" )
	// Tell all the threads to exit, then check to see that they did 
	g_bRunThread = FALSE;	// Used for all threads that are not monitoring a message queue
	if( NULL != g_hSpeechRecoEvent ) SetEvent( g_hSpeechRecoEvent );	// signal the thread so it will check the global shut down flag
	if( NULL != g_hKinectDepthReadyEvent ) SetEvent( g_hKinectDepthReadyEvent );

	//  Note!  we don't use WM_QUIT, as it gets filtered out if we turn off "windowing" messages
	::PostThreadMessage( g_dwLaserScannerCommWriteThreadId,	WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwServoCommWriteThreadId,		WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwMotorCommThreadId,				WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwControlThreadId,				WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwSoundThreadId,					WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwSpeakThreadId,					WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwCameraVidCapThreadId,			WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );
	::PostThreadMessage( g_dwServerSendThreadId,			WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );	

	Sleep(1000); // Let arms get into final position before shutting shoulders off
	// Dyna servos are hanlded automatically by HW_SET_POWER_MODE

	// Turn off Shoulder Servos --- NOW HANDLED BY SERVO SLEEP AUTOMATICALLY
//	ROBOT_LOG( TRUE,  "Disabling Kerr Servo Motors\n" )
//	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable = FALSE;
//	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = TRUE;
//	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable = FALSE;
//	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = TRUE;
//	::PostThreadMessage( g_dwSmartServoCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_SERVO_TORQUE_ENABLE, 0, 0 );
//	Sleep(100);
	// Tell Arduino to go into reset mode - motors off, etc. -->> Changed for Loki! just turn off the Eyes!
	//ROBOT_LOG( TRUE,  "Resetting Arduino\n" )
	//::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_RESET_CPU, 0, 0 );

	::PostThreadMessage( g_dwSmartServoCommThreadId,			WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );

	/* handled by CCameraModule
	if( INVALID_HANDLE_VALUE != g_hCameraVidCapThread) 
		{
			ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Camera VidCap Thread to exit...\n" )
			WaitForSingleObject( g_hCameraVidCapThread, INFINITE );
			CloseHandle( g_hCameraVidCapThread );
		}
	*/

	/* handled by CKinectModule
	if( INVALID_HANDLE_VALUE != g_hKinectThread) 
		{
			ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Kinect Thread to exit...\n" )
			WaitForSingleObject( g_hKinectThread, INFINITE );
			CloseHandle( g_hKinectThread );
		}
	*/

	// Shutdown the robot Timer thread
	if( INVALID_HANDLE_VALUE != g_hTimerThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Timer Thread to exit...\n" )
		WaitForSingleObject( g_hTimerThread, 1000 );
		CloseHandle( g_hTimerThread );
	}
	
	// Shutdown robot control thread
	if( INVALID_HANDLE_VALUE != g_hControlThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Control Thread to exit...\n" )
		WaitForSingleObject( g_hControlThread, 2000 );
		CloseHandle( g_hControlThread );
	}

	if( INVALID_HANDLE_VALUE != g_hSoundThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Sound Thread to exit...\n" )
		WaitForSingleObject( g_hSoundThread, 1000 );
		CloseHandle( g_hSoundThread );
	}

	if( INVALID_HANDLE_VALUE != g_hSpeakThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Speak Thread to exit...\n" )
		WaitForSingleObject( g_hSpeakThread, 1000 );
		CloseHandle( g_hSpeakThread );
	}

	if( INVALID_HANDLE_VALUE != g_hArduinoReadThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hArduinoReadThread to exit...\n" )
		WaitForSingleObject( g_hArduinoReadThread, 1000 );
		CloseHandle( g_hArduinoReadThread );
	}
	
	if( INVALID_HANDLE_VALUE != g_hServoThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hServoThread to exit...\n" )
		WaitForSingleObject( g_hServoThread, 1000 );
		CloseHandle( g_hServoThread );
	}

	if( INVALID_HANDLE_VALUE != g_hMotorWriteThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hMotorWriteThread to exit...\n" )
		WaitForSingleObject( g_hMotorWriteThread, 1000 );
		CloseHandle( g_hMotorWriteThread );
	}

	if( INVALID_HANDLE_VALUE != g_hMotorReadThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hMotorReadThread to exit...\n" )
		WaitForSingleObject( g_hMotorReadThread, 1000 );
		CloseHandle( g_hMotorReadThread );
	}

	/*if( INVALID_HANDLE_VALUE != g_hTrexWriteThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hTrexWriteThread to exit...\n" )
		WaitForSingleObject( g_hTrexWriteThread, 1000 );
		CloseHandle( g_hTrexWriteThread );
	}*/

	if( INVALID_HANDLE_VALUE != g_hLaserScannerWriteThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hLaserScannerWriteThread to exit...\n" )
		WaitForSingleObject( g_hLaserScannerWriteThread, 1000 );
		CloseHandle( g_hLaserScannerWriteThread );
	}

	if( INVALID_HANDLE_VALUE != g_hLaserScannerReadThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hLaserScannerReadThread to exit...\n" )
		WaitForSingleObject( g_hLaserScannerReadThread, 1000 );
		CloseHandle( g_hLaserScannerReadThread );
	}

	if( INVALID_HANDLE_VALUE != g_hGPSThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hGPSThread to exit...\n" )
		WaitForSingleObject( g_hGPSThread, 1000 );
		CloseHandle( g_hGPSThread );
	}

	// Shutdown the robot socket threads
	if( INVALID_HANDLE_VALUE != g_ServerSockStruct.hReceiveThread )
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Server Socket Receive Thread to exit...\n" )
		WaitForSingleObject( g_ServerSockStruct.hReceiveThread, 500 ); 
		//Note: Socket "accept" blocks if no connection, so don't wait forever
	}

	if( INVALID_HANDLE_VALUE != g_ServerSockStruct.hSendThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Server Socket Send Thread to exit...\n" )
		WaitForSingleObject( g_ServerSockStruct.hSendThread, 1000 );
		CloseHandle( g_ServerSockStruct.hSendThread );
	}

	// Close COM Ports

	if( INVALID_HANDLE_VALUE != g_hLaserScannerCommPort ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow Laser Comm Thread to complete before releasing COMM Port...\n" )
		Sleep(100);	// Give time for the write thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Laser Comm Port...\n" )
		CloseCommPort( "Laser" , g_hLaserScannerCommPort );	// The thread shuts down automatically when the comm port closes
	}

	if( INVALID_HANDLE_VALUE != g_hMotorCommPort ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow Motor Comm Thread to complete before releasing COMM Port...\n" )
		Sleep(100);	// Give time for the write thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Motor Comm Port...\n" )
		CloseCommPort( "Motor", g_hMotorCommPort ); 	// The thread shuts down automatically when the comm port closes
	}
			

	/* Handled by Smart Servo Thread.

	if( INVALID_HANDLE_VALUE != g_hKerrServoCommPort ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow Kerr Comm Thread to complete before releasing COMM Port...\n" )
		Sleep(100);	// Give time for the ArduinoCommWrite thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Kerr Comm Port...\n" )
		CloseCommPort( "Kerr" , g_hKerrServoCommPort );	// The thread shuts down automatically when the comm port closes
	}

	if( INVALID_HANDLE_VALUE != g_hDynaServoCommPort_AX12 ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow Dynamixel AX12 Comm Thread to complete before releasing COMM Port...\n" )
		Sleep(100);	// Give time for the  thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Dynamexel AX12 Comm Port...\n" )
		CloseCommPort( "Dynamixel AX12" , g_hDynaServoCommPort_AX12 );	// The thread shuts down automatically when the comm port closes
	}

	if( INVALID_HANDLE_VALUE != g_hDynaServoCommPort_RX64 ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow Dynamixel RX64 Comm Thread to complete before releasing COMM Port...\n" )
		Sleep(100);	// Give time for the  thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Dynamexel RX64 Comm Port...\n" )
		CloseCommPort( "Dynamixel RX64" , g_hDynaServoCommPort_RX64 );	// The thread shuts down automatically when the comm port closes
	}
	*/

	if( INVALID_HANDLE_VALUE != g_hSmartServoThread ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Smart Servo Thread to exit...\n" )
		WaitForSingleObject( g_hSmartServoThread, 1000 );
		CloseHandle( g_hSmartServoThread );
	}

	// Shut down Arduino last, after Servo Power has been turned off by DynaControl!
	::PostThreadMessage( g_dwArduinoCommWriteThreadId,			WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );	// Quit after sending last message

	if( INVALID_HANDLE_VALUE != g_hArduinoWriteThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for g_hArduinoWriteThread to exit...\n" )
		WaitForSingleObject( g_hArduinoWriteThread, 1000 );
		CloseHandle( g_hArduinoWriteThread );
	}

	if( INVALID_HANDLE_VALUE != g_hArduinoCommPort ) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Sleeping to allow ArduinoCommWrite Thread to complete before releasing COMM Port...\n" )
		Sleep(1000);	// Give time for the ArduinoCommWrite thread to complete before releasing the Serial Port

		// Close the Serial Port
		ROBOT_LOG( TRUE,  "OnDestroy: Shutting down Arduino Comm Port...\n" )
		CloseCommPort( "Arduino" , g_hArduinoCommPort );	// The read thread shuts down automatically when the comm port closes
	}






////////////////////////////////////////////////////////////////////
#else	// ROBOTCLIENT code

	// Tell all the threads to exit, then check to see that they did 
	g_bRunThread = FALSE;
	::PostThreadMessage( g_dwClientSendThreadId, WM_ROBOT_THREAD_EXIT_CMD, 0, 0 );	


	// Shutdown client socket threads
	if( INVALID_HANDLE_VALUE != g_ClientSockStruct.hClientSockSendThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Client Socket Send Thread to exit...\n" )
		WaitForSingleObject( g_ClientSockStruct.hClientSockSendThread, INFINITE ); //(1000?)
		CloseHandle( g_ClientSockStruct.hClientSockSendThread );
	}

	if( INVALID_HANDLE_VALUE != g_ClientSockStruct.hClientSockReceiveThread) 
	{
		ROBOT_LOG( TRUE,  "OnDestroy: Waiting for Client Socket Receive Thread to exit...\n" )
		WaitForSingleObject( g_ClientSockStruct.hClientSockReceiveThread, 500 );
		//Note: Socket "accept" blocks if no connection, so don't wait forever
		CloseHandle( g_ClientSockStruct.hClientSockReceiveThread );
	}


#endif	// ROBOTCLIENT
////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////
	// Release allocated robot data
	// NOTE: Don't use ROBOT_LOG, we are deleting the logging mechanism!
	TRACE( "\n=========================================================================\n" );
	TRACE( "                       SHUT DOWN:  Releasing Global Objects in Robot.cpp \n" );

	// Tell display it can blank (assumes PC is set to blank the display quickly, once this been called)
	/// DAVES MOVED TO GLOBAL LOOOP: SetThreadExecutionState(ES_CONTINUOUS); // No extra flags means "stop requiring the display"

	SAFE_DELETE( g_pGPSData );
	SAFE_DELETE( g_pFullSensorStatus );
	SAFE_DELETE( g_pNavSensorSummary );
	SAFE_DELETE( g_pLaserSummary );
	SAFE_DELETE( g_pKinectSummary );	

	SAFE_DELETE( g_pKinectObjects2D );
	SAFE_DELETE( g_pKinectObjects3D );
	SAFE_DELETE( g_pLaserScannerData );
	SAFE_DELETE( g_pIRobotStatus );
	SAFE_DELETE( g_pKobukiStatus );
	
	

	TRACE( "                       SHUT DOWN:  Deleting Critical Section Locks \n" );
	g_CriticalSectionsInitialized = FALSE;					// Prevent CS use after this point	
	DeleteCriticalSection( &g_csSpeakQueue );				// for sync access to Speech queue
	DeleteCriticalSection( &g_csDisplayLock );				// for sync access to Status buffers
	DeleteCriticalSection( &g_csServoLock );				// for sync access to Servo Control buffer
	DeleteCriticalSection( &g_csKinectPointCloudLock );		// for sync access to Kinect Point Cloud buffer
	DeleteCriticalSection( &g_csKinectSummaryDataLock );	// for sync access to Kinect summary data buffer
	DeleteCriticalSection( &g_csLaserSummaryDataLock );		// for sync access to Laser summary data buffer
	DeleteCriticalSection( &g_csLaserDataLock );			// for sync access to Laser data buffer array
	DeleteCriticalSection( &g_csKinectHumanTrackingLock );	// for sync access to array of detected Humans

	Sleep(1);
	TRACE( "                       SHUT DOWN:  Complete \n" );
	TRACE( "===========================================================================\n\n" );

	if( NULL != g_LogFile )
	{
		fclose( g_LogFile );
		TRACE( " ***> View Log file at: %s\n\n", LogFileName );
		///system( "notepad c:\\temp\\RobotLog.txt" );
		///system( "type RobotLog.txt; pause" );
		///system( "pause" );
	}

#ifndef INTEL_NO_ITTNOTIFY_API
	TRACE( "===========================================================================\n" );
	TRACE( " WARNING! ITT ENABLED, WILL CAUSE MEMORY LEAKS. \n TO DISABLE, see INTEL_NO_ITTNOTIFY_API in Globals.h\n" );
	TRACE( "===========================================================================\n\n" );
#endif

	// NOT NEEDED - Handled automatically
	//TRACE("Dumping any Memory Leaks:\n");
	//_CrtDumpMemoryLeaks();

	return CWinApp::ExitInstance();
}
