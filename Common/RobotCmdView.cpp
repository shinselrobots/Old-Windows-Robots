// RobotCmdView.cpp : implementation file
//

#include "stdafx.h"
#include "ClientOrServer.h"
#include "Globals.h"
#include "resource.h"
#include "Robot.h"
//#include "ArmControlDlg.h"
#include "HardwareConfig.h"
//#include "..\Common\HardwareConfig.h"


#include <mmsystem.h>
//#include <dsound.h>
//#include <dmksctrl.h>
//#include <dmusici.h>
//#include <dmusicc.h>
//#include <dmusicf.h>

//#include <winsock.h>
//#include <rapi.h>
//#include <basetsd.h>
//#include "RobotSharedParams.h"
//#include "NMEAParser.h"
#include "Module.h"

#include "Thread.h"
///#include "Joystick.h"
#include "RadarDisplayWnd.h"
#include "LaserDisplayWnd.h"
#include "HtmlCtrl.h"

#if ( ROBOT_SERVER == 1 )
#include "HWInterface.h"
#else
#include "RobotClientSock.h"
#endif

//#include "cv.h"
//#include "cvcam.h"
//#include "highgui.h"

#include "RobotCmdDoc.h"
#include "RobotCmdView.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


/////////////////////////////////////////////////////////////////////////////
// CRobotCmdView

IMPLEMENT_DYNCREATE(CRobotCmdView, CFormView)

CRobotCmdView::CRobotCmdView()
	: CFormView(CRobotCmdView::IDD)
	, FaceCaptureName(_T(""))
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	//{{AFX_DATA_INIT(CRobotCmdView)
	m_UseJoystick = FALSE;
	m_BehaviorMode = _T("");
	m_MoveDistance = _T("");
	m_TextToSpeak = _T("");
	m_TurnAmount = _T("");
	m_SendTextToAI = FALSE;
	//}}AFX_DATA_INIT


	
// Used for both Client and Server

	// Initialize all data members
	//m_hIcon; // no need to initialize
	//m_pArmControlDlg = new ArmControlDlg(this);	// Modeless Dlg

	m_bTimerExpiredReported= FALSE;		// Form Data
	// m_RadarDisplay;	// Form Data
	m_CameraZoomSpeed = 5;
	m_CameraPanSpeed = 5;	// TODO!  Set this in the GUI!
	m_nMaxClientKeepAliveCount;
	m_nLineNum = 0;
	m_nSpeedSlider = SLIDER_CENTER;
	m_nTurnSlider = SLIDER_CENTER;
///	m_pDI = NULL;         
///	m_pJoystick = NULL;   
	m_GPSConnectionIndicator = FALSE;
	m_bTrackDllLoaded = FALSE;
	m_bTrackColorSearching = FALSE;
	m_nAvoidObjectRange = 48;	// inches
	
	// m_diDevCaps is initialized later
	// m_HtmlCtrl is initialzied later

	// Threads
//	m_hSocketThread = INVALID_HANDLE_VALUE;

	// See 	LASER_SCANNER_STATE_T; // used by g_LaserScannerState
	g_LaserScannerState.bReaderInitialized = FALSE;
	g_LaserScannerState.LastCmd[0] = 0;
	g_pLaserScannerData->NumberOfSamples = 0;

	m_RedBrush = new CBrush(BACKGROUND_COLOR_RED); // Red background
	m_YellowBrush = new CBrush(BACKGROUND_COLOR_YELLOW); // Yellow background
	m_GreenBrush = new CBrush(BACKGROUND_COLOR_GREEN); // Green background
	m_BlueBrush = new CBrush(BACKGROUND_COLOR_BLUE); // Blue background

	m_BatteryStatus = UNKNOWN;
	m_LocalUser = FALSE; // default to remote user level for control priority
}

CRobotCmdView::~CRobotCmdView()
{

// This is one of the first functions called on shutdown, so tell robot code to start shutting down

	ROBOT_LOG( TRUE,  "\n\n\n\n=========================================================================\n" )
	ROBOT_LOG( TRUE,  "~CRobotCmdView:                SHUT DOWN\n" )
	ROBOT_LOG( TRUE,  "===========================================================================\n\n" )
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	// TODO: These things cause a fault when attempted at shutdown
	// Turn off the camera
	//SendCommand( WM_ROBOT_CAMERA_POWER_CMD, (DWORD)POWER_ON, 0 );
	// Turn XP Wireless Zero Config back on
	//StartWirelessZeroConfig();

	// shut down vidcap. This tends to take a while, and crashes if it does not clean up correctly
//	ROBOT_LOG( TRUE,  "~CRobotCmdView Requesting VidCap shutdown...\n" )
//	g_bRunVidCapThread = FALSE;
	//g_bRunKinectThread = FALSE;

	// Move Head and arms to sleep position, and turn off servos
	ROBOT_LOG( TRUE,  "~CRobotCmdView Moving Head and Arms to Sleep Position...\n" )
		::PostThreadMessage( g_dwSmartServoCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_POWER_MODE, SYSTEM_SHUT_DOWN, 0 );

	// Tell Arduino to stop sending updates, so the COM port can drain out
	::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_GET_STATUS, 0, FALSE ); 	// Stop sending status

	::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_LED_EYES, LED_EYES_OFF, 0 ); 	// Turn off Eyes
	SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)FALSE ); // Turn lights off

	ROBOT_LOG( TRUE,  "~CRobotCmdView Stopping Wheel Motors...\n" )
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
			::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_MOTOR_STOP, 0, 0 );
			//::PostThreadMessage( g_dwMotorCommThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_KINECT_POWER, 0, 0 ); // Turn off Kinect Power
		#else
			ROBOT_ASSERT(0);
		#endif


	//turn off laser scanning, so the COM port input can drain out
	g_bLaserContinuousScanEnabled = FALSE;



	#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
		////////////////////////////////////////////////////////////////
		// KLUDGE - delete MS Kinect here for now - TODO-MUST
		ROBOT_LOG( TRUE,  "~CRobotCmdView Uninit Kinect Nui...\n" )

		CKinectNui *pKinectNui = (CKinectNui*)g_pKinectNui;
		pKinectNui->Nui_UnInit();
		SAFE_DELETE( pKinectNui );
		g_pKinectNui = INVALID_HANDLE_VALUE;
	#endif

	RobotSleep(100, pDomainGUIThread); // Get Servo commands started

	// Shut down the Kinect C# process, and OpenCV Camera capture process
	TerminateKinectApp();
	TerminateCameraApp();

	// delete allocated stuff
	//delete m_pArmControlDlg;
	delete m_RedBrush;
	delete m_GreenBrush;
	delete m_YellowBrush;
	delete m_BlueBrush;

}

// Overrides
void CRobotCmdView::OnInitialUpdate()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	ROBOT_LOG( TRUE,  "starting... \n" )
	CFormView::OnInitialUpdate();

	//  Resize the frame to match the dialog.
    GetParentFrame()->RecalcLayout();
    ResizeParentToFit( FALSE );	// Force Parent to grow!

	////////////////////////////////////////////////////////////////
	// Robot Cmd View Custom initialization:

	// General Globals are initialized in Robot.cpp

	g_RobotCmdViewHWND = GetSafeHwnd();
    if( INVALID_HANDLE_VALUE == g_RobotCmdViewHWND )
    {
		ROBOT_LOG( TRUE,  "Error getting Dialog Handle!\n" )
		ASSERT(0);
    }


	// Initialize GUI controls
	SetDlgItemText(IDC_CONNECT_SPEED, "00 ms");
	SetDlgItemText(IDC_POWER_DISPLAY, "OFF");
	SetDlgItemText( IDC_SERVO_NUM, "0"); 

	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_ACTION_MODE);
	pComboBox->SetCurSel(0);	// Set the combo box to first item

	pComboBox = (CComboBox*) GetDlgItem(IDC_BEHAVIOR_MODE);
	pComboBox->SetCurSel(0);	// Set the combo box to first item

	pComboBox = (CComboBox*) GetDlgItem(IDC_PLAY_MUSIC);
	pComboBox->SetCurSel(0);	// Set the combo box to first item

	pComboBox = (CComboBox*) GetDlgItem(IDC_HEAD_TILT);
	pComboBox->SetCurSel(0);	// Set the combo box to first item

	pComboBox = (CComboBox*) GetDlgItem(IDC_LASER_ZOOM);
	pComboBox->SetCurSel(0);	// Set the combo box to first item


//	SetDlgItemText( IDC_AVOID_OBJ_MAX_RANGE, "48");	// inches
//	SetDlgItemText( IDC_PATH_SPEED_ADD, "0");		// speed ticks

	CString strText;
	CSliderCtrl* pSlider;

	// Initialize the SpeedSlider Control and text box
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
	pSlider->SetRange(1, 255);
	pSlider->SetPos(m_nSpeedSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);

	// Initialize the TurnSlider Control and text box
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_TURN_SLIDER);
	pSlider->SetRange(1, 255);
	pSlider->SetPos(m_nTurnSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_TURN_STATIC, strText);

	// Initialize Pan Speed edit box
	SetDlgItemText( IDC_PAN_SPEED, "7"); 

	// Initialize Radar display

	// Create display widgets
	CRect widgetRect;
	CWnd *pWnd;

	// "Radar" display of sensors
	pWnd = GetDlgItem( IDC_RADAR_WIN );
	pWnd->GetClientRect( &widgetRect );
	pWnd->MapWindowPoints( this, &widgetRect );
	m_RadarDisplay.Create( NULL, NULL, WS_CHILD|WS_VISIBLE, widgetRect, this, 0 );

	// Laser Scanner display
	pWnd = GetDlgItem( IDC_LASER_DISPLAY_WIN );
	pWnd->GetClientRect( &widgetRect );
	pWnd->MapWindowPoints( this, &widgetRect );
	m_LaserDisplay.Create( NULL, NULL, WS_CHILD|WS_VISIBLE, widgetRect, this, 0 );

	// If using a WebCam, do this to position the video on the GUI:
	/***
	pWnd = GetDlgItem( IDC_CAMERA_WIN );
	pWnd->GetClientRect( &widgetRect );
	pWnd->MapWindowPoints( this, &widgetRect );
	m_HtmlCtrl.Create( NULL, NULL, WS_CHILD|WS_VISIBLE, widgetRect, this, 0 );
	***/

	// If using video capture, do this to position the video on the GUI
	pWnd = GetDlgItem( IDC_CAMERA_WIN );
	pWnd->GetClientRect( &g_CameraWindowRect );
	pWnd->MapWindowPoints( this, &g_CameraWindowRect );


	// Set Joystick active by default
	CheckDlgButton(IDC_USE_JOYSTICK, 1);

	// Laser active by default
	g_bLaserContinuousScanEnabled = TRUE;
	CheckDlgButton( IDC_LASER_SCAN_ENABLE, g_bLaserContinuousScanEnabled );


	// LED Eyes on + blink by default
	CheckDlgButton(IDC_ENABLE_LED_EYES, 1);
	CheckDlgButton(IDC_LED_EYES_BLINK, 1);
	// Don't send command right away, allow time for robot to start up
	// SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 );
	
///    SelectJoyStick( g_RobotMainFrameHWND );

	ROBOT_DISPLAY( TRUE, "Robot CmdView Common block Initialized" )


////////////////////////////////////////////////////////////////////
#if ( ROBOT_SERVER != 1 )		// ROBOTCLIENT code

	//SetWindowText(_T("    Robot  [ Client ]"));
	SetDlgItemText( IDC_CLIENT_OR_SERVER, "CLIENT" );
	ROBOT_DISPLAY( TRUE, "Robot Client Initialized")

////////////////////////////////////////////////////////////////////
#else	// ROBOT_SERVER code

	SetDlgItemText( IDC_IP_ADDR1, (LPCTSTR)"Server Mode"); 


	// Turn on the camera
//	TODO SendCommand( WM_ROBOT_CAMERA_POWER_CMD, (DWORD)POWER_ON, 0 );


	// Display Robot Server IP address
/*	char szHostName[1024];
	HOSTENT* lphostent;

	if(gethostname (szHostName, 1024) == SOCKET_ERROR)
	{
		ROBOT_LOG( TRUE,  "ERROR: Could not get host name\n"))
	}
	else
	{
		lphostent = gethostbyname(szHostName);
		if(lphostent == NULL)
		{
			ROBOT_LOG( TRUE,  "Could not get host information\n" )
		}
		else
		{
			//lphostent.h_addr_list
			SetDlgItemText( IDC_IP_ADDR1, (LPCTSTR)"192.168.1.106"); 
		}
	}
*/
	// Turn off XP Wireless Zero Config, so it won't mess with our data throughput!
#ifndef __WIN_CE_TARGET	// Not supported on WinCE devices
	// Hangs???
	// StopWirelessZeroConfig();
#endif


	// Example of how to control a system menu!
	//SendMessage( WM_SYSCOMMAND, SC_KEYMENU, 'f');



#endif // ROBOT_SERVER
////////////////////////////////////////////////////////////////////

	// Enable/disable selected sensors
/*	CheckDlgButton(IDC_ENABLE_IR0, 0);
	CheckDlgButton(IDC_ENABLE_IR1, 1);
	CheckDlgButton(IDC_ENABLE_IR2, 1);
	CheckDlgButton(IDC_ENABLE_IR3, 1);
	CheckDlgButton(IDC_ENABLE_IR4, 1);
	CheckDlgButton(IDC_ENABLE_IR5, 0);

	CheckDlgButton(IDC_ENABLE_US0, 1);
	CheckDlgButton(IDC_ENABLE_US1, 1);
	CheckDlgButton(IDC_ENABLE_US2, 1);
	CheckDlgButton(IDC_ENABLE_US3, 1);
	CheckDlgButton(IDC_ENABLE_US4, 1);
	CheckDlgButton(IDC_ENABLE_US5, 1);
	CheckDlgButton(IDC_ENABLE_US6, 1);
*/
//	SendCommand( WM_ROBOT_ENABLE_SENSOR, SENSOR_IR0, 0 );	// 0 = Disable
//	SendCommand( WM_ROBOT_ENABLE_SENSOR, SENSOR_IR5, 0 );	// 0 = Disable


	#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
		////////////////////////////////////////////////////////////////
		// KLUDGE - enable MS Kinect here for now - TODO-MUST

		CKinectNui *pKinectNui = new CKinectNui();  
		g_pKinectNui = (void*)pKinectNui; // Make pointer to class available to video callback


		// Clean state the class
		pKinectNui->Nui_Zero();

		// Bind application window handle
		if( INVALID_HANDLE_VALUE == g_RobotCmdViewHWND )
		{
			ROBOT_ASSERT(0);
		}
		pKinectNui->m_hWnd = g_RobotCmdViewHWND;

		// Initialize and start NUI processing
		pKinectNui->Nui_Init();

		////////////////////////////////////////////////////////////////
	#endif

	ROBOT_LOG( TRUE,  "\n=========================================================================\n" )
	ROBOT_LOG( TRUE,  "                          Initialization Complete \n" )
	ROBOT_LOG( TRUE,  "===========================================================================\n\n" )

}


void CRobotCmdView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRobotCmdView)
	DDX_Control(pDX, IDC_STATUS_LISTBOX, m_StatusListBox);
	DDX_Check(pDX, IDC_USE_JOYSTICK, m_UseJoystick);
	DDX_CBString(pDX, IDC_MOVE_DISTANCE, m_MoveDistance);
	DDX_Text(pDX, IDC_TEXT_TO_SPEAK, m_TextToSpeak);
	DDV_MaxChars(pDX, m_TextToSpeak, 80);
	DDX_CBString(pDX, IDC_TURN_AMOUNT, m_TurnAmount);
	DDX_Check(pDX, IDC_SEND_TO_AI, m_SendTextToAI);
	DDX_Text(pDX, IDC_FACE_CAPTURE_NAME, FaceCaptureName);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CRobotCmdView, CFormView)
	//{{AFX_MSG_MAP(CRobotCmdView)
	ON_WM_HSCROLL()
	ON_WM_VSCROLL()
	ON_BN_CLICKED(IDC_CENTER_BUTTON, OnCenterButton)
	ON_BN_CLICKED(IDC_STOP_BUTTON, OnStopButton)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_12V_ON, On12vOn)
	ON_BN_CLICKED(IDC_12V_OFF, On12vOff)
	ON_BN_CLICKED(IDC_DISCONNECT_FROM_HOST, OnDisconnectFromHost)
	ON_BN_CLICKED(IDC_USE_JOYSTICK, OnUseJoystick)
	ON_BN_CLICKED(IDC_CAMERA_WIN, OnCameraWin)
	ON_WM_LBUTTONDOWN()
	ON_BN_CLICKED(IDC_MOVE_DISTANCE_FORWARD, OnMoveDistanceForward)
	ON_BN_CLICKED(IDC_TURN_DISTANCE_LEFT, OnTurnDistanceLeft)
	ON_BN_CLICKED(IDC_TURN_DISTANCE_RIGHT, OnTurnDistanceRight)
	ON_BN_CLICKED(IDC_MOVE_DISTANCE_REVERSE, OnMoveDistanceReverse)
	ON_BN_CLICKED(IDC_OPEN_DEFAULT_PATH, OnOpenDefaultPath)
	ON_BN_CLICKED(IDC_OPEN_DEFAULT_MAP, OnOpenDefaultMap)
	ON_COMMAND(ID_SEL_CMD_VIEW_BTN, OnSelCmdViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_CMD_VIEW_BTN, OnUpdateSelCmdViewBtn)
	ON_COMMAND(ID_SEL_MAP_VIEW_BTN, OnSelMapViewBtn)
	ON_COMMAND(ID_SEL_PATH_VIEW_BTN, OnSelPathViewBtn)
	ON_BN_CLICKED(IDC_EXECUTE_PATH, OnExecutePath)
	ON_BN_CLICKED(IDC_CAM_DOWN, OnCamDown)
	ON_BN_CLICKED(IDC_CAM_DOWN_LEFT, OnCamDownLeft)
	ON_BN_CLICKED(IDC_CAM_DOWN_RIGHT, OnCamDownRight)
	ON_BN_CLICKED(IDC_CAM_LEFT, OnCamLeft)
	ON_BN_CLICKED(IDC_CAM_RIGHT, OnCamRight)
	ON_BN_CLICKED(IDC_CAM_STOP, OnCamStop)
	ON_BN_CLICKED(IDC_CAM_UP, OnCamUp)
	ON_BN_CLICKED(IDC_CAM_UP_LEFT, OnCamUpLeft)
	ON_BN_CLICKED(IDC_CAM_UP_RIGHT, OnCamUpRight)
	ON_BN_CLICKED(IDC_CAM_CENTER, OnCamCenter)
//	ON_BN_CLICKED(IDC_BRAKE_BUTTON, OnBrakeButton)
	ON_BN_CLICKED(IDC_CANCEL_PATH, OnCancelPath)
	ON_BN_CLICKED(IDC_RESUME_PATH, OnResumePath)
	ON_BN_CLICKED(IDC_PAUSE_PATH, OnPausePath)
	ON_BN_CLICKED(IDC_CB_HIGH_GEAR, OnCbHighGear)
	ON_BN_CLICKED(IDC_TEST_BRAKE, OnTestBrake)
	ON_BN_CLICKED(IDC_COLOR_TRACK_SEARCH_BTN, OnColorTrackSearch)
	ON_BN_CLICKED(IDC_COLOR_TRACK_CALIBRATE_BTN, OnColorTrackCalibrate)
	ON_BN_CLICKED(IDC_ENABLE_COLLISION_MODULE, OnEnableCollisionModule)
	ON_BN_CLICKED(IDC_ENABLE_AVOIDANCE_MODULE, OnEnableAvoidanceModule)
	ON_EN_CHANGE(IDC_CAMERA_MANUAL_COLOR_CALDATA, OnChangeCameraManualColorCaldata)
	ON_UPDATE_COMMAND_UI(ID_SEL_MAP_VIEW_BTN, OnUpdateSelMapViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_PATH_VIEW_BTN, OnUpdateSelPathViewBtn)
	ON_COMMAND(ID_SEL_SETUP_VIEW_BTN, OnSelSetupViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_SETUP_VIEW_BTN, OnUpdateSelSetupViewBtn)
	ON_BN_CLICKED(IDC_ENABLE_GPS_PATH, OnEnableGpsPath)
	ON_BN_CLICKED(IDC_ZOOM_OUT, OnZoomOut)
	ON_BN_CLICKED(IDC_ZOOM_IN, OnZoomIn)
	ON_BN_CLICKED(IDC_ZOOM_STOP, OnZoomStop)
	ON_CBN_SELCHANGE(IDC_BEHAVIOR_MODE, OnSelchangeBehaviorMode)
	ON_CBN_SELCHANGE(IDC_ACTION_MODE, OnSelchangeActionMode)
	ON_BN_CLICKED(IDC_ENABLE_LED_EYES, OnEnableLedEyes)
	ON_BN_CLICKED(IDC_ENABLE_CAMERA_LIGHTS, OnEnableCameraLights)
	ON_WM_KEYDOWN()
	ON_BN_CLICKED(IDC_LED_EYES_BLINK, OnLedEyesBlink)
	ON_BN_CLICKED(IDC_OPEN_DOWNSTAIRS_MAP, OnOpenDownstairsMap)
	ON_BN_CLICKED(IDC_OPEN_UPSTAIRS_MAP, OnOpenUpstairsMap)
//	ON_BN_CLICKED(IDC_OPEN_BLANK_MAP, OnOpenBlankMap)
	ON_BN_CLICKED(IDC_ENABLE_OBJECT_NAV_UPDATE, OnEnableObjectNavUpdate)
	ON_CBN_SELCHANGE(IDC_PLAY_MUSIC, OnSelchangePlayMusic)
	ON_BN_CLICKED(IDC_ENABLE_AUX_LIGHTS, OnEnableAuxLights)
	ON_BN_CLICKED(IDC_STOP_PANIC_STOP, OnStopPanicStop)
	ON_EN_VSCROLL(IDC_TEXT_TO_SPEAK, OnVscrollTextToSpeak)
	ON_COMMAND(ID_Q, OnQ_Key)
	ON_COMMAND(ID_A, OnKEY_A)
	ON_COMMAND(ID_D, OnKey_D)
	ON_COMMAND(ID_E, OnKey_E)
	ON_COMMAND(ID_S, OnKey_S)
	ON_COMMAND(ID_W, OnKey_W)
	ON_COMMAND(ID_X, OnKey_X)
	ON_COMMAND(ID_KEY_PLUS, OnKey_Plus)
	ON_COMMAND(ID_KEY_MINUS, OnKey_Minus)
	ON_COMMAND(ID_KEY_ZERO, OnKey_Zero)
	ON_CBN_SELCHANGE(IDC_HEAD_TILT, OnSelchangeHeadTilt)
	ON_COMMAND(ID_C, OnKey_C)
	ON_COMMAND(ID_ARM_MOVEMENT, OnArmMovementDlg)
	ON_BN_CLICKED(IDC_TAKE_SNAPSHOT, OnTakeSnapshot)
	ON_BN_CLICKED(IDC_RECORD_VIDEO, OnRecordVideo)
	ON_BN_CLICKED(IDC_RECORD_VIDEO_STOP, OnRecordVideoStop)
	//}}AFX_MSG_MAP

	ON_MESSAGE( (WM_ROBOT_DISPLAY_STATUS_MESSAGES), OnRobotDisplayMessage )
	ON_MESSAGE( (WM_ROBOT_DISPLAY_TCP_TIME), OnRobotDisplayTcpTime )
	ON_MESSAGE( (WM_ROBOT_DISPLAY_SINGLE_ITEM), OnRobotDisplaySingleItem )
	ON_MESSAGE( (WM_ROBOT_DISPLAY_BULK_ITEMS), OnRobotDisplayBulkItem )
	ON_MESSAGE( (WM_ROBOT_DISPLAY_OPEN_DATA_FILE), OnRobotDisplayOpenDataFile )

	ON_CBN_SELCHANGE(IDC_LASER_ZOOM, &CRobotCmdView::OnCbnSelchangeLaserZoom)
	ON_BN_CLICKED(IDC_LASER_SCAN_ENABLE, &CRobotCmdView::OnBnClickedLaserScanEnable)
	ON_BN_CLICKED(IDC_KINECT_UP, &CRobotCmdView::OnBnClickedKinectUp)
	ON_BN_CLICKED(IDC_KINECT_DOWN, &CRobotCmdView::OnBnClickedKinectDown)
	ON_STN_CLICKED(IDC_STAT_IR_AD_VERT_FRONT_LEFT, &CRobotCmdView::OnStnClickedStatIrAdVertFrontLeft)
	//	ON_STN_CLICKED(IDC_PIC_STATUS, &CRobotCmdView::OnStnClickedPicStatus)
	ON_STN_CLICKED(IDC_PIC_STATUS, &CRobotCmdView::OnStnClickedPicStatus)
	ON_BN_CLICKED(IDC_LOCAL_USER_CB, &CRobotCmdView::OnBnClickedLocalUser)
	ON_BN_CLICKED(IDC_KINECT_PWR_ENABLE, &CRobotCmdView::OnBnClickedKinectPwrEnable)
	ON_WM_CTLCOLOR()
	//ON_BN_CLICKED(IDC_BRAKE_BUTTON, &CRobotCmdView::OnBnClickedBrakeButton)
	ON_BN_CLICKED(IDC_CAPTURE_FACE, &CRobotCmdView::OnBnClickedCaptureFace)
	ON_STN_CLICKED(IDC_IR_BUMPER_ARM_L_FINGER_R, &CRobotCmdView::OnStnClickedIrBumperArmLFingerR)
	//ON_STN_CLICKED(IDC_BATTERY_WARNING_TEXT, &CRobotCmdView::OnStnClickedBatteryWarningText)
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// Overrides



#ifdef _DEBUG
CRobotCmdDoc* CRobotCmdView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRobotCmdDoc)));
	return (CRobotCmdDoc*)m_pDocument;
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdView diagnostics

#ifdef _DEBUG
void CRobotCmdView::AssertValid() const
{
	CFormView::AssertValid();
}

void CRobotCmdView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif //_DEBUG



//****************************************************************************
// CRobotCmdView message handlers
//****************************************************************************


void CRobotCmdView::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	// Handle Horizontal Sliders
	CString strText;
	CSliderCtrl* pSlider =	(CSliderCtrl*) pScrollBar;
	int nTemp;

	if( pScrollBar != NULL )	// The window scroll bars pass in NULL to this
	{
		switch( pScrollBar->GetDlgCtrlID() ) {

			case IDC_TURN_SLIDER:
			//ROBOT_LOG( TRUE,  "updating TURN slider control\n" )
			//ROBOT_DISPLAY( TRUE, "updating TURN slider control")
			// Turn range: -64 <--> +64
			nTemp = (pSlider->GetPos() - SLIDER_CENTER) / 2;	// divide to make turn less sensitive then speed
			strText.Format("%d", nTemp);
			SetDlgItemText(IDC_MOTOR_TURN_STATIC, strText);
			// Send the command to the motor control.  Loop in globals.cpp filters multiple small 
			g_MotorCurrentTurnCmd = nTemp;	// Center
			ROBOT_LOG( TRUE,  "Turn = %d\n", g_MotorCurrentTurnCmd )

		}
	}

	CFormView::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CRobotCmdView::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{
	// Handle Vertical Sliders
	int nTemp;
	CString strText;
	CSliderCtrl* pSlider =	(CSliderCtrl*) pScrollBar;
	CSpinButtonCtrl* pAvoidObjRangeSpin = (CSpinButtonCtrl*) pScrollBar;

	if( pScrollBar != NULL )	// The window scroll bars pass in NULL to this
	{
		switch( pScrollBar->GetDlgCtrlID() ) {

			case IDC_SPEED_SLIDER:
			//ROBOT_LOG( TRUE,  "updating SPEED slider control\n" )
			//ROBOT_DISPLAY( TRUE, "updating SPEED slider control")

			// Speed Range: -127 <--> +127
			nTemp = SLIDER_CENTER - pSlider->GetPos();

			strText.Format("%d", nTemp);
			SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);
			// Send the command to the motor control.  Loop in globals.cpp filters multiple small 
			g_MotorCurrentSpeedCmd = nTemp;
			ROBOT_LOG( TRUE,  "Speed = %d\n", g_MotorCurrentSpeedCmd )
			break;

	
/*			case IDC_AVOID_OBJ_MAX_RANGE_SPIN:
			// Handle spin control for setting the Avoid Object range
			nTemp = pAvoidObjRangeSpin->GetPos();
			strText.Format(_T("%d"), nTemp);
			SetDlgItemText(IDC_AVOID_OBJ_MAX_RANGE, strText);
			//Send the command to the module classes
//			SendCommand( WM_ROBOT_SET_AVOID_OBJ_RANGE, 0, (DWORD)nTemp );
			break;
*/

	
			case IDC_TEST_SERVO_SPIN:
			// Servo Spin Control
			nTemp = pSlider->GetPos();
			strText.Format(_T("%d"), nTemp);
			SetDlgItemText(IDC_SERVO_VALUE, strText);
			char strServoNum[8];
			UINT result = GetDlgItemText( IDC_SERVO_NUM, strServoNum, 2); 
			int nServoNum = atoi(strServoNum);
			SendCommand( WM_ROBOT_SERVO_CMD, nServoNum, (DWORD)nTemp ); // Servo number and Position
			break;
		}
	}

	CFormView::OnVScroll(nSBCode, nPos, pScrollBar);
}


void CRobotCmdView::OnCenterButton() 
{
	// Center the TurnSlider Control and text box
	CString strText;
	CSliderCtrl* pSlider = (CSliderCtrl*) GetDlgItem(IDC_TURN_SLIDER);
	m_nTurnSlider = SLIDER_CENTER;
	pSlider->SetPos(m_nTurnSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_TURN_STATIC, strText);
	// Send the command to the motor control.  Loop in globals.cpp filters multiple small changes
	g_MotorCurrentTurnCmd = 0;	// Center

}

void CRobotCmdView::OnStopButton() 
{

	// Center the SpeedSlider Control and text box
	CString strText;
	CSliderCtrl* pSlider = (CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
	m_nSpeedSlider = SLIDER_CENTER;
	pSlider->SetPos(m_nSpeedSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);
	OnCenterButton();	// Center the wheels on Stop command

	// Set the motor control state
	g_MotorCurrentSpeedCmd = 0;	// Stop
	g_MotorCurrentTurnCmd = 0;	// Center

	// Manual Stop button will override collision and avoidance behaviors, causing an immediate stop
	SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );
	//SendCommand( WM_ROBOT_JOYSTICK_DRIVE_CMD, 0, 0 );

	// Cancel any current behavior
	SendCommand( WM_ROBOT_SET_ACTION_CMD, ACTION_MODE_NONE, 0 );

}

void CRobotCmdView::OnDestroy() 
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

/*
////////////////////////////////////////////////////////////////
	// KLUDGE - delete MS Kinect here for now - TODO-MUST

	CKinectNui *pKinectNui = g_pKinectNui;
	pKinectNui->Nui_UnInit();
	SAFE_DELETE( pKinectNui );
	g_pKinectNui = INVALID_HANDLE_VALUE;
*/

	CFormView::OnDestroy();
}


void CRobotCmdView::On12vOn() 
{
	// Used for SERVO power!
	SendCommand( WM_ROBOT_SERVO_POWER_CMD, 0, POWER_ON );
}

void CRobotCmdView::On12vOff() 
{
	SendCommand( WM_ROBOT_SERVO_POWER_CMD, 0, POWER_OFF );
}


/*  Save this code, incase I need it to measure round-trip time.
void CRobotCmdView::StartTimer()
{
	m_nTimer = 0;
}
int CRobotCmdView::StopTimer()
{
	int temp = m_nTimer;
	//ROBOT_LOG( TRUE,  "timer = %d\n", m_nTimer)

	CString strText;
	strText.Format("%d ms", m_nTimer );
	SetDlgItemText(IDC_CONNECT_SPEED, strText);
	m_nTimer = -1;	// stop counting up
	return temp;
}
*/

void CRobotCmdView::DisplayPower(BOOL On)
{
	if( On )
		SetDlgItemText( IDC_POWER_DISPLAY, "ON" );
	else
		SetDlgItemText( IDC_POWER_DISPLAY, "OFF" );
}

/*
void CRobotCmdView::OnResetWatchdog() 
{
	// Process the Reset Watchdog button.
	// Resets without turning on 12v
	SendCommand( WM_ROBOT_RESET_WATCHDOG_CMD, 0, 0 );
	
}
*/
void CRobotCmdView::OnDisconnectFromHost() 
{
#if ( ROBOT_SERVER != 1 )		// ROBOTCLIENT code

	//SendCommand( WM_ROBOT_12V_POWER_CMD, 0, POWER_OFF );
	SendCommand( WM_ROBOT_CLIENT_DISCONNECT, 0, 0 );	// Tell server we are disconnecting
	RobotSleep(500, pDomainGUIThread);	// Allow time for message to get sent
	g_bConnectedToServer = FALSE;
	
	// Tell all the threads to exit, then check to see that they did 
	g_bRunThread = FALSE;

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
	
	CloseSocket( &g_ClientSockStruct );
	g_ClientSockStruct.hClientSockSendThread = INVALID_HANDLE_VALUE;
	g_ClientSockStruct.hClientSockReceiveThread = INVALID_HANDLE_VALUE;

#endif // ROBOTCLIENT code

}


/////////////////////////////////////////////////////////////////////////////
// Custom Message Handlers
//
__itt_string_handle* pshDisplayMsg = __itt_string_handle_create("GUI Display Msg");
LRESULT CRobotCmdView::OnRobotDisplayMessage(WPARAM wParam, LPARAM lParam)
{
	// ROBOT_LOG( TRUE,  "OnRobotDisplayMessage Called!\n" )
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);

	// This function displays queued up text from multiple threads
	// The text is stored in global memory, protected by critical sections
	// This function is ONLY called by the GUI thread, in response to
	// WM_ROBOT_DISPLAY_STATUS_MESSAGES

	int nStart = 0;
	int nEnd = 0;
	CString strText;

	if( !g_CriticalSectionsInitialized )
	{
		ROBOT_LOG( TRUE,  " Ignoring OnRobotDisplayMessage - g_CriticalSectionsInitialized = FALSE\n" )
		return 0;
	}

	//ROBOT_LOG( TRUE,  "DEBUG MSG= %s\n", g_StatusMessagesToDisplay )

	// Append the new line to the status box
	{
		///TAL_SCOPED_TASK_NAMED("LOCK");
		__itt_task_begin(pDomainGUIThread, __itt_null, __itt_null, pshDisplayMsg);
		__itt_task_begin(pDomainGUIThread, __itt_null, __itt_null, psh_csDisplayLock);
		EnterCriticalSection(&g_csDisplayLock);

		int nLength = g_StatusMessagesToDisplay.GetLength();
		if( nLength > 0 )
		{
			while( TRUE )
			{
				nEnd = g_StatusMessagesToDisplay.Find('\n', nStart);
				if( -1 == nEnd )
					break;
				CString strText;
				if( nEnd < nStart )
				{
					ROBOT_LOG( TRUE,  "ERROR Adding String to listbox, nEnd < nStart!")
				}
				else
				{
					strText = g_StatusMessagesToDisplay.Mid(nStart, (nEnd-nStart));
					nStart = nEnd+1;	// for the next string
				}
				if( LB_ERRSPACE == m_StatusListBox.AddString(strText) )
				{
					ROBOT_LOG( TRUE,  "ERROR = Status Listbox Out of Space!")
					ROBOT_ASSERT(0);
				}
				if( nStart >= nLength )
					break;
			}
			g_StatusMessagesToDisplay.Empty();	// clear out the message buffer
		}
		
		LeaveCriticalSection(&g_csDisplayLock);
		__itt_task_end(pDomainGUIThread);	}

	// Scroll view to bottom item
	m_StatusListBox.SetCurSel( m_StatusListBox.GetCount() -1 ) ;	
	__itt_task_end(pDomainGUIThread);

	return 0;
}


LRESULT CRobotCmdView::OnRobotDisplayTcpTime(WPARAM wParam, LPARAM lParam)
{
	// This function displays TCP/IP trip time
	// This function is ONLY called by the GUI thread, in response to
	// WM_ROBOT_DISPLAY_TCP_TIME
	// lParam holds the trip time in milliseconds
	IGNORE_UNUSED_PARAM (wParam);

	CString strText;
	strText.Format( "%d", lParam / 1000 );
		SetDlgItemText( IDC_CONNECT_SPEED, strText );

	return 0;
}

__itt_string_handle* pshDisplayBulk = __itt_string_handle_create("GUI Display Bulk");
LRESULT CRobotCmdView::OnRobotDisplayBulkItem(WPARAM Item, LPARAM lParam)
{
	// This function displays various Bulk items (For example Arduino sensor readings or Radar Sensor readings)
	// from the Arduino module.  This function is ONLY called by the GUI thread, 
	// in response to WM_ROBOT_DISPLAY_BULK_ITEMS
	// Item indicates the item to display.  lParam holds the sensor number, etc.
	__itt_task_begin(pDomainGUIThread, __itt_null, __itt_null, pshDisplayBulk);

	CString strText;
	static int LastConnectedToPIC = SUBSYSTEM_WAITING;
	static int LastConnectedToDyna = SUBSYSTEM_WAITING;
	static int LastConnectedToRX64 = SUBSYSTEM_WAITING;
	static int LastConnectedToKerr = SUBSYSTEM_WAITING;
	static int LastConnectedToMotor = SUBSYSTEM_WAITING;
	static int LastKinectReady = SUBSYSTEM_WAITING;
	static int LastCameraReady = SUBSYSTEM_WAITING;
	static int LastLaserReady = SUBSYSTEM_WAITING;
	static int LastArmReady_L = SUBSYSTEM_WAITING;
	static int LastArmReady_R = SUBSYSTEM_WAITING;
	static int SystemInitCompleteReported = FALSE;

	static BOOL LastWheelDropCaster = FALSE;
	static BOOL LastBumperLeft = FALSE;
	static BOOL LastBumperRight = FALSE;

	/////////////////////////////////////////////////////////////////////////////////////
	// Handle Global sub-system Status updates if any changes

#if( ROBOT_TYPE == TURTLE )

	// Note, Left and Right wheel drops not use; wheels bolted in place for stability
	if( g_pIRobotStatus->WheelDropCaster != LastWheelDropCaster )
	{
		strText = ( (g_pIRobotStatus->WheelDropCaster == 0) ? _T("Caster OK") : _T("Caster Drop") );
		SetDlgItemText( IDC_CASTER_DROP, (LPCTSTR)strText );
		LastWheelDropCaster = g_pIRobotStatus->WheelDropCaster;
	}

	if( g_pIRobotStatus->BumperLeft != LastBumperLeft )
	{
		strText = ( (g_pIRobotStatus->BumperLeft == 0) ? _T("Left Bumper") : _T("Left Hit") );
		SetDlgItemText( IDC_STAT_BMPR_LEFT, (LPCTSTR)strText );
		LastBumperLeft = g_pIRobotStatus->BumperLeft;
	}

	if( g_pIRobotStatus->BumperRight != LastBumperRight )
	{
		strText = ( (g_pIRobotStatus->BumperRight == 0) ? _T("Right Bumper") : _T("Right Hit") );
		SetDlgItemText( IDC_STAT_BMPR_RIGHT, (LPCTSTR)strText );
		LastBumperRight = g_pIRobotStatus->BumperRight;
	}


#endif



	if( g_DynaSubSystemStatus != LastConnectedToDyna )
	{
		if( SUBSYSTEM_CONNECTED == g_DynaSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_DynaSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_DynaSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_DYNA_STATUS, (LPCTSTR)strText );
		LastConnectedToDyna = g_DynaSubSystemStatus;
	}

	if( g_MotorSubSystemStatus != LastConnectedToMotor )
	{
		if( SUBSYSTEM_CONNECTED == g_MotorSubSystemStatus  )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_MotorSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_MotorSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("<<OFFLINE!>>") );
		SetDlgItemText( IDC_MOTOR_STATUS, (LPCTSTR)strText );
		LastConnectedToMotor = g_MotorSubSystemStatus;
	}

	if( g_KinectSubSystemStatus != LastKinectReady )
	{
		if( SUBSYSTEM_CONNECTED == g_KinectSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_KinectSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_KinectSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_KINECT_STATUS, (LPCTSTR)strText );
		LastKinectReady = g_KinectSubSystemStatus;
	}

	///////// Only on Loki Robot //////////////
	#if (ROBOT_TYPE == LOKI) 

	if( g_CameraSubSystemStatus != LastCameraReady )
	{
		if( SUBSYSTEM_CONNECTED == g_CameraSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_CameraSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_CameraSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_CAMERA_STATUS, (LPCTSTR)strText );
		LastCameraReady = g_CameraSubSystemStatus;
	}

	if( g_RX64SubSystemStatus != LastConnectedToRX64 )
	{
		if( SUBSYSTEM_CONNECTED == g_RX64SubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_RX64SubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_RX64SubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_RX64_STATUS, (LPCTSTR)strText );
		LastConnectedToRX64 = g_RX64SubSystemStatus;
	}

	if( g_KerrSubSystemStatus != LastConnectedToKerr )
	{
		if( SUBSYSTEM_CONNECTED == g_KerrSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_KerrSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_KerrSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_KERR_STATUS, (LPCTSTR)strText );
		LastConnectedToKerr = g_KerrSubSystemStatus;
	}
	
	if( g_LaserSubSystemStatus != LastLaserReady )
	{
		if( SUBSYSTEM_CONNECTED == g_LaserSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_LaserSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_LaserSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_LASER_STATUS, (LPCTSTR)strText );
		LastLaserReady = g_LaserSubSystemStatus;
	}

	if( g_LeftArmSubSystemStatus != LastArmReady_L )
	{
		if( SUBSYSTEM_CONNECTED == g_LeftArmSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_LeftArmSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_LeftArmSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_ARM_L_STATUS, (LPCTSTR)strText );
		LastArmReady_L = g_LeftArmSubSystemStatus;
	}

	if( g_RightArmSubSystemStatus != LastArmReady_R )
	{
		if( SUBSYSTEM_CONNECTED == g_RightArmSubSystemStatus )
			strText.Format( _T("Ready") );
		else if( SUBSYSTEM_DISABLED == g_RightArmSubSystemStatus )
			strText.Format( _T("Disabled") );
		else if( SUBSYSTEM_WAITING == g_RightArmSubSystemStatus )
			strText.Format( _T("Initializing") );
		else
			strText.Format( _T("ERROR") );
		SetDlgItemText( IDC_ARM_R_STATUS, (LPCTSTR)strText );
		LastArmReady_R = g_RightArmSubSystemStatus;
	}
	#endif


	// Report Initial Status
	if( !SystemInitCompleteReported )
	{
		#if (ROBOT_TYPE == LOKI) 
		if( ( (SUBSYSTEM_CONNECTED == g_CameraSubSystemStatus) ||(SUBSYSTEM_DISABLED == g_CameraSubSystemStatus) ) && // Camera is optional
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				(SUBSYSTEM_CONNECTED == g_RX64SubSystemStatus) &&
			#endif
			(SUBSYSTEM_CONNECTED == g_KerrSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_LaserSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_LeftArmSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_RightArmSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_DynaSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_MotorSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_KinectSubSystemStatus)  )
		#else
		if( (SUBSYSTEM_CONNECTED == g_DynaSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_MotorSubSystemStatus) &&
			(SUBSYSTEM_CONNECTED == g_KinectSubSystemStatus)  )
		#endif
		{
			SpeakText( "All Systems Ready" );
			SystemInitCompleteReported = TRUE;
			SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE ); // Turn lights on
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 ); // Make sure eyes are enabled
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Now, handle the message

	if( ROBOT_RESPONSE_RADAR_SAMPLE == Item )
	{
		// lParam is the Sensor number

		if( US_ARRAY1 == lParam )
		{
			m_RadarDisplay.SetData( lParam, ULTRASONIC1_SAMPLES, g_ScaledSensorData[lParam] );
		}
		else if( IR_ARRAY1 == lParam )
		{
			m_RadarDisplay.SetData( lParam, IR1_SAMPLES, g_ScaledSensorData[lParam] );
			//strText.Format( "%d", (UINT)g_ScaledSensorData[lParam][IR1_SAMPLES/2] );  // display the middle value
			//SetDlgItemText( IDC_IR_DISPLAY, strText );
		}
		else
		{
			ROBOT_LOG( TRUE,  "ERROR - Bad value for ROBOT_RESPONSE_RADAR_SAMPLE lParam\n" )
		}
	}
	else if( ROBOT_RESPONSE_PIC_STATUS == Item )
	{

		// Handle Status Info from the Arduino
		// Process Status Flags first, then the rest of the status fields
		// Note, we don't use ROBOT_RESPONSE_PIC_READY like we do for other modules

		if( 1 == ROBOT_SIMULATION_MODE )
		{
			strText.Format( _T("SIMULATION") );
			SetDlgItemText( IDC_PIC_STATUS, (LPCTSTR)strText );
		}
		else if( g_ArduinoSubSystemStatus != LastConnectedToPIC )
		{
			if( SUBSYSTEM_CONNECTED == g_ArduinoSubSystemStatus )
				strText.Format( _T("Ready") );
			else if( SUBSYSTEM_DISABLED == g_ArduinoSubSystemStatus )
				strText.Format( _T("Disabled") );
			else if( SUBSYSTEM_WAITING == g_ArduinoSubSystemStatus )
				strText.Format( _T("Initializing") );
			else // SUBSYSTEM_FAILED
				strText.Format( _T("ERROR") );
			SetDlgItemText( IDC_PIC_STATUS, (LPCTSTR)strText );
			LastConnectedToPIC = g_ArduinoSubSystemStatus;
		}

		if( g_pFullSensorStatus->StatusFlags & HW_STATUS_WATCHDOG_EXPIRED )
		{			
			// Watchdog timer expired.
			strText.Format( _T("ERR") );
			SetDlgItemText( IDC_STAT_WATCHDOG, (LPCTSTR)strText );

			// See if we already displayed an error message to the log
			if( !m_bTimerExpiredReported ) 
			{
				ROBOT_DISPLAY( TRUE, "Arduino Watchdog Timer Expired" )
				m_bTimerExpiredReported = TRUE;
			}
		}
		else
		{
			// Timer not expired.  
			strText.Format( _T("OK") );
			SetDlgItemText( IDC_STAT_WATCHDOG, (LPCTSTR)strText );

			// See if we need to display a messge
			if( m_bTimerExpiredReported )
			{
				ROBOT_DISPLAY( TRUE, "Arduino Watchdog Timer OK now!" )
				m_bTimerExpiredReported = FALSE;
			}
		}

		if( g_pFullSensorStatus->StatusFlags & HW_STATUS_POWER_ON_INDICATOR ) 
		{	
			strText.Format( _T("ON") );
			SetDlgItemText( IDC_STAT_POWER, (LPCTSTR)strText );
		}
		else
		{
			strText.Format( _T("OFF") );
			SetDlgItemText( IDC_STAT_POWER, (LPCTSTR)strText );
		}

		//TODO: g_pFullSensorStatus->ThermalArray[];

		///////////////////////////////////////////////////////////////////////////////
		#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT


			double BatteryVoltage = (double)g_pFullSensorStatus->Battery0 / 100.0;	// convert from hundredths of a volt

			strText.Format( _T("%2.1f"), BatteryVoltage );
			SetDlgItemText( IDC_STAT_BATTERY, (LPCTSTR)strText ); 
			//CProgressCtrl* p = (CProgressCtrl*)g_RobotCEDlg->GetDlgItem(IDC_BATTERY_METER);
			//p->SetPos( (int)BatteryVoltage );

			// TODO-CAR-MUST!  Change these to use Battery VOLTAGE not Arduino Units
			if( BatteryVoltage <= 108 )
			{
				strText = "Battery Critical! <5.9v";
			}
			else if( BatteryVoltage <= 135 )
			{
				strText = "Battery Very Low! <6.5v";
			}
			else if( BatteryVoltage <= 140 )
			{
				strText = "Battery Getting Low! <6.8v";
			}
			else if( BatteryVoltage < 150 ) 
			{
				strText = "Battery Good: 6.8v+";
			}
			else if( BatteryVoltage < 181 ) 
			{
				strText = "Battery Good: 7.0v";
			}
			else if( BatteryVoltage < 205 ) 
			{
				strText = "Battery Good: 7.5v";
			}
			else if( BatteryVoltage < 228 ) 
			{
				strText = "Battery Very Good: 8.0v";
			}
			else if( BatteryVoltage < 238 ) 
			{
				strText = "Battery Full: 8.5v";
			}
			else
			{
				strText = "Battery Full: 9+ v";
			}
			SetDlgItemText( IDC_BATTERY_WARNING_TEXT, (LPCTSTR)strText );


		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

			double BatteryVoltage = (double)g_pFullSensorStatus->Battery0 / 100.0;	// convert from hundredths of a volt

			strText.Format( _T("%2.1f"), BatteryVoltage );
			SetDlgItemText( IDC_STAT_BATTERY, (LPCTSTR)strText ); 
			//CProgressCtrl* p = (CProgressCtrl*)g_RobotCEDlg->GetDlgItem(IDC_BATTERY_METER);
			//p->SetPos( (int)BatteryVoltage );

			strText = "Battery Fully Charged";
			m_BatteryStatus = OK;

			if( BatteryVoltage <= 10.5 )
			{
				strText = "Battery Critical! < 10v";
				m_BatteryStatus = URGENT;
			}
			else if( BatteryVoltage <= 11.5 )
			{
				strText = "Battery Recharge Needed! < 11.5v";
				m_BatteryStatus = URGENT;
			}
			else if( BatteryVoltage <= 12.4 )
			{
				strText = "Battery Getting Low";
				m_BatteryStatus = LOW;
			}
			else if( BatteryVoltage < 12.7 ) 
			{
				strText = "Battery Good";
				m_BatteryStatus = OK;
			}
			else if( BatteryVoltage < 13.0 ) 
			{
				strText = "Battery Very Good";
				m_BatteryStatus = OK;
			}
			else if( BatteryVoltage < 13.5 ) 
			{
				strText = "On Charger";
				m_BatteryStatus = OK;
			}
			SetDlgItemText( IDC_BATTERY_WARNING_TEXT, (LPCTSTR)strText );

			// Display an urgent Warning if battery getting low!
			/*** TOO MUCH JUNK IN THE LOG FILE
			if( BatteryVoltage <= 10.5 )
			{
				CString MsgString;
				MsgString.Format( _T("*** WARNING! - %s ***\n"), strText );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}
			***/
		
		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE

			ROBOT_ASSERT(0); // TODO g_pKobukiStatus

			// Note: iRobotCreate data in g_pIRobotStatus, not g_pFullSensorStatus
			int BatteryChargeLevel = ( g_pIRobotStatus->BatteryCapacity != 0 ? 
				(g_pIRobotStatus->BatteryCharge * 100) / g_pIRobotStatus->BatteryCapacity : 0 );


			strText.Format( _T("%d %%"), BatteryChargeLevel );
			SetDlgItemText( IDC_STAT_BATTERY, (LPCTSTR)strText );
			SetDlgItemText( IDC_BATTERY_WARNING_TEXT, (LPCTSTR)strText ); // KLUDGE!

			strText.Format( _T("%2.2f volts"), (float)g_pIRobotStatus->BatteryVoltage / 1000.0 );
			SetDlgItemText( IDC_STAT_BATTERY_VOLTS, (LPCTSTR)strText );

			strText = ( g_pIRobotStatus->InHomeBase ? "Yes" : "No" );
			SetDlgItemText( IDC_STAT_BASE_DOCKED, (LPCTSTR)strText );

			strText = ( g_pIRobotStatus->ChargingPlugInserted ? "Yes" : "No" );
			SetDlgItemText( IDC_STAT_PLUG_INSERTED, (LPCTSTR)strText );

			strText.Format( _T("%d"), g_pIRobotStatus->BatteryCurrent );
			SetDlgItemText( IDC_STAT_CURRENT_DRAW, (LPCTSTR)strText );


			// TODO!  Set color of indicator to match battery status.  See: m_BatteryStatus = URGENT;
			strText = "";


			//CProgressCtrl* p = (CProgressCtrl*)g_RobotCEDlg->GetDlgItem(IDC_BATTERY_METER);
			//p->SetPos( (int)BatteryVoltage );
/*
			// TODO-TURTLE
			if( BatteryCapacityMaH <= 500 )
			{
				strText = "Battery Critical! < 500";
			}
			else if( BatteryCapacityMaH <= 800 )
			{
				strText = "Battery Very Low! Recharge Needed! < 800";
			}
			else if( BatteryCapacityMaH <= 1000 )
			{
				strText = "Battery Low! Recharge Needed! < 1000";
			}
			else if( BatteryCapacityMaH <= 1500 )
			{
				strText = "Battery Getting Low! < 1500";
			}
			else if( BatteryCapacityMaH < 2000 ) 
			{
				strText = "Battery Good";
			}
			else if( BatteryCapacityMaH < 2500 ) 
			{
				strText = "Battery Very Good";
			}
			else if( BatteryCapacityMaH < 3000 ) 
			{
				strText = "Battery Full";
			}
			else
			{
				strText = "Battery Fully Charged";
			}

			// Display an urgent Warning if battery getting low!
			if( BatteryCapacityMaH <= 500 )
			{
				CString MsgString;
				MsgString.Format( _T("*** WARNING! - %s ***\n"), strText );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}
*/

		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP

			// Note: Teleop Robot data in g_KobukiStatus, not g_pFullSensorStatus
			//int BatteryChargeLevel = ( g_pIRobotStatus->BatteryCapacity != 0 ? 
			//	(g_pIRobotStatus->BatteryCharge * 100) / g_pIRobotStatus->BatteryCapacity : 0 );


			strText.Format( _T("%3d%%"), (int)g_pKobukiStatus->BatteryPercent );
			SetDlgItemText( IDC_STAT_BATTERY, (LPCTSTR)strText );

			switch( g_pKobukiStatus->BatteryLevelEnum )
			{
				case 0:
					strText.Format( _T("Dangerously Low") );break;
				case 1:
					strText.Format( _T("Battery Low") );break;
				case 2:
					strText.Format( _T("Battery OK") );break;
				case 3:
					strText.Format( _T("Battery at Maximum") );break;
				default:
					strText.Format( _T("Battery Enum Error %d"), g_pKobukiStatus->BatteryLevelEnum );
			}
			SetDlgItemText( IDC_BATTERY_WARNING_TEXT, (LPCTSTR)strText );


			strText.Format( _T("%2.1f volts"), g_pKobukiStatus->BatteryVoltage );
			SetDlgItemText( IDC_STAT_BATTERY_VOLTS, (LPCTSTR)strText );

			//strText = ( g_pIRobotStatus->InHomeBase ? "Yes" : "No" );
			//SetDlgItemText( IDC_STAT_BASE_DOCKED, (LPCTSTR)strText );

			//strText = ( g_pIRobotStatus->ChargingPlugInserted ? "Yes" : "No" );

			switch( g_pKobukiStatus->BatteryChargeStateEnum )
			{
				case 0:
					strText.Format( _T("Disch") );break;
				case 1:
					strText.Format( _T("Fully") );break;
				case 2:
					strText.Format( _T("Chrgn") );break;
				default:
					strText.Format( _T("Err") );
			}
			SetDlgItemText( IDC_STAT_PLUG_INSERTED, (LPCTSTR)strText );

			//strText.Format( _T("%d"), g_pIRobotStatus->BatteryCurrent );
			//SetDlgItemText( IDC_STAT_CURRENT_DRAW, (LPCTSTR)strText );


			// TODO!  Set color of indicator to match battery status.  See: m_BatteryStatus = URGENT;
			strText = "";


			//CProgressCtrl* p = (CProgressCtrl*)g_RobotCEDlg->GetDlgItem(IDC_BATTERY_METER);
			//p->SetPos( (int)BatteryVoltage );
/*
			// TODO-TURTLE
			if( BatteryCapacityMaH <= 500 )
			{
				strText = "Battery Critical! < 500";
			}
			else if( BatteryCapacityMaH <= 800 )
			{
				strText = "Battery Very Low! Recharge Needed! < 800";
			}
			else if( BatteryCapacityMaH <= 1000 )
			{
				strText = "Battery Low! Recharge Needed! < 1000";
			}
			else if( BatteryCapacityMaH <= 1500 )
			{
				strText = "Battery Getting Low! < 1500";
			}
			else if( BatteryCapacityMaH < 2000 ) 
			{
				strText = "Battery Good";
			}
			else if( BatteryCapacityMaH < 2500 ) 
			{
				strText = "Battery Very Good";
			}
			else if( BatteryCapacityMaH < 3000 ) 
			{
				strText = "Battery Full";
			}
			else
			{
				strText = "Battery Fully Charged";
			}
			SetDlgItemText( IDC_BATTERY_WARNING_TEXT, (LPCTSTR)strText );

			// Display an urgent Warning if battery getting low!
			if( BatteryCapacityMaH <= 500 )
			{
				CString MsgString;
				MsgString.Format( _T("*** WARNING! - %s ***\n"), strText );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}
*/

		///////////////////////////////////////////////////////////////////////////////
		#else
			#error BAD SENSOR_CONFIG_TYPE!  
		#endif



		// Last Error	
		static BOOL bFlip = 1;
		if( bFlip )
		{
			bFlip = 0;
			strText.Format( _T("--") );
		}
		else
		{
			bFlip = 1;
			strText.Format( _T("%d"), g_pFullSensorStatus->LastError );
		}
		SetDlgItemText( IDC_STAT_LAST_ERROR, (LPCTSTR)strText ); 

		// Debug Code
		strText.Format( _T("%d"), g_pFullSensorStatus->DebugCode );
		SetDlgItemText( IDC_STAT_DEBUG, (LPCTSTR)strText );

		// Bumper Status - no longer used like this
		//strText.Format( _T("%02X %02X"), g_pFullSensorStatus->HWBumper, g_pFullSensorStatus->IRBumper );
		//SetDlgItemText( IDC_BUMPER_DISPLAY, (LPCTSTR)strText );

//		strText.Format( "%d", foobar );


		// The Radar Array is mapped left to right in a circle.
		// For GUI display, convert from TenthInches to Inches

		int TempIRArray[FIXED_IR_SAMPLES];	

		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[0] / 10) );	// Left Side
		SetDlgItemText( IDC_STAT_IR0,(LPCTSTR)strText );
		TempIRArray[0] = g_pFullSensorStatus->IR[0];

		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[1] / 10) );	// Forward Low Mounted
		SetDlgItemText( IDC_STAT_IR1,(LPCTSTR)strText );
		TempIRArray[1] = g_pFullSensorStatus->IR[1];

		// DISABLED FOR LOKI
//		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[2] / 10) );	// Forward Head Mounted
//		SetDlgItemText( IDC_STAT_IR2,(LPCTSTR)strText );
		TempIRArray[2] = g_pFullSensorStatus->IR[2];

//		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[3] / 10) );	// Forward Head Mounted
//		SetDlgItemText( IDC_STAT_IR3,(LPCTSTR)strText );
		TempIRArray[3] = g_pFullSensorStatus->IR[3];


		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[4] / 10) );	// Forward Low Mounted
		SetDlgItemText( IDC_STAT_IR4,(LPCTSTR)strText );
		TempIRArray[4] = g_pFullSensorStatus->IR[4];

		strText.Format( _T("%d"), (g_pFullSensorStatus->IR[5] / 10) );	// Right Side
		SetDlgItemText( IDC_STAT_IR5,(LPCTSTR)strText );
		TempIRArray[5] = g_pFullSensorStatus->IR[5];

		m_RadarDisplay.SetData( 
			FIXED_IR_ARRAY,		// IR Array #1 (only one implemented for now)
			FIXED_IR_SAMPLES,		// Number of samples in the array
			TempIRArray );// Pointer to the Data


		// UltraSonic  
		// The Radar Array is mapped left to right in a circle.
		// But to keep the GUI consistant, it's grouped Odd/Even
		//	Sensor Number:	        1 0 2 	(0 center)
		//  Array position:		  0 1 2 3 4 
		//  GUI:                5 3 1 0 2 4 6
		///////////////////////////////////////////////////////////////////////////////
		#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

			UINT TempUSArray[FIXED_US_SAMPLES];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[1] );	// Side Left
			SetDlgItemText( IDC_STAT_US5, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[1];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[2] );	// Angle Left
			SetDlgItemText( IDC_STAT_US3, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[2];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[3] );	// Front Left
			SetDlgItemText( IDC_STAT_US1, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[3];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[0] );	// Camera (center)
			SetDlgItemText( IDC_STAT_US0, (LPCTSTR)strText );
			TempUSArray[1] = g_pFullSensorStatus->US[0];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[4] );	// Front Right
			SetDlgItemText( IDC_STAT_US2, (LPCTSTR)strText );
			TempUSArray[2] = g_pFullSensorStatus->US[4];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[5] );	// Angle Left
			SetDlgItemText( IDC_STAT_US4, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[5];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[6] );	// Side Left
			SetDlgItemText( IDC_STAT_US6, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[6];

			m_RadarDisplay.SetData( 
				FIXED_US_ARRAY,		// US Array #1
				(FIXED_US_SAMPLES),	// Number of samples in the array
				TempUSArray );		// Pointer to the Data

		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

			int TempUSArray[FIXED_US_SAMPLES];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[1] );	// Front Left
			SetDlgItemText( IDC_STAT_US1, (LPCTSTR)strText );
			TempUSArray[0] = g_pFullSensorStatus->US[1];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[0] );	// Camera (center)
			SetDlgItemText( IDC_STAT_US0, (LPCTSTR)strText );
			TempUSArray[1] = g_pFullSensorStatus->US[0];

			strText.Format( _T("%u"), g_pFullSensorStatus->US[2] );	// Front Right
			SetDlgItemText( IDC_STAT_US2, (LPCTSTR)strText );
			TempUSArray[2] = g_pFullSensorStatus->US[2];

			m_RadarDisplay.SetData( 
				FIXED_US_ARRAY,		// US Array #1
				(FIXED_US_SAMPLES),	// Number of samples in the array
				TempUSArray );		// Pointer to the Data

			// Display Bumper Status
			const char *IrHit = "IR Hit";
			const char *BmprHit =  "B Hit";
			const char *NoHit = "bmpr";
			CStatic * pTextBox = 0;

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_IR_BUMPER_LOW_FRONT_LEFT);
			if( g_pFullSensorStatus->IRBumperFrontLeft )
			{
				//Beep(1000,500);
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_FRONT_LEFT, IrHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_FRONT_LEFT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_IR_BUMPER_LOW_FRONT_RIGHT);
			if( g_pFullSensorStatus->IRBumperFrontRight )
			{
				//Beep(1000,500);
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_FRONT_RIGHT, IrHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_FRONT_RIGHT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_CLIFF_LEFT);
			if( g_pFullSensorStatus->CliffLeft )
			{
				SetDlgItemText( IDC_STAT_CLIFF_LEFT, "CLIFF" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_CLIFF_LEFT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_CLIFF_RIGHT);
			if( g_pFullSensorStatus->CliffRight )
			{
				SetDlgItemText( IDC_STAT_CLIFF_RIGHT, "CLIFF" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_CLIFF_RIGHT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_IR_BUMPER_LOW_REAR_LEFT);
			if( g_pFullSensorStatus->IRBumperRearLeft  )
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_REAR_LEFT, IrHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_REAR_LEFT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_IR_BUMPER_LOW_REAR_RIGHT);
			if( g_pFullSensorStatus->IRBumperRearRight )
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_REAR_RIGHT, IrHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_IR_BUMPER_LOW_REAR_RIGHT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_PIR_LEFT);
			if( g_pFullSensorStatus->PIRMotionLeft )
			{
				SetDlgItemText( IDC_STAT_PIR_LEFT, "PIR" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_PIR_LEFT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_PIR_RIGHT);
			if( g_pFullSensorStatus->PIRMotionRight )
			{
				SetDlgItemText( IDC_STAT_PIR_RIGHT, "PIR" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_PIR_RIGHT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			// Right Arm sensors
// TODO:	if( ARM_L_IR_BUMPER_INSIDE_CLAW )
//				SetDlgItemText( IDC_IR_BUMPER_ARM_R_INSIDE_CLAW, "Claw" );

			/*
			pTextBox = (CStatic*) GetDlgItem(IDC_IR_BUMPER_ARM_L_ELBOW);
			if( (ARM_L_IR_BUMPER_OBJECT_ELBOW) || (ARM_L_HW_BUMPER_OBJECT_ELBOW) ) // Indicate hit on IR or HW Bumper!
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_ELBOW, "Elbow" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_ELBOW, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_IR_BUMPER_ARM_R_ELBOW);
			if( (ARM_R_IR_BUMPER_OBJECT_ELBOW) || (ARM_R_HW_BUMPER_OBJECT_ELBOW) )	// Indicate hit on IR or HW Bumper!
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_R_ELBOW, "Elbow" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_R_ELBOW, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}
			*/

			pTextBox = (CStatic*) GetDlgItem(IDC_IR_BUMPER_ARM_L_FINGER_R);
			if( g_pFullSensorStatus->ArmLeftBumperFingerRight )
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_FINGER_R, "Finger" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_FINGER_R, NoHit );

				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_IR_BUMPER_ARM_L_FINGER_L);
			if( g_pFullSensorStatus->ArmLeftBumperFingerLeft )
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_FINGER_L, "Finger" );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_IR_BUMPER_ARM_L_FINGER_L, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			// Analog Hand sensor
			strText.Format( _T("%u"), (ARM_L_IR_SENSOR_CLAW / 10) );	// Hand - convert from TenthInches
			SetDlgItemText( IDC_IR_HAND_L, (LPCTSTR)strText );

			// Hardware Switch Bumpers
			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_BMPR_FRONT);
			if( g_pNavSensorSummary->BumperHitFront() )
			{
				SetDlgItemText( IDC_STAT_BMPR_FRONT, BmprHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_BMPR_FRONT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_BMPR_REAR);
			if( g_pFullSensorStatus->HWBumperRear )
			{
				SetDlgItemText( IDC_STAT_BMPR_REAR, BmprHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_BMPR_REAR, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_BMPR_LEFT);
			if( g_pFullSensorStatus->HWBumperSideLeft )
			{
				SetDlgItemText( IDC_STAT_BMPR_LEFT, BmprHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_BMPR_LEFT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			pTextBox = (CStatic*) GetDlgItem(IDC_STAT_BMPR_RIGHT);
			if( g_pFullSensorStatus->HWBumperSideRight )
			{
				SetDlgItemText( IDC_STAT_BMPR_RIGHT, BmprHit );
				pTextBox->EnableWindow(1);// Enable
			}
			else
			{
				SetDlgItemText( IDC_STAT_BMPR_RIGHT, NoHit );
				pTextBox->EnableWindow(0);// Disable
			}

			strText.Format( _T("%u"), (ARM_R_IR_SENSOR_CLAW / 10) );	// Hand - convert from TenthInches
			SetDlgItemText( IDC_IR_HAND_R, (LPCTSTR)strText );
			
			// Range Sensors in Head
			// Compensate for fact that Head sensors are set back from front of robot
			strText.Format( _T("%u"), (g_pFullSensorStatus->IR[2] - HEAD_IR_OFFSET_FROM_FRONT_TENTH_INCHES) / 10 );	// Right Long Range IR in Head
			SetDlgItemText( IDC_IR_HEAD_R, (LPCTSTR)strText );

			strText.Format( _T("%u"), (g_pFullSensorStatus->IR[3] - HEAD_IR_OFFSET_FROM_FRONT_TENTH_INCHES) /10 );	// Left Long Range IR in Head
			SetDlgItemText( IDC_IR_HEAD_L, (LPCTSTR)strText );




		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE

			// TODO-TURTLE

		///////////////////////////////////////////////////////////////////////////////
		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP

			// TODO-TURTLE

		///////////////////////////////////////////////////////////////////////////////
		#else
			#error BAD SENSOR_CONFIG_TYPE!  
		#endif
		///////////////////////////////////////////////////////////////////////////////

		// Odometer
		strText.Format( _T("%d"), (int)(g_pFullSensorStatus->OdometerTenthInches / 10.0) );
		SetDlgItemText( IDC_STAT_ODOMETER, (LPCTSTR)strText );


//		#if MOTOR_CONTROL_TYPE != ER1_MOTOR_CONTROL
			// Motor Tic Counts
			strText.Format( _T("%d"), g_pFullSensorStatus->Tachometer );
			SetDlgItemText( IDC_STAT_TACH, (LPCTSTR)strText );
//		#endif

		// Display approx heading direction
		// WARNING - THIS IS ACTUALLY DEGREES FROM WHERE KOBUKI WAS POINTING WHEN POWERED ON!
		SetDlgItemText( IDC_COMPASS_DIR, _T( DegreesToCompassRoseString(g_pFullSensorStatus->CompassHeading) ) );

		// Show range to closest object
		strText.Format( _T("%4.1f"), g_pNavSensorSummary->nFrontObjectDistance );
		SetDlgItemText( IDC_STAT_NEAREST_OBJECT_DISTANCE, (LPCTSTR)strText );

		// Show position of the Kinect sensor 
		SetDlgItemInt( IDC_STAT_KINECT_POSITION,
			(g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees) / 10);


		//printf( "I2C: C1=%03u CL=%04lu Calc=%4lu\r ", Compass1, gCompass, CalDeg );  
		// Sample calculations:
		//Right90 = (gCompass + 90)%360;
		//Left90  = (gCompass + 270)%360;
		//Rear    = (gCompass + 180)%360;
		//printf( "I2C: Fwd=%04lu R=%04lu L=%4lu, Rev=%4lu\r ", gCompass, Right90, Left90, Rear );  

		if( g_pFullSensorStatus->CompassHeading > 360 )
		{
			//TODO-CAR ROBOT_LOG( TRUE,   "ERROR!  Invalid Compass value: %d\n"), g_pFullSensorStatus->CompassHeading );
		}
	
		strText.Format( _T("%d"), g_pFullSensorStatus->CompassHeading );
		SetDlgItemText( IDC_STAT_COMPASS, (LPCTSTR)strText );

		//strText.Format( _T("%d"), g_pFullSensorStatus->CompassError );
		//SetDlgItemText( IDC_STAT_COMPASS_ERROR, (LPCTSTR)strText );

		// Video Processing Frame Rate
		//strText.Format( _T("%000.2f"), g_pFullSensorStatus->VideoFPS );
		//SetDlgItemText( IDC_STAT_FPS, (LPCTSTR)strText );



		///////////////////////////////////////////////////////////////////
		// LOKI ONLY
		#if (ROBOT_TYPE == LOKI)

		///////////////////////////////////////////////////////////////////
		// TURTLE ONLY
		#elif (ROBOT_TYPE == TURTLE) 

		///////////////////////////////////////////////////////////////////
		// CARBOT ONLY
		#elif (ROBOT_TYPE == CARBOT) 
		// Check status of the RC Kill Switch and Button2

		if( (g_pFullSensorStatus->StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
			strText.Format( _T("ON") );
		else
			strText.Format( _T("OFF") );
		SetDlgItemText( IDC_RC1_STATUS, (LPCTSTR)strText );

		if( (g_pFullSensorStatus->StatusFlags & HW_STATUS_RC_BUTTON_2) )
			strText.Format( _T("ON") );
		else
			strText.Format( _T("OFF") );
		SetDlgItemText( IDC_RC2_DISPLAY, (LPCTSTR)strText );



		#endif





	}
	else if( ROBOT_RESPONSE_GPS_DATA  == Item )
	{
		if(	g_pGPSData->dwCommandCount < 3)	// wait until we get some good data from the GPS
		{
			strText.Format( _T("None") );
			SetDlgItemText( IDC_STAT_GPS, (LPCTSTR)strText );
			strText.Format( _T(" ") );	// Blink Off
			SetDlgItemText( IDC_STATIC_GPS_BLINK, (LPCTSTR)strText );
		}
		else
		{
			// GPS Data ready.

			if( TRUE == m_GPSConnectionIndicator )
			{		
				strText.Format( _T("*") );	// Blink On
			}
			else 
			{
				strText.Format( _T(" ") );	// Blink Off
			}
			SetDlgItemText( IDC_STATIC_GPS_BLINK, (LPCTSTR)strText );
			m_GPSConnectionIndicator = !m_GPSConnectionIndicator;

			CString FixText, LatText, LongText;

			switch(g_pGPSData->btGSAFixMode)
			{
				case 1 : FixText = _T("None"); 
					break;
				case 2 : FixText = _T("2D"); 
					break;
				case 3 : FixText = _T("3D"); 
					break;
				default : FixText = _T("Err"); 
					break;
			}
			SetDlgItemText(IDC_STAT_GPS, FixText);

			// Latitude / Longitude / Altitude
			LatText.Format("%f", g_pGPSData->dGGALatitude);
			SetDlgItemText(IDC_STAT_GPS_LAT, LatText);

			LongText.Format("%f", g_pGPSData->dGGALongitude);
			SetDlgItemText(IDC_STAT_GPS_LONG, LongText);
			//strText.Format("%f", g_pGPSData->dGGAAltitude);
			//SetDlgItemText(IDC_STAT_GPS_ALT, strText);

			SetDlgItemInt(IDC_STAT_GPS_SATS_IN_VIEW, g_pGPSData->wGSVTotalNumSatsInView);

			// Log GPS data
			ROBOT_LOG( TRUE,  "GPS:  %s %d Sats  Lat:%s  Long:%s \n",
				FixText, g_pGPSData->wGSVTotalNumSatsInView, LatText, LongText )
		
			/**************** GPS TODO! **********************************
			// Update NMEA sentence count
			SetDlgItemInt(IDC_NMEA_RX_COUNT, g_pGPSData->dwCommandCount);

			// GGA GPS Quality
			switch(g_pGPSData->btGGAGPSQuality)
			{
				case 0 : strText = _T("Fix not available"); break;
				case 1 : strText = _T("GPS sps mode"); break;
				case 2 : strText = _T("Differential GPS, SPS mode, fix valid"); break;
				case 3 : strText = _T("GPS PPS mode, fix valid"); break;
				default : strText = _T("Unknown"); break;
			}
			SetDlgItemText(IDC_GPS_QUAL, strText);

			// GSA Fix mode and Dops
			switch(g_pGPSData->btGSAMode)
			{
				case 'M' : strText = _T("(Manual)"); break;
				case 'A' : strText = _T("(Automatic)"); break;
				default : strText = _T("(?)"); break;
			}
			SetDlgItemText(IDC_GSA_MODE,strText);

			strText.Format(_T("%.02f"), g_pGPSData->dGSAVDOP);
			SetDlgItemText(IDC_VDOP, strText);
			strText.Format(_T("%.02f"), g_pGPSData->dGSAHDOP);
			SetDlgItemText(IDC_HDOP, strText);
			strText.Format(_T("%.02f"), g_pGPSData->dGSAPDOP);
			SetDlgItemText(IDC_PDOP, strText);

			// GSA satellites used in solution
			strText = _T("");
			for(int i = 0; i < 12; i++)
			{
				strText2.Format("%02d ", g_pGPSData->wGSASatsInSolution[i]);
				strText += strText2;
			}
			SetDlgItemText(IDC_SATS_USED_IN_SOL, strText);

			// GSV signal quality/azimuth/elevation
			SetDlgItemInt(IDC_SATS_IN_VIEW, g_pGPSData->wGSVTotalNumSatsInView);
			strText = _T("");
			for(i = 0; i < g_pGPSData->wGSVTotalNumSatsInView; i++)
			{
				strText2.Format(_T("%d\t"), g_pGPSData->GSVSatInfo[i].m_wPRN);
				strText += strText2;
			}
			SetDlgItemText(IDC_GSV_PRN, strText);
			strText = _T("");
			for(i = 0; i < g_pGPSData->wGSVTotalNumSatsInView; i++)
			{
				strText2.Format(_T("%d\t"), g_pGPSData->GSVSatInfo[i].m_wSignalQuality);
				strText += strText2;
			}
			SetDlgItemText(IDC_GSV_SNR, strText);
			strText = _T("");
			for(i = 0; i < g_pGPSData->wGSVTotalNumSatsInView; i++)
			{
				strText2.Format(_T("%d\t"), g_pGPSData->GSVSatInfo[i].m_wAzimuth);
				strText += strText2;
			}
			SetDlgItemText(IDC_GSV_AZ, strText);
			strText = _T("");
			for(i = 0; i < g_pGPSData->wGSVTotalNumSatsInView; i++)
			{
				strText2.Format(_T("%d\t"), g_pGPSData->GSVSatInfo[i].m_wElevation);
				strText += strText2;
			}
			SetDlgItemText(IDC_GSV_ELV, strText);

			*************************************************************/
		}

	}
	else
	{
		// Error - unmapped message!
		ROBOT_LOG( TRUE,  "ERROR!  Unmapped Message 0x%02lX sent to OnRobotDisplayBulkItem!\n", Item )
	}
	__itt_task_end(pDomainGUIThread);

	return 0;
}

LRESULT CRobotCmdView::OnRobotDisplayOpenDataFile(WPARAM PathType, LPARAM MapType)
{
	LPCTSTR lpszFileName = DEFAULT_PATH_FILE;
	CString MsgString;

	if( PATH_TYPE_NONE != PathType )
	{
		// Only Default handled for now.  How to handle Blank Path type?

		if( NULL != AfxGetApp()->OpenDocumentFile(lpszFileName) )	//CWinApp::
		{
			MsgString.Format( "Path file %s opened", lpszFileName );
		}
		else
		{
			MsgString.Format( "\nERROR - Could not open Path file %s\n", lpszFileName );
		}
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}

	// Note: loading map will also load the robots last location that was saved with the map
	if( MAP_TYPE_NONE != MapType )
	{
		if( MAP_TYPE_UPSTAIRS == MapType )
		{
			lpszFileName = DEFAULT_UPSTAIRS_MAP_FILE;
		}
		else if( MAP_TYPE_DOWNSTAIRS == MapType )
		{
			lpszFileName = DEFAULT_DOWNSTAIRS_MAP_FILE;
		}
		else	// MAP_TYPE_BLANK
		{
			lpszFileName = DEFAULT_MAP_FILE;	// Blank Map
		}

		if( NULL != AfxGetApp()->OpenDocumentFile(lpszFileName) )	//CWinApp::
		{
			MsgString.Format( "Map file %s opened", lpszFileName );
		}
		else
		{
			MsgString.Format( "ERROR - Could not open Map file %s\n", lpszFileName );
		}
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}

	return 0;
}


__itt_string_handle* pshDisplaySingle = __itt_string_handle_create("GUI Display Single");
LRESULT CRobotCmdView::OnRobotDisplaySingleItem(WPARAM Item, LPARAM lParam)
{
	// This function displays various items from the module
	// This function is ONLY called by the GUI thread, in response to
	// WM_ROBOT_DISPLAY_SINGLE_ITEM
	// Item indicates the item to display.  lParam holds the value

	CString strText;
	__itt_task_begin(pDomainGUIThread, __itt_null, __itt_null, pshDisplaySingle);

	if( ROBOT_RESPONSE_DRIVE_MODULE_OWNER == Item )
	{
		CString ModuleName;
		UINT Module = lParam;	
		ModuleNumberToName( Module, ModuleName );

		SetDlgItemText( IDC_DRIVE_MODULE_OWNER, (LPCTSTR)ModuleName );
	}
	else if( ROBOT_RESPONSE_LASER_SCANNER_DATA == Item )
	{
			m_LaserDisplay.SetData();
	}
	else if( ROBOT_RESPONSE_KINECT_DATA == Item )
	{
			m_LaserDisplay.SetData();
	}
	else if( ROBOT_RESPONSE_COLOR_BLOB_CAL_RESULT == Item )
	{
		UINT ColorBlobCr = LOWORD(lParam);	
		UINT ColorBlobCb = HIWORD(lParam);
		strText.Format( _T("Cr = %d, Cb = %d"),  
				ColorBlobCr, ColorBlobCb );
		SetDlgItemText( IDC_STAT_COLOR_CAL_RESULT, (LPCTSTR)strText );
	}
	else if( ROBOT_RESPONSE_PIC_12V_POWER == Item )
	{
		if( lParam == TRUE ) strText = "ON";
		else strText = "OFF";
		SetDlgItemText( IDC_STAT_POWER, (LPCTSTR)strText );
	}
	else if( ROBOT_RESPONSE_PIC_LAST_ERROR == Item )
	{
		strText.Format( _T("%d"), lParam );
		SetDlgItemText( IDC_STAT_LAST_ERROR, (LPCTSTR)strText ); 
	}
	else if( ROBOT_RESPONSE_PIC_WATCHDOG == Item )
	{
		if( lParam == TRUE ) strText = "Expired";
		else strText = "OK";
		SetDlgItemText( IDC_STAT_WATCHDOG, (LPCTSTR)strText );
	}
	else if( ROBOT_RESPONSE_PIC_VERSION == Item )
	{
		/*	Reported on Power up, so no need to display in the GUI
		strText.Format( _T("%d"), lParam );
		SetDlgItemText( IDC_STAT_VERSION, (LPCTSTR)strText );
		*/
	}
	else if( ROBOT_RESPONSE_PIC_DEBUG1 == Item )
	{
		strText.Format( _T("%d"), lParam );
		SetDlgItemText( IDC_STAT_DEBUG, (LPCTSTR)strText );
	}
	else if( ROBOT_RESPONSE_MOTOR_SPEED == Item )
	{
			// Motor Distance update from ER1 Pilot Controller
			strText.Format( _T("%d"), lParam );
			SetDlgItemText( IDC_STAT_TACH, (LPCTSTR)strText );
	}
	else
	{
		// Error - unmapped message!
		ROBOT_LOG( TRUE,  "ERROR!  Unmapped Message 0x%02lX sent to OnRobotDisplayPICItem!\n", Item )
	}

	__itt_task_end(pDomainGUIThread);
	return 0;
}




void CRobotCmdView::OnUseJoystick() 
{
	//CButton* CheckBox = (CButton*)GetDlgItem( IDC_SPEED_SLIDER );
	//CSliderCtrl* SpeedSlider = (CSliderCtrl*)GetDlgItem( IDC_SPEED_SLIDER );
	//CSliderCtrl* TurnSlider = (CSliderCtrl*)GetDlgItem( IDC_TURN_SLIDER );

	// Set the sliders to middle (stop) position
	CString strText;
	m_nSpeedSlider = SLIDER_CENTER;
	m_nTurnSlider = SLIDER_CENTER;
	CSliderCtrl* pSlider;
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
	pSlider->SetPos(m_nSpeedSlider);
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_TURN_SLIDER);
	pSlider->SetPos(m_nTurnSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);
	SetDlgItemText(IDC_MOTOR_TURN_STATIC, strText);

/***2008
if( IsDlgButtonChecked(IDC_USE_JOYSTICK) )
	{
	    SelectJoyStick( g_RobotCmdViewHWND );
	}
	else
	{
	    UnSelectJoyStick( g_RobotCmdViewHWND );
	}
***/
}


void CRobotCmdView::OnCameraWin() 
{
	// TODO: Add your control notification handler code here
	
}
                                                                                                                    

void CRobotCmdView::OnLButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	
	CFormView::OnLButtonDown(nFlags, point);
}


void CRobotCmdView::OnMoveDistanceForward() 
{
	UpdateData(TRUE); // Get values from the GUI
	DWORD nTenthInches = 0;
//	DWORD Direction = FORWARD;
	CString strDistance = m_MoveDistance;
//	CString strDirection = m_MoveDirection;
	ROBOT_LOG( TRUE,  "Move Forward: %s\n", strDistance )


	if( strDistance == "6 inches" )	nTenthInches = 60;
	else if( strDistance == "1 foot" )		nTenthInches = 120;
	else if( strDistance == "2 feet" )		nTenthInches = 240;
	else if( strDistance == "3 feet" )		nTenthInches = 360;
	else if( strDistance == "4 feet" )		nTenthInches = 480;
	else if( strDistance == "10 feet" )		nTenthInches = 1200;
 	else			
	{
		ROBOT_LOG( TRUE,  "ERROR - UNKNOWN DISTANCE!\n" )
		return;
	}
	SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, nTenthInches, FORWARD );	// wParam = distance in TenthInches, lParam = direction
	
}

void CRobotCmdView::OnMoveDistanceReverse() 
{
	UpdateData(TRUE); // Get values from the GUI
	DWORD nTenthInches = 0;
	CString strDistance = m_MoveDistance;
	ROBOT_LOG( TRUE,  "Move Reverse: %s inches\n", strDistance )

	if( strDistance == "6 inches" )	nTenthInches = 60;
	else if( strDistance == "1 foot" )		nTenthInches = 120;
	else if( strDistance == "2 feet" )		nTenthInches = 240;
	else if( strDistance == "3 feet" )		nTenthInches = 360;
	else if( strDistance == "4 feet" )		nTenthInches = 480;
	else if( strDistance == "10 feet" )		nTenthInches = 1200;
 	else			
	{
		ROBOT_LOG( TRUE,   "ERROR - UNKNOWN DISTANCE!\n" )
		return;
	}
	SendCommand( WM_ROBOT_MOVE_SET_DISTANCE_CMD, nTenthInches, REVERSE );	// wParam = distance in TENTH INCHES, lParam = direction
}
void CRobotCmdView::OnTurnDistanceLeft() 
{
	UpdateData(TRUE); // Get values from the GUI
	DWORD nDegrees = 0;
	CString strAmount = m_TurnAmount;
	ROBOT_LOG( TRUE,   "Turn Left: %s \n", strAmount )

	if( strAmount == "5 deg" )	nDegrees = 5;
	else if( strAmount == "10 deg" )	nDegrees = 10;
	else if( strAmount == "22 deg" )	nDegrees = 22;
	else if( strAmount == "45 deg" )	nDegrees = 45;
	else if( strAmount == "90 deg" )	nDegrees = 90;
	else if( strAmount == "180 deg" )	nDegrees = 180;
 	else			
	{
		ROBOT_LOG( TRUE,   "ERROR - UNKNOWN DISTANCE!\n" )
		return;
	}
	SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, nDegrees, TURN_LEFT_MED );	// wParam = distance in degrees, lParam = direction and speed

 }

void CRobotCmdView::OnTurnDistanceRight() 
{

	UpdateData(TRUE); // Get values from the GUI
	DWORD nDegrees = 0;
	CString strAmount = m_TurnAmount;
	ROBOT_LOG( TRUE,   "Turn Right: %s \n", strAmount )

	if( strAmount == "5 deg" )	nDegrees = 5;
	else if( strAmount == "10 deg" )	nDegrees = 10;
	else if( strAmount == "22 deg" )	nDegrees = 22;
	else if( strAmount == "45 deg" )	nDegrees = 45;
	else if( strAmount == "90 deg" )	nDegrees = 90;
	else if( strAmount == "180 deg" )	nDegrees = 180;
 	else			
	{
		ROBOT_LOG( TRUE,   "ERROR - UNKNOWN DISTANCE!\n" )
		return;
	}
	SendCommand( WM_ROBOT_TURN_SET_DISTANCE_CMD, nDegrees, TURN_RIGHT_MED );	// wParam = distance in degrees, lParam = direction and speed
}




void CRobotCmdView::OnOpenDefaultPath() 
{
	// Tell the Server to open the default path file
	SendCommand( WM_ROBOT_OPEN_DATA_FILE, PATH_TYPE_DEFAULT, MAP_TYPE_BLANK );

	// Open a local copy too, so the client map will show the path
	OnRobotDisplayOpenDataFile( PATH_TYPE_DEFAULT, MAP_TYPE_BLANK );
}


void CRobotCmdView::OnOpenDefaultMap() 
{
	LPCTSTR lpszFileName = DEFAULT_MAP_FILE; // Blank Map
	if( NULL != AfxGetApp()->OpenDocumentFile(lpszFileName) )
	{
		ROBOT_LOG( TRUE,  "Map file opened\n" )
	}
	else
	{
		ROBOT_LOG( TRUE,  "ERROR - Could not open default Map file \n" )
	}
	
}

void CRobotCmdView::OnSelCmdViewBtn() 
{
	GetParentFrame()->ActivateFrame();	//RecalcLayout();
//    ResizeParentToFit( FALSE );	// Force Parent to grow!
//	nCmdShow = SW_SHOWMAXIMIZED;
//	CMDIChildWnd::ActivateFrame(nCmdShow);
	
}
void CRobotCmdView::OnSelMapViewBtn() 
{
	::PostMessage( g_RobotMapViewHWND, (UINT)WM_COMMAND, ID_SEL_MAP_VIEW_BTN, 0);	
}
void CRobotCmdView::OnSelPathViewBtn() 
{
	::PostMessage( g_RobotPathViewHWND, (UINT)WM_COMMAND, ID_SEL_PATH_VIEW_BTN, 0);	
}
void CRobotCmdView::OnSelSetupViewBtn() 
{
	::PostMessage( g_RobotSetupViewHWND, (UINT)WM_COMMAND, ID_SEL_SETUP_VIEW_BTN, 0);	
}



void CRobotCmdView::OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
}

void CRobotCmdView::OnUpdateSelMapViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotMapViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CRobotCmdView::OnUpdateSelPathViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotPathViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CRobotCmdView::OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotSetupViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}


/////////////////////////////////////////////////////////////////////////////
// Camera Pan/Tilt/Zoom Command Handlers
//

void CRobotCmdView::OnCamStop() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_STOP, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamUp() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_UP, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamLeft() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_LEFT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamDown() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_DOWN, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamRight() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_RIGHT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamUpLeft() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_UP_LEFT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamUpRight() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_UP_RIGHT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamDownLeft() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_DOWN_LEFT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamDownRight() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_DOWN_RIGHT, m_CameraPanSpeed );
}

void CRobotCmdView::OnCamCenter() // Send Absolute Centering commands
{
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, m_CameraPanSpeed );
	SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_CENTER, m_CameraPanSpeed );
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_HEAD_TILT);
	pComboBox->SetCurSel(0);	// Set the combo box back to first item
}

void CRobotCmdView::OnZoomOut() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_ZOOM_CMD, (DWORD)CAMERA_ZOOM_OUT, m_CameraZoomSpeed );
}

void CRobotCmdView::OnZoomIn() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_ZOOM_CMD, (DWORD)CAMERA_ZOOM_IN, m_CameraZoomSpeed );
}

void CRobotCmdView::OnZoomStop() 
{
	SendCommand( WM_ROBOT_USER_CAMERA_ZOOM_CMD, (DWORD)CAMERA_ZOOM_STOP, m_CameraZoomSpeed );
}

void CRobotCmdView::OnSelchangeHeadTilt() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_HEAD_TILT);
	int nMode = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE,   "Requesting New Head Tilt: %d\n", nMode )

	switch( nMode ) 
	{
		case 0:
			SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_CENTER, m_CameraPanSpeed );
		break;

		case 1:
			SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT, m_CameraPanSpeed );
		break;

		case 2:
			SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_TENTHDEGREES_TILT_RIGHT, m_CameraPanSpeed );
		break;

		default:
			ROBOT_LOG( TRUE,  "LOGIC ERROR! Bad OnSelchangeHeadTilt\n" )
	}

}


//////////////////////////////////////////////////////////////////////

//void CRobotCmdView::OnBrakeButton() 
//{
//	// Center the TurnSlider Control and text box
//	CString strText;
//	CSliderCtrl* pSlider =
//		(CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
//	m_nSpeedSlider = SLIDER_CENTER;
//	pSlider->SetPos(m_nSpeedSlider);
//	strText.Format("0");
//	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);
//	// Send the command to the motor control
//	g_MotorCurrentSpeedCmd = 0;	// Stop
//	g_MotorCurrentTurnCmd = 0;	// Center
//	// Special Case the BRAKE. don't wait for nothing!
//	SendCommand( WM_ROBOT_BRAKE_CMD, 0, 0 );
//	OnPausePath();	// Pause current path that is running!
//	OnCenterButton();	// Center the wheels on Stop command
//}


void CRobotCmdView::OnExecutePath() 
{
	// Start Following the Nav Path! (not the Grid Path (GRID_PATH_EXECUTE_START), which is started from the Map View)
//	ROBOT_DISPLAY( TRUE, "Executing Path" )
	SendCommand( WM_ROBOT_EXECUTE_PATH, 
		NAV_PATH_EXECUTE_START,		// wParam
		1 );	// lParam: 1 = Wait for bumper to start, 0 = Don't wait
}

void CRobotCmdView::OnCancelPath() 
{
	// Cancel the Path!
//	ROBOT_DISPLAY( TRUE, "Path Cancel" )
	SendCommand( WM_ROBOT_EXECUTE_PATH, 
		PATH_EXECUTE_CANCEL,	// wParam
		1 );	// lParam: 1 = Wait for bumper to start, 0 = Don't wait
}

void CRobotCmdView::OnPausePath() 
{
	SendCommand( WM_ROBOT_EXECUTE_PATH, 
		PATH_EXECUTE_PAUSE,	// wParam
		1 );	// lParam: 1 = Wait for bumper to start, 0 = Don't wait
}

void CRobotCmdView::OnResumePath() 
{
	SendCommand( WM_ROBOT_EXECUTE_PATH, 
		PATH_EXECUTE_RESUME,	// wParam
		1 );	// lParam: 1 = Wait for bumper to start, 0 = Don't wait
}


void CRobotCmdView::OnCbHighGear() 
{
	if( IsDlgButtonChecked(IDC_CB_HIGH_GEAR) )
	{
		SendCommand( WM_ROBOT_SET_GEAR_CMD, 0, 1 );	// 1 = High Gear
	}
	else
	{
		SendCommand( WM_ROBOT_SET_GEAR_CMD, 0, 0 );	// 0 = Low Gear
	}
}

void CRobotCmdView::OnTestBrake() 
{
	// Center the TurnSlider Control and text box
	CString strText;
	CSliderCtrl* pSlider =
		(CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
	m_nSpeedSlider = SLIDER_CENTER;
	pSlider->SetPos(m_nSpeedSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);
	// Send the command to the motor control
	g_MotorCurrentSpeedCmd = 0;	// Stop
	g_MotorCurrentTurnCmd = 0;	// Center
	// Special Case the BRAKE. don't wait for nothing!
	SendCommand( WM_ROBOT_BRAKE_CMD, 0, 1 );				// /DEBUG Test Mode!!!!!
	OnPausePath();	// Pause current path that is running!
}


void CRobotCmdView::OnColorTrackCalibrate() 
{
	// Send comand to calibrate Color Blob Tracking to color currently in center of camera view
	SendCommand( WM_ROBOT_COLOR_BLOB_AUTO_CAL_CMD, 0, 0 );
}

void CRobotCmdView::OnColorTrackSearch() 
{


	if( !m_bTrackColorSearching )
	{
		// Not currently searching, start the search
		m_bTrackColorSearching = TRUE;
		SetDlgItemText(IDC_COLOR_TRACK_SEARCH_BTN, "Stop");	// Toggle the button

		SendCommand( WM_ROBOT_COLOR_BLOB_SEARCH_CMD, TRUE, 0 );		// wParam = Start

	}
	else
	{
		// Currently searching
		m_bTrackColorSearching = FALSE;
		SetDlgItemText(IDC_COLOR_TRACK_SEARCH_BTN, "Search");	// Toggle the button

		SendCommand( WM_ROBOT_COLOR_BLOB_SEARCH_CMD, FALSE, 0 );	// wParam = Stop
	}

}


void CRobotCmdView::OnEnableCollisionModule() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_COLLISION_MODULE) )
	{
		SendCommand( WM_ROBOT_ENABLE_COLLISION_MODULE, 1, 0 );	// Enable
	}
	else
	{
		SendCommand( WM_ROBOT_ENABLE_COLLISION_MODULE, 0, 0 );	// Disable
	}
	
}

void CRobotCmdView::OnEnableAvoidanceModule() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_AVOIDANCE_MODULE) )
	{
		SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 1, 0 );	// Enable
	}
	else
	{
		SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 0, 0 );	// Disable
	}
	
}

void CRobotCmdView::OnEnableGpsPath() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_GPS_PATH) )
	{
		SendCommand( WM_ROBOT_ENABLE_GPS_PATH, 1, 0 );	// Enable
	}
	else
	{
		SendCommand( WM_ROBOT_ENABLE_GPS_PATH, 0, 0 );	// Disable
	}

}

void CRobotCmdView::OnEnableObjectNavUpdate() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_OBJECT_NAV_UPDATE) )
	{
		SendCommand( WM_ROBOT_ENABLE_OBJECT_NAV_UPDATE, 1, 0 );	// Enable
	}
	else
	{
		SendCommand( WM_ROBOT_ENABLE_OBJECT_NAV_UPDATE, 0, 0 );	// Disable
	}
}



void CRobotCmdView::OnChangeCameraManualColorCaldata() 
{
	// TODO: If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CFormView::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.
	
	// TODO: Add your control notification handler code here
	
}


void CRobotCmdView::OnSelchangeBehaviorMode() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_BEHAVIOR_MODE);
	int nMode = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE,   "Requesting New Behavior Mode: %d\n", nMode )

	SendCommand( WM_ROBOT_SET_BEHAVIOR_CMD, (DWORD)nMode, 0 );

	// For this combo box, we do NOT reset combo box to default
	// Behaviors are "sticky"
	// pComboBox->SetCurSel(0);	// Set the combo box to first item

}

void CRobotCmdView::OnSelchangeActionMode() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_ACTION_MODE);
	int nMode = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE,   "Requesting New Action Mode: %d\n", nMode )


	// implemented in menu:
	// Ready;Pickup Close;Pickup Any;Photo;Follow;Danger;Karate Demo;
	/* Defined in RobotSharedParams.h:

		ACTION_MODE_NONE = 0,						// No Action mode pending
		ACTION_MODE_PICKUP_CLOSE_OBJECT,			// Look for and pickup nearest object that is within reach of the robot
		ACTION_MODE_PICKUP_OBJECTS,					// Look for and pickup object anywhere in front of robot
		ACTION_MODE_TAKE_PHOTO,						//
		ACTION_MODE_FOLLOW_PERSON,					//
		ACTION_MODE_FREAK_OUT,						//
		ACTION_MODE_KARATE_DEMO,					//
	*/

	SendCommand( WM_ROBOT_SET_ACTION_CMD, (DWORD)nMode, 1 );	// TRUE = start mode
	pComboBox->SetCurSel(0);	// Reset the combo box, so the same command can be repeated
	UpdateData(FALSE); // Get send to GUI
}

void CRobotCmdView::OnEnableLedEyes() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_LED_EYES) )
	{
		// Eyes are enabled
		if( IsDlgButtonChecked(IDC_LED_EYES_BLINK) )
		{
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 );
		}
		else
		{
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_OPEN, 0 );
		}
	
	}
	else
	{
		SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_CLOSE, 0 );
	}
}

void CRobotCmdView::OnEnableCameraLights() 
{
	//TODO - FIX THIS TEMP KLUDGE!
	if( IsDlgButtonChecked(IDC_ENABLE_CAMERA_LIGHTS) )
	{
		SendCommand( WM_ROBOT_IR_SENSOR_POWER_CMD, 0, (DWORD)TRUE );
//		SendCommand( WM_ROBOT_LIGHT_POWER_CMD, 0, (DWORD)TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_IR_SENSOR_POWER_CMD, 0, (DWORD)FALSE );
//		SendCommand( WM_ROBOT_LIGHT_POWER_CMD, 0, (DWORD)FALSE );
	}
}

void CRobotCmdView::OnEnableAuxLights() 
{
	if( IsDlgButtonChecked(IDC_ENABLE_AUX_LIGHTS) )
	{
		SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_AUX_LIGHT_POWER_CMD, 0, (DWORD)FALSE );
	}
}

void CRobotCmdView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags) 
{
	ROBOT_LOG( TRUE,  "KEY PRESSED: %d, RepCnt: %u, Flags: %02X\n", nChar, nRepCnt, nFlags)
	
	CFormView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CRobotCmdView::OnLedEyesBlink() 
{

	if( IsDlgButtonChecked(IDC_ENABLE_LED_EYES) )
	{
		// Eyes are enabled
		if( IsDlgButtonChecked(IDC_LED_EYES_BLINK) )
		{
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_BLINK, 0 );
		}
		else
		{
			SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_OPEN, 0 );
		}
	}
	else
	{
		SendCommand( WM_ROBOT_SET_LED_EYES_CMD, (DWORD)LED_EYES_OFF, 0 );
	}
	
}


//void CRobotCmdView::OnOpenBlankMap() 
//{
//	// Tell the Server to open the default path file
//	SendCommand( WM_ROBOT_OPEN_DATA_FILE, PATH_TYPE_NONE, MAP_TYPE_BLANK );
//
//	// Open a local copy too, so the client map will show the path
//	OnRobotDisplayOpenDataFile( PATH_TYPE_NONE, MAP_TYPE_BLANK );
//
//}

void CRobotCmdView::OnOpenUpstairsMap() 
{
	// Tell the Server to open the default path file
	SendCommand( WM_ROBOT_OPEN_DATA_FILE, PATH_TYPE_NONE, MAP_TYPE_UPSTAIRS );

	// Open a local copy too, so the client map will show the path
	OnRobotDisplayOpenDataFile( PATH_TYPE_NONE, MAP_TYPE_UPSTAIRS );

}

void CRobotCmdView::OnOpenDownstairsMap() 
{
	// Tell the Server to open the default path file
	SendCommand( WM_ROBOT_OPEN_DATA_FILE, PATH_TYPE_NONE, MAP_TYPE_DOWNSTAIRS );

	// Open a local copy too, so the client map will show the path
	OnRobotDisplayOpenDataFile( PATH_TYPE_NONE, MAP_TYPE_DOWNSTAIRS );

}



void CRobotCmdView::OnSelchangePlayMusic() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_PLAY_MUSIC);
	int nMusic = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE,   "Requesting Music: %d\n", nMusic )

	SendCommand( WM_ROBOT_PLAY_MUSIC, (DWORD)nMusic, 0 );
	// Reset the combo box to default
	pComboBox->SetCurSel(0);	// Set the combo box to first item

}


void CRobotCmdView::OnStopPanicStop() 
{
	// TEST: Pause Mode
	g_GlobalPause = !g_GlobalPause;

	CButton* Button = (CButton*)GetDlgItem( IDC_STOP_PANIC_STOP );
	if( g_GlobalPause )
	{
		Button->SetWindowText("Cont");	// Next press of the button will reset the Panic
		ROBOT_LOG( TRUE,   "********> ROBOT PAUSED! - All motors inhibited\n" )

	}
	else
	{
		Button->SetWindowText("Pause");	// Next press of the button will reset the Panic
		ROBOT_LOG( TRUE,   "********> ROBOT UNPAUSED! - Resuming Operation\n" )
	}

/*
	// Panic Stop will stop motors, and override other modules!
	//static BOOL ButtonState = 0;

	// Prevent conflicting commands to the motor control
	g_MotorCurrentSpeedCmd = 0;	// Stop
	g_MotorCurrentTurnCmd = 0;	// Center
	// Note - DO NOT send directly to the motor thread. Sometimes command going
	// to Arduino, sometimes to ER1 or Servo controller.
	SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );

	// Update the GUI
	CheckDlgButton(IDC_OVERRIDE_MODE, 1);

	// Center the SpeedSlider Control and text box
	CString strText;
	CSliderCtrl* pSlider = (CSliderCtrl*) GetDlgItem(IDC_SPEED_SLIDER);
	m_nSpeedSlider = SLIDER_CENTER;
	pSlider->SetPos(m_nSpeedSlider);
	strText.Format("0");
	SetDlgItemText(IDC_MOTOR_SPEED_STATIC, strText);


	//OnPausePath();	// Pause current path that is running!
	OnCenterButton();	// Center the wheels on Stop command

	// User pushing Panic will turn off Collision and Avoidance modules.
	//CheckDlgButton(IDC_ENABLE_COLLISION_MODULE, 0);
	//CheckDlgButton(IDC_ENABLE_AVOIDANCE_MODULE, 0);
	//SendCommand( WM_ROBOT_ENABLE_COLLISION_MODULE, 0, 0 );	// Disable
	//SendCommand( WM_ROBOT_ENABLE_AVOIDANCE_MODULE, 0, 0 );	// Disable

	//Button->SetWindowText("Reset");	// Next press of the button will reset the Panic
	//ButtonState = !ButtonState;
	//CButton* Button = (CButton*)GetDlgItem( IDC_STOP_PANIC_STOP );
	*/

}


void CRobotCmdView::OnVscrollTextToSpeak() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members

	int StrLen = m_TextToSpeak.GetLength();
	if( StrLen < 3)
	{
		// Just a Return/Linefeed with no data.  Clear out the CR/LF
		m_TextToSpeak.Empty();
		UpdateData(FALSE);	// Force data exchange data members back to GUI
	}
	else
	{
		// Remove the CR/LF from the string while copying to the global string 
		// for sending to other thread or to the server

		g_ClientTextToSend = _T( m_TextToSpeak.Left(StrLen-2) );	

		ROBOT_LOG( TRUE,  "Final size = %d\n", g_ClientTextToSend.GetLength() )
		ROBOT_LOG( TRUE,  "Sending Text To Server: [%s]\n", g_ClientTextToSend)
		m_TextToSpeak.Empty();
		UpdateData(FALSE);	// Force data exchange data members back to GUI


		if( m_SendTextToAI )
		{
			SendCommand( WM_ROBOT_TEXT_MESSAGE_TO_SERVER, WM_ROBOT_TEXT_TO_AI, 0 );
		}
		else
		{
			SendCommand( WM_ROBOT_TEXT_MESSAGE_TO_SERVER, WM_ROBOT_SPEAK_TEXT, SPEAK_TEXT_FROM_BUFFER );
		}
	}

}

/*
void CRobotCmdView::OnPlaySound() 
{

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	if( 0 == m_TextToSpeak.GetLength() )
	{
		// Temp KLUDGE - Do Darth Demo if no text entered!
		//SendCommand( WM_ROBOT_PLAY_SOUND, SOUND_DARTH_VADER, 0 );
		SendCommand( WM_ROBOT_TEXT_MESSAGE_TO_SERVER, WM_ROBOT_PLAY_SOUND, SOUND_DARTH_VADER );
	}
	else
	{
		// Copy to global string for playing
		// or, if this is the client, for sending to the server
		g_ClientTextToSend = _T(m_TextToSpeak);	
		SendCommand( WM_ROBOT_TEXT_MESSAGE_TO_SERVER, WM_ROBOT_SPEAK_TEXT, SPEAK_TEXT_FROM_BUFFER );
	}
}
*/


void CRobotCmdView::OnQ_Key() 
{
	if( 'Q' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'Q';

	ROBOT_LOG( TRUE,  "KEY Curve Left\n" )
	g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
	g_MotorCurrentTurnCmd = -(g_SpeedSetByKeyboard/3);
	if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
	{
		g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
	}	
}

void CRobotCmdView::OnKey_E() 
{
	if( 'E' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'E';

	ROBOT_LOG( TRUE,  "KEY Curve Right\n" )
	g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
	g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard/3);
	if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
	{
		g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
	}
}

void CRobotCmdView::OnKEY_A() 
{
	if( 'A' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'A';

	ROBOT_LOG( TRUE,  "KEY Spin Left\n" )
	g_MotorCurrentSpeedCmd = 0;
	g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard * -70) / 100; // n%.  note negative value
	if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
	{
		g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
	}
}

void CRobotCmdView::OnKey_D() 
{
	if( 'D' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'D';

	ROBOT_LOG( TRUE,  "KEY Spin Right\n" )
	g_MotorCurrentSpeedCmd = 0;
	g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard * 70) / 100;	// n%
	if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
	{
		g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
	}
}



void CRobotCmdView::OnKey_S() 
{
/*
if( 'S' == g_LastKey )
	{
		return;	// ignore repeated key
	}
	g_LastKey = 'S';
*/
	ROBOT_LOG( TRUE,  "KEY Stop\n" )
	g_MotorCurrentSpeedCmd = SPEED_STOP;
	g_MotorCurrentTurnCmd = 0;	// Center

	// When user presses Stop, supress other modules, so they don't keep robot moving!
//	SendCommand( WM_ROBOT_SUPPRESS_MODULE, 
//		AVOID_OBJECT_MODULE, SUPPRESS ); // Modules, Suppress/UnSupress
	SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );
	SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_PAUSE,	1 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait

}

void CRobotCmdView::OnKey_W() 
{
	if( 'W' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'W';
	ROBOT_LOG( TRUE,  "KEY Forward\n" )
	g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
	g_MotorCurrentTurnCmd = 0;	// Center
	
}

void CRobotCmdView::OnKey_X() 
{
	if( 'X' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'X';

	ROBOT_LOG( TRUE,  "KEY Backup\n" )
	g_MotorCurrentSpeedCmd = SPEED_REV_MED_SLOW;	// Keyboard Speed ignored by backup

}


void CRobotCmdView::OnKey_Plus() 
{
	// + = Increase Speed
	g_SpeedSetByKeyboard += 5;
	if( g_SpeedSetByKeyboard > SPEED_FULL_FWD ) g_SpeedSetByKeyboard = SPEED_FULL_FWD;
	CString MsgString;
	MsgString.Format("Speed Increased to %d\n", g_SpeedSetByKeyboard);
	ROBOT_DISPLAY( TRUE, (MsgString))
}

void CRobotCmdView::OnKey_Minus() 
{
	// - = Decrease Speed
	g_SpeedSetByKeyboard -= 5;
	if( g_SpeedSetByKeyboard < 0 ) g_SpeedSetByKeyboard = 0;
	CString MsgString;
	MsgString.Format("Speed Decreased to %d\n", g_SpeedSetByKeyboard);
	ROBOT_DISPLAY( TRUE, (MsgString))
}

void CRobotCmdView::OnKey_Zero() 
{
	// 0 = Default Speed
	g_SpeedSetByKeyboard = SPEED_FWD_MED_SLOW;
	CString MsgString;
	MsgString.Format("Speed set to Default (%d)\n", SPEED_FWD_MED_SLOW);
	ROBOT_DISPLAY( TRUE, (MsgString))
}


void CRobotCmdView::OnKey_C() 
{
	if( 'C' == g_LastKey )
	{
		//return;	// ignore repeated key
	}
	g_LastKey = 'C';

	ROBOT_LOG( TRUE,  "KEY Continue path\n" )
	g_MotorCurrentSpeedCmd = 0;
	g_MotorCurrentTurnCmd = g_SpeedSetByKeyboard;
	if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
	{
		g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
	}
	//SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );
	SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_RESUME,	0 );

}

void CRobotCmdView::OnArmMovementDlg() 
{
}

/*void CRobotCmdView::OnArmControlDlg() 
{

	// Launch the modeless dialog

	// Create the dialog if not created already
	if( m_pArmControlDlg->GetSafeHwnd() == 0 )
	{
		BOOL result = m_pArmControlDlg->Create();	// Display the dialog window
	}
	
}
*/

void CRobotCmdView::OnTakeSnapshot() 
{
	SendCommand( WM_ROBOT_CAMERA_TAKE_SNAPSHOT, 0, 0 );	
}

void CRobotCmdView::OnRecordVideo() 
{
	SendCommand( WM_ROBOT_CAMERA_RECORD_VIDEO, TRUE, 0 );	//
}

void CRobotCmdView::OnRecordVideoStop() 
{
	SendCommand( WM_ROBOT_CAMERA_RECORD_VIDEO, FALSE, 0 );	// 
}



void CRobotCmdView::OnCbnSelchangeLaserZoom()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_LASER_ZOOM);
	int nZoom = pComboBox->GetCurSel()+1;
	ROBOT_LOG( TRUE,   "Requesting New Laser Zoom Mode: %d\n", nZoom )
	m_LaserDisplay.SetWindowZoom( nZoom );

}

void CRobotCmdView::OnBnClickedLaserScanEnable()
{
	// Enable to Global timer to send periodic requests for laser scans
	g_bLaserContinuousScanEnabled = IsDlgButtonChecked(IDC_LASER_SCAN_ENABLE);
	if( !IsDlgButtonChecked(IDC_LASER_SCAN_ENABLE) )
	{
		// When laser disabled, clear last time stamp, so modules know not to use Laser data
		g_pLaserSummary->SampleTimeStamp = 0;
	}
}

void CRobotCmdView::OnBnClickedKinectUp()
{
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees += 50;	// Move in 5 degree increments
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update = TRUE;
	SendCommand( WM_ROBOT_SET_KINECT_POSITION, 0, FALSE );	// Set Position but not speed

}

void CRobotCmdView::OnBnClickedKinectDown()
{
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees -= 50;	// Move in 5 degree increments
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update = TRUE;
	SendCommand( WM_ROBOT_SET_KINECT_POSITION, 0, FALSE );	// Set Position but not speed
}

void CRobotCmdView::OnStnClickedStatIrAdVertFrontLeft()
{
	// TODO: Add your control notification handler code here
}

//void CRobotCmdView::OnStnClickedPicStatus()
//{
//	// TODO: Add your control notification handler code here
//}

void CRobotCmdView::OnStnClickedPicStatus()
{
	// TODO: Add your control notification handler code here
}


void CRobotCmdView::OnBnClickedLocalUser()
{
	// True = all commands issued as LOCAL user, otherwise as Remote user.  
	// Allow testing Local/Remote behavior over Remote Desktop
	m_LocalUser = IsDlgButtonChecked(IDC_LOCAL_USER_CB);
	if( m_LocalUser )
	{
		SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL, 0 );
	}
	else
	{
		SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_REMOTE, 0 );
	}

}

void CRobotCmdView::OnBnClickedKinectPwrEnable()
{
	/*** USE SETUP VERSION INSTEAD! ***
	if( IsDlgButtonChecked(IDC_KINECT_PWR_ENABLE) )
	{
		// Enable Kinect Power
		SendCommand( WM_ROBOT_KINECT_POWER_ENABLE_CMD, 0, (DWORD)TRUE );	
	}
	else
	{
		SendCommand( WM_ROBOT_KINECT_POWER_ENABLE_CMD, 0, (DWORD)FALSE );	
	}
	***/
}


HBRUSH CRobotCmdView::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{

	int CtrlID = pWnd->GetDlgCtrlID();
	int Status = SUBSYSTEM_FAILED;

	// For specific controls, change background color according to status
	switch( CtrlID )
	{
		case IDC_PIC_STATUS:
		{
			if ( (ROBOT_TYPE == LOKI) && (1 == ROBOT_SIMULATION_MODE) )
			{
				// Indicate Simulation Mode
 				pDC->SetBkColor(BACKGROUND_COLOR_YELLOW);
 				return (HBRUSH)(m_YellowBrush->GetSafeHandle());
			}
			else
			{
				Status = g_ArduinoSubSystemStatus;
				break;
			}
		}

		case IDC_BATTERY_WARNING_TEXT:
		{


			if( URGENT == m_BatteryStatus )
			{
 				pDC->SetBkColor(BACKGROUND_COLOR_RED);
 				return (HBRUSH)(m_RedBrush->GetSafeHandle());
			}
			else if( LOW == m_BatteryStatus )
			{
 				pDC->SetBkColor(BACKGROUND_COLOR_YELLOW);
 				return (HBRUSH)(m_YellowBrush->GetSafeHandle());
			}
			else if( OK == m_BatteryStatus )
			{
 				pDC->SetBkColor(BACKGROUND_COLOR_GREEN);
 				return (HBRUSH)(m_GreenBrush->GetSafeHandle());
			}
			else
			{	
				// status not set yet. just return normal theme color
				return CFormView::OnCtlColor(pDC, pWnd, nCtlColor);
			}			
			break;
		}

		case IDC_DYNA_STATUS:
		{	
			Status = g_DynaSubSystemStatus;
			break;
		}
		case IDC_MOTOR_STATUS:
		{	
			Status = g_MotorSubSystemStatus;
			break;
		}
		case IDC_KINECT_STATUS:
		{	
			Status = g_KinectSubSystemStatus;
			break;
		}


#if( ROBOT_TYPE == LOKI )
		case IDC_RX64_STATUS:
		{	
			Status = g_RX64SubSystemStatus;
			break;
		}
		case IDC_KERR_STATUS:
		{	
			Status = g_KerrSubSystemStatus;
			break;
		}
		case IDC_CAMERA_STATUS:
		{	
			Status = g_CameraSubSystemStatus;
			break;
		}
		case IDC_LASER_STATUS:
		{	
			Status = g_LaserSubSystemStatus;
			break;
		}
		case IDC_ARM_L_STATUS:
		{	
			Status = g_LeftArmSubSystemStatus;
			break;
		}
		case IDC_ARM_R_STATUS:
		{	
			Status = g_RightArmSubSystemStatus;
			break;
		}
#elif( ROBOT_TYPE == TURTLE )
		// Note, Left and Right wheel drops not use; wheels bolted in place for stability
		/*
		case IDC_LEFT_WHEEL_DROP:
		{	
			if( !g_pIRobotStatus->WheelDropLeft )
				Status = SUBSYSTEM_CONNECTED;
			break;
		}
		case IDC_RIGHT_WHEEL_DROP:
		{	
			if( !g_pIRobotStatus->WheelDropRight )
				Status = SUBSYSTEM_CONNECTED;
			break;
		}
		*/
		case IDC_CASTER_DROP:
		{	
			if( !g_pIRobotStatus->WheelDropCaster )
				Status = SUBSYSTEM_CONNECTED;
			break;
		}
		case IDC_STAT_BMPR_LEFT:
		{	
			if( !g_pIRobotStatus->BumperLeft )
				Status = SUBSYSTEM_CONNECTED;
			break;
		}
		case IDC_STAT_BMPR_RIGHT:
		{	
			if( !g_pIRobotStatus->BumperRight )
				Status = SUBSYSTEM_CONNECTED;
			break;
		}
#endif

		default:
		{	// For other controls, just return normal theme color
			return CFormView::OnCtlColor(pDC, pWnd, nCtlColor);
		}
	}	

	if( SUBSYSTEM_CONNECTED == Status )
	{
 		//pDC->SetTextColor(RGB(0, 0, 0));
 		pDC->SetBkColor(BACKGROUND_COLOR_GREEN);
 		return (HBRUSH)(m_GreenBrush->GetSafeHandle());
	}
	else if( SUBSYSTEM_WAITING == Status )
	{
 		//pDC->SetTextColor(RGB(0, 0, 0));
 		pDC->SetBkColor(BACKGROUND_COLOR_YELLOW);
 		return (HBRUSH)(m_YellowBrush->GetSafeHandle());
	}
	else if( SUBSYSTEM_DISABLED == Status )
	{
 		//pDC->SetTextColor(RGB(0, 0, 0));
 		pDC->SetBkColor(BACKGROUND_COLOR_BLUE);
 		return (HBRUSH)(m_BlueBrush->GetSafeHandle());
	}
	else if( SUBSYSTEM_FAILED == Status )
	{
 		//pDC->SetTextColor(RGB(0, 0, 0));
 		pDC->SetBkColor(BACKGROUND_COLOR_RED);
 		return (HBRUSH)(m_RedBrush->GetSafeHandle());
	}
	else
	{
		ROBOT_ASSERT(0); // logic error
	}
	return CFormView::OnCtlColor(pDC, pWnd, nCtlColor);
}




void CRobotCmdView::OnBnClickedCaptureFace()
{
	// TODO: Add your control notification handler code here
		UpdateData(TRUE);	// Force data exchange from GUI to data members
		g_FaceCaptureName = FaceCaptureName;
		SendCommand( WM_ROBOT_CAMERA_CAPTURE_FACE, 0, 0 );
}


void CRobotCmdView::OnStnClickedIrBumperArmLFingerR()
{
	// TODO: Add your control notification handler code here
}

