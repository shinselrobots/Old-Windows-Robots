// SetupView.cpp : implementation file

#include "stdafx.h"
#include "ClientOrServer.h"
#include "RobotConfig.h"
#include "Globals.h"
//#include "resource.h"
#include "Robot.h"
#include "Module.h"
#include "..\Common\HardwareCmds.h"
#include "RobotSharedParams.h"

#if ( ROBOT_SERVER == 1 )
#include "HWInterface.h"
#include "RobotServerSock.h"
#else
#include "RobotClientSock.h"
#endif

#include "SetupDoc.h"
#include "SetupView.h"
//#include "RobotSpeech.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define KEY_ARROW_UP	38
#define KEY_ARROW_DOWN	40
#define KEY_ARROW_LEFT	37
#define KEY_ARROW_RIGHT	39
#define KEY_ESC			27
#define KEY_RETURN		13

#define KEY_PLUS		187
#define KEY_MINUS		189
#define KEY_ZERO		1


BOOL CALLBACK EnumWindowCallBack( HWND hwnd, LPARAM lParam );
FIND_WINDOW_HANDLE_STRUCT_T FWHS;	// Global to this module


BOOL CALLBACK EnumWindowCallBack(HWND hwnd, LPARAM lParam) 
{ 
	FIND_WINDOW_HANDLE_STRUCT_T * pFWHS = (FIND_WINDOW_HANDLE_STRUCT_T * )lParam; 
	DWORD FoundProcessId; 
	CString Title;
	CString ProgramToFind;
	ProgramToFind = "iTunes";

	GetWindowThreadProcessId ( hwnd, &FoundProcessId ); 

	// note: In order to make sure we have the MainFrame, verify that the title 
	// has Zero-Length. Under Windows 98, sometimes the Client Window ( which doesn't 
	// have a title ) is enumerated before the MainFrame 

	CWnd::FromHandle( hwnd )->GetWindowText(Title);
	ROBOT_LOG( TRUE,  " DEBUG ENUM WINDOW CALLBACK: Title = %s\n", Title )
	if( ProgramToFind == Title )
	{
		ROBOT_LOG( TRUE,  " DEBUG ENUM WINDOW CALLBACK: FOUND ITUNES!!! \n" )

	}

	if ( (FoundProcessId  == pFWHS->ProcessInfo.dwProcessId) && (Title.GetLength() != 0) ) 
	{ 
		ROBOT_LOG( TRUE,  "FOUND!")
		pFWHS->hWndFound = hwnd; 
		return false; 
	} 
	else 
	{ 
		// Keep enumerating 
		return true; 
	} 
}


/////////////////////////////////////////////////////////////////////////////
// Setup

IMPLEMENT_DYNCREATE(Setup, CFormView)

Setup::Setup()
	: CFormView(Setup::IDD)
	, m_strDynaRX64SerialPort(_T(""))
	, m_strLaserSerialPort(_T(""))
	, m_nLaserScansToRequest(0)
	, m_nObjectX(0)
	, m_nObjectY(0)
	, m_nObjectZ(0)
	, m_HeadPanServoSet(0)
	, m_HeadTiltServoSet(0)
	, m_HeadSideTiltServoSet(0)
	, m_HeadPanServo(0)
	, m_HeadTiltServo(0)
	, m_HeadSideTiltServo(0)
	, m_LaserScanStartAngle(0)
	, m_LaserScanEndAngle(0)
	, m_LaserScanStepAngle(0)
	, m_KinectTiltServoSet(0)
	, m_KinectTiltServo(0)
	, m_nFindObjectX(0)
	, m_nFindObjectY(0)
	, m_nFindObjectZ(0)
	, m_ScriptFileName(_T(""))
	, m_EditScriptTextLine(_T(""))
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	//{{AFX_DATA_INIT(Setup)
	m_strGpsSerialPort = _T("");
	m_strServoSerialPort = _T("");
	m_strMotorSerialPort = _T("");
	m_strCameraSerialPort = _T("");
	m_strPicSerialPort = _T("");
	m_strRobotIPAddress = _T("");
	m_strDynaServoSerialPort = _T("");
	m_strDynaRX64SerialPort = _T("");
	m_nDynaServoTestID = 0;
	m_nDynaServoTestPos = 0;
	m_nDynaServoRegNum = 0;
	m_nDynaServoRegWriteValue = 0;
	m_DynaServoNumOfRegs = 0;
	m_CliffSensorsEnabled = FALSE;
	m_strKerrServoSerialPort = _T("");
	m_ArmSpeedR = _T("");
	m_ArmSpeedL = _T("");
	m_RightShoulderRotate = 0;
	m_RightWristRotate = 0;
	m_RightElbowBend = 0;
	m_RightElbowRotate = 0;
	m_RightGrip = 0;
	m_EnableArmServosRight = FALSE;
	m_EnableArmServosLeft = FALSE;
	m_RightElbowBendSet = 0;
	m_RightElbowRotateSet = 0;
	m_RightGripSet = 0;
	m_RightShoulderRotateSet = 0;
	m_RightWristRotateSet = 0;
	m_LeftShoulderRotate = 0;
	m_LeftWristRotate = 0;
	m_LeftElbowBend = 0;
	m_LeftElbowRotate = 0;
	m_LeftGrip = 0;
	m_EnableArmServosLeft = FALSE;
	m_LeftElbowBendSet = 0;
	m_LeftElbowRotateSet = 0;
	m_LeftGripSet = 0;
	m_LeftShoulderRotateSet = 0;
	m_LeftWristRotateSet = 0;
	m_ScriptFileHandle = NULL;

	//}}AFX_DATA_INIT
}

Setup::~Setup()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	// Shut down video camera capture and windows
	ROBOT_LOG( TRUE,  "~CRobotSetup Requesting VidCap shutdown...\n" )
	EnableCamera( LEFT_CAMERA,  FALSE );
	EnableCamera( RIGHT_CAMERA,  FALSE );
	EnableCamera( KINECT_DEPTH,  FALSE );
	EnableCamera( KINECT_VIDEO,  FALSE );
	Sleep(10);
	g_bRunVidCapThread = FALSE;
	g_bRunKinectThread = FALSE;
	g_bRunDepthCameraThread = FALSE;

}

void Setup::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(Setup)
	DDX_Text(pDX, IDC_GPS_COM_PORT, m_strGpsSerialPort);
	DDV_MaxChars(pDX, m_strGpsSerialPort, 10);
	DDX_Text(pDX, IDC_SERVO_COM_PORT, m_strServoSerialPort);
	DDX_Text(pDX, IDC_MOTOR_COM_PORT, m_strMotorSerialPort);
	DDX_Text(pDX, IDC_CAMERA_COM_PORT, m_strCameraSerialPort);
	DDX_Text(pDX, IDC_PIC_COM_PORT, m_strPicSerialPort);
	DDX_Text(pDX, IDC_IP_ADDR1, m_strRobotIPAddress);
	DDX_Text(pDX, IDC_DYNA_SERVO_COM_PORT, m_strDynaServoSerialPort);
	DDX_Text(pDX, IDC_DYNA_SERVO_TEST_ID, m_nDynaServoTestID);
	DDV_MinMaxUInt(pDX, m_nDynaServoTestID, 0, 1024);
	DDX_Text(pDX, IDC_DYNA_SERVO_TEST_POS, m_nDynaServoTestPos);
	DDV_MinMaxUInt(pDX, m_nDynaServoTestPos, 0, 10000);
	DDX_Text(pDX, IDC_DYNA_SERVO_REG_NUM, m_nDynaServoRegNum);
	DDV_MinMaxUInt(pDX, m_nDynaServoRegNum, 0, 1024);
	DDX_Text(pDX, IDC_DYNA_SERVO_REG_WRITE_VALUE, m_nDynaServoRegWriteValue);
	DDV_MinMaxUInt(pDX, m_nDynaServoRegWriteValue, 0, 65535);
	DDX_Text(pDX, IDC_DYNA_SERVO_NUM_OF_REGISTERS, m_DynaServoNumOfRegs);
	DDV_MinMaxUInt(pDX, m_DynaServoNumOfRegs, 0, 100);
	DDX_Check(pDX, IDC_CLIFF_SENSORS, m_CliffSensorsEnabled);
	DDX_Text(pDX, IDC_KERR_SERVO_COM_PORT, m_strKerrServoSerialPort);
	DDV_MaxChars(pDX, m_strKerrServoSerialPort, 16);

	DDX_CBString(pDX, IDC_ARM_SPEED_R, m_ArmSpeedR);
	DDX_Check(pDX, IDC_ENABLE_SERVOS_R, m_EnableArmServosRight);

	DDX_Text(pDX, IDC_RIGHT_SHOULDER_ROTATE, m_RightShoulderRotate);
	DDV_MinMaxInt(pDX, m_RightShoulderRotate, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_WRIST_ROTATE, m_RightWristRotate);
	DDV_MinMaxInt(pDX, m_RightWristRotate, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_ELBOW_BEND, m_RightElbowBend);
	DDV_MinMaxInt(pDX, m_RightElbowBend, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_ELBOW_ROTATE, m_RightElbowRotate);
	DDV_MinMaxInt(pDX, m_RightElbowRotate, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_GRIP, m_RightGrip);
	DDV_MinMaxInt(pDX, m_RightGrip, -999, 999);

	DDX_Text(pDX, IDC_RIGHT_ELBOW_BEND_SET, m_RightElbowBendSet);
	DDV_MinMaxInt(pDX, m_RightElbowBendSet, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_ELBOW_ROTATE_SET, m_RightElbowRotateSet);
	DDV_MinMaxInt(pDX, m_RightElbowRotateSet, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_GRIP_SET, m_RightGripSet);
	DDV_MinMaxInt(pDX, m_RightGripSet, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_SHOULDER_ROTATE_SET, m_RightShoulderRotateSet);
	DDV_MinMaxInt(pDX, m_RightShoulderRotateSet, -999, 999);
	DDX_Text(pDX, IDC_RIGHT_WRIST_ROTATE_SET, m_RightWristRotateSet);
	DDV_MinMaxInt(pDX, m_RightWristRotateSet, -999, 999);


	DDX_CBString(pDX, IDC_ARM_SPEED_L, m_ArmSpeedL);
	DDX_Check(pDX, IDC_ENABLE_SERVOS_L, m_EnableArmServosLeft);

	DDX_Text(pDX, IDC_LEFT_SHOULDER_ROTATE, m_LeftShoulderRotate);
	DDV_MinMaxInt(pDX, m_LeftShoulderRotate, -999, 999);
	DDX_Text(pDX, IDC_LEFT_WRIST_ROTATE, m_LeftWristRotate);
	DDV_MinMaxInt(pDX, m_LeftWristRotate, -999, 999);
	DDX_Text(pDX, IDC_LEFT_ELBOW_BEND, m_LeftElbowBend);
	DDV_MinMaxInt(pDX, m_LeftElbowBend, -999, 999);
	DDX_Text(pDX, IDC_LEFT_ELBOW_ROTATE, m_LeftElbowRotate);
	DDV_MinMaxInt(pDX, m_LeftElbowRotate, -999, 999);
	DDX_Text(pDX, IDC_LEFT_GRIP, m_LeftGrip);
	DDV_MinMaxInt(pDX, m_LeftGrip, -999, 999);

	DDX_Text(pDX, IDC_LEFT_ELBOW_BEND_SET, m_LeftElbowBendSet);
	DDV_MinMaxInt(pDX, m_LeftElbowBendSet, -999, 999);
	DDX_Text(pDX, IDC_LEFT_ELBOW_ROTATE_SET, m_LeftElbowRotateSet);
	DDV_MinMaxInt(pDX, m_LeftElbowRotateSet, -999, 999);
	DDX_Text(pDX, IDC_LEFT_GRIP_SET, m_LeftGripSet);
	DDV_MinMaxInt(pDX, m_LeftGripSet, -999, 999);
	DDX_Text(pDX, IDC_LEFT_SHOULDER_ROTATE_SET, m_LeftShoulderRotateSet);
	DDV_MinMaxInt(pDX, m_LeftShoulderRotateSet, -999, 999);
	DDX_Text(pDX, IDC_LEFT_WRIST_ROTATE_SET, m_LeftWristRotateSet);
	DDV_MinMaxInt(pDX, m_LeftWristRotateSet, -999, 999);

	//}}AFX_DATA_MAP
	DDX_Text(pDX, IDC_DYNA_RX64_COM_PORT, m_strDynaRX64SerialPort);
	DDX_Text(pDX, IDC_LASER_COM_PORT, m_strLaserSerialPort);
	DDV_MinMaxUInt(pDX, m_nLaserScansToRequest, 0, 1000);
	DDX_Text(pDX, IDC_OBJECT_X, m_nObjectX);
	DDX_Text(pDX, IDC_OBJECT_Y, m_nObjectY);
	DDX_Text(pDX, IDC_OBJECT_Z, m_nObjectZ);
	DDX_Text(pDX, IDC_HEAD_PAN_SET, m_HeadPanServoSet);
	DDV_MinMaxInt(pDX, m_HeadPanServoSet, -360, 360);
	DDX_Text(pDX, IDC_HEAD_TILT_SET, m_HeadTiltServoSet);
	DDV_MinMaxInt(pDX, m_HeadTiltServoSet, -360, 360);
	DDX_Text(pDX, IDC_HEAD_SIDETITL_SET, m_HeadSideTiltServoSet);
	DDV_MinMaxInt(pDX, m_HeadSideTiltServoSet, -360, 360);
	DDX_Text(pDX, IDC_HEAD_PAN_GET, m_HeadPanServo);
	DDV_MinMaxInt(pDX, m_HeadPanServo, -360, 360);
	DDX_Text(pDX, IDC_HEAD_TILT_GET, m_HeadTiltServo);
	DDV_MinMaxInt(pDX, m_HeadTiltServo, -360, 360);
	DDX_Text(pDX, IDC_HEAD_SIDETILT_GET, m_HeadSideTiltServo);
	DDV_MinMaxInt(pDX, m_HeadSideTiltServo, -360, 360);
	DDX_Text(pDX, IDC_START_ANGLE, m_LaserScanStartAngle);
	DDV_MinMaxInt(pDX, m_LaserScanStartAngle, -180, 180);
	DDX_Text(pDX, IDC_END_ANGLE, m_LaserScanEndAngle);
	DDV_MinMaxInt(pDX, m_LaserScanEndAngle, -180, 180);
	DDX_Text(pDX, IDC_STEP_ANGLE, m_LaserScanStepAngle);
	DDV_MinMaxInt(pDX, m_LaserScanStepAngle, 0, 1000);
	DDX_Text(pDX, IDC_KINECT_TILT_SET, m_KinectTiltServoSet);
	DDX_Text(pDX, IDC_KINECT_TILT_GET, m_KinectTiltServo);
	DDX_Text(pDX, IDC_OBJECT_X, m_nFindObjectX);
	DDV_MinMaxInt(pDX, m_nFindObjectX, -1000, 1000);
	DDX_Text(pDX, IDC_OBJECT_Y, m_nFindObjectY);
	DDV_MinMaxInt(pDX, m_nFindObjectY, -1000, 1000);
	DDX_Text(pDX, IDC_OBJECT_Z, m_nFindObjectZ);
	DDV_MinMaxInt(pDX, m_nFindObjectZ, -1000, 1000);
#if( TURTLE != ROBOT_TYPE )
	DDX_Text(pDX, IDC_EDIT_SCRIPT_FILENAME, m_ScriptFileName);
	DDX_Text(pDX, IDC_EDIT_SCRIPT_TEXT_LINE, m_EditScriptTextLine);
#endif
}


BEGIN_MESSAGE_MAP(Setup, CFormView)
	//{{AFX_MSG_MAP(Setup)
	ON_BN_CLICKED(IDC_SAVE_TO_DISK, OnSaveToDisk)
	ON_BN_CLICKED(IDC_CONNECT_TO_HOST, OnConnectToHost)
	ON_BN_CLICKED(IDC_CAMERA_PWR_ON, OnCameraPwrOn)
	ON_BN_CLICKED(IDC_CAMERA_PWR_OFF, OnCameraPwrOff)
	ON_BN_CLICKED(IDC_CONNECT_TO_IP_CAMERA, OnConnectToIpCamera)
	ON_CBN_SELCHANGE(IDC_AVOID_OBJ_RANGE, OnSelchangeAvoidObjRange)
	ON_CBN_SELCHANGE(IDC_CAMERA_TRACKING_COLOR_THRESHOLD, OnSelchangeCameraTrackingColorThreshold)
	ON_BN_CLICKED(IDC_CAMERA_SCAN_DISTANCE_BTN, OnCameraScanDistanceBtn)
	ON_BN_CLICKED(IDC_CAMERA_MANUAL_COLOR_CAL_BTN, OnCameraManualColorCalBtn)
	ON_BN_CLICKED(IDC_CONNECT_TO_LOCAL_HOST, OnConnectToLocalHost)
	ON_BN_CLICKED(IDC_RADAR_SCAN, OnRadarScan)
	ON_BN_CLICKED(IDC_COMPASS_CAL_MODE, OnCompassCalMode)
	ON_BN_CLICKED(IDC_COMPASS_CAL_POINT, OnCompassCalPoint)
	ON_BN_CLICKED(IDC_RESET_ODOMETER, OnResetOdometer)
	ON_BN_CLICKED(IDC_RESET_WATCHDOG, OnResetWatchdog)
	ON_BN_CLICKED(IDC_START_ZC_SVC, OnStartZcSvc)
	ON_BN_CLICKED(IDC_STOP_ZC_SVC, OnStopZcSvc)
	ON_COMMAND(ID_SEL_CMD_VIEW_BTN, OnSelCmdViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_CMD_VIEW_BTN, OnUpdateSelCmdViewBtn)
	ON_COMMAND(ID_SEL_MAP_VIEW_BTN, OnSelMapViewBtn)
	ON_COMMAND(ID_SEL_PATH_VIEW_BTN, OnSelPathViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_MAP_VIEW_BTN, OnUpdateSelMapViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_PATH_VIEW_BTN, OnUpdateSelPathViewBtn)
	ON_COMMAND(ID_SEL_SETUP_VIEW_BTN, OnSelSetupViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_SETUP_VIEW_BTN, OnUpdateSelSetupViewBtn)
	ON_BN_CLICKED(IDC_GPS_OPEN_PORT, OnGpsOpenPort)
	ON_BN_CLICKED(IDC_OPEN_SERVO_PORT, OnOpenServoPort)
	ON_BN_CLICKED(IDC_OPEN_MOTOR_PORT, OnOpenMotorPort)
	ON_BN_CLICKED(IDC_OPEN_CAMERA_PORT, OnOpenCameraPort)
	ON_CBN_SELCHANGE(IDC_CAM_PAN_SPEED, OnSelchangeCamPanSpeed)
	ON_BN_CLICKED(IDC_CAMERA_POS_ABS_BTN, OnCameraPosAbsBtn)
	ON_BN_CLICKED(IDC_OPEN_PIC_PORT, OnOpenArduinoPort)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_FACE, OnCameraEnableTrackingFace)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_COLORS, OnCameraEnableTrackingColors)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_CONES, OnCameraEnableTrackingCones)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_OBJECTS, OnCameraEnableTrackingObjects)
	ON_CBN_SELCHANGE(IDC_CAM_ZOOM_LEVEL_ABS, OnSelchangeCamZoomLevelAbs)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_OBJECT_MOTION, OnCameraEnableTrackingObjectMotion)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_SHOW_MOTION_VIEW, OnCameraEnableShowMotionView)
	ON_CBN_SELCHANGE(IDC_CAM_FRAME_SIZE, OnSelchangeCamFrameSize)
	ON_BN_CLICKED(IDC_VIDEO_FORMAT_BTN, OnVideoFormatBtn)
	ON_BN_CLICKED(IDC_VIDEO_PROP_BTN, OnVideoPropBtn)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_COLOR_AUTO_CAL_BTN, OnColorAutoCalBtn)
	ON_BN_CLICKED(IDC_VIDEO_SELECT_CAMERA, OnVideoSelectCamera)
	ON_BN_CLICKED(IDC_ENABLE_CAMERA_1, OnEnableCamera1)
	ON_BN_CLICKED(IDC_ENABLE_CAMERA_2, OnEnableCamera2)
	ON_BN_CLICKED(IDC_CAM_DISPLAY_MODE_ENABLE, OnCamDisplayModeEnable)
	ON_BN_CLICKED(IDC_CAM_TRACK_MODE_ENABLE, OnCamTrackModeEnable)
	ON_BN_CLICKED(IDC_CAM_IR_TRACKING_ENABLE, OnCamIrTrackingEnable)
	ON_BN_CLICKED(IDC_PERSONAL_SPACE, OnPersonalSpace)
	ON_BN_CLICKED(IDC_OPEN_DYNA_SERVO_PORT, OnOpenDynaServoPort)
	ON_BN_CLICKED(IDC_DYNA_SERVO_TEST_GO, OnDynaServoTestGo)
	ON_BN_CLICKED(IDC_SERVO_GET_STATUS, OnServoGetStatus)
	ON_BN_CLICKED(IDC_DYNA_SERVO_REG_READ, OnDynaServoRegRead)
	ON_BN_CLICKED(IDC_DYNA_SERVO_REG_WRITE_BYTE, OnDynaServoRegWriteByte)
	ON_BN_CLICKED(IDC_DYNA_SERVO_REG_WRITE_WORD, OnDynaServoRegWriteWord)
	ON_BN_CLICKED(IDC_SPEECH_RECO_SEND_TO_AI, OnSpeechRecoSendToAi)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_FACE_IDENTIFY, OnCameraEnableFaceIdentification)
	ON_BN_CLICKED(IDC_CLIFF_SENSORS, OnCliffSensors)
	ON_BN_CLICKED(IDC_OPEN_KERR_SERVO_PORT, OnOpenKerrServoPort)
	ON_BN_CLICKED(IDC_READ_RIGHT_ARM_POSITION, OnReadRightArmPosition)
	ON_BN_CLICKED(IDC_SET_RIGHT_ARM_POSITION, OnSetRightArmPosition)
	ON_CBN_SELCHANGE(IDC_ARM_SPEED_R, OnSelchangeArmSpeedR)
	ON_CBN_SELCHANGE(IDC_R_ARM_POS_PRESET, OnSelchangeRArmPosPreset)
	ON_BN_CLICKED(IDC_ENABLE_SERVOS_R, OnEnableServosR)
	ON_BN_CLICKED(IDC_COPY_RIGHT_ARM_POSITION, OnCopyRightArmPosition)

	ON_BN_CLICKED(IDC_READ_LEFT_ARM_POSITION, OnReadLeftArmPosition)
	ON_BN_CLICKED(IDC_SET_LEFT_ARM_POSITION, OnSetLeftArmPosition)
	ON_CBN_SELCHANGE(IDC_ARM_SPEED_L, OnSelchangeArmSpeedL)
	ON_CBN_SELCHANGE(IDC_L_ARM_POS_PRESET, OnSelchangeLArmPosPreset)
	ON_BN_CLICKED(IDC_ENABLE_SERVOS_L, OnEnableServosL)
	ON_BN_CLICKED(IDC_COPY_LEFT_ARM_POSITION, OnCopyLeftArmPosition)

	ON_BN_CLICKED(IDC_DYNA_USB_ENABLE, OnDynaUsbEnable)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_STEREO_VISION, OnCameraEnableStereoVision)
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_TRACKING_HAND, OnCameraEnableTrackingHand)
	ON_BN_CLICKED(IDC_ENABLE_VIDEO_PROCESSING, OnEnableVideoProcessing)
	ON_WM_VSCROLL()
	ON_BN_CLICKED(IDC_CAMERA_ENABLE_MATCH_OBJECTS, OnCameraEnableMatchObjects)

	ON_BN_CLICKED(IDC_LAUNCH_APP, &Setup::OnBnClickedLaunchApp)
	ON_BN_CLICKED(IDC_ITUNES_PRIOR, &Setup::OnBnClickedItunesPrior)
	ON_BN_CLICKED(IDC_ITUNES_NEXT, &Setup::OnBnClickedItunesNext)
//	ON_CBN_SELCHANGE(IDC_PC_PWR_MODE, &Setup::OnCbnSelchangePcPwrMode)
	ON_CBN_SELCHANGE(IDC_PC_PWR_MODE, &Setup::OnCbnSelchangePcPwrMode)
	ON_BN_CLICKED(IDC_ITUNES_FULLSCREEN, &Setup::OnBnClickedItunesFullscreen)
	ON_CBN_SELCHANGE(IDC_BOTH_ARM_POS_PRESET, &Setup::OnCbnSelchangeBothArmPosPreset)

	//}}AFX_MSG_MAP

	ON_MESSAGE( (WM_ROBOT_DISPLAY_SINGLE_ITEM), OnRobotDisplaySingleItem )
	ON_MESSAGE( (WM_ROBOT_REMOTE_GUI_CMD), OnRemoteGuiCommand )
	ON_MESSAGE( (WM_ROBOT_GET_CAMERA_SETTINGS), OnGetCameraSettings )
	ON_MESSAGE( (WM_ROBOT_SERVO_STATUS_READY), OnServoStatusReady )
	
	ON_BN_CLICKED(IDC_OPEN_DYNA_RX64_SERVO_PORT, &Setup::OnBnClickedOpenDynaRx64ServoPort)
	ON_BN_CLICKED(IDC_OPEN_LASER_PORT, &Setup::OnBnClickedOpenLaserPort)
	ON_BN_CLICKED(IDC_ENABLE_LASER_POWER, &Setup::OnBnClickedEnableLaserPower)
	ON_BN_CLICKED(IDC_REQUEST_LASER_SCANS, &Setup::OnBnClickedRequestLaserScans)
	ON_BN_CLICKED(IDC_FIND_OBJ_AT_XYZ, &Setup::OnBnClickedFindObjAtXyz)
	ON_BN_CLICKED(IDC_CHECK_LOCATION, &Setup::OnBnClickedCheckLocation)
	ON_BN_CLICKED(IDC_COPY_HEAD_POSITION, &Setup::OnBnClickedCopyHeadPosition)
	ON_BN_CLICKED(IDC_SET_HEAD_POSITION, &Setup::OnBnClickedSetHeadPosition)
	ON_BN_CLICKED(IDC_LASER_SEARCH_FOR_OBJECTS, &Setup::OnBnClickedLaserSearchForObjects)
	ON_BN_CLICKED(IDC_SET_LASER_POSITION, &Setup::OnBnClickedSetLaserPosition)
	ON_BN_CLICKED(IDC_ENABLE_KINECT, &Setup::OnBnClickedEnableKinect)
	ON_EN_CHANGE(IDC_RIGHT_SHOULDER_ROTATE, &Setup::OnEnChangeRightShoulderRotate)
	ON_CBN_SELCHANGE(IDC_CAM_DISPLAY_SIZE, &Setup::OnCbnSelchangeCamDisplaySize)
	ON_CBN_SELCHANGE(IDC_KINECT_DISPLAY_SIZE, &Setup::OnCbnSelchangeKinectDisplaySize)
	ON_BN_CLICKED(IDC_FLIP_KINECT, &Setup::OnBnClickedFlipKinect)
	ON_BN_CLICKED(IDC_KINECT_PWR, &Setup::OnBnClickedKinectPwr)
#if( TURTLE != ROBOT_TYPE )
	ON_BN_CLICKED(IDC_EDIT_SCRIPT_FILE, &Setup::OnBnClickedEditFile)
	ON_BN_CLICKED(IDC_RUN_SCRIPT, &Setup::OnBnClickedRunScript)
	ON_BN_CLICKED(IDC_ADD_SCRIPT_DELAY, &Setup::OnBnClickedAddScriptTextLine)
	ON_BN_CLICKED(IDC_SAVE_ARM_TO_SCRIPT_RIGHT, &Setup::OnBnClickedSaveArmToScriptRight)
	ON_BN_CLICKED(IDC_SAVE_ARM_TO_SCRIPT_LEFT, &Setup::OnBnClickedSaveArmToScriptLeft)
	ON_BN_CLICKED(IDC_CLOSE_SCRIPT_FILE, &Setup::OnBnClickedCloseScriptFile)
	ON_EN_VSCROLL(IDC_EDIT_SCRIPT_TEXT_LINE, &Setup::OnEnVscrollEditScriptTextLine)
#endif
	END_MESSAGE_MAP()



/////////////////////////////////////////////////////////////////////////////
// Overrides


#ifdef _DEBUG
SetupDoc* Setup::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(SetupDoc)));
	return (SetupDoc*)m_pDocument;
}
#endif //_DEBUG


/////////////////////////////////////////////////////////////////////////////
// Setup diagnostics

#ifdef _DEBUG
void Setup::AssertValid() const
{
	CFormView::AssertValid();
}

void Setup::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// Setup message handlers

 
SIZE Setup::GetFrameSize( UINT EnumSize ) 
{
	SIZE FrameSize;
	FrameSize.cx = 160;
	FrameSize.cy = 120;

	switch( EnumSize )
	{
		case 0:
		{
			FrameSize.cx = 160;
			FrameSize.cy = 120;
			break;
		}
		case 1:
		{
			FrameSize.cx = 320;
			FrameSize.cy = 240;
			break;
		}
		case 2:
		{
			FrameSize.cx = 640;
			FrameSize.cy = 480;
			break;
		}
		default:
		{
			FrameSize.cx = 160;
			FrameSize.cy = 120;
		}
	}
	return FrameSize;
}



void Setup::OnInitialUpdate() 
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

// WAIT FOR DEBUGGER TO ATTACH
#define WAIT_FOR_DEBUGGER 0
#if WAIT_FOR_DEBUGGER == 1

	ROBOT_LOG( TRUE,  "WAITING FOR DEBUGGER...\n" )
	Sleep(3000, pDomainGUIThread);
	ROBOT_LOG( TRUE,  "DONE WAITING FOR DEBUGGER\n" )
#endif

	CFormView::OnInitialUpdate();
	g_RobotSetupViewHWND = GetSafeHwnd();	// Allow other windows to send me messages

#if ( ROBOT_SERVER == 1 )

	// Enable Kinect Power
	CheckDlgButton( IDC_KINECT_PWR, GetDocument()->m_bEnableKinectPower );
	if( GetDocument()->m_bEnableKinectPower )
	{
		g_KinectPowerEnabled = TRUE; // Allows enabling, even though the thread has not started yet
	}

	// Enable Dynamixel Servo Power
	CheckDlgButton( IDC_DYNA_USB_ENABLE, GetDocument()->m_bEnableDynaServos );
//	if( GetDocument()->m_bEnableDynaServos )
	{
		g_DynaPowerEnabled = TRUE; // Allows enabling, even though the thread has not started yet
		g_DynaSubSystemStatus = SUBSYSTEM_WAITING;
		#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
			g_RX64SubSystemStatus = SUBSYSTEM_WAITING;
		#else
			g_RX64SubSystemStatus = SUBSYSTEM_DISABLED;
		#endif
	}
	/**
	else
	{
		g_DynaPowerEnabled = FALSE;
		g_DynaSubSystemStatus = SUBSYSTEM_DISABLED;
	}
	***/

	if( MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL )
	{
		// Power for Kinect and Servos enabled by the iRobot Base, so power up the base first
		ROBOT_LOG( TRUE,  "Opening iRobot base MOTOR PORT\n" )
		m_strMotorSerialPort = GetDocument()->m_strMotorSerialPort;
		OpenMotorPort();
	}
	RobotSleep(1000, pDomainGUIThread); // Let Kinect start up

	// Init global data blocks
	memset(g_BulkServoCmd, 0, (sizeof(BULK_SERVO_CMD_T))* NUMBER_OF_SMART_SERVOS+1);
	memset(g_BulkServoStatus, 0, (sizeof(BULK_SERVO_STATUS_T))* NUMBER_OF_SMART_SERVOS+1);

	// Camera settings - OLD
	g_Camera[LEFT_CAMERA].State = CAMERA_STATE_NOT_ENABLED;
	g_Camera[RIGHT_CAMERA].State = CAMERA_STATE_NOT_ENABLED;
	g_Camera[KINECT_DEPTH].State = CAMERA_STATE_INITIALIZED;
	g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;

	// For communicating with Camera App
	m_CameraRequest.RequestType = CAMERA_REQUEST_ENABLE_FEATURES;
	m_CameraRequest.RequestData.EnableFeatures.FaceRecognition = 0;
	m_CameraRequest.RequestData.EnableFeatures.FaceTracking = 0;
	m_CameraRequest.RequestData.EnableFeatures.ObjectMatch = 0;
	m_CameraRequest.RequestData.EnableFeatures.VideoEnable = 0;


	// Initialize Servo Default Speeds
	for( int ServoNum=0; ServoNum < (NUMBER_OF_SMART_SERVOS+1); ServoNum++ )
	{
		g_BulkServoCmd[ServoNum].Speed = SERVO_SPEED_MED_SLOW;
	}
#endif // ROBOT_SERVER

	g_bLaserContinuousScanEnabled = TRUE;

	// Copy all of the strings from the document to the view
	m_strPicSerialPort = GetDocument()->m_strPicSerialPort;
	m_strGpsSerialPort = GetDocument()->m_strGpsSerialPort;
	m_strServoSerialPort = GetDocument()->m_strServoSerialPort;
	m_strDynaServoSerialPort = GetDocument()->m_strDynaServoSerialPort;
	m_strDynaRX64SerialPort = GetDocument()->m_strDynaRX64SerialPort;
	m_strKerrServoSerialPort = GetDocument()->m_strKerrServoSerialPort;
	m_strLaserSerialPort = GetDocument()->m_strLaserSerialPort;
	m_strMotorSerialPort = GetDocument()->m_strMotorSerialPort;
	m_strCameraSerialPort = GetDocument()->m_strCameraSerialPort;
	m_strRobotIPAddress = GetDocument()->m_strRobotIPAddress;
	m_ScriptFileName = "RobotScript_";

	UpdateData(FALSE);	// Force data exchange from data members to GUI


	CheckDlgButton( IDC_ENABLE_SERVOS_R, GetDocument()->m_EnableArmServosRight );
	if ( GetDocument()->m_EnableArmServosRight )
	{
		g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable = TRUE;
		g_RightArmSubSystemStatus = SUBSYSTEM_WAITING;
	}
	else
	{
		g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable = FALSE;
		g_RightArmSubSystemStatus = SUBSYSTEM_DISABLED;
	}

	CheckDlgButton( IDC_ENABLE_SERVOS_L, GetDocument()->m_EnableArmServosLeft );
	if ( GetDocument()->m_EnableArmServosLeft )
	{
		g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable = TRUE;
		g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable = TRUE;
		g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING;
	}
	else
	{
		g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable = FALSE;
		g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable = FALSE;
		g_LeftArmSubSystemStatus = SUBSYSTEM_DISABLED;
	}


	//////////////////////////////////////////////////////////////////////////
	// Open COM Ports
	#if ( ROBOT_SERVER == 1 )

		ROBOT_LOG( TRUE,  "\n********************************************\n" )
		ROBOT_LOG( TRUE,  "Opening COM Ports...\n" )
		if ( (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_KOBUKI_WITH_ARDUINO) )
		{
			ROBOT_LOG( TRUE,  "     Arduino PORT\n" )
			OpenArduinoPort();
		}

		if( MOTOR_CONTROL_TYPE != IROBOT_MOTOR_CONTROL )
		{
			// port opened earlier for iRobot (in order to enable the Kinect power sooner)
			// for all other motor control types, open the port (if any) and start control threads now
			ROBOT_LOG( TRUE,  "     MOTOR PORT\n" )
			OpenMotorPort();
		}

		ROBOT_LOG( TRUE,  "     SMART SERVO PORTS (AX12, RX64, Kerr)\n" )
		OpenSmartServoPort();

		if( TURTLE != ROBOT_TYPE )
		{
			ROBOT_LOG( TRUE,  "     LASER PORT\n" )
			OpenLaserPort();
		}

		// WARNING - FOLLOWING PORTS DISABLED!

		//ROBOT_LOG( TRUE,  "     Servo PORT\n" ) // Analog servos
		//OpenServoPort();

		//ROBOT_LOG( TRUE,  "     Sony Camera COM PORT\n" )
		//OpenCameraPort();

		//ROBOT_LOG( TRUE,  "     GPS PORT\n" )
		//OpenGPSPort();


	ROBOT_LOG( TRUE,  "\n********************************************\n" )


#endif // ROBOT_SERVER


	// Do Initialization of Robot parameters
	CString strText;
	BOOL bEnable;
	ROBOT_LOG( TRUE,  "Initializing Robot Configuration Parameters\n" )


	// Initialize settings for the external camera app
	// OLD: Camera Processing is reset by OnGetCameraSettings() due to lag in thread startup
	if ( NULL != g_pCameraRequestSharedMemory )
	{
		ROBOT_LOG( TRUE,  "Initializing Camera App Config" )
		if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
		{
			bEnable = GetDocument()->m_bEnableCamera1;
			CheckDlgButton( IDC_ENABLE_CAMERA_1, bEnable );
			m_CameraRequest.RequestData.EnableFeatures.VideoEnable = bEnable;
			ROBOT_LOG( TRUE, "    Camera1 Video Enable = %d",  bEnable )

			bEnable = GetDocument()->m_EnableFaceIdentification;
			CheckDlgButton( IDC_CAMERA_ENABLE_FACE_IDENTIFY, bEnable );
			m_CameraRequest.RequestData.EnableFeatures.FaceRecognition = bEnable;
			ROBOT_LOG( TRUE, "    Face ID Enable = %d",  bEnable )

			bEnable = GetDocument()->m_bEnableTrackingFace;
			CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_FACE, bEnable );
			m_CameraRequest.RequestData.EnableFeatures.FaceTracking = bEnable;
			ROBOT_LOG( TRUE, "    Face Tracking Enable = %d",  bEnable )

			bEnable = GetDocument()->m_bEnableMatchingObjects;
			CheckDlgButton( IDC_CAMERA_ENABLE_MATCH_OBJECTS, bEnable );
			m_CameraRequest.RequestData.EnableFeatures.ObjectMatch = bEnable;
			ROBOT_LOG( TRUE, "    Object Matching Enable = %d",  bEnable )

			ROBOT_LOG( TRUE, "... Done\n" )

			// Notify the Camera App of our new settings
			CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
			SetEvent( g_hCameraRequestEvent );  // Send request
		}
		else
		{
			ROBOT_LOG( TRUE, "ERROR: Can't request video!  Did you have AUTO_LAUNCH_CAMERA_APP enabled?\n" )
		}
	}

	


	// Start right camera first to mimimize windows moving around
	CheckDlgButton( IDC_ENABLE_CAMERA_2, GetDocument()->m_bEnableCamera2 );
	OnEnableCamera2();
	CheckDlgButton( IDC_ENABLE_CAMERA_1, GetDocument()->m_bEnableCamera1 );
	OnEnableCamera1();
	CheckDlgButton( IDC_ENABLE_KINECT, GetDocument()->m_bEnableKinect );
	OnBnClickedEnableKinect();

	// Initialize state of Camera VidCap object detection features
	CheckDlgButton( IDC_ENABLE_VIDEO_PROCESSING, GetDocument()->m_bEnableEnableVideoProcessing );

	CheckDlgButton( IDC_FLIP_KINECT, GetDocument()->m_bEnableKinectVideoFlip );

	CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_FACE, GetDocument()->m_bEnableTrackingFace );

	CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_COLORS, GetDocument()->m_bEnableTrackingColors );

	CheckDlgButton( IDC_CAMERA_ENABLE_FACE_IDENTIFY, GetDocument()->m_EnableFaceIdentification );

	CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_CONES, GetDocument()->m_bEnableTrackingCones );

	CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_OBJECTS, GetDocument()->m_bEnableTrackingObjects );
	
	CheckDlgButton( IDC_CAMERA_ENABLE_MATCH_OBJECTS, GetDocument()->m_bEnableMatchingObjects );
	
	CheckDlgButton( IDC_CAMERA_ENABLE_TRACKING_OBJECT_MOTION, GetDocument()->m_bEnableTrackingMotion );

	CheckDlgButton( IDC_CAMERA_ENABLE_SHOW_MOTION_VIEW, GetDocument()->m_bEnableShowMotionView );

	CheckDlgButton( IDC_CAMERA_ENABLE_STEREO_VISION, GetDocument()->m_bEnableStereoVision );

	CheckDlgButton( IDC_CLIFF_SENSORS, GetDocument()->m_bEnableCliffSensors );

	
	// Initialize Combo Boxes
	CComboBox* pComboBox = NULL;

	pComboBox = (CComboBox*) GetDlgItem(IDC_AVOID_OBJ_RANGE);
	pComboBox->SetCurSel( (OBJECT_AVOID_DEFAULT_FEET) );

	pComboBox = (CComboBox*) GetDlgItem(IDC_CAMERA_TRACKING_COLOR_THRESHOLD);
	pComboBox->SetCurSel( CAMERA_THRESHOLD_DEFAULT );

	strText.Format("%d", CAMERA_SCAN_DIST_DEFAULT);
	SetDlgItemText( IDC_CAMERA_SCAN_DISTANCE, strText );

	pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_PAN_SPEED);
	pComboBox->SetCurSel( 3 );
	SendCommand( WM_ROBOT_USER_CAMERA_PAN_TILT_SPEED_CMD, 0, (DWORD)SERVO_SPEED_MED_SLOW );


	pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_ZOOM_LEVEL_ABS);
	pComboBox->SetCurSel( 0 );

	pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_FRAME_SIZE);
	pComboBox->SetCurSel( GetDocument()->m_VidCapSizeSelected );

	pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_DISPLAY_SIZE);
	pComboBox->SetCurSel( GetDocument()->m_VideoDisplaySizeSelected );

	pComboBox = (CComboBox*) GetDlgItem(IDC_KINECT_DISPLAY_SIZE);
	pComboBox->SetCurSel( GetDocument()->m_KinectDisplaySizeSelected );

	pComboBox = (CComboBox*) GetDlgItem(IDC_ARM_SPEED_R);
	pComboBox->SetCurSel( 3 );	// SERVO_SPEED_MED

	pComboBox = (CComboBox*) GetDlgItem(IDC_R_ARM_POS_PRESET);
	pComboBox->SetCurSel( 0 );	// 
	
	pComboBox = (CComboBox*) GetDlgItem(IDC_L_ARM_POS_PRESET);
	pComboBox->SetCurSel( 0 );	// 

	pComboBox = (CComboBox*) GetDlgItem(IDC_BOTH_ARM_POS_PRESET);
	pComboBox->SetCurSel( 0 );	// 


	m_nTestSlider1 = 10;
	m_nTestSlider2 = 10;

	CSliderCtrl* pSlider;

	// Initialize the Test Slider1 Control and text box
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_SLIDER_1);
	pSlider->SetRange(0, 255);
	pSlider->SetPos(m_nTestSlider1);
	strText.Format("0");
	SetDlgItemText(IDC_SLIDER_1_TXT, strText);

	// Initialize the Test Slider2 Control and text box
	pSlider = (CSliderCtrl*) GetDlgItem(IDC_SLIDER_2);
	pSlider->SetRange(0, 255);
	pSlider->SetPos(m_nTestSlider2);
	strText.Format("0");
	SetDlgItemText(IDC_SLIDER_2_TXT, strText);

//	UpdateData(FALSE); // Set values on the GUI


	// Initialize the Avoid Object Range spin control and text box
/*	CSpinButtonCtrl* pAvoidObjRangeSpin =
	(CSpinButtonCtrl*) GetDlgItem(IDC_AVOID_OBJ_MAX_RANGE_SPIN);
	pAvoidObjRangeSpin->SetRange(0, 50);
	pAvoidObjRangeSpin->SetPos(m_nAvoidObjectRange);
	strText.Format(_T("%d", m_nAvoidObjectRange);
	SetDlgItemText(IDC_AVOID_OBJ_MAX_RANGE, strText);
*/

	// Initialize GUI controls
	//SetDlgItemText(IDC_CONNECT_SPEED, "00 ms");


	// Initialize the IP address edit box
//	SetDlgItemText( IDC_IP_ADDR1, "192.168.0.40"); 
//	SetDlgItemText( IDC_IP_ADDR2, "192.168.1.41"); 




////////////////////////////////////////////////////////////////////
#if ( ROBOT_SERVER == 1 )



	// Create Server socket send and receive threads
	// Note that Client socket code is handled in Setup::ConnectToHost()
	DWORD dwTempThreadId;

	g_ServerSockStruct.hReceiveThread = CreateThread( 
		NULL, 0, ServerSockReceiveThreadProc, 
		(LPVOID)&g_ServerSockStruct, 0, &dwTempThreadId );
		ROBOT_LOG( TRUE,  "Created Server Socket Receive Thread. ID = (0x%x)", dwTempThreadId )

	g_ServerSockStruct.hSendThread = CreateThread( 
		NULL, 0, ServerSockSendThreadProc, 
		(LPVOID)&g_ServerSockStruct, 0, &g_dwServerSendThreadId );
		ROBOT_LOG( TRUE,  "Created Server Socket Send Thread. ID = (0x%x)", g_dwServerSendThreadId )


	// Turn on Eyes
	::PostThreadMessage( g_dwArduinoCommWriteThreadId, WM_ROBOT_MESSAGE_BASE+HW_SET_LED_EYES, LED_EYES_ON, 0 ); 	// Turn on Eyes		
		
	// Send initial camera config to the camera module
	OnGetCameraSettings(0,0);	// Starts Camera Capture if enabled!


#else
////////////////////////////////////////////////////////////////////
// ROBOTCLIENT

	// Initialize Socket structure
	g_ClientSockStruct.hDlgWnd = GetSafeHwnd();
	// Note that the client Socket threads are created on demand in ConnectToHost()

	g_bConnectedToServer = FALSE;

#endif
	ROBOT_LOG( TRUE,  "Done Initializing Setup Parameters" )

}

////////////////////////////////////////////////////////////////////
void Setup::OpenGPSPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening GPS Port ==================\n" )
	// GPS Data Parser Serial Port
	DWORD dwTempThreadId;
//	g_hGPSCommPort = OpenCommPort(_T("COM8:", CBR_4800, GPS_COMM_DEVICE);	// COMx, 4800baud
	g_hGPSCommPort = OpenCommPort( m_strGpsSerialPort, CBR_4800, GPS_COMM_DEVICE );	// COMx, 4800baud
	if (g_hGPSCommPort == INVALID_HANDLE_VALUE)
	{
		ROBOT_DISPLAY( TRUE, "Unable to open GPS COM Port %s", m_strGpsSerialPort )
		//AfxMessageBox( _T("Unable to open GPS COM Port!") );
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "GPS COM Port %s Opened", m_strGpsSerialPort )

		// Create the thread that will be reading the comm port
		// and pass it a pointer to the GPS NMEAParser object
		g_hGPSThread = CreateThread(NULL, 0, GPSCommReadThreadFunc, (LPVOID)&g_hGPSCommPort, 0, &dwTempThreadId);
		if(g_hGPSThread == NULL)	
		{
			ROBOT_LOG( TRUE,  "Error Creating Comm Read Thread." )
			CloseCommPort( "GPS", g_hGPSCommPort );
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created GPS Comm Read Thread. ID = (0x%x)", dwTempThreadId )
		}
	}
#endif	// ROBOT_SERVER code

}

////////////////////////////////////////////////////////////////////
void Setup::OpenLaserPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code

	// g_dwLaserScannerCommWriteThreadId
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Laser Scanner Port ==================\n" )
	CString strStatus;
	DWORD dwTempThreadId;
	g_hLaserScannerCommPort = OpenCommPort( m_strLaserSerialPort, CBR_9600, LASER_SCANNER_COMM_DEVICE );	// COMx, baud
	if (INVALID_HANDLE_VALUE == g_hLaserScannerCommPort )
	{
		ROBOT_DISPLAY( TRUE, "Unable to open LASER SCANNER COM Port %s", m_strLaserSerialPort )
		//AfxMessageBox( _T("Unable to open LASER SCANNER COM Port!") );
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "LASER SCANNER COM Port %s Opened", m_strLaserSerialPort )

		// Create the thread that will be reading the comm port and pass it a pointer to the com port
		g_hLaserScannerReadThread = CreateThread(NULL, 0, LaserScannerCommReadThreadFunc, (LPVOID)&g_hLaserScannerCommPort, 0, &dwTempThreadId);
		if( NULL == g_hLaserScannerReadThread )	
		{
			ROBOT_LOG( TRUE,  "Error Creating LASER SCANNER Read Comm Thread." )
			CloseCommPort( "LASER", g_hLaserScannerCommPort );
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created LASER SCANNER Read Comm Thread. ID = (0x%x)", dwTempThreadId )

			// Create the thread that will be writing commands to the comm port and pass it a pointer to the com port  
			g_hLaserScannerWriteThread = CreateThread(NULL, 0, LaserScannerCommWriteThreadFunc, (LPVOID)&g_hLaserScannerCommPort, 0, &g_dwLaserScannerCommWriteThreadId);
			if( NULL == g_hLaserScannerWriteThread )	
			{
				ROBOT_LOG( TRUE, "Error Creating LASER SCANNER Write Comm Thread." )
				CloseCommPort( "LASER", g_hLaserScannerCommPort );
			}
			else
			{
				ROBOT_LOG( TRUE,  "Created LASER SCANNER Write Comm Thread. ID = (0x%x)", g_dwLaserScannerCommWriteThreadId )
			}
		}

	}
#endif	// ROBOT_SERVER code

}



////////////////////////////////////////////////////////////////////
void Setup::OpenServoPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Servo Port ==================" )
	//DWORD dwTempThreadId;
	CString strStatus;
	g_hServoCommPort = OpenCommPort( m_strServoSerialPort, CBR_9600, SERVO_COMM_DEVICE );	// COMx, 9600baud
	if (g_hServoCommPort == INVALID_HANDLE_VALUE)
	{
		ROBOT_DISPLAY( TRUE, "Unable to open Servo COM Port %s", m_strServoSerialPort )
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "Servo COM Port %s Opened", m_strServoSerialPort )
		
		// Create the thread that will be writing to the comm port
		g_hServoThread = CreateThread(NULL, 0, ServoCommWriteThreadFunc, (LPVOID)g_hServoCommPort, 0, &g_dwServoCommWriteThreadId);
		if(g_hServoThread == NULL)	
		{
			ROBOT_LOG( TRUE, "Error Creating Servo Comm Write Thread." )
			CloseCommPort( "Servo", g_hServoCommPort );
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created Servo Comm Write Thread. ID = (0x%x)", g_dwServoCommWriteThreadId )
		}
	}
#endif	// ROBOT_SERVER code

}

////////////////////////////////////////////////////////////////////
void Setup::OpenCameraPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Camera Port ==================\n" )

#if CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA

	//DWORD dwTempThreadId;
	CString strStatus;
	g_hCameraCommPort = OpenCommPort( m_strCameraSerialPort, CBR_9600, CAMERA_COMM_DEVICE );	// COMx, 9600baud
	if (g_hCameraCommPort == INVALID_HANDLE_VALUE)
	{
		ROBOT_DISPLAY( TRUE, "Unable to open Camera COM Port %s", m_strCameraSerialPort )
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "Camera COM Port %s Opened", m_strCameraSerialPort );

		// Create the thread that will be reading the comm port
		hSonyThread = CreateThread(NULL, 0, CameraCommWriteThreadFunc, (LPVOID)g_hCameraCommPort, 0, &g_dwCameraCommWriteThreadId);
		if(hSonyThread == NULL)	
		{
			ROBOT_LOG( TRUE, "Error Creating Camera Comm Write Thread." )
			CloseCommPort( "Camera", g_hCameraCommPort );
		}
		else
		{
			CloseHandle(hSonyThread);	// No need to hold onto the thread handle (thread exits when the COMM port closes)
			ROBOT_LOG( TRUE,  "Created Camera Comm Write Thread. ID = (0x%x)", g_dwCameraCommWriteThreadId )
		}
	}
#else
	ROBOT_LOG( TRUE,  "WARNING:  SONY Camera Not enabled for this Robot Type\n" )

#endif // CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA

#endif	// ROBOT_SERVER code

}

////////////////////////////////////////////////////////////////////
void Setup::OpenMotorPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Motor Port ==================\n" )

	#if( MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL )
		// if controlled indirectly through the Arduino/Arduino, no separate motor thread needed
		ROBOT_DISPLAY( TRUE, "Using Arduino/Arduino MOTOR_CONTROL_TYPE, no Motor Thread needed" )
		return;

	#elif( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
		// Kobuki uses external control program.  
		// Don't open COM port, but do start the thread that talks to the external process
		ROBOT_DISPLAY( TRUE, "Motor COM Port not needed for KOBUKI_MOTOR_CONTROL, but starting thread for Inter Process Communication (IPC)" )
		// Create the thread that will be writing to the Kobuki App
		HANDLE g_hMotorWriteThread = CreateThread(NULL, 0, MotorCommThreadFunc, (LPVOID)0, 0, &g_dwMotorCommThreadId);
		if(g_hMotorWriteThread == NULL)	
		{
			ROBOT_LOG( TRUE, "Error Creating Motor Write Thread for Kobuki." )
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created Motor Comm Thread for Kobuki. ID = (0x%x)", g_dwMotorCommThreadId )
		}

	#else

		//DWORD dwTempThreadId;
		CString strStatus;


		#if( MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL )
			g_hMotorCommPort = OpenCommPort( m_strMotorSerialPort,  19200, MOTOR_COMM_DEVICE );	// COMx,  19,200 baud

		#elif MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL
			g_hMotorCommPort = OpenCommPort( m_strMotorSerialPort, 250000, MOTOR_COMM_DEVICE );	// COMx, 250,000 baud

		#elif MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL
			g_hMotorCommPort = OpenCommPort( m_strMotorSerialPort, CBR_57600, MOTOR_COMM_DEVICE );	// COMx, 57600 baud

		//#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL
			//g_hMotorCommPort = OpenCommPort( m_strMotorSerialPort, CBR_57600, MOTOR_COMM_DEVICE );	// COMx, 57600 baud

		#else
			ROBOT_DISPLAY( TRUE, "Motor COM Port NOT USED for this MOTOR_CONTROL_TYPE" )
			// return;
		#endif

		if (g_hMotorCommPort == INVALID_HANDLE_VALUE)
		{
			ROBOT_DISPLAY( TRUE, "Unable to open Motor COM Port %s", m_strMotorSerialPort )

			// ***********************************************************
			// TURN THIS ON TO DEBUG!!!
			//		g_MotorControlDebug = TRUE;
			//      Set ROBOT_SIMULATION_MODE
			//		ROBOT_DISPLAY( TRUE, "Motor Debug Mode Enabled" )
			//		ROBOT_DISPLAY( TRUE, "MOTOR SIMULATION ENABLED!" )
			//		g_hMotorCommPort = SIMULATED_SIO_HANDLE;
			// ***********************************************************

		}
		else
		{
			ROBOT_DISPLAY( TRUE, "Motor COM Port %s Opened", m_strMotorSerialPort );

			#if ( MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL )
				// Create the thread that will be reading the comm port and pass it a pointer to the com port
				DWORD dwTempThreadId;
				g_hMotorReadThread = CreateThread(NULL, 0, iRobotCommReadThreadFunc, (LPVOID)&g_hMotorCommPort, 0, &dwTempThreadId);
				if( NULL == g_hMotorReadThread )	
				{
					ROBOT_LOG( TRUE,  "Error Creating iRobot Read Comm Thread." )
					CloseCommPort( "Motor", g_hMotorCommPort );
				}
				else
				{
					ROBOT_LOG( TRUE,  "Created iRobot Read Comm Thread. ID = (0x%x)", dwTempThreadId )
				}

			#endif
		}
		if (g_hMotorCommPort != INVALID_HANDLE_VALUE)	// make sure it's still valid (for iRobot, might have been closed if read thread create failed)
		{
			// Create the thread that will be writing to the comm port
			HANDLE g_hMotorWriteThread = CreateThread(NULL, 0, MotorCommThreadFunc, (LPVOID)g_hMotorCommPort, 0, &g_dwMotorCommThreadId);
			if(g_hMotorWriteThread == NULL)	
			{
				ROBOT_LOG( TRUE, "Error Creating Motor Comm Write Thread." )
				CloseCommPort( "Motor", g_hMotorCommPort );
			}
			else
			{
				ROBOT_LOG( TRUE,  "Created Motor Comm Write Thread. ID = (0x%x)", g_dwMotorCommThreadId )
				#if MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL
					g_MotorSubSystemStatus = SUBSYSTEM_CONNECTED; // Currently, we don't read status from the TREX motor control, so just assume all is good if the port opens OK
				#endif
			}
		}
	#endif	// (MOTOR_CONTROL_TYPE != ARDUINO_MOTOR_CONTROL or KOBUKI_MOTOR_CONTROL )
#endif	// ROBOT_SERVER code
}

////////////////////////////////////////////////////////////////////
void Setup::OpenSmartServoPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Smart Servo Ports (Dyna and Kerr) ==================\n" )
	CString strStatus;

	// Open The AX12 Comm Port
	if( INVALID_HANDLE_VALUE == g_hDynaServoCommPort_AX12 )
	{
		g_hDynaServoCommPort_AX12 = OpenCommPort( m_strDynaServoSerialPort, 1000000, DYNA_SERVO_AX12_COMM_DEVICE );	// COMx, 1 Megabit!
		if (g_hDynaServoCommPort_AX12 == INVALID_HANDLE_VALUE)
		{
			ROBOT_DISPLAY( TRUE, "Dynamixel AX12 Servo COM Port %s FAILED <------", m_strDynaServoSerialPort ) 
		}
		else
		{
			ROBOT_DISPLAY( TRUE, "Dynamixel AX12 Servo COM Port %s Opened", m_strDynaServoSerialPort )
		}
	}

	#if( LOKI == ROBOT_TYPE )

		// Open The RX64 Comm Port
		#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
			if( INVALID_HANDLE_VALUE == g_hDynaServoCommPort_RX64 )
			{
				g_hDynaServoCommPort_RX64 = OpenCommPort( m_strDynaRX64SerialPort, 1000000, DYNA_SERVO_RX64_COMM_DEVICE );	// COMx, 1 Megabit!
				if (g_hDynaServoCommPort_RX64 == INVALID_HANDLE_VALUE)
				{
					ROBOT_DISPLAY( TRUE, "Dynamixel RX64 Servo COM Port %s FAILED <------", m_strDynaRX64SerialPort )
				}
				else
				{
					ROBOT_DISPLAY( TRUE, "Dynamixel RX64 Servo COM Port %s Opened", m_strDynaRX64SerialPort )
				}
			}
		#endif

		// Open The KERR Comm Port
		if( INVALID_HANDLE_VALUE == g_hKerrServoCommPort )
		{
			g_hKerrServoCommPort = OpenCommPort( m_strKerrServoSerialPort, CBR_19200, KERR_SERVO_COMM_DEVICE );	// COMx, 19,200 baud
			if (g_hKerrServoCommPort == INVALID_HANDLE_VALUE)
			{
				ROBOT_DISPLAY( TRUE, "Kerr Servo COM Port %s FAILED <------", m_strKerrServoSerialPort )
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "Kerr Servo COM Port %s Opened", m_strKerrServoSerialPort )
			}
		}
	#endif

	#if( LOKI == ROBOT_TYPE )
		if( (g_hDynaServoCommPort_AX12  != INVALID_HANDLE_VALUE) &&
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				(g_hDynaServoCommPort_RX64  != INVALID_HANDLE_VALUE) &&
			#endif
			(g_hKerrServoCommPort		!= INVALID_HANDLE_VALUE) )
	#else
		if( (g_hDynaServoCommPort_AX12  != INVALID_HANDLE_VALUE) )
	#endif
	{
		// Create the thread that will be writing to and reading from the comm ports
		g_hSmartServoThread = CreateThread(NULL, 0, SmartServoCommThreadFunc, (LPVOID)0, 0, &g_dwSmartServoCommThreadId);
		if(g_hSmartServoThread == NULL)	
		{
			ROBOT_LOG( TRUE, "Error Creating Smart Servo Comm Thread.\n" )
			CloseCommPort( "Dynamixel AX12" , g_hDynaServoCommPort_AX12 );
			CloseCommPort( "Dynamixel RX64" , g_hDynaServoCommPort_RX64 );
			CloseCommPort( "Kerr" , g_hKerrServoCommPort );
			ROBOT_LOG( TRUE, "Smart Servo COMM ports closed.\n" )
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created SmartServo Comm Thread. ID = (0x%x)", g_dwSmartServoCommThreadId )
			ROBOT_DISPLAY( TRUE, "Smart Servo com ports Opened Sucessfully.  Smart Servo Thread started!" )
			ROBOT_DISPLAY( TRUE, "======================================================================\n" )

		}
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "\n==================================================================================" )
		ROBOT_DISPLAY( TRUE, "Error Opening some of the Smart Servo Comm ports.  Smart Servo Thread not started!" )
		ROBOT_DISPLAY( TRUE, "==================================================================================\n" )
	}

#endif	// ROBOT_SERVER code
}


////////////////////////////////////////////////////////////////////
void Setup::OpenArduinoPort()
{
#if ( ROBOT_SERVER == 1 )		// ROBOT_SERVER code
	DWORD dwTempThreadId;
	CString strStatus;
	
	ROBOT_LOG( TRUE,  "DEBUG ========================== SETUP View Opening Arduino Port ==================\n" )
	g_hArduinoCommPort = OpenCommPort(_T(m_strPicSerialPort), CBR_19200, ARDUINO_COMM_DEVICE);	// COMx, 19200baud
//	g_hArduinoCommPort = OpenCommPort(_T(m_strPicSerialPort), CBR_9600, PIC_COMM_DEVICE);	// COMx, 9600baud
	if( (g_hArduinoCommPort == INVALID_HANDLE_VALUE) || (g_hArduinoCommPort == SIMULATED_SIO_HANDLE) )
	{
		if ( 1 == ROBOT_SIMULATION_MODE )
		{
			ROBOT_DISPLAY( TRUE, "Arduino SIMULATION ENABLED!" )
			g_hArduinoCommPort = SIMULATED_SIO_HANDLE;

		}
		else 
		{
			if( (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI) || (SENSOR_CONFIG_TYPE ==  SENSOR_CONFIG_KOBUKI_WITH_ARDUINO) )
			{
				// Loki and Turtle with Arduino REQUIRES Arduino!
				ROBOT_DISPLAY( TRUE, "Error!! Unable to open Arduino COM Port %s", m_strPicSerialPort )
				AfxMessageBox("Unable to open Arduino COM Port, REQUIRED for Loki!");

				//ROBOT_ASSERT(0);
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "Unable to open Arduino COM Port %s", m_strPicSerialPort )
			}
		}


		// Setup initial values for simulation
		#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT
			//#define NUM_IR_SENSORS					  4 // 2 forward facing, 2 side facing
			//#define NUM_US_SENSORS					  7 // #0 mounted on camera, #1-6 in half-circle
			g_RawArduinoStatus.US[0] = 125;	// Center sensor
			g_RawArduinoStatus.US[1] = 42;	
			g_RawArduinoStatus.US[2] = 42;	
			g_RawArduinoStatus.US[3] = 42;
			g_RawArduinoStatus.US[4] = 42;
			g_RawArduinoStatus.US[5] = 42;
			g_RawArduinoStatus.US[6] = 42;

			g_RawArduinoStatus.IR[0] = 75; // Center Left
			g_RawArduinoStatus.IR[1] = 75;
			g_RawArduinoStatus.IR[2] = 35;	// Center Left
			g_RawArduinoStatus.IR[3] = 35;	// Center Right

		#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI
			//#define NUM_IR_SENSORS					  6	// 2 head mounted LR, 2 front facing LR, 2 side mounted SR 
			//#define NUM_US_SENSORS					  3	// 1 head mounted, 2 front mounted
/* DISABLED
			g_RawArduinoStatus.US[0] = 125;	// Center sensor
			g_RawArduinoStatus.US[1] = 105;	// left
			g_RawArduinoStatus.US[2] = 105;	// right
*/
			g_RawArduinoStatus.IR[0] = 75;
			g_RawArduinoStatus.IR[1] = 50;
			g_RawArduinoStatus.IR[2] = 35;	// Center Left
			g_RawArduinoStatus.IR[3] = 35;	// Center Right
			g_RawArduinoStatus.IR[4] = 50;
			g_RawArduinoStatus.IR[5] = 75;
		#endif

		Sleep(2000); // give time for speaking thread to start up
		SpeakText( "Simulation Mode" );

		//AfxMessageBox( _T("Unable to open Arduino COM Port!") );

		// DEBUG For Testing New Sound files!
		/**
		for( int i=9; i>0; i-- )
		{
			PostThreadMessage( g_dwSoundThreadId, (WM_ROBOT_PLAY_SOUND), i, 0);
			Sleep(5000);	
		}
		**/
	
	}
	else
	{

		ROBOT_DISPLAY( TRUE, "Arduino COM Port %s Opened", m_strPicSerialPort )

		// Create the thread that will be reading the comm port
		g_hArduinoReadThread = CreateThread(NULL, 0, ArduinoCommReadThreadFunc, (LPVOID)g_hArduinoCommPort, 0, &dwTempThreadId);
		if(g_hArduinoReadThread == NULL)	
		{
			ROBOT_LOG( TRUE, "Error Creating Comm Read Thread." )
			CloseCommPort( "Arduino", g_hArduinoCommPort );
		}
		else
		{
			ROBOT_LOG( TRUE,  "Created Arduino Comm Read Thread. ID = (0x%x)", dwTempThreadId )
		}
	}


	// Create the thread that will be writing to the Arduino comm port,
	// Or simulating the Arduino (if a real Arduino is not connected)
	// The thread handle needs to be saved, so messages can be sent to the thread.


	g_hArduinoWriteThread = CreateThread(NULL, 0, ArduinoCommWriteThreadFunc, (LPVOID)g_hArduinoCommPort, 0, &g_dwArduinoCommWriteThreadId);
	if(g_hArduinoWriteThread == NULL)	
	{
		ROBOT_LOG( TRUE, "Error Creating Comm Write Thread." )
		CloseCommPort( "Arduino", g_hArduinoCommPort );
	}
	else
	{
		ROBOT_LOG( TRUE,  "Created Arduino Comm Write Thread. ID = (0x%x)", g_dwArduinoCommWriteThreadId )

		// Send command to get version
		int Message = WM_ROBOT_MESSAGE_BASE+HW_GET_VERSION;
		PostThreadMessage( g_dwArduinoCommWriteThreadId, Message, 0, 0 );
	}

#endif	// ROBOT_SERVER code
}


////////////////////////////////////////////////////////////////////

void Setup::OnSaveToDisk() 
{
	if (UpdateData() != TRUE)
		return;

	GetDocument()->m_strPicSerialPort = m_strPicSerialPort;
	GetDocument()->m_strGpsSerialPort = m_strGpsSerialPort;
	GetDocument()->m_strServoSerialPort = m_strServoSerialPort;
	GetDocument()->m_strDynaServoSerialPort = m_strDynaServoSerialPort;
	GetDocument()->m_strDynaRX64SerialPort = m_strDynaRX64SerialPort;
	GetDocument()->m_strKerrServoSerialPort = m_strKerrServoSerialPort;
	GetDocument()->m_strMotorSerialPort = m_strMotorSerialPort;
	GetDocument()->m_strCameraSerialPort = m_strCameraSerialPort;
	GetDocument()->m_strRobotIPAddress = m_strRobotIPAddress;
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

}

void Setup::OnConnectToHost() 
{
	ConnectToHost( FALSE ); 
}

void Setup::ConnectToHost( BOOL LocalHost ) 
{
	IGNORE_UNUSED_PARAM (LocalHost);

#if ( ROBOT_SERVER != 1 )	// ROBOTCLIENT only

	if( LocalHost )
	{
		strncpy_s( g_ClientSockStruct.szIPAddress, "127.0.0.1", sizeof(g_ClientSockStruct.szIPAddress) );
	}
	else
	{
		// Get Server IP address
		//UpdateData(TRUE);	// Force data exchange from GUI to data members
		//g_ClientSockStruct.szIPAddress = m_strRobotIPAddress;
		UINT result = GetDlgItemText( IDC_IP_ADDR1, g_ClientSockStruct.szIPAddress, 99); 
		GetDocument()->m_strRobotIPAddress = g_ClientSockStruct.szIPAddress;	// Save it
	}

	if( ConnectSocket(&g_ClientSockStruct) )
	{
		// Connected

		// Create client Send socket thread
		//DWORD dwClientSocketThreadId;
		g_ClientSockStruct.hClientSockSendThread = 
			CreateThread( NULL, 0, ClientSockSendThreadProc, 
			(LPVOID)&g_ClientSockStruct, 0, &(g_dwClientSendThreadId) );
		ROBOT_LOG( TRUE,  "Created Client Socket Send Thread. ID = (0x%x)", g_dwClientSendThreadId )

		// Create client Receive socket thread
		DWORD dwTempThreadId;
		g_ClientSockStruct.hClientSockReceiveThread = 
			CreateThread( NULL, 0, ClientSockReceiveThreadProc, 
			(LPVOID)&g_ClientSockStruct, 0, &dwTempThreadId );
		ROBOT_LOG( TRUE,  "Created Client Socket Receive Thread. ID = (0x%x)", dwTempThreadId )

		//g_ClientSockStruct.bConnectionEnabled = TRUE;  // UDP Socket bind sucessful, threads started

		// Send a message to server so it will establish a connection with us
		// SendCommand( WM_ROBOT_CLIENT_CONNECT, 0, 0 );
	}
#else
	ROBOT_DISPLAY( TRUE, "This app built in Server Mode!  Connect to Host not supported!" )

#endif // ROBOTCLIENT only

}


//////////////////////////////////////////////////////////////////////////
// Frame Button Handlers
//


void Setup::OnSelSetupViewBtn() 
{
	GetParentFrame()->ActivateFrame();	//RecalcLayout();
}
void Setup::OnSelCmdViewBtn() 
{
	::PostMessage( g_RobotCmdViewHWND, (UINT)WM_COMMAND, ID_SEL_CMD_VIEW_BTN, 0);	
}
void Setup::OnSelMapViewBtn() 
{
	::PostMessage( g_RobotMapViewHWND, (UINT)WM_COMMAND, ID_SEL_MAP_VIEW_BTN, 0);	
}
void Setup::OnSelPathViewBtn() 
{
	::PostMessage( g_RobotPathViewHWND, (UINT)WM_COMMAND, ID_SEL_PATH_VIEW_BTN, 0);	
}


void Setup::OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);

}

void Setup::OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotCmdViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void Setup::OnUpdateSelMapViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotMapViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void Setup::OnUpdateSelPathViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotPathViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}




void Setup::OnCameraPwrOn() 
{
	SendCommand( WM_ROBOT_CAMERA_POWER_CMD, 0, (DWORD)POWER_ON );
	
}

void Setup::OnCameraPwrOff() 
{
	SendCommand( WM_ROBOT_CAMERA_POWER_CMD, 0, (DWORD)POWER_OFF );
	
}

void Setup::OnConnectToIpCamera() 
{
	// Get Camera IP address
	char	szCameraIPAddress[100];
	UINT result = GetDlgItemText( IDC_IP_ADDR2, szCameraIPAddress, 99); 
	//mHtmlView.Navigate
// TODO-CAR-MUST	m_HtmlCtrl.Navigate2(_T(szCameraIPAddress), NULL, NULL);

}


void Setup::OnSelchangeAvoidObjRange() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_AVOID_OBJ_RANGE);
	int nRange = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Requesting New Avoid Obj Range: %d feet\n", nRange )
	SendCommand( WM_ROBOT_SET_AVOID_OBJ_RANGE, 0, (DWORD)nRange );
}

void Setup::OnSelchangeCameraTrackingColorThreshold() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_CAMERA_TRACKING_COLOR_THRESHOLD);
	int nColorThreshold = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Requesting New Color Threshold: %d\n", nColorThreshold )

	SendCommand( WM_ROBOT_SET_CAMERA_COLOR_THRESHOLD, 0, (DWORD)nColorThreshold );
}

void Setup::OnCameraScanDistanceBtn() 
{
	char	szDistance[32];
	UINT result = GetDlgItemText( IDC_CAMERA_SCAN_DISTANCE, szDistance, 31);

	int nScanDistance = atoi(szDistance);
	ROBOT_LOG( TRUE, "Setting Path Cone Scan Distance = %d\n", nScanDistance )
	SendCommand( WM_ROBOT_SET_SCAN_DISTANCE_CMD, 0, (DWORD)nScanDistance );
	
}

void Setup::OnColorAutoCalBtn() 
{
	// Send comand to calibrate Color Blob Tracking to color currently in center of camera view
	SendCommand( WM_ROBOT_COLOR_BLOB_AUTO_CAL_CMD, 0, 0 );
	
}

void Setup::OnCameraManualColorCalBtn() 
{
	char	szCal[32];
	UINT result;

	result = GetDlgItemText( IDC_CAMERA_COLOR_CR, szCal, 31); 
	int nCrValue = atoi(szCal);

	result = GetDlgItemText( IDC_CAMERA_COLOR_CB, szCal, 31); 
	int nCbValue = atoi(szCal);

	ROBOT_LOG( TRUE, "Setting Calibrated Cr = %d, Cb = 0\n", nCrValue )

	// Send comand to calibrate Color Blob manually
	SendCommand( WM_ROBOT_COLOR_BLOB_MANUAL_CAL_CMD, nCrValue, nCbValue );
}

void Setup::OnConnectToLocalHost() 
{
	ConnectToHost( TRUE ); 
	
}

void Setup::OnRadarScan() 
{
	static BOOL bRadarOn = FALSE;	// Toggle to track button up/down state

	if( bRadarOn )
	{
		// Stop
		bRadarOn = FALSE;
		SetDlgItemText(IDC_RADAR_SCAN, "Radar On");
		SendCommand( WM_ROBOT_ENABLE_RADAR_SCAN_CMD, 0/*Sensor*/, 0/*On/Off*/ ); 
//		SendCommand( WM_ROBOT_ENABLE_RADAR_SCAN_CMD, 2/*Sensor*/, 0/*On/Off*/ ); 
	}
	else
	{
		// Start
		bRadarOn = TRUE;
		SetDlgItemText(IDC_RADAR_SCAN, "Radar Off");
		SendCommand( WM_ROBOT_ENABLE_RADAR_SCAN_CMD, 0/*Sensor*/, 1/*On/Off*/ );  
//		SendCommand( WM_ROBOT_ENABLE_RADAR_SCAN_CMD, 2/*Sensor*/, 1/*On/Off*/ );  
	}
	
}

void Setup::OnCompassCalMode() 
{
	static BOOL bCalMode = FALSE;	// Toggle to track button up/down state

	if( !bCalMode )
	{
		// Start
		bCalMode = TRUE;
		SetDlgItemText(IDC_RADAR_SCAN, "Compass Cal");
		ROBOT_LOG( TRUE, "Sending Compass Calibration Mode START command\n" )
		SendCommand( WM_ROBOT_SET_COMPASS_CAL_MODE, 0, 1 );	// 1 = Start
	}
	else
	{
		// Stop
		bCalMode = FALSE;
		SetDlgItemText(IDC_RADAR_SCAN, "Cal Stop");
		ROBOT_LOG( TRUE, "Sending Compass Calibration Mode STOP command\n" )
		SendCommand( WM_ROBOT_SET_COMPASS_CAL_MODE, 0, 0 ); //0 = Stop
	}
}

void Setup::OnCompassCalPoint() 
{
		ROBOT_LOG( TRUE, "Sending Compass Calibration POINT command\n" )
		SendCommand( WM_ROBOT_SET_COMPASS_CAL_POINT, 0, 0 );
}

void Setup::OnResetOdometer() 
{
	SendCommand( WM_ROBOT_RESET_ODOMETER, 0, 0 );
	
}

void Setup::OnResetWatchdog() 
{
	// Process the Reset Watchdog button.
	// Resets without turning on 12v
	SendCommand( WM_ROBOT_RESET_WATCHDOG_CMD, 0, 0 );
}

void Setup::OnStartZcSvc() 
{
	StopWirelessZeroConfig();
	
}

void Setup::OnStopZcSvc() 
{
	StartWirelessZeroConfig();
	
}




/////////////////////////////////////////////////////////////////////////////
// Utilities - Start and stop the 802.11 "Zero Config" service
// This service is needed to connect to 802.11, but then causes a 3.5 second
// delay in IP traffic every 63 seconds!
//

void Setup::StartWirelessZeroConfig() 
{ 
    SERVICE_STATUS ssStatus; 
    DWORD dwOldCheckPoint; 
    DWORD dwStartTickCount;
    DWORD dwWaitTime;
    DWORD dwStatus;

	// Open the SCM database
	SC_HANDLE hSCManager = OpenSCManager( NULL, NULL, SC_MANAGER_CONNECT );
	if ( !hSCManager ) 
	{
		ROBOT_LOG( TRUE,  "OpenSCManager()", GetLastError() )
		return;
	}


    SC_HANDLE schService = OpenService( 
        hSCManager,		// SCM database 
        "WZCSVC",		// Wireless Zero Config Service
        SERVICE_ALL_ACCESS); 
 
    if (schService == NULL) 
    { 
        ROBOT_LOG( TRUE,  "ERROR: OpenService\n" )
    }
 
    if (!StartService(
            schService,  // handle to service 
            0,           // number of arguments 
            NULL) )      // no arguments 
    {
        ROBOT_LOG( TRUE,  "ERROR: StartService\n" )
    }
    else 
    {
        ROBOT_LOG( TRUE,  "Service start pending.\n" )
    }
 
    // Check the status until the service is no longer start pending. 
 
    if (!QueryServiceStatus( 
            schService,   // handle to service 
            &ssStatus) )  // address of status information structure
    {
        ROBOT_LOG( TRUE,  "ERROR: QueryServiceStatus\n" )
    }
 
    // Save the tick count and initial checkpoint.

    dwStartTickCount = GetTickCount();
    dwOldCheckPoint = ssStatus.dwCheckPoint;

    while (ssStatus.dwCurrentState == SERVICE_START_PENDING) 
    { 
        // Do not wait longer than the wait hint. A good interval is 
        // one tenth the wait hint, but no less than 1 second and no 
        // more than 10 seconds. 
 
        dwWaitTime = ssStatus.dwWaitHint / 10;

        if( dwWaitTime < 1000 )
            dwWaitTime = 1000;
        else if ( dwWaitTime > 10000 )
            dwWaitTime = 10000;

        Sleep( dwWaitTime );

        // Check the status again. 
 
        if (!QueryServiceStatus( 
                schService,   // handle to service 
                &ssStatus) )  // address of structure
            break; 
 
        if ( ssStatus.dwCheckPoint > dwOldCheckPoint )
        {
            // The service is making progress.

            dwStartTickCount = GetTickCount();
            dwOldCheckPoint = ssStatus.dwCheckPoint;
        }
        else
        {
            if(GetTickCount()-dwStartTickCount > ssStatus.dwWaitHint)
            {
                // No progress made within the wait hint
                break;
            }
        }
    } 

    if (ssStatus.dwCurrentState == SERVICE_RUNNING) 
    {
        ROBOT_LOG( TRUE,  "StartService SUCCESS.\n" )
        dwStatus = NO_ERROR;
    }
    else 
    { 
        ROBOT_LOG( TRUE,  "\nService not started. \n" )
        ROBOT_LOG( TRUE,  "  Current State: %d\n", ssStatus.dwCurrentState)
        ROBOT_LOG( TRUE,  "  Exit Code: %d\n", ssStatus.dwWin32ExitCode)
        ROBOT_LOG( TRUE,  "  Service Specific Exit Code: %d\n",
            ssStatus.dwServiceSpecificExitCode)
        ROBOT_LOG( TRUE,  "  Check Point: %d\n", ssStatus.dwCheckPoint)
        ROBOT_LOG( TRUE,  "  Wait Hint: %d\n", ssStatus.dwWaitHint)
        dwStatus = GetLastError();
    } 
 
    CloseServiceHandle(schService); 
} 



DWORD Setup::StopService( 
	  SC_HANDLE hSCM, 
	  SC_HANDLE hService, 
      BOOL fStopDependencies, 
	  DWORD dwTimeout ) 
{
   SERVICE_STATUS ss;
   DWORD dwStartTime = GetTickCount();

   // Make sure the service is not already stopped
   if ( !QueryServiceStatus( hService, &ss ) )
      return GetLastError();

   if ( ss.dwCurrentState == SERVICE_STOPPED ) 
      return ERROR_SUCCESS;

   // If a stop is pending, just wait for it
   while ( ss.dwCurrentState == SERVICE_STOP_PENDING ) 
   {
      Sleep( ss.dwWaitHint );
      if ( !QueryServiceStatus( hService, &ss ) )
         return GetLastError();

      if ( ss.dwCurrentState == SERVICE_STOPPED )
         return ERROR_SUCCESS;

      if ( GetTickCount() - dwStartTime > dwTimeout )
         return ERROR_TIMEOUT;
   }

   // If the service is running, dependencies must be stopped first
   if ( fStopDependencies ) 
   {
      DWORD i;
      DWORD dwBytesNeeded;
      DWORD dwCount;

      LPENUM_SERVICE_STATUS   lpDependencies = NULL;
      ENUM_SERVICE_STATUS     ess;
      SC_HANDLE               hDepService;

      // Pass a zero-length buffer to get the required buffer size
      if ( EnumDependentServices( hService, SERVICE_ACTIVE, 
         lpDependencies, 0, &dwBytesNeeded, &dwCount ) ) 
      {
         // If the Enum call succeeds, then there are no dependent
         // services so do nothing
      } 
      else 
      {
         if ( GetLastError() != ERROR_MORE_DATA )
            return GetLastError(); // Unexpected error

         // Allocate a buffer for the dependencies
         lpDependencies = (LPENUM_SERVICE_STATUS) HeapAlloc( 
               GetProcessHeap(), HEAP_ZERO_MEMORY, dwBytesNeeded );

         if ( !lpDependencies )
            return GetLastError();

         __try {
            // Enumerate the dependencies
            if ( !EnumDependentServices( hService, SERVICE_ACTIVE, 
                  lpDependencies, dwBytesNeeded, &dwBytesNeeded,
                  &dwCount ) )
               return GetLastError();

            for ( i = 0; i < dwCount; i++ ) 
            {
               ess = *(lpDependencies + i);

               // Open the service
               hDepService = OpenService( hSCM, ess.lpServiceName, 
                     SERVICE_STOP | SERVICE_QUERY_STATUS );
               if ( !hDepService )
                  return GetLastError();

               __try {
                   // Send a stop code
                  if ( !ControlService( hDepService, 
                           SERVICE_CONTROL_STOP,
                           &ss ) )
                     return GetLastError();

                  // Wait for the service to stop
                  while ( ss.dwCurrentState != SERVICE_STOPPED ) 
                  {
                      Sleep( ss.dwWaitHint );
                     if ( !QueryServiceStatus( hDepService, &ss ) )
                        return GetLastError();

                     if ( ss.dwCurrentState == SERVICE_STOPPED )
                        break;

                     if ( GetTickCount() - dwStartTime > dwTimeout )
                        return ERROR_TIMEOUT;
                  }

               } 
               __finally 
               {
                  // Always release the service handle
                  CloseServiceHandle( hDepService );

               }
            }

         } 
         __finally 
         {
            // Always free the enumeration buffer
            HeapFree( GetProcessHeap(), 0, lpDependencies );
         }
      } 
   }

   // Send a stop code to the main service
   if ( !ControlService( hService, SERVICE_CONTROL_STOP, &ss ) )
      return GetLastError();

   // Wait for the service to stop
   while ( ss.dwCurrentState != SERVICE_STOPPED ) 
   {
      Sleep( ss.dwWaitHint );
      if ( !QueryServiceStatus( hService, &ss ) )
         return GetLastError();

      if ( ss.dwCurrentState == SERVICE_STOPPED )
         break;

      if ( GetTickCount() - dwStartTime > dwTimeout )
         return ERROR_TIMEOUT;
   }

   // Return success
   return ERROR_SUCCESS;
}

// Helper function to display an error message 
void Setup::DisplayError( LPTSTR szAPI, DWORD dwError ) 
{
	IGNORE_UNUSED_PARAM (szAPI);
	LPTSTR lpBuffer = NULL;

   FormatMessage( FORMAT_MESSAGE_ALLOCATE_BUFFER |
         FORMAT_MESSAGE_FROM_SYSTEM, NULL, dwError,
         MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
         (LPTSTR) &lpBuffer, 0, NULL );

   ROBOT_LOG( TRUE,  "%s failed:\n", szAPI )
   ROBOT_LOG( TRUE,  "    error code = %u\n", dwError )
   ROBOT_LOG( TRUE,  "    message    = %s\n", lpBuffer )

   LocalFree( lpBuffer );
}

void Setup::StopWirelessZeroConfig() 
// Stop the Windows Zero Config Service, which causes 802.11 to stop 
// for 3.5 seconds every 63 seconds!
{
   SC_HANDLE hSCM;
   SC_HANDLE hService;
   DWORD     dwError;

	char ServiceName[] = "WZCSVC";

   __try 
   {
      // Open the SCM database
      hSCM = OpenSCManager( NULL, NULL, SC_MANAGER_CONNECT );
      if ( !hSCM ) 
      {
         DisplayError( "OpenSCManager()", GetLastError() );
//         DisplayError( "OpenSCManager()", GetLastError() );
         __leave;
      }

      // Open the specified service
      hService = OpenService( hSCM, ServiceName, SERVICE_STOP
            | SERVICE_QUERY_STATUS | SERVICE_ENUMERATE_DEPENDENTS );
      if ( !hService ) 
      {
         DisplayError( "OpenService()", GetLastError() );
         __leave;
      }

      // Try to stop the service, specifying a 30 second timeout
      dwError = StopService( hSCM, hService, TRUE, 30000 ) ;
      if ( dwError == ERROR_SUCCESS )
         _tprintf( "Service stopped.\n" );
      else
         DisplayError( "StopService()", dwError );

   } 
   __finally 
   {
      if ( hService )
         CloseServiceHandle( hService );

      if ( hSCM )
         CloseServiceHandle( hSCM );
   }
}


void Setup::OnGpsOpenPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strGpsSerialPort = m_strGpsSerialPort;

	OpenGPSPort();
	
}

void Setup::OnOpenServoPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strServoSerialPort = m_strServoSerialPort;

	OpenServoPort();
	
}

void Setup::OnOpenDynaServoPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strDynaServoSerialPort = m_strDynaServoSerialPort;

	OpenSmartServoPort(); // Opens AX12, RX64, and Kerr ports
		
}

void Setup::OnBnClickedOpenDynaRx64ServoPort()
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strDynaRX64SerialPort = m_strDynaRX64SerialPort;

	OpenSmartServoPort(); // Opens AX12, RX64, and Kerr ports
}


void Setup::OnOpenKerrServoPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strKerrServoSerialPort = m_strKerrServoSerialPort;

	OpenSmartServoPort(); // Opens AX12, RX64, and Kerr ports
	
}


void Setup::OnOpenMotorPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strMotorSerialPort = m_strMotorSerialPort;

	OpenMotorPort();
	
}

void Setup::OnOpenCameraPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strCameraSerialPort = m_strCameraSerialPort;

	OpenCameraPort();
	
}

void Setup::OnSelchangeCamPanSpeed() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_PAN_SPEED);
	int nCameraPanSpeed = (pComboBox->GetCurSel() + 1) * 16;
	if( nCameraPanSpeed > SERVO_SPEED_MAX )
	{
		nCameraPanSpeed = SERVO_SPEED_MAX;
	}
	ROBOT_LOG( TRUE, "Requesting New camera pan speed: %02X\n", nCameraPanSpeed )

/*
#define SERVO_SPEED_STOP					0x00	// Steps by 16 each, to allow for variable speeds if needed
#define SERVO_SPEED_VERY_SLOW				0x10	
#define SERVO_SPEED_SLOW					0x20
#define SERVO_SPEED_MED_SLOW				0x30
#define SERVO_SPEED_MED						0x40
#define SERVO_SPEED_MED_FAST				0x50
#define SERVO_SPEED_FAST					0x60
#define SERVO_SPEED_MAX						0x70
*/



	SendCommand( WM_ROBOT_USER_CAMERA_PAN_TILT_SPEED_CMD, 0, (DWORD)nCameraPanSpeed );

}

void Setup::OnCameraPosAbsBtn() 
{
	char	szPosition[32];
	UINT result = GetDlgItemText( IDC_CAMERA_POS_ABS_VALUE_X, szPosition, 31);
	int nPositionX = atoi(szPosition);
	result = GetDlgItemText( IDC_CAMERA_POS_ABS_VALUE_Y, szPosition, 31);
	int nPositionY = atoi(szPosition);

	ROBOT_LOG( TRUE, "Moving Camera to absolute position %d,%d\n", nPositionX, nPositionY )
	SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD,  (DWORD)nPositionX, 0 );
	SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)nPositionY, 0 );

}

void Setup::OnOpenArduinoPort() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strPicSerialPort = m_strPicSerialPort;

	OpenArduinoPort();
	
	
}

void Setup::PostNcDestroy() 
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )
	
	CFormView::PostNcDestroy();
}


void Setup::EnableCamera( UINT nCamera, BOOL bEnable ) 
{
	if( bEnable )
	{
		// if any camera is enabled, center the camera to start
		SendCommand( WM_ROBOT_USER_CAMERA_PAN_CMD, (DWORD)CAMERA_PAN_ABS_CENTER, 6 ); // Initialize camera to center position
		
		g_Camera[LEFT_CAMERA].FrameSize = GetFrameSize(GetDocument()->m_VidCapSizeSelected);	// Always set Left and Right camera to the same frame size
		g_Camera[RIGHT_CAMERA].FrameSize = GetFrameSize(GetDocument()->m_VidCapSizeSelected);
		g_Camera[LEFT_CAMERA].DisplaySize = GetFrameSize(GetDocument()->m_VideoDisplaySizeSelected);	// Display size can be different than frame size
		g_Camera[RIGHT_CAMERA].DisplaySize = GetFrameSize(GetDocument()->m_VideoDisplaySizeSelected);
	}

	if( (LEFT_CAMERA == nCamera) || (ALL_CAMERAS == nCamera) )
	{
		if( bEnable)
		{
			g_Camera[LEFT_CAMERA].State = CAMERA_STATE_INIT_REQUESTED;
		}
		else
		{
			// Diable camera
			if( CAMERA_STATE_NOT_ENABLED != g_Camera[LEFT_CAMERA].State )
			{
				g_Camera[LEFT_CAMERA].State = CAMERA_STATE_SHUTDOWN_REQUESTED;
			}
		}
	}

	if( (RIGHT_CAMERA == nCamera) || (ALL_CAMERAS == nCamera) )
	{
		if( bEnable)
		{
			g_Camera[RIGHT_CAMERA].State = CAMERA_STATE_INIT_REQUESTED;
		}
		else
		{
			// Diable camera
			if( CAMERA_STATE_NOT_ENABLED != g_Camera[RIGHT_CAMERA].State )
			{
				g_Camera[RIGHT_CAMERA].State = CAMERA_STATE_SHUTDOWN_REQUESTED;
			}
			if( CAMERA_STATE_NOT_ENABLED != g_Camera[LEFT_CAMERA].State )
			{
				g_Camera[LEFT_CAMERA].State = CAMERA_STATE_SHUTDOWN_REQUESTED;	// KLUDGE for stereo
			}
		}
	}

	if( (KINECT_DEPTH == nCamera) || (ALL_CAMERAS == nCamera) )
	{
		if( bEnable)
		{
			g_Camera[KINECT_DEPTH].FrameSize.cx = DEPTH_CAPTURE_SIZE_MAX_X;	// TODO: THIS IS NOT USED, BUT IF WAS IS A BUG! Kinect video size is NOT fixed
			g_Camera[KINECT_DEPTH].FrameSize.cy = DEPTH_CAPTURE_SIZE_MAX_Y;
			g_Camera[KINECT_VIDEO].FrameSize.cx = DEPTH_CAPTURE_SIZE_MAX_X;
			g_Camera[KINECT_VIDEO].FrameSize.cy = DEPTH_CAPTURE_SIZE_MAX_Y;
			g_Camera[KINECT_DEPTH].DisplaySize = GetFrameSize(GetDocument()->m_KinectDisplaySizeSelected);
			g_Camera[KINECT_VIDEO].DisplaySize = GetFrameSize(GetDocument()->m_KinectDisplaySizeSelected);
			g_Camera[KINECT_DEPTH].Flip = GetDocument()->m_bEnableKinectVideoFlip;
			g_Camera[KINECT_VIDEO].Flip = GetDocument()->m_bEnableKinectVideoFlip;
			g_Camera[KINECT_DEPTH].State = CAMERA_STATE_INITIALIZED;
		}
		else
		{
/* TODO 			// Diable camera
			if( CAMERA_STATE_NOT_ENABLED != g_Camera[KINECT_DEPTH].State )
			{
				g_Camera[KINECT_DEPTH].State = CAMERA_STATE_SHUTDOWN_REQUESTED;
			}
			*/
		}
	}

	if( (KINECT_VIDEO == nCamera) || (ALL_CAMERAS == nCamera) )
	{
		if( bEnable)
		{
			g_Camera[KINECT_VIDEO].State = CAMERA_STATE_INIT_REQUESTED;
		}
		else
		{
			// Diable camera
			if( CAMERA_STATE_NOT_ENABLED != g_Camera[KINECT_VIDEO].State )
			{
				g_Camera[KINECT_VIDEO].State = CAMERA_STATE_SHUTDOWN_REQUESTED;
			}
		}
	}


}

void Setup::OnCameraEnableShowMotionView() 
{
	BOOL bEnable = IsDlgButtonChecked(IDC_CAMERA_ENABLE_SHOW_MOTION_VIEW);
	GetDocument()->m_bEnableShowMotionView = bEnable;	// Persist
	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_SHOW_MOTION_VIEW, (DWORD)bEnable );
}


void Setup::OnEnableVideoProcessing() 
{
	// Enable or disable all video processing (overrides checkboxes below)
	if( IsDlgButtonChecked(IDC_ENABLE_VIDEO_PROCESSING) )
	{
		GetDocument()->m_bEnableEnableVideoProcessing = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_VIDCAP_PROCESSING, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableEnableVideoProcessing = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_VIDCAP_PROCESSING, (DWORD)FALSE );
	}
	
}


void Setup::OnCameraEnableTrackingFace() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_FACE) )
	{
		GetDocument()->m_bEnableTrackingFace = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_FACE, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableTrackingFace = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_FACE, (DWORD)FALSE );
	}

}

void Setup::OnCameraEnableTrackingColors() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_COLORS) )
	{
		GetDocument()->m_bEnableTrackingColors = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_COLORS, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableTrackingColors = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_COLORS, (DWORD)FALSE );
	}
}

void Setup::OnCameraEnableFaceIdentification() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_FACE_IDENTIFY) )
	{
		GetDocument()->m_EnableFaceIdentification = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_FACE_IDENTIFICATION, (DWORD)TRUE );
		SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)CAMERA_TILT_TENTHDEGREES_TRACK_LASER_MAX_UP-50, (DWORD)TRUE );		
	}
	else
	{
		GetDocument()->m_EnableFaceIdentification = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_FACE_IDENTIFICATION, (DWORD)FALSE );
		SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, CAMERA_TILT_CENTER, (DWORD)TRUE );		
	}
}	

void Setup::OnCameraEnableTrackingCones() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_CONES) )
	{
		GetDocument()->m_bEnableTrackingCones = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_CONES, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableTrackingCones = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_CONES, (DWORD)FALSE );
	}
}

void Setup::OnCameraEnableTrackingObjects() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_OBJECTS) )
	{
		GetDocument()->m_bEnableTrackingObjects = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_OBJECTS, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableTrackingObjects = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_OBJECTS, (DWORD)FALSE );
	}
}

void Setup::OnSelchangeCamZoomLevelAbs() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_ZOOM_LEVEL_ABS);
	int nZoomLevel = pComboBox->GetCurSel();

	ROBOT_LOG( TRUE, "GUI Requesting Zoom Level: %d \n", nZoomLevel )
	SendCommand( WM_ROBOT_CAMERA_ZOOM_ABS_CMD, (DWORD)nZoomLevel, 0 );
}

void Setup::OnCameraEnableTrackingObjectMotion() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_OBJECT_MOTION) )
	{
		GetDocument()->m_bEnableTrackingMotion = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_MOTION, (DWORD)TRUE );
	}
	else
	{
		// Close the motion tracker window
		GetDocument()->m_bEnableTrackingMotion = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_MOTION, (DWORD)FALSE );
	}
	
}


void Setup::OnSelchangeCamFrameSize() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_FRAME_SIZE);
	GetDocument()->m_VidCapSizeSelected = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Set VidCap size to: %d\n", GetDocument()->m_VidCapSizeSelected )
}

void Setup::OnVideoFormatBtn() 
{
	SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_SHOW_FORMAT_DIALOG, TRUE );
}

void Setup::OnVideoPropBtn() 
{
	SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_SHOW_PROP_DIALOG, TRUE );
}

void Setup::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar) 
{

	// Handle Horizontal Sliders
	CString strText;
	CSliderCtrl* pSlider =	(CSliderCtrl*) pScrollBar;
	UINT nTemp;

	if( pScrollBar != NULL )	// The window scroll bars pass in NULL to this
	{
		switch( pScrollBar->GetDlgCtrlID() ) 
		{
			case IDC_SLIDER_1:
			{
				nTemp = pSlider->GetPos();
				strText.Format("%d", nTemp);
				SetDlgItemText(IDC_SLIDER_1_TXT, strText);
				// Send the command to module - currently used to adjust color blob parameters
				SendCommand( WM_ROBOT_SET_CAMERA_COLOR_THRESHOLD, SET_COLOR_THRESHOLD_CR, (DWORD)nTemp );
				break;
			}
			case IDC_SLIDER_2:
			{
				nTemp = pSlider->GetPos();
				strText.Format("%d", nTemp);
				SetDlgItemText(IDC_SLIDER_2_TXT, strText);
				// Send the command to module - currently used to adjust color blob parameters
				SendCommand( WM_ROBOT_SET_CAMERA_COLOR_THRESHOLD, SET_COLOR_THRESHOLD_CB, (DWORD)nTemp );
				break;
			}

		}
	}

	CFormView::OnHScroll(nSBCode, nPos, pScrollBar);
}


LRESULT Setup::OnRemoteGuiCommand(WPARAM Item, LPARAM lParam)
{
	// This function handles commands from remote GUI, and maps them to the Server GUI
	// Item indicates the item to handle.  lParam holds the value/parameter
	// DISABLED FUNCTION!
	IGNORE_UNUSED_PARAM (Item);
	IGNORE_UNUSED_PARAM (lParam);

	CString strText;

/*	if( ROBOT_REMOTE_ENABLE_CAMERA == Item )
	{
		UINT nCamera = LOWORD(lParam);	
		BOOL bEnable = HIWORD(lParam);

		EnableCamera(nCamera, bEnable);
	}
*/
//	else if( ROBOT_REMOTE_START_CAMERA == Item )
//	{
//		StartCamera((BOOL)lParam);
//	}
/*
	else if( ROBOT_REMOTE_ENABLE_MOTION_VIEW == Item )
	{
		CameraEnableShowMotionView((BOOL)lParam);
	}
*/
	// ignore unmapped messages
	return 0;
}

LRESULT Setup::OnRobotDisplaySingleItem(WPARAM Item, LPARAM lParam)
{
	// This function displays various items from the server modules
	// This function is ONLY called by the GUI thread, in response to
	// WM_ROBOT_DISPLAY_SINGLE_ITEM
	// Item indicates the item to display.  lParam holds the value

	CString strText;

	if( ROBOT_RESPONSE_COLOR_BLOB_CAL_RESULT == Item )
	{

		UINT ColorBlobCr = LOWORD(lParam);	
		UINT ColorBlobCb = HIWORD(lParam);
		strText.Format( "Cr = %d, Cb = %d", ColorBlobCr, ColorBlobCb );
		SetDlgItemText( IDC_COLOR_CAL_STATUS, (LPCTSTR)strText );
	}


	// ignore unmapped messages, they are intended for the control GUI page
	return 0;
}


LRESULT Setup::OnGetCameraSettings(WPARAM wParam, LPARAM lParam)
{
	// Request from the Camera module to send current settings
	// Happens at startup, since the module takes longer to startup then this form.
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);

	CString strText;

	// Initialize state of Camera VidCap object detection features
	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_VIDCAP_PROCESSING, (DWORD)(GetDocument()->m_bEnableEnableVideoProcessing) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_TRACKING_FACE, (DWORD)(GetDocument()->m_bEnableTrackingFace) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_TRACKING_COLORS, (DWORD)(GetDocument()->m_bEnableTrackingColors) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_FACE_IDENTIFICATION, (DWORD)(GetDocument()->m_EnableFaceIdentification) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_TRACKING_CONES, (DWORD)(GetDocument()->m_bEnableTrackingCones) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_TRACKING_OBJECTS, (DWORD)(GetDocument()->m_bEnableTrackingObjects) );
	
	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_MATCHING_OBJECTS, (DWORD)(GetDocument()->m_bEnableMatchingObjects) );
	
	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_TRACKING_MOTION, (DWORD)(GetDocument()->m_bEnableTrackingMotion) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_SHOW_MOTION_VIEW, (DWORD)(GetDocument()->m_bEnableShowMotionView) );

	SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, 
		CAMERA_ENABLE_STEREO_VISION, (DWORD)(GetDocument()->m_bEnableStereoVision) );


	// Kludge - not a camera setting, but needs to be delayed until module thread started...
	SendCommand( WM_ROBOT_ENABLE_CLIFF_SENSORS, GetDocument()->m_bEnableCliffSensors, (DWORD)0 );

	return 0;
}


void Setup::OnVideoSelectCamera() 
{
	/*
	if( 0 != cvcamGetProperty(0, CVCAM_VIDEOFORMAT, NULL) )
	{
		ROBOT_DISPLAY( TRUE, "Camera Selection Failed!" )
	}
*/

	/*Pops up a camera(s) selection dialog
	Return value - number of cameras selected (0,1 or 2);
	Argument: an array of selected cameras numbers
	NULL if none selected. Should be released with free() when not needed.
	if NULL passed, not used.
	*/
//	int nCamera = cvcamSelectCamera(NULL);
//	ROBOT_LOG( TRUE,  "CAMERA SELECTED = %d\n", nCamera )
//	CVCAM_API int nCamera = cvcamSelectCamera(int** out);
}

void Setup::OnEnableCamera1() 
{

	BOOL bEnable =  IsDlgButtonChecked(IDC_ENABLE_CAMERA_1);
	GetDocument()->m_bEnableCamera1 = bEnable;	// Persist

	if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
	{
		//g_FaceCaptureName = "Fred"; // DEBUG
		m_CameraRequest.RequestData.EnableFeatures.VideoEnable = (int)bEnable;
		CopyMemory((PVOID)g_pCameraRequestSharedMemory, &m_CameraRequest, (sizeof(CAMERA_REQUEST_T)));
		SetEvent( g_hCameraRequestEvent );  // Send request
		ROBOT_LOG( TRUE, "Enabling Camera1 Video" )
	}
	else
	{
		ROBOT_LOG( TRUE, "ERROR: Can't request video!  Did you have AUTO_LAUNCH_CAMERA_APP enabled?\n" )
	}

	// Start/Stop camera
	/*** NOT USED WITH NEW CAMERA IN A SEPARATE APP
	BOOL bEnable =  IsDlgButtonChecked(IDC_ENABLE_CAMERA_1);
	GetDocument()->m_bEnableCamera1 = bEnable;	// Persist
	EnableCamera( LEFT_CAMERA,  bEnable );
	if( bEnable )
		g_CameraSubSystemStatus = SUBSYSTEM_WAITING;
	else
		g_CameraSubSystemStatus = SUBSYSTEM_DISABLED;
	***/
}

void Setup::OnEnableCamera2() 
{
	// Start/Stop camera
	/*** NOT USED WITH NEW CAMERA IN A SEPARATE APP
	BOOL bEnable =  IsDlgButtonChecked(IDC_ENABLE_CAMERA_2);
	GetDocument()->m_bEnableCamera2 = bEnable;	// Persist
	EnableCamera( RIGHT_CAMERA,  bEnable );
	***/
}

void Setup::OnCamDisplayModeEnable() 
{
#if( CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA )

	if( IsDlgButtonChecked(IDC_CAM_DISPLAY_MODE_ENABLE) )
	{
		SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_DISPLAY_ENABLE, TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_DISPLAY_ENABLE, FALSE );
	}
#endif
}

void Setup::OnCamTrackModeEnable() 
{	
#if( CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA )
	if( IsDlgButtonChecked(IDC_CAM_TRACK_MODE_ENABLE) )
	{
		SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_TRACK_FACE, TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_CAMERA_MODE_CMD, (DWORD)CAMERA_MODE_TRACK_FACE, FALSE );
	}
#endif
}

void Setup::OnCamIrTrackingEnable() 
{
	// KLUDGE - used for PIR (Thermal) Instead!
	if( IsDlgButtonChecked(IDC_CAM_IR_TRACKING_ENABLE) )
	{
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_IR, (DWORD)TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_IR, (DWORD)FALSE );
	}
}

void Setup::OnPersonalSpace() 
{
	if( IsDlgButtonChecked(IDC_PERSONAL_SPACE) )
	{
		ROBOT_LOG( TRUE, "Requesting New Behavior Mode: BEHAVIOR_PERSONAL_SPACE TRUE\n" )
		SendCommand( WM_ROBOT_SET_BEHAVIOR_CMD, BEHAVIOR_PERSONAL_SPACE, (DWORD)TRUE );
	}
	else
	{
		ROBOT_LOG( TRUE, "Requesting New Behavior Mode: BEHAVIOR_PERSONAL_SPACE FALSE\n" )
		SendCommand( WM_ROBOT_SET_BEHAVIOR_CMD, BEHAVIOR_PERSONAL_SPACE, (DWORD)FALSE );
	}
	
}


void Setup::OnDynaServoTestGo() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "TEST: ID = %d, POS = %d\n", m_nDynaServoTestID, m_nDynaServoTestPos )

	SendCommand( WM_ROBOT_SERVO_CMD, m_nDynaServoTestID, (DWORD)m_nDynaServoTestPos ); // Servo number and Position
		
}

void Setup::OnServoGetStatus() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Requesting status for Servo ID %d\n", m_nDynaServoTestID )

	SendCommand( WM_ROBOT_GET_SERVO_STATUS, m_nDynaServoTestID, 0 ); // Servo number
	
}

void Setup::OnDynaServoRegRead() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Requesting register %d for Servo ID %d\n", m_nDynaServoRegNum, m_nDynaServoTestID )

	// NOTE -KLUDGE writes DIRECT TO DYNA Thread - will not work in remote mode!
	WPARAM wParam = MAKEWORD(m_nDynaServoTestID,m_nDynaServoRegNum ); // Servo number and Register Number packed together
	LPARAM lParam =  m_DynaServoNumOfRegs;	// Number of registers to read
	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_READ_SERVO_REGISTER), wParam, lParam );

}

void Setup::OnDynaServoRegWriteByte() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Writing BYTE to register %d in Servo ID %d\n", m_nDynaServoRegNum, m_nDynaServoTestID )

	// NOTE -KLUDGE writes DIRECT TO DYNA Thread - will not work in remote mode!
	WPARAM wParam = MAKEWORD(m_nDynaServoTestID, m_nDynaServoRegNum ); // Servo number and Register Number packed together
	LPARAM lParam =  m_nDynaServoRegWriteValue;

	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_WRITE_SERVO_REGISTER_BYTE), wParam, lParam );
}

void Setup::OnDynaServoRegWriteWord() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Writing WORD to register %d in Servo ID %d\n", m_nDynaServoRegNum, m_nDynaServoTestID )
	
	// NOTE -KLUDGE writes DIRECT TO DYNA Thread - will not work in remote mode!
	WPARAM wParam = MAKEWORD(m_nDynaServoTestID,m_nDynaServoRegNum ); // Servo number and Register Number packed together
	LPARAM lParam =  m_nDynaServoRegWriteValue;

	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_WRITE_SERVO_REGISTER_WORD), wParam, lParam );
}


// TODO - remove this.
void Setup::OnSpeechRecoSendToAi() 
{
#if ( ROBOT_SERVER == 1 )
	//m_RobotSpeak.SendRecoToAI( IsDlgButtonChecked(IDC_SPEECH_RECO_SEND_TO_AI) );
#endif
}



void Setup::OnReadRightArmPosition() 
{
#if ( ROBOT_SERVER == 1 )

	// Get current status for each of the Arm servos
	// Displayed in trace log, and updated to a global memory block.
	// Once the status is read, the result is posted back to this GUI via 
	// WM_ROBOT_SERVO_STATUS_READY which maps to OnServoStatusReady()

	SendCommand( WM_ROBOT_GET_SMART_SERVO_STATUS, ALL_SERVOS, FALSE );

#else
	ROBOT_DISPLAY( TRUE, "ReadRightArmPosition Not supported in Client Mode (local server command only)" )
#endif

}

void Setup::OnSetRightArmPosition() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Setup::OnSetRightArmPosition - Moving Arm under User Control\n" )

	// Set position for each of the Arm servos
	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = m_RightShoulderRotateSet * 10;
	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = m_RightElbowRotateSet * 10;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = m_RightElbowBendSet * 10;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = m_RightWristRotateSet * 10;
	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = m_RightGripSet * 10;
	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update = TRUE;

	// Now, move all the servos for the arm
	SendCommand( WM_ROBOT_SET_ARM_POSITION, RIGHT_ARM, TRUE );	// Set Position and Speed too
}


void Setup::OnSelchangeArmSpeedR() 
{
	ROBOT_LOG( TRUE,  "Setup::OnSelchangeArmSpeedR - Setting all Right Arm Servos Speed\n" )

	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_ARM_SPEED_R);
	UINT nArmSpeed = pComboBox->GetCurSel();
	nArmSpeed = nArmSpeed << 4;	// High Nibble is the Generic speed. See definition of SERVO_SPEED_SLOW
	ROBOT_LOG( TRUE, "Setting Arm Speed to %2X\n", nArmSpeed )

	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update = TRUE;

	// Don't set the speed, will be handled at next "set", 
	// but do send it to the behavior control, so it can use the default speed
	SendCommand( WM_ROBOT_SET_ARM_DEFAULT_SPEED, RIGHT_ARM, nArmSpeed );

}

void Setup::OnReadLeftArmPosition() 
{
#if ( ROBOT_SERVER == 1 )

	// Get current status for each of the Arm servos
	// Displayed in trace log, and updated to a global memory block.
	// Once the status is read, the result is posted back to this GUI via 
	// WM_ROBOT_SERVO_STATUS_READY which maps to OnServoStatusReady()

	SendCommand( WM_ROBOT_GET_SMART_SERVO_STATUS, ALL_SERVOS, FALSE );

#else
	ROBOT_DISPLAY( TRUE, "ReadLeftArmPosition Not supported in Client Mode (local server command only)" )
#endif

}

void Setup::OnSetLeftArmPosition() 
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Setup::OnSetLeftArmPosition - Moving Arm under User Control\n" )

	// Set position for each of the Arm servos
	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees = m_LeftShoulderRotateSet * 10;
	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees = m_LeftElbowRotateSet * 10;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees = m_LeftElbowBendSet * 10;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees = m_LeftWristRotateSet * 10;
	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees = m_LeftGripSet * 10;
	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update = TRUE;

	// Now, move all the servos for the arm
	SendCommand( WM_ROBOT_SET_ARM_POSITION, LEFT_ARM, TRUE );	// Set Position and Speed too
}


void Setup::OnSelchangeArmSpeedL() 
{
	ROBOT_LOG( TRUE,  "Setup::OnSelchangeArmSpeedL - Setting all Left Arm Servos Speed\n" )

	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_ARM_SPEED_L);
	UINT nArmSpeed = pComboBox->GetCurSel();
	nArmSpeed = nArmSpeed << 4;	// High Nibble is the Generic speed. See definition of SERVO_SPEED_SLOW
	ROBOT_LOG( TRUE, "Setting Arm Speed to %2X\n", nArmSpeed )

	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update = TRUE;

	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Speed = nArmSpeed;
	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update = TRUE;

	// Don't set the speed, will be handled at next "set", 
	// but do send it to the behavior control, so it can use the default speed
	SendCommand( WM_ROBOT_SET_ARM_DEFAULT_SPEED, LEFT_ARM, nArmSpeed );

}


void Setup::OnCliffSensors() 
{
	if( IsDlgButtonChecked(IDC_CLIFF_SENSORS) )
	{
		GetDocument()->m_bEnableCliffSensors = TRUE;	// Persist
		SendCommand( WM_ROBOT_ENABLE_CLIFF_SENSORS, (DWORD)TRUE, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableCliffSensors = FALSE;	// Persist
		SendCommand( WM_ROBOT_ENABLE_CLIFF_SENSORS, (DWORD)FALSE, (DWORD)FALSE );
	}
}


void Setup::OnSelchangeRArmPosPreset() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_R_ARM_POS_PRESET);
	int nArmPos = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Setting Arm to Preset Position %d\n", nArmPos )

/*
Arm Motions
1: Home
2: Home 2
3: Extend Arm
4: Open Claw
5. Close Claw
6: Extend Arm All
7: Shake (Ready)
8: Shake (Move)
9: Pickup Object
10: Put Down Object
11: Throw Object Front
12: Throw Object Back
13: Look At Hand
14: Scratch Head
15: Hands Up
16: Scratch Back
17. Wave
18. Grab Coke
19. Lift Object
20. Find on Floor
21. Turn Door Handle

#define ARM_MOVEMENT_NONE					0x00	// No movement pending
#define ARM_MOVEMENT_HOME					0x01	// 
#define ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW	0x02	// 
etc..
*/

	SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)nArmPos );	// Right/Left arm, Movement to execute, 
	
}

void Setup::OnEnableServosR() 
{

	// GUI request to enable/disable Servo Torque for Right Arm 
	ROBOT_LOG( TRUE,  "Setup::OnEnableServosR - User setting Servo Enable State\n" )

	BOOL Enable = IsDlgButtonChecked(IDC_ENABLE_SERVOS_R);

	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Enable = Enable;

	g_BulkServoCmd[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Update = TRUE;

	GetDocument()->m_EnableArmServosRight = Enable;	// Persist
	if(Enable)
		g_RightArmSubSystemStatus = SUBSYSTEM_WAITING; 
	else
		g_RightArmSubSystemStatus = SUBSYSTEM_DISABLED; 

	SendCommand( WM_ROBOT_SET_SERVO_TORQUE_ENABLE, 0, 0 );	// wParam and lParam not used

	
}




void Setup::OnSelchangeLArmPosPreset() 
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_L_ARM_POS_PRESET);
	int nArmPos = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Setting Arm to Preset Position %d\n", nArmPos )

	SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)nArmPos );	// Right/Left arm, Movement to execute, 
}

void Setup::OnEnableServosL() 
{

	// GUI request to enable/disable Servo Torque for Left Arm 
	ROBOT_LOG( TRUE,  "Setup::OnEnableServosL - User setting Servo Enable State\n" )

	BOOL Enable = IsDlgButtonChecked(IDC_ENABLE_SERVOS_L);

	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Enable = Enable;
	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Enable = Enable;

	g_BulkServoCmd[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_LEFT_ARM_WRIST_SERVO_ID].Update = TRUE;
	g_BulkServoCmd[DYNA_LEFT_ARM_CLAW_SERVO_ID].Update = TRUE;

	GetDocument()->m_EnableArmServosLeft = Enable;	// Persist
	if(Enable)
		g_LeftArmSubSystemStatus = SUBSYSTEM_WAITING; 
	else
		g_LeftArmSubSystemStatus = SUBSYSTEM_DISABLED; 

	SendCommand( WM_ROBOT_SET_SERVO_TORQUE_ENABLE, 0, 0 );	// wParam and lParam not used
}


LRESULT Setup::OnServoStatusReady(WPARAM wParam, LPARAM lParam)
{
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);
#if ( ROBOT_SERVER == 1 )

	// Copy current status for each of the Arm servos from the global memory block.
	// This function invoked by WM_ROBOT_SERVO_STATUS_READY

	// CAUSES ASSERT???	UpdateData(TRUE);	// Force data exchange from GUI to data members

	SetDlgItemInt( IDC_HEAD_PAN_GET,
		(g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_HEAD_TILT_GET,
		(g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_HEAD_SIDETILT_GET,
		(g_BulkServoStatus[DYNA_CAMERA_SIDETILT_SERVO_ID].PositionTenthDegrees) / 10);


	SetDlgItemInt( IDC_KINECT_TILT_GET,
		(g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees) / 10);


	SetDlgItemInt( IDC_RIGHT_ELBOW_ROTATE,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_RIGHT_ELBOW_BEND,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_RIGHT_WRIST_ROTATE,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_RIGHT_GRIP,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees) / 10); 

	SetDlgItemInt( IDC_RIGHT_SHOULDER_ROTATE,
		(g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees) / 10);

	UINT KerrStatusRight = g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].StatusFlags;
	CheckDlgButton( IDC_ARM_LIMIT_R, !(KerrStatusRight & 0x20)  );	// KERR_LIMIT1


	SetDlgItemInt( IDC_LEFT_ELBOW_ROTATE,
		(g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_LEFT_ELBOW_BEND,
		(g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_LEFT_WRIST_ROTATE,
		(g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_LEFT_GRIP,
		(g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].PositionTenthDegrees) / 10); 

	SetDlgItemInt( IDC_LEFT_SHOULDER_ROTATE,
		(g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].PositionTenthDegrees) / 10);

	UINT KerrStatusLeft = g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].StatusFlags;
	CheckDlgButton( IDC_ARM_LIMIT_L, !(KerrStatusLeft & 0x20)  );	// KERR_LIMIT1


// Update Load on each servo

	SetDlgItemInt( IDC_HEAD_PAN_LOAD,
		(g_BulkServoStatus[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_HEAD_TILT_LOAD,
		(g_BulkServoStatus[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees) / 10);

	SetDlgItemInt( IDC_HEAD_SIDETILT_LOAD,
		(g_BulkServoStatus[DYNA_CAMERA_SIDETILT_SERVO_ID].PositionTenthDegrees) / 10);



	SetDlgItemInt( IDC_RIGHT_ELBOW_ROTATE_LOAD,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].Load) );

	SetDlgItemInt( IDC_RIGHT_ELBOW_BEND_LOAD,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].Load) );

	SetDlgItemInt( IDC_RIGHT_WRIST_ROTATE_LOAD,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].Load) );

	SetDlgItemInt( IDC_RIGHT_GRIP_LOAD,
		(g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].Load) ); 

	SetDlgItemInt( IDC_RIGHT_SHOULDER_ROTATE_LOAD,
		(g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].Load) );

	SetDlgItemInt( IDC_LEFT_ELBOW_ROTATE_LOAD,
		(g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].Load) );

	SetDlgItemInt( IDC_LEFT_ELBOW_BEND_LOAD,
		(g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].Load) );

	SetDlgItemInt( IDC_LEFT_WRIST_ROTATE_LOAD,
		(g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].Load) );

	SetDlgItemInt( IDC_LEFT_GRIP_LOAD,
		(g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].Load) ); 

	SetDlgItemInt( IDC_LEFT_SHOULDER_ROTATE_LOAD,
		(g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].Load) );





#else
//	ROBOT_DISPLAY( TRUE, "OnServoStatusReady Not supported in Client Mode (local server command only)" )
#endif

	return 0;
}


void Setup::OnCopyRightArmPosition() 
{
	// Copy current status for each of the Arm servos to the Set data members 
	// Displayed in trace log, and updated to a global memory block.
	// Once the status is read, the result is posted back to this GUI via 
	// WM_ROBOT_SERVO_STATUS_READY which maps to OnServoStatusReady()

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	m_RightElbowRotateSet		= m_RightElbowRotate;
	m_RightElbowBendSet			= m_RightElbowBend;
	m_RightWristRotateSet		= m_RightWristRotate;
	m_RightGripSet				= m_RightGrip;
	m_RightShoulderRotateSet	= m_RightShoulderRotate;

	UpdateData(FALSE);	// Force data exchange from data members to GUI

	
}

void Setup::OnCopyLeftArmPosition() 
{
	// Copy current status for each of the Arm servos to the Set data members 
	// Displayed in trace log, and updated to a global memory block.
	// Once the status is read, the result is posted back to this GUI via 
	// WM_ROBOT_SERVO_STATUS_READY which maps to OnServoStatusReady()

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	m_LeftElbowRotateSet		= m_LeftElbowRotate;
	m_LeftElbowBendSet			= m_LeftElbowBend;
	m_LeftWristRotateSet		= m_LeftWristRotate;
	m_LeftGripSet				= m_LeftGrip;
	m_LeftShoulderRotateSet		= m_LeftShoulderRotate;

	UpdateData(FALSE);	// Force data exchange from data members to GUI
	
}

void Setup::OnDynaUsbEnable() 
{
	ROBOT_LOG( TRUE, "OnDynaUsbEnable NOT IMPLEMENTED" )
/*
	if( IsDlgButtonChecked(IDC_DYNA_USB_ENABLE) )
	{
		SendCommand( WM_ROBOT_ENABLE_DYNA_CONTROLLER_CMD, 0, (DWORD)TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_ENABLE_DYNA_CONTROLLER_CMD, 0, (DWORD)FALSE );
	}
*/	
}

void Setup::OnCameraEnableStereoVision() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_STEREO_VISION) )
	{
		GetDocument()->m_bEnableStereoVision = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_STEREO_VISION, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableStereoVision = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_STEREO_VISION, (DWORD)FALSE );
	}
	
}

void Setup::OnCameraEnableTrackingHand() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_TRACKING_HAND) )
	{
		//GetDocument()->m_bEnableTrackingHand = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_HAND, (DWORD)TRUE );
	}
	else
	{
		//GetDocument()->m_bEnableTrackingHand = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_TRACKING_HAND, (DWORD)FALSE );
	}
	
}


void Setup::OnCameraEnableMatchObjects() 
{
	if( IsDlgButtonChecked(IDC_CAMERA_ENABLE_MATCH_OBJECTS) )
	{
		GetDocument()->m_bEnableMatchingObjects = TRUE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_MATCHING_OBJECTS, (DWORD)TRUE );
	}
	else
	{
		GetDocument()->m_bEnableMatchingObjects = FALSE;	// Persist
		SendCommand( WM_ROBOT_CAMERA_ENABLE_FEATURE, CAMERA_ENABLE_MATCHING_OBJECTS, (DWORD)FALSE );
	}
}




void Setup::OnBnClickedLaunchApp()
{

/*	System();
	WinExec("Explorer.exe", SW_MAXIMIZE);
	WinExec("Control.exe", SW_NORMAL);
	WinExec("Control.exe Desktop", SW_NORMAL);

	WinExec("Control.exe Date/Time", SW_NORMAL);
	WinExec("C:\\Program Files\\Internet Expl orer\\IEXPLORE.EXE", SW_MAXIMIZE);
*/

//EXAMPLE: size_t ExecuteProcess(std::wstring FullPathToExe, std::wstring Parameters, size_t SecondsToWait) 
 
	int SecondsToWait = 30;
    //size_t iMyCounter = 0, iReturnVal = 0, iPos = 0; 
    DWORD dwExitCode = 0; 
    //std::wstring sTempStr = L""; 
	// If need to pass parameters, see example at: http://www.goffconcepts.com/techarticles/development/cpp/createprocess.html
	// http://msdn.microsoft.com/en-us/library/ms682425(VS.85).aspx

    /* CreateProcess API initialization */ 
    STARTUPINFO StartupInfo; 
    PROCESS_INFORMATION ProcessInfo; 
    memset(&StartupInfo, 0, sizeof(StartupInfo)); 
    memset(&ProcessInfo, 0, sizeof(ProcessInfo)); 
    StartupInfo.cb = sizeof(StartupInfo); 

    memset(&FWHS, 0, sizeof(FWHS)); 


	if( CreateProcess(
		"C:\\Program Files (x86)\\iTunes\\iTunes.exe",	//  __in_opt     LPCTSTR lpApplicationName,
		"",											//  __inout_opt  LPTSTR lpCommandLine,
		0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpProcessAttributes,
		0,											//  __in_opt     LPSECURITY_ATTRIBUTES lpThreadAttributes,
		false,										//  __in         BOOL bInheritHandles,
		CREATE_DEFAULT_ERROR_MODE,					//  __in         DWORD dwCreationFlags,
		0,											//  __in_opt     LPVOID lpEnvironment,
		0,											//  __in_opt     LPCTSTR lpCurrentDirectory,
		&StartupInfo,								// __in         LPSTARTUPINFO lpStartupInfo,
		&(FWHS.ProcessInfo) )		//  __out        LPPROCESS_INFORMATION lpProcessInformation
	)
	{ 
		RobotSleep(10000, pDomainGUIThread);
		ROBOT_LOG( TRUE,  "WAITING FOR PROCESS LAUNCH...\n" )
         /* Watch the process. */ 
        dwExitCode = WaitForSingleObject(ProcessInfo.hProcess, (SecondsToWait * 1000)); 
		if( WAIT_OBJECT_0 == dwExitCode )
		{
			ROBOT_LOG( TRUE,  "DONE\n" )
		}
		else
		{
			ROBOT_LOG( TRUE,  "TIMED OUT!\n" )
		}
    } 
    else 
    { 
        /* CreateProcess failed */ 
		ROBOT_DISPLAY( TRUE, "ERROR: ITunes launch failed!  Return Code = %d", GetLastError() )
    } 


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
	FWHS.hWndFound  = NULL;

	// Enumerate all top level windows on the desktop, and find the itunes one
	EnumWindows ( EnumWindowCallBack, (LPARAM)&FWHS ) ;

//	SendMessage ( FWHS.hWndFound, Msg, wParam, lParam );



	/* When done, tell the process to exit
		BOOL WINAPI TerminateProcess(
		  __in  HANDLE hProcess,
		  __in  UINT uExitCode
		);
	*/

    /* TODO - Release handles */ 
    //CloseHandle(ProcessInfo.hProcess); 
    //CloseHandle(ProcessInfo.hThread); 
}

void Setup::OnBnClickedItunesPrior()
{
	//SendMessage(FWHS.hWndFound, WM_CANCELMODE, 0, 0 );
	//HWND hfoo;
	::SendMessage( FWHS.hWndFound, WM_KEYDOWN, KEY_ARROW_LEFT, 0 );
 
	//BOOL bResult = ::PostMessage( FWHS.hWndFound, WM_CANCELMODE, 0,0 )	// UINT Msg, WPARAM wParam, LPARAM lParam

}

void Setup::OnBnClickedItunesNext()
{
	::SendMessage( FWHS.hWndFound, WM_KEYDOWN, KEY_ARROW_RIGHT, 0 );
}


void Setup::OnCbnSelchangePcPwrMode()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_PC_PWR_MODE);
	int nMode = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Setting Standby Power Mode to %d\n", nMode )

	SendCommand( WM_ROBOT_POWER_MODE_CMD, (DWORD)0, (DWORD)nMode );	// See LAPTOP_POWER_MODE_DEFAULT 
}


void Setup::OnBnClickedItunesFullscreen()
{
	ROBOT_LOG( TRUE,  "iTunes Fullscreen -- DONT USE THIS - DEBUG ONLY FOR NOW!!\n" )
	/*
	for( int i = 5; i < 100; i++ )
	{
		ROBOT_LOG( TRUE,  "SENDING ----------> %d\n", i );
		::SendMessage( FWHS.hWndFound, WM_KEYDOWN, i, 0 );
		RobotSleep(5000, pDomainGUIThread);
	}
	*/
}

void Setup::OnCbnSelchangeBothArmPosPreset()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_BOTH_ARM_POS_PRESET);
	int nArmPos = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Setting Arm to Preset Position %d\n", nArmPos )

	int nCommand = 0;
	// Map selection to command
	switch( nArmPos )
	{
		case 1: nCommand = ARM_MOVEMENT_HOME1; break;
		case 2: nCommand = ARM_MOVEMENT_HOME2; break;
		case 3: nCommand = ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW; break;
		case 4: nCommand = ARM_MOVEMENT_OPEN_CLAW; break;
		case 5: nCommand = ARM_MOVEMENT_THROW_OBJECT_FRONT; break;
		case 6: nCommand = ARM_MOVEMENT_PUT_IN_BASKET; break;
		case 7: nCommand = ARM_MOVEMENT_PUT_DOWN_OBJECT; break;
		case 8: nCommand = ARM_MOVEMENT_ARM_UP_FULL; break;
		case 9: nCommand = ARM_MOVEMENT_KARATE; break;
		case 10: nCommand = ARM_MOVEMENT_90_DEGREE; break;
		default: nCommand = 0;
	}

	SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)nCommand );	// Right/Left arm, Movement to execute, 
}


/***
void Setup::OnCbnSelchangeBothArmPosPresetx()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_BOTH_ARM_POS_PRESET);
	int nArmPos = pComboBox->GetCurSel();
	ROBOT_LOG( TRUE, "Setting Arm to Preset Position %d\n", nArmPos )

	int nCommand = 0;
	// Map selection to command
	switch( nArmPos )
	{
		case 1: nCommand = ARM_MOVEMENT_HOME1; break;
		case 2: nCommand = ARM_MOVEMENT_HOME2; break;
		case 3: nCommand = ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW; break;
		case 4: nCommand = ARM_MOVEMENT_OPEN_CLAW; break;
		case 5: nCommand = ARM_MOVEMENT_THROW_OBJECT_FRONT; break;
		case 6: nCommand = ARM_MOVEMENT_PUT_IN_BASKET; break;
		case 7: nCommand = ARM_MOVEMENT_PUT_DOWN_OBJECT; break;
		case 8: nCommand = ARM_MOVEMENT_ARM_UP_FULL; break;
		case 9: nCommand = ARM_MOVEMENT_KARATE; break;
		default: nCommand = 0;
	}

	SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)BOTH_ARMS, (DWORD)nCommand );	// Right/Left arm, Movement to execute, 
}
**/
void Setup::OnCbnSelchangeMoveBothArmsPreset()
{
	// TODO: Add your control notification handler code here
	// ROBOT_LOG( TRUE,  "got here\n" )
}


void Setup::OnBnClickedOpenLaserPort()
{
	// Open Laser Scanner serial port
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	GetDocument()->m_strLaserSerialPort = m_strLaserSerialPort;

	OpenLaserPort();
}

void Setup::OnBnClickedEnableLaserPower()
{
	if( IsDlgButtonChecked(IDC_ENABLE_LASER_POWER) )
	{
		PostThreadMessage( g_dwLaserScannerCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_LASER_SCANNER_POWER), 0, POWER_ON );
	}
	else
	{
		PostThreadMessage( g_dwLaserScannerCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_LASER_SCANNER_POWER), 0, POWER_OFF );
	}

}

void Setup::OnBnClickedRequestLaserScans()
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
/*	// DEBUG: 
	if( 0 == m_LaserScanStartAngle )
	{
		g_bLaserScanEnabled = FALSE; // stop frequent servo updates
	}
	else
	{ 
		//g_bLaserScanEnabled = TRUE; // start frequent servo updates
		DWORD NumberOfScans = (DWORD)(m_LaserScanStartAngle);	// KLUDE for testing
		PostThreadMessage( g_dwLaserScannerCommWriteThreadId, (WM_ROBOT_MESSAGE_BASE+HW_LASER_REQUEST_SCAN), 0, NumberOfScans );
	}
*/
	// Send request to do a laser scan
	// wParam contains Start and End Angle in tenth degrees
	// lParam contains step angle in tenth degrees
	DWORD wParam = (DWORD)MAKELONG((SHORT)(m_LaserScanStartAngle*10), (SHORT)(m_LaserScanEndAngle*10));	// LoWord, HiWord in Tenth Degrees
	DWORD lParam = (DWORD)(m_LaserScanStepAngle);	// already in Tenth Degrees
	SendCommand( WM_ROBOT_DO_LASER_SCANS, (DWORD)wParam, (DWORD)lParam );


}

void Setup::OnBnClickedFindObjAtXyz()
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "DEBUG: LOOK AT XYZ REQUEST AT %d, %d, %d\n", m_nObjectX, m_nObjectY, m_nObjectZ )

	long ArmXYZParam1 = MAKELONG((int)(m_nFindObjectZ*10), (int)(m_nFindObjectX*10));	// LoWord, HiWord in TenthInches  (to allow for fractions)
	long ArmXYZParam2 = (int)(m_nFindObjectY*10);									// in TenthInches (to allow for fractions)

	//SendCommand( WM_ROBOT_FIND_OBJECT_AT_XYZ_CMD, (DWORD)ArmXYZParam1, (DWORD)ArmXYZParam2 );
	SendCommand( WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD, (DWORD)ArmXYZParam1, (DWORD)ArmXYZParam2 );
	
}

void Setup::OnBnClickedCheckLocation()
{
	// Do SLAM Location Check (used for testing)
	SendCommand( WM_ROBOT_SLAM_CHECK_CMD, 0, 0 );
}

void Setup::OnBnClickedCopyHeadPosition()
{
	// Copy current status for each of the Head servos to the Set data members 
	// Displayed in trace log, and updated to a global memory block.
	// Once the status is read, the result is posted back to this GUI via 
	// WM_ROBOT_SERVO_STATUS_READY which maps to OnServoStatusReady()

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	m_HeadPanServoSet			= m_HeadPanServo;
	m_HeadTiltServoSet			= m_HeadTiltServo;
	m_HeadSideTiltServoSet		= m_HeadSideTiltServo;

	UpdateData(FALSE);	// Force data exchange from data members to GUI
}

void Setup::OnBnClickedSetHeadPosition()
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Setup::OnSetLeftArmPosition - Moving Arm under User Control\n" )

	// Set position for each of the Head servos
	g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].PositionTenthDegrees = m_HeadPanServoSet * 10;
	g_BulkServoCmd[DYNA_CAMERA_PAN_SERVO_ID].Update = TRUE;

	if( DYNA_CAMERA_TILT_SERVO_ID < 100 ) 
	{
		g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].PositionTenthDegrees = m_HeadTiltServoSet * 10;
		g_BulkServoCmd[DYNA_CAMERA_TILT_SERVO_ID].Update = TRUE;
	}

	if( DYNA_CAMERA_SIDETILT_SERVO_ID < 100 ) 
	{
		g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].PositionTenthDegrees = m_HeadSideTiltServoSet * 10;
		g_BulkServoCmd[DYNA_CAMERA_SIDETILT_SERVO_ID].Update = TRUE;
	}

	// Now, move all the servos for the head
	SendCommand( WM_ROBOT_SET_HEAD_POSITION, 0, FALSE );	// FALSE = Don't Set Speed

}

void Setup::OnBnClickedLaserSearchForObjects()
{
	if( IsDlgButtonChecked(IDC_LASER_SEARCH_FOR_OBJECTS) )
	{
		SendCommand( WM_ROBOT_SET_ACTION_CMD, ACTION_MODE_PICKUP_OBJECTS, TRUE );
	}
	else
	{
		SendCommand( WM_ROBOT_SET_ACTION_CMD, ACTION_MODE_PICKUP_OBJECTS, FALSE );
	}
}

void Setup::OnBnClickedSetLaserPosition() // Actually, now used for Kinect!
{
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	ROBOT_LOG( TRUE,  "Setup: Moving Kinect under User Control\n" )

	// Set position for the Kinect servo
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees = m_KinectTiltServoSet * 10;
	g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update = TRUE;

	// Now, move Kinect servo
	SendCommand( WM_ROBOT_SET_KINECT_POSITION, 0, FALSE );	// FALSE = Don't Set Speed

}

void Setup::OnBnClickedEnableKinect()
{
	// Start/Stop Kinect cameras
	/*** NOT USED ANYMORE?
	BOOL bEnable =  IsDlgButtonChecked(IDC_ENABLE_KINECT);
	GetDocument()->m_bEnableKinect = bEnable;	// Persist
	EnableCamera( KINECT_DEPTH,  bEnable );
	EnableCamera( KINECT_VIDEO,  bEnable );
	if( bEnable )
		g_KinectSubSystemStatus = SUBSYSTEM_WAITING;
	else
		g_KinectSubSystemStatus = SUBSYSTEM_DISABLED;
		*/
}

void Setup::OnEnChangeRightShoulderRotate()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CFormView::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}

void Setup::OnCbnSelchangeCamDisplaySize()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_CAM_DISPLAY_SIZE);
	GetDocument()->m_VideoDisplaySizeSelected = pComboBox->GetCurSel();
	g_Camera[LEFT_CAMERA].DisplaySize = GetFrameSize(GetDocument()->m_VideoDisplaySizeSelected);
	g_Camera[RIGHT_CAMERA].DisplaySize = GetFrameSize(GetDocument()->m_VideoDisplaySizeSelected);
	ROBOT_LOG( TRUE, "Set Video Display size to: %d\n", GetDocument()->m_VideoDisplaySizeSelected )
}

void Setup::OnCbnSelchangeKinectDisplaySize()
{
	UpdateData(TRUE); // Get values from the GUI
	CComboBox* pComboBox = (CComboBox*) GetDlgItem(IDC_KINECT_DISPLAY_SIZE);
	GetDocument()->m_KinectDisplaySizeSelected = pComboBox->GetCurSel();
	g_Camera[KINECT_DEPTH].DisplaySize = GetFrameSize(GetDocument()->m_KinectDisplaySizeSelected);
	g_Camera[KINECT_VIDEO].DisplaySize = GetFrameSize(GetDocument()->m_KinectDisplaySizeSelected);
	ROBOT_LOG( TRUE, "Set Kinect Display size to: %d\n", GetDocument()->m_KinectDisplaySizeSelected )
}



void Setup::OnBnClickedFlipKinect()
{
	// Flip Kindect Depth and video windows if checked

	if( IsDlgButtonChecked(IDC_FLIP_KINECT) )
	{
		GetDocument()->m_bEnableKinectVideoFlip = TRUE;	// Persist
		g_Camera[KINECT_DEPTH].Flip = TRUE;
		g_Camera[KINECT_VIDEO].Flip = TRUE;
	}
	else
	{
		GetDocument()->m_bEnableKinectVideoFlip = FALSE;	// Persist
		g_Camera[KINECT_DEPTH].Flip = FALSE;
		g_Camera[KINECT_VIDEO].Flip = FALSE;
	}

}

void Setup::OnBnClickedKinectPwr()
{
	if( IsDlgButtonChecked(IDC_KINECT_PWR) )
	{
		// Enable Kinect Power
		SendCommand( WM_ROBOT_KINECT_POWER_ENABLE_CMD, 0, (DWORD)TRUE );	
	}
	else
	{
		SendCommand( WM_ROBOT_KINECT_POWER_ENABLE_CMD, 0, (DWORD)FALSE );	
	}
}


///////////////////////////////////////////////////////////////////////////
// SCRIPT GENERATION

void Setup::OnBnClickedEditFile()
{
	// Open or Create file as needed
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	//char* FileName = m_ScriptFileName;
	int LinesWritten = 0;
	CString ScriptFilePath = "c:\\temp\\";
	ScriptFilePath += m_ScriptFileName;
	ScriptFilePath += ".txt";

	// Open the file for writing
	errno_t nError;
	nError = fopen_s( &m_ScriptFileHandle, ScriptFilePath, "a+" ); // Append mode, creates if it does not exist
	if( 0 != nError )
	{
		ROBOT_LOG( TRUE, "Could Not open Script File (%s)!\n", ScriptFilePath )
		return;
	}

	fprintf(m_ScriptFileHandle, "\n// ---------- Adding Scripts ----------\n" );
	ROBOT_LOG( TRUE,  "Script file Opened: %s\n", ScriptFilePath)


}


void Setup::OnBnClickedRunScript()
{
	// Run Script File
	UpdateData(TRUE);	// Force data exchange from GUI to data members

	g_ScriptToRun = "c:\\temp\\";
	g_ScriptToRun += m_ScriptFileName;
	g_ScriptToRun += ".txt";

	ROBOT_LOG( TRUE,  "Sending Script to Behavior Module: %s\n", g_ScriptToRun)

	SendCommand( WM_ROBOT_SET_ACTION_CMD, ACTION_MODE_RUN_SCRIPT, 0 );
}


void Setup::OnBnClickedSaveArmToScriptRight()
{
	if( NULL == m_ScriptFileHandle )
	{
		ROBOT_DISPLAY( TRUE, "ERROR: Script File Not Opened" )
		return;
	}

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	// ":" = Field Start (:Right)
	fprintf(m_ScriptFileHandle, ":RP %d,%d,%d,%d,%d\n", 
		m_RightShoulderRotate, m_RightElbowRotate, m_RightElbowBend, m_RightWristRotate, m_RightGrip );
}


void Setup::OnBnClickedSaveArmToScriptLeft()
{
	if( NULL == m_ScriptFileHandle )
	{
		ROBOT_DISPLAY( TRUE, "ERROR: Script File Not Opened" )
		return;
	}

	UpdateData(TRUE);	// Force data exchange from GUI to data members
	// ":" = Field Start (:Left)
	fprintf(m_ScriptFileHandle, ":LP %d,%d,%d,%d,%d\n", 
		m_LeftShoulderRotate, m_LeftElbowRotate, m_LeftElbowBend, m_LeftWristRotate, m_LeftGrip );

}
///////////////////////////////////////////////////////////////////////////


void Setup::OnBnClickedCloseScriptFile()
{

	// Close script file previously opened
	if( NULL == m_ScriptFileHandle )
	{
		ROBOT_DISPLAY( TRUE, "Script File Not Opened.  Nothing to Close" )
		return;
	}

	fprintf(m_ScriptFileHandle, "// End of File\n" );
	fprintf(m_ScriptFileHandle, "///////////////////////////////\r\n" );

	if( fclose( m_ScriptFileHandle ) )
	{
		ROBOT_DISPLAY( TRUE, "Error Closing Script File (%s)!\n", m_ScriptFileName )
	}
	m_ScriptFileHandle = NULL;
	ROBOT_DISPLAY( TRUE, "Script File (%s) Closed.\n", m_ScriptFileName )

}

void Setup::OnBnClickedAddScriptTextLine()
{
	// using the button does not clear the box
	if( NULL == m_ScriptFileHandle )
	{
		ROBOT_DISPLAY( TRUE, "ERROR: Script File Not Opened" )
		return;
	}
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	// Example ":" = Field Start (:Delay)
	fprintf(m_ScriptFileHandle, "%s\n", m_EditScriptTextLine );
	ROBOT_LOG( TRUE, "Script: Added [%s]", m_EditScriptTextLine )

}

void Setup::OnEnVscrollEditScriptTextLine()
{
	// Pressing return will clear the box with each entry.
	if( NULL == m_ScriptFileHandle )
	{
		ROBOT_DISPLAY( TRUE, "ERROR: Script File Not Opened" )
		return;
	}
	UpdateData(TRUE);	// Force data exchange from GUI to data members
	// Example ":" = Field Start (:Delay)
	fprintf(m_ScriptFileHandle, "%s", m_EditScriptTextLine ); // CR already added by box
	ROBOT_LOG( TRUE, "Script: Added [%s]", m_EditScriptTextLine )

	// Pressing return will clear the box with each entry.
	m_EditScriptTextLine.Empty();
	UpdateData(FALSE);	// Force data exchange data members back to GUI

}
