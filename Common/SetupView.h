#if !defined(AFX_SETUPVIEW_H__D394D195_2E66_4D03_9A4E_D48587DFAFDA__INCLUDED_)
#define AFX_SETUPVIEW_H__D394D195_2E66_4D03_9A4E_D48587DFAFDA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SetupView.h : header file
//

#include "RobotType.h"
#include <winsvc.h>		// For Start/Stop service

//////////////////////////////////////
// TODO TRIM THIS LIST!
// Windows Header Files:
//#include <windows.h>
//#include <commdlg.h>

// C RunTime Header Files
//#include <stdlib.h>
//#include <malloc.h>
//#include <memory.h>
//#include <tchar.h>

// ATL Header Files
#include <atlbase.h>

// Multi-Language Header File
#include <mlang.h>

// RichEdit 2.0
#include <richedit.h>

//#ifdef _WIN32_WCE
//#include<WinCEStub.h>
//#define ICON_BIG            1
//#endif

// SAPI Header Files
//#include <sphelper.h>
//#include <spddkhlp.h>
#include "Speak.h"

// Local Header Files
//#include "reco.h"
//#include "resource.h"


//////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// Setup form view

#ifndef __AFXEXT_H__
#include <afxext.h>
#endif

class Setup : public CFormView
{
protected:
	Setup();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(Setup)

// Form Data
public:
	//{{AFX_DATA(Setup)
	enum { IDD = IDD_SETUP_FORM };
	CString	m_strGpsSerialPort;
	CString	m_strServoSerialPort;
	CString	m_strMotorSerialPort;
	CString	m_strCameraSerialPort;
	CString	m_strPicSerialPort;
	CString	m_strRobotIPAddress;
	CString	m_strDynaServoSerialPort;
	UINT	m_nDynaServoTestID;
	UINT	m_nDynaServoTestPos;
	UINT	m_nDynaServoRegNum;
	UINT	m_nDynaServoRegWriteValue;
	UINT	m_DynaServoNumOfRegs;
	int		m_RightShoulderRotate;
	int		m_RightWristRotate;
	int		m_RightElbowBend;
	int		m_RightElbowRotate;
	int		m_RightGrip;
	int		m_LeftShoulderRotate;
	int		m_LeftWristRotate;
	int		m_LeftElbowBend;
	int		m_LeftElbowRotate;
	int		m_LeftGrip;
	BOOL	m_CliffSensorsEnabled;
	CString	m_ArmSpeedR;
	CString	m_ArmSpeedL;
	CString	m_strKerrServoSerialPort;
	BOOL	m_EnableArmServosRight;
	BOOL	m_EnableArmServosLeft;
	int		m_RightElbowBendSet;
	int		m_RightElbowRotateSet;
	int		m_RightGripSet;
	int		m_RightShoulderRotateSet;
	int		m_RightWristRotateSet;
	int		m_LeftElbowBendSet;
	int		m_LeftElbowRotateSet;
	int		m_LeftGripSet;
	int		m_LeftShoulderRotateSet;
	int		m_LeftWristRotateSet;
	//}}AFX_DATA

// Attributes
public:
	//BOOL	m_bCameraConnected;
//	CvCapture* m_pCapture;
	UINT	m_nTestSlider1;
	UINT	m_nTestSlider2;

// Operations
public:

// Overrides
	SetupDoc* GetDocument();

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(Setup)
	public:
	virtual void OnInitialUpdate();
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void PostNcDestroy();
	//}}AFX_VIRTUAL

// Implementation
protected:
	virtual ~Setup();

	void Setup::ConnectToHost( BOOL LocalHost );
	void StartWirelessZeroConfig();
	DWORD StopService( SC_HANDLE hSCM, SC_HANDLE hService, BOOL fStopDependencies, DWORD dwTimeout );
	void DisplayError( LPTSTR szAPI, DWORD dwError );
	void StopWirelessZeroConfig();

	void OpenGPSPort();
	void OpenServoPort();
	void OpenSmartServoPort();
	//void OpenKerrServoPort();
	void OpenLaserPort();
	void OpenCameraPort();
	void OpenMotorPort();
	void OpenPicPort();
//	CvCapture* InitVidCap();	// Returns pointer to cap struction if sucessful
//	void ReleaseVidCap(); 
	SIZE GetFrameSize( UINT EnumSize ); // Return currently selected frame size 
	void EnableCamera( UINT nCamera, BOOL bEnable );
//	void CameraEnableShowMotionView( BOOL bEnableView );


/////////////////////////////////////////////////////////////////////////////////////////////////////
protected:

	CAMERA_REQUEST_T m_CameraRequest;


/* DAVES Removed
#if ( ROBOT_SERVER == 1 )	
	CRobotSpeak m_RobotSpeak;	// Robot Speech Recognition Class
#endif
*/

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
	//{{AFX_MSG(Setup)
	afx_msg void OnSaveToDisk();
	afx_msg void OnConnectToHost();
	afx_msg void OnCameraPwrOn();
	afx_msg void OnCameraPwrOff();
	afx_msg void OnConnectToIpCamera();
	afx_msg void OnSelchangeAvoidObjRange();
	afx_msg void OnSelchangePathSpeedAdder();
	afx_msg void OnSelchangeCameraTrackingColorThreshold();
	afx_msg void OnCameraScanDistanceBtn();
	afx_msg void OnCameraManualColorCalBtn();
	afx_msg void OnConnectToLocalHost();
	afx_msg void OnRadarScan();
	afx_msg void OnCompassCalMode();
	afx_msg void OnCompassCalPoint();
	afx_msg void OnResetOdometer();
	afx_msg void OnResetWatchdog();
	afx_msg void OnStartZcSvc();
	afx_msg void OnStopZcSvc();
	afx_msg void OnSelCmdViewBtn();
	afx_msg void OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelMapViewBtn();
	afx_msg void OnSelPathViewBtn();
	afx_msg void OnUpdateSelMapViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelPathViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelSetupViewBtn();
	afx_msg void OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnGpsOpenPort();
	afx_msg void OnOpenServoPort();
	afx_msg void OnOpenMotorPort();
	afx_msg void OnOpenCameraPort();
	afx_msg void OnSelchangeCamPanSpeed();
	afx_msg void OnCameraPosAbsBtn();
	afx_msg void OnOpenPicPort();
	afx_msg void OnCameraEnableTrackingFace();
	afx_msg void OnCameraEnableTrackingColors();
	afx_msg void OnCameraEnableTrackingCones();
	afx_msg void OnCameraEnableTrackingObjects();
	afx_msg void OnSelchangeCamZoomLevelAbs();
	afx_msg void OnCameraEnableTrackingObjectMotion();
	afx_msg void OnCameraEnableShowMotionView();
	afx_msg void OnSelchangeCamFrameSize();
	afx_msg void OnVideoFormatBtn();
	afx_msg void OnVideoPropBtn();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnColorAutoCalBtn();
	afx_msg void OnVideoSelectCamera();
	afx_msg void OnEnableCamera1();
	afx_msg void OnEnableCamera2();
	afx_msg void OnCamDisplayModeEnable();
	afx_msg void OnCamTrackModeEnable();
	afx_msg void OnCamIrTrackingEnable();
	afx_msg void OnPersonalSpace();
	afx_msg void OnOpenDynaServoPort();
	afx_msg void OnDynaServoTestGo();
	afx_msg void OnServoGetStatus();
	afx_msg void OnDynaServoRegRead();
	afx_msg void OnDynaServoRegWriteByte();
	afx_msg void OnDynaServoRegWriteWord();
	afx_msg void OnSpeechRecoSendToAi();
	afx_msg void OnCameraEnableFaceIdentification();
	afx_msg void OnCliffSensors();
	afx_msg void OnOpenKerrServoPort();
	afx_msg void OnReadRightArmPosition();
	afx_msg void OnSetRightArmPosition();
	afx_msg void OnSelchangeArmSpeedR();
	afx_msg void OnSelchangeRArmPosPreset();
	afx_msg void OnEnableServosR();
	afx_msg void OnCopyRightArmPosition();
	afx_msg void OnReadLeftArmPosition();
	afx_msg void OnSetLeftArmPosition();
	afx_msg void OnSelchangeArmSpeedL();
	afx_msg void OnSelchangeLArmPosPreset();
	afx_msg void OnEnableServosL();
	afx_msg void OnCopyLeftArmPosition();
	afx_msg void OnDynaUsbEnable();
	afx_msg void OnCameraEnableStereoVision();
	afx_msg void OnCameraEnableTrackingHand();
	afx_msg void OnEnableVideoProcessing();
	afx_msg void OnCameraEnableMatchObjects();


	//}}AFX_MSG

	afx_msg LRESULT OnRobotDisplaySingleItem(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnRemoteGuiCommand(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnGetCameraSettings(WPARAM wParam, LPARAM lParam);
	//afx_msg LRESULT OnSpeakingCompleteEvent(WPARAM wParam, LPARAM lParam);
	//afx_msg LRESULT OnSpeakText(WPARAM wParam, LPARAM lParam);	
	afx_msg LRESULT OnServoStatusReady(WPARAM wParam, LPARAM lParam);	

	DECLARE_MESSAGE_MAP()


public:
	afx_msg void OnBnClickedLaunchApp();
	afx_msg void OnCbnSelchangeStandbyPowerMode();
	afx_msg void OnBnClickedItunesPrior();
	afx_msg void OnBnClickedItunesNext();
	afx_msg void OnCbnSelchangePcPowerMode();
//	afx_msg void OnCbnSelchangePcPwrMode();
	afx_msg void OnCbnSelchangePcPwrMode();
	afx_msg void OnBnClickedItunesFullscreen();
	afx_msg void OnCbnSelchangeBothArmPosPreset();
	afx_msg void OnCbnSelchangeMoveBothArmsPreset();
	afx_msg void OnBnClickedOpenDynaRx64ServoPort();
	CString m_strDynaRX64SerialPort;
	afx_msg void OnBnClickedOpenLaserPort();
	CString m_strLaserSerialPort;
	afx_msg void OnBnClickedEnableLaserPower();
	afx_msg void OnBnClickedRequestLaserScans();
	UINT m_nLaserScansToRequest;
	afx_msg void OnBnClickedFindObjAtXyz();
	int m_nObjectX;
	int m_nObjectY;
	int m_nObjectZ;
	afx_msg void OnBnClickedCheckLocation();
	afx_msg void OnBnClickedCopyHeadPosition();
	afx_msg void OnBnClickedSetHeadPosition();
	int m_HeadPanServoSet;
	int m_HeadTiltServoSet;
	int m_HeadSideTiltServoSet;
	int m_HeadPanServo;
	int m_HeadTiltServo;
	int m_HeadSideTiltServo;
	afx_msg void OnBnClickedLaserSearchForObjects();
	int m_LaserScanStartAngle;
	int m_LaserScanEndAngle;
	int m_LaserScanStepAngle;
	afx_msg void OnBnClickedSetLaserPosition();
	int m_KinectTiltServoSet;
	int m_KinectTiltServo;
	int m_nFindObjectX;
	int m_nFindObjectY;
	int m_nFindObjectZ;
	afx_msg void OnBnClickedEnableKinect();
	afx_msg void OnEnChangeRightShoulderRotate();
	afx_msg void OnCbnSelchangeCamDisplaySize();
	afx_msg void OnCbnSelchangeKinectDisplaySize();
	CString m_VidDisplaySizeSelected;
	CString m_KinectDisplaySizeSelected;
	afx_msg void OnBnClickedFlipKinect();
	afx_msg void OnBnClickedKinectPwr();
	afx_msg void OnBnClickedEditFile();
	afx_msg void OnBnClickedRunScript();
	afx_msg void OnBnClickedAddScriptTextLine();
	afx_msg void OnBnClickedSaveArmToScriptRight();
	afx_msg void OnBnClickedSaveArmToScriptLeft();
	CString m_ScriptFileName;
	FILE   *m_ScriptFileHandle;
	afx_msg void OnBnClickedCloseScriptFile();
	CString m_EditScriptTextLine;
	afx_msg void OnEnVscrollEditScriptTextLine();
};

#ifndef _DEBUG  // debug version in SetupView.cpp
inline SetupDoc* Setup::GetDocument()
   { return (SetupDoc*)m_pDocument; }
#endif

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SETUPVIEW_H__D394D195_2E66_4D03_9A4E_D48587DFAFDA__INCLUDED_)
