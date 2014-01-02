#if !defined(AFX_ROBOTCMDVIEW_H__C3CDE9BE_FF13_414D_93E6_6028A38B6AAB__INCLUDED_)
#define AFX_ROBOTCMDVIEW_H__C3CDE9BE_FF13_414D_93E6_6028A38B6AAB__INCLUDED_

#include "RobotType.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



/////////////////////////////////////////////////////////////////////////////
// RobotCmdView form view

//-----------------------------------------------------------------------------
// Defines, constants, and global variables
//-----------------------------------------------------------------------------
#define SLIDER_CENTER 128
#define BACKGROUND_COLOR_RED	RGB( 255, 0, 0 )
#define BACKGROUND_COLOR_YELLOW	RGB( 255, 255, 0 )
#define BACKGROUND_COLOR_GREEN	RGB( 0, 255, 0 )
#define BACKGROUND_COLOR_BLUE	RGB( 0, 127, 255 )

enum BATTERY_STATUS { 
	UNKNOWN = 0,
	URGENT,
	LOW,
	OK,
};


//-----------------------------------------------------------------------------
// Defines, constants, and global variables
//-----------------------------------------------------------------------------

#define COMM_BUFF_SIZE 200


class ArmControlDlg;	// forward declaration, so we don't need to include ArmControlDlg everywhere this header file is used

class CRobotCmdView : public CFormView
{
protected:
	CRobotCmdView();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(CRobotCmdView)

// Form Data
public:
	BOOL m_bTimerExpiredReported;
	CRadarDisplayWnd m_RadarDisplay;
	CLaserDisplayWnd m_LaserDisplay;
	void DisplayTCPIPTripTime( int nTripTime );
	void DisplayPower( BOOL On );

	//{{AFX_DATA(CRobotCmdView)
	enum { IDD = IDD_ROBOTCMD_FORM };
	CListBox	m_StatusListBox;
	BOOL	m_UseJoystick;
	CString	m_BehaviorMode;
	CString	m_MoveDistance;
	CString	m_TextToSpeak;
	CString	m_TurnAmount;
	BOOL	m_SendTextToAI;
	//}}AFX_DATA

// Attributes
public:
	CRobotCmdDoc* GetDocument();

// Operations
public:
	BOOL m_GPSConnectionIndicator;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRobotCmdView)
	public:
	virtual void OnInitialUpdate();
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	virtual ~CRobotCmdView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
// Used for both Client and Server

	HICON				 m_hIcon;
	DWORD				 m_CameraZoomSpeed;
	DWORD			 	 m_CameraPanSpeed;
	int 				 m_nMaxClientKeepAliveCount;
	int					 m_nLineNum;
	int					 m_nSpeedSlider;
	int					 m_nTurnSlider;
///	LPDIRECTINPUT8       m_pDI;//              = NULL;         
///	LPDIRECTINPUTDEVICE8 m_pJoystick; //       = NULL;     
///	DIDEVCAPS            m_diDevCaps;
///	CHtmlCtrl			 m_HtmlCtrl;
	BOOL				 m_bTrackDllLoaded;
	BOOL				 m_bTrackColorSearching;
	int 				 m_nAvoidObjectRange;	// TenthInches
	ArmControlDlg		*m_pArmControlDlg;
	CBrush				*m_RedBrush;	// Red background
	CBrush				*m_YellowBrush;	// Yellow background
	CBrush				*m_GreenBrush;	// Green background
	CBrush				*m_BlueBrush;	// Blue background
	BATTERY_STATUS		 m_BatteryStatus;
	BOOL				 m_LocalUser;	// True = all commands issued as LOCAL user, otherwise as Remote user.  Allow testing Local/Remote behavior over Remote Desktop


//	HANDLE				 m_hSocketThread;
	
	// Client Only (!ROBOT_SERVER)
	CLIENT_SOCKET_STRUCT m_ClientSockStruct;
	//HANDLE				 m_hClientTCPThread;

	// ROBOT_SERVER Only
//	HANDLE				 m_hStatusThread;
//	ROBOT_THREAD_STRUCT  m_rt;
//	STATUS_THREAD_STRUCT m_st;


	// Generated message map functions
	//{{AFX_MSG(CRobotCmdView)
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnCenterButton();
	afx_msg void OnStopButton();
	afx_msg void OnDestroy();
	afx_msg void On12vOn();
	afx_msg void On12vOff();
	afx_msg void OnDisconnectFromHost();
	afx_msg void OnUseJoystick();
	afx_msg void OnCameraWin();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMoveDistanceForward();
	afx_msg void OnTurnDistanceLeft();
	afx_msg void OnTurnDistanceRight();
	afx_msg void OnMoveDistanceReverse();
	afx_msg void OnOpenDefaultPath();
	afx_msg void OnOpenDefaultMap();
	afx_msg void OnSelCmdViewBtn();
	afx_msg void OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelMapViewBtn();
	afx_msg void OnSelPathViewBtn();
	afx_msg void OnExecutePath();
	afx_msg void OnCamDown();
	afx_msg void OnCamDownLeft();
	afx_msg void OnCamDownRight();
	afx_msg void OnCamLeft();
	afx_msg void OnCamRight();
	afx_msg void OnCamStop();
	afx_msg void OnCamUp();
	afx_msg void OnCamUpLeft();
	afx_msg void OnCamUpRight();
	afx_msg void OnCamCenter();
//	afx_msg void OnBrakeButton();
	afx_msg void OnCancelPath();
	afx_msg void OnResumePath();
	afx_msg void OnPausePath();
	afx_msg void OnCbHighGear();
	afx_msg void OnTestBrake();
	afx_msg void OnColorTrackSearch();
	afx_msg void OnColorTrackCalibrate();
	afx_msg void OnEnableCollisionModule();
	afx_msg void OnEnableAvoidanceModule();
	afx_msg void OnChangeCameraManualColorCaldata();
	afx_msg void OnUpdateSelMapViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelPathViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelSetupViewBtn();
	afx_msg void OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnEnableGpsPath();
	afx_msg void OnZoomOut();
	afx_msg void OnZoomIn();
	afx_msg void OnZoomStop();
	afx_msg void OnSelchangeBehaviorMode();
	afx_msg void OnSelchangeActionMode();
	afx_msg void OnEnableLedEyes();
	afx_msg void OnEnableCameraLights();
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnLedEyesBlink();
	afx_msg void OnOpenDownstairsMap();
	afx_msg void OnOpenUpstairsMap();
//	afx_msg void OnOpenBlankMap();
	afx_msg void OnEnableObjectNavUpdate();
	afx_msg void OnSelchangePlayMusic();
	afx_msg void OnEnableAuxLights();
	afx_msg void OnStopPanicStop();
	afx_msg void OnVscrollTextToSpeak();
	afx_msg void OnQ_Key();
	afx_msg void OnKEY_A();
	afx_msg void OnKey_D();
	afx_msg void OnKey_E();
	afx_msg void OnKey_S();
	afx_msg void OnKey_W();
	afx_msg void OnKey_X();
	afx_msg void OnKey_Plus();
	afx_msg void OnKey_Minus();
	afx_msg void OnKey_Zero();
	afx_msg void OnSelchangeHeadTilt();
	afx_msg void OnKey_C();
	afx_msg void OnArmMovementDlg();
	afx_msg void OnTakeSnapshot();
	afx_msg void OnRecordVideo();
	afx_msg void OnRecordVideoStop();
	//}}AFX_MSG
	afx_msg LRESULT OnRobotDisplayMessage(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnRobotDisplaySingleItem(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnRobotDisplayTcpTime(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnRobotDisplayBulkItem(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnRobotDisplayOpenDataFile(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnRemoteGuiCommand(WPARAM Item, LPARAM lParam);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnCbnSelchangeLaserZoom();
	afx_msg void OnBnClickedLaserScanEnable();
	afx_msg void OnStnClickedStatColorCalResult2();
	afx_msg void OnBnClickedKinectUp();
	afx_msg void OnBnClickedKinectDown();
	afx_msg void OnStnClickedStatIrAdVertFrontLeft();
//	afx_msg void OnStnClickedPicStatus();
	afx_msg void OnStnClickedPicStatus();
	afx_msg void OnBnClickedLocalUser();
	afx_msg void OnBnClickedKinectPwrEnable();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
//	afx_msg void OnBnClickedBrakeButton();
	afx_msg void OnBnClickedCaptureFace();
	CString FaceCaptureName;
	afx_msg void OnStnClickedIrBumperArmLFingerR();
//	afx_msg void OnStnClickedBatteryWarningText();
};
#ifndef _DEBUG  // debug version in RobotView.cpp
inline CRobotCmdDoc* CRobotCmdView::GetDocument()
   { return (CRobotCmdDoc*)m_pDocument; }
#endif


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ROBOTCMDVIEW_H__C3CDE9BE_FF13_414D_93E6_6028A38B6AAB__INCLUDED_)
