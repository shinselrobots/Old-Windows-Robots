#if !defined(AFX_SETUPDOC_H__25571A97_A26A_4FA9_8788_A3B5EF71BC67__INCLUDED_)
#define AFX_SETUPDOC_H__25571A97_A26A_4FA9_8788_A3B5EF71BC67__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SetupDoc.h : header file
//
#include "RobotConfig.h"

/////////////////////////////////////////////////////////////////////////////
// SetupDoc document

class SetupDoc : public CDocument
{
protected:
	SetupDoc();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(SetupDoc)

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(SetupDoc)
	public:
	virtual void Serialize(CArchive& ar);   // overridden for document i/o
	protected:
	virtual BOOL OnNewDocument();
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~SetupDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
public:
	CString m_strPicSerialPort;
	CString m_strGpsSerialPort;
	CString m_strServoSerialPort;
	CString m_strDynaServoSerialPort;
	CString m_strDynaRX64SerialPort;
	CString m_strLaserSerialPort;
	CString m_strKerrServoSerialPort;
	CString m_strMotorSerialPort;
	CString m_strCameraSerialPort;
	CString m_strRobotIPAddress;
	BOOL	m_bEnableCamera1;
	BOOL	m_bEnableCamera2;
	BOOL	m_bEnableKinect;
	BOOL	m_bEnableEnableVideoProcessing;
	BOOL	m_bEnableTrackingFace;
	BOOL	m_bEnableTrackingColors;
	BOOL	m_EnableFaceIdentification;
	BOOL	m_bEnableTrackingCones;
	BOOL	m_bEnableTrackingObjects;
	BOOL	m_bEnableMatchingObjects;
	BOOL	m_bEnableTrackingMotion;
	BOOL	m_bEnableShowMotionView;
	BOOL	m_bEnableCliffSensors;
	BOOL	m_bEnableSpeechRecognition;
	UINT	m_VidCapSizeSelected;
	BOOL	m_EnableArmServosRight;
	BOOL	m_EnableArmServosLeft;
	BOOL	m_bEnableStereoVision;
	UINT	m_VideoDisplaySizeSelected;
	UINT	m_KinectDisplaySizeSelected;
	BOOL	m_bEnableKinectVideoFlip;
	BOOL	m_bEnableKinectPower;
	BOOL	m_bEnableDynaServos;

	//{{AFX_MSG(SetupDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SETUPDOC_H__25571A97_A26A_4FA9_8788_A3B5EF71BC67__INCLUDED_)
