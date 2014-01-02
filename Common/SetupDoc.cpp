// SetupDoc.cpp : implementation file
//

#include "stdafx.h"
#include "Globals.h"
#include "Robot.h"
#include "SetupDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// SetupDoc

IMPLEMENT_DYNCREATE(SetupDoc, CDocument)

SetupDoc::SetupDoc()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

}

BOOL SetupDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	// New Setup document. Initialize Setup variables to defaults
	m_strPicSerialPort =			"COM12:";
	m_strGpsSerialPort =			"COM13:";
	m_strServoSerialPort =			"COM14:";
	m_strDynaServoSerialPort =		"COM15:";
	m_strDynaRX64SerialPort =		"COM16";
	m_strKerrServoSerialPort =		"COM17:";
	m_strLaserSerialPort =			"COM18:";
	m_strMotorSerialPort =			"COM19:";
	m_strCameraSerialPort =			"COM20:";
	m_strRobotIPAddress =			"192.168.1.200";
	m_bEnableCamera1 =				FALSE;
	m_bEnableCamera2 =				FALSE;
	m_bEnableKinect =				FALSE;
	m_bEnableEnableVideoProcessing= FALSE;
	m_bEnableTrackingFace =			FALSE;
	m_bEnableTrackingColors =		FALSE;
	m_EnableFaceIdentification =	FALSE;
	m_bEnableTrackingCones =		FALSE;
	m_bEnableMatchingObjects =		FALSE;
	m_bEnableTrackingObjects =		FALSE;
	m_bEnableTrackingMotion =		FALSE;
	m_bEnableShowMotionView =		FALSE;
	m_VidCapSizeSelected =			0;
	m_bEnableCliffSensors =			FALSE;
	m_bEnableSpeechRecognition =	FALSE;
	m_EnableArmServosRight =		FALSE;
	m_EnableArmServosLeft =			FALSE;
	m_bEnableStereoVision =			FALSE;
	m_VideoDisplaySizeSelected =	1;
	m_KinectDisplaySizeSelected =	1;
	m_bEnableKinectVideoFlip =		FALSE;
	m_bEnableKinectPower =			FALSE;
	m_bEnableDynaServos =			FALSE;

	return TRUE;
}

SetupDoc::~SetupDoc()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

}


BEGIN_MESSAGE_MAP(SetupDoc, CDocument)
	//{{AFX_MSG_MAP(SetupDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// SetupDoc diagnostics

#ifdef _DEBUG
void SetupDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void SetupDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// SetupDoc serialization

void SetupDoc::Serialize(CArchive& ar)
{
	WORD w;

	if (ar.IsStoring())
	{
		ROBOT_LOG( TRUE,  "Saving Setup Settings to Disk\n" )
		ar << m_strPicSerialPort;
		ar << m_strGpsSerialPort;
		ar << m_strServoSerialPort;
		ar << m_strDynaServoSerialPort;
		ar << m_strDynaRX64SerialPort;
		ar << m_strLaserSerialPort;
		ar << m_strKerrServoSerialPort;
		ar << m_strMotorSerialPort;
		ar << m_strCameraSerialPort;
		ar << m_strRobotIPAddress;		
		w = (WORD)m_bEnableCamera1; ar << w;
		w = (WORD)m_bEnableCamera2; ar << w;
		w = (WORD)m_bEnableKinect; ar << w;
		w = (WORD)m_bEnableEnableVideoProcessing; ar << w;
		w = (WORD)m_bEnableTrackingFace; ar << w;
		w = (WORD)m_bEnableTrackingColors; ar << w;
		w = (WORD)m_EnableFaceIdentification; ar << w;
		w = (WORD)m_bEnableTrackingCones; ar << w;
		w = (WORD)m_bEnableTrackingObjects; ar << w;
		w = (WORD)m_bEnableMatchingObjects; ar << w;
		w = (WORD)m_bEnableTrackingMotion; ar << w;
		w = (WORD)m_bEnableShowMotionView; ar << w;
		w = (WORD)m_VidCapSizeSelected; ar << w;
		w = (WORD)m_bEnableCliffSensors; ar << w;
		w = (WORD)m_bEnableSpeechRecognition; ar <<w;
		w = (WORD)m_EnableArmServosRight; ar <<w;
		w = (WORD)m_EnableArmServosLeft; ar <<w;
		w = (WORD)m_bEnableStereoVision; ar <<w;
		w = (WORD)m_VideoDisplaySizeSelected; ar << w;
		w = (WORD)m_KinectDisplaySizeSelected; ar << w;

		w = (WORD)m_bEnableKinectVideoFlip; ar << w;

		w = (WORD)m_bEnableKinectPower; ar << w;
		w = (WORD)m_bEnableDynaServos; ar << w;
		

	}
	else
	{
		// Reading in from archive
		ar >> m_strPicSerialPort;
		ar >> m_strGpsSerialPort;
		ar >> m_strServoSerialPort;
		ar >> m_strDynaServoSerialPort;
		ar >> m_strDynaRX64SerialPort;
		ar >> m_strLaserSerialPort;
		ar >> m_strKerrServoSerialPort;
		ar >> m_strMotorSerialPort;
		ar >> m_strCameraSerialPort;
		ar >> m_strRobotIPAddress;
		ar >> w; m_bEnableCamera1 = w;
		ar >> w; m_bEnableCamera2 = w;
		ar >> w; m_bEnableKinect = w;
		ar >> w; m_bEnableEnableVideoProcessing = w;
		ar >> w; m_bEnableTrackingFace = w;
		ar >> w; m_bEnableTrackingColors = w;
		ar >> w; m_EnableFaceIdentification = w;
		ar >> w; m_bEnableTrackingCones = w;
		ar >> w; m_bEnableTrackingObjects = w;
		ar >> w; m_bEnableMatchingObjects = w;
		ar >> w; m_bEnableTrackingMotion = w;
		ar >> w; m_bEnableShowMotionView = w;
		ar >> w; m_VidCapSizeSelected = w;
		ar >> w; m_bEnableCliffSensors = w;
		ar >> w; m_bEnableSpeechRecognition = w;
		ar >> w; m_EnableArmServosRight = w;
		ar >> w; m_EnableArmServosLeft = w;
		ar >> w; m_bEnableStereoVision = w;
		ar >> w; m_VideoDisplaySizeSelected = w;
		ar >> w; m_KinectDisplaySizeSelected = w;

		ar >> w; m_bEnableKinectVideoFlip = w;

		// NEW!
		ar >> w; m_bEnableKinectPower = w;
		ar >> w; m_bEnableDynaServos = w;

	}
}

/////////////////////////////////////////////////////////////////////////////
// SetupDoc commands
