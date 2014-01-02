// Robot.h : main header file for the ROBOT application
//

#if !defined(AFX_ROBOT_H__CA1CBB4E_0128_4A91_AB27_D9F95D4C6E1C__INCLUDED_)
#define AFX_ROBOT_H__CA1CBB4E_0128_4A91_AB27_D9F95D4C6E1C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"       // main symbols
#include "RobotType.h"

// Define Screen size!
// Robot Computer (Alienware) display is 1366 x 768 (leave room for the Windows Taskbar, or set it to autohide?)

// Normal size:
#define ROBOT_DISPLY_SIZE_X	948
#define ROBOT_DISPLY_SIZE_Y	765

// For Kinect SDK with DirectX drawing:
//#define ROBOT_DISPLY_SIZE_X	1220
//#define ROBOT_DISPLY_SIZE_Y	765


/////////////////////////////////////////////////////////////////////////////
// CRobotApp:
// See Robot.cpp for the implementation of this class
//

class CRobotApp : public CWinApp
{
public:
	CRobotApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRobotApp)
	public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();
	//}}AFX_VIRTUAL

// Implementation
	//{{AFX_MSG(CRobotApp)
	afx_msg void OnAppAbout();
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ROBOT_H__CA1CBB4E_0128_4A91_AB27_D9F95D4C6E1C__INCLUDED_)
