#if !defined(AFX_ROTATEMAPDLG_H__B11356DB_6748_49FE_BA6E_B4859AC5CBD5__INCLUDED_)
#define AFX_ROTATEMAPDLG_H__B11356DB_6748_49FE_BA6E_B4859AC5CBD5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// RotateMapDlg.h : header file
//
#include "RobotType.h"

/////////////////////////////////////////////////////////////////////////////
// CRotateMapDlg dialog

class CRotateMapDlg : public CDialog
{
// Construction
public:
	CRotateMapDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CRotateMapDlg)
	enum { IDD = IDD_ROTATE_MAP };
	int		m_MapRotationAmount;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRotateMapDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CRotateMapDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	int m_MapScaleAmount;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ROTATEMAPDLG_H__B11356DB_6748_49FE_BA6E_B4859AC5CBD5__INCLUDED_)
