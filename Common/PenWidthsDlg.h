#if !defined(AFX_PENWIDTHSDLG_H__D451B786_B6EE_442C_A4C2_8F3911064A63__INCLUDED_)
#define AFX_PENWIDTHSDLG_H__D451B786_B6EE_442C_A4C2_8F3911064A63__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// PenWidthsDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CPenWidthsDlg dialog

class CPenWidthsDlg : public CDialog
{
// Construction
public:
	CPenWidthsDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CPenWidthsDlg)
	enum { IDD = IDD_PEN_WIDTHS };
	int		m_nThickWidth;
	int		m_nThinWidth;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPenWidthsDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CPenWidthsDlg)
	afx_msg void OnDefaultPenWidths();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PENWIDTHSDLG_H__D451B786_B6EE_442C_A4C2_8F3911064A63__INCLUDED_)
