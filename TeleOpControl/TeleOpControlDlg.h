
// TeleOpControlDlg.h : header file
//

#pragma once


// CTeleOpControlDlg dialog
class CTeleOpControlDlg : public CDHtmlDialog
{
// Construction
public:
	CTeleOpControlDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_TELEOPCONTROL_DIALOG, IDH = IDR_HTML_TELEOPCONTROL_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

	HRESULT OnButtonOK(IHTMLElement *pElement);
	HRESULT OnButtonCancel(IHTMLElement *pElement);

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	DECLARE_DHTML_EVENT_MAP()
public:
	afx_msg void OnBnClickedCameraWin();
	afx_msg void OnBnClickedCamStop();
};
