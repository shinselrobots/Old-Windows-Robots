// RotateMapDlg.cpp : implementation file
//

#include "stdafx.h"
#include "robot.h"
#include "RotateMapDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CRotateMapDlg dialog


CRotateMapDlg::CRotateMapDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRotateMapDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CRotateMapDlg)
	m_MapRotationAmount = 0;
	//}}AFX_DATA_INIT
	m_MapScaleAmount = 0;
}


void CRotateMapDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRotateMapDlg)
	DDX_Text(pDX, IDC_MAP_ROTATION_AMOUNT, m_MapRotationAmount);
	DDV_MinMaxInt(pDX, m_MapRotationAmount, -359, 359);
	//}}AFX_DATA_MAP
	DDX_Text(pDX, IDC_MAP_SCALE_AMOUNT, m_MapScaleAmount);
	DDV_MinMaxInt(pDX, m_MapScaleAmount, 0, 10000);
}


BEGIN_MESSAGE_MAP(CRotateMapDlg, CDialog)
	//{{AFX_MSG_MAP(CRotateMapDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRotateMapDlg message handlers
