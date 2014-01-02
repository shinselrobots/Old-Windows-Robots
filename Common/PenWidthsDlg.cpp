// PenWidthsDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Robot.h"
#include "PenWidthsDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CPenWidthsDlg dialog


CPenWidthsDlg::CPenWidthsDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CPenWidthsDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CPenWidthsDlg)
	m_nThickWidth = 0;
	m_nThinWidth = 0;
	//}}AFX_DATA_INIT
}


void CPenWidthsDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CPenWidthsDlg)
	DDX_Text(pDX, IDC_THICK_PEN_WIDTH, m_nThickWidth);
	DDV_MinMaxInt(pDX, m_nThickWidth, 1, 20);
	DDX_Text(pDX, IDC_THIN_PEN_WIDTH, m_nThinWidth);
	DDV_MinMaxInt(pDX, m_nThinWidth, 1, 20);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CPenWidthsDlg, CDialog)
	//{{AFX_MSG_MAP(CPenWidthsDlg)
	ON_BN_CLICKED(IDC_DEFAULT_PEN_WIDTHS, OnDefaultPenWidths)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CPenWidthsDlg message handlers

void CPenWidthsDlg::OnDefaultPenWidths() 
{
	m_nThinWidth = 2;
	m_nThickWidth = 5;
	UpdateData(FALSE);  // write to GUI

	
}
