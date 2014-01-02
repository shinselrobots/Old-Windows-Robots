// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "Robot.h"

#include "MainFrm.h"
#include "Globals.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CMainFrame

IMPLEMENT_DYNAMIC(CMainFrame, CMDIFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CMDIFrameWnd)
	//{{AFX_MSG_MAP(CMainFrame)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	ON_WM_CREATE()
	//}}AFX_MSG_MAP
	ON_UPDATE_COMMAND_UI(ID_INDICATOR_POS, OnUpdatePos)	// X,Y for Map position
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // status line indicator
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
	ID_INDICATOR_POS,	// Added by Dave - current X, Y position on map
};

/////////////////////////////////////////////////////////////////////////////
// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{
	// NOTE!
	// All Global allocaiton and initialization done here, as this is the first thing to run
	// and ~CMainFrame() is the last to exit
	// //TAL_SCOPED_TASK();	
	ROBOT_LOG( TRUE, "============= ORDER  ==========" )
	//TAL_DescribeCategoryMask(DETAILED_INFO, "DETAILED_INFO", "Details about application execution.");
	//TAL_ParamStr(TAL_LOG_LEVEL_1, DETAILED_INFO, "ExeName", argv[0]);
	
}

CMainFrame::~CMainFrame()
{
	// NOTE!
	// All Global deallocaiton done here, as this is the last to exit

	// //TAL_SCOPED_TASK();	
	ROBOT_LOG( TRUE, "============= ORDER  ==========" )



}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	// //TAL_SCOPED_TASK();	
	ROBOT_LOG( TRUE, "============= ORDER  ==========" )
	static char* mytext = "Foobar - custom status bar text";

	if (CMDIFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n" );
		return -1;      // fail to create
	}

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators, sizeof(indicators)/sizeof(UINT))	

	)
	{
		TRACE0("Failed to create status bar\n" );
		return -1;      // fail to create
	}
	else
	{
		// Hide the Status Bar by default, to provide more room for controls?
		//	m_wndStatusBar.ShowWindow(SW_HIDE);
	}

//	m_wndStatusBar.SetPaneInfo(1,ID_INDICATOR_POS,SBPS_NORMAL,200);
//	m_wndStatusBar.SetWindowText(mytext);


	// TODO: Delete these three lines if you don't want the toolbar to
	//  be dockable
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndToolBar);


	g_RobotMainFrameHWND = GetSafeHwnd();
    if( INVALID_HANDLE_VALUE == g_RobotMainFrameHWND )
    {
		ROBOT_LOG( TRUE, "Error getting Main Frame Dialog Handle!\n")
		ROBOT_ASSERT(0);
    }

//	m_PictureWindow.SubclassWindow(g_RobotMainFrameHWND);
//	m_PictureWindow.nMessageHandler = CPictureWindow::BackGroundPaint;
//	m_PictureWindow.Load("background.bmp");


	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	// THIS IS THE PROPER PLACE TO RESIZE THE FRAME!
	// USE THIS!
	cs.cx =  ROBOT_DISPLY_SIZE_X;
	cs.cy = ROBOT_DISPLY_SIZE_Y;
	// Position on desktop!
	cs.x =  0;
	cs.y = 0;


	if( !CMDIFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return TRUE;
}

void CMainFrame::OnUpdatePos(CCmdUI *pCmdUI)
{
	// Added by Dave for Map X,Y position indicator
    pCmdUI->Enable(); 
    CString strPos;
    strPos.Format( "Not In Map View"); 
    pCmdUI->SetText( strPos ); 
}


/////////////////////////////////////////////////////////////////////////////
// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CMDIFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CMDIFrameWnd::Dump(dc);
}

#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CMainFrame message handlers

