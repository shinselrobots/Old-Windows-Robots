// RobotCmdDoc.cpp : implementation file
//

#include "stdafx.h"
#include "Globals.h"
#include "Robot.h"
#include "RobotCmdDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdDoc

IMPLEMENT_DYNCREATE(CRobotCmdDoc, CDocument)

CRobotCmdDoc::CRobotCmdDoc()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )
}

BOOL CRobotCmdDoc::OnNewDocument()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )

	if (!CDocument::OnNewDocument())
		return FALSE;
	return TRUE;
}

CRobotCmdDoc::~CRobotCmdDoc()
{
	ROBOT_LOG( TRUE,  "============= ORDER  ==========" )
}


BEGIN_MESSAGE_MAP(CRobotCmdDoc, CDocument)
	//{{AFX_MSG_MAP(CRobotCmdDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdDoc diagnostics

#ifdef _DEBUG
void CRobotCmdDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CRobotCmdDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdDoc serialization

void CRobotCmdDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}

/////////////////////////////////////////////////////////////////////////////
// CRobotCmdDoc commands
