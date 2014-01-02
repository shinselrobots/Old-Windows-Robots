// PathDoc.cpp : CPathDoc class
// Data store for Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Globals.h"
#include "Robot.h"
#include "PathDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



/////////////////////////////////////////////////////////////////////////////
// CPathDoc

IMPLEMENT_DYNCREATE(CPathDoc, CDocument)

CPathDoc::CPathDoc()
{
	m_CustomFileOpen = FALSE;	// By default, assume the default path file is used

}

BOOL CPathDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	g_pSegmentList = &m_SegmentList;		// Share the current PATH with other modules that need access to it!
	g_pWaypointList= &m_WaypointList;
	m_CustomFileOpen = TRUE;


	// Waypoint Map Structures
/*	CSegmentStruct* pSegmentStruct = new CSegmentStruct();
	pSegmentStruct->m_SegmentName = "SomeName";
	pSegmentStruct->m_SegmentBehavior = "Behave";
	pSegmentStruct->m_SegmentParam = 1234;
	m_SegmentList.AddTail(pSegmentStruct);
*/
	return TRUE;
}
/////////////////////////////////////////////////////////////////////////////
// CPathDoc commands

BOOL CPathDoc::OnOpenDocument(LPCTSTR lpszPathName) 
{
	if (!CDocument::OnOpenDocument(lpszPathName))
		return FALSE;
	
	g_pSegmentList = &m_SegmentList;		// Share the current PATH with other modules that need access to it!
	g_pWaypointList= &m_WaypointList;
	CString strDefaultFile = DEFAULT_PATH_FILE;
	if(  0 != strDefaultFile.CompareNoCase(lpszPathName) )
	{
		// Not the default path
		m_CustomFileOpen = TRUE;
	}

	return TRUE;
}


CPathDoc::~CPathDoc()
{
}

BEGIN_MESSAGE_MAP(CPathDoc, CDocument)
	//{{AFX_MSG_MAP(CPathDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


void CPathDoc::DeleteContents()
{
	// The document is being closed. 
	
	// Invalidate the global pointers to the data
	g_pSegmentList = NULL;
	g_pWaypointList= NULL;


	// Now delete the data
	POSITION pos;

	// SegmentList
	pos = m_SegmentList.GetHeadPosition();
	while (pos != NULL)
	{
		delete m_SegmentList.GetNext(pos);
	}
	m_SegmentList.RemoveAll();


	// WaypointList
	pos = m_WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		delete m_WaypointList.GetNext(pos);
	}
	m_WaypointList.RemoveAll();
}


/////////////////////////////////////////////////////////////////////////////
// CPathDoc serialization

void CPathDoc::Serialize(CArchive& ar)
{
	POSITION pos;
	WORD nCount;
	WORD w;

	// General items to store
	if (ar.IsStoring())
	{
		ar << gGPSOriginLat;
		ar << gGPSOriginLong;
	}
	else
	{
		ar >> gGPSOriginLat;
		ar >> gGPSOriginLong;
	}

	// SegmentStruct Read/Write
	if (ar.IsStoring())
	{
		nCount = (WORD)m_SegmentList.GetCount();
		ar << nCount;
		pos = m_SegmentList.GetHeadPosition();
		while (pos != NULL)
		{
			CSegmentStruct* pSegmentStruct = m_SegmentList.GetNext(pos);

			ar << pSegmentStruct->m_SegmentName;
			w = (WORD)pSegmentStruct->m_SegmentFromWaypointID; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentToWaypointID; ar << w;
			ar << pSegmentStruct->m_SegmentBehavior;
			ar << pSegmentStruct->m_SegmentSpeed;
			w = (WORD)pSegmentStruct->m_SegmentDirection; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentDistanceFeet; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentDistanceInches; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentFollowDistance; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentAvoidDistance; ar << w;
			w = (WORD)pSegmentStruct->m_SegmentFollowLeftRight; ar << w;

			w = (WORD)pSegmentStruct->m_CompassCorrection; ar << w;

			nCount--;
		}
		ASSERT(nCount == 0);
	}
	else
	{
		ar >> nCount;
		while (nCount-- > 0)
		{
			CSegmentStruct* pSegmentStruct = new CSegmentStruct;

			ar >> pSegmentStruct->m_SegmentName;
			ar >> w; pSegmentStruct->m_SegmentFromWaypointID = w;
			ar >> w; pSegmentStruct->m_SegmentToWaypointID = w;
			ar >> pSegmentStruct->m_SegmentBehavior;
			ar >> pSegmentStruct->m_SegmentSpeed;
			ar >> w; pSegmentStruct->m_SegmentDirection = w;
			ar >> w; pSegmentStruct->m_SegmentDistanceFeet = w;
			ar >> w; pSegmentStruct->m_SegmentDistanceInches = w;
			ar >> w; pSegmentStruct->m_SegmentFollowDistance = w;
			ar >> w; pSegmentStruct->m_SegmentAvoidDistance = w;
			ar >> w; pSegmentStruct->m_SegmentFollowLeftRight = w;

			// Change following 2 lines to read old path files
			//pSegmentStruct->m_CompassCorrection = 0;
			ar >> w; pSegmentStruct->m_CompassCorrection = w;

			m_SegmentList.AddTail(pSegmentStruct);
		}
	}

	// WaypointStruct Read/Write
	if (ar.IsStoring())
	{
		nCount = (WORD)m_WaypointList.GetCount();
		ar << nCount;
		pos = m_WaypointList.GetHeadPosition();
		while (pos != NULL)
		{
			CWaypointStruct* pWaypointStruct = m_WaypointList.GetNext(pos);

			ar << pWaypointStruct->m_WaypointName;
			w = (WORD)pWaypointStruct->m_WaypointID; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLocationFeetX; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLocationInchesX; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLocationFeetY; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLocationInchesY; ar << w;

			ar << pWaypointStruct->m_WaypointLandmarkType1;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkDirection1; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkRange1; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkHeight1; ar << w;

			ar << pWaypointStruct->m_WaypointLandmarkType2;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkDirection2; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkRange2; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkHeight2; ar << w;

			ar << pWaypointStruct->m_WaypointLandmarkType3;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkDirection3; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkRange3; ar << w;
			w = (WORD)pWaypointStruct->m_WaypointLandmarkHeight3; ar << w;

			nCount--;
		}
		ASSERT(nCount == 0);
	}
	else
	{
		ar >> nCount;
		while (nCount-- > 0)
		{
			CWaypointStruct* pWaypointStruct = new CWaypointStruct;

			ar >> pWaypointStruct->m_WaypointName;
			ar >> w; pWaypointStruct->m_WaypointID = w;
			ar >> w; pWaypointStruct->m_WaypointLocationFeetX = w;
			ar >> w; pWaypointStruct->m_WaypointLocationInchesX = w;
			ar >> w; pWaypointStruct->m_WaypointLocationFeetY = w;
			ar >> w; pWaypointStruct->m_WaypointLocationInchesY = w;

			ar >> pWaypointStruct->m_WaypointLandmarkType1;
			ar >> w; pWaypointStruct->m_WaypointLandmarkDirection1 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkRange1 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkHeight1 = w;

			ar >> pWaypointStruct->m_WaypointLandmarkType2;
			ar >> w; pWaypointStruct->m_WaypointLandmarkDirection2 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkRange2 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkHeight2 = w;

			ar >> pWaypointStruct->m_WaypointLandmarkType3;
			ar >> w; pWaypointStruct->m_WaypointLandmarkDirection3 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkRange3 = w;
			ar >> w; pWaypointStruct->m_WaypointLandmarkHeight3 = w;

			m_WaypointList.AddTail(pWaypointStruct);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////
// CPathDoc diagnostics

#ifdef _DEBUG
void CPathDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CPathDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG



BOOL CPathDoc::SaveModified() 
{
	if(	m_CustomFileOpen )
	{
		return CDocument::SaveModified();	// Invoke the file save dialog
	}
	else
	{
		if (!DoFileSave())
		{
			ROBOT_LOG( TRUE, "ERROR saving file %s!\n", DEFAULT_PATH_FILE);
		}
	}
	return 1;	// continue shutting down
}


BOOL CPathDoc::OnSaveDocument(LPCTSTR lpszPathName) 
{
	// TODO: Add your specialized code here and/or call the base class
	
	return CDocument::OnSaveDocument(lpszPathName);
}
