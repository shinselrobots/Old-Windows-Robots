// PathView.cpp : User Interface of the CPathDoc class
// Data store for Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Robot.h"
#include "Globals.h"
#include "PathDoc.h"
#include "PathView.h"
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CPathView

IMPLEMENT_DYNCREATE(CPathView, CFormView)

CPathView::CPathView()
	: CFormView(CPathView::IDD)
{
	//{{AFX_DATA_INIT(CPathView)
	m_SegmentName = _T("S0");
	m_SegmentFromWaypointID = 0;
	m_SegmentToWaypointID = 1;
	m_SegmentDirection = 0;
	m_SegmentDistanceFeet = 0;
	m_SegmentDistanceInches = 0;
	m_SegmentBehavior = _T("");
	m_SegmentFollowDistance = 0;
	m_SegmentSpeed = _T("");
	m_SegmentFollowLeftRight = -1;
	m_WaypointName = _T("");
	m_WaypointID = 0;
	m_WaypointLocationFeetX = 0;
	m_WaypointLocationInchesX = 0;
	m_WaypointLocationFeetY = 0;
	m_WaypointLocationInchesY = 0;
	m_WaypointLandmarkType1 = _T("");
	m_WaypointLandmarkType2 = _T("");
	m_WaypointLandmarkType3 = _T("");
	m_WaypointLandmarkDirection1 = 0;
	m_WaypointLandmarkDirection2 = 0;
	m_WaypointLandmarkDirection3 = 0;
	m_WaypointLandmarkHeight1 = 0;
	m_WaypointLandmarkHeight2 = 0;
	m_WaypointLandmarkHeight3 = 0;
	m_WaypointLandmarkRange1 = 0;
	m_WaypointLandmarkRange2 = 0;
	m_WaypointLandmarkRange3 = 0;
	m_GPSAnchorPointX = 0;
	m_GPSAnchorPointY = 0;
	m_SegmentAvoidDistance = 0;
	m_CompassCorrection = 0;
	//}}AFX_DATA_INIT
}

CPathView::~CPathView()
{
	g_RobotPathViewHWND = NULL;
}

// Overrides
void CPathView::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();
	POSITION pos;
	g_RobotPathViewHWND = GetSafeHwnd();	// Allow other windows to send me messages
	m_GPSConnectionIndicator = FALSE;

	// Initialize default selections for each drop down list
	SetDlgItemText( IDC_SEGMENT_BEHAVIOR, "Compass+GPS"); 
	SetDlgItemText( IDC_SEGMENT_SPEED, "Med"); 
	SetDlgItemText( IDC_WAYPOINT_LANDMARK_TYPE1, "None"); 
	SetDlgItemText( IDC_WAYPOINT_LANDMARK_TYPE2, "None"); 
	SetDlgItemText( IDC_WAYPOINT_LANDMARK_TYPE3, "None"); 
	SetDlgItemText( IDC_WAYPOINT_NAME, "Start"); 
	SetDlgItemText( IDC_WAYPOINT_X_FEET, "100"); 
	SetDlgItemText( IDC_WAYPOINT_Y_FEET, "100"); 
	SetDlgItemText( IDC_SEGMENT_AVOID_DISTANCE, "60");	// Inches - overridden by g_GlobalMaxAvoidObjectDetectionFeet!


	// Display current origin
	CString strText;
	if( (-1 != gGPSOriginLat) || (-1 != gGPSOriginLong)  )	// if not set, use default string "not set"
	{
		strText.Format("%lf", gGPSOriginLat);
		SetDlgItemText( IDC_PATH_ORIGIN_LAT, strText); 
		strText.Format("%lf", gGPSOriginLong);
		SetDlgItemText( IDC_PATH_ORIGIN_LONG, strText); 
	}

	// Copy all of the strings from the document's SegmentList
	// to the Segment Listbox.
	m_SegmentListBox.ResetContent();
	CSegmentStructList& SegmentList = GetDocument()->m_SegmentList;
	pos = SegmentList.GetHeadPosition();
	while (pos != NULL)
	{
		CSegmentStruct* pSegmentStruct = SegmentList.GetNext(pos);
		AddSegmentStructToListBox(pSegmentStruct);
	}

	// Copy all of the strings from the document's WaypointList
	// to the Waypoint Listbox.
	m_WaypointListBox.ResetContent();
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;
	pos = WaypointList.GetHeadPosition();
	if( NULL == pos )
	{
			OnWaypointAdd();	// Empty list, add the Start WP
	}
	while (pos != NULL)
	{
		CWaypointStruct* pWaypointStruct = WaypointList.GetNext(pos);
		AddWaypointStructToListBox(pWaypointStruct);

		if( 0 == pWaypointStruct->m_WaypointID )
		{
			// Starting Waypoint.  Set robot map position to start here!
			g_pFullSensorStatus->CurrentLocation.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
			g_pFullSensorStatus->CurrentLocation.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
		}
	}

	// Make the window the size of the main dialog.
    GetParentFrame()->RecalcLayout();
    ResizeParentToFit( FALSE );
}


void CPathView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CPathView)
	DDX_Control(pDX, IDC_SEGMENT_LISTBOX, m_SegmentListBox);
	DDX_Text(pDX, IDC_SEGMENT_NAME, m_SegmentName);
	DDX_Text(pDX, IDC_SEGMENT_FROM_WAYPOINT_ID, m_SegmentFromWaypointID);
	DDX_Text(pDX, IDC_SEGMENT_TO_WAYPOINT_ID, m_SegmentToWaypointID);
	DDX_Text(pDX, IDC_SEGMENT_DIRECTION, m_SegmentDirection);
	DDX_Text(pDX, IDC_SEGMENT_DISTANCE_FEET, m_SegmentDistanceFeet);
	DDX_Text(pDX, IDC_SEGMENT_DISTANCE_INCHES, m_SegmentDistanceInches);
	DDX_CBString(pDX, IDC_SEGMENT_BEHAVIOR, m_SegmentBehavior);
	DDX_Text(pDX, IDC_SEGMENT_FOLLOW_DISTANCE, m_SegmentFollowDistance);
	DDX_CBString(pDX, IDC_SEGMENT_SPEED, m_SegmentSpeed);
	DDX_Radio(pDX, IDC_SEGMENT_FOLLOW_LEFT, m_SegmentFollowLeftRight);
	DDX_Control(pDX, IDC_WAYPOINT_LISTBOX, m_WaypointListBox);
	DDX_Text(pDX, IDC_WAYPOINT_NAME, m_WaypointName);
	DDX_Text(pDX, IDC_WAYPOINT_ID, m_WaypointID);
	DDX_Text(pDX, IDC_WAYPOINT_X_FEET, m_WaypointLocationFeetX);
	DDX_Text(pDX, IDC_WAYPOINT_X_INCHES, m_WaypointLocationInchesX);
	DDV_MinMaxUInt(pDX, m_WaypointLocationInchesX, 0, 12);
	DDX_Text(pDX, IDC_WAYPOINT_Y_FEET, m_WaypointLocationFeetY);
	DDX_Text(pDX, IDC_WAYPOINT_Y_INCHES, m_WaypointLocationInchesY);
	DDV_MinMaxUInt(pDX, m_WaypointLocationInchesY, 0, 12);
	DDX_CBString(pDX, IDC_WAYPOINT_LANDMARK_TYPE1, m_WaypointLandmarkType1);
	DDX_CBString(pDX, IDC_WAYPOINT_LANDMARK_TYPE2, m_WaypointLandmarkType2);
	DDX_CBString(pDX, IDC_WAYPOINT_LANDMARK_TYPE3, m_WaypointLandmarkType3);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_DIRECTION1, m_WaypointLandmarkDirection1);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_DIRECTION2, m_WaypointLandmarkDirection2);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_DIRECTION3, m_WaypointLandmarkDirection3);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_HEIGHT1, m_WaypointLandmarkHeight1);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_HEIGHT2, m_WaypointLandmarkHeight2);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_HEIGHT3, m_WaypointLandmarkHeight3);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_RANGE1, m_WaypointLandmarkRange1);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_RANGE2, m_WaypointLandmarkRange2);
	DDX_Text(pDX, IDC_WAYPOINT_LANDMARK_RANGE3, m_WaypointLandmarkRange3);
	DDX_Text(pDX, IDC_ANCHOR_POINT_X, m_GPSAnchorPointX);
	DDX_Text(pDX, IDC_ANCHOR_POINT_Y, m_GPSAnchorPointY);
	DDX_Text(pDX, IDC_SEGMENT_AVOID_DISTANCE, m_SegmentAvoidDistance);
	DDX_Text(pDX, IDC_SEGMENT_COMPASS_CORRECTION, m_CompassCorrection);
	DDV_MinMaxInt(pDX, m_CompassCorrection, -360, 360);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CPathView, CFormView)
	//{{AFX_MSG_MAP(CPathView)
	ON_LBN_SELCHANGE(IDC_SEGMENT_LISTBOX, OnSelchangeSegmentListBox)
	ON_BN_CLICKED(IDC_SEGMENT_ADD, OnSegmentAdd)
	ON_BN_CLICKED(IDC_SEGMENT_INSERT, OnSegmentInsert)
	ON_BN_CLICKED(IDC_SEGMENT_REMOVE, OnSegmentRemove)
	ON_BN_CLICKED(IDC_SEGMENT_UPDATE, OnSegmentUpdate)
	ON_BN_CLICKED(IDC_SEGMENT_REMOVE_ALL, OnSegmentRemoveAll)
	ON_LBN_SELCHANGE(IDC_WAYPOINT_LISTBOX, OnSelchangeWaypointListBox)
	ON_BN_CLICKED(IDC_WAYPOINT_ADD, OnWaypointAdd)
	ON_BN_CLICKED(IDC_WAYPOINT_INSERT, OnWaypointInsert)
	ON_BN_CLICKED(IDC_WAYPOINT_UPDATE, OnWaypointUpdate)
	ON_BN_CLICKED(IDC_WAYPOINT_MOVE_UP, OnWaypointMoveUp)
	ON_BN_CLICKED(IDC_WAYPOINT_MOVE_DOWN, OnWaypointMoveDown)
	ON_BN_CLICKED(IDC_WAYPOINT_REMOVE, OnWaypointRemove)
	ON_BN_CLICKED(IDC_WAYPOINT_REMOVE_ALL, OnWaypointRemoveAll)
	ON_BN_CLICKED(IDC_SEGMENT_MOVE_UP, OnSegmentMoveUp)
	ON_BN_CLICKED(IDC_SEGMENT_MOVE_DOWN, OnSegmentMoveDown)
	ON_BN_CLICKED(IDC_CALCUALTE_NEW_WAYPOINT, OnCalcualteNewWaypoint)
	ON_COMMAND(ID_SEL_PATH_VIEW_BTN, OnSelPathViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_PATH_VIEW_BTN, OnUpdateSelPathViewBtn)
	ON_COMMAND(ID_SEL_CMD_VIEW_BTN, OnSelCmdViewBtn)
	ON_COMMAND(ID_SEL_MAP_VIEW_BTN, OnSelMapViewBtn)
	ON_BN_CLICKED(IDC_CALCUALTE_NEW_GPS_WAYPOINT, OnCalcualteNewGpsWaypoint)
	ON_BN_CLICKED(IDC_SET_GPS_ANCHOR_POINT, OnSetGpsAnchorPoint)
	ON_BN_CLICKED(IDC_RECALCUALTE_ALL_WAYPOINTS, OnRecalcualteAllWaypoints)
	ON_COMMAND(ID_SEL_SETUP_VIEW_BTN, OnSelSetupViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_SETUP_VIEW_BTN, OnUpdateSelSetupViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_CMD_VIEW_BTN, OnUpdateSelCmdViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_MAP_VIEW_BTN, OnUpdateSelMapViewBtn)
	ON_BN_CLICKED(IDC_RECALC_SEGMENTS, OnRecalcSegments)
	//}}AFX_MSG_MAP
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, CView::OnFilePrintPreview)
	ON_MESSAGE( (WM_ROBOT_DISPLAY_BULK_ITEMS), OnPathDisplayBulkItem )
	ON_MESSAGE( (WM_ROBOT_ADD_WAYPOINT_LOCATION), OnAddWaypointLocation )

END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// Overrides


#ifdef _DEBUG
CPathDoc* CPathView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CPathDoc)));
	return (CPathDoc*)m_pDocument;
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CPathView diagnostics

#ifdef _DEBUG
void CPathView::AssertValid() const
{
	CFormView::AssertValid();
}

void CPathView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif //_DEBUG


/////////////////////////////////////////////////////////////////////////////
// CPathView internal implementation

// Segment Data
CSegmentStruct* CPathView::FindSegmentStruct(int& nSel, POSITION& pos)
{

	nSel = m_SegmentListBox.GetCurSel();
	if (nSel == LB_ERR)
	{
		AfxMessageBox("From the listbox select the entry to be updated or removed.");
		return NULL;
	}

	// The CSegmentStruct pointer was saved as the listbox entry's data item.
	CSegmentStruct* pSegmentStruct = (CSegmentStruct*)m_SegmentListBox.GetItemDataPtr(nSel);

	// Find the CSegmentStruct pointer in the CTypedPtrList.
	pos = GetDocument()->m_SegmentList.Find(pSegmentStruct);

	// If the CSegmentStruct is displayed in the listbox, it should also be
	// found in the CTypedPtrList.
	ASSERT(pos != NULL);

	return pSegmentStruct;
}
void CPathView::AddSegmentStructToListBox(CSegmentStruct* pSegmentStruct, int nSel)
{
	// Add new CSegmentStruct to the listbox.
	CString str;
	pSegmentStruct->FormatSegmentStruct(str);
	if (nSel == -1)
		nSel = m_SegmentListBox.AddString(str);
	else
		m_SegmentListBox.InsertString(nSel, str);

	// Save the CSegmentStruct pointer as the listbox entry's "data item".
	m_SegmentListBox.SetItemDataPtr(nSel, pSegmentStruct);
}


//Waypoint Data
CWaypointStruct* CPathView::FindWaypointStruct(int& nSel, POSITION& pos)
{
	nSel = m_WaypointListBox.GetCurSel();
	if (nSel == LB_ERR)
	{
		AfxMessageBox("From the listbox select the entry to be updated or removed.");
		return NULL;
	}

	// The CWaypointStruct pointer was saved as the listbox entry's data item.
	CWaypointStruct* pWaypointStruct = (CWaypointStruct*)m_WaypointListBox.GetItemDataPtr(nSel);

	// Find the CWaypointStruct pointer in the CTypedPtrList.
	pos = GetDocument()->m_WaypointList.Find(pWaypointStruct);

	// If the CWaypointStruct is displayed in the listbox, it should also be
	// found in the CTypedPtrList.
	ASSERT(pos != NULL);

	return pWaypointStruct;
}
void CPathView::AddWaypointStructToListBox(CWaypointStruct* pWaypointStruct, int nSel)
{
	// Add new CWaypointStruct to the listbox.
	CString str;
	pWaypointStruct->FormatWaypointStruct(str);
	if (nSel == -1)
		nSel = m_WaypointListBox.AddString(str);
	else
		m_WaypointListBox.InsertString(nSel, str);

	// Save the CWaypointStruct pointer as the listbox entry's "data item".
	m_WaypointListBox.SetItemDataPtr(nSel, pWaypointStruct);
}



/////////////////////////////////////////////////////////////////////////////
// CPathView message handlers

/////////////////////////////////////////////////////////////////////////////
// Segment message handlers

void CPathView::CopyToSegmentStruct( CSegmentStruct* pSegmentStruct ) 
{
	// Copies all the local data members to a struct 

	pSegmentStruct->m_SegmentName = m_SegmentName;
	pSegmentStruct->m_SegmentFromWaypointID = m_SegmentFromWaypointID;
	pSegmentStruct->m_SegmentToWaypointID = m_SegmentToWaypointID;
	pSegmentStruct->m_SegmentBehavior = m_SegmentBehavior;
	pSegmentStruct->m_SegmentSpeed = m_SegmentSpeed;
	pSegmentStruct->m_SegmentDirection = m_SegmentDirection;
	pSegmentStruct->m_SegmentDirection = m_CompassCorrection;
	pSegmentStruct->m_SegmentDistanceFeet = m_SegmentDistanceFeet;
	pSegmentStruct->m_SegmentDistanceInches = m_SegmentDistanceInches;
	pSegmentStruct->m_SegmentFollowDistance = m_SegmentFollowDistance;
	pSegmentStruct->m_SegmentAvoidDistance = m_SegmentAvoidDistance;
	pSegmentStruct->m_SegmentFollowLeftRight = m_SegmentFollowLeftRight;

}

void CPathView::OnSegmentAdd() 
{
	if (UpdateData() != TRUE)
		return;

	// Add new CSegmentStruct to the CTypedPtrList
	CSegmentStruct* pSegmentStruct = new CSegmentStruct;

	CopyToSegmentStruct( pSegmentStruct ); 

	GetDocument()->m_SegmentList.AddTail(pSegmentStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit


	AddSegmentStructToListBox(pSegmentStruct);

	// Automatically increment to the next waypoint
	m_SegmentFromWaypointID++;
	m_SegmentToWaypointID = m_SegmentFromWaypointID+1;
	m_SegmentName.Format("S%u", m_SegmentFromWaypointID );
	UpdateData(FALSE);	// Force data exchange from View data members to GUI

}

void CPathView::OnSegmentInsert() 
{
	if (UpdateData() != TRUE)
		return;

	int nSel;
	POSITION pos;
	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	if (FindSegmentStruct(nSel, pos) == NULL)
		return;

	// Insert new CSegmentStruct in the CTypedPtrList
	CSegmentStruct* pSegmentStruct = new CSegmentStruct;

	CopyToSegmentStruct( pSegmentStruct ); 

	GetDocument()->m_SegmentList.InsertBefore(pos, pSegmentStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	AddSegmentStructToListBox(pSegmentStruct, nSel);
}

void CPathView::OnSegmentUpdate() 
{
	if (UpdateData() != TRUE)
		return;

	int nSel;
	POSITION pos;
	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	CSegmentStruct* pSegmentStruct = FindSegmentStruct(nSel, pos);
	if (pSegmentStruct == NULL)
		return;

	// Replace the value of the CSegmentStruct in the CTypedPtrList.
	CopyToSegmentStruct( pSegmentStruct ); 
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	// Replace the displayed CSegmentStruct in the listbox by removing
	// the old listbox entry and adding a new entry.
	m_SegmentListBox.DeleteString(nSel);
	AddSegmentStructToListBox(pSegmentStruct, nSel);

	ROBOT_LOG( TRUE,  "DEBUG - SEGMENT FOLLOW LEFT/RIGHT = %d\n", pSegmentStruct->m_SegmentFollowLeftRight)
}

void CPathView::OnSegmentRemove() 
{
	int nSel;
	POSITION pos;
	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	CSegmentStruct* pSegmentStruct = FindSegmentStruct(nSel, pos);
	if (pSegmentStruct == NULL)
		return;

	// Remove the CSegmentStruct ptr from the CTypedPtrList.
	GetDocument()->m_SegmentList.RemoveAt(pos);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	// Delete the CSegmentStruct object. (The CTypePtrList only holds the ptr.)
	SAFE_DELETE( pSegmentStruct );

	// Remove the corresponding entry from the listbox.
	m_SegmentListBox.DeleteString(nSel);
}

void CPathView::OnSegmentRemoveAll() 
{
	CSegmentStructList& SegmentList = GetDocument()->m_SegmentList;

	// Delete all of the CSegmentStruct objects pointed to
	// by the CTypedPtrList. Then remove all of the
	// CSegmentStruct pointers from the CTypedPtrList, which
	// is faster than removing each individually.
	POSITION pos = SegmentList.GetHeadPosition();
	while (pos != NULL)
	{
		delete SegmentList.GetNext(pos);
	}
	SegmentList.RemoveAll();
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	// Remove all of the corresponding formatted strings from the listbox.
	m_SegmentListBox.ResetContent();
}

void CPathView::OnSelchangeSegmentListBox() 
{
	// Update the edit control to reflect the new selection
	// in the listbox.

	int nSel;
	POSITION pos;
	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	CSegmentStruct* pSegmentStruct = FindSegmentStruct(nSel, pos);

	// Copy data the other way (from the struct to the data member)
	m_SegmentName = pSegmentStruct->m_SegmentName;
	m_SegmentFromWaypointID = pSegmentStruct->m_SegmentFromWaypointID;
	m_SegmentToWaypointID = pSegmentStruct->m_SegmentToWaypointID;
	m_SegmentBehavior = pSegmentStruct->m_SegmentBehavior;
	m_SegmentSpeed = pSegmentStruct->m_SegmentSpeed;
	m_SegmentDirection = pSegmentStruct->m_SegmentDirection;
	m_SegmentDirection = pSegmentStruct->m_CompassCorrection;
	m_SegmentDistanceFeet = pSegmentStruct->m_SegmentDistanceFeet;
	m_SegmentDistanceInches = pSegmentStruct->m_SegmentDistanceInches;
	m_SegmentFollowDistance = pSegmentStruct->m_SegmentFollowDistance;
	m_SegmentAvoidDistance = pSegmentStruct->m_SegmentAvoidDistance;
	m_SegmentFollowLeftRight = pSegmentStruct->m_SegmentFollowLeftRight;


	UpdateData(FALSE);
}

void CPathView::OnSegmentMoveUp() 
{

	int nSel;
	POSITION OldPos, PriorPos;

	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	CSegmentStruct* pSegmentStruct = FindSegmentStruct(nSel, OldPos);
	if (pSegmentStruct == NULL)
		return;
	
	if( nSel < 1 )	// Cant move any higher
		return;
	
	// Find pointer to prior entry in CTypedPtrList
	PriorPos = OldPos;
	GetDocument()->m_SegmentList.GetPrev( PriorPos );	// Changes NewPos to point to prior obj
	if( PriorPos == NULL )	// Cant move any higher
		return;

	// Remove the old pointer from the CTypedPtrList and listbox
	GetDocument()->m_SegmentList.RemoveAt(OldPos);
	m_SegmentListBox.DeleteString(nSel);

	// Insert in new location
	GetDocument()->m_SegmentList.InsertBefore(PriorPos, pSegmentStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	AddSegmentStructToListBox(pSegmentStruct, (nSel-1));

	m_SegmentListBox.SetCurSel(nSel-1);	// Select new location
	
}

void CPathView::OnSegmentMoveDown() 
{
	int nSel;
	POSITION OldPos, NextPos;

	// Find the CSegmentStruct in the CTypedPtrList and listbox.
	CSegmentStruct* pSegmentStruct = FindSegmentStruct(nSel, OldPos);
	if (pSegmentStruct == NULL)
		return;

	// Find pointer to prior entry in CTypedPtrList
	NextPos = OldPos;
	GetDocument()->m_SegmentList.GetNext( NextPos );	// Changes NewPos to point to next obj
	if( NextPos == NULL )	// Cant move any lower
		return;

	// Insert in new location
	GetDocument()->m_SegmentList.InsertAfter(NextPos, pSegmentStruct);
	AddSegmentStructToListBox(pSegmentStruct, (nSel+2));

	// Remove the old pointer from the CTypedPtrList and listbox
	GetDocument()->m_SegmentList.RemoveAt(OldPos);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	m_SegmentListBox.DeleteString(nSel);

	m_SegmentListBox.SetCurSel(nSel+1);	// Select new location
	
}


/////////////////////////////////////////////////////////////////////////////
// Waypoint message handlers

void CPathView::CopyToWaypointStruct( CWaypointStruct* pWaypointStruct ) 
{
	// Copies all the data members to a struct 

	pWaypointStruct->m_WaypointName = m_WaypointName;
	pWaypointStruct->m_WaypointID = m_WaypointID;
	pWaypointStruct->m_WaypointLocationFeetX = m_WaypointLocationFeetX;
	pWaypointStruct->m_WaypointLocationInchesX = m_WaypointLocationInchesX;
	pWaypointStruct->m_WaypointLocationFeetY = m_WaypointLocationFeetY;
	pWaypointStruct->m_WaypointLocationInchesY = m_WaypointLocationInchesY;

	pWaypointStruct->m_WaypointLandmarkType1 = m_WaypointLandmarkType1;
	pWaypointStruct->m_WaypointLandmarkDirection1 = m_WaypointLandmarkDirection1;
	pWaypointStruct->m_WaypointLandmarkRange1 = m_WaypointLandmarkRange1;
	pWaypointStruct->m_WaypointLandmarkHeight1 = m_WaypointLandmarkHeight1;

	pWaypointStruct->m_WaypointLandmarkType2 = m_WaypointLandmarkType2;
	pWaypointStruct->m_WaypointLandmarkDirection2 = m_WaypointLandmarkDirection2;
	pWaypointStruct->m_WaypointLandmarkRange2 = m_WaypointLandmarkRange2;
	pWaypointStruct->m_WaypointLandmarkHeight2 = m_WaypointLandmarkHeight2;

	pWaypointStruct->m_WaypointLandmarkType3 = m_WaypointLandmarkType3;
	pWaypointStruct->m_WaypointLandmarkDirection3 = m_WaypointLandmarkDirection3;
	pWaypointStruct->m_WaypointLandmarkRange3 = m_WaypointLandmarkRange3;
	pWaypointStruct->m_WaypointLandmarkHeight3 = m_WaypointLandmarkHeight3;

}

void CPathView::CopyWaypointStruct( CWaypointStruct* pToWaypointStruct, CWaypointStruct* pFromWaypointStruct ) 
{
	// Copies all the data members to a struct 

	pToWaypointStruct->m_WaypointName = pFromWaypointStruct->m_WaypointName;
	pToWaypointStruct->m_WaypointID = pFromWaypointStruct->m_WaypointID;
	pToWaypointStruct->m_WaypointLocationFeetX = pFromWaypointStruct->m_WaypointLocationFeetX;
	pToWaypointStruct->m_WaypointLocationInchesX = pFromWaypointStruct->m_WaypointLocationInchesX;
	pToWaypointStruct->m_WaypointLocationFeetY = pFromWaypointStruct->m_WaypointLocationFeetY;
	pToWaypointStruct->m_WaypointLocationInchesY = pFromWaypointStruct->m_WaypointLocationInchesY;

	pToWaypointStruct->m_WaypointLandmarkType1 = pFromWaypointStruct->m_WaypointLandmarkType1;
	pToWaypointStruct->m_WaypointLandmarkDirection1 = pFromWaypointStruct->m_WaypointLandmarkDirection1;
	pToWaypointStruct->m_WaypointLandmarkRange1 = pFromWaypointStruct->m_WaypointLandmarkRange1;
	pToWaypointStruct->m_WaypointLandmarkHeight1 = pFromWaypointStruct->m_WaypointLandmarkHeight1;

	pToWaypointStruct->m_WaypointLandmarkType2 = pFromWaypointStruct->m_WaypointLandmarkType2;
	pToWaypointStruct->m_WaypointLandmarkDirection2 = pFromWaypointStruct->m_WaypointLandmarkDirection2;
	pToWaypointStruct->m_WaypointLandmarkRange2 = pFromWaypointStruct->m_WaypointLandmarkRange2;
	pToWaypointStruct->m_WaypointLandmarkHeight2 = pFromWaypointStruct->m_WaypointLandmarkHeight2;

	pToWaypointStruct->m_WaypointLandmarkType3 = pFromWaypointStruct->m_WaypointLandmarkType3;
	pToWaypointStruct->m_WaypointLandmarkDirection3 = pFromWaypointStruct->m_WaypointLandmarkDirection3;
	pToWaypointStruct->m_WaypointLandmarkRange3 = pFromWaypointStruct->m_WaypointLandmarkRange3;
	pToWaypointStruct->m_WaypointLandmarkHeight3 = pFromWaypointStruct->m_WaypointLandmarkHeight3;

}

void CPathView::OnWaypointAdd() 
{
	if (UpdateData() != TRUE)
		return;

	// Add new CWaypointStruct to the CTypedPtrList
	CWaypointStruct* pWaypointStruct = new CWaypointStruct;

	CopyToWaypointStruct( pWaypointStruct );

	if( 0 == pWaypointStruct->m_WaypointID )
	{
		// Starting Waypoint.  Set robot map position to start here!
		g_pFullSensorStatus->CurrentLocation.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
		g_pFullSensorStatus->CurrentLocation.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
	}

	GetDocument()->m_WaypointList.AddTail(pWaypointStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	AddWaypointStructToListBox(pWaypointStruct);
}

void CPathView::OnWaypointInsert() 
{
	if (UpdateData() != TRUE)
		return;

	int nSel;
	POSITION pos;
	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	if (FindWaypointStruct(nSel, pos) == NULL)
		return;

	// Insert new CWaypointStruct in the CTypedPtrList
	CWaypointStruct* pWaypointStruct = new CWaypointStruct;

//	pWaypointStruct->m_WaypointName = m_WaypointName;
//	pWaypointStruct->m_WaypointFollowGap = m_WaypointFollowGap;
	CopyToWaypointStruct( pWaypointStruct );

	if( 0 == pWaypointStruct->m_WaypointID )
	{
		// Starting Waypoint.  Set robot map position to start here!
		g_pFullSensorStatus->CurrentLocation.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
		g_pFullSensorStatus->CurrentLocation.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
	}

	GetDocument()->m_WaypointList.InsertBefore(pos, pWaypointStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	AddWaypointStructToListBox(pWaypointStruct, nSel);
}

void CPathView::OnWaypointUpdate() 
{
	if (UpdateData() != TRUE)
		return;

	int nSel;
	POSITION pos;
	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	CWaypointStruct* pWaypointStruct = FindWaypointStruct(nSel, pos);
	if (pWaypointStruct == NULL)
		return;

	// Replace the value of the CWaypointStruct in the CTypedPtrList.
//	pWaypointStruct->m_WaypointName = m_WaypointName;
//	pWaypointStruct->m_WaypointFollowGap = m_WaypointFollowGap;
	CopyToWaypointStruct( pWaypointStruct );

	if( 0 == pWaypointStruct->m_WaypointID )
	{
		// Starting Waypoint.  Set robot map position to start here!
		g_pFullSensorStatus->CurrentLocation.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
		g_pFullSensorStatus->CurrentLocation.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
	}

	// Replace the displayed CWaypointStruct in the listbox by removing
	// the old listbox entry and adding a new entry.
	m_WaypointListBox.DeleteString(nSel);
	AddWaypointStructToListBox(pWaypointStruct, nSel);
}

void CPathView::OnWaypointRemove() 
{
	int nSel;
	POSITION pos;
	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	CWaypointStruct* pWaypointStruct = FindWaypointStruct(nSel, pos);
	if (pWaypointStruct == NULL)
		return;

	if( 0 == pWaypointStruct->m_WaypointID )
	{
		// Starting Waypoint.  Reset robot map position to default
		g_pFullSensorStatus->CurrentLocation.x = DEFAULT_ROBOT_START_POSITION_X;
		g_pFullSensorStatus->CurrentLocation.y = DEFAULT_ROBOT_START_POSITION_X;
	}

	// Remove the CWaypointStruct ptr from the CTypedPtrList.
	GetDocument()->m_WaypointList.RemoveAt(pos);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	// Delete the CWaypointStruct object. (The CTypePtrList only holds the ptr.)
	SAFE_DELETE( pWaypointStruct );

	// Remove the corresponding entry from the listbox.
	m_WaypointListBox.DeleteString(nSel);
}

void CPathView::OnWaypointRemoveAll() 
{
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;

	// Delete all of the CWaypointStruct objects pointed to
	// by the CTypedPtrList. Then remove all of the
	// CWaypointStruct pointers from the CTypedPtrList, which
	// is faster than removing each individually.
	POSITION pos = WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		delete WaypointList.GetNext(pos);
	}
	WaypointList.RemoveAll();
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

	// Reset robot map position to default
	g_pFullSensorStatus->CurrentLocation.x = DEFAULT_ROBOT_START_POSITION_X;
	g_pFullSensorStatus->CurrentLocation.y = DEFAULT_ROBOT_START_POSITION_Y;

	// Remove all of the corresponding formatted strings from the listbox.
	m_WaypointListBox.ResetContent();
}

void CPathView::OnSelchangeWaypointListBox() 
{
	// Update the edit control to reflect the new selection
	// in the listbox.

	int nSel;
	POSITION pos;
	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	CWaypointStruct* pWaypointStruct = FindWaypointStruct(nSel, pos);

//	m_WaypointName = pWaypointStruct->m_WaypointName;
//	m_WaypointFollowGap = pWaypointStruct->m_WaypointFollowGap;

	// Copy data the other way (from the struct to the GUI data member)
	m_WaypointName = pWaypointStruct->m_WaypointName;
	m_WaypointID = pWaypointStruct->m_WaypointID;
	m_WaypointLocationFeetX = pWaypointStruct->m_WaypointLocationFeetX;
	m_WaypointLocationInchesX = pWaypointStruct->m_WaypointLocationInchesX;
	m_WaypointLocationFeetY = pWaypointStruct->m_WaypointLocationFeetY;
	m_WaypointLocationInchesY = pWaypointStruct->m_WaypointLocationInchesY;

	m_WaypointLandmarkType1 = pWaypointStruct->m_WaypointLandmarkType1;
	m_WaypointLandmarkDirection1 = pWaypointStruct->m_WaypointLandmarkDirection1;
	m_WaypointLandmarkRange1 = pWaypointStruct->m_WaypointLandmarkRange1;
	m_WaypointLandmarkHeight1 = pWaypointStruct->m_WaypointLandmarkHeight1;

	m_WaypointLandmarkType2 = pWaypointStruct->m_WaypointLandmarkType2;
	m_WaypointLandmarkDirection2 = pWaypointStruct->m_WaypointLandmarkDirection2;
	m_WaypointLandmarkRange2 = pWaypointStruct->m_WaypointLandmarkRange2;
	m_WaypointLandmarkHeight2 = pWaypointStruct->m_WaypointLandmarkHeight2;

	m_WaypointLandmarkType3 = pWaypointStruct->m_WaypointLandmarkType3;
	m_WaypointLandmarkDirection3 = pWaypointStruct->m_WaypointLandmarkDirection3;
	m_WaypointLandmarkRange3 = pWaypointStruct->m_WaypointLandmarkRange3;
	m_WaypointLandmarkHeight3 = pWaypointStruct->m_WaypointLandmarkHeight3;

	UpdateData(FALSE);
}


void CPathView::OnWaypointMoveUp() 
{

	int nSel;
	POSITION OldPos, PriorPos;

	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	CWaypointStruct* pWaypointStruct = FindWaypointStruct(nSel, OldPos);
	if (pWaypointStruct == NULL)
		return;
	
	if( nSel < 1 )	// Cant move any higher
		return;
	
	// Find pointer to prior entry in CTypedPtrList
	PriorPos = OldPos;
	GetDocument()->m_WaypointList.GetPrev( PriorPos );	// Changes NewPos to point to prior obj
	if( PriorPos == NULL )	// Cant move any higher
		return;

	// Remove the old pointer from the CTypedPtrList and listbox
	GetDocument()->m_WaypointList.RemoveAt(OldPos);
	m_WaypointListBox.DeleteString(nSel);

	// Insert in new location
	GetDocument()->m_WaypointList.InsertBefore(PriorPos, pWaypointStruct);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	AddWaypointStructToListBox(pWaypointStruct, (nSel-1));

	m_WaypointListBox.SetCurSel(nSel-1);	// Select new location

}

void CPathView::OnWaypointMoveDown() 
{
	int nSel;
	POSITION OldPos, NextPos;

	// Find the CWaypointStruct in the CTypedPtrList and listbox.
	CWaypointStruct* pWaypointStruct = FindWaypointStruct(nSel, OldPos);
	if (pWaypointStruct == NULL)
		return;

	// Find pointer to prior entry in CTypedPtrList
	NextPos = OldPos;
	GetDocument()->m_WaypointList.GetNext( NextPos );	// Changes NewPos to point to next obj
	if( NextPos == NULL )	// Cant move any lower
		return;

	// Insert in new location
	GetDocument()->m_WaypointList.InsertAfter(NextPos, pWaypointStruct);
	AddWaypointStructToListBox(pWaypointStruct, (nSel+2));

	// Remove the old pointer from the CTypedPtrList and listbox
	GetDocument()->m_WaypointList.RemoveAt(OldPos);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	m_WaypointListBox.DeleteString(nSel);

	m_WaypointListBox.SetCurSel(nSel+1);	// Select new location

	
}


void CPathView::OnRecalcualteAllWaypoints() 
{
	// Given starting Waypoint X,Y plus distance and direction, 
	// calculate new Waypoint X,Y for each segment termination.

	POSITION pos;
	CWaypointStruct* pWaypointStruct;
	CWaypointStruct* pStartWaypoint = new CWaypointStruct;

	int HighestWaypointID = 1;
	double X, Y;	// Delta from last Waypoint
	int MapX, MapY;	// Absolute coordinates, allows for negative numbers
	BOOL WaypointFound = FALSE;

	UpdateData(TRUE);	// Force data exchange from GUI to View data members

	// Get the first Waypoint's data
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;
	pos = WaypointList.GetHeadPosition();
	if(NULL == pos)
	{
		AfxMessageBox("Need a valid first Waypoint");
		return;
	}

	// Save the first waypoint.
	pWaypointStruct = WaypointList.GetHead();
	MapX = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
	MapY = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
	CopyWaypointStruct( pStartWaypoint, pWaypointStruct );


	// Clear the Waypoint CTypedPtrList. 
	pos = WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		delete WaypointList.GetNext(pos);
	}
	WaypointList.RemoveAll();
	// Clear the listbox in prep for receiving new values
	m_WaypointListBox.ResetContent();

	// but add back in the Starting Waypoint
	GetDocument()->m_WaypointList.AddTail(pStartWaypoint);
	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	AddWaypointStructToListBox(pStartWaypoint);

	// For each segment, calculate and insert the waypoint data
	CSegmentStructList& SegmentList = GetDocument()->m_SegmentList;
	pos = SegmentList.GetHeadPosition();
	while (pos != NULL)
	{
		CSegmentStruct* pSegmentStruct = SegmentList.GetNext(pos);
		CWaypointStruct* pWaypointStruct = new CWaypointStruct;

		// Initialize the new Waypoint Struct
		pWaypointStruct->m_WaypointName = _T("");
		pWaypointStruct->m_WaypointID = 0;
		pWaypointStruct->m_WaypointLocationFeetX = 0;
		pWaypointStruct->m_WaypointLocationInchesX = 0;
		pWaypointStruct->m_WaypointLocationFeetY = 0;
		pWaypointStruct->m_WaypointLocationInchesY = 0;
		pWaypointStruct->m_WaypointLandmarkType1 = _T("None");
		pWaypointStruct->m_WaypointLandmarkType2 = _T("None");
		pWaypointStruct->m_WaypointLandmarkType3 = _T("None");
		pWaypointStruct->m_WaypointLandmarkDirection1 = 0;
		pWaypointStruct->m_WaypointLandmarkDirection2 = 0;
		pWaypointStruct->m_WaypointLandmarkDirection3 = 0;
		pWaypointStruct->m_WaypointLandmarkHeight1 = 0;
		pWaypointStruct->m_WaypointLandmarkHeight2 = 0;
		pWaypointStruct->m_WaypointLandmarkHeight3 = 0;
		pWaypointStruct->m_WaypointLandmarkRange1 = 0;
		pWaypointStruct->m_WaypointLandmarkRange2 = 0;
		pWaypointStruct->m_WaypointLandmarkRange3 = 0;


		// Calculate new X,Y
		int Distance = (pSegmentStruct->m_SegmentDistanceFeet *12) + pSegmentStruct->m_SegmentDistanceInches;

		if( pSegmentStruct->m_SegmentDirection >= 360 )
		{
			AfxMessageBox("Segment Direction must be less than 360 degrees!");
			return;
		}

		// Convert Degrees to Radians then do the math
		//double TwoPi = 2.0 * 3.141592;
		//double DirectionRadians = (((double)m_SegmentDirection / 360.0) * TWO_PI);
		double DirectionRadians = (double)(pSegmentStruct->m_SegmentDirection) * DEGREES_TO_RADIANS;
		X = Distance * sin( DirectionRadians );	// Returns negative numbers as needed
		Y = Distance * cos( DirectionRadians );
		
		if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
		if(Y>=0) Y += 0.5; else Y -= 0.5;
		MapX = (int)MapX + (int)X;		// Get new Map absolute X,Y
		MapY = (int)MapY + (int)Y;

		if( (MapX < 0) || (MapY < 0) || (MapX > MAX_MAP_SIZE) || (MapY > MAX_MAP_SIZE) )
		{
			CString ErrorMsg;
			ErrorMsg.Format("New value is outside map boundries!\nX = %d  Y = %d", 
				MapX, MapY);
			AfxMessageBox( (LPCTSTR)ErrorMsg );
			return;
		}
	
		//pSegmentStruct->m_SegmentToWaypointID = HighestWaypointID++;

		
		pWaypointStruct->m_WaypointID = pSegmentStruct->m_SegmentToWaypointID;
		pWaypointStruct->m_WaypointName.Format("WP%u", pWaypointStruct->m_WaypointID);
	
		// Convert to Feet and Inches and insert calculated data into Waypoint struct
		pWaypointStruct->m_WaypointLocationFeetX = MapX / 12;
		pWaypointStruct->m_WaypointLocationFeetY = MapY / 12;
		pWaypointStruct->m_WaypointLocationInchesX = (int )MapX % 12;
		pWaypointStruct->m_WaypointLocationInchesY = (int )MapY % 12;

		GetDocument()->m_WaypointList.AddTail(pWaypointStruct);
		GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

		AddWaypointStructToListBox(pWaypointStruct);

	
/**
	// Copy all of the strings from the document's WaypointList
	// to the Waypoint Listbox.
	m_WaypointListBox.ResetContent();
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;
	pos = WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		CWaypointStruct* pWaypointStruct = WaypointList.GetNext(pos);
		AddWaypointStructToListBox(pWaypointStruct);

		if( 0 == pWaypointStruct->m_WaypointID )
		{
			// Starting Waypoint.  Set robot map position to start here!
			g_pFullSensorStatus->CurrentLocation.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
			g_pFullSensorStatus->CurrentLocation.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
		}
	}
	
***/	
	
	}

	UpdateData(FALSE);	// Force data exchange from View data members to GUI

}

void CPathView::OnCalcualteNewWaypoint() 
{
	// Given starting Waypoint X,Y plus distance and direction, 
	// calculate new Waypoint X,Y.  Also, find a new Waypoint ID.

	POSITION pos;
	CWaypointStruct* pWaypointStruct, pStartWaypoint;
	int HighestWaypointID = -1;
	double X, Y;	// Delta from Start Waypoint
	int MapX, MapY;	// Absolute coordinates, allows for negative numbers
	BOOL WaypointFound = FALSE;
	int StartX, StartY;

	UpdateData(TRUE);	// Force data exchange from GUI to View data members

	//Find Starting Waypoint data
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;
	pos = WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		pWaypointStruct = WaypointList.GetNext(pos);

		if( (int)pWaypointStruct->m_WaypointID > HighestWaypointID )
		{
			HighestWaypointID = (int)pWaypointStruct->m_WaypointID;
		}

		if( pWaypointStruct->m_WaypointID == m_SegmentFromWaypointID )
		{
			WaypointFound = TRUE;
			StartX = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
			StartY = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
		}
	}
	if( !WaypointFound )
	{
		AfxMessageBox("From Waypoint ID is not valid!\nPlease enter valid ID");
		return;
	}

	// Calculate new X,Y
	int Distance = (m_SegmentDistanceFeet *12) + m_SegmentDistanceInches;

	if( m_SegmentDirection > 360 )
	{
		AfxMessageBox("Segment Direction must be less than 360 degrees!");
		return;
	}

	// Convert Degrees to Radians then do the math
	//double TwoPi = 2.0 * 3.141592;
	//double DirectionRadians = (((double)m_SegmentDirection / 360.0) * TwoPi);
	double DirectionRadians = (double)m_SegmentDirection * DEGREES_TO_RADIANS;
	X = Distance * sin( DirectionRadians );	// Returns negative numbers as needed
	Y = Distance * cos( DirectionRadians );
	
	if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
	if(Y>=0) Y += 0.5; else Y -= 0.5;
	MapX = (int)StartX + (int)X;		// Get new Map absolute X,Y
	MapY = (int)StartY + (int)Y;

	if( (MapX < 0) || (MapY < 0) || (MapX > MAX_MAP_SIZE) || (MapY > MAX_MAP_SIZE) )
	{
		CString ErrorMsg;
		ErrorMsg.Format("New value is outside map boundries!\nX = %d  Y = %d", 
			MapX, MapY);
		AfxMessageBox( (LPCTSTR)ErrorMsg );
		return;
	}
	
	// Select the Waypoint to update
	if( m_SegmentToWaypointID == 0 )
	{
		// user left blank
		m_SegmentToWaypointID = HighestWaypointID+1;
	}
	m_WaypointID = m_SegmentToWaypointID;
	m_WaypointName.Format("WP%u", m_WaypointID);
	
	// Convert to Feet and Inches and insert calculated data into Waypoint Form
	m_WaypointLocationFeetX = MapX / 12;
	m_WaypointLocationFeetY = MapY / 12;
	m_WaypointLocationInchesX = (int )MapX % 12;
	m_WaypointLocationInchesY = (int )MapY % 12;
	//CString Str;


	UpdateData(FALSE);	// Force data exchange from View data members to GUI

}



/**
void CPathView::PrintPageHeader(CDC *pDC, CPrintInfo *pInfo, CString &strHeader)
{
	// Specify left text alignment
	pDC->SetTextAlign(TA_LEFT);

	// Print a page header consisting of the name of
	// the document and a horizontal line
	pDC->TextOut(0, -25, strHeader);  // 1/4 inch down

	// Draw a line across the page, below the header
	TEXTMETRIC textMetric;
	pDC->GetTextMetrics(&textMetric);
	int y = -35 - textMetric.tmHeight;	// line 1/10th in. below text
	pDC->MoveTo(0, y);							// from left margin
	pDC->LineTo(pInfo->m_rectDraw.right, y);	//  to right margin

	// Subtract from the drawing rectangle the space used by header.
	y -= 25;     // space 1/4 inch below (top of) line
	pInfo->m_rectDraw.top += y;

}
**/

void CPathView::OnPrint(CDC* pDC, CPrintInfo* pInfo) 
{
#define PATH_TEXT_HEIGHT	50
#define PATH_FIRST_LINE		250
#define LINES_PER_PAGE		60
#define LEFT_MARGIN			100

//	if (pInfo->m_nCurPage == 1)     // page no. 1 is the title page
//	{
//		PrintTitlePage(pDC, pInfo);
//	}

	// Note: forms can only use MM_TEXT.  Bottom of the page is about 3000+ pixels
	// probably 300dpi * 11inches high.

	// Prepare a font size for printing
	LOGFONT logFont;
	memset(&logFont, 0, sizeof(LOGFONT));
	logFont.lfHeight = PATH_TEXT_HEIGHT;
	logFont.lfPitchAndFamily = FF_MODERN;	// Ask for any evenly spaced font
	CFont font;
	CFont* pOldFont = NULL;
	if (font.CreateFontIndirect(&logFont))
	{
		pOldFont = pDC->SelectObject(&font);
	}

	CString strText;

	// Print the Page Header	
	// Get the file name, to be displayed on title page
	pDC->SetTextAlign(TA_CENTER);
	strText = GetDocument()->GetTitle();
	pDC->TextOut(pInfo->m_rectDraw.right/2, 100, strText);	// Center Title

	int LineNumber = 1;
	pDC->SetTextAlign(TA_LEFT);

	// Print the Table Header
	strText.Format("No. Segment");
	pDC->TextOut(pInfo->m_rectDraw.left+LEFT_MARGIN, 
		190 , strText);
	strText.Format("From  To   Dir   Distance          Speed     Behavior");
	pDC->TextOut(pInfo->m_rectDraw.left+LEFT_MARGIN+400, 
		200, strText);

	// Draw a line
	pDC->MoveTo(pInfo->m_rectDraw.left+LEFT_MARGIN, 210 + PATH_TEXT_HEIGHT);							// from left margin
	pDC->LineTo(pInfo->m_rectDraw.right-LEFT_MARGIN,210 + PATH_TEXT_HEIGHT);							// from left margin

	// Print the data
	// Iterate through CSegmentStruct pointers in the CTypedPtrList.
	POSITION pos = GetDocument()->m_SegmentList.GetHeadPosition();
	while (pos != NULL)
	{

		CSegmentStruct* pSegmentStruct = GetDocument()->m_SegmentList.GetNext(pos);

		strText.Format("%02d: %s", LineNumber, pSegmentStruct->m_SegmentName);
		pDC->TextOut(pInfo->m_rectDraw.left+LEFT_MARGIN, 
			(PATH_FIRST_LINE + (PATH_TEXT_HEIGHT*LineNumber)), strText);

		strText.Format("WP%02u->%02u   %03u (%+d)   %04u' %02u\" %13s     %s", 
			pSegmentStruct->m_SegmentFromWaypointID, 
			pSegmentStruct->m_SegmentToWaypointID,
			pSegmentStruct->m_SegmentDirection,
			pSegmentStruct->m_CompassCorrection,			
			pSegmentStruct->m_SegmentDistanceFeet,
			pSegmentStruct->m_SegmentDistanceInches,
			pSegmentStruct->m_SegmentSpeed,
			pSegmentStruct->m_SegmentBehavior);
		pDC->TextOut(pInfo->m_rectDraw.left+LEFT_MARGIN+400, 
			(PATH_FIRST_LINE + (PATH_TEXT_HEIGHT*LineNumber)), strText);

/*		w = (WORD)pSegmentStruct->m_SegmentFollowDistance; ar << w;
		w = (WORD)pSegmentStruct->m_SegmentAvoidDistance; ar << w;
		w = (WORD)pSegmentStruct->m_SegmentFollowLeftRight; ar << w;
*/
		if( LineNumber++ >= LINES_PER_PAGE )
		{
			break;	// Cant print any more - TODO-CAR - make this handle multiple pages
		}
	}


/*
//	m_WaypointName = pWaypointStruct->m_WaypointName;
//	m_WaypointFollowGap = pWaypointStruct->m_WaypointFollowGap;

	// Copy data the other way (from the struct to the GUI data member)
	m_WaypointName = pWaypointStruct->m_WaypointName;
	m_WaypointID = pWaypointStruct->m_WaypointID;
	m_WaypointLocationFeetX = pWaypointStruct->m_WaypointLocationFeetX;
	m_WaypointLocationInchesX = pWaypointStruct->m_WaypointLocationInchesX;
	m_WaypointLocationFeetY = pWaypointStruct->m_WaypointLocationFeetY;
	m_WaypointLocationInchesY = pWaypointStruct->m_WaypointLocationInchesY;

	m_WaypointLandmarkType1 = pWaypointStruct->m_WaypointLandmarkType1;
	m_WaypointLandmarkDirection1 = pWaypointStruct->m_WaypointLandmarkDirection1;
	m_WaypointLandmarkRange1 = pWaypointStruct->m_WaypointLandmarkRange1;
	m_WaypointLandmarkHeight1 = pWaypointStruct->m_WaypointLandmarkHeight1;

	m_WaypointLandmarkType2 = pWaypointStruct->m_WaypointLandmarkType2;
	m_WaypointLandmarkDirection2 = pWaypointStruct->m_WaypointLandmarkDirection2;
	m_WaypointLandmarkRange2 = pWaypointStruct->m_WaypointLandmarkRange2;
	m_WaypointLandmarkHeight2 = pWaypointStruct->m_WaypointLandmarkHeight2;

	m_WaypointLandmarkType3 = pWaypointStruct->m_WaypointLandmarkType3;
	m_WaypointLandmarkDirection3 = pWaypointStruct->m_WaypointLandmarkDirection3;
	m_WaypointLandmarkRange3 = pWaypointStruct->m_WaypointLandmarkRange3;
	m_WaypointLandmarkHeight3 = pWaypointStruct->m_WaypointLandmarkHeight3;

*/

	
	// Print the Footer
	pDC->SetTextAlign(TA_CENTER);
	strText = "-- End --";
	pDC->TextOut(pInfo->m_rectDraw.right/2, 3000, strText);
	
	
	
	if (pOldFont != NULL)
	{
		pDC->SelectObject(pOldFont);
	}
	
}

/////////////////////////////////////////////////////////////////////////////
// CPathView printing

BOOL CPathView::OnPreparePrinting(CPrintInfo* pInfo)
{
	pInfo->SetMaxPage(1);  // the document is two pages long:
	// the first page is the title page, the second page is the drawing

	BOOL bRet = DoPreparePrinting (pInfo);	// default preparation

	pInfo->m_nNumPreviewPages = 1;		//Preview 1 pages at a time
	// Set this value after calling DoPreparePrinting to override
	// value read from registry
	return bRet;

}


void CPathView::OnSelPathViewBtn() 
{
	GetParentFrame()->ActivateFrame();	//RecalcLayout();
}
void CPathView::OnSelCmdViewBtn() 
{
	::PostMessage( g_RobotCmdViewHWND, (UINT)WM_COMMAND, ID_SEL_CMD_VIEW_BTN, 0);	
}
void CPathView::OnSelMapViewBtn() 
{
	::PostMessage( g_RobotMapViewHWND, (UINT)WM_COMMAND, ID_SEL_MAP_VIEW_BTN, 0);	
}
void CPathView::OnSelSetupViewBtn() 
{
	::PostMessage( g_RobotSetupViewHWND, (UINT)WM_COMMAND, ID_SEL_SETUP_VIEW_BTN, 0);	
}


void CPathView::OnUpdateSelPathViewBtn(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);

}
void CPathView::OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotCmdViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CPathView::OnUpdateSelMapViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotMapViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CPathView::OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotSetupViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

LRESULT CPathView::OnAddWaypointLocation(WPARAM LocationX, LPARAM LocationY)
{
	// User has selected Waypoint Mode on the Map View,
	// and clicked a spot to add a waypoint.  Populate the form and allow
	// user to enter additional data.

	POSITION pos;
	CWaypointStruct* pWaypointStruct;
	int HighestWaypointID = 0;

	//Find current highest Waypoint number, and add one for the next one
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;
	pos = WaypointList.GetHeadPosition();
	while (pos != NULL)
	{
		pWaypointStruct = WaypointList.GetNext(pos);

		if( (int)pWaypointStruct->m_WaypointID > HighestWaypointID )
		{
			HighestWaypointID = (int)pWaypointStruct->m_WaypointID;
		}
	}
	m_WaypointID = HighestWaypointID+1;
	m_WaypointName.Format("WP%u", m_WaypointID);
	
	// Convert to Feet and Inches and insert calculated data into Waypoint Form
	m_WaypointLocationFeetX = LocationX / 12;
	m_WaypointLocationFeetY = LocationY / 12;
	m_WaypointLocationInchesX = (int )LocationX % 12;
	m_WaypointLocationInchesY = (int )LocationY % 12;

	UpdateData(FALSE);	// Force data exchange from View data members to GUI

	return 0;
}

LRESULT CPathView::OnPathDisplayBulkItem(WPARAM Item, LPARAM lParam)
{
	// This function can display various Bulk items,
	// but is only used here for GPS data and possibly Compass data...
	IGNORE_UNUSED_PARAM (lParam);
	CString strText;

	if( ROBOT_RESPONSE_PIC_STATUS == Item )
	{
		// Handle Status Info from the Arduino

	}
	else if( ROBOT_RESPONSE_GPS_DATA  == Item )
	{

		if(	g_pGPSData->dwCommandCount < 3)	// wait until we get some good data from the GPS
		{
			strText.Format( _T("None") );
			SetDlgItemText( IDC_STAT_GPS, (LPCTSTR)strText );
			strText.Format( _T(" ") );	// Blink Off
			SetDlgItemText( IDC_STATIC_GPS_BLINK, (LPCTSTR)strText );
		}
		else
		{
			// GPS Data ready.

			if( TRUE == m_GPSConnectionIndicator )
			{		
				strText.Format( _T("*") );	// Blink On
			}
			else 
			{
				strText.Format( _T(" ") );	// Blink Off
			}
			SetDlgItemText( IDC_STATIC_GPS_BLINK2, (LPCTSTR)strText );
			m_GPSConnectionIndicator = !m_GPSConnectionIndicator;

			switch(g_pGPSData->btGSAFixMode)
			{
				case 1 : strText = _T("None"); break;
				case 2 : strText = _T("2D"); break;
				case 3 : strText = _T("3D"); break;
				default : strText = _T("Err"); break;
			}
			SetDlgItemText(IDC_STAT_GPS2, strText);

			// Latitude / Longitude / Altitude
			strText.Format("%f", g_pGPSData->dGGALatitude);
			SetDlgItemText(IDC_STAT_GPS_LAT2, strText);
			strText.Format("%f", g_pGPSData->dGGALongitude);
			SetDlgItemText(IDC_STAT_GPS_LONG2, strText);
			//strText.Format("%f", g_pGPSData->dGGAAltitude);
			//SetDlgItemText(IDC_STAT_GPS_ALT2, strText);

			SetDlgItemInt(IDC_STAT_GPS_SATS_IN_VIEW2, g_pGPSData->wGSVTotalNumSatsInView);
		
			/**************** GPS TODO! **********************************
			// GGA GPS Quality
			switch(g_pGPSData->btGGAGPSQuality)
			{
				case 0 : strText = _T("Fix not available"); break;
				case 1 : strText = _T("GPS sps mode"); break;
				case 2 : strText = _T("Differential GPS, SPS mode, fix valid"); break;
				case 3 : strText = _T("GPS PPS mode, fix valid"); break;
				default : strText = _T("Unknown"); break;
			}
			SetDlgItemText(IDC_GPS_QUAL2, strText);
			*************************************************************/
		}

	}
	else
	{
		// it's ok, we get lots of messages we just ignore
		// ROBOT_LOG( TRUE,  "ERROR!  Unmapped Message 0x%02lX sent to OnPathDisplayBulkItem!\n", Item )
	}
	return 0;
}


void CPathView::OnCalcualteNewGpsWaypoint() 
{
	if( (-1 == gGPSOriginLat) && (-1 == gGPSOriginLong) )
	{
		AfxMessageBox("GPS Anchor Point must be set first!");
		return;
	}
	
	POINT Pos = GPSDegreesToRWTenthInches( 
		g_pGPSData->dGGALatitude, g_pGPSData->dGGALongitude );

	// Convert to Feet and Inches and insert calculated data into Waypoint Form
	m_WaypointLocationFeetX = Pos.x / 12;
	m_WaypointLocationFeetY = Pos.y / 12;
	m_WaypointLocationInchesX = (int )Pos.x % 12;
	m_WaypointLocationInchesY = (int )Pos.y % 12;

	UpdateData(FALSE);	// Force data exchange from View data members to GUI
	
}

void CPathView::OnSetGpsAnchorPoint() 
{

	double DeltaLat, DeltaLong;
	//DeltaLat = 3.654321;

	if (UpdateData() != TRUE)
		return;

	if( (-1 != gGPSOriginLat) || (-1 != gGPSOriginLong) )
	{
		if( IDNO == AfxMessageBox(
			"Are you sure you want to RESET the Anchor Point?", MB_YESNO) )
		{
			return;
		}
	}


	// Map GPS to Real World Coordinates used by Path and Map
	// Calculate the Map Origin, based upon the GPS coordinate and scale factor

	// Calculation used:
	// Origin Lat = Anchor Lat - ( delta degrees)
	// where delta degrees = delta inches / K inches/deg
	// So, Origin Lat = Anchor Lat - ( DeltaIn / GPS_TENTH_INCHES_PER_DEGREE_LAT)

	DeltaLat = (double)(m_GPSAnchorPointX*12) / GPS_TENTH_INCHES_PER_DEGREE_LAT;
	gGPSOriginLat = g_pGPSData->dGGALatitude - DeltaLat;
		
	DeltaLong = (double)(m_GPSAnchorPointY*12) / GPS_TENTH_INCHES_PER_DEGREE_LAT;
	gGPSOriginLong = g_pGPSData->dGGALongitude - DeltaLong;

	CString strText;
	strText.Format("%lf", gGPSOriginLat);
	SetDlgItemText( IDC_PATH_ORIGIN_LAT, strText); 
	strText.Format("%lf", gGPSOriginLong);
	SetDlgItemText( IDC_PATH_ORIGIN_LONG, strText); 


	
}


void CPathView::OnRecalcSegments() 
{

	// Recalculate the angle and distance for each segment,
	// based upon information for each Waypoint
	// Allows modifying path by changing location of a waypoint


	POSITION SegPos, WPPos;
	CWaypointStruct* pWaypointStruct;
	CWaypointStructList& WaypointList = GetDocument()->m_WaypointList;

//	double X, Y;	// Delta from last Waypoint
//	int MapX, MapY;	// Absolute coordinates, allows for negative numbers
	BOOL FromWaypointFound, ToWaypointFound;
	POINT Start, End;

	UpdateData(TRUE);	// Force data exchange from GUI to View data members

	// For each segment, calculate and update from waypoint data
	CSegmentStructList& SegmentList = GetDocument()->m_SegmentList;
	SegPos = SegmentList.GetHeadPosition();
	while (SegPos != NULL)
	{
		CSegmentStruct* pSegmentStruct = SegmentList.GetNext(SegPos);

		// Got the Segment, now get Waypoint Data
		WPPos = WaypointList.GetHeadPosition();
		FromWaypointFound = FALSE;
		ToWaypointFound = FALSE;
		while (WPPos != NULL)
		{
			pWaypointStruct = WaypointList.GetNext(WPPos);

			if( pWaypointStruct->m_WaypointID == pSegmentStruct->m_SegmentFromWaypointID )
			{
				FromWaypointFound = TRUE;
				Start.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
				Start.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
			}
			if( pWaypointStruct->m_WaypointID == pSegmentStruct->m_SegmentToWaypointID )
			{
				ToWaypointFound = TRUE;
				End.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
				End.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
			}
			if( FromWaypointFound && ToWaypointFound )
			{
				break;
			}
		}
		if( !FromWaypointFound )
		{
			CString ErrorStr;
			ErrorStr.Format("Segment %s From Waypoint ID is not valid!\nPlease enter valid ID", pSegmentStruct->m_SegmentName );
			AfxMessageBox( ErrorStr );
			return;
		}
		if( !ToWaypointFound )
		{
			CString ErrorStr;
			ErrorStr.Format("Segment %s To Waypoint ID is not valid!\nPlease enter valid ID", pSegmentStruct->m_SegmentName );
			AfxMessageBox( ErrorStr );
		}

		// Recalculate and update Segment data
		int  Distance = CalculateDistance(Start, End);
		pSegmentStruct->m_SegmentDistanceFeet = Distance / 12;
		pSegmentStruct->m_SegmentDistanceInches = (int )(Distance % 12);

		pSegmentStruct->m_SegmentDirection = CalculateAngle(Start, End);


	}

	UpdateData(FALSE);	// Force data exchange from View data members to GUI

	// Remove all of the strings from the listbox.
	m_SegmentListBox.ResetContent();

	// And repopulate it
	m_SegmentListBox.ResetContent();
	POSITION pos = SegmentList.GetHeadPosition();
	while (pos != NULL)
	{
		CSegmentStruct* pSegmentStruct = SegmentList.GetNext(pos);
		AddSegmentStructToListBox(pSegmentStruct);
	}


	GetDocument()->SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit

}
