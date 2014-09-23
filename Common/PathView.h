// PathView.h : User Interface of the CPathDoc class
// Data store for Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_PATHVIEW_H__AECC39A8_5274_418E_9152_F87AFA351A81__INCLUDED_)
#define AFX_PATHVIEW_H__AECC39A8_5274_418E_9152_F87AFA351A81__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RobotConfig.h"

/////////////////////////////////////////////////////////////////////////////
/*  This is the viewer/editor for the Map Path data.

A PATH consists of a number of SEGMENTS, each terminated at a WAYPOINT
SegmentList is a list of Waypoints(plus other data), in sequence to a given destination
(may introduce idea of several Paths, search them for least number of waypoints
from current location.  Assume 1 PATH for now per saved file!)
each Segment has:

	Segment Name
	NextWaypoint ID
	 Navigation Behavior:
		Compass+GPS (Compass, distance counter, and GPS)
		Follow Wall
		Follow Curb
		Follow Dropoff
		Follow Hall
		Go Through Narrow Opening (Doorway)
	Speed
	Optional Parameters:
		Distance from object
		Right/Left
		Largest Gap Size (like driveways) (or expected size of door?)

each Waypoint has:
	Waypoint ID
	Waypoint Name
	X,Y position
	Landmark array[3 items max]

each Landmark has:
	Landmark Type:
		Left Edge
		Right Edge
		Point
		Wall
		DropOff
		Slope (for hills)?
	Range
	Direction (absolute from North)
	Height/Slope amount? (Low, High for Curbs?)

*/

/////////////////////////////////////////////////////////////////////////////
// CPathView form view

#ifndef __AFXEXT_H__
#include <afxext.h>
#endif

class CPathView : public CFormView
{
protected:
	CPathView();           // protected constructor used by dynamic creation
	DECLARE_DYNCREATE(CPathView)

// Overrides		
	CPathDoc* GetDocument();

// Form Data
public:
	//{{AFX_DATA(CPathView)
	enum { IDD = IDD_PATH_FORM };
	CListBox	m_SegmentListBox;
	CString	m_SegmentName;
	int 	m_SegmentFromWaypointID;
	int 	m_SegmentToWaypointID;
	int 	m_SegmentDirection;
	int 	m_SegmentDistanceFeet;
	int 	m_SegmentDistanceInches;
	CString	m_SegmentBehavior;
	int 	m_SegmentFollowDistance;
	CString	m_SegmentSpeed;
	int		m_SegmentFollowLeftRight;
	CListBox	m_WaypointListBox;
	CString	m_WaypointName;
	int 	m_WaypointID;
	int 	m_WaypointLocationFeetX;
	int 	m_WaypointLocationInchesX;
	int 	m_WaypointLocationFeetY;
	int 	m_WaypointLocationInchesY;
	CString	m_WaypointLandmarkType1;
	CString	m_WaypointLandmarkType2;
	CString	m_WaypointLandmarkType3;
	int 	m_WaypointLandmarkDirection1;
	int 	m_WaypointLandmarkDirection2;
	int 	m_WaypointLandmarkDirection3;
	int 	m_WaypointLandmarkHeight1;
	int 	m_WaypointLandmarkHeight2;
	int 	m_WaypointLandmarkHeight3;
	int 	m_WaypointLandmarkRange1;
	int 	m_WaypointLandmarkRange2;
	int 	m_WaypointLandmarkRange3;
	int 	m_GPSAnchorPointX;
	int 	m_GPSAnchorPointY;
	int 	m_SegmentAvoidDistance;
	int		m_CompassCorrection;
	//}}AFX_DATA

// Implementation
protected:
	CSegmentStruct* FindSegmentStruct(int& nSel, POSITION& pos);
	void AddSegmentStructToListBox(CSegmentStruct* pSegmentStruct, int nSel = -1);
	void CopyToSegmentStruct( CSegmentStruct* pSegmentStruct ); 

	CWaypointStruct* FindWaypointStruct(int& nSel, POSITION& pos);
	void AddWaypointStructToListBox(CWaypointStruct* pWaypointStruct, int nSel = -1);
	void CopyToWaypointStruct( CWaypointStruct* pWaypointStruct ); 
	void CopyWaypointStruct( CWaypointStruct* pToWaypointStruct, CWaypointStruct* pFromWaypointStruct );

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPathView)
	public:
	virtual void OnInitialUpdate();
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual void OnPrint(CDC* pDC, CPrintInfo* pInfo);
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	//}}AFX_VIRTUAL

// Implementation
public:
	BOOL m_GPSConnectionIndicator;
//	void PrintPageHeader(CDC* pDC, CPrintInfo* pInfo, CString& strHeader);
//	void PrintTitlePage(CDC* pDC, CPrintInfo* pInfo);

protected:
	virtual ~CPathView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

	// Generated message map functions
	//{{AFX_MSG(CPathView)
	afx_msg void OnSelchangeSegmentListBox();
	afx_msg void OnSegmentAdd();
	afx_msg void OnSegmentInsert();
	afx_msg void OnSegmentRemove();
	afx_msg void OnSegmentUpdate();
	afx_msg void OnSegmentRemoveAll();
	afx_msg void OnSelchangeWaypointListBox();
	afx_msg void OnWaypointAdd();
	afx_msg void OnWaypointInsert();
	afx_msg void OnWaypointUpdate();
	afx_msg void OnWaypointMoveUp();
	afx_msg void OnWaypointMoveDown();
	afx_msg void OnWaypointRemove();
	afx_msg void OnWaypointRemoveAll();
	afx_msg void OnSegmentMoveUp();
	afx_msg void OnSegmentMoveDown();
	afx_msg void OnCalcualteNewWaypoint();
	afx_msg void OnSelPathViewBtn();
	afx_msg void OnUpdateSelPathViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelCmdViewBtn();
	afx_msg void OnSelMapViewBtn();
	afx_msg void OnCalcualteNewGpsWaypoint();
	afx_msg void OnSetGpsAnchorPoint();
	afx_msg void OnRecalcualteAllWaypoints();
	afx_msg void OnSelSetupViewBtn();
	afx_msg void OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelMapViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnRecalcSegments();
	//}}AFX_MSG
	afx_msg LRESULT OnPathDisplayBulkItem(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnAddWaypointLocation(WPARAM LocationX, LPARAM LocationY);
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in PathView.cpp
inline CPathDoc* CPathView::GetDocument()
   { return (CPathDoc*)m_pDocument; }
#endif
/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PATHVIEW_H__AECC39A8_5274_418E_9152_F87AFA351A81__INCLUDED_)
