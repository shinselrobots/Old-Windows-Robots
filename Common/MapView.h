// MapView.h : interface of the CMapView class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_MAPVIEW_H__A970A302_36FC_4F73_82A9_6C70AF03298E__INCLUDED_)
#define AFX_MAPVIEW_H__A970A302_36FC_4F73_82A9_6C70AF03298E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RobotConfig.h"

class CMapView : public CScrollView
{
protected: // create from serialization only
	CMapView();
	DECLARE_DYNCREATE(CMapView)

// Attributes
public:
	CMapDoc* GetDocument();
protected:
	CStroke*	m_pStrokeCur;	// The stroke in progress
	CPoint		m_ptPrev;	// The last mouse pt in the stroke in progress
	CPoint		m_ptCurrent; // current mouse location when drawing long stroke
	BOOL		m_DoingLongStroke;
	BOOL		m_ShowRobotTrails;
	BOOL		m_ShowGridMapSquares;
	BOOL		m_ShowSensorReadings;
	CRect		m_MapBoundry;		// Boundry of the map
	BOOL		m_RealWorldInitialized;
	BOOL		m_GPSEnabled;
	double		m_MC_MapZoom;
//	int			m_LastKey;
//	int			m_SpeedSetByKeyboard;
//	int			m_LastSpeedSetByKeyboard;

//	BOOL		 m_LocalUser;		
	int			 m_nCurrentSpeed;	// keep track of speed and turn for commands
	int			 m_nCurrentTurn;



// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMapView)
	public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	virtual void OnInitialUpdate();
	protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnUpdate(CView* pSender, LPARAM lHint, CObject* pHint);
	virtual void OnPrint(CDC* pDC, CPrintInfo* pInfo);
	//}}AFX_VIRTUAL


// Implementation
public:
	BYTE	m_LastFixMode;
	POINT	m_LastGPSPos;
	POINT	m_LastRobotPos;
	POINT	m_RW_TargetPoint;	// Place user targeted on the map for robot to move to
//	WORD	m_nCompassHeading;
//	WORD	m_nCompassError;
	POINT	m_RW_MapBottomLeft;
	POINT	m_RW_MapTopRight;

	void	PrintPageHeader( CDC* pDC, CPrintInfo* pInfo, CString& strHeader );
	void	PrintTitlePage( CDC* pDC, CPrintInfo* pInfo );
	void	PaintSensor( CDC* pDC, POINT RWRobotPos, int  RobotHeading, double SensorOffsetDegrees, int  SensorFOVDegrees, double ObjectDistance );
	POINT	TranslateRWtoMC( POINT RWFrom );
	POINT	TranslateRWtoMC(  FPOINT RWFrom );
	POINT	TranslateMCtoRW( POINT MCPoint );
//	CSize	GetRWMapSize() { return m_RWMapSize; }			// Inline
//	CSize	GetRWMapOrigin() { return m_RWMapOrigin; }			// Inline
	bool	UpdateBoundry( POINT RW );
	bool	UpdateBoundry( FPOINT RW );

	bool	AddPage( int Direction );
	CSize	GetMCZoomedMapSize();
	void	SendDriveCommand( int Speed, int Turn );

	virtual ~CMapView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CMapView)
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnSelMapViewBtn();
	afx_msg void OnUpdateSelMapViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelPathViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnSelCmdViewBtn();
	afx_msg void OnSelPathViewBtn();
	afx_msg void OnSelSetupViewBtn();
	afx_msg void OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI);
	afx_msg void OnTypeObstacle();
	afx_msg void OnTypeUserdraw();
	afx_msg void OnTypeWallobject();
	afx_msg void OnUpdateTypeObstacle(CCmdUI* pCmdUI);
	afx_msg void OnUpdateTypeUserdraw(CCmdUI* pCmdUI);
	afx_msg void OnUpdateTypeWallobject(CCmdUI* pCmdUI);
	afx_msg void OnNavigationMode();
	afx_msg void OnUpdateNavigationMode(CCmdUI* pCmdUI);
	afx_msg void OnWaypointLocation();
	afx_msg void OnUpdateWaypointLocation(CCmdUI* pCmdUI);
	afx_msg void OnAddPageAbove();
	afx_msg void OnUpdateAddPageAbove(CCmdUI* pCmdUI);
	afx_msg void OnAddPageBelow();
	afx_msg void OnUpdateAddPageBelow(CCmdUI* pCmdUI);
	afx_msg void OnAddPageLeft();
	afx_msg void OnUpdateAddPageLeft(CCmdUI* pCmdUI);
	afx_msg void OnAddPageRight();
	afx_msg void OnUpdateAddPageRight(CCmdUI* pCmdUI);
	afx_msg void OnShowGridmapSquares();
	afx_msg void OnShowRobotTrails();
	afx_msg void OnShowSensorReadings();
	afx_msg void OnUpdateShowRobotTrails(CCmdUI* pCmdUI);
	afx_msg void OnUpdateShowGridmapSquares(CCmdUI* pCmdUI);
	afx_msg void OnUpdateShowSensorReadings(CCmdUI* pCmdUI);
	afx_msg void OnGpsEnable();
	afx_msg void OnUpdateGpsEnable(CCmdUI* pCmdUI);
	afx_msg void OnZoomMap025();
	afx_msg void OnZoomMap050();
	afx_msg void OnZoomMap100();
	afx_msg void OnZoomMap150();
	afx_msg void OnZoomMap200();
	afx_msg void OnZoomMap400();
	afx_msg void OnExportToTextFile();
	afx_msg void OnImportFromTextFile();
	afx_msg void OnClearMouseTrail();
	afx_msg void OnClearSensorReadings();
	afx_msg void OnUpdateZoomMap025(CCmdUI* pCmdUI);
	afx_msg void OnUpdateZoomMap050(CCmdUI* pCmdUI);
	afx_msg void OnUpdateZoomMap100(CCmdUI* pCmdUI);
	afx_msg void OnUpdateZoomMap150(CCmdUI* pCmdUI);
	afx_msg void OnUpdateZoomMap200(CCmdUI* pCmdUI);
	afx_msg void OnUpdateZoomMap400(CCmdUI* pCmdUI);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnSetCurrentLocation();
	afx_msg void OnUpdateSetCurrentLocation(CCmdUI* pCmdUI);
	afx_msg void OnTypeCliffObject();
	afx_msg void OnUpdateTypeCliffObject(CCmdUI* pCmdUI);
	afx_msg void OnTypeDoorObject();
	afx_msg void OnUpdateTypeDoorObject(CCmdUI* pCmdUI);
	afx_msg void OnTypeLowObstacle();
	afx_msg void OnUpdateTypeLowObstacle(CCmdUI* pCmdUI);
	//}}AFX_MSG
	afx_msg LRESULT OnMapDisplayBulkItem(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnMapBulkItemToServer(WPARAM Item, LPARAM lParam);
	afx_msg LRESULT OnRobotUpdateView(WPARAM Item, LPARAM lParam);	
	afx_msg void OnUpdatePos(CCmdUI *pCmdUI);
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in MapView.cpp
inline CMapDoc* CMapView::GetDocument()
   { return (CMapDoc*)m_pDocument; }
#endif

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_MAPVIEW_H__A970A302_36FC_4F73_82A9_6C70AF03298E__INCLUDED_)
