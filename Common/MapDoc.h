// MapDoc.h : interface of the CMapDoc class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_MAPDOC_H__DF7B087E_CEC6_4BCA_A190_B4887449BF57__INCLUDED_)
#define AFX_MAPDOC_H__DF7B087E_CEC6_4BCA_A190_B4887449BF57__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RobotConfig.h"
#include "Module.h"

#define MAP_PAGE_WIDTH			4000		// Real World Tenth Inches;  8 printed inches
#define MAP_PAGE_HEIGHT			4000
#define MAP_BOUNDRY_SIZE		 600		// Real World Tenth Inches (5 feet); .6 Printed inches

#define UPDATE_GPS_POINT		1L
#define UPDATE_TICK_POINT		2L
//#define UPDATE_GPS_POINT		3L

// Modes for saving and restoring map data
#define SERIALIZE_MODE_NORMAL					0
#define SERIALIZE_MODE_EXPORT_TO_TEXT_FILE		1
#define SERIALIZE_MODE_IMPORT_FROM_TEXT_FILE	2


/////////////////////////////////////////////////////////////////////////////
// class CStroke
// A stroke is a series of connected points in the Map view
class CStroke : public CObject
{
public:
	CStroke( int  nPenWidth );

protected:
	CStroke( );
	DECLARE_SERIAL( CStroke )

// Attributes


protected:
	CRect	m_rectBounding;	// smallest rect that surrounds all of the points in the stroke
	int 	m_nPenWidth;	// One width applies to entire stroke
//	UNIT	m_StrokeType;	// Type of object:  Wall, etc.

public:
	CArray<CPoint, CPoint>   m_pointArray;  // Series of connected points
	CRect&	GetBoundingRect() { return m_rectBounding; }	// Inline
	int 	GetPenWidth() { return m_nPenWidth; }
//	int 	GetStrokeType() { return m_StrokeType; }

// Operations
	public:
	BOOL DrawStroke( CDC* pDC );


	void FinishStroke();
	virtual void Serialize( CArchive& ar );
};



/////////////////////////////////////////////////////////////////////////////
class CMapDoc : public CDocument
{
protected: // create from serialization only
	CMapDoc();
	DECLARE_DYNCREATE(CMapDoc)

// Attributes
public:
	CTypedPtrList<CObList, CStroke*> m_strokeList;	// Line segments that are not visible to the Robot
	CTypedPtrList<CObList, CStroke*> m_DoorList;
	CTypedPtrList<CObList, CStroke*> m_LowObstacleList;	// Low objects that the robot can see over
	CTypedPtrList<CObList, CStroke*> m_ObstacleList;	// High objects that the robot can not see over
	CTypedPtrList<CObList, CStroke*> m_WallList;		// Walls
	CTypedPtrList<CObList, CStroke*> m_CliffList;		// Drop offs, cliffs, stairs, etc.


protected:
	BOOL		m_bThickPen;		// Thick currently selected or not 
	int 		m_nThinWidth;		// Current definition of thin
	int 		m_nThickWidth;		// Current definition of thick 
	int 		m_DrawMode;			// Drawing a User, Wall, or Object stroke 
//	CGridMap*	m_GridMap;			// GridMap object

	CRobotTrailStructList	m_RobotTrailList;
	CGPSPointStructList		m_GPSPointList;
	CMapSensorStructList	m_MapSensorList;



// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMapDoc)
	public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
	virtual BOOL OnOpenDocument(LPCTSTR lpszPathName);
	virtual void DeleteContents();
	protected:
	virtual BOOL SaveModified();
	//}}AFX_VIRTUAL

// Implementation
public:
	BOOL		m_CustomFileOpen;
	CStroke*	NewStroke( int  DrawMode );
	CStroke*	NewWallStroke();
	CStroke*	NewObstacleStroke();
	virtual		~CMapDoc();
	CPen*		GetCurrentPen() { return &m_PenCurrent; }	// Inline
	int 		GetStrokeType() { return m_StrokeType; }	// Inline
	void		AddRobotTrailPoint( POINT RobotPos );
	void		AddGPSPoint( POINT GPSPos, BYTE FixMode );	// Position and Quality of fix
	void		AddSensorData( POINT RWPos, int nHeading );	// Position and Direction
	void		SetDrawMode( int  NewStrokeType );		// Select User, Wall, or Object Strokes
	int 		GetDrawMode();
	int 		GetGridmapObjectType( int  DrawMode );	// Return object type associated with selected drawing mode 
	void		ExportToTextFile( char* FileName );
	void		ImportFromTextFile( char* FileName );
	CRect		GetMapBoundry();
	void		RotateMapPoints( CPoint MapCenter, int RotationAmount );
	void		ScaleMapPoints( double ScaleAmount ); // Utility not used in normal operation
	void		CreateGridMapFromVectorData(); 


	CRobotTrailStructList* GetRobotTrailList() { return &m_RobotTrailList; }	// Inline
	CGPSPointStructList* GetGPSPointList() { return &m_GPSPointList; }	// Inline
	CMapSensorStructList* GetMapSensorList() { return &m_MapSensorList; }	// Inline


#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	void ReplacePen();
	void InitDocument();
	CPen m_PenCurrent;
	int  m_nPenWidth;
	int  m_StrokeType;
	//{{AFX_MSG(CMapDoc)
	afx_msg void OnEditClearAll();
	afx_msg void OnPenThickOrThin();
	afx_msg void OnUpdateEditClearAll(CCmdUI* pCmdUI);
	afx_msg void OnUpdatePenThickOrThin(CCmdUI* pCmdUI);
	afx_msg void OnPenPenwidths();
	afx_msg void OnEditUndo();
	afx_msg void OnRotateMap();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_MAPDOC_H__DF7B087E_CEC6_4BCA_A190_B4887449BF57__INCLUDED_)

