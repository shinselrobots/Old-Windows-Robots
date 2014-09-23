#ifndef __LASER_DISPLAY_WND_H__
#define __LASER_DISPLAY_WND_H__
// LaserDisplayWnd.h
// Display range data from Hokuyo URG-04LX-UG01 Laser Scanner

#include "RobotConfig.h"

/////////////////////////////////////////////////////////////////////////////
// CLaserDisplayWnd window


class CLaserDisplayWnd : public CWnd
{
// Construction
public:
	CLaserDisplayWnd();

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CLaserDisplayWnd)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CLaserDisplayWnd();

	void SetData();
	void SetWindowZoom( int  nZoom );
	void GetMinMaxInfo( double *pMin, double *pMax );

	// Generated message map functions
protected:
	//{{AFX_MSG(CLaserDisplayWnd)
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

	CFont m_scaleFont, m_valueFont;
	CPen m_solidPen;
	CPen m_graphPen;
	CPen m_scalePen;
	CPen m_XYPen;
	CPen m_KinectXYPen;
	CPen m_ZoneOutlinePen;
	CBrush m_ZoneOutlineBrush;
	CBrush m_ZoneOutlineBrushArm;
	CPen m_ZonePen;
	CPen m_DoorwayPen;
	int m_dMinScale, m_dMaxScale;
	//COLORREF m_c1, m_c2, m_c3, m_c4, m_c5, m_c6;
	double m_min, m_max;
	int m_nSample;	// number of Laser sample to display
	int  m_WindowZoom;
};

/////////////////////////////////////////////////////////////////////////////
#endif
