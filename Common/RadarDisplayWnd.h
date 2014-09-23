#ifndef __RADAR_DISPLAY_WND_H__
#define __RADAR_DISPLAY_WND_H__
// RadarDisplayWnd.h : header file
//
#include "RobotConfig.h"

/////////////////////////////////////////////////////////////////////////////
// CRadarDisplayWnd window

#define MAX_DATAPOINTS 64

class CRadarDisplayWnd : public CWnd
{
// Construction
public:
	CRadarDisplayWnd();

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRadarDisplayWnd)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CRadarDisplayWnd();

	void SetData( int  SensorNumber, int  Length, int *  Buffer );
	void GetMinMaxInfo( double *pMin, double *pMax );
	CPoint CalculateEndPoint( double XCenter, double Degree, double Distance );

	// Generated message map functions
protected:
	//{{AFX_MSG(CRadarDisplayWnd)
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

	CFont m_scaleFont, m_valueFont;
	CPen m_solidPen;
	CPen m_scalePen;

	CPen m_FixedUSPen;
	CPen m_FixedIRPen;

	CPen m_ScanningUSPen;
	CPen m_ScanningIRPen;

	CPen m_ObjectCenterPen;
	CPen m_ObjectSizePen;


	int m_dMinScale, m_dMaxScale;
	COLORREF m_c1, m_c2, m_c3, m_c4, m_c5;
	double m_min, m_max;
	//double m_datapoints[MAX_SENSORS][MAX_DATAPOINTS];

	double m_FixedIrArray[FIXED_IR_SAMPLES];
	double m_FixedUltraSonicArray[FIXED_US_SAMPLES];
	double m_IrArray1[IR1_SAMPLES];
	double m_UltraSonicArray1[US1_SAMPLES];

	int m_nSample;	// number of Radar sample to display
};

/////////////////////////////////////////////////////////////////////////////
#endif
