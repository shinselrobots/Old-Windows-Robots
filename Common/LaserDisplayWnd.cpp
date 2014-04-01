// LaserDisplayWnd.cpp : implementation file for Laser Display class
// Display range data from Hokuyo URG-04LX-UG01 Laser Scanner

#include "stdafx.h"
#include "Globals.h"
#include "LaserDisplayWnd.h"
#include "osbmp.h"
#include "math.h"
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



/////////////////////////////////////////////////////////////////////////////
// CLaserDisplayWnd

#define Y_NEGATIVE_TENTH_INCHES						120.0	// 12 inches

#define COLOR_BLACK				RGB(  0,  0,  0)	// Black

#define COLOR_MED_RED			RGB(200,  0,  0)	// Red
#define COLOR_LIGHT_PINK		RGB(216, 104, 155)	// Light Pink

#define COLOR_MED_GREEN			RGB(  0,200,  0)	// Green
#define COLOR_LIGHT_GREEN		RGB(200,255,200)	// Light Green
#define COLOR_GREEN				RGB(  0,255,  0)	// Green
#define COLOR_GREEN2			RGB(  0,192,  0)	// Green
#define COLOR_GREEN3			RGB(  0,128,  0)	// Green
#define COLOR_GREEN4			RGB(  0, 64,  0)	// Green
#define COLOR_DARK_GREEN		RGB( 10,110, 10)	// Dark Green 
#define COLOR_DARK_GREEN2		RGB(  0, 50,  0)	// Dark Green

#define COLOR_DARK_BLUE			RGB( 20,29, 255)	// Dark Blue
#define COLOR_DARK_BLUE2		RGB( 10,10, 127)	// Very Dark Blue
#define COLOR_BLUE				RGB(  0,  0,255)	// Blue
#define COLOR_MED_BLUE			RGB(  0,  0,200)	// Blue
#define COLOR_LIGHT_BLUE		RGB(180,180,255)	// Light Blue
#define COLOR_LIGHT_BLUE1		RGB( 60, 60,255)	// Light Blue
#define COLOR_LIGHT_BLUE2		RGB(190, 210,255)	// Light Blue

#define COLOR_ORANGE			RGB(255,120,0)		// Orange
#define COLOR_ORANGE2			RGB(100,50,0)		// Dark Orange

#define COLOR_PURPLE			RGB(200,  0,250)	// Purple
#define COLOR_DARK_PURPLE		RGB(85,  0,113)		// Dark Purple
#define COLOR_MED_PURPLE		RGB(190,  105,230)	// Med Purple
#define COLOR_LIGHT_PURPLE		RGB(210,  200,225)	// Light Purple
#define COLOR_PLUMB				RGB(115,  7,125)	// Plumb

#define COLOR_GRAY				RGB(200,200,200)	// Gray
#define COLOR_LIGHT_GRAY		RGB(128,128,128)	// Light Gray
#define COLOR_DARK_GRAY			RGB( 20, 20, 20)	// Dark Gray
#define COLOR_WHITE				RGB(255,255,255)	// White



CLaserDisplayWnd::CLaserDisplayWnd()
{
	m_min = 0.0;
	m_max = 255.0;
	m_nSample = 0;

	m_dMinScale = 0;
	m_dMaxScale = 200;
}

CLaserDisplayWnd::~CLaserDisplayWnd()
{
}


BEGIN_MESSAGE_MAP(CLaserDisplayWnd, CWnd)
	//{{AFX_MSG_MAP(CLaserDisplayWnd)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CLaserDisplayWnd message handlers

int CLaserDisplayWnd::OnCreate(LPCREATESTRUCT lpCreateStruct) 
{
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	m_scaleFont.CreateFont( 12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, 0,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
		CLIP_DEFAULT_PRECIS, PROOF_QUALITY, 
		TMPF_TRUETYPE, "Verdana" );

	m_valueFont.CreateFont( 18, 0, 0, 0, FW_NORMAL, FALSE, FALSE, 0,
		DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
		CLIP_DEFAULT_PRECIS, PROOF_QUALITY, 
		TMPF_TRUETYPE, "Verdana Bold" );


	m_solidPen.CreatePen( PS_SOLID, 1, COLOR_GREEN );
	m_graphPen.CreatePen( PS_SOLID, 1, COLOR_LIGHT_BLUE1 );
	m_scalePen.CreatePen( PS_SOLID, 1, COLOR_GREEN3 );
	m_XYPen.CreatePen(    PS_SOLID, 2, COLOR_GREEN2 ); //COLOR_LIGHT_BLUE1
	m_KinectXYPen.CreatePen( PS_SOLID, 2, COLOR_WHITE );
	m_ZonePen.CreatePen(  PS_SOLID, 2, COLOR_ORANGE );
	m_ZoneOutlinePen.CreatePen(  PS_DOT, 1, COLOR_ORANGE );
	m_DoorwayPen.CreatePen( PS_SOLID, 4, COLOR_MED_RED );
	m_ZoneOutlineBrush.CreateSolidBrush( COLOR_LIGHT_BLUE1 );
	m_ZoneOutlineBrushArm.CreateSolidBrush( COLOR_ORANGE );

	m_WindowZoom = 1;

	return 0;
}

void CLaserDisplayWnd::SetData()
{

	// Update display
	CRect wndRect;
	GetClientRect( &wndRect );
	InvalidateRect( wndRect, FALSE );
	UpdateWindow();

}
void CLaserDisplayWnd::SetWindowZoom( int  nZoom )
{
	m_WindowZoom = nZoom;

	// Update display
	CRect wndRect;
	GetClientRect( &wndRect );
	InvalidateRect( wndRect, FALSE );
	UpdateWindow();

}

void CLaserDisplayWnd::OnPaint() 
{
	int i; 
	double j;
	CPaintDC paintdc(this); // device context for painting	
	COffscreenDC dc( &paintdc, NULL );
	
	CRect clientRect, graphRect, textRect, objRect, ZoneRect;
	CString csTemp;

	// Get the window size
	GetClientRect( &clientRect );

	// TODO - add SCALE/ZOOM box, and apply scaling here!
	// Data coordinates (prior to scaling for GUI)
//	double MaxValue = g_LaserScannerState.LargestValue * 1.5;	// add a little boarder

	double MaxValue = (double)LASER_RANGEFINDER_TENTH_INCHES_MAX / (double)m_WindowZoom;
	double XCenter = MaxValue; // LASER_RANGEFINDER_TENTH_INCHES_MAX;
	double XMax = MaxValue*2.0;// LASER_RANGEFINDER_TENTH_INCHES_MAX*2;	// for half circle
	double YMax = MaxValue*1.2;	// allow negative values, behind front of robot
	 
	// Calculations
	graphRect = clientRect;
	graphRect.left += 43;
	graphRect.right -= 5;
	graphRect.top += 15;
	graphRect.bottom -= 15;	//25

	int graphDeltaY = graphRect.bottom - graphRect.top;
	int graphDeltaX = graphRect.right - graphRect.left;

	double YScale = (double)(graphDeltaY) / (YMax + Y_NEGATIVE_TENTH_INCHES);
	double XScale = (double)(graphDeltaX) / XMax;	

	// Clear client rect
	dc.FillSolidRect( clientRect, PALETTERGB( 0, 0, 0 ) );

	// Scale text
	CFont *pOldFont = dc.SelectObject( &m_scaleFont );
	dc.SetBkColor( PALETTERGB( 0, 0, 0 ) );
	
	// Draw graph background
	dc.FillSolidRect( graphRect, COLOR_GREEN4 );


	// Draw Scale Grid
	dc.SelectObject( &m_scalePen );
	int XCenterScaled = graphRect.left + (int)(XScale*XCenter);
	// Draw CenterLine
	dc.MoveTo( XCenterScaled +1, graphRect.top );
	dc.LineTo( XCenterScaled +1, graphRect.bottom );
	dc.MoveTo( XCenterScaled -1, graphRect.top );
	dc.LineTo( XCenterScaled -1, graphRect.bottom );
	
	// Draw Vertical grid lines
	for( j = 0.0; j < XCenter; j+=120.0 )	// every foot
	{
		dc.MoveTo( XCenterScaled + (int)(XScale*j), graphRect.top );
		dc.LineTo( XCenterScaled + (int)(XScale*j), graphRect.bottom );
		dc.MoveTo( XCenterScaled - (int)(XScale*j), graphRect.top );
		dc.LineTo( XCenterScaled - (int)(XScale*j), graphRect.bottom );
	}

	// Draw Horizontal grid lines and text labels
	dc.SetTextColor( COLOR_GREEN2 );
	for( j = 0.0; j < (YMax + Y_NEGATIVE_TENTH_INCHES); j+=120.0 )	// every foot
	{
		csTemp.Format( "%2.1f", ((j-Y_NEGATIVE_TENTH_INCHES) / 120.0) );	// Feet
		textRect.SetRect( 
			0, 
			((graphRect.bottom) - (int)(YScale*j)) - 9, 
			graphRect.left - 5, 
			((graphRect.bottom) - (int)(YScale*j)) + 9 );
		dc.DrawText( csTemp, textRect, DT_RIGHT|DT_SINGLELINE );
		dc.MoveTo( graphRect.left,  (graphRect.bottom) - (int)(YScale*j) );
		dc.LineTo( graphRect.right, (graphRect.bottom) - (int)(YScale*j) );
	}
	dc.SelectObject( pOldFont );

	// Draw Robot Arms
	objRect.left =   graphRect.left +   (int)(XScale * (XCenter-(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES))) -1;		
	objRect.right =  graphRect.left +   (int)(XScale * (XCenter+(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES))) +1;
	objRect.top =    (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-30));	
	objRect.bottom = (graphRect.bottom) - (int)(YScale * 20);
	dc.FillSolidRect( objRect, COLOR_DARK_BLUE2 );

	// Draw "Robot" body
	objRect.left =   graphRect.left +   (int)(XScale * (XCenter-HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) - 1;		
	objRect.right =  graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) + 1;
	objRect.top =    (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));	
	objRect.bottom = graphRect.bottom;
	dc.FillSolidRect( objRect, COLOR_DARK_BLUE2 );


/*
	// color TEST
	objRect.left =   graphRect.left +   (int)(XScale * (XCenter-HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) - 1;		
	objRect.right =  graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) + 1;
	objRect.top =    (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES+38));	
	objRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES+36));
	dc.FillSolidRect( objRect, COLOR_LIGHT_GREEN );
*/

	////////////////////////////////////////////////////////////////////
	// Draw Sensor Zone readings
	////////////////////////////////////////////////////////////////////
	int ZoneY = 0;
	int ZoneX = 0;
	dc.SelectObject( &m_ZonePen );

	// RightFrontZone
	ZoneRect.left =   graphRect.left +   (int)(XScale * (XCenter)) +2;		
	ZoneRect.right =  ZoneRect.left +   (int)(XScale * (HALF_ROBOT_BODY_WIDTH_TENTH_INCHES));
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nRightFrontZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left + 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right - 0), ZoneY );
//	dc.MoveTo( (graphRect.left + (int)(XScale * (XCenter)) + 3), ZoneY );		
//	dc.LineTo( (graphRect.left + (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) - 2), ZoneY );

	// LeftFrontZone
	ZoneRect.right =   graphRect.left +   (int)(XScale * (XCenter)) -2;		
	ZoneRect.left =	   ZoneRect.right -   (int)(XScale * (HALF_ROBOT_BODY_WIDTH_TENTH_INCHES));
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nLeftFrontZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left - 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right + 0), ZoneY );
//	dc.MoveTo( (graphRect.left + (int)(XScale * (XCenter-HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) + 2), ZoneY );		
//	dc.LineTo( (graphRect.left + (int)(XScale * (XCenter)) - 3), ZoneY );

	// RightArmZone
	ZoneRect.left =    graphRect.left +   (int)(XScale * (XCenter + HALF_ROBOT_BODY_WIDTH_TENTH_INCHES));
	ZoneRect.right =   ZoneRect.left  +   (int)(XScale * (ROBOT_ARM_WIDTH_TENTH_INCHES));		
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nRightArmZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left + 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right - 0), ZoneY );
//	dc.MoveTo( (graphRect.left + (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) + 2), ZoneY );		
//	dc.LineTo( (graphRect.left + (int)(XScale * (XCenter+HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES)) - 1), ZoneY );


	// LeftArmZone
	ZoneRect.right =    graphRect.left +   (int)(XScale * (XCenter - HALF_ROBOT_BODY_WIDTH_TENTH_INCHES));
	ZoneRect.left =   ZoneRect.right  -   (int)(XScale * (ROBOT_ARM_WIDTH_TENTH_INCHES));		
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nLeftArmZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left - 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right + 0), ZoneY );
//	dc.MoveTo( (graphRect.left + (int)(XScale * (XCenter-HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES)) + 1), ZoneY );		
//	dc.LineTo( (graphRect.left + (int)(XScale * (XCenter-HALF_ROBOT_BODY_WIDTH_TENTH_INCHES)) - 2), ZoneY );


	// RightFrontSideZone
	ZoneRect.left =    graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES));		
	ZoneRect.right =   ZoneRect.left +   (int)(XScale * (FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES));
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nRightFrontSideZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left + 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right - 0), ZoneY );


	// LeftFrontSideZone
	ZoneRect.right =  graphRect.left +   (int)(XScale * (XCenter - (HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES)));		
	ZoneRect.left =   ZoneRect.right -   (int)(XScale * (FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES));
	ZoneRect.top =    (graphRect.top);
	ZoneRect.bottom = (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneY = graphRect.bottom - (int)( YScale * (g_pNavSensorSummary->nLeftFrontSideZone + Y_NEGATIVE_TENTH_INCHES) );
	dc.MoveTo( (ZoneRect.left - 0),  ZoneY );		
	dc.LineTo( (ZoneRect.right + 0), ZoneY );


	// nRightSideZone
	ZoneRect.left =    graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES));		
	ZoneRect.right =   ZoneRect.left +   (int)(XScale * (FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)); // TODO!!!
	ZoneRect.top =    (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));	
	ZoneRect.bottom = graphRect.bottom;
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneX = graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+g_pNavSensorSummary->nRightSideZone)) - 1;
	dc.MoveTo( ZoneX, (graphRect.bottom - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-10))) );		
	dc.LineTo( ZoneX, (graphRect.bottom - (int)(YScale * 10)) );

	// nLeftSideZone
	ZoneRect.right =   graphRect.left +   (int)(XScale * (XCenter - (HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+ROBOT_ARM_WIDTH_TENTH_INCHES)));		
	ZoneRect.left =    ZoneRect.right -   (int)(XScale * (FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES));
	ZoneRect.top =    (graphRect.bottom) - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-2));	
	ZoneRect.bottom = graphRect.bottom;
	dc.FrameRect( ZoneRect, &m_ZoneOutlineBrush );
	// Draw current Value as a bar
	ZoneX = graphRect.left +   (int)(XScale * (XCenter-(HALF_ROBOT_BODY_WIDTH_TENTH_INCHES+g_pNavSensorSummary->nLeftSideZone))) - 1;
	dc.MoveTo( ZoneX, (graphRect.bottom - (int)(YScale * (Y_NEGATIVE_TENTH_INCHES-10))) );
	dc.LineTo( ZoneX, (graphRect.bottom - (int)(YScale * 10)) );


/*

	// bRightCliff
//	if( g_pNavSensorSummary->bRightCliff )
	{
		objRect.left =   graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES)) - 1;		
		objRect.right =  graphRect.left +   (int)(XScale * (XCenter+HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES + 30)) - 1;
		objRect.top =    graphRect.bottom - (int)(YScale * (60));	
		objRect.bottom = graphRect.bottom - (int)(YScale * (30));
		dc.FillSolidRect( clientRect, PALETTERGB( 255, 90, 90 ) );	}

	// bLeftCliff
//	if( g_pNavSensorSummary->bLeftCliff )
	{
		objRect.left =   graphRect.left +   (int)(XScale * ((XCenter-HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES) - 30)) - 1;		
		objRect.right =  graphRect.left +   (int)(XScale * (XCenter-HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES)) - 1;
		objRect.top =    graphRect.bottom - (int)(YScale * (60));	
		objRect.bottom = graphRect.bottom - (int)(YScale * (30));
		dc.FillSolidRect( clientRect, PALETTERGB( 255, 90, 90 ) );
	}

*/

	////////////////////////////////////////////////////////////////////
	// LASER
	////////////////////////////////////////////////////////////////////
	CPen *pOldPen = dc.SelectObject( &m_graphPen );
	int  NumberOfLaserSamples = 0;

	NumberOfLaserSamples = g_pLaserScannerData->NumberOfSamples;
/**
	// Show Phantom lines, connecting all known objects
	if( 0 != NumberOfLaserSamples )
	{
		double XData, YData;

		for( i = 0; i < (int)NumberOfLaserSamples; i++ ) 
		{
			XData = g_pLaserScannerData->ScanPoints[i].X ; // tenth inches
			YData = g_pLaserScannerData->ScanPoints[i].Y ;

			if( (abs(XData) >= MaxValue) || (YData >= MaxValue) || (YData  < (-Y_NEGATIVE_TENTH_INCHES)) )
			{
				continue;// Off the chart (zoomed in too far), don't show value	
			}
			XData += XCenter;

			if( 0 == i )
			{
				dc.MoveTo( graphRect.left + (int)(XScale * XData), (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
			}
			else
			{
				dc.LineTo( graphRect.left + (int)(XScale * XData), (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
			}
		}

	} // if( 0 != NumberOfLaserSamples )
**/

	////////////////////////////////////////////////////////////////////
	// Draw Laser X,Y values
	////////////////////////////////////////////////////////////////////
	dc.SelectObject( &m_XYPen );

	if( 0 != NumberOfLaserSamples )
	{
		double XData, YData;
		for( i = 0; i < (int)NumberOfLaserSamples; i++ ) 
		{
			XData = g_pLaserScannerData->ScanPoints[i].X ; // tenth inches
			YData = g_pLaserScannerData->ScanPoints[i].Y ;

			if( (abs(XData) >= MaxValue) || (YData >= MaxValue) || (YData  < (-Y_NEGATIVE_TENTH_INCHES)) )
			{
				continue;// Off the chart (zoomed in too far), don't show value	
			}

			XData += XCenter;
			dc.MoveTo( graphRect.left + (int)(XScale * XData) - 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
			dc.LineTo( graphRect.left + (int)(XScale * XData) + 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
		}

	} // if( 0 != NumberOfLaserSamples )


	////////////////////////////////////////////////////////////////////
	// Draw KINECT 2D X,Y values
	////////////////////////////////////////////////////////////////////
	dc.SelectObject( &m_KinectXYPen );

	if( NULL != g_KinectPointCloud )
	{
		double XData, YData;
	
		for( i = 0; i < g_KinectPointCloud->FrameSizeX; i++ ) 
		{
			XData = g_KinectPointCloud->MapPoints2D[i].X; // tenth inches
			YData = g_KinectPointCloud->MapPoints2D[i].Y ;

			if( (abs(XData) >= MaxValue) || (YData >= MaxValue) || (YData  < (-Y_NEGATIVE_TENTH_INCHES)) )
			{
				continue;// Off the chart (zoomed in too far), don't show value	
			}

			XData += XCenter;
			dc.MoveTo( graphRect.left + (int)(XScale * XData) - 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
			dc.LineTo( graphRect.left + (int)(XScale * XData) + 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );

		}

	} 



	////////////////////////////////////////////////////////////////////
	// Draw Doorways
	////////////////////////////////////////////////////////////////////

//	const int MAX_DOORWAYS = 5;


	dc.SelectObject( &m_DoorwayPen );

/***
	if( 0 != NumberOfLaserSamples )
	{
		//////////////////// LASER //////////////////////
		// Laser Scanner installed and working, so use it!

		// for laser finder, highest granular increment should be: .36 degrees?360°/1024?
		// ASSUMES 180 degrees!  Need to change to 240 degree?
		int XData, YData;
		POINT2D_T RightEdge = {0,0};
		POINT2D_T LeftEdge = {0,0};
		int LastX = 0;
		int LastY = 0;
		int TargetClearDistance = (__max(g_pNavSensorSummary->nRightFrontSideZone, g_pNavSensorSummary->nLeftFrontSideZone) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) ; // tenth inches
		
		for( i = 0; i < (int)NumberOfLaserSamples; i++ ) 
		{
			XData = g_pLaserScannerData->ScanPoints[i].X ; // tenth inches
			YData = g_pLaserScannerData->ScanPoints[i].Y ;

			if( (XData <= (HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES))  &&
				(XData >= -(HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)) )
			{
				// point is inside RightFrontSideZone to LeftFrontSideZone
				if( 0 == RightEdge.X )
				{ 
					// looking for right edge
					if( YData >= TargetClearDistance )
					{
						// Found inside of door edge!
						RightEdge.X = LastX;	// Use the last valid data
						RightEdge.Y = LastY;
					}
				}
				else
				{
					// Right edge found, looking for left edge
					if( YData < TargetClearDistance )
					{
						if( (RightEdge.X - XData) >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES )
						{
							// Found it!
							LeftEdge.X = XData;
							LeftEdge.Y = YData;

							// Draw where we think the door is!

							// Draw Left Door Frame
							dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); 
							dc.LineTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );

							// Draw line connecting the two door frames
							dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) -  (int)(YScale * (LeftEdge.Y  + Y_NEGATIVE_TENTH_INCHES))  ); 
							dc.LineTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) );	// Connect to Right Door Frame

							// Draw Right Door Frame
							dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); // Draw Right Door Frame
							dc.LineTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );


							break;
						}
						else
						{
							// too narrow, restart search
							RightEdge.X = 0;
						}
					}
					else
					{
		//				dc.MoveTo( graphRect.left + (int)(XScale * (XCenter+XData)) - 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
		//				dc.LineTo( graphRect.left + (int)(XScale * (XCenter+XData)) + 1, (graphRect.bottom) - (int)(YScale * (YData+Y_NEGATIVE_TENTH_INCHES)) );
					}
				}
				LastX = XData;
				LastY = YData;
			}
		}
	} // if( 0 != NumberOfLaserSamples )
	else
	{

****/

		//////////////////// KINECT //////////////////////
		// No Laser, draw Kinect door detection

		// DEBUG ONLY!
		//ROBOT_LOG( TRUE, "\n\nKINECT DOOR SCAN: ========================================================== \n\n" )

		//int NumberOfKinectSamples = g_KinectPointCloud->FrameSizeX;
		//int nMaxDoorways = MAX_DOORWAYS;
		//DOORWAY_T  DoorWaysFound[MAX_DOORWAYS];	// array of potential multiple doors
		//POINT2D_T *pPointArray = g_KinectPointCloud->WallPoints;

		//int nDoorwaysFound = FindDoors( NumberOfKinectSamples, nMaxDoorways, pPointArray, DoorWaysFound );


		for( int i=0; i < g_pNavSensorSummary->nDoorWaysFound; i++ )
		{
			// Draw where we think the door is
			ROBOT_LOG( TRUE, "Doorway %d:  Center = %d, Width = %d", 
				i, g_pNavSensorSummary->DoorWaysFound[i].CenterX, g_pNavSensorSummary->DoorWaysFound[i].Width )

			// Draw Left Door Frame
			dc.MoveTo( graphRect.left + (int)(XScale * (XCenter +  g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.X )) + 1, 
					(graphRect.bottom) - (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); 

			dc.LineTo( graphRect.left + (int)(XScale * (XCenter + g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.X)) + 1,
				(graphRect.bottom) - (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );

			// Draw line connecting the two door frames
			dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.X)) + 1, 
				(graphRect.bottom) -  (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].LeftEdge.Y  + Y_NEGATIVE_TENTH_INCHES))  );

			dc.LineTo( graphRect.left + (int)(XScale * (XCenter + g_pNavSensorSummary->DoorWaysFound[i].RightEdge.X)) + 1, 
				(graphRect.bottom) - (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) );	// Connect to Right Door Frame

			// Draw Right Door Frame
			dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + g_pNavSensorSummary->DoorWaysFound[i].RightEdge.X)) + 1, 
				(graphRect.bottom) - (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); // Draw Right Door Frame

			dc.LineTo( graphRect.left + (int)(XScale * (XCenter + g_pNavSensorSummary->DoorWaysFound[i].RightEdge.X)) + 1, 
				(graphRect.bottom) - (int)(YScale * (g_pNavSensorSummary->DoorWaysFound[i].RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );
		}

/************
		if( NumberOfKinectSamples > 0 )
		{
			int X, Y;
			POINT2D_T RightEdge = {0,0};
			POINT2D_T LeftEdge = {0,0};
			int LastX = 0;
			int LastY = 0;
			//int LeftOpenAreaX = 0;
			//int RightOpenAreaX = 0;
			//int TargetDistance = 0;
			//int TargetClearDistance = DOOR_SPOTTING_DISTANCE_TENTH_INCHES + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES;
			// int TargetClearDistance = (__max(g_pNavSensorSummary->nRightFrontSideZone, g_pNavSensorSummary->nLeftFrontSideZone) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) ; // tenth inches

			int nDoorwaysFound = 0;
			DOORWAY_T DoorwayFound[MAX_DOORWAYS];	// array of potential multiple doors 
			for( int i=0; i<MAX_DOORWAYS; i++ )
			{
				DoorwayFound[i].CenterX = 0;
				DoorwayFound[i].Width = 0;
				DoorwayFound[i].RightEdge.X = 0;
				DoorwayFound[i].RightEdge.Y = 0;
				DoorwayFound[i].LeftEdge.X = 0;
				DoorwayFound[i].LeftEdge.Y = 0;
			}

			// Track last door edge found (default to first sample, in case door extends off sensor range to the right)
			// Look for the open path/door from right to left
			bool FirstValidSampleFound = FALSE;
			for( int i = 0; i < NumberOfKinectSamples; i++ ) 
			{
				X = g_KinectPointCloud->WallPoints[i].X; // tenth inches  WallPoints  MapPoints2D
				Y = g_KinectPointCloud->WallPoints[i].Y ;


				// Skip any bad values
				if( (X >= LASER_RANGEFINDER_TENTH_INCHES_ERROR) ||
					(Y >= LASER_RANGEFINDER_TENTH_INCHES_ERROR) )
				{
					continue;
				}

				// DEBUG ONLY!!!
				//ROBOT_LOG( TRUE, "KINECT DOOR SCAN: x = %d:  y = %d, ", X,Y )

				if( !FirstValidSampleFound )
				{
					if( X < LASER_RANGEFINDER_TENTH_INCHES_MAX )
					{
						LastX = X;
						LastY = Y;
						FirstValidSampleFound = true;
					}
					continue;
				}

				if( 0 == RightEdge.X )
				{ 
					// looking for right edge
					int TargetDistance = __min(LastY, DOOR_SPOTTING_DISTANCE_TENTH_INCHES) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES;
					if( Y >= TargetDistance )
					{	// Found inside of door edge! (or may be that the door extends off sensor range to the left)
						RightEdge.X = LastX;	// Use the last valid data
						RightEdge.Y = __min(LastY, DOOR_SPOTTING_DISTANCE_TENTH_INCHES); // Use the last valid data
					}
				}
				else
				{
					// Right edge found, looking for Left edge
					if( Y < RightEdge.Y + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES )
					{
						int DoorwayWidth = RightEdge.X - X;
						if( (DoorwayWidth >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES) &&
							(DoorwayWidth <= 420) )
						{
							// Found it!
							LeftEdge.X = X;
							LeftEdge.Y = Y;

							// Found a Doorway

								// Draw where we think the door is!

								// Draw Left Door Frame
								dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); 
								dc.LineTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (LeftEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );

								// Draw line connecting the two door frames
								dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + LeftEdge.X)) + 1, (graphRect.bottom) -  (int)(YScale * (LeftEdge.Y  + Y_NEGATIVE_TENTH_INCHES))  ); 
								dc.LineTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) );	// Connect to Right Door Frame

								// Draw Right Door Frame
								dc.MoveTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) - 5 ); // Draw Right Door Frame
								dc.LineTo( graphRect.left + (int)(XScale * (XCenter + RightEdge.X)) + 1, (graphRect.bottom) - (int)(YScale * (RightEdge.Y + Y_NEGATIVE_TENTH_INCHES)) + 5 );

							// Save the results
							DoorwayFound[nDoorwaysFound].Width = RightEdge.X - LeftEdge.X;
							DoorwayFound[nDoorwaysFound].CenterX = RightEdge.X - (DoorwayWidth/2);
							RightEdge.X = 0; // clear values to search for next doorway
							LeftEdge.X = 0;
							RightEdge.Y = 0;
							LeftEdge.Y = 0;

							if( ++nDoorwaysFound >= MAX_DOORWAYS)
							{
								ROBOT_LOG( TRUE, "ERROR!  Too Many Doorways Found!" )
								break;
							}
						}
						else
						{
							// Doorway too narrow, restart search
							RightEdge.X = 0;
						}
					}
				}
				LastX = X;
				LastY = Y;

			}	// for loop

			if( nDoorwaysFound > 0 )
			{
				ROBOT_LOG( TRUE, "At lest one Doorway / Path Found" )
				for(int i=0; i<nDoorwaysFound; i++)
				{
					ROBOT_LOG( TRUE, "Doorway %d:  Center = %d, Width = %d", i, DoorwayFound[i].CenterX, DoorwayFound[i].Width )

					// TODO-MUST-DAVES - Find door nearest center!
				}
			}

		}	// Number of Kinect Samples > 0

	} // if( 0 == NumberOfLaserSamples )

*******/


	////////////////////////////////
	// Graph Decoration
	dc.SelectObject( &m_solidPen );

//	if( midpoint > 0 )
	{
		dc.MoveTo( graphRect.left - 5, graphRect.bottom );
		dc.LineTo( graphRect.right, graphRect.bottom );
	}

	dc.MoveTo( graphRect.left, graphRect.top );
	dc.LineTo( graphRect.left, graphRect.bottom );

	dc.MoveTo( graphRect.left + 5, graphRect.top );
	dc.LineTo( graphRect.left, graphRect.top );

	dc.MoveTo( graphRect.left + 5, graphRect.bottom );
	dc.LineTo( graphRect.left, graphRect.bottom );

	dc.SelectObject( pOldPen );
}


BOOL CLaserDisplayWnd::OnEraseBkgnd(CDC* pDC) 
{
	IGNORE_UNUSED_PARAM (pDC);
	// We return TRUE without erasing in order to minimize blinking
	return TRUE;
}


void CLaserDisplayWnd::GetMinMaxInfo( double *pMin, double *pMax )
{
	*pMin = m_min;
	*pMax = m_max;
}

