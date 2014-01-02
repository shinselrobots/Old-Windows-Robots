// RadarDisplayWnd.cpp

#include "stdafx.h"
#include "Globals.h"
#include "RadarDisplayWnd.h"
#include "osbmp.h"
#include "math.h"
#include "HardwareConfig.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define ULTRASONIC_SENSOR_SPREAD_DEGREES	20
#define DISPLAY_TRIM_TENTH_INCHES			240	// 2 feet
#define DISPLAY_RANGE_MAX_TENTH_INCHES		IR_GUI_TENTH_INCHES_MAX	// change this if you want to change the range
/////////////////////////////////////////////////////////////////////////////
// CRadarDisplayWnd

#define MAXdouble 100000000
#define MINdouble -100000000

//#define m_dMaxScale 110
//#define m_dMinScale -25

//#define m_dMaxScale 257
//#define m_dMinScale -137

//#define m_dMaxScale 40
//#define m_dMinScale 20

CRadarDisplayWnd::CRadarDisplayWnd()
{
//	m_c1 = PALETTERGB( 255, 0, 0 );
//	m_c2 = PALETTERGB( 192, 0, 0 );
//	m_c3 = PALETTERGB( 128, 0, 0 );

	m_c1 = PALETTERGB( 0, 255, 0 );
	m_c2 = PALETTERGB( 0, 192, 0 );
	m_c3 = PALETTERGB( 0, 128, 0 );
	m_c4 = PALETTERGB( 0, 64, 0 );
	m_c5 = PALETTERGB( 255, 64, 64 );	// red

//	m_c1 = PALETTERGB( 0, 0, 255 );
//	m_c2 = PALETTERGB( 0, 0, 192 );
//	m_c3 = PALETTERGB( 0, 0, 128 );


	int i;
	for( i=0; i<IR1_SAMPLES; i++ )
		m_IrArray1[i] = 0; //(IR_GUI_TENTH_INCHES_MAX);	// WARNING! If Circle too big on GUI, fix this!

	for( i=0; i<FIXED_IR_SAMPLES; i++ )
		m_FixedIrArray[i] = (IR_GUI_TENTH_INCHES_MAX);

	for( i=0; i<US1_SAMPLES; i++ )
		m_UltraSonicArray1[i] = (ULTRASONIC_TENTH_INCHES_MAX - DISPLAY_TRIM_TENTH_INCHES);

	
	m_min = 0.0;
	m_max = 255.0;
	m_nSample = 0;

	m_dMinScale = 0;
	m_dMaxScale = 200;
}

CRadarDisplayWnd::~CRadarDisplayWnd()
{
}

CPoint CRadarDisplayWnd::CalculateEndPoint( double XCenter, double Degree, double Distance )
{	
	CPoint EndPoint;
	double Radians;


	if( Degree < 90 )
	{
		Radians = Degree * 0.017452778;	// convert to radians
		EndPoint.x = (long)(XCenter - cos(Radians)*Distance);
		EndPoint.y = (long)((sin(Radians)) * Distance);
	}
	else if( Degree > 90 )
	{
		Radians = (180 - Degree) * 0.017452778;	// convert to radians
		EndPoint.x = (long)(XCenter + cos(Radians) * Distance);
		EndPoint.y = (long)(sin(Radians) * Distance);
	}
	else
	{
		// Degree == 90
		EndPoint.y = (long)XCenter;
		EndPoint.y = (long)Distance;
	}

	return EndPoint;
}


BEGIN_MESSAGE_MAP(CRadarDisplayWnd, CWnd)
	//{{AFX_MSG_MAP(CRadarDisplayWnd)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CRadarDisplayWnd message handlers

int CRadarDisplayWnd::OnCreate(LPCREATESTRUCT lpCreateStruct) 
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


#define RED					RGB(255, 50, 50)	// Red
#define GREEN				RGB(  0,200,  0)	// Green
#define BLUE				RGB( 60, 60,255)	// Blue
#define LIGHT_GREEN			RGB(200,255,200)	// Light Green
#define LIGHT_BLUE			RGB(180,180,255)	// Light Blue

#define OBJECT_FOUND		RGB(255,255,120)	// Yellow

	m_solidPen.CreatePen( PS_SOLID, 1, m_c1 );
	m_scalePen.CreatePen( PS_SOLID, 1, m_c3 );

	m_FixedUSPen.CreatePen( PS_SOLID, 3, BLUE );
	m_FixedIRPen.CreatePen( PS_SOLID, 2, RED );

	m_ScanningUSPen.CreatePen( PS_SOLID, 4, LIGHT_BLUE );
	m_ScanningIRPen.CreatePen(PS_SOLID, 4, LIGHT_BLUE);

	m_ObjectCenterPen.CreatePen(PS_SOLID, 8, OBJECT_FOUND);
	m_ObjectSizePen.CreatePen(PS_SOLID, 1, OBJECT_FOUND);

	return 0;
}



////////////////////////////////////////////////////////////////////////////////////
// OnPaint - All interesting Radar drawing happens here!
void CRadarDisplayWnd::OnPaint() 
{
	int j;
	CPaintDC paintdc(this); // device context for painting	
	COffscreenDC dc( &paintdc, NULL );
	
	CRect clientRect, graphRect, textRect;
	CString csTemp;

	// Get the window size
	GetClientRect( &clientRect );

	// Data coordinates (prior to scaling for GUI)
	double XCenter = (DISPLAY_RANGE_MAX_TENTH_INCHES - DISPLAY_TRIM_TENTH_INCHES);	// Cut down width of graph
	double XMax = (DISPLAY_RANGE_MAX_TENTH_INCHES - DISPLAY_TRIM_TENTH_INCHES)*2;	// for half circle
	double YMax = DISPLAY_RANGE_MAX_TENTH_INCHES;
	 
	// Calculations
	graphRect = clientRect;	// These values set the black boarder around the display
	graphRect.left += 12;	// leave extra room for the labels
	graphRect.right -= 2;
	graphRect.top += 2;
	graphRect.bottom -= 2;

	int graphDeltaY = graphRect.bottom - graphRect.top;
	int graphDeltaX = graphRect.right - graphRect.left;
	int BarSize = graphDeltaX / MAX_DATAPOINTS;	// Size of each sample in pixels

	double YScale = graphDeltaY / YMax;	// Size of each step in the grid
	double XScale = graphDeltaX / XMax;	

	// Clear client rect
	dc.FillSolidRect( clientRect, PALETTERGB( 0, 0, 0 ) );

	// Scale text
	CFont *pOldFont = dc.SelectObject( &m_scaleFont );
	dc.SetBkColor( PALETTERGB( 0, 0, 0 ) );
	
	// Draw graph background
	dc.FillSolidRect( graphRect, m_c4 );


	// Draw Scale Grid
	CPen *pOldPen = dc.SelectObject( &m_scalePen );
	int XCenterScaled = (int)(graphRect.left + (XScale*XCenter));
	// Draw CenterLine
	dc.MoveTo( XCenterScaled +1, graphRect.top );
	dc.LineTo( XCenterScaled +1, graphRect.bottom );
	dc.MoveTo( XCenterScaled -1, graphRect.top );
	dc.LineTo( XCenterScaled -1, graphRect.bottom );

	// Draw Vertical grid lines
	for( j = 0; j < XCenter; j+=120 )	// 12 inches per foot
	{
		dc.MoveTo( (int)(XCenterScaled + XScale*j), graphRect.top );
		dc.LineTo( (int)(XCenterScaled + XScale*j), graphRect.bottom );
		dc.MoveTo( (int)(XCenterScaled - XScale*j), graphRect.top );
		dc.LineTo( (int)(XCenterScaled - XScale*j), graphRect.bottom );
	}

	// Draw Horizontal grid lines and text labels
	dc.SetTextColor( m_c2 );
	for( j = 0; j < YMax; j+=120 )
	{
		csTemp.Format( "%d", (j / 120) );
		textRect.SetRect( 
			0, 
			(int)((graphRect.bottom - YScale*j) - 9), 
			graphRect.left - 5, 
			(int)((graphRect.bottom - YScale*j) + 9) );
		dc.DrawText( csTemp, textRect, DT_RIGHT|DT_SINGLELINE );
		dc.MoveTo( graphRect.left,  (int)(graphRect.bottom - YScale*j) );
		dc.LineTo( graphRect.right, (int)(graphRect.bottom - YScale*j) );
	}
	dc.SelectObject( pOldFont );

	// Draw graph values (around a half-circle)
	double Degree;
	double Distance;
//	double XData, YData;
//	double Radians;
//	double DegreeIncrement;
	CPoint LineStartPoint, LineEndPoint;
	int SensorNumber;

	////////////////////////////////////////////////////////////////////////////////////
	// Fixed UltraSonic Sensors
	dc.SelectObject( &m_FixedUSPen );	// BLUE

	// Note, US[0] camera sensor handled correctly by SensorOffsetDegrees_US
	for( SensorNumber = 0; SensorNumber < NUM_US_SENSORS; SensorNumber++ )
	{
		Distance = g_SensorStatus.US[SensorNumber];
		if( (Distance >= (double) DISPLAY_RANGE_MAX_TENTH_INCHES ) ||	(Distance < 9.0) )	// Tenth inches!
		{
			continue; // Outside max range of sensor
		}

		Degree = (90.0 - SensorOffsetDegrees_US[SensorNumber]) - SENSOR_HALF_FOV_DEGREES_US;
		LineStartPoint = CalculateEndPoint( XCenter, Degree, Distance );

		Degree = (90.0 - SensorOffsetDegrees_US[SensorNumber]) + SENSOR_HALF_FOV_DEGREES_US;
		LineEndPoint = CalculateEndPoint( XCenter, Degree, Distance );
		
//		dc.MoveTo( (int)(graphRect.left + (XScale * LineStartPoint.x)), (int)(graphRect.bottom - (YScale * LineStartPoint.y) ));
//		dc.LineTo( (int)(graphRect.left + (XScale * LineEndPoint.x)), (int)(graphRect.bottom - (YScale * LineEndPoint.y) ));

	}


	////////////////////////////////////////////////////////////////////////////////////
	// Fixed IR Sensors!
	dc.SelectObject( &m_FixedIRPen );	// Red

	for( SensorNumber = 0; SensorNumber < NUM_IR_SENSORS; SensorNumber++ )
	{
		Distance = g_SensorStatus.IR[SensorNumber];
		if( (Distance >= IR_LR_TENTH_INCHES_MAX ) ||	(Distance < 9.0) )	// Tenth Inches!
		{
			continue; // Outside max range of sensor
		}
		// For IR, make the line longer for close objects, so it shows up better
		int  IR_Line_Degrees = (IR_LR_TENTH_INCHES_MAX - (int)Distance) / 20;	// SENSOR_HALF_FOV_DEGREES_IR not use for Radar IR
		if( IR_Line_Degrees <  SENSOR_HALF_FOV_DEGREES_IR )
		{
			IR_Line_Degrees = SENSOR_HALF_FOV_DEGREES_IR;
		}

		Degree = (90 + SensorOffsetDegrees_IR[SensorNumber]) - IR_Line_Degrees;	
		LineStartPoint = CalculateEndPoint( XCenter, Degree, Distance );

		Degree = (90 + SensorOffsetDegrees_IR[SensorNumber]) + IR_Line_Degrees;
		LineEndPoint = CalculateEndPoint( XCenter, Degree, Distance );

		//Debug
		//dc.MoveTo( 15, 5 );
		//dc.LineTo( 285, 185);
		// For debug:  valid values range from top left = 15,5 to bottom right = 285,185
		dc.MoveTo( (int)(graphRect.left + (XScale * LineStartPoint.x)), (int)(graphRect.bottom - (YScale * LineStartPoint.y) ));
		dc.LineTo( (int)(graphRect.left + (XScale * LineEndPoint.x)), (int)(graphRect.bottom - (YScale * LineEndPoint.y) ));


	}

/***









	DegreeIncrement = (double)180 / (double)(FIXED_IR_SAMPLES*2+1);
	// step angle of each sample - double up to make the sensor lines not so long


	CRect LineStartRect, LineEndRect;
	int ArcSegment;
	BOOL SkipThisAngle = TRUE;	// Skip every other position to break up the readings on the display
	int  SensorToDisplay = 0;

	for( i = 0; i < ((FIXED_IR_SAMPLES*2) + 1); i++ )
	{
		ArcSegment = i;	// Display every other line (leave gaps between sensor readings)

		if( SkipThisAngle )
		{
			SkipThisAngle = FALSE;
			continue;	// Skip to next arc
		}
		else
		{
			SkipThisAngle = TRUE;	// Setup for next loop
		}


		SensorToDisplay = i/2;
		Distance = m_FixedIrArray[SensorToDisplay];
		if( (Distance > IR_GUI_INCHES_MAX )	||	
			(Distance < 0) )		// inches!
		{
			// Outside max range of sensor
			Distance = NO_OBJECT_IN_RANGE;
		}

		if( (NO_OBJECT_IN_RANGE == Distance) || (0 == Distance) )	// Out of range, or no value
		{
			continue;	// leave blank
		}

//		Distance += ROBOT_SIZE;	// Add 6 inches to represent the robot itself
		for( int z = 0; z <=1; z++ )	// Begining and end points
		{
			if( z == 0 )
			{
				Degree = DegreeIncrement * ArcSegment;
			}
			else
			{
				Degree = DegreeIncrement * (ArcSegment+1);
			}

			if( Degree < 90 )
			{
				Radians = Degree * 0.017452778;	// convert to radians
				XData = XCenter - cos(Radians)*Distance;
				YData = (sin(Radians)) * Distance;
			}
			else if( Degree > 90 )
			{
				Radians = (180 - Degree) * 0.017452778;	// convert to radians
				XData = XCenter + cos(Radians) * Distance;
				YData = sin(Radians) * Distance;
			}
			else
			{
				// Degree == 90
				XData = XCenter;
				YData = Distance;
			}

			if( z == 0 )
			{
				LineStartRect.left =   (long)XData;
				LineStartRect.bottom = (long)YData;
				if( (5 == SensorToDisplay ) && ( LineStartRect.bottom < 5 ) )
				{
					LineStartRect.bottom = 5;	// Min lengh of side bar
				}
			}
			else
			{
				LineEndRect.left = (long)XData;
				LineEndRect.bottom = (long)YData;
				if( (0 == SensorToDisplay ) && ( LineEndRect.bottom < 5 ) )
				{
					LineEndRect.bottom = 5;	// Min lengh of side bar
				}
			}

		}
		
		dc.MoveTo( (int)(graphRect.left + (XScale * LineStartRect.left)), (int)(graphRect.bottom - (YScale * LineStartRect.bottom)) );
		dc.LineTo( (int)(graphRect.left + (XScale * LineEndRect.left)), (int)(graphRect.bottom - (YScale * LineEndRect.bottom)) );
	
	}
***/

/****************
	////////////////////////////////////////////////////////////////////////////////////
	// Scanning UltraSonic Sensor: US_ARRAY1
	dc.SelectObject( &m_ScanningUSPen );

	DegreeIncrement = (double)180 / (double)US1_SAMPLES;	// step angle of each sample
	// 90 degree sweep:
	DegreeIncrement /= 2;
	
	for( i = 0; i < US1_SAMPLES; i++ )
	{
		Distance = m_UltraSonicArray1[i];
		if( Distance > 85 )	// inches!
		{
			// Outside max range of sensor
			Distance = DISPLAY_RANGE_MAX_TENTH_INCHES-10;	// show half-circle on graph
		}
		// 180 degree sweep: Degree = DegreeIncrement * i;
		// 90 degree sweep: 
		Degree = (DegreeIncrement * i) +45;

		if( Degree < 90 )
		{
			Radians = Degree * DEGREES_TO_RADIANS;	// convert to radians
			XData = XCenter - cos(Radians)*Distance;
			YData = (sin(Radians)) * Distance;
		}
		else if( Degree > 90 )
		{
			Radians = (180 - Degree) * DEGREES_TO_RADIANS;	// convert to radians
			XData = XCenter + cos(Radians) * Distance;
			YData = sin(Radians) * Distance;
		}
		else
		{
			// Degree == 90
			XData = XCenter;
			YData = Distance;
		}
	
		dc.MoveTo( graphRect.left + (XScale * XData) - 4, graphRect.bottom - (YScale * YData) );
		dc.LineTo( graphRect.left + (XScale * XData) + 4, graphRect.bottom - (YScale * YData) );
	
	}

	////////////////////////////////////////////////////////////////////////////////////
	// Display Detected US Objects!
	dc.SelectObject( &m_ObjectSizePen );	// Thin Yellow

	int DegreeStart = (DegreeIncrement * g_pSensorSummary->nClosestRadarObjectLeft) +45;
	int DegreeEnd = (DegreeIncrement * g_pSensorSummary->nClosestRadarObjectRight) +45;
	int XStart, XEnd, YStart, YEnd;

	Distance = g_pSensorSummary->nClosestRadarObjectDistance;

	if( Distance < DISPLAY_RANGE_MAX_TENTH_INCHES-10 )
	{
		// Not Outside max range of sensor

		// 180 degree sweep: Degree = DegreeIncrement * i;
		// 90 degree sweep: 

		// Start of Line
		if( DegreeStart < 90 )
		{
			Radians = DegreeStart * 0.017452778;	// convert to radians
			XStart = XCenter - cos(Radians)*Distance;
			YStart = (sin(Radians)) * Distance;
		}
		else if( DegreeStart > 90 )
		{
			Radians = (180 - DegreeStart) * 0.017452778;	// convert to radians
			XStart = XCenter + cos(Radians) * Distance;
			YStart = sin(Radians) * Distance;
		}
		else
		{
			// DegreeStart == 90
			XStart = XCenter;
			YStart = Distance;
		}

		// End of Line
		if( DegreeEnd < 90 )
		{
			Radians = DegreeEnd * 0.017452778;	// convert to radians
			XEnd = XCenter - cos(Radians)*Distance;
			YEnd = (sin(Radians)) * Distance;
		}
		else if( DegreeEnd > 90 )
		{
			Radians = (180 - DegreeEnd) * 0.017452778;	// convert to radians
			XEnd = XCenter + cos(Radians) * Distance;
			YEnd = sin(Radians) * Distance;
		}
		else
		{
			// DegreeEnd == 90
			XEnd = XCenter;
			YEnd = Distance;
		}
	
		dc.MoveTo( graphRect.left + (XScale * XStart) - 2, graphRect.bottom - (YScale * YStart) );
		dc.LineTo( graphRect.left + (XScale * XEnd) + 2, graphRect.bottom - (YScale * YEnd) );
 
		// Display Center of Object
		dc.SelectObject( &m_ObjectCenterPen );	// Thick Yellow
		Degree = (DegreeIncrement * g_pSensorSummary->nClosestRadarObjectLocation) +45;
		if( Degree < 90 )
		{
			Radians = Degree * DEGREES_TO_RADIANS;	// convert to radians
			XData = XCenter - cos(Radians)*Distance;
			YData = (sin(Radians)) * Distance;
		}
		else if( Degree > 90 )
		{
			Radians = (180 - Degree) * DEGREES_TO_RADIANS;	// convert to radians
			XData = XCenter + cos(Radians) * Distance;
			YData = sin(Radians) * Distance;
		}
		else
		{
			// Degree == 90
			XData = XCenter;
			YData = Distance;
		}
	
		dc.MoveTo( graphRect.left + (XScale * XData) - 6, graphRect.bottom - (YScale * YData) );
		dc.LineTo( graphRect.left + (XScale * XData) + 6, graphRect.bottom - (YScale * YData) );
	
		

	}	// Display Object, Distance < DISPLAY_RANGE_MAX_TENTH_INCHES

*****************/

	////////////////////////////////////////////////////////////////////////////////////
	// Graph Decoration
	dc.SelectObject( &m_solidPen );

//	if( midpoint > 0 )
	{
		dc.MoveTo( graphRect.left - 5, graphRect.bottom );
		dc.LineTo( graphRect.right, graphRect.bottom );
	}

	dc.MoveTo( graphRect.left, graphRect.top );
	dc.LineTo( graphRect.left, graphRect.bottom );

	dc.MoveTo( graphRect.left - 5, graphRect.top );
	dc.LineTo( graphRect.left, graphRect.top );

	dc.MoveTo( graphRect.left - 5, graphRect.bottom );
	dc.LineTo( graphRect.left, graphRect.bottom );

	dc.SelectObject( pOldPen );
}


BOOL CRadarDisplayWnd::OnEraseBkgnd(CDC* pDC) 
{
	IGNORE_UNUSED_PARAM (pDC);
	// We return TRUE without erasing in order to minimize blinking
	return TRUE;
}


void CRadarDisplayWnd::GetMinMaxInfo( double *pMin, double *pMax )
{
	*pMin = m_min;
	*pMax = m_max;
}



void CRadarDisplayWnd::SetData( int  SensorNumber, int  Length, int *  Buffer )
{
	switch( SensorNumber )
	{
		case FIXED_IR_ARRAY:
		{
			for( int  i=0; i<Length; i++ )
			{
				m_FixedIrArray[i] = Buffer[i];	
			}
			break;
		}
		case FIXED_US_ARRAY:
		{
			for( int  i=0; i<Length; i++ )
			{
				m_FixedUltraSonicArray[i] = Buffer[i];	
			}
			break;
		}
		case IR_ARRAY1:
		{
//			ROBOT_LOG( TRUE,  "IR RADAR: ");
			for( int  i=0; i<Length; i++ )
			{
				m_IrArray1[i] = Buffer[i];	
//				ROBOT_LOG( TRUE,  "%d=%d ", i, Buffer[i]);
			}
//			ROBOT_LOG( TRUE,  "\n" );
			break;
		}
		case US_ARRAY1:
		{
//			ROBOT_LOG( TRUE,  "DEBUG US RADAR: ");
			for( int  i=0; i<Length; i++ )
			{
				m_UltraSonicArray1[i] = Buffer[i];	
//				ROBOT_LOG( TRUE,  "%d=%d ", i, Buffer[i]);
			}
//			ROBOT_LOG( TRUE,  "\n" );
			break;
		}
	}

	// Update display
	CRect wndRect;
	GetClientRect( &wndRect );
	InvalidateRect( wndRect, FALSE );
	UpdateWindow();

}

