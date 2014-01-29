// MapView.cpp : implementation of the CMapView class
//
// RW = Real World coordinates, in inches
// MC = Logical Map Coordinates for display, and are subject to current Zoom factor




#include "stdafx.h"
#include "Globals.h"
#include "Robot.h"
#include "MainFrm.h"
#include "MapDoc.h"
#include "MapView.h"
#include <math.h>
#include "HardwareConfig.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// Constants
#define ZoomPercent25		0.025
#define ZoomPercent50		0.050
#define ZoomPercent100		0.100
#define ZoomPercent150		0.150
#define ZoomPercent200		0.200
#define ZoomPercent400		0.400

#define ROBOT_BODY_DRAW_RADIUS_3	120.0	// Outer size of Robot
#define ROBOT_BODY_DRAW_RADIUS_2	 90.0	// Outer size of Robot
#define ROBOT_BODY_DRAW_RADIUS_1	 60.0	// Outer size of Robot

#define TENTHINCHES_PER_FOOT		120



/////////////////////////////////////////////////////////////////////////////
// CMapView

IMPLEMENT_DYNCREATE(CMapView, CScrollView)

BEGIN_MESSAGE_MAP(CMapView, CScrollView)
	//{{AFX_MSG_MAP(CMapView)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_RBUTTONDOWN()
	ON_COMMAND(ID_SEL_MAP_VIEW_BTN, OnSelMapViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_MAP_VIEW_BTN, OnUpdateSelMapViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_PATH_VIEW_BTN, OnUpdateSelPathViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_CMD_VIEW_BTN, OnUpdateSelCmdViewBtn)
	ON_COMMAND(ID_SEL_CMD_VIEW_BTN, OnSelCmdViewBtn)
	ON_COMMAND(ID_SEL_PATH_VIEW_BTN, OnSelPathViewBtn)
	ON_COMMAND(ID_SEL_SETUP_VIEW_BTN, OnSelSetupViewBtn)
	ON_UPDATE_COMMAND_UI(ID_SEL_SETUP_VIEW_BTN, OnUpdateSelSetupViewBtn)
	ON_COMMAND(ID_TYPE_OBSTACLE, OnTypeObstacle)
	ON_COMMAND(ID_TYPE_USERDRAW, OnTypeUserdraw)
	ON_COMMAND(ID_TYPE_WALLOBJECT, OnTypeWallobject)
	ON_UPDATE_COMMAND_UI(ID_TYPE_OBSTACLE, OnUpdateTypeObstacle)
	ON_UPDATE_COMMAND_UI(ID_TYPE_USERDRAW, OnUpdateTypeUserdraw)
	ON_UPDATE_COMMAND_UI(ID_TYPE_WALLOBJECT, OnUpdateTypeWallobject)
	ON_COMMAND(ID_NAVIGATION_MODE, OnNavigationMode)
	ON_UPDATE_COMMAND_UI(ID_NAVIGATION_MODE, OnUpdateNavigationMode)
	ON_COMMAND(IDC_WAYPOINT_LOCATION, OnWaypointLocation)
	ON_UPDATE_COMMAND_UI(IDC_WAYPOINT_LOCATION, OnUpdateWaypointLocation)
	ON_COMMAND(ID_ADD_PAGE_ABOVE, OnAddPageAbove)
	ON_UPDATE_COMMAND_UI(ID_ADD_PAGE_ABOVE, OnUpdateAddPageAbove)
	ON_COMMAND(ID_ADD_PAGE_BELOW, OnAddPageBelow)
	ON_UPDATE_COMMAND_UI(ID_ADD_PAGE_BELOW, OnUpdateAddPageBelow)
	ON_COMMAND(ID_ADD_PAGE_LEFT, OnAddPageLeft)
	ON_UPDATE_COMMAND_UI(ID_ADD_PAGE_LEFT, OnUpdateAddPageLeft)
	ON_COMMAND(ID_ADD_PAGE_RIGHT, OnAddPageRight)
	ON_UPDATE_COMMAND_UI(ID_ADD_PAGE_RIGHT, OnUpdateAddPageRight)
	ON_COMMAND(ID_SHOW_GRIDMAP_SQUARES, OnShowGridmapSquares)
	ON_COMMAND(ID_SHOW_ROBOT_TRAILS, OnShowRobotTrails)
	ON_COMMAND(ID_SHOW_SENSOR_READINGS, OnShowSensorReadings)
	ON_UPDATE_COMMAND_UI(ID_SHOW_ROBOT_TRAILS, OnUpdateShowRobotTrails)
	ON_UPDATE_COMMAND_UI(ID_SHOW_GRIDMAP_SQUARES, OnUpdateShowGridmapSquares)
	ON_UPDATE_COMMAND_UI(ID_SHOW_SENSOR_READINGS, OnUpdateShowSensorReadings)
	ON_COMMAND(ID_GPS_ENABLE, OnGpsEnable)
	ON_UPDATE_COMMAND_UI(ID_GPS_ENABLE, OnUpdateGpsEnable)
	ON_COMMAND(ID_ZOOM_MAP_025, OnZoomMap025)
	ON_COMMAND(ID_ZOOM_MAP_050, OnZoomMap050)
	ON_COMMAND(ID_ZOOM_MAP_100, OnZoomMap100)
	ON_COMMAND(ID_ZOOM_MAP_150, OnZoomMap150)
	ON_COMMAND(ID_ZOOM_MAP_200, OnZoomMap200)
	ON_COMMAND(ID_ZOOM_MAP_400, OnZoomMap400)
	ON_COMMAND(ID_EXPORT_TO_FILE, OnExportToTextFile)
	ON_COMMAND(ID_IMPORT_FROM_FILE, OnImportFromTextFile)
	ON_COMMAND(ID_CLEAR_MOUSE_TRAIL, OnClearMouseTrail)
	ON_COMMAND(ID_CLEAR_SENSOR_READINGS, OnClearSensorReadings)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_025, OnUpdateZoomMap025)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_050, OnUpdateZoomMap050)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_100, OnUpdateZoomMap100)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_150, OnUpdateZoomMap150)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_200, OnUpdateZoomMap200)
	ON_UPDATE_COMMAND_UI(ID_ZOOM_MAP_400, OnUpdateZoomMap400)
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_COMMAND(ID_SET_CURRENT_LOCATION, OnSetCurrentLocation)
	ON_UPDATE_COMMAND_UI(ID_SET_CURRENT_LOCATION, OnUpdateSetCurrentLocation)
	ON_COMMAND(ID_TYPE_CLIFF, OnTypeCliffObject)
	ON_UPDATE_COMMAND_UI(ID_TYPE_CLIFF, OnUpdateTypeCliffObject)
	ON_COMMAND(ID_TYPE_DOOR_OBJECT, OnTypeDoorObject)
	ON_UPDATE_COMMAND_UI(ID_TYPE_DOOR_OBJECT, OnUpdateTypeDoorObject)
	ON_COMMAND(ID_TYPE_LOW_OBSTACLE, OnTypeLowObstacle)
	ON_UPDATE_COMMAND_UI(ID_TYPE_LOW_OBSTACLE, OnUpdateTypeLowObstacle)
	//}}AFX_MSG_MAP
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, CView::OnFilePrintPreview)
	ON_MESSAGE( (WM_ROBOT_DISPLAY_BULK_ITEMS), OnMapDisplayBulkItem )
	ON_MESSAGE( (WM_ROBOT_BULK_DATA_TO_SERVER), OnMapBulkItemToServer )
	ON_MESSAGE( (WM_ROBOT_UPDATE_VIEW), OnRobotUpdateView )


	ON_UPDATE_COMMAND_UI(ID_INDICATOR_POS, OnUpdatePos)	// for Map X,Y display


END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMapView construction/destruction

CMapView::CMapView()
{
	m_DoingLongStroke = FALSE;
	m_ShowRobotTrails = TRUE;
	m_ShowGridMapSquares = TRUE;
	m_ShowSensorReadings = TRUE;

	m_LastGPSPos.x = 0;
	m_LastGPSPos.y = 0;
	m_LastFixMode = 0;
	m_LastRobotPos.x = 0;
	m_LastRobotPos.y = 0;
	m_RW_TargetPoint.x = 0;
	m_RW_TargetPoint.y = 0;

	m_MC_MapZoom = ZoomPercent100;
	// Initial document size is 800 x 900 logical units (8"x 9"). Maps to TENTH INCHES!
	m_RW_MapBottomLeft.x = 0;
	m_RW_MapBottomLeft.y = 0;
	m_RW_MapTopRight.x = 0; (LONG)MAP_PAGE_WIDTH;
	m_RW_MapTopRight.y = 0; (LONG)MAP_PAGE_HEIGHT;

	m_RealWorldInitialized = FALSE;
	m_GPSEnabled = FALSE;

}

CMapView::~CMapView()
{
	g_RobotMapViewHWND = NULL;
}


BOOL CMapView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	//cs.cx =  410;
	//cs.cy = 415;


	return CView::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// To keep the map simple, I propose to use a size that corresponds to 8"
// on paper (easy for printing).  In this system, 1 unit will equal 1 inch
// in the "real world" (so a page will display 800 inches = 66 feet)
// The map will grow by even pages, to avoid continually being at the edge
// of the map, as the robot explores.
//
// Conventions: 
//		RW = Real World coordinates
//		MD = Map Display coordinates
//
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
// CMapView drawing

void CMapView::OnDraw(CDC* pDC)
{
	//#define OFFSET_X	20
	//#define OFFSET_Y	-20
	#define TEXT_FONT_HEIGHT 16

	#define USER_STROKE_LINE_COLOR	RGB(  0,  0,  0)	// Black
	#define WALL_LINE_COLOR			RGB(115,  7,125)	// Plumb
	#define DOOR_LINE_COLOR			RGB(180,180,255)	// Light Blue
	#define CLIFF_LINE_COLOR		RGB(255,120,0)		// Orange
	#define LOW_OBSTACLE_LINE_COLOR	RGB(128,128,128)	// Light Gray

	#define PATH_COST_SQUARE_COLOR4	RGB(85,  0,113)	// Dark Purple
	#define PATH_COST_SQUARE_COLOR3	RGB(190,  105,230)	// Med Purple
//	#define PATH_COST_SQUARE_COLOR3	RGB(245,  80,10)	// Orange
	#define PATH_COST_SQUARE_COLOR2	RGB(210,  200,225)	// Light Purple

	#define OBSTACLE_LINE_COLOR		RGB( 10,110, 10)	// Dark Green 
	#define BOX_LINE_COLOR			RGB(200,  0,  0)	// Red
	#define WAYPOINT_COLOR			RGB(  0,200,  0)	// Green
	#define SEGMENT_LINE_COLOR		RGB(  0,  0,200)	// Blue
	#define ROBOT_ICON_COLOR		RGB(  0,  0,  0)	// Black
	#define ROBOT_ER1_TRAIL_COLOR	RGB( 20,255, 20)	// Dark Blue
	#define ROBOT_MOUSE_TRAIL_COLOR	RGB(200,200,200)	// Gray
	#define GRID_LINE_COLOR			RGB(200,255,200)	// Light Green
	#define WAYPOINT_LINE_COLOR		RGB(180,180,255)	// Light Blue
	#define GPS_LOST_COLOR			RGB(200,  0,  0)	// Red
	#define GPS_2D_COLOR			RGB(200,255,200)	// Light Green
	#define GPS_3D_COLOR			RGB(  0,200,  0)	// Green
	#define GPS_CHANGE_POS_COLOR	RGB(200,  0,250)	// Purple
	#define IR_SENSOR_COLOR			RGB(216, 104, 155)	// Light Pink
	#define US_SENSOR_COLOR			RGB(190, 210,255)	// Light Blue


	#define WALL_SQUARE_COLOR		RGB(245, 157,255)	// Light Plumb
	#define WALL_SQUARE_LINE_WIDTH	10					


	#define STROKE_LINE_WIDTH		1
	#define SEGMENT_LINE_WIDTH		1
	#define ROBOT_TRAIL_WIDTH		1
	#define IR_SENSOR_LINE_WIDTH	3
	#define US_SENSOR_LINE_WIDTH	1

	#define WAYPOINT_LINE_WIDTH		4

	#define WAYPOINT_BOX_SIZE		4
	#define ROBOT_ICON_LINE_WIDTH	3
	#define ROBOT_ICON_OUTLINE_SIZE 10
	#define GPS_TRAIL_BOX_SIZE		8
	#define ROBOT_TRAIL_BOX_SIZE	6
	#define SENSOR_BOX_SIZE			4
	#define HEADING_LINE_LEN		300

	POINT RWFrom, RWTo, RWWaypoint, RWGPSpoint, RWRobot;	// Real World coordinates
	POINT MCFrom, MCTo, MCWaypoint, MCGPSpoint, MCRobot;	// Logical Map Coordinates
	long X, Y;
	double dX, dY;	// Used in angle calculations
	int NewX, NewY;	// Absolute coordinates, allows for negative numbers

	CString strLabel;
	POSITION pos;	// for walking through lists

	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	// Get the invalidated rectangle of the view, or in the case
	// of printing, the clipping region of the printer DC.
	CRect rectClip;
	CRect rectStroke;
	pDC->GetClipBox(&rectClip);

	pDC->LPtoDP(&rectClip);
	rectClip.InflateRect(1, 1); // avoid rounding to nothing

	//Note: CScrollView::OnPaint() will have already adjusted the
	//viewpoint origin before calling OnDraw(), to reflect the
	//currently scrolled position.

	// Prepare a font size for displaying text
	LOGFONT logFont;
	memset(&logFont, 0, sizeof(LOGFONT));
	logFont.lfHeight = TEXT_FONT_HEIGHT;
	CFont font;
	CFont* pOldFont = NULL;
	if (font.CreateFontIndirect(&logFont))
	{
		pOldFont = pDC->SelectObject(&font);
	}
	pDC->SetTextAlign(TA_LEFT);

	// Initialize all the pens we need

	CPen penSegment;
	if( !penSegment.CreatePen(PS_SOLID, SEGMENT_LINE_WIDTH, SEGMENT_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating Segment Pen!\n" );
		return;
	}
	CPen* pOldPen = pDC->SelectObject( &penSegment );	// Save the old pen

	CPen penIRSensor;	// IR Sensor
	if( !penIRSensor.CreatePen(PS_SOLID, IR_SENSOR_LINE_WIDTH, IR_SENSOR_COLOR))
	{
		ROBOT_LOG( TRUE, "MapView: Error creating IRSensor Pen!\n" );
		return;
	}
	CPen penUSSensor;	// UltraSonic Sensor
	if( !penUSSensor.CreatePen(PS_SOLID, US_SENSOR_LINE_WIDTH, US_SENSOR_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating USSensor Pen!\n" );
		return;
	}
	CPen penCliffSquare;
	if( !penCliffSquare.CreatePen(PS_SOLID, 1, CLIFF_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating CliffSquare Pen!\n" );
		return;
	}
	CPen penWallSquare;
	if( !penWallSquare.CreatePen(PS_SOLID, 1, WALL_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating WallSquare Pen!\n" );
		return;
	}
	CPen penObstacleSquare;
	if( !penObstacleSquare.CreatePen(PS_SOLID, 1, OBSTACLE_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penObstacleSquare Pen!\n" );
		return;
	}
	CPen penLowObstacleSquare;
	if( !penLowObstacleSquare.CreatePen(PS_SOLID, 1, LOW_OBSTACLE_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penLowObstacleSquare Pen!\n" );
		return;
	}
	CPen penCostSquare4;
	if( !penCostSquare4.CreatePen(PS_SOLID, 1, PATH_COST_SQUARE_COLOR4))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penCostSquare4 Pen!\n" );
		return;
	}
	CPen penCostSquare3;
	if( !penCostSquare3.CreatePen(PS_SOLID, 1, PATH_COST_SQUARE_COLOR3))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penCostSquare3 Pen!\n" );
		return;
	}
	CPen penCostSquare2;
	if( !penCostSquare2.CreatePen(PS_SOLID, 1, PATH_COST_SQUARE_COLOR2))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penCostSquare2 Pen!\n" );
		return;
	}
	CPen penBox;
	if( !penBox.CreatePen(PS_SOLID, SEGMENT_LINE_WIDTH, BOX_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating Box Pen!\n" );
		return;
	}
	CPen penWaypoint;
	if( !penWaypoint.CreatePen(PS_SOLID, WAYPOINT_LINE_WIDTH, WAYPOINT_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating Waypoint Pen!\n" );
		return;
	}
	CPen penRobotIcon;
	if( !penRobotIcon.CreatePen(PS_SOLID, ROBOT_ICON_LINE_WIDTH, ROBOT_ICON_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating RobotIcon Pen!\n" );
		return;
	}
	CPen penRobotER1MotorTrail;
	if( !penRobotER1MotorTrail.CreatePen(PS_SOLID, ROBOT_ICON_LINE_WIDTH, ROBOT_ER1_TRAIL_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating penRobotER1MotorTrail Pen!\n" );
		return;
	}
	CPen penRobotMouseTrail;
	if( !penRobotMouseTrail.CreatePen(PS_SOLID, ROBOT_TRAIL_WIDTH, ROBOT_MOUSE_TRAIL_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating RobotMouseTrail Pen!\n" );
		return;
	}
	CPen penRobotGPS;
	if( !penRobotGPS.CreatePen(PS_SOLID, ROBOT_ICON_LINE_WIDTH, ROBOT_ICON_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating RobotIcon Pen!\n" );
		return;
	}
	CPen penGPSLost;
	if( !penGPSLost.CreatePen(PS_SOLID, 3, GPS_LOST_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating GPSLost Pen!\n" );
		return;
	}
	CPen penGPS2D;
	if( !penGPS2D.CreatePen(PS_SOLID, 3, GPS_2D_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating GPS2D Pen!\n" );
		return;
	}
	CPen penGPS3D;
	if( !penGPS3D.CreatePen(PS_SOLID, 3, GPS_3D_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating GPS3D Pen!\n" );
		return;
	}
	CPen penGPSNewPos;
	if( !penGPSNewPos.CreatePen(PS_SOLID, 3, GPS_CHANGE_POS_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating GPSNewPos Pen!\n" );
		return;
	}
	CPen penGridLines;
	if( !penGridLines.CreatePen(PS_SOLID, 1, GRID_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating GPS3D Pen!\n" );
		return;
	}
	CPen penWaypointFinderLine;	// Waypoint "finder line"
	if( !penWaypointFinderLine.CreatePen(PS_DOT, 1, WAYPOINT_LINE_COLOR))
	{
		ROBOT_LOG( TRUE,  "MapView: Error creating Waypoint Finder Line Pen!\n" );
		return;
	}

	/////////////////////////////////////////////////////////////////////////////
	// Draw the boarders and grid
	//CSize MC_MapSize = GetMCZoomedMapSize(); 
	POINT MC_MapBottomLeft = TranslateRWtoMC( m_RW_MapBottomLeft );
	POINT MC_MapTopRight = TranslateRWtoMC( m_RW_MapTopRight );

	pDC->SelectObject( &penBox );

	// Map Edge boundry
	int nBoundrySize = 1; // No pad space
	pDC->MoveTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );	// Top edge
	pDC->LineTo(  (MC_MapTopRight.x - nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );
	pDC->LineTo(  (MC_MapTopRight.x - nBoundrySize),	(MC_MapTopRight.y - nBoundrySize) );
	pDC->LineTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapTopRight.y - nBoundrySize) );
	pDC->LineTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );


	// Map the "trigger" boundry
	nBoundrySize = (int)((double)MAP_BOUNDRY_SIZE * m_MC_MapZoom);
	pDC->MoveTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );	// Top edge
	pDC->LineTo(  (MC_MapTopRight.x - nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );
	pDC->LineTo(  (MC_MapTopRight.x - nBoundrySize),	(MC_MapTopRight.y - nBoundrySize) );
	pDC->LineTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapTopRight.y - nBoundrySize) );
	pDC->LineTo(  (MC_MapBottomLeft.x + nBoundrySize),	(MC_MapBottomLeft.y + nBoundrySize) );

	
	int RWGridSpacing = 1200; // inches (1200 TenthInches = 10 feet)
	if(m_MC_MapZoom >= ZoomPercent400)
	{
		RWGridSpacing = 120; // 1 foot spacing 
	}
	else if(m_MC_MapZoom >= ZoomPercent200)
	{
		RWGridSpacing = 600; // 5 foot spacing 
	}

	// Draw the Grid
	pDC->SelectObject( &penGridLines );
	int nRWLine;

	// Draw vertical lines
	pDC->SetTextAlign(TA_CENTER);
//	CSize OriginDbg = GetRWMapOrigin();

	RWFrom.y = m_RW_MapBottomLeft.y;		// Length of vertical lines
	RWTo.y   = m_RW_MapTopRight.y - 1;
	int RWLineStart;
	RWLineStart = m_RW_MapBottomLeft.x+RWGridSpacing;
	RWLineStart = RWLineStart - (RWLineStart % RWGridSpacing);
	for( nRWLine = RWLineStart; nRWLine < m_RW_MapTopRight.x; nRWLine+=RWGridSpacing )
//	for( nLine = LineStart; nLine < (GetMapOrigin().cx+MapSize.cx); nLine+=120 )
	{
		// draw a vertical line every RWGridSpacing inches (10 feet default)
		RWFrom.x = nRWLine;
		RWTo.x = nRWLine;
		MCFrom = TranslateRWtoMC(RWFrom);
		MCTo = TranslateRWtoMC(RWTo);
		pDC->MoveTo( MCFrom );
		pDC->LineTo( MCTo );
		strLabel.Format("%dft", (nRWLine/120));
		pDC->TextOut( MCFrom.x, (MCFrom.y + (TEXT_FONT_HEIGHT*2)), strLabel );
	}

	// Draw horizontal lines
	pDC->SetTextAlign(TA_LEFT);

	RWFrom.x = m_RW_MapBottomLeft.x;
	RWTo.x   = m_RW_MapTopRight.x - 1;
	RWLineStart = m_RW_MapBottomLeft.y + RWGridSpacing;
	RWLineStart = RWLineStart - (RWLineStart % RWGridSpacing);	// Start on even 10 foot boundry
	for( nRWLine = RWLineStart; nRWLine < m_RW_MapTopRight.y; nRWLine+=RWGridSpacing )
//	for( nLine = LineStart; nLine < (GetMapOrigin().cx+MapSize.cy); nLine+=120 )
	{
		// draw a line every 10 feet
		RWFrom.y = nRWLine;
		RWTo.y = nRWLine;
		MCFrom = TranslateRWtoMC(RWFrom);
		MCTo = TranslateRWtoMC(RWTo);
		pDC->MoveTo( MCFrom );
		pDC->LineTo( MCTo );

		strLabel.Format("%dft", (nRWLine/120));
		pDC->TextOut( MCFrom.x + (TEXT_FONT_HEIGHT) , (MCFrom.y + (TEXT_FONT_HEIGHT/2)), strLabel );
	}

	
	/////////////////////////////////////////////////////////////////////////////
	// Draw The GPS location "mouse trail"
	if( m_GPSEnabled )
	{

		int GPSFixMode;
		CGPSPointStructList* pGPSPointList = pDoc->GetGPSPointList();
		CGPSPointStruct* pGPSPointStruct;
		if( pGPSPointList != NULL )
		{
			//nCount = (WORD)pGPSPointList->GetCount();
			pos = pGPSPointList->GetHeadPosition();
			while (pos != NULL)
			{
				pGPSPointStruct = pGPSPointList->GetNext(pos);

				// Get the GPS info
				RWGPSpoint.x = pGPSPointStruct->m_Pos.x;
				RWGPSpoint.y = pGPSPointStruct->m_Pos.y;
				GPSFixMode   = pGPSPointStruct->m_FixMode;

				// Trap insane values
				if( RWGPSpoint.x > 0xFFFFF ) 
				{
					ROBOT_LOG( TRUE,  "WARNING! Bad RWGPSpoint.x = %d", RWGPSpoint.x );
					RWGPSpoint.x = 0;
				}
				if( RWGPSpoint.y > 0xFFFFF ) 
				{
					ROBOT_LOG( TRUE,  "WARNING! Bad RWGPSpoint.y = %d", RWGPSpoint.y );
					RWGPSpoint.y = 0;
				}

				// Check the point is within current Map.  Expand Map if needed
				UpdateBoundry( RWGPSpoint );

				// Translate from Real World to Map Coordinates, including inverting Y axis
				MCGPSpoint = TranslateRWtoMC(RWGPSpoint);

				// Select color/style to indicate fix quality
				if( 2 == GPSFixMode )
				{
					// Got 2D fix 
					pDC->SelectObject( &penGPS2D );
				}
				else if( 3 == GPSFixMode )		
				{
					// Got 3D fix
					pDC->SelectObject( &penGPS3D );
				}
				else if( 4 == GPSFixMode )		
				{
					// Special Flag - Resetting Robot position to GPS location!
					pDC->SelectObject( &penGPSNewPos );
				}
				else
				{
					// Bad fix?
					pDC->SelectObject( &penGPSLost );
				}	

				// Draw a GPS point Marker
////TODO				pDC->Ellipse( (int)(MCGPSpoint.x - ROBOT_BODY_DRAW_RADIUS_2), (int)(MCGPSpoint.y + ROBOT_BODY_DRAW_RADIUS_2), (int)(MCGPSpoint.x + (ROBOT_BODY_DRAW_RADIUS_2+1)), (int)(MCGPSpoint.y -(ROBOT_BODY_DRAW_RADIUS_2+1)) );
			}
		}
	}


	/////////////////////////////////////////////////////////////////////////////
	// Draw The Robot location "mouse trail"
	if( m_ShowRobotTrails )
	{
		CRobotTrailStructList* pRobotTrailList = pDoc->GetRobotTrailList();
		CRobotTrailStruct* pRobotTrailStruct;
		if( pRobotTrailList != NULL )
		{
			//nCount = (WORD)pRobotTrailList->GetCount();
			pDC->SelectObject( &penRobotMouseTrail );
			pos = pRobotTrailList->GetHeadPosition();
			while (pos != NULL)
			{
				pRobotTrailStruct = pRobotTrailList->GetNext(pos);

				// Get the position info
				RWRobot.x = pRobotTrailStruct->m_Pos.x;
				RWRobot.y = pRobotTrailStruct->m_Pos.y;
				
				// Check the point is within current Map.  Expand Map if needed
				UpdateBoundry( RWRobot );

				// Translate from Real World to Map Coordinates, including inverting Y axis
				MCRobot = TranslateRWtoMC(RWRobot);


				// Draw a Robot position point Marker
				pDC->Ellipse( (int)(MCRobot.x - (ROBOT_BODY_DRAW_RADIUS_2 * m_MC_MapZoom)+1.0), (int)((MCRobot.y + (ROBOT_BODY_DRAW_RADIUS_2 * m_MC_MapZoom))-1.0), 
					(int)(MCRobot.x + (ROBOT_BODY_DRAW_RADIUS_2 * m_MC_MapZoom)+1.0), (int)((MCRobot.y - (ROBOT_BODY_DRAW_RADIUS_2 * m_MC_MapZoom))-1.0) );
			}
		}
	}	
	
	
	/////////////////////////////////////////////////////////////////////////////
	// Draw the GridMap objects.  Includes Walls, etc.

	if( m_ShowGridMapSquares && (NULL != g_pGridMap) )
	{
		POINT CellLocation, DisplayLocation, MCPoint;
		int CellValue = GRIDMAP_OBJECT_NONE;
		int OldCellValue = GRIDMAP_OBJECT_NONE;
		pDC->SelectObject( &penWallSquare );
	//	CGridMap *pGridMap = pDoc->GetGridMap();
		POINT GridMapSize = g_pGridMap->GetMapSize();

		for( CellLocation.y = 0; CellLocation.y < GridMapSize.y; CellLocation.y++ )
		{
			for( CellLocation.x = 0; CellLocation.x < GridMapSize.x; CellLocation.x++ )
			{
				CellValue = g_pGridMap->GetGridMapCell(CellLocation);
				if( CellValue != GRIDMAP_OBJECT_NONE )
				{
					if( CellValue != OldCellValue )
					{
						// differnt type of object, need to change color
						if( GRIDMAP_OBJECT_LOW_OBSTACLE == CellValue )
						{
							pDC->SelectObject( &penLowObstacleSquare );
						}
						else if( GRIDMAP_OBJECT_OBSTACLE == CellValue )
						{
							pDC->SelectObject( &penObstacleSquare );
						}
						else if( GRIDMAP_OBJECT_WALL == CellValue )
						{
							pDC->SelectObject( &penWallSquare );
						}
						else if( GRIDMAP_OBJECT_CLIFF == CellValue )
						{
							pDC->SelectObject( &penCliffSquare );
						}
						else if( GRIDMAP_OBJECT_NAVIGATION_FLAG == CellValue )		// Show the CALCULATED PATH!
						{
							pDC->SelectObject( &penSegment );	// Use same colors as Segments for other kind of path
						}

						else if( CellValue > 5 )
						{
							ROBOT_LOG( TRUE,  "ERROR - MapView: Bad CellValue Type ( > 5) = %d\n", CellValue);
						}
						else if( CellValue >= 4 )
						{
							pDC->SelectObject( &penCostSquare4 );
						}
						else if( CellValue >= 3 )
						{
							pDC->SelectObject( &penCostSquare3 );
						}
						else if( CellValue >= 2 )
						{
							pDC->SelectObject( &penCostSquare2 );
						}
						else
						{
							ROBOT_LOG( TRUE,  "ERROR - MapView: Bad CellValue Type ( > 2) = %d\n", CellValue);
						}

					}

					DisplayLocation.x = (long)(CellLocation.x * GRID_MAP_RESOLUTION);	// Scale by number of inches per cell
					DisplayLocation.y = (long)(CellLocation.y * GRID_MAP_RESOLUTION);

					// Object is within current Map.  Expand Map if needed
					UpdateBoundry( DisplayLocation );

					// Translate from Real World to Map Coordinates, including inverting Y axis
					MCPoint = TranslateRWtoMC(DisplayLocation);
	//				MCTo.x = MCFrom.x + 5;
	//				MCTo.y = MCFrom.y;
					int  SquareSize = (int )(HALF_GRID_MAP_RESOLUTION * m_MC_MapZoom);

					// Draw the square
					pDC->MoveTo(  (MCPoint.x - SquareSize), (MCPoint.y + SquareSize) );
					pDC->LineTo(  (MCPoint.x + SquareSize), (MCPoint.y + SquareSize) );
					pDC->LineTo(  (MCPoint.x + SquareSize), (MCPoint.y - SquareSize) );
					pDC->LineTo(  (MCPoint.x - SquareSize), (MCPoint.y - SquareSize) );
					pDC->LineTo(  (MCPoint.x - SquareSize), (MCPoint.y + SquareSize) );
				}
			}
		}
	}
	// Draw current stroke in progress (already in MC units)
	if( m_DoingLongStroke )
	{
		pDC->SelectObject( &penBox );
		pDC->MoveTo( m_ptPrev );
		pDC->LineTo( m_ptCurrent );
	}


	/////////////////////////////////////////////////////////////////////////////
	// Draw Strokes (includes graphics such as landscape or house plans)

	CTypedPtrList<CObList, CStroke*> *strokeList;
	unsigned long StrokeColor;

/**
	CTypedPtrList<CObList, CStroke*> *UserList = &(pDoc->m_strokeList);
	CTypedPtrList<CObList, CStroke*> *DoorList = &(pDoc->m_DoorList);
	CTypedPtrList<CObList, CStroke*> *LowObstacleList = &(pDoc->m_LowObstacleList);
	CTypedPtrList<CObList, CStroke*> *ObstacleList = &(pDoc->m_ObstacleList);
	CTypedPtrList<CObList, CStroke*> *WallList = &(pDoc->m_WallList);
	CTypedPtrList<CObList, CStroke*> *CliffList = &(pDoc->m_CliffList);
**/

	for(int i=0; i<=5; i++)
	{
		// Note - i does not need to map to the stroke type, it's just an iterator
		// in general draw from solid objects to less solid
		if( 0 == i)	// Draw Walls 
		{	
			strokeList = &pDoc->m_WallList;
			StrokeColor = WALL_LINE_COLOR;
		}
		else if( 1 == i)	// Draw Cliffs
		{	
			strokeList = &pDoc->m_CliffList;
			StrokeColor = CLIFF_LINE_COLOR;
		}
		else if( 2 == i)	// Draw Obstacles
		{	
			strokeList = &pDoc->m_ObstacleList;
			StrokeColor = OBSTACLE_LINE_COLOR;
		}
		else if( 3 == i)	// Draw Obstacles
		{	
			strokeList = &pDoc->m_LowObstacleList;
			StrokeColor = OBSTACLE_LINE_COLOR;
		}
		else if( 4 == i)	// Draw Obstacles
		{	
			strokeList = &pDoc->m_DoorList;
			StrokeColor = DOOR_LINE_COLOR;
		}
		else if( 5 == i)	// Draw User scribbles
		{	
			strokeList = &pDoc->m_strokeList;
			StrokeColor = USER_STROKE_LINE_COLOR;
		}
		else
		{
			ROBOT_LOG( TRUE,  "MapDoc.cpp - Bad i in Draw Strokes!");
			ROBOT_ASSERT(0);
		}

		pos = strokeList->GetHeadPosition( );
		while (pos != NULL)
		{
			CStroke* pStroke = strokeList->GetNext(pos);
			rectStroke = pStroke->GetBoundingRect();

			//CPoint temp = rectStroke.BottomRight();
			//temp.x *= 10; temp.y *= 10;
			//rectStroke.BottomRight() = TranslateRWtoMC(temp);

			// convert from RealWorld to Map coordinates

			rectStroke.BottomRight() =	TranslateRWtoMC(rectStroke.BottomRight() );
			rectStroke.TopLeft() =	TranslateRWtoMC(rectStroke.TopLeft() );

			//rectStroke.BottomRight() =	TranslateRWtoMC(rectStroke.BottomRight() );
			//rectStroke.TopLeft() =	TranslateRWtoMC(rectStroke.TopLeft() );

			pDC->LPtoDP(&rectStroke);
			rectStroke.InflateRect(1, 1);

			if (!rectStroke.IntersectRect(&rectStroke, &rectClip))
			{
	// TODO!!			continue;	// skip anything out of the change rectangle
			}


			// Draw the strokes as a series of line segments
			int  StrokePenWidth = pStroke->GetPenWidth();
			CPen penStroke;
			if( !penStroke.CreatePen(PS_SOLID, StrokePenWidth, StrokeColor) ) 
			{
				ROBOT_LOG( TRUE,  "MapView: Error creating Dynamic Stroke Pen!\n" );
				return;
			}
			CPen* pOldStrokePen = pDC->SelectObject( &penStroke );


			POINT MCPoint = TranslateRWtoMC( pStroke->m_pointArray[0] );
			pDC->MoveTo( MCPoint );
			for( int i=1; i < pStroke->m_pointArray.GetSize(); i++ )
			{
				MCPoint = TranslateRWtoMC( pStroke->m_pointArray[i] );
				pDC->LineTo( MCPoint );
			}

			pDC->SelectObject( pOldStrokePen );	// Restore prior pen!
		}
	}

	/////////////////////////////////////////////////////////////////////////////
	// Draw the planed Path:  Waypoints and Segments


	/////////////////////////////////////////////////////////////////////////////
	// SEGMENTS
	POSITION SegPos;
	//pDC->SetTextAlign(TA_CENTER);
	BOOL WaypointFound = FALSE;


	pDC->SelectObject( &penSegment );
	pDC->SetTextAlign(TA_CENTER);
	if( g_pSegmentList != NULL )
	{
		//nCount = (WORD)g_pSegmentList->GetCount();
		SegPos = g_pSegmentList->GetHeadPosition();
		while (SegPos != NULL)
		{
			CSegmentStruct* pSegmentStruct = g_pSegmentList->GetNext(SegPos);

			// Got a Segment
			// Find Starting Waypoint data
			RWFrom.x = -1; RWFrom.y = -1; RWTo.x = -1; RWTo.y = -1;
			if( g_pWaypointList != NULL )
			{
				pos = g_pWaypointList->GetHeadPosition();
				while (pos != NULL)
				{
					CWaypointStruct* pWaypointStruct = g_pWaypointList->GetNext(pos);

					if( pWaypointStruct->m_WaypointID == pSegmentStruct->m_SegmentFromWaypointID )
					{
						RWFrom.x = ((pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX) * 10; // convert to TenthInches
						RWFrom.y = ((pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY) * 10; // convert to TenthInches
					}
					if( pWaypointStruct->m_WaypointID == pSegmentStruct->m_SegmentToWaypointID )
					{
						RWTo.x = ((pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX) * 10; // convert to TenthInches
						RWTo.y = ((pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY) * 10; // convert to TenthInches
					}
				}
			}
			if( (-1 == RWFrom.x) || (-1 == RWFrom.y) )
			{
				strLabel.Format("MapView: From Waypoint ID is not valid for segment %s\n", pSegmentStruct->m_SegmentName);
				ROBOT_LOG( TRUE, strLabel);
			}
			else if( (-1 == RWTo.x) || (-1 == RWTo.y) )
			{
				strLabel.Format("MapView: To Waypoint ID is not valid for segment %s\n", pSegmentStruct->m_SegmentName);
				ROBOT_LOG( TRUE, strLabel);
			}
			else
			{
				// Check both ends of segment are within current Map.  Expand Map if needed
				UpdateBoundry( RWFrom );
				UpdateBoundry( RWTo );

				// Translate from Real World to Map Coordinates, including inverting Y axis
				MCFrom = TranslateRWtoMC(RWFrom);
				MCTo = TranslateRWtoMC(RWTo);

				// Draw the line segment
				pDC->MoveTo( MCFrom );
				pDC->LineTo( MCTo );
				//  Text Anchor Point        1/2 line length
				X = MCFrom.x + ((MCTo.x - MCFrom.x) /2);
				Y = MCFrom.y + ((MCTo.y - MCFrom.y) /2);

				strLabel.Format("Segment %s", pSegmentStruct->m_SegmentName);
				pDC->TextOut( X, (Y + TEXT_FONT_HEIGHT*2), strLabel );

				strLabel.Format("From: %02u To:%02u",
					pSegmentStruct->m_SegmentFromWaypointID, pSegmentStruct->m_SegmentToWaypointID);
				pDC->TextOut( X, (Y + TEXT_FONT_HEIGHT), strLabel );

				if( 0 == pSegmentStruct->m_CompassCorrection )
				{
					strLabel.Format("%d Deg  %d ft", 
						pSegmentStruct->m_SegmentDirection,	pSegmentStruct->m_SegmentDistanceFeet );
				}
				else
				{	// Show compass correction for this segment
					strLabel.Format("%d (%+d) Deg  %d ft", pSegmentStruct->m_SegmentDirection,	
						pSegmentStruct->m_CompassCorrection, pSegmentStruct->m_SegmentDistanceFeet );
				}
				pDC->TextOut( X, Y, strLabel );


				strLabel.Format("%s", 
					pSegmentStruct->m_SegmentBehavior );
				pDC->TextOut( X, (Y - (TEXT_FONT_HEIGHT)), strLabel );
			}
		}
	}
	pDC->SetTextAlign(TA_LEFT);

	// WAYPOINTS
	if( g_pWaypointList != NULL )
	{
		//nCount = (WORD)g_pWaypointList->GetCount();
		pos = g_pWaypointList->GetHeadPosition();
		while (pos != NULL)
		{
			CWaypointStruct* pWaypointStruct = g_pWaypointList->GetNext(pos);

			// Display the Waypoint info
			RWWaypoint.x = ((pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX) * 10; // convert to TenthInches
			RWWaypoint.y = ((pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY) * 10; // convert to TenthInches

			
			// Check the Waypoint is within current Map.  Expand Map if needed
			UpdateBoundry( RWWaypoint );

			// Translate from Real World to Map Coordinates, including inverting Y axis
			MCWaypoint = TranslateRWtoMC(RWWaypoint);

			// Draw a Waypoint Marker
			pDC->SelectObject( &penWaypointFinderLine );
			pDC->MoveTo(  (5), (MCWaypoint.y) );	// Draw a marker line
			pDC->LineTo(  (MCWaypoint.x - WAYPOINT_BOX_SIZE), (MCWaypoint.y) );	// Draw a marker line

			pDC->SelectObject( &penWaypoint );
			pDC->MoveTo(  (MCWaypoint.x - WAYPOINT_BOX_SIZE), (MCWaypoint.y + WAYPOINT_BOX_SIZE) );	// Draw a marker line
			pDC->LineTo(  (MCWaypoint.x + WAYPOINT_BOX_SIZE), (MCWaypoint.y + WAYPOINT_BOX_SIZE) );
			pDC->LineTo(  (MCWaypoint.x + WAYPOINT_BOX_SIZE), (MCWaypoint.y - WAYPOINT_BOX_SIZE) );
			pDC->LineTo(  (MCWaypoint.x - WAYPOINT_BOX_SIZE), (MCWaypoint.y - WAYPOINT_BOX_SIZE) );
			pDC->LineTo(  (MCWaypoint.x - WAYPOINT_BOX_SIZE), (MCWaypoint.y + WAYPOINT_BOX_SIZE) );


			// Put Finder Label near the left edge
			strLabel.Format("WP-%02u  %s", 
				pWaypointStruct->m_WaypointID, 
				pWaypointStruct->m_WaypointName );

			pDC->TextOut( 5,(MCWaypoint.y + (TEXT_FONT_HEIGHT/2)), strLabel );

			// Print the Waypoint label near the waypoint
/*
			strLabel.Format("WP-%02u  X=%u\' %u\" Y=%u\' %u\"  %s", 
				pWaypointStruct->m_WaypointID, 
				pWaypointStruct->m_WaypointLocationFeetX, 
				pWaypointStruct->m_WaypointLocationInchesX, 
				pWaypointStruct->m_WaypointLocationFeetY, 
				pWaypointStruct->m_WaypointLocationInchesY, 
				pWaypointStruct->m_WaypointName );

			pDC->TextOut( MCWaypoint.x + (WAYPOINT_BOX_SIZE*4),(MCWaypoint.y + (TEXT_FONT_HEIGHT/2)), strLabel );
*/
			// display the Waypoint ID and name
			strLabel.Format("%u: %s", 
				pWaypointStruct->m_WaypointID, 
				pWaypointStruct->m_WaypointName );
			pDC->TextOut( MCWaypoint.x + (WAYPOINT_BOX_SIZE*4),(MCWaypoint.y - (TEXT_FONT_HEIGHT)), strLabel );

			
			// display the Waypoint Landmark Type
			strLabel.Format("Landmark: %s", pWaypointStruct->m_WaypointLandmarkType1); 
			pDC->TextOut( MCWaypoint.x + (WAYPOINT_BOX_SIZE*4),(MCWaypoint.y - (TEXT_FONT_HEIGHT*2)), strLabel );

			
			// display the Waypoint location
			strLabel.Format("X=%u\' %u\" Y=%u\' %u\" ",
				pWaypointStruct->m_WaypointLocationFeetX, 
				pWaypointStruct->m_WaypointLocationInchesX, 
				pWaypointStruct->m_WaypointLocationFeetY, 
				pWaypointStruct->m_WaypointLocationInchesY);
			pDC->TextOut( MCWaypoint.x + (WAYPOINT_BOX_SIZE*4),(MCWaypoint.y - (TEXT_FONT_HEIGHT*3)), strLabel );

			//pDC->TextOut( X, Y, strLabel );

			// Show Landmarks
			if( pWaypointStruct->m_WaypointLandmarkType1 )
			{
			}

		}
	}


	/////////////////////////////////////////////////////////////////////////////
	// Draw all the sensor readings the robot recorded

	if( m_ShowSensorReadings )
	{
		int nDebugSensor  = 0;
		int SensorNumber = 0;
		CMapSensorStructList* pMapSensorList = pDoc->GetMapSensorList();
		CMapSensorStruct* pMapSensorStruct;
		if( pMapSensorList != NULL )
		{
			nDebugSensor++;
			pos = pMapSensorList->GetHeadPosition();
			while (pos != NULL)
			{
				pMapSensorStruct = pMapSensorList->GetNext(pos);

				// Get the location Robot was when sensor info collected
				RWRobot.x = pMapSensorStruct->m_RWPos.x;
				RWRobot.y = pMapSensorStruct->m_RWPos.y;
				
				// Check the location point is within current Map.  Expand Map if needed
				UpdateBoundry( RWRobot );

				// Translate from Real World to Map Coordinates, including inverting Y axis
				MCRobot = TranslateRWtoMC(RWRobot);

				if( pMapSensorStruct->m_Heading <= 360 )
				{
					// Paint the IR sensor readings
					pDC->SelectObject( &penIRSensor );	// IR Sensors
					for( SensorNumber = 0; SensorNumber < NUM_IR_SENSORS; SensorNumber++ )
					{
						if( pMapSensorStruct->IR[SensorNumber] < NO_OBJECT_IN_RANGE )	// Don't paint out of range sensor values
						{
							PaintSensor( pDC, RWRobot, pMapSensorStruct->m_Heading, SensorOffsetDegrees_IR[SensorNumber], 
								SENSOR_HALF_FOV_DEGREES_IR, pMapSensorStruct->IR[SensorNumber] );
						}
					}

					// Paint the UltraSonic sensor readings
					pDC->SelectObject( &penUSSensor );	// Ultrasonic Sensors
					for( SensorNumber = 0; SensorNumber < NUM_US_SENSORS; SensorNumber++ )
					{
						if( pMapSensorStruct->US[SensorNumber] < NO_OBJECT_IN_RANGE )	// Don't paint out of range sensor values
						{
							PaintSensor( pDC, RWRobot, pMapSensorStruct->m_Heading, SensorOffsetDegrees_US[SensorNumber], 
								SENSOR_HALF_FOV_DEGREES_US, pMapSensorStruct->US[SensorNumber] );
						}
					}
				}
				else
				{
					ROBOT_LOG( TRUE,  "WARNING - Map View: Recorded Sensor Compass exceeds 360 degrees!\n" );
				}
			}
		}
	}



	/////////////////////////////////////////////////////////////////////////////
	// Draw the curent Target point to move to, as selected by the user
	// Indicate to the user the target location
	// Draw a temporary X on the screen (erases on next update)
	#define SIZE_OF_TARGET	8.0
	int CrossSize = (int)(SIZE_OF_TARGET * m_MC_MapZoom);

	// first, see if we have arrived at the target. If so, erase the target
	if( (abs(m_RW_TargetPoint.x - g_pFullSensorStatus->CurrentLocation.x) < 120) &&	// within 1 foot of target
		(abs(m_RW_TargetPoint.y - g_pFullSensorStatus->CurrentLocation.y) < 120) )
	{
		// Close enough, erase the target
		m_RW_TargetPoint.x = 0;
		m_RW_TargetPoint.y = 0;
	}

	// If target is defined, draw it
	if( (0 != m_RW_TargetPoint.x) && (0 != m_RW_TargetPoint.y) )
	{
		pDC->SelectObject( &penRobotIcon );
		POINT MCTargetPt = TranslateRWtoMC(m_RW_TargetPoint);

		pDC->MoveTo( MCTargetPt.x-CrossSize, MCTargetPt.y+CrossSize );	// Top Left
		pDC->LineTo( MCTargetPt.x+CrossSize, MCTargetPt.y-CrossSize );	// Bottom Right
		pDC->MoveTo( MCTargetPt.x+CrossSize, MCTargetPt.y+CrossSize );	// Top Right
		pDC->LineTo( MCTargetPt.x-CrossSize, MCTargetPt.y-CrossSize );	// Bottom Left
	}


	/////////////////////////////////////////////////////////////////////////////
	// Draw the curent position of the Robot, as indicated by ER1 Stepper Motors (differential drive)
	/*** DOES NOT WORK RIGHT NOW
	if(	(0 != g_pFullSensorStatus->CurrentLocationMotor.x) &&	(0 != g_pFullSensorStatus->CurrentLocationMotor.y) )
	{
		// Check the Robot is within current Map.  Expand Map if needed
		UpdateBoundry( g_pFullSensorStatus->CurrentLocationMotor );

		// Translate from Real World to Map Coordinates, including inverting Y axis
		MCRobot = TranslateRWtoMC( g_pFullSensorStatus->CurrentLocationMotor );

		// Assumes GDI orientation, with Y=0 at the top
		pDC->SelectObject( &penRobotER1MotorTrail );
		pDC->Ellipse( (int)(MCRobot.x - (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom)+1.0), (int)((MCRobot.y + (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom))-1.0), 
			(int)(MCRobot.x + (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom)+1.0), (int)((MCRobot.y - (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom))-1.0) );
	}
	***/

	/////////////////////////////////////////////////////////////////////////////
	// Draw the curent position of the Robot, as indicated by GPS
	if( m_GPSEnabled )
	{
		if(	(g_GPSGotFirstFix) &&	// wait until we get some good data from the GPS
			((-1 != gGPSOriginLat) || (-1 != gGPSOriginLong)) ) // Need anchor point
		{
			// Check the Robot is within current Map.  Expand Map if needed
			UpdateBoundry( g_pFullSensorStatus->CurrentLocationGPS );

			// DRAW GPS
			// Translate from Real World to Map Coordinates, including inverting Y axis
			if( !((0 == g_pFullSensorStatus->CurrentLocationGPS.x) && (0 == g_pFullSensorStatus->CurrentLocationGPS.y)) )
			{
				// Only draw if GPS connected
				POINT rwCurrentLocationGPS = { (LONG)(g_pFullSensorStatus->CurrentLocationGPS.x), (LONG)(g_pFullSensorStatus->CurrentLocationGPS.y) };
				MCRobot = TranslateRWtoMC( rwCurrentLocationGPS );
				// Assumes GDI orientation, with Y=0 at the top
				pDC->SelectObject( &penRobotIcon );
				pDC->Ellipse( (int)(MCRobot.x - (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom)+1.0), (int)((MCRobot.y + (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom))-1.0), 
					(int)(MCRobot.x + (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom)+1.0), (int)((MCRobot.y - (ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom))-1.0) );
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////
	// Draw the curent position of the Robot, as indicated by Compass and Odometer
	if(	(0 != g_pFullSensorStatus->CurrentLocation.x) &&	(0 != g_pFullSensorStatus->CurrentLocation.y) )
	{
		// Check the Robot is within current Map.  Expand Map if needed
		UpdateBoundry( g_pFullSensorStatus->CurrentLocation );

		// Translate from Real World to Map Coordinates, including inverting Y axis
		//RWRobot = g_pFullSensorStatus->CurrentLocation;

		RWRobot.x = (LONG)(g_pFullSensorStatus->CurrentLocation.x);
		RWRobot.y = (LONG)(g_pFullSensorStatus->CurrentLocation.y);


		MCRobot = TranslateRWtoMC( RWRobot );
		// Assumes GDI orientation, with Y=0 at the top
		pDC->SelectObject( &penRobotIcon );

		LONG Radius = (LONG)(ROBOT_BODY_DRAW_RADIUS_3 * m_MC_MapZoom);
		pDC->Ellipse( (MCRobot.x - Radius), (MCRobot.y + Radius), MCRobot.x + Radius, MCRobot.y - Radius);

		Radius = (LONG)(ROBOT_BODY_DRAW_RADIUS_2 * m_MC_MapZoom);
		pDC->Ellipse( (MCRobot.x - Radius), (MCRobot.y + Radius), MCRobot.x + Radius, MCRobot.y - Radius);

		Radius = (LONG)(ROBOT_BODY_DRAW_RADIUS_1 * m_MC_MapZoom);
		pDC->Ellipse( (MCRobot.x - Radius), (MCRobot.y + Radius), MCRobot.x + Radius, MCRobot.y - Radius);

		// Draw direction indicator
		if( g_pFullSensorStatus->CompassHeading <= 360 )
		{
			// Convert Degrees to Radians then do the math
			//double TwoPi = 2.0 * 3.141592;
			double DirectionRadians = (double)g_pFullSensorStatus->CompassHeading * DEGREES_TO_RADIANS;
			dX = HEADING_LINE_LEN * m_MC_MapZoom * sin( DirectionRadians );	// Returns negative numbers as needed
			dY = HEADING_LINE_LEN * m_MC_MapZoom * cos( DirectionRadians );
			
			if(dX>=0) dX += 0.5; else dX -= 0.5;	// Cast will truncate, this will round instead
			if(dY>=0) dY += 0.5; else dY -= 0.5;
			NewX = (int)MCRobot.x + (int)dX;		// Get new Map absolute X,Y
			NewY = (int)MCRobot.y + (int)dY;

			pDC->MoveTo(  MCRobot.x, MCRobot.y );
			pDC->LineTo(  NewX, NewY );
		}
		else
		{
			ROBOT_LOG( TRUE,  "WARNING - Map View: Compass exceeds 360 degrees!\n" );

		}


		//Draw the CURRENT Sensor readings

		int SensorNumber = 0;
		// Paint the IR sensor readings
		pDC->SelectObject( &penIRSensor );	// IR Sensors
		for( SensorNumber = 0; SensorNumber < NUM_IR_SENSORS; SensorNumber++ )
		{
			if( g_pFullSensorStatus->IR[SensorNumber] < NO_OBJECT_IN_RANGE )	// Don't paint out of range sensor values
			{
				PaintSensor( pDC, RWRobot, g_pFullSensorStatus->CompassHeading, SensorOffsetDegrees_IR[SensorNumber], SENSOR_HALF_FOV_DEGREES_IR, g_pFullSensorStatus->IR[SensorNumber] );
			}
		}

		// Paint the UltraSonic sensor readings
		pDC->SelectObject( &penUSSensor );	// Ultrasonic Sensors
		for( SensorNumber = 0; SensorNumber < NUM_US_SENSORS; SensorNumber++ )
		{
			if( g_pFullSensorStatus->US[SensorNumber] < NO_OBJECT_IN_RANGE )	// Don't paint out of range sensor values
			{
				PaintSensor( pDC, RWRobot, g_pFullSensorStatus->CompassHeading, SensorOffsetDegrees_US[SensorNumber], SENSOR_HALF_FOV_DEGREES_US, g_pFullSensorStatus->US[SensorNumber] );
			}
		}

	}

	/////////////////////////////////////////////////////////////////////////////
	// Done with drawing. Restore the Device Context
	pDC->SelectObject( pOldPen );	

	if (pOldFont != NULL)
	{
		pDC->SelectObject(pOldFont);
	}

}




/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// CMapView printing

BOOL CMapView::OnPreparePrinting(CPrintInfo* pInfo)
{
	pInfo->SetMaxPage(2);  // the document is two pages long:
	// the first page is the title page, the second page is the drawing

	BOOL bRet = DoPreparePrinting (pInfo);	// default preparation

	pInfo->m_nNumPreviewPages = 2;		//Preview 2 pages at a time
	// Set this value after calling DoPreparePrinting to override
	// value read from registry
	return bRet;

}

void CMapView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CMapView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}

/////////////////////////////////////////////////////////////////////////////
// CMapView diagnostics

#ifdef _DEBUG
void CMapView::AssertValid() const
{
	CScrollView::AssertValid();
}

void CMapView::Dump(CDumpContext& dc) const
{
	CScrollView::Dump(dc);
}

CMapDoc* CMapView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMapDoc)));
	ASSERT_VALID(m_pDocument);
	return (CMapDoc*)m_pDocument;
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CMapView message handlers


// MOUSE BUTTON HANDLERS
void CMapView::OnLButtonDown(UINT nFlags, CPoint point) 
{
	IGNORE_UNUSED_PARAM (nFlags);

	// Convert from device coordinates to real world coordinates to store in the document.
	CClientDC dc(this);
	OnPrepareDC(&dc); // set up mapping mode and viewport origin
	dc.DPtoLP(&point);// Device Point to Logical Point

	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	POINT RWPoint = TranslateMCtoRW(point);	// Real World coord


	if( (DRAW_MODE_SET_LOCATION == pDoc->GetDrawMode()) )
	{
		// Reset Robot's view of the world, to think it is where the user clicked.
		// Good for fixing the robot when it is confused! :-)
		SendCommand( WM_ROBOT_SET_CURRENT_LOCATION, RWPoint.x, RWPoint.y);
		// Force the global value to be updated immediately
		g_pFullSensorStatus->CurrentLocation.x = RWPoint.x;	// Get new Map absolute X,Y
		g_pFullSensorStatus->CurrentLocation.y = RWPoint.y;
		g_pFullSensorStatus->CurrentLocationMotor.x = RWPoint.x;	// Get new Map absolute X,Y
		g_pFullSensorStatus->CurrentLocationMotor.y = RWPoint.y;


	}
	else if( (DRAW_MODE_WAYPOINT_LOCATION == pDoc->GetDrawMode()) )
	{
		// Add a Waypoint each time user clicks either mouse button

		::PostMessage( g_RobotPathViewHWND, (int )WM_COMMAND, ID_SEL_PATH_VIEW_BTN, 0);	
		::PostMessage( g_RobotPathViewHWND, (int )(WM_ROBOT_ADD_WAYPOINT_LOCATION), 
			RWPoint.x, RWPoint.y);	
	}
	else if( (DRAW_MODE_NAVIGATION_GOAL == pDoc->GetDrawMode()) )
	{
		// Tell robot to find a path to this location!
		DWORD Param = (DWORD)MAKELONG(RWPoint.x, RWPoint.y); //LOWORD, HIWORD

		SendCommand( WM_ROBOT_GOTO_GRID_LOCATION_CMD, Param, 0 );
		SendCommand( WM_ROBOT_EXECUTE_PATH, GRID_PATH_EXECUTE_START,	0 );	
		// lParam: 1 = Wait for bumper to start, 0 = Don't wait

		// Indicate to the user the target location
		// Yes, the math is funky, but this is how the path finder does it, and we want exact same
		double X = RWPoint.x + 0.5; // Straight integer cast will truncate, this will round instead
		double Y = RWPoint.y + 0.5;
		m_RW_TargetPoint.x = (LONG)(X / GRID_MAP_RESOLUTION);
		m_RW_TargetPoint.y = (LONG)(Y / GRID_MAP_RESOLUTION);
		m_RW_TargetPoint.x *= (LONG)GRID_MAP_RESOLUTION;
		m_RW_TargetPoint.y *= (LONG)GRID_MAP_RESOLUTION;

/***
#define WAYPOINT_OFFICE_X				341
#define WAYPOINT_OFFICE_Y				248

#define WAYPOINT_MASTER_BEDROOM_X		282
#define WAYPOINT_MASTER_BEDROOM_Y		580

#define WAYPOINT_HEATHER_BEDROOM_X		291
#define WAYPOINT_HEATHER_BEDROOM_Y		448

#define WAYPOINT_AMBER_BEDROOM_X		466
#define WAYPOINT_AMBER_BEDROOM_Y		236

#define WAYPOINT_MASTER_BATHROOM_X		368
#define WAYPOINT_MASTER_BATHROOM_Y		655

***/

	}
	else
	{

		if( !m_DoingLongStroke )
		{
			// Pressing the mouse button in the view window starts a new stroke.
			m_pStrokeCur = pDoc->NewStroke( pDoc->GetDrawMode() );

			// Add first point to the new stroke
			if( RWPoint.x < 0) RWPoint.x = 0;	// trap bad values
			if( RWPoint.y < 0) RWPoint.y = 0;	
			m_pStrokeCur->m_pointArray.Add(RWPoint); 

			SetCapture( );  // Capture the mouse until button up

			// This will be the MoveTo( ) anchor point for the LineTo() 
			// the next point, as the user drags the mouse
			m_ptPrev = point;	
			m_ptCurrent = point;
		}
		else
		{
			// We are doing a long stroke.  Ignore button down
		}
	}
	pDoc->UpdateAllViews(NULL, 0L, NULL);	// Force view update

	// replace base class completely:	
	// CView::OnLButtonDown(nFlags, point);	
}


void CMapView::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// Mouse button up is only interesting if the user is 
	// currently drawing a new stroke by dragging the captured mouse.
	IGNORE_UNUSED_PARAM (nFlags);

	if( GetCapture( ) != this )
	{
		// If this window (view) didn't capture the mouse,
		// the user isn't drawing in this window.
		return;    
	}
	else if(( DRAW_MODE_WAYPOINT_LOCATION == GetDocument()->GetDrawMode() ) ||
			( DRAW_MODE_NAVIGATION_GOAL   == GetDocument()->GetDrawMode() )	)
	{		
		return;    // ignore button up
	}

	CMapDoc* pDoc = GetDocument();
	CClientDC dc( this );
	int  DrawMode = pDoc->GetDrawMode();

	// CScrollView changes the viewport origin and mapping mode.
	// It's necessary to convert the point from device coordinates
	// to logical coordinates, such as are stored in the document.
	OnPrepareDC(&dc); // set up mapping mode and viewport origin
	dc.DPtoLP(&point);
	POINT RWPoint = TranslateMCtoRW(point);	// Real World coord

	CPen* pOldPen = dc.SelectObject( pDoc->GetCurrentPen( ) );
	dc.MoveTo( m_ptPrev );
	dc.LineTo( point );
	dc.SelectObject( pOldPen );
	if( RWPoint.x < 0) RWPoint.x = 0;	// trap bad values
	if( RWPoint.y < 0) RWPoint.y = 0;	
	m_pStrokeCur->m_pointArray.Add(RWPoint);


	// Tell the stroke item that we're done adding points to it.
	// This is so it can finish computing its bounding rectangle.
	m_pStrokeCur->FinishStroke();

	// Now, if this is an Wall or Object, add to the local GridMap
	g_pGridMap->AddGridMapData( m_pStrokeCur->m_pointArray, pDoc->GetGridmapObjectType(DrawMode) );


	// And, if this is the client, send the data to the server
	#if ( ROBOT_SERVER != 1 )	// Client only

		POINT* pPointArray = (POINT*)g_ClientBulkData;	// point to the global buffer

		// Validate global buffer is big enough!
		int nStorageNeeded = m_pStrokeCur->m_pointArray.GetSize() * sizeof(POINT);
		if( nStorageNeeded > BULK_DATA_SIZE )
		{
			ROBOT_LOG( TRUE,  "Too much storage needed (%d bytes) for Global Buffer\n", nStorageNeeded);
			AfxMessageBox( _T("Too Many strokes for Global Buffer!") );	
		}
		else
		{
			for( int i=0; i < m_pStrokeCur->m_pointArray.GetSize(); i++ )
			{
				POINT tmpPoint = m_pStrokeCur->m_pointArray[i];
				*pPointArray = tmpPoint; //m_pStrokeCur->m_pointArray[i];
				pPointArray++;
			}
			g_ClientBulkDataLength = nStorageNeeded;	// Tell socket thread how much data is in the buffer!
			// Send to the Server
			SendCommand( WM_ROBOT_BULK_DATA_TO_SERVER,
					BULK_DATA_TYPE_MAP_STROKE,
					DrawMode ); 	// Type of stroke being drawn, eg, WALL
		}
	#endif	// Not ROBOT_SERVER

	// Tell the other views that this stroke has been added
	// so that they can invalidate this stroke's area in their
	// client area.
//	pDoc->UpdateAllViews(this, 0L, m_pStrokeCur);
//	pDoc->UpdateAllViews(this, 0L, NULL);
	pDoc->UpdateAllViews(NULL, 0L, this);
//	pDoc->UpdateAllViews(NULL, 0L, NULL);


	// Release the mouse capture established at the beginning of the mouse drag.
	ReleaseCapture( );    

	m_DoingLongStroke = FALSE;	// End the long stroke if we were doing one

	pDoc->UpdateAllViews(NULL, 0L, NULL);	// Force view update
	// replace base class completely:	
	// CView::OnLButtonUp(nFlags, point);
}

///////////////////////////////////////////////////////////////////
// WM_RBUTTONDOWN handler.
//
// Trap this message and display the Button Properties pop-up menu.
// The main frame receives the pop-up menu messages. This allows the
// status bar to be updated with the help text.
///////////////////////////////////////////////////////////////////

/*
// Drawing Modes
#define DRAW_MODE_USER_STROKE		0x00
#define DRAW_MODE_DOOR				0x01
#define DRAW_MODE_LOW_OBSTACLE		0x02
#define DRAW_MODE_OBSTACLE			0x03
#define DRAW_MODE_WALL				0x04
#define DRAW_MODE_CLIFF				0x05
#define DRAW_MODE_WAYPOINT_LOCATION	0x06
#define DRAW_MODE_NAVIGATION_GOAL	0x07
#define DRAW_MODE_SET_LOCATION		0x08	// Set current / start locatio of Robot
#define NUMBER_OF_DRAW_MODES		0x09	// This should be number of types listed above
*/

void CMapView::OnRButtonDown(UINT flags, CPoint point)
{
	// Pressing the RIGHT mouse button in the view window starts a new long line Stroke.
	IGNORE_UNUSED_PARAM (flags);

	CClientDC dc(this);
	OnPrepareDC(&dc); // set up mapping mode and viewport origin
	dc.DPtoLP(&point);// Device Point to Logical Point

	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	POINT RWPoint = TranslateMCtoRW(point);	// Real World coord

	if( (DRAW_MODE_WAYPOINT_LOCATION == pDoc->GetDrawMode()) )
	{
		// Add a Waypoint each time user clicks the left mouse button

		::PostMessage( g_RobotPathViewHWND, (int )WM_COMMAND, ID_SEL_PATH_VIEW_BTN, 0);	
		::PostMessage( g_RobotPathViewHWND, (int )(WM_ROBOT_ADD_WAYPOINT_LOCATION), 
			RWPoint.x, RWPoint.y);	
	}
	else if( (DRAW_MODE_NAVIGATION_GOAL == pDoc->GetDrawMode()) )
	{
		// Do nothing, right mouse not used in nav mode
		return;
	}
	else
	{
		if( !m_DoingLongStroke )
		{
			// Starting a new long stroke
			m_DoingLongStroke = TRUE;


			m_pStrokeCur = pDoc->NewStroke( pDoc->GetDrawMode() );

			// Add first point to the new stroke
			if( RWPoint.x < 0) RWPoint.x = 0;	// trap bad values
			if( RWPoint.y < 0) RWPoint.y = 0;	
			m_pStrokeCur->m_pointArray.Add(RWPoint); 

			SetCapture( );  // Capture the mouse until done drawing

			// This will be the MoveTo( ) anchor point for the LineTo() 
			// the next point, as the user drags the mouse
			m_ptPrev = point;
			m_ptCurrent = point;

		}
		else
		{
			// Continuing long strokes hooked together (such as making a box)


			if( GetCapture( ) != this )
			{
				// If this window (view) didn't capture the mouse, the 
				// user isn't drawing in this window.
				return;        
			}

			if( RWPoint.x < 0) RWPoint.x = 0;	// trap bad values
			if( RWPoint.y < 0) RWPoint.y = 0;	
			m_pStrokeCur->m_pointArray.Add(RWPoint);

			// Draw a line from the previous detected point in the mouse
			// drag to the current point.
			CPen* pOldPen = dc.SelectObject( pDoc->GetCurrentPen( ) );

			dc.MoveTo( m_ptPrev );
			dc.LineTo( point );
			dc.SelectObject( pOldPen );
			m_ptPrev = point;
			m_ptCurrent = point;

		}

	pDoc->UpdateAllViews(NULL, 0L, NULL);	// Force view update
	// replace base class completely:	
	// CView::OnRButtonDown(nFlags, point);	
	}
}

void CMapView::OnMouseMove(UINT nFlags, CPoint point) 
{
	IGNORE_UNUSED_PARAM (nFlags);

	CClientDC dc( this );
	// It's necessary to convert the point from device coordinates
	// to logical coordinates, such as are stored in the document.
	OnPrepareDC(&dc);  // set up mapping mode and viewport origin
	dc.DPtoLP(&point);

	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	POINT RWPoint = TranslateMCtoRW(m_ptCurrent);


	m_ptCurrent = point;	// for X,Y display

	// Mouse movement is interesting only if the user is currently drawing
	// a new stroke by dragging the captured mouse.
	if( GetCapture( ) != this )
	{
		// If this window (view) didn't capture the mouse, the 
		// user isn't drawing in this window.
		return;        
	}


	if( !m_DoingLongStroke )
	{
		// Only add points if doing a basic draw, not a long line draw
		if( RWPoint.x < 0) RWPoint.x = 0;	// trap bad values
		if( RWPoint.y < 0) RWPoint.y = 0;	
		m_pStrokeCur->m_pointArray.Add(RWPoint);

	}


	// Draw a line from the previous detected point in the mouse
	// drag to the current point.
	CPen* pOldPen = dc.SelectObject( pDoc->GetCurrentPen( ) );

	dc.MoveTo( m_ptPrev );
	dc.LineTo( point );
	dc.SelectObject( pOldPen );

	if( !m_DoingLongStroke )
	{
		m_ptPrev = point;
	}
	else
	{
		m_ptCurrent = point;

		pDoc->UpdateAllViews(NULL, 0L, NULL);
	}


	// replace base class completely:	
	// CView::OnMouseMove(nFlags, point);
}

void CMapView::OnUpdate(CView* pSender, LPARAM lHint, CObject* pHint) 
{
	// The document has informed this view that some data has changed.
	// Override the base class that repaints the whole screen to just
	// update the part that changed!  Called by UpdateAllViews().
	IGNORE_UNUSED_PARAM (pSender);

	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	if (pHint != NULL)
	{
		CClientDC dc(this);
		CClientDC* pDC = &dc;
		OnPrepareDC(&dc);	// reset orgin to compensate for scroll

		if (pHint->IsKindOf(RUNTIME_CLASS(CStroke)))
		{
			// The hint is that a stroke has been added (or changed).
			// So, invalidate its rectangle.
			CStroke* pStroke = (CStroke*)pHint;

			CRect rectInvalid = pStroke->GetBoundingRect();
			rectInvalid.BottomRight() =	TranslateRWtoMC(rectInvalid.BottomRight() );
			rectInvalid.TopLeft() =		TranslateRWtoMC(rectInvalid.TopLeft() );

			dc.LPtoDP(&rectInvalid);	// convert the other way - Logical -> Device coordinates
			InvalidateRect(&rectInvalid);
			return;
		}
		else if (pHint->IsKindOf(RUNTIME_CLASS(CMapView)))
		{
			// The hint is CMapView, which we view as meaning the pointer points to the area to update
			// So, invalidate its rectangle.

			CMapDoc* pDoc = GetDocument();
			ASSERT_VALID(pDoc);

			if( UPDATE_GPS_POINT == lHint )
			{
				// Request to update most recent GPS location
				POINT MCRobot = TranslateRWtoMC( g_pFullSensorStatus->CurrentLocationGPS );
				CRect rectInvalid(
					(MCRobot.x-200),		// Left
					(MCRobot.y+200),		// Top
					(MCRobot.x+200),		// Right
					(MCRobot.y-200) );		// Bottom
				dc.LPtoDP(&rectInvalid);
				InvalidateRect(&rectInvalid);
				return;
			}
			else if( UPDATE_TICK_POINT == lHint )
			{
				// Request to update most recent location, including sensor readings!
				POINT MCRobot = TranslateRWtoMC( g_pFullSensorStatus->CurrentLocation );
				CRect rectInvalid(
					(int)(MCRobot.x-(400.0*m_MC_MapZoom)),		// Left	(20in if just robot, 60 to include sensor readings)
					(int)(MCRobot.y+(400.0*m_MC_MapZoom)),		// Top
					(int)(MCRobot.x+(400.0*m_MC_MapZoom)),		// Right
					(int)(MCRobot.y-(400.0*m_MC_MapZoom)) );	// Bottom
				dc.LPtoDP(&rectInvalid);
				InvalidateRect(&rectInvalid);
				return;
			}
		}

	}
	// We can't interpret the hint, so assume that anything might
	// have been updated.
	Invalidate();
	return;

	
}

void CMapView::OnInitialUpdate() 
{
	CScrollView::OnInitialUpdate();
	g_RobotMapViewHWND = GetSafeHwnd();	// Allow other windows to send me messages

	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	/*
	SetScrollSizes also has two other parameters for which we use 
	the default values. These are CSize values that represent the size of 
	one page and one line, the distances to be scrolled if the user 
	clicks the scroll bar or a scroll arrow. The default values are 1/10th 
	and 1/100th of the document size, respectively.
	*/
	
}

void CMapView::OnPrint(CDC* pDC, CPrintInfo* pInfo) 
{
	if (pInfo->m_nCurPage == 1)     // page no. 1 is the title page
	{
		PrintTitlePage(pDC, pInfo);
	}
	else
	{
		CString strHeader = GetDocument()->GetTitle();

		PrintPageHeader(pDC, pInfo, strHeader);
		// PrintPageHeader() subtracts out from the pInfo->m_rectDraw the
		// amount of the page used for the header.
		pDC->SetWindowOrg(pInfo->m_rectDraw.left,-pInfo->m_rectDraw.top);

		// Now print the rest of the page
		OnDraw(pDC);
	}
	
	// CScrollView::OnPrint(pDC, pInfo); // default handler
}

void CMapView::PrintTitlePage(CDC *pDC, CPrintInfo *pInfo)
{
	// Prepare a font size for displaying the file name
	LOGFONT logFont;
	memset(&logFont, 0, sizeof(LOGFONT));
	logFont.lfHeight = 75; //  3/4th inch high in MM_LOENGLISH 
	CFont font;
	CFont* pOldFont = NULL;
	if (font.CreateFontIndirect(&logFont))
	{
		pOldFont = pDC->SelectObject(&font);
	}

	// Get the file name, to be displayed on title page
	CString strPageTitle = GetDocument()->GetTitle();

	// Display the file name 1 inch below top of the page,
	// centered horizontally
	pDC->SetTextAlign(TA_CENTER);
	pDC->TextOut(pInfo->m_rectDraw.right/2, -100, strPageTitle);

	if (pOldFont != NULL)
	{
		pDC->SelectObject(pOldFont);
	}
	



}

void CMapView::PrintPageHeader(CDC *pDC, CPrintInfo *pInfo, CString &strHeader)
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////
LRESULT CMapView::OnMapBulkItemToServer(WPARAM FunctionType, LPARAM StrokeObjectType)
{
	// This function receives map info from the client, such as drawn walls, etc.
	if( BULK_DATA_TYPE_MAP_STROKE == FunctionType )
	{
		CString strText;
		CMapDoc* pDoc = GetDocument();
		ASSERT_VALID(pDoc);
		CStroke*	pStroke;	// The stroke being saved

		POINT tmpPoint;

		// Create a new stroke object
		pStroke = pDoc->NewStroke( StrokeObjectType );

		// Get the points out of the linear buffer
		POINT* pPointArray = (POINT*)g_ClientBulkData;	// point to the global buffer
		int NumberOfPoints = g_ClientBulkDataLength / sizeof(POINT);
		for( int i=0; i < NumberOfPoints; i++ )
		{
			tmpPoint = *pPointArray++;

			// Add each point to the new stroke
			if( tmpPoint.x < 0) tmpPoint.x = 0;	// trap bad values
			if( tmpPoint.y < 0) tmpPoint.y = 0;	
			pStroke->m_pointArray.Add(tmpPoint);

		}
		// Tell the stroke item that we're done adding points to it.
		// This is so it can finish computing its bounding rectangle.
		pStroke->FinishStroke();

		// Now, if this is an Wall or Object, add to the local GridMap

		g_pGridMap->AddGridMapData( pStroke->m_pointArray, pDoc->GetGridmapObjectType(StrokeObjectType) );
	
		pDoc->UpdateAllViews(NULL, 0L, NULL);
	}
	else
	{
		ASSERT(0);	// Unknown command!
	}
return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
LRESULT CMapView::OnMapDisplayBulkItem(WPARAM Item, LPARAM lParam)
{
	// This function can display various Bulk items,
	// but is only used here for position data and possibly Distance and Compass data...
	IGNORE_UNUSED_PARAM (lParam);

	CString strText;
	CMapDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);

	static int nDebugSensor = 0;

	if( ROBOT_RESPONSE_PIC_STATUS == Item )
	{
		// Handle Status Info from the Arduino
		// Includes current Location and Sensor readings

	
		// Values are stored in Real World coordinates
		POINT RWRobot = { (int)g_pFullSensorStatus->CurrentLocation.x, (int)g_pFullSensorStatus->CurrentLocation.y };

		if( (RWRobot.x != m_LastRobotPos.x) ||		// Don't store repeated identical values
			(RWRobot.y != m_LastRobotPos.y) ||
			(RWRobot.x < 120 )				||		// and trap illegal values (robot is at least 24" diameter
			(RWRobot.y < 120 )				 )
		{
			// Store the location data for our "mouse trail"
			pDoc->AddRobotTrailPoint( RWRobot );
			m_LastRobotPos = RWRobot;
			

			// Save sensor reading into Map database
			// Sensor reading are based upon current heading and position of the robot
			// NOTE! Robot only records sensor values ONCE if not moving!
			if( g_pFullSensorStatus->CompassHeading <= 360 )
			{
				// Sensors only make sense if we know what direction we are heading!
				// The sensor info itself is in the global g_pFullSensorStatus
//				ROBOT_LOG( TRUE,  "MAP DEBUG - Adding Sensor Data!\n" );
				pDoc->AddSensorData(
					RWRobot,			// Location
					g_pFullSensorStatus->CompassHeading );	// Heading

				nDebugSensor++;
//				ROBOT_LOG( TRUE,  "MAP DEBUG - Added %d Sensor Data items to DOC!\n", nDebugSensor);

			}
			else
			{
				ROBOT_LOG( TRUE,  "MAP DOC ERROR!  - g_pFullSensorStatus->CompassHeading > 360!!!\n" );
			}

			pDoc->UpdateAllViews(NULL, UPDATE_TICK_POINT, this);	// TODO: should this be CurrentGPSPoint????
		}
	
		pDoc->UpdateAllViews(NULL, UPDATE_TICK_POINT, this);	// 2 = Current Point
	}
	else if( ROBOT_RESPONSE_GPS_DATA  == Item )
	{
		// Handle GPS data.  Note that g_pFullSensorStatus->CurrentLocationGPS has already been updated
		// by Module to indicate the Real World location for other modules (not used by
		// this function)

		if( (2 == g_pGPSData->btGSAFixMode) || (3 == g_pGPSData->btGSAFixMode) )
		{
			g_GPSGotFirstFix = TRUE;
		}

		if(	(g_GPSGotFirstFix) &&	// wait until we get some good data from the GPS
			((-1 != gGPSOriginLat) || (-1 != gGPSOriginLong)) ) // Need anchor point
		{

			// Convert from GPS to Map coordinates
			POINT GPSPos = GPSDegreesToRWTenthInches( g_pGPSData->dGGALatitude, g_pGPSData->dGGALongitude );


			if( (GPSPos.x != m_LastGPSPos.x) ||		// Don't store repeated identical values
				(GPSPos.y != m_LastGPSPos.y) ||
				(g_pGPSData->btGSAFixMode != m_LastFixMode) ) // Unless the fix quality changed
			{
				// Store the location data for our "mouse trail"
				pDoc->AddGPSPoint( GPSPos,	g_pGPSData->btGSAFixMode );	// Position and Quality of fix
				m_LastGPSPos = GPSPos;
				
				pDoc->UpdateAllViews(NULL, UPDATE_GPS_POINT, this);	// 1 = CurrentGPSPoint
			}
		}

	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMapView::PaintSensor( CDC* pDC, POINT RWRobotPos, int  RobotHeading, double SensorOffsetDegrees, int  SensorFOVDegrees, double ObjectDistance )
{
	// This function draws recorded objects spotted by sensors
	// Given current robot direction, offset of the sensor, and reported range
	//const double TwoPi = 2.0 * 3.141592;	// Convert Degrees to Radians then do the math

	double SensorDirection = RobotHeading + SensorOffsetDegrees;
	if( SensorDirection >= 360 ) 
	{
		SensorDirection -= 360;
	}
	if( SensorDirection < 0 ) 
	{
		SensorDirection += 360;
	}

	if( ObjectDistance > NO_OBJECT_IN_RANGE )
	{
		return;	// Don't paint a spot
	}

	// get two points to either side of the range value, and draw a line between them

	POINT RWObjectPt;
	// First Point
	double SensorDirectionRadians = (SensorDirection + SensorFOVDegrees) * DEGREES_TO_RADIANS;
	double dX = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * sin( SensorDirectionRadians );	// Returns negative numbers as needed
	double dY = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * cos( SensorDirectionRadians );
	if(dX>=0) dX += 0.5; else dX -= 0.5;	// Cast will truncate, this will round instead
	if(dY>=0) dY += 0.5; else dY -= 0.5;

	RWObjectPt.x = (int)RWRobotPos.x + (int)dX;		// Get new Map absolute X,Y
	RWObjectPt.y = (int)RWRobotPos.y + (int)dY;
	POINT MCObjectPt1 = TranslateRWtoMC(RWObjectPt);	// Convert from RealWorld to Map display coordinates

	// Second Point
	SensorDirectionRadians = (SensorDirection - SensorFOVDegrees) * DEGREES_TO_RADIANS;
	dX = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * sin( SensorDirectionRadians );	// Returns negative numbers as needed
	dY = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * cos( SensorDirectionRadians );

	RWObjectPt.x = (int)RWRobotPos.x + (int)(dX/10.0);		// Get new Map absolute X,Y
	RWObjectPt.y = (int)RWRobotPos.y + (int)(dY/10.0);		// Converted from TenthInches to Inches
	POINT MCObjectPt2 = TranslateRWtoMC(RWObjectPt);		// Convert from RealWorld to Map display coordinates

	// Draw a line between the two points
	pDC->MoveTo(  MCObjectPt1.x, MCObjectPt1.y );
	pDC->LineTo(  MCObjectPt2.x, MCObjectPt2.y );

}

POINT CMapView::TranslateRWtoMC( FPOINT RWFloat )
{
		POINT RW = { (LONG)RWFloat.x, (LONG)RWFloat.y };
		return TranslateRWtoMC( RW );
}

POINT CMapView::TranslateRWtoMC( POINT RW )
{
	// Translate Real World TenthInches to Map Coordinates
	POINT MC;	// Map Coordinates

	if( RW.x >= m_RW_MapBottomLeft.x )
	{
		// Within the border
		MC.x = (LONG)( (double)(RW.x - m_RW_MapBottomLeft.x) * m_MC_MapZoom );
	}
	else
	{
		// Clip at the border
		MC.x = (LONG)( ((double)m_RW_MapBottomLeft.x - 1.0) * m_MC_MapZoom);
	}


	if( RW.y >= m_RW_MapBottomLeft.y )
	{
		// Within the border
		MC.y = (LONG)( (double)(RW.y - m_RW_MapTopRight.y) * m_MC_MapZoom );	// Invert Y Axis	
	}
	else
	{
		// Clip at the border
		MC.y = (LONG)( (double)(-m_RW_MapTopRight.y) * m_MC_MapZoom );	// Invert Y Axis	
	}
	return MC;

}

POINT CMapView::TranslateMCtoRW( POINT MCPoint )
{
	// Translate Map Coordinates to Real World
	// Note - Traps are just backup.  
	// UpdateBoundry should have been called BEFORE this!

	POINT RW;	// Real World Coordinates

	RW.x = (LONG)( (double)MCPoint.x / m_MC_MapZoom ) + m_RW_MapBottomLeft.x;

	RW.y = m_RW_MapTopRight.y + (LONG)( (double)MCPoint.y / m_MC_MapZoom ); // Invert Y Axis

	
	//	RW.y = (LONG)(-1.0 * (double)MCPoint.y / m_MC_MapZoom) + m_RW_MapBottomLeft.y; // Invert Y Axis


	/***
	if( RWFrom.x >= m_RW_MapBottomLeft.x )
	{
		RW.x = (LONG)((double)(MCPoint.x  + m_RW_MapBottomLeft.x) / m_MC_MapZoom);
	}
	else
	{
		MC.x = 0;
		ROBOT_LOG( TRUE,  "ERROR! - Map Coordinate X < 0! \n" );
	}

	//if( RWFrom.y >= m_RW_MapBottomLeft.y )
	//{
		// TODO-MUST! ROBOT_ASSERT(0); // I think this code has an error!  Need to step through it!
		//temp1 not used?
		double tempy = (double)MCPoint.y;
		int temp = (int)(tempy / 2.0); // THIS IS NEVER USED????
		int temp1 = (int)(tempy / m_MC_MapZoom);
		int temp2 = temp + (m_RW_MapTopRight.y-1);
		int temp3 = (INT)temp2;

		RW.y = m_RW_MapTopRight.y - (LONG)((double)(MCPoint.y - m_RW_MapBottomLeft.y) / m_MC_MapZoom );	// Invert Y Axis	
		//RW.y = m_RW_MapTopRight.y - (LONG)((double)(MCPoint.y - m_RW_MapBottomLeft.y) / m_MC_MapZoom );	// Invert Y Axis	

	}
	else
	{
		MC.y = - (m_RW_MapTopRight.y-1);
		ROBOT_LOG( TRUE,  "ERROR! - Map Coordinate Y < 0! \n" );
	}	

	***/

	return RW;

}

CSize CMapView::GetMCZoomedMapSize()
{
	CSize ZoomedMapSize;
	
	ZoomedMapSize.cx = (LONG)((double)(m_RW_MapTopRight.x - m_RW_MapBottomLeft.x) * m_MC_MapZoom );
	ZoomedMapSize.cy = (LONG)((double)(m_RW_MapTopRight.y - m_RW_MapBottomLeft.y) * m_MC_MapZoom );
	return ZoomedMapSize;
}

bool CMapView::UpdateBoundry( FPOINT RW )
{
		POINT Point = { (LONG)RW.x, (LONG)RW.y };
		return UpdateBoundry( Point );
}

bool CMapView::UpdateBoundry( POINT RW )
{
	// checks to see if the RealWorld point is within currently defined boundry
	// if not, expands Map as needed
	// Returns flag to indicate that the boundry changed
	bool BoundryChanged = FALSE;

	if( !m_RealWorldInitialized )
	{
		// Map has not been initialized to the real world yet.
		// Use this first point to initialize the first page

		// get the number of even multiple pages from the RW origin
		// Fairly common that this might be zero if the origin is close to an edge
		int NumberOfPages_x = (RW.x / MAP_PAGE_WIDTH);
		m_RW_MapBottomLeft.x = MAP_PAGE_WIDTH * NumberOfPages_x;

		int NumberOfPages_y = (RW.y / MAP_PAGE_HEIGHT);
		m_RW_MapBottomLeft.y = MAP_PAGE_HEIGHT * NumberOfPages_y;

		// Start out with a map that is one page in size
		m_RW_MapTopRight.x = m_RW_MapBottomLeft.x + MAP_PAGE_WIDTH;
		m_RW_MapTopRight.y = m_RW_MapBottomLeft.y + MAP_PAGE_HEIGHT;

		//double MapPageY = (double)RW.y / MAP_PAGE_HEIGHT;
		//m_RW_MapBottomLeft.y = (LONG)(MapPageY * MAP_PAGE_HEIGHT);


		BoundryChanged = TRUE;
		m_RealWorldInitialized = TRUE;
	}


	// Add as many pages as it takes in any direction (but don't go negative past origin 0,0)
	// unless noted, all coordinates in RealWorld

	while( RW.x <= (m_RW_MapBottomLeft.x + MAP_BOUNDRY_SIZE) )	// West Edge
	{
		if( !AddPage( WEST ) )
		{
			break;
		}
		else
			BoundryChanged = TRUE;
	}
	while( RW.x >= (m_RW_MapTopRight.x - MAP_BOUNDRY_SIZE) )	// East Edge
	{
		if( !AddPage( EAST ) )
		{
			break;
		}
		else
			BoundryChanged = TRUE;
	}
	while( RW.y <= (m_RW_MapBottomLeft.y + MAP_BOUNDRY_SIZE) )	// South Edge
	{
		if( !AddPage( SOUTH ) )
		{
			break;
		}
		else
			BoundryChanged = TRUE;
	}
	while( RW.y >= (m_RW_MapTopRight.y   - MAP_BOUNDRY_SIZE) ) // North Edge
	{
		if( !AddPage( NORTH ) )
		{
			break;
		}
		else
			BoundryChanged = TRUE;
	}

	if( BoundryChanged )
	{
		// Boundry was changed.  Update scroll bars to fit new map size
		SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() ); 
		GetDocument()->UpdateAllViews( NULL );
	}
	return (BoundryChanged);	// Tell calling function that the boundry changed
}
 

bool CMapView::AddPage( int Direction )
{
	static BOOL fDisplayError = TRUE; // Only display the error once

	switch( Direction )
	{

		case EAST:
		{
			m_RW_MapTopRight.x += (LONG)(MAP_PAGE_WIDTH );		// Make Map wider
			break;
		}
		case NORTH:
		{
			m_RW_MapTopRight.y += (LONG)(MAP_PAGE_HEIGHT);	// Make Map taller
			break;
		}
		case WEST:
		{
			if( m_RW_MapBottomLeft.x < (LONG)(MAP_PAGE_WIDTH))
			{
				ROBOT_DISPLAY( fDisplayError, "\nERROR! Robot outside WEST Map Bountry! m_RW_MapBottomLeft.x = %d\n", m_RW_MapBottomLeft.x )
				fDisplayError = FALSE; // Only display once
				return FALSE;
			}
			m_RW_MapBottomLeft.x -= (LONG)(MAP_PAGE_WIDTH);	// Make map wider to the left
			break;
		}
		case SOUTH:
		{
			if( m_RW_MapBottomLeft.y < (LONG)(MAP_PAGE_HEIGHT) )
			{
				ROBOT_DISPLAY( fDisplayError, "\nERROR! Robot outside SOUTH Map Bountry! m_RW_MapBottomLeft.y = %d\n", m_RW_MapBottomLeft.y )
				fDisplayError = FALSE; // Only display once
				return FALSE;
			}
			m_RW_MapBottomLeft.y -= (LONG)(MAP_PAGE_HEIGHT);	// Move origin down
			break;
		}
		default:
		{
			ROBOT_DISPLAY( TRUE,  "ERROR! - BAD DIRECTION IN AddPage \n" );
			return FALSE;
		}
		// Things went OK, so clear any prior error
		fDisplayError = TRUE;
	}
	ROBOT_LOG( TRUE,  "MAP DEBUG - -> Added Page.   Direction = %d\n", Direction);

	return TRUE;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Event Handlers
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void CMapView::OnSelMapViewBtn() 
{
	GetParentFrame()->ActivateFrame();
//	pCmdUI->SetCheck( TRUE );

}
void CMapView::OnSelCmdViewBtn() 
{
	::PostMessage( g_RobotCmdViewHWND, (int )WM_COMMAND, ID_SEL_CMD_VIEW_BTN, 0);	
}
void CMapView::OnSelPathViewBtn() 
{
	::PostMessage( g_RobotPathViewHWND, (int )WM_COMMAND, ID_SEL_PATH_VIEW_BTN, 0);	
}
void CMapView::OnSelSetupViewBtn() 
{
	::PostMessage( g_RobotSetupViewHWND, (int )WM_COMMAND, ID_SEL_SETUP_VIEW_BTN, 0);	
}



void CMapView::OnUpdateSelMapViewBtn(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
	// pCmdUI->Enable( FALSE );	
	//pCmdUI->SetCheck( TRUE );
	
}

void CMapView::OnUpdateSelPathViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotPathViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
	
}

void CMapView::OnUpdateSelCmdViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotCmdViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CMapView::OnUpdateSelSetupViewBtn(CCmdUI* pCmdUI) 
{
	if( g_RobotSetupViewHWND != NULL )
		pCmdUI->Enable( TRUE );
	else
		pCmdUI->Enable( FALSE );
}

void CMapView::OnUpdatePos(CCmdUI *pCmdUI)
{
    pCmdUI->Enable();
    CString strPos;

	POINT RWPoint = TranslateMCtoRW(m_ptCurrent);
	POINT RWPrevious = TranslateMCtoRW(m_ptPrev);
	int  Length = CalculateDistance( RWPrevious, RWPoint );
	int  Angle = CalculateAngle( RWPrevious, RWPoint );

	strPos.Format( "X=%03d\' %02d\" Y=%03d\' %02d\"  L=%03d\' %02d\", A=%d deg", 
		RWPoint.x/TENTHINCHES_PER_FOOT, RWPoint.x%TENTHINCHES_PER_FOOT,  // TenthInches!
		RWPoint.y/TENTHINCHES_PER_FOOT, RWPoint.y%TENTHINCHES_PER_FOOT, 
		Length/TENTHINCHES_PER_FOOT, Length%TENTHINCHES_PER_FOOT,
		Angle ); 

    pCmdUI->SetText( strPos ); 
}

LRESULT CMapView::OnRobotUpdateView(WPARAM Item, LPARAM lParam)
{	
	IGNORE_UNUSED_PARAM (Item);
	IGNORE_UNUSED_PARAM (lParam);
	GetDocument()->UpdateAllViews(NULL, 0L, NULL);
	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Drawing Modes Handlers

void CMapView::OnZoomMap025() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap025\n" );
	m_MC_MapZoom = ZoomPercent25;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

void CMapView::OnZoomMap050() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap050\n" );
	m_MC_MapZoom = ZoomPercent50;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

void CMapView::OnZoomMap100() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap100\n" );
	m_MC_MapZoom = ZoomPercent100;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

void CMapView::OnZoomMap150() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap150\n" );
	m_MC_MapZoom = ZoomPercent150;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

void CMapView::OnZoomMap200() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap200\n" );
	m_MC_MapZoom = ZoomPercent200;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

void CMapView::OnZoomMap400() 
{
	ROBOT_LOG( TRUE,  "OnZoomMap400\n" );
	m_MC_MapZoom = ZoomPercent400;
	// Scale Scroll Bars to the size of the document	
	SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() );
	GetDocument()->UpdateAllViews( NULL );
}

/* TODO DAVES ADD THESE HANDLERS!!!!!
#define DRAW_MODE_USER_STROKE		0x00
#define DRAW_MODE_DOOR				0x01
#define DRAW_MODE_LOW_OBSTACLE		0x02
#define DRAW_MODE_OBSTACLE			0x03
#define DRAW_MODE_WALL				0x04
#define DRAW_MODE_CLIFF				0x05
#define DRAW_MODE_WAYPOINT_LOCATION	0x06
#define DRAW_MODE_NAVIGATION_GOAL	0x07
#define DRAW_MODE_SET_LOCATION		0x08	// Set current / start locatio of Robot
#define NUMBER_OF_DRAW_MODES		0x09	// This should be number of types listed above
*/

void CMapView::OnTypeObstacle() 
{
	ROBOT_LOG( TRUE,  "OnTypeObstacle\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_OBSTACLE ); 	
}

void CMapView::OnTypeUserdraw() 
{
	ROBOT_LOG( TRUE,  "OnTypeUserdraw\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_USER_STROKE ); 	
}

void CMapView::OnTypeWallobject() 
{
	ROBOT_LOG( TRUE,  "OnTypeWallobject\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_WALL ); 	
}

void CMapView::OnWaypointLocation() 
{
	ROBOT_LOG( TRUE,  "OnWaypointLocation\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_WAYPOINT_LOCATION ); 	
}

void CMapView::OnNavigationMode() 
{
	ROBOT_LOG( TRUE,  "OnNavigationMode\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_NAVIGATION_GOAL ); 	
}


void CMapView::OnExportToTextFile() 
{
	ROBOT_LOG( TRUE,  "OnExportToTextFile\n" );
	char *FileName = "c:\\temp\\Map.txt";
	GetDocument()->ExportToTextFile( FileName );
}

void CMapView::OnImportFromTextFile() 
{
	ROBOT_LOG( TRUE,  "OnImportFromTextFile\n" );
	char *FileName = "c:\\temp\\Map.txt";
	GetDocument()->ImportFromTextFile( FileName );
}

void CMapView::OnClearMouseTrail() 
{
	// Clear the current mouse trail from the Map 
	ROBOT_LOG( TRUE,  "OnClearMouseTrail\n" );

	// Delete items in the Robot Trail List
	POSITION pos;
	CRobotTrailStructList* pRobotTrailList = GetDocument()->GetRobotTrailList();
	pos = pRobotTrailList->GetHeadPosition();
	while (pos != NULL)
	{
		delete pRobotTrailList->GetNext(pos);
	}
	pRobotTrailList->RemoveAll();

	GetDocument()->UpdateAllViews(NULL, 0L, NULL);	// Force view update

}

void CMapView::OnClearSensorReadings() 
{
	// Clear the current sensor readings from the Map  
	ROBOT_LOG( TRUE,  "OnClearSensorReadings\n" );

	// Delete items in the Robot Sensor List
	POSITION pos;
	CMapSensorStructList* pMapSensorList = GetDocument()->GetMapSensorList();
	pos = pMapSensorList->GetHeadPosition();
	while (pos != NULL)
	{
		delete pMapSensorList->GetNext(pos);
	}
	pMapSensorList->RemoveAll();

	GetDocument()->UpdateAllViews(NULL, 0L, NULL);	// Force view update

}


// Drawing Modes Update Handlers

void CMapView::OnUpdateZoomMap025(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.025 == m_MC_MapZoom );
}

void CMapView::OnUpdateZoomMap050(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.050 == m_MC_MapZoom );
}

void CMapView::OnUpdateZoomMap100(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.10 == m_MC_MapZoom );
}

void CMapView::OnUpdateZoomMap150(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.15 == m_MC_MapZoom );
}

void CMapView::OnUpdateZoomMap200(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.20 == m_MC_MapZoom );
}

void CMapView::OnUpdateZoomMap400(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( 0.40 == m_MC_MapZoom );
}



void CMapView::OnUpdateTypeObstacle(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_OBSTACLE == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateTypeUserdraw(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_USER_STROKE == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateTypeWallobject(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_WALL == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateNavigationMode(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_NAVIGATION_GOAL == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateWaypointLocation(CCmdUI* pCmdUI) 
{
	if( g_RobotPathViewHWND != NULL )
	{
		pCmdUI->Enable( TRUE );
		// Add check mark to currently selected type on the menu
		pCmdUI->SetCheck( (DRAW_MODE_WAYPOINT_LOCATION == GetDocument()->GetStrokeType()) );
	}
	else
	{
		pCmdUI->Enable( FALSE );
	}
}

void CMapView::OnAddPageAbove() 
{
	// Expand the map
	if( AddPage(NORTH) )
	{
		// Boundry was changed.  Update scroll bars to fit new map size
		SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() ); 
		GetDocument()->UpdateAllViews( NULL );
	}
	else
	{
		AfxMessageBox( _T("Can't add page to the NORTH!") );
	}
}

void CMapView::OnAddPageBelow() 
{
	// Expand the map
	if( AddPage(SOUTH) )
	{
		// Boundry was changed.  Update scroll bars to fit new map size
		SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() ); 
		GetDocument()->UpdateAllViews( NULL );
	}
	else
	{
		AfxMessageBox( _T("Can't add page to the SOUTH!") );
	}
}

void CMapView::OnAddPageLeft() 
{
	// Expand the map
	if( AddPage(WEST) )
	{
		// Boundry was changed.  Update scroll bars to fit new map size
		SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() ); 
		GetDocument()->UpdateAllViews( NULL );
	}
	else
	{
		AfxMessageBox( _T("Can't add page to the WEST!") );
	}
}


void CMapView::OnAddPageRight() 
{
	// Expand the map
	if( AddPage(EAST) )
	{
		// Boundry was changed.  Update scroll bars to fit new map size
		SetScrollSizes( MM_LOENGLISH, GetMCZoomedMapSize() ); 
		GetDocument()->UpdateAllViews( NULL );
	}
	else
	{
		AfxMessageBox( _T("Can't add page to the EAST!") );
	}
}

void CMapView::OnUpdateAddPageAbove(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
	// TODO: Add your command update UI handler code here

}

void CMapView::OnUpdateAddPageBelow(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
	// TODO: Add your command update UI handler code here
	
}

void CMapView::OnUpdateAddPageLeft(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
	// TODO: Add your command update UI handler code here
	
}

void CMapView::OnUpdateAddPageRight(CCmdUI* pCmdUI) 
{
	IGNORE_UNUSED_PARAM (pCmdUI);
	// TODO: Add your command update UI handler code here
	
}

void CMapView::OnShowGridmapSquares() 
{
	// Toggle on and off
	m_ShowGridMapSquares = !m_ShowGridMapSquares;
	GetDocument()->UpdateAllViews(NULL, 0L, this);
}

void CMapView::OnShowRobotTrails() 
{
	// Toggle on and off
	m_ShowRobotTrails = !m_ShowRobotTrails;
	GetDocument()->UpdateAllViews(NULL, 0L, this);
}

void CMapView::OnShowSensorReadings() 
{
	// Toggle on and off
	m_ShowSensorReadings = !m_ShowSensorReadings;
	GetDocument()->UpdateAllViews(NULL, 0L, this);
}

void CMapView::OnGpsEnable() 
{
	m_GPSEnabled = !m_GPSEnabled;
	GetDocument()->UpdateAllViews(NULL, 0L, this);
}


void CMapView::OnUpdateShowRobotTrails(CCmdUI* pCmdUI) 
{
	pCmdUI->SetCheck( m_ShowRobotTrails );
	
}

void CMapView::OnUpdateShowGridmapSquares(CCmdUI* pCmdUI) 
{
	pCmdUI->SetCheck( m_ShowGridMapSquares );
	
}

void CMapView::OnUpdateShowSensorReadings(CCmdUI* pCmdUI) 
{
	pCmdUI->SetCheck( m_ShowSensorReadings );
	
}

void CMapView::OnUpdateGpsEnable(CCmdUI* pCmdUI) 
{
	pCmdUI->SetCheck( m_ShowSensorReadings );
}

#define NO_ACTIVE_KEY	((int )-1)

#define KEY_Q			81	
#define KEY_W			87
#define KEY_E			69
#define KEY_A			65	
#define KEY_S			83	
#define KEY_D			68
#define KEY_X			88	
#define KEY_C			67
#define KEY_R			82
#define KEY_O			79

	
#define KEY_ARROW_UP	38
#define KEY_ARROW_DOWN	40
#define KEY_ARROW_LEFT	37
#define KEY_ARROW_RIGHT	39
#define KEY_ESC			27
#define KEY_RETURN		13

#define KEY_PLUS		187
#define KEY_MINUS		189
#define KEY_ZERO		1

void CMapView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags) 
{
//	static int  m_LastOverrideState = SET_USER_RELEASED;

	if( nChar == (UINT)g_LastKey )
	{
		return;	// ignore repeated key
	}

	ROBOT_LOG( TRUE,  "KEY DOWN: %d, RepCnt: %u, Flags: %02X\n", nChar, nRepCnt, nFlags);

	// Map standard Gamer keys, with stop added, (WSADQEX) to movement
	// QWE = FwdLeft, Fwd, FwdRight
	// ASD = Spin Left, Stop, Spin Right
	// X = Back up straight

/***
	// Set / Update Override mode when any motion key pressed
	switch( nChar )
	{
		case KEY_W:			// W = Forward
		case KEY_ARROW_UP:	// Up Arrow
		case KEY_A:				// A = Spin Left
		case KEY_ARROW_LEFT:	// Left Arrow
		case KEY_D:				// D = Spin Right
		case KEY_ARROW_RIGHT:	// Right Arrow
		case KEY_Q:		// Q = Curve Left
		case KEY_E:		// E = Curve Right
		case KEY_X:				// X = Backup
		case KEY_ARROW_DOWN:	// Down Arrow
		{
			if( SET_USER_RELEASED == m_LastOverrideState)
			{
				// Need to take control with key press
				ROBOT_LOG( TRUE,  "KEY - Taking Control - Sending SET_USER_NORMAL\n" );
				SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_PAUSE,	0 );	// Pause any path that might be going on, to allow user to take control
				SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL, 0 );	// normal operation, Avoid and Collision active
				m_LastOverrideState = SET_USER_OVERRIDE_STATE_SENT;
			}
			else if( SET_USER_OVERRIDE_AND_STOP == m_LastOverrideState)
			{
				// Temporary override had been set with stop command.  Resume normal on key press
				ROBOT_LOG( TRUE,  "KEY - Resuming from Stop - Sending SET_USER_NORMAL\n" );
				SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_NORMAL, 0 );
				m_LastOverrideState = SET_USER_OVERRIDE_STATE_SENT;
			}
			break;
		}
	}
***/

FORCE_SPEED_CHANGE:

	switch( nChar )
	{
		case KEY_ESC:		// Escape key
		case KEY_RETURN:	// Return
		{
			// Not used
			break;
		}

/**		case KEY_O:	// Override Avoid and Collision modules
		{
			ROBOT_LOG( TRUE,  "KEY - Override All selected - Sending SET_USER_OVERRIDE\n" );
			SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_PAUSE,	1 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
			SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_OVERRIDE, 0 );
			m_LastOverrideState = SET_USER_OVERRIDE_STATE_SENT;
			break;
		}

		case KEY_R:	// Release Override (if active) and User Control
		{
			ROBOT_LOG( TRUE,  "KEY - Release Override selected - Sending SET_USER_RELEASED\n" );
			SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_RELEASED, 0 );
			SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_RESUME,	0 ); 
			m_LastOverrideState = SET_USER_RELEASED;
			break;
		}
**/
		case KEY_W:			// W = Forward
		case KEY_ARROW_UP:	// Up Arrow
		{
			ROBOT_LOG( TRUE,  "KEY Forward\n" );
			g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
			g_MotorCurrentTurnCmd = 0;	// Center
			break;
		}
		case KEY_A:				// A = Spin Left
		case KEY_ARROW_LEFT:	// Left Arrow
		{
			ROBOT_LOG( TRUE,  "KEY Spin Left\n" );
			g_MotorCurrentSpeedCmd = 0;
			g_MotorCurrentTurnCmd = -g_SpeedSetByKeyboard; // note negative value
			if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
			{
				g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
			}
			break;
		}
		case KEY_D:				// D = Spin Right
		case KEY_ARROW_RIGHT:	// Right Arrow
		{
			ROBOT_LOG( TRUE,  "KEY Spin Right\n" );
			g_MotorCurrentSpeedCmd = 0;
			g_MotorCurrentTurnCmd = g_SpeedSetByKeyboard;
			if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
			{
				g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
			}
			break;
		}
		case KEY_Q:		// Q = Curve Left
		// case 166:	// Page Left does not work
		{
			ROBOT_LOG( TRUE,  "KEY Curve Left\n" );
			g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
			g_MotorCurrentTurnCmd = -(g_SpeedSetByKeyboard/2);
			if( g_MotorCurrentTurnCmd < TURN_LEFT_MAX )	
			{
				g_MotorCurrentTurnCmd = TURN_LEFT_MAX;
			}
			break;
		}
		case KEY_E:		// E = Curve Right
		// case 167:	// Page Right does not work
		{
			ROBOT_LOG( TRUE,  "KEY Curve Right\n" );
			g_MotorCurrentSpeedCmd = g_SpeedSetByKeyboard;
			g_MotorCurrentTurnCmd = (g_SpeedSetByKeyboard/2);
			if( g_MotorCurrentTurnCmd > TURN_RIGHT_MAX )	
			{
				g_MotorCurrentTurnCmd = TURN_RIGHT_MAX;
			}
			break;
		}
		case KEY_S:	// S = Stop
		case 35:	// "End" = Stop
		{
			ROBOT_LOG( TRUE,  "KEY Stop\n" );
			g_MotorCurrentSpeedCmd = SPEED_STOP;
			g_MotorCurrentTurnCmd = 0;	// Center
			// Manual Stop button will override collision and avoidance behaviors, causing an immediate stop
			SendCommand( WM_ROBOT_SET_USER_PRIORITY, SET_USER_LOCAL_AND_STOP, 0 );
			//SendCommand( WM_ROBOT_EXECUTE_PATH, PATH_EXECUTE_PAUSE,	1 ); // lParam: 1 = Wait for bumper to start, 0 = Don't wait
			//m_LastOverrideState = SET_USER_OVERRIDE_AND_STOP;
			break;
		}
		case KEY_X:				// X = Backup
		case KEY_ARROW_DOWN:	// Down Arrow
		{
			ROBOT_LOG( TRUE,  "KEY Backup\n" );
			g_MotorCurrentSpeedCmd = SPEED_REV_MED_SLOW; // Keyboard Speed ignored by backup
			//g_SpeedSetByKeyboard = SPEED_FWD_MED_SLOW;	
			break;
		}
		case KEY_PLUS:	// + = Increase Speed
		{
			g_SpeedSetByKeyboard += 5;
			if( g_SpeedSetByKeyboard > SPEED_FULL_FWD ) g_SpeedSetByKeyboard = SPEED_FULL_FWD;
			CString MsgString;
			MsgString.Format("Speed Increased to %d\n", g_SpeedSetByKeyboard);
			ROBOT_DISPLAY( TRUE, (MsgString))

			nChar = g_LastKey;
			goto FORCE_SPEED_CHANGE;	// cause speed change to take effect
			break;
		}
		case KEY_MINUS:	// - = Decrease Speed
		{
			g_SpeedSetByKeyboard -= 5;
			if( g_SpeedSetByKeyboard < 0 ) g_SpeedSetByKeyboard = 0;
			CString MsgString;
			MsgString.Format("Speed Decreased to %d\n", g_SpeedSetByKeyboard);
			ROBOT_DISPLAY( TRUE, (MsgString))

			nChar = g_LastKey;
			goto FORCE_SPEED_CHANGE;	// cause speed change to take effect
			break;
		}
		case KEY_ZERO:	// 0 = Default Speed
		{
			g_SpeedSetByKeyboard = SPEED_FWD_MED_SLOW;
			CString MsgString;
			MsgString.Format("Speed set to Default (%d)\n", g_SpeedSetByKeyboard);
			ROBOT_DISPLAY( TRUE, (MsgString))
			nChar = g_LastKey;
			goto FORCE_SPEED_CHANGE;	// cause speed change to take effect
			break;
		}
	}

	if( (KEY_PLUS != nChar) && (KEY_MINUS != nChar) && (KEY_ZERO != nChar) )
	{
		g_LastKey = nChar;
	}

	CScrollView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CMapView::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags) 
{

	//ROBOT_LOG( TRUE,  "KEY UP: %d, RepCnt: %u, Flags: %02X\n", nChar, nRepCnt, nFlags);
	if( (KEY_Q == g_LastKey) || (KEY_E == g_LastKey) )
	{
		// Last key was curve left or right, so now continue going forward center
		ROBOT_LOG( TRUE,  "KeyUp - continue forward center\n" );
		g_LastKey = 0;
		g_MotorCurrentTurnCmd = 0;	// Center
	}

	CScrollView::OnKeyUp(nChar, nRepCnt, nFlags);
}

void CMapView::OnSetCurrentLocation() 
{
	ROBOT_LOG( TRUE,  "OnSetCurrentLocation\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_SET_LOCATION );	
	
}

void CMapView::OnTypeCliffObject() 
{
	ROBOT_LOG( TRUE,  "OnTypeCliffObject\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_CLIFF ); 	
}

void CMapView::OnTypeDoorObject() 
{
	ROBOT_LOG( TRUE,  "OnTypeDoorObject\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_DOOR ); 	
}

void CMapView::OnTypeLowObstacle() 
{
	ROBOT_LOG( TRUE,  "OnTypeLowObstacle\n" );
	GetDocument()->SetDrawMode( DRAW_MODE_LOW_OBSTACLE ); 	
}



void CMapView::OnUpdateTypeCliffObject(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_CLIFF == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateTypeDoorObject(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_DOOR == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateTypeLowObstacle(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_LOW_OBSTACLE == GetDocument()->GetStrokeType()) );
}

void CMapView::OnUpdateSetCurrentLocation(CCmdUI* pCmdUI) 
{
	// Add check mark to currently selected type on the menu
	pCmdUI->SetCheck( (DRAW_MODE_SET_LOCATION == GetDocument()->GetStrokeType()) );
}

