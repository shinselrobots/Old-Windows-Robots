// MapDoc.cpp : implementation of the CMapDoc class
//

#include "stdafx.h"
#include "Globals.h"
#include "Module.h"
#include "Robot.h"
#include "MapDoc.h"
#include "PenWidthsDlg.h"
#include "RotateMapDlg.h"
#include <math.h>
//#include <vector>


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;

IMPLEMENT_SERIAL( CStroke, CObject, 2 )	// Revision number of the serialized data

/////////////////////////////////////////////////////////////////////////////
// CStroke
//
// Strokes are used to manually draw lines on the map.
// useful for marking known landmarks, etc.

/////////////////////////////////////////////////////////////////////////////

CStroke::CStroke()
{
// This empty constructor should be used by
// the application framework for serialization only
}

CStroke::CStroke(int  nPenWidth)
{
	m_nPenWidth = nPenWidth;
	m_rectBounding.SetRectEmpty();	// Initialize the bounding box around a stroke.  Used for UI updates.

}

/////////////////////////////////////////////////////////////////////////////
BOOL CStroke::DrawStroke( CDC* pDC )
{
	IGNORE_UNUSED_PARAM (pDC);

	/**********
	// Draw a series of line segments
	CPen penStroke;
	if( !penStroke.CreatePen(PS_SOLID, m_nPenWidth, RGB(0,0,0))) 
		return FALSE;

	CPen* pOldPen = pDC->SelectObject( &penStroke );

	POINT MCPoint = pDoc->TranslateRWtoMC( m_pointArray[0] );

	pDC->MoveTo( MCPoint );
	for( int i=1; i < m_pointArray.GetSize(); i++ )
	{
		MCPoint = pDoc->TranslateRWtoMC( m_pointArray[i] );
		pDC->LineTo( MCPoint );
	}

	pDC->SelectObject( pOldPen );	// Remember to restore the Device Context!

  *********/
	return TRUE;


}


void CStroke::FinishStroke()
{
	// We had started drawing a stroke (several line segments connected together),
	// and are now done drawing this stroke.
	if( m_pointArray.GetSize() == 0 )
	{
		m_rectBounding.SetRectEmpty();
		return;
	}

	CPoint pt = m_pointArray[0];
	m_rectBounding = CRect( pt.x, pt.y, pt.x, pt.y );
	for (int i=1; i < m_pointArray.GetSize(); i++)
	{
		// If the point lies outside of the accumulated bounding
		// rectangle, then inflate the bounding rect to include it.
		pt = m_pointArray[i];

		m_rectBounding.left   = min(m_rectBounding.left, pt.x);
		m_rectBounding.right  = max(m_rectBounding.right, pt.x);
		m_rectBounding.top    = max(m_rectBounding.top, pt.y);
		m_rectBounding.bottom = min(m_rectBounding.bottom, pt.y);
	}

	// Add the pen width to the bounding rectangle.  This is needed
	// to account for the width of the stroke when invalidating
	// the screen.  Convert negative DC values to positive value for CRect.
	m_rectBounding.InflateRect(CSize(m_nPenWidth,-(int)m_nPenWidth));

	// For this applicaiton, however, we want to add room for the grid squares too.
	m_rectBounding.InflateRect(CSize(20,-20));

}


void CStroke::Serialize( CArchive& ar )
{
	if( ar.IsStoring( ) )
	{
		ar << m_rectBounding;
		ar << (WORD)m_nPenWidth;
		m_pointArray.Serialize( ar );
	}
	else
	{
		WORD w;
		ar >> m_rectBounding;
		ar >> w; m_nPenWidth = w;
		m_pointArray.Serialize( ar );
	}
}



/////////////////////////////////////////////////////////////////////////////
// CMapDoc
/////////////////////////////////////////////////////////////////////////////

IMPLEMENT_DYNCREATE(CMapDoc, CDocument)

BEGIN_MESSAGE_MAP(CMapDoc, CDocument)
	//{{AFX_MSG_MAP(CMapDoc)
	ON_COMMAND(ID_EDIT_CLEAR_ALL, OnEditClearAll)
	ON_COMMAND(ID_PEN_THICK_OR_THIN, OnPenThickOrThin)
	ON_UPDATE_COMMAND_UI(ID_EDIT_CLEAR_ALL, OnUpdateEditClearAll)
	ON_UPDATE_COMMAND_UI(ID_PEN_THICK_OR_THIN, OnUpdatePenThickOrThin)
	ON_COMMAND(ID_PEN_PENWIDTHS, OnPenPenwidths)
	ON_COMMAND(ID_EDIT_UNDO, OnEditUndo)
	ON_COMMAND(ID_ROTATE_MAP, OnRotateMap)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMapDoc construction/destruction

CMapDoc::CMapDoc()
{
	m_CustomFileOpen = TRUE;	// By default, DON'T assume the default path file is used

}

CMapDoc::~CMapDoc()
{
}

BOOL CMapDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	InitDocument();
	m_CustomFileOpen = TRUE;

	return TRUE;
}

BOOL CMapDoc::OnOpenDocument(LPCTSTR lpszPathName) 
{

	// Don't need to call InitDocument() here, because it gets called by the Serialize function

	if (!CDocument::OnOpenDocument(lpszPathName))
		return FALSE;

	CString strDefaultFile = DEFAULT_MAP_FILE;
	if(  0 == strDefaultFile.CompareNoCase(lpszPathName) )
	{
		// Default path.  Clear the map when loading (throw away the last session)
		// Note that the file will be saved automatically upon app shutdown
		//DeleteContents( ); // DON'T DO THIS, it Deletes the data just loaded!
	}
	else
	{
		// Not the default path
		m_CustomFileOpen = TRUE;	// Prompt to save the file
	}
	
	UpdateAllViews( NULL );
	return TRUE;
}
//DEFAULT_GRID_MAP_FILE


/////////////////////////////////////////////////////////////////////////////
// CMapDoc Implementation

#define INITIAL_RW_MAP_PAGE_HEIGTH 800
#define INITIAL_RW_MAP_PAGE_WIDTH 800

void CMapDoc::InitDocument()
{
	m_bThickPen = FALSE;
	m_nThinWidth = 2;     // Default thin pen is 2 pixels wide
	m_nThickWidth = 5;    // Default thick pen is 5 pixels wide
	m_DrawMode = DRAW_MODE_USER_STROKE;	// Default to user stroke, can change to Wall, Obstacle, etc.
	ReplacePen();   // Initialize pen according to current width

	// Delete any old Grid Map!  Only one allowed at a time!
	SAFE_DELETE( g_pGridMap );
	g_pGridMap = new CGridMap;	// Create a new GridMap


}


CStroke* CMapDoc::NewStroke( int  DrawMode )
{
	CStroke* pStrokeItem = new CStroke(m_nPenWidth);

	if( DRAW_MODE_USER_STROKE == DrawMode )
	{
		m_strokeList.AddTail( pStrokeItem );
	}
	else if( DRAW_MODE_DOOR == DrawMode )
	{
		m_DoorList.AddTail( pStrokeItem );
	}
	else if( DRAW_MODE_LOW_OBSTACLE == DrawMode )
	{
		m_LowObstacleList.AddTail( pStrokeItem );
	}
	else if( DRAW_MODE_OBSTACLE == DrawMode )
	{
		m_ObstacleList.AddTail( pStrokeItem );
	}
	else if( DRAW_MODE_WALL == DrawMode )
	{
		m_WallList.AddTail( pStrokeItem );
	}
	else if( DRAW_MODE_CLIFF == DrawMode )
	{
		m_CliffList.AddTail( pStrokeItem );
	}

	SetModifiedFlag( );    // Mark document as modified to confirm File Close.
	return pStrokeItem;
}





void CMapDoc::DeleteContents() 
{
	POSITION pos;

	// Delete items in the Robot Trail List
	pos = m_RobotTrailList.GetHeadPosition();
	while (pos != NULL)
	{
		delete m_RobotTrailList.GetNext(pos);
	}
	m_RobotTrailList.RemoveAll();

	// Delete items in the GPS Point List
	pos = m_GPSPointList.GetHeadPosition();
	while (pos != NULL)
	{
		delete m_GPSPointList.GetNext(pos);
	}
	m_GPSPointList.RemoveAll();

	// Delete items in the  Sensor Readings List
	pos = m_MapSensorList.GetHeadPosition();
	while (pos != NULL)
	{
		delete m_MapSensorList.GetNext(pos);
	}
	m_MapSensorList.RemoveAll();


	// Delete all the Stroke Lists that were drawn
	while( !m_strokeList.IsEmpty( ) )
	{
		delete m_strokeList.RemoveHead( );
	}
	while( !m_DoorList.IsEmpty( ) )
	{
		delete m_DoorList.RemoveHead( );
	}
	while( !m_LowObstacleList.IsEmpty( ) )
	{
		delete m_LowObstacleList.RemoveHead( );
	}
	while( !m_ObstacleList.IsEmpty( ) )
	{
		delete m_ObstacleList.RemoveHead( );
	}
	while( !m_WallList.IsEmpty( ) )
	{
		delete m_WallList.RemoveHead( );
	}
	while( !m_CliffList.IsEmpty( ) )
	{
		delete m_CliffList.RemoveHead( );
	}


	SAFE_DELETE( g_pGridMap );	// delete the GridMap
	
	CDocument::DeleteContents();
}

void CMapDoc::ReplacePen()
{

	m_nPenWidth = m_bThickPen ? m_nThickWidth : m_nThinWidth;

	// Change the current pen to reflect the new width.
	m_PenCurrent.DeleteObject( );
	m_PenCurrent.CreatePen( PS_SOLID, m_nPenWidth, RGB(0,0,0) );
}

/////////////////////////////////////////////////////////////////////////////
void CMapDoc::AddRobotTrailPoint( POINT RobotPos )
{

	// btGSAFixMode: 2= 2D fix, 3=3D fix

	// Add new CRobotTrailStruct to the CTypedPtrList
	CRobotTrailStruct* pRobotTrailStruct = new CRobotTrailStruct;

	pRobotTrailStruct->m_Pos.x = RobotPos.x;
	pRobotTrailStruct->m_Pos.y = RobotPos.y;

	m_RobotTrailList.AddTail(pRobotTrailStruct);
	SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	//UpdateAllViews( NULL, this,   );
}

/////////////////////////////////////////////////////////////////////////////
void CMapDoc::AddGPSPoint( POINT GPSPos, BYTE FixMode )	// Position and Quality of fix
{

	// btGSAFixMode: 2= 2D fix, 3=3D fix

	// Add new CGPSPointStruct to the CTypedPtrList
	CGPSPointStruct* pGPSPointStruct = new CGPSPointStruct;

	pGPSPointStruct->m_Pos.x = GPSPos.x;
	pGPSPointStruct->m_Pos.y = GPSPos.y;
	pGPSPointStruct->m_FixMode = FixMode;

	m_GPSPointList.AddTail(pGPSPointStruct);
	SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	//UpdateAllViews( NULL, this,   );

}

/////////////////////////////////////////////////////////////////////////////
void CMapDoc::AddSensorData( POINT RWPos, int nHeading )	// Position and Direction
{
	// The sensor info is in the global structure: g_pFullSensorStatus

	// Add new CMapSensorStruct to the CTypedPtrList
	CMapSensorStruct* pMapSensorStruct = new CMapSensorStruct;

	pMapSensorStruct->m_RWPos.x = RWPos.x;
	pMapSensorStruct->m_RWPos.y = RWPos.y;
	pMapSensorStruct->m_Heading = nHeading;
	int i;
	BOOL ObjectsDetected = FALSE;
// ROBOT_LOG( TRUE,  "DEBUG MAPVIEW - US SENSORS:  ")
	for( i = 0; i < NUM_US_SENSORS; i++ )
	{
		if( g_pFullSensorStatus->US[i] < NO_OBJECT_IN_RANGE )
		{
			ObjectsDetected = TRUE;
		}
		pMapSensorStruct->US[i] = g_pFullSensorStatus->US[i]; 
//		ROBOT_LOG( TRUE,  " %u = %u, ", i, pMapSensorStruct->US[i] )
	}
//	ROBOT_LOG( TRUE,  "inches\n" );
	for( i = 0; i < NUM_IR_SENSORS; i++ )
	{
		if( g_pFullSensorStatus->IR[i] < NO_OBJECT_IN_RANGE )
		{
			ObjectsDetected = TRUE;
		}
		pMapSensorStruct->IR[i] = g_pFullSensorStatus->IR[i]; 
	}

	if( ObjectsDetected )
	{
		m_MapSensorList.AddTail(pMapSensorStruct);
		SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
		//UpdateAllViews( NULL, this,   );
	}
	else
	{
		delete pMapSensorStruct;	// don't fill up the list with empty sensor info
	}

}


void CMapDoc::ExportToTextFile( char* FileName )
{

	FILE *File;
	CTypedPtrList<CObList, CStroke*> *strokeList;
	CRect	StrokeBB;	// smallest rect that surrounds all of the points in the stroke
	int 	StrokePenWidth;	// One width applies to entire stroke
	char	ObjectTypeChar;
	int		DebugStrokeNumber = 1;

	int LinesWritten = 0;

	// Open the file for writing
	errno_t nError;
	nError = fopen_s( &File, FileName, "w" );
	if( 0 != nError )
	{
		ROBOT_LOG( TRUE, "Could Not open Export File (%s)!\n", FileName )
		return;
	}


	// Ok, now dump the map data
	// We only care about User Strokes, Walls, Objects, etc.

	// ":" = Field Start (Example, :Walls)

	fprintf(File, "\n// Exported Map Data\n" );
	ObjectTypeChar = 'Z';	// error flag

	for(int i=0; i<=5; i++)
	{
		// Note - i does not need to map to the stroke type, it's just an iterator
		// in general draw from solid objects to less solid
		if( 0 == i)	// Draw Walls 
		{
			fprintf(File, "// Wall Strokes\n" );
			strokeList = &m_WallList;
			ObjectTypeChar = 'W';
		}
		else if( 1 == i)	// Cliffs
		{	
			fprintf(File, "// Cliff Strokes\n" );
			strokeList = &m_CliffList;
			ObjectTypeChar = 'C';
		}
		else if( 2 == i)	// Obstacles
		{	
			fprintf(File, "// Obstacle Strokes\n" );
			strokeList = &m_ObstacleList;
			ObjectTypeChar = 'O';
		}
		else if( 3 == i)	// Low Obstacles
		{	
			fprintf(File, "// Low Obstacle Strokes\n" );
			strokeList = &m_LowObstacleList;
			ObjectTypeChar = 'L';
		}
		else if( 4 == i)	// Doors
		{	
			fprintf(File, "// Door Strokes\n" );
			strokeList = &m_DoorList;
			ObjectTypeChar = 'D';
		}
		else if( 5 == i)	// User scribbles
		{	
			fprintf(File, "// User Strokes\n" );
			strokeList = &m_strokeList;
			ObjectTypeChar = 'U';
		}
		else
		{
			ROBOT_ASSERT(0);
		}

		POSITION pos = strokeList->GetHeadPosition( );
		while (pos != NULL)
		{
			CStroke* pStroke = strokeList->GetNext(pos);
			StrokeBB = pStroke->GetBoundingRect();
			StrokePenWidth = pStroke->GetPenWidth();

			// Note: No need to store BB!

			// Stroke Info -  Pen Width, number of lines is appended to the object type
			fprintf(File, "// Item %d\n", DebugStrokeNumber++ );
			fprintf(File, ":%c %d,%d\n", ObjectTypeChar, StrokePenWidth, pStroke->m_pointArray.GetSize() );
			LinesWritten++;

			// Export the series of line segments in a stroke
			//fprintf(File, "// Stroke Lines - x,y, x,y, x,y, ...\n" );
			//LinesWritten++;
			for( int i=0; i < pStroke->m_pointArray.GetSize(); i++ )
			{
				fprintf(File, ":XY %d,%d\n", pStroke->m_pointArray[i].x, pStroke->m_pointArray[i].y );
				LinesWritten++;
			}
			fprintf(File, "\n" );
		}
	}

	fprintf(File, "// End of File\n" );
	fprintf(File, "///////////////////////////////\r\n" );
	ROBOT_LOG( TRUE, "Export to file (%s):  %d Lines Written!\n", FileName, LinesWritten )
	CString strStatus;
	strStatus.Format("Export to file (%s):  %d Lines Written!\n", FileName, LinesWritten );
	ROBOT_DISPLAY( TRUE, strStatus )

	if( fclose( File ) )
	{
		ROBOT_LOG( TRUE, "Error Closing Export File (%s)!\n", FileName )
	}


}

void CMapDoc::ImportFromTextFile( char* FileName )
{
	FILE		*File;
	char		tmpStr[40];
	char		FieldType;
	int			nFieldsRead;
	int			nLineWidth = 0;
	int			x, y;
	int			nStrokes = 0;
	POINT		Point;
	CStroke*	pStrokeItem = NULL;
//	BOOL		bDoingStroke = FALSE;
	BOOL		bGridMapItem = FALSE;
	int 		GridmapObjectType = GRIDMAP_OBJECT_NONE;
	int			DebugStrokeNumber = 1;
	char		*pStrRead;

	ROBOT_LOG( TRUE, "\n\n =================================================================\n" )
	ROBOT_LOG( TRUE, "IMPORTING TEXT FROM FILE (%s)\n", FileName )

	// Open the file for reading
	errno_t nError;
	nError = fopen_s( &File, FileName, "r" );
	if( 0 != nError )
	{
		ROBOT_LOG( TRUE, "Could Not open Import File (%s)!\n", FileName )
		return;
	}

	// Ok, now get the map data
	// We only care about User Strokes, Walls, and Objects
	nFieldsRead = 0;

		// See OnLButtonDown, OnLButtonUp
		// don't need to store bounding box!


		// TODO - Use fscanf
/*
		nFieldsRead = fscanf( File, "%s", tmpStr );	// title string
		if( EOF == nFieldsRead ) return;
			ROBOT_LOG( TRUE,  "%s\n", tmpStr)
		nFieldsRead = fscanf( File, "%s", tmpStr );	// Walls string
		if( EOF == nFieldsRead ) return;
			ROBOT_LOG( TRUE,  "%s\n", tmpStr)

//		nFieldsRead = scanf( "%d %3.2f %c %C %s %S", &i, &fp, &c, &wc, s, ws );
		if( EOF == nFieldsRead )
		{
			ROBOT_LOG( TRUE,  "Done Reading!\n" )
		}
*/
	pStrRead = NULL;

	while( TRUE )
	{
		pStrRead = fgets(tmpStr, sizeof(tmpStr), File);
		if( NULL == pStrRead )
		{
			// EOF?
			int Eof = feof( File );
			if( 0 == Eof )
			{
				ROBOT_LOG( TRUE, "ERROR - Aborted before End of File.\n")
			}
			int FileError = ferror( File );
			if( 0 != FileError )
			{
				ROBOT_LOG( TRUE, "FILE ERROR = %d\n\n", FileError)
				ROBOT_ASSERT(0);
			}
			break;
		}

		//ROBOT_LOG( TRUE, "READ: %s", tmpStr)

		// KLUDGE NEEDED  TO MAKE WINDOWS LAZY READ WORK!  SHEESH!!!!
		Sleep(2);

		if( 10 == tmpStr[0] )	// Carriage Return
		{			
			//ROBOT_LOG( TRUE, "BLANK LINE\n", tmpStr)
			continue;	// Blank line
		}

		if( 0 == strncmp(tmpStr, "// End", 6) )
		{			
			ROBOT_LOG( TRUE, "%s", tmpStr)	// Print the comment
			continue; // DEBUG!!!
		}

		if( 0 == strncmp(tmpStr, "//", 2) )
		{			
			ROBOT_LOG( TRUE, "%s", tmpStr)	// Print the comment
			continue;
		}
		else if( ':' == tmpStr[0] )
		{
			// Begin Field
			FieldType = tmpStr[1];
			if( ('X' != FieldType) &&	// Not receiving XY data
				(0 != nStrokes)		)	// But not all strokes were read
			{
				ROBOT_LOG( TRUE, "ERROR! Not all expected Strokes were found!\n")
				ROBOT_ASSERT(0);
			}

			switch( FieldType )
			{
				case'X':	// XY Data
				{	
					sscanf_s(tmpStr, ":XY %ld,%ld", &x, &y);
					ROBOT_LOG( TRUE,  "XY Data: x=%d, y=%d\n", x, y)
						////////////////////// IF IMPORTIG OLD FILES, MULTIPLY BY 10 HERE
						// To convert inches to tenthinches
					Point.x = x;// * 10; 
					Point.y = y;// * 10;
					pStrokeItem->m_pointArray.Add(Point);
					if( nStrokes-- < 0 )
					{
						ROBOT_LOG( TRUE, "ERROR! nStrokes < 0!\n")
						ROBOT_ASSERT(0);
					}
					else if(0 == nStrokes )
					{
						pStrokeItem->FinishStroke();
						ROBOT_LOG( TRUE, "Finished Stroke\n")

						if( bGridMapItem )
						{
							// Now, if this is an Wall or Object, add to the local GridMap
							g_pGridMap->AddGridMapData( pStrokeItem->m_pointArray, GridmapObjectType );
						}
					}
					break;
				}
				case 'W':	// Walls 
				{	
					ROBOT_LOG( TRUE, "%d Walls - ", DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":W %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_WallList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_WALL;
					bGridMapItem = TRUE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				case 'C':	// Cliffs
				{	
					ROBOT_LOG( TRUE, "%d Cliffs - ", DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":C %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_CliffList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_CLIFF;
					bGridMapItem = TRUE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				case 'O':	// Obstacles
				{	
					ROBOT_LOG( TRUE, "%d Obstacles - ", DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":O %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_ObstacleList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_OBSTACLE;
					bGridMapItem = TRUE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				case 'L':	// LowObstacles
				{	
					ROBOT_LOG( TRUE, "%d Low Obstacles - ",DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":L %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_LowObstacleList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_LOW_OBSTACLE;
					bGridMapItem = TRUE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				case 'D':	// Doors
				{	
					ROBOT_LOG( TRUE, "%d Doors - ", DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":D %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_DoorList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_DOOR;
					bGridMapItem = TRUE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				case'U':	// User strokes
				{	
					ROBOT_LOG( TRUE, "%d User Strokes - ", DebugStrokeNumber++ )
					sscanf_s(tmpStr, ":U %d,%d", &nLineWidth, &nStrokes);
					ROBOT_LOG( TRUE, "Line Width = %d, Number Strokes = %d\n", nLineWidth, nStrokes )
					pStrokeItem = new CStroke(nLineWidth);
					m_strokeList.AddTail( pStrokeItem );
					GridmapObjectType = GRIDMAP_OBJECT_NONE;
					bGridMapItem = FALSE;
					ROBOT_LOG( TRUE,"Added New Blank Stroke\n")
					break;
				}
				default:
				{
					ROBOT_LOG( TRUE, "ERROR! ImportFromTextFile: Unknown Field type (%s) \n", tmpStr )
					GridmapObjectType = GRIDMAP_OBJECT_NONE;
					bGridMapItem = FALSE;
				//	ROBOT_ASSERT(0);
				}
			}
			continue;
		}
		else
		{
			ROBOT_LOG( TRUE, "BAD LINE: {%s}\n", tmpStr)
		}
	}

	SetModifiedFlag( );    // Mark document as modified to confirm File Close.
	UpdateAllViews(NULL, 0L, NULL);	// Update the GUI View

	if( fclose( File ) )
	{
		ROBOT_LOG( TRUE, "Error Closing Export File (%s)!\n", FileName )
	}

	ROBOT_LOG( TRUE,"\n\n =================================================================\n")

}

/////////////////////////////////////////////////////////////////////////////
// CMapDoc serialization
// WARNING! If you add anything to this function, also add it to 
// ExportToTextFile and ImportFromTextFile.

void CMapDoc::Serialize(CArchive& ar)
{
	WORD w;

//#define STORING_SENSOR_AND_ROBOT_TRAILS
#ifdef STORING_SENSOR_AND_ROBOT_TRAILS
	WORD nCount;
	POSITION pos;
#endif

	if( ar.IsStoring() )
	{
		// Saving Map data to disk
		// Save current Robot Location (becomes default location when loading the map)
		w = (WORD)g_pFullSensorStatus->CurrentLocation.x; ar << w;
		w = (WORD)g_pFullSensorStatus->CurrentLocation.y; ar << w;
#ifdef STORING_SENSOR_AND_ROBOT_TRAILS
		// Robot Trail
		nCount = (WORD)m_RobotTrailList.GetCount();
		ar << nCount;
		pos = m_RobotTrailList.GetHeadPosition();
		while (pos != NULL)
		{
			CRobotTrailStruct* pRobotTrailStruct = m_RobotTrailList.GetNext(pos);
			w = (WORD)pRobotTrailStruct->m_Pos.x; ar << w;
			w = (WORD)pRobotTrailStruct->m_Pos.y; ar << w;
			nCount--;
		}
		ROBOT_ASSERT(nCount == 0);

		// GPS Trail
		nCount = (WORD)m_GPSPointList.GetCount();
		ar << nCount;
		pos = m_GPSPointList.GetHeadPosition();
		while (pos != NULL)
		{
			CGPSPointStruct* pGPSPointStruct = m_GPSPointList.GetNext(pos);
			w = (WORD)pGPSPointStruct->m_Pos.x; ar << w;
			w = (WORD)pGPSPointStruct->m_Pos.y; ar << w;
			w = (WORD)pGPSPointStruct->m_FixMode; ar << w;

			nCount--;
		}
		ROBOT_ASSERT(nCount == 0);


		// Sensor readings
		nCount = (WORD)m_MapSensorList.GetCount();
		ar << nCount;
		pos = m_MapSensorList.GetHeadPosition();
		while (pos != NULL)
		{
			CMapSensorStruct* pMapSensorStruct = m_MapSensorList.GetNext(pos);
			w = (WORD)pMapSensorStruct->m_RWPos.x; ar << w;
			w = (WORD)pMapSensorStruct->m_RWPos.y; ar << w;
			w = (WORD)pMapSensorStruct->m_Heading; ar << w;

			int i;
			for( i = 0; i < NUM_US_SENSORS; i++ )
			{
				w = (WORD)pMapSensorStruct->US[i]; ar << w;
			}
			for( i = 0; i < NUM_IR_SENSORS; i++ )
			{
				w = (WORD)pMapSensorStruct->IR[i]; ar << w;
			}

			nCount--;
		}
		ROBOT_ASSERT(nCount == 0);
#endif
	}
	else
	{
		// Restoring Map data from disk
		InitDocument();

		// Restore default position of Robot, based upon where the robot was when the map was saved.
		ar >> w; g_pFullSensorStatus->CurrentLocation.x = w;
		ar >> w; g_pFullSensorStatus->CurrentLocation.y = w;

//#define STORING_SENSOR_AND_ROBOT_TRAILS
#ifdef STORING_SENSOR_AND_ROBOT_TRAILS
		// Robot Trail
		ar >> nCount;
		while (nCount-- > 0)
		{
			CRobotTrailStruct* pRobotTrailStruct = new CRobotTrailStruct;

			ar >> w; pRobotTrailStruct->m_Pos.x = w;
			ar >> w; pRobotTrailStruct->m_Pos.y = w;

			m_RobotTrailList.AddTail(pRobotTrailStruct);
		}

		// GPS
		ar >> nCount;
		while (nCount-- > 0)
		{
			CGPSPointStruct* pGPSPointStruct = new CGPSPointStruct;

			ar >> w; pGPSPointStruct->m_Pos.x = w;
			ar >> w; pGPSPointStruct->m_Pos.y = w;
			ar >> w; pGPSPointStruct->m_FixMode = (BYTE)w;

			m_GPSPointList.AddTail(pGPSPointStruct);
		}


		// Sensor readings
		ar >> nCount;
		while (nCount-- > 0)
		{
			CMapSensorStruct* pMapSensorStruct = new CMapSensorStruct;

			ar >> w; pMapSensorStruct->m_RWPos.x = w;
			ar >> w; pMapSensorStruct->m_RWPos.y = w;
			ar >> w; pMapSensorStruct->m_Heading = w;


			int i;
			for( i = 0; i < NUM_US_SENSORS; i++ )
			{
				ar >> w; pMapSensorStruct->US[i] = w;
			}
			for( i = 0; i < NUM_IR_SENSORS; i++ )
			{
				ar >> w; pMapSensorStruct->IR[i] = w;
			}

			m_MapSensorList.AddTail(pMapSensorStruct);
		}
#endif
	}	// done restoring some map data from disk


	// Now restore Drawn Strokes: User, Walls, and Objects.


	if ( ar.IsStoring() )
	{
		ROBOT_LOG( TRUE,"Writing Stroke Data \n")
	}
	else
	{
		ROBOT_LOG( TRUE,"Reading Stroke Data \n")
	}

	// This automatically handles both saving and loading
	ROBOT_LOG( TRUE,"User Strokes: ")
	m_strokeList.Serialize( ar );

	ROBOT_LOG( TRUE,"Doors: ")
	m_DoorList.Serialize( ar );	

	ROBOT_LOG( TRUE,"Low Obstacles: ")
	m_LowObstacleList.Serialize( ar );	

	ROBOT_LOG( TRUE,"Obstacles: ")
	m_ObstacleList.Serialize( ar );	

	ROBOT_LOG( TRUE,"Walls: ")
	m_WallList.Serialize( ar );		

	ROBOT_LOG( TRUE,"Cliffs: ")
	m_CliffList.Serialize( ar );	


	// If loading, recreate the GridMap from stoke data, now that the strokes are loaded.
	if ( !ar.IsStoring() )
	{
		CreateGridMapFromVectorData();
	}

}



/////////////////////////////////////////////////////////////////////////////
// CMapDoc diagnostics

#ifdef _DEBUG
void CMapDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CMapDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


/////////////////////////////////////////////////////////////////////////////
// CMapDoc command handlers


void CMapDoc::OnEditClearAll() 
{
	DeleteContents( );
	SetModifiedFlag();		// Tell CDocument to prompt to save changes on exit
	UpdateAllViews( NULL );

}

void CMapDoc::OnPenThickOrThin() 
{
	// Toggle the state of the pen between thin and thick.
	m_bThickPen = !m_bThickPen;

	// Change the current pen to reflect the new width.
	ReplacePen( );

	
}

void CMapDoc::OnUpdateEditClearAll(CCmdUI* pCmdUI) 
{
	// Enable the user-interface object (menu item or tool- 
	// bar button) if the document is non-empty, i.e., has 
	// at least one stroke.
	pCmdUI->Enable( !m_strokeList.IsEmpty( ) );	// pCmdUI is the pointer to the menu item

	
}

void CMapDoc::OnUpdatePenThickOrThin(CCmdUI* pCmdUI) 
{
	// Add check mark to Pen Thick Line menu item if the current
	// pen width is "thick."
	pCmdUI->SetCheck( m_bThickPen );

}


void CMapDoc::OnPenPenwidths() 
{
	CPenWidthsDlg dlg;

	// Initialize dialog data
	dlg.m_nThinWidth = m_nThinWidth;
	dlg.m_nThickWidth = m_nThickWidth;

	// Invoke the dialog box
	if (dlg.DoModal() == IDOK)
	{
		// retrieve the dialog data
		m_nThinWidth = dlg.m_nThinWidth;
		m_nThickWidth = dlg.m_nThickWidth;

		// Update the pen used by views when drawing new strokes
		// to reflect the new pen widths for "thick" and "thin".
		ReplacePen();
	}

}

void CMapDoc::OnRotateMap() 
{
	CRotateMapDlg dlg;
	int RotationAmount = 0;
	double ScaleAmount = 0;

	// Initialize dialog data
	dlg.m_MapRotationAmount = 0;

	// Invoke the dialog box
	if (dlg.DoModal() == IDOK)
	{
		// retrieve the dialog data
		RotationAmount = dlg.m_MapRotationAmount;
		if( 0 == dlg.m_MapScaleAmount )  // trap common user error
			ScaleAmount = 100.0; // normal is 100% (no scaling)
		else
			ScaleAmount = dlg.m_MapScaleAmount / 100.0;
	}
	else
	{
		return;
	}

	// Now, Rotate the Map!
	ROBOT_LOG( TRUE,"Map Rotation Requested: %d\n", RotationAmount )
	ROBOT_LOG( TRUE,"Map Scale Requested: %2.2f\n", ScaleAmount )

	// Find center of map.  Rotate around this point
	CRect MapBoundry;// = CRect( pt.x, pt.y, pt.x, pt.y );
	MapBoundry = GetMapBoundry();
	CPoint MapCenter = MapBoundry.CenterPoint();

	ROBOT_LOG( TRUE,"DEBUG - Map CENTER found at: %d,%d\n", MapCenter.x, MapCenter.y )

	// For each stroke point, 
	// Convert to Polar Coordinates
	// Rotate the value
	// Convert back to Rectangular Coordinates
	RotateMapPoints( MapCenter, RotationAmount );

	ScaleMapPoints( ScaleAmount ); 

	// Once all the vectors are moved, re-calculate the GridMap points

	// Delete the old Grid Map, and create a new one
	SAFE_DELETE( g_pGridMap );
	g_pGridMap = new CGridMap;	// Create a new GridMap

	// Now calculate the grid data from the vectors.
	CreateGridMapFromVectorData();


}
///////////////////////////////////////////////////////////////////////////
void CMapDoc::CreateGridMapFromVectorData() 
{
	// Create the full gridmap from wall, obstacle, etc. data
	// Used when rotating the map and when restoring from serialized file
	// Warning!  Assumes clean map to start with!

	ROBOT_ASSERT(g_pGridMap);

	POSITION pos;
	CTypedPtrList<CObList, CStroke*> *DoorList = &(m_DoorList);
	CTypedPtrList<CObList, CStroke*> *LowObstacleList = &(m_LowObstacleList);
	CTypedPtrList<CObList, CStroke*> *ObstacleList = &(m_ObstacleList);
	CTypedPtrList<CObList, CStroke*> *WallList = &(m_WallList);
	CTypedPtrList<CObList, CStroke*> *CliffList = &(m_CliffList);

	// Do Doors
	pos = DoorList->GetHeadPosition( );
	while (pos != NULL)
	{
		CStroke* pStroke = DoorList->GetNext(pos);
		// Add to the GridMap
		g_pGridMap->AddGridMapData( pStroke->m_pointArray, GRIDMAP_OBJECT_DOOR );
	}

	// Do Low Obstacles
	pos = LowObstacleList->GetHeadPosition( );
	while (pos != NULL)
	{
		CStroke* pStroke = LowObstacleList->GetNext(pos);
		// Add to the GridMap
		g_pGridMap->AddGridMapData( pStroke->m_pointArray, GRIDMAP_OBJECT_LOW_OBSTACLE );
	}

	// Do High Obstacles
	pos = ObstacleList->GetHeadPosition( );
	while (pos != NULL)
	{
		CStroke* pStroke = ObstacleList->GetNext(pos);
		// Add to the GridMap
		g_pGridMap->AddGridMapData( pStroke->m_pointArray, GRIDMAP_OBJECT_OBSTACLE );
	}

	// Do Walls
	pos = WallList->GetHeadPosition( );
	while (pos != NULL)
	{
		CStroke* pStroke = WallList->GetNext(pos);
		// Add to the GridMap
		g_pGridMap->AddGridMapData( pStroke->m_pointArray, GRIDMAP_OBJECT_WALL );
	}

	// Do Cliffs
	pos = CliffList->GetHeadPosition( );
	while (pos != NULL)
	{
		CStroke* pStroke = CliffList->GetNext(pos);
		// Add to the GridMap
		g_pGridMap->AddGridMapData( pStroke->m_pointArray, GRIDMAP_OBJECT_CLIFF );
	}



}

///////////////////////////////////////////////////////////////////////////
void CMapDoc::RotateMapPoints( CPoint MapCenter, int RotationAmount ) 
{
	// For each stroke point, 
	// Convert to Polar Coordinates
	// Rotate the value
	// Convert back to Rectangular Coordinates

//	CRect MapBoundry;
	CPoint pt;
	POINT TempPoint;
	POSITION pos;
	double PolarDegrees;
	double PolarDistance;
//	BOOL FirstPoint = TRUE;	// special case the first point for finding boundry

	/////////////////////////////////////////////////////////////////////////////
	// Handle Strokes (includes graphics such as landscape or house plans)

	CTypedPtrList<CObList, CStroke*> *strokeList;

	for(int i=0; i<=5; i++)
	{
		// Note - i does not need to map to the stroke type, it's just an iterator
		// in general draw from solid objects to less solid
		if( 0 == i)	// Draw Walls 
		{	
			strokeList = &m_WallList;
		}
		else if( 1 == i)	// Draw Cliffs
		{	
			strokeList = &m_CliffList;
		}
		else if( 2 == i)	// Draw Obstacles
		{	
			strokeList = &m_ObstacleList;
		}
		else if( 3 == i)	// Draw Obstacles
		{	
			strokeList = &m_LowObstacleList;
		}
		else if( 4 == i)	// Draw Obstacles
		{	
			strokeList = &m_DoorList;
		}
		else if( 5 == i)	// Draw User scribbles
		{	
			strokeList = &m_strokeList;
		}
		else
		{
			ROBOT_LOG( TRUE,"MapDoc.cpp - Bad i in GetBoundry Strokes!")
			ASSERT(0);
		}


		pos = strokeList->GetHeadPosition( );
		while (pos != NULL)
		{
			CStroke* pStroke = strokeList->GetNext(pos);
			for( int i=0; i < pStroke->m_pointArray.GetSize(); i++ )
			{
				pt = pStroke->m_pointArray[i];

				// Convert to Polar
				PolarDegrees = CalculateAngle( MapCenter, pt );	// From center to pt
				PolarDistance = CalculateDistance( MapCenter, pt);

				// Rotate
				PolarDegrees += (double)RotationAmount;

				// Convert back to Rectangular and save the value back
				TempPoint = ConvertPolarToRectangular( MapCenter, PolarDegrees, PolarDistance );

				ROBOT_LOG( TRUE,"DEBUG - ROTATE: Before:%d,%d  After: %d,%d\n", pt.x, pt.y, TempPoint.x, TempPoint.y)
				pStroke->m_pointArray[i].x = TempPoint.x;
				pStroke->m_pointArray[i].y = TempPoint.y;
			}
		}
	}

	// Tell the view to update, and show the rotated map!
	UpdateAllViews( NULL );

}

///////////////////////////////////////////////////////////////////////////
void CMapDoc::ScaleMapPoints( double ScaleAmount ) 
{
	// KLUDGE_SCALE_INSTEAD_OF_ROTATE
	// For maintenance purposes, this allows stretching maps, etc.
	// Not used for normal operation
	// For each stroke point, apply a scale factor

	CPoint pt;
	POSITION pos;

	/////////////////////////////////////////////////////////////////////////////
	// Handle Strokes (includes graphics such as landscape or house plans)

	CTypedPtrList<CObList, CStroke*> *strokeList;

	for(int i=0; i<=5; i++)
	{
		// Note - i does not need to map to the stroke type, it's just an iterator
		// in general draw from solid objects to less solid
		if( 0 == i)	// Draw Walls 
		{	
			strokeList = &m_WallList;
		}
		else if( 1 == i)	// Draw Cliffs
		{	
			strokeList = &m_CliffList;
		}
		else if( 2 == i)	// Draw Obstacles
		{	
			strokeList = &m_ObstacleList;
		}
		else if( 3 == i)	// Draw Obstacles
		{	
			strokeList = &m_LowObstacleList;
		}
		else if( 4 == i)	// Draw Obstacles
		{	
			strokeList = &m_DoorList;
		}
		else if( 5 == i)	// Draw User scribbles
		{	
			strokeList = &m_strokeList;
		}
		else
		{
			ROBOT_LOG( TRUE,"MapDoc.cpp - Bad i in ScaleMapPoints !")
			ASSERT(0);
		}


		pos = strokeList->GetHeadPosition( );
		while (pos != NULL)
		{
			CStroke* pStroke = strokeList->GetNext(pos);
			for( int i=0; i < pStroke->m_pointArray.GetSize(); i++ )
			{
				pt = pStroke->m_pointArray[i];
				double X = (double)pt.x * ScaleAmount;
				double Y = (double)pt.y * ScaleAmount;

				ROBOT_LOG( TRUE,"DEBUG - SCALE MAP POINTS: Before:%d,%d  After: %d,%d\n", pt.x, pt.y, (LONG)X, (LONG)Y)
				pStroke->m_pointArray[i].x = (LONG)X;
				pStroke->m_pointArray[i].y = (LONG)Y;
			}
		}
	}

	// Tell the view to update, and show the rotated map!
	UpdateAllViews( NULL );

}

///////////////////////////////////////////////////////////////////////////
CRect CMapDoc::GetMapBoundry() 
{
	CRect MapBoundry;
	CPoint pt;
	POSITION pos;
	BOOL FirstPoint = TRUE;	// special case the first point for finding boundry

	/////////////////////////////////////////////////////////////////////////////
	// Handle Strokes (includes graphics such as landscape or house plans)

	CTypedPtrList<CObList, CStroke*> *strokeList;

	for(int i=0; i<3; i++)
	{
		// Note - i does not map to a stroke type, it's just an iterator
		if( 0 == i)	// Draw Walls
		{	
			strokeList = &m_WallList;
		}
		else if( 1 == i)	// Draw Obstacles
		{	
			strokeList = &m_ObstacleList;
		}
		else if( 2 == i)	// Draw User scribbles
		{	
			strokeList = &m_strokeList;
		}
		else
		{
			ROBOT_LOG( TRUE,"MapDoc.cpp - Bad i in GetBoundry Strokes!")
			ROBOT_ASSERT(0);
		}
		
		pos = strokeList->GetHeadPosition( );
		while (pos != NULL)
		{
			CStroke* pStroke = strokeList->GetNext(pos);
			for( int i=0; i < pStroke->m_pointArray.GetSize(); i++ )
			{
				pt = pStroke->m_pointArray[i];

				if( FirstPoint )
				{
					MapBoundry.left   = pt.x;
					MapBoundry.right  = pt.x;
					MapBoundry.top    = pt.y;
					MapBoundry.bottom = pt.y;
					FirstPoint = FALSE;
				}
				else
				{
					MapBoundry.left   = min(MapBoundry.left, pt.x);
					MapBoundry.right  = max(MapBoundry.right, pt.x);
					MapBoundry.top    = max(MapBoundry.top, pt.y);
					MapBoundry.bottom = min(MapBoundry.bottom, pt.y);
				}

			}
		}
	}
	return MapBoundry;

}
///////////////////////////////////////////////////////////////////////////

BOOL CMapDoc::SaveModified() 
{
	if(	m_CustomFileOpen )
	{
		return CDocument::SaveModified();	// Invoke the file save dialog
	}
	else
	{
		if (!DoFileSave())
		{
			ROBOT_LOG( TRUE,"ERROR saving file %s!\n",DEFAULT_MAP_FILE)
		}
	}
	return 1;	// continue shutting down
}

void CMapDoc::OnEditUndo() 
{
	if( DRAW_MODE_USER_STROKE == m_DrawMode )
	{
		if( !m_strokeList.IsEmpty( ) )
		{
			delete m_strokeList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_DOOR == m_DrawMode )
	{
		if( !m_DoorList.IsEmpty( ) )
		{
			delete m_DoorList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_LOW_OBSTACLE == m_DrawMode )
	{
		if( !m_LowObstacleList.IsEmpty( ) )
		{
			delete m_LowObstacleList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_OBSTACLE == m_DrawMode )
	{
		if( !m_ObstacleList.IsEmpty( ) )
		{
			delete m_ObstacleList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_WALL == m_DrawMode )
	{
		if( !m_WallList.IsEmpty( ) )
		{
			delete m_WallList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_CLIFF == m_DrawMode )
	{
		if( !m_CliffList.IsEmpty( ) )
		{
			delete m_CliffList.RemoveTail( );
		}
	}
	else if( DRAW_MODE_WAYPOINT_LOCATION == m_DrawMode )
	{
		// Do nothing, handled by PathView
	}
	// For other types, just do nothing, handled by PathView


	UpdateAllViews( NULL );

}

void CMapDoc::SetDrawMode( int  NewDrawMode ) 
{
	m_DrawMode = NewDrawMode;
	ROBOT_LOG( TRUE,"DrawMode set to %d\n", m_DrawMode)
}

int  CMapDoc::GetDrawMode() 
{
	// return current draw mode
	return	m_DrawMode;
}

int  CMapDoc::GetGridmapObjectType( int  DrawMode ) 
{
	// Return object type associated with current drawing mode

	int  ObjectType = GRIDMAP_OBJECT_NONE;

	switch( DrawMode )
	{

		case DRAW_MODE_USER_STROKE:
		{
			ObjectType = GRIDMAP_OBJECT_NONE;
			break;
		}
		case DRAW_MODE_DOOR:
		{
			ObjectType = GRIDMAP_OBJECT_DOOR;
			break;
		}
		case DRAW_MODE_LOW_OBSTACLE:
		{
			ObjectType = GRIDMAP_OBJECT_LOW_OBSTACLE;
			break;
		}
		case DRAW_MODE_OBSTACLE:
		{
			ObjectType = GRIDMAP_OBJECT_OBSTACLE;
			break;
		}
		case DRAW_MODE_WALL:
		{
			ObjectType = GRIDMAP_OBJECT_WALL;
			break;
		}
		case DRAW_MODE_CLIFF:
		{
			ObjectType = GRIDMAP_OBJECT_CLIFF;
			break;
		}
		case DRAW_MODE_WAYPOINT_LOCATION:
		{
			ObjectType = GRIDMAP_OBJECT_NONE;
			break;
		}
		case DRAW_MODE_SET_LOCATION:
		{
			ObjectType = GRIDMAP_OBJECT_NONE;
			break;
		}
		case DRAW_MODE_NAVIGATION_GOAL:
		{
			ObjectType = GRIDMAP_OBJECT_NAVIGATION_FLAG;  // will be ignored by path planning
			break;
		}
		default:
		{
			ROBOT_LOG( TRUE,"ERROR - MapDoc::GetGridmapObjectType - Bad Draw Mode = %d\n", DrawMode)
		}
	}
			
	return	ObjectType;
}




