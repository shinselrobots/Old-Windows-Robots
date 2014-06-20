// GridNavModule.cpp: CGridNavModule class implementation
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "resource.h"
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CGridMap
// This class implements a grid that overlays the RealWorld map
// the grid is used for automatic path planning, using the MicroPather A* solver
/////////////////////////////////////////////////////////////////////////////

CGridMap::CGridMap()
{

	// Clear the GridMap (for indoor use, since this is a set size)
	for( int x=0; x<GRID_MAP_SIZE_MAX; x++ )
	{
		for( int y=0; y<GRID_MAP_SIZE_MAX; y++ )
		{
			m_GridMapArray[x][y] = GRIDMAP_OBJECT_NONE;
		}
	}

	m_pPather = new MicroPather( this, (GRID_MAP_SIZE_MAX/4) );
	ClearPath();
	m_CurrentPathItem =0;

	for( int pts=0; pts <  POINTS_TO_AVERAGE; pts++ )
	{
		m_NextCellPt[pts].x = GRIDMAP_OBJECT_NONE;
		m_NextCellPt[pts].y = GRIDMAP_OBJECT_NONE;
	}
	m_GridMapCurrentSize.x = 0;	// Current size of the used portion of the map
	m_GridMapCurrentSize.y = 0;

}

CGridMap::~CGridMap() 
{
	SAFE_DELETE( m_pPather );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CGridMap::ClearPath()
{
	m_Path.resize( 0 );
	m_CurrentPathItem =0;
}

void CGridMap::ClearPathCache()
{
	// only call if cell cost changes, such as door open/closed
	m_pPather->Reset();
	m_CurrentPathItem =0;
}

void CGridMap::EraseOldPaths()
{
	// Erase old path data from the GridMapArray
	for( int x=0; x<GRID_MAP_SIZE_MAX; x++ )
	{
		for( int y=0; y<GRID_MAP_SIZE_MAX; y++ )
		{
			if( GRIDMAP_OBJECT_NAVIGATION_FLAG == m_GridMapArray[x][y] )
			{
				m_GridMapArray[x][y] = GRIDMAP_OBJECT_NONE;
			}
		}
	}
	m_CurrentPathItem =0;


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
POINT CGridMap::GridCellToRW( POINT GridCell )
{
	// Convert from GridMap(6" squares, or whatever) to RealWorld Map (TenthInches)
	POINT RWPoint;
	RWPoint.x = (LONG)((GridCell.x * GRID_MAP_RESOLUTION) - HALF_GRID_MAP_RESOLUTION);
	RWPoint.y = (LONG)((GridCell.y * GRID_MAP_RESOLUTION) - HALF_GRID_MAP_RESOLUTION);
	return RWPoint;
}

POINT CGridMap::RWToGridCell( POINT RWPoint )
{
	// Convert from RealWorld Map (TenthInches) to GridMap(6" squares, or whatever)
	POINT GridCell;
	GridCell.x = (LONG)((RWPoint.x + HALF_GRID_MAP_RESOLUTION) / GRID_MAP_RESOLUTION);
	GridCell.y = (LONG)((RWPoint.y + HALF_GRID_MAP_RESOLUTION) / GRID_MAP_RESOLUTION);
	return GridCell;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
POINT CGridMap::GetNextCellLocationRW()
{
	// Get next cell and convert to RW coordinates
	POINT NextCellPt, Sum;
	NextCellPt.x = -1;
	NextCellPt.y = -1;

	int Nx = NextCellPt.x;
	int Ny = NextCellPt.y;
	int k;


	if( m_CurrentPathItem < (int)m_Path.size() )
	{
		NodeToXY( m_Path[m_CurrentPathItem], &Nx, &Ny );

		if( ((int)m_Path.size() -1) == m_CurrentPathItem )
		{
			// Last item = final destation
			NextCellPt.x = (LONG)(Nx * GRID_MAP_RESOLUTION);
			NextCellPt.y = (LONG)(Ny * GRID_MAP_RESOLUTION);
		}
		else
		{
			// Path Smoothing
			// Shift values and add new point
			for( k=0; k < (POINTS_TO_AVERAGE-1); k++ ) 
			{
				m_NextCellPt[k].x = m_NextCellPt[k+1].x;
				m_NextCellPt[k].y = m_NextCellPt[k+1].y;
			}
			m_NextCellPt[k].x = Nx;
			m_NextCellPt[k].y = Ny;

			// Find the average of the points
			Sum.x = 0;
			Sum.y = 0;
			for( k=0; k<POINTS_TO_AVERAGE; k++ ) 
			{
				Sum.x += m_NextCellPt[k].x;
				Sum.y += m_NextCellPt[k].y;
			}

			NextCellPt.x = (LONG)((Sum.x * GRID_MAP_RESOLUTION) / POINTS_TO_AVERAGE);
			NextCellPt.y = (LONG)((Sum.y * GRID_MAP_RESOLUTION) / POINTS_TO_AVERAGE);

		}

		m_CurrentPathItem++;


	}
	else
	{

		ROBOT_LOG( TRUE,"GridMap: Path Complete.\n")
	}
	return NextCellPt;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
int CGridMap::FindPath(POINT Start, POINT Finish)
{
	float TotalCost;
	m_CurrentPathItem =0;


	// Convert from RealWorld coordinates (inches) to GridMap Coordinates (cell size)
	POINT CellStart; 
	POINT CellFinish;


	double X = Start.x + 0.5; // Straight integer cast will truncate, this will round instead
	double Y = Start.y + 0.5;
	CellStart.x = (LONG)((X / GRID_MAP_RESOLUTION));
	CellStart.y = (LONG)((Y / GRID_MAP_RESOLUTION));

	X = Finish.x + 0.5; // Straight integer cast will truncate, this will round instead
	Y = Finish.y + 0.5;
	CellFinish.x = (LONG)((X / GRID_MAP_RESOLUTION));
	CellFinish.y = (LONG)((Y / GRID_MAP_RESOLUTION));




	// Make sure the user did not click on an illegal spot
	if( m_GridMapArray[CellFinish.x][CellFinish.y] >= GRIDMAP_OBJECT_PATH_BLOCKING ) 
	{
		CString MsgString;
		MsgString.Format( "GridMap: Don't send me into an object!");
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		return NO_SOLUTION;
	}

	// and find the best path

	int result = m_pPather->Solve( 
		XYToNode( CellStart.x,  CellStart.y ), 
		XYToNode( CellFinish.x, CellFinish.y ), 
		&m_Path, &TotalCost );



/*	int result = m_pPather->Solve( 
		XYToNode( (Start.x/GRID_MAP_RESOLUTION),  (Start.y/GRID_MAP_RESOLUTION) ), 
		XYToNode( (Finish.x/GRID_MAP_RESOLUTION), (Finish.y/GRID_MAP_RESOLUTION) ), 
		&m_Path, &TotalCost );
*/
	if( SOLVED != result )
	{
		CString MsgString;
		MsgString.Format( "GridMap: Can't find Path from %d,%d to %d,%d!", 
			Start.x, Start.y, Finish.x, Finish.y  );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	}
	else
	{

		CString MsgString;
		MsgString.Format( "GridMap: Path found from %d,%d to %d,%d!.  Cost = %3.2f", 
			Start.x, Start.y, Finish.x, Finish.y, TotalCost );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )


		int  k;
		ROBOT_LOG( TRUE, "\n\nDEBUG - PATH FOUND: \n")
		int Px, Py;
		for( k=0; k < (int)m_Path.size(); ++k ) 
		{
			NodeToXY( m_Path[k], &Px, &Py );
			ROBOT_LOG( TRUE,"Step %d = (%d,%d)\n", k, Px, Py)
			// Make the Path show up on the map! 
			AddGridPoint(Py, Px, GRIDMAP_OBJECT_NAVIGATION_FLAG);		
		}

		// Initialize Path Smoothing
		for( k=0; k<POINTS_TO_AVERAGE; k++ ) 
		{
			NodeToXY( m_Path[k], &Px, &Py );
			m_NextCellPt[k].x = Px;
			m_NextCellPt[k].y = Py;
		}
	
	}

	return result;	//	SOLVED,	NO_SOLUTION, START_END_SAME

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
int  CGridMap::GetGridMapCell(POINT Cell)
{ 
	int  CellValue =  m_GridMapArray[Cell.x][Cell.y]; 
	return CellValue;
}

int  CGridMap::GetGridMapCell(int  x, int  y)
{ 
	int  CellValue =  m_GridMapArray[x][y]; 
	return CellValue;
}


// Each square cell on the map is called a "node"
// Nodes are passed around by the MicoPanther A* search
void CGridMap::NodeToXY( void* node, int* x, int* y ) 
{
	*y = HIWORD( (DWORD)node );
	*x = LOWORD( (DWORD)node );
}

void* CGridMap::XYToNode( int x, int y )
{
	DWORD 	Node = (DWORD)MAKELONG(x,y); //LOWORD, HIWORD
	return (void*) ( Node );
}
		

float CGridMap::LeastCostEstimate( void* nodeStart, void* nodeEnd ) 
	{
		int xStart, yStart, xEnd, yEnd;
		NodeToXY( nodeStart, &xStart, &yStart );
		NodeToXY( nodeEnd, &xEnd, &yEnd );

		// Compute the minimum path cost using distance measurement. It is possible
		// to compute the exact minimum path using the fact that you can move only 
		// on a straight line or on a diagonal, and this will yield a better result.

		int dx = xStart - xEnd;
		int dy = yStart - yEnd;
		return (float) sqrt( (double)(dx*dx) + (double)(dy*dy) );

	}

void CGridMap::AdjacentCost( void* node, std::vector< StateCost > *neighbors ) 
	{
		int x, y;
		const int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
		const int dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
		const float cost[8] = { 1.0f, 1.41f, 1.0f, 1.41f, 1.0f, 1.41f, 1.0f, 1.41f };

		NodeToXY( node, &x, &y );

		for( int i=0; i<8; ++i ) 
		{
			int nx = x + dx[i];
			int ny = y + dy[i];

			// Get cost of having path use this cell
			if (    nx >= 0 && nx < GRID_MAP_SIZE_MAX 
				 && ny >= 0 && ny < GRID_MAP_SIZE_MAX )
			{
				// inside the map

				int  CellValue = GetGridMapCell(nx, ny);
				if( CellValue >= GRIDMAP_OBJECT_PATH_BLOCKING )
				{
					// Impassable object in this location
					StateCost nodeCost = { XYToNode( nx, ny ), FLT_MAX };	// Blocked by object
					neighbors->push_back( nodeCost );
				}
				else
				{
					// multiply cell value cost times diagnal cost multiplier
					StateCost nodeCost = { XYToNode( nx, ny ), (cost[i] * CellValue) };
					neighbors->push_back( nodeCost );
				}
			}
			else
			{
				// Outside Edge of the map!
				StateCost nodeCost = { XYToNode( nx, ny ), FLT_MAX };	
				neighbors->push_back( nodeCost );
			}
		}
	}

void CGridMap::PrintStateInfo( void* node ) 
	{
		int x, y;
		NodeToXY( node, &x, &y );
		ROBOT_LOG( TRUE, "CGridMap State Info: %d,%d\n", x, y )
	}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CGridMap::AddGridPoint(int y, int x, int  ObjectType )
{

	// Trap bad values
	if( x < 0 ) x = 0;
	if( y < 0 ) y = 0;
	if( x > (GRID_MAP_SIZE_MAX-1) ) x = (GRID_MAP_SIZE_MAX-1);
	if( y > (GRID_MAP_SIZE_MAX-1) ) y = (GRID_MAP_SIZE_MAX-1);


	// Mark this cell as occupied by a wall or object
	m_GridMapArray[x][y] = (BYTE)ObjectType;

	// For drawing optmization, keep track of how much of the map is used
	if(x > m_GridMapCurrentSize.x ) m_GridMapCurrentSize.x = x; 
	if(y > m_GridMapCurrentSize.y ) m_GridMapCurrentSize.y = y;
	TRACE("DEBUG: GridMapSize = %d, %d\n", m_GridMapCurrentSize.x, m_GridMapCurrentSize.y );

	// Because we don't want the robot to come too close to walls and objects, mark the 
	// surrounding cells as "expensive" to encourage the path planner to stay out of them
	// if possible.

	if( ObjectType >= GRIDMAP_OBJECT_PATH_BLOCKING )
	{
		// yep, it's an object of some sort, raise the cost of surrounding cells
		// We work with a grid of 3 cells to each side, Cost = 2 to 4 (higher closer to the object)

		int dx, dy;	// delta from current cell
		int nx, ny;	// neighbor cell location
		int ax, ay; // absolute values of index
		int Cost;	// Path Cost to assign the cell
		for( dx = -4; dx <= 4; dx++ )
		{
			for( dy = -4; dy <= 4; dy++)
			{
				nx = x + dx;
				ny = y + dy;
				if (    nx >= 0 && nx < GRID_MAP_SIZE_MAX 
					 && ny >= 0 && ny < GRID_MAP_SIZE_MAX )
				{
					ax = abs(dx);
					ay = abs(dy);
					// for immediately ajacent cells, mark as impassible!
					// this is becase the Robot has "width" but the path planner
					// does not think about that, and will try to let the robot
					// squeeze through a 6 inch hole! :-)
					if( (ax  < 1) && (ay < 1) )
					{
						// immediately ajacent 2 cells (within 12 inches)
						Cost = ObjectType;
					}
					else if( ax > ay )
					{
						Cost = 5 - ax;
					}
					else
					{
						Cost = 5 - ay;
					}

					if(	Cost > m_GridMapArray[nx][ny] )
					{
						// higher cost then whatever may already be there
						m_GridMapArray[nx][ny] = (BYTE)Cost;

						// Expand map size used for cost indication squares
						if(nx > m_GridMapCurrentSize.x ) m_GridMapCurrentSize.x = nx; 
						if(ny > m_GridMapCurrentSize.y ) m_GridMapCurrentSize.y = ny; 

					}
				}
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CGridMap::FindNearestObject(
		POINT	 RWPhantomObject,				// IN  Object X,Y relative to robot
		int 	 SearchDistanceTenthInches,		// IN  Distance in TenthInches to search from current location
		POINT	&RWNearestObjectDelta,			// OUT X,Y difference between sensor reading and Map
		int 	&RWNearestObjectDistance,		// OUT distance between sensor reading and Map
		int 	&ObjectType )					// OUT Type of object that was found on the Map
{
	// find the nearest object to the given Real World x,y coords.

	POINT NearestObjectDeltaCell = { GRID_MAP_SIZE_MAX, GRID_MAP_SIZE_MAX };
	int   NearestObjectDistanceCell = GRID_MAP_SIZE_MAX;
	double  CurrentObjectDistanceCell;

	// Convert to Gridmap Cells
	POINT PhantomObjectCell = RWToGridCell( RWPhantomObject );	// Convert from RealWorld Map (TenthInches) to GridMap(6" squares, or whatever)

	// Trap bad values

	if( 0 == m_GridMapCurrentSize.x || 0 == m_GridMapCurrentSize.y )
	{	// Empty map
		return;
	}

	if( (PhantomObjectCell.x < 0) || (PhantomObjectCell.y < 0)	||
		(PhantomObjectCell.x+1 > (m_GridMapCurrentSize.x) )		||
		(PhantomObjectCell.y+1 > (m_GridMapCurrentSize.y) )		 )	// m_GridMapCurrentSize might be zero
	{
		ROBOT_LOG( TRUE,"ERROR CGridMap::FindNearestObject: Bad PhantomObjectCell value, ")

		if( (PhantomObjectCell.x < 0) || (PhantomObjectCell.y < 0) )
		{
			ROBOT_LOG( TRUE,"PhantomObjectCell X,Y = Negative Value!  ")
		}
		if(	(PhantomObjectCell.x+1 > (m_GridMapCurrentSize.x) )	)
		{
			ROBOT_LOG( TRUE,"PhantomObjectCell.x > m_GridMapCurrentSize.x  ")
		}
		if( (PhantomObjectCell.y+1 > (m_GridMapCurrentSize.y) )		 )
		{
			ROBOT_LOG( TRUE,"PhantomObjectCell.y > m_GridMapCurrentSize.y ")
		}

		ROBOT_LOG( TRUE,"\n")
		return;
	}

	int DeltaX, DeltaY;	// delta from current cell
	int NeighborX, NeighborY;	// neighbor cell location

	// Convert from TenthInches to grid size
	int CellSearchDistance = (int)(((float)SearchDistanceTenthInches + HALF_GRID_MAP_RESOLUTION) / GRID_MAP_RESOLUTION);
	DeltaX = -CellSearchDistance;

	for( DeltaX = -CellSearchDistance; DeltaX <= CellSearchDistance; DeltaX++ )
	{
		for( DeltaY = -CellSearchDistance; DeltaY <= CellSearchDistance; DeltaY++)
		{
			NeighborX = PhantomObjectCell.x + DeltaX;
			NeighborY = PhantomObjectCell.y + DeltaY;
			if( (NeighborX >= 0 && NeighborX < m_GridMapCurrentSize.x) && 
				(NeighborY >= 0 && NeighborY < m_GridMapCurrentSize.x) )
			{
				ObjectType = m_GridMapArray[NeighborX][NeighborY];
				if( ObjectType >= GRIDMAP_OBJECT_PATH_BLOCKING )
				{
					// found an object.
					CurrentObjectDistanceCell = sqrt( (double)(DeltaX*DeltaX) + (double)(DeltaY*DeltaY) );
					if( CurrentObjectDistanceCell < NearestObjectDistanceCell )
					{
						// New nearest object
						NearestObjectDistanceCell = int (CurrentObjectDistanceCell);
						NearestObjectDeltaCell.x = DeltaX;
						NearestObjectDeltaCell.y = DeltaY;
					}
				}
			}
		}
	}

	// Ok, got the delta to the nearest object's cell position.  Convert back to Real World
	RWNearestObjectDelta = GridCellToRW( NearestObjectDeltaCell );
	RWNearestObjectDistance = (int )(((double)NearestObjectDistanceCell * GRID_MAP_RESOLUTION) - HALF_GRID_MAP_RESOLUTION);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CGridMap::AddGridMapData(CArray<CPoint, CPoint> &PointArray, int  ObjectType)
{
	// Line segments to add to the Grid Map
	CPoint PrevPt = PointArray[0];
	CPoint CurrPt = PrevPt;
	CPoint Step;
	int i;               // loop counter 
	int ystep, xstep;    // the step on y and x axis 
	int error;           // the error accumulated during the increment 
	int errorprev;       // *vision the previous value of the error variable 
	int ddy, ddx;     // compulsory variables: the double values of dy and dx 

	for (int nSegment=1; nSegment < PointArray.GetSize(); nSegment++)
	{
		CurrPt = PointArray[nSegment]; // X,Y in TenthInches from Origin


		// Fill GridMap entries for line from PrevPt to CurrPt
		// This uses Bresenham's Algorithm, as modified by Eugen Dedu
		// for a supercover line algorithm (fills in more points)
		// http://lifc.univ-fcomte.fr/~dedu/projects/bresenham/index.html
		// If the line passes through a corner, the both squares are drawn.  
		// If you want to remove this, you can simply remove the else part dealing with the corner.

		// Convert from RealWorld Map (TenthInches) to GridMap(6" squares, or whatever)
		int x1 = (int)(((float)PrevPt.x + GRID_MAP_RESOLUTION/2.0) / GRID_MAP_RESOLUTION);
		int y1 = (int)(((float)PrevPt.y + GRID_MAP_RESOLUTION/2.0) / GRID_MAP_RESOLUTION);
		int x2 = (int)(((float)CurrPt.x + GRID_MAP_RESOLUTION/2.0) / GRID_MAP_RESOLUTION);
		int y2 = (int)(((float)CurrPt.y + GRID_MAP_RESOLUTION/2.0) / GRID_MAP_RESOLUTION);

		int y = y1;
		int x = x1;  // the line points 
		int dx = x2 - x1; 
		int dy = y2 - y1; 
		AddGridPoint( y1, x1, ObjectType );  // first point 
		if (dy < 0)
		{ 
			ystep = -1; 
			dy = -dy; 
		}
		else
		{
			ystep = 1; 
		}
		if (dx < 0)
		{ 
			xstep = -1; 
			dx = -dx; 
		}
		else
		{
			xstep = 1; 
		}
		ddy = 2 * dy;  // work with "doubled" values for full precision 
		ddx = 2 * dx; 
		if (ddx >= ddy)
		{  
			// first octant (0 <= slope <= 1) 
			// compulsory initialization (even for errorprev, needed when dx==dy) 
			errorprev = error = dx;  // start in the middle of the square 
			for (i=0 ; i < dx ; i++)
			{  
				// do not use the first point (already done) 
				x += xstep; 
				error += ddy; 
				if (error > ddx)
				{
					// increment y if AFTER the middle ( > ) 
					y += ystep; 
					error -= ddx; 
					// three cases (octant == right->right-top for directions below): 
					if (error + errorprev < ddx)  // bottom square also 
						AddGridPoint( y-ystep, x, ObjectType ); 
					else if (error + errorprev > ddx)  // left square also 
						AddGridPoint( y, x-xstep, ObjectType ); 
					else
					{  
						// corner: bottom and left squares also 
						AddGridPoint( y-ystep, x, ObjectType ); 
						AddGridPoint( y, x-xstep, ObjectType ); 
					} 
				} 
				AddGridPoint( y, x, ObjectType ); 
				errorprev = error; 
			} 
		}
		else
		{
			// the same as above 
			errorprev = error = dy; 
			for (i=0 ; i < dy ; i++)
			{ 
				y += ystep; 
				error += ddx; 
				if (error > ddy)
				{ 
					x += xstep; 
					error -= ddy; 
					if (error + errorprev < ddy) 
						AddGridPoint( y, x-xstep, ObjectType ); 
					else if (error + errorprev > ddy) 
						AddGridPoint( y-ystep, x, ObjectType ); 
					else
					{ 
						AddGridPoint( y, x-xstep, ObjectType ); 
						AddGridPoint( y-ystep, x, ObjectType ); 
					} 
				} 
				AddGridPoint( y, x, ObjectType ); 
				errorprev = error; 
			} 
			// ROBOT_ASSERT ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm 
		}

		PrevPt = CurrPt;	// use as starting point for next line segment

	}	// end for each segment

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//	MODULE: GridNavModule
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#if ( ROBOT_SERVER == 1 )

/* State Machine:
	DISABLED
	IDLE
	GET_NEXT_WAYPOINT
	TURNING_TOWARD_WAYPOINT
	DRIVING_TO_WAYPOINT
	WAYPOINT_NEAR
	WAYPOINT_REACHED
  */



CGridNavModule::CGridNavModule( CDriveControlModule *pDriveControlModule, CSensorModule *pSensorModule )
{
	m_pDriveCtrl = pDriveControlModule;
	InitVariables();
//	NOTE: GLOBAL: g_PGridMap;
	// Note: pSensorModule passed in, so Gridmap can do things like provide compass correction
	// (not implemented yet, see NavModule.cpp)

}

void CGridNavModule::InitVariables()
{
	m_GridPathStarted = FALSE;
	m_Goal.x = LONG_MAX;
	m_Goal.y = LONG_MAX;
	m_NavState = IDLE;
}


void CGridNavModule::CancelPath()
{
	m_pDriveCtrl->SetSpeedAndTurn( GRID_NAV_MODULE, SPEED_STOP, TURN_CENTER );
	m_pDriveCtrl->ReleaseOwner( GRID_NAV_MODULE );
	InitVariables();
	ROBOT_DISPLAY( TRUE, "Cancel or End of Path.  GridNav PathState: IDLE" )
	m_NavState = IDLE;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CGridNavModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (lParam);

	CString MsgString;
	//BYTE nDistHigh, nDistLow;
	switch( uMsg ) 
	{
		case WM_ROBOT_GOTO_GRID_LOCATION_CMD:
		{
			// Command from user to head to location!
			g_bCmdRecognized = TRUE;
			// wParam = xy point
			// lParam = not used

			if( NULL == g_pGridMap )
			{
				// No Map created yet, so just return;
				MsgString.Format( "ERROR! WM_ROBOT_GOTO_GRID_LOCATION_CMD - No Grid Map Created Yet!" );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				return;
			}
			// Initialize state machine to head for target location

			m_Goal.x = LOWORD(wParam);
			m_Goal.y = HIWORD(wParam);

			// Erase any old paths
			g_pGridMap->EraseOldPaths();

			MsgString.Format( "Finding Path to %d, %d",	m_Goal.x, m_Goal.y );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			// Find the Path
			if( SOLVED == g_pGridMap->FindPath( FPointToPoint(g_pFullSensorStatus->CurrentLocation), m_Goal ) )
			{
				m_NextCellLocationRW = g_pGridMap->GetNextCellLocationRW();
				m_NavState = TURN_TO_NEXT_WAYPOINT;	// turn to the direction of the first cell
				// Set object avoidence to Grid Map Navigation default
				g_SegmentAvoidObjectRangeTenthInches = GRID_MAP_AVOID_OBJECT_DISTANCE;			

				// Start Executing path!
				// User said it's ok, so release user control priority
//				m_pDriveCtrl->ReleaseOwner( LOCAL_USER_MODULE );
				if( !m_pDriveCtrl->CheckAndSetOwner( GRID_NAV_MODULE ) )
				{
					ROBOT_DISPLAY( TRUE, "GridNav: Unable to set Owner of DriveControl.  will keep trying...\n" )
				}
			}
			else
			{
				m_NavState = IDLE;
			}

			// Post message to the GUI display (local or remote), to repaint the display
			SendResponse( WM_ROBOT_UPDATE_VIEW,	0,0 );

			return;
		}
		break;


		case WM_ROBOT_EXECUTE_PATH:
		{
			/*
			#define PATH_EXECUTE_CANCEL					0x00
			#define NAV_PATH_EXECUTE_START				0x01
			#define GRID_PATH_EXECUTE_START				0x02
			#define PATH_EXECUTE_PAUSE					0x03
			#define PATH_EXECUTE_RESUME					0x04
			*/
			// Command from user to execute the loaded Path!
			// wParam: Execution Mode
			// lParam: 1 = Wait for switch to start		0 = Don't wait (NOT USED)
			g_bCmdRecognized = TRUE;

			if( PATH_EXECUTE_CANCEL == wParam )	// Grid Path Cancel
			{
				if( m_GridPathStarted )
				{
					// Stop executing path
					ROBOT_DISPLAY( TRUE, "Path Execution Cancelled!" )
					CancelPath();
				}
				return;
			}

			if( PATH_EXECUTE_PAUSE == wParam )
			{
				if( m_GridPathStarted && !m_PausePath )
				{
					// If path started and not already paused! (if paused would get bogus speed and turn)
					m_PausePath = TRUE;
					m_PauseSpeed = m_pDriveCtrl->GetCurrentSpeed();
					m_PauseTurn = m_pDriveCtrl->GetCurrentTurn();
					m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_STOP );
					ROBOT_DISPLAY( TRUE, "Path Paused!" )
				}
				return;
			}

			// Starting or resuming a path.  Make sure we have a valid GridMap
			if( NULL == g_pGridMap )
			{
				// No Map created yet, so just return;
				MsgString.Format( "ERROR! WM_ROBOT_EXECUTE_PATH - No Grid Map Created Yet!" );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				return;
			}

			if( GRID_PATH_EXECUTE_START == wParam )	// Grid Path Start
			{
				// Start Executing path!
				// User said it's ok, so release user control priority
				m_pDriveCtrl->ReleaseOwner( LOCAL_USER_MODULE );
				
				// Initialize state machine to head for Goal waypoint
				m_GridPathStarted = TRUE;
				m_PausePath = FALSE;
				m_NavState = START_NEW_PATH;
				ROBOT_DISPLAY( TRUE, "GridNav PathState: START_NEW_PATH" )
				ROBOT_DISPLAY( TRUE, "Executing Path!" )
				return;
			}

			if( PATH_EXECUTE_RESUME == wParam )
			{
				if( m_GridPathStarted )
				{
					m_PausePath = FALSE;
//ERROR??					m_pDriveCtrl->ReleaseOwner( 0xFF );	// Override all other owners!
					m_pDriveCtrl->SetSpeedAndTurn( GRID_NAV_MODULE, m_PauseSpeed, m_PauseTurn );
					ROBOT_DISPLAY( TRUE, "Path Resumed!" )
				}
			}
			else
			{
				// should never get here
				ROBOT_ASSERT(0);
			}

			return;
		}
		break;

		case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			if( NULL == g_pGridMap )
			{
				// No Map created yet, so just return;
				return;
			}

			if( IDLE == m_NavState )
			{
				// Nothing to do
				return;
			}

			if( m_PausePath )
			{
				// Path has been paused.  Don't do anything until resume command issued.
				return;
			}

			if( (!m_pDriveCtrl->IsOwner(GRID_NAV_MODULE)) &&
				(m_NavState >= TURN_TO_NEXT_WAYPOINT)		)
			{
				// We were following a path, but lost control of the drive wheels
				// Due to collision or other problem.
				// See if we have regained control yet. 
				if( m_pDriveCtrl->CheckAndSetOwner( GRID_NAV_MODULE ) )
				{
					// Control regained, recalculate the route to the destination
					if( SOLVED == g_pGridMap->FindPath( FPointToPoint(g_pFullSensorStatus->CurrentLocation), m_Goal ) )
					{
						m_NextCellLocationRW = g_pGridMap->GetNextCellLocationRW();
						m_NavState = TURN_TO_NEXT_WAYPOINT;	// turn to the direction of the first cell
					}
					else
					{
						CancelPath();
					}

					ROBOT_DISPLAY( TRUE, "GridNav: Unable to set Owner of DriveControl.  will keep trying...\n" )
				}

				// Post message to the GUI display (local or remote), to repaint the display
				SendResponse( WM_ROBOT_UPDATE_VIEW,	0,0 );
				return;
			}

CheckNavState:			

			int Distance = CalculateDistance( FPointToPoint(g_pFullSensorStatus->CurrentLocation), m_NextCellLocationRW );
			if( Distance <= (GRID_MAP_RESOLUTION *2) )
			{
				// Cell reached.  Head to next cell!
				MsgString.Format( "Location %d, %d reached.  Distance = %d  Heading for next one.",
					m_NextCellLocationRW.x, m_NextCellLocationRW.y, Distance );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

				m_NextCellLocationRW = g_pGridMap->GetNextCellLocationRW();	// get next cell location

				// See if we have reached the end of the path
				if( (-1 == m_NextCellLocationRW.x) &&	(-1 == m_NextCellLocationRW.y) )
				{
					// End of path reached
					m_NavState = END_OF_PATH_REACHED;
				}
				else
				{
					m_NavState = TURN_TO_NEXT_WAYPOINT;
				}
			}


			///////////////////////////////////////////////////////////////////
			// Process GridNav State Machine

			// TODO - Sometimes this applies but not always 
			//  consider avoid object, but close to goal - could go past goal!
			//  need concept of distance to goal!  If distance to goal < distance to object, ignore object?

			int  HeadingToNextCell = CalculateAngle( FPointToPoint(g_pFullSensorStatus->CurrentLocation), m_NextCellLocationRW);
			int TurnDegrees = CalculateTurn(g_pFullSensorStatus->CompassHeading, HeadingToNextCell);

			switch (m_NavState) 
			{
				case IDLE:
				{
					// Nothing to do
					return;
				}	
				break;


				case TURN_TO_NEXT_WAYPOINT:
				{
					// Time to head toward the next waypoint
					// First we turn, then we drive toward the waypoint

					if( abs(TurnDegrees) < 15 )
					{
						// Heading close to the right direction.  Move forward at recommended speed
						ROBOT_DISPLAY( TRUE, "PathState: DRIVING_TO_WAYPOINT" )
						m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_FWD_MED );	
						m_NavState = DRIVING_TO_WAYPOINT;
						goto CheckNavState;
					}

					if( TurnDegrees < 0 )
					{	// Turn Left
						if( abs(TurnDegrees) >= 60 )
						{
							// turn as sharp as possible
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_LEFT_MED );	// for ER1, TURN_LEFT_FAST makes it erratic
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_STOP );
							
						}
						else if( abs(TurnDegrees) >= 30 )
						{
							// turn sharp 
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_LEFT_MED );
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_STOP );
							
						}
						else
						{
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_LEFT_MED_SLOW );	
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_FWD_SLOW );
						}
					}
					else
					{	// Turn Right
						if( abs(TurnDegrees) >= 60 )
						{
							// turn as sharp as possible
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_RIGHT_MED );	
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_STOP );
						}
						else if( abs(TurnDegrees) >= 30 )
						{
							// turn sharp 
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_RIGHT_MED );
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_STOP );
							
						}
						else
						{
							m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, TURN_RIGHT_MED_SLOW );	
							m_pDriveCtrl->SetSpeed( GRID_NAV_MODULE, SPEED_FWD_SLOW );
						}
					}

				}
				break;
						
				case DRIVING_TO_WAYPOINT:
				{
					if( abs(TurnDegrees) > 100 )
					{
						// Probably drove past the next cell!
						// Just skip it and go to the next one 
						// (If we went past it too, we'll back track to get on course)
						MsgString.Format( "Passed location (%d,%d), skipping to next one.",
							m_NextCellLocationRW.x, m_NextCellLocationRW.y );
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						m_NextCellLocationRW = g_pGridMap->GetNextCellLocationRW();	// get next cell location
						m_NavState = TURN_TO_NEXT_WAYPOINT;
						goto CheckNavState;
					}

					int NewTurn = (int)(TurnDegrees * TURN_MULTIPLIER);
					m_pDriveCtrl->SetTurn( GRID_NAV_MODULE, NewTurn );	

				}
				break;

		
				case END_OF_PATH_REACHED:
				{
					// Done with path.  Go back to Idle and wait for the next command.
					// Someday, maybe play music here? :-)
					ROBOT_DISPLAY( TRUE, "GridNav Module: End of Path Reached!\n" )
					CancelPath();
				}
				break;

				default:
				{
					MsgString.Format( "ERROR! Illegal m_NavState = %0xU", m_NavState );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					CancelPath();
				}
				break;
			}	// Switch NavState

		}	// end Case Status Ready
		break;


		default:
		{
			// It's OK to have messages that are not handled by this module!
			//ROBOT_LOG( TRUE, "Note:Cmd not handled by NavModule: %02X", uMsg )
		}
	}	// switch( uMsg )
	
}	// End ProcessMessage
#endif // ROBOT_SERVER
//////////////////////////////////////////////////////////////////////////////////////////////////////////


