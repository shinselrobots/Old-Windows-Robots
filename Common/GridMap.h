
// GridMap: CGridMap class implementation
// Stores world data as a grid map of tiles, and provides 
// A* navigation using MicroPather 
//
//////////////////////////////////////////////////////////////////////

#ifndef __GRIDMAP_H__
#define __GRIDMAP_H__

#include "RobotType.h"
#include "micropather.h"	// required here since Map Doc inherits from MicroPanther class "Graph"
using namespace micropather;

/*
#define NO_STROKE				00
#define USER_STROKE				01
#define WALL_STROKE				02
#define OBSTACLE_STROKE			03
#define WAYPOINT_LOCATION		04
#define NAVIGATION_MODE			05
#define NUMBER_OF_STROKE_TYPES	06	// This should be number of types listed above
*/

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

// Grid Map Object Types
//#define NAVIGATION_FLAG_OBJECT	0x00	// will be ignored by path planning
//#define NO_OBJECT					0x01	// 1 is the default path "penalty"
//#define WALL_OBJECT				0xFF

#define GRIDMAP_OBJECT_NAVIGATION_FLAG		0x00	// will be ignored by path planning
#define GRIDMAP_OBJECT_NONE					0x01	// 1 is the default path "penalty"
#define GRIDMAP_OBJECT_DOOR					0x02

// Numbers greater then GRIDMAP_OBJECT_PATH_BLOCKING are considered impassable objects
#define GRIDMAP_OBJECT_PATH_BLOCKING	0xF0
#define GRIDMAP_OBJECT_LOW_OBSTACLE		0xFC
#define GRIDMAP_OBJECT_OBSTACLE			0xFD
#define GRIDMAP_OBJECT_WALL				0xFE
#define GRIDMAP_OBJECT_CLIFF			0xFF

#define GRID_MAP_RESOLUTION			60.0	// TenthInches - Size of each square in the GridMap (6")
#define HALF_GRID_MAP_RESOLUTION	(GRID_MAP_RESOLUTION/2)	// For drawing box around location
#define GRID_MAP_SIZE_MAX		  1000	// Number of squares in each dimension
#define POINTS_TO_AVERAGE			3	// For path smoothing


////////////////////////////////////////////////////////////////////////////
// class CGridMap
class CGridMap : public Graph	// inheret from MicroPanther Graph class
{

public:
CGridMap();
virtual ~CGridMap();


// Attributes
protected:
	BYTE	m_GridMapArray[GRID_MAP_SIZE_MAX][GRID_MAP_SIZE_MAX];
	std::vector<void*> m_Path;	// Path found by Pather
	MicroPather* m_pPather;		// Pointer to the Pather path finding class
	int 	m_CurrentPathItem;	// Current Item in the path, incremented as we reach each cell
	POINT	m_NextCellPt[POINTS_TO_AVERAGE];
	POINT	m_GridMapCurrentSize;	// Current size of the used portion of the map

// Operations
public:
	void	ClearMap();		// Clear out the map
	void	AddGridPoint(int y, int x, int  ObjectType);


	void	FindNearestObject(
				POINT RWPhantomObject, int  SearchDistance,										// IN Params
				POINT &RWNearestObjectDelta, int   &RWNearestObjectDistance, int  &ObjectType );// OUT Params

	void	AddGridMapData(CArray<CPoint, CPoint> &PointArray, int  StrokeType);
	int 	GetGridMapCell(POINT Cell);		// Given a POINT, Get cell value
	int 	GetGridMapCell(int  x, int  y);	// x,y version of the same
	POINT	GetMapSize() { return m_GridMapCurrentSize; } // Inline

	// int 	GetGridMapCell(POINT Cell) { return (int )(m_GridMapArray[Cell.x][Cell.y]); } // Inline

	void	NodeToXY( void* node, int* x, int* y );	// For storing cell info
	void*	XYToNode( int x, int y );		
	void	ClearPath();	
	int		FindPath(POINT Start, POINT Finish);
	float	LeastCostEstimate( void* nodeStart, void* nodeEnd ); 
	void	AdjacentCost( void* node, std::vector< StateCost > *neighbors ); 
	void	PrintStateInfo( void* node ); 
	void	ClearPathCache();
	void	EraseOldPaths();
	POINT	GetNextCellLocationRW();
	POINT	GridCellToRW( POINT GridCell );	// Convert from GridMap(6" squares, or whatever) to RealWorld Map (TenthInches)
	POINT	RWToGridCell( POINT RWPoint );	// Convert from RealWorld Map (TenthInches) to GridMap(6" squares, or whatever)

};

#endif

