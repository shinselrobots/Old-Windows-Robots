// PathStruct.h : 
// Data structures used for storing Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////
#if !defined(ROBOT_PATHSTRUCT_H)
#define ROBOT_PATHSTRUCT_H

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "RobotType.h"
//#include "..\Common\HardwareCmds.h"


// Align values with radio button settings
#define SEGMENT_FOLLOW_NONE		-1
#define SEGMENT_FOLLOW_LEFT		0
#define SEGMENT_FOLLOW_RIGHT	1


typedef struct	tagFPOINT
{
    double  x;
    double  y;
} FPOINT;		// double POINT structure, for higher accuracy


/////////////////////////////////////////////////////////////////////////////
// Name: SegmentStruct - Holds the list of all Path Segments
//
class CSegmentStruct
{
// Attributes
public:

	CString	m_SegmentName;
	int 	m_SegmentFromWaypointID;
	int 	m_SegmentToWaypointID;
	CString	m_SegmentBehavior;
	CString	m_SegmentSpeed;
	int 	m_SegmentDirection;
	int		m_CompassCorrection;	// Compass error correction to add to Direction
	int 	m_SegmentDistanceFeet;
	int 	m_SegmentDistanceInches;
	int 	m_SegmentFollowDistance;
	int 	m_SegmentAvoidDistance;
	int		m_SegmentFollowLeftRight;


// Operations
public:
	void FormatSegmentStruct(CString& str);
	int GetSpeed();	// Returns speed as a numerical value
};

/////////////////////////////////////////////////////////////////////////////
// Name: WaypointStruct - Holds the list of all Path Waypoints
//
class CWaypointStruct
{
// Attributes
public:

	CString	m_WaypointName;
	int 	m_WaypointID;
	int 	m_WaypointLocationFeetX;
	int 	m_WaypointLocationInchesX;
	int 	m_WaypointLocationFeetY;
	int 	m_WaypointLocationInchesY;

	CString	m_WaypointLandmarkType1;
	int 	m_WaypointLandmarkDirection1;
	int 	m_WaypointLandmarkRange1;
	int 	m_WaypointLandmarkHeight1;

	CString	m_WaypointLandmarkType2;
	int 	m_WaypointLandmarkDirection2;
	int 	m_WaypointLandmarkRange2;
	int 	m_WaypointLandmarkHeight2;

	CString	m_WaypointLandmarkType3;
	int 	m_WaypointLandmarkDirection3;
	int 	m_WaypointLandmarkRange3;
	int 	m_WaypointLandmarkHeight3;



// Operations
public:
	void FormatWaypointStruct(CString& str);
//	int  GetLandMarkType( int  LandMarkNumber );
};



/////////////////////////////////////////////////////////////////////////////
// Name: RobotTrailStruct - Holds data for each recorded Robot position point
//
class CRobotTrailStruct
{
// Attributes
public:

	POINT	m_Pos;		

// Operations
public:

};

/////////////////////////////////////////////////////////////////////////////
// Name: GPSPointStruct - Holds data for each recorded GPS point
//
class CGPSPointStruct
{
// Attributes
public:

	POINT	m_Pos;		
	BYTE	m_FixMode;

// Operations
public:
	//void FormatGPSStruct(CString& str);
};

/////////////////////////////////////////////////////////////////////////////
// Name: CMapSensorStruct - Holds sensor data for each reading from the robot
//
class CMapSensorStruct
{
// Attributes
public:

	POINT	m_RWPos;		
	int		m_Heading;
	int		US[NUM_US_SENSORS];
	int		IR[NUM_IR_SENSORS];

// Operations
public:
	//void FormatGPSStruct(CString& str);
};

typedef CTypedPtrList<CPtrList, CSegmentStruct*>	CSegmentStructList;
typedef CTypedPtrList<CPtrList, CWaypointStruct*>	CWaypointStructList;
typedef CTypedPtrList<CPtrList, CGPSPointStruct*>	CGPSPointStructList;
typedef CTypedPtrList<CPtrList, CRobotTrailStruct*>	CRobotTrailStructList;
typedef CTypedPtrList<CPtrList, CMapSensorStruct*>	CMapSensorStructList;


#endif // !defined(ROBOT_PATHSTRUCT_H)
