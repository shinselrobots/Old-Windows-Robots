// PathStruct.cpp
// Data structures used for storing Paths; Segments, and Waypoints
/////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "RobotType.h"
#include "HardwareCmds.h"
#include "HWInterfaceParams.h"

#include "Robot.h"
#include "PathStruct.h"
#include "Globals.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CSegmentStruct

// TODO - rename this to FormatForListBox ?
void CSegmentStruct::FormatSegmentStruct(CString& str)
{
	// Format for displaying in List Box

	str.Format(_T("%s\t%02u-%02u    %03u  %03u-%02u  %-18s\t%03u"),
		(LPCTSTR)m_SegmentName, 
		m_SegmentFromWaypointID,
		m_SegmentToWaypointID,
		m_SegmentDirection,
		m_SegmentDistanceFeet,
		m_SegmentDistanceInches,
		(LPCTSTR)m_SegmentBehavior);
}


int CSegmentStruct::GetSpeed()
{
	// Return Speed as a numerical value

	if( "Slow"			== m_SegmentSpeed )	return SPEED_FWD_SLOW;
	if( "Med Slow"		== m_SegmentSpeed )	return SPEED_FWD_MED_SLOW;
	if( "Med"			== m_SegmentSpeed )	return SPEED_FWD_MED;
	if( "Med Fast"		== m_SegmentSpeed )	return SPEED_FWD_MED_FAST;
	if( "Fast"			== m_SegmentSpeed )	return SPEED_FWD_FAST;
	if( "Full Speed"	== m_SegmentSpeed )	return SPEED_FULL_FWD;

	if( "Rev Slow"		== m_SegmentSpeed )	return SPEED_REV_SLOW;
	if( "Rev Med Slow"	== m_SegmentSpeed )	return SPEED_REV_MED_SLOW;
	if( "Rev Med"		== m_SegmentSpeed )	return SPEED_REV_MED;

	// default - must be an error!
	return SPEED_STOP;
}


/////////////////////////////////////////////////////////////////////////////
// CWaypointStruct

void CWaypointStruct::FormatWaypointStruct(CString& str)
{
	// Format for displaying in List Box

	str.Format(_T("%02u    %04u  %04u\t%s"),
		m_WaypointID,
		m_WaypointLocationFeetX,
		m_WaypointLocationFeetY,
		m_WaypointName);

}

