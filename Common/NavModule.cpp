// NavModule.cpp: CWayPointNavModule class implementation
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only


#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


///////////////////////////////////////////////////////////////////////////////
//	MODULE: WayPointNavModule

/* State Machine:
	DISABLED
	IDLE
	GET_NEXT_WAYPOINT
	TURNING_TOWARD_WAYPOINT
	DRIVING_TO_WAYPOINT
	WAYPOINT_NEAR
	WAYPOINT_REACHED
  */


CWayPointNavModule::CWayPointNavModule( CDriveControlModule *pDriveControlModule, CSensorModule *pSensorModule )
{
	// Note: pSensorModule passed in, so WayPointNavModule can do things like provide compass correction
	m_pDriveCtrl = pDriveControlModule;
	m_pSensorModule = pSensorModule;
	ASSERT( NULL != m_pDriveCtrl );
	ASSERT( NULL != m_pSensorModule );
	m_pHeadControl = new HeadControl();	// For controlling camera head servos

	InitVariables();
}

CWayPointNavModule::~CWayPointNavModule()
{
	// Release resources
	SAFE_DELETE(m_pHeadControl);
	ROBOT_LOG( TRUE,  "~CWayPointNavModule done\n" )
}

signed int CWayPointNavModule::Norm360(int RawAngle)
{
	return( RawAngle % 360 );
}

signed int CWayPointNavModule::Norm180(int RawAngle)
{
	return( Norm360(RawAngle+180) - 180 );

}

void CWayPointNavModule::InitVariables()
{
	gNavigationTimer = 0;
	m_NavState = IDLE;
	m_NavPathStarted = FALSE;
	m_PausePath = FALSE;
	m_GoalWayPoint = NO_WAYPOINT;
	m_CurrentWayPoint = NO_WAYPOINT;
	m_CurrentLocation.x = 0;
	m_CurrentLocation.y = 0;
	m_CurrentDirection = NO_DIRECTION;
	m_NextWayPoint = NO_WAYPOINT;
	m_NextLocation.x = 0;
	m_NextLocation.y = 0;
	m_NextDirection = NO_DIRECTION;
	m_NextDistance = NO_DISTANCE;
	m_PauseSpeed = SPEED_STOP;
	m_PauseTurn = TURN_CENTER;
//	m_NearestRadarObjectDirecton = NO_OBJECT_IN_RANGE;
//	m_NearestRadarObjectRange = NO_OBJECT_IN_RANGE; 
	m_ScanForConeDistance = CAMERA_SCAN_DIST_DEFAULT;
	m_TrackingCone = FALSE;
	m_Last_Cone_directon = NO_HEADING;
}

// Consolidate all Speed and Turn calls here for easier debugging
void CWayPointNavModule::SetSpeedAndTurn( int Speed, int Turn )
{
	m_pDriveCtrl->SetSpeedAndTurn( WAY_POINT_NAV_MODULE, Speed, Turn );
}

void CWayPointNavModule::SetSpeed( int Speed )
{
	m_pDriveCtrl->SetSpeed( WAY_POINT_NAV_MODULE, Speed );
}

void CWayPointNavModule::SetTurn( int Turn )
{
	m_pDriveCtrl->SetTurn( WAY_POINT_NAV_MODULE, Turn );
}

void CWayPointNavModule::CancelPath()
{
	SetSpeedAndTurn( SPEED_STOP, TURN_CENTER );
	m_pDriveCtrl->ReleaseOwner( WAY_POINT_NAV_MODULE );
	m_pSensorModule->SetCompassCorrection( 0 );

	InitVariables();
	ROBOT_DISPLAY( TRUE, "Cancel Path.  Nav PathState: IDLE" )
	m_NavState = IDLE;
}

void CWayPointNavModule::LandmarkUpdateLocation()
{
	// Update global location in real world coordinates.
	// Update via Compass and distance to Waypoint Landmark.

	// WARNING!  This ASSUMES robot is pointing at the Waypoint!
	// Given Waypoint X,Y plus distance and direction from the waypoint,
	// calculate new Current Location X,Y.
	// Note that this will override the last location calculated by SensorModule::UpdateLocation()!

	ROBOT_LOG( TRUE,  "NAVMODULE: Update Location based upon Landmark.  Old = %d,%d ",
		g_SensorStatus.CurrentLocation.x, g_SensorStatus.CurrentLocation.y)

	int WayPointX = (m_pCurrentWaypoint->m_WaypointLocationFeetX * 12) + m_pCurrentWaypoint->m_WaypointLocationInchesX;
	int WayPointY = (m_pCurrentWaypoint->m_WaypointLocationFeetY * 12) + m_pCurrentWaypoint->m_WaypointLocationInchesY;

	int Distance = g_pSensorSummary->nFrontObjectDistance;		// distance to Landmark;
	int Direction  = (g_SensorStatus.CompassHeading + 180)%360;	// From cone TO robot!


	if( Distance < 6 )	// inches
	{
		// Collision or too close to matter what direction we are pointing
		g_SensorStatus.CurrentLocation.x = WayPointX;
		g_SensorStatus.CurrentLocation.y = WayPointY ;
	}
	else
	{
		// Convert Degrees to Radians then do the math
		double DirectionRadians = (double)Direction * DEGREES_TO_RADIANS;
		double X = Distance * sin( DirectionRadians );	// Returns negative numbers as needed
		double Y = Distance * cos( DirectionRadians );
		
		if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
		if(Y>=0) Y += 0.5; else Y -= 0.5;
		g_SensorStatus.CurrentLocation.x = WayPointX + (int)X;		// Get new Map absolute X,Y
		g_SensorStatus.CurrentLocation.y = WayPointY + (int)Y;
	}

	ROBOT_LOG( TRUE,  "New = %d,%d\n", g_SensorStatus.CurrentLocation.x, g_SensorStatus.CurrentLocation.y)

}

int CWayPointNavModule::CheckForObject()
{
	// See if it's time to stop
	CString MsgString;
	//BYTE nDistHigh, nDistLow;

	// check for Bumper hit (good indication that we are there :-)!
	if( HW_BUMPER_HIT_FRONT )
	{		
		m_pDriveCtrl->Brake( WAY_POINT_NAV_MODULE );	// Stop the robot!

		// Update MAP info!  We know where we are (we think)!
		LandmarkUpdateLocation();

		// Waypoint reached.  Head to next Waypoint!
		ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
		MsgString.Format( "BUMPER HIT! Landmark Waypoint %02u (%s) reached.  Braking.",
			m_pCurrentWaypoint->m_WaypointID, m_pCurrentWaypoint->m_WaypointName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		// Stop searching for color blobs
		PostThreadMessage( g_dwCameraVidCapThreadId, 
			(WM_ROBOT_COLOR_BLOB_LOOK_AHEAD_CMD), FALSE, 0 );		// wParam = Stop

		ROBOT_DISPLAY( TRUE, "PathState: ROBOT_BRAKING" )
		m_NavState = ROBOT_BRAKING;
		SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
		return 1;
	}

	// See if we are close enough yet
	if( g_pSensorSummary->nFrontObjectDistance <= m_pCurrentWaypoint->m_WaypointLandmarkRange1) 
	{	// Close enough. Update Map with this new postion data,
		// And go to the next waypoint

		// Update MAP info!  We know where we are (we think)!
		LandmarkUpdateLocation();

		// Waypoint reached.  Head to next Waypoint!
		ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
		MsgString.Format( "Landmark Waypoint %02u (%s) reached.  Heading for next one.",
			m_pCurrentWaypoint->m_WaypointID, m_pCurrentWaypoint->m_WaypointName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		// Stop searching for color blobs
		PostThreadMessage( g_dwCameraVidCapThreadId, 
			(WM_ROBOT_COLOR_BLOB_LOOK_AHEAD_CMD), FALSE, 0 );	// wParam = Stop

		if( g_pSensorSummary->nFrontObjectDistance <= WAYPOINT_BACKUP_RANGE )
		{
			// Got too close too fast!
			m_pDriveCtrl->Brake( WAY_POINT_NAV_MODULE );

			ROBOT_DISPLAY( TRUE, "PathState: ROBOT_BRAKING")
			m_NavState = ROBOT_BRAKING;
			//SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
		}
		else
		{
			// Coast until the next Waypoint is calculated...
			//SetSpeed( SPEED_STOP );	
			ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
			m_NavState = GET_NEXT_SEGMENT;
			//SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
		}
		return 1;
	}
	return 0;
}

void CWayPointNavModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{

	CString MsgString;
	//BYTE nDistHigh, nDistLow;
	switch( uMsg )  
	{
		case WM_ROBOT_GOTO_WAYPOINT_CMD:
		{
			// Command from user to seek a new Goal!
			g_bCmdRecognized = TRUE;
			// wParam = WayPoint Number
			// lParam = not used

			// Initialize state machine to head for Goal waypoint
			m_GoalWayPoint = wParam;
			m_NavState = GET_NEXT_WAYPOINT;

			MsgString.Format( "WayPoint Goal=%d, Next=%d",	m_GoalWayPoint, m_NextWayPoint );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			return;
		}
		break;

		case WM_ROBOT_SET_SCAN_DISTANCE_CMD:
		{
			// When the robot is within this distance to the next waypoint,
			// Start scanning for cone if one has not been found already
			g_bCmdRecognized = TRUE;
			CString MsgString;
			MsgString.Format("Setting Cone Scan Distance to %u feet", lParam);
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			m_ScanForConeDistance = (int )lParam;
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
			// lParam: 1 = Wait for switch to start		0 = Don't wait
			g_bCmdRecognized = TRUE;

			if( NAV_PATH_EXECUTE_START == wParam )
			{
				// Start Executing path!
				// User said it's ok, so release user control priority
				m_pDriveCtrl->ReleaseOwner( LOCAL_USER_MODULE );
				
				// Initialize state machine to head for Goal waypoint
				m_NavPathStarted = TRUE;
				m_PausePath = FALSE;
				m_NavState = START_NEW_PATH;
				ROBOT_DISPLAY( TRUE,  "PathState: START_NEW_PATH")
				ROBOT_DISPLAY( TRUE,  "Executing Path!")
			}
			else if( PATH_EXECUTE_PAUSE == wParam )
			{
				if( m_NavPathStarted && !m_PausePath )
				{
					// If path started and not already paused
					m_PausePath = TRUE;
					m_PauseSpeed = m_pDriveCtrl->GetCurrentSpeed();
					m_PauseTurn = m_pDriveCtrl->GetCurrentTurn();
					SetSpeed( SPEED_STOP );
					ROBOT_DISPLAY( TRUE, "Path Paused!")
				}
			}
			else if( PATH_EXECUTE_RESUME == wParam )
			{
				if( m_NavPathStarted )
				{
					// Ignored if Path not started yet
					m_PausePath = FALSE;
					m_pDriveCtrl->ReleaseOwner( 0xFF );	// Override all other owners!
					SetSpeedAndTurn( m_PauseSpeed, m_PauseTurn );
					ROBOT_DISPLAY( TRUE, "Path Resumed!")
				}
			}
			else
			{
				// Stop executing path
				if( m_NavPathStarted )
				{
					// Ignored if Path not started yet
					ROBOT_DISPLAY( TRUE, "Path Execution Cancelled!")
					CancelPath();
				}
			}

			if( 0 != lParam )
			{
				m_WaitForStartSwitch = TRUE;
			}

			return;
		}
		break;

		case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			// See if we need to wait for a delay to complete
			if( gNavigationTimer != 0 ) 
			{
				// In NavModule, all timer functions are looking for camera color blobs
				if( g_ColorBlobDetected )
				{
					gNavigationTimer = 0;	// reset the timer
				}
				else
				{
					// Return if color blob not detected, and timer has not expired.
					return;
				}
			}


			if( (m_NavState >= TURN_TO_NEXT_WAYPOINT) &&
				(!m_pDriveCtrl->IsOwner(WAY_POINT_NAV_MODULE) ))
			{
				// We were following a path, but lost control of the drive wheels
				// Due to collision or other problem.  Go to a good state and continue.
				m_NavState = TURN_TO_NEXT_WAYPOINT;
			}


			///////////////////////////////////////////////////////////////////
			// Process WayPoint State Machine

			// TODO - Sometimes this applies but not always 
			//  consider avoid object, but close to goal - could go past goal!
			//  need concept of distance to goal!  If distance to goal < distance to object, ignore object?

			if( m_PausePath )
			{
				// Path has been paused.  Don't do anything until resume command issued.
				return;
			}

CheckNavState:			
			switch (m_NavState) 
			{
				case IDLE:
				{
					// Nothing to do
					return;
				}	
				break;

				case START_NEW_PATH:	// Gets First Segment
				{

					// Starting new Path. Path consists of Segments between Waypoints
					if( m_WaitForStartSwitch )
					{
						if( !(g_SensorStatus.StatusFlags & HW_STATUS_RC_BUTTON_2) )
						{
							// Start button on the RC remote not pushed yet
							ROBOT_DISPLAY( TRUE, "Execute Path: Waiting for Start Switch")
							return;
						}
						ROBOT_DISPLAY( TRUE, "Execute Path: Start Switch Pressed!  Here we Go!" )
						SpeakCannedPhrase( SPEAK_EXECUTING_PATH );

					}
					else
					{
						ROBOT_DISPLAY( TRUE, "Execute Path: Start Switch Bypassed!  Here we Go!" )
						SpeakCannedPhrase( SPEAK_EXECUTING_PATH );
					}

					// Ok, start down the path
					// We ASSUME the robot is started at Waypoint0
					m_pCurrentSegment = GetFirstSegment();
					m_pCurrentWaypoint = GetNextWaypointStruct( m_pCurrentSegment );
					if( NULL == m_pCurrentSegment )
					{
						// Error, no path defined.
						MsgString.Format( "ERROR! CWayPointNavModule: Path not selected or no Segments in Path!");
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						CancelPath();
					}
					else if( NULL == m_pCurrentWaypoint )
					{
						// Error!  Should never have a Segment with no Waypoint!
						MsgString.Format( "ERROR! CWayPointNavModule: No destination Waypoint for First Segment!");
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						CancelPath();
					}
					else
					{
						ROBOT_DISPLAY( TRUE, "PathState: TURN_TO_NEXT_WAYPOINT")
						m_NavState = TURN_TO_NEXT_WAYPOINT;
						goto CheckNavState;
					}
				}
				break;

				case GET_NEXT_SEGMENT:
				{
					// Get the next segment in our path, but first reset any variables from the last Segment

					// Stop searching for color blobs
					PostThreadMessage( g_dwCameraVidCapThreadId, 
					(WM_ROBOT_COLOR_BLOB_LOOK_AHEAD_CMD), FALSE, 0 );		// wParam = Stop
					m_TrackingCone = FALSE;
					m_Last_Cone_directon = NO_HEADING;
					m_pDriveCtrl->EnableModule( AVOID_OBJECT_MODULE );	// Turn on obstacle avoidance handler
					m_pDriveCtrl->EnableModule( COLLISION_MODULE );		// Turn on collision handler
					g_SegmentAvoidObjectRangeTenthInches = 0xFFFFF;			// Set object avoidance to no limit for the segment

					ROBOT_DISPLAY( TRUE, "\nExecute Path: Getting Next Segment" )
					m_pCurrentSegment = GetNextSegment();
					if( NULL == m_pCurrentSegment )
					{
						// Must be at the end of the Path!
						ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
						MsgString.Format( "End of Path Reached!");
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						CancelPath();
						ROBOT_DISPLAY( TRUE, "PathState: END_OF_PATH_REACHED")
						m_NavState = END_OF_PATH_REACHED;
					}
					else
					{
						m_pCurrentWaypoint = GetNextWaypointStruct( m_pCurrentSegment );
						if( NULL == m_pCurrentWaypoint )
						{
							// Error!  Should never have a Segment with no Waypoint!
							MsgString.Format( "ERROR! CWayPointNavModule: No Waypoint for Segment!");
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							CancelPath();
						}
						else
						{
							// Tell sensor module to compensate for known compass error for this segment
							m_pSensorModule->SetCompassCorrection( m_pCurrentSegment->m_CompassCorrection );

							//SetSpeedAndTurn( SPEED_STOP, TURN_CENTER );
							g_SensorStatus.DistanceToWaypoint = CalculateDistanceToWaypoint( 
								g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );

							ROBOT_DISPLAY( TRUE, "PathState: TURN_TO_NEXT_WAYPOINT")
							m_NavState = TURN_TO_NEXT_WAYPOINT;
							// Set object avoidance limit for this segment
							g_SegmentAvoidObjectRangeTenthInches = m_pCurrentSegment->m_SegmentAvoidDistance*10; //TenthInches??			

							Beep(500, 100);	// Beep to indicate New Segment starting
							Beep(1000, 100);	// Beep to indicate New Segment starting
							Beep(500, 100);	// Beep to indicate New Segment starting
							goto CheckNavState;
						}
					}
				}
				break;

				case TURN_TO_NEXT_WAYPOINT:
				{
					// Time to head toward the next waypoint
					// First we turn, then we drive straight toward the waypoint

					int  Heading = CalculateHeadingToWaypoint( 
						g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					if( RESULT_FAILURE == Heading )
					{
						// Can't calculate!  Error already printed. Use Segment info instead!
						Heading = m_pCurrentSegment->m_SegmentDirection;
					}

					int HeadingCheck = Heading - m_pCurrentSegment->m_SegmentDirection;
					if( abs(HeadingCheck) > 10 )
					{
						// More then 10 degrees difference in calculations!!!
						MsgString.Format( "WARNING! Calculated Heading (%u) != Path Heading (%u)",
							Heading, m_pCurrentSegment->m_SegmentDirection );
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					}
					
					int TurnDegrees = CalculateTurn(g_SensorStatus.CompassHeading, Heading);

					if( TurnDegrees < 0 )
					{	// Turn Left
						if( abs(TurnDegrees) > 25 )
						{
							// turn as sharp as possible
							SetTurn( TURN_LEFT_FAST );	
						}
						else if( abs(TurnDegrees) > 15 )
						{
							SetTurn( TURN_LEFT_MED_SLOW );	
						}
						else 
						{
							SetTurn( TURN_LEFT_SLOW );	
						}
					}
					else
					{	// Turn Right
						if( abs(TurnDegrees) > 25 )
						{
							// turn as sharp as possible
							SetTurn( TURN_RIGHT_FAST );	
						}
						else if( abs(TurnDegrees) > 15 )
						{
							SetTurn( TURN_RIGHT_MED_SLOW );	
						}
					}
					SetSpeed( SPEED_FWD_MED_SLOW );
					ROBOT_DISPLAY( TRUE, "PathState: TURNING_TOWARD_WAYPOINT")

					MsgString.Format( "Begin Turning toward W%u: Target Heading: %03u.  Current: %03u, Turn: %d",
						m_pCurrentWaypoint->m_WaypointID, Heading, g_SensorStatus.CompassHeading, TurnDegrees );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					
					m_NavState = TURNING_TOWARD_WAYPOINT;

				}
				break;
						
				case TURNING_TOWARD_WAYPOINT:
				{
					// Wait until we are done turning, then accelerate

					int  Heading = CalculateHeadingToWaypoint(g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					int TurnDegrees = CalculateTurn(g_SensorStatus.CompassHeading, Heading );

					MsgString.Format( "Turning toward W%u: Target Heading: %03u.  Current: %03u, Turn: %d",
						m_pCurrentWaypoint->m_WaypointID, Heading, g_SensorStatus.CompassHeading, TurnDegrees );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

					// See if turn done or we overshoot the turn
					int CurrentTurn = m_pDriveCtrl->GetCurrentTurn();
					int TurnFudge = 30;
					if( "Go Straight" == m_pCurrentSegment->m_SegmentBehavior )
					{
						// not gonna use the compass once we start, so make sure we are pointing in the right direction at the start!
						TurnFudge = 10;
					}

					if( ( abs(TurnDegrees) < TurnFudge)						||	// Close enough head for next waypoint
						((CurrentTurn < TURN_CENTER) && (TurnDegrees > 0 )) ||	// Turning Left, but Right needed
						((CurrentTurn > TURN_CENTER) && (TurnDegrees < 0 )) )	// Turning Right, but Left needed
						
					{
						// Heading close to the right direction.  Move forward at recommended speed

						int NewTurn = (int)(TurnDegrees * TURN_MULTIPLIER);
						// For DEBUG Only:
						//	MsgString.Format( "DEBUG NAV Module: NewTurn= %d", NewTurn );
						//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						SetTurn( NewTurn );	

						int NewSpeed = m_pCurrentSegment->GetSpeed();
						SetSpeed( NewSpeed );	

						// Now Drive to the Waypoint!

						if( "Cone" == m_pCurrentWaypoint->m_WaypointLandmarkType1 )
						{
							// Look for Cone with Camera straight ahead (no scanning).
							// If cone found, track and head for it.
							// To prevent false triggers, the final segment should be kept
							// short enough to avoiding seeing non-cone objects.
							ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: Passively looking for cone\n" )
							PostThreadMessage( g_dwCameraVidCapThreadId, 
								(WM_ROBOT_COLOR_BLOB_LOOK_AHEAD_CMD), TRUE, 0 );		// wParam = Start
						}
						else if( "Follow Curb" == m_pCurrentSegment->m_SegmentBehavior )
						{
							ROBOT_ASSERT(0);
							// TODO - Need to fix Camera position commands
							/***
							// Turn camera toward curb, and use US Sensor to track the distance
							if( SEGMENT_FOLLOW_LEFT == m_pCurrentSegment->m_SegmentFollowLeftRight )
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: Looking for Curb Left\n" )
								SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, CAMERA_PAN_CURB_LEFT, 0 );  
								SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, CAMERA_TILT_CURB, 0 );  
							}
							else if( SEGMENT_FOLLOW_RIGHT == m_pCurrentSegment->m_SegmentFollowLeftRight )
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: Looking for Curb Right\n" )
								SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)CAMERA_PAN_CURB_RIGHT, 0 );  
								SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)CAMERA_TILT_CURB, 0 );  
							}
							else
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: ERROR! Follow Curb with Left/Right Not set!\n" )
							}
							***/
						}

						else if( "Follow Wall" == m_pCurrentSegment->m_SegmentBehavior )
						{
							ROBOT_ASSERT(0);
							// TODO - Need to fix Camera position commands
							/***
							// Turn camera toward wall, and use US Sensor to track the distance
							SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, CAMERA_TILT_CENTER, 0 );  
							if( SEGMENT_FOLLOW_LEFT == m_pCurrentSegment->m_SegmentFollowLeftRight )
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: Looking for Wall Left\n" )
								SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, CAMERA_PAN_CURB_LEFT, 0 );  
							}
							else if( SEGMENT_FOLLOW_RIGHT == m_pCurrentSegment->m_SegmentFollowLeftRight )
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: Looking for Wall Right\n" )
								SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)CAMERA_PAN_CURB_RIGHT, 0 );  
							}
							else
							{
								ROBOT_DISPLAY( TRUE, "TURNING_TOWARD_WAYPOINT: ERROR! Follow Wall with Left/Right Not set!\n" )
							}
							***/
						}
						else if( "Go Straight" == m_pCurrentSegment->m_SegmentBehavior )
						{
							// No compass, just go straight!
							ROBOT_LOG( TRUE,  "NavModule: Going Straight!\n" )
							NewTurn = 0;
							SetTurn( NewTurn );	
						}


/*						if( "Compass+GPS" != m_pCurrentSegment->m_SegmentBehavior )
						{
							// Doing something special (not just dead reckoning )
							// Possible Options: 
							Compass+GPS
							Follow Wall
							Follow Curb
							Follow Dropoff
							Follow Hall
							Enter Doorway
								--- params  
							CString	m_SegmentBehavior;
							CString	m_SegmentSpeed;
							int 	m_SegmentDirection;
							int 	m_SegmentDistanceFeet;
							int 	m_SegmentDistanceInches;
							int 	m_SegmentFollowDistance;
							int 	m_SegmentAvoidDistance;
							int		m_SegmentFollowLeftRight;
						}
*/



						ROBOT_DISPLAY( TRUE, "PathState: DRIVING_TO_WAYPOINT")
						m_NavState = DRIVING_TO_WAYPOINT;
					}
						
				}
				break;
						
				case DRIVING_TO_WAYPOINT:
				{
					g_SensorStatus.DistanceToWaypoint = CalculateDistanceToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					int  Heading = CalculateHeadingToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					int TurnDegrees = CalculateTurn( g_SensorStatus.CompassHeading, Heading );
					int NewTurn;
					if( "Go Straight" == m_pCurrentSegment->m_SegmentBehavior )
					{
						// No compass, just go straight!
						ROBOT_LOG( TRUE,  "NavModule: Going Straight!\n" )
						NewTurn = 0;
						SetTurn( NewTurn );	
					}
					else
					{
						NewTurn = (int)(TurnDegrees * TURN_MULTIPLIER);
					}

					if( g_SensorStatus.DistanceToWaypoint < WAYPOINT_SENSOR_RANGE)
					{
						// Slow down when close to the waypoint
						if( m_pDriveCtrl->GetCurrentSpeed() > SPEED_FWD_MED_SLOW )
						{
							SetSpeed( SPEED_FWD_MED_SLOW );
						}
					}

					//////////////////////////////////////////////////////////////////////////////////////
					// Check Distance to Waypoint, see if we are there yet, 
					// or need to change behavior due to proximity to WP.

					MsgString.Format( "Driving to W%u: Distance: %03u ft. Direction: %03u deg.  Current: %03u deg, Turn Needed: %d deg",
						m_pCurrentWaypoint->m_WaypointID, (g_SensorStatus.DistanceToWaypoint/12), Heading, g_SensorStatus.CompassHeading, TurnDegrees );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

					if( "Cone" == m_pCurrentWaypoint->m_WaypointLandmarkType1)
					{
						if( g_SensorStatus.DistanceToWaypoint <= m_ScanForConeDistance *12)	//Feet
						{
							// Go to SEEK_LANDMARK state to close in on a cone
							ROBOT_DISPLAY( TRUE, "PathState: SEEKING_LANDMARK_WAYPOINT")
							m_NavState = SEEKING_LANDMARK_WAYPOINT;
							//SpeakCannedPhrase( SPEAK_LOOKING_FOR_LANDMARK );
							goto CheckNavState;
						}
					}
					else if( ("None" == m_pCurrentWaypoint->m_WaypointLandmarkType1) ||
							 (    "" == m_pCurrentWaypoint->m_WaypointLandmarkType1) )
					{
						// No Landmark.  Just stop when we get to the right place
						if( g_SensorStatus.DistanceToWaypoint < WAYPOINT_STOP_RANGE )
						{
							// Waypoint reached.  Head to next Waypoint!
							ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
							MsgString.Format( "Waypoint %02u (%s) reached.  Heading for next one.",
								m_pCurrentWaypoint->m_WaypointID, m_pCurrentWaypoint->m_WaypointName );
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							// Coast until the next Waypoint is calculated...
							//SetSpeed( SPEED_STOP );	
							ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
							m_NavState = GET_NEXT_SEGMENT;
							//SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
							goto CheckNavState;
						}
					}
					else
					{
						// This is a Landmark Waypoint.  
						// Start looking for Landmark in front of the robot
						 if( g_SensorStatus.DistanceToWaypoint < WAYPOINT_SENSOR_RANGE)
						{
							// Start looking for the landmark
							m_pDriveCtrl->SuppressModule( AVOID_OBJECT_MODULE );	// Turn off obstacle avoidance

							// Start IR Radar scanning
							// SendCommand( WM_ROBOT_ENABLE_RADAR_SCAN_CMD, 2/*Sensor*/, 1/*On/Off*/ );  
							ROBOT_DISPLAY( TRUE, "PathState: SEEKING_LANDMARK_WAYPOINT")
							m_NavState = SEEKING_LANDMARK_WAYPOINT;
							//SpeakCannedPhrase( SPEAK_LOOKING_FOR_LANDMARK );
							goto CheckNavState;
						}
					}
					if( (abs(TurnDegrees) > 90 ) &&
						( g_SensorStatus.DistanceToWaypoint < WAYPOINT_SENSOR_RANGE) )
					{
						// OOPS!  We must have just passed the Waypoint!
						// (if far away, it might just be a glitch)
						// Assume we are there, and head for the next one!
						ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
						MsgString.Format( "DRIVING_TO_WAYPOINT: Oops! Passed Waypoint.  Heading for next one. Error details: dist = %d, WP Turn = %d",
							g_SensorStatus.DistanceToWaypoint, TurnDegrees);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						// Coast until the next Waypoint is calculated...
						//SetSpeed( SPEED_STOP );	
						ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
						m_NavState = GET_NEXT_SEGMENT;
						//SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
						goto CheckNavState;
					}

					//////////////////////////////////////////////////////////////////////////////////////
					// Not close to Waypoint yet, handle special cases for turns

					if( "Cone" == m_pCurrentWaypoint->m_WaypointLandmarkType1)
					{
						if( g_ColorBlobDetected )
						{
							// Camera is tracking a cone! Steer towards it
							// Point the wheels in the same direction as the camera!
							NewTurn = (m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) * CAMERA_TO_WHEEL_SCALE;
							ROBOT_LOG( TRUE,  "DRIVE_TO_WAYPOINT is Tracking a Cone! WheelTurn = %d\n", NewTurn )
							m_TrackingCone = TRUE;
							m_Last_Cone_directon = CalculateCameraDirection( g_SensorStatus.CompassHeading, 
								(m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) );

						}
						else if( m_TrackingCone )
						{
							// we were tracking a cone, but lost it!
							ROBOT_LOG( TRUE,  "DRIVE_TO_WAYPOINT Lost Cone!\n" )
							m_TrackingCone = FALSE;
						}

					}
					else if( ("Follow Curb" == m_pCurrentSegment->m_SegmentBehavior ) ||
							 ("Follow Wall" == m_pCurrentSegment->m_SegmentBehavior ) )
					{
						// Camera is pointing toward curb.  SensorFusion should have already handled this

						if( SEGMENT_FOLLOW_LEFT == m_pCurrentSegment->m_SegmentFollowLeftRight )
						{
							// Track curb/wall on the Left.  Use closest value

							if( g_pSensorSummary->nLeftSideZone > ULTRASONIC_TENTH_INCHES_MAX )
							{
								// Curb/wall out of sensor range.  Do nothing (will track Compass)
								MsgString.Format( "Follow Left Wall/Curb - Out of Range. Distance = %d",g_pSensorSummary->nLeftSideZone ); 
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							else
							{
								if( g_pSensorSummary->nLeftSideZone > (m_pCurrentSegment->m_SegmentFollowDistance + SEGMENT_FOLLOW_DISTANCE_FUDGE) )
								{
									// too far from the curb/wall, turn toward the curb/wall
									NewTurn = -SEGMENT_FOLLOW_TURN_AMOUNT;	// Negative = Turn Left
								}
								else if( g_pSensorSummary->nLeftSideZone < (m_pCurrentSegment->m_SegmentFollowDistance - SEGMENT_FOLLOW_DISTANCE_FUDGE) )
								{
									// too close to the curb/wall, turn away from the curb/wall
									NewTurn = SEGMENT_FOLLOW_TURN_AMOUNT;	// Positive = Turn Right
								}
								else
								{
									// Tracking parallel to curb/wall.  Center wheels
									NewTurn = 0;
								}
								MsgString.Format( "Tracking Left Wall/Curb. Distance = %d, Turn = %d", g_pSensorSummary->nLeftSideZone, NewTurn );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
						}
						else if( SEGMENT_FOLLOW_RIGHT == m_pCurrentSegment->m_SegmentFollowLeftRight )
						{
							// Track curb/wall on the Right.
							if( g_pSensorSummary->nRightSideZone > ULTRASONIC_TENTH_INCHES_MAX )
							{
								// Curb/wall out of sensor range.  Do nothing (will track Compass)
								MsgString.Format( "Follow Right Wall/Curb - Out of Range. Distance = %d", g_pSensorSummary->nRightSideZone ); 
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							else 
							{
								if( g_pSensorSummary->nRightSideZone > (m_pCurrentSegment->m_SegmentFollowDistance + SEGMENT_FOLLOW_DISTANCE_FUDGE) )
								{
									// too far from the curb/wall, turn toward the curb/wall
									NewTurn = SEGMENT_FOLLOW_TURN_AMOUNT;	// Positive = Turn Right
								}
								else if( g_pSensorSummary->nRightSideZone < (m_pCurrentSegment->m_SegmentFollowDistance - SEGMENT_FOLLOW_DISTANCE_FUDGE) )
								{
									// too close to the curb/wall, turn away from the curb/wall
									NewTurn = -SEGMENT_FOLLOW_TURN_AMOUNT;	// Negative = Turn Left
								}
								else
								{
									// Tracking parallel to curb/wall.  Center wheels
									NewTurn = 0;
								}
								MsgString.Format( "Tracking Right Wall/Curb. Distance = %d, Turn = %d", g_pSensorSummary->nRightSideZone, NewTurn );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
						}
					}

					//MsgString.Format( "Drive to Waypoint Requesting Turn: %d", NewTurn );
					//ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					SetTurn( NewTurn );	

				}
				break;

				case ROBOT_BRAKING:
				{
					// We ran into or are really close to the waypoint
					// Wait for Arduino to report that we are done braking
					if( m_pDriveCtrl->RobotStopped() )
					{
						// Done breaking, now back up
						ROBOT_DISPLAY( TRUE, "Brake complete, Backing Up" )
						m_pDriveCtrl->SetMoveDistance( WAY_POINT_NAV_MODULE, SPEED_REV_MED, 
							TURN_CENTER, MOVE_DISTANCE_MED);

						ROBOT_DISPLAY( TRUE, "PathState: BACKING_UP_FROM_WAYPOINT")
						m_NavState = BACKING_UP_FROM_WAYPOINT;
					}
				}
				break;

				case BACKING_UP_FROM_WAYPOINT:
				{
					// Wait for Arduino to report that we are done backing
					// TODO-CAR: Ignore reports of collision in front, while we were backing up
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done backing up, get next segment
						ROBOT_DISPLAY( TRUE, "Backup complete, Getting next Segment" )
						ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
						m_NavState = GET_NEXT_SEGMENT;
					}
				}
				break;

				case SCANNING_FOR_CONE1:
				{
					// Stopped and looked for a cone, or tracking one after stopping.  
					// Did we hit something?
					if ( CheckForObject() )
					{
						ROBOT_LOG( TRUE,  "SCANNING_FOR_CONE1: CheckForObject Returned TRUE!\n" )
						goto CheckNavState;
					}

					// Did we find a Cone?
					if( g_ColorBlobDetected )
					{
						// Yep, Camera is tracking a cone! Steer towards it
						// Point the wheels in the same direction as the camera!
						int NewTurn = (m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) * CAMERA_TO_WHEEL_SCALE;
						MsgString.Format( "SCANNING_FOR_CONE: Found Cone! WheelTurn = %d", NewTurn );
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						SetTurn( NewTurn );	
						SetSpeed( SPEED_FWD_SLOW );
						m_Last_Cone_directon = CalculateCameraDirection( g_SensorStatus.CompassHeading, 
							(m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) );
						//ROBOT_DISPLAY( TRUE, "PathState: SEEKING_LANDMARK_WAYPOINT")
						// Stay in SCANNING_FOR_CONE1: m_NavState = SEEKING_LANDMARK_WAYPOINT;
						break;				
					}
					else
					{
						// This close, we should be looking at a cone, but none found.
						g_SensorStatus.DistanceToWaypoint = CalculateDistanceToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
						int  Heading = CalculateHeadingToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
						int TurnDegrees = CalculateTurn( g_SensorStatus.CompassHeading, Heading );
						int NewTurn = (int)(TurnDegrees * TURN_MULTIPLIER);

						if( g_SensorStatus.DistanceToWaypoint < 36 )
						{
							// less then 3 feet away, but can't see cone.
							// Abort and head for the next one
							SetSpeed( SPEED_STOP );	
							ROBOT_DISPLAY( TRUE, "SCANNING_FOR_CONE: Skipping Waypoint - Can't see Cone!")
							ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
							m_NavState = GET_NEXT_SEGMENT;
							SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
							break;
						}
						else if( abs(TurnDegrees) > 90 )
						{
							// OOPS!  We must have just passed the Waypoint!
							// Assume we are there, and head for the next one!
							ROBOT_LOG( TRUE,  "\n------------------------------------------------\n" )	/// Make this stand out in the log file!
							MsgString.Format( "SCANNING_FOR_CONE: Oops! Passed Waypoint.\n        Heading for next one. Error details: dist = %d, WP Turn = %d",
							g_SensorStatus.DistanceToWaypoint, TurnDegrees);
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							// Coast until the next Waypoint is calculated...
							SetSpeed( SPEED_STOP );	
							ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
							m_NavState = GET_NEXT_SEGMENT;
							SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
							break;
						}
						else
						{
							// Move forward a bit and try again, all the while continuing to scan
							MsgString.Format( "SCANNING_FOR_CONE: Moving %d inches to look for cone!", MOVE_DISTANCE_MED );
							ROBOT_DISPLAY( TRUE, MsgString )
							if( NO_HEADING != m_Last_Cone_directon )
							{
								TurnDegrees = CalculateTurn( g_SensorStatus.CompassHeading, Heading );
								NewTurn = (int)(TurnDegrees * TURN_MULTIPLIER);
							}
							m_pDriveCtrl->SetMoveDistance( WAY_POINT_NAV_MODULE, SPEED_FWD_SLOW, 
								NewTurn, MOVE_DISTANCE_MED);
							ROBOT_DISPLAY( TRUE, "PathState: SCANNING_FOR_CONE2")
							m_NavState = SCANNING_FOR_CONE2;
						}
					}
				}

				case SCANNING_FOR_CONE2:
				{
					// Moving forward short distance while scanning for cone then stop.
					// Wait for Arduino to report that we are done moving, even if a cone spotted.
					// We do this to prevent cycling back and forth when on the edge of detection range

					// Did we hit something?
					if ( CheckForObject() )
					{
						ROBOT_LOG( TRUE,  "SCANNING_FOR_CONE2: CheckForObject Returned TRUE!\n" )
						goto CheckNavState;
					}

					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Did we find one while moving?
						if( g_ColorBlobDetected )
						{
							// Yep, Camera is tracking a cone! Chase it!
							// Point the wheels in the same direction as the camera!
							int NewTurn = (m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) * CAMERA_TO_WHEEL_SCALE;
							SetTurn( NewTurn );	
							SetSpeed( SPEED_FWD_SLOW );	
							MsgString.Format( "SCANNING_FOR_CONE2: Found Cone! WheelTurn = %d", NewTurn );
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							m_Last_Cone_directon = CalculateCameraDirection( g_SensorStatus.CompassHeading, 
								(m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) );
							ROBOT_DISPLAY( TRUE, "PathState: SEEKING_LANDMARK_WAYPOINT")
							m_NavState = SEEKING_LANDMARK_WAYPOINT;
							break;
						}
						else
						{
							// Done moving, pause for a bit, then check again
							gNavigationTimer = ONE_SECOND_DELAY*5;	// allow n seconds for the camera to find something
							ROBOT_DISPLAY( TRUE, "PathState: SCANNING_FOR_CONE1")
							m_NavState = SCANNING_FOR_CONE1;						
							break;
						}
					}
				}
				break;

				case SEEKING_LANDMARK_WAYPOINT:
				{
					// Heading for a waypoint with a Landmark.
					g_SensorStatus.DistanceToWaypoint = CalculateDistanceToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					int  Heading = CalculateHeadingToWaypoint( g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );
					int TurnDegrees = CalculateTurn( g_SensorStatus.CompassHeading, Heading );

					///////////////////////////////////////////////////////////////////
					if ( CheckForObject() )
					{
						ROBOT_LOG( TRUE,  "SEEKING_LANDMARK_WAYPOINT: CheckForObject Returned TRUE!\n" )
						goto CheckNavState;
					}
					///////////////////////////////////////////////////////////////////
					// Ok, not time to stop, so see if we detect the object, and head for it

					g_SensorStatus.DistanceToWaypoint = CalculateDistanceToWaypoint( 
						g_SensorStatus.CurrentLocation, m_pCurrentWaypoint );

					MsgString.Format( "Seeking Landmark. Dist to WP: %03u ft. Closest Obj: %03u in.",
						(g_SensorStatus.DistanceToWaypoint/12), g_pSensorSummary->nFrontObjectDistance );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

					// Two possible methods:
					// 1.  When far away, Find object using Radar and possibly Camera (for cones)
					// 2.  When close, drive to object using static sensors

					/****
					// Using RADAR
					if( m_NearestRadarObject < NO_OBJECT_IN_RANGE )
					{
						// Object found head towards it
						// Max turn value is 30
						int RadarTurn;
						
						if( m_NearestRadarObjectDirecton <= 7 )
						{
							// Object is on the left.  Zero turn for two center values
							RadarTurn =(m_NearestRadarObjectDirecton - 7) * RADAR_TURN_MULTIPLIER;
						}
						else
						{
							// Object is on the right (dir = 8-15). Zero turn for two center values
							RadarTurn =(m_NearestRadarObjectDirecton - 8) * RADAR_TURN_MULTIPLIER;
						}

						SetTurn( RadarTurn );	

					}
					****/


					// Using Static Sensors
					// These routines can OVERRIDE the Radar calculation above!
					if( "Cone" == m_pCurrentWaypoint->m_WaypointLandmarkType1 )
					{
						if( g_ColorBlobDetected )
						{
							// Camera is tracking a cone! Steer towards it
							// Point the wheels in the same direction as the camera!
							int NewTurn = (m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) * CAMERA_TO_WHEEL_SCALE;
							MsgString.Format( "SEEKING_LANDMARK: Tracking Cone! WheelTurn = %d", NewTurn );
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							SetTurn( NewTurn );	
							SetSpeed( SPEED_FWD_SLOW );
							m_Last_Cone_directon = CalculateCameraDirection( g_SensorStatus.CompassHeading, 
								(m_pHeadControl->GetPanPosition() - CAMERA_PAN_CENTER) );
							break;
						}
						else
						{
							// This close, we should be looking at a cone.  Stop and Scan for it.
							PostThreadMessage( g_dwCameraVidCapThreadId, 
								(WM_ROBOT_COLOR_BLOB_SEARCH_CMD), TRUE, 0 );		// wParam = Start
							SetSpeed( SPEED_STOP );	
							gNavigationTimer = ONE_SECOND_DELAY*5;	// allow n seconds for the camera to find something
							ROBOT_LOG( TRUE,  "WAY_POINT_NAV_MODULE: No Cone! Stopping to Scan for Cone\n" )
							ROBOT_DISPLAY( TRUE, "PathState: SCANNING_FOR_CONE1")
							m_NavState = SCANNING_FOR_CONE1;						
							break;
						}
					}
					else if( ("Pole" == m_pCurrentWaypoint->m_WaypointLandmarkType1) ||
							 ("Tree" == m_pCurrentWaypoint->m_WaypointLandmarkType1) )
					{
						// For now, assume the object is supposed to be straight ahead, 
						// TODO-CAR-MUST: use direction: m_WaypointLandmarkDirection1

						// Look for object somewhere ahead, and turn toward the object
						// This assumes not a cluttered environment!

						// Highest priority given to long range IR, pointing slightly inward 

						if( g_pSensorSummary->nFrontObjectDistance <= m_pCurrentWaypoint->m_WaypointLandmarkRange1) 
						{
							// Waypoint reached!!!
							ROBOT_DISPLAY( TRUE, "Waypoint Reached! Object Found!" )

							SetTurn( TURN_CENTER );	
							SetSpeed( SPEED_STOP );	
							g_SensorStatus.DistanceToWaypoint = 0;
							ROBOT_DISPLAY( TRUE, "PathState: GET_NEXT_SEGMENT")
							m_NavState = GET_NEXT_SEGMENT;									
							SpeakCannedPhrase( SPEAK_WAYPOINT_REACHED );
							break;
						}					
						
						// Not there yet.  Find the object and head for it!

						if( (g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT]  <= IR_LR_DETECT_RANGE_TENTH_INCHES ) ||		
							(g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] <= IR_LR_DETECT_RANGE_TENTH_INCHES ) )		
						{
							// Forward facing Long Range IR sensors found something!
							// Turn toward the Object

							if( (g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT]  <= IR_LR_DETECT_RANGE_TENTH_INCHES ) &&		
								(g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] <= IR_LR_DETECT_RANGE_TENTH_INCHES ) )		
							{
								// Both Long Range Sensors see the object
								SetTurn( TURN_CENTER );	
								MsgString.Format( "Heading for IR Object Straight Ahead!  Range L=%d R=%d",
									g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT], g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							else if( g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT] <= IR_LR_DETECT_RANGE_TENTH_INCHES )	
							{
								// Left IR sees it, but the right does not.  Turn Left
								SetTurn( TURN_LEFT_SLOW );	
								MsgString.Format( "Heading for IR Object Ahead Left! Range L=%d R=%d",
									g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT], g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							else
							{
								// Right IR sees it, but the left does not.  Turn Right
								SetTurn( TURN_RIGHT_SLOW );	
								MsgString.Format( "Heading for IR Object Ahead Right!  Range L=%d R=%d",
									g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT], g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							SetSpeed( SPEED_FWD_SLOW );	
						}
						else
						{
							// Not dead ahead, but picked up by an US sensor

							if( abs((int)(g_SensorStatus.US[US_SENSOR_FRONT_LEFT] - g_SensorStatus.US[US_SENSOR_FRONT_RIGHT]) < US_RANGE_FUDGE_AMOUNT_TENTH_INCHES) )
							{
								// Both US Sensors see the same object go straight
								SetTurn( TURN_CENTER );	
								MsgString.Format( "Heading for US Object Straight Ahead!  Range L=%d R=%d",
									g_SensorStatus.US[US_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
							}
							else if( g_SensorStatus.US[US_SENSOR_FRONT_LEFT] < g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] )
							{
								// Object is closer on the Left, so Turn Left
								if( g_pSensorSummary->nFrontObjectDistance > IR_LR_DETECT_RANGE_TENTH_INCHES )
								{
									// Probably out of range for the Long Range IR sensor.   Do gentle turn
									SetTurn( TURN_LEFT_SLOW );	
									MsgString.Format( "Heading for US Object Far Ahead Left!  Range L=%d R=%d",
										g_SensorStatus.US[US_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );
									ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
								}
								else
								{
									// Close to object, do sharper turn
									SetTurn( TURN_LEFT_MED_SLOW );
									MsgString.Format( "Heading for US Object Near Ahead Left!  Range L=%d R=%d",
										g_SensorStatus.US[US_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );
									ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
								}
							}
							else
							{
								// Turn Right
								if( g_pSensorSummary->nFrontObjectDistance <= IR_LR_DETECT_RANGE_TENTH_INCHES )
								{
									// Probably out of range for the Long Range IR sensor.   Do gentle turn
									SetTurn( TURN_RIGHT_SLOW );	
									MsgString.Format( "Heading for US Object Far Ahead Right!  Range L=%d R=%d",
										g_SensorStatus.US[US_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );
									ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
								}
								else
								{
									// Close to object, do sharper turn
									SetTurn( TURN_RIGHT_MED_SLOW );
									MsgString.Format( "Heading for US Object Near Ahead Right!  Range L=%d R=%d",
										g_SensorStatus.US[US_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );
									ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
								}
							}
							SetSpeed( SPEED_FWD_SLOW );	

						}
					}
				}
				break;
				
				case END_OF_PATH_REACHED:
				{
					// Done with path.  Go back to Idle and wait for the next command.
					// Someday, maybe play music here? :-)
					ROBOT_DISPLAY( TRUE, "PathState: IDLE")
					m_pSensorModule->SetCompassCorrection( 0 );	// Clear any prior Compass Correction
					m_pDriveCtrl->ReleaseOwner( WAY_POINT_NAV_MODULE );
					m_NavState = IDLE;
				}
				break;

				default:
				{
					MsgString.Format( "ERROR! Illegal m_NavState = %d", m_NavState );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					CancelPath();
				}
				break;
			}	// Switch NavState

		}	// end Case Status Ready
		break;

		case WM_ROBOT_SENSOR_DATA:
		{
			g_bCmdRecognized = TRUE;
			// Process Bulk Sensor data received from Arduino
			// Find out which SCANNING sensor has been updated
			// lParam is the Scanning Sensor number
			if( US_ARRAY1 == lParam )
			{
				// Was converted to inches by Sensor module

				// Send data to the collision detector
				// TODO!   - do this here, or add new module?

			}
			else if( US_ARRAY1 == lParam )
			{
				// Was converted to inches by Sensor module

				if( SEEKING_LANDMARK_WAYPOINT == m_NavState )
				{

					// Find nearest object, so we can see if it is the waypoint
//					m_NearestRadarObjectDirecton = NO_OBJECT_IN_RANGE;
//					m_NearestRadarObjectRange = NO_OBJECT_IN_RANGE;
/****
TODO- REPLACE ALL THIS WITH STATIC SENSOR DATA!!!

					int range;
					// find nearest object
					for( int i=0; i< US1_SAMPLES; i++ )
					{
						range = g_ScaledSensorData[lParam][i];
						if( range < m_NearestRadarObjectRange )
						{
							m_NearestRadarObjectDirecton = i;
							m_NearestRadarObjectRange = range;
						}
					}
****/
				}	
			}
			// Illegal values are trapped by Sensor module.
		}
		break;



		default:
		{
			// It's OK to have messages that are not handled by this module!
			//ROBOT_LOG( TRUE,  "Note:Cmd not handled by NavModule: %02X", uMsg )
		}
	}	// switch( uMsg ) 
}

int CWayPointNavModule::CalculateCameraDirection( int CurrentHeading, int CameraOffset)
{
	if( (CurrentHeading > 360) || (CameraOffset > 360) )
	{
		ROBOT_LOG( TRUE,  "\n>>>> ERROR!!! Bad Heading or Camera Value!!!\n\n" )
		return NO_HEADING;	// Bad value
	}

	int CameraHeading = CurrentHeading + CameraOffset;	// pos = CW, neg = CCW
	if( CameraHeading > 360 )
	{
		CameraHeading = CameraHeading-360;
	}
	else if( CameraHeading < 0 )
	{
		CameraHeading = CameraHeading+360;
	}

	ROBOT_LOG( TRUE,  "CAMERA_DIR: Object at heading %d", CameraHeading)
	return CameraHeading;	// Success

}


int  CWayPointNavModule::CalculateHeadingToWaypoint( FPOINT CurrentLocation, CWaypointStruct* pTargetWaypoint )
{
	if( NULL == m_pCurrentWaypoint )
	{
		CString MsgString;
		MsgString.Format( "ERROR! CalculateHeadingToWaypoint: NULL Waypoint!");
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		return RESULT_FAILURE;	// Bad value
	}
	
	POINT RWTo;
	RWTo.x = (pTargetWaypoint->m_WaypointLocationFeetX * 12) + pTargetWaypoint->m_WaypointLocationInchesX;
	RWTo.y = (pTargetWaypoint->m_WaypointLocationFeetY * 12) + pTargetWaypoint->m_WaypointLocationInchesY;


	int  DirectionToWaypoint = CalculateAngle( CurrentLocation, RWTo );

	return DirectionToWaypoint;
}

int CWayPointNavModule::CalculateDistanceToWaypoint( FPOINT CurrentLocation, CWaypointStruct* pTargetWaypoint )
{
	if( NULL == m_pCurrentWaypoint )
	{
		CString MsgString;
		MsgString.Format( "ERROR! CalculateDistanceToWaypoint: NULL Waypoint!");
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		return 0;	// Bail from any calculations!
	}
	POINT RWTo;
	RWTo.x = (pTargetWaypoint->m_WaypointLocationFeetX * 12) + pTargetWaypoint->m_WaypointLocationInchesX;
	RWTo.y = (pTargetWaypoint->m_WaypointLocationFeetY * 12) + pTargetWaypoint->m_WaypointLocationInchesY;
	int DistanceToWaypoint = CalculateDistance( CurrentLocation, RWTo );
	return DistanceToWaypoint;
}


// Macro used by LineIntersectTest function:
#define SAME_SIGNS( a, b )	\
		(((long) ((unsigned long) a ^ (unsigned long) b)) >= 0 )

int  CWayPointNavModule::LineIntersectTest( 
	long x1, long y1, long x2, long y2,	// First Line
	long x3, long y3, long x4, long y4, // Second Line
	long *Ix, long *Iy )				// Return the Intersection Point (in case calling function cares)
{

	// Calculates if two lines intersect.  From "xlines.c" AUTHOR: Mukesh Prasad, GPU Gems 2
	// DONT_INTERSECT    0
	// DO_INTERSECT      1
	// COLLINEAR         2

	long a1, a2, b1, b2, c1, c2; /* Coefficients of line eqns. */
	long r1, r2, r3, r4;         /* 'Sign' values */
	long denom, offset, num;     /* Intermediate values */

	// Compute a1, b1, c1, where line joining points 1 and 2 is "a1 x  +  b1 y  +  c1  =  0".

	a1 = y2 - y1;
	b1 = x1 - x2;
	c1 = x2 * y1 - x1 * y2;

	// Compute r3 and r4.
	r3 = a1 * x3 + b1 * y3 + c1;
	r4 = a1 * x4 + b1 * y4 + c1;

    // Check signs of r3 and r4.  If both point 3 and point 4 lie on
    // same side of line 1, the line segments do not intersect.
	if (	r3 != 0 &&
			r4 != 0 &&
			SAME_SIGNS( r3, r4 ))
		return ( DONT_INTERSECT );

    // Compute a2, b2, c2
	a2 = y4 - y3;
	b2 = x3 - x4;
	c2 = x4 * y3 - x3 * y4;

	// Compute r1 and r2
	r1 = a2 * x1 + b2 * y1 + c2;
	r2 = a2 * x2 + b2 * y2 + c2;

	// Check signs of r1 and r2.  If both point 1 and point 2 lie
	// on same side of second line segment, the line segments do not intersect.
	if (	r1 != 0 &&
			r2 != 0 &&
			SAME_SIGNS( r1, r2 ))
		return ( DONT_INTERSECT );

	// Line segments intersect: compute intersection point. 
	denom = a1 * b2 - a2 * b1;
	if ( denom == 0 )
		return ( COLLINEAR );
	offset = denom < 0 ? - denom / 2 : denom / 2;

	// The denom/2 is to get rounding instead of truncating.  It
	// is added or subtracted to the numerator, depending upon the sign of the numerator.
	num = b1 * c2 - b2 * c1;
	*Ix = ( num < 0 ? num - offset : num + offset ) / denom;

	num = a2 * c1 - a1 * c2;
	*Iy = ( num < 0 ? num - offset : num + offset ) / denom;

	return ( DO_INTERSECT );

}


CWaypointStruct* CWayPointNavModule::GetNextWaypointStruct(CSegmentStruct* pSegmentStruct)
{
	// Get Waypoint info for the current Segment
	CWaypointStruct* pWaypointStruct = NULL;
	CString strErrorMsg;
	POINT RWTo;
	RWTo.x = 0; RWTo.y=0;

	if( pSegmentStruct != NULL )
	{
		if( g_pWaypointList != NULL )
		{
			POSITION pos = g_pWaypointList->GetHeadPosition();
			while (pos != NULL)
			{
				pWaypointStruct = g_pWaypointList->GetNext(pos);
				if( pWaypointStruct->m_WaypointID == pSegmentStruct->m_SegmentToWaypointID )
				{
					RWTo.x = (pWaypointStruct->m_WaypointLocationFeetX * 12) + pWaypointStruct->m_WaypointLocationInchesX;
					RWTo.y = (pWaypointStruct->m_WaypointLocationFeetY * 12) + pWaypointStruct->m_WaypointLocationInchesY;
					break;
				}
			}
		}
		if( (-1 == RWTo.x) || (-1 == RWTo.y) )
		{
			strErrorMsg.Format("WaypointNav: To Waypoint ID is not valid for segment %s\n", pSegmentStruct->m_SegmentName);
			ROBOT_LOG( TRUE, strErrorMsg)
		}
	}
	return pWaypointStruct;
}

CSegmentStruct* CWayPointNavModule::GetFirstSegment()
{
	// Get the first Segment in the list

	CSegmentStruct* pSegmentStruct = NULL;

	if( g_pSegmentList != NULL )
	{
		m_NextSegmentPos = g_pSegmentList->GetHeadPosition();
		if( m_NextSegmentPos != NULL )
		{	
			// Get the structure and increment the pointer for the next time
			pSegmentStruct = g_pSegmentList->GetNext(m_NextSegmentPos);
		}
	}
	return pSegmentStruct;
}

////////////////////////////////////////////////////////////////////////////////////
CSegmentStruct* CWayPointNavModule::GetNextSegment()
{
	// Get the Next Semgment in the list

	CSegmentStruct* pSegmentStruct = NULL;

	if( g_pSegmentList != NULL )
	{
		if( m_NextSegmentPos != NULL )
		{
			// Get the structure and increment the pointer for the next time
			pSegmentStruct = g_pSegmentList->GetNext(m_NextSegmentPos);
		}
	}
	return pSegmentStruct;
}

#endif // ROBOT_SERVER	// This module used for Robot Server only


