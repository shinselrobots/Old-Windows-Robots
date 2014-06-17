// DriveAvoidanceModule.cpp: Object avoidance behaviors for drive control

#include "stdafx.h"
#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "thread.h"
#include "HardwareConfig.h"
#include "module.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//#define AVOIDANCE_MOVES_DISABLED_FOR_TESTING

// Global to this module (defined as extern in Module.h)
//int					gAutoNavigateMode = MODE_MANUAL_OVERRIDE;

///__itt_string_handle* pshMotorCommand = __itt_string_handle_create("Motor Command, Speed, Turn = ");

/*
__itt_string_handle* pshCase1 = __itt_string_handle_create("1 Case");
__itt_string_handle* pshCase2 = __itt_string_handle_create("2 Case");
__itt_string_handle* pshCase3 = __itt_string_handle_create("3 Case");
__itt_string_handle* pshCase4 = __itt_string_handle_create("4 Case");
__itt_string_handle* pshCase5 = __itt_string_handle_create("5 Case");
__itt_string_handle* pshCase6 = __itt_string_handle_create("6 Case");
__itt_string_handle* pshCase7 = __itt_string_handle_create("7 Case");
__itt_string_handle* pshCase8 = __itt_string_handle_create("8 Case");
__itt_string_handle* pshCase9 = __itt_string_handle_create("9 Case");

*/




///////////////////////////////////////////////////////////////////////////////
//	MODULE: OBJECT AVOIDANCE MODULE

#define AVOIDANCE_FORWARD_SPEED							SPEED_FWD_MED_SLOW
#define AVOIDANCE_FORWARD_DISTANCE						 20 // TenthInches
#define AVOIDANCE_TURN_SPEED							SPEED_FWD_MED_SLOW
#define DOOR_SPOTTING_DISTANCE_TENTH_INCHES				480
#define DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES		300
#define DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES		230	// 23 inches.  Robot is 22 inches wide, including arms!
#define DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES2		( ROBOT_BODY_WIDTH_TENTH_INCHES + (FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES * 2) )

CAvoidObjectModule::CAvoidObjectModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_AvoidanceState = DISABLED;
	m_WarningDirection = 0;
	m_PriorSpeed = SPEED_STOP;
	m_SideAvoidDistanceTenthInches = SIDE_THREAT_NORM_THRESHOLD;
	gAvoidanceTimer = 0;
	m_LastObjectDirection = 0;

#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
	m_ArmsInSafePosition = FALSE;
	m_pKinectServoControl = NULL;
	m_KinectInObjectSpottingPosition = FALSE;
	m_WaitingToMoveKinect = FALSE;
	m_pKinectServoControl = new KinectServoControl;

	#if( ROBOT_HAS_RIGHT_ARM )
		m_pArmControlRight = new ArmControl( RIGHT_ARM );	// For arm position information
	#endif

	#if( ROBOT_HAS_LEFT_ARM )
		m_pArmControlLeft = new ArmControl( LEFT_ARM );	// For arm position information
	#endif
#endif

}

CAvoidObjectModule::~CAvoidObjectModule()
{
	ROBOT_LOG( TRUE, "~CAvoidObjectModule()\n" )
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
	SAFE_DELETE( m_pKinectServoControl );

	#if( ROBOT_HAS_RIGHT_ARM )
		SAFE_DELETE(m_pArmControlRight);
	#endif
	#if( ROBOT_HAS_LEFT_ARM )
		SAFE_DELETE(m_pArmControlLeft);
	#endif
#endif
}


void CAvoidObjectModule::ProcessMessage( UINT uMsg, WPARAM wParam, LPARAM lParam ) // AVOID_OBJECT_MODULE
{

	int  TurnAmount = 0;	// absolute amount to turn
	int	Turn = 0;			// Amount * direction
	int TurnLeft = 0;
	int TurnRight = 0;
	int Speed = 0;


	switch( uMsg )
	{
		case WM_ROBOT_ENABLE_AVOIDANCE_MODULE:
		{
			// Command from user to enable/disable the Avoidance Module
			g_bCmdRecognized = TRUE;
			// wParam = Enabled/Disabled
			// lParam = not used

			if( wParam )
			{
				// module enabled
				m_AvoidanceState = IDLE;
				ROBOT_DISPLAY( TRUE, "Avoidance Module Enabled" )
				#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
					m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_OBJECT_AVOIDANCE_POSITION );
					m_KinectInObjectSpottingPosition = TRUE;
				#endif
			}
			else
			{
				m_AvoidanceState = DISABLED;
				m_pDriveCtrl->ReleaseOwner( AVOID_OBJECT_MODULE );
				#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
					if( m_KinectInObjectSpottingPosition )
					{
						m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_HUMAN_DETECT_START_POSITION );	// By default, leave Kinect in a position where it can find humans
						m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_COLLISION_AVOIDANCE);
					}
				#endif
				ROBOT_DISPLAY( TRUE, "Avoidance Module Disabled" )
			}
			return;
		}
		break;

		//case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		case WM_ROBOT_KOBUKI_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;
			//BYTE nDistHigh, nDistLow;
			CString MsgString;


			if( DISABLED == m_AvoidanceState )
			{
				// This module is disabled
				return;
			}

/*			if( !m_pDriveCtrl->IsOwner(AVOID_OBJECT_MODULE) )
			{
				// This module lost control of the drive wheels.  Go to a good state.
				m_AvoidanceState = IDLE;
			}
*/

			// If needed, make sure Kinect is in postion to provide obstacle data
			HandleKinectPosition();

			if( 0 == m_pDriveCtrl->GetCurrentSpeed()  )
			//if( !m_pDriveCtrl->MovementCommandPending() )
			{
				// Not moving. Don't do Avoidance behavior if we are not moving

				if( m_pDriveCtrl->IsOwner(AVOID_OBJECT_MODULE) )
				{
					// this module has control from last time.  Release control now
					//ROBOT_DISPLAY( TRUE, "Avoidance State: Done Avoiding Object" )
					//m_pDriveCtrl->SetSpeedAndTurn( AVOID_OBJECT_MODULE, SPEED_STOP, TURN_CENTER );
					m_AvoidanceState = IDLE;
					m_pDriveCtrl->ReleaseOwner( AVOID_OBJECT_MODULE );
				}
				return;
			}

			//int DbgSpeed = m_pDriveCtrl->GetCurrentSpeed();
			//int DbgTurn = m_pDriveCtrl->GetCurrentTurn();
			//ROBOT_LOG( TRUE, "speed = %d, Turn = %d\n", DbgSpeed, DbgTurn )

			// Side threshold is controlled by a persistance to avoid thrashing
			// also used for narrow corridors and doorways
			if( gAvoidanceTimer != 0 ) 
			{
				m_SideAvoidDistanceTenthInches = SIDE_THREAT_MIN_THRESHOLD;
			}
			else
			{
				m_SideAvoidDistanceTenthInches = SIDE_THREAT_NORM_THRESHOLD;
				m_LastObjectDirection = 0;	// reset after timer expires  TODO REMOVE THIS?
			}


			///////////////////////////////////////////////////////////////////
			// See if an object has been found that is closer then BOTH
			// the Global limit set in the GUI, AND the limit set for the current path segment

			// Avoid objects.  Turn while looking for a clear path

			// NOT USED: int  AvoidObjectDistanceTenthInches = __min( (g_GlobalMaxAvoidObjectDetectionFeet * 120), g_SegmentAvoidObjectRangeTenthInches );

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

				if(	(g_pNavSensorSummary->nFrontObjectDistance <= AvoidObjectDistanceTenthInches)		||
					(g_pNavSensorSummary->nLeftSideZone < m_SideAvoidDistanceTenthInches)			||
					(g_pNavSensorSummary->nRightSideZone < m_SideAvoidDistanceTenthInches)		)
				{
					// Object in front or on the side within threshold area to avoid
					// Ignore current state, we always "reset" when an object to avoid is seen

					ROBOT_ASSERT(0); // TODO!  NEED TO IMPLEMENT THIS FOR CARBOT!
					/*****
					// TODO-CAR-MUST - 
					// This is old Carbot code.  currently using code below for both Carbot and Loki
					// need to determine if Carbot needs different code for it's 6 US sensors!

					// First, see if there is a clear path anywhere ahead.  
					// Order of these is important!  Must check forward first!
					if( g_pNavSensorSummary->nLeftFrontZone > AvoidObjectDistanceTenthInches )
					{
						return -10; // Clear path ahead Left!
					}
					if( g_pNavSensorSummary->nRightFrontZone > AvoidObjectDistanceTenthInches )
					{
						return 10; // Clear path ahead Right!
					}
					if( g_pNavSensorSummary->nLeftArmZone > AvoidObjectDistanceTenthInches )
					{
						return -20; // Clear path ahead Hard Left!
					}
					if( g_pNavSensorSummary->nRightArmZone > AvoidObjectDistanceTenthInches )
					{
						return 20; // Clear path ahead Hard Right!
					}

					// Ok, not so easy.  Try calculating potential threats
					int ThreatsLeft = 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nLeftFrontZone)*3 ) + 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nLeftArmZone)*2 ) + 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nLeftSideZone)    );

					int ThreatsRight = 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nRightFrontZone)*3 ) + 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nRightArmZone)*2 ) + 
						( (NO_OBJECT_IN_RANGE - g_pNavSensorSummary->nRightSideZone)    );

					return( ThreatsRight - ThreatsLeft );
					****/
				}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif ( (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_KOBUKI_WITH_ARDUINO) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP_KOBUKI) )

				if( m_pDriveCtrl->GetCurrentSpeed() < 0 )
				{
					// Backing up!
					if( g_pNavSensorSummary->nRearObjectDistance <= REAR_THREAT_THRESHOLD )
					{
						// Backing up, and Object detected in the way!
						ROBOT_LOG( TRUE, "Rear object detected while backing up" )
						// Just stop when this happens, and cancel any avoidance behavior
						m_pDriveCtrl->SetSpeedAndTurn( AVOID_OBJECT_MODULE, SPEED_STOP, TURN_CENTER );
						m_AvoidanceState = IDLE;
					}
					break;
				}

				// Going Forward
				//ROBOT_LOG(TRUE, "DEBUG: nFrontObjectDistance = %d", g_pNavSensorSummary->nFrontObjectDistance )

				// See if we should move arms out of harms way
				CheckArmSafePosition();


				// Start Avoidance Behavior
				Turn = 0;
				BOOL ChangeSpeedOrDirection = TRUE;
				int  RecommendedDirectionVector = 0;

				if( DetectAndHandleCliff(Turn, Speed) )
				{
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Avoiding Cliff\n" )

				}
				else if( DetectAndHandleDoorway(Turn, Speed) )
				{
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Heading for Doorway\n" )

				}
				else if( (g_pNavSensorSummary->nRightArmZone <= ARM_ZONE_THREAT_MIN_THRESHOLD) ||
						 (g_pNavSensorSummary->nLeftArmZone <= ARM_ZONE_THREAT_MIN_THRESHOLD) )
				{
					// Avoid very close objects in key zones
					// close object about to hit an arm
					if( g_pNavSensorSummary->nRightArmZone < g_pNavSensorSummary->nLeftArmZone )
					{
						// closest object is on the right
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Left to avoid object on Front Right Arm Zone\n" )
						Turn = TURN_LEFT_MED;	// hard turn
					}
					else
					{
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Right to avoid object on Front Left Arm Zone\n" )
						Turn = TURN_RIGHT_MED;	// hard turn
					}
					Speed = SPEED_FWD_SLOW;	// force a pivot turn
 
				}
				else if( g_pNavSensorSummary->nFrontObjectDistance <= FRONT_ZONE_THREAT_NORM_THRESHOLD )
				{
					// There is an object somewhere in front of us. 
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: FrontObjectDistance = %d, direction = %d\n",
						g_pNavSensorSummary->nFrontObjectDistance, g_pNavSensorSummary->nFrontObjectDirection )

					if( g_pNavSensorSummary->nFrontObjectDirection > 0 )
					{
						// Object to the Right, Turn Left
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Left to avoid object on Front Right\n" )
						Turn = TURN_LEFT_MED_SLOW;
					}
					else if( g_pNavSensorSummary->nFrontObjectDirection < 0 )
					{
						// Object to the Left, Turn Right
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Right to avoid object on Front Left\n" )
						Turn = TURN_RIGHT_MED_SLOW;
					}
					else
					{
						// Object Dead Ahead. Check side sensors for a hint
						if( g_pNavSensorSummary->nSideObjectDirection > 0 )
						{
							// Object to the Right, Turn Left
							ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead, turning Left (other object on right side)\n" )
							Turn = TURN_LEFT_MED_SLOW;
						}
						if( g_pNavSensorSummary->nSideObjectDirection < 0 )
						{
							// Object to the Left, Turn Right
							ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead, turning Right (other object on left side)\n" )
							Turn = TURN_RIGHT_MED_SLOW;
						}
						else
						{
							// No idea which way to turn, so do a random turn
							int RandomNumber = ((3 * rand()) / RAND_MAX);
							if( RandomNumber >= 2 )
							{
								Turn = TURN_LEFT_MED_SLOW;
								ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead!  Random Turn Left!\n" )
							}
							else
							{
								Turn = TURN_RIGHT_MED_SLOW;
								ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead!  Random Turn Right!\n" )
							}

								Speed = SPEED_FWD_MED_SLOW;
								//Turn = TURN_RIGHT_MED_SLOW; // when in doubt, turn right
							}

					}	// End of object dead ahead

					// If object is very close, stop and force a turn on axis (But make sure we are turning, so we don't just stall)
					if( (g_pNavSensorSummary->nFrontObjectDistance <= FRONT_ZONE_THREAT_MIN_THRESHOLD) && (0 != Turn) )
					{
						Speed = SPEED_STOP;
					}


				}	// End object somewhere in front of us. 
				else if( g_pNavSensorSummary->nSideObjectDistance < m_SideAvoidDistanceTenthInches )
				{
					// No object ahead, but object on the side within threshold area to avoid
					if( g_pNavSensorSummary->nSideObjectDirection > 0 )
					{
						// Object to the Right, Turn Left
						Turn = TURN_LEFT_SLOW; // Leave speed, just a gentle turn
					}
					else
					{
						// Object is closer to the Left
						Turn = TURN_RIGHT_SLOW; 
					}
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turn %d to avoid object on side\n", Turn )
				}
				else
				{
					// Nothing to avoid
					ChangeSpeedOrDirection = FALSE; 
				}


				////////////////////////////////////////////////////////////////////////
				// Now, do the speed and turn changes!
				if( ChangeSpeedOrDirection )
				{
					// Object found.  Avoidance is needed
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turn = %d, Speed = %d\n", Turn, Speed )
					#ifndef AVOIDANCE_MOVES_DISABLED_FOR_TESTING
						m_pDriveCtrl->SetSpeedAndTurn( AVOID_OBJECT_MODULE, Speed, Turn );
					#else
						m_pDriveCtrl->SetSpeedAndTurn( AVOID_OBJECT_MODULE, SPEED_STOP, TURN_CENTER );
					ROBOT_DISPLAY( TRUE, "WARNING: AVOIDANCE_MOVES_DISABLED_FOR_TESTING" )					
						RobotSleep(3000, pDomainControlThread);
						RobotSleep(100, pDomainControlThread); // put break here
					#endif
					if( 0 != Turn )
					{
						m_AvoidanceState = AVOIDING;
					}
					ROBOT_DISPLAY( TRUE, "Avoidance State: AVOIDING" )

				} 
				else if( m_pDriveCtrl->IsOwner(AVOID_OBJECT_MODULE) )
				{
					// No objects to avoid, but this module has control from last time.  Release control now
					ROBOT_DISPLAY( TRUE, "Avoidance State: Done Avoiding Object" )
					m_AvoidanceState = IDLE;
					m_pDriveCtrl->ReleaseOwner( AVOID_OBJECT_MODULE );
				}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			#else
				#error BAD SENSOR_CONFIG_TYPE!  
			#endif
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
		}	// end case WM_ROBOT_SENSOR_STATUS_READY
	}
}

void CAvoidObjectModule::CheckArmSafePosition()
{
	// See if we should move arms out of harms way
	if(	(g_pNavSensorSummary->nFrontObjectDistance <= PROTECT_ARMS_FRONT_THREAT_THRESHOLD) ||
		(g_pNavSensorSummary->nSideObjectDistance < PROTECT_ARMS_SIDE_THREAT_THRESHOLD) )
	{
		// Objects close by, raise the arms to a safe position
		if( !m_ArmsInSafePosition )
		{
			#if ( ROBOT_SERVER == 1 )
				#if( ROBOT_HAS_LEFT_ARM )
					m_pArmControlLeft->MoveArmToSafePosition();
				#endif
				#if( ROBOT_HAS_RIGHT_ARM )
					m_pArmControlRight->MoveArmToSafePosition();
				#endif
			#endif
			m_ArmsInSafePosition = TRUE;
		}
	}
	else if( m_ArmsInSafePosition )
	{
		// No need to keep them in safe postion anymore

		#if ( ROBOT_SERVER == 1 )
			#if( ROBOT_HAS_LEFT_ARM )
				m_pArmControlLeft->MoveArmHome();
			#endif
			#if( ROBOT_HAS_RIGHT_ARM )
				m_pArmControlRight->MoveArmHome();
			#endif
		#endif
		m_ArmsInSafePosition = FALSE;
	}

}

BOOL CAvoidObjectModule::DetectAndHandleCliff( int &Turn, int &Speed )
{

	if( g_pNavSensorSummary->CliffDetected() )
	{
		// Cliff detected!
		Speed = SPEED_STOP;	// force a pivot turn

		if( g_pNavSensorSummary->CliffFront() )
		{
			// Cliff on both sides  Oh no!  what to do?
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: Cliff Ahead Front!!!\n" )
			//SpeakText( "Error! Cliff on both sides") );	
			Turn = TURN_CENTER;	// NO turn, just stop!
		}
		else if( g_pNavSensorSummary->bCliffLeft )
		{		
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: Cliff Left Side!\n" )
			// SpeakText( "Cliff Left" );	
			Turn = TURN_RIGHT_MED_SLOW;
		}
		else if( g_pNavSensorSummary->bCliffRight )
		{		
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: Cliff Right Side!\n" )
			//g_ClientTextToSend = "Cliff Right" );	
			Turn = TURN_LEFT_MED_SLOW;
		}
		else if( g_pNavSensorSummary->bWheelDropLeft || g_pNavSensorSummary->bWheelDropRight )
		{
			Turn = TURN_CENTER;	// Wheel dropped!  Just stop so we don't tip over!
		}
		else
		{
			// Logic Error
			ROBOT_ASSERT(0);
		}
		return TRUE; // Cliff detected
	}
	else
	{
		return FALSE; // no Cliff
	}
}



/////////////////////////////////////////////////////////////////
BOOL CAvoidObjectModule::DetectAndHandleDoorway( int &Turn, int &Speed )
{
	// This function only gets called if there is an object in the RightFrontSideZone and LeftFrontSideZone
	// Since both are obstructed, look for a gap between them big enough for the robot to fit.
	// Returns TRUE if a doorway was found.  Note, also works for corridors.

	if( 0 == g_pNavSensorSummary->nDoorWaysFound  )
	{
		return FALSE; // no doorway detected
	}

	// at least one doorway found
	int BestCenterX = MAX_INT;
	int BestCenterIndex = 0;
	for( int i = 0; i < g_pNavSensorSummary->nDoorWaysFound; i++ )
	{

		// find doorway closest to center front of robot
		int centerX = g_pNavSensorSummary->DoorWaysFound[i].CenterX;

		if( abs(centerX) < BestCenterX )
		{
			BestCenterX = centerX;
			BestCenterIndex = i;
		}
	}

	// Found Doorway closest to center front of robot
	int DoorwayWidth = g_pNavSensorSummary->DoorWaysFound[BestCenterIndex].Width;

		Turn = (BestCenterX / 5);		// /10 tenth inches  * 2 if more turn needed 
		ROBOT_LOG( TRUE, "DOORWAY: Center = %d,  Turn = %d",  BestCenterX, Turn )
/*		if( DoorwayCenter > 50 )		// tenth inches off course
			Turn = TURN_RIGHT_MED_SLOW;
		else if( DoorwayCenter > 10 )
			Turn = TURN_RIGHT_SLOW;	
		else if( DoorwayCenter < -10 )
			Turn = TURN_LEFT_SLOW;	
		else if( DoorwayCenter < -50 )
			Turn = TURN_LEFT_MED_SLOW;	
		else
			Turn = TURN_CENTER;
*/
		ROBOT_LOG( TRUE, "**********************************\n" )
		ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: DOORWAY Found - Center = %d, Width = %d Inches\n", BestCenterX/10, DoorwayWidth/10 )
		ROBOT_LOG( TRUE, "**********************************\n" )

		return TRUE;
}




#ifdef REMOVE_ALL_THIS

	int RightEdgeX = 0;
	int LeftEdgeX = 0;
	int X, Y;
	int PreviousX = 0;
	int PreviousY = 0;

//	const int DoorwayMinClearArea = DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES;
//	const int DoorwayMinClearArea2 = DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES2;


	if( (g_pNavSensorSummary->nLeftFrontSideZone  > DOOR_SPOTTING_DISTANCE_TENTH_INCHES) ||
		(g_pNavSensorSummary->nRightFrontSideZone > DOOR_SPOTTING_DISTANCE_TENTH_INCHES) ) 
	{
		return FALSE; // no door frame found in range
	}
	// THIS IS wrong
	int TargetClearDistance = (__max(g_pNavSensorSummary->nRightFrontSideZone, g_pNavSensorSummary->nLeftFrontSideZone) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) ; // tenth inches

	int  NumberOfLaserSamples = g_pLaserScannerData->NumberOfSamples;
	if( NumberOfLaserSamples > 0 )
	{
		//////////////////// LASER //////////////////////
		// Laser Scanner installed and working, so use it!

		for( int i = 0; i < (int)NumberOfLaserSamples; i++ ) // Scanner goes Right to Left
		{
			X = g_pLaserScannerData->ScanPoints[i].X ; // tenth inches
			Y = g_pLaserScannerData->ScanPoints[i].Y ;

			if( (X <= (HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES))  &&
				(X >= -(HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)) )
			{
				// point is inside RightFrontSideZone to LeftFrontSideZone
				if( 0 == RightEdgeX )
				{ 
					// looking for right edge
					if( Y >= TargetClearDistance )
					{	// Found inside of door edge!
						RightEdgeX = PreviousX;	// Use the last valid data
					}
				}
				else
				{
					// Right edge found, looking for left edge
					if( Y < TargetClearDistance )
					{
						if( (RightEdgeX - X) >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES ) // TODO Fix for Teleop!
						{
							// Found it!
							LeftEdgeX = X;
							break;
						}
						else
						{
							// too narrow, restart search
							RightEdgeX = 0;
						}
					}
				}
				PreviousX = X;

			}	// inside zone if interest
		}	// for loop

	}	// Number of Laser Sample > 0
	else if( NULL != g_KinectPointCloud )
	{	
		//////////////////// KINECT //////////////////////

		int NumberOfKinectSamples = g_KinectPointCloud->FrameSizeX; // Kinect goes  **Left to Right**
		if( NumberOfKinectSamples > 0 )
		{
	
			for( int i = 0; i < NumberOfKinectSamples; i++ ) 
			{
				X = g_KinectPointCloud->MapPoints2D[i].X; // tenth inches
				Y = g_KinectPointCloud->MapPoints2D[i].Y ;

				if( (X <= (HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES))  &&
					(X >= -(HALF_ROBOT_BODY_WITH_ARMS_WIDTH_TENTH_INCHES+FRONT_SIDE_ZONE_WIDTH_TENTH_INCHES)) )
				{
					// point is inside RightFrontSideZone to LeftFrontSideZone
					if( 0 == LeftEdgeX )
					{ 
						// looking for left edge
						if( Y >= TargetClearDistance )
						{	// Found inside of door edge!
							LeftEdgeX = PreviousX;	// Use the last valid data
						}
					}
					else
					{
						// Left edge found, looking for right edge
						if( Y < TargetClearDistance )
						{
							if( (LeftEdgeX - X) >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES )
							{
								// Found it!
								RightEdgeX = X;
								break;
							}
							else
							{
								// too narrow, restart search
								LeftEdgeX = 0;
							}
						}
					}
					PreviousX = X;

				}	// inside zone if interest
			}	// for loop
		}	// Number of Kinect Sample > 0

	} // Use Laser Scanner or Kinect

	if( 0 != LeftEdgeX )
	{
		// Found Doorway
		int DoorwayWidth = RightEdgeX - LeftEdgeX;
		int DoorwayCenter = RightEdgeX - (DoorwayWidth/2);
		Turn = (DoorwayCenter / 5);		// /10 tenth inches  * 2 for more turn 
		ROBOT_LOG( TRUE, "DOORWAY: Center = %d,  Turn = %d",  DoorwayCenter, Turn )
/*		if( DoorwayCenter > 50 )		// tenth inches off course
			Turn = TURN_RIGHT_MED_SLOW;
		else if( DoorwayCenter > 10 )
			Turn = TURN_RIGHT_SLOW;	
		else if( DoorwayCenter < -10 )
			Turn = TURN_LEFT_SLOW;	
		else if( DoorwayCenter < -50 )
			Turn = TURN_LEFT_MED_SLOW;	
		else
			Turn = TURN_CENTER;
*/
		ROBOT_LOG( TRUE, "**********************************\n" )
		ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: DOORWAY Found - Center = %d, Width = %d Inches\n", DoorwayCenter/10, DoorwayWidth/10 )
		ROBOT_LOG( TRUE, "**********************************\n" )

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
#endif // REMOVE_ALL_THIS


/////////////////////////////////////////////////////////////////
void CAvoidObjectModule::HandleKinectPosition( )
{
	// This function will make sure Kinect is in correct position for spotting objectes, if needed

	if(	(0 == m_pDriveCtrl->GetCurrentSpeed()) && (0 == m_pDriveCtrl->GetCurrentTurn())  )
	{
		// Not moving. Ignore sensors so we don't suddenly start moving if someone obscures a sensor!
		// Also, move Kinect to default position for spotting Humans

		if( m_pKinectServoControl->GetTiltPosition() < KINECT_OBJECT_AVOIDANCE_POSITION + 100 )
		{
			// Kinect in position to spot objects, but we are not moving
			if( !m_WaitingToMoveKinect )
			{
				// not yet waiting so set a timer to keep from thrashing the Kinect servo.  once it expires, we'll move the servo
				gAvoidanceTimer = THREAT_PERSISTANCE_TIME;
				m_WaitingToMoveKinect = TRUE;
			}
			else
			{
				// waiting for timer to expire
				if( 0 == gAvoidanceTimer )
				{
					// expired
					// Move Kinect back into position to detect people, 
					#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
///TODO-MUST_REMOVED FOR DEBUG_ONLY!						m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_HUMAN_DETECT_START_POSITION );
					#endif
					m_WaitingToMoveKinect = FALSE;
				}
			}
		}
	}
	else
	{
		// Robot is moving	(not just doing a spin in place)		
		if( m_pKinectServoControl->GetTiltPosition() > KINECT_OBJECT_AVOIDANCE_POSITION + 100 )
		{
			// Kinect in not in position to spot objects!
			// Note: this can be suppressed, for example by the "Follow Me" behavior
			#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
				m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_OBJECT_AVOIDANCE_POSITION );
			#endif
		}
	}

}


/////////////////////////////////////////////////////////////////
// Create a 2D grid of area in front and side of robot to determine where objects are
//Grid is MAX_RANGE x MAX_RANGE, anchored to front of robot


BOOL CAvoidObjectModule::BuildWeightedOccupancyGrid()
{

//	int MaxValue = LASER_RANGEFINDER_TENTH_INCHES_MAX;
//	double XCenter = MaxValue; // LASER_RANGEFINDER_TENTH_INCHES_MAX;
//	double XMax = MaxValue*2.0;// LASER_RANGEFINDER_TENTH_INCHES_MAX*2;	// for half circle
//	double YMax = MaxValue*1.2;	// allow negative values, behind front of robot

	// Clear the grid to all zeros
	for( int GridX = 0; GridX < OCCUPANCY_GRID_SIZE; GridX++ ) 
	{
		for( int GridY = 0; GridY < OCCUPANCY_GRID_SIZE; GridY++ ) 
		{
			m_OccupancyGrid[GridX][GridY] = 0;
		}
	}
	
	if( NULL == g_KinectPointCloud )
	{
		return false; 
	}

	int XData, YData;


	/////////////// DEBUG //////////////
	// Left Side
	for( int GridXRaw = 0;  GridXRaw > (-OCCUPANCY_GRID_SIZE/2);  GridXRaw-- ) 
	{
		for( int GridY = 0; GridY < OCCUPANCY_GRID_SIZE; GridY++ ) 
		{

			int GridX = GridXRaw + (OCCUPANCY_GRID_SIZE / 2);		// 0 is in center of grid in X direction
			if( GridX < 0 )
				ROBOT_ASSERT(0);

			// good value.  Apply weight based upon grid square location
			int WeightX = (OCCUPANCY_GRID_SIZE /2) - abs(GridXRaw);
			int WeightY = OCCUPANCY_GRID_SIZE - GridY;
			int Weight = (WeightX * WeightY * 100) / (OCCUPANCY_GRID_SIZE * OCCUPANCY_GRID_SIZE);

			if( Weight > m_OccupancyGrid[GridX][GridY] ) // save largest weight found
			{
				m_OccupancyGrid[GridX][GridY] = Weight; // indicate an object is in the box
			}

		}
	}

	// right
	for( int GridXRaw = 0; GridXRaw < (OCCUPANCY_GRID_SIZE/2);  GridXRaw++ ) 
	{
		for( int GridY = 0; GridY < OCCUPANCY_GRID_SIZE; GridY++ ) 
		{

			int GridX = GridXRaw + (OCCUPANCY_GRID_SIZE / 2);		// 0 is in center of grid in X direction
			if( GridX < 0 )
				ROBOT_ASSERT(0);
			if( GridX >= OCCUPANCY_GRID_SIZE-1 )
				ROBOT_ASSERT(0);

			// good value.  Apply weight based upon grid square location
			int WeightX = (OCCUPANCY_GRID_SIZE /2) - abs(GridXRaw);
			int WeightY = OCCUPANCY_GRID_SIZE - GridY;
			int Weight = (WeightX * WeightY * 100) / (OCCUPANCY_GRID_SIZE * OCCUPANCY_GRID_SIZE);

			if( Weight > m_OccupancyGrid[GridX][GridY] ) // save largest weight found
			{
				m_OccupancyGrid[GridX][GridY] = Weight; // indicate an object is in the box
			}

		}
	}




	return true;




	/////////////////////////

	
	for( int i = 0; i < g_KinectPointCloud->FrameSizeX; i++ ) 
	{
		XData = g_KinectPointCloud->MapPoints2D[i].X; // tenth inches
		YData = g_KinectPointCloud->MapPoints2D[i].Y ;

		if( (XData >= LASER_RANGEFINDER_TENTH_INCHES_MAX) || (YData >= LASER_RANGEFINDER_TENTH_INCHES_MAX) )
		{
			continue;
		}

		int GridXRaw = XData / 60;	// tenth inches
		int GridX = GridXRaw + (OCCUPANCY_GRID_SIZE / 2);		// 0 is in center of grid in X direction
		if( GridX < 0 )
			ROBOT_ASSERT(0);

		int GridY = YData / 60;	// tenth inches

		if( (abs(GridX) >= OCCUPANCY_GRID_SIZE) || (abs(GridX) < 0) || (GridY > OCCUPANCY_GRID_SIZE) )
		{
			// Bad value
			continue;
		}

		// good value.  Apply weight based upon grid square location
		int WeightX = (OCCUPANCY_GRID_SIZE /2) - abs(GridXRaw);
		int WeightY = OCCUPANCY_GRID_SIZE - GridY;
		int Weight = (WeightX * WeightY * 100) / (OCCUPANCY_GRID_SIZE * OCCUPANCY_GRID_SIZE);

		if( Weight > m_OccupancyGrid[GridX][GridY] ) // save largest weight found
		{
			m_OccupancyGrid[GridX][GridY] = Weight; // indicate an object is in the box
		}
	}


	return true;
}

/////////////////////////////////////////////////////////////////
// Determine the direction robot should move
//

int CAvoidObjectModule::RecommendClearestDirection()
{
	int LeftWeight = 0;
	int RightWeight = 0;

	if( !BuildWeightedOccupancyGrid() )
	{
		return 0; // no grid created
	}

	// Left Side
	ROBOT_LOG( TRUE, "DEBUG: Occupancy Grid\nLeft:\n" )
	for( int GridX = 0; GridX < OCCUPANCY_GRID_SIZE/2; GridX++ ) 
	{
		for( int GridY = 0; GridY < OCCUPANCY_GRID_SIZE; GridY++ ) 
		{
			LeftWeight += m_OccupancyGrid[GridX][GridY];
			TRACE("%3d ", m_OccupancyGrid[GridX][GridY] );
		}
		TRACE("\n");
	}

	// Right Side
	TRACE("\nRight:\n");
	for( int GridX = OCCUPANCY_GRID_SIZE/2; GridX < OCCUPANCY_GRID_SIZE; GridX++ ) 
	{
		for( int GridY = 0; GridY < OCCUPANCY_GRID_SIZE; GridY++ ) 
		{
			RightWeight += m_OccupancyGrid[GridX][GridY];
			TRACE("%3d ", m_OccupancyGrid[GridX][GridY] );
		}
		TRACE("\n");
	}

	TRACE("\n\n");

	// TODO: normalize to some max value?
	int Direction = (RightWeight - LeftWeight);
	TRACE("DEBUG: Occupancy Grid Direction = %d\n\n", Direction);
	return Direction;
}

// End of AvoidObjectModule




