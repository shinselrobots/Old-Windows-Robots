// DriveBehaviorModule.cpp: Lower level behaviors for drive control, such as object avoidance and docking


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
//	MODULE: COLLISION HANDLING MODULE

CCollisionModule::CCollisionModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_CollisionState = DISABLED;
	m_CollisionDirection = 0;
	gCollisionTimer = 0;
	m_RandomTurn = 0;
}

CCollisionModule::~CCollisionModule()
{
		ROBOT_LOG( TRUE, "~CCollisionModule()\n" )
}



void CCollisionModule::ProcessMessage( 
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);

	switch( uMsg )
	{
		case WM_ROBOT_ENABLE_COLLISION_MODULE:		// COLLISION_MODULE
		{

			// Command from user to enable/disable the Collision Module
			g_bCmdRecognized = TRUE;
			// wParam = Enabled/Disabled
			// lParam = not used

			if( wParam )
			{
				// module enabled
				m_CollisionState = IDLE;
				ROBOT_DISPLAY( TRUE, "Collision Module Enabled" )
			}
			else
			{
				m_CollisionState = DISABLED;
				m_pDriveCtrl->ReleaseOwner( COLLISION_MODULE );
				ROBOT_DISPLAY( TRUE, "Collision Module Disabled" )
			}
			return;
		}

		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;
			//BYTE nDistHigh, nDistLow;
			CString MsgString;

			if( DISABLED == m_CollisionState )
			{
				// This module is disabled
				return;
			}

			if( !m_pDriveCtrl->IsOwner(COLLISION_MODULE) )
			{
				// This module lost control of the drive wheels.  Go to a good state.
				m_CollisionState = IDLE;
			}
			
			if( (IDLE == m_CollisionState ) && 
				(0 == m_pDriveCtrl->GetCurrentSpeed()) && 
				(0 == m_pDriveCtrl->GetCurrentTurn())  )
			{
				// Not currently moving.  
				// Ignore sensors so we don't suddenly start moving if someone obscures a sensor!
				return;
			}

			// See if we need to wait for a delay to complete (CARBOT ONLY!)
			if( gCollisionTimer != 0 ) 
			{
				ROBOT_LOG( TRUE, "Waiting for Collision Timer (%d)\n", gCollisionTimer )
				return;

			}

			///////////////////////////////////////////////////////////////////
			// Process Collision State Machine

			////////////////////////////////////////////////////////////////
			#if ( (ROBOT_TYPE == LOKI) || ( ROBOT_TYPE == TURTLE) )
			////////////////////////////////////////////////////////////////
			#define COLLISION_BACKUP_SPEED				SPEED_REV_MED_SLOW
			#define COLLISION_BACKUP_DISTANCE			MOVE_DISTANCE_MED
			#define COLLISION_INITIAL_ROTATION_AMOUNT	15 // degrees
			#define COLLISION_FINAL_ROTATION_AMOUNT		25 // degrees	 - don't keep driving into the wall


//			MsgString.Format("DEBUG FRONT SENSORS! Range=%d Tenth Inches, Angle=%d",  
//				g_pSensorSummary->nFrontObjectDistance, g_pSensorSummary->nFrontObjectDirection);
//			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );

			if( BRAKING != m_CollisionState )
			{
				// unless we are already Braking due to a collision, handle new collision
				// even if currently handling another collision.
/** TODO - is this needed?
				// Check for object obstructing elbow (the most common collision, since they stick out!
				if( g_pSensorSummary->bObjectElbowLeft || g_pSensorSummary->bObjectElbowRight )
				{
					if( g_pSensorSummary->bObjectElbowLeft && g_pSensorSummary->bObjectElbowRight )
					{
						// collision on both sides  Oh no!  what to do?
					ROBOT_DISPLAY( TRUE, "COLLISION MODULE: ERROR! Elbow collision on Both Sides!!!\n" )
						SpeakText( "Error! Arm hits on both sides" );	
						Turn = TURN_CENTER;	// NO turn, just stop!
						Speed = SPEED_STOP;	
					}
					else if( g_pSensorSummary->bObjectElbowLeft )
					{		
						ROBOT_DISPLAY( TRUE, "COLLISION MODULE: Arm Left Side!\n" )
						//SpeakText( "Elbow hit Left" );	
						Turn = TURN_RIGHT_MED_SLOW;
						Speed = SPEED_REV_SLOW;	// force a backward pivot turn on opposite wheel
					}
					else if( g_pSensorSummary->bObjectElbowRight )
					{		
						ROBOT_DISPLAY( TRUE, "COLLISION MODULE: Arm Right Side!\n" )
						//SpeakText( "Elbow hit Right" );	
						Turn = TURN_LEFT_MED_SLOW;
						Speed = SPEED_REV_SLOW;	// force a backward pivot turn
					}
					else
					{
						// Logic Error
						ROBOT_ASSERT(0);
					}
					// ER1 Controller does not have "Breaking" but stop does a pretty fast stop!
					// Seems to ignore the deceleration profile... just stops fast
					m_pDriveCtrl->Stop( COLLISION_MODULE );
					//SpeakCannedPhrase( SPEAK_COLLISION );

					m_CollisionState = BRAKING;
					ROBOT_DISPLAY( TRUE, "Collision State: BRAKING")
					return;	// Let the motor control issue the stop command!

				}

***/
				if( (g_pSensorSummary->nFrontObjectDistance <= OBJECT_COLLISION) ||
					(g_pSensorSummary->nSideObjectDistance  <= OBJECT_COLLISION) ||					
					((g_pSensorSummary->nRearObjectDistance <= OBJECT_COLLISION)&& (m_pDriveCtrl->GetCurrentSpeed() < SPEED_STOP))	) 	// Only look at rear sensor if backing up!
				{
					// Collision
					ROBOT_LOG( TRUE, "COLLISION MODULE: Collision detected\n" )

					CString MsgString, SensorString;
					if( g_pSensorSummary->nFrontObjectDistance <= OBJECT_COLLISION )
					{
						// Collision was in front
						m_CollisionDirection = g_pSensorSummary->nFrontObjectDirection; // Save so later we know where the threat came from.
						if( HW_BUMPER_HIT_FRONT )
							SensorString.Format( "Front Bumper: ");
						else
							SensorString.Format( "Front Sensor: ");
						MsgString.Format("COLLISION! %s Range=%d inches, Angle=%d", SensorString, 
							g_pSensorSummary->nFrontObjectDistance/10, g_pSensorSummary->nFrontObjectDirection);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					}
					else if( g_pSensorSummary->nSideObjectDistance <= OBJECT_COLLISION )
					{
						// Collision was on a side
						m_CollisionDirection = g_pSensorSummary->nSideObjectDirection; // Save so later we know where the threat came from.
						if( HW_BUMPER_HIT_SIDE_LEFT || HW_BUMPER_HIT_SIDE_RIGHT )
							SensorString.Format( "Side Bumper: ");
						else
							SensorString.Format( "Side Sensor: ");
						SensorString.Format("COLLISION! Side Sensor: Range=%d inches, Angle=%d", 
							g_pSensorSummary->nSideObjectDistance/10, g_pSensorSummary->nSideObjectDirection);
						ROBOT_DISPLAY( TRUE, (LPCTSTR)SensorString )
					}
					else if( g_pSensorSummary->nRearObjectDistance <= OBJECT_COLLISION )
					{
						// Collision was at the rear!
						m_CollisionDirection = g_pSensorSummary->nRearObjectDirection; // Save so later we know where the threat came from.
						MsgString.Format("COLLISION! Rear Bumper: Range=%d inches, Angle=%d", 0, m_CollisionDirection);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					}
					else
					{
						// Should never get here!
						ROBOT_DISPLAY( TRUE, "ERROR! Module.cpp Logic Error - Collision State Machine" )
						ROBOT_ASSERT(0);
					}

					// Loki Controller does not have "Breaking" but stop does a pretty fast stop!
					m_pDriveCtrl->Stop( COLLISION_MODULE );
					//SpeakCannedPhrase( SPEAK_COLLISION );
					m_CollisionState = BRAKING;
					ROBOT_DISPLAY( TRUE, "Collision State: BRAKING" )
					return;	// Let the motor control issue the stop command!
				}
			}	
						
			
			switch (m_CollisionState) 
			{
				case IDLE:	// LOKI
				{
					// Handled by the default collision detection above
				}
				break;

				case BRAKING:	// LOKI
				{
					// Wait for Arduino to report that we are done braking
					if( m_pDriveCtrl->RobotStopped() )
					{
						// Done Braking.  Now move away from the object

						// Pick a random direction to use in case the collision was straight ahead
						if( rand() > (RAND_MAX/2) )
							m_RandomTurn = TURN_LEFT_SLOW;
						else
							m_RandomTurn = TURN_RIGHT_SLOW;

						if( REAR_BUMPER_ANGLE == m_CollisionDirection )
						{
							// Robot was backing up when we hit object on bumper, so just move slightly to remove pressure from the bumper
							// we were probably just avoiding something else anyway...
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_FWD_MED_SLOW, 
								TURN_CENTER, 30, STOP_AFTER );	// TenthInches
							ROBOT_LOG( TRUE, "Collision Module - Moving Forward\n" )
							m_CollisionState = FINAL_MOVE;
							ROBOT_DISPLAY( TRUE, "Collision State: FINAL_MOVE" )
							return;
						}
						else if( REAR_LEFT == m_CollisionDirection )
						{
							// Robot was backing up when we hit on Left side, so move forward Right, then Left
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, SPEED_FWD_MED_SLOW, // pivot on one wheel
								TURN_RIGHT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module - Moving Forward Right\n" )
							m_CollisionState = TURN_FORWARD1;
							ROBOT_DISPLAY( TRUE, "Collision State: TURN_FORWARD1" )
							return;
						}
						else if( REAR_RIGHT == m_CollisionDirection )
						{
							// Robot was backing up when we hit on Right side, so move forward Left, then Right
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, SPEED_FWD_MED_SLOW, // pivot on one wheel
								TURN_LEFT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module - Moving Forward Left\n" )
							m_CollisionState = TURN_FORWARD1;
							ROBOT_DISPLAY( TRUE, "Collision State: TURN_FORWARD1" )
							return;
						}
						// ROBOT WAS MOVING FORWARD
						else if( m_CollisionDirection < 0 )
						{
							// Object on the Left, so pivot Right, and then left
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, COLLISION_BACKUP_SPEED, // pivot on wheel
								TURN_RIGHT_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Rotate Right\n" )
						}
						else if( m_CollisionDirection > 0 )
						{
							// Object on the Right, so pivot Left, and then right
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, COLLISION_BACKUP_SPEED, // pivot on wheel
								TURN_LEFT_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Rotate Left\n" )
						}
						else
						{
							// Object straight ahead.  Back away curve in random direction
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, COLLISION_BACKUP_SPEED, 
								m_RandomTurn, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Obj ahead, Backing Straight\n" )
						}								
						m_CollisionState = TURN_BACKUP1;
						ROBOT_DISPLAY( TRUE, "Collision State: TURN_BACKUP1" )
						return;
					}
				}
				break;

				case TURN_BACKUP1:	// LOKI
				{
					// Wait for drive control to report that we are done rotating
					if( m_pDriveCtrl->TurnRotationCompleted() )
					{
						// Done with slight rotation. Now back up straight
						m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, COLLISION_BACKUP_SPEED, 
							TURN_CENTER, COLLISION_BACKUP_DISTANCE, DONT_STOP_AFTER);
						ROBOT_LOG( TRUE, "Collision Module - Backing Straight\n" )
							
						m_CollisionState = BACKING1;
						ROBOT_DISPLAY( TRUE, "Collision State: BACKING1" )
						return;
					}						
				}	
				break;

				case BACKING1:	// LOKI
				{
					// Wait for drive control to report that we are done backing
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done with first part of backing up. Turn the other way
						if( m_CollisionDirection < 0 )
						{
							// Object on the Left, so rotate the other direction (swing the back-end to the right)
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, COLLISION_BACKUP_SPEED, // pivot on wheel
								TURN_LEFT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Rotate Right\n" )
						}
						else if( m_CollisionDirection > 0 )
						{
							// Object on the Right, so rotate the other way
							m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, COLLISION_BACKUP_SPEED, // pivot on wheel
								TURN_RIGHT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Rotate Left\n" )
						}
						else
						{
							// Object straight ahead.  Random turn the other way
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, COLLISION_BACKUP_SPEED, 
								m_RandomTurn, COLLISION_BACKUP_DISTANCE, DONT_STOP_AFTER);
							ROBOT_LOG( TRUE, "Collision Module 1- Obj ahead, Backing Straight\n" )
						}								
						m_CollisionState = TURN_BACKUP2;
						ROBOT_DISPLAY( TRUE, "Collision State: TURN_BACKUP2" )
						return;
					}						
				}	
				break;

				case TURN_BACKUP2:	// LOKI
				{
					// Wait for drive control to report that we are done rotating after backing
					if( m_pDriveCtrl->TurnRotationCompleted() )
					{
						// Done turning.  Ready to go foward again.
						m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_FWD_MED_SLOW, 
							TURN_CENTER, 60); // TenthInches
						ROBOT_LOG( TRUE, "Collision Module - Moving forward slightly\n" )
							
						m_CollisionState = FINAL_MOVE;
						ROBOT_DISPLAY( TRUE, "Collision State: FINAL_MOVE" )
						return;
					}						
				}	
				break;

				// Colision while backing up
				case TURN_FORWARD1:	// LOKI
				{
					// Moving forward after a rear collision.
					// Done with first part of S turn. Turn the other way
					if( m_CollisionDirection < 0 )
					{
						// Object on the Left, so swing back left again
						m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, SPEED_FWD_MED_SLOW, // pivot on wheel
							TURN_LEFT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, STOP_AFTER);
						ROBOT_LOG( TRUE, "Collision Module 1- Rotate Right\n" )
					}
					else if( m_CollisionDirection > 0 )
					{
						// Object on the Right, so swing back right again
						m_pDriveCtrl->SetTurnRotation( COLLISION_MODULE, SPEED_FWD_MED_SLOW, // pivot on wheel
							TURN_RIGHT_MED_SLOW, COLLISION_INITIAL_ROTATION_AMOUNT, STOP_AFTER);
						ROBOT_LOG( TRUE, "Collision Module 1- Rotate Left\n" )
					}
					else
					{
						// Object straight ahead.  Random turn the other way
						m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_FWD_MED_SLOW, 
							-m_RandomTurn, COLLISION_BACKUP_DISTANCE, DONT_STOP_AFTER);
						ROBOT_LOG( TRUE, "Collision Module 1- Obj ahead, Backing Straight\n" ) 
					}								
					m_CollisionState = FINAL_MOVE;
					ROBOT_DISPLAY( TRUE, "Collision State: FINAL_MOVE" )
					return;
				}	
				break;

				case FINAL_MOVE:	// LOKI
				{
					// Wait for drive control to report that we are done moving forward
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done moving forward, 
						// Done recovering from collision. Ready to resume where we left off.
						m_CollisionDirection = 0;
						m_CollisionState = IDLE;
						ROBOT_DISPLAY( TRUE, "Collision State: Done --> IDLE" )
						m_pDriveCtrl->ReleaseOwner( COLLISION_MODULE );
					}
				}
				break;

				default:
				{
					MsgString.Format( "ERROR! Illegal m_CollisionState = %d", m_CollisionState );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					m_CollisionState = IDLE;
					m_pDriveCtrl->ReleaseOwner( COLLISION_MODULE );
				}
			}	// Switch Collision State


			////////////////////////////////////////////////////////////////
			#elif ROBOT_TYPE == CARBOT
			////////////////////////////////////////////////////////////////
			switch (m_CollisionState) 
			{
				case IDLE:	// CARBOT
				{
					if( (g_pSensorSummary->nFrontObjectDistance < OBJECT_COLLISION) ||
						(g_pSensorSummary->nSideObjectDistance < OBJECT_COLLISION) )
					{
						// object very close, or bumper hit
						ROBOT_LOG( TRUE, "COLLISION MODULE: Collision detected\n" )

						CString MsgString, SensorString;
						if( g_pSensorSummary->nFrontObjectDistance < g_pSensorSummary->nSideObjectDistance )
						{
							// Collision was in front
							m_CollisionDirection = g_pSensorSummary->nFrontObjectDirection; // Save so later we know where the threat came from.

							if( HW_BUMPER_HIT_FRONT )
							{
								SensorString.Format( "Front Bumper: ");
							}
							else
							{
								SensorString.Format( "Front Sensor: ");
							}
						}
						else
						{
							// Collision was on a side
							m_CollisionDirection = g_pSensorSummary->nSideObjectDirection; // Save so later we know where the threat came from.

							SensorString.Format("COLLISION! Side Sensor: Range=%d inches, Angle=%d", 
								g_pSensorSummary->nSideObjectDistance/10, g_pSensorSummary->nSideObjectDirection);
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						}

						MsgString.Format("COLLISION! %s Range=%d inches, Angle=%d", SensorString, 
							g_pSensorSummary->nFrontObjectDistance/10, g_pSensorSummary->nFrontObjectDirection);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )


						m_pDriveCtrl->Brake( COLLISION_MODULE );
						//SpeakCannedPhrase( SPEAK_COLLISION );

						m_CollisionState = BRAKING;
						ROBOT_DISPLAY( TRUE, "Collision State: BRAKING" )
					}
				}
				break;

				case BRAKING:	// CARBOT
				{
					// Wait for Arduino to report that we are done braking
					if( m_pDriveCtrl->RobotStopped() )
					{
						// Done Braking.  Now back away from the object
						if( m_CollisionDirection < 0 )
						{
							// Object on the Left, turn wheels Right while backing
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_RIGHT_MED_SLOW, MOVE_DISTANCE_MED);
						}
						else if( m_CollisionDirection > 0 )
						{
							// Object on the Right, turn wheels Left while backing
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_LEFT_MED_SLOW, MOVE_DISTANCE_MED);
						}
						else
						{
							// Object straight ahead.  Back away straight
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_CENTER, MOVE_DISTANCE_MED);
						}	
							
						m_CollisionState = BACKING1;
						ROBOT_DISPLAY( TRUE, "Collision State: BACKING1" )
						return;
					}						
				}	
				break;
			

				case BACKING1:	// CARBOT
				{
					// Wait for drive control to report that we are done backing
					// Ignore reports of collision in front, while we were backing up
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done with first part of backing up.
						// If curving turn wheels the other way to prepare robot for
						// going around the object.
						if( m_CollisionDirection < 0 )
						{
							// Object on the Left, now turn wheels Left while backing
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_LEFT_MED, MOVE_DISTANCE_MED_SHORT);
						}
						else if( m_CollisionDirection > 0 )
						{
							// Object on the Right, now turn wheels Right while backing
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_RIGHT_MED, MOVE_DISTANCE_MED_SHORT);
						}
						else
						{
							// Object straight ahead.  Back away straight
							m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_REV_MED_SLOW, 
								TURN_CENTER, MOVE_DISTANCE_MED_SHORT);
						}	
							
						m_CollisionState = BACKING2;
						ROBOT_DISPLAY( TRUE, "Collision State: BACKING2" )
						return;
					}						
				}	
				break;

				case BACKING2:	// CARBOT
				{
					// Wait for drive control to report that we are done backing
					// TODO: Monitor collision in back, while we were backing up
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done backing up, now wait for timer to allow for coasting, and for wheels to center
						m_pDriveCtrl->SetTurn(COLLISION_MODULE, TURN_CENTER); // center wheels while coasting
						gCollisionTimer = MOVE_DISTANCE_COASTING_TIME;
						m_CollisionState = FORWARD1;
						ROBOT_DISPLAY( TRUE, "Collision State: FORWARD1" )
					}
				}
				break;

				case FORWARD1:	// CARBOT
				{
					// Timer completed
					// Done backing up, now turn wheels and go forward
					if( m_CollisionDirection > 0 )
					{
						// Turn Left (point away from objects to the right).
						m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_FWD_MED_SLOW, 
							TURN_LEFT_FAST, MOVE_DISTANCE_MED_SHORT);
					}
					else
					{
						// Turn Right
						m_pDriveCtrl->SetMoveDistance( COLLISION_MODULE, SPEED_FWD_MED_SLOW, 
							TURN_RIGHT_FAST, MOVE_DISTANCE_MED_SHORT);
					}
					m_CollisionState = FORWARD2;
					ROBOT_DISPLAY( TRUE, "Collision State: FORWARD2" )
				}
				break;

				case FORWARD2:	// CARBOT
				{
					// Wait until we are done going forward
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done recovering from collision. Ready to resume where we left off.
						m_CollisionDirection = 0;
						m_CollisionState = IDLE;
						ROBOT_DISPLAY( TRUE, "Collision State: Done --> IDLE" )
						m_pDriveCtrl->ReleaseOwner( COLLISION_MODULE );
						return;
					}
				}
				break;

				default:
				{
					MsgString.Format( "ERROR! Illegal m_CollisionState = %d", m_CollisionState );
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					m_CollisionState = IDLE;
					m_pDriveCtrl->ReleaseOwner( COLLISION_MODULE );
				}
			}	// Switch Collision State


			////////////////////////////////////////////////////////////////
			#else
				#error BAD ROBOT_TYPE
			#endif
			////////////////////////////////////////////////////////////////


		}	// Switch Status Ready
	}	// Switch UMsg

}
// End of CollisionModule



///////////////////////////////////////////////////////////////////////////////
//	MODULE: OBJECT AVOIDANCE MODULE

#define AVOIDANCE_FORWARD_SPEED		SPEED_FWD_MED_SLOW
#define AVOIDANCE_FORWARD_DISTANCE						 20 // TenthInches
#define AVOIDANCE_TURN_SPEED		SPEED_FWD_MED_SLOW
#define DOOR_SPOTTING_DISTANCE_TENTH_INCHES				480
#define DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES		300
#define DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES		230	// 23 inches.  Robot is 22 inches wide, including arms!


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
#endif
			}
			else
			{
				m_AvoidanceState = DISABLED;
				m_pDriveCtrl->ReleaseOwner( AVOID_OBJECT_MODULE );
	#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
				m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_HUMAN_DETECT_START_POSITION );	// By default, leave Kinect in a position where it can find humans
				m_pKinectServoControl->ReleaseOwner(KINECT_TILT_OWNER_COLLISION_AVOIDANCE);
#endif
				ROBOT_DISPLAY( TRUE, "Avoidance Module Disabled" )
			}
			return;
		}
		break;

		case WM_ROBOT_SERVO_STATUS_READY:
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;
			//BYTE nDistHigh, nDistLow;
			CString MsgString;


			if( DISABLED == m_AvoidanceState )
			{
				// This module is disabled
				return;
			}

			if( !m_pDriveCtrl->IsOwner(AVOID_OBJECT_MODULE) )
			{
				// This module lost control of the drive wheels.  Go to a good state.
				m_AvoidanceState = IDLE;
			}

			if( (IDLE == m_AvoidanceState) &&
				(0 == m_pDriveCtrl->GetCurrentSpeed()) && 
				(0 == m_pDriveCtrl->GetCurrentTurn())  )
			{
				// Not currently moving.
				if( m_PriorSpeed != SPEED_STOP )
				{
					// we were moving but now we are not.  Set a timer to keep from thrashing the Kinect servo
						gAvoidanceTimer = THREAT_PERSISTANCE_TIME;
						m_PriorSpeed = SPEED_STOP;
				}

				// Ignore sensors so we don't suddenly start moving if someone obscures a sensor!
				// Also, leave Kinect in default position until robot starts moving

				// Move Kinect into position to detect people, if no higher owner has control
				if( gAvoidanceTimer == 0 ) 
				{
					#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
						m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_HUMAN_DETECT_START_POSITION );
					#endif
				}

				return;
			}

			//int DbgSpeed = m_pDriveCtrl->GetCurrentSpeed();
			//int DbgTurn = m_pDriveCtrl->GetCurrentTurn();
			//ROBOT_LOG( TRUE, "speed = %d, Turn = %d\n", DbgSpeed, DbgTurn )

			// Move Kinect into position to detect objects
			#if ( ROBOT_SERVER == 1 ) //////////////// SERVER ONLY //////////////////
				m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_COLLISION_AVOIDANCE, KINECT_OBJECT_AVOIDANCE_POSITION );
			#endif

			if( IDLE == m_AvoidanceState )
			{
				// This module was not in control at last status update
				// Save the last commanded speed
				m_PriorSpeed = m_pDriveCtrl->GetCurrentSpeed();
				Speed = m_PriorSpeed;	// Use this speed as default while avoiding.  May be overriden below.
			}
			else
			{
				// This module was in control at last status update
				// Restore the prior speed for use in calculations below
				Speed = m_PriorSpeed;
			}

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

			int  AvoidObjectDistanceTenthInches = __min( (g_GlobalMaxAvoidObjectDetectionFeet * 120), g_SegmentAvoidObjectRangeTenthInches );

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

				if(	(g_pSensorSummary->nFrontObjectDistance <= AvoidObjectDistanceTenthInches)		||
					(g_pSensorSummary->nLeftSideZone < m_SideAvoidDistanceTenthInches)			||
					(g_pSensorSummary->nRightSideZone < m_SideAvoidDistanceTenthInches)		)
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
					if( g_pSensorSummary->nLeftFrontZone > AvoidObjectDistanceTenthInches )
					{
						return -10; // Clear path ahead Left!
					}
					if( g_pSensorSummary->nRightFrontZone > AvoidObjectDistanceTenthInches )
					{
						return 10; // Clear path ahead Right!
					}
					if( g_pSensorSummary->nLeftArmZone > AvoidObjectDistanceTenthInches )
					{
						return -20; // Clear path ahead Hard Left!
					}
					if( g_pSensorSummary->nRightArmZone > AvoidObjectDistanceTenthInches )
					{
						return 20; // Clear path ahead Hard Right!
					}

					// Ok, not so easy.  Try calculating potential threats
					int ThreatsLeft = 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nLeftFrontZone)*3 ) + 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nLeftArmZone)*2 ) + 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nLeftSideZone)    );

					int ThreatsRight = 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nRightFrontZone)*3 ) + 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nRightArmZone)*2 ) + 
						( (NO_OBJECT_IN_RANGE - g_pSensorSummary->nRightSideZone)    );

					return( ThreatsRight - ThreatsLeft );
					****/
				}

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif ( (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP) )

				if( m_pDriveCtrl->GetCurrentSpeed() < 0 )
				{
					// Backing up!
					if( g_pSensorSummary->nRearObjectDistance <= REAR_THREAT_THRESHOLD )
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
				Turn = 0;
				ROBOT_LOG(TRUE, "DEBUG: nFrontObjectDistance = %d", g_pSensorSummary->nFrontObjectDistance )

				// See if we should move arms out of harms way
				CheckArmSafePosition();

				if(	(g_pSensorSummary->nFrontObjectDistance <= AvoidObjectDistanceTenthInches) ||
					(g_pSensorSummary->nSideObjectDistance < m_SideAvoidDistanceTenthInches) )
				{
					// Something to avoid

					// Tighen up the side distance; focus on avoiding current obstacle
					gAvoidanceTimer = THREAT_PERSISTANCE_TIME;
					// Slow down while manuvering
/**
					Speed = Speed - SPEED_FWD_SLOW;	// subtract one "speed level" from current speed
					if( Speed < SPEED_FWD_SLOW ) Speed = SPEED_FWD_SLOW;
					if( g_pSensorSummary->nFrontObjectDistance <= AVOIDANCE_SHARP_TURN_RANGE )
					{
						// Object is close!  A sharp turn is needed!
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Forcing sharp turn to avoid close object\n" )
						Speed = SPEED_FWD_SLOW;	// force almost a pivot turn
					}
**/
				}

				// Start Avoidance Behavior

				if( DetectAndHandleCliff(Turn, Speed) )
				{
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Avoiding Cliff\n" )
				}
				// Avoid very close objects in key zones
				else if( (g_pSensorSummary->nRightArmZone <= ARM_ZONE_THREAT_MIN_THRESHOLD) ||
						 (g_pSensorSummary->nLeftArmZone <= ARM_ZONE_THREAT_MIN_THRESHOLD) )
				{
					// close object about to hit an arm
					if( g_pSensorSummary->nRightArmZone < g_pSensorSummary->nLeftArmZone )
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
				else if( DetectAndHandleDoorway(Turn, Speed) )
				{
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: AVOID_OBJECT_MODULE: Heading for Doorway\n" )

				}				
				else if( g_pSensorSummary->nFrontObjectDistance <= AvoidObjectDistanceTenthInches )
				{
					// There is an object somewhere in front of us. 
					ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: FrontObjectDistance = %d, direction = %d\n",
						g_pSensorSummary->nFrontObjectDistance, g_pSensorSummary->nFrontObjectDirection )

					if( g_pSensorSummary->nFrontObjectDirection > 0 )
					{
						// Object to the Right, Turn Left
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Left to avoid object on Front Right\n" )
						Turn = TURN_LEFT_MED_SLOW;
					}
					else if( g_pSensorSummary->nFrontObjectDirection < 0 )
					{
						// Object to the Left, Turn Right
						ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Turning Right to avoid object on Front Left\n" )
						Turn = TURN_RIGHT_MED_SLOW;
					}
					else
					{
						// Object Dead Ahead. Check side sensors for a hint
						if( 0 != g_pSensorSummary->nSideObjectDirection )
						{
							// In range of one of the side sensors ( and both not reading identical)
							if( g_pSensorSummary->nSideObjectDirection > 0 )
							{
								// Object to the Right, Turn Left
								ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead, turning Left (other object on right side)\n" )
								Turn = TURN_LEFT_MED_SLOW;
							}
							if( g_pSensorSummary->nSideObjectDirection < 0 )
							{
								// Object to the Left, Turn Right
								ROBOT_LOG( TRUE, "AVOID_OBJECT_MODULE: Object Dead Ahead, turning Right (other object on left side)\n" )
								Turn = TURN_RIGHT_MED_SLOW;
							}
							else
							{
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
						}
					}	// End of object dead ahead

					// If object is very close, stop and force a turn on axis
					if( g_pSensorSummary->nFrontObjectDistance <= FRONT_ZONE_THREAT_MIN_THRESHOLD )
					{
						Speed = SPEED_STOP;
					}


				}	// End object somewhere in front of us. 
				else if( g_pSensorSummary->nSideObjectDistance < m_SideAvoidDistanceTenthInches )
				{
					// No object ahead, but object on the side within threshold area to avoid
					if( g_pSensorSummary->nSideObjectDirection > 0 )
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


				////////////////////////////////////////////////////////////////////////
				// Now, do the speed and turn changes!
				if( 0 != Turn )
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
					m_AvoidanceState = TURNING1;
					ROBOT_DISPLAY( TRUE, "Avoidance State: TURNING1" )

				} 
/***
				else if( TURNING1 == m_AvoidanceState )
				{
					// We had been turning to avoid an object, and the way is clear now.
					// Move forward a little before releasing control, so we don't just drive right back into the same object!
					#ifndef AVOIDANCE_MOVES_DISABLED_FOR_TESTING
						m_pDriveCtrl->SetMoveDistance( AVOID_OBJECT_MODULE, Speed, 
							TURN_CENTER, AVOIDANCE_FORWARD_DISTANCE, DONT_STOP_AFTER);	// Don't stop when done, smoothly move to normal speed
					#endif
					ROBOT_LOG( TRUE, "Avoidance Module - Avoided, now going Forward Straight for a bit\n" )
					m_AvoidanceState = FORWARD1;
					ROBOT_DISPLAY( TRUE, "Avoidance State: FORWARD1" )
				}
				else if( FORWARD1 == m_AvoidanceState )
				{
					// Wait for drive control to report that we are done moving forward
					if( m_pDriveCtrl->MoveDistanceCompleted() )
					{
						// Done moving forward, 
						// Done recovering from object Avoidance. Ready to resume where we left off.
						m_AvoidanceState = IDLE;
						ROBOT_DISPLAY( TRUE, "Avoidance State: Done --> IDLE" )
						m_pDriveCtrl->ReleaseOwner( AVOID_OBJECT_MODULE );
					}
				}
***/
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
	if(	(g_pSensorSummary->nFrontObjectDistance <= PROTECT_ARMS_FRONT_THREAT_THRESHOLD) ||
		(g_pSensorSummary->nSideObjectDistance < PROTECT_ARMS_SIDE_THREAT_THRESHOLD) )
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
	if( g_pSensorSummary->bLeftCliff || g_pSensorSummary->bRightCliff )
	{
		// Cliff detected!
		Speed = SPEED_STOP;	// force a pivot turn

		if( g_pSensorSummary->bLeftCliff && g_pSensorSummary->bRightCliff )
		{
			// Cliff on both sides  Oh no!  what to do?
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: ERROR! Cliff on Both Sides!!!\n" )
			//SpeakText( "Error! Cliff on both sides") );	
			Turn = TURN_CENTER;	// NO turn, just stop!
		}
		else if( g_pSensorSummary->bLeftCliff )
		{		
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: Cliff Left Side!\n" )
			// SpeakText( "Cliff Left" );	
			Turn = TURN_RIGHT_MED_SLOW;
		}
		else if( g_pSensorSummary->bRightCliff )
		{		
			ROBOT_DISPLAY( TRUE, "AVOID_OBJECT_MODULE: Cliff Right Side!\n" )
			//g_ClientTextToSend = "Cliff Right" );	
			Turn = TURN_LEFT_MED_SLOW;
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

	int  NumberOfSamples = 0;
	int RightEdgeX = 0;
	int LeftEdgeX = 0;
	int X, Y;
	int PreviousX = 0;
	int PreviousY = 0;

	if( (g_pSensorSummary->nLeftFrontSideZone  > DOOR_SPOTTING_DISTANCE_TENTH_INCHES) ||
		(g_pSensorSummary->nRightFrontSideZone > DOOR_SPOTTING_DISTANCE_TENTH_INCHES) ) 
	{
		return FALSE; // no door frame found in range
	}
	int TargetClearDistance = (__max(g_pSensorSummary->nRightFrontSideZone, g_pSensorSummary->nLeftFrontSideZone) + DOORWAY_MIN_CLEAR_AREA_DEPTH_TENTH_INCHES) ; // tenth inches

	NumberOfSamples = g_pLaserScannerData->NumberOfSamples;
	if( NumberOfSamples <= 0 )
	{
		return FALSE; // Bad data
	}

	for( int i = 0; i < (int)NumberOfSamples; i++ ) // Scanner goes Right to Left
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
					if( (RightEdgeX - X) >= DOORWAY_MIN_CLEAR_AREA_WIDTH_TENTH_INCHES )
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


// End of AvoidObjectModule




