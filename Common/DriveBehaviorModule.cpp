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
//				g_pNavSensorSummary->nFrontObjectDistance, g_pNavSensorSummary->nFrontObjectDirection);
//			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );

			if( BRAKING != m_CollisionState )
			{
				// unless we are already Braking due to a collision, handle new collision
				// even if currently handling another collision.
/** TODO - is this needed?
				// Check for object obstructing elbow (the most common collision, since they stick out!
				if( g_pNavSensorSummary->bObjectElbowLeft || g_pNavSensorSummary->bObjectElbowRight )
				{
					if( g_pNavSensorSummary->bObjectElbowLeft && g_pNavSensorSummary->bObjectElbowRight )
					{
						// collision on both sides  Oh no!  what to do?
					ROBOT_DISPLAY( TRUE, "COLLISION MODULE: ERROR! Elbow collision on Both Sides!!!\n" )
						SpeakText( "Error! Arm hits on both sides" );	
						Turn = TURN_CENTER;	// NO turn, just stop!
						Speed = SPEED_STOP;	
					}
					else if( g_pNavSensorSummary->bObjectElbowLeft )
					{		
						ROBOT_DISPLAY( TRUE, "COLLISION MODULE: Arm Left Side!\n" )
						//SpeakText( "Elbow hit Left" );	
						Turn = TURN_RIGHT_MED_SLOW;
						Speed = SPEED_REV_SLOW;	// force a backward pivot turn on opposite wheel
					}
					else if( g_pNavSensorSummary->bObjectElbowRight )
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
				if( (g_pNavSensorSummary->nFrontObjectDistance <= OBJECT_COLLISION) ||
					(g_pNavSensorSummary->nSideObjectDistance  <= OBJECT_COLLISION) ||					
					((g_pNavSensorSummary->nRearObjectDistance <= OBJECT_COLLISION)&& (m_pDriveCtrl->GetCurrentSpeed() < SPEED_STOP))	) 	// Only look at rear sensor if backing up!
				{
					// Collision
					ROBOT_LOG( TRUE, "COLLISION MODULE: Collision detected\n" )

					CString MsgString, SensorString;
					if( g_pNavSensorSummary->nFrontObjectDistance <= OBJECT_COLLISION )
					{ 
						// Collision was in front
						m_CollisionDirection = g_pNavSensorSummary->nFrontObjectDirection; // Save so later we know where the threat came from.
						if( g_pNavSensorSummary->BumperHitFront() )
							SensorString.Format( "Front Bumper: ");
						else
							SensorString.Format( "Front Sensor: ");
						MsgString.Format("COLLISION! %s Range=%d inches, Angle=%d", SensorString, 
							g_pNavSensorSummary->nFrontObjectDistance/10, g_pNavSensorSummary->nFrontObjectDirection);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					}
					else if( g_pNavSensorSummary->nSideObjectDistance <= OBJECT_COLLISION )
					{
						// Collision was on a side
						m_CollisionDirection = g_pNavSensorSummary->nSideObjectDirection; // Save so later we know where the threat came from.
						if( g_pFullSensorStatus->HWBumperSideLeft || g_pFullSensorStatus->HWBumperSideRight )
							SensorString.Format( "Side Bumper: ");
						else
							SensorString.Format( "Side Sensor: ");
						SensorString.Format("COLLISION! Side Sensor: Range=%d inches, Angle=%d", 
							g_pNavSensorSummary->nSideObjectDistance/10, g_pNavSensorSummary->nSideObjectDirection);
						ROBOT_DISPLAY( TRUE, (LPCTSTR)SensorString )
					}
					else if( g_pNavSensorSummary->nRearObjectDistance <= OBJECT_COLLISION )
					{
						// Collision was at the rear!
						m_CollisionDirection = g_pNavSensorSummary->nRearObjectDirection; // Save so later we know where the threat came from.
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
					if( (g_pNavSensorSummary->nFrontObjectDistance < OBJECT_COLLISION) ||
						(g_pNavSensorSummary->nSideObjectDistance < OBJECT_COLLISION) )
					{
						// object very close, or bumper hit
						ROBOT_LOG( TRUE, "COLLISION MODULE: Collision detected\n" )

						CString MsgString, SensorString;
						if( g_pNavSensorSummary->nFrontObjectDistance < g_pNavSensorSummary->nSideObjectDistance )
						{
							// Collision was in front
							m_CollisionDirection = g_pNavSensorSummary->nFrontObjectDirection; // Save so later we know where the threat came from.

							if( g_pNavSensorSummary->BumperHitFront() )
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
							m_CollisionDirection = g_pNavSensorSummary->nSideObjectDirection; // Save so later we know where the threat came from.

							SensorString.Format("COLLISION! Side Sensor: Range=%d inches, Angle=%d", 
								g_pNavSensorSummary->nSideObjectDistance/10, g_pNavSensorSummary->nSideObjectDirection);
							ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
						}

						MsgString.Format("COLLISION! %s Range=%d inches, Angle=%d", SensorString, 
							g_pNavSensorSummary->nFrontObjectDistance/10, g_pNavSensorSummary->nFrontObjectDirection);
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





