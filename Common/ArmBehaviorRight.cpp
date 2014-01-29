// ArmBehaviorRight.cpp: Subset of Behavior Module for Right Arm Control
// 
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
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
// Macros and defines

#define DEBUG_ARM_MOVEMENT		0

#define ARM_RANDOM_MOVE_CHECK_TIME 8 //how often to move the arm, in number of Sensor updates from Arduino

//#define OBJECT_DETECT_BOTH_FINGERS	0xC0
//#define OBJECT_DETECT_LEFT_FINGER	0x80
//#define OBJECT_DETECT_RIGHT_FINGER	0x40

#define PICKUP_OBJECT_MAX_Y			80	// TenthInches - from front of robot
#define PICKUP_OBJECT_MAX_X			150	// TenthInches - from center of robot

DWORD dbgTimeR = GetTickCount();

__itt_string_handle* pshHandleArmMovementRequestRight = __itt_string_handle_create("HandleArmMovementRequestRight");
__itt_string_handle* pshPickupObjectXYZRight = __itt_string_handle_create("PickupObjectXYZRight");
__itt_string_handle* pshPutDownObjectRight = __itt_string_handle_create("PutDownObjectRight");
__itt_string_handle* pshHandleArmServoStatusUpdateRight = __itt_string_handle_create("HandleArmServoStatusUpdateRight");
//	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh);
//	__itt_task_end(pDomainControlThread);


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// RIGHT ARM INITIAL MOVEMENT
void CBehaviorModule::HandleArmMovementRequestRight( WPARAM wParam, LPARAM lParam )
{
#if ROBOT_HAS_RIGHT_ARM

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmMovementRequestRight);
		m_ArmMovementRight = lParam;
		// by default, each movement should complete before the next begins
		// Can be overridden below, and gArmTimerRight used instead
		m_ArmWaitForMoveToCompleteRight = TRUE;	
		// On each arm move command, reset values to default speed (as specified by GUI)
		m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	

		// Disable random arm movements during most operations
		if( (ARM_MOVEMENT_NONE != m_ArmMovementRight) &&
			(ARM_MOVEMENT_HOME1 != m_ArmMovementRight) )
		{
			m_pArmControlRight->EnableIdleArmMovement(FALSE);
		}

		switch( m_ArmMovementRight )  // lParam is the movement requested
		{
			case ARM_MOVEMENT_NONE:
			{
				ROBOT_LOG( TRUE,"BehaviorModule: ACTION_MODE_NONE requested\n")
				break;
			}
			case ARM_MOVEMENT_HOME1:	// Home with arm down (sensors in position to move)
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME1\n")
				m_pArmControlRight->MoveArmHome();
				break;
			}
			case ARM_MOVEMENT_HOME2:	// Home with arm up
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME2\n")
				m_pArmControlRight->SetArmPosition( 
					RIGHT_ARM_SHOULDER_HOME2, RIGHT_ARM_ELBOW_ROTATE_HOME2, RIGHT_ARM_ELBOW_BEND_HOME2, 
					RIGHT_ARM_WRIST_ROTATE_HOME2, RIGHT_ARM_CLAW_HOME2 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_ArmMovementStateRight = 1;	// go to first state
				m_ArmMovementRight = lParam;	// Tell the Right arm state machine to continue this movement
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW: // Extend Arm
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW\n")
				m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW: // Extend Arm
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW\n")
				m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW:	
			{
				// Determine if claw should be open or closed, based upon claw torque load
				UINT ClawTorqueRight = m_pArmControlRight->GetClawTorque();

				if( ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT )
				{
					// Claw is holding something!
					// Same as ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW (LOAD DETECTED)\n")
					m_pArmControlRight->EnableIdleArmMovement(FALSE);
					m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePositionAndSpeed();				
					m_ArmMovementStateRight = 1;	// go to first state
					m_ArmMovementRight = ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW;	// Tell the Right arm state machine to continue this movement

					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						case 1:  SpeakText( "all right" );break;
						default: SpeakText( "Here you go" );break;
					}

				}
				else
				{
					// Not holding anything
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW (NO LOAD)\n")
					// Same as ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW
					m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePositionAndSpeed();				
					m_ArmMovementStateRight = 1;	// go to first state
					m_ArmMovementRight = ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW;	// Tell the arm state machine to continue this movement


					// Respond with random phrases
					int RandomNumber = ((4 * rand()) / RAND_MAX);
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						case 1:  SpeakText( "all right" );break;
						case 2:  SpeakText( "what do you have?" );break;
						default: SpeakText( "For me?" );break;
					}

				}

				break;
			}
			case ARM_MOVEMENT_EXTEND_FULL: // Extend Arm Fully
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_FULL\n")
				m_pArmControlRight->SetArmPosition(95, 0, 5, 0, 1 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_SHAKE_READY: // Extend Arm to shake hands
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SHAKE_READY\n")
				m_pArmControlRight->SetArmPosition(
					RIGHT_ARM_SHOULDER_ARM_SHAKE, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE,
					RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_ArmMovementStateRight = 1;	// nothing to do
				break;
			}
			case ARM_MOVEMENT_LOOK_AT_HAND: // Look at whatever might be in the right hand
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_LOOK_AT_HAND\n")
				m_pArmControlRight->SetArmPosition(50, 0, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				gHeadIdle = FALSE;
				// Look Near Expected Hand Position 
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-80, CAMERA_TILT_CENTER-40, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
				break;
			}
			case ARM_MOVEMENT_SCRATCH_HEAD: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SCRATCH_HEAD\n")
				m_pArmControlRight->SetArmPosition(130, -14, 130, 20, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();
				m_RepeatCount = 4; //number of times to scratch head
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				//gHeadIdle = FALSE;
				HeadCenter(); // Look Forward
				break;
			}
			case ARM_MOVEMENT_KARATE: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_KARATE\n")
				m_pArmControlRight->SetArmPosition(110, 0, 130, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				gHeadIdle = FALSE;
				HeadCenter(); // Look Forward
				break;
			}
			case ARM_MOVEMENT_PICKUP_OBJECT_BLIND: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PICKUP_OBJECT_BLIND\n")
				m_pArmControlRight->SetArmPosition(-45, 3, 100, -90, 80 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
				break;
			}

			case ARM_MOVEMENT_PICKUP_OBJECT_XYZ: // 
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshPickupObjectXYZRight);
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: Initial Right ARM_MOVEMENT_PICKUP_OBJECT_XYZ\n")
				// Get into inital pickup position
				ROBOT_ASSERT(0); // not implemented

				m_pArmControlRight->SetClawTorque(STRONG_TORQUE); // get better grip
				m_pArmControlRight->SetArmPosition(-20, 3, 100, 0, 110 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePosition();	
				m_ArmWaitForMoveToCompleteRight = TRUE;
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ObjectDetectCount = 0;
				m_ArmMovementStateRight = 1;	// go to first state
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
				__itt_task_end(pDomainControlThread);
				break;
			}

			case ARM_MOVEMENT_PUT_DOWN_OBJECT: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_DOWN_OBJECT\n")
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshPutDownObjectRight);
				m_pArmControlRight->SetClawTorque(STRONG_TORQUE); // get better grip
				m_pArmControlRight->SetArmPosition(-23, 3, 65, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 5;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				__itt_task_end(pDomainControlThread);
				break;
			}
			case ARM_MOVEMENT_OPEN_CLAW: // Open Claw
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_OPEN_CLAW\n")
				m_pArmControlRight->SetClawTorque(STRONG_TORQUE);	// reset to strong torque
				m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 0, RIGHT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_GRAB_COKE: // Grab Coke
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_GRAB_COKE\n")
				m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
				m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_CLOSE_CLAW_FULL: // Close Claw Full
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_CLOSE_CLAW_FULL\n")
				m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
				m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_LIFT_OBJECT: // Lift Object
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_LIFT_OBJECT\n")
				m_pArmControlRight->SetArmPosition(NOP, NOP, 70, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_ARM_UP_FULL: // Full Up
			{			
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_ARM_UP_FULL\n")
				m_pArmControlRight->EnableIdleArmMovement(FALSE);
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	// elbow slower, to avoid hitting ground
				m_pArmControlRight->ExecutePositionAndSpeed();

				m_ArmWaitForMoveToCompleteRight = FALSE;	// Don't wait for each movement!
				gArmTimerRight = 32;	// 1/10 second per count!
				m_pArmControlRight->SetArmPosition( RIGHT_ARM_SHOULDER_ARM_STRAIGHT_UP, 0, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePosition();				
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_SCRATCH_BACK:
			{	
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SCRATCH_BACK\n")
				m_pArmControlRight->SetArmPosition( 190, 0, 125, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 50;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
  			case ARM_MOVEMENT_SALUTE: // I am not the droid you are looking for...
			{			
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SALUTE\n")
				// Make the salute command a bit faster then other commands
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );	
				m_pArmControlRight->SetArmPosition( 70, 0, 100, 0, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!

				m_pArmControlRight->EnableIdleArmMovement(TRUE);
				gHeadIdle = TRUE;
				m_ArmMovementStateRight = 0;	// back to Idle state
				m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
				break;
			}
			case ARM_MOVEMENT_WAVE: 
			{			
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_WAVE\n")
				// Make the wave command a bit faster then other commands
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );	
				m_pArmControlRight->SetArmPosition( 70, 0, 100, 0, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_THROW_OBJECT_FRONT: 
			{	
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_THROW_OBJECT_FRONT\n")
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
				m_pArmControlRight->SetArmPosition( 50, 0, -20, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_PUT_IN_BASKET: 
			{	
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_IN_BASKET NOT IMPLEMENTED FOR RIGHT ARM!\n")
				break;
			}

			case ARM_MOVEMENT_IDENTIFY_OBJECT: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_IDENTIFY_OBJECT\n")
				// Close Claw (in case it's not closed already)
				m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
				m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				break;
			}

			default:
			{
				ROBOT_LOG( TRUE,"ERROR - Unhandled ARM MOVEMENT RIGHT command (%02X)!\n", m_ArmMovementRight)
			}
		} // switch m_ArmMovementRight
	__itt_task_end(pDomainControlThread);

	#endif // ROBOT_HAS_RIGHT_ARM

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleArmServoStatusUpdate
// Handle status update from the servos.
// Check Arm Movement State Machine.  Only used with movements that have multiple steps
// Uses m_ArmMovementStateRight to track progress
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::HandleArmServoStatusUpdateRight( WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);

#if ROBOT_HAS_RIGHT_ARM

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmServoStatusUpdateRight);

	if( ARM_MOVEMENT_NONE == m_ArmMovementRight ) 
	{
		// nothing to do.  Check to see if we should do random arm movements
		//RandomArmMovements()
	}
	else
	{
		// Something to do.  

		// Get current X,Y,Z position for debug
		// static int LastArmX=0, LastArmY=0,LastArmZ=0;
		//FPOINT3D_T ArmXYZ;
		//m_pArmControlRight->GetTargetArmXYZ(FPOINT3D_T &ArmXYZ )

		// Tell camera to go to ABSOLUTE PAN/TILT position pointing at X,Y,Z in space
		// wParam = X,Z in TenthInches
		// lParam = Y (distance from robot) in TenthInches

		//long ArmXYZParam1 = (DWORD)MAKELONG((int)(ArmXYZ.Z*10.0), (int)(ArmXYZ.X*10.0));	// LoWord, HiWord in inches * 10 (to allow for fractions)
		//long ArmXYZParam2 = (int)(ArmXYZ.Y*10.0);								// in inches * 10 (to allow for fractions)
		//SendCommand( WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD, ArmXYZParam1, ArmXYZParam2 );

		
		// See if we are ready to do the next step
		if( 0 != gArmTimerRight)
		{
			ROBOT_LOG( DEBUG_ARM_MOVEMENT, "WAITING FOR ARM TIMER RIGHT = %d\n", gArmTimerRight)
		}

		if( (0 == gArmTimerRight) &&
			((!m_ArmWaitForMoveToCompleteRight) || (m_pArmControlRight->CheckArmPosition(DEBUG_ARM_MOVEMENT))) )	//	TRUE = Verbose
		{
			// Last movement completed.  Do the next movement.
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			// RIGHT ARM ADDITIONAL MOVEMENTS
			switch( m_ArmMovementRight )  // Arm Movement in progress
			{
				case ARM_MOVEMENT_NONE:
				case ARM_MOVEMENT_HOME1:				// Home
				case ARM_MOVEMENT_EXTEND_FULL:		// Extend Arm Fully
				case ARM_MOVEMENT_OPEN_CLAW:		// Open Claw
				case ARM_MOVEMENT_GRAB_COKE:		// Grab Coke
				case ARM_MOVEMENT_CLOSE_CLAW_FULL:	// Close Claw Full
				case ARM_MOVEMENT_LIFT_OBJECT:		// Lift Object
				{
					m_ArmMovementRight = ARM_MOVEMENT_NONE;
					break;	// Nothing to do
				}

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_HOME2: // Arm locked Up position
				{	
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// Arm has moved into locked position, now remove any load on the servo to save battery and servo wear
							// Just get current position and set desired to that.
							int Shoulder, ElbowRotate, ElbowBend, Wrist, Claw;
							m_pArmControlRight->GetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
							m_pArmControlRight->SetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight = 0;	// back to Idle state
							m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteRight = TRUE;
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_HOME2

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_ARM_UP_FULL: // Raise your right hand...
				{	
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							m_pArmControlRight->SetArmPosition( RIGHT_ARM_SHOULDER_ARM_STRAIGHT_UP, NOP, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;
							m_ArmWaitForMoveToCompleteRight = TRUE;
							break;
						}
						case 2:
						{
							m_pArmControlRight->SetArmPosition( NOP, -90, NOP, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();

							m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
							m_pArmControlRight->ExecutePositionAndSpeed();

							m_ArmMovementStateRight = 0;	// back to Idle state
							m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteRight = TRUE;
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_ARM_UP_FULL

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SCRATCH_BACK: // Scratch Back
				{	
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							m_pArmControlRight->SetArmPosition( 225, -60, 125, 140, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 20;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2:
						{
							m_pArmControlRight->SetArmPosition( 205, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							////gArmTimerRight = 20;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlRight->SetArmPosition( 225, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:
						{
							m_pArmControlRight->SetArmPosition( 205, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlRight->SetArmPosition( 225, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6:
						{
							m_pArmControlRight->SetArmPosition( 205, -40, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 7:
						{
							RightArmHome();	// Home
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_SCRATCH_BACK

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_WAVE: // Wave
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Wave 1
						{
							// Respond with random phrases
							int RandomNumber = ((6 * rand()) / RAND_MAX);
							ROBOT_LOG( TRUE,"DEBUG: RAND = %d\n", RandomNumber)
							switch( RandomNumber )
							{
								case 0:  SpeakText( "Hello" );break;
								case 1:  SpeakText( "Hi there!" );break;
								case 2:  SpeakText( "Hey there");break;
								case 3:  SpeakText( "Hi" );break;									
								default: SpeakText( "Hello!" ); // If const is larger number, this gets called more often
							}
		

							m_pArmControlRight->SetArmPosition( NOP, -5, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2:
						{
							m_pArmControlRight->SetArmPosition( NOP, 5, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3:	// Wave 2
						{
							m_pArmControlRight->SetArmPosition( NOP, -5, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:
						{
							m_pArmControlRight->SetArmPosition( NOP, 5, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5:
						{
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT STATE RIGHT(%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_WAVE:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_THROW_OBJECT_FRONT: // Throw Object
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// Set speed of all servos (used by each when moved)
	
							// Tighten Grip on object
							m_ArmWaitForMoveToCompleteRight = FALSE;	// Don't wait for each movement!
							gArmTimerRight = 5;	// 1/10 second per count!
							m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
							int NewClawPosition = m_pArmControlRight->GetClawPosition() - 10;	// n degrees "tighter"
							m_pArmControlRight->SetArmPosition( NOP, NOP, NOP, NOP, NewClawPosition ); // Set Claw position
							m_ArmMovementStateRight++;	// go to next state
							m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX );
							m_pArmControlRight->ExecutePositionAndSpeed();
							break;
						}

/**** NEW ***/
						case 2: // snap the elbow, ////NEW!!!!  and open the hand
						{								
							m_ArmWaitForMoveToCompleteRight = FALSE;	// Don't wait for each movement!
							gArmTimerRight = 30;	// 1/10 second per count!
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							m_pArmControlRight->SetArmDelay( NOP, NOP, NOP, NOP, 100 );	// Delayed Claw opening (MS)

							m_pArmControlRight->SetArmPosition( NOP, NOP, 120, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlRight->SetArmPosition( NOP, NOP, 120, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//m_pArmControlRight->SetArmPosition( NOP, NOP, NOP, NOP, 120 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlRight->ExecutePosition();
							break;
						}
						case 3:
						{
							//Open the hand
							m_pArmControlRight->SetArmPosition( NOP, NOP, NOP, NOP, 120 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							m_ArmWaitForMoveToCompleteRight = TRUE;	// wait for all to complete before moving to Home
							m_pArmControlRight->ExecutePosition();
							break;
						}
						case 4:
						{
							// Home
							m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
							m_ArmWaitForMoveToCompleteRight = TRUE;	// Reset to default
							// Speed is automatically set back to normal
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT STATE RIGHT(%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_THROW_OBJECT_FRONT:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PUT_IN_BASKET: // Throw Object
				{
					// NOT IMPLEMENTED FOR RIGHT ARM
					RightArmHome();	// Home
					break;
				}	// case ARM_MOVEMENT_PUT_IN_BASKET:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SHAKE_READY: // Shake Hands
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Wait for Claw sensor to detect something
								// (will usually note person's hand )
						{
							if( g_pNavSensorSummary->nObjectClawRight < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES )
							{
								// Object detected.  
								int RandomNumber = ((6 * rand()) / RAND_MAX);
								ROBOT_LOG( TRUE,"DEBUG: RAND = %d\n", RandomNumber)
								RandomNumber = 2;
								switch( RandomNumber )
								{
									case 0:  SpeakText( "It is nice to meet you" );break;
									case 1:  SpeakText( "Live long, and prosper" );break;
									case 2:  SpeakText( "How are you?" );break;
									default:  SpeakText( "I am pleased to meet you" );break;

									/*
									case 0:  SpeakText( "Nice to meet you" );break;
									case 1:  SpeakText( "Live long, and prosper" );break;
									case 2:  SpeakText( "You are, and always shall be, my friend." );break;
									case 3:  SpeakText( "How are you?" );break;
									default:  SpeakText( "I am pleased to meet you" );break;
									*/
								}
			

								// Shake uses TIMER instead of watching servo position, so 
								// if user holding on too tight, motion does not stop
								m_ArmWaitForMoveToCompleteRight = FALSE;
								dbgTimeR = GetTickCount();
								gArmTimerRight = 0;	// 1/10 second per count!
								m_ArmMovementStateRight++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}
						case 2: // Start shake
						{

							m_pArmControlRight->SetArmPosition(
								RIGHT_ARM_SHOULDER_ARM_SHAKE_DOWN, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();

							gArmTimerRight = 3;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3: // Shake 1
						{
							ROBOT_LOG( TRUE,"DEBUG: Shake 2 time = %d\n", (GetTickCount() - dbgTimeR))
							m_pArmControlRight->SetArmPosition(
								RIGHT_ARM_SHOULDER_ARM_SHAKE_UP, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_UP,
								RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 4;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:
						{
							ROBOT_LOG( TRUE,"DEBUG: Shake 3 time = %d\n", GetTickCount() - dbgTimeR)
							m_pArmControlRight->SetArmPosition(
								RIGHT_ARM_SHOULDER_ARM_SHAKE_DOWN, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 4;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5:	// Shake 2
						{
							ROBOT_LOG( TRUE,"DEBUG: Shake 4 time = %d\n", GetTickCount() - dbgTimeR)
							m_pArmControlRight->SetArmPosition(
								RIGHT_ARM_SHOULDER_ARM_SHAKE_UP, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_UP,
								RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 4;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6:
						{
							ROBOT_LOG( TRUE,"DEBUG: Shake 5 time = %d\n", GetTickCount() - dbgTimeR)
							m_pArmControlRight->SetArmPosition(
								RIGHT_ARM_SHOULDER_ARM_SHAKE_DOWN, RIGHT_ARM_ELBOW_ROTATE_ARM_SHAKE, RIGHT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								RIGHT_ARM_WRIST_ROTATE_ARM_SHAKE, RIGHT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 10;	// PAUSE so user will let go
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 7:
						{
							ROBOT_LOG( TRUE,"DEBUG: Shake Done time = %d\n", GetTickCount() - dbgTimeR)
							m_ArmWaitForMoveToCompleteRight = TRUE; // back to normal
							RightArmHome();	// Home
							// Tell any waiting behavior that we are done with this task
							m_SubTaskComplete = TRUE;
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_SHAKE_MOVE:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW: // Extending arm to take an object
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
/*** TODO - Need to fix claw sensor
						case 1:
						{
							if( 0 == ARM_R_IR_BUMPER_INSIDE_CLAW )
							{
								// Got good sensor reading (sometimes this sensor is flaky)
								// Go to next state and wait for object to be detected
								ROBOT_LOG( TRUE,"Extend Arm: Good Sensor reading.  Waiting for object\n")
								m_ArmMovementStateRight=2;	// wait for object detection
							}
							else
								//ROBOT_LOG( TRUE,"Extend Arm: Bad Sensor reading!  Switching to Plan B\n")
								m_ArmMovementStateRight = 3;	// Use back up algorythm
							}
							break;
						}

						case 2: // Waiting for object detected inside claw
						{
							if( ARM_R_IR_BUMPER_INSIDE_CLAW )
							{
								// Object detected.  Close claw.
								ROBOT_LOG( TRUE,"Extend Arm: Object detected. Closing Claw\n")
								m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
								m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlRight->ExecutePositionAndSpeed();
								m_ArmMovementStateRight = 4;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}
***/
						case 1: // Wait for Claw sensors to detect something
								// (will usually note person's hand as they provide the object)
						{
							if( g_pNavSensorSummary->nObjectClawRight < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES  )	// 
							{
								// Object detected.  Close claw.
								ROBOT_LOG( TRUE,"Extend Arm: Hand Object detected. Closing Claw\n")
								m_pArmControlRight->SetClawTorque(STRONG_TORQUE);
								m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlRight->ExecutePositionAndSpeed();
								m_ArmMovementStateRight++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}

						case 2:	// Claw closed.  Now examine the object
						{
							/*** This does not work, can't handle a CD!
							// Check to make sure there is something in the claw
							if( g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].PositionTenthDegrees < 0 )
							{
								// Looks like we did not get any object
								// zero is loosly closed claw.  Tightly closed is up to -10
								m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlRight->ExecutePositionAndSpeed();
								SpeakText( "I did not get anything.  Please try again" );	
			
								m_ArmMovementStateRight = 1; // Try Again
							}
							else  ****/
							//{
								// Examine the Object
								SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_IDENTIFY_OBJECT );	// Right/Left arm, Movement to execute, 
								m_ArmMovementStateRight = 0;	// back to Idle state
								m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
								ROBOT_LOG( TRUE,"Extend Arm: Sending Examine Object command\n")
						//	}
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW: // Extending arm to give an object
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Wait for claw sensor to detect something
								// (will usually note person's hand as they take the object)
						{
							if( g_pNavSensorSummary->nObjectClawRight < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES  )	// 
							{
								// Object detected.  Open claw.
								ROBOT_LOG( TRUE,"Extend Arm: Hand Object detected. Opening Claw\n")
								m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, RIGHT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlRight->ExecutePositionAndSpeed();
								m_ArmMovementStateRight++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}

						case 2:	// Claw Opened.  return to home position
						{
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_LOOK_AT_HAND:
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// m_pArmControlRight->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->SetArmPosition(NOP, -20, 120, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();

							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+160, CAMERA_TILT_CENTER-140, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							//gArmTimerRight = 20;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state

							break;
						}
						case 3:
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, -60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:	// 
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 10;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6:
						{
							// Home
							HeadCenter(); // Look Forward
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_LOOK_AT_HAND:


				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SCRATCH_HEAD:
				{
					const int DelayTime = 6;
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, CAMERA_TILT_CENTER, -9 );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

							m_pArmControlRight->SetArmPosition(130, -14, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();


							int RandomNumber = ((2 * rand()) / RAND_MAX);
							RandomNumber = 0;
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "6.9 billion, divide by 164 million, divided by the cubed root of a rudabega, equals 42 " );break;
								//case 1:  SpeakText( "6.9 billion, multiply by 84.6, carry the 1, and divide by the weight of an electron " );break;
								default:  SpeakText( "if I take the speed of light, and divided by the cubed root of a rudabega" );break;
							}
		


							gArmTimerRight = DelayTime;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlRight->SetArmPosition(120, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = DelayTime;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlRight->SetArmPosition(130, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = DelayTime;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4: // 
						{
							m_pArmControlRight->SetArmPosition(120, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = DelayTime;	// 1/10 second per count!
							if( --m_RepeatCount > 0 )
							{
								m_ArmMovementStateRight = 3; // scratch again
							}
							else
							{
								m_ArmMovementStateRight++;	// go to next state
							}
							break;
						}
						case 5:
						{
							// Home
							HeadCenter(); // Look Forward
							m_pArmControlRight->MoveArmHome( m_ArmSpeedRight );	// Arm Home
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6:
						{
							SpeakText( "The meaning of life is 42. If you dont get this joke, you should read the hichhikers guide to the galaxy" );
							m_ArmMovementStateRight = 0;
							RightArmHome();	// Home, and done with task
							break;
						}

						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_SCRATCH_HEAD:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_KARATE:
				{
					ROBOT_LOG( TRUE,"DEBUG KARATE: m_ArmMovementStateRight = %d\n", m_ArmMovementStateRight)
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							HeadCenter();
							// m_pArmControlRight->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->SetArmPosition(105, -13, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlRight->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlRight->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:	// 
						{
							m_pArmControlRight->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlRight->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6:
						{
							// Home
							HeadCenter(); // Look Forward
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_SCRATCH_HEAD:


				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PICKUP_OBJECT_XYZ:
				{
					ROBOT_ASSERT(0); // NOT IMPLEMENTED!
					if( m_ArmMovementStateRight > 1 ) 
					{
						gArmTimerRight = 2;	// Give time to settle - 1/10 second per count!

						if( 0 != g_pFullSensorStatus->ArmRightBumperElbow )
						{
							m_ObjectDetectCount++;
							ROBOT_LOG( TRUE,"Find Object: Object Detected: %02X  m_ObjectDetectCount = %d\n", 
								g_RawArduinoStatus.ArmBumperR, m_ObjectDetectCount )
							m_ArmMovementStateRight--;	// Stay at prior state
							//gArmTimerRight = 10;	// Give time to settle - 1/10 second per count!

							if( m_ObjectDetectCount >= 5 )
							{
								// Object detected
								m_ArmMovementStateRight++; // = 13; //Next Phase in the search
								//m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
								m_ObjectDetectCount = 0;
							}
						}
						else
						{
							m_ObjectDetectCount = 0;	// reset counter
						}
						ROBOT_LOG( TRUE,"DEBUG FIND OBJ: %d\n",m_ArmMovementStateRight)
						//gArmTimerRight = 20;	// 1/10 second per count!

					}

					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Move arm down into position
						{
							m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_VERY_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MED );
							m_pArmControlRight->SetArmPosition(-40, 3, 85, 0, 40 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePositionAndSpeed();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-35, NOP, 90, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							gArmTimerRight = 5;
							break;
						}
						case 3: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-30, NOP, 90, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-25, NOP, 80, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-20, NOP, 70, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-15, NOP, 65, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}

						case 7: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-10, NOP, 65, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 8: // Move forward
						{
							m_pArmControlRight->SetArmPosition(-5, NOP, 60, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 9: // Move forward
						{
							m_pArmControlRight->SetArmPosition(0, NOP, 55, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 10: // Move forward
						{
							m_pArmControlRight->SetArmPosition(5, NOP, 50, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 11: // Move forward
						{
							m_pArmControlRight->SetArmPosition(10, NOP, 45, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 12: // Move forward
						{
							m_pArmControlRight->SetArmPosition(15, NOP, 40, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 13: // Move forward
						{
							m_pArmControlRight->SetArmPosition(20, NOP, 35, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight++;	// go to next state
							break;
						}								
						
						case 14:  // Home - Object not found
						{
							gArmTimerRight = 0;
							RightArmHome();
							break;
						}


						case 15: // zero in on object
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, 50 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							m_ArmMovementStateRight = 16;	// go to next state
							break;
						}								
						case 16:  // Home
						{
							gArmTimerRight = 0;
							RightArmHome();
							break;
						}

						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_PICKUP_OBJECT_XYZ:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PICKUP_OBJECT_BLIND:
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Move arm down into position
						{
							//m_pArmControlRight->SetArmPosition(-12, -4, 44, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->SetArmPosition(-22, -2, 65, -90, 90 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlRight->SetArmPosition(-28, 3, 61, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2: // Move arm down into position 2
						{
							m_pArmControlRight->SetArmPosition(-18, -2, 50, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3: // Grab Object
						{
							m_pArmControlRight->SetArmPosition(-20, NOP, 54, NOP, RIGHT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4:  // Determine if Object found, and if so, bend elbow
						{
							if( m_pArmControlRight->GetClawPosition() < RIGHT_ARM_CLAW_CLOSED_LOOSE )
							{
								// No object grabbed
								m_ArmMovementStateRight = 7; // handle error case
							}
							else
							{
								m_pArmControlRight->SetArmPosition(NOP, 3, 80, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlRight->ExecutePosition();
								//gArmTimerRight = 5;	// 1/10 second per count!
								m_ArmMovementStateRight++;	// go to next state
							}
							break;
						}
						case 5:	// Lift object in front of face
						{
							m_pArmControlRight->SetArmPosition(60, NOP, 120, -60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlRight->SetArmPosition(80, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+400, CAMERA_TILT_CENTER-300, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 6: // Look at it
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+300, CAMERA_TILT_CENTER-60, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateRight++;	// go to next state
							int RandomNumber = ((5 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "This is very interesting" );break;
								case 1:  SpeakText( "Whatever" );break;
								case 2:  SpeakText( "Nice" );break;
								case 3:  SpeakText( "I like this" );break;
								default:  SpeakText( "Ok" );break;
							}
		
							break;
						}
						case 7:	// Now what?
						{
							m_pArmControlRight->SetArmPosition(NOP, 0, 80, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							HeadCenter(); // Look Forward
							int RandomNumber = ((5 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "What should I do with it?" );break;
								case 1:  SpeakText( "Now what?" );break;
								case 2:  SpeakText( "Here you go" );break;
								case 3:  SpeakText( "What's Next?" );break;
								default:  SpeakText( "What is your next command, oh great one?" );break;
							}
		
							m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmMovementStateRight = 0;	// back to Idle state
							// Give object to user
							SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)RIGHT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW );	// Right/Left arm, Movement to execute, 
							break;
						}
						case 8:	// Did not pick anything up!
						{
							HeadCenter(); // Look Forward
							int RandomNumber = ((2 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "My Eye Hand coordination is not very good yet" );break;
								default:  SpeakText( "Darn, I did not get it" );break;
							}
		
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_PICKUP_OBJECT_BLIND:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PUT_DOWN_OBJECT:
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Set object down
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, 60, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 2:	// Drop it
						{
							m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 3:	// Move Hand so we don't bump it
						{
							m_pArmControlRight->SetClawTorque(STRONG_TORQUE); // reset torque
							m_pArmControlRight->SetArmPosition(-25, NOP, 80, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 4: // Home
						{
							HeadCenter(); // Look Forward
							RightArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_PUT_DOWN_OBJECT:


				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_IDENTIFY_OBJECT:
				{
					switch( m_ArmMovementStateRight )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Lift object in front of face
						{
							m_pArmControlRight->SetArmPosition(42, -12, 120, 70, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							// Point camera at arm, and zoom in on object
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+216, CAMERA_TILT_CENTER-190, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateRight++;	// go to next state
							gArmTimerRight = 20;	// 1/10 second per count!
							break;
						}
						case 2: // Look at it in position #1
						{
							// Send command to vision system to identify the object.
							// NOTE!  The response to this will go to a different processing path (see WM_ROBOT_CAMERA_MATCH_COMPLETE)
							// and then return to the next case statement when done...
/*									int RandomNumber = ((6 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "I am checking my Data Base" );break;
								case 1:  SpeakText( "I am thinking" );break;
								case 2:  SpeakText( "I wonder what this is" );break;
								case 3:  SpeakText( "Working" );break;
								default:  SpeakText( "Lets see if I recognize this" );break;
							}
		
*/
							// Make sure Vision System is enabled (Hangs up if we don't do this check!)
							if( CAMERA_STATE_NOT_ENABLED == g_Camera[LEFT_CAMERA].State )
							{
								m_ArmMovementStateRight = 6;	// Got to end processing
								SpeakText( "My vision processing is disabled" );	
			
							}
							else
							{
								// Give time for the zoom to settle before grabbing the frame to match
								//Sleep(2000);
								gArmTimerLeft = 200;	// 1/10 second per count!
								SendCommand( WM_ROBOT_CAMERA_MATCH_OBJECT, 0, 0 );
								m_ArmMovementStateRight++;	// go to next state
							}
							break;
						}
						case 3: // waiting for vision to complete
						{
							// Nothing to do.  State machine must be moved in the vision processing case statement
						//	ROBOT_LOG( TRUE,"Behavior Module: ARM_MOVEMENT_IDENTIFY_OBJECT:3: Servo update ignored! Waiting for vision system")
							break;
						}
						case 4: // Waiting for wrist to complete rotating 90 degrees, so we can look at object again
						{
							// Ok, done rotating now.   Look at it in position #2
							// Send command to vision system to identify the object.
							// NOTE!  The response to this will go to a different processing path
							// and then return to the next case statement when done...

							SendCommand( WM_ROBOT_CAMERA_MATCH_OBJECT, 0, 0 );
							//SpeakText( "Hmmm" );	
							m_ArmMovementStateRight++;	// go to next state
							break;
						}
						case 5: // waiting for vision to complete
						{
							// Nothing to do.  State machine must be moved in the vision processing case statement
							ROBOT_LOG( TRUE,"Behavior Module: ARM_MOVEMENT_IDENTIFY_OBJECT:5: Servo update ignored! Waiting for vision system")
							break;
						}
						case 6:	// Now what?
						{
							m_pArmControlRight->SetArmPosition(NOP, 0, 80, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlRight->ExecutePosition();
							//gArmTimerRight = 5;	// 1/10 second per count!
							HeadCenter(); // Look Forward
							m_ArmMovementStateRight = 0;	// back to Idle state
							m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
							SpeakText( "What should I do with it?" );	
		
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateRight)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateRight )
					break;
				}	// case ARM_MOVEMENT_IDENTIFY_OBJECT:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_TURN_DOOR_HANDLE: // 
				{	
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// Get into position, but don't hit claw on the door!
							m_pArmControlLeft->SetArmPosition( 100, 0, 120, 90, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 2:
						{
							// OK, now move claw closer to the door, and get into position to start looking for a handle with sensors
							m_NextShoulderPosition = 100;
							m_pArmControlLeft->SetArmPosition( m_NextShoulderPosition, 0, 0, 90, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 3:
						{
							// Get distance to door as read by hand sensor.  This will be used as a threshold to detect the handle
							//m_SensorThreshold = (ARM_R_IR_SENSOR_CLAW?? or g_pNavSensorSummary->nObjectClawLeft??) - 1; // Distance to door - 1 inch fudge factor for variance
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 4:
						{
							// Was the handle found by the hand sensor?
							int SensorThreshold = 40; // Tenth inches
							if(  ARM_R_IR_SENSOR_CLAW  <= SensorThreshold ) // TODO!! Check load on elbow - maybe detect that way??
							{
								// Handle found!  Go to next state
								m_ArmMovementStateLeft++;
							}
							else
							{
								// Not found yet.  move the claw vertically, looking for the handle.  
								// Stay in this state until handle found or limit reached
								if(m_NextShoulderPosition <= 90)
								{
									// Error, no door handle found!
									ROBOT_DISPLAY( TRUE, "========================" )
									ROBOT_DISPLAY( TRUE, "DOOR HANDLE NOT FOUND!!!" )
									m_ArmMovementStateLeft = 0;	// back to Idle state
									m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
									m_ArmWaitForMoveToCompleteLeft = TRUE;
									m_TurnHandleState = TURN_HANDLE_STATE_FAIL;
								}
								else
								{
									// still looking for door handle
									m_NextShoulderPosition -= 2; // degrees!
									m_pArmControlLeft->SetArmPosition( m_NextShoulderPosition, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
									m_pArmControlLeft->ExecutePosition();
									// STAY IN SAME STATE m_ArmMovementStateLeft++;
									m_ArmWaitForMoveToCompleteLeft = TRUE;
								}
							}
							break;
						}
						case 5:
						{
							// Handle Found!  Close claw on handle
							m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, NOP, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 6:
						{
							// Claw closed, now rotat handle (combined write and elbow rotations?
							m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, -30, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 7:
						{
							// Knob turned. Ready to open door
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							m_TurnHandleState = TURN_HANDLE_STATE_SUCCESS;
						}
						case 8:
						{
							// Done
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_TURN_DOOR_HANDLE


				////////////////////////////////////////////////////////////////////////////////////////////////////
				default:
				{
					ROBOT_LOG( TRUE,"BehaviorModule: ERROR - Unhandled Arm Movement Right (%02X)!\n", m_ArmMovementRight)
				}
			} // switch m_ArmMovementRight

		} //  if ( 0 != gArmTimerRight )
	}	// if( ARM_MOVEMENT_NONE != m_ArmMovementRight )

__itt_task_end(pDomainControlThread);

	#endif //ROBOT_HAS_RIGHT_ARM

}	// HandleServoStatusUpdateRight




#endif // ROBOT_SERVER
