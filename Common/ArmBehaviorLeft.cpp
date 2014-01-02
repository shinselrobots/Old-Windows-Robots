// ArmBehavior.cpp: Subset of Behavior Module for Arm Control
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
#define ARM_CLEAR_BODY_TIME_DELAY  8 // 1/10 second per count! - Delay before bending arm to allow elbow to clear the body in the movement

//#define OBJECT_DETECT_BOTH_FINGERS	0xC0
//#define OBJECT_DETECT_LEFT_FINGER	0x80
//#define OBJECT_DETECT_RIGHT_FINGER	0x40

#define PICKUP_OBJECT_MAX_Y			80	// TenthInches - from front of robot
#define PICKUP_OBJECT_MAX_X			150	// TenthInches - from center of robot
#define PICKUP_OBJECT_DEFAULT_Z		15	// TenchInches - assume objects are best gripped at around an inch from the floor

__itt_string_handle* pshHandleArmMovementRequestLeft = __itt_string_handle_create("HandleArmMovementRequestLeft");
__itt_string_handle* pshInitPickupObjectXYZLeft = __itt_string_handle_create("InitPickupObjectXYZLeft");
__itt_string_handle* pshInitPutDownObject = __itt_string_handle_create("InitPutDownObject");
__itt_string_handle* pshInitOpenClaw = __itt_string_handle_create("InitOpenClaw");
__itt_string_handle* pshInitThrowObjectBack = __itt_string_handle_create("InitThrowObjectBack");
__itt_string_handle* pshHandleArmServoStatusUpdateLeft = __itt_string_handle_create("HandleArmServoStatusUpdateLeft");
__itt_string_handle* pshSomethingToDo = __itt_string_handle_create("SomethingToDo");
__itt_string_handle* pshLookAtHand = __itt_string_handle_create("LookAtHand");
__itt_string_handle* pshPickupObjectXYZLeft = __itt_string_handle_create("PickupObjectXYZLeft");
__itt_string_handle* pshFirstStep = __itt_string_handle_create("FirstStep");
__itt_string_handle* pshMoveToReadyPosition = __itt_string_handle_create("MoveToReadyPosition");
__itt_string_handle* pshGrabObject = __itt_string_handle_create("GrabObject");
__itt_string_handle* pshLiftObject = __itt_string_handle_create("LiftObject");
__itt_string_handle* pshPutDownObjectLeft = __itt_string_handle_create("PutDownObjectLeft");
__itt_string_handle* pshIdentifyObject = __itt_string_handle_create("IdentifyObject");

// Markers
__itt_string_handle* pshWaitingForArmLeft = __itt_string_handle_create("WaitingForArmLeft");
__itt_string_handle* pshWaitingForArmTimerLeft = __itt_string_handle_create("WaitingForArmTimerLeft");
__itt_string_handle* pshNothingToDo = __itt_string_handle_create("NothingToDo");
__itt_string_handle* pshObjectOutOfRange = __itt_string_handle_create("ObjectOutOfRange");
__itt_string_handle* pshAbortNoSolution = __itt_string_handle_create("AbortNoSolution");
__itt_string_handle* pshAbortNoObjectInClaw = __itt_string_handle_create("AbortNoObjectInClaw");
__itt_string_handle* pshSucessPickUpComplete = __itt_string_handle_create("SucessPickUpComplete");
__itt_string_handle* pshFinalNoObjectInClaw = __itt_string_handle_create("FinalNoObjectInClaw");

// examples:
//	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh);
//	__itt_task_end(pDomainControlThread);


DWORD dbgTimeL = GetTickCount();

//***************************************************************************************************************************************
// HandleArmMovementRequest - LEFT ARM INITIAL MOVEMENT
// Process Arm Behavior commands and do inital movemment for arms (move into first position)
//***************************************************************************************************************************************
void CBehaviorModule::HandleArmMovementRequestLeft( WPARAM wParam, LPARAM lParam )
{
		__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmMovementRequestLeft);
		m_ArmMovementLeft = lParam;
		// by default, each movement should complete before the next begins
		// Can be overridden below, and gArmTimerLeft used instead
		m_ArmWaitForMoveToCompleteLeft = TRUE;	
		// On each arm move command, reset values to default speed (as specified by GUI)
		m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	

		// Disable random arm movements during most operations
		if( (ARM_MOVEMENT_NONE != m_ArmMovementLeft) &&
			(ARM_MOVEMENT_HOME1 != m_ArmMovementLeft) )
		{
			m_pArmControlLeft->EnableIdleArmMovement(FALSE);
		}


		switch( m_ArmMovementLeft )  // lParam is the movement requested
		{
			case ARM_MOVEMENT_NONE:
			{
				ROBOT_LOG( TRUE,"BehaviorModule: ACTION_MODE_NONE requested\n")
				break;
			}
			case ARM_MOVEMENT_HOME1:	// Home with arm down (sensors in position to move)
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME1\n")
				m_pArmControlLeft->MoveArmHome();
				break;
			}
			case ARM_MOVEMENT_HOME2:	// Home with arm up
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME2\n")
				m_pArmControlLeft->SetArmPosition(
					LEFT_ARM_SHOULDER_HOME2, LEFT_ARM_ELBOW_ROTATE_HOME2, LEFT_ARM_ELBOW_BEND_HOME2, 
					LEFT_ARM_WRIST_ROTATE_HOME2, LEFT_ARM_CLAW_HOME2 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				m_ArmMovementStateLeft= 1;	// go to first state
				m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW: // Extend Arm
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW\n")
				// Added this - look at hand... is this correct?
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
				gArmTimerLeft = ARM_CLEAR_BODY_TIME_DELAY;	// 1/10 second per count!
				m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, 90 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW: // Extend Arm
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW\n")
				// Move head toward hand 
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
				gArmTimerLeft = ARM_CLEAR_BODY_TIME_DELAY;	// 1/10 second per count!
				m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, LEFT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}

			case ARM_MOVEMENT_TAKE_OBJECT:	
			{
				// Move head to look at hand 
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER-150, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement! (just need to get arm past body before bending elbow)
				gArmTimerLeft = ARM_CLEAR_BODY_TIME_DELAY;	// 1/10 second per count!

				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_TAKE_OBJECT\n")
				// Same as ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW
				m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, 90 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				m_ArmMovementStateLeft = 1;	// go to first state
				m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW;	// Tell the Left arm state machine to continue this movement

				// Respond with random phrases
				int RandomNumber = ((4 * rand()) / RAND_MAX);
				RandomNumber = 2;
				switch( RandomNumber ) // Respond with random phrases
				{
					case 0:  SpeakText( "OK" );break;
					case 1:  SpeakText( "Great!" );break;
					default: SpeakText( "what do you have?" );break;
				}
				break;
			}

			case ARM_MOVEMENT_GIVE_OBJECT:	
			{
				// Move head to look at hand 
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER-150, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement! (just need to get arm past body before bending elbow)
				gArmTimerLeft = ARM_CLEAR_BODY_TIME_DELAY;	// 1/10 second per count!

				// Determine if claw should be open or closed, based upon claw torque load
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
				{
					// Claw is holding something!
					// Same as ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_GIVE_OBJECT (LOAD DETECTED)\n")
					m_pArmControlLeft->EnableIdleArmMovement(FALSE);
					m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
					m_ArmMovementStateLeft = 1;	// go to first state
					m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW;	// Tell the Left arm state machine to continue this movement

					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					RandomNumber = 1;
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						default: SpeakText( "Here you go" );break;
					}

					//gArmTimerLeft = 10;	// 1/10 second per count!
				}
				else
				{
					// Not holding anything
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_GIVE_OBJECT (NO LOAD)\n")

					// Respond with random phrases
					int RandomNumber = ((4 * rand()) / RAND_MAX);
					RandomNumber = 2;
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "Give what to you?  I dont have anyting" );break;
						default: SpeakText( "I dont have anything to give you" );break;
					}
				}

				break;
			}

			case ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW:	
			{
				// Move head to look at hand 
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER-150, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement! (just need to get arm past body before bending elbow)
				gArmTimerLeft = ARM_CLEAR_BODY_TIME_DELAY;	// 1/10 second per count!

				// Determine if claw should be open or closed, based upon claw torque load
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
				{
					// Claw is holding something!
					// Same as ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW (LOAD DETECTED)\n")
					m_pArmControlLeft->EnableIdleArmMovement(FALSE);
					m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
					m_ArmMovementStateLeft = 1;	// go to first state
					m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW;	// Tell the Left arm state machine to continue this movement

					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					RandomNumber = 1;
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						default: SpeakText( "Here you go" );break;
					}

					//gArmTimerLeft = 10;	// 1/10 second per count!
				}
				else
				{
					// Not holding anything
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW (NO LOAD)\n")
					// Same as ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW
					m_pArmControlLeft->SetArmPosition( 60, LEFT_ARM_ELBOW_ROTATE_HOME1, 80,	5, 90 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
					m_ArmMovementStateLeft = 1;	// go to first state
					m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW;	// Tell the Left arm state machine to continue this movement

					// Respond with random phrases
					int RandomNumber = ((4 * rand()) / RAND_MAX);
					RandomNumber = 2;
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						case 1:  SpeakText( "Ready" );break;
						case 2:  SpeakText( "what do you have?" );break;
						default: SpeakText( "For me?" );break;
					}

				}

				break;
			}
			case ARM_MOVEMENT_EXTEND_FULL: // Extend Arm Fully
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_FULL\n")
				m_pArmControlLeft->SetArmPosition(95, 0, 5, 0, 1 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_SHAKE_READY: // Extend Arm to shake hands
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SHAKE_READY\n")
				m_pArmControlLeft->EnableIdleArmMovement(FALSE);
				m_pArmControlLeft->SetArmPosition(
					LEFT_ARM_SHOULDER_ARM_SHAKE, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE,
					LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				m_ArmMovementStateLeft = 1;	// nothing to do
				break;
			}
			case ARM_MOVEMENT_LOOK_AT_HAND: // Look at whatever might be in the left hand
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_LOOK_AT_HAND\n")
				m_pArmControlLeft->SetArmPosition(50, 0, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_VERY_SLOW, SERVO_SPEED_VERY_SLOW, SERVO_SPEED_VERY_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-80, CAMERA_TILT_CENTER-40, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				break;
			}
			case ARM_MOVEMENT_SCRATCH_HEAD: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SCRATCH_HEAD\n")
				m_pArmControlLeft->SetArmPosition(110, 0, 130, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				gHeadIdle = FALSE;
				HeadCenter(); // Look Forward
				break;
			}
			case ARM_MOVEMENT_KARATE: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_KARATE\n")
				m_pArmControlLeft->SetArmPosition(110, 0, 130, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				gHeadIdle = FALSE;
				HeadCenter(); // Look Forward
				break;
			}
			case ARM_MOVEMENT_PICKUP_OBJECT_BLIND: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PICKUP_OBJECT_BLIND\n")
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
				m_ArmWaitForMoveToCompleteLeft = TRUE;
				m_pArmControlLeft->SetArmPosition(-45, 15, 100, 0, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}

			case ARM_MOVEMENT_PICKUP_OBJECT_XYZ: // 
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshInitPickupObjectXYZLeft);
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: Initial Left ARM_MOVEMENT_PICKUP_OBJECT_XYZ\n")
				m_ObjectDetectCount = 0;
				// Move into starting position
				ROBOT_LOG( TRUE,"Moving Arm into Ready position, Claw Open\n")
			//	m_pArmControlLeft->SetClawTorque(STRONG_TORQUE); // get better grip
//				m_pArmControlLeft->SetArmPosition(-20, -3, 60, 0, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip

				m_pArmControlLeft->SetArmPosition(
					LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, LEFT_ARM_ELBOW_BEND_HOME1, 
					LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_OPEN_MAX ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip


				m_pArmControlLeft->ExecutePosition();	
				//gArmTimerLeft = 30;	// 1/10 second per count!
				gHeadIdle = FALSE;
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-500, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
				m_ArmMovementStateLeft = 1;	// go to first state
				__itt_task_end(pDomainControlThread);
				break;
			}

			case ARM_MOVEMENT_PUT_DOWN_OBJECT: // 
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshInitPutDownObject);
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_DOWN_OBJECT\n")
				// Look at the object with head
				m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
				m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
				m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );

				m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE); // relax the servvo a bit
				m_pArmControlLeft->SetArmPosition( 3, -1, 60, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip (TODO-DWS: is this location good for Kinect to find?)
//				m_pArmControlLeft->SetArmPosition(-20, -10, 68, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 5;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				__itt_task_end(pDomainControlThread);
				break;
			}
			case ARM_MOVEMENT_OPEN_CLAW: // Open Claw
			{
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshInitOpenClaw);
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_OPEN_CLAW\n")
				m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
				m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 0, LEFT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				__itt_task_end(pDomainControlThread);
				break;
			}
			case ARM_MOVEMENT_GRAB_COKE: // Grab Coke
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_GRAB_COKE\n")
				m_pArmControlLeft->SetClawTorque(500);
				m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, 20 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_CLOSE_CLAW_FULL: // Close Claw Full
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_CLOSE_CLAW_FULL\n")
				m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);
				m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_LIFT_OBJECT: // Lift Object
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_LIFT_OBJECT\n")
				m_pArmControlLeft->SetArmPosition(NOP, NOP, 70, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				break;
			}
			case ARM_MOVEMENT_ARM_UP_FULL: // Full Up
			{			
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_ARM_UP_FULL\n")
				m_pArmControlLeft->EnableIdleArmMovement(FALSE);
				m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	// elbow slower, to avoid hitting ground
				m_pArmControlLeft->ExecutePositionAndSpeed();

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
				gArmTimerLeft = 32;	// 1/10 second per count!
				m_pArmControlLeft->SetArmPosition( LEFT_ARM_SHOULDER_ARM_STRAIGHT_UP, 0, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePosition();				
				break;
			}
			case ARM_MOVEMENT_SCRATCH_BACK:
			{	
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_SCRATCH_BACK\n")
				m_pArmControlLeft->SetArmPosition( 190, 0, 125, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 50;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_WAVE: 
			{			
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_WAVE\n")
				m_pArmControlLeft->SetArmPosition( 90, 0, 90, 0, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_THROW_OBJECT_FRONT: 
			{	
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_THROW_OBJECT_FRONT\n")
				m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
				m_pArmControlLeft->SetArmPosition( 50, 0, -20, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}
			case ARM_MOVEMENT_PUT_IN_BASKET: 
			{	
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshInitThrowObjectBack);
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_IN_BASKET\n")
				m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
				m_ArmMovementStateLeft = 1;	// go to first state
				__itt_task_end(pDomainControlThread);
				break;
			}

			case ARM_MOVEMENT_IDENTIFY_OBJECT: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_IDENTIFY_OBJECT\n")
				// Close Claw (in case it's not closed already)
				//m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);
				m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state
				break;
			}

			default:
			{
				ROBOT_LOG( TRUE,"ERROR - Unhandled ARM MOVEMENT LEFT command (%02X)!\n", m_ArmMovementLeft)
			}
		} // switch m_ArmMovementLeft
	__itt_task_end(pDomainControlThread);
}






//***************************************************************************************************************************************
// HandleArmServoStatusUpdate
// Handle status update from the servos.
// Check Arm Movement State Machine.  Only used with movements that have multiple steps
// Uses m_ArmMovementStateRight or m_ArmMovementStateLeft to track progress
//***************************************************************************************************************************************
void CBehaviorModule::HandleArmServoStatusUpdateLeft( WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);
		__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmServoStatusUpdateLeft);

	// Get current X,Y,Z position for debug
//			static int LastArmX=0, LastArmY=0,LastArmZ=0;
//			double ArmX, ArmY, ArmZ;
//			m_pArmControlLeft->GetArmXYZ(ArmX, ArmY, ArmZ, TRUE );	// TRUE = get Target position, not current

	// Tell camera to go to ABSOLUTE PAN/TILT position pointing at X,Y,Z in space
	// wParam = X,Z in TenthInches
	// lParam = Y (distance from robot) in TenthInches

	//long ArmXYZParam1 = MAKELONG((int)(ArmZ*10.0), (int)(ArmX*10.0));	// LoWord, HiWord in inches * 10 (to allow for fractions)
	//long ArmXYZParam2 = (int)(ArmY*10.0);								// in inches * 10 (to allow for fractions)
	//SendCommand( WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD, ArmXYZParam1, ArmXYZParam2 );

	if( ARM_MOVEMENT_NONE != m_ArmMovementLeft ) 
	{
		// Something to do.  See if we are ready to do the next step

		if( 0 != gArmTimerLeft)
		{
			__itt_marker(pDomainControlThread, __itt_null, pshWaitingForArmTimerLeft, __itt_marker_scope_task);
			ROBOT_LOG( DEBUG_ARM_MOVEMENT, "WAITING FOR ARM TIMER LEFT = %d\n", gArmTimerLeft)
			__itt_task_end(pDomainControlThread); // pshHandleArmServoStatusUpdateLeft
			return;
		}

		BOOL ArmInPosition = m_pArmControlLeft->CheckArmPosition(DEBUG_ARM_MOVEMENT);//	TRUE = Verbose
		if( m_ArmWaitForMoveToCompleteLeft && !ArmInPosition )
		{
			//__itt_marker(pDomainControlThread, __itt_null, pshWaitingForArmLeft, __itt_marker_scope_task);
			if( DEBUG_ARM_MOVEMENT ) ROBOT_LOG( TRUE,"HandleArmServoStatusUpdateLeft: WAITING FOR ARM MOVE TO COMPLETE LEFT \n")
			__itt_task_end(pDomainControlThread); // pshHandleArmServoStatusUpdateLeft
			return;
		}

		{ 
			__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshSomethingToDo);

			// Last movement completed.  Do the next movement.
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// LEFT ARM ADDITIONAL MOVEMENTS
			switch( m_ArmMovementLeft )  // Arm Movement in progress
			{
				case ARM_MOVEMENT_NONE:
				case ARM_MOVEMENT_HOME1:			// Home
				case ARM_MOVEMENT_EXTEND_FULL:		// Extend Arm Fully
				case ARM_MOVEMENT_OPEN_CLAW:		// Open Claw
				case ARM_MOVEMENT_GRAB_COKE:		// Grab Coke
				case ARM_MOVEMENT_CLOSE_CLAW_FULL:	// Close Claw Full
				case ARM_MOVEMENT_LIFT_OBJECT:		// Lift Object
				{
					m_ArmMovementLeft = ARM_MOVEMENT_NONE;
					break;	// Nothing to do
				}

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_HOME2: // Arm locked Up position
				{	
					switch( m_ArmMovementStateLeft )
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
							m_pArmControlLeft->GetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
							m_pArmControlLeft->SetArmPosition( Shoulder, ElbowRotate, ElbowBend, Wrist, Claw  );
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_HOME2

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_ARM_UP_FULL: // Raise your left hand...
				{	
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							m_pArmControlLeft->SetArmPosition( LEFT_ARM_SHOULDER_ARM_STRAIGHT_UP, NOP, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_ArmMovementStateLeft++;
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
						case 2:
						{
							m_pArmControlLeft->SetArmPosition( NOP, -90, NOP, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();

							m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	
							m_pArmControlLeft->ExecutePositionAndSpeed();

							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_ARM_UP_FULL

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SCRATCH_BACK: // Scratch Back
				{	
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							m_pArmControlLeft->SetArmPosition( 225, -60, 125, 140, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2:
						{
							m_pArmControlLeft->SetArmPosition( 205, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							////gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition( 225, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:
						{
							m_pArmControlLeft->SetArmPosition( 205, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition( 225, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 6:
						{
							m_pArmControlLeft->SetArmPosition( 205, -40, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 7:
						{
							LeftArmHome();	// Home
							break;
						}
					}
					break;
				}	// case ARM_MOVEMENT_SCRATCH_BACK

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_WAVE: // Wave
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// Make the wave command a bit faster then other commands
							m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft+1, m_ArmSpeedLeft+1, m_ArmSpeedLeft+1, m_ArmSpeedLeft+1, m_ArmSpeedLeft+1 );	
							m_pArmControlLeft->SetArmPosition( 90, 20, 90, 0, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2: // Wave 1
						{
							// Respond with random phrases
							int RandomNumber = ((5 * rand()) / RAND_MAX);
							ROBOT_LOG( TRUE,"DEBUG: RAND = %d\n", RandomNumber)
							switch( RandomNumber )
							{
								case 0:  SpeakText( "Hello, nice to meet you" );break;
								case 1:  SpeakText( "Hi there!" );break;
								case 2:  SpeakText( "Hi, what's up?" );break;
								case 3:  SpeakText( "Hello, how are you?" );break;
								default: SpeakText( "Hello!" ); // If const is larger number, this gets called more often
							}
		

							m_pArmControlLeft->SetArmPosition( NOP, -20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition( NOP, 20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:	// Wave 2
						{
							m_pArmControlLeft->SetArmPosition( NOP, -20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition( NOP, 20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
/***
						case 6:	// Wave 3
						{
							m_pArmControlLeft->SetArmPosition( NOP, -20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 7:
						{
							m_pArmControlLeft->SetArmPosition( NOP, 20, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
***/
						case 6:
						{
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT STATE LEFT(%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_WAVE:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_THROW_OBJECT_FRONT: // Throw Object
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// Set speed of all servos (used by each when moved)
	
							// Tighten Grip on object
							m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_pArmControlLeft->SetClawTorque(500);
							int NewClawPosition = m_pArmControlLeft->GetClawPosition() - 10;	// n degrees "tighter"
							m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, NOP, NewClawPosition ); // Set Claw position
							m_ArmMovementStateLeft++;	// go to next state
							m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX, SERVO_SPEED_MAX );
							m_pArmControlLeft->ExecutePositionAndSpeed();
							break;
						}

/**** NEW ***/
						case 2: // snap the elbow, ////NEW!!!!  and open the hand
						{								
							m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
							gArmTimerLeft = 30;	// 1/10 second per count!
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							m_pArmControlLeft->SetArmDelay( NOP, NOP, NOP, NOP, 100 );	// Delayed Claw opening (MS)

							m_pArmControlLeft->SetArmPosition( NOP, NOP, 120, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlLeft->SetArmPosition( NOP, NOP, 120, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, NOP, 120 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlLeft->ExecutePosition();
							break;
						}
						case 3:
						{
							//Open the hand
							m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, NOP, 120 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							m_ArmWaitForMoveToCompleteLeft = TRUE;	// wait for all to complete before moving to Home
							m_pArmControlLeft->ExecutePosition();
							break;
						}
						case 4:
						{
							// Home
							m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);
							m_ArmWaitForMoveToCompleteLeft = TRUE;	// Reset to default
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT STATE LEFT(%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_THROW_OBJECT_FRONT:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PUT_IN_BASKET: // Put Object in basket on Loki's back.  See m_PutObjectInCarryBasket references
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Start sequence of steps
						{
							// Put the object in the carry basket on Loki's back
							HeadCenter();
							ROBOT_LOG( TRUE,"Begin put object in basket\n")
							m_pArmControlLeft->SetArmPosition( 170, 0, 140, 65, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePositionAndSpeed();				
							m_ArmMovementStateLeft++;
							break;
						}
						case 2:	// Move into position
						{
							m_pArmControlLeft->SetArmPosition( 190, 30, 150, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePositionAndSpeed();				
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;
							break;
						}
						case 3:	// Open claw
						{
							m_pArmControlLeft->SetArmPosition( NOP, NOP, NOP, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;
							break;
						}
						case 4:	// Move claw out of basket
						{
							// Put the object in the carry basket on Loki's back
							HeadCenter();
							m_pArmControlLeft->SetArmPosition( 170, 0, 140, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePositionAndSpeed();				
							m_ArmMovementStateLeft++;
							break;
						}
						case 5:	// Move to home position
						{
							// do this here to assure arm reaches home before setting m_ObjectPickupComplete
							m_pArmControlLeft->MoveArmHome( m_ArmSpeedLeft );
							m_ArmMovementStateLeft++;
							break;
						}
						case 6:	// Done
						{
							// Move to home position and exit
							if( !m_ObjectPickupComplete )
							{
								// A pickup routine was in progress
								__itt_marker(pDomainControlThread, __itt_null, pshSucessPickUpComplete, __itt_marker_scope_task);
							}
							LeftArmHome();	// Home
							HeadCenter();
							m_ObjectPickupComplete = TRUE;
							m_PutObjectInCarryBasket = FALSE;
							break;
						}

						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT STATE LEFT(%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_PUT_IN_BASKET:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SHAKE_READY: // Shake Hands
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Wait for finger tip sensors to detect something
								// (will usually note person's hand )
						{
							if( ARM_L_IR_SENSOR_CLAW < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES  )	// 
							{
								// Object detected.  
								int RandomNumber = ((5 * rand()) / RAND_MAX);
								ROBOT_LOG( TRUE,"DEBUG: RAND = %d\n", RandomNumber)
								switch( RandomNumber )
								{
									case 0:  SpeakText( "Hello, nice to meet you" );break;
									case 1:  SpeakText( "You are, and always shall be, my friend" );break;
									case 2:  SpeakText( "Hello, how are you?" );break;
									case 3:  SpeakText( "Hello, I am pleased to meet you" );break;
									default: SpeakText( "Hello!" ); // If const is larger number, this gets called more often
								}
			

								m_pArmControlLeft->SetArmPosition(
									LEFT_ARM_SHOULDER_ARM_SHAKE_DOWN, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
									LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePosition();

								//gArmTimerLeft = 20;	// 1/10 second per count!
								m_ArmMovementStateLeft++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}
						case 2: // Shake 1
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_UP, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_UP,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_DOWN, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:	// Shake 2
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_UP, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_UP,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_DOWN, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
/*								case 6:	// Shake 3
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_UP, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_UP,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 7:
						{
							m_pArmControlLeft->SetArmPosition(
								LEFT_ARM_SHOULDER_ARM_SHAKE_DOWN, LEFT_ARM_ELBOW_ROTATE_ARM_SHAKE, LEFT_ARM_ELBOW_BEND_ARM_SHAKE_DOWN,
								LEFT_ARM_WRIST_ROTATE_ARM_SHAKE, LEFT_ARM_CLAW_ARM_SHAKE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
*/								case 6:
						{
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_SHAKE_MOVE:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW: // Extending arm to take an object
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}

						case 1:
						{	// Move arm toward center of body
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							m_pArmControlLeft->SetArmPosition( NOP, 8, NOP,	NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();				
							m_ArmMovementStateLeft++;	// go to next state
							break;	
						}

						case 2: // Wait for claw sensor to detect something
								// (will usually note person's hand as they provide the object)
						{
							if( g_pSensorSummary->nObjectClawLeft < CLAW_DETECT_TRIGGER_DISTANCE_TENTH_INCHES  )	// 
							{
								// Object detected.  Close claw.
								ROBOT_LOG( TRUE,"Extend Arm: Fingertip Object detected. Closing Claw\n")
								m_ArmWaitForMoveToCompleteLeft = TRUE;
								m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);
								m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePositionAndSpeed();
								m_ArmMovementStateLeft++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}

						case 3: // Wait for claw to close completely
								// (so it does not drop CD or dollar bill)
						{
							//Sleep(500);
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_pArmControlLeft->ClawRegripObject(); // Regrip so we don't overload the servo
							m_ArmMovementStateLeft++;	// Examine Object
							break;
						}


						case 4:	// Claw closed.  Now examine the object
						{
							// This code will kick off object identificaiton with the Camera App,
							// if the Camera app is enabled.  Otherwise, just speak a random phrase.

							if( (NULL != g_pCameraRequestSharedMemory) && (NULL != g_hCameraRequestEvent) )
							{
								// Camera is enabled, Identify the object

								/////////////////////////////////////////////////////////////////////////////////////////////////
								// TODO - Use Kinect to detect if there is an object in the hand
								// Just view a box of pixels above the finger tips
								//	SpeakText( "I did not get anything.  Please try again" );	
								//	m_ArmMovementStateLeft = 1; // Try Again
								/////////////////////////////////////////////////////////////////////////////////////////////////

								// Examine the Object
								SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_IDENTIFY_OBJECT );	// Left/Left arm, Movement to execute, 
								m_ArmMovementStateLeft = 0;	// back to Idle state
								m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
								ROBOT_LOG( TRUE,"Extend Arm: Sending Examine Object command\n")
								break;
							}
							else
							{
								// Camera is disabled, just say something
								#if ( PUBLIC_DEMO == 1)
									RandomNumber = 0;
									SpeakText( "This is my ball" );
								#else
									int RandomNumber = ((4 * rand()) / RAND_MAX);
									switch( RandomNumber )
									{
										case 0:  SpeakText( "This is nice" );break;
										case 1:  SpeakText( "This is cool" );break;
										case 2:  SpeakText( "This is my favorite new toy" );break;
										default: SpeakText( "Thank you" ); // If const is larger number, this gets called more often
									}
								#endif //( PUBLIC_DEMO == 1)

								HeadCenter(); // Look Forward
								LeftArmHome();	// Home
								m_ArmMovementStateLeft = 0;	// back to Idle state
								m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
								break;
							}
						}

						case 5:	// DONE  - move are home with object in hand
						{
							//ROBOT_ASSERT(0); // this code no longer used?
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							break;
						}


						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW: // Extending arm to give an object
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}

						case 1:
						{	// Move arm toward center of body
							m_ArmWaitForMoveToCompleteLeft = TRUE;
							m_pArmControlLeft->SetArmPosition( NOP, 8, NOP,	NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();				
							m_ArmMovementStateLeft++;	// go to next state
							break;	
						}
						case 2: // Wait for head sensor to detect something
								// (should note person's hand as they take the object)
						{
							// TODO-MUST HEAD IR SENSOR NOT WORKING - WORK AROUND: USE FINGER TIP SENSORS
							// **** TODO-MUST WHY IS I2C-IT IR NOT USED???
							//if( g_SensorStatus.IR[2] < 160  )	// tenth inches
							if( ARM_L_IR_BUMPER_OBJECT_FINGER_L || ARM_L_IR_BUMPER_OBJECT_FINGER_R )
							{
								// Hand detected.  Open claw.
								ROBOT_LOG( TRUE,"Extend Arm: Head sensors detected hand. Opening Claw\n")
								m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePositionAndSpeed();
								m_ArmMovementStateLeft++;	// go to next state
							}
							// Otherwise, just continue to wait
							break;
						}

						case 3:	// Claw Opened.  return to home position
						{
							// Move head back to forward 
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_LOOK_AT_HAND:
				{
					__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshLookAtHand);
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							// m_pArmControlLeft->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->SetArmPosition(NOP, -20, 120, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-160, CAMERA_TILT_CENTER-140, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							//gArmTimerLeft = 20;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state

							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, -60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:	// 
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 10;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 6:
						{
							// Home
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				__itt_task_end(pDomainControlThread);
				}	// case ARM_MOVEMENT_LOOK_AT_HAND:


				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_SCRATCH_HEAD:
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT, 7 );
							// m_pArmControlLeft->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->SetArmPosition(105, -13, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlLeft->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:	// 
						{
							m_pArmControlLeft->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 6:
						{
							// Home
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_SCRATCH_HEAD:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_KARATE:
				{
					ROBOT_LOG( TRUE,"DEBUG KARATE: m_ArmMovementStateLeft = %d\n", m_ArmMovementStateLeft)
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:
						{
							SendCommand( WM_ROBOT_CAMERA_SIDETILT_ABS_CMD, (DWORD)CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT, 7 );
							// m_pArmControlLeft->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->SetArmPosition(105, -13, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2: // 
						{
							m_pArmControlLeft->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:
						{
							m_pArmControlLeft->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 4:	// 
						{
							m_pArmControlLeft->SetArmPosition(110, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5:
						{
							m_pArmControlLeft->SetArmPosition(105, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 6:
						{
							// Home
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_SCRATCH_HEAD:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PICKUP_OBJECT_XYZ:
				{
					__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshPickupObjectXYZLeft);
					// Pickup Object previously found by Kinect.
					// Object position is stored in m_ObjectY, m_ObjectX
					// Servo positions to reach the object stored in this structure:
					ARM_SERVOS_POSITION_T TargetServo;
					TargetServo.ShoulderAngle = 0;
					TargetServo.ElbowRotateAngle = 0;
					TargetServo.ElbowBendAngle = 90;

					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							__itt_marker(pDomainControlThread, __itt_null, pshNothingToDo, __itt_marker_scope_task);

							break;	// Nothing to do
						}
						case 1: 
						{
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshFirstStep);
							ROBOT_LOG( TRUE,"\n===========> BehaviorModule: First Step Left - ARM_MOVEMENT_PICKUP_OBJECT_XYZ\n")

							FPOINT3D_T TargetXYZ;
							TargetXYZ.Y = (double)m_ObjectY + 10.0;	// TenthInches TODO-DWS - FUDGE FACTOR ADDED ARM WAS REACHING TOO SHORT! (Is there a better place to fix this?)
							TargetXYZ.X = m_ObjectX - 5.0; // TenthInches TODO-DWS - FUDGE FACTOR ADDED ARM WAS REACHING TOO FAR RIGHT!
							TargetXYZ.Z = PICKUP_OBJECT_DEFAULT_Z; // Hardcode for now - TenthInches
							BOOL bSolutionFound = FALSE;

							// TODO! Need a robust mechanism here for all positions the object can be in!
							if( (m_ObjectY > PICKUP_OBJECT_MAX_Y) ||(m_ObjectX > PICKUP_OBJECT_MAX_X) )
							{
								// Object out of range!
								bSolutionFound = FALSE;
								CString MsgString;
								ROBOT_DISPLAY( TRUE, "ARM BEHAVIOR MODULE: OBJECT OUT OF RANGE!" )
								MsgString.Format( "m_ObjectY = %d, m_ObjectX = %d", m_ObjectY/10, m_ObjectX/10 );
								ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
								ROBOT_DISPLAY( TRUE, "---------------------------" )
								__itt_marker(pDomainControlThread, __itt_null, pshObjectOutOfRange, __itt_marker_scope_task);
							}
							else
							{
								// Calculate position needed to pickup the object
								bSolutionFound = m_pArmControlLeft->CalculateArmMoveToXYZ( TargetXYZ, TargetServo );
							}

				//%^**************** DEBUG ********
				//#define DISABLE_PICKUP
				#ifdef DISABLE_PICKUP

					ROBOT_LOG( TRUE,"************** ACTUAL PICKUP DISABLED FOR DEBUG, DONE WITH MOVEMENT ***********\n")
					m_ArmMovementStateLeft = 0;	// back to Idle state
					m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
					__itt_task_end(pDomainControlThread); // pshMoveToReadyPosition

				#else

							if( bSolutionFound )
							{
								ROBOT_LOG( TRUE,"************** MOVING ARM TO OBJECT ***********\n")
								__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshMoveToReadyPosition);
								// Move the Arm into pickup ready position
								m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
								m_pArmControlLeft->SetArmPosition((int)TargetServo.ShoulderAngle, (int)TargetServo.ElbowRotateAngle, (int)TargetServo.ElbowBendAngle, (int)TargetServo.WristRotateAngle, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePositionAndSpeed();
								m_pArmControlLeft->CalibratePressureSensors(); // calibrate sensors just before we pick up an object
								m_ArmMovementStateLeft++;	// go to next state
								__itt_task_end(pDomainControlThread); // pshMoveToReadyPosition
							}
							else
							{
								ROBOT_LOG( TRUE,"************** NO SOLUTION FOUND, ABORTING1 ***********\n")
								LeftArmHome();	// Home
								m_ObjectPickupComplete = TRUE;
								m_PutObjectInCarryBasket = FALSE;
								__itt_marker(pDomainControlThread, __itt_null, pshAbortNoSolution, __itt_marker_scope_task);
							}
				#endif
							__itt_task_end(pDomainControlThread); // pshFirstStep
							break;
						}
						case 2: // Grab Object
						{
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGrabObject);
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_CLOSED_SNUG ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							ROBOT_LOG( TRUE,"DEBUG: GRAB OBJECT TIMER SET\n")
							gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							__itt_task_end(pDomainControlThread);

							break;
						}
						case 3:  // Determine if Object found, and if so, bend elbow
						{
							if( !m_pArmControlLeft->IsObjectInClaw() ) // m_pArmControlLeft->GetClawPosition() < LEFT_ARM_CLAW_CLOSED_LOOSE )
							{
								// No object grabbed
								m_ArmMovementStateLeft = 5; // handle error case
								ROBOT_LOG( TRUE,"DEBUG NO OBJECT GRABBED: ClawPosition = %d TenthInches\n", m_pArmControlLeft->GetClawPosition() )
								ROBOT_LOG( TRUE,"DEBUG: RETRY TIMER SET\n")
								gArmTimerLeft = 5;	// 1/10 second per count! - delay and then retry in next step
								__itt_marker(pDomainControlThread, __itt_null, pshAbortNoObjectInClaw, __itt_marker_scope_task);
							}
							else
							{
								// Lift the object off the ground
								ROBOT_LOG( TRUE,"**********************************\n", )
								ROBOT_LOG( TRUE,"********  OBJECT GRABBED!  *******\n\n", )
								__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshLiftObject);
								m_pArmControlLeft->SetArmPosition( NOP, NOP, (int)(TargetServo.ElbowBendAngle)+20, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePosition();
								m_nObjectsPickedUp++; // indicate sucess!
								m_ArmMovementStateLeft++;	// go to next state
								__itt_task_end(pDomainControlThread);

							}
							break;
						}
						case 4:	// Got it, now what?
						{
							if( m_PutObjectInCarryBasket )
							{
								//__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshInitThrowObjectBack);
								ROBOT_LOG( TRUE,"\n===========> BehaviorModule: Calling ARM_MOVEMENT_PUT_IN_BASKET\n")
								m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
								m_ArmMovementLeft = ARM_MOVEMENT_PUT_IN_BASKET;
								m_ArmMovementStateLeft = 1;	// go to first state
								break;
							}
							else
							{
								// Just move to home position and exit
								m_pArmControlLeft->ClawRegripObject(); // Regrip so we don't overload the servo
								int RandomNumber = ((5 * rand()) / RAND_MAX);
								switch( RandomNumber ) // Respond with random phrases
								{
									case 0:  SpeakText( "What should I do with it?" );break;
									case 1:  SpeakText( "Now what?" );break;
									case 2:  SpeakText( "OK, got it" );break;
									case 3:  SpeakText( "What's Next?" );break;
									default:  SpeakText( "What is your next command, oh great one?" );break;
								}
			
								m_ArmMovementStateLeft = 6;	// go to end state
							}
							break;
						}
						case 5:	// Did not pick anything up!
						{
							// Retry again, just incase we checked too soon
							if( m_pArmControlLeft->IsObjectInClaw() ) // m_pArmControlLeft->GetClawPosition() > LEFT_ARM_CLAW_CLOSED_LOOSE )
							{
								// Object WAS grabbed after all!
								m_ArmMovementStateLeft = 3; // Try again
								ROBOT_LOG( TRUE,"DEBUG: RETRY 2 TIMER SET\n")
								gArmTimerLeft = 5;	// 1/10 second per count!
								break;
							}

							// Nope, still missed it. Too bad, so sad.
							HeadCenter();
							int RandomNumber = ((2 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "My Eye Hand coordination is not very good yet" );break;
								default:  SpeakText( "Darn, I did not get it" );break;
							}
		
							LeftArmHome();	// Home
							ROBOT_LOG( TRUE,"************** OBJECT NOT PICKED UP, ABORTING 1***********\n")
							m_ObjectPickupComplete = TRUE;
							m_PutObjectInCarryBasket = FALSE;
							__itt_marker(pDomainControlThread, __itt_null, pshFinalNoObjectInClaw, __itt_marker_scope_task);
							break;
						}
						case 6:	// Done - success!
						{
							// Move to home position and exit
							LeftArmHome();	// Home
							HeadCenter();
							ROBOT_LOG( TRUE,"************** Object Picked up 1 ***********\n")
							m_ObjectPickupComplete = TRUE;
							m_PutObjectInCarryBasket = FALSE;
							__itt_marker(pDomainControlThread, __itt_null, pshSucessPickUpComplete, __itt_marker_scope_task);
							break;
						}

						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							m_ObjectPickupComplete = TRUE;
							m_PutObjectInCarryBasket = FALSE;
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				__itt_task_end(pDomainControlThread);
				}	// case ARM_MOVEMENT_PICKUP_OBJECT_XYZ:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PICKUP_OBJECT_BLIND:
				{
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1: // Move arm down into position
						{
							// m_pArmControlLeft->SetArmPosition(65, -20, 100, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->SetArmPosition(-20, 14, 59, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 2: // Grab Object
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, LEFT_ARM_CLAW_CLOSED_TIGHT ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3:  // Determine if Object found, and if so, bend elbow
						{
							if( !m_pArmControlLeft->IsObjectInClaw() ) //m_pArmControlLeft->GetClawPosition() < LEFT_ARM_CLAW_CLOSED_LOOSE )
							{
								// No object grabbed
								m_ArmMovementStateLeft = 7; // handle error case
							}
							else
							{
								m_pArmControlLeft->SetArmPosition(NOP, 3, 80, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
								m_pArmControlLeft->ExecutePosition();
								//gArmTimerLeft = 5;	// 1/10 second per count!
								m_ArmMovementStateLeft++;	// go to next state
							}
							break;
						}
						case 4:	// Lift object in front of face
						{
							m_pArmControlLeft->SetArmPosition(60, NOP, 120, -60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							//m_pArmControlLeft->SetArmPosition(80, NOP, NOP, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-400, CAMERA_TILT_CENTER-300, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 5: // Look at it
						{
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-300, CAMERA_TILT_CENTER-60, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateLeft++;	// go to next state
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
						case 6:	// Now what?
						{
							m_pArmControlLeft->SetArmPosition(NOP, 0, 80, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
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
		
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ObjectPickupComplete = TRUE;
							ROBOT_LOG( TRUE,"************** Object Picked up 2 ***********\n")

							// Give object to user
							SendCommand( WM_ROBOT_SET_ARM_MOVEMENT, (DWORD)LEFT_ARM, (DWORD)ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW );	// Left/Left arm, Movement to execute, 
							break;
						}
						case 7:	// Did not pick anything up!
						{
							HeadCenter();
							int RandomNumber = ((2 * rand()) / RAND_MAX);
							switch( RandomNumber ) // Respond with random phrases
							{
								case 0:  SpeakText( "My Eye Hand coordination is not very good yet" );break;
								default:  SpeakText( "Darn, I did not get it" );break;
							}
		
							LeftArmHome();	// Home
							m_ObjectPickupComplete = TRUE;
							ROBOT_LOG( TRUE,"************** OBJECT NOT PICKED UP, ABORTING 3***********\n")
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							m_ObjectPickupComplete = TRUE;
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
				}	// case ARM_MOVEMENT_PICKUP_OBJECT_BLIND:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_PUT_DOWN_OBJECT:
				{
					__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshPutDownObjectLeft);
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Set object down
						{
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCase1);
							m_pArmControlLeft->SetArmPosition(NOP, NOP, 50, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							__itt_task_end(pDomainControlThread);
							break;
						}
						case 2:	// Drop it
						{
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCase2);
							m_pArmControlLeft->SetClawTorque(MED_TORQUE); // more torque to open claw?
							m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second (100ms) per count!
							m_ArmMovementStateLeft++;	// go to next state
							__itt_task_end(pDomainControlThread);
							break;
						}
						case 3:	// Move Hand so we don't bump it
						{
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCase3);
							m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE); // reset torque
							m_pArmControlLeft->SetArmPosition(-25, NOP, 80, NOP, 100 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							m_ArmMovementStateLeft++;	// go to next state
							__itt_task_end(pDomainControlThread);
							break;
						}
						case 4: // Home
						{	
							__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCase4);
							HeadCenter(); // Look Forward
							LeftArmHome();	// Home
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							__itt_task_end(pDomainControlThread);
							break;
						}
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
					__itt_task_end(pDomainControlThread);
				}	// case ARM_MOVEMENT_PUT_DOWN_OBJECT:

				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				case ARM_MOVEMENT_IDENTIFY_OBJECT:
				{
					__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshIdentifyObject);
					switch( m_ArmMovementStateLeft )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Lift object in front of face
						{
							m_pArmControlLeft->SetArmPosition(42, -12, 120, 70, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							// Point camera at arm, and zoom in on object
							//SendCommand( WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD, ArmXYZParam1, ArmXYZParam2 );
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-216, CAMERA_TILT_CENTER-190, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
							m_ArmMovementStateLeft++;	// go to next state
							gArmTimerLeft = 20;	// 1/10 second per count!
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
							// Give time for the zoom to settle before grabbing the frame to match
							//Sleep(2000);
							gArmTimerLeft = 200;	// 1/10 second per count!
							SendCommand( WM_ROBOT_CAMERA_MATCH_OBJECT, 0, 0 );
							m_ArmMovementStateLeft++;	// go to next state
							break;
						}
						case 3: // waiting for vision to complete
						{
							// Nothing to do.  State machine must be moved in the vision processing case statement
						//	ROBOT_LOG( TRUE,"Behavior Module: ARM_MOVEMENT_IDENTIFY_OBJECT:3: Servo update ignored! Waiting for vision system")
							break;
						}
/********** NOT NEEDED, HANDLED BY HandleCameraMatchComplete()

						case 4: // Waiting for wrist to complete rotating 90 degrees, so we can look at object again
						{
							// Ok, done rotating now.   Look at it in position #2
							// Send command to vision system to identify the object.
							// NOTE!  The response to this will go to a different processing path
							// and then return to the next case statement when done...

							SendCommand( WM_ROBOT_CAMERA_MATCH_OBJECT, 0, 0 );
							//SpeakText( "Hmmm" );	
							m_ArmMovementStateLeft++;	// go to next state
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
							m_pArmControlLeft->SetArmPosition(NOP, 0, 80, NOP, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
							m_pArmControlLeft->ExecutePosition();
							//gArmTimerLeft = 5;	// 1/10 second per count!
							HeadCenter(); // Look Forward
							m_ArmMovementStateLeft = 0;	// back to Idle state
							m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
							SpeakText( "What should I do with it?" );	
		
							break;
						}
**************/
						default:
						{
							ROBOT_LOG( TRUE,"ERROR - Bad ARM MOVEMENT in state machine%02X)!\n", m_ArmMovementStateLeft)
							ROBOT_ASSERT(0);
						}
					} // switch( m_ArmMovementStateLeft )
					break;
					__itt_task_end(pDomainControlThread);
				}	// case ARM_MOVEMENT_IDENTIFY_OBJECT:


				////////////////////////////////////////////////////////////////////////////////////////////////////
				default:
				{
					ROBOT_LOG( TRUE,"BehaviorModule: ERROR - Unhandled Arm Movement Left (%02X)!\n", m_ArmMovementLeft)
				}
			} // switch m_ArmMovementLeft

			__itt_task_end(pDomainControlThread);
		} //  if ( 0 != gArmTimerLeft )
	}	// if( ARM_MOVEMENT_NONE != m_ArmMovementLeft )

	__itt_task_end(pDomainControlThread);
}	// HandleServoStatusUpdateLeft


#endif // ROBOT_SERVER
