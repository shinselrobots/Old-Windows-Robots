// ArmBehavior.cpp: Subset of Behavior Module for Arm Control
// 
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
//#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


///////////////////////////////////////////////////////////////////////////////
// Macros and defines

#define ARM_RANDOM_MOVE_CHECK_TIME 8 //how often to move the arm, in number of Sensor updates from Arduino

//#define OBJECT_DETECT_BOTH_FINGERS	0xC0
//#define OBJECT_DETECT_LEFT_FINGER	0x80
//#define OBJECT_DETECT_RIGHT_FINGER	0x40

#define PICKUP_OBJECT_MAX_Y			80	// TenthInches - from front of robot
#define PICKUP_OBJECT_MAX_X			150	// TenthInches - from center of robot

DWORD dbgTime = GetTickCount();

__itt_string_handle* pshHandleArmMovementRequest = __itt_string_handle_create("HandleArmMovementRequest");
__itt_string_handle* pshHandleCameraMatchResponse = __itt_string_handle_create("HandleCameraMatchComplete");
__itt_string_handle* pshHandleArmServoStatusUpdate = __itt_string_handle_create("HandleArmServoStatusUpdate");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleArmMovementRequest
// Process Arm Behavior commands and do inital movemment for arms (move into first position)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::HandleArmMovementRequest( WPARAM wParam, LPARAM lParam )
{
	CString MsgString;
//	UINT ActionRequested;
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmMovementRequest);

	if( LEFT_ARM == wParam )	// wParam is Right, Left, or Both Arms
	{
		HandleArmMovementRequestLeft( wParam, lParam );
	}
	else if( RIGHT_ARM == wParam )
	{
		HandleArmMovementRequestRight( wParam, lParam );
	}
	else if( BOTH_ARMS == wParam )
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// BOTH ARM INITIAL MOVEMENT
		m_ArmMovementBoth = lParam;
		// by default, each movement should complete before the next begins
		// Can be overridden below, and gArmTimerRight used instead
		m_ArmWaitForMoveToCompleteRight = TRUE;	
		m_ArmWaitForMoveToCompleteLeft = TRUE;	
		// On each arm move command, reset values to default speed (as specified by GUI)
		m_pArmControlRight->SetArmSpeed( m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight, m_ArmSpeedRight );	
		m_pArmControlLeft->SetArmSpeed( m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft, m_ArmSpeedLeft );	


		switch( m_ArmMovementBoth )  // lParam is the movement requested
		{
			case ARM_MOVEMENT_NONE:
			{
				ROBOT_LOG( TRUE,"BehaviorModule: ACTION_MODE_NONE requested\n")
				break;
			}
			case ARM_MOVEMENT_HOME1:	// Home position (sensors in position to move)
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME1 Both Arms\n")
				// TODO FIX THIS KLUDE - for now kill any actions!
				// Need to create a new message ARM_MOVEMENT_STOP or something.
				m_CurrentActionMode = ACTION_MODE_NONE;
				m_CurrentTask = TASK_NONE;
				m_TaskState = 0;
				RightArmHome();
				LeftArmHome();

				break;
			}
			case ARM_MOVEMENT_HOME2:	// Home with arm up
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_HOME2 Both arms\n")
				m_pArmControlRight->SetArmPosition( 
					RIGHT_ARM_SHOULDER_HOME2, RIGHT_ARM_ELBOW_ROTATE_HOME2, RIGHT_ARM_ELBOW_BEND_HOME2, 
					RIGHT_ARM_WRIST_ROTATE_HOME2, RIGHT_ARM_CLAW_HOME2 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				m_ArmMovementStateRight = 1;	// go to first state
				m_ArmMovementRight = lParam;	// Tell the Right arm state machine to continue this movement

				m_pArmControlLeft->SetArmPosition( 
					LEFT_ARM_SHOULDER_HOME2, LEFT_ARM_ELBOW_ROTATE_HOME2, LEFT_ARM_ELBOW_BEND_HOME2, 
					LEFT_ARM_WRIST_ROTATE_HOME2, LEFT_ARM_CLAW_HOME2 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();		
				m_ArmMovementStateLeft = 1;	// go to first state
				m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement
				break;
			}

			case ARM_MOVEMENT_90_DEGREE:	// Move both arms to 90 Degree position for calibration / mechanical adjustment
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_90_DEGREE Both arms\n")
				m_pArmControlRight->SetArmPosition( 
					RIGHT_ARM_SHOULDER_90_DEGREE, RIGHT_ARM_ELBOW_ROTATE_90_DEGREE, RIGHT_ARM_ELBOW_BEND_90_DEGREE, 
					RIGHT_ARM_WRIST_ROTATE_90_DEGREE, RIGHT_ARM_CLAW_90_DEGREE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();			
				m_ArmMovementStateRight = 0;			// back to Idle state
				m_ArmMovementRight = ARM_MOVEMENT_NONE; // back to Idle state
				m_ArmWaitForMoveToCompleteRight = TRUE;


				m_pArmControlLeft->SetArmPosition( 
					LEFT_ARM_SHOULDER_90_DEGREE, LEFT_ARM_ELBOW_ROTATE_90_DEGREE, LEFT_ARM_ELBOW_BEND_90_DEGREE, 
					LEFT_ARM_WRIST_ROTATE_90_DEGREE, LEFT_ARM_CLAW_90_DEGREE ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();		
				m_ArmMovementStateLeft = 0;				// back to Idle state
				m_ArmMovementLeft = ARM_MOVEMENT_NONE; // back to Idle state
				m_ArmWaitForMoveToCompleteLeft = TRUE;

				break;
			}

			case ARM_MOVEMENT_OPEN_CLAW:	
			{
				// Try to figure out which claw to open, based upon arm position
				if( (gArmInHomePositionRight) && (gArmInHomePositionLeft)	||	// both arms home
					(!gArmInHomePositionRight) && (!gArmInHomePositionLeft)	)	// both arms not home
				{
					// Don't know which claw to open, so open both!
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_OPEN_CLAW Both Arms\n")
					m_pArmControlRight->SetClawTorque(STRONG_TORQUE);	// Right arm is the strong one
					m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 0, RIGHT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePositionAndSpeed();				

					m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
					m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 0, LEFT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				

				}
				else if( !gArmInHomePositionRight )
				{
					// Right arm not in home.  Probably doing something so assume command applies to this arm!
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_OPEN_CLAW Assume Right Arm\n")
					m_pArmControlRight->SetClawTorque(STRONG_TORQUE);	// reset to strong torque
					m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 0, RIGHT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlRight->ExecutePositionAndSpeed();				

				}
				else if( !gArmInHomePositionLeft )
				{
					// Left arm not in home.  Probably doing something so assume command applies to this arm!
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_OPEN_CLAW Assume Left Arm\n")
					m_pArmControlLeft->SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
					m_pArmControlLeft->SetArmPosition(NOP, NOP, NOP, 0, LEFT_ARM_CLAW_OPEN_NORMAL ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
				}
				else
				{
					ROBOT_ASSERT(0);	// logic error
				}

				break;
			}
			case ARM_MOVEMENT_THROW_OBJECT_FRONT:	
			{
				// Try to figure out which arm(s) we are talking about, based upon claw torque load
				// Note that both arms may be holding something.  If so, throw both!

				UINT ClawTorqueRight = m_pArmControlRight->GetClawTorque();
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( (ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT) || (ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT) )
				{
					// At least one of the hands is holding something
					if( ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!
						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_THROW_OBJECT_FRONT (LOAD RIGHT)\n")
						m_pArmControlRight->EnableIdleArmMovement(FALSE);
						m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
						m_pArmControlRight->SetArmPosition( 50, 0, -20, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						m_pArmControlRight->ExecutePositionAndSpeed();				
						m_ArmMovementStateRight = 1;	// go to first state
						m_ArmMovementRight = lParam;	// Tell the Right arm state machine to continue this movement

					}

					if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!
						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_THROW_OBJECT_FRONT (LOAD LEFT)\n")
						m_pArmControlLeft->EnableIdleArmMovement(FALSE);
						m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
						m_pArmControlLeft->SetArmPosition( 50, 0, -20, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						m_pArmControlLeft->ExecutePositionAndSpeed();				
						m_ArmMovementStateLeft = 1;	// go to first state
						m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement
					}
				}
				else
				{
					SpeakText( "I don't have anything to throw" );
				}

				break;
			}
			case ARM_MOVEMENT_PUT_IN_BASKET:	
			{
				// Left arm only
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
				{
					// Claw is holding something!
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_IN_BASKET\n")
					m_pArmControlRight->EnableIdleArmMovement(FALSE);
					m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
					m_pArmControlLeft->SetArmPosition( 180, 0, -20, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
					m_ArmMovementStateLeft = 1;	// go to first state
					m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement
				}
				else
				{
					SpeakText( "I don't have anything to put in my basket" );

				}
				break;
			}
			case ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW:	
			case ARM_MOVEMENT_EXTEND_ARM_AUTO_CLAW:	
			{
				// Try to figure out which arm(s) we are talking about, based upon claw torque load
				// Note that both arms may be holding something.  If so, extend both!
				// If neither arm holding anything, extend the left arm with open claw

				UINT ClawTorqueRight = m_pArmControlRight->GetClawTorque();
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( (ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT) || (ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT) )
				{
					// At least one of the hands is holding something
					if( ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!
						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM (LOAD RIGHT)\n")
						m_pArmControlRight->EnableIdleArmMovement(FALSE);
						m_pArmControlRight->SetArmPosition( 60, -5, 80,	5, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						m_pArmControlRight->ExecutePositionAndSpeed();				
						m_ArmMovementStateRight = 1;	// go to first state
						m_ArmMovementRight = ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW;	// Tell the Right arm state machine to continue this movement
					}

					if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!

						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM (LOAD LEFT)\n")
						m_pArmControlLeft->EnableIdleArmMovement(FALSE);
						// Move head toward hand 
						gHeadIdle = FALSE;

						// Tell camera to go to ABSOLUTE PAN/TILT position specified
						m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-120, CAMERA_TILT_CENTER, NOP );	// Tenthdegrees!, No SideTilt
						m_pHeadControl->ExecutePosition( HEAD_OWNER_BEHAVIOR_P1 );

						m_pArmControlLeft->SetArmPosition( 60, 8, 80,	5, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						m_pArmControlLeft->ExecutePositionAndSpeed();				
						m_ArmMovementStateLeft = 1;	// go to first state
						m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_CLOSE_CLAW;	// Tell the Left arm state machine to continue this movement
					}

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
					// Neither hand holding anything
					ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_EXTEND_ARM (NO LOAD), Entending Left arm (DEFAULT)\n")
					// Same as ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW
					m_pArmControlLeft->SetArmPosition( 60, 8, 80,	5, 70 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
					m_pArmControlLeft->ExecutePositionAndSpeed();				
					m_ArmMovementStateLeft = 1;	// go to first state
					m_ArmMovementLeft = ARM_MOVEMENT_EXTEND_ARM_OPEN_CLAW;	// Tell the Left arm state machine to continue this movement


					// Respond with random phrases
					int RandomNumber = ((3 * rand()) / RAND_MAX);
					switch( RandomNumber ) // Respond with random phrases
					{
						case 0:  SpeakText( "OK" );break;
						case 1:  SpeakText( "all right" );break;
						default: SpeakText( "For me?" );break;
					}

				}

				break;
			}
			case ARM_MOVEMENT_PUT_DOWN_OBJECT:
			{
				// Try to figure out which arm(s) we are talking about, based upon claw torque load
				// Note that both arms may be holding something.  If so, put down both!

				ROBOT_LOG( TRUE, "************ ERROR: ARM_MOVEMENT_PUT_DOWN_OBJECT SHOULD NOT BE GETTING CALLED FOR BOTH ARMS! ************\n" )
				UINT ClawTorqueRight = m_pArmControlRight->GetClawTorque();
				UINT ClawTorqueLeft = m_pArmControlLeft->GetClawTorque();

				if( (ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT) || (ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT) )
				{
					// At least one of the hands is holding something
					if( ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!
						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_DOWN_OBJECT (LOAD RIGHT)\n")
						m_pArmControlRight->EnableIdleArmMovement(FALSE);
						m_pArmControlRight->SetClawTorque(STRONG_TORQUE); // get better grip

						if( !(ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT) )
						{
							// Right hand only.  Look at the object with head
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER+600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
						}
						//m_pArmControlRight->SetArmPosition(-23, 3, 65, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						//m_pArmControlRight->ExecutePositionAndSpeed();				
						m_ArmMovementStateRight = 1;	// go to first state
						m_ArmMovementRight = lParam;	// Tell the Right arm state machine to continue this movement
					}

					if( ClawTorqueLeft >= CLAW_TORQUE_DETECT_OBJECT )
					{
						// Claw is holding something!
						ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_PUT_DOWN_OBJECT (LOAD LEFT)\n")
						m_pArmControlRight->EnableIdleArmMovement(FALSE);
						m_pArmControlLeft->SetClawTorque(MED_TORQUE); // get better grip
						if( !(ClawTorqueRight >= CLAW_TORQUE_DETECT_OBJECT) )
						{
							// Left hand only.  Look at the object with head
							m_pHeadControl->SetHeadSpeed( HEAD_OWNER_BEHAVIOR_P1, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
							m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER-600, CAMERA_TILT_CENTER-500, CAMERA_SIDETILT_CENTER );
							m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_BEHAVIOR_P1 );
						}
						//m_pArmControlLeft->SetArmPosition(-20, 15, 68, 0, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
						//m_pArmControlLeft->ExecutePositionAndSpeed();				
						m_ArmMovementStateLeft = 1;	// go to first state
						m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement
					}
				}
				else
				{
					SpeakText( "I don't have anything to put down" );

				}

				break;
			}
			case ARM_MOVEMENT_ARM_UP_FULL: // Full Up
			{	
				// Put both arms up!
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_ARM_UP_FULL (BOTH)\n")
				m_pArmControlRight->EnableIdleArmMovement(FALSE);
				m_pArmControlRight->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	// elbow slower, to avoid hitting ground
				m_pArmControlRight->ExecutePositionAndSpeed();

				m_ArmWaitForMoveToCompleteRight = FALSE;	// Don't wait for each movement!
				gArmTimerRight = 32;	// 1/10 second per count!
				m_pArmControlRight->SetArmPosition( RIGHT_ARM_SHOULDER_ARM_STRAIGHT_UP, 0, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePosition();				
				m_ArmMovementStateRight = 1;	// go to first state
				m_ArmMovementRight = lParam;	// Tell the Right arm state machine to continue this movement


				m_pArmControlLeft->EnableIdleArmMovement(FALSE);
				m_pArmControlLeft->SetArmSpeed( SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_FAST, SERVO_SPEED_MED_FAST );	// elbow slower, to avoid hitting ground
				m_pArmControlLeft->ExecutePositionAndSpeed();

				m_ArmWaitForMoveToCompleteLeft = FALSE;	// Don't wait for each movement!
				gArmTimerLeft = 32;	// 1/10 second per count!
				m_pArmControlLeft->SetArmPosition( LEFT_ARM_SHOULDER_ARM_STRAIGHT_UP, 0, -7, 90, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePosition();				
				m_ArmMovementStateLeft = 1;	// go to first state
				m_ArmMovementLeft = lParam;	// Tell the Left arm state machine to continue this movement

				break;
			}
			case ARM_MOVEMENT_KARATE: // 
			{
				ROBOT_LOG( TRUE,"\n===========> BehaviorModule: ARM_MOVEMENT_KARATE\n")
				m_pArmControlRight->EnableIdleArmMovement(FALSE);
				m_pArmControlRight->SetArmPosition(110, 0, 130, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePositionAndSpeed();				
				//gArmTimerRight = 30;	// 1/10 second per count!
				m_ArmMovementStateRight = 1;	// go to first state
				gHeadIdle = FALSE;
				HeadCenter(); // Look Forward
				m_pArmControlLeft->EnableIdleArmMovement(FALSE);
				m_pArmControlLeft->SetArmPosition(110, 0, 130, 90, 0 ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlLeft->ExecutePositionAndSpeed();				
				//gArmTimerLeft = 30;	// 1/10 second per count!
				m_ArmMovementStateLeft = 1;	// go to first state

				break;
			}
			default:
			{
				ROBOT_LOG( TRUE,"ERROR - Unhandled ARM MOVEMENT BOTH command (%02X)!\n", m_ArmMovementBoth)
			}
		} // switch m_ArmMovementBoth
	}	// if BOTH_ARMS == wParam 
	else
	{
		ROBOT_ASSERT(0); // Logic Error
	}

	__itt_task_end(pDomainControlThread);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleCameraMatchComplete
// Handle update from the vision system.  Response to WM_ROBOT_CAMERA_MATCH_OBJECT
// wParam: bObjectFound, lParam: ObjectID
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::HandleCameraMatchComplete( WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (lParam);
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleCameraMatchComplete);

	BOOL bObjectFound = (BOOL)wParam;
	int  ObjectID = (int)lParam;
	
	if( bObjectFound )
	{
		// Move head back to forward 
		HeadCenter(); // Look Forward

		// If object found, look up in database and say something about it. TODO 
		CString TextToSay;
		TextToSay.Format( "This is %s",  g_ObjectName);	
		SpeakText( TextToSay );

/*  NOT IMPLEMENTED SINCE MOVED TO OPENCV 2 - TODO
		if( NULL != g_pObjectKnowledge )
		{
			ObjectInfoList* pObjectInfoList = g_pObjectKnowledge->GetObjectInfoList();
			ObjectInfo *pObjectInfo = pObjectInfoList->GetAt( ObjectPos );
			CString MsgString;
			MsgString.Format( "BEHAVIOR MODULE: OBJECT MATCH! [%s]", pObjectInfo->strObjectName );
			g_PostStatus( (LPCTSTR)MsgString );
			g_PostStatus( "---------------------------" );

			CString TextToSay;
			TextToSay.Format( "This is a %s. %s", pObjectInfo->strObjectName, pObjectInfo->strComment );	
			SpeakText( TextToSay );
		}
		else
*/
		{
			ROBOT_ASSERT(0);
		}


		// Done!  If Arm was moving to examine object, Move arm home and reset Arm state machine back to idle
		HeadCenter(); // Look Forward
		gHeadIdle = TRUE;
		if( ARM_MOVEMENT_IDENTIFY_OBJECT == m_ArmMovementLeft )
		{
			LeftArmHome();
		}
		if( ARM_MOVEMENT_IDENTIFY_OBJECT == m_ArmMovementRight )
		{
			RightArmHome();
		}
	}
	else
	{
		// Object not found	// ***TODO!! Need to review all this code!


		if( ARM_MOVEMENT_IDENTIFY_OBJECT == m_ArmMovementRight )
		{
			// Robot has object in hand, and is examining it.

/*** TODO, after optimizing.  for now don't rotate hand.
			if( 3 == m_ArmMovementStateRight )
			{						
				// Failed first test, now turn wrist 90 degrees and try again
				m_pArmControlRight->SetArmPosition(NOP, NOP, NOP, 60, NOP ); // Shoulder, ElbowRotate, ElbowBend, Wrist, Grip
				m_pArmControlRight->ExecutePosition();
				m_ArmMovementStateRight++;	// go to next state
				int RandomNumber = ((3 * rand()) / RAND_MAX);
				switch( RandomNumber ) // Respond with random phrases
				{
					case 0:  SpeakText( "I will try a different view angle" );break;
					case 1:  SpeakText( "Maybe if I rotate it" );break;
					default:  SpeakText( "I'm not recognizing this" );break;
				}


				// this will be picked up by the Servo Status complete code below...
				break;
			}
			else
***/
			{
				// Failed second test
				m_ArmMovementRight = ARM_MOVEMENT_NONE;
				int RandomNumber = ((5 * rand()) / RAND_MAX);
				switch( RandomNumber ) // Respond with random phrases
				{

					case 0:  SpeakText( "This is nice" );break;
					case 1:  SpeakText( "Thanks, I have been looking for this" );break;
					default:  SpeakText( "Honestly, I have no idea what this is. But thanks." );break;
				
/**				
					case 0:  SpeakText( "I do not recognize this" );break;
					case 1:  SpeakText( "I can't find this in my data banks" );break;
					case 2:  SpeakText( "What is it?" );break;
					case 3:  SpeakText( "I don't know what this is" );break;
					default:  SpeakText( "What is this?" );break;
**/
				}

				// Done!  If Arm was moving to examine object, Move arm home and reset Arm state machine back to idle
				HeadCenter(); // Look Forward
				gHeadIdle = TRUE;
				if( ARM_MOVEMENT_IDENTIFY_OBJECT == m_ArmMovementLeft )
				{
					LeftArmHome();
				}
				if( ARM_MOVEMENT_IDENTIFY_OBJECT == m_ArmMovementRight )
				{
					RightArmHome();
				}
			}
		}
	}
	__itt_task_end(pDomainControlThread);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleArmServoStatusUpdate
// Handle status update from the servos.
// Check Arm Movement State Machine.  Only used with movements that have multiple steps
// Uses m_ArmMovementStateRight and m_ArmMovementStateLeft to track progress
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CBehaviorModule::HandleArmServoStatusUpdate( WPARAM wParam, LPARAM lParam )
{
	IGNORE_UNUSED_PARAM (wParam);
	IGNORE_UNUSED_PARAM (lParam);
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshHandleArmServoStatusUpdate);

	HandleArmServoStatusUpdateLeft( wParam, lParam );
	HandleArmServoStatusUpdateRight( wParam, lParam );

	__itt_task_end(pDomainControlThread);
}




#endif // ROBOT_SERVER
