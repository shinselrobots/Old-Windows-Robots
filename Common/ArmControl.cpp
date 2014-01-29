// ArmControl.cpp: Arm Control class
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "ArmControl.h"

#define DEBUG_LEFT_ARM_CALIBRATION			   1   // When enabled, shows current calcuated XYZ position of Left finger tips

#define IDLE_ARM_MOVEMENT_DEGREES			   3	// +/- Degrees - Amount joints move during idle
#define IDLE_ARM_MOVEMENT_FREQUENCY			3000	// Miliseconds - Time to wait since last move before idle movement.
#define SERVO_STALL_TIME_LIMIT				  20 	// Tenth Seconds - Time to wait before declairing that a servo is stalled
#define SERVO_STALL_TOLERANCE_TENTH_DEGREES	  10	// Number of degrees movement between samples for detecting stalls

// Arm length measurements (for Kinematics)
#define SHOULDER_BONE_LEN_TENTH_INCHES							165.0	// Tenth_Inches
#define FOREARM_BONE_LEN_TENTH_INCHES_L							147.5	// Tenth_Inches
#define FOREARM_BONE_LEN_TENTH_INCHES_R							132.5	// Tenth_Inches
#define SHOULDER_JOINT_HEIGHT_ABOVE_GROUND_TENTH_INCHES_Z		277.0	// Tenth_Inches
#define SHOULDER_JOINT_DIST_FROM_ROBOT_CENTER_TENTH_INCHES_X	 95.0	// Tenth_Inches
#define SHOULDER_JOINT_DIST_FROM_ROBOT_FRONT_TENTH_INCHES_Y		 80.0	// Tenth_Inches
#define ELBOW_ROTATE_OFFSET_L									  0.5	// Degrees
#define ELBOW_ROTATE_OFFSET_R									  0.0	// Degrees

enum SERVO_STATUS_T {			
		SERVO_IDLE = 0,		// Servo not receiving a command
		SERVO_MOVING,		// Servo in the process of actually moving
		SERVO_STALL,		// Servo has momentarily stalled
		SERVO_ABORT,		// Servo stuck at current position, time to abort the move	
		SERVO_IN_POSITION	// Servo sucessfully reached target position
};
 

__itt_string_handle* pshMoveArmHome = __itt_string_handle_create("MoveArmHome");
__itt_string_handle* pshSetArmPosition = __itt_string_handle_create("SetArmPosition");
__itt_string_handle* pshSetArmSpeed = __itt_string_handle_create("SetArmSpeed");
__itt_string_handle* pshExecutePosition = __itt_string_handle_create("ExecutePosition");
__itt_string_handle* pshExecutePositionAndSpeed = __itt_string_handle_create("ExecutePositionAndSpeed");
__itt_string_handle* pshGetTargetArmXYZ = __itt_string_handle_create("GetTargetArmXYZ");
__itt_string_handle* pshGetCurrentArmXYZ = __itt_string_handle_create("GetCurrentArmXYZ");
__itt_string_handle* pshCalculateArmXYZ = __itt_string_handle_create("CalculateArmXYZ");
__itt_string_handle* pshCalculateArmMoveToXYZ = __itt_string_handle_create("CalculateArmMoveToXYZ");
__itt_string_handle* pshCheckArmPosition = __itt_string_handle_create("CheckArmPosition");
__itt_string_handle* pshGetClawTorque = __itt_string_handle_create("GetClawTorque");
__itt_string_handle* pshGetArmPosition = __itt_string_handle_create("GetArmPosition");
__itt_string_handle* pshCheckServoLimit = __itt_string_handle_create("CheckServoLimit");


///////////////////////////////////////////////////////////////////////////////
// ArmControl


ArmControl::ArmControl( int  ArmNumber )
{
	m_ArmNumber = ArmNumber;
	m_ClawTorque = DYNA_TORQUE_LIMIT_MAX;
	m_LastMovementTime = GetTickCount();
	m_ArmMoveTimeLimit = DEFAULT_ARM_MOVE_TIME_LIMIT_TENTH_SECONDS;	// Default timeout

	m_LastShoulder = MAX_INT;
	m_LastElbowRotate = MAX_INT;
	m_LastElbowBend = MAX_INT;
	m_LastWrist = MAX_INT;
	m_LastClaw = MAX_INT;
	m_PressureSensorCalibrationL = 0;
	m_PressureSensorCalibrationR = 0;


	if(RIGHT_ARM == m_ArmNumber )
	{
		m_ShoulderServoID =		KERR_RIGHT_ARM_SHOULDER_SERVO_ID;
		m_ElbowRotateServoID =	DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID;
		m_ElbowBendServoID =	DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID;
		m_WristServoID =		DYNA_RIGHT_ARM_WRIST_SERVO_ID;
		m_ClawServoID =			DYNA_RIGHT_ARM_CLAW_SERVO_ID;
	}
	else if(LEFT_ARM == m_ArmNumber )
	{
		m_ShoulderServoID =		KERR_LEFT_ARM_SHOULDER_SERVO_ID;
		m_ElbowRotateServoID =	DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID;
		m_ElbowBendServoID =	DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID;
		m_WristServoID =		DYNA_LEFT_ARM_WRIST_SERVO_ID;
		m_ClawServoID =			DYNA_LEFT_ARM_CLAW_SERVO_ID;
	}
	else
	{
		ROBOT_ASSERT(0); // BAD VALUE!
	}

}

ArmControl::~ArmControl()
{
	// Release resources
	ROBOT_LOG( TRUE,"~ArmControl done\n")
}



BOOL ArmControl::IsObjectInPickupZone( int ObjectTenthInchesY, int ObjectTenthInchesX )
{
	if(LEFT_ARM != m_ArmNumber )
	{
		return FALSE; // TODO: Right Arm NOT IMPLEMENTED!
	}

	if( (ObjectTenthInchesY <= OBJECT_PICKUP_LEFT_MAX_Y) && // Distance to object in TenthInches
		(ObjectTenthInchesX < OBJECT_PICKUP_LEFT_MAX_X) &&  // Toward Center of robot on left (negative values)
		(ObjectTenthInchesX > OBJECT_PICKUP_LEFT_MIN_X) )	// Toward outside of robot on left (negative values)
	{
		ROBOT_LOG( TRUE,"Object is in Pickup Zone\n")
		return TRUE;
	}

	ROBOT_LOG( TRUE,"Object not in Pickup Zone: Y = %d, X = %d\n", ObjectTenthInchesY, ObjectTenthInchesX)
	return FALSE;
}


//-----------------------------------------------------------------------------
// Name: Move Arm Home
// Desc: quickly sets given arm to Home position.
// Sets Speed too, if optional parameter set
// NOTE!  Calls execute_position Too!
//-----------------------------------------------------------------------------
void ArmControl::MoveArmHome( int  Speed )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshMoveArmHome);

	if( SERVO_NO_CHANGE != Speed )	// Optional Parameter
	{
		SetArmSpeed( Speed, Speed, Speed, Speed, Speed );	
	}

	if(RIGHT_ARM == m_ArmNumber )
	{
		if( GetClawTorque() < CLAW_TORQUE_DETECT_OBJECT ) 
		{
			// Not holding anyting.  Move claw back to home position too
			SetArmPosition(
				RIGHT_ARM_SHOULDER_HOME1, RIGHT_ARM_ELBOW_ROTATE_HOME1, RIGHT_ARM_ELBOW_BEND_HOME1, 
				RIGHT_ARM_WRIST_ROTATE_HOME1, RIGHT_ARM_CLAW_HOME1 );
			SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
		}
		else
		{
			// Something in hand.  Leave claw alone (unless it's open all the way, then it hits the body)
			if(	g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees > 400 ) // TenthDegrees!)
			{
	  			SetArmPosition(
					RIGHT_ARM_SHOULDER_HOME1, RIGHT_ARM_ELBOW_ROTATE_HOME1, RIGHT_ARM_ELBOW_BEND_HOME1, 
					RIGHT_ARM_WRIST_ROTATE_HOME1, 40 );
			}
			else
			{
	  			SetArmPosition(
					RIGHT_ARM_SHOULDER_HOME1, RIGHT_ARM_ELBOW_ROTATE_HOME1, RIGHT_ARM_ELBOW_BEND_HOME1, 
					RIGHT_ARM_WRIST_ROTATE_HOME1, NOP );
			}
		}

	}
	else if(LEFT_ARM == m_ArmNumber )
	{
		if( GetClawTorque() < CLAW_TORQUE_DETECT_OBJECT ) 
		{
			// Not holding anyting.  Move claw back to home position too
			SetArmPosition(
				LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, LEFT_ARM_ELBOW_BEND_HOME1, 
				LEFT_ARM_WRIST_ROTATE_HOME1, LEFT_ARM_CLAW_HOME1 ); 
			SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
		}
		else
		{
			// Something in hand.  Leave claw alone.
			SetArmPosition(
				LEFT_ARM_SHOULDER_HOME1, LEFT_ARM_ELBOW_ROTATE_HOME1, LEFT_ARM_ELBOW_BEND_HOME1, 
				LEFT_ARM_WRIST_ROTATE_HOME1, NOP ); 
		}

	}
	else
	{
		ROBOT_ASSERT(0);
	}

	ExecutePositionAndSpeed();
	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: Move Arm to Safe Position
// Desc: quickly sets given arm to "Home2", or "safe" position, to avoid hitting objects
// Sets Speed too, if optional parameter set
// NOTE!  Calls execute_position Too!
//-----------------------------------------------------------------------------
void ArmControl::MoveArmToSafePosition( int  Speed )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshMoveArmHome);

	if( SERVO_NO_CHANGE != Speed )	// Optional Parameter
	{
		SetArmSpeed( Speed, Speed, Speed, Speed, Speed );	
	}

	if(RIGHT_ARM == m_ArmNumber )
	{
		if( GetClawTorque() < CLAW_TORQUE_DETECT_OBJECT ) 
		{
			// Not holding anyting.  Move claw back to home position too
			SetArmPosition(
				RIGHT_ARM_SHOULDER_HOME2, RIGHT_ARM_ELBOW_ROTATE_HOME2, RIGHT_ARM_ELBOW_BEND_HOME2, 
				RIGHT_ARM_WRIST_ROTATE_HOME2, RIGHT_ARM_CLAW_HOME2 );
			SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
		}
		else
		{
			// Something in hand.  Leave claw alone (unless it's open all the way, then it hits the body)
			if(	g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees > 400 ) // TenthDegrees!)
			{
	  			SetArmPosition(
					RIGHT_ARM_SHOULDER_HOME2, RIGHT_ARM_ELBOW_ROTATE_HOME2, RIGHT_ARM_ELBOW_BEND_HOME2, 
					RIGHT_ARM_WRIST_ROTATE_HOME2, 40 );
			}
			else
			{
	  			SetArmPosition(
					RIGHT_ARM_SHOULDER_HOME2, RIGHT_ARM_ELBOW_ROTATE_HOME2, RIGHT_ARM_ELBOW_BEND_HOME2, 
					RIGHT_ARM_WRIST_ROTATE_HOME2, NOP );
			}
		}

	}
	else if(LEFT_ARM == m_ArmNumber )
	{
		if( GetClawTorque() < CLAW_TORQUE_DETECT_OBJECT ) 
		{
			// Not holding anyting.  Move claw back to home position too
			SetArmPosition(
				LEFT_ARM_SHOULDER_HOME2, LEFT_ARM_ELBOW_ROTATE_HOME2, LEFT_ARM_ELBOW_BEND_HOME2, 
				LEFT_ARM_WRIST_ROTATE_HOME2, LEFT_ARM_CLAW_HOME2 ); 
			SetClawTorque(GENTLE_TORQUE);	// reset to gentle torque
		}
		else
		{
			// Something in hand.  Leave claw alone.
			SetArmPosition(
				LEFT_ARM_SHOULDER_HOME2, LEFT_ARM_ELBOW_ROTATE_HOME2, LEFT_ARM_ELBOW_BEND_HOME2, 
				LEFT_ARM_WRIST_ROTATE_HOME2, NOP ); 
		}

	}
	else
	{
		ROBOT_ASSERT(0);
	}

	ExecutePositionAndSpeed();
	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: Set Arm Position
// Desc: quickly sets precise position of the given arm in DEGREES
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void ArmControl::SetArmPosition(int Shoulder, int ElbowRotate, int ElbowBend, int Wrist, int Claw )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshSetArmPosition);

//	ROBOT_LOG( TRUE,"SetArmPosition:  Shoulder = %d, ElbowRotate = %d, ElbowBend = %d, Wrist = %d, Claw = %d  degrees (MAX=NOP)\n",
//		 Shoulder, ElbowRotate, ElbowBend, Wrist, Claw )

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csServoLock);
	EnterCriticalSection(&g_csServoLock);

	if( SERVO_NO_CHANGE != Shoulder )
	{
		Shoulder *= 10;	// Convert to TenthDegrees
		CheckServoLimit(m_ShoulderServoID, Shoulder);
		g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees = Shoulder;
		g_BulkServoCmd[m_ShoulderServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != ElbowRotate )
	{
		ElbowRotate *= 10;	// Convert to TenthDegrees
		CheckServoLimit(m_ElbowRotateServoID, ElbowRotate);
		g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees = ElbowRotate;
		g_BulkServoCmd[m_ElbowRotateServoID].Update = TRUE;

	}
	if( SERVO_NO_CHANGE != ElbowBend )
	{
		ElbowBend *= 10;	// Convert to TenthDegrees
		CheckServoLimit(m_ElbowBendServoID, ElbowBend);
		g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees = ElbowBend;
		g_BulkServoCmd[m_ElbowBendServoID].Update = TRUE;

	}
	if( SERVO_NO_CHANGE != Wrist )
	{
		Wrist *= 10;	// Convert to TenthDegrees
		CheckServoLimit(m_WristServoID, Wrist);
		g_BulkServoCmd[m_WristServoID].PositionTenthDegrees = Wrist;
		g_BulkServoCmd[m_WristServoID].Update = TRUE;

	}
	if( SERVO_NO_CHANGE != Claw )
	{
		Claw *= 10;	// Convert to TenthDegrees
		CheckServoLimit(m_ClawServoID, Claw);
		g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees = Claw;
		g_BulkServoCmd[m_ClawServoID].Update = TRUE;
	}
	LeaveCriticalSection(&g_csServoLock);
	__itt_task_end(pDomainControlThread);

	// If arm moving, assume we are no longer in home position.
	// will be updated with next status update
	gArmInHomePositionRight = FALSE;	// Arm is not in home position
	gArmInHomePositionLeft = FALSE;	// Arm is not in home position

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: Set Arm Speed
// Desc: Sets speed for all servos (each can be a different speed)
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void ArmControl::SetArmSpeed(int  Shoulder, int  ElbowRotate, int  ElbowBend, int  Wrist, int  Claw )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshSetArmSpeed);

	if( SERVO_NO_CHANGE != Shoulder )
	{
		g_BulkServoCmd[m_ShoulderServoID].Speed = Shoulder;
		g_BulkServoCmd[m_ShoulderServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != ElbowRotate )
	{
		g_BulkServoCmd[m_ElbowRotateServoID].Speed = ElbowRotate;
		g_BulkServoCmd[m_ElbowRotateServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != ElbowBend )
	{
		g_BulkServoCmd[m_ElbowBendServoID].Speed = ElbowBend;
		g_BulkServoCmd[m_ElbowBendServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != Wrist )
	{
		g_BulkServoCmd[m_WristServoID].Speed = Wrist;
		g_BulkServoCmd[m_WristServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != Claw )
	{
		g_BulkServoCmd[m_ClawServoID].Speed = Claw;
		g_BulkServoCmd[m_ClawServoID].Update = TRUE;
	}

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: Set Arm Delay
// Desc: Sets delay for all servos (each can be a different delay)
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void ArmControl::SetArmDelay(int  Shoulder, int  ElbowRotate, int  ElbowBend, int  Wrist, int  Claw )
{
	if( SERVO_NO_CHANGE != Shoulder )
	{
		g_BulkServoCmd[m_ShoulderServoID].Delay = Shoulder;
	}
	if( SERVO_NO_CHANGE != ElbowRotate )
	{
		g_BulkServoCmd[m_ElbowRotateServoID].Delay = ElbowRotate;
	}
	if( SERVO_NO_CHANGE != ElbowBend )
	{
		g_BulkServoCmd[m_ElbowBendServoID].Delay = ElbowBend;
	}
	if( SERVO_NO_CHANGE != Wrist )
	{
		g_BulkServoCmd[m_WristServoID].Delay = Wrist;
	}
	if( SERVO_NO_CHANGE != Claw )
	{
		g_BulkServoCmd[m_ClawServoID].Delay = Claw;
	}
}


//-----------------------------------------------------------------------------
// Name: SetRightClawTorque
// Desc: Sets Claw Torque value
//-----------------------------------------------------------------------------
void ArmControl::SetClawTorque( int  Torque )
{

	if( Torque > DYNA_TORQUE_LIMIT_MAX )
	{
		ROBOT_DISPLAY( TRUE, "ERROR! DYNA_RIGHT_ARM_CLAW exceeds DYNA_TORQUE_LIMIT_MAX limit" )
		Torque = DYNA_TORQUE_LIMIT_MAX;
	}
	m_ClawTorque = Torque;
	SendCommand( WM_ROBOT_SET_SERVO_TORQUE_LIMIT, DYNA_RIGHT_ARM_CLAW_SERVO_ID, m_ClawTorque );	// Set Torque: 0 - 1023 DYNA_TORQUE_LIMIT_MAX
}

//-----------------------------------------------------------------------------
// Name: ClawRegripObject
// Desc: After grabbing an object, reset the servo to hold it firmly, 
//       but not overload the servo
//-----------------------------------------------------------------------------
void ArmControl::ClawRegripObject( )
{
	// Current commanded position might be much tigher than physical condition, which can eventually overheat the servo
	// Get current position, plus a little less, and make that the commanded position
	// Note: leaves torque at prior value, which should normally be DYNA_TORQUE_LIMIT_MAX to assure a strong grip
	// All values in DEGREES
	int Torque = abs( g_BulkServoStatus[m_ClawServoID].Load );
	if( Torque > 200 )
	{
		int OldCommand = (g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees) / 10;
		int CurrentPosition = GetClawPosition();
		int NewClawPosition = CurrentPosition;	// lessen load
		SetArmPosition( NOP, NOP, NOP, NOP, NewClawPosition ); // Set Claw position
		ExecutePosition();
		ROBOT_LOG( TRUE,"ClawRegripObject: OldCommand = %d, CurrentPosition = %d, NewPosition = %d degrees \n",
			OldCommand, CurrentPosition, NewClawPosition)
	}
	else
	{
		ROBOT_LOG( TRUE,"ClawRegripObject: No Regrip needed.  Torque = %d\n", Torque )
	}
}


void ArmControl::ClearServoStallTimers()
{
	if(LEFT_ARM == m_ArmNumber )
	{
		g_BulkServoStatus[KERR_LEFT_ARM_SHOULDER_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_LEFT_ARM_WRIST_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_LEFT_ARM_CLAW_SERVO_ID].StallTimer = TIMER_NOT_SET;
	}
	else if(RIGHT_ARM == m_ArmNumber )
	{
		g_BulkServoStatus[KERR_RIGHT_ARM_SHOULDER_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_RIGHT_ARM_WRIST_SERVO_ID].StallTimer = TIMER_NOT_SET;
		g_BulkServoStatus[DYNA_RIGHT_ARM_CLAW_SERVO_ID].StallTimer = TIMER_NOT_SET;
	}
	else ROBOT_ASSERT(0);

}


//-----------------------------------------------------------------------------
// Name: ExecutePosition
// Desc: Commits movement for all servos setup previously
//-----------------------------------------------------------------------------
void ArmControl::ExecutePosition()
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshExecutePosition);
	//ROBOT_DISPLAY( TRUE, "DEBUG: ExecutePosition\n" )

	// Clear the stall timers
	ClearServoStallTimers();

	SendCommand( WM_ROBOT_SET_ARM_POSITION, m_ArmNumber, FALSE );	// FALSE = Don't Set Speed
	m_LastMovementTime = GetTickCount();

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: ExecutePositionAndSpeed
// Desc: Commits movement and speed for all servos setup previously
//-----------------------------------------------------------------------------
void ArmControl::ExecutePositionAndSpeed()
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshExecutePositionAndSpeed);
	//ROBOT_DISPLAY( TRUE, "DEBUG: ExecutePositionAndSpeed\n" )

	// Handle Global Pause for debugging
	if( g_GlobalPause )
	{
		// Global Pause requested!
		// for Arm control, this just blocks further servo commands until unpaused (freeze at last commanded position)
		return;
	}

	// Clear the stall timers
	ClearServoStallTimers();


	SendCommand( WM_ROBOT_SET_ARM_POSITION, m_ArmNumber, TRUE );	// TRUE = Set Speed
	m_LastMovementTime = GetTickCount();

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: SetServoTimeout
// Desc: Set max time move will wait to reach commanded position before timing out
// This prevents the robot from hanging if a servo can't reach commanded position
//-----------------------------------------------------------------------------
void ArmControl::SetServoTimeout( int  nOwner, int  Timeout )
{
	//IGNORE_UNUSED_PARAM (nOwner);
	m_ArmMoveTimeLimit = Timeout;
}



//-----------------------------------------------------------------------------
// Name: GetTargetArmXYZ, GetCurrentArmXYZ, CalculateArmXYZ
// Desc: Return current or target X,Y,Z position of Finger Tip
// X= Side to side from robot center, Y= distance from front of robot, Z = height above ground in TENTH INCHES
// If nObjectDistance is non-zero, return calculated position of object detected by hand sensors
//-----------------------------------------------------------------------------
void ArmControl::GetTargetArmXYZ( FPOINT3D_T &ArmXYZ, double nObjectDistance )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGetTargetArmXYZ);
	// Target position info was requested
	// ArmXYZ returns X,Y,Z, in Floating Point TENTH INCHES

	ARM_SERVOS_POSITION_T Servo;

	Servo.ShoulderAngle = (g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees / 10.0);
	Servo.ElbowBendAngle = (g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees / 10.0);
	Servo.ElbowRotateAngle = (g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees / 10.0);

	//ROBOT_LOG( TRUE,"GET TARGET: ")
	CalculateArmXYZ( ArmXYZ, Servo, nObjectDistance );

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
void ArmControl::GetCurrentArmXYZ( FPOINT3D_T &ArmXYZ, double nObjectDistance )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGetCurrentArmXYZ);
	// Current position info was requested
	// ArmXYZ returns X,Y,Z, in Floating Point TENTH INCHES

	ARM_SERVOS_POSITION_T MyServo;

	MyServo.ShoulderAngle = (g_BulkServoStatus[m_ShoulderServoID].PositionTenthDegrees / 10.0);
	MyServo.ElbowBendAngle = (g_BulkServoStatus[m_ElbowBendServoID].PositionTenthDegrees / 10.0);
	MyServo.ElbowRotateAngle = (g_BulkServoStatus[m_ElbowRotateServoID].PositionTenthDegrees / 10.0);

	//ROBOT_LOG( TRUE,"GET CURRENT: ")
	CalculateArmXYZ( ArmXYZ, MyServo, nObjectDistance );

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
void ArmControl::CalculateArmXYZ(FPOINT3D_T &ArmXYZ, ARM_SERVOS_POSITION_T Servo, double nObjectDistance )

{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCalculateArmXYZ);
	// ArmXYZ returns X,Y,Z, in Floating Point TENTH INCHES
	ArmXYZ.X = 0.0;
	ArmXYZ.Y = 0.0;
	ArmXYZ.Z = 0.0;

	#if (DEBUG_LEFT_ARM_CALIBRATION == 1)
		if(LEFT_ARM == m_ArmNumber )
		{
			ROBOT_LOG( TRUE, "CalculateArmXYZ from Servo values (real or future): SERVO Values:  LEFT ARM = Shoulder=%0.2f, ElbowBend=%0.2f, ElbowRotate=%0.2f  DEGREES\n",
				Servo.ShoulderAngle,  Servo.ElbowBendAngle, Servo.ElbowRotateAngle  )
		}
	#endif

	// Tune arm info for accuracy
	double ForearmBoneLen = 0;
	double ShoulderBoneLen = 0;
	
	if( LEFT_ARM == m_ArmNumber )
	{
		ForearmBoneLen = FOREARM_BONE_LEN_TENTH_INCHES_L;
		ShoulderBoneLen = SHOULDER_BONE_LEN_TENTH_INCHES;
		//Servo.ElbowBendAngle -= 2.5;
		Servo.ElbowRotateAngle -= 7.0; //ELBOW_ROTATE_OFFSET_L
		//Servo.ShoulderAngle += 0;
	}
	else if( RIGHT_ARM == m_ArmNumber )
	{	
		ForearmBoneLen = FOREARM_BONE_LEN_TENTH_INCHES_R;
		ShoulderBoneLen = SHOULDER_BONE_LEN_TENTH_INCHES;
		//Servo.ElbowBendAngle += 0;
		//Servo.ElbowRotateAngle += 0; //ELBOW_ROTATE_OFFSET_L
		//Servo.ShoulderAngle += 0;
	}
	else ROBOT_ASSERT(0);



	// Initially, use Elbow Joint as the Origin
	// Assume all servo at 0,0,0, arm pointing stright toward the ground (and below it!)

	double ElbowVectorLen = sin(DEGREES_TO_RADIANS *Servo.ElbowBendAngle) * (ForearmBoneLen + nObjectDistance);
	double Z =  cos(DEGREES_TO_RADIANS*Servo.ElbowBendAngle) * (ForearmBoneLen + nObjectDistance) * -1.0;
	double Y =  cos(DEGREES_TO_RADIANS*Servo.ElbowRotateAngle) * ElbowVectorLen;
	double X =  sin(DEGREES_TO_RADIANS*Servo.ElbowRotateAngle) * ElbowVectorLen;

	//ROBOT_LOG( TRUE, "DEBUG FIRST XYZ:  X=%0.2f, Y=%0.2f, Z=%0.2f \n", X / 10.0, Y / 10.0, Z / 10.0 )

	// Translate the Origin to be at the Shoulder
	Z -= ShoulderBoneLen;

	// Do Shoulder Calculations to enable shoulder rotation
	// Create vector from shoulder to finger tip in the Y,Z plane
	double ShoulderVectorLen = sqrt( Y*Y + Z*Z );
	double ShoulderVectorAngle = atan2( Y, Z ) * -RADIANS_TO_DEGREES;	// from shoulder to tip of finger.  Note (X,Y) ==> (Y/X)
	
	// Add shoulder rotation
	ShoulderVectorAngle += Servo.ShoulderAngle;

	// Convert back to Y,Z (x constant)
	ArmXYZ.X = X;	// shoulder rotate has no effect on X
	ArmXYZ.Y = sin(DEGREES_TO_RADIANS*ShoulderVectorAngle) * ShoulderVectorLen * -1.0;
	ArmXYZ.Z = cos(DEGREES_TO_RADIANS*ShoulderVectorAngle) * ShoulderVectorLen;

	// Convert to Real World
	ArmXYZ.Z += (double)SHOULDER_JOINT_HEIGHT_ABOVE_GROUND_TENTH_INCHES_Z;

	// Calculate position of Elbow
	double ElbowZ =  cos(DEGREES_TO_RADIANS*Servo.ShoulderAngle) * ShoulderBoneLen * -1.0;
	double ElbowY =  sin(DEGREES_TO_RADIANS*Servo.ShoulderAngle) * ShoulderBoneLen;
	ElbowZ += SHOULDER_JOINT_HEIGHT_ABOVE_GROUND_TENTH_INCHES_Z;
	ElbowY -= SHOULDER_JOINT_DIST_FROM_ROBOT_FRONT_TENTH_INCHES_Y;

	// Position relative to Robot Front Center (TODO - change to center of Robot (between drive wheels)
	// TODO - compensate Left and Right Arms differently, and update X,Y,Z as needed (probably just X?)
	// if(RIGHT_ARM == m_ArmNumber ) ...
	if(LEFT_ARM == m_ArmNumber )
	{
		ArmXYZ.X -= SHOULDER_JOINT_DIST_FROM_ROBOT_CENTER_TENTH_INCHES_X;
	}
	else if(RIGHT_ARM == m_ArmNumber )
	{
		ArmXYZ.X += SHOULDER_JOINT_DIST_FROM_ROBOT_CENTER_TENTH_INCHES_X;
	}

	ArmXYZ.Y -= SHOULDER_JOINT_DIST_FROM_ROBOT_FRONT_TENTH_INCHES_Y;

	if(LEFT_ARM == m_ArmNumber )
	{
//		ArmXYZ.X *= -1;  // Left Arm X values are in the Negative direction
	}

	#if (DEBUG_LEFT_ARM_CALIBRATION == 1)

		if(LEFT_ARM == m_ArmNumber )
		{
			ROBOT_LOG( TRUE, "DEBUG ARM XYZ:  X=%0.2f, Y=%0.2f, Z=%0.2f \n", ArmXYZ.X / 10.0, ArmXYZ.Y / 10.0, ArmXYZ.Z / 10.0 )
			ROBOT_LOG( TRUE,"ELBOW: Y=%0.2f, Z=%0.2f\n", ElbowY, ElbowZ )
		}
	#endif

	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: CalculateArmMoveToXYZ
// Desc: Return servo values needed to move arm Finger Tip to target X,Y,Z position
// X= Side to side from robot center, Y= distance from front of robot, Z = height above ground
//-----------------------------------------------------------------------------
#define TOLERANCE_X		2.0	 // Tenthinches
#define TOLERANCE_Y		2.0	 // Tenthinches
#define TOLERANCE_Z		2.0	 // Tenthinches
#define SERVO_STEP		0.50 // Degrees

// keep values in reasonable limits
#define SERVO_LIMIT_SHOULDER_MIN	  (-30.0)	// Degrees
#define SERVO_LIMIT_SHOULDER_MAX	    30.0 

#define SERVO_LIMIT_ELBOW_BEND_MIN		 0.0
#define SERVO_LIMIT_ELBOW_BEND_MAX	    40.0

#define SERVO_LIMIT_ELBOW_ROTATE_MIN  (-40.0)
#define SERVO_LIMIT_ELBOW_ROTATE_MAX    40.0



BOOL ArmControl::CalculateArmMoveToXYZ(FPOINT3D_T &TargetXYZ, ARM_SERVOS_POSITION_T &Servo )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCalculateArmMoveToXYZ);
	// Iterate to find optimal position
	// Uses "virtual arm" movements to iterate

	FPOINT3D_T ArmXYZ;
	BOOL bFloorOnly = TRUE;  // WARNING - FOR NOW, ONLY VALID FOR OBJECTS ON FLOOR

	// Get current servo position as a starting point
	Servo.ShoulderAngle = ((float)g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees / 10.0);
	Servo.ElbowBendAngle = ((float)g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees / 10.0);
	Servo.ElbowRotateAngle = ((float)g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees / 10.0);
	CalculateArmXYZ( ArmXYZ, Servo );

	ROBOT_LOG( TRUE, "\nDEBUG:  ***** TARGET ARM XYZ:  X=%0.2f, Y=%0.2f, Z=%0.2f  *****\n\n", TargetXYZ.X / 10.0, TargetXYZ.Y / 10.0, TargetXYZ.Z / 10.0 )


	// EVERYTHING IN TENTH INCHES!!!
	if( bFloorOnly && (TargetXYZ.Z > 50.0) )  // 5 inches is too tall!
	{
		// WARNING - FOR NOW, ONLY VALID FOR OBJECTS ON FLOOR
		ROBOT_ASSERT(0);
		__itt_task_end(pDomainControlThread);
		return FALSE; // returns same position that arm in currently in, just in case, to avoid breaking arm
	}

	// For now, set fixed Z value for the pick up
	//TargetXYZ.Z = 10.0; //TenthInches - FUDGE KLUDGE - Pickup Z HARDCODED Here!

	// Tune for overshoot or undershoot the object
	// (Combination of arm mechanism and Kinect object spotter tolerance)
	TargetXYZ.Y -= 5.0; // TenthInches


	// Find initial Y position, with finger just above ground
	while( (ArmXYZ.Y > TargetXYZ.Y) || (ArmXYZ.Z < 10.0) || (ArmXYZ.Z > 20.0) )
	{
		// Arm too far forward, ahead of the object, or claw below the ground
		// move arm back until it meets requirements
		if( ArmXYZ.Y > TargetXYZ.Y )
		{
			Servo.ShoulderAngle -= 1.0; // degrees!
			if( Servo.ShoulderAngle < SERVO_LIMIT_SHOULDER_MIN ) 
				Servo.ShoulderAngle = SERVO_LIMIT_SHOULDER_MIN;
		}
		if( ArmXYZ.Z < 10.0 )
		{	// Too Low			
			Servo.ElbowBendAngle += 1.0; // Keep arm above ground
			if( Servo.ElbowBendAngle > SERVO_LIMIT_ELBOW_BEND_MAX ) 
				Servo.ElbowBendAngle = SERVO_LIMIT_ELBOW_BEND_MAX;
		}
		if( ArmXYZ.Z > 20.0 )
		{	// Too High			
			Servo.ElbowBendAngle -= 1.0; // Keep arm close to ground
			if( Servo.ElbowBendAngle < SERVO_LIMIT_ELBOW_BEND_MIN ) 
				Servo.ElbowBendAngle = SERVO_LIMIT_ELBOW_BEND_MIN;
		}

		CalculateArmXYZ( ArmXYZ, Servo );
	}
	ROBOT_LOG( TRUE, "DEBUG:  STARTING SOLUTION SERVOS:  Shoulder=%0.2f, ElbowBend=%0.2f, ElbowRotate=%0.2f \n",
		Servo.ShoulderAngle, Servo.ElbowBendAngle, Servo.ElbowRotateAngle )

	// OK, arm in reasonable starting position. Find pick up position
	int nLoopCount = 0;
	FPOINT3D_T LastArmXYZ;
	LastArmXYZ.X = -1000.0;
	LastArmXYZ.Y = -1000.0;
	LastArmXYZ.Z = -1000.0;
	BOOL FoundX, FoundY, FoundZ;
	for( nLoopCount=0; nLoopCount<10000; nLoopCount++ )
	{
		// Iterate Y
		FoundY = FALSE;
		if( ArmXYZ.Y < (TargetXYZ.Y-TOLERANCE_Y) )
		{
			Servo.ShoulderAngle += SERVO_STEP;
			if( Servo.ShoulderAngle > SERVO_LIMIT_SHOULDER_MAX ) 
				Servo.ShoulderAngle = SERVO_LIMIT_SHOULDER_MAX;
		}
		else if( (ArmXYZ.Y > TargetXYZ.Y+TOLERANCE_Y) )
		{
			Servo.ShoulderAngle -= SERVO_STEP;	// Overshot
			if( Servo.ShoulderAngle < SERVO_LIMIT_SHOULDER_MIN ) 
				Servo.ShoulderAngle = SERVO_LIMIT_SHOULDER_MIN;
		}
		else
		{
			FoundY = TRUE;
		}

	
		// Iterate Z
		FoundZ = FALSE;
		if( ArmXYZ.Z < (TargetXYZ.Z-TOLERANCE_Z) )
		{
			Servo.ElbowBendAngle += SERVO_STEP; // UP
			if( Servo.ElbowBendAngle > SERVO_LIMIT_ELBOW_BEND_MAX ) 
				Servo.ElbowBendAngle = SERVO_LIMIT_ELBOW_BEND_MAX;
		}
		else if( (ArmXYZ.Z > TargetXYZ.Z+TOLERANCE_Z) )
		{
			Servo.ElbowBendAngle -= SERVO_STEP; // DOWN
			if( Servo.ElbowBendAngle < SERVO_LIMIT_ELBOW_BEND_MIN ) 
				Servo.ElbowBendAngle = SERVO_LIMIT_ELBOW_BEND_MIN;
		}
		else
		{
			FoundZ = TRUE;
		}

		// Iterate X
		FoundX = FALSE;
		if( ArmXYZ.X < (TargetXYZ.X-TOLERANCE_X) )
		{
			Servo.ElbowRotateAngle += SERVO_STEP; //
			if( Servo.ElbowRotateAngle > SERVO_LIMIT_ELBOW_ROTATE_MAX ) 
				Servo.ElbowRotateAngle = SERVO_LIMIT_ELBOW_ROTATE_MAX;
		}
		else if( (ArmXYZ.X > TargetXYZ.X+TOLERANCE_X) )
		{
			Servo.ElbowRotateAngle -= SERVO_STEP; //
			if( Servo.ElbowRotateAngle > SERVO_LIMIT_ELBOW_ROTATE_MIN ) 
				Servo.ElbowRotateAngle = SERVO_LIMIT_ELBOW_ROTATE_MIN;
		}
		else
		{
			FoundX = TRUE;
		}

		// See if we found a solution
		if( FoundX && FoundY && FoundZ )
		{
			break; // found solution!
		}

		CalculateArmXYZ( ArmXYZ, Servo );
///		ROBOT_LOG( TRUE, "DEBUG SERVOS: Sho=%0.2f, ElbowBend=%0.2f, ElbowRotate=%0.2f \n", Servo.ShoulderAngle, Servo.ElbowBendAngle, Servo.ElbowRotateAngle )

		if( (LastArmXYZ.X == ArmXYZ.X)  && (LastArmXYZ.Y == ArmXYZ.Y) && (LastArmXYZ.Z == ArmXYZ.Z) )
		{
			ROBOT_LOG( TRUE,"***************** STUCK IN ITERATION LOOP *****************\n")
			return FALSE;
		}

		LastArmXYZ.X = ArmXYZ.X;
		LastArmXYZ.Y = ArmXYZ.Y;
		LastArmXYZ.Z = ArmXYZ.Z;
	}

	if( !(FoundX && FoundY && FoundZ) )
	{
		// No solution was found!
		ROBOT_LOG( TRUE, "CalculateArmMoveToXYZ FAILED!\n" )
		__itt_task_end(pDomainControlThread);
		return FALSE;
	}
	else
	{
		Servo.WristRotateAngle = (Servo.ElbowRotateAngle / 2.0);
		ROBOT_LOG( TRUE, "CalculateArmMoveToXYZ FOUND in %d steps!\n", nLoopCount )
		ROBOT_LOG( TRUE, "\nDEBUG:  ***** SOLUTION ARM XYZ:  X=%0.2f, Y=%0.2f, Z=%0.2f  *****\n", ArmXYZ.X / 10.0, ArmXYZ.Y / 10.0, ArmXYZ.Z / 10.0 )
		ROBOT_LOG( TRUE, "DEBUG:  ***** SOLUTION SERVOS:  Shoulder=%0.2f, ElbowBend=%0.2f, ElbowRotate=%0.2f  *****\n\n",
			Servo.ShoulderAngle, Servo.ElbowBendAngle, Servo.ElbowRotateAngle )
		__itt_task_end(pDomainControlThread);
		return TRUE;
	}
}



//-----------------------------------------------------------------------------
// Name: Check Servo Position
// Desc: Checks for Servo position and any stalls while moving into positon.  Used by CheckArmPosition()
// Returns SERVO_STATUS_T
//-----------------------------------------------------------------------------
int ArmControl::CheckServoPosition( const char *ServoName, int ServoID, int Delta, int &LastPosition, int ToleranceTenthDegrees )
{

	int Status = SERVO_MOVING;
	int CurrentPosition = g_BulkServoStatus[ServoID].PositionTenthDegrees;

	if( (int)(abs(Delta)) < ToleranceTenthDegrees )
	{
		// Servo in position
		Status = SERVO_IN_POSITION;
	}
	else
	{
		if( abs(LastPosition - CurrentPosition) > SERVO_STALL_TOLERANCE_TENTH_DEGREES )
		{
			// Not stalled
			//ROBOT_LOG( TRUE,"CheckServoPosition: %s moving, Delta = %d degrees\n", ServoName, Delta/10 )
			g_BulkServoStatus[ServoID].StallTimer = TIMER_NOT_SET;
			Status = SERVO_MOVING;
		}
		else
		{
			Status = SERVO_STALL;
			//ROBOT_LOG( TRUE,"CheckServoPosition: %s stall at position %d degrees, Last = %d, target = %d\n", 
			//	ServoName, CurrentPosition/10, LastPosition/10, g_BulkServoCmd[ServoID].PositionTenthDegrees / 10 )

			if( TIMER_NOT_SET == g_BulkServoStatus[ServoID].StallTimer )
			{
				// no timer started yet, so this is the first time a stall is detected
				g_BulkServoStatus[ServoID].StallTimer = SERVO_STALL_TIME_LIMIT;
				//ROBOT_LOG( TRUE, "DEBUG: Setting %s Stall Timer\n", ServoName )
			}
			else if( TIMER_SERVO_RESET == g_BulkServoStatus[ServoID].StallTimer ) // used to make sure we only send the command below once
			{
				Status = SERVO_ABORT;
			}
			else if( 0 == g_BulkServoStatus[ServoID].StallTimer )
			{
				Status = SERVO_ABORT;
				//ROBOT_LOG( TRUE," ********* CheckServoPosition:  %s STALL ABORT **********\n", ServoName)
				//ROBOT_LOG( TRUE,"********* CheckServoPosition:  %s STALLED:  RESETTING TARGET POSITION! **********\n")
				// Timer expired.  Set commanded position to current position (give up moving) to avoid overloading the servo
				//g_BulkServoStatus[ServoID].StallTimer = TIMER_SERVO_RESET; // only do the SendCommand once!
				//g_BulkServoCmd[ServoID].PositionTenthDegrees = g_BulkServoStatus[ServoID].PositionTenthDegrees;
				//SendCommand( WM_ROBOT_SET_ARM_POSITION, RIGHT_ARM, FALSE ); // TRUE = Set Speed
			}
			else
			{
				//ROBOT_LOG( TRUE, "DEBUG: %s Stall Timer = %d\n", ServoName, g_BulkServoStatus[ServoID].StallTimer )
			}
		}
	}

	// Done with stall tests.  Update servo's last position.
	LastPosition = CurrentPosition;

	return Status;

}


//-----------------------------------------------------------------------------
// Name: Check Arm Position
// Desc: Are we there yet?  Check to see if arm has reached commanded position
// Verbose flag will print out the servo that is blocking move complete status
// OPTIONAL parameters for joints will use supplied value if not SERVO_NO_CHANGE
//-----------------------------------------------------------------------------
#define ARM_MOVEMENT_DEBUG 0
BOOL ArmControl::CheckArmPosition(BOOL verbose, int  ToleranceTenthDegrees,
		int DesiredShoulderDegrees, int DesiredElbowRotateDegrees, int DesiredElbowBendDegrees, int DesiredWristDegrees, int DesiredClawDegrees )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCheckArmPosition);

///	HandleGlobalPause();

	// get a snapshot of current position (no lock required)
	int CurrentShoulder =		g_BulkServoStatus[m_ShoulderServoID].PositionTenthDegrees;
	int CurrentElbowRotate =	g_BulkServoStatus[m_ElbowRotateServoID].PositionTenthDegrees;
	int CurrentElbowBend =		g_BulkServoStatus[m_ElbowBendServoID].PositionTenthDegrees;
	int CurrentWrist =			g_BulkServoStatus[m_WristServoID].PositionTenthDegrees;
	int CurrentClaw =			g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees;

	int DeltaShoulder = 0;
	int DeltaElbowRotate = 0;
	int DeltaElbowBend = 0;
	int DeltaWrist = 0;
	int DeltaClaw = 0;


	if( ROBOT_SIMULATION_MODE )
	{

		__itt_task_end(pDomainControlThread);
		return TRUE;	// Handle simulation mode (no servos!)
	}

	if( LEFT_ARM == m_ArmNumber )
	{
		if(verbose) ROBOT_LOG( TRUE,"\nCheckArmPosition: LEFT ARM\n")
	}
	else
	{
		if(verbose) ROBOT_LOG( TRUE,"\nCheckArmPosition: RIGHT ARM\n")
	}

	/////////////////////////////////////////////////////////////////////////////
	// Find out the delta between what was requested and where each servo is now
	if( SERVO_NO_CHANGE == DesiredShoulderDegrees )
	{	// Normal mode - use the last commanded value for comparison
		DeltaShoulder = g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees - CurrentShoulder;
	}
	else
	{	// Special mode - use the supplied value for comparison
		DeltaShoulder = (DesiredShoulderDegrees * 10) - CurrentShoulder;
	}

	if( SERVO_NO_CHANGE == DesiredElbowRotateDegrees )
	{	// Normal mode - use the last commanded value for comparison
		DeltaElbowRotate = g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees - CurrentElbowRotate;
			g_BulkServoStatus[m_ElbowRotateServoID].PositionTenthDegrees;
	}
	else
	{	// Special mode - use the supplied value for comparison
		DeltaElbowRotate = (DesiredElbowRotateDegrees * 10) - CurrentElbowRotate;
	}

	if( SERVO_NO_CHANGE == DesiredElbowBendDegrees )
	{	// Normal mode - use the last commanded value for comparison
		DeltaElbowBend = g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees - CurrentElbowBend;
	}
	else
	{	// Special mode - use the supplied value for comparison
		DeltaElbowBend = (DesiredElbowBendDegrees * 10) - CurrentElbowBend;
	}

	if( SERVO_NO_CHANGE == DesiredWristDegrees )
	{	// Normal mode - use the last commanded value for comparison
		DeltaWrist = g_BulkServoCmd[m_WristServoID].PositionTenthDegrees - CurrentWrist;
	}
	else
	{	// Special mode - use the supplied value for comparison
		DeltaWrist = (DesiredWristDegrees * 10) - CurrentWrist;
	}

	if( SERVO_NO_CHANGE == DesiredClawDegrees )
	{	// Normal mode - use the last commanded value for comparison
		DeltaClaw = g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees - CurrentClaw;
	}
	else
	{	// Special mode - use the supplied value for comparison
		DeltaClaw = (DesiredClawDegrees * 10) - CurrentClaw;
	}

/***
	// Special case Claw.  Just closes to "snug" around the object.
	// Note, only applies to Positive (closing claw) loads, not negative (opening claw) loads
	if( (g_BulkServoStatus[m_ClawServoID].Load >= m_ClawTorque) &&
		(int )((-DeltaClaw)) > ToleranceTenthDegrees )
	{
		// At max load, but not at commanded position. 
		// So, to avoid overloading the servo, set commanded position to current position.
		ROBOT_LOG( ARM_MOVEMENT_DEBUG, "CheckArmPosition:  Claw Load Max, resetting Target Position\n" )
		g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees = g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees+1;
		g_BulkServoCmd[m_ClawServoID].Update = TRUE;
		SendCommand( WM_ROBOT_SET_ARM_POSITION, RIGHT_ARM, FALSE ); // TRUE = Set Speed
	}
***/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Now, report out first blocking joint in fastest to slowest order ( so we know who's probably blocking progress )
	// and check for stalled servos

	int ClawServoStatus = SERVO_IN_POSITION;
	int WristServoStatus = SERVO_IN_POSITION;
	int ElbowRotateServoStatus = SERVO_IN_POSITION;
	int ElbowBendServoStatus = SERVO_IN_POSITION;
	int ShoulderServoStatus = SERVO_IN_POSITION;

	if( (int)(abs(DeltaClaw)) > ToleranceTenthDegrees )
	{		
		ClawServoStatus = CheckServoPosition( "Claw", m_ClawServoID, DeltaClaw, m_LastClaw, ToleranceTenthDegrees );
	}
	if( (int)(abs(DeltaWrist)) > ToleranceTenthDegrees )
	{
		WristServoStatus = CheckServoPosition( "Wrist", m_WristServoID, DeltaWrist, m_LastWrist, ToleranceTenthDegrees );
	}
	if( (int)(abs(DeltaElbowRotate)) > ToleranceTenthDegrees )
	{
		ElbowRotateServoStatus = CheckServoPosition( "ElbowRotate", m_ElbowRotateServoID, DeltaElbowRotate, m_LastElbowRotate, ToleranceTenthDegrees );
	}
	if( (int)(abs(DeltaElbowBend)) > ToleranceTenthDegrees )
	{
		ElbowBendServoStatus = CheckServoPosition( "ElbowBend", m_ElbowBendServoID, DeltaElbowBend, m_LastElbowBend, ToleranceTenthDegrees );
	}
	if( (int)(abs(DeltaShoulder)) > ToleranceTenthDegrees )
	{
		ShoulderServoStatus = CheckServoPosition( "Shoulder", m_ShoulderServoID, DeltaShoulder, m_LastShoulder, ToleranceTenthDegrees );
	}

	if( ((SERVO_IN_POSITION == ClawServoStatus)			|| (SERVO_ABORT == ClawServoStatus)) && 
		((SERVO_IN_POSITION == WristServoStatus)		|| (SERVO_ABORT == WristServoStatus)) && 
		((SERVO_IN_POSITION == ElbowRotateServoStatus)	|| (SERVO_ABORT == ElbowRotateServoStatus)) && 
		((SERVO_IN_POSITION == ElbowBendServoStatus)	|| (SERVO_ABORT == ElbowBendServoStatus)) && 
		((SERVO_IN_POSITION == ShoulderServoStatus)		|| (SERVO_ABORT == ShoulderServoStatus))  )
	{
		// Arm movement complete.  All servos either in position or stalled
		// Determine if Arm was moving into Home position
		if( RIGHT_ARM == m_ArmNumber )
		{
			if(verbose) ROBOT_LOG( TRUE,"CheckArmPosition: Right Arm movement done\n")
			//FPOINT3D_T ArmXYZ;
			//GetCurrentArmXYZ( ArmXYZ ); // Trace done in the function.

			if(	(RIGHT_ARM_SHOULDER_HOME1*10 == g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees)	&&
				(RIGHT_ARM_ELBOW_ROTATE_HOME1*10 == g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees)	&&
				(RIGHT_ARM_ELBOW_BEND_HOME1*10 == g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees)	&&
				(RIGHT_ARM_WRIST_ROTATE_HOME1*10 == g_BulkServoCmd[m_WristServoID].PositionTenthDegrees)	)
				//(RIGHT_ARM_CLAW_HOME1*10 == g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees)	-- Don't care about claw for "Home"
			{
				gArmInHomePositionRight = TRUE;	// Arm is in home position
			}
			else
			{
				gArmInHomePositionRight = FALSE;
				ROBOT_LOG( ARM_MOVEMENT_DEBUG, "DEBUG - RIGHT ARM NOT IN HOME POSITION\n" )
			}
		}
		else if( LEFT_ARM == m_ArmNumber )
		{
			ROBOT_LOG( verbose,"CheckArmPosition: Left Arm movement done\n")
			
			// TODO - DEBUG???		
			//FPOINT3D_T ArmXYZ;
			//GetCurrentArmXYZ( ArmXYZ ); // Trace done in the function.

			if(	(LEFT_ARM_SHOULDER_HOME1*10 == g_BulkServoCmd[m_ShoulderServoID].PositionTenthDegrees)	&&
				(LEFT_ARM_ELBOW_ROTATE_HOME1*10 == g_BulkServoCmd[m_ElbowRotateServoID].PositionTenthDegrees)	&&
				(LEFT_ARM_ELBOW_BEND_HOME1*10 == g_BulkServoCmd[m_ElbowBendServoID].PositionTenthDegrees)	&&
				(LEFT_ARM_WRIST_ROTATE_HOME1*10 == g_BulkServoCmd[m_WristServoID].PositionTenthDegrees)	)
				//(LEFT_ARM_CLAW_HOME1*10 == g_BulkServoCmd[m_ClawServoID].PositionTenthDegrees)	-- Don't care about claw for "Home"
			{
				gArmInHomePositionLeft = TRUE;	// Arm is in home position
			}
			else
			{
				gArmInHomePositionLeft = FALSE;
				// ROBOT_LOG( ARM_MOVEMENT_DEBUG, "DEBUG - LEFT ARM NOT IN HOME POSITION\n" )
				//ROBOT_LOG( TRUE, "" )
			}
		}
		else ROBOT_ASSERT(0);

		__itt_task_end(pDomainControlThread);
		return TRUE; // Arm movement done
	}

	m_LastMovementTime = GetTickCount();	// update last move timer - still moving

	__itt_task_end(pDomainControlThread);
	return FALSE;	// Arm still moving
}


//-----------------------------------------------------------------------------
// Name: GetClawTorque
// Desc: Returns current Load on the Claw
//-----------------------------------------------------------------------------
int  ArmControl::GetClawTorque( )
{
	// Note: Negative torque means hand blocked while opening!
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGetClawTorque);

	int Torque = g_BulkServoStatus[m_ClawServoID].Load;
	//ROBOT_LOG( TRUE,"DEBUG: GetClawTorque: Torque = %d\n", Torque)

	if( Torque > DYNA_TORQUE_LIMIT_MAX )
	{
		ROBOT_DISPLAY( TRUE, "ERROR! GetClawTorque exceeds DYNA_TORQUE_LIMIT_MAX limit" )
		Torque = DYNA_TORQUE_LIMIT_MAX;
	}

	__itt_task_end(pDomainControlThread);
	return (int )Torque;
}

//-----------------------------------------------------------------------------
// Name: IsObjectInClaw
// Desc: Returns TRUE if the claw is holding an object
//-----------------------------------------------------------------------------
BOOL ArmControl::IsObjectInClaw( )
{
	if( LEFT_ARM == m_ArmNumber )
	{
		if( (GetPressureLoadPercent() > 30) ||  // Pressure sensors sense somehing in the hand (> 30% force)
			(GetClawPosition() > LEFT_ARM_CLAW_CLOSED_LOOSE) ) // hand did not close all the way, something stopped it
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		ROBOT_LOG( TRUE,"IsObjectInClaw not Supported for Right Claw!\n")
		return FALSE;
	}
}

//-----------------------------------------------------------------------------
// Name: GetClawPosition
// Desc: Returns current position in degrees for Claw
//-----------------------------------------------------------------------------
int ArmControl::GetClawPosition( )
{
	return g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees / 10;
}

//-----------------------------------------------------------------------------
// Name: GetWristPosition
// Desc: Returns current position in degrees for Right Wrist
//-----------------------------------------------------------------------------
int ArmControl::GetWristPosition( )
{
	return g_BulkServoStatus[m_WristServoID].PositionTenthDegrees / 10;
}


//-----------------------------------------------------------------------------
// Name: GetShoulderPosition
// Desc: Returns current position in degrees for Shoulder
//-----------------------------------------------------------------------------
int ArmControl::GetShoulderPosition( )
{
	return g_BulkServoStatus[m_ShoulderServoID].PositionTenthDegrees / 10;
}

//-----------------------------------------------------------------------------
// Name: Get Arm Position
// Desc: Returns current position in DEGREES for each joint
//-----------------------------------------------------------------------------
void ArmControl::GetArmPosition( int &Shoulder, int &ElbowRotate, int &ElbowBend, int &Wrist, int &Claw  )
{
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshGetArmPosition);

	Shoulder = g_BulkServoStatus[m_ShoulderServoID].PositionTenthDegrees / 10;

	ElbowRotate = g_BulkServoStatus[m_ElbowRotateServoID].PositionTenthDegrees / 10;

	ElbowBend = g_BulkServoStatus[m_ElbowBendServoID].PositionTenthDegrees / 10;

	Wrist = g_BulkServoStatus[m_WristServoID].PositionTenthDegrees / 10;

	Claw = g_BulkServoStatus[m_ClawServoID].PositionTenthDegrees / 10;

	__itt_task_end(pDomainControlThread);
}


//-----------------------------------------------------------------------------
// Name: Check Servo Limit
// Desc: Checks to make sure commanded value is within limits (in TenthDegrees)
// NOTE: HANDLES BOTH LEFT AND RIGHT ARM!
//-----------------------------------------------------------------------------

void ArmControl::CheckServoLimit( int  ServoID, int &PositionTenthDegrees )
{
	// Make sure servo command is within Degree limits
	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, pshCheckServoLimit);

	switch( ServoID )
	{
		// Right Arm
		case DYNA_RIGHT_ARM_ELBOW_ROTATE_SERVO_ID:
		{
			if( PositionTenthDegrees > RIGHT_ARM_ELBOW_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_ELBOW_ROTATE exceeds MAX limit" )
				PositionTenthDegrees = RIGHT_ARM_ELBOW_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < RIGHT_ARM_ELBOW_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_ELBOW_ROTATE below MIN limit" )
				PositionTenthDegrees = RIGHT_ARM_ELBOW_ROTATE_MIN;
			}
			break;
		}
		case DYNA_RIGHT_ARM_ELBOW_BEND_SERVO_ID:
		{
			if( PositionTenthDegrees > RIGHT_ARM_ELBOW_BEND_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_ELBOW_BEND exceeds MAX limit" )
				PositionTenthDegrees = RIGHT_ARM_ELBOW_BEND_MAX;
			}
			else if( PositionTenthDegrees < RIGHT_ARM_ELBOW_BEND_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_ELBOW_BEND below MIN limit" )
				PositionTenthDegrees = RIGHT_ARM_ELBOW_BEND_MIN;
			}
			break;
		}
		case DYNA_RIGHT_ARM_WRIST_SERVO_ID:
		{
			if( PositionTenthDegrees > RIGHT_ARM_WRIST_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_WRIST exceeds MAX limit" )
				PositionTenthDegrees = RIGHT_ARM_WRIST_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < RIGHT_ARM_WRIST_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_WRIST below MIN limit" )
				PositionTenthDegrees = RIGHT_ARM_WRIST_ROTATE_MIN;
			}
			break;
		}
		case DYNA_RIGHT_ARM_CLAW_SERVO_ID:
		{
			if( PositionTenthDegrees > RIGHT_ARM_CLAW_OPEN_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_CLAW exceeds MAX limit" )
				PositionTenthDegrees = RIGHT_ARM_CLAW_OPEN_MAX;
			}
			else if( PositionTenthDegrees < RIGHT_ARM_CLAW_CLOSED_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! RIGHT_ARM_CLAW below MIN limit" )
				PositionTenthDegrees = RIGHT_ARM_CLAW_CLOSED_MIN;
			}
			break;
		}
		case KERR_RIGHT_ARM_SHOULDER_SERVO_ID:
		{
			if( PositionTenthDegrees > RIGHT_SHOULDER_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! KERR_RIGHT_ARM_SHOULDER exceeds MAX limit" )
				PositionTenthDegrees = RIGHT_SHOULDER_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < RIGHT_SHOULDER_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! KERR_RIGHT_ARM_SHOULDER below MIN limit" )
				PositionTenthDegrees = RIGHT_SHOULDER_ROTATE_MIN;
			}
			break;
		}
		

		// Left Arm
		case DYNA_LEFT_ARM_ELBOW_ROTATE_SERVO_ID:
		{
			if( PositionTenthDegrees > LEFT_ARM_ELBOW_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_ELBOW_ROTATE exceeds MAX limit" )
				PositionTenthDegrees = LEFT_ARM_ELBOW_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < LEFT_ARM_ELBOW_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_ELBOW_ROTATE below MIN limit" )
				PositionTenthDegrees = LEFT_ARM_ELBOW_ROTATE_MIN;
			}
			break;
		}
		case DYNA_LEFT_ARM_ELBOW_BEND_SERVO_ID:
		{
			if( PositionTenthDegrees > LEFT_ARM_ELBOW_BEND_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_ELBOW_BEND exceeds MAX limit" )
				PositionTenthDegrees = LEFT_ARM_ELBOW_BEND_MAX;
			}
			else if( PositionTenthDegrees < LEFT_ARM_ELBOW_BEND_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_ELBOW_BEND below MIN limit" )
				PositionTenthDegrees = LEFT_ARM_ELBOW_BEND_MIN;
			}
			break;
		}
		case DYNA_LEFT_ARM_WRIST_SERVO_ID:
		{
			if( PositionTenthDegrees > LEFT_ARM_WRIST_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_WRIST exceeds MAX limit" )
				PositionTenthDegrees = LEFT_ARM_WRIST_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < LEFT_ARM_WRIST_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_WRIST below MIN limit" )
				PositionTenthDegrees = LEFT_ARM_WRIST_ROTATE_MIN;
			}
			break;
		}
		case DYNA_LEFT_ARM_CLAW_SERVO_ID:
		{
			if( PositionTenthDegrees > LEFT_ARM_CLAW_OPEN_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_CLAW exceeds MAX limit" )
				PositionTenthDegrees = LEFT_ARM_CLAW_OPEN_MAX;
			}
			else if( PositionTenthDegrees < LEFT_ARM_CLAW_CLOSED_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! LEFT_ARM_CLAW below MIN limit" )
				PositionTenthDegrees = LEFT_ARM_CLAW_CLOSED_MIN;
			}
			break;
		}
		case KERR_LEFT_ARM_SHOULDER_SERVO_ID:
		{
			if( PositionTenthDegrees > LEFT_SHOULDER_ROTATE_MAX )
			{
				ROBOT_DISPLAY( TRUE, "ERROR! KERR_LEFT_ARM_SHOULDER exceeds MAX limit" )
				PositionTenthDegrees = LEFT_SHOULDER_ROTATE_MAX;
			}
			else if( PositionTenthDegrees < LEFT_SHOULDER_ROTATE_MIN)
			{
				ROBOT_DISPLAY( TRUE, "ERROR! KERR_LEFT_ARM_SHOULDER below MIN limit" )
				PositionTenthDegrees = LEFT_SHOULDER_ROTATE_MIN;
			}
			break;
		}
		default:
		{
			ROBOT_ASSERT(0); // unhandled servo
		}
	}
	__itt_task_end(pDomainControlThread);
}

//-----------------------------------------------------------------------------
// Name: CalibratePressureSensors
// USE: Calibrate pressure sensors with no-load, just before use to measure a load. 
// Base values for Pressure sensors can vary wildly with use.
// It seems that each time pressure is applied the sensor distorts, and may or may not return
// completely to previous shape.  Then can cause false pressure values.
//-----------------------------------------------------------------------------
BOOL ArmControl::CalibratePressureSensors()
{
	if(LEFT_ARM == m_ArmNumber )
	{
		// Get current values for pressure sensors, and save as a baseline 
		m_PressureSensorCalibrationL = g_pFullSensorStatus->LeftHandRawPressureL;
		m_PressureSensorCalibrationR = g_pFullSensorStatus->LeftHandRawPressureR;

		if( (0 == m_PressureSensorCalibrationL) || (0 == m_PressureSensorCalibrationR) )
		{
			ROBOT_LOG( TRUE,  " ERROR - Can't calibrate Hand pressure!  Sensor error? L=%3d, R=%3d", 
				m_PressureSensorCalibrationL, m_PressureSensorCalibrationL )
		}

		ROBOT_LOG( TRUE,  " DEBUG Hand pressure calibrated at: L=%3d, R=%3d", 
			m_PressureSensorCalibrationL, m_PressureSensorCalibrationL )
		return TRUE;
	}
	else if( RIGHT_ARM == m_ArmNumber )
	{
		ROBOT_DISPLAY( TRUE, "ERROR!  Right Hand Pressure Sensor not Implemented!" )
		return FALSE;
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "ERROR!  Bad hand value!" )
		return FALSE;
	}
}

//-----------------------------------------------------------------------------
// Name: GetPressureLoadPercent
// Desc: Returns Pressure of object in hand 0 = no object, 100% = max pressure
// Requires Calibration before EACH use, as the sensors deform
//-----------------------------------------------------------------------------
int ArmControl::GetPressureLoadPercent()
{
	if( (0 == m_PressureSensorCalibrationL) || (0 == m_PressureSensorCalibrationR) )
	{
		// Not calibrated, ignore
		return 0;
	}

	if(LEFT_ARM == m_ArmNumber )
	{
		int PercentL = 100 - ((g_pFullSensorStatus->LeftHandRawPressureL * 100) / m_PressureSensorCalibrationL);
		int PercentR = 100 - ((g_pFullSensorStatus->LeftHandRawPressureR * 100) / m_PressureSensorCalibrationR);
		int PercentMax = max(PercentL, PercentR);
		ROBOT_LOG( TRUE,  " DEBUG Hand pressure measured at: %3d, L=%3d, R=%3d", 
			PercentMax, PercentL, PercentR )

		return PercentMax; // use the greatest of the two sensors to determine load
	}
	else if( RIGHT_ARM == m_ArmNumber )
	{
		ROBOT_DISPLAY( TRUE, "ERROR!  Right Hand Pressure Sensor not Implemented!" )
		return 0;
	}
	else
	{
		ROBOT_DISPLAY( TRUE, "ERROR!  Bad hand value!" )
		return 0;
	}
}


//-----------------------------------------------------------------------------
// Name: Enable Idle Arm Movement
// Desc: Allows arms to move randomly when not otherwise being used
//-----------------------------------------------------------------------------
void ArmControl::EnableIdleArmMovement( BOOL bEnable )
{

if( LEFT_ARM == m_ArmNumber )
	{
		gArmIdleLeft = bEnable;
	}
	else if( RIGHT_ARM == m_ArmNumber )
	{
		gArmIdleRight = bEnable;
	}
	else ROBOT_ASSERT(0);

}

//-----------------------------------------------------------------------------
// Name: Idle Arm Movement Enabled
// Desc: Returns true if Idle is enabled
//-----------------------------------------------------------------------------
BOOL ArmControl::IdleArmMovementEnabled()
{
	if( LEFT_ARM == m_ArmNumber )
	{
		return gArmIdleLeft;
	}
	else if( RIGHT_ARM == m_ArmNumber )
	{
		return gArmIdleRight;
	}
	else
	{
		ROBOT_ASSERT(0);
		return 0;
	}
}
//-----------------------------------------------------------------------------
// Name: Do Idle Arm Movement
// Desc: Move arms when idle, to give more appearance of "life"
//-----------------------------------------------------------------------------
void ArmControl::DoIdleArmMovement()
{

	///////////////////////////////////////////////////////////
	// TODO!  DISABLED FOR NOW!!!
	return;
	///////////////////////////////////////////////////////////



	// TODO - increase arm movement amount when talking?
	if( !gKerrControlInitialized )
	{
		// Shoulder motors not yet initialized
		return;
	}

	if( (LEFT_ARM == m_ArmNumber) && !gArmIdleLeft )
	{
			return;	// Arm not idle
	}
	if( (RIGHT_ARM == m_ArmNumber) && !gArmIdleRight )
	{
			return;	// Arm not idle
	}

	int TimeSinceLastMovement =  GetTickCount() - m_LastMovementTime;
	int RandomTime = (IDLE_ARM_MOVEMENT_FREQUENCY * 2 * rand()) / RAND_MAX;
	if( TimeSinceLastMovement < (IDLE_ARM_MOVEMENT_FREQUENCY + RandomTime) )
	{
		return; // wait a while
	}

	// OK, time to move the arm

	int ShoulderDelta = ((IDLE_ARM_MOVEMENT_DEGREES * 2 * rand()) / RAND_MAX) - IDLE_ARM_MOVEMENT_DEGREES;
	int ElbowRotateDelta = ((IDLE_ARM_MOVEMENT_DEGREES * 2 * rand()) / RAND_MAX) - IDLE_ARM_MOVEMENT_DEGREES;
	int ElbowBendDelta  = ((IDLE_ARM_MOVEMENT_DEGREES * 2 * rand()) / RAND_MAX) - IDLE_ARM_MOVEMENT_DEGREES;
	int WristDelta  = ((IDLE_ARM_MOVEMENT_DEGREES * 2 * rand()) / RAND_MAX) - IDLE_ARM_MOVEMENT_DEGREES;
	//int ClawDelta = ((IDLE_ARM_MOVEMENT_DEGREES * 2 * rand()) / RAND_MAX) - IDLE_ARM_MOVEMENT_DEGREES;
	// TODO - Validate that claw is not holding anying before doing a random open/close!

	SetArmSpeed( SERVO_SPEED_VERY_SLOW, SERVO_SPEED_VERY_SLOW, SERVO_SPEED_VERY_SLOW, SERVO_SPEED_VERY_SLOW, NOP );

	if(RIGHT_ARM == m_ArmNumber )
	{
		SetArmPosition( 
			RIGHT_ARM_SHOULDER_HOME1+ShoulderDelta+5, 
			RIGHT_ARM_ELBOW_ROTATE_HOME1+ElbowRotateDelta, 
			RIGHT_ARM_ELBOW_BEND_HOME1+ElbowBendDelta, 
			RIGHT_ARM_WRIST_ROTATE_HOME1+WristDelta, NOP );
	}
	else if(LEFT_ARM == m_ArmNumber )
	{
		SetArmPosition( 
			LEFT_ARM_SHOULDER_HOME1+ShoulderDelta+5, 
			LEFT_ARM_ELBOW_ROTATE_HOME1+ElbowRotateDelta, 
			LEFT_ARM_ELBOW_BEND_HOME1+ElbowBendDelta, 
			LEFT_ARM_WRIST_ROTATE_HOME1+WristDelta, NOP );
	}
	else
	{
		ROBOT_ASSERT(0);
	}

	// TODO - Validate that the movement stays within valid limits (does not hit hand on anything)
	ExecutePositionAndSpeed();
}


#endif // ROBOT_SERVER - This module used for Robot Server only

