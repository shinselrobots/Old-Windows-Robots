// ArmControl.cpp: Arm Control class
//
#ifndef __ARM_CONTROL_H_INCLUDED
#define __ARM_CONTROL_H_INCLUDED

#include "Globals.h"

#define DEFAULT_ARM_MOVE_TIME_LIMIT_TENTH_SECONDS	  50	// Default time for m_ArmMoveTimeLimit = 5 Seconds (50 Tenth Seconds)
#define OBJECT_PICKUP_LEFT_MAX_Y		  80 // TenthInches - Max distance object can be in front of robot and still get picked up
#define OBJECT_PICKUP_LEFT_MAX_X		 -80 // TenthInches - Max distance object can be toward Center of robot and still get picked up
#define OBJECT_PICKUP_LEFT_MIN_X		-120 // TenthInches - Max distance object can be to Left of robot and still get picked up

#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only


/////////////////////////////////////////////////////////////////////////////
class ArmControl
{


public:


	ArmControl::ArmControl( int  ArmNumber);

	ArmControl::~ArmControl();

	bool ArmControl::ArmInstalled(); // test to see if arm is even installed on the robot.

	BOOL IsObjectInPickupZone( int ObjectTenthInchesY, int ObjectTenthInchesX );

// Set Functions

	void ClearServoStallTimers();

	//-----------------------------------------------------------------------------
	// Name: Move Arm Home
	// Desc: quickly sets given arm to Home position.
	// Sets Speed too, if optional parameter set
	// NOTE!  Calls execute_position Too!
	//-----------------------------------------------------------------------------
	void MoveArmHome( int  Speed = SERVO_NO_CHANGE );

	//-----------------------------------------------------------------------------
	// Name: Move Arm to Safe Position
	// Desc: quickly sets given arm to "Home2", or "safe" position, to avoid hitting objects
	// Sets Speed too, if optional parameter set
	// NOTE!  Calls execute_position Too!
	//-----------------------------------------------------------------------------
	void MoveArmToSafePosition( int  Speed = SERVO_NO_CHANGE );


	//-----------------------------------------------------------------------------
	// Name: Set Arm Position
	// Desc: quickly sets precise position of the given arm
	// Use special value SERVO_NO_CHANGE to skip a given servo
	//-----------------------------------------------------------------------------
	void SetArmPosition( int Shoulder, int ElbowRotate, int ElbowBend, int Wrist, int Claw );

	//-----------------------------------------------------------------------------
	// Name: Set Arm Speed
	// Desc: Sets speed for all servos (each can be a different speed)
	// Use special value SERVO_NO_CHANGE to skip a given servo
	//-----------------------------------------------------------------------------
	void SetArmSpeed( int  Shoulder, int  ElbowRotate, int  ElbowBend, int  Wrist, int  Claw );

	//-----------------------------------------------------------------------------
	// Name: Set Arm Delay
	// Desc: Sets delay for all servos (each can be a different delay)
	// Use special value SERVO_NO_CHANGE to skip a given servo
	//-----------------------------------------------------------------------------
	void SetArmDelay( int  Shoulder, int  ElbowRotate, int  ElbowBend, int  Wrist, int  Claw );

	//-----------------------------------------------------------------------------
	// Name: SetClawTorque
	// Desc: Sets Claw Torque value
	//-----------------------------------------------------------------------------
	void SetClawTorque( int  Torque );

	//-----------------------------------------------------------------------------
	// Name: ClawRegripObject
	// Desc: After grabbing an object, reset the servo to hold it firmly, 
	//       but not overload the servo
	//-----------------------------------------------------------------------------
	void ClawRegripObject( );

	//-----------------------------------------------------------------------------
	// Name: ExecutePosition
	// Desc: Commits movement for all servos setup previously
	//-----------------------------------------------------------------------------
	void ExecutePosition();

	//-----------------------------------------------------------------------------
	// Name: ExecutePositionAndSpeed
	// Desc: Commits movement and speed for all servos setup previously
	//-----------------------------------------------------------------------------
	void ExecutePositionAndSpeed();

	//-----------------------------------------------------------------------------
	// Name: Enable Idle Arm Movement
	// Desc: Allows arms to move randomly when not otherwise being used
	//-----------------------------------------------------------------------------
	void ArmControl::EnableIdleArmMovement( BOOL bEnable );

	//-----------------------------------------------------------------------------
	// Name: Do Idle Arm Movement
	// Desc: Move arms when idle, to give more appearance of "life"
	//-----------------------------------------------------------------------------
	void DoIdleArmMovement();



// Get Functions

	//-----------------------------------------------------------------------------
	// Name: Check Servo Position
	// Desc: Checks for Servo position and any stalls while moving into positon.  Used by CheckArmPosition()
	// Returns SERVO_STATUS_T
	//-----------------------------------------------------------------------------
	int CheckServoPosition( const char *ServoName, int ServoID, int Delta, int &LastPosition, int ToleranceTenthDegrees );

	//-----------------------------------------------------------------------------
	// Name: Check Arm Position
	// Desc: Are we there yet?  Check to see if arm has reached commanded position
	// Verbose flag will print out the servo that is blocking move complete status
	// OPTIONAL parameters for joints will use supplied value if not SERVO_NO_CHANGE
	//-----------------------------------------------------------------------------
	//BOOL CheckArmPosition(BOOL verbose );
	BOOL CheckArmPosition(BOOL verbose, int  ToleranceTenthDegrees=ARM_JOINT_DELTA_MAX_TENTHDEGREES, int DesiredShoulderDegrees=SERVO_NO_CHANGE, int DesiredElbowRotateDegrees=SERVO_NO_CHANGE, 
		int DesiredElbowBendDegrees=SERVO_NO_CHANGE, int DesiredWristDegrees=SERVO_NO_CHANGE, int DesiredClawDegrees=SERVO_NO_CHANGE );

	//-----------------------------------------------------------------------------
	// Name: SetServoTimeout
	// Desc: Set max time move will wait to reach commanded position before timing out
	// This prevents the robot from hanging if a servo can't reach commanded position
	//-----------------------------------------------------------------------------
	void SetServoTimeout( int  nOwner, int  Timeout );

	//-----------------------------------------------------------------------------
	// Name: Check Servo Limit
	// Desc: Checks to make sure commanded value is within limits (in TenthDegrees)
	// NOTE: HANDLES BOTH LEFT AND RIGHT ARM!
	//-----------------------------------------------------------------------------
	void CheckServoLimit( int  ServoID, int &PositionTenthDegrees );


	//-----------------------------------------------------------------------------
	// Name: GetClawTorque
	// Desc: Returns current Load on the Claw (how tight it is holding an object)
	//-----------------------------------------------------------------------------
	int  GetClawTorque( );

	//-----------------------------------------------------------------------------
	// Name: IsObjectInClaw
	// Desc: Returns TRUE if the claw is holding an object
	//-----------------------------------------------------------------------------
	BOOL ArmControl::IsObjectInClaw( );

	//-----------------------------------------------------------------------------
	// Name: GetClawPosition
	// Desc: Returns current for Claw
	//-----------------------------------------------------------------------------
	int GetClawPosition( );

	//-----------------------------------------------------------------------------
	// Name: GetWristPosition
	// Desc: Returns current for Wrist
	//-----------------------------------------------------------------------------
	int GetWristPosition( );

	//-----------------------------------------------------------------------------
	// Name: GetShoulderPosition
	// Desc: Returns current for Shoulder
	//-----------------------------------------------------------------------------
	int GetShoulderPosition( );

	//-----------------------------------------------------------------------------
	// Name: Get Arm Position
	// Desc: Returns current position in DEGREES of all joints
	//-----------------------------------------------------------------------------
	void GetArmPosition( int &Shoulder, int &ElbowRotate, int &ElbowBend, int &Wrist, int &Claw  );

	//-----------------------------------------------------------------------------
	// Name: CalibratePressureSensors
	// USE: Calibrate pressure sensors with no-load, just before use to measure a load. 
	// Base values for Pressure sensors can vary wildly with use.
	// It seems that each time pressure is applied the sensor distorts, and may or may not return
	// completely to previous shape.  Then can cause false pressure values.
	//-----------------------------------------------------------------------------
	BOOL CalibratePressureSensors();

	//-----------------------------------------------------------------------------
	// Name: GetPressureLoadPercent
	// Desc: Returns Pressure of object in hand 0 = no object, 100% = max pressure
	// Requires Calibration before EACH use, as the sensors deform
	//-----------------------------------------------------------------------------
	int GetPressureLoadPercent();

	//-----------------------------------------------------------------------------
	// Name: Idle Arm Movement Enabled
	// Desc: Returns true if Idle is enabled
	//-----------------------------------------------------------------------------
	BOOL IdleArmMovementEnabled();

	//-----------------------------------------------------------------------------
	// Name: GetTargetArmXYZ, GetCurrentArmXYZ, CalculateArmXYZ
	// Desc: Return current or target X,Y,Z position of Finger Tip
	// X= Side to side from robot center, Y= distance from front of robot, Z = height above ground
	// If bGetTargetPosition set, return Target position arm is moving towards.  Else, return current position of arm.
	// If nObjectDistance is non-zero, return calculated position of object detected by hand sensors
	//-----------------------------------------------------------------------------
	void GetTargetArmXYZ( FPOINT3D_T &ArmXYZ, double nObjectDistance=0 );

	void GetCurrentArmXYZ( FPOINT3D_T &ArmXYZ, double nObjectDistance=0 );

	void CalculateArmXYZ( FPOINT3D_T &ArmXYZ, ARM_SERVOS_POSITION_T Servo, double nObjectDistance=0 );

	//-----------------------------------------------------------------------------
	// Name: CalculateArmMoveToXYZ
	// Desc: Return servo values needed to move arm Finger Tip to target X,Y,Z position
	// X= Side to side from robot center, Y= distance from front of robot, Z = height above ground
	// Returns true if a soluton is found
	//-----------------------------------------------------------------------------
	BOOL CalculateArmMoveToXYZ(FPOINT3D_T &TargetXYZ, ARM_SERVOS_POSITION_T &Servo );


private:

	int 	m_ArmNumber;	// Which Arm, Rigth or Left
	int		m_ShoulderServoID;
	int		m_ElbowRotateServoID;
	int		m_ElbowBendServoID;
	int		m_WristServoID;
	int		m_ClawServoID;
	int		m_ClawTorque;
	int 	m_LastMovementTime;
	int		m_ArmMoveTimeLimit;

	int		m_LastShoulder;
	int		m_LastElbowRotate;
	int		m_LastElbowBend;
	int		m_LastWrist;
	int		m_LastClaw;
	int		m_PressureSensorCalibrationL;
	int		m_PressureSensorCalibrationR;


};
#endif // ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#endif // __ARM_CONTROL_H_INCLUDED