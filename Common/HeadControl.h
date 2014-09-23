// HeadControl.cpp: Head Control class
// Controls head positioning servos
#include "RobotConfig.h"


#ifndef __HEAD_CONTROL_H_INCLUDED
#define __HEAD_CONTROL_H_INCLUDED

#define CENTER_OF_ROBOT_TO_FRONT_TENTH_INCHES		  80.00	// Inches 1/2 front to tail wheel)
#define CENTER_OF ROBOT_TO_SIDE_TENTH_INCHES		  80.00	// Inches

#define DEFAULT_HEAD_MOVE_TIME_LIMIT_TENTH_SECONDS	  30	// Default time for m_HeadMoveTimeLimit  = 3 Seconds (30 Tenth Seconds)

/////////////////////////////////////////////////////////////////////////////
class HeadControl
{

public:


	HeadControl::HeadControl( );

	HeadControl::~HeadControl();

// Set Functions

	//-----------------------------------------------------------------------------
	// Name: Set Head Position
	// Desc: quickly sets precise position of the Head in TENTHDEGREES	
	// Use special value SERVO_NO_CHANGE to skip a given servo
	//-----------------------------------------------------------------------------
	void SetHeadPosition( int  nOwner, int Pan, int Tilt, int SideTilt, BOOL TrackingLimited = FALSE );

	//-----------------------------------------------------------------------------
	// Name: Set Head Position Center
	// Desc: Reset head to straight forward (home) position
	// Calls SetHeadPosition with absolute position in TENTH DEGREES
	//-----------------------------------------------------------------------------
	void SetHeadPositionCenter( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: Set Head Position Center Human
	// Desc: Reset head to straight forward (home) position, but tilt up to where Human faces usually are
	// Calls SetHeadPosition with absolute position in TENTH DEGREES
	// Does an automatic Execute!
	//-----------------------------------------------------------------------------
	void HeadControl::SetHeadPositionCenterHuman( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: Set Head Position Relative
	// Desc: Sets precise position of the Head RELATIVE to it's current position
	// Use special value SERVO_NO_CHANGE to skip a given servo
	// Set TrackingLimited flag to limit movement to front of robot (while tracking motion, etc.)
	//-----------------------------------------------------------------------------
	void SetHeadPositionRelative(int  nOwner, int PanDelta, int TiltDelta, int SideTiltDelta, BOOL TrackingLimited = FALSE );

	//-----------------------------------------------------------------------------
	// Name: Look At XYZ
	// Desc: Point camera at position X,Y,Z in space in TENTH INCHES
	//-----------------------------------------------------------------------------
	void LookAtXYZ(int  nOwner, int X, int Y, int Z );

	//-----------------------------------------------------------------------------
	// Name: XYZtoPanTilt
	// Desc: Calculates Pan/Tilt needed to point camera at position X,Y,Z in space in TENTH INCHES
	//-----------------------------------------------------------------------------
	void XYZtoPanTilt( int X, int Y, int Z, int &PanTenthDegrees, int &TiltTenthDegrees );

	//-----------------------------------------------------------------------------
	// Name: ServoTiltDegreesToLaserYInches
	// Desc: Calculates distance from the front of the robot to the laser line (Y)
	// for any given angle
	//-----------------------------------------------------------------------------
	double HeadControl::ServoTiltDegreesToLaserYTenthInches( double ServoTiltDegrees );

	//-----------------------------------------------------------------------------
	// Name: YtoTilt
	// Desc: Calculates tilt needed to look at spot at given distance
	// in front of the robot
	//-----------------------------------------------------------------------------
	double HeadControl::YtoTilt( double TargetY );

	//-----------------------------------------------------------------------------
	// Name: Set Head Speed
	// Desc: Sets speed for all servos (each can be a different speed)
	// Use special value SERVO_NO_CHANGE to skip a given servo
	//-----------------------------------------------------------------------------
	void SetHeadSpeed(int  nOwner, int  Pan, int  Tilt, int  SideTilt );

	//-----------------------------------------------------------------------------
	// Name: Do Idle Head Movement
	// Desc: Move head around to simulate life
	//-----------------------------------------------------------------------------
	void HeadControl::DoIdleMovement( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: ExecutePosition
	// Desc: Commits movement for all servos setup previously
	//-----------------------------------------------------------------------------
	void ExecutePosition( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: ExecutePositionAndSpeed
	// Desc: Commits movement and speed for all servos setup previously
	//-----------------------------------------------------------------------------
	void ExecutePositionAndSpeed( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: Stop
	// Desc: Tell Head servos to stop moving immediately
	//-----------------------------------------------------------------------------
	void HeadControl::Stop( int  nOwner );

	//-----------------------------------------------------------------------------
	// Name: SetServoCompliance
	// Desc: Set compliance slope of the specified servo
	// This is how accurate the servo will track to requested, but may also put extra stress on the servo.
	//-----------------------------------------------------------------------------
	void HeadControl::SetServoCompliance( int  nOwner, int  nServo, int  Compliance );

	//-----------------------------------------------------------------------------
	// Name: SetUserHeadSpeed
	// Desc: Sets the default Pan/Tilt speed for all user movement requests
	//-----------------------------------------------------------------------------
	void SetUserHeadSpeed( int Speed ) {m_nUserPanTiltSpeed = Speed;}	// inline

	//-----------------------------------------------------------------------------
	// Name: SetServoTimeout
	// Desc: Set max time move will wait to reach commanded position before timing out
	// This prevents the robot from hanging if a servo can't reach commanded position
	//-----------------------------------------------------------------------------
	void SetServoTimeout( int  nOwner, int  Timeout );


// Get Functions
	//-----------------------------------------------------------------------------
	// Name: Check Head Position
	// Desc: Are we there yet?  Check to see if Head has reached commanded position
	// Verbose flag will print out the servo that is blocking move complete status
	//-----------------------------------------------------------------------------
	BOOL CheckHeadPosition(BOOL verbose );

	//-----------------------------------------------------------------------------
	// Name: Check Servo Limit
	// Desc: Checks to make sure commanded value is within limits (in TenthDegrees)
	//-----------------------------------------------------------------------------
	void CheckServoLimit( int  ServoID, int &PositionTenthDegrees );

	//-----------------------------------------------------------------------------
	// Name: Check Tracking Limits
	// Desc: Keep camera toward front of robot, so it does not go off in la la land!
	//       Only applies for some operations, such as motion or laser tracking
	//-----------------------------------------------------------------------------
	void HeadControl::CheckTrackingLimits( int &NewPan, int &NewTilt);	// values in TenthDegrees

	//-----------------------------------------------------------------------------
	// Name: Get Head Position
	// Desc: Returns current position of all head servos in TENTH DEGREES
	//-----------------------------------------------------------------------------
	void GetHeadPosition( int &Pan, int &Tilt, int &SideTilt );

	//-----------------------------------------------------------------------------
	// Name: Get Pan Position
	// Desc: Returns current Pan position in TENTH DEGREES
	//-----------------------------------------------------------------------------
	int HeadControl::GetPanPosition(  );

	//-----------------------------------------------------------------------------
	// Name: Get Tilt Position
	// Desc: Returns current Tilt position in TENTH DEGREES
	//-----------------------------------------------------------------------------
	int HeadControl::GetTiltPosition(  );

	//-----------------------------------------------------------------------------
	// Name: Get Side Tilt Position
	// Desc: Returns current SideTilt position in TENTH DEGREES
	//-----------------------------------------------------------------------------
	int HeadControl::GetSideTiltPosition(  );

	//-----------------------------------------------------------------------------
	// Name: Get Head Speed
	// Desc: Returns current speed of head servos
	//-----------------------------------------------------------------------------
	void GetHeadSpeed( int  &PanSpeed, int  &TiltSpeed, int  &SideTiltSpeed );
	int GetPanSpeed() { return m_PanSpeed; }	// inline
	int GetTiltSpeed() { return m_TiltSpeed; }	// inline
	int GetSideTiltSpeed() { return m_SideTiltSpeed; }	// inline

	//-----------------------------------------------------------------------------
	// Name: GetUserHeadSpeed
	// Desc: Gets the Pan/Tilt speed for all user movement requests
	//-----------------------------------------------------------------------------
	int GetUserHeadSpeed() { return m_nUserPanTiltSpeed; }	// inline



	/////////////////////////////////////////////////////////////////////////////////////////
	// HEAD CONTROL ARBITRATOR
	// Tracks who is control of the head, assigning control based upon a strict priority
	/////////////////////////////////////////////////////////////////////////////////////////

	//-----------------------------------------------------------------------------
	// Name: Check And Set Owner
	// Desc: Checks for current owner, and if requesting module is higher priority
	// changes to new module.  Returns TRUE if module got ownership.
	//-----------------------------------------------------------------------------
	BOOL CheckAndSetOwner( int  NewOwner );

	BOOL IsOwner( int  NewOwner );

	BOOL ReleaseOwner( int  NewOwner );



private:

	int		m_PanServoID;
	int		m_TiltServoID;
	int		m_SideTiltServoID;
	int		m_nUserPanTiltSpeed;
	int		m_HeadMoveTimeLimit;
	int		m_RandomHeadCounter;
	int		m_PanSpeed;
	int		m_TiltSpeed;
	int		m_SideTiltSpeed;
};

#endif // __HEAD_CONTROL_H_INCLUDED