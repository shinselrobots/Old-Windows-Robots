// HeadControl.cpp: Head Control class
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "HeadControl.h"
#include "HardwareConfig.h"

#define MIN_IDLE_TIME_MS			3000	// Number of MS to wait since last camera move before doing IDLE behavior movements

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
// HeadControl
// WARNING!!  All Head postion info is in TENTH DEGREES!
// (arm control might use degrees)


// TODO!
//	#if (CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA)
// Need to reimplement all this for Sony or regular servos
// See old code dated 12/31/08 or earlier


HeadControl::HeadControl( )
{
	m_PanServoID =		DYNA_CAMERA_PAN_SERVO_ID;
	m_TiltServoID =		DYNA_CAMERA_TILT_SERVO_ID;
	m_SideTiltServoID =	DYNA_CAMERA_SIDETILT_SERVO_ID;
	m_nUserPanTiltSpeed = SERVO_SPEED_MED_SLOW;
	m_PanSpeed = SERVO_SPEED_MED_SLOW;
	m_TiltSpeed = SERVO_SPEED_MED_SLOW;
	m_SideTiltSpeed = SERVO_SPEED_MED_SLOW;

	m_HeadMoveTimeLimit = DEFAULT_HEAD_MOVE_TIME_LIMIT_TENTH_SECONDS;	// Default timeout
	m_RandomHeadCounter = 0;

	ROBOT_LOG( TRUE,"Head Control constructor complete")
}

HeadControl::~HeadControl()
{
	// Release resources
	ROBOT_LOG( TRUE,"~HeadControl done\n")
}


//-----------------------------------------------------------------------------
// Name: Set Head Position
// Desc: quickly sets precise position of the Head in TENTHDEGREES
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void HeadControl::SetHeadPosition(int  nOwner, int Pan, int Tilt, int SideTilt, BOOL TrackingLimited )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	if( TrackingLimited )
	{
		CheckTrackingLimits( Pan, Tilt );
	}

	{
		__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csServoLock);
		EnterCriticalSection(&g_csServoLock);

		if( SERVO_NO_CHANGE != Tilt )
		{
			CheckServoLimit(m_TiltServoID, Tilt);
			g_BulkServoCmd[m_TiltServoID].PositionTenthDegrees = Tilt;
			g_BulkServoCmd[m_TiltServoID].Update = TRUE;
	//		ROBOT_LOG( TRUE,"SetHeadPosition: Tilt\n")

		}
		if( SERVO_NO_CHANGE != Pan )
		{
			CheckServoLimit(m_PanServoID, Pan);
			g_BulkServoCmd[m_PanServoID].PositionTenthDegrees = Pan;
			g_BulkServoCmd[m_PanServoID].Update = TRUE;
	//		ROBOT_LOG( TRUE,"SetHeadPosition: Pan\n")
		}
		if( SERVO_NO_CHANGE != SideTilt )
		{
			CheckServoLimit(m_SideTiltServoID, SideTilt);
			g_BulkServoCmd[m_SideTiltServoID].PositionTenthDegrees = SideTilt;
			g_BulkServoCmd[m_SideTiltServoID].Update = TRUE;
	//		ROBOT_LOG( TRUE,"SetHeadPosition: SideTilt\n")

		}
		LeaveCriticalSection(&g_csServoLock);
		__itt_task_end(pDomainControlThread);
	}
}

//-----------------------------------------------------------------------------
// Name: Set Head Position Center
// Desc: Reset head to straight forward (home) position
// Calls SetHeadPosition with absolute position in TENTH DEGREES
// Does an automatic Execute!
//-----------------------------------------------------------------------------
void HeadControl::SetHeadPositionCenter( int  nOwner )
{
	SetHeadPosition( nOwner, CAMERA_PAN_CENTER, CAMERA_TILT_CENTER+20, CAMERA_SIDETILT_CENTER );
	ExecutePosition( nOwner );
}

//-----------------------------------------------------------------------------
// Name: Set Head Position Center Human
// Desc: Reset head to straight forward (home) position, but tilt up to where Human faces usually are
// Calls SetHeadPosition with absolute position in TENTH DEGREES
// Does an automatic Execute!
//-----------------------------------------------------------------------------
void HeadControl::SetHeadPositionCenterHuman( int  nOwner )
{
	SetHeadPosition( nOwner, CAMERA_PAN_CENTER, CAMERA_HUMAN_DETECT_START_POSITION, CAMERA_SIDETILT_CENTER );
	ExecutePosition( nOwner );
}

//-----------------------------------------------------------------------------
// Name: Set Head Position Relative
// Desc: Sets precise position of the Head RELATIVE to it's current position in TENTHDEGREES
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void HeadControl::SetHeadPositionRelative( int  nOwner, int PanDelta, int TiltDelta, int SideTiltDelta, BOOL TrackingLimited )
{
	int Pan = SERVO_NO_CHANGE;
	int Tilt = SERVO_NO_CHANGE;
	int SideTilt = SERVO_NO_CHANGE;

	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	if( SERVO_NO_CHANGE != TiltDelta )
	{
		Tilt = (g_BulkServoStatus[m_TiltServoID].PositionTenthDegrees) + TiltDelta;
	}
	if( SERVO_NO_CHANGE != PanDelta )
	{
		Pan = (g_BulkServoStatus[m_PanServoID].PositionTenthDegrees) + PanDelta;
	}
	if( SERVO_NO_CHANGE != SideTiltDelta )
	{
		SideTilt = (g_BulkServoStatus[m_SideTiltServoID].PositionTenthDegrees) + SideTiltDelta;
	}
 
	SetHeadPosition( nOwner, Pan, Tilt, SideTilt, TrackingLimited );
//	ROBOT_LOG( TRUE, "m_CurrentTask = %d", m_CurrentTask )
}

//-----------------------------------------------------------------------------
// Name: Set Head Speed
// Desc: Sets speed for all servos (each can be a different speed)
// Use special value SERVO_NO_CHANGE to skip a given servo
//-----------------------------------------------------------------------------
void HeadControl::SetHeadSpeed( int  nOwner, int  Pan, int  Tilt, int  SideTilt )
{
	const int MaxHeadMoveSpeed = SERVO_SPEED_MED;

	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	if( SERVO_NO_CHANGE != Pan )
	{
		if( Pan > MaxHeadMoveSpeed ) Pan = MaxHeadMoveSpeed; // KLUDGE
		m_PanSpeed = Pan;
		g_BulkServoCmd[m_PanServoID].Speed = Pan;
		g_BulkServoCmd[m_PanServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != Tilt )
	{
		if( Tilt > MaxHeadMoveSpeed ) Tilt = MaxHeadMoveSpeed; // KLUDGE
		m_TiltSpeed = Tilt;
		g_BulkServoCmd[m_TiltServoID].Speed = Tilt;
		g_BulkServoCmd[m_TiltServoID].Update = TRUE;
	}
	if( SERVO_NO_CHANGE != SideTilt )
	{
		if( SideTilt > MaxHeadMoveSpeed ) SideTilt = MaxHeadMoveSpeed; // KLUDGE
		m_SideTiltSpeed = SideTilt;
		g_BulkServoCmd[m_SideTiltServoID].Speed = SideTilt;
		g_BulkServoCmd[m_SideTiltServoID].Update = TRUE;
	}
	//ROBOT_LOG( TRUE, "Head Speed set to %d, %d, %d by owner %d", m_PanSpeed, m_TiltSpeed, m_SideTiltSpeed, nOwner )
}


//-----------------------------------------------------------------------------
// Name: Do Idle Head Movement
// Desc: Move head around to simulate life
//-----------------------------------------------------------------------------
void HeadControl::DoIdleMovement( int  nOwner )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}
	if( gHeadIdle && (GetTickCount() > g_LastCameraMoveTime + MIN_IDLE_TIME_MS ) && (m_RandomHeadCounter++ > 5) )
	{
		// Look around for someone to talk to, or something to do
		m_RandomHeadCounter = 0;
		CvPoint position;
//				double FrameWidth = m_pVideoFrameLeft->width;
//				double FrameHeight = m_pVideoFrameLeft->height;

		double fRandom_SideTilt = 0.5 - ((double)(rand()) / RAND_MAX);
		int SideTilt = (int)(fRandom_SideTilt * ((double)CAMERA_SIDETILT_TENTHDEGREES_TILT_LEFT));

		double fRandom_X = 0.5 - ((double)(rand()) / RAND_MAX);
		position.x = (int)(fRandom_X * ((double)CAMERA_PAN_TENTHDEGREES_MAX_RIGHT / 4.0));	// MAX value is pointing to the back of the robot!

		double fRandom_Y = 0.5 - ((double)(rand()) / RAND_MAX);
		position.y = (int)( fRandom_Y * ((double)CAMERA_TILT_TENTHDEGREES_MAX_UP / 4.0) );  // 2.0 About 17 degrees?  Try 11 degrees? - TODO-DWS
		// ROBOT_LOG( TRUE,"Random Camera Position = %d, %d,  Frame = %d, %d\n", position.x, position.y, m_pVideoFrameLeft->width, m_pVideoFrameLeft->height )

		// Random movements, but never too far from straight forward
		int CameraPanPos =  (int)( (double)(CAMERA_PAN_CENTER) + (double)position.x);
		int CameraTiltPos = (int)( (double)(CAMERA_TILT_FACE_POSITION-50) + (double)position.y);

		SetHeadSpeed(nOwner, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_SLOW );
		SetHeadPosition( nOwner, CameraPanPos, CameraTiltPos, SideTilt ); 
		ExecutePositionAndSpeed( nOwner );
		//ROBOT_LOG( TRUE,"***************> FRIENDLY SPEED SLOW\n")

				
		// TO DEBUG CAMERA CONTROL, ENABLE THIS:	
		// ROBOT_LOG( TRUE,"CAMERA RANDOM: Point: Pan = %d  Tilt = %d\n", CameraPanPos, CameraTiltPos )

		//Sleep(1);	// make sure the move starts before changing speed
		//DebugCmdCount++;
		//ROBOT_LOG( TRUE,"***************> RANDOM HEAD REQUEST %d\n", DebugCmdCount)
	}
	
}

//-----------------------------------------------------------------------------
// Name: Look At XYZ
// Desc: Point camera at position X,Y,Z in space in TENTH INCHES
// Calls SetHeadPosition with absolute position in TENTH DEGREES
//-----------------------------------------------------------------------------
void HeadControl::LookAtXYZ( int  nOwner, int X, int Y, int Z )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	int PanTenthDegrees = 0;
	int TiltTenthDegrees = 0;

	// Calculate Pan/Tilt needed to point camera at position X,Y,Z in space in TENTH INCHES

	// Special case when head looking forward
	if( (0 == X) && (0 == Z) )
	{
		PanTenthDegrees = 0;
		// Convert from TenthInches to Inches, call function, and convert degrees to tenthdegrees
		double Tilt = YtoTilt( (double)Y );
		TiltTenthDegrees = (int)( Tilt * 10.0 );
	}
	else
	{
		// WARNING - Less Accurate!
		XYZtoPanTilt( X, Y, Z, PanTenthDegrees, TiltTenthDegrees );
	}

	// ROBOT_LOG( TRUE, "LOOK AT XYZ: Pan = %d, Tilt = %d\n", PanTenthDegrees/10, TiltTenthDegrees/10 )
	SetHeadPosition( nOwner, PanTenthDegrees, TiltTenthDegrees, NOP );
	ExecutePosition( nOwner );
}



//-----------------------------------------------------------------------------
// Name: ServoTiltDegreesToLaserYTenthInches
// Desc: Calculates distance from the front of the robot to the laser line (Y)
// for any given angle
//-----------------------------------------------------------------------------

double HeadControl::ServoTiltDegreesToLaserYTenthInches( double ServoTiltDegrees )
{
	// Height is based upon height to servo + angle of head
	double CameraHeightAboveFloor = CAMERA_SERVO_HEIGHT_ABOVE_GROUND_TENTH_INCHES + 
		( cos(DEGREES_TO_RADIANS * ServoTiltDegrees) * CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES );

	double CameraDistanceFromFront = CAMERA_SERVO_DISTANCE_FROM_FRONT_TENTH_INCHES +  
		( sin(DEGREES_TO_RADIANS * ServoTiltDegrees ) * CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES);

	double InsideAngle = 90 + ServoTiltDegrees;
	double Y = (CameraHeightAboveFloor * tan(DEGREES_TO_RADIANS*InsideAngle) );
	Y = Y - CameraDistanceFromFront;

	return Y;
}


//-----------------------------------------------------------------------------
// Name: YtoTilt
// Desc: Calculates tilt needed to look exactly at the given distance
// in front of the robot in TenthInches
//-----------------------------------------------------------------------------
double HeadControl::YtoTilt( double TargetYTenthInches )
{
	//////////////////////////////////////////////////
	// Get approx Tilt

	// Convert from TENTH INCHES to double inches and Normalize camera position relative to Robot
	double StartingY = TargetYTenthInches + CAMERA_SERVO_DISTANCE_FROM_FRONT_TENTH_INCHES;

	double StartingHeightAboveFloor = CAMERA_SERVO_HEIGHT_ABOVE_GROUND_TENTH_INCHES + CAMERA_HEIGHT_ABOVE_CAMERA_SERVO_TENTH_INCHES;

	// Calculate angle
	double StartingTiltAngle = atan2( StartingY, StartingHeightAboveFloor ) * RADIANS_TO_DEGREES;	// Opposite / Adjacent

	StartingTiltAngle -= 90.0;

/***
	// convert from circle (0 deg = straight up) to Robot (+=up, -=down)
	if( StartingTiltAngle >= 90 )
	{	// between straight forward and straight down in front		
		StartingTiltAngle = (StartingTiltAngle - 90.0);
	}
	else if( StartingTiltAngle >= 0 )
	{	// between straight up and straight forward
		StartingTiltAngle = 90 - StartingTiltAngle;
	}
	else if( StartingTiltAngle >= -90 )
	{	// between straight up and straight back
		StartingTiltAngle = 90 + StartingTiltAngle;
	}
	else
	{	// between straight back and straight down in back
		StartingTiltAngle = StartingTiltAngle + 90;
	}
**/

	//////////////////////////////////////////////////
	// Now, Iterate for exact value

	//ROBOT_LOG( TRUE,"StartingAngle = %3.1f  ", StartingTiltAngle)

	double TestAngle = StartingTiltAngle;
	double TestY = ServoTiltDegreesToLaserYTenthInches( TestAngle );
	int iterations1 = 0;
	int iterations2 = 0;

	while( TestY < TargetYTenthInches )
	{
		TestAngle += 1.0;	// increase by 1 degree
		TestY = ServoTiltDegreesToLaserYTenthInches( TestAngle );
		iterations1++;
	}

	// Ok, found within 1 degree
	TestY -= 1.0; // back up a bit
	while( TestY < TargetYTenthInches )
	{
		TestAngle += 0.1;	// decrease by .1 degree
		TestY = ServoTiltDegreesToLaserYTenthInches( TestAngle );
		iterations2++;
	}

	// Ok, found to within 1/10 degree!
	//ROBOT_LOG( TRUE,"DEBUG =======> StartAngle = %3.1f,  iterations1 = %d, iterations2 = %d\n", StartingTiltAngle, iterations1, iterations2)
	return TestAngle;

	// convert from double to integer in Tenthdegrees
	//TiltTenthDegrees = (int)(TiltAngle * 10.0);

}
//-----------------------------------------------------------------------------
// Name: XYZtoPanTilt
// Desc: Calculates Pan/Tilt needed to point camera at position X,Y,Z in space in TENTH INCHES
//-----------------------------------------------------------------------------
void HeadControl::XYZtoPanTilt( int X, int Y, int Z, int &PanTenthDegrees, int &TiltTenthDegrees )
{

	// Normalize camera position relative to Robot
	double doubleX = ((double)X );
	double doubleY = ((double)Y ) + CAMERA_CENTER_DIST_FROM_ROBOT_FRONT_TENTH_INCHES;
	double doubleZ = ((double)Z ) - CAMERA_CENTER_HEIGHT_ABOVE_GROUND_TENTH_INCHES;

	// Calculate angle
	double PanAngle =  atan2( doubleX, doubleY ) * RADIANS_TO_DEGREES;	// Opposite / Adjacent
	double TiltAngle = atan2( doubleY, doubleZ ) * RADIANS_TO_DEGREES;	// Opposite / Adjacent

	// convert from circle (0 deg = straight up) to Robot (+=up, -=down)
	if( TiltAngle >= 90 )
	{
		// between straight forward and straight down in front
		TiltAngle = TiltAngle - 90.0;
		TiltAngle *= -1.0;
	}
	else if( TiltAngle >= 0 )
	{
		// between straight up and straight forward
		TiltAngle = 90 - TiltAngle;
	}
	else if( TiltAngle >= -90 )
	{
		// between straight up and straight back
		TiltAngle = 90 + TiltAngle;
	}
	else
	{
		// between straight back and straight down in back
		TiltAngle = TiltAngle + 90;
	}


	// convert from double to integer in Tenthdegrees
	PanTenthDegrees =  (int)(PanAngle * 10.0);	
	TiltTenthDegrees = (int)(TiltAngle * 10.0);

}

//-----------------------------------------------------------------------------
// Name: ExecutePosition
// Desc: Commits movement for all servos setup previously
//-----------------------------------------------------------------------------
void HeadControl::ExecutePosition( int  nOwner )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	// set the "watchdog" timer, in case the servo does not reach the commanded position
	gHeadMoveTimeout = m_HeadMoveTimeLimit;

	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	//SendCommand( WM_ROBOT_SET_HEAD_POSITION, 0, FALSE );	// FALSE = Don't Set Speed
	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_HEAD_POSITION), 0, (DWORD)FALSE ); // FALSE = Don't Set Speed
}

//-----------------------------------------------------------------------------
// Name: ExecutePositionAndSpeed
// Desc: Commits movement and speed for all servos setup previously
//-----------------------------------------------------------------------------
void HeadControl::ExecutePositionAndSpeed( int  nOwner )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	// set the "watchdog" timer, in case the servo does not reach the commanded position
	gHeadMoveTimeout = m_HeadMoveTimeLimit;

	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	// SendCommand( WM_ROBOT_SET_HEAD_POSITION, 0, TRUE );		// TRUE = Set Speed
	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_HEAD_POSITION), 0, (DWORD)TRUE ); // TRUE = Set Speed
}

//-----------------------------------------------------------------------------
// Name: Stop
// Desc: Tell Head servos to stop moving immediately
//-----------------------------------------------------------------------------
void HeadControl::Stop( int  nOwner )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_CAMERA_STOP), 0, 0 );
}
//-----------------------------------------------------------------------------
// Name: SetServoCompliance
// Desc: Set compliance slope of the specified servo
// This is how accurate the servo will track to requested, but may also put extra stress on the servo.
//-----------------------------------------------------------------------------
void HeadControl::SetServoCompliance( int  nOwner, int  nServo, int  Compliance )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}
	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_SERVO_COMPLIANCE), (DWORD)nServo, (DWORD)Compliance );
}

//-----------------------------------------------------------------------------
// Name: SetServoTimeout
// Desc: Set max time move will wait to reach commanded position before timing out
// This prevents the robot from hanging if a servo can't reach commanded position
//-----------------------------------------------------------------------------
void HeadControl::SetServoTimeout( int  nOwner, int  Timeout )
{
	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}
	m_HeadMoveTimeLimit = Timeout;
}

//-----------------------------------------------------------------------------
// Name: Check Head Position
// Desc: Are we there yet?  Check to see if Head has reached commanded position
// Verbose flag will print out the servo that is blocking move complete status
//-----------------------------------------------------------------------------
BOOL HeadControl::CheckHeadPosition( BOOL verbose )
{
	if( (INVALID_HANDLE_VALUE == g_hArduinoCommPort) || (1 == ROBOT_SIMULATION_MODE) )
	{
		return TRUE;	// Handle simulation mode (no servos!)
	}

	if( 0 == gHeadMoveTimeout )
	{
		if( verbose ) ROBOT_LOG( TRUE,"CheckHeadPosition: Head Move Timed Out!  Proceeding to next step...\n")
		return TRUE;
	}

	int DeltaPan = g_BulkServoCmd[m_PanServoID].PositionTenthDegrees -
		g_BulkServoStatus[m_PanServoID].PositionTenthDegrees;

	int DeltaTilt = g_BulkServoCmd[m_TiltServoID].PositionTenthDegrees -
		g_BulkServoStatus[m_TiltServoID].PositionTenthDegrees;

	int DeltaSideTilt = g_BulkServoCmd[m_SideTiltServoID].PositionTenthDegrees -
		g_BulkServoStatus[m_SideTiltServoID].PositionTenthDegrees;


	// Report out first blocking joint

	if( abs(DeltaTilt) > HEAD_JOINT_DELTA_MAX_TENTHDEGREES )
	{
		if( verbose )
		{
			// In verbose mode, show why we are not yet in position
			ROBOT_LOG( TRUE,"CheckHeadPosition: DeltaTilt = %d Degrees\n", DeltaTilt/10)
		}
	}
	else if( abs(DeltaSideTilt) > HEAD_JOINT_DELTA_MAX_TENTHDEGREES )
	{
		if( verbose )
		{
			// In verbose mode, show why we are not yet in position
			ROBOT_LOG( TRUE,"CheckHeadPosition: DeltaSideTilt = %d Degrees\n", DeltaSideTilt/10)
		}
	}
	else if( abs(DeltaPan) > HEAD_JOINT_DELTA_MAX_TENTHDEGREES )
	{
		if( verbose )
		{
			// In verbose mode, show why we are not yet in position
			ROBOT_LOG( TRUE,"CheckHeadPosition: DeltaPan = %d Degrees\n", DeltaPan/10)
		}
	}
	else
	{
		if( verbose )
		{
			// All joints within target range!  Yipee!
			ROBOT_LOG( TRUE,"CheckHeadPosition: Head in Target Position\n")
		}
		return TRUE;
	}

	return FALSE;	// Did not reach target position yet
}


//-----------------------------------------------------------------------------
// Name: Get Head Position
// Desc: Returns current position in TENTH DEGREES for each joint
//-----------------------------------------------------------------------------
void HeadControl::GetHeadPosition( int &Pan, int &Tilt, int &SideTilt  )
{
	Pan = g_BulkServoStatus[m_PanServoID].PositionTenthDegrees;

	Tilt = g_BulkServoStatus[m_TiltServoID].PositionTenthDegrees;

	SideTilt = g_BulkServoStatus[m_SideTiltServoID].PositionTenthDegrees;
}

//-----------------------------------------------------------------------------
// Name: Get Head Speed
// Desc: Returns current speed of primary head servos
//-----------------------------------------------------------------------------
void HeadControl::GetHeadSpeed( int  &PanSpeed, int  &TiltSpeed, int  &SideTiltSpeed )
{
	PanSpeed = g_BulkServoStatus[m_PanServoID].Speed;

	TiltSpeed = g_BulkServoStatus[m_TiltServoID].Speed;

	SideTiltSpeed = g_BulkServoStatus[m_SideTiltServoID].Speed;
}

//-----------------------------------------------------------------------------
// Name: Get Pan Position
// Desc: Returns current Pan position in TENTH DEGREES
//-----------------------------------------------------------------------------
int HeadControl::GetPanPosition(  )
{
	return g_BulkServoStatus[m_PanServoID].PositionTenthDegrees;
}

//-----------------------------------------------------------------------------
// Name: Get Tilt Position
// Desc: Returns current Tilt position in TENTH DEGREES
//-----------------------------------------------------------------------------
int HeadControl::GetTiltPosition(  )
{
	return g_BulkServoStatus[m_TiltServoID].PositionTenthDegrees;
}

//-----------------------------------------------------------------------------
// Name: Get Side Tilt Position
// Desc: Returns current SideTilt position in TENTH DEGREES
//-----------------------------------------------------------------------------
int HeadControl::GetSideTiltPosition(  )
{
	return( g_BulkServoStatus[m_SideTiltServoID].PositionTenthDegrees );
}


//-----------------------------------------------------------------------------
// Name: Check Servo Limit
// Desc: Checks to make sure commanded value is within limits (in TenthDegrees)
//-----------------------------------------------------------------------------

void HeadControl::CheckServoLimit( int  ServoID, int &PositionTenthDegrees )
{
	// Make sure servo command is within Degree limits

	switch( ServoID )
	{
		case DYNA_CAMERA_PAN_SERVO_ID:
		{
			if( PositionTenthDegrees > CAMERA_PAN_TENTHDEGREES_MAX_RIGHT )
			{
				ROBOT_DISPLAY( TRUE, "HeadControl: DYNA_CAMERA_PAN_SERVO_ID hit MAX TENTHDEGREES RIGHT limit" )
				PositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_RIGHT;
			}
			else if( PositionTenthDegrees < CAMERA_PAN_TENTHDEGREES_MAX_LEFT)
			{
				ROBOT_DISPLAY( TRUE, "HeadControl: ERROR! DYNA_CAMERA_PAN_SERVO_ID hit MAX TENTHDEGREES LEFT limit" )
				PositionTenthDegrees = CAMERA_PAN_TENTHDEGREES_MAX_LEFT;
			}
			break;
		}
		case DYNA_CAMERA_TILT_SERVO_ID:
		{
			if( PositionTenthDegrees > CAMERA_TILT_TENTHDEGREES_MAX_UP )
			{
				//ROBOT_DISPLAY( TRUE, "HeadControl: DYNA_CAMERA_TILT_SERVO_ID hit MAX TENTHDEGREES UP limit" )
				PositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_UP;
			}
			else if( PositionTenthDegrees < CAMERA_TILT_TENTHDEGREES_MAX_DOWN)
			{
				ROBOT_DISPLAY( TRUE, "HeadControl: DYNA_CAMERA_TILT_SERVO_ID hit MAX TENTHDEGREES DOWN limit" )
				//PositionTenthDegrees = CAMERA_TILT_TENTHDEGREES_MAX_DOWN;
			}
			break;
		}
		case DYNA_CAMERA_SIDETILT_SERVO_ID:
		{
			if( PositionTenthDegrees > CAMERA_SIDETILT_TENTHDEGREES_MAX_LEFT )
			{
				ROBOT_DISPLAY( TRUE, "HeadControl: DYNA_CAMERA_SIDETILT_SERVO_ID hit MAX TENTHDEGREES LEFT limit" )
				PositionTenthDegrees = CAMERA_SIDETILT_TENTHDEGREES_MAX_LEFT;
			}
			else if( PositionTenthDegrees < CAMERA_SIDETILT_TENTHDEGREES_MAX_RIGHT)
			{
				ROBOT_DISPLAY( TRUE, "HeadControl: DYNA_CAMERA_SIDETILT_SERVO_ID hit MAX TENTHDEGREES RIGHT limit" )
				PositionTenthDegrees = CAMERA_SIDETILT_TENTHDEGREES_MAX_RIGHT;
			}
			break;
		}
		
		default:
		{
			ROBOT_ASSERT(0); // unhandled servo
		}
	}
}


//-----------------------------------------------------------------------------
// Name: Check Tracking Limits
// Desc: Keep camera toward front of robot, so it does not go off in la la land!
//       Only applies for some operations, such as motion or laser tracking
//-----------------------------------------------------------------------------
void HeadControl::CheckTrackingLimits( int &NewPan, int &NewTilt)	// values in TenthDegrees
{
	if( NewPan > CAMERA_PAN_TENTHDEGREES_MAX_TRACK_RIGHT )
	{
		ROBOT_LOG( TRUE," CAMERAMODULE: Pan Track Limit Reached: CAMERA_PAN_TENTHDEGREES_MAX_RIGHT\n")
		NewPan = CAMERA_PAN_TENTHDEGREES_MAX_TRACK_RIGHT;
	}
	else if( NewPan < CAMERA_PAN_TENTHDEGREES_MAX_TRACK_LEFT )
	{
		ROBOT_LOG( TRUE," CAMERAMODULE: Pan Track Limit Reached: CAMERA_PAN_TENTHDEGREES_MAX_LEFT\n")
		NewPan = CAMERA_PAN_TENTHDEGREES_MAX_TRACK_LEFT;
	}

	// Make sure Tilt command is within Camera limits
	if( NewTilt > CAMERA_TILT_TENTHDEGREES_MAX_TRACK_UP )
	{
		ROBOT_LOG( TRUE," CAMERAMODULE: Tilt Track Limit Reached: CAMERA_TILT_TENTHDEGREES_MAX_UP\n")
		NewTilt = CAMERA_TILT_TENTHDEGREES_MAX_TRACK_UP;
	}
	else if( NewTilt < CAMERA_TILT_TENTHDEGREES_MAX_TRACK_DOWN )
	{
		ROBOT_LOG( TRUE," CAMERAMODULE: Tilt Track Limit Reached: CAMERA_TILT_TENTHDEGREES_MAX_DOWN\n")
		//NewTilt = CAMERA_TILT_TENTHDEGREES_MAX_TRACK_DOWN;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////
// HEAD CONTROL ARBITRATOR
// Tracks who is control of the head, assigning control based upon a strict priority
/////////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
// Name: Check And Set Owner
// Desc: Checks for current owner, and if requesting module is higher priority
// changes to new module.  Returns TRUE if module got ownership.
//-----------------------------------------------------------------------------
BOOL HeadControl::CheckAndSetOwner( int  NewOwner )
{
	if( NewOwner == gHeadCurrentOwner )
	{
		// Owner reasking for control
		// Silently reset timer for how long to maintain control
		gHeadOwnerTimer = 50;	// 1/10 seconds
		return TRUE;
	}

	CString MsgString, NewOwnerName, CurrentOwnerName;
	HeadOwnerNumberToName( gHeadCurrentOwner, CurrentOwnerName );
	HeadOwnerNumberToName( NewOwner, NewOwnerName );

	if( (0 == gHeadOwnerTimer) && (HEAD_OWNER_NONE != gHeadCurrentOwner) )
	{
		// ownership timed out
		MsgString.Format( "Head Control: Owner %s has timed out", CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		gHeadCurrentOwner = HEAD_OWNER_NONE;
	}

	if( NewOwner < gHeadCurrentOwner )
	{
		// Higher priority module has control
/*		if( NewOwner > HEAD_OWNER_RANDOM_MOVEMENT ) // don't report random movement requests for control, too noisy
		{
			MsgString.Format( "Request by %s Rejected: %s has control",NewOwnerName, CurrentOwnerName );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			ROBOT_LOG( TRUE,"gHeadOwnerTimer = %d\n", gHeadOwnerTimer)
		}
*/
		return FALSE;
	}

	// No one with higher priority has control.
	if( NewOwner > gHeadCurrentOwner )
	{
		MsgString.Format( "Head Control: NewOwner %s has taken control from %s",NewOwnerName, CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );
		gHeadCurrentOwner = NewOwner;
		// TODO = Need to see if all functions that use USER control are REALLY users. (Need to pass owner with WM_ command)
		if( HEAD_OWNER_USER_CONTROL == NewOwner )
		{
			ROBOT_LOG( TRUE,"TODO - Is this really HEAD_OWNER_USER_CONTROL?\n")
		}
		
	}

	// Requesting module has control now
	// Set timer for how long to maintain control
	gHeadOwnerTimer = 50;	// 1/10 seconds


	// Indicate on the GUI who is in control
	// SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_DRIVE_MODULE_OWNER, Module );
	return TRUE;

}

BOOL HeadControl::IsOwner( int  NewOwner )
{
	if( NewOwner != gHeadCurrentOwner )
	{
		// Not current owner
		return FALSE;
	}
	return TRUE;
}

BOOL HeadControl::ReleaseOwner( int  NewOwner )
{
	if( NewOwner < gHeadCurrentOwner )
	{
		// Higher priority module has control already so just ignore.
/*		CString MsgString, ModuleName, OwnerName;
		HeadOwnerNumberToName( Module, ModuleName );
		HeadOwnerNumberToName( gHeadCurrentOwner, OwnerName );
		MsgString.Format( "Module %s releasing control, but %s already has it", ModuleName, OwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
*/
		return FALSE;
	}

	// No one with higher priority has control.  Release ownership.
	CString MsgString, NewOwnerName;
	HeadOwnerNumberToName( NewOwner, NewOwnerName );
	MsgString.Format( "Head Control: Owner %s releasing control", NewOwnerName );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	gHeadCurrentOwner = HEAD_OWNER_NONE;
	return TRUE;

}



#endif // ROBOT_SERVER - This module used for Robot Server only

