// Drive Control Module.cpp
// modified on Teleop

#include "stdafx.h"
#include <math.h>
#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "RobotType.h"
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Global to this module (defined as extern in Module.h)

/*
///__itt_string_handle* pshMotorCommand = __itt_string_handle_create("Motor Command, Speed, Turn = ");
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
// Drive Control Arbitrator
///////////////////////////////////////////////////////////////////////////////
CDriveControlModule::CDriveControlModule()
{
	m_MotorsPaused		= FALSE;
	m_Command			= HW_SET_MOTOR_STOP;
	m_LastMotorCommand	= HW_SET_MOTOR_STOP;
	m_Speed				= SPEED_STOP;
	m_LastMotorSpeed	= SPEED_STOP;
	m_Turn				= TURN_CENTER;
	m_LastMotorTurn		= TURN_CENTER;
	m_Acceleration		= ACCELERATION_MEDIUM;
	m_BrakeLastTachometer = 0;

	m_SpeedRequested	= SPEED_STOP; 
	m_SpeedCurrentSetting = SPEED_STOP;
	m_TachometerTarget	= 0;
	m_MotorForward		= TRUE;
	m_UserOverrideMode	= FALSE;

	m_DistHigh = 0;
	m_DistLow = 0;
	m_DriveOwner = NO_MODULE;	// Which Module is currently in charge

	m_MoveDistanceRemaining = 0;
	m_TurnRotationRemaining = 0;
	m_LastCompassHeading = 0;
	m_StopAfterMoveDistance = TRUE;
	m_StopAfterTurnRotation = TRUE;
	m_ModulesSuppressed = NO_MODULE;	// All modules enabled

	gMotorSpeedTimer = 0;
	gBrakeTimer = 0;

	m_TrackCompassHeading = FALSE;
	m_TargetCompassHeading = 0;

	memset( m_ModuleCommandRequest, 0x00, (sizeof(int) * NUMBER_OF_MOTOR_OWNERS) );
	memset( m_ModuleSpeedRequest, 0x00, (sizeof(int) * NUMBER_OF_MOTOR_OWNERS) );
	memset( m_ModuleTurnRequest, 0x00, (sizeof(int) * NUMBER_OF_MOTOR_OWNERS) );

}

///////////////////////////////////////////////////////////////////////////////
// Functions related to Arduino Status
///////////////////////////////////////////////////////////////////////////////

/*
void CDriveControlModule::SetPicAck(int  Command, int  SpeedServoVal, int  TurnServoVal)
{
	// Called by Sensor Module. Provides lastest Arduino status info
	// Arduino returns absolute Servo values. Need to convert to logical values

	m_LastMotorCommand = Command;

	//TODO-SERVO fix this to match direction of servo travel
	m_LastMotorSpeed = SpeedServoVal - SPEED_SERVO_CENTER;
	m_LastMotorTurn = TurnServoVal  - TURN_SERVO_CENTER;
}
*/


void CDriveControlModule::UpdateTurnRotation(double RotationAngleAmount)
{
	// See if previously TurnRotation (rotate in place) has been completed
	// RotationAngleAmount is negative if turning left
	// m_TurnRotationRemaining is always positive (direction independent)

	if( (0 == m_TurnRotationRemaining) && !m_TrackCompassHeading )
	{
		// no controlled turn in progress
		return;
	}

	if( m_TurnRotationRemaining > 361.0 )
	{
		ROBOT_ASSERT(0);
	}

	if( m_TrackCompassHeading || (1 == ROBOT_SIMULATION_MODE) )
	{
		// Turn in progress, and controlled by Compass Heading
		int TurnDelta = 0;

		if( m_Turn > 0 )
		{
			// Turning Right
			if( (g_pFullSensorStatus->CompassHeading < 90) && (m_LastCompassHeading > 270) )
			{
				// Crossing 360 degrees during this sample
				TurnDelta = (g_pFullSensorStatus->CompassHeading +360) - m_LastCompassHeading;
			}
			else
			{
				TurnDelta = g_pFullSensorStatus->CompassHeading - m_LastCompassHeading;
			}
		}
		else
		{
			// Turning Left
			if( (g_pFullSensorStatus->CompassHeading > 270) && (m_LastCompassHeading < 90) )
			{
				// Crossing 360 degrees during this sample
				TurnDelta = (m_LastCompassHeading +360) - g_pFullSensorStatus->CompassHeading;
			}
			else
			{
				TurnDelta = m_LastCompassHeading - g_pFullSensorStatus->CompassHeading;
			}
		}

		m_TurnRotationRemaining -= TurnDelta;
		m_LastCompassHeading = g_pFullSensorStatus->CompassHeading;

		// OK Calculations completed, now see if we are done with the turn
		if( m_TurnRotationRemaining <= TURN_ROTATION_COMPLETE_COMPENSATION )
		{
			// Done! Note that any overshoot simply makes m_TurnRotationRemaining go negative
			ROBOT_LOG( TRUE, "COMPASS TURN DONE: Compass = %3.2f, Remaining = %3.2f\n", 
					g_pFullSensorStatus->CompassHeading, m_TurnRotationRemaining )
		}
		else
		{
			ROBOT_LOG( TRUE, "DEBUG: Turn Compass = %d, Remaining = %3.2f\n", 
				g_pFullSensorStatus->CompassHeading, m_TurnRotationRemaining )

			// Slow down when getting close, so we don't overshoot (compass updates are slow to settle)
			if( (m_TurnRotationRemaining > TURN_ROTATION_COMPLETE_COMPENSATION) && 
				(m_TurnRotationRemaining < 30.0)  && (abs(m_Turn) > TURN_RIGHT_MED_SLOW) )
			{
				m_Turn = (int)( (float)m_Turn * 0.90 ); // percent to reduce speed of turn
				ExecuteCommand();
			}
		}
	}
	else
	{ 
		// Turn in progress, controlled by motor odometers (probably?)
		double PosRotationAngleAmount = fabs(RotationAngleAmount);
		m_TurnRotationRemaining = m_TurnRotationRemaining - PosRotationAngleAmount;
			ROBOT_LOG( TRUE, "DEBUG UpdateTurnRotation: Turn Amt = %3.2f, Remaining = %3.2f\n", 
				PosRotationAngleAmount, m_TurnRotationRemaining )
	}

	if( m_TurnRotationRemaining <= TURN_ROTATION_COMPLETE_COMPENSATION )
	{
		// Turn completed!
		m_TurnRotationRemaining = 0;
		m_TrackCompassHeading = FALSE;
		ROBOT_DISPLAY( TRUE, "SetTurnRotation Completed" )

		if( m_StopAfterTurnRotation )
		{

			// keep track of last setting each module requested, as hints for other modules
			m_ModuleCommandRequest[m_DriveOwner] = HW_SET_MOTOR_STOP;
			m_ModuleSpeedRequest[m_DriveOwner] = SPEED_STOP;
			m_ModuleTurnRequest[m_DriveOwner] = TURN_CENTER;

			m_Command = HW_SET_MOTOR_STOP;
			m_Speed = SPEED_STOP;
			m_Turn = TURN_CENTER;
			ExecuteCommand();
		}
	}
}

void CDriveControlModule::UpdateMoveDistance(double OdometerUpdateTenthInches)
{
	// See if previously request move has been completed
	if( m_MoveDistanceRemaining > 0 )
	{
		// A SetMoveDistance was/is in progress
		m_MoveDistanceRemaining = m_MoveDistanceRemaining - fabs(OdometerUpdateTenthInches);
		ROBOT_LOG( TRUE, "DEBUG UpdateMoveDistance: Move Amt = %3.2f, Remaining = %3.2f Inches\n", 
			fabs(OdometerUpdateTenthInches/10.0), m_MoveDistanceRemaining/10.0 )

		if( m_MoveDistanceRemaining <= MOVE_DISTANCE_COASTING_COMPENSATION )
		{
			// Move completed!
			m_MoveDistanceRemaining = 0;
			ROBOT_DISPLAY( TRUE, "SetMoveDistance Completed" )
			if( m_StopAfterMoveDistance )
			{
				// keep track of last setting each module requested, as hints for other modules
				m_ModuleCommandRequest[m_DriveOwner] = HW_SET_MOTOR_STOP;
				m_ModuleSpeedRequest[m_DriveOwner] = SPEED_STOP;
				m_ModuleTurnRequest[m_DriveOwner] = TURN_CENTER;

				m_Command = HW_SET_MOTOR_STOP;
				m_Speed = SPEED_STOP;
				m_Turn = TURN_CENTER;
				ExecuteCommand();
			}
		}
	}
}

BOOL CDriveControlModule::RobotStopped()
{
	// See if the robot is moving or stopped

	#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL		// For ER1 Robot
		if( g_pFullSensorStatus->OdometerUpdateTenthInches  < 5.0 )	// TODO-MUST What value should be here?
		{
			ROBOT_LOG( TRUE, "DEBUG STOPPED???==========> %3.2f, TRUE!\n", g_pFullSensorStatus->OdometerUpdateTenthInches )
			return TRUE;
		}
		else
		{
			ROBOT_LOG( TRUE, "DEBUG STOPPED???==========> %3.2f, FALSE!\n", g_pFullSensorStatus->OdometerUpdateTenthInches )
			return FALSE;
		}
	#else
		if( (g_pFullSensorStatus->Tachometer  < 2) && (0 == m_MoveDistanceRemaining) && (0 == m_TurnRotationRemaining) )
			return TRUE;
		else
			return FALSE;

	#endif

}




///////////////////////////////////////////////////////////////////////////////
// Commands to Motors
///////////////////////////////////////////////////////////////////////////////
// 
// Negotiate speed and turn between modules, then execute
// Note: lower priority modules can suppress higher priority modules
// and lower priority modules can also do safer things:
// - Request to go slower (even stop)
// - Request to turn tighter


void CDriveControlModule::Stop(int  Module, int Acceleration)
{
	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_MOTOR_STOP;
	m_ModuleSpeedRequest[Module] = SPEED_STOP;
	//m_ModuleTurnRequest[Module] = ;


	int Result = CheckAndSetOwner(Module);
	if( (MODULE_OWNERSHIP_REQUEST_SUCCESS == Result)   ||
		(MODULE_HIGHER_PRIORITY_HAS_CONTROL == Result) )  // allow lower priority modules to still call stop
	{
		m_DriveOwner = Module;	
		m_Command = HW_SET_MOTOR_STOP;
		m_Speed = SPEED_STOP;
		m_Acceleration = Acceleration;
		m_MoveDistanceRemaining = 0;
		m_TrackCompassHeading = 0;
		m_TurnRotationRemaining = 0;
		m_LastCompassHeading = 0;

		CString MsgString, ModuleName;
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "%s: New Stop Command Requested" ,ModuleName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();
	}
}

void CDriveControlModule::Brake(int  Module, int Acceleration)
{
	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_MOTOR_BRAKE;
	m_ModuleSpeedRequest[Module] = SPEED_STOP;
	//m_ModuleTurnRequest[Module] = TURN_CENTER;

	// Used only for Seeker CarBot.  Note: Once a Brake operation starts, the Pic won't abort until done.
	int Result = CheckAndSetOwner(Module);
	if( (MODULE_OWNERSHIP_REQUEST_SUCCESS == Result)   ||
		(MODULE_HIGHER_PRIORITY_HAS_CONTROL == Result) )  // allow lower priority modules to still call brake
	{
		m_DriveOwner = Module;	
		m_Command = HW_SET_MOTOR_BRAKE;
		m_Speed = SPEED_STOP;			// After braking, speed set by Arduino to Stop
		m_Acceleration = Acceleration;
		m_Turn = TURN_CENTER;			// And wheel centered
		m_TrackCompassHeading = 0;
		m_TurnRotationRemaining = 0;
		CString MsgString, ModuleName;
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "%s: New Brake Command Requested",ModuleName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();
	}
}


BOOL CDriveControlModule::SetTurnRotation( int  Module, int Speed, int Turn, int TurnAmountDegrees, BOOL StopAfterTurn )
{
	// Note: A SetTurnRotation can be over ridden by another command, which will abort the turn.
	// Lower priority modules will NOT override.  They can't do a set turn if higher priory has control
	// Turn direction set by TurnAmountDegrees.  Amount set by ABS TurnAmountDegrees

	int  AbsTurnAmountDegrees = abs(TurnAmountDegrees);
	m_LastCompassHeading = g_pFullSensorStatus->CompassHeading;  // Track changes at each update
	m_MoveDistanceRemaining = 0;
	m_TurnRotationRemaining = 0;

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	m_ModuleSpeedRequest[Module] = SPEED_STOP;
	m_ModuleTurnRequest[Module] = Turn;

	if( MODULE_OWNERSHIP_REQUEST_SUCCESS != CheckAndSetOwner(Module) )
	{
		return FALSE; // Failed to set turn
	}

	CString MsgString, ModuleName;
	m_DriveOwner = Module;	
	m_Command = HW_SET_SPEED_AND_TURN;
	m_Acceleration = ACCELERATION_MEDIUM;
	// TODO - Speed not currently used, assumes rotate in place.  add calc to allow complex turns?

	if( AbsTurnAmountDegrees > 360 )
	{
		ROBOT_ASSERT(0);
	}
	m_StopAfterTurnRotation = StopAfterTurn;
	m_Speed = 0;
	m_Turn = Turn;	// This sets the speed/tightness of the turn
	m_TurnRotationRemaining = AbsTurnAmountDegrees; // Non-Zero tells other modules that turn not completed.

	//TODO-MUST #if( SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP )
		// Using the Kobuki base.  Gyro is best accuracy!
		m_TrackCompassHeading = TRUE; // Indicate heading in progress
		if( m_Turn < 0 )
		{
			// Turning Left
			m_TargetCompassHeading = g_pFullSensorStatus->CompassHeading - TurnAmountDegrees;
			if( m_TargetCompassHeading < 0 )
			{
				m_TargetCompassHeading += 360;
			}
		}
		else
		{	// Turning Right
			m_TargetCompassHeading = g_pFullSensorStatus->CompassHeading + TurnAmountDegrees;
			if( m_TargetCompassHeading >= 360 )
			{
				m_TargetCompassHeading -= 360;
			}
		}
		ROBOT_LOG( TRUE,  "DEBUG TURN Heading = %d", m_TargetCompassHeading )
	//#endif
	
	ModuleNumberToName( Module, ModuleName );
	MsgString.Format( "%s: New SetTurnRotation Requested: Speed:%d Turn %d RotationAmountDegrees %d",
		ModuleName, m_Speed, m_Turn, AbsTurnAmountDegrees );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	ExecuteCommand();
	return TRUE;
}

BOOL CDriveControlModule::SetTurnToCompassDirection( int  Module, int Speed, int Turn, int DesiredCompassHeading, BOOL StopAfterTurn )
{
	// Note: A SetTurnRotation can be over ridden by another command, which will abort the turn.
	// Lower priority modules will NOT override.  They can't do a setturn if higher priory has control

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	m_ModuleSpeedRequest[Module] = SPEED_STOP;
	m_ModuleTurnRequest[Module] = Turn;

	if( MODULE_OWNERSHIP_REQUEST_SUCCESS != CheckAndSetOwner(Module) )
	{
		return FALSE; // Failed to set mode
	}

	CString MsgString, ModuleName;
	m_DriveOwner = Module;	
	m_Command = HW_SET_SPEED_AND_TURN;
	m_Acceleration = ACCELERATION_MEDIUM;

	// TODO - Speed not currently used, assumes rotate in place.  add calc to allow complex turns?

	// calculate direction of turn.  Positive is right turn, Negative is Left turn
	int TurnDegrees = CalculateTurn(g_pFullSensorStatus->CompassHeading, DesiredCompassHeading);

	m_StopAfterTurnRotation = StopAfterTurn;
	m_Speed = 0;
	if( TurnDegrees < 0 )
	{
		// turn Left
		m_Turn = abs(Turn) * -1;	// This sets the speed/tightness of the turn
	}
	else
	{
		// turn Right
		m_Turn = abs(Turn);	// This sets the speed/tightness of the turn
	}

	m_TrackCompassHeading = TRUE; // Indicate heading in progress
	m_TargetCompassHeading = DesiredCompassHeading;
	m_TurnRotationRemaining = abs(TurnDegrees); // Non-Zero tells other modules that turn not completed.
	ModuleNumberToName( Module, ModuleName );
	MsgString.Format( "%s: New SetTurnToCompassDirection Requested: Speed:%d Turn %d RotationAmountDegrees %d",
		ModuleName, m_Speed, m_Turn, TurnDegrees );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	ExecuteCommand();

	return TRUE;
}


BOOL CDriveControlModule::SetMoveDistance( int  Module, int Speed, int Turn, int  DistanceTenthInches, BOOL StopAfterMove, int Acceleration )
{
	// Note: A SetMoveDistance can be over ridden by another command, which will abort the move dist.
	// Lower priority modules will NOT override.  They can't do a SetMove if higher priory has control

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	m_ModuleSpeedRequest[Module] = Speed;
	m_ModuleTurnRequest[Module] = Turn;

	if( MODULE_OWNERSHIP_REQUEST_SUCCESS != CheckAndSetOwner(Module) )
	{
		return FALSE; // failed to set move
	}

	if( Speed != 0 )
	{
		CString MsgString, ModuleName;
		m_DriveOwner = Module;	
		m_Command = HW_SET_SPEED_AND_TURN;
		m_Speed = Speed;
		m_Acceleration = Acceleration;
		m_TrackCompassHeading = 0;
		m_TurnRotationRemaining = 0;
		m_StopAfterMoveDistance = StopAfterMove;
		m_Turn = Turn;
		m_MoveDistanceRemaining = DistanceTenthInches; // Non-Zero tells other modules that move not completed.
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "%s: New SetMoveDistance Requested: Speed:%d Turn %d DistanceTenthInches %3.1f",ModuleName, m_Speed, m_Turn, m_MoveDistanceRemaining );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();
	}
	else
	{
		m_MoveDistanceRemaining = 0;
	}
	return TRUE;
}

void CDriveControlModule::SetSpeed( int  Module, int Speed, int Acceleration )
{

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	m_ModuleSpeedRequest[Module] = Speed;
	//m_ModuleTurnRequest[Module] = Turn;

	int Result = CheckAndSetOwner(Module);
	if( MODULE_OWNERSHIP_REQUEST_SUCCESS == Result )
	{
		CString MsgString, ModuleName;
		m_DriveOwner = Module;	
		m_Command = HW_SET_SPEED_AND_TURN;	
		// Set the speed.  Drive control keeps track of the last speed
		m_Speed = Speed;
		m_Acceleration = Acceleration;
		m_MoveDistanceRemaining = 0;
		m_TurnRotationRemaining = 0;
		m_TrackCompassHeading = 0;
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "%s: New Speed Requested: %d", ModuleName, Speed );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();

	}
	else if( MODULE_HIGHER_PRIORITY_HAS_CONTROL == Result )  
	{
		// allow lower priority modules to still call this function under limited circumstances
		if( (0 == m_TurnRotationRemaining) && // don't interrupt "SetTurn" commands
			(0 == m_MoveDistanceRemaining) ) // don't interrupt "SetMove" commands
		{
			if( ((m_Speed > 0) && (Speed < m_Speed)) ||	// moving forward
				((m_Speed < 0) && (Speed > m_Speed))  )	// moving backward
			{
				// Slower speed requested by lower priory module
				CString MsgString, ModuleName;
				m_DriveOwner = Module;	
				m_Command = HW_SET_SPEED_AND_TURN;	
				// Set the speed.  Drive control keeps track of the last speed
				m_Speed = Speed;
				m_Acceleration = Acceleration;
				m_MoveDistanceRemaining = 0;
				m_TurnRotationRemaining = 0;
				m_TrackCompassHeading = 0;
				ModuleNumberToName( Module, ModuleName );
				MsgString.Format( "%s: New Speed Requested: %d", ModuleName, Speed );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				ExecuteCommand();
			}
		}
	}
}

void CDriveControlModule::SetTurn( int  Module, int Turn, int Acceleration )
{

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	//m_ModuleSpeedRequest[Module] = Speed;
	m_ModuleTurnRequest[Module] = Turn;


	CString MsgString, ModuleName;
	int Result = CheckAndSetOwner(Module);

	if( MODULE_OWNERSHIP_REQUEST_SUCCESS == Result) 
	{
		m_DriveOwner = Module;
		m_Command = HW_SET_SPEED_AND_TURN;
		if( Turn < TURN_LEFT_MAX ) Turn = TURN_LEFT_MAX;	// Negative = Left
		if( Turn > TURN_RIGHT_MAX ) Turn = TURN_RIGHT_MAX;	// Positive = Right
		m_Turn = Turn ;	// Set the turn.  Drive control keeps track of the last speed
		m_MoveDistanceRemaining = 0;
		m_TrackCompassHeading = 0;
		m_TurnRotationRemaining = 0;
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "%s: New Turn Requested: %d",ModuleName, Turn );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();

	}	
	else if( MODULE_HIGHER_PRIORITY_HAS_CONTROL == Result )  
	{
		// allow lower priority modules to still call this function under limited circumstances
		if( (0 == m_TurnRotationRemaining) && // don't interrupt "SetTurn" commands
			(0 == m_MoveDistanceRemaining) ) // don't interrupt "SetMove" commands
		{
			if( ((m_Turn > 0) && (Turn > m_Turn)) ||	// turning right
				((m_Turn < 0) && (Turn < m_Turn))  )	// turning left
			{
				// Extra turn requested by lower priory module
				m_DriveOwner = Module;
				m_Command = HW_SET_SPEED_AND_TURN;
				if( Turn < TURN_LEFT_MAX ) Turn = TURN_LEFT_MAX;	// Negative = Left
				if( Turn > TURN_RIGHT_MAX ) Turn = TURN_RIGHT_MAX;	// Positive = Right
				m_Turn = Turn ;	// Set the turn.  Drive control keeps track of the last speed
				m_MoveDistanceRemaining = 0;
				m_TrackCompassHeading = 0;
				m_TurnRotationRemaining = 0;
				ModuleNumberToName( Module, ModuleName );
				MsgString.Format( "%s: New Low Priority Turn Requested: %d",ModuleName, Turn );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				ExecuteCommand();
			}
		}
	}
}

void CDriveControlModule::SetSpeedAndTurn( int  Module, int Speed, int Turn, int Acceleration )
{
	CString MsgString, ModuleName;

	// keep track of last setting each module requested, as hints for other modules
	m_ModuleCommandRequest[Module] = HW_SET_SPEED_AND_TURN;
	m_ModuleSpeedRequest[Module] = Speed;
	m_ModuleTurnRequest[Module] = Turn;

	int Result = CheckAndSetOwner(Module);

	if( MODULE_OWNERSHIP_REQUEST_SUCCESS == Result )
	{
		// New owner taking control
		m_DriveOwner = Module;	
		m_Command = HW_SET_SPEED_AND_TURN;
		m_Speed = Speed;
		m_Acceleration = Acceleration;
		m_Turn = Turn;
		m_MoveDistanceRemaining = 0;
		m_TrackCompassHeading = 0;
		m_TurnRotationRemaining = 0;
		CString MsgString, ModuleName;
		ModuleNumberToName( Module, ModuleName );
		MsgString.Format( "New Owner %s: New Speed: %d and Turn %d Requested",
			ModuleName, Speed, Turn );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		ExecuteCommand();

	}
	else if( MODULE_ALREADY_IS_OWNER == Result  )
	{
		if( (m_Speed != Speed) || (m_Turn != Turn) || (m_Acceleration != Acceleration) )
		{
			m_DriveOwner = Module;	
			m_Command = HW_SET_SPEED_AND_TURN;
			m_Speed = Speed;
			m_Acceleration = Acceleration;
			m_Turn = Turn;
			m_MoveDistanceRemaining = 0;
			m_TrackCompassHeading = 0;
			m_TurnRotationRemaining = 0;
			CString MsgString, ModuleName;
			ModuleNumberToName( Module, ModuleName );
			MsgString.Format( "%s: New Speed: %d and Turn %d Requested",
				ModuleName, Speed, Turn );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			ExecuteCommand();
		}

	}
	else if( MODULE_HIGHER_PRIORITY_HAS_CONTROL == Result )  
	{
		// allow lower priority modules to still call this function under limited circumstances
		if( (0 == m_TurnRotationRemaining) && // don't interrupt "SetTurn" commands
			(0 == m_MoveDistanceRemaining) ) // don't interrupt "SetMove" commands
		{
			BOOL ChangeRequested = FALSE;

			if( ((m_Speed > 0) && (Speed < m_Speed)) ||	// moving forward
				((m_Speed < 0) && (Speed > m_Speed))  )	// moving backward
			{
				// Slower speed requested by lower priory module
				ChangeRequested = TRUE;
				CString MsgString, ModuleName;
				// Set the speed.  Drive control keeps track of the last speed
				m_Speed = Speed;
				m_Acceleration = Acceleration;
				ModuleNumberToName( Module, ModuleName );
				MsgString.Format( "%s: New Low Priority Speed Requested: %d", ModuleName, Speed );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}

			if( ((m_Turn > 0) && (Turn > m_Turn)) ||	// turning right
				((m_Turn < 0) && (Turn < m_Turn))  )	// turning left
			{
				// Extra turn requested by lower priory module
				ChangeRequested = TRUE;
				if( Turn < TURN_LEFT_MAX ) Turn = TURN_LEFT_MAX;	// Negative = Left
				if( Turn > TURN_RIGHT_MAX ) Turn = TURN_RIGHT_MAX;	// Positive = Right
				m_Turn = Turn ;	// Set the turn.  Drive control keeps track of the last speed
				ModuleNumberToName( Module, ModuleName );
				MsgString.Format( "%s: New Low Priority Turn Requested: %d",ModuleName, Turn );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}

			if( ChangeRequested )
			{
				// Change requested by lower priory module
				m_DriveOwner = Module;
				m_Command = HW_SET_SPEED_AND_TURN;
				m_MoveDistanceRemaining = 0;
				m_TrackCompassHeading = 0;
				m_TurnRotationRemaining = 0;
				ExecuteCommand();
			}
		}
	}
}


BOOL CDriveControlModule::CheckAndSetOwner( int  NewOwner )
{
	if( NewOwner == m_DriveOwner )
	{
		// Owner reasking for control
		// Silently reset timer for how long to maintain control
		gDriveControlOwnerTimer = 20;	// 1/10 seconds
		return MODULE_ALREADY_IS_OWNER;
	}

	CString MsgString, NewOwnerName, CurrentOwnerName;
	ModuleNumberToName( NewOwner, NewOwnerName );
	ModuleNumberToName( m_DriveOwner, CurrentOwnerName );

	if( (0 == gDriveControlOwnerTimer) && (NO_MODULE != m_DriveOwner) )
	{
		// ownership timed out
		MsgString.Format( "Motor Control: Owner %s has timed out", CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		m_ModuleCommandRequest[m_DriveOwner] = HW_SET_MOTOR_STOP;
		m_ModuleSpeedRequest[m_DriveOwner] = SPEED_STOP;
		m_ModuleTurnRequest[m_DriveOwner] = TURN_CENTER;

		m_DriveOwner = NO_MODULE;
	}


	if( (0 != (m_ModulesSuppressed & NewOwner)) )
	{
		// Module is suppressed
		ModuleNumberToName( NewOwner, NewOwnerName );
		ModuleNumberToName( m_DriveOwner, CurrentOwnerName );
		MsgString.Format("Request by %s Rejected: Suppressed by %s",NewOwnerName, CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		return MODULE_SUPPRESSED;
	}

	if( NewOwner < m_DriveOwner )
	{
		// Higher priority module has control
		ModuleNumberToName( NewOwner, NewOwnerName );
		ModuleNumberToName( m_DriveOwner, CurrentOwnerName );
		MsgString.Format( "Request by %s Rejected: %s has control",NewOwnerName, CurrentOwnerName );
		ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )
		ROBOT_LOG( TRUE,"gDriveControlOwnerTimer = %d\n", gDriveControlOwnerTimer)
		return MODULE_HIGHER_PRIORITY_HAS_CONTROL;
	}

	// No one with higher priority has control.
	if( NewOwner > m_DriveOwner )
	{
		ROBOT_DISPLAY( TRUE, "==================================================================")
		ModuleNumberToName( NewOwner, NewOwnerName );
		ModuleNumberToName( m_DriveOwner, CurrentOwnerName );
		MsgString.Format( "Module %s has taken control from %s",NewOwnerName, CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

		// Requesting module has control now
		m_DriveOwner = NewOwner;
		// Set timer for how long to maintain control
		gDriveControlOwnerTimer = 20;	// 1/10 seconds

//		Beep(1000, 200);	// Beep to indicate Module change
	}

	// Requesting module has control now
	// Indicate on the GUI who is in control
	SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_DRIVE_MODULE_OWNER, m_DriveOwner );
	return MODULE_OWNERSHIP_REQUEST_SUCCESS;

}

BOOL CDriveControlModule::IsOwner( int  Module )
{
	if( (0 != (m_ModulesSuppressed & Module)) ||
		(Module != m_DriveOwner ) )
	{
		// Not current owner
		return FALSE;
	}
	return TRUE;
}

BOOL CDriveControlModule::ReleaseOwner( int  Module )
{
	if( Module < m_DriveOwner )
	{
		// Higher priority module has control already so just ignore.
/*		CString MsgString, ModuleName, OwnerName;
		ModuleNumberToName( Module, ModuleName );
		ModuleNumberToName( m_DriveOwner, OwnerName );
		MsgString.Format( "Module %s releasing control, but %s already has it",ModuleName, OwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
*/
		return FALSE;
	}

	// No one with higher priority has control.  Release Drive owner.
	CString MsgString, ModuleName;
	ModuleNumberToName( Module, ModuleName );
	MsgString.Format( "Module %s releasing control",ModuleName );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	ROBOT_DISPLAY( TRUE, "==================================================================")
	m_DriveOwner = NO_MODULE;
	return TRUE;

}

void CDriveControlModule::SuppressModule( int  Module )
{
	if( m_DriveOwner == Module )
	{
		// This module is currently the drive owner.  Release control then suppress it.
		ReleaseOwner( Module );
	}
	m_ModulesSuppressed |= Module;	// Set the flag
}

void CDriveControlModule::EnableModule( int  Module )
{
	m_ModulesSuppressed &= !Module;	// Clear the flag
}


///////////////////////////////////////////////////////////////////////////////
// ExecuteCommand - sends the actual command to the Servo controller as needed
//
void CDriveControlModule::ExecuteCommand()
{
	CString MsgString, OwnerName;

	if( g_GlobalPause )
	{
		// Global Pause requested!
		if( !m_MotorsPaused )
		{
			// Not yet paused, so stop everyting now
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, ACCELERATION_INSTANT);
			m_MotorsPaused = TRUE;
		}
		return;
	}
	else if( m_MotorsPaused )
	{
			// Motors were paused, now resume
			m_MotorsPaused = FALSE;
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "RESUMING FROM PAUSE!  Module Owner = %s  Command: %d Speed = %d Turn = %d", 
				OwnerName, m_Command, (m_Speed), (m_Turn));
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				// TODO DAVE - Accel
			DWORD SpeedAndAcc = MAKELONG((WORD)(signed short)m_Speed, m_Acceleration ); // Low,High.  Default functions will cast to BYTE, chopping off Acc
			SendHardwareCmd( HW_SET_SPEED_AND_TURN, SpeedAndAcc, m_Turn );		
			return;
	}



//////////////////////////////////////////////////////////////////////////////////////////
#if( (MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL) )


	if( (m_LastMotorCommand != m_Command )	||
		(m_LastMotorSpeed != m_Speed )		||
		(m_LastMotorTurn != m_Turn )	)
	{
		int commandparams[3];
		commandparams[0] =  m_Command;
		commandparams[1] =  m_Speed;
		commandparams[2] =  m_Turn;
		///__itt_marker(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_marker_scope_task);
		///__itt_metadata_add(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_metadata_s32, 3, (void*)&commandparams);

		// new command is pending, and needs to be sent
		if( m_LastMotorCommand != m_Command )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Cmd = %02X, Ack Cmd = %02X\n", m_Command, m_LastMotorCommand )
		if( m_LastMotorSpeed != m_Speed )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Speed = %d, Ack Speed = %d\n", m_Speed, m_LastMotorSpeed )
		if( m_LastMotorTurn != m_Turn )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )

		if( HW_SET_MOTOR_BRAKE == m_Command )
		{
			// Send the BRAKE command.
			ROBOT_DISPLAY( TRUE, "Sending STOP/BRAKE command")
			SendHardwareCmd(HW_SET_MOTOR_BRAKE, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else if( HW_SET_MOTOR_STOP == m_Command )
		{
			// Send the STOP command.
			ROBOT_DISPLAY( TRUE, "Sending STOP command")
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else
		{
			// Regular Motor commands
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending command: %d Speed = %d Turn = %d", 
				OwnerName, m_Command, (m_Speed), (m_Turn));
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			SendHardwareCmd( HW_SET_SPEED_AND_TURN, m_Speed, m_Turn );		
		}

		m_LastMotorCommand = m_Command;
		m_LastMotorSpeed = m_Speed;
		m_LastMotorTurn = m_Turn;

	}
	else
	{
		// No new Speed command.  Call the Speed Control function to adjust speed as needed 
		//TODO-LOKI! ENABLE THIS, after filling in the values for  SpeedToTachTable

		#if( MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL ) 
			SendHardwareCmd( HW_UPDATE_MOTOR_SPEED_CONTROL, 0, 0 ); // Unused params		
		#endif
	}


//////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL

	if( (m_LastMotorCommand != m_Command )	||
		(m_LastMotorSpeed != m_Speed )		||
		(m_LastMotorTurn != m_Turn )	)
	{
		int commandparams[3];
		commandparams[0] =  m_Command;
		commandparams[1] =  m_Speed;
		commandparams[2] =  m_Turn;
		///__itt_marker(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_marker_scope_task);
		///__itt_metadata_add(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_metadata_s32, 3, (void*)&commandparams);

		// new command is pending, and needs to be sent
		if( m_LastMotorCommand != m_Command )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Cmd = %02X, Ack Cmd = %02X\n", m_Command, m_LastMotorCommand )
		if( m_LastMotorSpeed != m_Speed )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Speed = %d, Ack Speed = %d\n", m_Speed, m_LastMotorSpeed )
		if( m_LastMotorTurn != m_Turn )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )

		if( HW_SET_MOTOR_BRAKE == m_Command )
		{
			// Send the BRAKE command.
			ROBOT_DISPLAY( TRUE, "Sending STOP/BRAKE command")
			SendHardwareCmd(HW_SET_MOTOR_BRAKE, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else if( HW_SET_MOTOR_STOP == m_Command )
		{
			// Send the STOP command.
			ROBOT_DISPLAY( TRUE, "Sending STOP command")
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else
		{
			// Regular Motor commands
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending command: %d Speed = %d Turn = %d", 
				OwnerName, m_Command, (m_Speed), (m_Turn));
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			SendHardwareCmd( HW_SET_SPEED_AND_TURN, m_Speed, m_Turn );		
		}

		m_LastMotorCommand = m_Command;
		m_LastMotorSpeed = m_Speed;
		m_LastMotorTurn = m_Turn;

	}
/////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL

	if( (m_LastMotorCommand != m_Command )	||
		(m_LastMotorSpeed != m_Speed )		||
		(m_LastMotorTurn != m_Turn )	)
	{
		int commandparams[3];
		commandparams[0] =  m_Command;
		commandparams[1] =  m_Speed;
		commandparams[2] =  m_Turn;
		///__itt_marker(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_marker_scope_task);
		///__itt_metadata_add(pDomainMotorControl, __itt_null, pshMotorCommand, __itt_metadata_s32, 3, (void*)&commandparams);

		// new command is pending, and needs to be sent
		if( m_LastMotorCommand != m_Command )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Kobuki Cmd = %02X, Ack Cmd = %02X\n", m_Command, m_LastMotorCommand )
		if( m_LastMotorSpeed != m_Speed )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Kobuki Speed = %d, Ack Speed = %d\n", m_Speed, m_LastMotorSpeed )
		if( m_LastMotorTurn != m_Turn )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Kobuki Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )

		if( HW_SET_MOTOR_BRAKE == m_Command )
		{
			// Send the BRAKE command.
			ROBOT_DISPLAY( TRUE, "Sending STOP/BRAKE command")
			SendHardwareCmd(HW_SET_MOTOR_BRAKE, 0, 0); // TODO - Add Acceleration!
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else if( HW_SET_MOTOR_STOP == m_Command )
		{
			// Send the STOP command.
			ROBOT_DISPLAY( TRUE, "Sending STOP command")
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, m_Acceleration); // Accel is sent as 2nd parameter for Stop
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else
		{
			// Regular Motor commands
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending command: %d Speed = %d Turn = %d", 
				OwnerName, m_Command, (m_Speed), (m_Turn));
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			#if MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL
				// Speed includes both speed and acceleration
				DWORD SpeedAndAcc = MAKELONG((WORD)(signed short)m_Speed, m_Acceleration ); // Low,High.  Default functions will cast to BYTE, chopping off Acc
				SendHardwareCmd( HW_SET_SPEED_AND_TURN, SpeedAndAcc, m_Turn );		
			#else
				SendHardwareCmd( HW_SET_SPEED_AND_TURN, m_Speed, m_Turn );		
			#endif

		}

		m_LastMotorCommand = m_Command;
		m_LastMotorSpeed = m_Speed;
		m_LastMotorTurn = m_Turn;

	}

//////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL		// For ER1 Robot

	if( (m_LastMotorCommand != m_Command )	||
		(m_LastMotorSpeed != m_Speed )		||
		(m_LastMotorTurn != m_Turn )	)
	{
		// new command is pending, and needs to be sent
		if( m_LastMotorCommand != m_Command )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Cmd = %02X, Ack Cmd = %02X\n", m_Command, m_LastMotorCommand )
		if( m_LastMotorSpeed != m_Speed )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Speed = %d, Ack Speed = %d\n", m_Speed, m_LastMotorSpeed )
		if( m_LastMotorTurn != m_Turn )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )

		if( HW_SET_MOTOR_BRAKE == m_Command )
		{
			// TODO-ER1: Send the BRAKE command. - For now, this is just a stop command!!
			ROBOT_DISPLAY( TRUE, "Sending STOP/BRAKE command")
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else if( HW_SET_MOTOR_STOP == m_Command )
		{
			// Send the STOP command.
			ROBOT_DISPLAY( TRUE, "Sending STOP command")
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
			m_Turn = TURN_CENTER;	// Brake or Stop automaticaly sets the turn to center
			m_Speed = SPEED_STOP;	// and the speed to Stop when done		
		}
		else
		{
			// Regular Motor commands
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending command: %d Speed = %d Turn = %d", 
				OwnerName, m_Command, (m_Speed), (m_Turn));
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			SendHardwareCmd( HW_SET_SPEED_AND_TURN, m_Speed, m_Turn );		
		}

		m_LastMotorCommand = m_Command;
		m_LastMotorSpeed = m_Speed;
		m_LastMotorTurn = m_Turn;

	}
//////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL		// For CarBot

	static BOOL bFirstSpeedCommand = TRUE;

	// TODO!!! TEST Brake, Speed Control, and Move set distance commands!

	// Handle Turn
	if( m_LastMotorTurn != m_Turn )
	{
		// new turn command is pending, and needs to be sent
		ROBOT_LOG( MOTOR_DBG, "Execute: New Turn = %d, Ack Turn = %d\n", m_Turn, m_LastMotorTurn )
		SendHardwareCmd( HW_SET_TURN, m_Turn,0 );		
		m_LastMotorTurn = m_Turn;
	}

	if( 0 != gBrakeTimer )
	{
		// Currently Braking.  Ignore any pending speed command (it will get handled 
		// next time, since it stays in m_Speed until overridden)
		BrakeControl();
		return;
	}

	// Handle Speed
	if( (m_LastMotorCommand != m_Command )	||
		(m_LastMotorSpeed != m_Speed )		)
	{
		// New Speed command

		if( m_LastMotorCommand != m_Command )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Cmd = %02X, Ack Cmd = %02X\n", m_Command, m_LastMotorCommand )
		if( m_LastMotorSpeed != m_Speed )
			ROBOT_LOG( MOTOR_DBG, "Execute: New Speed = %d, Ack Speed = %d\n", m_Speed, m_LastMotorSpeed )

		if( HW_SET_MOTOR_STOP == m_Command )
		{
			// Send the STOP command.
			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending STOP Command", OwnerName);
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			m_Speed = SPEED_STOP;
			m_SpeedRequested = SPEED_STOP;	 
			m_SpeedCurrentSetting = SPEED_STOP;
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
		}
		else if( HW_SET_MOTOR_BRAKE == m_Command )
		{
			// Send the Brake command first
			SendHardwareCmd(HW_SET_MOTOR_BRAKE, SPEED_BRAKE_AMOUNT, SPEED_BRAKE_TIME);
			m_Speed = SPEED_STOP;			// The speed will be Stop when done		
			m_SpeedRequested = SPEED_STOP;	 
			m_SpeedCurrentSetting = SPEED_STOP;	// for Speed control (disabled while braking)
			m_BrakeLastTachometer = MAX_INT;
			m_TachometerTarget = 0;

			// Then automatically set turn to center
			ROBOT_DISPLAY( TRUE, "Sending BRAKE command")
			m_Turn = TURN_CENTER;	
			m_LastMotorTurn = m_Turn;
			SendHardwareCmd( HW_SET_TURN, m_Turn,0 );		
		}
		else
		{
			// Regular Motor commands - could be a Joystick type Stop, though

			// on first real motor command, make sure the car is in gear!
			if( bFirstSpeedCommand && (0 != m_Speed) )		
			{
				bFirstSpeedCommand = FALSE;
				SendHardwareCmd( HW_SET_GEAR, 0, LOW_GEAR );	// Servo position for Low
				RobotSleep(500, pDomainControlThread);	// give time for the servo thread to put in gear
			}

			ModuleNumberToName( m_DriveOwner, OwnerName );
			MsgString.Format( "From %s Sending command: %d Speed = %d", OwnerName, m_Command, (m_Speed) );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

			m_SpeedRequested = m_Speed;	 
			m_SpeedCurrentSetting = m_SpeedRequested;	
			// TODO-LOKI?? m_SpeedCurrentSetting will be modified as needed by speed control
			SendHardwareCmd( HW_SET_SPEED, m_SpeedRequested,0 );	

			// Set direction flag used by Odometer, 
			// Assumes controling module will first stop before changing direction
			if( m_Speed < 0 )
				m_MotorForward = FALSE;
			else
				m_MotorForward = TRUE;

			// Set the target Tachometer reading for the speed control.
			m_TachometerTarget = SetTachometerTarget( m_Speed );

			gMotorSpeedTimer = NEW_SPEED_SET_TIME;		// give time for motors to respond before checking again

		}

		m_LastMotorCommand = m_Command;
		m_LastMotorSpeed = m_Speed;
	}
	else
	{
		// No new Speed command.  Call the SpeedControl function to adjust speed servo as needed 
		//TODO-CAR-MUST! ENABLE THIS, after filling in the values for  SpeedToTachTable
		//				SpeedControl();
	}
	
#else
	#error BAD MOTOR_CONTROL_TYPE
#endif
//////////////////////////////////////////////////////////////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
// Speed Control
// Only used on PC when using an external servo controller!
// Handles Speed control, Brake, and Move Set Distance.

///////////////////////////////////////////////////////////////////////////////


// TODO-CAR-MUST - fill in these values from observed Tach values at various speeds!
const int SpeedToTachTable[] =
//  0    1    2    3    4    5    6    7    8    9   10   11   12   13  14  15		// Target Speed
{   0,   0,   0,   10, 15,  17,  19,  30,  34,  36,  38,  40,  42,  44, 46, 48 };	// Target Tach value


int  CDriveControlModule::SetTachometerTarget( int TargetSpeed )
{
	// given a target Speed, return the corresponding target Tachometer value

	int  Tach = 0;
	int  Speed = abs( TargetSpeed );

	for( int index=0; index <= 15; index++ )	// 
	{
		if( Speed >= SpeedToTachTable[index]  )
		{
			Tach = index * 3;	// TODO-CAR-MUST supply the right multiplier here!
			return Tach;
		}
	}
	return Tach;
}


void CDriveControlModule::BrakeControl()
{
	if( 0 != gBrakeTimer )	// 100ms timer count down in globals.cpp
	{
		// Braking in progress!
		if( (g_pFullSensorStatus->Tachometer < 4)							||	// Stopped, or nearly so
			(g_pFullSensorStatus->Tachometer > (m_BrakeLastTachometer + 2)) )	// No longer slowing down!  Probably going in reverse!
		{
			// Note the fudge factor applied to avoid thinking braking is done too early
			// Stop the motor and indicate that Braking is done.

			ROBOT_DISPLAY( TRUE, "BrakeControl: Braking Complete." )
			m_Speed = SPEED_STOP;
			m_SpeedRequested = SPEED_STOP;	 
			m_SpeedCurrentSetting = SPEED_STOP;
			m_BrakeLastTachometer = 0;
			m_TachometerTarget = 0;
			gBrakeTimer = 0;
			SendHardwareCmd(HW_SET_MOTOR_STOP, 0, 0);
		}
		else if(g_pFullSensorStatus->Tachometer < m_BrakeLastTachometer)
		{
			// Still slowing down, continue to monitor motor speed
			m_BrakeLastTachometer = g_pFullSensorStatus->Tachometer;
		}
	}
}


void CDriveControlModule::SpeedControl()
{
	// This function called from ExecuteCommand(), which is called every time 
	// a WM_ROBOT_SENSOR_STATUS_READY update is processed.
	// However, it is NOT called when a new command get issued.

	// Member functtions used: m_Command, m_Speed, m_Turn?
	//		m_LastMotorCommand = m_Command;
	//		m_LastMotorSpeed = m_Speed;
	//		m_LastMotorTurn = m_Turn;
#if ( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
	// Used by Seeker Carbot
	// None of this is used if ER1 Pilot is being used for motor control.

	// #define SPEED_STOP				 	  0		//
	// #define SPEED_FULL_FWD				125		// WARNING! if you swap these, update m_CurrentSpeed in Module.cpp!!!
	// #define SPEED_FULL_REV			  (-125)

	///////////////////////////////////////////////////////////////////////////////
	// Calculate Speed Control feedback for the motors, and adjust speed
	// (Moved from Arduino code)

	

	#if (ROBOT_TYPE == CARBOT)
		// Check if Power is enabled with the RC "dead man switch" control
		// Note: to debug speed control, disable the this test
		if( !(g_pFullSensorStatus->StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
		{
			// RC Power Button not enabled.  Delay and disable speed control.
			// Otherwise, causes a speed surge when the power is restored again
			gMotorSpeedTimer = NEW_SPEED_SET_TIME * 2;// give time for motors to respond after power is restored
			return;
		}

		// See if we need to wait for brake delay to complete
		if( gBrakeTimer != 0 ) 
		{
			return;
		}

	#endif

	// See if we need to wait for a delay to complete
	if( gMotorSpeedTimer != 0 ) 
	{
		return;
	}


	if( (0 != m_TachometerTarget) && (0 != m_SpeedRequested)	&& 
		(HW_SET_MOTOR_STOP != m_Command) &&	(HW_SET_MOTOR_BRAKE != m_Command) )
	{
		// Requested speed not zero, and not braking, so perform motor speed control.

		ROBOT_LOG( TRUE,  " Called\n");
		return; //DEBUG

		if( g_pFullSensorStatus->Tachometer < m_TachometerTarget-1 )
		{
			// Going too slow
			if( m_MotorForward )
			{
				if( m_SpeedCurrentSetting < (m_SpeedRequested + MAX_SPEED_CORRECTION) )
				{
					m_SpeedCurrentSetting++;	// Speed up forward
				}
			}
			else
			{
				if( m_SpeedCurrentSetting > (m_SpeedRequested - MAX_SPEED_CORRECTION) )
				{
					m_SpeedCurrentSetting--;	// Speed up backward
				}
			}
			SendHardwareCmd( HW_SET_SPEED, m_SpeedCurrentSetting,0 );	
			gMotorSpeedTimer = SPEED_CHANGE_TIME;		// give time for motors to respond before checking again
		}
		else if( g_pFullSensorStatus->Tachometer > m_TachometerTarget+1 )
		{
			// Going too fast
			if( m_MotorForward )
			{
//						if( SpeedServo > HW_SPEED_STOP )	// don't overshoot center!
				if( ( m_SpeedRequested >= (SPEED_SERVO_CENTER+SPEED_FWD_SLOW) ) &&
					( m_SpeedCurrentSetting > (SPEED_SERVO_CENTER+SPEED_FWD_SLOW) ) )	// ??don't cycle on and off!
				{
					m_SpeedCurrentSetting--;	// Slow down forward
				}
			}
			else
			{
//						if( SpeedServo < HW_SPEED_STOP )	// don't overshoot center!
				if( (m_SpeedRequested <= (SPEED_SERVO_CENTER-SPEED_REV_SLOW)) &&
					(m_SpeedCurrentSetting < (SPEED_SERVO_CENTER-SPEED_REV_SLOW)) )	// ??don't cycle on and off!
				{
					m_SpeedCurrentSetting++;	// Slow down backward
				}
			}
			SendHardwareCmd( HW_SET_SPEED, m_SpeedCurrentSetting,0 );	
			gMotorSpeedTimer = SPEED_CHANGE_TIME;		// give time for motors to respond before checking again
		}
	}

	// Note for Set Move distance, see UpdateMoveDistance()

#endif //  ( (MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL) )
}


