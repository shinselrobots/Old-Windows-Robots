// SensorModule.cpp: SensorModule class implementation
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
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define BUMPER_DEBUG_ENABLED
#define ENABLE_BUMPERS 1 // Enable or disable reacting to bumper hits.  Disable if debugging or hardware problem

///////////////////////////////////////////////////////////////////////////////
//	MODULE: SensorModule

CSensorModule::CSensorModule( CDriveControlModule *pDriveControlModule )
{
	// Initialize members
	m_pDriveCtrl = pDriveControlModule;
	m_LastCompassHeading = 0;
	m_LastOdometerReadingR = 0;
	m_LastOdometerReadingL = 0;
	m_MotorPowerEnabled = FALSE;
	m_LandmarkUpdatesEnabled = FALSE;
	m_CompassCorrection = 0;

	int i;
	for( i=0; i < ACC_MAX_SAMPLES; i++ )
	{
		m_AccelHistoryX[i] = 0;
		m_AccelHistoryY[i] = 0;
	}
	m_AccelHistorySample = 0;

	for( i=0; i < GPS_HISTORY_MAX_SAMPLES; i++ )
	{
		m_GPSHistory[i] = i+20;	// Set at random angles to guarantee not in a straight line
	}
	m_GPSHistorySample = 0;	// Current Sample in the array
	m_GPSLastSample.x = 0;
	m_GPSLastSample.y = 0;

	m_GPSNavigationEnabled = FALSE;
	m_CliffSensorsEnabled = FALSE;

#if( ROBOT_HAS_RIGHT_ARM )
	m_pArmControlRight = new ArmControl( RIGHT_ARM );	// For arm position information
#endif

#if( ROBOT_HAS_LEFT_ARM )
	m_pArmControlLeft = new ArmControl( LEFT_ARM );	// For arm position information
#endif

	// Note: Initial robot location set by DEFAULT_ROBOT_START_POSITION_X,Y in Robot.cpp with other global initialization

}

CSensorModule::~CSensorModule()
{
	#if( ROBOT_HAS_RIGHT_ARM )
		SAFE_DELETE(m_pArmControlRight);
	#endif
	#if( ROBOT_HAS_LEFT_ARM )
		SAFE_DELETE(m_pArmControlLeft);
	#endif
}

void CSensorModule::SetCompassCorrection( int CompassCorrection )
{
	// Compensate compass readings based upon knowledge known by other modules, such as the NavModule
	if( (CompassCorrection > 360) || (CompassCorrection < -360) )
	{
		ROBOT_DISPLAY( TRUE, "ERROR!  Bad CompassCorrection!" )
		m_CompassCorrection = 0;
	}
	else
	{
		m_CompassCorrection = CompassCorrection;
	}
}


void CSensorModule::UpdateFromTwoOdometers( double OdometerUpdateTenthInchesL, double OdometerUpdateTenthInchesR)
{
	// Handle odometer updates for robots that have separate odometers for the right and left wheels
	// Such as LOKI with TReX Controller+Arduino, ER1 Controller, or Helmetbot
		
	// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
	g_SensorStatus.OdometerUpdateTenthInches = (OdometerUpdateTenthInchesL+OdometerUpdateTenthInchesR) / 2.0;

	// Update the system Odometer with average of the two wheels
	g_SensorStatus.OdometerTenthInches += g_SensorStatus.OdometerUpdateTenthInches;

		
	// Update the "Tach" display.
	g_SensorStatus.Tachometer = (int)(g_SensorStatus.OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

				ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
					OdometerUpdateTenthInchesL, OdometerUpdateTenthInchesR, g_SensorStatus.Tachometer, g_SensorStatus.OdometerUpdateTenthInches )
		
	// Update MoveDistance counter, in case a programmed move is in progress
	m_pDriveCtrl->UpdateMoveDistance( g_SensorStatus.OdometerUpdateTenthInches );

	// TODO - Currently using turn based upon the difference between the two wheels!
	// How reliable is the Compass?  Use compass instead / in addition to?
	// calculate turn based upon amount one wheel is moving more then other.
	// turn Right is positive, left negative, so 
	// TurnDistance = OdometerReadingL - OdometerReadingR;
	// Using the "small angle formula" (which works fine for big angles!)
	// If r = distance between wheels (assume right wheel is center of a circle, and left rotates around it)
	// segment / 2pi*r = theta / 360
	// theta = segment * 360 / 2pi*r;  
	// TurnAngle = TurnDistance * 360 / 2pi * DISTANCE_BETWEEN_WHEELS
	// TURN_CONSTANT = (360 / 2pi * DISTANCE_BETWEEN_WHEELS)
	// TurnAngle = TurnDistance * TURN_CONSTANT

	double TurnDistance = (OdometerUpdateTenthInchesL - OdometerUpdateTenthInchesR);
	const double Tconst = 0.45; // 0.61 TURN_CONSTANT;	// If turn overshoots, make this constant bigger
	double TurnAngleUpdate = (TurnDistance * Tconst);

	// Update TurnRotation counter, in case a programmed turn (rotate in place) is in progress
	m_pDriveCtrl->UpdateTurnRotation( TurnAngleUpdate );

	/***
	if( fabs(TurnAngleUpdate) > 5)
	{
		ROBOT_LOG( TRUE,  "BigTurn\n" )
		ROBOT_LOG( TRUE,  "const = %3.2f\n", Tconst )
	}
	***/
	if( -1 == g_SensorStatus.CalculatedMotorHeading )
	{
		// Heading not initialized
		// Wait for first motor movement, then grab the compass heading to initialize direction
		if( 0 != g_SensorStatus.OdometerUpdateTenthInches )
		{
			g_SensorStatus.CalculatedMotorHeading = m_LastCompassHeading;
		}
	}
	else
	{
		g_SensorStatus.CalculatedMotorHeading += TurnAngleUpdate;
		// TODO - see if "drifting" occurs due to round off error.  If so, keep angle to higher precision?
		//	ROBOT_LOG( TRUE,  "========> Turn %0.3f --> Calculated MOTOR HEADING: %0.2f\n", TurnAngleUpdate, g_SensorStatus.CalculatedMotorHeading)
	}


	UpdateLocation();	// Update the internal view of where we are in the world!

}


void CSensorModule::ProcessMessage( 
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	static BYTE fooAccelX = 0;
	static BYTE fooAccelY = 255;
	int nSensorNumber = 0;

	switch( uMsg ) 
	{
		case WM_ROBOT_ENABLE_OBJECT_NAV_UPDATE:
		{
			// Enable/disable updating the robot's current location based upon
			// landmarks (objects) that it sees, mapped to the GridMap
			g_bCmdRecognized = TRUE;
			// wParam = Enabled/Disabled
			// lParam = not used
			if( wParam )
			{
				// Enabled
				m_LandmarkUpdatesEnabled = TRUE;
				ROBOT_DISPLAY( TRUE, "Object Landmark Navigation Updates are Enabled" )
			}
			else
			{
				m_LandmarkUpdatesEnabled = FALSE;
				ROBOT_DISPLAY( TRUE, "Object Landmark Navigation Updates are Disabled" )
			}
			return;
		}
		break;


		case WM_ROBOT_SET_CURRENT_LOCATION:
		{
			// Command from user to override the current location of the robot
			g_bCmdRecognized = TRUE;
			// wParam = X
			// lParam = Y
			g_SensorStatus.CurrentLocation.x = (int)wParam;	// Get new Map absolute X,Y
			g_SensorStatus.CurrentLocation.y = (int)lParam;
			g_SensorStatus.CurrentLocationMotor.x = (int)wParam;	// Get new Map absolute X,Y
			g_SensorStatus.CurrentLocationMotor.y = (int)lParam;
			return;
		}
		break;


		case WM_ROBOT_ENABLE_GPS_PATH:
		{
			// Command from user to enable/disable the GPS in naviation
			g_bCmdRecognized = TRUE;
			// wParam = Enabled/Disabled
			// lParam = not used

			if( wParam )
			{
				// module enabled
				m_GPSNavigationEnabled = TRUE;
				ROBOT_DISPLAY( TRUE, "GPS Navigation Enabled" )
			}
			else
			{
				m_GPSNavigationEnabled = FALSE;
				ROBOT_DISPLAY( TRUE, "GPS Navigation Disabled" )
			}
			return;
		}
		break;


		case WM_ROBOT_ENABLE_CLIFF_SENSORS:
		{
			// Command from user to enable/disable the cliff sensors
			g_bCmdRecognized = TRUE;
			// wParam = Enabled/Disabled
			// lParam = not used

			if( wParam )
			{
				// module enabled
				m_CliffSensorsEnabled = TRUE;
				ROBOT_DISPLAY( TRUE,  "Cliff Sensors Enabled" )
			}
			else
			{
				m_CliffSensorsEnabled = FALSE;
				ROBOT_DISPLAY( TRUE, "Cliff Sensors Disabled" )
			}
			return;
		}
		break;


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

				//*********************************************************************
				// IMPORTANT!  This is where g_RawArduinoStatus gets copied to g_SensorStatus
				// Process Status block from the Arduino.  Contains IR data and other stuff
				// First thing is to initialize the processed Sensor Status block

				// ERROR: Don't do this, it corrupts current position: memset( &g_SensorStatus, SENSOR_DISABLED, sizeof(SENSOR_STATUS_T) );

				// Copy basic stuff first
				g_SensorStatus.StatusFlags =	g_RawArduinoStatus.StatusFlags;
				g_SensorStatus.LastError =		g_RawArduinoStatus.LastError;
				g_SensorStatus.DebugCode =		g_RawArduinoStatus.DebugCode;

				//ROBOT_LOG( TRUE,  "***********  DEBUG ArmBumpeL = %02X\n", g_RawArduinoStatus.ArmBumperR )

				///////////////////////////////////////////////////////////////////////////////////////////////

				g_SensorStatus.Tachometer =	g_RawArduinoStatus.Tachometer;
				//ROBOT_LOG( TRUE,  "----------------> DEBUG TACH = %d\n", g_RawArduinoStatus.Tachometer)
				//ROBOT_LOG( TRUE,  "----------------> DEBUG Battery = %d\n", g_RawArduinoStatus.Battery0)

				g_SensorStatus.HWBumper = g_RawArduinoStatus.Bumper;
				g_SensorStatus.IRBumper = 0;	// No IR Bumpers on CarBot

				if( 0 != g_SensorStatus.HWBumper )
				{
					CString MsgString;
					MsgString.Format("HW BUMPER HIT: %04lX", g_SensorStatus.HWBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}


				// MUST MATCH INVERT JUMPER SETTING!
				if( INVERT_RC_BUTTON_PWR_ENABLE )	// Change this in Globals.h if jumper changes!
				{
					if( (g_SensorStatus.StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
					{
						// bit is set, clear it
						g_SensorStatus.StatusFlags &= (0xFF^HW_STATUS_RC_BUTTON_PWR_ENABLE); // Clear Flag
					}
					else
					{
						// bit is clear, set it
						g_SensorStatus.StatusFlags |=  HW_STATUS_RC_BUTTON_PWR_ENABLE; // Set Flag
					}
				}

				/* TODO-CAR-MUST - need to calculate the mapping from units to Volts for Carbot!
				if( 0 != g_RawArduinoStatus.Battery0 )
				{
					// Y=MX+B: Voltage = (Arduino Value * 0.042) + 3.15
					double BatteryVoltageHundredths = ( ((double)g_RawArduinoStatus.Battery0 * 0.042) + 3.15 ) * 100;
					g_SensorStatus.Battery0 = (int ) BatteryVoltageHundredths;	// Pass in milivolts, so we don't need double
				}
				*/


				//////////////////////////////////////////////////////////////////////////
				// ACCELEROMETER

				// First, normalize Accelerometer readings, so level = 0,0
				int TempAccX = ACCEL_OFFSET_X - g_RawArduinoStatus.AccelX;
				int TempAccY = g_RawArduinoStatus.AccelY - ACCEL_OFFSET_Y;

				// TODO - Limit Accelerations to 30 degree slopes? (to ignore really stong start/stops?)
				// OR find median value (not average) - discard top and bottom numbers...
				// Now, average Acceleration values to remove instantaneous accelerations 
				// from motors and brakes

				// Overwrite oldest reading (always keep "N" most recent readings)
				m_AccelHistoryX[m_AccelHistorySample] = TempAccX;
				m_AccelHistoryY[m_AccelHistorySample] = TempAccY;
				m_AccelHistorySample++;

				if( m_AccelHistorySample >= ACC_MAX_SAMPLES )
				{
					m_AccelHistorySample = 0;	// wrap back to first element
				}

				// Get Average
				int SumX = 0;
				int SumY = 0;
				for( int i=0; i < ACC_MAX_SAMPLES; i++ )
				{
					SumX += m_AccelHistoryX[i];
					SumY += m_AccelHistoryY[i];
				}

				g_SensorStatus.AccelX = SumX / ACC_MAX_SAMPLES;
				g_SensorStatus.AccelY = SumY / ACC_MAX_SAMPLES;

				//	ROBOT_LOG( TRUE,  "DEBUG Accel Inst: X=%03d Y=%03d  Ave: X=%03d Y=%03d\n", 
				//		TempAccX, TempAccY, g_SensorStatus.AccelX, g_SensorStatus.AccelY )
				//	ROBOT_LOG( TRUE,  "DEBUG Accel: X=%03d  Y=%03d\n", g_RawArduinoStatus.AccelX, g_RawArduinoStatus.AccelY)

				int  SlopeDir;
				int  SlopeAmt;
				ConvertAccToTilt( g_SensorStatus.AccelX , g_SensorStatus.AccelY, SlopeDir, SlopeAmt);

				//////////////////////////////////////////////////////////////////////////
				// COMPASS
				// TempCompass is 0-3599 (359.9 degrees)
				int  Compass = 0;
				int  TempCompass = g_RawArduinoStatus.CompassHigh <<8;
				TempCompass += g_RawArduinoStatus.CompassLow;

				//	ROBOT_LOG( TRUE,  "DEBUG Compass %04X HEX\n", TempCompass)
				TempCompass = TempCompass / 10;
				//	ROBOT_LOG( TRUE,  "Compass = %03u  \n", TempCompass)

				if( TempCompass > 360 )
				{
					ROBOT_LOG( TRUE,  "ERROR!! - Sensor Module: Compass exceeds 360 degrees!\n" )
					//	TempCompass = 0;
				}

				// Correct Compass reading for tilt errors
				Compass = CompensateForTilt( TempCompass, SlopeDir, SlopeAmt );
				//	ROBOT_LOG( TRUE,  "DEBUG COMPASS Raw=%03d, New=%03d  Slope: Dir=%03d, Amt=%03d\n", 
				//		TempCompass, Compass, SlopeDir, SlopeAmt)

				if( 0 != m_CompassCorrection )
				{
					ROBOT_LOG( TRUE,  "CompassCorreciton = %d\n", m_CompassCorrection)
				}

				Compass += m_CompassCorrection;	// Add in any correction info available
				if( Compass > 360 ) Compass -= 360;
				g_SensorStatus.CompassHeading = Compass;


				//////////////////////////////////////////////////////////////////////////
				// ODOMETER
				// g_SensorStatus.Odometer is the distance since the last Odometer RESET
				// g_SensorStatus.OdometerUpdate is the distance since the last Odometer READING
				// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
				// RAW Arduino Status is in TICKS, Laptop Arduino Status is in TenthInches

				// Using Arduino for Odometry.  For ER1, see WM_ROBOT_ER1_ODOMETER_READY
				// This Robot uses a Single Odometer!
				// Save prior Odometer reading
				int LastOdometer = g_SensorStatus.Odometer;	// in Inches

				// "Unpack" Arduino Odometer reading (in Ticks)
				signed short TempOdometerTicks = g_RawArduinoStatus.OdometerHigh <<8;		// Shift the high byte ("hundreds and tens") over
				TempOdometerTicks += g_RawArduinoStatus.OdometerLow;					// Add in the low byte

				// Update the system Odometer
				g_SensorStatus.OdometerTenthInches = TempOdometerTicks * TICKS_PER_TENTH_INCH;
				//ROBOT_LOG( TRUE,  "DEBUG ODOM = %d\n", g_SensorStatus.Odometer)

				// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
				g_SensorStatus.OdometerUpdate = g_SensorStatus.Odometer - LastOdometer;
				// Update MoveDistance counter, in case a programmed move is in progress
				m_pDriveCtrl->UpdateMoveDistance( g_SensorStatus.OdometerUpdate );
				// (used on ER1: m_pDriveCtrl->UpdateTurnRotation( ?? );

				//////////////////////////////////////////////////////////////////////////
				// IR SENSORS
				#if DEBUG_IR == 1
					ROBOT_LOG( TRUE,  "IR (RAW)  = ");
				#endif

				// IR Sensor 0,3 = Side, Long Range
				// IR Sensor 1,2 = Forward, Long Range
				for( nSensorNumber=0; nSensorNumber<NUM_IR_SENSORS; nSensorNumber++ )
				{
					#if DEBUG_IR == 1
						ROBOT_LOG( TRUE,  " %u:%u",nSensorNumber, g_RawArduinoStatus.IR[nSensorNumber] );
					#endif
					// All are Long Range
					g_SensorStatus.IR[nSensorNumber] = ScaleLongRangeIR(g_RawArduinoStatus.IR[nSensorNumber], 0);
				}
				#if DEBUG_IR == 1
					ROBOT_LOG( TRUE,  "   INCHES= ")
					for( nSensorNumber=0; nSensorNumber<NUM_IR_SENSORS; nSensorNumber++ )
					{
						ROBOT_LOG( TRUE,  " %u:%u",nSensorNumber, g_SensorStatus.IR[nSensorNumber] );
					}
					ROBOT_LOG( TRUE,  "\n" );
				#endif


				//////////////////////////////////////////////////////////////////////////
				// ULTRASONIC
				#ifdef ULTRASONICS_NOT_DISABLED //  TODO-MUST!!

					for( nSensorNumber=0; nSensorNumber < NUM_US_SENSORS; nSensorNumber++ )
					{
						g_SensorStatus.US[nSensorNumber] = NO_OBJECT_IN_RANGE;
					}


		//			ROBOT_LOG( TRUE,  " DEBUG ULTRASONIC 0 = %d\n", g_RawArduinoStatus.US[0]);

					#if DEBUG_ULTRASONIC == 1
						ROBOT_LOG( TRUE,  "US (RAW)  = ");
					#endif

					// Get US Sensor Data			
					for( nSensorNumber=0; nSensorNumber < NUM_US_SENSORS; nSensorNumber++ )
					{

						if( HW_SENSOR_NO_OBJECT == g_RawArduinoStatus.US[nSensorNumber] )
						{
							#if DEBUG_ULTRASONIC == 1
								ROBOT_LOG( TRUE,  "   %01u:OOR",nSensorNumber );	// Out Of Range
							#endif
	 						g_SensorStatus.US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
						}
						else if( HW_SENSOR_NOT_ATTACHED == g_RawArduinoStatus.US[nSensorNumber] )
						{
							#if DEBUG_ULTRASONIC == 1
								ROBOT_LOG( TRUE,  "   %01u:NON",nSensorNumber );	// Sensor Not Attached
							#endif
	 						g_SensorStatus.US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
						}
						else if( HW_SENSOR_ERROR == g_RawArduinoStatus.US[nSensorNumber] )
						{
							#if DEBUG_ULTRASONIC == 1
								ROBOT_LOG( TRUE,  "   %01u:ERR",nSensorNumber );	// Read Error
							#endif
	 						g_SensorStatus.US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
						}
						else
						{
							#if DEBUG_ULTRASONIC == 1
								ROBOT_LOG( TRUE,  "   %01u:%03u",nSensorNumber, g_RawArduinoStatus.US[nSensorNumber] );
							#endif
							// Convert from raw value to inches
							///////////////////////////////////////////////////////////////////////////////
							// US Sensor 0   = Forward, Camera mounted
							// US Sensor 1-6 = Forward, bottom mounted, numbered from left to right (1-6)

	 						//g_SensorStatus.US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];	// For DEBUG
							g_SensorStatus.US[nSensorNumber] = ScaleSRF04UltraSonic(g_RawArduinoStatus.US[nSensorNumber]);
							#if DEBUG_ULTRASONIC == 1
					//			ROBOT_LOG( TRUE,  "==%03u", g_SensorStatus.US[nSensorNumber] );
							#endif

						}
						#if DEBUG_ULTRASONIC == 1
				//			ROBOT_LOG( TRUE,  "==%3u", g_SensorStatus.US[nSensorNumber] );
						#endif
					}
					#if DEBUG_ULTRASONIC == 1
								ROBOT_LOG( TRUE,  "\n" );
					#endif

					#if DEBUG_ULTRASONIC == 1
						ROBOT_LOG( TRUE,  "US INCHES =  ");
						for( nSensorNumber=0; nSensorNumber< NUM_US_SENSORS; nSensorNumber++ )
						{
							ROBOT_LOG( TRUE,  "  %01u:%3u\"",nSensorNumber, g_SensorStatus.US[nSensorNumber] );
						}
						ROBOT_LOG( TRUE,  "\n" ); //TenthInches???
					#endif

				#endif // ULTRASONICS_NOT_DISABLED
					
				//////////////////////////////////////////////////////////////////////////
				// Check Status Flags
				// See if Move completed
				//			if( 0 != (g_SensorStatus.StatusFlags & HW_STATUS_MOVE_DISTANCE_COMPLETE) )
				//			{
				//				// Move completed, Arduino stopped motors and is awaiting next command
				//				m_pDriveCtrl->SetMoveDistanceCompleted( SENSOR_MODULE ); // Tell Motor control move is done
				//			}

				if( (g_SensorStatus.StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
				{
					// Power is enabled
					if( !m_MotorPowerEnabled )
					{
						// State changed
						m_MotorPowerEnabled = TRUE;
						ROBOT_LOG( TRUE,  "RC_BUTTON_POWER is ON\n" );
					}
				}
				else
				{
					// Power is disabled
					if( m_MotorPowerEnabled )
					{
						// State changed
						m_MotorPowerEnabled = FALSE;
						ROBOT_LOG( TRUE,  "RC_BUTTON_POWER is OFF\n" )
					}
				}

				// For Debug:
				//ROBOT_LOG( MOTOR_DBG, "Status Flags = %02X\n", g_SensorStatus.StatusFlags )


				// Add line break in log if debugging IR or Ultrasonic sensors
				#if ( (DEBUG_IR == 1) || (DEBUG_ULTRASONIC == 1) )
						ROBOT_LOG( TRUE,  "\n" );
				#endif


				///////////////////////////////////////////////////////////////////////////
				// Done processing sensor data.
				// Now do sensor fusion to combine the data in a meaningful way
				DoSensorFusion();

				// Now post the status to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
					0 );									// Param2 = not used


			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

				//*********************************************************************
				// IMPORTANT!  This is where g_RawArduinoStatus gets copied to g_SensorStatus
				// Process Status block from the Arduino.  Contains IR data and other stuff
				// First thing is to initialize the processed Sensor Status block

				// ERROR: corrupts current position! memset( &g_SensorStatus, SENSOR_DISABLED, sizeof(SENSOR_STATUS_T) );	// Int all sensors to disabled

				// Copy basic stuff first
				g_SensorStatus.StatusFlags =	g_RawArduinoStatus.StatusFlags;
				g_SensorStatus.LastError =		g_RawArduinoStatus.LastError;
				g_SensorStatus.DebugCode =		g_RawArduinoStatus.DebugCode;

				//ROBOT_LOG( TRUE,  "***********  DEBUG ArmBumpeL = %02X\n", g_RawArduinoStatus.ArmBumperR )

				// Determine if thermal object found, and sumarize results
				#ifdef THERMAL_SENSOR_PROCESSING_ENABLED // DISABLED! 
					#define THERMAL_THRESHOLD  2  // amount above ambient floor needed to trigger a hit

					int Threshold = g_RawArduinoStatus.ThermalArray[0] + THERMAL_THRESHOLD;
					g_SensorStatus.ThermalPosition = 0;
					int MaxValue = 0;
					int Position = 0;

					// Look for strongest reading
					for(int i=1; i<9; i++)
					{
						if( (g_RawArduinoStatus.ThermalArray[i] > Threshold) && 
							(g_RawArduinoStatus.ThermalArray[i] > MaxValue) )
						{
							// Object detected.
							MaxValue = g_RawArduinoStatus.ThermalArray[i];
							Position = i;
						}
					}
					if( Position == 0 )
					{
						g_SensorStatus.ThermalPosition = 0;
					}
					else if( Position <= 4 )
					{
						// left side
						g_SensorStatus.ThermalPosition = Position - 5;	// Negative = left of center
						//ROBOT_LOG( TRUE,  "DEBUG Thermal Position = %d\n", g_SensorStatus.ThermalPosition)
					}
					else // Position >=5
					{
						// right side
						g_SensorStatus.ThermalPosition = Position - 4;	// Positive = right of center
						//ROBOT_LOG( TRUE,  "DEBUG Thermal Position = %d\n", g_SensorStatus.ThermalPosition)
					}

				#endif

				#ifdef THERMAL_SENSOR_DEBUG_ENABLED
					ROBOT_LOG( TRUE,  "DEBUG Thermal: ")
					for(int i=1; i<9; i++)
					{
						if( g_RawArduinoStatus.ThermalArray[i] <= Threshold )
						{
							g_SensorStatus.ThermalArray[i] = 0;
							ROBOT_LOG( TRUE,  "_ " )
						}
						else
						{
							g_SensorStatus.ThermalArray[i] = g_RawArduinoStatus.ThermalArray[i] - Threshold;
							ROBOT_LOG( TRUE,  "%d ", g_SensorStatus.ThermalArray[i] )
						}
					}
					ROBOT_LOG( TRUE,  "\n" )
				#endif

				g_SensorStatus.HWBumper = g_RawArduinoStatus.HWBumper;
				g_SensorStatus.IRBumper = g_RawArduinoStatus.IRBumper;
				g_SensorStatus.ArmBumperL = g_RawArduinoStatus.ArmBumperL;
				g_SensorStatus.ArmBumperR = g_RawArduinoStatus.ArmBumperR;

				if( 0 != g_SensorStatus.HWBumper && ENABLE_BUMPERS )
				{
					CString MsgString;
					MsgString.Format("HW BUMPER HIT: %04lX", g_SensorStatus.HWBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}

				#ifdef BUMPER_DEBUG_ENABLED
						if( 0 != g_SensorStatus.IRBumper )
					{
						CString MsgString;
						MsgString.Format("IR BUMPER HIT: %04lX", g_SensorStatus.IRBumper);
						ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
					}
					if( 0 != g_SensorStatus.ArmBumperR )
					{
						if( 2 == g_SensorStatus.ArmBumperR )
						{
							ROBOT_LOG( TRUE,  "CLAW!!!\n" )
							//Beep(1000, 200);
						}
						ROBOT_LOG( TRUE,  "Right Arm Bumper: %02X\n", g_RawArduinoStatus.ArmBumperR)
					}
				#endif

				if( 0 != g_RawArduinoStatus.Battery0 )
				{
					// Y=MX+B: Voltage = (Arduino Value * 0.041) + 7.0  See Sensor Tests.xlsx for graph
					double BatteryVoltageHundredths = ( ((double)g_RawArduinoStatus.Battery0 * 0.041) + 7.0 ) * 100.0;
					g_SensorStatus.Battery0 = (int ) BatteryVoltageHundredths;	// Pass in milivolts, so we don't need double
				}


				//////////////////////////////////////////////////////////////////////////
				// COMPASS
				// TempCompass is 0-3599 (359.9 degrees)
				int  Compass = 0;
				int  TempCompass = g_RawArduinoStatus.CompassHigh <<8;
				TempCompass += g_RawArduinoStatus.CompassLow;

				//	ROBOT_LOG( TRUE,  "DEBUG Compass %04X HEX\n", TempCompass)
				TempCompass = TempCompass / 10;
				//	ROBOT_LOG( TRUE,  "Compass = %03u  \n", TempCompass)

				if( TempCompass > 360 )
				{
					ROBOT_LOG( TRUE,  "ERROR!! - Sensor Module: Compass exceeds 360 degrees!\n" )
					//	TempCompass = 0;
				}

				// For Loki, compensate for Compass installed backwards
				Compass = TempCompass + 180;
				if( 0 != m_CompassCorrection )
				{
					ROBOT_LOG( TRUE,  "CompassCorreciton = %d\n", m_CompassCorrection)
				}

				Compass += m_CompassCorrection;	// Add in any correction info available
				if( Compass > 360 ) Compass -= 360;
				g_SensorStatus.CompassHeading = Compass;


				//////////////////////////////////////////////////////////////////////////
				// Commands from Android Phone (if connected)

				if( g_SensorStatus.AndroidConnected != (BOOL)g_RawArduinoStatus.AndroidConnected )
				{
					g_SensorStatus.AndroidConnected = (BOOL)g_RawArduinoStatus.AndroidConnected;
					if( g_SensorStatus.AndroidConnected )
					{
						ROBOT_LOG( TRUE,  "****** Android Connected! ******\n" )
					}
					else
					{
						ROBOT_LOG( TRUE,  "****** Android Disconnected! ******\n" )
						g_SensorStatus.AndroidCommand = 0;
						g_SensorStatus.AndroidCompass = 0;
						g_SensorStatus.AndroidPitch = 0;
						g_SensorStatus.AndroidRoll = 0;
					}
				}

				if( g_SensorStatus.AndroidConnected )
				{
					g_SensorStatus.AndroidCommand = g_RawArduinoStatus.AndroidCmd;
					g_SensorStatus.AndroidAccEnabled = g_RawArduinoStatus.AndroidAccEnabled;
					if( 0 != g_SensorStatus.AndroidCommand )
					{
						ROBOT_LOG( TRUE,  "DEBUG: AndroidCommand = %d", g_SensorStatus.AndroidCommand )
					}

					if( g_SensorStatus.AndroidAccEnabled )
					{
						// Accelerometer and compass measurements are in degrees
						signed short  TempAndroidCompass = g_RawArduinoStatus.AndroidCompassHigh <<8;
						TempAndroidCompass += g_RawArduinoStatus.AndroidCompassLow;
						g_SensorStatus.AndroidCompass = TempAndroidCompass;

						signed short  TempAndroidPitch = g_RawArduinoStatus.AndroidPitchHigh <<8;
						TempAndroidPitch += g_RawArduinoStatus.AndroidPitchLow;
						g_SensorStatus.AndroidPitch = TempAndroidPitch;

						signed short  TempAndroidRoll = g_RawArduinoStatus.AndroidRollHigh <<8;
						TempAndroidRoll += g_RawArduinoStatus.AndroidRollLow;
						g_SensorStatus.AndroidRoll = TempAndroidRoll;
						ROBOT_LOG( TRUE,  "DEBUG: AndroidCompass = %d", g_SensorStatus.AndroidCompass )
						ROBOT_LOG( TRUE,  "DEBUG: AndroidPitch = %d", g_SensorStatus.AndroidPitch )
						ROBOT_LOG( TRUE,  "DEBUG: AndroidRoll = %d\n", g_SensorStatus.AndroidRoll )
					}
					else
					{
						g_SensorStatus.AndroidCompass = 0;
						g_SensorStatus.AndroidPitch = 0;
						g_SensorStatus.AndroidRoll = 0;
					}
				}

				//////////////////////////////////////////////////////////////////////////
				// ODOMETER
				// g_SensorStatus.Odometer is the distance since the last Odometer RESET
				// g_SensorStatus.OdometerUpdate is the distance since the last Odometer READING
				// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
				// RAW Arduino Status is in TICKS, Laptop Arduino Status is in TenthInches

				#if MOTOR_CONTROL_TYPE != ER1_MOTOR_CONTROL	// *NOT* ER1!
				// Using Arduino for Odometry.  For ER1, see WM_ROBOT_ER1_ODOMETER_READY below

					#if NUMBER_OF_WHEEL_ODOMETERS == 2
					if( g_RawArduinoStatus.nOdometerSamples > 0 ) // avoid divide by zero
					{
						static char* strFwd = "FWD";
						static char* strRev = "REV";
						char* LeftDir = strFwd;
						char* RightDir = strFwd;

							// Left wheel
							// "Unpack" Arduino Odometer readings (in Ticks)
							unsigned short TempOdometerTicksL = g_RawArduinoStatus.OdometerHighL <<8;	// Shift the high byte ("hundreds and tens") over
							TempOdometerTicksL += g_RawArduinoStatus.OdometerLowL;						// Add in the low byte
							//if( TempOdometerTicksL & 0x8000 )
							if( g_MotorKludgeRevL ) // KLUDGE!!!
							{
								// "Reverse" bit set.  
								TempOdometerTicksL &= 0x7FFF; // clear the reverse bit
								g_SensorStatus.TachometerTicksL = (int)TempOdometerTicksL * -1; 
								LeftDir = strRev;
								//TRACE("TACH Left = REV ");
							}
							else
							{
								// "Reverse" bit not set
								TempOdometerTicksL &= 0x7FFF; // clear the reverse bit - KLUDGE
								g_SensorStatus.TachometerTicksL = TempOdometerTicksL; 
								//TRACE("TACH Left= FWD ");
							}

							// Right wheel
							// "Unpack" Arduino Odometer readings (in Ticks)
							unsigned short TempOdometerTicksR = g_RawArduinoStatus.OdometerHighR <<8;	// Shift the high byte ("hundreds and tens") over
							TempOdometerTicksR += g_RawArduinoStatus.OdometerLowR;						// Add in the low byte
							//if( TempOdometerTicksR & 0x8000 )
							if( g_MotorKludgeRevR ) // KLUDGE!!!
							{
								// "Reverse" bit set.  
								TempOdometerTicksR &= 0x7FFF; // clear the reverse bit
								g_SensorStatus.TachometerTicksR = (int)TempOdometerTicksR * -1;
								RightDir = strRev;
								//TRACE("TACH Right = REV ");
							}
							else
							{
								// "Reverse" bit not set
								TempOdometerTicksR &= 0x7FFF; // clear the reverse bit - KLUDGE
								g_SensorStatus.TachometerTicksR = TempOdometerTicksR;
								// TRACE("TACH Right = FWD ");
							}

							//TRACE("\n");
						
							//////////////////////////////////////////////
							// DEBUG
							//*
							if( (0 != g_SensorStatus.TachometerTicksL) || (0 != g_SensorStatus.TachometerTicksR) )
							{
								ROBOT_LOG( TRUE, "TACH_L: %s %4d   TACH_R: %s %4d, Samples: %d", 
									LeftDir, g_SensorStatus.TachometerTicksL, RightDir, g_SensorStatus.TachometerTicksR, g_RawArduinoStatus.nOdometerSamples )
							} 
							//*/


							// We get a weird rounding error with zero values so kludge it here
							double OdometerUpdateTenthInchesL = 0;
							if( 0 != g_SensorStatus.TachometerTicksL ) 
								OdometerUpdateTenthInchesL = g_SensorStatus.TachometerTicksL / TICKS_PER_TENTH_INCH;

							double OdometerUpdateTenthInchesR = 0;
							if( 0 != g_SensorStatus.TachometerTicksR ) 
								OdometerUpdateTenthInchesR = g_SensorStatus.TachometerTicksR / TICKS_PER_TENTH_INCH;

							UpdateFromTwoOdometers( OdometerUpdateTenthInchesL, OdometerUpdateTenthInchesR);

							// DEBUG STUFF
							if( (0 != g_SensorStatus.TachometerTicksL) || (0 != g_SensorStatus.TachometerTicksL) )
							{
						//		ROBOT_LOG( TRUE,    "**** ODOM DEBUG: g_SensorStatus.TachometerTicksL = %d, g_SensorStatus.TachometerTicksR = %d\n", g_SensorStatus.TachometerTicksL, g_SensorStatus.TachometerTicksR )
								//ROBOT_LOG( TRUE,  "**** ODOM DEBUG: OdometerHigh, Low L = %02X, %02X\n", g_RawArduinoStatus.OdometerHighL, g_RawArduinoStatus.OdometerLowL )
								//ROBOT_LOG( TRUE,  "**** ODOM DEBUG: OdometerHigh, Low R = %02X, %02X\n", g_RawArduinoStatus.OdometerHighR, g_RawArduinoStatus.OdometerLowR )
							}
							/***
							// Calibration for TICKS_PER_TENTH_INCH
												static int TempOdomTicksL = 0;
												static int TempOdomTicksR = 0;
												if( 0 != g_SensorStatus.TachometerTicksL ) TempOdomTicksL +=g_SensorStatus.TachometerTicksL;
												if( 0 != g_SensorStatus.TachometerTicksR ) TempOdomTicksR +=g_SensorStatus.TachometerTicksR;

								ROBOT_LOG( TRUE,  "**** ODOM DEBUG: TempOdomTicksL, R = %d, %d\n", TempOdomTicksL, TempOdomTicksR )
							***/
							// Calibration for Tachometer:
							//	ROBOT_LOG( TRUE,  "**** ODOM DEBUG: SPEED REQUEST = %d  RAW TACH: R = %d, L = %d \n", g_MotorCurrentSpeedCmd, g_SensorStatus.TachometerTicksL, g_SensorStatus.TachometerTicksR );
					}
					#else	// Single Odometer for robot.

						// Save prior Odometer reading
						int LastOdometer = g_SensorStatus.Odometer;	// in Inches

						// "Unpack" Arduino Odometer reading (in Ticks)
						signed short TempOdometerTicks = g_RawArduinoStatus.OdometerHigh <<8;		// Shift the high byte ("hundreds and tens") over
						TempOdometerTicks += g_RawArduinoStatus.OdometerLow;					// Add in the low byte

						// Update the system Odometer
						g_SensorStatus.OdometerTenthInches = TempOdometerTicks * TICKS_PER_TENTH_INCH;
						//ROBOT_LOG( TRUE,  "DEBUG ODOM = %d\n", g_SensorStatus.Odometer)

						// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
						g_SensorStatus.OdometerUpdate = g_SensorStatus.Odometer - LastOdometer;
						// Update MoveDistance counter, in case a programmed move is in progress
						m_pDriveCtrl->UpdateMoveDistance( g_SensorStatus.OdometerUpdate );
						// (used on ER1: m_pDriveCtrl->UpdateTurnRotation( ?? );

					#endif

				#endif

				g_SensorStatus.LeftHandRawPressureL = g_RawArduinoStatus.LeftHandPressureL;
				g_SensorStatus.LeftHandRawPressureR = g_RawArduinoStatus.LeftHandPressureR;
				//ROBOT_LOG( TRUE,  " DEBUG RAW HAND PRESSURE L =  %3d, R = %3d", g_SensorStatus.LeftHandRawPressureL, g_SensorStatus.LeftHandRawPressureR )


				//////////////////////////////////////////////////////////////////////////
				// IR SENSORS

				//-------------------------------------------------------------------------
				// Analog IR Sensors connected to the Arduino
				// IR Sensor 0,5 = Side, Wide Angle, short range
				// IR Sensor 1,4 = Forward, Long Range
				// IR Sensor 2,3 = Forward, Head mounted Long Range

				g_SensorStatus.IR[0] = ScaleWideIR(g_RawArduinoStatus.IR[0]); // Left Side, Wide Angle, short range
				g_SensorStatus.IR[1] = ScaleLongRangeIR(g_RawArduinoStatus.IR[1], 0); // Left Forward, Long Range
				g_SensorStatus.IR[2] = ScaleLongRangeIR(g_RawArduinoStatus.IR[2], 0); // // Right Long Range IR in Head, compensate for sensor differences
				g_SensorStatus.IR[3] = ScaleLongRangeIR(g_RawArduinoStatus.IR[3], 0); // Left Long Range IR in Head, compensate for sensor differences
				g_SensorStatus.IR[4] = ScaleLongRangeIR(g_RawArduinoStatus.IR[4], 0); // Right Forward, Long Range
				g_SensorStatus.IR[5] = ScaleWideIR(g_RawArduinoStatus.IR[5]); // Right Side, Wide Angle, short range

				// TODO-MUST: BAD SENSORS NEED TO FIX.  JUST DISABLE FOR NOW
				//g_SensorStatus.IR[1] = NO_OBJECT_IN_RANGE;
				//g_SensorStatus.IR[4] = NO_OBJECT_IN_RANGE;

				//	if(g_SensorStatus.IR[0] < 5) ROBOT_LOG( TRUE,  "******** IR0 ********\n" )
				//	if(g_SensorStatus.IR[5] < 5) ROBOT_LOG( TRUE,  "******** IR5 ********\n" )

				// Now, show the values in Inches

				#if DEBUG_IR == 1
					TRACE( "IR, RAW, INCHES: ");
					for( nSensorNumber=0; nSensorNumber< NUM_IR_SENSORS; nSensorNumber++ )
					{
						TRACE("  %2u=%3u,%3u",
							nSensorNumber, g_RawArduinoStatus.IR[nSensorNumber], g_SensorStatus.IR[nSensorNumber]/10 );
					}
					TRACE( "\n" );
				#endif


					//-------------------------------------------------------------------------
					// I2C A/D Sensors (From I2C A/D Expansion chip on the Loki expansion board)

				#ifdef VERTICAL_IR_BUMPERS_ENABLED
					// VERTICAL IR "Bumper" Sensors
					// We use 3 A/D sensors pointing up at front of the Robot to detect
					// tables, etc. that lower sensors might miss.
					// Even though they are analog, we use them as digital "Someting is there" sensors.
					g_SensorStatus.IR2[0] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[0], 0);	// Left
					g_SensorStatus.IR2[1] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[1], 0);	// Right
				#endif

					// Head sensors (analog signal sent to expansion board in robot base) Long Range IR sensors
					// Note that Head sensors are set back 8 inches from front panel, so compensation needed at time when used
					//ROBOT_LOG( TRUE,  " DEBUG RAW HEAD IR 2 = %d, 3 = %d\n", g_RawArduinoStatus.IR2[2], g_RawArduinoStatus.IR2[3] )
				/* DISABLED
					// Right Long Range IR in Head
					g_SensorStatus.IR2[2] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[2], 0); // compensate for sensor differences
					//if( TempHeadRange < NO_OBJECT_IN_RANGE ) TempHeadRange += 1;// compensate for sensor differences
					//g_SensorStatus.IR2[2] = TempHeadRange;

					// Left Long Range IR in Head
					g_SensorStatus.IR2[3] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[3], -10); // compensate for sensor differences
					//if( TempHeadRange < NO_OBJECT_IN_RANGE ) TempHeadRange -= 1;// compensate for sensor differences
					//g_SensorStatus.IR2[3] = TempHeadRange;
				*/

				// Other I2C A/D IR sensors: 
				/* (NOT CONNECTED)
				g_SensorStatus.IR2[5] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[5], 0);
				g_SensorStatus.IR2[6] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[6], 0);
				g_SensorStatus.IR2[7] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[7], 0);
				*/
				//	ROBOT_LOG( TRUE,  "   HEAD IR = %03d = %04X\n",   g_SensorStatus.IR2[2], g_SensorStatus.IR2[2] )
				//	ROBOT_LOG( TRUE,  "   HEAD US = %03d = %04X\n\n", g_SensorStatus.IR2[3], g_SensorStatus.IR2[3] )

				//-------------------------------------------------------------------------
				// I2C-IT IR Sensors.  Values reported in Inches, so multiply by 10 for TenthInches.
				// (Centemeters is an options too.  See Arduino code) 
				// TODO - Read Centemeters for greater accuracy, then convert directly to tenthinches!
				for( nSensorNumber=0; nSensorNumber<NUM_IR3_SENSORS; nSensorNumber++ )
				{
					g_SensorStatus.IR3[nSensorNumber] = g_RawArduinoStatus.IR3[nSensorNumber] * 10;
				}

				/*
				//	#if DEBUG_IR == 1
					ROBOT_LOG( TRUE,  "   I2C-IT = ");
					nSensorNumber = 0;
					for( nSensorNumber=0; nSensorNumber < 4; nSensorNumber++ )	// NUM_IR3_SENSORS
					{
						ROBOT_LOG( TRUE,  "  %01u:%03u",nSensorNumber, g_RawArduinoStatus.IR3[nSensorNumber] );
					}
					ROBOT_LOG( TRUE,  " Inches\n" ); //TenthInches???
				//	#endif
				/*/

				//ROBOT_LOG( TRUE,  "I2C-IT  %01u:%03u\n",0, g_RawArduinoStatus.IR3[0] );

					
				//////////////////////////////////////////////////////////////////////////
				// Check Status Flags
				// See if Move completed
	//			if( 0 != (g_SensorStatus.StatusFlags & HW_STATUS_MOVE_DISTANCE_COMPLETE) )
	//			{
	//				// Move completed, Arduino stopped motors and is awaiting next command
	//				m_pDriveCtrl->SetMoveDistanceCompleted( SENSOR_MODULE ); // Tell Motor control move is done
	//			}

				if( (g_SensorStatus.StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
				{
					// Power is enabled
					if( !m_MotorPowerEnabled )
					{
						// State changed
						m_MotorPowerEnabled = TRUE;
						ROBOT_LOG( TRUE,  "RC_BUTTON_POWER is ON\n" );
					}
				}
				else
				{
					// Power is disabled
					if( m_MotorPowerEnabled )
					{
						// State changed
						m_MotorPowerEnabled = FALSE;
						ROBOT_LOG( TRUE,  "RC_BUTTON_POWER is OFF\n" )
					}
				}

				// For Debug:
				//ROBOT_LOG( MOTOR_DBG, "Status Flags = %02X\n", g_SensorStatus.StatusFlags )

				// Add line break in log if debugging IR or Ultrasonic sensors
				#if ( (DEBUG_IR == 1) || (DEBUG_ULTRASONIC == 1) )
						ROBOT_LOG( TRUE,  "\n" );
				#endif


				///////////////////////////////////////////////////////////////////////////
				// Done processing sensor data.
				// Now do sensor fusion to combine the data in a meaningful way
				DoSensorFusion();

				// Now post the status to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
					0 );									// Param2 = not used


			
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE

				//*********************************************************************
				// IMPORTANT!  This is where g_RawArduinoStatus gets copied to g_SensorStatus
				// Process Status block from the Arduino.  Contains IR data and other stuff
				// First thing is to initialize the processed Sensor Status block

				// ERROR: corrupts current position! memset( &g_SensorStatus, SENSOR_DISABLED, sizeof(SENSOR_STATUS_T) );	// Int all sensors to disabled


				g_SensorStatus.StatusFlags =	0;
				g_SensorStatus.LastError =		0;
				g_SensorStatus.DebugCode =		0;

				g_SensorStatus.HWBumper = 0;

				if( g_piRobotStatus->BumperLeft )
				{
					g_SensorStatus.HWBumper |= HW_BUMPER_FRONT_MASK;
					g_SensorStatus.HWBumper |= HW_BUMPER_SIDE_LEFT_MASK;
				}
				if( g_piRobotStatus->BumperRight )
				{
					g_SensorStatus.HWBumper |= HW_BUMPER_FRONT_MASK;
					g_SensorStatus.HWBumper |= HW_BUMPER_SIDE_RIGHT_MASK;
				}
				if( 0 != g_SensorStatus.HWBumper )
				{
					CString MsgString;
					MsgString.Format("HW BUMPER HIT: %04lX", g_SensorStatus.HWBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}

				g_SensorStatus.IRBumper = 0;
				if( (g_piRobotStatus->CliffFrontLeft) || (g_piRobotStatus->CliffLeft) )
				{
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_LEFT_MASK;
				}
				if( (g_piRobotStatus->CliffFrontRight) || (g_piRobotStatus->CliffRight) )
				{
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_RIGHT_MASK;
				}
				if( 0 != g_SensorStatus.IRBumper )
				{
					CString MsgString;
					MsgString.Format("CLIFF Detected: %04lX", g_SensorStatus.IRBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}

				// Note that this is the current charge in mA Hours, not Voltage!
				g_RawArduinoStatus.Battery0 = g_piRobotStatus->BatteryCharge;


				//////////////////////////////////////////////////////////////////////////
				// ACCELEROMETER

				// First, normalize Accelerometer readings, so level = 0,0
				#ifdef ACCELEROMETER_INSTALLED
					// Accelerometer only installed in Carbot?
					int TempAccX = ACCEL_OFFSET_X - g_RawArduinoStatus.AccelX;
					int TempAccY = g_RawArduinoStatus.AccelY - ACCEL_OFFSET_Y;

					// TODO - Limit Accelerations to 30 degree slopes? (to ignore really stong start/stops?)
					// OR find median value (not average) - discard top and bottom numbers...
					// Now, average Acceleration values to remove instantaneous accelerations 
					// from motors and brakes

					// Overwrite oldest reading (always keep "N" most recent readings)
					m_AccelHistoryX[m_AccelHistorySample] = TempAccX;
					m_AccelHistoryY[m_AccelHistorySample] = TempAccY;
					m_AccelHistorySample++;

					if( m_AccelHistorySample >= ACC_MAX_SAMPLES )
					{
						m_AccelHistorySample = 0;	// wrap back to first element
					}

					// Get Average
					int SumX = 0;
					int SumY = 0;
					for( int i=0; i < ACC_MAX_SAMPLES; i++ )
					{
						SumX += m_AccelHistoryX[i];
						SumY += m_AccelHistoryY[i];
					}

					g_SensorStatus.AccelX = SumX / ACC_MAX_SAMPLES;
					g_SensorStatus.AccelY = SumY / ACC_MAX_SAMPLES;

					//	ROBOT_LOG( TRUE,  "DEBUG Accel Inst: X=%03d Y=%03d  Ave: X=%03d Y=%03d\n", 
					//		TempAccX, TempAccY, g_SensorStatus.AccelX, g_SensorStatus.AccelY )
					//	ROBOT_LOG( TRUE,  "DEBUG Accel: X=%03d  Y=%03d\n", g_RawArduinoStatus.AccelX, g_RawArduinoStatus.AccelY)

					int  SlopeDir;
					int  SlopeAmt;
					ConvertAccToTilt( g_SensorStatus.AccelX , g_SensorStatus.AccelY, SlopeDir, SlopeAmt);
				#endif

				//////////////////////////////////////////////////////////////////////////
				// COMPASS
				// TempCompass is 0-3599 (359.9 degrees)
				#ifdef COMPASS_INSTALLED
					int  Compass = 0;
					int  TempCompass = g_RawArduinoStatus.CompassHigh <<8;
					TempCompass += g_RawArduinoStatus.CompassLow;

					//	ROBOT_LOG( TRUE,  "DEBUG Compass %04X HEX\n", TempCompass)
					TempCompass = TempCompass / 10;
					//	ROBOT_LOG( TRUE,  "Compass = %03u  \n", TempCompass)

					if( TempCompass > 360 )
					{
						ROBOT_LOG( TRUE,  "ERROR!! - Sensor Module: Compass exceeds 360 degrees!\n" )
						//	TempCompass = 0;
					}
					Compass = TempCompass;

					if( 0 != m_CompassCorrection )
					{
						ROBOT_LOG( TRUE,  "CompassCorreciton = %d\n", m_CompassCorrection)
					}

					Compass += m_CompassCorrection;	// Add in any correction info available
					if( Compass > 360 ) Compass -= 360;
					g_SensorStatus.CompassHeading = Compass;
				#endif


				//////////////////////////////////////////////////////////////////////////
				// ODOMETER
				// g_SensorStatus.Odometer is the distance since the last Odometer RESET
				// g_SensorStatus.OdometerUpdate is the distance since the last Odometer READING
				// ?Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
				// ?RAW Arduino Status is in TICKS, Laptop Arduino Status is in TenthInches

		
				// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
				g_SensorStatus.OdometerUpdateTenthInches = (double)g_piRobotStatus->Distance / 2.540;	 // convert MM to TenthInches

				// Update the system Odometer with average of the two wheels
				g_SensorStatus.OdometerTenthInches += g_SensorStatus.OdometerUpdateTenthInches;

		
				// Update the "Tach" display.
				g_SensorStatus.Tachometer = (int)(g_SensorStatus.OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

				//		ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
				//			OdometerdoubleInchesL, OdometerdoubleInchesR, g_SensorStatus.Odometer, g_SensorStatus.OdometerUpdate )
		
				// Update MoveDistance counter, in case a programmed move is in progress
				m_pDriveCtrl->UpdateMoveDistance( g_SensorStatus.OdometerUpdateTenthInches );

				double TurnAngleUpdate = g_piRobotStatus->Angle;

				// Update TurnRotation counter, in case a programmed turn (rotate in place) is in progress
				m_pDriveCtrl->UpdateTurnRotation( TurnAngleUpdate );

				if( -1 == g_SensorStatus.CalculatedMotorHeading )
				{
					// Heading not initialized
					// Wait for first motor movement, then grab the compass heading to initialize direction
					if( 0 != g_SensorStatus.OdometerUpdateTenthInches )
					{
						g_SensorStatus.CalculatedMotorHeading = m_LastCompassHeading;
					}
				}
				else
				{
					g_SensorStatus.CalculatedMotorHeading += TurnAngleUpdate;
					// TODO - see if "drifting" occurs due to round off error.  If so, keep angle to higher precision?
					//	ROBOT_LOG( TRUE,  "========> Turn %0.3f --> Calculated MOTOR HEADING: %0.2f\n", TurnAngleUpdate, g_SensorStatus.CalculatedMotorHeading)
				}

				UpdateLocation();	// Update the internal view of where we are in the world!

					
				//////////////////////////////////////////////////////////////////////////
				// Check Status Flags
				// See if Move completed
				//	if( 0 != (g_SensorStatus.StatusFlags & HW_STATUS_MOVE_DISTANCE_COMPLETE) )
				//	{
				//	// Move completed, Arduino stopped motors and is awaiting next command
				//	m_pDriveCtrl->SetMoveDistanceCompleted( SENSOR_MODULE ); // Tell Motor control move is done
				//	}


				///////////////////////////////////////////////////////////////////////////
				// Done processing sensor data.
				// Now do sensor fusion to combine the data in a meaningful way
				DoSensorFusion();

				// Now post the status to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
					0 );									// Param2 = not used


				/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP

				//*********************************************************************
				// IMPORTANT!  This is where Raw Status gets copied to g_SensorStatus
				// Process Status block from the Kobuki Base.  Motor data and other stuff
				// First thing is to initialize the processed Sensor Status block

				// ERROR: corrupts current position! memset( &g_SensorStatus, SENSOR_DISABLED, sizeof(SENSOR_STATUS_T) );	// Int all sensors to disabled


				g_SensorStatus.StatusFlags =	0;
				g_SensorStatus.LastError =		0;
				g_SensorStatus.DebugCode =		0;
				g_SensorStatus.HWBumper = 0;
				g_SensorStatus.IRBumper = 0;

				//////////////////////////////////////////////////////////////////////////
				// Kobuki Bumpers
				if( g_pKobukiStatus->Bumper & 0x01 ) // Right Bumper
				{					
					g_SensorStatus.HWBumper |= HW_BUMPER_SIDE_RIGHT_MASK;
				}
				if( g_pKobukiStatus->Bumper & 0x02 ) // Center Bumper
				{
					g_SensorStatus.HWBumper |= HW_BUMPER_FRONT_MASK;
				}
				if( g_pKobukiStatus->Bumper & 0x04 ) // Left Bumper
				{
					g_SensorStatus.HWBumper |= HW_BUMPER_SIDE_LEFT_MASK;
				}
				if( (0 != g_SensorStatus.HWBumper) && (0 == g_pKobukiStatus->BatteryChargeSourceEnum) ) // only display bumper messages if not docked
				{
					CString MsgString;
					MsgString.Format("HW BUMPER HIT: %02X, %04lX", g_pKobukiStatus->Bumper, g_SensorStatus.HWBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}

				//////////////////////////////////////////////////////////////////////////
				// Kobuki Cliff Sensors
				g_SensorStatus.Cliff = g_pKobukiStatus->Cliff;

				// TODO - FIX THIS!
				if( g_pKobukiStatus->Cliff & 0x01 ) // Right Cliff
				{					
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_RIGHT_MASK;
				}
				if( g_pKobukiStatus->Cliff & 0x02 ) // Center Cliff
				{
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_RIGHT_MASK; // Set both
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_LEFT_MASK;
				}
				if( g_pKobukiStatus->Cliff & 0x04 ) // Left Cliff
				{
					g_SensorStatus.IRBumper |= IR_BUMPER_CLIFF_LEFT_MASK;
				}
				if( 0 != g_SensorStatus.IRBumper )
				{
					CString MsgString;
					MsgString.Format("CLIFF Detected: %02X, %04lX", g_pKobukiStatus->Cliff, g_SensorStatus.IRBumper);
					ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				}

				//////////////////////////////////////////////////////////////////////////
				// Kobuki Wheel Drop
				g_SensorStatus.WheelDrop = g_pKobukiStatus->WheelDrop;


				//////////////////////////////////////////////////////////////////////////
				// Charging Dock IR Beacon
				g_SensorStatus.DockSensorRight = g_pKobukiStatus->DockRightSignal;
				g_SensorStatus.DockSensorCenter = g_pKobukiStatus->DockCenterSignal;
				g_SensorStatus.DockSensorLeft = g_pKobukiStatus->DockLeftSignal;

				//////////////////////////////////////////////////////////////////////////
				// BATTERY
				// Note that this is the current charge in % remaining, not Voltage!
				g_RawArduinoStatus.Battery0 = (BYTE)g_pKobukiStatus->BatteryPercent;
				g_SensorStatus.Battery0 = (int)g_pKobukiStatus->BatteryPercent;




				//////////////////////////////////////////////////////////////////////////
				// ACCELEROMETER

				// First, normalize Accelerometer readings, so level = 0,0
				#ifdef ACCELEROMETER_INSTALLED
					// Accelerometer only installed in Carbot?
					int TempAccX = ACCEL_OFFSET_X - g_RawArduinoStatus.AccelX;
					int TempAccY = g_RawArduinoStatus.AccelY - ACCEL_OFFSET_Y;

					// TODO - Limit Accelerations to 30 degree slopes? (to ignore really stong start/stops?)
					// OR find median value (not average) - discard top and bottom numbers...
					// Now, average Acceleration values to remove instantaneous accelerations 
					// from motors and brakes

					// Overwrite oldest reading (always keep "N" most recent readings)
					m_AccelHistoryX[m_AccelHistorySample] = TempAccX;
					m_AccelHistoryY[m_AccelHistorySample] = TempAccY;
					m_AccelHistorySample++;

					if( m_AccelHistorySample >= ACC_MAX_SAMPLES )
					{
						m_AccelHistorySample = 0;	// wrap back to first element
					}

					// Get Average
					int SumX = 0;
					int SumY = 0;
					for( int i=0; i < ACC_MAX_SAMPLES; i++ )
					{
						SumX += m_AccelHistoryX[i];
						SumY += m_AccelHistoryY[i];
					}

					g_SensorStatus.AccelX = SumX / ACC_MAX_SAMPLES;
					g_SensorStatus.AccelY = SumY / ACC_MAX_SAMPLES;

					//	ROBOT_LOG( TRUE,  "DEBUG Accel Inst: X=%03d Y=%03d  Ave: X=%03d Y=%03d\n", 
					//		TempAccX, TempAccY, g_SensorStatus.AccelX, g_SensorStatus.AccelY )
					//	ROBOT_LOG( TRUE,  "DEBUG Accel: X=%03d  Y=%03d\n", g_RawArduinoStatus.AccelX, g_RawArduinoStatus.AccelY)

					int  SlopeDir;
					int  SlopeAmt;
					ConvertAccToTilt( g_SensorStatus.AccelX , g_SensorStatus.AccelY, SlopeDir, SlopeAmt);
				#endif

				//////////////////////////////////////////////////////////////////////////
				// COMPASS


				// Compass Heading is in degrees
				// WARNING - THIS IS NOT REALLY COMPASS HEADING!
				// IT IS DEGREES FROM WHERE KOBUKI WAS POINTING WHEN POWERED ON!
				// (but still useful for determining turn amount)
				g_SensorStatus.CompassHeading = (int)(g_pKobukiStatus->GyroDegrees);


				//////////////////////////////////////////////////////////////////////////
				// ODOMETER
				// g_SensorStatus.Odometer is the distance since the last Base was powered on
				// g_SensorStatus.OdometerUpdate is the distance since the last Odometer READING
				// ?Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
				// ?RAW base Status is in TICKS, Laptop Arduino Status is in TenthInches

				// Get the average of the two wheels
				int OdometerTenthInches = (g_pKobukiStatus->OdometerLeft + g_pKobukiStatus->OdometerRight) / 2;
		
				// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
				// it's the current odometer reading minus the last odometer reading
				g_SensorStatus.OdometerUpdateTenthInches = OdometerTenthInches - g_SensorStatus.OdometerTenthInches;
					
				// Update the system Odometer
				g_SensorStatus.OdometerTenthInches = OdometerTenthInches;

		
				// Update the "Tach" display.
				g_SensorStatus.Tachometer = (int)(g_SensorStatus.OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

				//		ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
				//			OdometerdoubleInchesL, OdometerdoubleInchesR, g_SensorStatus.Odometer, g_SensorStatus.OdometerUpdate )
		
				// Update MoveDistance counter, in case a programmed move is in progress
				m_pDriveCtrl->UpdateMoveDistance( g_SensorStatus.OdometerUpdateTenthInches );


				// TODO-MUST test this, not sure it is right!
				double TurnAngleUpdate = g_pKobukiStatus->TurnRate;

				// Update TurnRotation counter, in case a programmed turn (rotate in place) is in progress
				m_pDriveCtrl->UpdateTurnRotation( TurnAngleUpdate );

				if( -1 == g_SensorStatus.CalculatedMotorHeading )
				{
					// Heading not initialized
					// Wait for first motor movement, then grab the compass heading to initialize direction
					if( 0 != g_SensorStatus.OdometerUpdateTenthInches )
					{
						g_SensorStatus.CalculatedMotorHeading = m_LastCompassHeading;
					}
				}
				else
				{
					g_SensorStatus.CalculatedMotorHeading += TurnAngleUpdate;
					// TODO - see if "drifting" occurs due to round off error.  If so, keep angle to higher precision?
					//	ROBOT_LOG( TRUE,  "========> Turn %0.3f --> Calculated MOTOR HEADING: %0.2f\n", TurnAngleUpdate, g_SensorStatus.CalculatedMotorHeading)
				}

				UpdateLocation();	// Update the internal view of where we are in the world!

					
				//////////////////////////////////////////////////////////////////////////
				// Check Status Flags
				// See if Move completed
				//	if( 0 != (g_SensorStatus.StatusFlags & HW_STATUS_MOVE_DISTANCE_COMPLETE) )
				//	{
				//	// Move completed, Arduino stopped motors and is awaiting next command
				//	m_pDriveCtrl->SetMoveDistanceCompleted( SENSOR_MODULE ); // Tell Motor control move is done
				//	}


				///////////////////////////////////////////////////////////////////////////
				// Done processing sensor data.
				// Now do sensor fusion to combine the data in a meaningful way
				DoSensorFusion();

				// Now post the status to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
					0 );									// Param2 = not used


				/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#else
				#error BAD SENSOR_CONFIG_TYPE!  
			#endif // robot sel
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		}	// end case WM_ROBOT_SENSOR_STATUS_READY
		break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		case WM_ROBOT_ER1_ODOMETER_READY:
		{
			g_bCmdRecognized = TRUE;
			// When using the ER1 Pilot controller, move distance and speed are reported
			// by the motor controller, not the Arduino.
			// wParam = LEFT_MOTOR, lParam = RIGHT_MOTOR

			//////////////////////////////////////////////////////////////////////////
			// ODOMETER
			// g_SensorStatus.Odometer is the distance since the last Odometer RESET
			// g_SensorStatus.OdometerUpdate is the distance moved since the last update
			// Note, The odometer value could be positive or negative, depending if we are going FWD or REV. from the last RESET
			// RAW ER1 Status is in TICKS, Laptop PicStatus is in TenthInches

			#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL

				// ER1 is used for Odometry instead of Arduino
				
				// Unlike the Arduino, the ER1 keeps a running distance total
				long LastOdometer = g_SensorStatus.Odometer;
				long OdometerReadingL = wParam;
				long OdometerReadingR = lParam;

				long OdometerUpdateL = OdometerReadingL - m_LastOdometerReadingL;	// high precision update
				long OdometerUpdateR = OdometerReadingR - m_LastOdometerReadingR;

				m_LastOdometerReadingL = OdometerReadingL;	// save for next time
				m_LastOdometerReadingR = OdometerReadingR;

				double OdometerdoubleTenthInchesL = (double)(OdometerReadingL)/TICKS_PER_TENTH_INCH;
				double OdometerdoubleTenthInchesR = (double)(OdometerReadingR)/TICKS_PER_TENTH_INCH;

				double OdometerUpdateTenthInchesL = (double)(OdometerUpdateL)/TICKS_PER_TENTH_INCH;
				double OdometerUpdateTenthInchesR = (double)(OdometerUpdateR)/TICKS_PER_TENTH_INCH;

				// Handle Two Odometers (Right and Left wheels)
				UpdateFromTwoOdometers( OdometerUpdateTenthInchesL, OdometerUpdateTenthInchesR);

				// Now post the status to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
					0 );									// Param2 = not used

			#else
				ROBOT_LOG( TRUE,  "SensorModule: Unexpected WM_ROBOT_ER1_ODOMETER_READY but MOTOR_CONTROL_TYPE not ER1_MOTOR_CONTROL!\n" )
				ROBOT_ASSERT(0);
			#endif	// MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL
		}
		break;

		case WM_ROBOT_GPS_DATA_READY:
		{
			g_bCmdRecognized = TRUE;

			// Process data block from the GPS device.

			POINT Pos = GPSDegreesToRWTenthInches( g_pGPSData->dGGALatitude, g_pGPSData->dGGALongitude );
			g_SensorStatus.CurrentLocationGPS.x = Pos.x;
			g_SensorStatus.CurrentLocationGPS.x = Pos.y;

			//-------------------------------------------------------------
			// TODO-GPS - Update RW Map with GPS if GPS is high confidence

			// Get current speed of robot from wheel sensors
			// Determine slope of last several readings - are they all in a line?
			// If all in a line, calculate approx speed, based upon spacing
			// is the speed about right?
			// if so, confidence is high
			//m=	y2 - y1 / x2 - x1	
			//if x2-x1 == 0, heading due East or West	
			//if y2-y1 == 0, heading due North or South	

			// If GPS confindence is high, and current pos is different by greater
			// then normal GPS error amt, then adjust current pos to pos indicated
			// by GPS.  thus, next turn calculation will be based upon new pos data
			// question: should we also use current direction as indicated by GPS
			// to correct compass heading?  what if we just turned?


			if( m_GPSNavigationEnabled )
			{

				// Get distance between current and last stored sample
				double DeltaX = (double)m_GPSLastSample.x - g_SensorStatus.CurrentLocationGPS.x;
				double DeltaY = (double)m_GPSLastSample.y - g_SensorStatus.CurrentLocationGPS.y;
				int  DistanceFromLastGPS = (int )(sqrt( (double)(DeltaX*DeltaX) + (double)(DeltaY*DeltaY) ));

				if( (DistanceFromLastGPS > 36) &&	// min number of inches between samples //Convert to TenthInches???
					(g_SensorStatus.OdometerUpdateTenthInches > 5) )	// Make sure robot is actually moving!
				{
					// New GPS reading.  Let's process it!
					// Get direction of travel between current and last stored sample
					int  CurrentAngle = CalculateAngle( m_GPSLastSample, FPointToPoint(g_SensorStatus.CurrentLocationGPS) );
					m_GPSLastSample = FPointToPoint(g_SensorStatus.CurrentLocationGPS);	// Save current position for next time

					// Keep an array of the last "N" direction samples. Overwrite oldest reading
					m_GPSHistory[m_GPSHistorySample] = CurrentAngle;
					m_GPSHistorySample++;
					if( m_GPSHistorySample >= GPS_HISTORY_MAX_SAMPLES )
					{
						m_GPSHistorySample = 0;	// wrap back to first element
					}

					// Get direction of travel between each sample.
					// Abort if the directions are too far off 
					BOOL TrailStraight = TRUE;
					for( int i=0; i < GPS_HISTORY_MAX_SAMPLES; i++ )
					{
						if( abs((int)(CurrentAngle - m_GPSHistory[i])) > GPS_TRAIL_DEGREE_TOLERANCE )
						{
							// One of the readings is too far off.  Don't trust that we have a good fix.
							TrailStraight = FALSE;
							break;
						}
					}

					// Check Quality of the fix
		/**	BYTE		btGGAGPSQuality;				// 0 = fix not available, 1 = GPS sps mode, 2 = Differential GPS, SPS mode, fix valid, 3 = GPS PPS mode, fix valid
			BYTE		btGSAMode;					// M = manual, A = automatic 2D/3D
			BYTE		btGSAFixMode;				// 1 = fix not available, 2 = 2D, 3 = 3D
			WORD		wGSVTotalNumSatsInView;		//
			//BYTE		btRMCDataValid;				// A = Data valid, V = navigation rx warning
		**/
					ROBOT_LOG( TRUE,  "DEBUG GPS - btGGAGPSQuality = %d, btGSAFixMode = %d, wGSVTotalNumSatsInView = %d \n",
						g_pGPSData->btGGAGPSQuality, g_pGPSData->btGSAFixMode, g_pGPSData->wGSVTotalNumSatsInView )

					if( TrailStraight && (3 == g_pGPSData->btGSAFixMode) )
					{
						// Looks like we have a good fix!
						// See if robot's current position is within the margin of error for the GPS location.
						// If not, adjust as needed
						g_SensorStatus.CurrentLocationGPS.x = 0;


						DeltaX = g_SensorStatus.CurrentLocation.x - g_SensorStatus.CurrentLocationGPS.x;
						DeltaY = g_SensorStatus.CurrentLocation.y - g_SensorStatus.CurrentLocationGPS.y;
						int  DistanceFromGPSCenter = (int )sqrt( (double)(DeltaX*DeltaX) + (double)(DeltaY*DeltaY) );

						if( DistanceFromGPSCenter > GPS_ERROR_AMOUNT )
						{
							// outside the GPS boundry.  Adjust RW coordinate to "move" robot to edge of GPS Error zone

		// /**** COMPLEX (BUT BETTER) METHOD:

							// Given GPS Center plus distance (GPS Error) and direction to current RW coordinates, 
							// calculate new current X,Y.
							double X, Y;	// Delta from Start Waypoint
							//int MapX, MapY;	// Absolute coordinates, allows for negative numbers

							int  AngleFromGPStoCurrent = CalculateAngle( g_SensorStatus.CurrentLocationGPS, g_SensorStatus.CurrentLocation );

							double DirectionRadians = AngleFromGPStoCurrent * DEGREES_TO_RADIANS;
							X = GPS_ERROR_AMOUNT * sin( DirectionRadians );	// Returns negative numbers as needed
							Y = GPS_ERROR_AMOUNT * cos( DirectionRadians );
							
							if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
							if(Y>=0) Y += 0.5; else Y -= 0.5;

		// *****/
		// SIMPLE METHOD (comment out section above for DEBUGGING):
		//					int X = 0;
		//					int Y = 0;

							// Update Robot's internal location to edge of the GPS Error Zone
							g_SensorStatus.CurrentLocation.x = g_SensorStatus.CurrentLocationGPS.x + X;		// Get new Map absolute X,Y
							g_SensorStatus.CurrentLocation.x = g_SensorStatus.CurrentLocationGPS.y + Y;

							ROBOT_LOG( TRUE,  "GPS - Updating current location with GPS reading!\n" )
							// Kludge - to indicate this "override" on the map, change the 
							// btGSAFixMode to act as a flag.
							g_pGPSData->btGSAFixMode = 4; // 1 = fix not available, 2 = 2D, 3 = 3D 4= Special Flag!
						
						}
					}	// If DistanceFromLastGPS > minimum amount
				}
			}	// if m_GPSNavigationEnabled

			// Finally, Post the GPS data to the GUI display (local or remote)
			SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
				ROBOT_RESPONSE_GPS_DATA,				// Param1 = Bulk data command
				0 );									// Param2 = not used

		}
		break;

		case WM_ROBOT_SENSOR_DATA:
		{
			g_bCmdRecognized = TRUE;
			// Process Bulk Sensor data received from Arduino
			// Find out which SCANNING Radar sensor has been updated
			// lParam is the Scanning Sensor number
			if( US_ARRAY1 == lParam )
			{
				// Convert to Tenth_Inches
				ScaleUltraSonicData( US1_SAMPLES, (BYTE*)g_BulkSensorData[lParam], g_ScaledSensorData[lParam] );

				// Post message to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_RADAR_SAMPLE,			// Param1 = Bulk data command
					US_ARRAY1 );					// Param2 = Array number
			}
			else if( IR_ARRAY1 == lParam )
			{
				// Convert to inches
				ScaleIRArrayData( IR1_SAMPLES, (BYTE*)g_BulkSensorData[lParam], g_ScaledSensorData[lParam] );

				// Post message to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
					ROBOT_RESPONSE_RADAR_SAMPLE,			// Param1 = Bulk data command
					IR_ARRAY1 );							// Param2 = Array number
			}
			else
			{
				ROBOT_LOG( TRUE,  "ERROR - CSensorModule: Bad value for WM_ROBOT_SENSOR_DATA lParam\n" )
			}
		}
		break;

	}
}

void CSensorModule::UpdateLocation()
{
	// Update global location in real world coordinates.
	// Uses "dead reckoning" via Odometer and Compass, and updates based upon relationship to obsticals, etc.
	// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
	// g_SensorStatus.OdometerUpdate holds distance, in (double)inches, since last reading.

	///////////////////////////////////////////////////////////////////////////////
	// First, estimate current location based upon "dead reckoning"
	// using compass and Odometer

	//POINT LastLocation = g_SensorStatus.CurrentLocation;
	if( g_SensorStatus.CompassHeading <= 360 )
	{
		// Good compass value.  Save the heading in case the next time we get a bad value
		m_LastCompassHeading = g_SensorStatus.CompassHeading;
	}
	// Note:  If the compass heading is bad, we ASSUME the heading has not changed significantly 
	// since the last good heading, so we calculate position based upon that.
	// TODO-ER1 - Use MotorHeading to fill in for compass? (see below)

	// Convert Degrees to Radians then do the math
	double DirectionRadians = (double)m_LastCompassHeading * DEGREES_TO_RADIANS;
	double X = g_SensorStatus.OdometerUpdateTenthInches * sin( DirectionRadians );	// Returns negative numbers as needed
	double Y = g_SensorStatus.OdometerUpdateTenthInches * cos( DirectionRadians );
	
	// Not needed - Now stored as high precision data
//	if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
//	if(Y>=0) Y += 0.5; else Y -= 0.5;
	
	g_SensorStatus.CurrentLocation.x += X;		// Get new Map absolute X,Y TENTHINCHES
	g_SensorStatus.CurrentLocation.y += Y;

	///////////////////////////////////////////////////////////////////////////////
	// Now, for ER1, estimate current location based upon "dead reckoning"
	// using odometer differences between motors (differential drive)

	//POINT LastLocation = g_SensorStatus.CurrentLocation;
//		m_LastMotorHeading = g_SensorStatus.CompassHeading;
/********* TODO
	if( -1 != g_SensorStatus.CalculatedMotorHeading )	// Make sure Heading is initialized
	{
		// Convert Degrees to Radians then do the math
		double DirectionRadians = g_SensorStatus.CalculatedMotorHeading * DEGREES_TO_RADIANS;
		double X = g_SensorStatus.OdometerUpdate * sin( DirectionRadians );	// Returns negative numbers as needed
		double Y = g_SensorStatus.OdometerUpdate * cos( DirectionRadians );
		
//		if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
//		if(Y>=0) Y += 0.5; else Y -= 0.5;
		g_SensorStatus.CurrentLocationMotor.x += X;		// Get new Map absolute X,Y
		g_SensorStatus.CurrentLocationMotor.y += Y;
	}

***********/

/*	ROBOT_LOG( TRUE,  "DEBUG POSITION: Compass=%d,%d,  Motor=%d,%d\n", 
		(int)g_SensorStatus.CurrentLocation.x, (int)g_SensorStatus.CurrentLocation.y,
		(int)g_SensorStatus.CurrentLocationMotor.x, (int)g_SensorStatus.CurrentLocationMotor.y )

*/

	///////////////////////////////////////////////////////////////////////////////
	// TODO-GPS!  Add GPS Location Update Here!!!
	///////////////////////////////////////////////////////////////////////////////




	///////////////////////////////////////////////////////////////////////////////
	// Now, see if known landmarks help us determine were we are more accurately

	// WORK IN PROGRESS - 
#ifdef THIS_STUFF_CURRENTLY_BROKEN
	// Update using GRIDMAP Objects
	// For each sensor, determine if an object detected.  See if these correlate
	// to objects on the gridmap, and if so, update location based upon this information
	// (this seems too black/white - should we use some sort of %confidence?)
	// for each sensor, if object found, correlate with an object on the grid map.

	// TODO - If multipe objects found, correlate x,y offsets to see if they agree or cancel each other
	// TODO - DO ALL THIS FOR US READINGS TOO!
	// TODO - Need to handle CLIFFS

	// Max distance from detected object to look for an object on the map
	#define MAX_IR_OFFSET_DISTANCE	24.0	// inches
	#define MAX_US_OFFSET_DISTANCE	24.0	// inches

	if( TRUE )
	{
		int SensorNumber;
		POINT PositionOffset;
		int  OffsetDistance = MAX_INT;
		FPOINT Sum = {0,0};
		FPOINT CorrectionNeeded = {0,0}; 
		FPOINT SensorOffset  = {0,0};
		int nSensorsToAverage = 0;
		int  ObjectType = GRIDMAP_OBJECT_NONE;

		// Handle the IR sensor readings
		for( SensorNumber = 0; SensorNumber < NUM_IR_SENSORS; SensorNumber++ )
		{
			CalculateSensorPositionOffset(
				MAX_IR_OFFSET_DISTANCE, SensorOffsetDegrees_IR[SensorNumber], g_SensorStatus.IR[SensorNumber],	// IN
				PositionOffset, OffsetDistance, ObjectType );													// OUT
				
			if( OffsetDistance < MAX_IR_OFFSET_DISTANCE )
				// Not sure if this is needed: (OffsetDistance > GRID_MAP_RESOLUTION ) //  greater then map round off
			{
				// Object found, within reasonable range of error
				nSensorsToAverage++;	// Keep track of the number of sensors to average
				Sum.x += PositionOffset.x;
				Sum.y += PositionOffset.y;
			}
		}
		// Now get the average correction needed in the X and Y directions
		if( 0 != nSensorsToAverage )
		{
			CorrectionNeeded.x = Sum.x/nSensorsToAverage;
			CorrectionNeeded.y = Sum.x/nSensorsToAverage;


			if( m_LandmarkUpdatesEnabled )
			{

				// Apply correction to current location!
				g_SensorStatus.CurrentLocation.x = (int) (g_SensorStatus.CurrentLocation.x + CorrectionNeeded.x );
				g_SensorStatus.CurrentLocation.y = (int) (g_SensorStatus.CurrentLocation.y + CorrectionNeeded.y );
				ROBOT_LOG( TRUE,  "POSITION CORRECTED by Object Detection.  DeltaX = %3.2f, DeltaY = %3.2f\n", CorrectionNeeded.x, CorrectionNeeded.y )
			}
			else
			{
				// For debugging this functionality:
//				ROBOT_LOG( TRUE,  "POSITION **RECOMMENDED** CORRECTION by Object Detection:  DeltaX = %03.1f, DeltaY = %03.1f\n", CorrectionNeeded.x, CorrectionNeeded.y )
			}
		}
	}


	/***** TODO-CAR - Future addition to update location using Landmarks!!

	if( NULL != m_pCurrentSegment )
	{
		if( ("Follow Wall" == m_pCurrentSegment->m_SegmentBehavior ) ||
			("Follow Curb" == m_pCurrentSegment->m_SegmentBehavior ) ||
			("Follow Hall" == m_pCurrentSegment->m_SegmentBehavior ) )
		{
			// Doing something special (not just dead reckoning )
			// Other possible Options: Compass+GPS, Follow Dropoff, Enter Doorway
			//CAR-TODO: UPdate location with Landmarks
		}
		else if( ("Follow Dropoff" == m_pCurrentSegment->m_SegmentBehavior )
		{

		}
	}
	*****/
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void CSensorModule::CalculateSensorPositionOffset( 
	int  SearchDistance, double SensorOffsetDegrees, int  ObjectDistance,	// IN Parameters
	POINT &PositionOffset, int  &OffsetDistance, int  &ObjectType )		// OUT Parameters
{
	// This function matches objects spotted by sensors to items on the gridmap
	// Given current robot direction, offset of the sensor, and reported range
	// OffsetDistance set to MAX_INT if not found within specified SearchDistance

	OffsetDistance = MAX_INT;				// No Object found
	PositionOffset.x = MAX_INT;
	PositionOffset.y = MAX_INT;

	if( NULL == g_pGridMap )
	{
		// No gridmap defined, just abort
		return;	// No object found
	}

	if( ObjectDistance >= NO_OBJECT_IN_RANGE )
	{
		return;	// No object found
	}

	double SensorDirection = g_SensorStatus.CompassHeading + SensorOffsetDegrees;
	if( SensorDirection >= 360 ) 
		SensorDirection -= 360;
	if( SensorDirection < 0 ) 
		SensorDirection += 360;

	// Map object location to where the object would be on the grid
	POINT	PhantomObjectPt;

	double SensorDirectionRadians = SensorDirection * DEGREES_TO_RADIANS;
	double dX = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * sin( SensorDirectionRadians );	// Returns negative numbers as needed
	double dY = (ObjectDistance + ROBOT_BODY_SENSOR_RADIUS_TENTH_INCHES) * cos( SensorDirectionRadians );
	if(dX>=0) dX += 0.5; else dX -= 0.5;	// Cast will truncate, this will round instead
	if(dY>=0) dY += 0.5; else dY -= 0.5;

	PhantomObjectPt.x = (int)g_SensorStatus.CurrentLocation.x + (int)dX; // Add delta to current Robot position
	PhantomObjectPt.y = (int)g_SensorStatus.CurrentLocation.y + (int)dY;


	// Ok, we've got the object's positon on the map. Find the nearest object to this point (if any)
	g_pGridMap->FindNearestObject(
		PhantomObjectPt, SearchDistance,				// IN Params
		PositionOffset, OffsetDistance, ObjectType );	// OUT Params

	// Return Offset X,Y and Distance.  Distance used to indicate if anything found in range.
}



void CSensorModule::FindNearestRadarObject( USHORT Length, int * ScaledBuffer )
{
	// Find object in array of US sensor values
#define RANGE_TOLERANCE_TENTH_INCHES 60 // TenthInches

	int  ObjectPosition = NO_OBJECT_IN_RANGE;
	int  ObjectLeft = NO_OBJECT_IN_RANGE;
	int  ObjectRight = NO_OBJECT_IN_RANGE;
	int  ObjectDistance = NO_OBJECT_IN_RANGE;

	// TODO - Average old and new!

	int  NearestObjectDirecton = NO_OBJECT_IN_RANGE;
	int  NearestObjectRange = NO_OBJECT_IN_RANGE;

	int  range;
	int i;


	// NOT USED
	ASSERT(0);

	// find nearest object
	for( i=0; i<Length; i++ )
	{
		range = ScaledBuffer[i];
		if( range < NearestObjectRange )
		{
			NearestObjectDirecton = i;
			NearestObjectRange = range;
		}
	}

	// OK, found the nearest point, now check for "center of mass" of object(s)
	BOOL MeasuringWidth = FALSE;
	int  ObjectWidth = NO_OBJECT_IN_RANGE;
	int  ObjectCenter = NO_OBJECT_IN_RANGE;

	// Search Left to Right
	for( i=0; i<Length; i++ )
	{
		range = ScaledBuffer[i];
		if( range < (NearestObjectRange + RANGE_TOLERANCE_TENTH_INCHES) )
		{
			// Object in delta range
				ObjectLeft = i;
				break;
		}
	}

	// Search Right to Left
	for( i=(Length-1); i >= 0; i++ )
	{
		range = ScaledBuffer[i];
		if( range < (NearestObjectRange + RANGE_TOLERANCE_TENTH_INCHES) )
		{
			// Object in delta range
				ObjectRight = i;
				break;
		}
	}

	// Ok, assuming we found an object, see if there is a gap in the middle
	// to indicate multiple objects
	int ObjectGapCount = 0;
	if( (ObjectLeft < NO_OBJECT_IN_RANGE) && (ObjectRight < NO_OBJECT_IN_RANGE) )
	{
		// got at least one object
		ObjectWidth = ObjectRight - ObjectLeft;
		
		for( int  i=ObjectLeft; i<=ObjectRight; i++ )
		{
			range = ScaledBuffer[i];
			if( range >= (NearestObjectRange + RANGE_TOLERANCE_TENTH_INCHES) )
			{
				// Outside the delta range
				if( ObjectGapCount++ > 3)	// abort if gap larger then 3
				{
					ROBOT_LOG( TRUE,  "Radar Find Object:  Gap to large: %d\n", ObjectGapCount)
					break;
				}
			}
			else
			{
				ObjectGapCount = 0;	// Reset if valid reading
			}
		}
	}

	// Ok, do the analysis
	if( (ObjectGapCount <= 3) && ((ObjectWidth-ObjectGapCount) >= 3) )
	{
		// Reasonably solid reading
//		g_pSensorSummary->nClosestRadarObjectDistance = NearestObjectRange;
//		g_pSensorSummary->nClosestRadarObjectLocation = ObjectLeft + (ObjectWidth/2); 
//		g_pSensorSummary->nClosestRadarObjectLeft = ObjectLeft;
//		g_pSensorSummary->nClosestRadarObjectRight = ObjectRight;
	
	
		ROBOT_LOG( TRUE,  "Radar Found Object at Pos: %d  Range: %d\n",ObjectPosition, ObjectDistance)
	}

}


// Standard Sharp GP2D12 IR sensor
// Value returned is TENTH INCHES
int  CSensorModule::ScaleIR(int  nReading )
{
	// NOTE!  Larger values are closer objects!  
	// Therefore, PIC_NO_OBJECT_IN_RANGE seems VERY CLOSE!  Trap here! 
	if( nReading >= PIC_NO_OBJECT_IN_RANGE )
		return NO_OBJECT_IN_RANGE;

	int MaxInches = IR_MAX_TENTH_INCHES / 10;
	for( int inches=0; inches <= MaxInches; inches++ )
	{
		if( nReading > IRDistanceTable[inches]  )
		{
			return (inches * 10);	// TenthInches
		}
	}
	return NO_OBJECT_IN_RANGE;	// No object in sensor range
}

// Sharp GP2Y0A21YK Wide Angle IR sensor
// Value returned is TENTH INCHES
int  CSensorModule::ScaleWideIR(int  nReading )
{
	// NOTE!  Larger values are closer objects!  
	// Therefore, PIC_NO_OBJECT_IN_RANGE seems VERY CLOSE!  Trap here! 
	if( nReading >= PIC_NO_OBJECT_IN_RANGE )
		return NO_OBJECT_IN_RANGE;

	int MaxInches = (IR_MAX_TENTH_INCHES / 10);
	for( int inches=0; inches <= MaxInches; inches++ )
	{
		if( nReading > DistanceTable_IRWideAngle[inches]  )
		{
			if( (inches*10) > IR_SR_MAX_RANGE_TENTH_INCHES )
			{
				return NO_OBJECT_IN_RANGE;	// No object in reliable sensor range
			}
			else
			{
				return (inches*10);
			}
		}
	}
	return NO_OBJECT_IN_RANGE;	// No object in sensor range
}

// Sharp GP2Y0A02YK Long Range IR sensor - 5 foot range
// Value returned is TENTH INCHES
int  CSensorModule::ScaleLongRangeIR(int  nReading, int Compensation )
{
	// Scale from raw value to inches.  Compensation may be passed in to correct known sensor differences
	// NOTE!  Larger values are closer objects!  
	// Therefore, PIC_NO_OBJECT_IN_RANGE seems VERY CLOSE!  Trap here! 
	if( nReading >= PIC_NO_OBJECT_IN_RANGE )
		return NO_OBJECT_IN_RANGE;

	if( nReading < 29 )	// Outside Arduino readable range value
		return NO_OBJECT_IN_RANGE;

	int RangeTenthInches = NO_OBJECT_IN_RANGE;

	int MaxInches = 48;	// Inches!
	for( int index=0; index <= MaxInches; index++ )	// 48" Max
	{
		if( nReading >= DistanceTable_IRLongRange[index]  )
		{
			RangeTenthInches = (index*10) + Compensation;
			if( RangeTenthInches < 0 ) RangeTenthInches = 0;
			return (RangeTenthInches);	// CONVERTED TO TENTH INCHES!!
		}
	}
	return NO_OBJECT_IN_RANGE;	// No object in sensor range
}

// Devantech SRF04 Ultrasonic Ranger - 8 foot range
// Value returned is TENTH INCHES
int  CSensorModule::ScaleSRF04UltraSonic(int  nReading )
{
	// Ultrasonic measurements are mostly linear, since they are time based
	// We use standard Y=MX+B, where M=Slope, B=Offset of the line  See RobotArrays.xls
	double CalculatedTenthInches = 0;

	// Y=MX+B, M=1.42,   B= -60
	CalculatedTenthInches = ((double)nReading * 1.42) - 620.0;
	if( CalculatedTenthInches < 1.0) CalculatedTenthInches = 0;

	if( CalculatedTenthInches > ULTRASONIC_TENTH_INCHES_MAX )
	{
		// Greater then "N" feet
		return NO_OBJECT_IN_RANGE;
	}

	return (int)(CalculatedTenthInches);

}

// Max EZ1 ultrasonic sensor
// Value returned is TENTH INCHES
int  CSensorModule::ScaleMaxEZ1UltraSonic(int  nReading )
{
	// Ultrasonic measurements are mostly linear, since they are time based
	// We use standard Y=MX+B, where M=Slope, B=Offset of the line
	// For Max EZ1, Y=MX+B, M=1.3800, B= -105, or Inches = 1.38*Reading - 105

//	CalculatedInches = ((double)nReading * 1.34) - 92.0;
	int CalculatedTenthInches = (int)(( ((double)nReading * 1.35) - 119.0 ) * 10.0);

	if( CalculatedTenthInches > ULTRASONIC_TENTH_INCHES_MAX )
	{
		// Greater then "N" inches
		return NO_OBJECT_IN_RANGE;
	}

	return (int)(CalculatedTenthInches);

}

// Max EZ1 ultrasonic sensor - analog output
// Value returned is TENTH INCHES
int  CSensorModule::ScaleMaxEZ1UltraSonicAnalog(int  nReading )
{
	// EZ1 Ultrasonic installed in Loki Head uses Analog out.
	// Measurements are mostly linear, since they are time based
	// We use standard Y=MX+B, where M=Slope, B=Offset of the line
	// For Max EZ1 Analog (as read by the Arduino), Y=MX+B, M=0.4375, B= 7, therefore Inches = 0.4375*Reading +7.0

	int CalculatedTenthInches = (int)(( ((double)nReading * 0.4375) + 7.0 ) * 10.0);

	if( CalculatedTenthInches > ULTRASONIC_TENTH_INCHES_MAX )
	{
		// Greater then "N" feet
		return NO_OBJECT_IN_RANGE;
	}

	return CalculatedTenthInches;

}


// Process array of US sensor values
void CSensorModule::ScaleUltraSonicData( USHORT Length, BYTE* InBuffer, int * OutBuffer )
{
	// Process the passed in buffer of UltraSonic data.
	ROBOT_LOG( TRUE,  "DEBUG RAW US RADAR: ");
	for( int i=0; i<Length; i++ )
	{
		ROBOT_LOG( TRUE,  "%d=%d ", i, InBuffer[i]);
	}
	ROBOT_LOG( TRUE,  "\n" );

	ROBOT_LOG( TRUE,  "DEBUG SCALED US RADAR: ");
	for( int i=0; i<Length; i++ )
	{
		OutBuffer[i] = ScaleSRF04UltraSonic(InBuffer[i]);
		ROBOT_LOG( TRUE,  "%d=%d ", i, OutBuffer[i]);
	}

	ROBOT_LOG( TRUE,  "\n" );

}

///////////////////////////////////////////////////////////////////////////////
void CSensorModule::ScaleIRArrayData( USHORT Length, BYTE* InBuffer, int * OutBuffer )
{
	// Process the passed in buffer of IR ranging data.

	for( int  i=0; i<Length; i++ )
	{
		OutBuffer[i] = ScaleLongRangeIR(InBuffer[i], 0);
	}
}

///////////////////////////////////////////////////////////////////////////////////////
void CSensorModule::ConvertAccToTilt( int AccelX, int AccelY, int  &SlopeDir, int  &SlopeAmt)
{
	// Given raw Acceleration values, returns direction of slope and steepness

	// Calculate direction of slope from the Tilt X,Y values
	double AngleRadians = atan2( (double)AccelX, (double)AccelY );	// X,Y backward from classical math
	double AngleDegrees = AngleRadians * RADIANS_TO_DEGREES;
	
	if( AngleDegrees <= 0 )
	{
		// Slope is on the Right Side
		AngleDegrees = AngleDegrees + 360;
	}	

	AngleDegrees += 0.5;	// Cast will truncate, this will round instead
	SlopeDir = (int )AngleDegrees;

	// Calculate steepness
	double TempSlopeAmt = ( sqrt((double)(AccelX*AccelX) + (double)(AccelY*AccelY)) );
	TempSlopeAmt = TempSlopeAmt * 0.380952380952381;
	TempSlopeAmt += 0.4;	// Cast will truncate, this will round instead
	SlopeAmt = (int )TempSlopeAmt;

}


///////////////////////////////////////////////////////////////////////////////////////
int  CSensorModule::CompensateForTilt( int  CompassIn, int  SlopeDir, int  SlopeAmt )
{

	const int LevelCorrection[12] =
	// Given Tilt, Slopedir, ReportedDir, returns Error correction needed (to add to reported value)
	{
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
 	   1, -1,  2,  4,  0, -3,  1, -1, -3, -2, -3, -4 };	// LEVEL ERROR!

/*****
	const int CompassCorrection[5][12][12] = 
	// Given Tilt, Slopedir, ReportedDir, returns Error correction needed (to add to reported value)
	// Tilt = 5-25 degrees, in  5 degree increments
	// SlopeDir =    0-360, in 30 degree increments
	// ReportedDir = 0-360, in 30 degree increments
	{ 

	// Tilt = 0 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
	// 359,31,58, 86,120,153,179,211,243,272,303,334	// LEVEL VALUE!


	// Tilt = 5 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
	   0,  2,  0,-13,  0,  0,  0, -3, -2, -2, -3,  0,	// Slopedir =   0
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  30
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  60
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  90
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 120
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 150
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 180
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 210
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 240
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 270
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 300
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 330

	// Tilt = 10 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
	// 359,31,58, 86,120,153,179,211,243,272,303,334	// LEVEL VALUE!
	   0,  0,  0,-24,  0,  0,  0, -3,  -5, -4, -6, -2,	// Slopedir =   0
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  30
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  60
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  90
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 120
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 150
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 180
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 210
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 240
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 270
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 300
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 330

	   
	// Tilt = 15 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir
	// 359,31,58, 86,120,153,179,211,243,272,303,334	// LEVEL VALUE!
 	// 1, -1,  2,  4,  0, -3,  1, -1, -3, -2, -3, -4 }	// LEVEL ERROR!
	// 0   1   2   3   4   5   6   7   8   9  10  11    -- INDEX

	  -2,-21,-40,-37,-35,-15,  5, 19, 29, 35, 20, 10,	// Slopedir =   0
	 -22,-34,-37,-29, -9,  1,  8, 27, 36, 23, 10,-11,	// Slopedir =  30
	 -42,-21,-33,-16,  1, 10, 25, 39, 22, 15,-20,-22,	// Slopedir =  60
	 -40,-33,-14, -1, 10, 25, 39, 27, 15,-23,-26,-46,	// Slopedir =  90
	 -33,-14, -1, 10, 30, 41, 38, 15,-20,-23,-53,-39,	// Slopedir = 120
	 -18, -3, 10, 33, 38, 32, 15,-22,-27,-54,-44,-35,	// Slopedir = 150
	  -5,  9, 31, 39, 32, 15,-13,-26,-50,-46,-32,-11,	// Slopedir = 180
	   9, 29, 35, 33, 10, -8,-29,-43,-45,-30,-17, -7,	// Slopedir = 210
	  30, 30, 24, 12, -7,-29,-43,-44,-40,-18, -10, 9,	// Slopedir = 240
	  28, 19,  1, -1,-22,-40,-38,-40,-13, -4,  9, 24,	// Slopedir = 270
	  18,  5, -1,-22,-32,-39,-36,-19, -1, 10, 27, 25,	// Slopedir = 300
	  10, -1,-21,-31,-39,-35,-12, -4, 10, 25, 26, 13,	// Slopedir = 330

	// Tilt = 20 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
	// 359,31,58, 86,120,153,179,211,243,272,303,334	// LEVEL VALUE!
	  44,  0,  0,  0,-39,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =   0
	   0,  0,  0,-35,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  30
	   0,  0,-37,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  60
	   0,-36,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  90
	 -34,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 120
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,-39,	// Slopedir = 150
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,-40,  0,	// Slopedir = 180
	   0,  0,  0,  0,  0,  0,  0,  0,  0,-38,  0,  0,	// Slopedir = 210
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 240
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 270
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 300
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 330

	// Tilt = 25 degrees
	// 0  30  60  90 120 150 180 210 240 270 300 330	// ReportedDir values
	// 359,31,58, 86,120,153,179,211,243,272,303,334	// LEVEL VALUE!
	   55,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =   0
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  30
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  60
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir =  90
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 120
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 150
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 180
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 210
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 240
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 270
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// Slopedir = 300
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };	// Slopedir = 330
*****/
	int ErrorAmount, LevelError;
	int  CompassInIndex;
	double Scale;
	int CorrectedValue;

	if( SlopeAmt == 0)
	{
		// Happens when no accelerometer attached
		return CompassIn;
	}

	CompassInIndex = CompassIn  / 30;
	if(CompassInIndex > 11)
	{
		ROBOT_LOG( TRUE,  "ERROR: Bad CompassInIndex = %d\n", CompassInIndex)
		CompassInIndex = 11;
	}
	LevelError = LevelCorrection[CompassInIndex];

	if( SlopeAmt <= 3)
	{
		ErrorAmount = LevelError;
	}
	else
	{
		Scale = 2.5 * SlopeAmt;		// 2.5 is the Scale Factor for tilt error
//		LevelError = LevelError * SlopeAmt / 2.5;
		ErrorAmount = (int)(sin( ((SlopeDir + CompassIn) * DEGREES_TO_RADIANS)) * Scale * -1);
	}

	CorrectedValue = CompassIn + ErrorAmount;

	if( CorrectedValue < 0 )
	{
		CorrectedValue = 360 + CorrectedValue;
	}
	
	if( CorrectedValue >= 360 )
	{
		CorrectedValue = CorrectedValue - 360;
	}

//	ROBOT_LOG( TRUE,  "\nDEBUG SlopeAmt=%02d, SlopeDir=%03d  CompassIn=%03d, ErrorAmt=%02d, LevelErr = %02d\n", 
//			SlopeAmt, SlopeDir, CompassIn, ErrorAmount, LevelError)
	
	return (int )CorrectedValue;
}


///////////////////////////////////////////////////////////////////////////////
int	CSensorModule::GetObjectDirection( int  SummaryLeft, int  SummaryRight, 
		int  IR_NumberLeft, int  IR_NumberRight, int  IR_SensorRange, int SensorAngle )
{
	// Calculate Direction of Object from forward center, for each of the Sensor pairs:
	// Forward, Angle, or Side

	int ObjectDirection = OBJECT_EQUAL_DISTANCE;

	if( abs((int)(SummaryRight - SummaryLeft) ) > US_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
	{
		// Object clearly to one side or the other. 
		ObjectDirection = ( SummaryRight < SummaryLeft ) ? SensorAngle : (-SensorAngle);
	}
	else
	{
		// Object seems dead ahead, within the US margin of error. 
		// Check IR sensor to see if we get better resolution
		if( (g_SensorStatus.IR[IR_NumberLeft]  < IR_SensorRange) ||	
			(g_SensorStatus.IR[IR_NumberRight] < IR_SensorRange) )
		{
			// Within Range of at least one of the IR sensors.  Use IR instead of US.
			if( abs((int)(g_SensorStatus.IR[IR_NumberRight] - g_SensorStatus.IR[IR_NumberLeft]) ) > IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
			{
				// object clearly to one side or the other
				ObjectDirection = ( g_SensorStatus.IR[IR_NumberRight] < g_SensorStatus.IR[IR_NumberLeft] ) 
					? SensorAngle : (-SensorAngle);
			}
			else
			{
				// Object pretty much dead ahead
				ObjectDirection = OBJECT_EQUAL_DISTANCE;
			}
		}
	}

	return ObjectDirection;
}

///////////////////////////////////////////////////////////////////////////////
int  CSensorModule::ReadElbowSensorsLeft( )
{
#if( ROBOT_HAS_LEFT_ARM )

	if( ARM_L_HW_BUMPER_OBJECT_ELBOW )
	{
		ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-L ELBOW HW BUMPER HIT! " )


///		g_pSensorSummary->bElbowHitLeft = TRUE;	// something hit the hardware bumper switch!
		return 0;	// Hit!
	}

	if( ARM_L_IR_SENSOR_ELBOW >= NO_OBJECT_IN_RANGE )
	{
		return ARM_L_IR_SENSOR_ELBOW;	// nothing detected by the IR sensor
	}

	if( ARM_L_IR_SENSOR_ELBOW >= FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_L )
	{
		// Object is at or beyond finger tip
		return ARM_L_IR_SENSOR_ELBOW - (int)FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_L;	// Distance from fingertip to object
	}

	// Object detected by Elbow sensor within length of arm
	// Check to see if wrist is rotated (left wrist servo blocks sensor at times)
	int WristPosDegrees = m_pArmControlLeft->GetWristPosition();
//	ROBOT_LOG( TRUE,  "**************** LEFT ARM HIT - WRIST = %d\n", WristPosDegrees)
	if( (WristPosDegrees > 66) && (WristPosDegrees < 87) )
	{
		// Claw Servo in front of Elbow sensor.  But, we can still
		// detect an object hitting the elbow behind the wrist
		if( ARM_L_IR_SENSOR_ELBOW < FOREARM_IR_TO_WRIST_OBSTRUCTION_TENTH_INCHES_L )
		{
			return 0;	// Hit!
		}
		// else we are probably detecting the wrist.  Ignore it.
		return NO_OBJECT_IN_RANGE;
	}

	// Wrist not rotated
	return 0;	// Hit!

#else
	return NO_OBJECT_IN_RANGE;
#endif

}
///////////////////////////////////////////////////////////////////////////////
int  CSensorModule::ReadElbowSensorsRight( )
{
#if( ROBOT_HAS_RIGHT_ARM )

	if( ARM_R_HW_BUMPER_OBJECT_ELBOW )
	{
		ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-R ELBOW HW BUMPER HIT! " )
///		g_pSensorSummary->bElbowHitRight = TRUE;	// something hit the hardware bumper switch!
		return 0;	// Hit!
	}

	if( ARM_R_IR_SENSOR_ELBOW >= NO_OBJECT_IN_RANGE )
	{
		return ARM_R_IR_SENSOR_ELBOW;	// nothing detected by the IR sensor
	}

	if( ARM_R_IR_SENSOR_ELBOW >= FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_R )
	{
		// Object is at or beyond finger tip
		return ARM_R_IR_SENSOR_ELBOW - (int)FOREARM_IR_TO_FINGER_TIP_TENTH_INCHES_R;	// Distance from fingertip to object
	}

	// Object detected by Elbow sensor within length of arm
	// Check to see if wrist is rotated
	int WristPosDegrees = m_pArmControlRight->GetWristPosition();
	ROBOT_LOG( TRUE,  "**************** RIGHT ARM HIT - WRIST = %d\n", WristPosDegrees)
/*	
	if( WristPosDegrees > 50 )
	{
		// Claw Servo in front of Elbow sensor.  Disable Elbow Sensor readings.
		if( ARM_R_IR_SENSOR_ELBOW < FOREARM_IR_TO_WRIST_OBSTRUCTION_R )
		{
			// Wrist rotated, but we still can detect object blocking the elbow
			ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-R ELBOW IR BUMPER HIT! " )
			return 0;	// Hit!
		}
		// else we are probably detecting the wrist.  Ignore it.
		return NO_OBJECT_IN_RANGE;
	}
*/
	// Wrist not rotated
	return 0;	// Hit!

#else
	return NO_OBJECT_IN_RANGE;
#endif

}

///////////////////////////////////////////////////////////////////////////////
void CSensorModule::DoSensorFusion()
{
	// Combine any redundant sensors, and summarize results into g_pSensorSummary

	// CARBOT Sensor Positions in arrays
	/*
	// Sensor Positions in arrays.  Defined in HardwareCmds.h
	#define	IR_SENSOR_SIDE_LEFT				 0
	#define	IR_SENSOR_FRONT_LEFT			 1
	#define	IR_SENSOR_FRONT_RIGHT			 2
	#define	IR_SENSOR_SIDE_RIGHT			 3

	// US Sensors are arranged in a semi-circle,
	// Numbered left to right (1-6)
	#define US_SENSOR_CAMERA				 0	// Mounted on Camera
	#define US_SENSOR_SIDE_LEFT				 1
	#define US_SENSOR_ANGLE_LEFT			 2
	#define US_SENSOR_FRONT_LEFT			 3
	#define US_SENSOR_FRONT_RIGHT			 4	
	#define US_SENSOR_ANGLE_RIGHT			 5
	#define US_SENSOR_SIDE_RIGHT			 6
	*/

	// Angle sensors use local variable, not in SensorSummary
	int  nAngleObjectDistance = NO_OBJECT_IN_RANGE;
	int nAngleObjectDirection = 0;


	///////////////////////////////////////////////////////////////////////////////
	#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

		// Detailed Fused Sensor summary, used in cases where general info is not enough
		g_pSensorSummary->nLeftSideZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nLeftArmZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nLeftFrontZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightFrontZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightArmZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightSideZone =		NO_OBJECT_IN_RANGE;

		// Summary sensor info
		g_pSensorSummary->nFrontObjectDistance =	NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nSideObjectDistance =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRearObjectDistance =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nFrontObjectDirection =	OBJECT_EQUAL_DISTANCE;
		g_pSensorSummary->nSideObjectDirection =	OBJECT_EQUAL_DISTANCE;
	//	g_pSensorSummary->nRearObjectDirection =	OBJECT_EQUAL_DISTANCE;
	//	g_pSensorSummary->bLeftCliff =				FALSE;
	//	g_pSensorSummary->bRightCliff =				FALSE;
	


	//	g_pSensorSummary->nClosestFrontObjectDistance = NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nClosestFrontObjectDirection = 0;	// Degrees from front (left is negative)
	//	g_pSensorSummary->nClearestPathDirection = 0;	// Degrees from front (left is negative)


		// CENTER FRONT SENSORS - Combine center front facing US, IR and bumper info
		// Bumper collisions override other sensors
		if( HW_BUMPER_HIT_FRONT_RIGHT && HW_BUMPER_HIT_FRONT_LEFT )
		{		
			// Center Hit on Both HW bumpers
			g_pSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;
			g_pSensorSummary->nLeftFrontZone = 0;
			g_pSensorSummary->nRightFrontZone = 0;
		}
		else if( HW_BUMPER_HIT_FRONT_LEFT )
		{		
			// Hit on Left HW bumper (mostly in front, but slightly to one side)
			g_pSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
			g_pSensorSummary->nLeftFrontZone = 0;
		}
		else if( HW_BUMPER_HIT_FRONT_RIGHT )
		{		
			// Hit on Right HW bumper
			g_pSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
			g_pSensorSummary->nRightFrontZone = 0;
		}
		else
		{
			// No Bumper collision, summarize other sensor info
			g_pSensorSummary->nLeftFrontZone = __min(
				g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT], g_SensorStatus.US[US_SENSOR_FRONT_LEFT] );
			g_pSensorSummary->nRightFrontZone = __min(
				g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT], g_SensorStatus.US[US_SENSOR_FRONT_RIGHT] );

		}

		// ANGLED FRONT SENSORS
		// Carbot Has US angle sensors only.  No angled IR
		g_pSensorSummary->nLeftArmZone =  g_SensorStatus.US[US_SENSOR_ANGLE_LEFT] ;
		g_pSensorSummary->nRightArmZone = g_SensorStatus.US[US_SENSOR_ANGLE_RIGHT] ;


		// SIDE SENSORS
		g_pSensorSummary->nLeftSideZone = __min(
			g_SensorStatus.IR[IR_SENSOR_SIDE_LEFT], g_SensorStatus.US[US_SENSOR_SIDE_LEFT] );
		g_pSensorSummary->nRightSideZone = __min(
			g_SensorStatus.IR[IR_SENSOR_SIDE_RIGHT], g_SensorStatus.US[US_SENSOR_SIDE_RIGHT] );
		g_pSensorSummary->nSideObjectDistance = __min( 
			g_pSensorSummary->nLeftSideZone, g_pSensorSummary->nRightSideZone );


		// Now, check Camera sensor and override prior calculated values if needed
		// TODO-CAR-MUST!!! NEED TO SET THE SERVO VALUES BELOW TO CORRECT VALUES!
		if(	g_CameraTiltPos > (CAMERA_TILT_CENTER -15) )	// Must not be pointing at the ground! 
		{
			if( g_CameraPanPos < (CAMERA_PAN_CENTER -30) )	
			{
				// Camera pointing Left Side
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nLeftSideZone )
				{
					g_pSensorSummary->nLeftSideZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -20) )	
			{
				// Camera pointing Left Angle
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nLeftArmZone )
				{
					g_pSensorSummary->nLeftArmZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -5) )	
			{
				// Camera pointing Left Front
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nLeftFrontZone )
				{
					g_pSensorSummary->nLeftFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +30) )	
			{
				// Camera pointing Right Side
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nRightSideZone)
				{
					g_pSensorSummary->nRightSideZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +20) )	
			{
				// Camera pointing Right Angle
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nRightArmZone )
				{
					g_pSensorSummary->nRightArmZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +5) )	
			{
				// Camera pointing Right Front
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nRightFrontZone )
				{
					g_pSensorSummary->nRightFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else
			{
				// Camera sensor is pointing forward.  Override both front sensors if needed.
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nLeftFrontZone )
				{
					g_pSensorSummary->nLeftFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nRightFrontZone )
				{
					g_pSensorSummary->nRightFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}

		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now summarize high level status

		if( 0 != g_pSensorSummary->nFrontObjectDistance )
		{
			// Skip this if a bumper was hit

			g_pSensorSummary->nFrontObjectDistance = __min( 
				g_pSensorSummary->nLeftFrontZone, g_pSensorSummary->nRightFrontZone );

			g_pSensorSummary->nFrontObjectDirection = GetObjectDirection(
				g_pSensorSummary->nLeftFrontZone, g_pSensorSummary->nRightFrontZone,
				IR_SENSOR_FRONT_LEFT, IR_SENSOR_FRONT_RIGHT, IR_LR_DETECT_RANGE, FORWARD_SENSOR_ANGLE ); // NOTE! Assumes *LONG RANGE IR* Sensors on front of robot!
		}

		// No Angle IR, so instead of calling GetObjectDirection(), just handle here
		if( abs(g_pSensorSummary->nRightArmZone - g_pSensorSummary->nLeftArmZone ) > US_RANGE_FUDGE_AMOUNT )
		{
			// Object clearly to one side or the other. 
			nAngleObjectDirection = ( g_pSensorSummary->nRightArmZone < g_pSensorSummary->nLeftArmZone ) ? ANGLE_SENSOR_ANGLE : (-ANGLE_SENSOR_ANGLE);
		}
		else
		{
			// Object pretty much dead ahead
			nAngleObjectDirection = OBJECT_EQUAL_DISTANCE;
		}

		// Now, combine angle and forward sensor info
		if( nAngleObjectDistance < g_pSensorSummary->nFrontObjectDistance )
		{
			// Closest Front object is at an angle
			g_pSensorSummary->nFrontObjectDistance = nAngleObjectDistance;
			g_pSensorSummary->nFrontObjectDirection = nAngleObjectDirection;
		}

		g_pSensorSummary->nSideObjectDirection = GetObjectDirection(
			g_pSensorSummary->nLeftSideZone, g_pSensorSummary->nRightSideZone,
			IR_SENSOR_SIDE_LEFT, IR_SENSOR_SIDE_RIGHT, IR_LR_DETECT_RANGE, SIDE_SENSOR_ANGLE ); // NOTE! Assumes *LONG RANGE IR* Sensors on side of robot!


	///////////////////////////////////////////////////////////////////////////////
	#elif ( SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI )
		////////////////////////////////////////////////////////
		// NOTE! Ultrasonic not reliable indoors for now!
		// use IR only! - TODO
		////////////////////////////////////////////////////////

		// CString strSensor;

		// Initialize SENSOR_SUMMARY_T
		// Note, if sensor not installed, use SENSOR_DISABLED
		g_pSensorSummary->nFrontObjectDistance =	NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nFrontObjectDirection =	OBJECT_EQUAL_DISTANCE;	// Degrees from front (left is negative)
		g_pSensorSummary->nSideObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nSideObjectDirection =	OBJECT_EQUAL_DISTANCE;
		g_pSensorSummary->nRearObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nRearObjectDirection =	OBJECT_EQUAL_DISTANCE;

		g_pSensorSummary->nLeftRearZone =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bLeftCliff =				FALSE;
		g_pSensorSummary->nObjectClawLeft =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nObjectArmLeft =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nLeftSideZone =			NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftFrontSideZone =		NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftArmZone =			NO_OBJECT_IN_RANGE;	// Object in front of Arm
		g_pSensorSummary->nLeftFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_NONE;	//////// Center
		g_pSensorSummary->nRightFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightArmZone =			NO_OBJECT_IN_RANGE; // Object in front of Arm
		g_pSensorSummary->nRightFrontSideZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightSideZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nObjectArmRight =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nObjectClawRight =		NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bRightCliff =				FALSE;
		g_pSensorSummary->nRightRearZone =			NO_OBJECT_IN_RANGE;	

		g_pSensorSummary->nClosestObjectFrontLeft =	NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nClosestObjectFrontRight= NO_OBJECT_IN_RANGE;


		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get Laser Scanner data, if the Laser exists.  May be overridden by other sensors later


		// ERROR??? CHECK THIS TODO-MUST-DWS
		// Note that laser data is in tenth inches, but other sensor are in inches (due to low resoluton)
		// For now, only inches used for object avoidance and colision.  Maybe change to tenthinches or float later?


		if( 0 != g_pLaserSummary->SampleTimeStamp )	 // Skip this if sensor not installed
		{
			int TimeSinceLastLaserScan =  GetTickCount() - g_pLaserSummary->SampleTimeStamp;
			if( TimeSinceLastLaserScan < 1000 )
			{	
				// Last scan was within 1 second
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csLaserSummaryDataLock);
				EnterCriticalSection(&g_csLaserSummaryDataLock);
			//		g_pSensorSummary->nLeftRearZone =			g_pLaserSummary->nLeftRearZone;	
			///		g_pSensorSummary->bLeftCliff =				g_pLaserSummary->bLeftCliff;
					g_pSensorSummary->nLeftSideZone =			g_pLaserSummary->nLeftSideZone;	 
					g_pSensorSummary->nLeftFrontSideZone =		g_pLaserSummary->nLeftFrontSideZone;	 
					g_pSensorSummary->nLeftArmZone =			g_pLaserSummary->nLeftArmZone;	// Object in front of Arm
					g_pSensorSummary->nLeftFrontZone =			g_pLaserSummary->nLeftFrontZone;
					g_pSensorSummary->nRightFrontZone =			g_pLaserSummary->nRightFrontZone;
					g_pSensorSummary->nRightArmZone =			g_pLaserSummary->nRightArmZone; // Object in front of Arm
					g_pSensorSummary->nRightFrontSideZone =		g_pLaserSummary->nRightFrontSideZone;
					g_pSensorSummary->nRightSideZone =			g_pLaserSummary->nRightSideZone;
			///		g_pSensorSummary->bRightCliff =				g_pLaserSummary->bRightCliff;
			//		g_pSensorSummary->nRightRearZone =			g_pLaserSummary->nRightRearZone;	
					/*
					g_pLaserSummary->RobotLocation.x = 0;
					g_pLaserSummary->RobotLocation.y = 0;
					g_pLaserSummary->CompassHeading = 0;
					g_pLaserSummary->SampleTimeStamp = 0;
					*/
				LeaveCriticalSection(&g_csLaserSummaryDataLock);
				__itt_task_end(pDomainControlThread);
			}
			else
			{
/// TODO-DWS fix this				ROBOT_LOG(TRUE, "ERROR: Last Laser Scan more than 1 second ago! (and value not zero)\n" )
			}
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get Kinect sensor data, if the Kinect exists.  May be overridden by other sensors later
		// Note that Kinect data is in tenth inches, but other sensor are in inches (due to low resoluton)
		// For now, only inches used for object avoidance and colision.  Maybe change to tenthinches or float later?

		if( 0 != g_pKinectSummary->SampleTimeStamp )	 // Skip this if sensor not installed
		{
			int TimeSinceLastKinectScan =  GetTickCount() - g_pKinectSummary->SampleTimeStamp;
			if( TimeSinceLastKinectScan < 1000 )
			{	
				// Last scan was within 1 second
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csKinectSummaryDataLock);
				EnterCriticalSection(&g_csKinectSummaryDataLock);
		//		g_pSensorSummary->nLeftRearZone =			__min( g_pSensorSummary->nLeftRearZone, g_pKinectSummary->nLeftRearZone );	
		///		g_pSensorSummary->bLeftCliff =				g_pKinectSummary->bLeftCliff;
				g_pSensorSummary->nLeftSideZone =			__min( g_pSensorSummary->nLeftSideZone, g_pKinectSummary->nLeftSideZone );	 
				g_pSensorSummary->nLeftFrontSideZone =		__min( g_pSensorSummary->nLeftFrontSideZone, g_pKinectSummary->nLeftFrontSideZone );	 
				g_pSensorSummary->nLeftArmZone =			__min( g_pSensorSummary->nLeftArmZone, g_pKinectSummary->nLeftArmZone );	// Object in front of Arm
				g_pSensorSummary->nLeftFrontZone =			__min( g_pSensorSummary->nLeftFrontZone, g_pKinectSummary->nLeftFrontZone );
				g_pSensorSummary->nRightFrontZone =			__min( g_pSensorSummary->nRightFrontZone, g_pKinectSummary->nRightFrontZone );
				g_pSensorSummary->nRightArmZone =			__min( g_pSensorSummary->nRightArmZone, g_pKinectSummary->nRightArmZone ); // Object in front of Arm
				g_pSensorSummary->nRightFrontSideZone =		__min( g_pSensorSummary->nRightFrontSideZone, g_pKinectSummary->nRightFrontSideZone );
				g_pSensorSummary->nRightSideZone =			__min( g_pSensorSummary->nRightSideZone, g_pKinectSummary->nRightSideZone );
		///		g_pSensorSummary->bRightCliff =				g_pKinectSummary->bRightCliff;
		//		g_pSensorSummary->nRightRearZone =			__min( g_pSensorSummary->nRightRearZone, g_pKinectSummary->nRightRearZone );	
				/*
				g_pKinectSummary->RobotLocation.x = 0;
				g_pKinectSummary->RobotLocation.y = 0;
				g_pKinectSummary->CompassHeading = 0;
				g_pKinectSummary->SampleTimeStamp = 0;
				*/
			}
			else
			{
				//ROBOT_LOG(TRUE, "ERROR: Last Kinect Scan more than 1 second ago! (and value not zero)\n" )
			}
		}
		LeaveCriticalSection(&g_csKinectSummaryDataLock);
		__itt_task_end(pDomainControlThread);


		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get first level summary data for each sensor.  May be overridden by bumpers later

		// Vertically pointing IR sensors
		//BOOL VerticalObjectDetectedFrontLeft   = (g_SensorStatus.IR2[0] < VERTICAL_IR_DETECT_RANGE_TENTH_INCHES) ? TRUE : FALSE;
		//BOOL VerticalObjectDetectedFrontRight  = (g_SensorStatus.IR2[1] < VERTICAL_IR_DETECT_RANGE_TENTH_INCHES) ? TRUE : FALSE;

		// Front sensors
		g_pSensorSummary->nLeftFrontZone  = __min( g_pSensorSummary->nLeftFrontZone,  g_SensorStatus.IR[IR_SENSOR_FRONT_LEFT] );
		g_pSensorSummary->nRightFrontZone = __min( g_pSensorSummary->nRightFrontZone, g_SensorStatus.IR[IR_SENSOR_FRONT_RIGHT] );

		// Claw IR sensors
		g_pSensorSummary->nObjectClawLeft  = (ARM_L_IR_SENSOR_CLAW <= 30) ? 0 : (ARM_L_IR_SENSOR_CLAW - 30); // distance from sensor to tip of claw - Tenth Inches
		g_pSensorSummary->nObjectClawRight = (ARM_R_IR_SENSOR_CLAW <= 30) ? 0 : (ARM_R_IR_SENSOR_CLAW - 30); // distance from sensor to tip of claw
		// NO LONGER USED
		// if( ARM_L_IR_BUMPER_OBJECT_FINGER_L || ARM_L_IR_BUMPER_OBJECT_FINGER_R )
		//{
			//g_pSensorSummary->nObjectClawLeft  = __min( g_pSensorSummary->nObjectClawLeft, 30);	// range of Fingertip IR sensors
		//}

		// Left Claw pressure sensors
		// TODO : int LeftHandPressure = g_pSensorSummary->LeftHandPressureL;


		// Angled IR "Bumpers"
		int BumperLeft =  (IR_BUMPER_OBJECT_FRONT_LEFT)  ? IR_BUMPER_RANGE: NO_OBJECT_IN_RANGE;
		int BumperRight = (IR_BUMPER_OBJECT_FRONT_RIGHT) ? IR_BUMPER_RANGE: NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nLeftArmZone  = __min( g_pSensorSummary->nLeftArmZone,  BumperLeft);
		g_pSensorSummary->nRightArmZone  = __min( g_pSensorSummary->nRightArmZone,  BumperRight);


		// Check Elbow mounted IR sensors.  Handles case of claw blocking IR
		/**
		g_pSensorSummary->nObjectArmLeft = ReadElbowSensorsLeft();
		if( 0 == g_pSensorSummary->nObjectArmLeft )
		{
			strSensor = "SensorModule: ARM-L ELBOW HIT! ";
			ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-L ELBOW HIT! " )
		}
		g_pSensorSummary->nObjectArmRight = ReadElbowSensorsRight();
		if( 0 == g_pSensorSummary->nObjectArmRight )
		{
			ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-R ELBOW HIT! " )
		}
		**/

		// Check Bumpers
		if( HW_BUMPER_HIT_FRONT && ENABLE_BUMPERS )
		{		
			// Only one front HW bumper on Loki
			g_pSensorSummary->nLeftFrontZone = 0; // Collision!
			g_pSensorSummary->nRightFrontZone = 0; // Collision!
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Front Bumper!\n" )
			SpeakText( "Sorry" );	
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now, continue combining sensor info for front sensors

		// Start with analog sensors, then digital.  Look for closest object on each front area (left and right).

	/*** DISABLE ARM SENSORS.  JUST USE THE LASER
		if( gArmInHomePositionLeft )
		{
			g_pSensorSummary->nLeftArmZone = __min( g_pSensorSummary->nLeftArmZone, g_pSensorSummary->nObjectClawLeft );
			g_pSensorSummary->nLeftArmZone = __min( g_pSensorSummary->nLeftArmZone, g_pSensorSummary->nObjectArmLeft );
		}

		if( gArmInHomePositionRight )
		{
			g_pSensorSummary->nRightArmZone = __min( g_pSensorSummary->nRightArmZone, g_pSensorSummary->nObjectClawRight );
			g_pSensorSummary->nRightArmZone = __min( g_pSensorSummary->nRightArmZone, g_pSensorSummary->nObjectArmRight );
		}
	***/
		g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftFrontZone ); // Front IR, Laser, and bumpers
		g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftArmZone );

		g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightFrontZone );
		g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightArmZone );

		if( abs((int)(g_pSensorSummary->nClosestObjectFrontRight - g_pSensorSummary->nClosestObjectFrontLeft) ) < IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
		{
			// within error margin.  Assume objects straight ahead
			g_pSensorSummary->nFrontObjectDistance = (g_pSensorSummary->nClosestObjectFrontRight + g_pSensorSummary->nClosestObjectFrontLeft) /2;
			g_pSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;	// Straight Ahead
		}
		else if( g_pSensorSummary->nClosestObjectFrontRight < g_pSensorSummary->nClosestObjectFrontLeft )
		{
			// Closest Object to the Right
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontRight;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
		}
		else
		{
			// Closest Object to the Left
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontLeft;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
		}

		// Display final result if debugging
	/*	if( g_pSensorSummary->nFrontObjectDistance < 18 ) // Test distance
		{
			#if REPORT_CLOSE_OBJECTS = 1
				CString strSensor;
				strSensor.Format( "SensorModule: FrontObjectDist = %d,  FrontObjectDir = %d, Closest FrontLeft = %d FrontRight = %d",
					g_pSensorSummary->nFrontObjectDistance, g_pSensorSummary->nFrontObjectDirection,
					g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nClosestObjectFrontRight );
				ROBOT_DISPLAY( TRUE, ((LPCTSTR)strSensor) )
			#endif

		}
	*/

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// REAR SENSORS
		// Loki has one Rear HW bumper, and 2 IR Bumpers
		if( HW_BUMPER_HIT_REAR && ENABLE_BUMPERS )
		{
			//SpeakCannedPhrase( SPEAK_COLLISION );
			g_pSensorSummary->nRearObjectDirection = OBJECT_EQUAL_DISTANCE;
			g_pSensorSummary->nRearObjectDistance = 0;	// Collision!
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Rear Bumper!\n" )
			g_pSensorSummary->nLeftRearZone = 0;
			g_pSensorSummary->nRightRearZone = 0;
		}
		else if( IR_BUMPER_OBJECT_REAR_LEFT || IR_BUMPER_OBJECT_REAR_RIGHT )
		{
			//SpeakCannedPhrase( SPEAK_COLLISION );
			g_pSensorSummary->nRearObjectDistance = 1;	// Almost Collision (since sensors sit in a bit from the back)

			if( IR_BUMPER_OBJECT_REAR_LEFT && IR_BUMPER_OBJECT_REAR_RIGHT )
			{
				// Hit on Both IR bumpers!  Oh no!  what to do?
				g_pSensorSummary->nRearObjectDirection = OBJECT_EQUAL_DISTANCE;
				g_pSensorSummary->nLeftRearZone = 1;
				g_pSensorSummary->nRightRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Both Rear IR Bumpers!\n " )
			}
			else if( IR_BUMPER_OBJECT_REAR_LEFT )
			{		
				// Hit on Left IR bumper
				g_pSensorSummary->nRearObjectDirection = REAR_LEFT;
				g_pSensorSummary->nLeftRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Rear Left IR Bumper!\n " )
			}
			else if( IR_BUMPER_OBJECT_REAR_RIGHT )
			{		
				// Hit on Right IR bumper
				g_pSensorSummary->nRearObjectDirection = REAR_RIGHT;
				g_pSensorSummary->nRightRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Rear Right IR Bumper!\n " )
			}
			else
			{
				// Logic Error
				ROBOT_ASSERT(0);
			}
		}

		// CLIFF SENSOR

		// Loki has 2 IR Cliff Sensors
		if( m_CliffSensorsEnabled )
		{
			if( IR_BUMPER_CLIFF_LEFT || IR_BUMPER_CLIFF_RIGHT )
			{
				//SpeakCannedPhrase( SPEAK_COLLISION );

				if( IR_BUMPER_CLIFF_LEFT && IR_BUMPER_CLIFF_RIGHT )
				{
					// Cliff on both sides  Oh no!  what to do?
					g_pSensorSummary->bLeftCliff = TRUE;
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: ERROR!!!! Cliff on Both Sides!!!\n" )
					SpeakText( "Error. Cliff on both sides" );	

				}
				else if( IR_BUMPER_CLIFF_LEFT )
				{		
					g_pSensorSummary->bLeftCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Left Side!\n" )
					SpeakText( "Cliff Left" );	


				}
				else if( IR_BUMPER_CLIFF_RIGHT )
				{		
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Right Side!\n" )
					SpeakText( "Cliff Right" );	

				}
				else
				{
					// Logic Error
					ROBOT_ASSERT(0);
				}
			}
		}

		// SIDE SENSORS
		// Loki has IR side sensors and side bumpers

		// Handle analog sensors first
		g_pSensorSummary->nLeftSideZone  = __min( g_pSensorSummary->nLeftSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_LEFT]);
		g_pSensorSummary->nRightSideZone  = __min( g_pSensorSummary->nRightSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_RIGHT]);


		// Then bumpers
		if( HW_BUMPER_HIT_SIDE_LEFT && ENABLE_BUMPERS )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
			g_pSensorSummary->nLeftSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Left Side Bumper!\n" )
			SpeakText( "Oops" );	
		}
		if( HW_BUMPER_HIT_SIDE_RIGHT && ENABLE_BUMPERS )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
			g_pSensorSummary->nRightSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Right Side Bumper!\n" )
			SpeakText( "Darn" );	
		}

		// Now, sumarize

		if( g_pSensorSummary->nLeftSideZone == g_pSensorSummary->nRightSideZone )
		{
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = OBJECT_EQUAL_DISTANCE;
		}
		else if( g_pSensorSummary->nLeftSideZone < g_pSensorSummary->nRightSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
		}
		else if( g_pSensorSummary->nRightSideZone < g_pSensorSummary->nLeftSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nRightSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
		}
		else
		{
			ROBOT_ASSERT(0); // Logic Error
		}

	
		// Check PIR Motion Sensors
		//	ROBOT_LOG( TRUE,  ">>> DEBUG PIR IR BYTE = %02X\n", (g_SensorStatus.IRBumper & 0xC0) )
		if( PIR_MOTION_DETECTED_LEFT || PIR_MOTION_DETECTED_RIGHT )
		{
			if( PIR_MOTION_DETECTED_LEFT && PIR_MOTION_DETECTED_RIGHT )
			{
				g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_BOTH;
				ROBOT_LOG( DEBUG_PIR, "SensorModule: HUMAN DETECTED Both Sensors!\n " )
			}
			else if( PIR_MOTION_DETECTED_LEFT )
			{
				g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_LEFT;
				ROBOT_LOG( DEBUG_PIR, "SensorModule: HUMAN DETECTED Left!\n " )
			}
			else if( PIR_MOTION_DETECTED_RIGHT )
			{
				g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_RIGHT;
				ROBOT_LOG( DEBUG_PIR, "SensorModule: HUMAN DETECTED Right!\n " )
			}
			else
			{
				ROBOT_DISPLAY( TRUE, "\n\nSensorModule: MOTION DETECT LOGIC ERROR!!! ***********\n\n" )
				ROBOT_ASSERT(0);
			}
		}




		// Now, check Camera sensor and override prior calculated values if needed
		// TODO-LOKI-MUST!!! NEED TO SET THE SERVO VALUES BELOW TO CORRECT VALUES!
		/******** TODO-MUST

		if(	g_CameraTiltPos > (CAMERA_TILT_CENTER -15) )	// Must not be pointing at the ground! 
		{
			if( g_CameraPanPos < (CAMERA_PAN_CENTER -30) )	
			{
				// Camera pointing Left Side
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nSideObjectDistance )
				{
					// Object is closer then previously reported
					g_pSensorSummary->nSideObjectDistance = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
					g_pSensorSummary->nLeftSideZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -20) )	
			{
				// Camera pointing Left Angle
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nAngleObjectDistance )
				{
					// Object is closer then previously reported
					g_pSensorSummary->nAngleObjectDistance = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nAngleObjectDirection = ANGLE_LEFT;
					g_pSensorSummary->nLeftArmZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			if( g_CameraPanPos > (CAMERA_PAN_CENTER +30) )	
			{
				// Camera pointing Right Side
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nSideObjectDistance )
				{
					// Object is closer then previously reported
					g_pSensorSummary->nSideObjectDistance = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
					g_pSensorSummary->nRightSideZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +20) )	
			{
				// Camera pointing Right Angle
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nAngleObjectDistance )
				{
					// Object is closer then previously reported
					g_pSensorSummary->nAngleObjectDistance = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nAngleObjectDirection = ANGLE_RIGHT;
					g_pSensorSummary->nRightArmZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
			else
			{
				// Camera sensor is pointing forward
				if( (g_SensorStatus.US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pSensorSummary->nFrontObjectDistance )
				{
					// Object is closer then previously reported
					g_pSensorSummary->nFrontObjectDistance = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;
					g_pSensorSummary->nLeftFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
					g_pSensorSummary->nRightFrontZone = g_SensorStatus.US[US_SENSOR_CAMERA];
				}
			}
		}
		*****/
	#elif ( SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE )

		// CString strSensor;

		// Initialize SENSOR_SUMMARY_T
		// Note, if sensor not installed, use SENSOR_DISABLED
		g_pSensorSummary->nFrontObjectDistance =	NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nFrontObjectDirection =	OBJECT_EQUAL_DISTANCE;	// Degrees from front (left is negative)
		g_pSensorSummary->nSideObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nSideObjectDirection =	OBJECT_EQUAL_DISTANCE;
		g_pSensorSummary->nRearObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nRearObjectDirection =	OBJECT_EQUAL_DISTANCE;

		g_pSensorSummary->nLeftRearZone =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bLeftCliff =				FALSE;
		g_pSensorSummary->nObjectClawLeft =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nObjectArmLeft =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nLeftSideZone =			NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftFrontSideZone =		NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftArmZone =			NO_OBJECT_IN_RANGE;	// Object in front of Arm
		g_pSensorSummary->nLeftFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_NONE;	//////// Center
		g_pSensorSummary->nRightFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightArmZone =			NO_OBJECT_IN_RANGE; // Object in front of Arm
		g_pSensorSummary->nRightFrontSideZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightSideZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nObjectArmRight =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nObjectClawRight =		NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bRightCliff =				FALSE;
		g_pSensorSummary->nRightRearZone =			NO_OBJECT_IN_RANGE;	

		g_pSensorSummary->nClosestObjectFrontLeft =	NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nClosestObjectFrontRight= NO_OBJECT_IN_RANGE;

			
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get Kinect sensor data, if the Kinect exists.  May be overridden by other sensors later
		// Note that Kinect data is in tenth inches, but other sensor are in inches (due to low resoluton)
		// For now, only inches used for object avoidance and colision.  Maybe change to tenthinches or float later?

		if( 0 != g_pKinectSummary->SampleTimeStamp )	 // Skip this if sensor not installed
		{
			int TimeSinceLastKinectScan =  GetTickCount() - g_pKinectSummary->SampleTimeStamp;
			if( TimeSinceLastKinectScan < 1000 )
			{	
				// Last scan was within 1 second
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csKinectSummaryDataLock);
				EnterCriticalSection(&g_csKinectSummaryDataLock);
		//		g_pSensorSummary->nLeftRearZone =			__min( g_pSensorSummary->nLeftRearZone, g_pKinectSummary->nLeftRearZone );	
		///		g_pSensorSummary->bLeftCliff =				g_pKinectSummary->bLeftCliff;
				g_pSensorSummary->nLeftSideZone =			__min( g_pSensorSummary->nLeftSideZone, g_pKinectSummary->nLeftSideZone );	 
				g_pSensorSummary->nLeftFrontSideZone =		__min( g_pSensorSummary->nLeftFrontSideZone, g_pKinectSummary->nLeftFrontSideZone );	 
				g_pSensorSummary->nLeftArmZone =			__min( g_pSensorSummary->nLeftArmZone, g_pKinectSummary->nLeftArmZone );	// Object in front of Arm
				g_pSensorSummary->nLeftFrontZone =			__min( g_pSensorSummary->nLeftFrontZone, g_pKinectSummary->nLeftFrontZone );
				g_pSensorSummary->nRightFrontZone =			__min( g_pSensorSummary->nRightFrontZone, g_pKinectSummary->nRightFrontZone );
				g_pSensorSummary->nRightArmZone =			__min( g_pSensorSummary->nRightArmZone, g_pKinectSummary->nRightArmZone ); // Object in front of Arm
				g_pSensorSummary->nRightFrontSideZone =		__min( g_pSensorSummary->nRightFrontSideZone, g_pKinectSummary->nRightFrontSideZone );
				g_pSensorSummary->nRightSideZone =			__min( g_pSensorSummary->nRightSideZone, g_pKinectSummary->nRightSideZone );
		///		g_pSensorSummary->bRightCliff =				g_pKinectSummary->bRightCliff;
		//		g_pSensorSummary->nRightRearZone =			__min( g_pSensorSummary->nRightRearZone, g_pKinectSummary->nRightRearZone );	
				/*
				g_pKinectSummary->RobotLocation.x = 0;
				g_pKinectSummary->RobotLocation.y = 0;
				g_pKinectSummary->CompassHeading = 0;
				g_pKinectSummary->SampleTimeStamp = 0;
				*/
			}
			else
			{
				//ROBOT_LOG(TRUE, "ERROR: Last Kinect Scan more than 1 second ago! (and value not zero)\n" )
			}
		}
		LeaveCriticalSection(&g_csKinectSummaryDataLock);
		__itt_task_end(pDomainControlThread);


		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get first level summary data for each sensor.  May be overridden by bumpers later


		// Check Bumpers
/*		if( HW_BUMPER_HIT_FRONT )
		{		
			// Only one front HW bumper on Loki
			g_pSensorSummary->nLeftFrontZone = 0; // Collision!
			g_pSensorSummary->nRightFrontZone = 0; // Collision!
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Front Bumper!\n" )
			SpeakText( "Sorry" );	
		}
*/

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now, continue combining sensor info for front sensors

		// Start with analog sensors, then digital.  Look for closest object on each front area (left and right).

		g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftFrontZone ); // Front IR, Laser, and bumpers
		g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftArmZone );

		g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightFrontZone );
		g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightArmZone );

		if( abs((int)(g_pSensorSummary->nClosestObjectFrontRight - g_pSensorSummary->nClosestObjectFrontLeft) ) < IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
		{
			// within error margin.  Assume objects straight ahead
			g_pSensorSummary->nFrontObjectDistance = (g_pSensorSummary->nClosestObjectFrontRight + g_pSensorSummary->nClosestObjectFrontLeft) /2;
			g_pSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;	// Straight Ahead
		}
		else if( g_pSensorSummary->nClosestObjectFrontRight < g_pSensorSummary->nClosestObjectFrontLeft )
		{
			// Closest Object to the Right
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontRight;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
		}
		else
		{
			// Closest Object to the Left
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontLeft;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
		}

		// Display final result if debugging
	/*	if( g_pSensorSummary->nFrontObjectDistance < 18 ) // Test distance
		{
			#if REPORT_CLOSE_OBJECTS = 1
				CString strSensor;
				strSensor.Format( "SensorModule: FrontObjectDist = %d,  FrontObjectDir = %d, Closest FrontLeft = %d FrontRight = %d",
					g_pSensorSummary->nFrontObjectDistance, g_pSensorSummary->nFrontObjectDirection,
					g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nClosestObjectFrontRight );
				ROBOT_DISPLAY( TRUE, ((LPCTSTR)strSensor) )
			#endif

		}
	*/

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// REAR SENSORS
		// CLIFF SENSOR

		// Loki has 2 IR Cliff Sensors
		/*		
		if( m_CliffSensorsEnabled )
		{
			if( IR_BUMPER_CLIFF_LEFT || IR_BUMPER_CLIFF_RIGHT )
			{
				//SpeakCannedPhrase( SPEAK_COLLISION );

				if( IR_BUMPER_CLIFF_LEFT && IR_BUMPER_CLIFF_RIGHT )
				{
					// Cliff on both sides  Oh no!  what to do?
					g_pSensorSummary->bLeftCliff = TRUE;
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: ERROR!!!! Cliff on Both Sides!!!\n" )
					SpeakText( "Error. Cliff on both sides" );	

				}
				else if( IR_BUMPER_CLIFF_LEFT )
				{		
					g_pSensorSummary->bLeftCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Left Side!\n" )
					SpeakText( "Cliff Left" );	


				}
				else if( IR_BUMPER_CLIFF_RIGHT )
				{		
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Right Side!\n" )
					SpeakText( "Cliff Right" );	

				}
				else
				{
					// Logic Error
					ROBOT_ASSERT(0);
				}
			}
		}
		*/
		// SIDE SENSORS
		// Loki has IR side sensors and side bumpers

		// Handle analog sensors first
		//g_pSensorSummary->nLeftSideZone  = __min( g_pSensorSummary->nLeftSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_LEFT]);
		//g_pSensorSummary->nRightSideZone  = __min( g_pSensorSummary->nRightSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_RIGHT]);

		/*
		// Then bumpers
		if( HW_BUMPER_HIT_SIDE_LEFT )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
			g_pSensorSummary->nLeftSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Left Side Bumper!\n" )
			SpeakText( "Oops" );	
		}
		if( HW_BUMPER_HIT_SIDE_RIGHT )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
			g_pSensorSummary->nRightSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Right Side Bumper!\n" )
			SpeakText( "Darn" );	
		}
		*/

		// Now, sumarize

		if( g_pSensorSummary->nLeftSideZone == g_pSensorSummary->nRightSideZone )
		{
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = OBJECT_EQUAL_DISTANCE;
		}
		else if( g_pSensorSummary->nLeftSideZone < g_pSensorSummary->nRightSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
		}
		else if( g_pSensorSummary->nRightSideZone < g_pSensorSummary->nLeftSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nRightSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
		}
		else
		{
			ROBOT_ASSERT(0); // Logic Error
		}


	///////////////////////////////////////////////////////////////////////////////
	#elif ( SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP )

		// CString strSensor;

		// Initialize SENSOR_SUMMARY_T
		// Note, if sensor not installed, use SENSOR_DISABLED
		g_pSensorSummary->nFrontObjectDistance =	NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nFrontObjectDirection =	OBJECT_EQUAL_DISTANCE;	// Degrees from front (left is negative)
		g_pSensorSummary->nSideObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nSideObjectDirection =	OBJECT_EQUAL_DISTANCE;
		g_pSensorSummary->nRearObjectDistance =		NO_OBJECT_IN_RANGE;
	//	g_pSensorSummary->nRearObjectDirection =	OBJECT_EQUAL_DISTANCE;

		g_pSensorSummary->nLeftRearZone =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bLeftCliff =				FALSE;
		g_pSensorSummary->nObjectClawLeft =			NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nObjectArmLeft =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nLeftSideZone =			NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftFrontSideZone =		NO_OBJECT_IN_RANGE;	// 
		g_pSensorSummary->nLeftArmZone =			NO_OBJECT_IN_RANGE;	// Object in front of Arm
		g_pSensorSummary->nLeftFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->MotionDetectedDirection =	MOTION_DETECTED_NONE;	//////// Center
		g_pSensorSummary->bFrontCliff =				FALSE;
		g_pSensorSummary->nRightFrontZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightArmZone =			NO_OBJECT_IN_RANGE; // Object in front of Arm
		g_pSensorSummary->nRightFrontSideZone =		NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nRightSideZone =			NO_OBJECT_IN_RANGE;
		g_pSensorSummary->nObjectArmRight =			NO_OBJECT_IN_RANGE;	// compensated for distance in front of robot
		g_pSensorSummary->nObjectClawRight =		NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->bRightCliff =				FALSE;
		g_pSensorSummary->nRightRearZone =			NO_OBJECT_IN_RANGE;	

		g_pSensorSummary->nClosestObjectFrontLeft =	NO_OBJECT_IN_RANGE;	
		g_pSensorSummary->nClosestObjectFrontRight= NO_OBJECT_IN_RANGE;

			
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get Kinect sensor data, if the Kinect exists.  May be overridden by other sensors later
		// Note that Kinect data is in tenth inches, but other sensor are in inches (due to low resoluton)
		// For now, only inches used for object avoidance and colision.  Maybe change to tenthinches or float later?

		if( 0 != g_pKinectSummary->SampleTimeStamp )	 // Skip this if sensor not installed
		{
			int TimeSinceLastKinectScan =  GetTickCount() - g_pKinectSummary->SampleTimeStamp;
			if( TimeSinceLastKinectScan < 1000 )
			{	
				// Last scan was within 1 second
				__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csKinectSummaryDataLock);
				EnterCriticalSection(&g_csKinectSummaryDataLock);
		// Teleop has no arms!
		//		g_pSensorSummary->nLeftRearZone =			__min( g_pSensorSummary->nLeftRearZone, g_pKinectSummary->nLeftRearZone );	
		///		g_pSensorSummary->bLeftCliff =				g_pKinectSummary->bLeftCliff;
				g_pSensorSummary->nLeftSideZone =			__min( g_pSensorSummary->nLeftSideZone, g_pKinectSummary->nLeftSideZone );	 
				g_pSensorSummary->nLeftFrontSideZone =		__min( g_pSensorSummary->nLeftFrontSideZone, g_pKinectSummary->nLeftFrontSideZone );	 
		//		g_pSensorSummary->nLeftArmZone =			__min( g_pSensorSummary->nLeftArmZone, g_pKinectSummary->nLeftArmZone );	// Object in front of Arm
				g_pSensorSummary->nLeftFrontZone =			__min( g_pSensorSummary->nLeftFrontZone, g_pKinectSummary->nLeftFrontZone );
				g_pSensorSummary->nRightFrontZone =			__min( g_pSensorSummary->nRightFrontZone, g_pKinectSummary->nRightFrontZone );
		//		g_pSensorSummary->nRightArmZone =			__min( g_pSensorSummary->nRightArmZone, g_pKinectSummary->nRightArmZone ); // Object in front of Arm
				g_pSensorSummary->nRightFrontSideZone =		__min( g_pSensorSummary->nRightFrontSideZone, g_pKinectSummary->nRightFrontSideZone );
				g_pSensorSummary->nRightSideZone =			__min( g_pSensorSummary->nRightSideZone, g_pKinectSummary->nRightSideZone );
		///		g_pSensorSummary->bRightCliff =				g_pKinectSummary->bRightCliff;
		//		g_pSensorSummary->nRightRearZone =			__min( g_pSensorSummary->nRightRearZone, g_pKinectSummary->nRightRearZone );	
				/*
				g_pKinectSummary->RobotLocation.x = 0;
				g_pKinectSummary->RobotLocation.y = 0;
				g_pKinectSummary->CompassHeading = 0;
				g_pKinectSummary->SampleTimeStamp = 0;
				*/
			}
			else
			{
				//ROBOT_LOG(TRUE, "ERROR: Last Kinect Scan more than 1 second ago! (and value not zero)\n" )
			}
		}
		LeaveCriticalSection(&g_csKinectSummaryDataLock);
		__itt_task_end(pDomainControlThread);


		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Get first level summary data for each sensor.  May be overridden by bumpers later


		// Check Bumpers
		if( HW_BUMPER_HIT_FRONT )
		{		
			g_pSensorSummary->nLeftFrontZone = 0; // Collision!
			g_pSensorSummary->nRightFrontZone = 0; // Collision!
		}
		else
		{
			if( HW_BUMPER_HIT_SIDE_LEFT )
				g_pSensorSummary->nLeftFrontZone = 0; // Collision!
			if( HW_BUMPER_HIT_SIDE_RIGHT )
				g_pSensorSummary->nRightFrontZone = 0; // Collision!
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now, continue combining sensor info for front sensors

		// Start with analog sensors, then digital.  Look for closest object on each front area (left and right).

		g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftFrontZone ); // Front IR, Laser, and bumpers
		// g_pSensorSummary->nClosestObjectFrontLeft = __min( g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nLeftArmZone );

		g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightFrontZone );
		// g_pSensorSummary->nClosestObjectFrontRight = __min( g_pSensorSummary->nClosestObjectFrontRight, g_pSensorSummary->nRightArmZone );

		if( abs((int)(g_pSensorSummary->nClosestObjectFrontRight - g_pSensorSummary->nClosestObjectFrontLeft) ) < IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
		{
			// within error margin.  Assume objects straight ahead
			g_pSensorSummary->nFrontObjectDistance = (g_pSensorSummary->nClosestObjectFrontRight + g_pSensorSummary->nClosestObjectFrontLeft) /2;
			g_pSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;	// Straight Ahead
		}
		else if( g_pSensorSummary->nClosestObjectFrontRight < g_pSensorSummary->nClosestObjectFrontLeft )
		{
			// Closest Object to the Right
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontRight;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
		}
		else
		{
			// Closest Object to the Left
			g_pSensorSummary->nFrontObjectDistance = g_pSensorSummary->nClosestObjectFrontLeft;
			g_pSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
		}

		// Display final result if debugging
	/*	if( g_pSensorSummary->nFrontObjectDistance < 18 ) // Test distance
		{
			#if REPORT_CLOSE_OBJECTS = 1
				CString strSensor;
				strSensor.Format( "SensorModule: FrontObjectDist = %d,  FrontObjectDir = %d, Closest FrontLeft = %d FrontRight = %d",
					g_pSensorSummary->nFrontObjectDistance, g_pSensorSummary->nFrontObjectDirection,
					g_pSensorSummary->nClosestObjectFrontLeft, g_pSensorSummary->nClosestObjectFrontRight );
				ROBOT_DISPLAY( TRUE, ((LPCTSTR)strSensor) )
			#endif

		}
	*/

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// REAR SENSORS
		// CLIFF SENSOR

		// Kobuki base has 3 IR Cliff Sensors, only 2 handled - TODO-MUST
		if( m_CliffSensorsEnabled )
		{
			if( IR_BUMPER_CLIFF_LEFT || IR_BUMPER_CLIFF_RIGHT )
			{
				//SpeakCannedPhrase( SPEAK_COLLISION );

				if( IR_BUMPER_CLIFF_LEFT && IR_BUMPER_CLIFF_RIGHT )
				{
					// Cliff on both sides  Oh no!  what to do?
					g_pSensorSummary->bLeftCliff = TRUE;
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff on Both Sides!!!\n" )
					//SpeakText( "Error. Cliff on both sides" );	

				}
				else if( IR_BUMPER_CLIFF_LEFT )
				{		
					g_pSensorSummary->bLeftCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Left Side!\n" )
					//SpeakText( "Cliff Left" );	
				}
				else if( IR_BUMPER_CLIFF_RIGHT )
				{		
					g_pSensorSummary->bRightCliff = TRUE;
					ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Right Side!\n" )
					//SpeakText( "Cliff Right" );	
				}
				else
				{
					// Logic Error
					ROBOT_ASSERT(0);
				}
			}
		}

		bool RightDrop = g_SensorStatus.WheelDrop && 0x01;
		bool LeftDrop = g_SensorStatus.WheelDrop && 0x02;


		// Kobuki also has wheel drop sensors - these always work
		if ( 0 != g_SensorStatus.WheelDrop )
		{
			if( RightDrop && LeftDrop )
			{
				// Cliff on both sides
				g_pSensorSummary->bLeftCliff = TRUE;
				g_pSensorSummary->bRightCliff = TRUE;
				ROBOT_DISPLAY( TRUE, "SensorModule: Cliff on Both Sides!!!\n" )
				//SpeakText( "Error. Cliff on both sides" );	

			}
			else if( LeftDrop )
			{		
				g_pSensorSummary->bLeftCliff = TRUE;
				ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Left Side!\n" )
				//SpeakText( "Cliff Left" );	
			}
			else if( RightDrop )
			{		
				g_pSensorSummary->bRightCliff = TRUE;
				ROBOT_DISPLAY( TRUE, "SensorModule: Cliff Right Side!\n" )
				//SpeakText( "Cliff Right" );	
			}
			else
			{
				// Logic Error
				ROBOT_ASSERT(0);
			}

		}


		// SIDE SENSORS TODO-MUST-TELEOP
		// Loki has IR side sensors and side bumpers

		// Handle analog sensors first
		//g_pSensorSummary->nLeftSideZone  = __min( g_pSensorSummary->nLeftSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_LEFT]);
		//g_pSensorSummary->nRightSideZone  = __min( g_pSensorSummary->nRightSideZone,  g_SensorStatus.IR[IR_SENSOR_SIDE_RIGHT]);

		/*
		// Then bumpers
		if( HW_BUMPER_HIT_SIDE_LEFT )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
			g_pSensorSummary->nLeftSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Left Side Bumper!\n" )
			SpeakText( "Oops" );	
		}
		if( HW_BUMPER_HIT_SIDE_RIGHT )
		{		
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
			g_pSensorSummary->nRightSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Right Side Bumper!\n" )
			SpeakText( "Darn" );	
		}
		*/

		// Now, sumarize

		if( g_pSensorSummary->nLeftSideZone == g_pSensorSummary->nRightSideZone )
		{
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = OBJECT_EQUAL_DISTANCE;
		}
		else if( g_pSensorSummary->nLeftSideZone < g_pSensorSummary->nRightSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nLeftSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_LEFT;
		}
		else if( g_pSensorSummary->nRightSideZone < g_pSensorSummary->nLeftSideZone )
		{		
			g_pSensorSummary->nSideObjectDistance = g_pSensorSummary->nRightSideZone;
			g_pSensorSummary->nSideObjectDirection = SIDE_RIGHT;
		}
		else
		{
			ROBOT_ASSERT(0); // Logic Error
		}



	///////////////////////////////////////////////////////////////////////////////
	#else
		#error BAD SENSOR_CONFIG_TYPE!  
	#endif
	///////////////////////////////////////////////////////////////////////////////


}

#endif // ROBOT_SERVER	// This module used for Robot Server only

