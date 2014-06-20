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
	g_pFullSensorStatus->OdometerUpdateTenthInches = (OdometerUpdateTenthInchesL+OdometerUpdateTenthInchesR) / 2.0;

	// Update the system Odometer with average of the two wheels
	g_pFullSensorStatus->OdometerTenthInches += g_pFullSensorStatus->OdometerUpdateTenthInches;

		
	// Update the "Tach" display.
	g_pFullSensorStatus->Tachometer = (int)(g_pFullSensorStatus->OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

				ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
					OdometerUpdateTenthInchesL, OdometerUpdateTenthInchesR, g_pFullSensorStatus->Tachometer, g_pFullSensorStatus->OdometerUpdateTenthInches )
		
	// Update MoveDistance counter, in case a programmed move is in progress
	m_pDriveCtrl->UpdateMoveDistance( g_pFullSensorStatus->OdometerUpdateTenthInches );

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
	if( -1 == g_pFullSensorStatus->CalculatedMotorHeading )
	{
		// Heading not initialized
		// Wait for first motor movement, then grab the compass heading to initialize direction
		if( 0 != g_pFullSensorStatus->OdometerUpdateTenthInches )
		{
			g_pFullSensorStatus->CalculatedMotorHeading = m_LastCompassHeading;
		}
	}
	else
	{
		g_pFullSensorStatus->CalculatedMotorHeading += TurnAngleUpdate;
		// TODO - see if "drifting" occurs due to round off error.  If so, keep angle to higher precision?
		//	ROBOT_LOG( TRUE,  "========> Turn %0.3f --> Calculated MOTOR HEADING: %0.2f\n", TurnAngleUpdate, g_pFullSensorStatus->CalculatedMotorHeading)
	}


	UpdateLocation();	// Update the internal view of where we are in the world!

}


void CSensorModule::ProcessMessage( 
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{

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
			g_pFullSensorStatus->CurrentLocation.x = (int)wParam;	// Get new Map absolute X,Y
			g_pFullSensorStatus->CurrentLocation.y = (int)lParam;
			g_pFullSensorStatus->CurrentLocationMotor.x = (int)wParam;	// Get new Map absolute X,Y
			g_pFullSensorStatus->CurrentLocationMotor.y = (int)lParam;
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


		#if( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
		case WM_ROBOT_KOBUKI_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ProcessKobukiStatus()
			// This is where Raw sensor data from Kobuki base gets repackaged and copied to g_pFullSensorStatus
			// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			ProcessKobukiStatus( ); 
		}
		break;
		#endif

		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			g_bCmdRecognized = TRUE;

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ProcessSensorStatus()
			// This is where Raw sensor data from Arduino gets repackaged and copied to g_pFullSensorStatus
			// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			ProcessSensorStatus( ); 
		}
		break;

		case WM_ROBOT_ER1_ODOMETER_READY:
		{
			g_bCmdRecognized = TRUE;
			// When using the ER1 Pilot controller, move distance and speed are reported
			// by the motor controller, not the Arduino.
			// wParam = LEFT_MOTOR, lParam = RIGHT_MOTOR

			//////////////////////////////////////////////////////////////////////////
			// ODOMETER
			// g_pFullSensorStatus->Odometer is the distance since the last Odometer RESET
			// g_pFullSensorStatus->OdometerUpdate is the distance moved since the last update
			// Note, The odometer value could be positive or negative, depending if we are going FWD or REV. from the last RESET
			// RAW ER1 Status is in TICKS, Laptop PicStatus is in TenthInches

			#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL

				// ER1 is used for Odometry instead of Arduino
				
				// Unlike the Arduino, the ER1 keeps a running distance total
				long LastOdometer = g_pFullSensorStatus->Odometer;
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
			g_pFullSensorStatus->CurrentLocationGPS.x = Pos.x;
			g_pFullSensorStatus->CurrentLocationGPS.x = Pos.y;

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
				double DeltaX = (double)m_GPSLastSample.x - g_pFullSensorStatus->CurrentLocationGPS.x;
				double DeltaY = (double)m_GPSLastSample.y - g_pFullSensorStatus->CurrentLocationGPS.y;
				int  DistanceFromLastGPS = (int )(sqrt( (double)(DeltaX*DeltaX) + (double)(DeltaY*DeltaY) ));

				if( (DistanceFromLastGPS > 36) &&	// min number of inches between samples //Convert to TenthInches???
					(g_pFullSensorStatus->OdometerUpdateTenthInches > 5) )	// Make sure robot is actually moving!
				{
					// New GPS reading.  Let's process it!
					// Get direction of travel between current and last stored sample
					int  CurrentAngle = CalculateAngle( m_GPSLastSample, FPointToPoint(g_pFullSensorStatus->CurrentLocationGPS) );
					m_GPSLastSample = FPointToPoint(g_pFullSensorStatus->CurrentLocationGPS);	// Save current position for next time

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
						g_pFullSensorStatus->CurrentLocationGPS.x = 0;


						DeltaX = g_pFullSensorStatus->CurrentLocation.x - g_pFullSensorStatus->CurrentLocationGPS.x;
						DeltaY = g_pFullSensorStatus->CurrentLocation.y - g_pFullSensorStatus->CurrentLocationGPS.y;
						int  DistanceFromGPSCenter = (int )sqrt( (double)(DeltaX*DeltaX) + (double)(DeltaY*DeltaY) );

						if( DistanceFromGPSCenter > GPS_ERROR_AMOUNT )
						{
							// outside the GPS boundry.  Adjust RW coordinate to "move" robot to edge of GPS Error zone

		// /**** COMPLEX (BUT BETTER) METHOD:

							// Given GPS Center plus distance (GPS Error) and direction to current RW coordinates, 
							// calculate new current X,Y.
							double X, Y;	// Delta from Start Waypoint
							//int MapX, MapY;	// Absolute coordinates, allows for negative numbers

							int  AngleFromGPStoCurrent = CalculateAngle( g_pFullSensorStatus->CurrentLocationGPS, g_pFullSensorStatus->CurrentLocation );

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
							g_pFullSensorStatus->CurrentLocation.x = g_pFullSensorStatus->CurrentLocationGPS.x + X;		// Get new Map absolute X,Y
							g_pFullSensorStatus->CurrentLocation.x = g_pFullSensorStatus->CurrentLocationGPS.y + Y;

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
	// g_pFullSensorStatus->OdometerUpdate holds distance, in (double)inches, since last reading.

	///////////////////////////////////////////////////////////////////////////////
	// First, estimate current location based upon "dead reckoning"
	// using compass and Odometer

	//POINT LastLocation = g_pFullSensorStatus->CurrentLocation;
	if( g_pFullSensorStatus->CompassHeading <= 360 )
	{
		// Good compass value.  Save the heading in case the next time we get a bad value
		m_LastCompassHeading = g_pFullSensorStatus->CompassHeading;
	}
	// Note:  If the compass heading is bad, we ASSUME the heading has not changed significantly 
	// since the last good heading, so we calculate position based upon that.
	// TODO-ER1 - Use MotorHeading to fill in for compass? (see below)

	// Convert Degrees to Radians then do the math
	double DirectionRadians = (double)m_LastCompassHeading * DEGREES_TO_RADIANS;
	double X = g_pFullSensorStatus->OdometerUpdateTenthInches * sin( DirectionRadians );	// Returns negative numbers as needed
	double Y = g_pFullSensorStatus->OdometerUpdateTenthInches * cos( DirectionRadians );
	
	// Not needed - Now stored as high precision data
//	if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
//	if(Y>=0) Y += 0.5; else Y -= 0.5;
	
	g_pFullSensorStatus->CurrentLocation.x += X;		// Get new Map absolute X,Y TENTHINCHES
	g_pFullSensorStatus->CurrentLocation.y += Y;

	///////////////////////////////////////////////////////////////////////////////
	// Now, for ER1, estimate current location based upon "dead reckoning"
	// using odometer differences between motors (differential drive)

	//POINT LastLocation = g_pFullSensorStatus->CurrentLocation;
//		m_LastMotorHeading = g_pFullSensorStatus->CompassHeading;
/********* TODO
	if( -1 != g_pFullSensorStatus->CalculatedMotorHeading )	// Make sure Heading is initialized
	{
		// Convert Degrees to Radians then do the math
		double DirectionRadians = g_pFullSensorStatus->CalculatedMotorHeading * DEGREES_TO_RADIANS;
		double X = g_pFullSensorStatus->OdometerUpdate * sin( DirectionRadians );	// Returns negative numbers as needed
		double Y = g_pFullSensorStatus->OdometerUpdate * cos( DirectionRadians );
		
//		if(X>=0) X += 0.5; else X -= 0.5;	// Cast will truncate, this will round instead
//		if(Y>=0) Y += 0.5; else Y -= 0.5;
		g_pFullSensorStatus->CurrentLocationMotor.x += X;		// Get new Map absolute X,Y
		g_pFullSensorStatus->CurrentLocationMotor.y += Y;
	}

***********/

/*	ROBOT_LOG( TRUE,  "DEBUG POSITION: Compass=%d,%d,  Motor=%d,%d\n", 
		(int)g_pFullSensorStatus->CurrentLocation.x, (int)g_pFullSensorStatus->CurrentLocation.y,
		(int)g_pFullSensorStatus->CurrentLocationMotor.x, (int)g_pFullSensorStatus->CurrentLocationMotor.y )

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
				MAX_IR_OFFSET_DISTANCE, SensorOffsetDegrees_IR[SensorNumber], g_pFullSensorStatus->IR[SensorNumber],	// IN
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
				g_pFullSensorStatus->CurrentLocation.x = (int) (g_pFullSensorStatus->CurrentLocation.x + CorrectionNeeded.x );
				g_pFullSensorStatus->CurrentLocation.y = (int) (g_pFullSensorStatus->CurrentLocation.y + CorrectionNeeded.y );
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

	double SensorDirection = g_pFullSensorStatus->CompassHeading + SensorOffsetDegrees;
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

	PhantomObjectPt.x = (int)g_pFullSensorStatus->CurrentLocation.x + (int)dX; // Add delta to current Robot position
	PhantomObjectPt.y = (int)g_pFullSensorStatus->CurrentLocation.y + (int)dY;


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
//		g_pNavSensorSummary->nClosestRadarObjectDistance = NearestObjectRange;
//		g_pNavSensorSummary->nClosestRadarObjectLocation = ObjectLeft + (ObjectWidth/2); 
//		g_pNavSensorSummary->nClosestRadarObjectLeft = ObjectLeft;
//		g_pNavSensorSummary->nClosestRadarObjectRight = ObjectRight;
	
	
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
		if( (g_pFullSensorStatus->IR[IR_NumberLeft]  < IR_SensorRange) ||	
			(g_pFullSensorStatus->IR[IR_NumberRight] < IR_SensorRange) )
		{
			// Within Range of at least one of the IR sensors.  Use IR instead of US.
			if( abs((int)(g_pFullSensorStatus->IR[IR_NumberRight] - g_pFullSensorStatus->IR[IR_NumberLeft]) ) > IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
			{
				// object clearly to one side or the other
				ObjectDirection = ( g_pFullSensorStatus->IR[IR_NumberRight] < g_pFullSensorStatus->IR[IR_NumberLeft] ) 
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


#endif // ROBOT_SERVER	// This module used for Robot Server only
