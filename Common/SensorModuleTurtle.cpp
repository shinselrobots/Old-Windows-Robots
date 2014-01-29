// SensorModuleLoki.cpp: SensorModule Loki specific functions 
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include <math.h>
#include "Globals.h"
#include "module.h"
//#include "thread.h"
#include "HardwareConfig.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define BUMPER_DEBUG_ENABLED
#define ENABLE_BUMPERS 1 // Enable or disable reacting to bumper hits.  Disable if debugging or hardware problem


///////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ProcessSensorStatus()
	// This is where Raw sensor data from Arduino or other sources (Like Kobuki base) get repackaged and copied to g_SensorStatus
	// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
	void CSensorModule::ProcessSensorStatus()
	{
		// See MotorControlIRobot.cpp for capabilities of the iRobot Create base

		// First thing is to initialize the processed Sensor Status block
		// Don't memset the structure, it corrupts current position!

		// Basic Data
		g_pFullSensorStatus->StatusFlags =	0;	// not used for Kobuki
		g_pFullSensorStatus->LastError =	0;
		g_pFullSensorStatus->DebugCode =	0;

		// Battery in the iRobot base
		g_pFullSensorStatus->Battery0 = g_pIRobotStatus->BatteryVoltage;
		// Not used yet: g_pFullSensorStatus->Battery1 = 0;	

		// Cliff and Wheel drops
		g_pFullSensorStatus->WheelDropLeft =	g_pIRobotStatus->WheelDropLeft;
		g_pFullSensorStatus->WheelDropRight =	g_pIRobotStatus->WheelDropRight;

		// Kobuki has 3 IR Cliff Sensors,  left, right, front
		if( m_CliffSensorsEnabled )
		{
			// iRobot Create has 2 cliff sensors in front, plus one on each side.  
			g_pFullSensorStatus->CliffFront =	g_pIRobotStatus->CliffFrontLeft || g_pIRobotStatus->CliffFrontRight;
			g_pFullSensorStatus->CliffLeft =	g_pIRobotStatus->CliffLeft;
			g_pFullSensorStatus->CliffRight =	g_pIRobotStatus->CliffRight;
		}
		else
		{
			g_pFullSensorStatus->CliffFront =	false;
			g_pFullSensorStatus->CliffRight =	false;
			g_pFullSensorStatus->CliffLeft =	false;
		}

		// Hardware Bumpers - iRobot only has right/left front		
		g_pFullSensorStatus->HWBumperFront =		g_pIRobotStatus->BumperLeft && g_pIRobotStatus->BumperRight; // both hit at same time
		g_pFullSensorStatus->HWBumperSideLeft =		g_pIRobotStatus->BumperLeft;
		g_pFullSensorStatus->IRBumperSideRight =	g_pIRobotStatus->BumperRight;
		g_pFullSensorStatus->HWBumperRear =			0; // no rear bumper

		// IR Bumpers - No IR Bumpers
		// Arm and finger sensors - no arms

		// Compass - Heading is in degrees
		// WARNING - THIS IS NOT REALLY COMPASS HEADING!
		// IT IS DEGREES SINCE LAST REQUEST (I THINK?)
		// (but still useful for determining turn amount)
		// TODO! g_pFullSensorStatus->CompassHeading = g_pIRobotStatus->Angle ); // in degrees, since last request


		// Heading and Odometry
		// Calculations are handled in this funciton, which also calls UpdateMoveDistance, UpdateLocation, etc.
		UpdateOdometer();

		// iRobot Create Dock IR Beacon
		// TODO:  use g_pIRobotStatus->IRcode
		// g_pFullSensorStatus->DockSensorRight =	g_pKobukiStatus->DockRightSignal;
		// g_pFullSensorStatus->DockSensorCenter = g_pKobukiStatus->DockCenterSignal;
		// g_pFullSensorStatus->DockSensorLeft =	g_pKobukiStatus->DockLeftSignal;

		// Get status of Android Phone bluetooth connection, and get commands
		// HandleAndroidPhone();  // requires Arduino with Bluetooth

		// Other Sensors and state

		// No tilt accelerometer on Loki
		//g_pFullSensorStatus->TiltAccelX =	0;			// Typically From Arduino.  zero = level
		//g_pFullSensorStatus->TiltAccelY =	0;	 		// Typically From Arduino.  zero = level

		// Analog Sensors - none installed
		// HandleAnalogSensors();


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

	}  // ProcessSensorStatus




	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR FUSION
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void CSensorModule::DoSensorFusion()
	{
		// Combine any redundant sensors, and summarize results into g_pNavSensorSummary

		// Angle sensors use local variable, not in SensorSummary
		int  nAngleObjectDistance = NO_OBJECT_IN_RANGE;
		int nAngleObjectDirection = 0;

		g_pNavSensorSummary->InitializeDefaults(); // Note, if sensor not installed, override with SENSOR_DISABLED
		g_pNavSensorSummary->bHWBumperFront = g_pFullSensorStatus->HWBumperFront; // for convenience make bumper available in summary
			
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
		//		g_pNavSensorSummary->nLeftRearZone =			__min( g_pNavSensorSummary->nLeftRearZone, g_pKinectSummary->nLeftRearZone );	
		///		g_pNavSensorSummary->bLeftCliff =				g_pKinectSummary->bLeftCliff;
				g_pNavSensorSummary->nLeftSideZone =			__min( g_pNavSensorSummary->nLeftSideZone, g_pKinectSummary->nLeftSideZone );	 
				g_pNavSensorSummary->nLeftFrontSideZone =		__min( g_pNavSensorSummary->nLeftFrontSideZone, g_pKinectSummary->nLeftFrontSideZone );	 
		//		g_pNavSensorSummary->nLeftArmZone =			__min( g_pNavSensorSummary->nLeftArmZone, g_pKinectSummary->nLeftArmZone );	// Object in front of Arm
				g_pNavSensorSummary->nLeftFrontZone =			__min( g_pNavSensorSummary->nLeftFrontZone, g_pKinectSummary->nLeftFrontZone );
				g_pNavSensorSummary->nRightFrontZone =			__min( g_pNavSensorSummary->nRightFrontZone, g_pKinectSummary->nRightFrontZone );
		//		g_pNavSensorSummary->nRightArmZone =			__min( g_pNavSensorSummary->nRightArmZone, g_pKinectSummary->nRightArmZone ); // Object in front of Arm
				g_pNavSensorSummary->nRightFrontSideZone =		__min( g_pNavSensorSummary->nRightFrontSideZone, g_pKinectSummary->nRightFrontSideZone );
				g_pNavSensorSummary->nRightSideZone =			__min( g_pNavSensorSummary->nRightSideZone, g_pKinectSummary->nRightSideZone );
		///		g_pNavSensorSummary->bRightCliff =				g_pKinectSummary->bRightCliff;
		//		g_pNavSensorSummary->nRightRearZone =			__min( g_pNavSensorSummary->nRightRearZone, g_pKinectSummary->nRightRearZone );	
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
		if( g_pFullSensorStatus->HWBumperFront )
		{		
			g_pNavSensorSummary->nLeftFrontZone = 0; // Collision!
			g_pNavSensorSummary->nRightFrontZone = 0; // Collision!
		}
		else
		{
			if( g_pFullSensorStatus->HWBumperSideLeft )
				g_pNavSensorSummary->nLeftFrontZone = 0; // Collision!
			if( g_pFullSensorStatus->HWBumperSideRight )
				g_pNavSensorSummary->nRightFrontZone = 0; // Collision!
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now, continue combining sensor info for front sensors

		// Start with analog sensors, then digital.  Look for closest object on each front area (left and right).

		g_pNavSensorSummary->nClosestObjectFrontLeft = __min( g_pNavSensorSummary->nClosestObjectFrontLeft, g_pNavSensorSummary->nLeftFrontZone ); // Front IR, Laser, and bumpers
		// g_pNavSensorSummary->nClosestObjectFrontLeft = __min( g_pNavSensorSummary->nClosestObjectFrontLeft, g_pNavSensorSummary->nLeftArmZone );

		g_pNavSensorSummary->nClosestObjectFrontRight = __min( g_pNavSensorSummary->nClosestObjectFrontRight, g_pNavSensorSummary->nRightFrontZone );
		// g_pNavSensorSummary->nClosestObjectFrontRight = __min( g_pNavSensorSummary->nClosestObjectFrontRight, g_pNavSensorSummary->nRightArmZone );

		if( abs((int)(g_pNavSensorSummary->nClosestObjectFrontRight - g_pNavSensorSummary->nClosestObjectFrontLeft) ) < IR_RANGE_FUDGE_AMOUNT_TENTH_INCHES )
		{
			// within error margin.  Assume objects straight ahead
			g_pNavSensorSummary->nFrontObjectDistance = (g_pNavSensorSummary->nClosestObjectFrontRight + g_pNavSensorSummary->nClosestObjectFrontLeft) /2;
			g_pNavSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;	// Straight Ahead
		}
		else if( g_pNavSensorSummary->nClosestObjectFrontRight < g_pNavSensorSummary->nClosestObjectFrontLeft )
		{
			// Closest Object to the Right
			g_pNavSensorSummary->nFrontObjectDistance = g_pNavSensorSummary->nClosestObjectFrontRight;
			g_pNavSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
		}
		else
		{
			// Closest Object to the Left
			g_pNavSensorSummary->nFrontObjectDistance = g_pNavSensorSummary->nClosestObjectFrontLeft;
			g_pNavSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
		}

		// Display final result if debugging
		/*	if( g_pNavSensorSummary->nFrontObjectDistance < 18 ) // Test distance
			{
				#if REPORT_CLOSE_OBJECTS = 1
					CString strSensor;
					strSensor.Format( "SensorModule: FrontObjectDist = %d,  FrontObjectDir = %d, Closest FrontLeft = %d FrontRight = %d",
						g_pNavSensorSummary->nFrontObjectDistance, g_pNavSensorSummary->nFrontObjectDirection,
						g_pNavSensorSummary->nClosestObjectFrontLeft, g_pNavSensorSummary->nClosestObjectFrontRight );
					ROBOT_DISPLAY( TRUE, ((LPCTSTR)strSensor) )
				#endif

			}
		*/

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// REAR SENSORS
		// CLIFF SENSOR

		// Kobuki base has 3 IR Cliff Sensors
		g_pNavSensorSummary->bCliffLeft =  g_pFullSensorStatus->CliffLeft;
		g_pNavSensorSummary->bCliffFront = g_pFullSensorStatus->CliffFront;
		g_pNavSensorSummary->bCliffRight = g_pFullSensorStatus->CliffRight;
		
		g_pNavSensorSummary->bWheelDropLeft =  g_pFullSensorStatus->WheelDropLeft;
		g_pNavSensorSummary->bWheelDropRight =  g_pFullSensorStatus->WheelDropRight;
		

		// Now, sumarize

		if( g_pNavSensorSummary->nLeftSideZone == g_pNavSensorSummary->nRightSideZone )
		{
			g_pNavSensorSummary->nSideObjectDistance = g_pNavSensorSummary->nLeftSideZone;
			g_pNavSensorSummary->nSideObjectDirection = OBJECT_EQUAL_DISTANCE;
		}
		else if( g_pNavSensorSummary->nLeftSideZone < g_pNavSensorSummary->nRightSideZone )
		{		
			g_pNavSensorSummary->nSideObjectDistance = g_pNavSensorSummary->nLeftSideZone;
			g_pNavSensorSummary->nSideObjectDirection = SIDE_LEFT;
		}
		else if( g_pNavSensorSummary->nRightSideZone < g_pNavSensorSummary->nLeftSideZone )
		{		
			g_pNavSensorSummary->nSideObjectDistance = g_pNavSensorSummary->nRightSideZone;
			g_pNavSensorSummary->nSideObjectDirection = SIDE_RIGHT;
		}
		else
		{
			ROBOT_ASSERT(0); // Logic Error
		}


	}	// DoSensorFusion

	//////////////////////////////////////////////////////////////////////////
	void CSensorModule::UpdateOdometer()
	{
		// g_pFullSensorStatus->Odometer is the distance since the last Odometer RESET
		// g_pFullSensorStatus->OdometerUpdate is the distance since the last Odometer READING
		// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
		// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)

		g_pFullSensorStatus->OdometerUpdateTenthInches = (double)g_pIRobotStatus->Distance / 2.540;	 // convert MM to TenthInches

		// Update the system Odometer with average of the two wheels
		g_pFullSensorStatus->OdometerTenthInches += g_pFullSensorStatus->OdometerUpdateTenthInches;

		
		// Update the "Tach" display.
		g_pFullSensorStatus->Tachometer = (int)(g_pFullSensorStatus->OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

		//		ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
		//			OdometerdoubleInchesL, OdometerdoubleInchesR, g_pFullSensorStatus->Odometer, g_pFullSensorStatus->OdometerUpdate )
		
		// Update MoveDistance counter, in case a programmed move is in progress
		m_pDriveCtrl->UpdateMoveDistance( g_pFullSensorStatus->OdometerUpdateTenthInches );

		double TurnAngleUpdate = g_pIRobotStatus->Angle;

		// Update TurnRotation counter, in case a programmed turn (rotate in place) is in progress
		m_pDriveCtrl->UpdateTurnRotation( TurnAngleUpdate );

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

#endif // SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP

#endif // ROBOT_SERVER	// This module used for Robot Server only