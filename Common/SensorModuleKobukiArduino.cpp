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

// Scale for A/D converter on Kobuki
const int  DistanceTable_IRWideAngleKobuki[] = 
//   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24     // Inches
{ 3000,3000,2500,1720,1340,1140, 950, 840, 740, 660, 600, 550, 500, 470, 430, 405, 375, 350, 320, 290, 275, 265, 250, 235, 220 };  // Reading


const int  DistanceTable_IRLongRangeKobuki[] =	
//feet 0		                                                   1		                                                   2
//in   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21   22	 23   24
  { 3500,3500,3500,3500,3440,3300,3320,3180,2933,2722,2540,2350,2180,2040,1900,1785,1680,1590,1445,1370,1310,1245,1195,1150,1100,
//feet 		                                                       3		                                                   4
//in       25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44   45   46   47   48
         1060,1055,1020, 975, 945, 920, 885, 860, 830, 815, 800, 765, 750, 740, 720, 705, 690, 680, 660, 650, 640, 625, 615, 600,
//feet 		                                                       5		                                                   6
//in       49   50   51   52   53   54   55   56   67   58   59   60   61   62   63   64   65   66   67   68   69   70   71   72
          580, 575, 570, 550, 545,  535, 530, 525, 515, 505, 500, 490, 480, 470, 460, 450, 440, 435, 425, 420, 415, 410, 405, 400 };






///////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_KOBUKI_WITH_ARDUINO

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ProcessSensorStatus()
	// This is where Raw sensor data from Arduino or other sources (Like Kobuki base) get repackaged and copied to g_pFullSensorStatus
	// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
	void CSensorModule::ProcessSensorStatus( UINT uMsg )
	{
		if( WM_ROBOT_SENSOR_STATUS_READY == uMsg )
		{
			// From Arduino
			// Get status of Android Phone bluetooth connection, and get commands
			HandleAndroidPhone();

		}
		else if( WM_ROBOT_KOBUKI_STATUS_READY == uMsg )
		{
			// From Kobuki Base
			static int LastRawIR = 0;

			// First thing is to initialize the processed Sensor Status block
			// Don't memset the structure, it corrupts current position!

			// Basic Data
			g_pFullSensorStatus->StatusFlags =	0;	// not used for Kobuki
			g_pFullSensorStatus->LastError =	0;
			g_pFullSensorStatus->DebugCode =	0;

			// Note that this is the current charge in % remaining, not Voltage!
			g_pFullSensorStatus->Battery0 = (int)g_pKobukiStatus->BatteryPercent;
			// Not used yet: g_pFullSensorStatus->Battery1 = 0;	

			// Cliff and Wheel drops
			g_pFullSensorStatus->WheelDropLeft =	g_pKobukiStatus->WheelDropLeft;
			g_pFullSensorStatus->WheelDropRight =	g_pKobukiStatus->WheelDropRight;

			// Kobuki has 3 IR Cliff Sensors,  left, right, front
			if( m_CliffSensorsEnabled )
			{
				g_pFullSensorStatus->CliffFront =	g_pKobukiStatus->CliffFront;
				g_pFullSensorStatus->CliffLeft =	g_pKobukiStatus->CliffLeft;
				g_pFullSensorStatus->CliffRight =	g_pKobukiStatus->CliffRight;
			}
			else
			{
				g_pFullSensorStatus->CliffFront =	false;
				g_pFullSensorStatus->CliffRight =	false;
				g_pFullSensorStatus->CliffLeft =	false;
			}

			// Hardware Bumpers, IR range switches, and pressure sensors
			g_pFullSensorStatus->HWBumperFront =		g_pKobukiStatus->BumperFront;
			g_pFullSensorStatus->HWBumperSideLeft =		g_pKobukiStatus->BumperLeft;
			g_pFullSensorStatus->IRBumperSideRight =	g_pKobukiStatus->BumperRight;
			g_pFullSensorStatus->HWBumperRear =			0; // no rear bumper on Kobuki

			// IR Bumpers - No IR Bumpers on Kobuki
			// Arm and finger sensors - no arms on Teleop

			// Compass - Heading is in degrees
			// WARNING - THIS IS NOT REALLY COMPASS HEADING!
			// IT IS DEGREES FROM WHERE KOBUKI WAS POINTING WHEN POWERED ON!
			// (but still useful for determining turn amount)
			g_pFullSensorStatus->CompassHeading = (int)(g_pKobukiStatus->GyroDegrees);


			// Heading and Odometry
			// Calculations are handled in this funciton, which also calls UpdateMoveDistance, UpdateLocation, etc.
			UpdateOdometer();

			// Kobuki Dock IR Beacon
			g_pFullSensorStatus->DockSensorRight =	g_pKobukiStatus->DockRightSignal;
			g_pFullSensorStatus->DockSensorCenter = g_pKobukiStatus->DockCenterSignal;
			g_pFullSensorStatus->DockSensorLeft =	g_pKobukiStatus->DockLeftSignal;

			// Get status of Android Phone bluetooth connection, and get commands
			// HandleAndroidPhone();  // requires Arduino with Bluetooth

			// Other Sensors and state

			// No tilt accelerometer on Loki
			//g_pFullSensorStatus->TiltAccelX =	0;			// Typically From Arduino.  zero = level
			//g_pFullSensorStatus->TiltAccelY =	0;	 		// Typically From Arduino.  zero = level

			// Analog Sensors
			for( int i=0; i<4; i++ )
			{
				g_pFullSensorStatus->IR[i] = g_pKobukiStatus->AnalogPort[i];
			}

			/// TODO-MUST-DAVES - figure out where to factor in offset from front of robot!!!
			// HandleAnalogSensors();
			//g_pFullSensorStatus->IR[0] = ScaleLongRangeIRKobuki( g_pKobukiStatus->AnalogPort[0], (-BASE_IR_OFFSET_FROM_FRONT_TENTH_INCHES + 0) );	// Left Side Long Range, Compesation
			//g_pFullSensorStatus->IR[1] = ScaleLongRangeIRKobuki( g_pKobukiStatus->AnalogPort[1], (-BASE_IR_OFFSET_FROM_FRONT_TENTH_INCHES + 0) );	// Right Side Long Range, Compensation
	//		g_pFullSensorStatus->IR[2] = ScaleWideIRKobuki( g_pKobukiStatus->AnalogPort[2] );			// Left Side Wide angle Short range
	//		g_pFullSensorStatus->IR[3] = ScaleWideIRKobuki( g_pKobukiStatus->AnalogPort[3] );			// Left Side Wide angle Short range


			/* DEBUG
			int AveIR = (g_pKobukiStatus->AnalogPort[0] + g_pKobukiStatus->AnalogPort[1] + LastRawIR ) / 3;
			int DeltaIR = g_pKobukiStatus->AnalogPort[1] - g_pKobukiStatus->AnalogPort[0];
			TRACE("IR Inches 0 = %3d, 1 = %3d,   RAW: IR0 = %4d,  IR1 = %4d,  Average = %4d, Delta = 4%d\n", 
				g_pFullSensorStatus->IR[0]/10, g_pFullSensorStatus->IR[1]/10, g_pKobukiStatus->AnalogPort[0],  g_pKobukiStatus->AnalogPort[1], AveIR, DeltaIR );

			LastRawIR = AveIR; // FOR DEBUG!!!
			*/

			/*
			// For DEBUG, show the values in Inches
				TRACE( "\nIR: (Raw,Inches) ");
				for( int nSensorNumber=0; nSensorNumber< 2; nSensorNumber++ ) // NUM_IR_SENSORS
				{
					TRACE("  %2u=%3u, %3u Inches",
						nSensorNumber, g_pKobukiStatus->AnalogPort[nSensorNumber], g_pFullSensorStatus->IR[nSensorNumber]/10 );
				}
				TRACE( "\n" );
				ROBOT_LOG( TRUE,  "\n" );
			*/


			///////////////////////////////////////////////////////////////////////////
			// Done processing sensor data.
			// Now do sensor fusion to combine the data in a meaningful way
			DoSensorFusion();

			// Now post the status to the GUI display (local or remote)
			SendResponse( WM_ROBOT_DISPLAY_BULK_ITEMS,	// command
				ROBOT_RESPONSE_PIC_STATUS,				// Param1 = Bulk data command
				0 );									// Param2 = not used
		}
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

		// Analog IR sensors
		g_pNavSensorSummary->nLeftFrontZone =	__min( g_pNavSensorSummary->nLeftFrontZone, g_pFullSensorStatus->IR[IR_SENSOR_FRONT_LEFT] );
		g_pNavSensorSummary->nRightFrontZone =	__min( g_pNavSensorSummary->nRightFrontZone, g_pFullSensorStatus->IR[IR_SENSOR_FRONT_RIGHT] );

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

		// Get the average of the two wheels
		int OdometerTenthInches = (g_pKobukiStatus->OdometerLeft + g_pKobukiStatus->OdometerRight) / 2;
		
		// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
		// it's the current odometer reading minus the last odometer reading
		g_pFullSensorStatus->OdometerUpdateTenthInches = OdometerTenthInches - g_pFullSensorStatus->OdometerTenthInches;
					
		// Update the system Odometer
		g_pFullSensorStatus->OdometerTenthInches = OdometerTenthInches;

		
		// Update the "Tach" display.
		g_pFullSensorStatus->Tachometer = (int)(g_pFullSensorStatus->OdometerUpdateTenthInches * 21); // Update * (10 +1 for roundoff) * 2 wheels 

		//		ROBOT_LOG( TRUE,  "DEBUG ODOM L=%3.2f, R=%3.2f, Average=%3.2f, Update=%3.2f\n", 
		//			OdometerdoubleInchesL, OdometerdoubleInchesR, g_pFullSensorStatus->Odometer, g_pFullSensorStatus->OdometerUpdate )
		
		// Update MoveDistance counter, in case a programmed move is in progress
		m_pDriveCtrl->UpdateMoveDistance( g_pFullSensorStatus->OdometerUpdateTenthInches );


		// TODO-MUST test this, not sure it is right!
		double TurnAngleUpdate = g_pKobukiStatus->TurnRate;

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

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sharp GP2Y0A21YK Wide Angle IR sensor
	// Value returned is TENTH INCHES
	// Kobuki has a different A/D converter, so the values are different
	#define KOBUKI_IR_SR_MAX_RANGE_RAW 3000

	int  CSensorModule::ScaleWideIRKobuki(int  nReading )
	{
		// NOTE!  Larger values are closer objects!
		// Therefore, PIC_NO_OBJECT_IN_RANGE seems VERY CLOSE!  Trap here! 
		if( nReading >= KOBUKI_IR_SR_MAX_RANGE_RAW )
			return NO_OBJECT_IN_RANGE;

		int MaxInches = (IR_MAX_TENTH_INCHES / 10);
		for( int inches=0; inches <= MaxInches; inches++ )
		{
			if( nReading > DistanceTable_IRWideAngleKobuki[inches]  )
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

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Sharp GP2Y0A02YK Long Range IR sensor - 5 foot range
	// Value returned is TENTH INCHES
	// Kobuki has a different A/D converter, so the values are different
	#define KOBUKI_IR_LR_MAX_RANGE_VALUE_RAW 3500	// minimum distance
	#define KOBUKI_IR_LR_MIN_RANGE_VALUE_RAW 450	// maximum distance

	int  CSensorModule::ScaleLongRangeIRKobuki(int  nReading, int Compensation )
	{
		// Scale from raw value to inches.  Compensation may be passed in to correct known sensor differences
		// NOTE!  Larger values are closer objects!  
		if( nReading >= KOBUKI_IR_LR_MAX_RANGE_VALUE_RAW )
			return NO_OBJECT_IN_RANGE;

		if( nReading < KOBUKI_IR_LR_MIN_RANGE_VALUE_RAW )	// Outside  readable range value
			return NO_OBJECT_IN_RANGE;

		int RangeTenthInches = NO_OBJECT_IN_RANGE;

		int MaxInches = 48;	// 4 ft
		for( int index=0; index <= MaxInches; index++ )	// Table goes to 72" Max - 6 feet (but noisy after ~4 feet)
		{
			if( nReading >= DistanceTable_IRLongRangeKobuki[index]  )
			{
				RangeTenthInches = (index*10) + Compensation;
				if( RangeTenthInches < 0 ) RangeTenthInches = 0;
				return (RangeTenthInches);	// CONVERTED TO TENTH INCHES!!
			}
		}
		return NO_OBJECT_IN_RANGE;	// No object in sensor range
	}

#endif // SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP_KOBUKI

#endif // ROBOT_SERVER	// This module used for Robot Server only