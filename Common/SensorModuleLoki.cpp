// SensorModuleLoki.cpp: SensorModule Loki specific functions 
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

//#include <math.h>
#include "Globals.h"
#include "module.h"
#include "HardwareConfig.h"
//#include "thread.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define BUMPER_DEBUG_ENABLED
#define ENABLE_BUMPERS 1 // Enable or disable reacting to bumper hits.  Disable if debugging or hardware problem

///////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ProcessSensorStatus()
	// This is where Raw sensor data from Arduino or other sources (Like Kobuki base) get repackaged and copied to g_pFullSensorStatus
	// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
	void CSensorModule::ProcessSensorStatus( UINT uMsg )
	{
		//int nSensorNumber = 0;

		// First thing is to initialize the processed Sensor Status block
		// Don't memset the structure, it corrupts current position!

		// Basic Data
		g_pFullSensorStatus->StatusFlags =	g_RawArduinoStatus.StatusFlags;
		g_pFullSensorStatus->LastError =	g_RawArduinoStatus.LastError;
		g_pFullSensorStatus->DebugCode =	g_RawArduinoStatus.DebugCode;

		if( 0 != g_RawArduinoStatus.Battery0 )
		{
			// Y=MX+B: Voltage = (Arduino Value * 0.041) + 7.0  See Sensor Tests.xlsx for graph
			double BatteryVoltageHundredths = ( ((double)g_RawArduinoStatus.Battery0 * 0.041) + 7.0 ) * 100.0;
			g_pFullSensorStatus->Battery0 = (int ) BatteryVoltageHundredths;	// Pass in milivolts, so we don't need double
		}
		else
		{
			g_pFullSensorStatus->Battery0 = 0;
		}
		// Not used yet: g_pFullSensorStatus->Battery1 =					0;			// Typically From Arduino

		// Cliff and Wheel drops
		// Loki does not have a wheel drop sensor
		g_pFullSensorStatus->WheelDropRight =	false;
		g_pFullSensorStatus->WheelDropLeft =	false;

		// Loki has 2 IR Cliff Sensors, on left and right front
		if( m_CliffSensorsEnabled )
		{
			g_pFullSensorStatus->CliffLeft =		(g_RawArduinoStatus.IRBumper & IR_BUMPER_CLIFF_LEFT_MASK) != 0;
			g_pFullSensorStatus->CliffRight =		(g_RawArduinoStatus.IRBumper & IR_BUMPER_CLIFF_RIGHT_MASK) != 0;
		}
		else
		{
			g_pFullSensorStatus->CliffRight =	false;
			g_pFullSensorStatus->CliffLeft =	false;
		}
		// If both sensors see a cliff, it's directly in front of us
		g_pFullSensorStatus->CliffFront =	g_pFullSensorStatus->CliffLeft && g_pFullSensorStatus->CliffRight;

		// Hardware Bumpers, IR range switches, and pressure sensors
		#if ENABLE_BUMPERS == 1
			if( 0 != g_RawArduinoStatus.HWBumper )
			{
				CString MsgString;
				MsgString.Format("HW BUMPER HIT: %04lX", g_RawArduinoStatus.HWBumper);
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			}
		
			// Hardware Bumpers
			g_pFullSensorStatus->HWBumperFront =		(g_RawArduinoStatus.HWBumper & HW_BUMPER_FRONT_MASK) != 0;
			g_pFullSensorStatus->HWBumperRear =			(g_RawArduinoStatus.HWBumper & HW_BUMPER_REAR_MASK) != 0;
			g_pFullSensorStatus->HWBumperSideLeft =		(g_RawArduinoStatus.HWBumper & HW_BUMPER_SIDE_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperSideRight =	(g_RawArduinoStatus.HWBumper & HW_BUMPER_SIDE_RIGHT_MASK) != 0;

			// IR Bumpers
			g_pFullSensorStatus->IRBumperFrontLeft =	(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperFrontRight =	(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperRearLeft =		(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperRearRight =	(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperSideLeft =		(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;
			g_pFullSensorStatus->IRBumperSideRight =	(g_RawArduinoStatus.HWBumper & IR_BUMPER_FRONT_LEFT_MASK) != 0;

			g_pFullSensorStatus->ArmRightBumperElbow =	(g_RawArduinoStatus.ArmBumperR & ARM_R_HW_BUMPER_ELBOW_MASK) != 0;
			g_pFullSensorStatus->ArmLeftBumperElbow =	(g_RawArduinoStatus.ArmBumperL & ARM_L_HW_BUMPER_ELBOW_MASK) != 0;
		#endif

		g_pFullSensorStatus->ArmLeftBumperFingerLeft =	(g_RawArduinoStatus.ArmBumperL & ARM_L_IR_BUMPER_FINGER_L_MASK) != 0;
		g_pFullSensorStatus->ArmLeftBumperFingerRight =	(g_RawArduinoStatus.ArmBumperL & ARM_L_IR_BUMPER_FINGER_R_MASK) != 0;
		g_pFullSensorStatus->ArmLeftBumperInsideClaw =	(g_RawArduinoStatus.ArmBumperL & ARM_L_BUMPER_INSIDE_CLAW_MASK) != 0;

		// Pressure sensor in left claw finger tips
		g_pFullSensorStatus->LeftHandRawPressureL =	(int)g_RawArduinoStatus.LeftHandPressureL;
		g_pFullSensorStatus->LeftHandRawPressureR =	(int)g_RawArduinoStatus.LeftHandPressureR;
		//ROBOT_LOG( TRUE,  " DEBUG RAW HAND PRESSURE L =  %3d, R = %3d", g_pFullSensorStatus->LeftHandRawPressureL, g_pFullSensorStatus->LeftHandRawPressureR )

		// Compass
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
		g_pFullSensorStatus->CompassHeading = Compass;


		// Heading and Odometry
		// Calculations are handled in this funciton, which also calls UpdateMoveDistance, UpdateLocation, etc.
		UpdateOdometer();

		// Kobuki Dock not used for Loki
		//g_pFullSensorStatus->DockSensorRight =		0;				
		//g_pFullSensorStatus->DockSensorCenter =		0;			// IR sensors for the Kobuki Dock
		//g_pFullSensorStatus->DockSensorLeft =			0;	

		// Get status of Android Phone bluetooth connection, and get commands
		HandleAndroidPhone();

		// Other Sensors and state
		g_pFullSensorStatus->PIRMotionLeft =	(g_RawArduinoStatus.IRBumper & IR_BUMPER_PIR_LEFT_MASK) != 0;
		g_pFullSensorStatus->PIRMotionRight =	(g_RawArduinoStatus.IRBumper & IR_BUMPER_PIR_RIGHT_MASK) != 0;
		HandleThermalSensor();	// process TPS Thermal sensor

		// No tilt accelerometer on Loki
		//g_pFullSensorStatus->TiltAccelX =	0;			// Typically From Arduino.  zero = level
		//g_pFullSensorStatus->TiltAccelY =	0;	 		// Typically From Arduino.  zero = level

		// Analog Sensors
		HandleAnalogSensors();


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


	} // ProcessSensorStatus





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

		////////////////////////////////////////////////////////
		// NOTE! Ultrasonic not reliable indoors for now!
		// use IR only! - TODO
		////////////////////////////////////////////////////////

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
			//		g_pNavSensorSummary->nLeftRearZone =			g_pLaserSummary->nLeftRearZone;	
			///		g_pNavSensorSummary->bLeftCliff =				g_pLaserSummary->bLeftCliff;
					g_pNavSensorSummary->nLeftSideZone =			g_pLaserSummary->nLeftSideZone;	 
					g_pNavSensorSummary->nLeftFrontSideZone =		g_pLaserSummary->nLeftFrontSideZone;	 
					g_pNavSensorSummary->nLeftArmZone =				g_pLaserSummary->nLeftArmZone;	// Object in front of Arm
					g_pNavSensorSummary->nLeftFrontZone =			g_pLaserSummary->nLeftFrontZone;
					g_pNavSensorSummary->nRightFrontZone =			g_pLaserSummary->nRightFrontZone;
					g_pNavSensorSummary->nRightArmZone =			g_pLaserSummary->nRightArmZone; // Object in front of Arm
					g_pNavSensorSummary->nRightFrontSideZone =		g_pLaserSummary->nRightFrontSideZone;
					g_pNavSensorSummary->nRightSideZone =			g_pLaserSummary->nRightSideZone;
			///		g_pNavSensorSummary->bRightCliff =				g_pLaserSummary->bRightCliff;
			//		g_pNavSensorSummary->nRightRearZone =			g_pLaserSummary->nRightRearZone;	
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
		//		g_pNavSensorSummary->nLeftRearZone =			__min( g_pNavSensorSummary->nLeftRearZone, g_pKinectSummary->nLeftRearZone );	
		///		g_pNavSensorSummary->bLeftCliff =				g_pKinectSummary->bLeftCliff;
				g_pNavSensorSummary->nLeftSideZone =			__min( g_pNavSensorSummary->nLeftSideZone, g_pKinectSummary->nLeftSideZone );	 
				g_pNavSensorSummary->nLeftFrontSideZone =		__min( g_pNavSensorSummary->nLeftFrontSideZone, g_pKinectSummary->nLeftFrontSideZone );	 
				g_pNavSensorSummary->nLeftArmZone =				__min( g_pNavSensorSummary->nLeftArmZone, g_pKinectSummary->nLeftArmZone );	// Object in front of Arm
				g_pNavSensorSummary->nLeftFrontZone =			__min( g_pNavSensorSummary->nLeftFrontZone, g_pKinectSummary->nLeftFrontZone );
				g_pNavSensorSummary->nRightFrontZone =			__min( g_pNavSensorSummary->nRightFrontZone, g_pKinectSummary->nRightFrontZone );
				g_pNavSensorSummary->nRightArmZone =			__min( g_pNavSensorSummary->nRightArmZone, g_pKinectSummary->nRightArmZone ); // Object in front of Arm
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


		// Front sensors
		g_pNavSensorSummary->nLeftFrontZone  = __min( g_pNavSensorSummary->nLeftFrontZone,  g_pFullSensorStatus->IR[IR_SENSOR_FRONT_LEFT] );
		g_pNavSensorSummary->nRightFrontZone = __min( g_pNavSensorSummary->nRightFrontZone, g_pFullSensorStatus->IR[IR_SENSOR_FRONT_RIGHT] );

		// Claw IR sensors
		g_pNavSensorSummary->nObjectClawLeft  = (ARM_L_IR_SENSOR_CLAW <= 30) ? 0 : (ARM_L_IR_SENSOR_CLAW - 30); // distance from sensor to tip of claw - Tenth Inches
		g_pNavSensorSummary->nObjectClawRight = (ARM_R_IR_SENSOR_CLAW <= 30) ? 0 : (ARM_R_IR_SENSOR_CLAW - 30); // distance from sensor to tip of claw
		// NO LONGER USED
		// if( ARM_L_IR_BUMPER_OBJECT_FINGER_L || ARM_L_IR_BUMPER_OBJECT_FINGER_R )
		//{
			//g_pNavSensorSummary->nObjectClawLeft  = __min( g_pNavSensorSummary->nObjectClawLeft, 30);	// range of Fingertip IR sensors
		//}

		// Left Claw pressure sensors
		// TODO : int LeftHandPressure = g_pNavSensorSummary->LeftHandPressureL;


		// Angled IR "Bumpers".  If hit detected, object is at least as close as max range of the sensor
		int BumperLeft =  g_pFullSensorStatus->IRBumperFrontLeft  ? IR_BUMPER_RANGE: NO_OBJECT_IN_RANGE;
		int BumperRight = g_pFullSensorStatus->IRBumperFrontLeft  ? IR_BUMPER_RANGE: NO_OBJECT_IN_RANGE;
		g_pNavSensorSummary->nLeftArmZone  = __min( g_pNavSensorSummary->nLeftArmZone,  BumperLeft);
		g_pNavSensorSummary->nRightArmZone  = __min( g_pNavSensorSummary->nRightArmZone,  BumperRight);


		// Check Elbow mounted IR sensors.  Handles case of claw blocking IR
		/**
		g_pNavSensorSummary->nObjectArmLeft = ReadElbowSensorsLeft();
		if( 0 == g_pNavSensorSummary->nObjectArmLeft )
		{
			strSensor = "SensorModule: ARM-L ELBOW HIT! ";
			ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-L ELBOW HIT! " )
		}
		g_pNavSensorSummary->nObjectArmRight = ReadElbowSensorsRight();
		if( 0 == g_pNavSensorSummary->nObjectArmRight )
		{
			ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: ARM-R ELBOW HIT! " )
		}
		**/

		// Check Bumpers
		if( g_pFullSensorStatus->HWBumperFront && (ENABLE_BUMPERS == 1) )
		{		
			// Only one front HW bumper on Loki
			g_pNavSensorSummary->nLeftFrontZone = 0; // Collision!
			g_pNavSensorSummary->nRightFrontZone = 0; // Collision!
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Front Bumper!\n" )
			SpeakText( "Sorry" );	
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now, continue combining sensor info for front sensors

		// Start with analog sensors, then digital.  Look for closest object on each front area (left and right).

	/*** DISABLE ARM SENSORS.  JUST USE THE LASER
		if( gArmInHomePositionLeft )
		{
			g_pNavSensorSummary->nLeftArmZone = __min( g_pNavSensorSummary->nLeftArmZone, g_pNavSensorSummary->nObjectClawLeft );
			g_pNavSensorSummary->nLeftArmZone = __min( g_pNavSensorSummary->nLeftArmZone, g_pNavSensorSummary->nObjectArmLeft );
		}

		if( gArmInHomePositionRight )
		{
			g_pNavSensorSummary->nRightArmZone = __min( g_pNavSensorSummary->nRightArmZone, g_pNavSensorSummary->nObjectClawRight );
			g_pNavSensorSummary->nRightArmZone = __min( g_pNavSensorSummary->nRightArmZone, g_pNavSensorSummary->nObjectArmRight );
		}
	***/
		g_pNavSensorSummary->nClosestObjectFrontLeft = __min( g_pNavSensorSummary->nClosestObjectFrontLeft, g_pNavSensorSummary->nLeftFrontZone ); // Front IR, Laser, and bumpers
		g_pNavSensorSummary->nClosestObjectFrontLeft = __min( g_pNavSensorSummary->nClosestObjectFrontLeft, g_pNavSensorSummary->nLeftArmZone );

		g_pNavSensorSummary->nClosestObjectFrontRight = __min( g_pNavSensorSummary->nClosestObjectFrontRight, g_pNavSensorSummary->nRightFrontZone );
		g_pNavSensorSummary->nClosestObjectFrontRight = __min( g_pNavSensorSummary->nClosestObjectFrontRight, g_pNavSensorSummary->nRightArmZone );

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
		// Loki has one Rear HW bumper, and 2 IR Bumpers
		if( g_pFullSensorStatus->HWBumperRear && (ENABLE_BUMPERS == 1) )
		{
			g_pNavSensorSummary->nRearObjectDirection = OBJECT_EQUAL_DISTANCE;
			g_pNavSensorSummary->nRearObjectDistance = 0;	// Collision!
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Rear Bumper!\n" )
			g_pNavSensorSummary->nLeftRearZone = 0;
			g_pNavSensorSummary->nRightRearZone = 0;
		}
		else if( g_pFullSensorStatus->IRBumperRearLeft  || g_pFullSensorStatus->IRBumperRearRight )
		{
			g_pNavSensorSummary->nRearObjectDistance = 1;	// Almost Collision (since sensors sit in a bit from the back)

			if( g_pFullSensorStatus->IRBumperRearLeft && g_pFullSensorStatus->IRBumperRearRight )
			{
				// Hit on Both IR bumpers!  Oh no!  what to do?
				g_pNavSensorSummary->nRearObjectDirection = OBJECT_EQUAL_DISTANCE;
				g_pNavSensorSummary->nLeftRearZone = 1;
				g_pNavSensorSummary->nRightRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Both Rear IR Bumpers!\n " )
			}
			else if( g_pFullSensorStatus->IRBumperRearLeft )
			{		
				// Hit on Left IR bumper
				g_pNavSensorSummary->nRearObjectDirection = REAR_LEFT;
				g_pNavSensorSummary->nLeftRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Rear Left IR Bumper!\n " )
			}
			else if( g_pFullSensorStatus->IRBumperRearRight )
			{		
				// Hit on Right IR bumper
				g_pNavSensorSummary->nRearObjectDirection = REAR_RIGHT;
				g_pNavSensorSummary->nRightRearZone = 1;
				ROBOT_LOG( REPORT_CLOSE_OBJECTS, "SensorModule: Object Rear Right IR Bumper!\n " )
			}
			else
			{
				// Logic Error
				ROBOT_ASSERT(0);
			}
		}

		// SIDE SENSORS
		// Loki has IR side sensors and side bumpers

		// Handle analog sensors first
		g_pNavSensorSummary->nLeftSideZone  = __min( g_pNavSensorSummary->nLeftSideZone,  g_pFullSensorStatus->IR[IR_SENSOR_SIDE_LEFT]);
		g_pNavSensorSummary->nRightSideZone  = __min( g_pNavSensorSummary->nRightSideZone,  g_pFullSensorStatus->IR[IR_SENSOR_SIDE_RIGHT]);


		// Then bumpers
		if( g_pFullSensorStatus->IRBumperSideLeft && (ENABLE_BUMPERS == 1) )
		{		
			g_pNavSensorSummary->nSideObjectDirection = SIDE_LEFT;
			g_pNavSensorSummary->nLeftSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Left Side Bumper!\n" )
			SpeakText( "Oops" );	
		}
		if( g_pFullSensorStatus->IRBumperSideRight && (ENABLE_BUMPERS == 1) )
		{		
			g_pNavSensorSummary->nSideObjectDirection = SIDE_RIGHT;
			g_pNavSensorSummary->nRightSideZone = 0;
			ROBOT_DISPLAY( TRUE, "SensorModule: BUMPER HIT!  Right Side Bumper!\n" )
			SpeakText( "Darn" );	
		}

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

	
		// Now, check Head mounted sensors and override prior calculated values if needed
		// TODO-LOKI-MUST!!! NEED TO SET THE SERVO VALUES BELOW TO CORRECT VALUES!
		/******** TODO-MUST

		if(	g_CameraTiltPos > (CAMERA_TILT_CENTER -15) )	// Must not be pointing at the ground! 
		{
			if( g_CameraPanPos < (CAMERA_PAN_CENTER -30) )	
			{
				// Camera pointing Left Side
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nSideObjectDistance )
				{
					// Object is closer then previously reported
					g_pNavSensorSummary->nSideObjectDistance = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nSideObjectDirection = SIDE_LEFT;
					g_pNavSensorSummary->nLeftSideZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -20) )	
			{
				// Camera pointing Left Angle
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nAngleObjectDistance )
				{
					// Object is closer then previously reported
					g_pNavSensorSummary->nAngleObjectDistance = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nAngleObjectDirection = ANGLE_LEFT;
					g_pNavSensorSummary->nLeftArmZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			if( g_CameraPanPos > (CAMERA_PAN_CENTER +30) )	
			{
				// Camera pointing Right Side
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nSideObjectDistance )
				{
					// Object is closer then previously reported
					g_pNavSensorSummary->nSideObjectDistance = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nSideObjectDirection = SIDE_RIGHT;
					g_pNavSensorSummary->nRightSideZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +20) )	
			{
				// Camera pointing Right Angle
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nAngleObjectDistance )
				{
					// Object is closer then previously reported
					g_pNavSensorSummary->nAngleObjectDistance = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nAngleObjectDirection = ANGLE_RIGHT;
					g_pNavSensorSummary->nRightArmZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else
			{
				// Camera sensor is pointing forward
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nFrontObjectDistance )
				{
					// Object is closer then previously reported
					g_pNavSensorSummary->nFrontObjectDistance = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;
					g_pNavSensorSummary->nLeftFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
					g_pNavSensorSummary->nRightFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
		}
		*****/


	}	// DoSensorFusion


	//////////////////////////////////////////////////////////////////////////
	void CSensorModule::UpdateOdometer()
	{
		// g_pFullSensorStatus->Odometer is the distance since the last Odometer RESET
		// g_pFullSensorStatus->OdometerUpdate is the distance since the last Odometer READING
		// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
		// RAW Arduino Status is in TICKS, Laptop Arduino Status is in TenthInches

		#if MOTOR_CONTROL_TYPE != ER1_MOTOR_CONTROL	// *NOT* ER1!
		// Using Arduino for Odometry.  For ER1, see WM_ROBOT_ER1_ODOMETER_READY in SensorModule.cpp

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
						g_pFullSensorStatus->TachometerTicksL = (int)TempOdometerTicksL * -1; 
						LeftDir = strRev;
						//TRACE("TACH Left = REV ");
					}
					else
					{
						// "Reverse" bit not set
						TempOdometerTicksL &= 0x7FFF; // clear the reverse bit - KLUDGE
						g_pFullSensorStatus->TachometerTicksL = TempOdometerTicksL; 
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
						g_pFullSensorStatus->TachometerTicksR = (int)TempOdometerTicksR * -1;
						RightDir = strRev;
						//TRACE("TACH Right = REV ");
					}
					else
					{
						// "Reverse" bit not set
						TempOdometerTicksR &= 0x7FFF; // clear the reverse bit - KLUDGE
						g_pFullSensorStatus->TachometerTicksR = TempOdometerTicksR;
						// TRACE("TACH Right = FWD ");
					}

					//TRACE("\n");
						
					//////////////////////////////////////////////
					// DEBUG
					//*
					if( (0 != g_pFullSensorStatus->TachometerTicksL) || (0 != g_pFullSensorStatus->TachometerTicksR) )
					{
						ROBOT_LOG( TRUE, "TACH_L: %s %4d   TACH_R: %s %4d, Samples: %d", 
							LeftDir, g_pFullSensorStatus->TachometerTicksL, RightDir, g_pFullSensorStatus->TachometerTicksR, g_RawArduinoStatus.nOdometerSamples )
					} 
					//*/


					// We get a weird rounding error with zero values so kludge it here
					double OdometerUpdateTenthInchesL = 0;
					if( 0 != g_pFullSensorStatus->TachometerTicksL ) 
						OdometerUpdateTenthInchesL = g_pFullSensorStatus->TachometerTicksL / TICKS_PER_TENTH_INCH;

					double OdometerUpdateTenthInchesR = 0;
					if( 0 != g_pFullSensorStatus->TachometerTicksR ) 
						OdometerUpdateTenthInchesR = g_pFullSensorStatus->TachometerTicksR / TICKS_PER_TENTH_INCH;

					// Now calculate and set the higher level odometer readings
					UpdateFromTwoOdometers( OdometerUpdateTenthInchesL, OdometerUpdateTenthInchesR);

					// DEBUG STUFF
					if( (0 != g_pFullSensorStatus->TachometerTicksL) || (0 != g_pFullSensorStatus->TachometerTicksL) )
					{
				//		ROBOT_LOG( TRUE,    "**** ODOM DEBUG: g_pFullSensorStatus->TachometerTicksL = %d, g_pFullSensorStatus->TachometerTicksR = %d\n", g_pFullSensorStatus->TachometerTicksL, g_pFullSensorStatus->TachometerTicksR )
						//ROBOT_LOG( TRUE,  "**** ODOM DEBUG: OdometerHigh, Low L = %02X, %02X\n", g_RawArduinoStatus.OdometerHighL, g_RawArduinoStatus.OdometerLowL )
						//ROBOT_LOG( TRUE,  "**** ODOM DEBUG: OdometerHigh, Low R = %02X, %02X\n", g_RawArduinoStatus.OdometerHighR, g_RawArduinoStatus.OdometerLowR )
					}
					/***
					// Calibration for TICKS_PER_TENTH_INCH
										static int TempOdomTicksL = 0;
										static int TempOdomTicksR = 0;
										if( 0 != g_pFullSensorStatus->TachometerTicksL ) TempOdomTicksL +=g_pFullSensorStatus->TachometerTicksL;
										if( 0 != g_pFullSensorStatus->TachometerTicksR ) TempOdomTicksR +=g_pFullSensorStatus->TachometerTicksR;

						ROBOT_LOG( TRUE,  "**** ODOM DEBUG: TempOdomTicksL, R = %d, %d\n", TempOdomTicksL, TempOdomTicksR )
					***/
					// Calibration for Tachometer:
					//	ROBOT_LOG( TRUE,  "**** ODOM DEBUG: SPEED REQUEST = %d  RAW TACH: R = %d, L = %d \n", g_MotorCurrentSpeedCmd, g_pFullSensorStatus->TachometerTicksL, g_pFullSensorStatus->TachometerTicksR );
			}
			#else	// Single Odometer for robot.

				// Save prior Odometer reading
				int LastOdometer = g_pFullSensorStatus->Odometer;	// in Inches

				// "Unpack" Arduino Odometer reading (in Ticks)
				signed short TempOdometerTicks = g_RawArduinoStatus.OdometerHigh <<8;		// Shift the high byte ("hundreds and tens") over
				TempOdometerTicks += g_RawArduinoStatus.OdometerLow;					// Add in the low byte

				// Update the system Odometer
				g_pFullSensorStatus->OdometerTenthInches = TempOdometerTicks * TICKS_PER_TENTH_INCH;
				//ROBOT_LOG( TRUE,  "DEBUG ODOM = %d\n", g_pFullSensorStatus->Odometer)

				// Update the distance traveled since the last update (for Navigation update and SetMoveDistance)
				g_pFullSensorStatus->OdometerUpdate = g_pFullSensorStatus->Odometer - LastOdometer;
				// Update MoveDistance counter, in case a programmed move is in progress
				m_pDriveCtrl->UpdateMoveDistance( g_pFullSensorStatus->OdometerUpdate );
				// (used on ER1: m_pDriveCtrl->UpdateTurnRotation( ?? );

			#endif

		#endif

	} // UpdateOdometer


	//////////////////////////////////////////////////////////////////////////
	void CSensorModule::HandleThermalSensor()
	{
		// Process data from the TPS Thermal sensor, which has 9 sensor elements for spotting humans 
		// Determine if thermal object found, and sumarize results
		#ifdef THERMAL_SENSOR_PROCESSING_ENABLED // DISABLED! 
			#define THERMAL_THRESHOLD  2  // amount above ambient floor needed to trigger a hit

			int Threshold = g_RawArduinoStatus.ThermalArray[0] + THERMAL_THRESHOLD;
			g_pFullSensorStatus->ThermalPosition = 0;
			int MaxValue = 0;
			int Position = 0;

			// Look for strongest reading
			for(int i= 1; i<9; i++)
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
				g_pFullSensorStatus->ThermalPosition = 0;
			}
			else if( Position <= 4 )
			{
				// left side
				g_pFullSensorStatus->ThermalPosition = Position - 5;	// Negative = left of center
				//ROBOT_LOG( TRUE,  "DEBUG Thermal Position = %d\n", g_pFullSensorStatus->ThermalPosition)
			}
			else // Position >=5
			{
				// right side
				g_pFullSensorStatus->ThermalPosition = Position - 4;	// Positive = right of center
				//ROBOT_LOG( TRUE,  "DEBUG Thermal Position = %d\n", g_pFullSensorStatus->ThermalPosition)
			}

		#endif

		#ifdef THERMAL_SENSOR_DEBUG_ENABLED
			ROBOT_LOG( TRUE,  "DEBUG Thermal: ")
			for(int i=1; i<9; i++)
			{
				if( g_RawArduinoStatus.ThermalArray[i] <= Threshold )
				{
					g_pFullSensorStatus->ThermalArray[i] = 0;
					ROBOT_LOG( TRUE,  "_ " )
				}
				else
				{
					g_pFullSensorStatus->ThermalArray[i] = g_RawArduinoStatus.ThermalArray[i] - Threshold;
					ROBOT_LOG( TRUE,  "%d ", g_pFullSensorStatus->ThermalArray[i] )
				}
			}
			ROBOT_LOG( TRUE,  "\n" )
		#endif

	}	// HandleThermalSensor


	//////////////////////////////////////////////////////////////////////////
	void CSensorModule::HandleAnalogSensors()
	{
		// Analog IR Sensors connected to the Arduino
		// IR Sensor 0,5 = Side, Wide Angle, short range
		// IR Sensor 1,4 = Forward, Long Range
		// IR Sensor 2,3 = Forward, Head mounted Long Range

		g_pFullSensorStatus->IR[0] = ScaleWideIR(g_RawArduinoStatus.IR[0]); // Left Side, Wide Angle, short range
		g_pFullSensorStatus->IR[1] = ScaleLongRangeIR(g_RawArduinoStatus.IR[1], 0); // Left Forward, Long Range
		g_pFullSensorStatus->IR[2] = ScaleLongRangeIR(g_RawArduinoStatus.IR[2], 0); // // Right Long Range IR in Head, compensate for sensor differences
		g_pFullSensorStatus->IR[3] = ScaleLongRangeIR(g_RawArduinoStatus.IR[3], 0); // Left Long Range IR in Head, compensate for sensor differences
		g_pFullSensorStatus->IR[4] = ScaleLongRangeIR(g_RawArduinoStatus.IR[4], 0); // Right Forward, Long Range
		g_pFullSensorStatus->IR[5] = ScaleWideIR(g_RawArduinoStatus.IR[5]); // Right Side, Wide Angle, short range

		// TODO-MUST: BAD SENSORS NEED TO FIX.  JUST DISABLE FOR NOW
		//g_pFullSensorStatus->IR[1] = NO_OBJECT_IN_RANGE;
		//g_pFullSensorStatus->IR[4] = NO_OBJECT_IN_RANGE;

		//	if(g_pFullSensorStatus->IR[0] < 5) ROBOT_LOG( TRUE,  "******** IR0 ********\n" )
		//	if(g_pFullSensorStatus->IR[5] < 5) ROBOT_LOG( TRUE,  "******** IR5 ********\n" )

		// Now, show the values in Inches

		#if DEBUG_IR == 1
			TRACE( "IR, RAW, INCHES: ");
			for( int nSensorNumber=0; nSensorNumber< NUM_IR_SENSORS; nSensorNumber++ )
			{
				TRACE("  %2u=%3u,%3u",
					nSensorNumber, g_RawArduinoStatus.IR[nSensorNumber], g_pFullSensorStatus->IR[nSensorNumber]/10 );
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
			g_pFullSensorStatus->IR2[0] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[0], 0);	// Left
			g_pFullSensorStatus->IR2[1] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[1], 0);	// Right
		#endif

			// Head sensors (analog signal sent to expansion board in robot base) Long Range IR sensors
			// Note that Head sensors are set back 8 inches from front panel, so compensation needed at time when used
			//ROBOT_LOG( TRUE,  " DEBUG RAW HEAD IR 2 = %d, 3 = %d\n", g_RawArduinoStatus.IR2[2], g_RawArduinoStatus.IR2[3] )
		/* DISABLED
			// Right Long Range IR in Head
			g_pFullSensorStatus->IR2[2] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[2], 0); // compensate for sensor differences
			//if( TempHeadRange < NO_OBJECT_IN_RANGE ) TempHeadRange += 1;// compensate for sensor differences
			//g_pFullSensorStatus->IR2[2] = TempHeadRange;

			// Left Long Range IR in Head
			g_pFullSensorStatus->IR2[3] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[3], -10); // compensate for sensor differences
			//if( TempHeadRange < NO_OBJECT_IN_RANGE ) TempHeadRange -= 1;// compensate for sensor differences
			//g_pFullSensorStatus->IR2[3] = TempHeadRange;
		*/

		// Other I2C A/D IR sensors: 
		/* (NOT CONNECTED)
		g_pFullSensorStatus->IR2[5] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[5], 0);
		g_pFullSensorStatus->IR2[6] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[6], 0);
		g_pFullSensorStatus->IR2[7] = ScaleLongRangeIR(g_RawArduinoStatus.IR2[7], 0);
		*/
		//	ROBOT_LOG( TRUE,  "   HEAD IR = %03d = %04X\n",   g_pFullSensorStatus->IR2[2], g_pFullSensorStatus->IR2[2] )
		//	ROBOT_LOG( TRUE,  "   HEAD US = %03d = %04X\n\n", g_pFullSensorStatus->IR2[3], g_pFullSensorStatus->IR2[3] )

		//-------------------------------------------------------------------------
		// I2C-IT IR Sensors.  Values reported in Inches, so multiply by 10 for TenthInches.
		// (Centemeters is an options too.  See Arduino code) 
		// TODO - Read Centemeters for greater accuracy, then convert directly to tenthinches!
		for( int nSensorNumber=0; nSensorNumber<NUM_IR3_SENSORS; nSensorNumber++ )
		{
			g_pFullSensorStatus->IR3[nSensorNumber] = g_RawArduinoStatus.IR3[nSensorNumber] * 10;
		}

		/*
		//	#if DEBUG_IR == 1
			ROBOT_LOG( TRUE,  "   I2C-IT = ");
			int nSensorNumber = 0;
			for( nSensorNumber=0; nSensorNumber < 4; nSensorNumber++ )	// NUM_IR3_SENSORS
			{
				ROBOT_LOG( TRUE,  "  %01u:%03u",nSensorNumber, g_RawArduinoStatus.IR3[nSensorNumber] );
			}
			ROBOT_LOG( TRUE,  " Inches\n" ); //TenthInches???
		//	#endif
		/*/

		//ROBOT_LOG( TRUE,  "I2C-IT  %01u:%03u\n",0, g_RawArduinoStatus.IR3[0] );


	}	// HandleAnalogSensors


#endif // SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

#endif // ROBOT_SERVER	// This module used for Robot Server only