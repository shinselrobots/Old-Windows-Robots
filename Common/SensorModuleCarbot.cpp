// SensorModuleCarBot.cpp: SensorModule CarBot (Seeker) specific functions 
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
//#define ENABLE_BUMPERS 1 // Enable or disable reacting to bumper hits.  Disable if debugging or hardware problem

///////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

	// WARNING! THIS CODE FOR CARBOT HAS NOT BEEN COMPILED OR TESTED IN A LONG TIME!
	// IT PROBABLY NEEDS LOTS OF UPDATES

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ProcessSensorStatus()
	// This is where Raw sensor data from Arduino or other sources (Like Kobuki base) get repackaged and copied to g_pFullSensorStatus
	// There are different implementations of this function for each robot type.  See "SensorModuleXXX" for each robot type.
	void CSensorModule::ProcessSensorStatus( )
	{
		int nSensorNumber = 0;

		// First thing is to initialize the processed Sensor Status block
		// Don't memset the structure, it corrupts current position!

		// Copy basic stuff first
		g_pFullSensorStatus->StatusFlags =		g_RawArduinoStatus.StatusFlags;
		//g_pFullSensorStatus->LastError =		g_RawArduinoStatus.LastError;
		g_pFullSensorStatus->DebugCode =		g_RawArduinoStatus.DebugCode;

		//ROBOT_LOG( TRUE,  "***********  DEBUG ArmBumpeL = %02X\n", g_RawArduinoStatus.ArmBumperR )

		///////////////////////////////////////////////////////////////////////////////////////////////

		g_pFullSensorStatus->Tachometer =	g_RawArduinoStatus.Tachometer;
		//ROBOT_LOG( TRUE,  "----------------> DEBUG TACH = %d\n", g_RawArduinoStatus.Tachometer)
		//ROBOT_LOG( TRUE,  "----------------> DEBUG Battery = %d\n", g_RawArduinoStatus.Battery0)

		g_pFullSensorStatus->HWBumper = g_RawArduinoStatus.Bumper;
		g_pFullSensorStatus->IRBumper = 0;	// No IR Bumpers on CarBot

		if( 0 != g_pFullSensorStatus->HWBumper )
		{
			CString MsgString;
			MsgString.Format("HW BUMPER HIT: %04lX", g_pFullSensorStatus->HWBumper);
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		}


		// MUST MATCH INVERT JUMPER SETTING!
		if( INVERT_RC_BUTTON_PWR_ENABLE )	// Change this in Globals.h if jumper changes!
		{
			if( (g_pFullSensorStatus->StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
			{
				// bit is set, clear it
				g_pFullSensorStatus->StatusFlags &= (0xFF^HW_STATUS_RC_BUTTON_PWR_ENABLE); // Clear Flag
			}
			else
			{
				// bit is clear, set it
				g_pFullSensorStatus->StatusFlags |=  HW_STATUS_RC_BUTTON_PWR_ENABLE; // Set Flag
			}
		}

		/* TODO-CAR-MUST - need to calculate the mapping from units to Volts for Carbot!
		if( 0 != g_RawArduinoStatus.Battery0 )
		{
			// Y=MX+B: Voltage = (Arduino Value * 0.042) + 3.15
			double BatteryVoltageHundredths = ( ((double)g_RawArduinoStatus.Battery0 * 0.042) + 3.15 ) * 100;
			g_pFullSensorStatus->Battery0 = (int ) BatteryVoltageHundredths;	// Pass in milivolts, so we don't need double
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

		g_pFullSensorStatus->AccelX = SumX / ACC_MAX_SAMPLES;
		g_pFullSensorStatus->AccelY = SumY / ACC_MAX_SAMPLES;

		//	ROBOT_LOG( TRUE,  "DEBUG Accel Inst: X=%03d Y=%03d  Ave: X=%03d Y=%03d\n", 
		//		TempAccX, TempAccY, g_pFullSensorStatus->AccelX, g_pFullSensorStatus->AccelY )
		//	ROBOT_LOG( TRUE,  "DEBUG Accel: X=%03d  Y=%03d\n", g_RawArduinoStatus.AccelX, g_RawArduinoStatus.AccelY)

		int  SlopeDir;
		int  SlopeAmt;
		ConvertAccToTilt( g_pFullSensorStatus->AccelX , g_pFullSensorStatus->AccelY, SlopeDir, SlopeAmt);

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
		g_pFullSensorStatus->CompassHeading = Compass;


		//////////////////////////////////////////////////////////////////////////
		// ODOMETER
		// g_pFullSensorStatus->Odometer is the distance since the last Odometer RESET
		// g_pFullSensorStatus->OdometerUpdate is the distance since the last Odometer READING
		// Note, The odometer value could be positive or negative, depending if we are going FWD or REV.
		// RAW Arduino Status is in TICKS, Laptop Arduino Status is in TenthInches

		// Using Arduino for Odometry.  For ER1, see WM_ROBOT_ER1_ODOMETER_READY
		// This Robot uses a Single Odometer!
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
			g_pFullSensorStatus->IR[nSensorNumber] = ScaleLongRangeIR(g_RawArduinoStatus.IR[nSensorNumber], 0);
		}
		#if DEBUG_IR == 1
			ROBOT_LOG( TRUE,  "   INCHES= ")
			for( nSensorNumber=0; nSensorNumber<NUM_IR_SENSORS; nSensorNumber++ )
			{
				ROBOT_LOG( TRUE,  " %u:%u",nSensorNumber, g_pFullSensorStatus->IR[nSensorNumber] );
			}
			ROBOT_LOG( TRUE,  "\n" );
		#endif


		//////////////////////////////////////////////////////////////////////////
		// ULTRASONIC
		#ifdef ULTRASONICS_NOT_DISABLED //  TODO-MUST!!

			for( nSensorNumber=0; nSensorNumber < NUM_US_SENSORS; nSensorNumber++ )
			{
				g_pFullSensorStatus->US[nSensorNumber] = NO_OBJECT_IN_RANGE;
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
	 				g_pFullSensorStatus->US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
				}
				else if( HW_SENSOR_NOT_ATTACHED == g_RawArduinoStatus.US[nSensorNumber] )
				{
					#if DEBUG_ULTRASONIC == 1
						ROBOT_LOG( TRUE,  "   %01u:NON",nSensorNumber );	// Sensor Not Attached
					#endif
	 				g_pFullSensorStatus->US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
				}
				else if( HW_SENSOR_ERROR == g_RawArduinoStatus.US[nSensorNumber] )
				{
					#if DEBUG_ULTRASONIC == 1
						ROBOT_LOG( TRUE,  "   %01u:ERR",nSensorNumber );	// Read Error
					#endif
	 				g_pFullSensorStatus->US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];
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

	 				//g_pFullSensorStatus->US[nSensorNumber] = g_RawArduinoStatus.US[nSensorNumber];	// For DEBUG
					g_pFullSensorStatus->US[nSensorNumber] = ScaleSRF04UltraSonic(g_RawArduinoStatus.US[nSensorNumber]);
					#if DEBUG_ULTRASONIC == 1
			//			ROBOT_LOG( TRUE,  "==%03u", g_pFullSensorStatus->US[nSensorNumber] );
					#endif

				}
				#if DEBUG_ULTRASONIC == 1
		//			ROBOT_LOG( TRUE,  "==%3u", g_pFullSensorStatus->US[nSensorNumber] );
				#endif
			}
			#if DEBUG_ULTRASONIC == 1
						ROBOT_LOG( TRUE,  "\n" );
			#endif

			#if DEBUG_ULTRASONIC == 1
				ROBOT_LOG( TRUE,  "US INCHES =  ");
				for( nSensorNumber=0; nSensorNumber< NUM_US_SENSORS; nSensorNumber++ )
				{
					ROBOT_LOG( TRUE,  "  %01u:%3u\"",nSensorNumber, g_pFullSensorStatus->US[nSensorNumber] );
				}
				ROBOT_LOG( TRUE,  "\n" ); //TenthInches???
			#endif

		#endif // ULTRASONICS_NOT_DISABLED
					
		//////////////////////////////////////////////////////////////////////////
		// Check Status Flags
		// See if Move completed
		//			if( 0 != (g_pFullSensorStatus->StatusFlags & HW_STATUS_MOVE_DISTANCE_COMPLETE) )
		//			{
		//				// Move completed, Arduino stopped motors and is awaiting next command
		//				m_pDriveCtrl->SetMoveDistanceCompleted( SENSOR_MODULE ); // Tell Motor control move is done
		//			}

		if( (g_pFullSensorStatus->StatusFlags & HW_STATUS_RC_BUTTON_PWR_ENABLE) )
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
		//ROBOT_LOG( MOTOR_DBG, "Status Flags = %02X\n", g_pFullSensorStatus->StatusFlags )


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


	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR FUSION
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void CSensorModule::DoSensorFusion()
	{
		// Combine any redundant sensors, and summarize results into g_pNavSensorSummary

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


		// Detailed Fused Sensor summary, used in cases where general info is not enough
		g_pNavSensorSummary->InitializeDefaults(); // Note, if sensor not installed, override with SENSOR_DISABLED
		g_pNavSensorSummary->bHWBumperFront = g_pFullSensorStatus->HWBumperFront; // for convenience make bumper available in summary

		// CENTER FRONT SENSORS - Combine center front facing US, IR and bumper info
		// Bumper collisions override other sensors
		if( g_pNavSensorSummary->BumperHitFront() )
		{		
			// Center Hit on Both HW bumpers
			g_pNavSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pNavSensorSummary->nFrontObjectDirection = OBJECT_EQUAL_DISTANCE;
			g_pNavSensorSummary->nLeftFrontZone = 0;
			g_pNavSensorSummary->nRightFrontZone = 0;
		}
		else if( HW_BUMPER_HIT_FRONT_LEFT )
		{		
			// Hit on Left HW bumper (mostly in front, but slightly to one side)
			g_pNavSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pNavSensorSummary->nFrontObjectDirection = FORWARD_LEFT;
			g_pNavSensorSummary->nLeftFrontZone = 0;
		}
		else if( HW_BUMPER_HIT_FRONT_RIGHT )
		{		
			// Hit on Right HW bumper
			g_pNavSensorSummary->nFrontObjectDistance = 0;	// Collision!
			g_pNavSensorSummary->nFrontObjectDirection = FORWARD_RIGHT;
			g_pNavSensorSummary->nRightFrontZone = 0;
		}
		else
		{
			// No Bumper collision, summarize other sensor info
			g_pNavSensorSummary->nLeftFrontZone = __min(
				g_pFullSensorStatus->IR[IR_SENSOR_FRONT_LEFT], g_pFullSensorStatus->US[US_SENSOR_FRONT_LEFT] );
			g_pNavSensorSummary->nRightFrontZone = __min(
				g_pFullSensorStatus->IR[IR_SENSOR_FRONT_RIGHT], g_pFullSensorStatus->US[US_SENSOR_FRONT_RIGHT] );

		}

		// ANGLED FRONT SENSORS
		// Carbot Has US angle sensors only.  No angled IR
		g_pNavSensorSummary->nLeftArmZone =  g_pFullSensorStatus->US[US_SENSOR_ANGLE_LEFT] ;
		g_pNavSensorSummary->nRightArmZone = g_pFullSensorStatus->US[US_SENSOR_ANGLE_RIGHT] ;


		// SIDE SENSORS
		g_pNavSensorSummary->nLeftSideZone = __min(
			g_pFullSensorStatus->IR[IR_SENSOR_SIDE_LEFT], g_pFullSensorStatus->US[US_SENSOR_SIDE_LEFT] );
		g_pNavSensorSummary->nRightSideZone = __min(
			g_pFullSensorStatus->IR[IR_SENSOR_SIDE_RIGHT], g_pFullSensorStatus->US[US_SENSOR_SIDE_RIGHT] );
		g_pNavSensorSummary->nSideObjectDistance = __min( 
			g_pNavSensorSummary->nLeftSideZone, g_pNavSensorSummary->nRightSideZone );


		// Now, check Camera sensor and override prior calculated values if needed
		// TODO-CAR-MUST!!! NEED TO SET THE SERVO VALUES BELOW TO CORRECT VALUES!
		if(	g_CameraTiltPos > (CAMERA_TILT_CENTER -15) )	// Must not be pointing at the ground! 
		{
			if( g_CameraPanPos < (CAMERA_PAN_CENTER -30) )	
			{
				// Camera pointing Left Side
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nLeftSideZone )
				{
					g_pNavSensorSummary->nLeftSideZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -20) )	
			{
				// Camera pointing Left Angle
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nLeftArmZone )
				{
					g_pNavSensorSummary->nLeftArmZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos < (CAMERA_PAN_CENTER -5) )	
			{
				// Camera pointing Left Front
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nLeftFrontZone )
				{
					g_pNavSensorSummary->nLeftFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +30) )	
			{
				// Camera pointing Right Side
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nRightSideZone)
				{
					g_pNavSensorSummary->nRightSideZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +20) )	
			{
				// Camera pointing Right Angle
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nRightArmZone )
				{
					g_pNavSensorSummary->nRightArmZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else if( g_CameraPanPos > (CAMERA_PAN_CENTER +5) )	
			{
				// Camera pointing Right Front
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nRightFrontZone )
				{
					g_pNavSensorSummary->nRightFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}
			else
			{
				// Camera sensor is pointing forward.  Override both front sensors if needed.
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nLeftFrontZone )
				{
					g_pNavSensorSummary->nLeftFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
				if( (g_pFullSensorStatus->US[US_SENSOR_CAMERA] + US_RANGE_FUDGE_AMOUNT) < g_pNavSensorSummary->nRightFrontZone )
				{
					g_pNavSensorSummary->nRightFrontZone = g_pFullSensorStatus->US[US_SENSOR_CAMERA];
				}
			}

		}


		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now summarize high level status

		if( 0 != g_pNavSensorSummary->nFrontObjectDistance )
		{
			// Skip this if a bumper was hit

			g_pNavSensorSummary->nFrontObjectDistance = __min( 
				g_pNavSensorSummary->nLeftFrontZone, g_pNavSensorSummary->nRightFrontZone );

			g_pNavSensorSummary->nFrontObjectDirection = GetObjectDirection(
				g_pNavSensorSummary->nLeftFrontZone, g_pNavSensorSummary->nRightFrontZone,
				IR_SENSOR_FRONT_LEFT, IR_SENSOR_FRONT_RIGHT, IR_LR_DETECT_RANGE, FORWARD_SENSOR_ANGLE ); // NOTE! Assumes *LONG RANGE IR* Sensors on front of robot!
		}

		// No Angle IR, so instead of calling GetObjectDirection(), just handle here
		if( abs(g_pNavSensorSummary->nRightArmZone - g_pNavSensorSummary->nLeftArmZone ) > US_RANGE_FUDGE_AMOUNT )
		{
			// Object clearly to one side or the other. 
			nAngleObjectDirection = ( g_pNavSensorSummary->nRightArmZone < g_pNavSensorSummary->nLeftArmZone ) ? ANGLE_SENSOR_ANGLE : (-ANGLE_SENSOR_ANGLE);
		}
		else
		{
			// Object pretty much dead ahead
			nAngleObjectDirection = OBJECT_EQUAL_DISTANCE;
		}

		// Now, combine angle and forward sensor info
		if( nAngleObjectDistance < g_pNavSensorSummary->nFrontObjectDistance )
		{
			// Closest Front object is at an angle
			g_pNavSensorSummary->nFrontObjectDistance = nAngleObjectDistance;
			g_pNavSensorSummary->nFrontObjectDirection = nAngleObjectDirection;
		}

		g_pNavSensorSummary->nSideObjectDirection = GetObjectDirection(
			g_pNavSensorSummary->nLeftSideZone, g_pNavSensorSummary->nRightSideZone,
			IR_SENSOR_SIDE_LEFT, IR_SENSOR_SIDE_RIGHT, IR_LR_DETECT_RANGE, SIDE_SENSOR_ANGLE ); // NOTE! Assumes *LONG RANGE IR* Sensors on side of robot!


	}



#endif // SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

#endif // ROBOT_SERVER	// This module used for Robot Server only