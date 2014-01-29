////////////////////////////////////////////////////////////////////////////////////////
//Arduino SIMULATION for Robot


// TODO!!!!  change simulator to handle differential driver robot (turn in place)


#include "stdafx.h"
//#include "thread.h"
//#include "module.h"

#include "ClientOrServer.h"
#include "Globals.h"
#include "HWInterface.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define PIC_SIO_BUF_LEN		 128
#define GPS_BUF_LEN			 256
#define SERVO_SYNC_0		0xFF	// required sync for SSC II controller
//#define SERVO_SYNC_0		0x80	// required sync for Pololu controller
#define SERVO_CONTROLLER_ID	0x01	// only one controller attached

//#if ( ROBOT_SERVER == 1 )  // Nothing in this file used for client!


////////////////////////////////////
// global variables used by Arduino Simulation (need to be static)
	char	ResponseMsg[255];
	int		OdometerTickCount = 0;
	int		SpeedServo = 0;
	int		TurnServo = TURN_SERVO_CENTER;
	BOOL	MotorForward = TRUE;



#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT
	int		TempCompass = 0;	// Initialize at 0 (North)

#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE
	int		TempCompass = 0;	// Initialize at 0 (North)

#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP
	int		TempCompass = 0;	// Initialize at 0 (North)

#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI
	int		TempCompass = 180;	// Kludge - compass mouned backward!

#else
	#error BAD SENSOR_CONFIG_TYPE!
#endif




////////////////////////////////////


void SimulateHardware( DWORD Cmd, DWORD Param1, DWORD Param2 )
{
#define int8 BYTE
#if ( ROBOT_SERVER == 1 )  // Nothing in this file used for client!

	// Act as though the Arduino were connected!

	static int LoopCounter = 0;
	static boolean gRadarScanEnabled = FALSE;
	static boolean BrakePending = FALSE;


	switch( Cmd )
	{

		case HW_RESET_CPU:
		{
			// Reset!
			sprintf_s(ResponseMsg, "AK1:RESET!");
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_INITIALIZE:
		{
			// Reset servos to neutral positions, etc.
			sprintf_s(ResponseMsg, "AK1:INIT");
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_GET_STATUS:
		{

			// Simulate that motor control and Path start switch are pressed
			g_RawArduinoStatus.StatusFlags = 
				HW_STATUS_RC_BUTTON_PWR_ENABLE | HW_STATUS_RC_BUTTON_2;	

			g_RawArduinoStatus.LastError = 0;
			g_RawArduinoStatus.DebugCode = 0;

			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT

				g_RawArduinoStatus.Bumper = 0;

			#elif ( (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE) || (SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP) )
				g_RawArduinoStatus.IRBumper = 0;
				g_RawArduinoStatus.HWBumper = 0;

			#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI

				g_RawArduinoStatus.IRBumper =	IR_BUMPER_CLIFF_LEFT_MASK + IR_BUMPER_CLIFF_RIGHT_MASK +
											IR_BUMPER_PIR_LEFT_MASK + IR_BUMPER_PIR_RIGHT_MASK ;
				g_RawArduinoStatus.HWBumper = 0;

			#else
				#error BAD SENSOR_CONFIG_TYPE!  
			#endif
	
			//g_RawArduinoStatus.AckMotorCommand = 0;
			//g_RawArduinoStatus.AckMotorSpeed = SpeedServo;
			//g_RawArduinoStatus.AckMotorTurn = TurnServo;
			g_RawArduinoStatus.Battery0 = 230;
			//g_RawArduinoStatus.Battery1 = 130;
			g_RawArduinoStatus.CompassHigh = 0;
			g_RawArduinoStatus.CompassLow = 0;
			//g_RawArduinoStatus.OdometerHigh = 0;
			//g_RawArduinoStatus.OdometerLow = 0;
			//g_RawArduinoStatus.Tachometer = 0;					
//			g_RawArduinoStatus.US[0] = 125;	// Center sensor
/* DISABLED
			for( int i=0; i<9; i++ )
			{
				g_RawArduinoStatus.ThermalArray[i] = 0;
			}
*/
			// Insert fake values for testing sensor range displays
			// LongRange IR = 12 - 145 (Raw Value) = 133 spread
			// ShortRangeWide IR = 23 -156 (Raw Value) = 133 spread
			// EZ Max Ultrasonic = 92 - 142 (Raw Value) = 50 spread

			int i;
/* DISABLED
			for( i = 0; i < NUM_US_SENSORS; i++ )
			{
				if( (g_RawArduinoStatus.US[i])-- < 92 ) 
					g_RawArduinoStatus.US[i] = 142;
			}
*/
			for( i = 0; i < NUM_IR_SENSORS; i++ )
			{
				if( ((g_RawArduinoStatus.IR[i]) -= 3) < 25 ) 
					g_RawArduinoStatus.IR[i] = 145;
			}

			for( i=0; i<NUM_IR3_SENSORS; i++ )
			{
				g_RawArduinoStatus.IR3[i] = PIC_NO_OBJECT_IN_RANGE;
			}

			// g_RawArduinoStatus.US[0] = (rand() / (RAND_MAX / 50)) + 92;	
//			ROBOT_LOG( TRUE,  "DEBUG US0 = %d\n", g_RawArduinoStatus.US[0] )

			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_CARBOT
				g_RawArduinoStatus.AccelX = ACCEL_OFFSET_X;
				g_RawArduinoStatus.AccelY = ACCEL_OFFSET_Y;
			#endif

			// IR Sensor 0,5 = Side, Long Range
			// IR Sensor 1,4 = Forward, Wide Angle, short range
			// IR Sensor 2,3 = Forward, Long Range

			// FAKE REAL WORLD HERE!

			if( BrakePending )
			{
				// Last command was brake command.  Pretend it completed.
				g_RawArduinoStatus.StatusFlags |= HW_STATUS_BRAKE_COMPLETE; // Set Flag
				BrakePending = FALSE;
			}

			if( gRadarScanEnabled )
			{
				if( LoopCounter > 0 )
				{
					LoopCounter--;
				}
				else
				{
					LoopCounter = 10;
					// Simulate Radar ready

					// Send Scanning IR's Range data
					sprintf_s(ResponseMsg, "AK4");
					ResponseMsg[3] = 0;	// First (and only) Scanning Sensor	
					memset( &ResponseMsg[4], 0x40, US_SCAN_MAX_SAMPLES );
					//SendResponse(IR_SCAN_MAX_SAMPLES+4);
					//gStatus.StatusFlags &= (0xFF^HW_STATUS_IR_SCAN_READY); // Clear Flag
				}
			}			

			
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#if( MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL )

				if( (SPEED_STOP != SpeedServo) )
				{
					// For differential steering robots, negative speed just works
					OdometerTickCount += (SpeedServo/5); // BOGUS MOVEMENT
				}
				// differential steering robots can tun in place
				// use 0 - 255, with center at 127
					TempCompass += ((TurnServo) / 4);	// BOGUS TURN

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif( MOTOR_CONTROL_TYPE == PIC_CONTROLLER )

				if( (SPEED_STOP != SpeedServo) )
				{
					if( MotorForward )
					{
						OdometerTickCount += 6; // BOGUS MOVEMENT
						//g_RawArduinoStatus.Tachometer = SpeedServo - SPEED_STOP;
					}
					else
					{
						OdometerTickCount -= 4; // BOGUS MOVEMENT
						//g_RawArduinoStatus.Tachometer = SPEED_STOP - SpeedServo;
					}
					// Apply proper offset depending upon interface
					// Arduino uses 0 - 127, with center at 64
					TempCompass -= ((TurnServo-TURN_SERVO_CENTER) / 2);	// BOGUS TURN
				}


			//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			#elif( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
				// Use this if controlling motor via servos from the Pololu Servo Controller
				// Servo values range from 
				if( (SPEED_SERVO_CENTER != SpeedServo) )
				{
					if( MotorForward )
					{
						OdometerTickCount += 6; // BOGUS MOVEMENT
						//g_RawArduinoStatus.Tachometer = SpeedServo - SPEED_STOP;
					}
					else
					{
						OdometerTickCount -= 4; // BOGUS MOVEMENT
						//g_RawArduinoStatus.Tachometer = SPEED_STOP - SpeedServo;
					}
					// Apply proper offset depending upon interface
					// Pololu uses 0 - 255, with center at 127
					TempCompass -= ((TurnServo-TURN_SERVO_CENTER) / 8);	// BOGUS TURN
				}

			#elif( (MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL) )
				// Use this if controlling the motors from the Pololu TReX Motor Controller, or indirectly through the Arduino/Arduino!
				// with Arduino reporting the odometer

				// For differential steering robots, negative speed just works
				OdometerTickCount = (int)(((double)SpeedServo/4.0) * TICKS_PER_TENTH_INCH); // BOGUS MOVEMENT

				// differential steering robots can tun in place
				// use 0 - 255, with center at 127
				TempCompass += ((TurnServo) / 4);	// BOGUS TURN

			#elif( MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL )
				// Use this if using the iRobot Create base
				// TODO-TURTLE! fix these numbers!
				// For differential steering robots, negative speed just works
				OdometerTickCount = (int)(((double)SpeedServo/4.0) * TICKS_PER_TENTH_INCH); // BOGUS MOVEMENT

				// differential steering robots can tun in place
				// use 0 - 255, with center at 127
				TempCompass += ((TurnServo) / 4);	// BOGUS TURN

			#elif( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
				// Use this if using the iRobot Create base
				// TODO-TURTLE! fix these numbers!
				// For differential steering robots, negative speed just works
				OdometerTickCount = (int)(((double)SpeedServo/4.0) * TICKS_PER_TENTH_INCH); // BOGUS MOVEMENT

				// differential steering robots can tun in place
				// use 0 - 255, with center at 127
				TempCompass += ((TurnServo) / 4);	// BOGUS TURN

			#else
				#error BAD MOTOR_CONTROL_TYPE			
			#endif


			if( TempCompass >= 360 )
				TempCompass -= 360;
			if( TempCompass < 0 )
				TempCompass += 360;

			if( TempCompass > 360 )
			{
				ROBOT_LOG( TRUE,  "Arduino SIMULATION - Compass exceeds 360\n" )
				TempCompass = 0;
			}
			else if( TempCompass < 0)
			{
				ROBOT_LOG( TRUE,  "Arduino SIMULATION - Compass < 0\n" )
				TempCompass = 359;
			}


			#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL

				// For ER1 Arduino not used for Odometry
				PostThreadMessage( g_dwControlThreadId, 
					(WM_ROBOT_ER1_ODOMETER_READY), 
					(OdometerTickCount*TICKS_PER_TENTH_INCH), 
					(OdometerTickCount*TICKS_PER_TENTH_INCH) ); // Left and Right Motor values

			#elif MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL

				// For iRobot, Arduino not used for Odometry
				// TODO-TURTLE

			#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL

				// For Kobuki, Arduino not used for Odometry
				// TODO-TURTLE

			#elif( (MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL) || (MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL) )
				// Use this if controlling the motors from the Pololu TReX Motor Controller, or indirectly through the Arduino/Arduino!
				g_RawArduinoStatus.OdometerLowL =  (int8)(OdometerTickCount & 0xFF);
				g_RawArduinoStatus.OdometerHighL = (int8)(OdometerTickCount >>8);
				g_RawArduinoStatus.OdometerLowR =  (int8)(OdometerTickCount & 0xFF);
				g_RawArduinoStatus.OdometerHighR = (int8)(OdometerTickCount >>8);
				g_RawArduinoStatus.nOdometerSamples = 1;

			 #elif( MOTOR_CONTROL_TYPE == SERVO_MOTOR_CONTROL )
				// used for carbot
				g_RawArduinoStatus.OdometerLow =  (int8)(OdometerTickCount & 0xFF);
				g_RawArduinoStatus.OdometerHigh = (int8)(OdometerTickCount >>8);

			#else
				#error BAD MOTOR_CONTROL_TYPE			
			#endif

			g_RawArduinoStatus.CompassLow =  (int8)((TempCompass * 10) & 0x00FF);
			g_RawArduinoStatus.CompassHigh = (int8)((TempCompass * 10) >>8);
			//g_RawArduinoStatus.CompassHigh +=  (int8)(gCompassErrorCount <<4);// Put number of bad readings in the top nibble

			
			// Now throw message in the queue, to indicate that the status has been updated
			if(!g_PicFirstStatusReceived)
			{
				// Force status to be updated on the GUI, for example when client connects
				PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_SENSOR_STATUS_READY), 1, 1 ); // use 1 to force status updates every time!
				g_PicFirstStatusReceived = TRUE;
			}
			else
			{
				PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_SENSOR_STATUS_READY), 0, 0 );
			}

			break;
		}

		case HW_GET_RADAR_SCAN_DATA:
		{
			// NOT USED ANYMORE!
			// Send Scanning IR's Range data
			sprintf_s(ResponseMsg, "AK4");
			ResponseMsg[3] = 0;	// First (and only) Scanning Sensor	
			memset( &ResponseMsg[4], 0x40, US_SCAN_MAX_SAMPLES );
			//SendResponse(IR_SCAN_MAX_SAMPLES+4);
			//gStatus.StatusFlags &= (0xFF^HW_STATUS_IR_SCAN_READY); // Clear Flag
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_CLEAR_ERROR:
		{
			sprintf_s(ResponseMsg, "AK1:CLR_ERR");
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_GET_VERSION:
		{
			g_ArduinoSubSystemStatus = SUBSYSTEM_CONNECTED;	// Receipt of version is confirmation that we are connected.
			// Now throw message in the queue, to indicate that the version has been updated
			PostThreadMessage( g_dwSpeakThreadId, WM_ROBOT_SPEAK_TEXT, SPEAK_SIMULATION_MODE, 0 );  // speak we are in simulation mode
			//PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_PIC_VERSION_READY), 0, 99 );	// Bogus Version
			break;
		}

		case HW_RESET_ODOMETER:
		{
			sprintf_s(ResponseMsg, "AK1:RST ODOM");
			OdometerTickCount = 0;
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_SET_MOTOR_STOP:
		{
//			g_RawArduinoStatus.AckMotorCommand = Cmd;
			SpeedServo = SPEED_STOP;
			TurnServo = TURN_SERVO_CENTER;
			sprintf_s(ResponseMsg, "Simulator: AK1:STOP");
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}
		case HW_SET_MOTOR_BRAKE:
		{
//			g_RawArduinoStatus.AckMotorCommand = Cmd;
			SpeedServo = SPEED_STOP;
			TurnServo = TURN_SERVO_CENTER;
			sprintf_s(ResponseMsg, "Simulator: AK1:BRAKE");
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			//gStatus.StatusFlags &=  !HW_STATUS_BRAKE_COMPLETE; // Clear "Complete" Flag
			BrakePending = TRUE;
			break;
		}

//		case HW_SET_MOVE_DISTANCE:
		case HW_SET_SPEED_AND_TURN:
		{
//			g_RawArduinoStatus.AckMotorCommand = Cmd;
			int tempSpeed = (signed char)LOWORD(Param1); // Does not use acceleration parameter
			SpeedServo = tempSpeed;

			int tempTurn = (signed char)Param2;
			TurnServo = tempTurn;

			if( (SPEED_STOP != SpeedServo) )
			{
				// Set direction flag used by Odometer
				if( SpeedServo < SPEED_STOP )
					MotorForward = FALSE;
				else
					MotorForward = TRUE;
			}

			if( SPEED_STOP == Param1 )
			{
				sprintf_s(ResponseMsg, "Simulator HW_SET_SPEED_AND_TURN: AK1:Motor Stop, TURN %03d", 
					(Param2));
			}
			else
			{
				sprintf_s(ResponseMsg, "Simulator HW_SET_SPEED_AND_TURN: AK1:SPEED %03d, TURN %03d", 
					(LOWORD(Param1)-SPEED_STOP), (Param2));
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}


/*		case HW_SET_SPEED:
		{
			SpeedServo = Param1;
			if( (SPEED_STOP != SpeedServo) )
			{
				// Set direction flag used by Odometer
				if( SpeedServo < SPEED_STOP )
					MotorForward = FALSE;
				else
					MotorForward = TRUE;
			}
			if( SPEED_STOP == Param1 )
			{
				sprintf_s(ResponseMsg, "Simulator HW_SET_SPEED: AK1:MOTOR STOP" );
			}
			else
			{
				sprintf_s(ResponseMsg, "Simulator HW_SET_SPEED: AK1:SPEED %03d", (Param1-SPEED_STOP));
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}
*/
		case HW_SET_TURN:
		{
			// Handle turn value, passed in Param1
			TurnServo = Param1;	// Set servo to requested turn command
			sprintf_s(ResponseMsg, "Simulator HW_SET_TURN: AK1:TURN %03d", (Param1));
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}	

		case HW_ENABLE_RADAR_SCAN:
		{
			sprintf_s(ResponseMsg, "AK1:RADAR");

			if( 0 == Param2 )
			{
				gRadarScanEnabled = FALSE;
			}
			else
			{
				gRadarScanEnabled = TRUE;
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_SET_LIGHT_POWER:
		{
			if( 0 == Param2 )
			{
				sprintf_s(ResponseMsg, "AK1:TODO:Lights Off");
				//output_high( PIN_LIGHT_PWR );	// Lights Off
			}
			else
			{
				sprintf_s(ResponseMsg, "AK1:TODO:Lights On");
				// output_low( PIN_LIGHT_PWR );	// Lights On
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_SET_LED_EYES:
		{
			if( 1 == Param2 )
			{
				sprintf_s(ResponseMsg, "AK1:Eyes On");
			}
			else
			{
				sprintf_s(ResponseMsg, "AK1:Eyes Off");
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )
			break;
		}

		case HW_SET_SERVO_POWER:
		{
			if( 0 == Param2 )
			{
				sprintf_s(ResponseMsg, "AK1:Servo Power Off");
			}
			else
			{
				sprintf_s(ResponseMsg, "AK1:Servo Power On");
			}
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}

		case HW_SET_CAMERA_PAN_ABS:
		{
			sprintf_s(ResponseMsg, "AK1:CAM ABS PAN POS=%02X", Param1);
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}	
		case HW_SET_CAMERA_TILT_ABS:
		{
			sprintf_s(ResponseMsg, "AK1:CAM ABS TILT POS=%02X", Param1);
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}	

		case HW_SET_CAMERA_PAN_TILT:
		{
			// For slow pan or tilt.  Runs until stopped or Servo end reached.
			sprintf_s(ResponseMsg, "AK1:CAM Pan=%02X Speed=%02X", Param1, Param2);
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}	
		case HW_SET_CAMERA_MODE:
		{
			sprintf_s(ResponseMsg, "AK1:HW_SET_CAMERA_MODE Mode=%02X, Value=%02X", Param1, Param2);
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
			break;
		}
		case HW_UPDATE_MOTOR_SPEED_CONTROL:
		{
			break;
		}
		default:
		{
			sprintf_s(ResponseMsg, "Arduino SIM NAK:Unknown Cmd:%02X %02X %02X\n", Cmd, Param1, Param2);
			ROBOT_DISPLAY( TRUE, ResponseMsg )	// pretend we got a response
		}
	}	// end SWITCH

#endif	

}	// SimulateHardware

