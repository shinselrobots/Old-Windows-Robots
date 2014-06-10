// KobukiControl.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

//#ifdef _MSC_VER
#pragma warning( disable : 4244 ) // turn off annoying warnings in ecl and kobuki driver
//#endif
#include <csignal>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#pragma warning( default : 4244 ) /* Reset to default state */

#pragma warning( disable : 4231 ) // turn off annoying warnings in ecl and kobuki driver
#include "kobuki_driver/kobuki.hpp"
#pragma warning( default : 4231 ) /* Reset to default state */

#include "kobuki_driver\modules\digital_output.hpp"

#include <windows.h>
#include <tchar.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

#include "KobukiCommon.h"
#include "KobukiControl.h"


#ifdef _DEBUG
	#error - Kobuki Driver only has NON-DEBUG .dlls!
#endif

#define DEBUG_DISPLAY_SENSORS 0

//////////////////////////////////////////////////////////////////////////////////////////
// Constants
#define TicksPerMM 11.72441658029856624751591
#define TicksPerTenthInch (2.54 * TicksPerMM)

// Tune these values for desired speed ramps
#define ACCELERATION_INSTANT_RAMP_AMOUNT	0.00
#define ACCELERATION_FAST_RAMP_AMOUNT		0.03	
#define ACCELERATION_MEDIUM_RAMP_AMOUNT		0.02
#define ACCELERATION_SLOW_RAMP_AMOUNT		0.01


// Global to both KobukiManager and main - todo fix this
	LPCTSTR			 pStatusSharedMemory = NULL;
	HANDLE			 hStatusEvent = NULL;


//////////////////////////////////////////////////////////////////////////////////////////
class CKobukiManager {
public:
	CKobukiManager() :
				dx(0.0), dth(0.0),
				slot_stream_data(&CKobukiManager::processStreamData, *this)
	{
		kobuki::Parameters parameters;
		parameters.sigslots_namespace = "/kobuki";
		parameters.device_port = "COM4";
		parameters.enable_acceleration_limiter = false;

		/* THIS DOES NOT SEEM TO WORK!
		// Other options:                                       // Defaults
		parameters.enable_acceleration_limiter = true;			// false
		parameters.linear_acceleration_limit = 0.3;			// 0.3 default
		parameters.linear_deceleration_limit = (-0.3*1.2);	// (-0.3*1.2)
		parameters.angular_acceleration_limit = (3.5);		// (3.5);
		parameters.angular_deceleration_limit = (-3.5*1.2);	// (-3.5*1.2);
		*/

		cout << "Calling Kobuki.init -------------> Port = " << parameters.device_port << endl;
		kobuki.init(parameters);
		cout << "Calling Kobuki.enable " << endl;
		kobuki.enable();
		cout << "Connecting stream data " << endl;
		slot_stream_data.connect("/kobuki/stream_data");
		cout << "KobukiManager initialized " << endl;
	}

	~CKobukiManager() {
		kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
		kobuki.disable();
		kobuki.shutdown(); // shut down the slots thread
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Get status data to send to the Robot control app
	void processStreamData() 
	{
		// Don't send an update every time!
		static int UpdateCount = 0; 
		KOBUKI_BASE_STATUS_T Status;
		memset( &Status, 0x00, sizeof( KOBUKI_BASE_STATUS_T ) );

		//std::cout << "DBG : processStreamData." << std::endl;
		double Radians = kobuki.getHeading();
		double Degrees = 0;
		//cout << "DBG : Radians = " << Radians;
		if( Radians < 0 )
		{	// 0 - 179 degrees
			Degrees = Radians * -57.2957795;
		}
		else
		{	// 180 - 359 degrees 
			Degrees = 360 - (Radians * 57.2957795);
		}
		Status.GyroDegrees = Degrees;
		//cout << " Raw = " << Degrees << "  Gyro = " << Status.GyroDegrees <<  std::endl;

		Status.TurnRate = kobuki.getAngularVelocity();

		kobuki::Battery Battery = kobuki.batteryStatus();
		Status.BatteryPercent = Battery.percent(); 
		Status.BatteryVoltage = Battery.voltage;
		Status.BatteryLevelEnum = Battery.level();
		Status.BatteryChargeSourceEnum = Battery.charging_source;
		Status.BatteryChargeStateEnum = Battery.charging_state;

		// CoreSensors API
		kobuki::CoreSensors::Data CoreData = kobuki.getCoreSensorData();
		Status.TimeStamp = CoreData.time_stamp;
		//cout << std::dec << " TIME = " << Status.TimeStamp <<  endl;

		Status.OdometerTicksLeft = CoreData.left_encoder;
		Status.OdometerTicksRight = CoreData.right_encoder;
		Status.OdometerLeft = (int)( (float)CoreData.left_encoder / TicksPerTenthInch );
		Status.OdometerRight = (int)( (float)CoreData.right_encoder / TicksPerTenthInch );

		Status.Bumper = CoreData.bumper;
		if( Status.Bumper != 0 )
		{
			cout << std::hex << " Bumper = " << Status.Bumper <<  endl;
		}

		Status.WheelDrop = CoreData.wheel_drop;
		if( Status.WheelDrop != 0 )
		{
			cout << std::hex << " Wheel Drop = " << Status.WheelDrop <<  endl;
		}

		Status.Cliff = CoreData.cliff;
		if( Status.Cliff != 0 )
		{
			cout << std::hex << " Cliff = " << Status.Cliff <<  endl;
		}


		// General Purpose IO Ports (25 pin connector on Kobuki) values from 0 to 4095
		kobuki::GpInput::Data GpInput = kobuki.getGpInputData();
		for( int i=0; i<4; i++ )
		{
			Status.AnalogPort[i] = GpInput.analog_input[i];
			 //cout << dec << "   IR" << i << ": " << setw(4) << Status.AnalogPort[i]; // debug IR sensors
		}
		 //cout << endl; // Debug IR sensors


		kobuki::DockIR::Data DockIR = kobuki.getDockIRData();
	
		Status.DockRightSignal = DockIR.docking[0];
		Status.DockCenterSignal = DockIR.docking[1];
		Status.DockLeftSignal = DockIR.docking[2];

		// display message if dock is in view
		if( (Status.DockRightSignal != 0) || (Status.DockCenterSignal != 0) || (Status.DockLeftSignal != 0) )
		{
			// Documentation is from the perspective of the DOCK.
			// I want perspective of the ROBOT (swap Right/Left)

			if( Status.DockCenterSignal & 0x01)	cout << " NR ";	
			else cout << " -- ";

			if( Status.DockCenterSignal & 0x02)	cout << " NC ";	
			else cout << " -- ";

			if( Status.DockCenterSignal & 0x04)	cout << " NL ";	
			else cout << " -- ";

			cout << "    ";

			if( Status.DockCenterSignal & 0x10)	cout << " FR ";	
			else cout << " -- ";

			if( Status.DockCenterSignal & 0x08)	cout << " FC ";	
			else cout << " -- ";

			if( Status.DockCenterSignal & 0x20)	cout << " FL ";	
			else cout << " -- ";

			cout << "    ";
			cout << std::hex << " Dock R = " << Status.DockRightSignal;
			cout << std::hex << " Dock C = " << Status.DockCenterSignal;
			cout << std::hex << " Dock L = " << Status.DockLeftSignal <<  endl;

		}

		
		#if ( DEBUG_DISPLAY_SENSORS  == 1 )

			cout << std::hex << " Dock R = " << Status.DockRightSignal;
			cout << std::hex << " Dock C = " << Status.DockCenterSignal;
			cout << std::hex << " Dock L = " << Status.DockLeftSignal <<  endl;


			cout << "Battery: " <<  Status.BatteryPercent << "% " << Status.BatteryVoltage << "v  - Level: ";

			switch( Status.BatteryLevelEnum )
			{
				case 0:
					cout << "Dangerous"; break;
				case 1:
					cout << "Low";	break;
				case 2:
					cout << "Healthy";	break;
				case 3:
					cout << "Maximum"; break;
				default:
					cout << "Level Enum Error";
			}

			cout << " Source: ";
			switch( Status.BatteryChargeSourceEnum )
			{
				case 0:
					cout << "not charging"; break;
				case 1:
					cout << "Adapter plug";	break;
				case 2:
					cout << "Dock charging";	break;
				default:
					cout << "Charge Enum Error";
			}

			cout << " State: ";
			switch( Status.BatteryChargeStateEnum )
			{
				case 0:
					cout << "Discharging"; break;
				case 1:
					cout << "Charged";	break;
				case 2:
					cout << "Charging";	break;
				default:
					cout << "Level Enum Error";
			}

			cout << endl;
			//cout << setw(6);
			cout << setiosflags(ios::fixed) << setprecision(2);
			cout << "Compass: " << Status.Compass << "  TurnRate: " << Status.TurnRate;	cout << endl;
			cout << "Odom Left: " << Status.OdometerLeft << "  Right: " <<  Status.OdometerRight << std::endl;
			cout << endl;
		#endif // DEBUG_DISPLAY_SENSORS
		/*
		ecl::Angle<double> getHeading() const;
		double getAngularVelocity() const;
		VersionInfo versionInfo() const { return VersionInfo(firmware.data.version, hardware.data.version, unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2); }
		Battery batteryStatus() const { return Battery(core_sensors.data.battery, core_sensors.data.charger); }

		ecl::Pose2D<double> pose_update;
		ecl::linear_algebra::Vector3d pose_update_rates;
		kobuki.updateOdometry(pose_update, pose_update_rates);
		pose *= pose_update;
		dx += pose_update.x();
		dth += pose_update.heading();
		*/
		// std::cout << "processStreamData: Pose dx = " << dx << ", dth =" << dth << std::endl;

		//processMotion();


		//////////////////////////////////////////////////////////////////////////////
		// Send data to Robot:
		if( (NULL != pStatusSharedMemory) && (NULL != hStatusEvent) )
		{
			CopyMemory( (PVOID)pStatusSharedMemory, &Status, (sizeof(KOBUKI_BASE_STATUS_T)) );
			SetEvent( hStatusEvent ); // Indicate that new data is available
		}
		//////////////////////////////////////////////////////////////////////////////


	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Move / Turn commands
	// void setBaseControl(const double &linear_velocity, const double &angular_velocity);
	void Move( double &linear_velocity, double &angular_velocity ) 
	{
		std::cout << "Sending Move Command  " << linear_velocity << ", " << angular_velocity << std::endl;
		kobuki.setBaseControl(linear_velocity, angular_velocity); 
	}


	//////////////////////////////////////////////////////////////////////////////////////////
	ecl::Pose2D<double> getPose() {
		return pose;
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Set External Power for 4 pins
	// Mask allows setting independently if desired
	// 0x01 for external power 3.3V 
	// 0x02 for external power 5V 
	// 0x04 for external power 12V/5A  - Servos
	// 0x08 for external power 12V/1.5A  - Kinect
	void SetPower( unsigned int ExternPower )
	{
		kobuki::DigitalOutput digital_output;
		for ( unsigned int i = 0; i < 4; ++i ) 
		{
			digital_output.mask[i] = true;
		}
		digital_output.values[0] = ((ExternPower & 0x01) == 1); // 3.3v
		digital_output.values[1] = ((ExternPower & 0x02) == 1); // 5v
		digital_output.values[2] = ((ExternPower & 0x04) == 1); // 12v, 5A - Servos
		digital_output.values[3] = ((ExternPower & 0x08) == 1); // 12v, 1.5A - Kinect

		std::cout << "Setting External Power to  " << std::hex << std::uppercase << ExternPower << std::endl;

		//kobuki.setExternalPower( digital_output );  // Causes NUC POWER TO SHUT DOWN!?! Maybe gliches the power line?
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Set the output pins of the expansion port
	// 0x01 for digital output ch. 0
	// 0x02 for digital output ch. 1
	// 0x04 for digital output ch. 2
	// 0x08 for digital output ch. 3
	void SetDigitalIO( unsigned int PinState )
	{
		kobuki::DigitalOutput digital_output;
		for ( unsigned int i = 0; i < 4; ++i ) 
		{
			digital_output.mask[i] = true;
		}
		digital_output.values[0] = ((PinState & 0x01) == 1);
		digital_output.values[1] = ((PinState & 0x02) == 1);
		digital_output.values[2] = ((PinState & 0x04) == 1);
		digital_output.values[3] = ((PinState & 0x08) == 1);

		std::cout << "Setting IO Pins to  " << std::hex << std::uppercase << PinState << std::endl;
		kobuki.setDigitalOutput( digital_output );
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	// Set LED State
	// Set the flags to turn on LEDs
	// 0x01 for red colour of LED1
	// 0x02 for green colour of LED1
	// 0x04 for red colour of LED2
	// 0x08 for green colour of LED2	
	void SetLedState( unsigned int LedState )
	{
		if(LedState & 0x01)
		{
			cout << "LED1 = Red" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Red );
		}
		else if(LedState & 0x02)
		{
			cout << "LED1 = Red" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Green );
		}
		else
		{
			cout << "LED1 = Off" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Black );
		}

		if(LedState & 0x04)
		{
			cout << "LED2 = Red" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Red );
		}
		else if(LedState & 0x08)
		{
			cout << "LED2 = Red" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Green );
		}
		else
		{
			cout << "LED2 = Off" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Black );
		}

	}


	//////////////////////////////////////////////////////////////////////////////////////////
	// Set the flags to set high on output pins of the expansion port
	// 0x01 for digital output ch. 0
	// 0x02 for digital output ch. 1
	// 0x04 for digital output ch. 2
	// 0x08 for digital output ch. 3
	void SetExternalIO( unsigned int IOState )
	{
		if(IOState & 0x01)
		{
			cout << "IO 0x01 = Red" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Red );
		}
		else if(IOState & 0x02)
		{
			cout << "IO 0x01 = Red" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Green );
		}
		else
		{
			cout << "IO 0x01 = Off" << endl;
			kobuki.setLed( kobuki::Led1, kobuki::Black );
		}

		if(IOState & 0x04)
		{
			cout << "IO 0x01 = Red" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Red );
		}
		else if(IOState & 0x08)
		{
			cout << "IO 0x01 = Red" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Green );
		}
		else
		{
			cout << "IO 0x01 = Off" << endl;
			kobuki.setLed( kobuki::Led2, kobuki::Black );
		}
	}

	//void kobuki::Kobuki::setDigitalOutput  ( const DigitalOutput &  digital_output ) 


	//////////////////////////////////////////////////////////////////////////////////////////
	// Data members
	private:
	double dx, dth;
	ecl::Pose2D<double> pose;
	kobuki::Kobuki kobuki;
	ecl::Slot<> slot_stream_data;

};  // Class CKobukiManager



//////////////////////////////////////////////////////////////////////////////////////////
// Signal Handler
//////////////////////////////////////////////////////////////////////////////////////////

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}


//////////////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////////////


int _tmain(int argc, _TCHAR* argv[])
//int main(int argc, char** argv)
{

	#if (DEBUG_MEMORY_LEAKS == 1 )
		_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	// Use to detect memory leaks!
	// _CrtSetBreakAlloc( Put allocator number here);

	// Global Variables
	LPCTSTR pCommandSharedMemory = NULL;
	HANDLE hCommandEvent = NULL;

	KOBUKI_COMMAND_T *LastCommand = new KOBUKI_COMMAND_T;
	memset( LastCommand, 0, sizeof(KOBUKI_COMMAND_T) );

	// for Smooth acceleration
	double TargetSpeed = 0.0;
	double TargetTurn = 0.0;
	double CurrentSpeed = 0.0;
	double CurrentTurn = 0.0;
	int    TargetAcceleration = 0;
	int    CurrentAcceleration = 0;
	double AccelerationRamp = 0.0;


	signal(SIGINT, signalHandler);
	std::cout << "Starting Kobuki Base Control..." << std::endl;

	std::cout << "Initializing shared memory..." << std::endl;

	// Initialize shared memory and events for communicating with the Robot control application
	int InitIPCResult = InitIPC( hCommandEvent, pCommandSharedMemory, hStatusEvent, pStatusSharedMemory  );

	if ( FAILED == InitIPCResult )
	{
		cerr << "InitIPC failed.  Exiting" << endl;
		return -1;
	}

	signal(SIGINT, signalHandler);

	std::cout << "Starting Kobuki Driver Manager..." << std::endl;

	CKobukiManager KobukiManager;
	std::cout << "KobukiManager initialized..." << std::endl;

	ecl::Sleep sleep(1);

/***
	// Open log file
	//#ifndef _DEBUG // optionally, only open the log file in Release mode
	errno_t nError = fopen_s( &g_LogFile, LogFileName, "w" );
	if( 0 == nError )
	{
		fprintf(g_LogFile, "STARTING LOG FILE.  Start Time: %s\n\n", COleDateTime::GetCurrentTime().Format() );
	}
	else
	{
		g_LogFile = NULL;
		CString ErrorStr;
		ErrorStr.Format( _T("Can't open Log File\n %s Error = %d"), LogFileName, nError );
		AfxMessageBox( ErrorStr );
	}
	//#endif
***/

	cout << "Robot Kobuki Base Control Starting..." << endl << endl;

	Sleep(100); // let robot code start up first

	cout << "Turning on external power by default..." << endl << endl;
	KobukiManager.SetPower( 0xFF );

	///////////////////////////////////////////////////////////////////////////////////
	// Command and Status Processing Loop.  
	std::cout << "Beginning Control Loop..." << std::endl;
	try 
	{
		int bRunLoop = 1;
		while( bRunLoop )
		{	
			const DWORD msTimeOut = 10;  // Sleep time.  
			if( STAND_ALONE_MODE != InitIPCResult )
			{
				DWORD dwResult = WaitForSingleObject(hCommandEvent, msTimeOut);
				if( WAIT_OBJECT_0 == dwResult ) 
				{
					// Event did not time out.  That means the Robot has posted a command to shared memory
					cout << "Command Received: " << endl;
					// Read from Shared Memory
					KOBUKI_COMMAND_T *Command = (KOBUKI_COMMAND_T*)pCommandSharedMemory;

					// Process the command block

					if( 0 != Command->bShutDown )
					{
						// Exit application.  Kobuki destructor automatically stops motors
						cout << "KOBUKI_COMMAND_SHUT_DOWN received" << endl;
						bRunLoop = 0;
						continue;
					}

					if( (Command->Speed != LastCommand->Speed) || (Command->Turn != LastCommand->Turn) )
					{
						// Move command received
						cout << "KOBUKI COMMAND MOVE: Speed = " << Command->Speed << " Turn = " << Command->Turn << " Acc = " << Command->Acceleration << endl;
						TargetSpeed = (double)Command->Speed / 100; // scale from centermeters / sec --> meters/sec
						TargetTurn = (double)Command->Turn / 100; // scale from 1/100 rad / sec --> rad/sec

						// Set Acceleration Ramp
						switch( Command->Acceleration )  
						{
							case ACCELERATION_INSTANT:
								AccelerationRamp = ACCELERATION_INSTANT_RAMP_AMOUNT;
								break;
							case ACCELERATION_FAST:
								AccelerationRamp = ACCELERATION_FAST_RAMP_AMOUNT;
								break;
							case ACCELERATION_MEDIUM:
								AccelerationRamp = ACCELERATION_MEDIUM_RAMP_AMOUNT;
								break;
							case ACCELERATION_SLOW:
								AccelerationRamp = ACCELERATION_SLOW_RAMP_AMOUNT;
								break;
						}

						LastCommand->Speed = Command->Speed;
						LastCommand->Turn = Command->Turn;
						LastCommand->Acceleration = Command->Acceleration;
					}

					if( Command->ExternPower != LastCommand->ExternPower )
					{
						cout << "KOBUKI COMMAND EXTERNAL POWER = " << Command->ExternPower << endl;
						cout << "   Kinect Power = " << ((Command->ExternPower & 0x04) == 1)  << endl;
						cout << "   Servo Power =  " << ((Command->ExternPower & 0x08) == 1)  << endl;
						KobukiManager.SetPower( Command->ExternPower );
						LastCommand->ExternPower = Command->ExternPower;
					}

					if( Command->DigitalIO != LastCommand->DigitalIO )
					{
						cout << "KOBUKI COMMAND DIGITAL I/O = " << Command->DigitalIO << endl;
						KobukiManager.SetExternalIO( Command->DigitalIO );
						LastCommand->DigitalIO = Command->DigitalIO;
					}

					if( Command->LEDState != LastCommand->LEDState )
					{
						cout << "KOBUKI COMMAND LED STATE = " << Command->LEDState << endl;
						KobukiManager.SetLedState( Command->LEDState );
						LastCommand->LEDState = Command->LEDState;
					}
				}
			}
			else
			{
				// Stand Alone mode - just show status for debug testing?
			}

			// Do Acceleration Ramp as needed
			//const double SpeedDelta = 0.02;
			//const double TurnDelta = 0.02;
			if( (CurrentSpeed != TargetSpeed) || (CurrentTurn !=  TargetTurn) )
			{
				// Need to adjust speed/turn.  Calculate the amount
				cout << "Speed Ramp: Target = " << TargetSpeed << " Current = " << CurrentSpeed << endl;
				if( ACCELERATION_INSTANT_RAMP_AMOUNT == AccelerationRamp )
				{
					// Instant response required (such as emergency stop)
					CurrentSpeed = TargetSpeed;
				}
				else
				{
					if( CurrentSpeed < TargetSpeed )
					{	// Need to speed up
						CurrentSpeed += AccelerationRamp;
						if( CurrentSpeed > TargetSpeed )
						{	// overshot
							CurrentSpeed = TargetSpeed; // at target 
						}
					}
					else if( CurrentSpeed > TargetSpeed )
					{	// need to slow down
						CurrentSpeed -= AccelerationRamp;
						if( CurrentSpeed < TargetSpeed )
						{	// overshot
							CurrentSpeed = TargetSpeed; // at target 
						}
					}
				}
				if( CurrentTurn != TargetTurn )
				{
					CurrentTurn = TargetTurn; // No ramp for turns
				}

				// Tell the base to move at this new speed
				KobukiManager.Move(CurrentSpeed, CurrentTurn );

			}


			// TODO - Get Status
			//ecl::Pose2D<double> pose;
			//pose = KobukiManager.getPose();
			//std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;

			//TODO - Put status into shared memory

			Sleep(50);  // tune this to adjust update rate and avoid over saturating the CPU

		} // Command and Status Processing Loop

	} // try
	catch ( ecl::StandardException &e ) 
	{
		std::cout << e.what();
	}


  std::cout << "Exiting..." << std::endl;
  // TODO  - DISABLED FOR NOW.  Turns off the display, and it does not turn back on!
  //  KobukiManager.SetPower( 0 ); // Turn off all external power (Kinect, Servos, etc.)
  SAFE_DELETE(LastCommand);

  Sleep(1000); // for debugging
  return 0;

}

