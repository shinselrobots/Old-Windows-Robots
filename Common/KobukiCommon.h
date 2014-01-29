// KobukiCommon.h
// Common defines used by Robot and KobukiControl process
// KobukiControl runs in it's own process, because driver only support non-debug
#ifndef __ROBOT_KOBUKI_SHARED_PARAMS_H__
#define __ROBOT_KOBUKI_SHARED_PARAMS_H__

#define KOBUKI_STATUS_SHARED_FILE_NAME	"RobotKobukiStatusMappingObject"
#define KOBUKI_COMMAND_SHARED_FILE_NAME	"RobotKobukiCommandMappingObject"

#define KOBUKI_STATUS_EVENT_NAME "RobotKobukiStatusEvent"
#define KOBUKI_COMMAND_EVENT_NAME "RobotKobukiCommandEvent"

//#define KOBUKI_MAX_NAME_LEN			 31
#define KOBUKI_REQUEST_BUFFER_SIZE  128
#define KOBUKI_RESPONSE_BUFFER_SIZE 128
 

//////////////////////////////////////////////////////////////////////////////////
// Messages from Robot app to Kobuki app
#ifndef ACCELERATION_SLOW
	#define ACCELERATION_SLOW		0
	#define ACCELERATION_MEDIUM		1
	#define ACCELERATION_FAST		2
	#define ACCELERATION_INSTANT	3
#endif

typedef struct
{
	//unsigned int			CommandType;
	//KOBUKI_COMMAND_PARAM_T  CommandParams;

	int				Speed;			// Meters per second
	int				Turn;			// Radians per second;  positive = turn left
	int				Acceleration;	// 0 = instant, 1 = fast, 2 = medium, 3 = slow
	unsigned int	DigitalIO;		// 4 bits
	unsigned int	LEDState;		// 4 bits
	unsigned int	ExternPower;	// 4 bits 
	unsigned int	bShutDown;		// Boolean - shut down if set

} KOBUKI_COMMAND_T;


//////////////////////////////////////////////////////////////////////////////////
// Status from Kobuki app to Robot app
  enum BatteryLevel {
    Dangerous = 0 ,
    Low,
    Healthy,
    Maximum
  };

  enum BatteryChargeSource {
    None = 0,
    Adapter,
    Dock
  };

  enum BatteryChargeState {
    Discharging = 0,
    Charged,
    Charging
  };

// General status info
typedef struct
{
	unsigned int	TimeStamp;	// when sensor data was reported
	unsigned int	Bumper;	// 
	unsigned int	WheelDrop;	// 
	unsigned int	Cliff;	// 
	unsigned int	LeftOdometer;	// 
	unsigned int	RightOdometer;	// 
	unsigned int	LeftPWM;	// 
	unsigned int	RightPWM;	// 
	unsigned int	ButtonPress;	// 
	unsigned int	ChargerState;	// 
	unsigned int	BatteryVoltage;	// 
	unsigned int	OverCurrentErrors;	// 
} KOBUKI_BASIC_SENSOR_T;


// Docking Station IR beacon detection
// 0x01 for NEAR_LEFT state
// 0x02 for NEAR_CENTER state
// 0x04 for NEAR_RIGHT state
// 0x08 for FAR_CENTER state
// 0x10 for FAR_LEFT state
// 0x20 for FAR_RIGHT state
typedef struct
{
	unsigned int	RightSignal;	//
	unsigned int	CenterSignal;	//
	unsigned int	LeftSignal;		//
} KOBUKI_DOCKING_IR_T;

// Inertial Sensor
typedef struct
{
	double	Angle;		//
	double	AngleRate;	//
} KOBUKI_INERTIAL_T;

// Cliff Sensor
typedef struct
{
	unsigned int	RightCliffA2D;	//
	unsigned int	CenterCliffA2D;	//
	unsigned int	LeftCliffA2D;	//
} KOBUKI_CLIFF_T;

// Wheel Current
typedef struct
{
	unsigned int	LeftMotor;	// in 10mA
	unsigned int	RightMotor;	// in 10mA
} KOBUKI_WHEEL_CURRENT_T;



// Not Implemented:
// General Purpose I/O pins
// Device Hardware, Firmware, version
// Device UUID



//////////////////////////////////////////////////
typedef struct
{
/*
	KOBUKI_BASIC_SENSOR_T	BasicSensor;
	KOBUKI_DOCKING_IR_T		DockingIR;
	KOBUKI_INERTIAL_T		Inertial;
	KOBUKI_CLIFF_T			CliffSensors;
	KOBUKI_WHEEL_CURRENT_T	WheelCurrent;
*/
	unsigned int	TimeStamp;				// when sensor data was reported
	double			GyroDegrees;			// 0-359 degrees of where Kobuki was powered up!
	double			TurnRate;				//

	int				OdometerTicksLeft;		// Raw Encoder ticks
	int				OdometerTicksRight;		// Raw Encoder ticks
	int				OdometerLeft;			// tenth inches - distance since power on
	int				OdometerRight;			// tenth inches - distance since power on

	double			BatteryVoltage;			// 
	double			BatteryPercent;			//
	unsigned int	BatteryLevelEnum;		// See enum BatteryLevel
	unsigned int	BatteryChargeSourceEnum;// See enum BatteryChargeSource
	unsigned int	BatteryChargeStateEnum;	// See enum BatteryChargeState

	unsigned int	Bumper;					// bit values:  Right = 1, Front = 2, Left = 4
	unsigned int	WheelDrop;				// bit values:  Right = 1, Left = 2
	unsigned int	Cliff;					// bit values:  Right = 1, Front = 2, Left = 4
	unsigned int	OverCurrentErrors;		// bit values:  Right = 1, Left = 2
	unsigned int	ButtonPress;			// bit values for button 1,2,3

	int				MotorPWMLeft;			// 
	int				MotorPWMRight;			// 
	unsigned int	MotorCurrentLeft;		// in 10mA
	unsigned int	MotorCurrentRight;		// in 10mA


	unsigned int	DockRightSignal;		//
	unsigned int	DockCenterSignal;		//
	unsigned int	DockLeftSignal;			//

	unsigned int	RightCliffA2D;			//
	unsigned int	CenterCliffA2D;			//
	unsigned int	LeftCliffA2D;			//


/* EACH SENSOR'S DOCK Flags will be set when signal is detected
0x01 for NEAR_LEFT state
0x02 for NEAR_CENTER state
0x04 for NEAR_RIGHT state
0x08 for FAR_CENTER state
ox10 for FAR_LEFT state
0x20 for FAR_RIGHT state	
*/


} KOBUKI_BASE_STATUS_T;
//////////////////////////////////////////////////


#endif // __ROBOT_KOBUKI_SHARED_PARAMS_H__