// DynamixelControl.h
// Interface for Dynamixel Digital Robot Servos
#pragma once
#include "Globals.h"
#include "ArmControl.h"
#include "HeadControl.h"

#define DYNA_ID_BROADCAST				0xFE	// Send message to ALL Dynamixel servos
#define DYNA_SINGLE_REGISTER			   1	// Number of BYTEs to read
#define DYNA_DOUBLE_REGISTER			   2	// Number of BYTEs to read
 
// Dynamixel Registers

//		Register Name					Reg Number
//===== ROM Registers.  Configure offline for servo startup.  Don't use during operation.
#define DYNA_ROM_MODEL_NUM					0x00	// 0
#define DYNA_ROM_FIRMWARE_VERS				0x02	// 2
#define DYNA_ROM_ID							0x03	// 3
#define DYNA_ROM_BAUD						0x04	// 4
#define DYNA_ROM_RETURN_DELAY				0x05	// 5
#define DYNA_ROM_CW_ANGLE_LIMIT				0x06	// 6
#define DYNA_ROM_CCW_ANGLE_LIMIT			0x08	// 8
#define DYNA_ROM_TEMPATURE_LIMIT			0x0B	//11
#define DYNA_ROM_LOW_VOLTAGE_LIMIT			0x0C	//12
#define DYNA_ROM_HIGH_VOLTAGE_LIMIT			0x0D	//13
#define DYNA_ROM_MAX_TORQUE_ROM				0x0E	//14	// don't use this position, use Ram instead!
#define DYNA_ROM_STATUS_RETURN_LEVEL		0x10	//16
#define DYNA_ROM_ALARM_LED					0x11	//17
#define DYNA_ROM_ALARM_SHUTDOWN				0x12	//18
#define DYNA_ROM_DOWN_CALIBRATION			0x14	//20
#define DYNA_ROM_UP_CALIBRATION				0x16	//22
//===== RAM Registers.  Set these during run-time ======
#define DYNA_REG_TORQUE_ENABLE				0x18	//24
#define DYNA_REG_LED						0x19	//25
#define DYNA_REG_CW_COMPLIANCE_MARGIN		0x1A	//26
#define DYNA_REG_CCW_COMPLIANCE_MARGIN		0x1B	//27
#define DYNA_REG_CW_COMPLIANCE_SLOPE		0x1C	//28
#define DYNA_REG_CCW_COMPLIANCE_SLOPE		0x1D	//29
#define DYNA_REG_GOAL_POSTION				0x1E	//30
#define DYNA_REG_MOVING_SPEED				0x20	//32
#define DYNA_REG_TORQUE_LIMIT				0x22	//34	- Use this to set torque
#define DYNA_REG_CURRENT_POSITION			0x24	//36
#define DYNA_REG_CURRENT_SPEED				0x26	//38
#define DYNA_REG_CURRENT_LOAD				0x28	//40
#define DYNA_REG_CURRENT_VOLTAGE			0x2A	//42
#define DYNA_REG_CURRENT_TEMPERATURE		0x2B	//43
#define DYNA_REG_REGISTERED_INSTRUCTION		0x2C	//44
#define DYNA_REG_MOVING						0x2E	//46
#define DYNA_REG_LOCK						0x2F	//47	// DONT USE THIS! REQUIRES POWER CYCLE TO CLEAR!
#define DYNA_REG_PUNCH						0x30	//48

// Controls how Dynamixels respond to instructions
#define DYNA_REPLY_NONE			// Dynamixel will never respond to command packets
#define DYNA_REPLY_READ_DATA	// Dynamixel will respond only to ReadData packets
#define DYNA_REPLY_ALL			// Dynamixel will respond to all command packets

#define DYNA_INSTRUCTION_PING			0x01 // Respond only with a status packet.
#define DYNA_INSTRUCTION_READ_DATA		0x02 // Read register data
#define DYNA_INSTRUCTION_WRITE_DATA		0x03 // Write register data
#define DYNA_INSTRUCTION_REG_WRITE		0x04 // Delay writing register data until an Action instruction is received
#define DYNA_INSTRUCTION_ACTION			0x05 // Perform pending RegWrite instructions
#define DYNA_INSTRUCTION_RESET			0x06 // Reset all registers (including ID) to default values
#define DYNA_INSTRUCTION_SYNC_WRITE		0x83 // Write register data to multiple Dynamixels at once

// Dynamixel Servo Limits
#define DYNA_SERVO_CENTER				0x1FF	//  511	
#define DYNA_ID_MAX						0x00FD	//  253
#define DYNA_ANGLE_LIMIT_MIN			0x0000	//    0	(Limit can be set 0-1023.  0=max CW (0 degrees).  150 degrees = centered
#define DYNA_ANGLE_LIMIT_MAX			0x03FF	// 1023 (Limit can be set 0-1023.  1023= max CCW (300 degrees)
#define DYNA_TEMPERATURE_LIMIT_MAX		0x0096	//  150 Degrees Celsius
#define DYNA_VOLTAGE_LIMIT_MIN			0x0032	//   50	( 5.0v)
#define DYNA_VOLTAGE_LIMIT_MAX			0x00FA	//  250 (25.0v)
#define DYNA_TORQUE_MAX					0x03FF	// 1023
#define DYNA_GOAL_POSITION_MAX			0x03FF	// 1023
#define DYNA_MOVING_SPEED_MAX			0x03FF	// 1023
//#define DYNA_TORQUE_LIMIT_MAX			0x03FF	// 1023 --> Moved to HardwareCmds.h
#define DYNA_PUNCH_MAX					0x03FF	// 1023


// Dynamixel Speed mappings
#define DYNA_VELOCITY_STOP					 0 
#define DYNA_VELOCITY_EXTREMELY_SLOW	     5
#define DYNA_VELOCITY_VERY_SLOW			    15
#define DYNA_VELOCITY_SLOW				    30
#define DYNA_VELOCITY_MED_SLOW			    60
#define DYNA_VELOCITY_MED				    80
#define DYNA_VELOCITY_MED_FAST			   200
#define DYNA_VELOCITY_FAST				   600
#define DYNA_VELOCITY_MAX				DYNA_MOVING_SPEED_MAX	//0x3FF = 1023


#define DYNA_MAX_DATA_BYTES		32	// Bytes in optional data field of command
#define DYNA_MAX_REPLY_BYTES	32	// Bytes in optional data field of reply
#define DYNA_CMD_HEADER_SIZE	 4	// Bytes in header: address, checksum, axis, command code
#define DYNA_READ_BUF_SIZE		64	// Size of buffer for reading messages from Dyna


/////////////////////////////////////////////////////////////////////////////
#pragma pack( 1 )	// don't let compiler pad between BYTEs

// command packet sent to Dynamixel servo:
// 0      1      2    3        4            4 + data-length
// [0xFF] [0xFF] [id] [length] [...data...] [checksum]
typedef struct DynaCmd_struct {
	BYTE Header1;
	BYTE Header2;
	BYTE ID;
	BYTE Length;
	BYTE Instruction;
	BYTE Data[DYNA_MAX_DATA_BYTES]; // (includes CheckSum after the data)
} DynaCmd_T;

// status packet returned from Dynamixel servo:
// 0      1      2    3        4       5            5 + data-length
// [0xFF] [0xFF] [id] [length] [error] [...data...] [checksum]
typedef struct DynaReply_struct {
	BYTE Header1;
	BYTE Header2;
	BYTE ID;
	BYTE Length;
	BYTE Error;
	BYTE Data[DYNA_MAX_REPLY_BYTES]; // (includes CheckSum after the data)
} DynaReply_T;

#pragma pack( 4 )


/////////////////////////////////////////////////////////////////////////////
class CDynaControl
{

public:
	CDynaControl();
	~CDynaControl();

void	Init();
void	HandleCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 );
void	PauseMotion( BOOL bPause ); // Handle global pause

// Make Private?
void	AddCheckSum( int  DataBytes );
void	GetStatus( int  ID );
BOOL	SendCmd( int  nParamBytes, int  nResponseBytes, int  nServoID, BOOL ResponseExpected = TRUE );
int 	GetResponseCheckSum( int  nResponseBytes );
char*	ServoIDToName( int  ServoID );
BOOL	DisplayServoError( int  ServoID, int  nError );
void	GotoSleepPosition();
void	GotoStartupPosition();
int		TicksToTenthDegree( int  ServoID, int  Ticks );
int 	TenthDegreeToTicks( int  ServoID, int TenthDegrees );
int 	DynaServoSpeedToServoSpeed( int  DynaServoSpeed );
int 	ServoSpeedToDynaServoSpeed( int  ServoSpeed );
double	CentigradeToFahrenheit( double Centigrade );
void	CheckServoLimit( int  ServoID, int  &PositionTicks );
void	PrepMultiServoData( int  ServoID, int  &nServoToSet, int  *ServoIDArray, int  *ServoValueArray, int  *ServoSpeedArray );
void	HandleRX64Servo( int  ServoID,  BOOL SetSpeed );


// Multi Servo Commands
void	EnableCameraServosTorque( BOOL bEnable );
void	EnableRightArmTorque( BOOL bEnable );
void	EnableLeftArmTorque( BOOL bEnable );
void	SetAllCameraServosSpeed( int  Speed );
void	SetAllCameraServosMaxTorque( int  MaxTorque );
void	GetAllCameraServosStatus();
void	SetCameraHeadPosition(int  HeadPanServo, int  HeadTiltServo, int  SideTiltServo );
void	SetMultiServoPosition(int  NumberOfServos, int  *ServoIDArray, int  *ServoValueArray );
void	SetMultiServoPositionAndSpeed(int  NumberOfServos, int  *ServoIDArray, int  *ServoValueArray,int  *ServoSpeedArray );


// Individual Servo Commands
void	SetServoSpeed( int  ServoID, int  Speed );
void	SetServoMaxTorque( int  ServoID, int  MaxTorque );
void	SetServoComplianceSlope( int  ServoID, int  ComplianceSlope );
void	EnableServoTorque( int  ServoID, BOOL bEnable );
void	SetServoPosition( int  ServoID, int  Position );
void	SetServoPositionAndSpeed( int  ServoID, int  Position, int  Speed );

void	SetServoLED( int  ServoID, BOOL State );
void	PingServo( int  ServoID );
void	SetShutdownConditions( int  ServoID, BYTE ShutdownFlags );
void	ChangeServoID( int  OldServoID, int  NewServoID );	// utility function only



// Servo Status Request Commands
int 	GetServoLoad( int  ServoID );
int 	GetServoTemperature( int  ServoID );
void	GetServoStatus( int  ServoID );	// Read all Current status registers

// Utility Functions
void	GetStatusOfMovingServos();
void	ReadServoRegisters(int  ServoID, int  Register, int  NumberOfRegisters);
void	WriteServoRegisterByte( int  ServoID, int  Register, int  Value );
void	WriteServoRegisterWord( int  ServoID, int  Register, int  Value );


protected:
	DynaCmd_T		m_DynaCmd;
	DynaReply_T		m_DynaReply;
	char			m_ReadBuf[DYNA_READ_BUF_SIZE];
	ArmControl		*m_pArmControlRight;
	ArmControl		*m_pArmControlLeft;
	HeadControl		*m_pHeadControl;
	DWORD			DynaCmdStartTime;
	BOOL			m_PowerEnabled;  // prevent servo motion when shut down (or in the process of shutting down)

	// Copy of Buld servo commands, to grab a "snapshot" when processing servo commands
	// Allows the global version (g_BulkServoCmd) to be modified while servos are being handled
	BULK_SERVO_CMD_T m_BulkServoCmdCopy[NUMBER_OF_SMART_SERVOS+1];	// (0 is "wasted space, but avoids common index error)

};
