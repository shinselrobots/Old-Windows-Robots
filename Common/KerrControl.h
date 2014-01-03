// KerrControl.h
// Interface for Kerr Servo Controller (used in robot Shoulder Joints)
// See PicSrvSc.pdf for details and www.jrkerr.com

#pragma once
#include "RobotType.h"
#include "ArmControl.h"


//Servo Module Command set:			CMD		   #Bytes	Comments
#define KERR_CMD_RESET_POS			0x00	// 0-5		Reset encoder counter to 0 (0 bytes)
#define KERR_CMD_SET_ADDRESS		0x01	// 2		Set address and group address (2 bytes)
#define KERR_CMD_DEFINE_STATUS		0x02	// 1		Define status items to return (1 byte)
#define KERR_CMD_READ_STATUS		0x03	// 1		Read value of current status items
#define KERR_CMD_LOAD_TRAJECTORY	0x04	// 1-14		Load trahectory date (1 - 14 bytes)
#define KERR_CMD_START_MOVE			0x05	// 0		Start pre-loaded trajectory (0 bytes)
#define KERR_CMD_SET_GAIN			0x06	// 15		Set servo gain and control parameters (13 or 14)
#define KERR_CMD_STOP_SERVO			0x07	// 1,5		Stop Servo OR SET ENABLE! (1 byte)
#define KERR_CMD_IO_CTRL			0x08	// 1		Define bit directions and set output (1 byte)
#define KERR_CMD_SET_HOMING			0x09	// 1		Define homing mode (1 byte)
#define KERR_CMD_SET_BAUD			0x0A 	// 1		Set the baud rate (1 byte)
#define KERR_CMD_CLEAR_BITS			0x0B	// 0		Save current pos. in home pos. register (0 bytes)
#define KERR_CMD_SAVE_AS_HOME		0x0C	// 0		Store the current home position in home position register
#define KERR_CMD_ADD_PATHPOINT		0x0D	// 0-14		Adds path points for path mode
#define KERR_CMD_NOP				0x0E	// 0		No operation - returns prev. defined status (0 bytes)
#define KERR_CMD_HARD_RESET			0x0F	// 0,1		RESET - no status is returned

//Servo Module RESET_POS control byte bit definitions:
//(if no control byte is used, reset is absolute)
#define KERR_REL_HOME			0x01	//Reset position relative to current home position
#define KERR_SET_POS			0x02	//Set the position to a specific value (v10 & >)

//Servo Module STATUSITEMS bit definitions (for DEF_STAT and READ_STAT):
#define KERR_GET_POSITION		0x01	//4 bytes data
#define KERR_GET_LOAD_CURRENT	0x02	//1 byte
#define KERR_GET_VELOCITY		0x04	//2 bytes	- Not Used by Loki
#define KERR_GET_AUX			0x08	//1 byte	- Not Used by Loki
#define KERR_GET_HOME_POS		0x10	//4 bytes	- Not Used by Loki
#define KERR_GET_DEVICE_TYPE	0x20	//2 bytes	- Not Used by Loki
#define KERR_GET_POS_ERROR		0x40	//2 bytes	- Not Used by Loki
#define KERR_GET_NPOINTS		0x80	//1 byte	- Not Used by Loki

//Servo Module LOAD_TRAJ control byte bit definitions:
#define KERR_LOAD_POS			0x01	//+4 bytes
#define KERR_LOAD_VELOCITY		0x02	//+4 bytes
#define KERR_LOAD_ACC			0x04	//+4 bytes
#define KERR_LOAD_PWM			0x08	//+1 byte
#define KERR_ENABLE_SERVO		0x10	//1 = servo mode, 0 = PWM mode
#define KERR_VEL_MODE			0x20	//1 = velocity mode, 0 = trap. position mode
#define KERR_REVERSE			0x40	//1 = command neg. PWM or vel, 0 = positive
#define KERR_MOVE_REL			0x40	//1 = move relative, 0 = move absolute
#define KERR_START_NOW			0x80	//1 = start now, 0 = wait for START_MOVE command

//Servo Module STOP_Servo control byte bit definitions:
#define KERR_AMP_ENABLE			0x01	//1 = raise amp enable output, 0 = lower amp enable
#define KERR_SERVO_OFF	  		0x02	//set to turn Servo off
#define KERR_STOP_ABRUPT   		0x04	//set to stop Servo immediately
#define KERR_STOP_SMOOTH	  	0x08	//set to decellerate Servo smoothly
#define KERR_STOP_HERE	  		0x10	//set to stop at position (4 add'l data bytes required)
#define KERR_ADV_FEATURE   		0x20	//enable features in ver. CMC

//Servo Module IO_CTRL control byte bit definitions:
#define KERR_SET_OUT1	  		0x01	//1 = set limit 1 output, 0 = clear limit 1 output
#define KERR_SET_OUT2	  		0x02	//1 = set limit 2 output, 0 = clear limit 1 output
#define KERR_IO1_IN		  		0x04	//1 = limit 1 is an input, 0 = limit 1 is an output
#define KERR_IO2_IN		  		0x08	//1 = limit 2 is an input, 0 = limit 1 is an output
#define KERR_LIMSTOP_OFF   		0x04	//turn off Servo on limit
#define KERR_LIMSTOP_ABRUPT 	0x08	//stop abruptly on limit
#define KERR_THREE_PHASE	  	0x10	//1 = 3-phase mode, 0 = single PWM channel
#define KERR_ANTIPHASE	  		0x20	//1 = antiphase (0 = 50% duty cycle), 0 = PWM & dir
#define KERR_FAST_PATH	  		0x40	//0 = 30 or 60 Hz path execution, 1 = 60 or 120 Hz
#define KERR_STEP_MODE	  		0x80	//0 = normal operation, 1 = Step & Direction enabled

//Servo Module SET_HOMING control byte bit definitions:
#define KERR_ON_LIMIT1	  		0x01	//home on change in limit 1
#define KERR_ON_LIMIT2	  		0x02	//home on change in limit 2
#define KERR_HOME_SERVO_OFF  	0x04	//turn Servo off when homed
#define KERR_ON_INDEX	  		0x08	//home on change in index
#define KERR_HOME_STOP_ABRUPT 	0x10	//stop abruptly when homed
#define KERR_HOME_STOP_SMOOTH 	0x20	//stop smoothly when homed
#define KERR_ON_POS_ERR	  		0x40	//home on excessive position error
#define KERR_ON_CUR_ERR	  		0x80	//home on overcurrent error

//Servo Module ADD_PATHPOINT frequency definitions
#define KERR_P_30HZ				 30		//30 hz path resolution
#define KERR_P_60HZ				 60		//60 hs path resolution
#define KERR_P_120HZ			120		//120 hs path resolution

//Servo Module HARD_RESET control byte bit definitions (v.10 and higher only):
#define KERR_SAVE_DATA	  		0x01	//save config. data in EPROM
#define KERR_RESTORE_ADDR  		0x02	//restore addresses on power-up
#define KERR_EPU_AMP	      	0x04  	//enable amplifier on power-up
#define KERR_EPU_SERVO     		0x08	//enable servo
#define KERR_EPU_STEP	  		0x10	//enable step & direction mode
#define KERR_EPU_LIMITS    		0x20	//enable limit switch protection
#define KERR_EPU_3PH	      	0x40	//enable 3-phase commutation
#define KERR_EPU_ANTIPHASE 		0x80	//enable antiphase PWM

//Servo Module Status byte bit definitions:
#define KERR_MOVE_DONE	  		0x01	//set when move done (trap. pos mode), when goal
										//vel. has been reached (vel mode) or when not servoing
#define KERR_CKSUM_ERROR	  	0x02	//checksum error in received command
#define KERR_OVERCURRENT	  	0x04	//set on overcurrent condition (sticky bit)
#define KERR_POWER_ON	  		0x08	//set when Servo power is on
#define KERR_POS_ERR		  	0x10	//set on excess pos. error (sticky bit)
#define KERR_LIMIT1		  		0x20	//value of limit 1 input
#define KERR_LIMIT2		  		0x40	//value of limit 2 input
#define KERR_HOME_IN_PROG  		0x80	//set while searching for home, cleared when home found

//Servo Module Auxilliary status byte bit definitions:
#define KERR_INDEX		  		0x01	//value of the encoder index signal
#define KERR_POS_WRAP	  		0x02	//set when 32 bit position counter wraps around
										//  (sticky bit)
#define KERR_SERVO_ON	  		0x04	//set when position servo is operating
#define KERR_ACCEL_DONE	  		0x08	//set when acceleration portion of a move is done
#define KERR_SLEW_DONE	  		0x10	//set when slew portion of a move is done
#define KERR_SERVO_OVERRUN 		0x20	//set if servo takes longer than the specified
										//servo period to execute
#define KERR_PATH_MODE	  		0x40	//path mode is enabled (v.5)


// Kerr Speed mappings
#define KERR_VELOCITY_STOP					      0	//
#define KERR_VELOCITY_VERY_SLOW				 100000	//
#define KERR_VELOCITY_SLOW					 300000	//
#define KERR_VELOCITY_MED_SLOW				 700000	//
#define KERR_VELOCITY_MED					1200000	//
#define KERR_VELOCITY_MED_FAST				2000000	//
#define KERR_VELOCITY_FAST					3000000	//
#define KERR_VELOCITY_MAX					6000000	//

#define KERR_ACCELERATION_STOP				      0	//
#define KERR_ACCELERATION_SLOW				    150	//
#define KERR_ACCELERATION_MED_SLOW			    600	//
#define KERR_ACCELERATION_MED				   1500	//
#define KERR_ACCELERATION_MED_FAST			   7000	//
#define KERR_ACCELERATION_FAST				  15000	//
#define KERR_ACCELERATION_MAX				  20000	//



/*


// Controls how Kerrs respond to instructions
#define KERR_REPLY_NONE			// Kerr will never respond to command packets
#define KERR_REPLY_READ_DATA	// Kerr will respond only to ReadData packets
#define KERR_REPLY_ALL			// Kerr will respond to all command packets

#define KERR_INSTRUCTION_PING			0x01 // Respond only with a status packet.
#define KERR_INSTRUCTION_READ_DATA		0x02 // Read register data
#define KERR_INSTRUCTION_WRITE_DATA		0x03 // Write register data
#define KERR_INSTRUCTION_REG_WRITE		0x04 // Delay writing register data until an Action instruction is received
#define KERR_INSTRUCTION_ACTION			0x05 // Perform pending RegWrite instructions
#define KERR_INSTRUCTION_RESET			0x06 // Reset all registers (including ID) to default values
#define KERR_INSTRUCTION_SYNC_WRITE		0x83 // Write register data to multiple Kerrs at once

// Kerr Limits
#define KERR_SERVO_CENTER				0x1FF	//  511	
#define KERR_ID_MAX						0x00FD	//  253
#define KERR_ANGLE_LIMIT_MIN			0x0000	//    0	(Limit can be set 0-1023.  0=max CW (0 degrees).  150 degrees = centered
#define KERR_ANGLE_LIMIT_MAX			0x03FF	// 1023 (Limit can be set 0-1023.  1023= max CCW (300 degrees)
#define KERR_TEMPERATURE_LIMIT_MAX		0x0096	//  150 Degrees Celsius
#define KERR_VOLTAGE_LIMIT_MIN			0x0032	//   50	( 5.0v)
#define KERR_VOLTAGE_LIMIT_MAX			0x00FA	//  250 (25.0v)
#define KERR_TORQUE_MAX					0x03FF	// 1023
#define KERR_GOAL_POSITION_MAX			0x03FF	// 1023
#define KERR_MOVING_SPEED_MAX			0x03FF	// 1023
#define KERR_TORQUE_LIMIT_MAX			0x03FF	// 1023
#define KERR_PUNCH_MAX					0x03FF	// 1023


#define KERR_MAX_DATA_BYTES		32	// Bytes in optional data field of command
#define KERR_MAX_REPLY_BYTES	32	// Bytes in optional data field of reply
#define KERR_CMD_HEADER_SIZE	 4	// Bytes in header: address, checksum, axis, command code
*/
#define KERR_MAX_DATA_BYTES		15	// Max number of data bytes in a command
#define KERR_MAX_REPLY_BYTES	15	// Max number of data bytes in returned status packet
#define KERR_READ_BUF_SIZE		64	// Size of buffer for reading messages from Kerr Controller
#define KERR_SIO_HEADER_BYTE	0xAA

#define KERR_MOVING_SPEED_MAX	1	// TODO-LOKI
#define KERR_ID_MAX				2	// Servos are 1 and 2


/////////////////////////////////////////////////////////////////////////////
#pragma pack( 1 )	// don't let compiler pad between BYTEs

// command packet sent to Kerr controller
// 0      1         2                         3...         3 + data-length
// [0xAA] [address] [Length Nib][Command Nib] [...data...] [checksum]
typedef struct KerrCmd_struct {
	BYTE Header;	//0xAA
	BYTE ID;		// 
	BYTE Command;
	BYTE Data[KERR_MAX_DATA_BYTES]; // (includes CheckSum after the data)
} KerrCmd_T;

typedef struct  {
	BYTE Status;
	BYTE Checksum; 
} KerrReplyShort_T;

typedef struct  {
//	BYTE ID;	
	BYTE Status;
	int  Position;
	BYTE Load;
	BYTE Checksum; 
} KerrReplyLong_T;

#pragma pack( 4 )


/////////////////////////////////////////////////////////////////////////////
class CKerrControl
{

public:
	CKerrControl();
	~CKerrControl();

void	Init();
void	InitControllers();
void	AddCheckSum( int  DataBytes );
void	HandleCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 );
void	GetStatus( int  ID );
int 	GetResponseCheckSum( int  nResponseBytes );
void	DisplayError( int  MotorNumber, int  nError );
void	CalibrateHomePosition( int  MotorNumber );
void	GotoSleepPosition();
BOOL	SendCmd( int  nParamBytes, int  nResponseBytes = 1, BOOL ResponseExpected = TRUE ); // 1 response byte by default

int		TicksToTenthDegree( int  ServoID, int Ticks );
int		TenthDegreeToTicks( int  MotorNumber, int TenthDegrees );

//int 	KerrServoSpeedToServoSpeed( int  KerrServoSpeed );
void	ServoSpeedToKerrSpeed( int  ArmSpeed, int  &Velocity, int  &Acceleration );

double	CentigradeToFahrenheit( double Centigrade );
void	CheckServoLimit( int  MotorNumber, int  &PositionTicks );
void	DisplayServoStatus( int  MotorNumber, int  nResponseBytes );
void	SetArmPosition(int  MotorNumber, int Position);
void	SetArmPositionAndSpeed(int  MotorNumber, int PositionTenthDegrees, int  Speed);
void	EnableServoTorque( int  MotorNumber, BOOL bEnable );
void	SetSpeed(int  MotorNumber, int  Speed);
int 	ArmNumberToMotorNumber( int  ArmNumber );
int 	ServoIDToMotorNumber( int  ServoID );
int 	MotorNumberToServoID( int  MotorNumber );



// Individual Servo Commands
void	SetServoSpeed( int  MotorNumber, int  Speed );


// Servo Status Request Commands
void	GetServoStatus( int  ServoID );	// Read all Current status registers


protected:
	KerrCmd_T			m_KerrCmd;
//	KerrReplyLong_T		m_KerrReply;
	char				m_KerrReplyBuf[KERR_READ_BUF_SIZE];
//	int					m_nStatusBytes;
	BOOL				m_HomePositionInitializedRight;
	BOOL				m_HomePositionInitializedLeft;
	ArmControl		   *m_pArmControlRight;
	ArmControl		   *m_pArmControlLeft;
	DWORD				KerrCmdStartTime;
	BOOL				m_ControllerIDsInitialized;
	BOOL				m_PowerEnabled;  // prevent servo motion when shut down (or in the process of shutting down)


};
