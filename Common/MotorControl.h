// MotorControl.h
// Interface for three main kinds of motor controllers
// -- PMD Pilot ER1 Motor Controller
// -- Pololu TReX Motor Controller
// -- iRobotCreate Base (Roomba)
// See TrexMotorControl.cpp, PilotMotorControl.cpp, or iRobotControl.cpp for implementations

#pragma once

#include "RobotType.h"
#include "KobukiCommon.h"

/************************************************************************
                  PMD Pilot ER1 Motor Controller
************************************************************************/

// PMD Pilot Motion Processor Commands
#define ClearInterrupt			0xAC
#define AdjustActualPostion		0xF5
#define ClearPositionError		0x47
#define GetActivityStatus		0xA6
#define GetActualVelocity		0xAD
#define GetCaptureValue			0x36
#define GetChecksum				0xF8
#define GetCommandedAcceleration 0xA7
#define GetCommandedPosition	0x1D
#define GetCommandedVelocity	0x1E
#define GetCurrentMotorCommand	0x3A
#define GetDerivative			0x9B
#define GetEventStatus			0x31
#define GetHostIOError			0xA5
#define GetIntegral				0x9A
#define GetInterruptAxis		0xE1
#define GetPhaseCommand			0xEA
#define GetPositionError		0x99
#define GetSignalStatus			0xA4
#define GetTime					0x3E
#define GetTraceCount			0xBB
#define GetTraceStatus			0xBA
#define GetPilotVersion 		0x8F
#define InitializPhase			0x7A
#define NoOperation				0x00
#define ReadAnalog				0xEF
#define ReadBuffer				0xC9
#define ReadIO					0x83
#define PilotReset				0x39
#define ResetEventStatus		0x34
#define SetAcceleration			0x90
#define GetAcceleration			0x4C
#define SetActualPostion		0x4D
#define GetActualPostion		0x37
#define SetActualPostionUnits	0xBE
#define GetActualPostionUnits	0xBF
#define SetAutoStopMode			0xD2
#define GetAutoStopMode			0xD3
#define SetAxisMode				0x87
#define GetAxisMode				0x88
#define SetAxisOutSource		0xED
#define GetAxisOutSource		0xEE
#define SetBreakpoint			0xD4
#define GetBreakpoint			0xD5
#define SetBreakpointValue		0xD6
#define GetBreakpointValue		0xD7
#define SetBufferFunction		0xCA
#define GetBufferFunction		0xCB
#define SetbufferLength			0xC2
#define GetBufferLength			0xC3
#define SetBufferReadIndex		0xC6
#define GetBufferReadIndex		0xC7
#define SetBufferStart			0xC0
#define GetBufferStart			0xC1
#define SetBufferWriteIndex		0xC4
#define GetBufferWriteIndex		0xC5
#define SetCaptureSource		0xD8
#define GetCaptureSource		0xD9
#define SetCommutationMode		0xE2
#define GetCommutationMode		0xE3
#define SetDeceleration			0x91
#define GetDeceleration			0x92
#define SetDerivativeTime		0x9C
#define GetDerivativeTime		0x9D
#define SetDiagnosticPortMode	0x89
#define GetDiagnosticPortMode	0x8A
#define SetEncoderModulus		0x8D
#define GetEncoderModulus		0x8E
#define SetEncoderSource		0xDA
#define GetEncoderSource		0xDB
#define SetEncoderToStepRatio	0xDE
#define GetEncoderToStepRatio	0xDF
#define SetIntegrationLimit		0x95
#define GetIntegrationLimit		0x96
#define SetInterruptMask		0x2F
#define GetInterruptMask		0x56
#define SetJerk					0x13
#define GetJerk					0x58
#define SetKaff					0x93
#define GetKaff					0x94
#define SetKd					0x27
#define GetKd					0x52
#define SetKi					0x26
#define GetKi					0x51
#define SetKout					0x9E
#define GetKout					0x9F
#define SetKp					0x25
#define GetKp					0x50
#define SetKvff					0x2B
#define GetKvff					0x54
#define SetLimitSwitchMode		0x80
#define GetLimitSwitchMode		0x81
#define SetMotionCompleteMode	0xEB
#define GetMotionCompleteMode	0xEC
#define SetMotorBias			0x0F
#define GetMotorBias			0x2D
#define SetMotorCommand			0x77
#define GetMotorCommand			0x69
#define SetMotorLimit			0x06
#define GetMotorLimit			0x07
#define SetMotorMode			0xDC
#define GetMotorMode			0xDD
#define SetNumberPhases			0x85
#define GetNumberPhases			0x86
#define SetOutputMode			0xE0
#define GetOutputMode			0x6E
#define SetPhaseAngle			0x84
#define GetPhaseAngle			0x2C
#define SetPhaseCorrectionMode	0xE8
#define GetPhaseCorrectionMode	0xE9
#define SetPhaseCounts			0x75
#define GetPhaseCounts			0x7D
#define SetPhaseInitializeMode	0xE4
#define GetPhaseInitializeMode	0xE5
#define SetPhaseInitializeTime	0x72
#define GetPhaseInitializeTime	0x7C
#define SetPhaseOffset			0x76
#define GetPhaseOffset			0x7B
#define SetPhasePrescale		0xE6
#define GetPhasePrescale		0xE7
#define SetPostion				0x10
#define GetPostion				0x4A
#define SetPostionErrorLimit	0x97
#define GetPostionErrorLimit	0x98
#define SetProfileMode			0xA0
#define GetProfileMode			0xA1
#define SetSampleTime			0x38
#define GetSampleTime			0x61
#define SetSerialPortMode		0x8B
#define GetSerialPortMode		0x8C
#define SetSettleTime			0xAA
#define GetSettleTime			0xAB
#define SetSettleWindow			0xBC
#define GetSettleWindow			0xBD
#define SetSignalSense			0xA2
#define GetSignalSense			0xA3
#define SetStartMode			0xCC
#define GetStartMode			0xCD
#define SetStartVelocity		0x6A
#define GetStartVelocity		0x6B
#define SetStopMode				0xD0
#define GetStopMode				0xD1
#define SetSynchronizationMode	0xF2
#define GetSynchronizationMode	0xF3
#define SetTraceMode			0xB0
#define GetTraceMode			0xB1
#define SetTracePeriod			0xB8
#define GetTracePeriod			0xB9
#define SetTraceStart			0xB2
#define GetTraceStart			0xB3
#define SetTraceStop			0xB4
#define GetTraceStop			0xB5
#define SetTraceVariable		0xB6
#define GetTraceVariable		0xB7
#define SetTrackingWindow		0xA8
#define GetTrackingWindow		0xA9
#define SetVelocity				0x11
#define GetVelocity				0xAB
#define PilotUpdate				0x1A
#define WriteBuffer				0xC8
#define WriteIO					0x82

#define MAX_DATA_BYTES		 6	// Bytes in optional data field of command
#define MAX_REPLY_BYTES		 6	// Bytes in optional data field of reply
#define CMD_HEADER_SIZE		 4	// Bytes in header: address, checksum, axis, command code
#define READ_BUF_SIZE		32	// Size of buffer for reading messages from Pilot


#define SPEED_DIVIDER	4096
#define PILOT_ACCELERATION_PROFILE		0x20	// 0x0000 00B0 = default acceleration profile for ER1
#define PILOT_DECELERATION_PROFILE		0x20	// 0x0000 00B0 = default deceleration profile for ER1


//SLOW SPEED RANGE: 
//#define PILOT_SPEED_TO_VELOCITY_FACTOR 0x03CA // Step size = 0x0001 E1B4 / 127 = 970 (0x3CA)

//MEDIUM SPEED RANGE:
//#define PILOT_SPEED_TO_VELOCITY_FACTOR 0x0800

//MEDIUM-FAST SPEED RANGE:
#define PILOT_SPEED_TO_VELOCITY_FACTOR 0x1000


// Fast speed range: This causes the motors to lock up a lot on the full, heavy robot
//#define PILOT_SPEED_TO_VELOCITY_FACTOR 0x2000

// Really fast, hard to control!
// #define PILOT_SPEED_TO_VELOCITY_FACTOR 0x4000 

/* Begin: Define constant for PWDReply
#define STATUS 0
#define CHECKSUM 1
#define DATA_START 2
#define DATA_END 7
#define CHECKSUMVALID 0
END: Define constant for PWDReply */
//#define PWMREPLYSIZE 8

#pragma pack( 1 )	// don't let compiler pad between bytes

typedef struct PilotCmd_struct {
	byte Address;
	byte Checksum;
	byte Axis;	// initialized by constructor, always 0
	byte Code;
	byte Data[MAX_DATA_BYTES];
} PilotCmd_T;

typedef struct PilotReply_struct {
	byte CommandStatus;	// Zero if command completed correctly
	byte Checksum;		// To be used to validate that this response is valid
	byte Data[MAX_REPLY_BYTES];
} PilotReply_T;


/*typedef struct PWDReply_struct {
	byte data[PWMREPLYSIZE];
	DWORD dwSize;
	DWORD Status();	// return 0(valid) if the valus is correct
	DWORD CheckChecksum();
	void PrintData();
} PWDReply; */

#pragma pack( 4 )


/////////////////////////////////////////////////////////////////////////////
class CPilotControl
{

public:
	CPilotControl();
	// ~CPilotControl();

void	Init();
void	DoCheckSum(int  DataBytes);
void	InitWheelMotor(int  MotorNumber);
void	SetWheelPowerLevel(int  PercentPowerLevel);
void	SendWheelCommand(int  Wheel, DWORD dwMotorSpeed);
void	SendWheelStopCommand(int  Wheel);
void	SendQuickWheelCommand(int  Wheel, DWORD dwMotorSpeed);
void	SetWheelSpeedAndTurn();
void	SetWheelAllStop();
void	HandlePilotCommand( int  Request, int  Param1, int  Param2, int  Option1, int  Option2 );
void	DoMotorUpdate();\
void	GetStatus();
DWORD	SpeedToVelocity(int dwSpeed);
void	SendCmd( int  nParamBytes, int  nBytesToRead );
void	SendWriteCmd( int  nParamBytes);
BOOL	SendReadCmd( int  nBytesToRead );
void	DisplayIOError();



protected:
	PilotCmd_T		m_PilotCmd;
	PilotReply_T	m_PilotReply;
	int				m_Speed;
	int				m_Turn;
	BOOL			m_LeftWheelStopped;
	BOOL			m_RightWheelStopped;
	char			m_ReadBuf[READ_BUF_SIZE];

};


/************************************************************************
                  Pololu TReX Motor Controller
************************************************************************/
#define TREX_CMD_BUF_SIZE		32
//#define TREX_MAX_REPLY_BYTES	 6	// Bytes in optional data field of reply
#define TREX_READ_BUF_SIZE		32	// Size of buffer for reading messages from Pilot

/////////////////////////////////////////////////////////////////////////////
class CTrexControl
{

public:
	CTrexControl();
	// ~CTrexControl();

void	Init();
void	DoCheckSum(int  DataBytes);
void	SendWheelCommand(int  Wheel, DWORD dwMotorSpeed);
void	SetWheelSpeedAndTurn();
void	HandleCommand( int  Request, int  Param1, int  Param2 );
void	GetStatus();
void	SendCmd( int  nParamBytes, int  nBytesToRead );
void	SendWriteCmd( int  nBytesToSend);
void	SpeedControl();
void	DisplayIOError();
int		SetTachometerTarget( int TargetSpeed );
BOOL	SingleWheelSpeedControl( int &MotorSpeedCmd, int MotorSpeedRequest, int TachometerTicks, int TachometerTarget, const char* WhichWheel );
void	TrexSendAccelerateCmd( int MotorSpeedL, int MotorSpeedR );


protected:

	char			m_CmdBuf[40];

	int 			m_TrexCmd;
//	PilotReply_T	m_PilotReply;
	int				m_Speed;
	int				m_Turn;

	int				m_MotorSpeedCmdLeft;
	int				m_MotorSpeedRequestLeft;
	int				m_TachometerTargetLeft;

	int				m_MotorSpeedCmdRight;
	int				m_MotorSpeedRequestRight;
	int				m_TachometerTargetRight;
	int				m_StraightTracking;

	char			m_ReadBuf[READ_BUF_SIZE];

};

/************************************************************************
                  iRobot Create (Roomba) control
************************************************************************/


#define LEFT_MOTOR							  1
#define RIGHT_MOTOR							  0
#define IROBOT_CMD_BUF_SIZE					 32		// Largest command with parameters is well below this
#define IROBOT_READ_BUF_SIZE			     512	// Handles multiple reads at once 


#pragma pack( 1 )	// don't let compiler pad between bytes

	// Todo - check that this works right, or use hibyte, lobyte
	typedef struct
	{
		BYTE Cmd;
		__int16  MotorSpeedR;
		__int16  MotorSpeedL;
	} IROBOT_MOTOR_CMD_T;
	#pragma pack( 4 )

	/***
typedef struct iRobotCmd_struct {
	byte Address;
	byte Checksum;
	byte Axis;	// initialized by constructor, always 0
	byte Code;
	byte Data[MAX_DATA_BYTES];
} iRobotCmd_T;

typedef struct iRobotReply_struct {
	byte CommandStatus;	// Zero if command completed correctly
	byte Checksum;		// To be used to validate that this response is valid
	byte Data[MAX_REPLY_BYTES];
} iRobotReply_T;
***/
#pragma pack( 4 )


/////////////////////////////////////////////////////////////////////////////
class CiRobotControl
{

public:
	CiRobotControl();
	~CiRobotControl();

void	Init();
void	SendStartCmd();
void	StartSensorStreamCmd();
void	SetMode( int mode );
void	SendMotorCmd( int MotorSpeedL, int MotorSpeedR );
void	SetWheelSpeedAndTurn();
void	SendCmd( int nCmdBytes );
void	UnDock();
void	Dock();
void	SetPort();



void	HandleCommand( int  Request, int  Param1, int  Param2 );

protected:
	BOOL			m_StartCmdSent;
	int				m_Speed;
	int				m_Turn;
	BOOL			m_LeftWheelStopped;
	BOOL			m_RightWheelStopped;
	BOOL			m_Docked;
	BOOL			m_LEDPower;
	BYTE			m_pCmdBuf[IROBOT_CMD_BUF_SIZE];
	BYTE*			m_pReadBuf;


};

/////////////////////////////////////////////////////////////////////////////
class CiRobotParser
{

public:
	CiRobotParser();
	~CiRobotParser();

	BOOL ParseBuffer( char* iRobotSIOBuf, int dwSIOBytesReceived );

//protected:

};

/************************************************************************
                  Kobuki Base control
************************************************************************/


#define LEFT_MOTOR							  1
#define RIGHT_MOTOR							  0
#define KOBUKI_CMD_BUF_SIZE					 32		// Largest command with parameters is well below this
#define KOBUKI_READ_BUF_SIZE			     512	// TODO Need to set to size of actual data (or get rid of this, since its shared memory?


#pragma pack( 1 )	// don't let compiler pad between bytes

	// Todo - check that this works right, or use hibyte, lobyte
	typedef struct
	{
		BYTE Cmd;
		__int16  MotorSpeedR;
		__int16  MotorSpeedL;
	} KOBUKI_MOTOR_CMD_T;
//	#pragma pack( 4 )

	/***
typedef struct KobukiCmd_struct {
	byte Address;
	byte Checksum;
	byte Axis;	// initialized by constructor, always 0
	byte Code;
	byte Data[MAX_DATA_BYTES];
} KobukiCmd_T;

typedef struct KobukiReply_struct {
	byte CommandStatus;	// Zero if command completed correctly
	byte Checksum;		// To be used to validate that this response is valid
	byte Data[MAX_REPLY_BYTES];
} KobukiReply_T;
***/
#pragma pack( 4 )


/////////////////////////////////////////////////////////////////////////////
class CKobukiControl
{

public:
	CKobukiControl();
	~CKobukiControl();

void	Init();
void	ShutDown();
//void	SendStartCmd();
//void	StartSensorStreamCmd();
//void	SetMode( int mode );
void	SendMotorCmd( int MotorSpeedL, int MotorSpeedR );
void	SetWheelSpeedAndTurn();
void	SendCmd( int nCmdBytes );
//void	UnDock();
//void	Dock();
void	GetStatus();
void	SetDigitalIO( unsigned int Pins );
void	SetExternalPower( );


void	HandleCommand( int  Request, int  Param1, int  Param2 );

protected:
	//BOOL			m_StartCmdSent;
	int				m_Speed;
	int				m_Turn;
	int				m_Acceleration; // enum 0 = instant, 1 = fast, 2 = medium, 3 = slow
	BOOL			m_LeftWheelStopped;
	BOOL			m_RightWheelStopped;
	BOOL			m_Docked;
	BOOL			m_LEDPower;
	BYTE			m_pCmdBuf[KOBUKI_CMD_BUF_SIZE]; //Message with sync and checksum
	BYTE			m_pCmdMsg[KOBUKI_CMD_BUF_SIZE]; // actual message to send
	BYTE*			m_pReadBuf;

	KOBUKI_COMMAND_T m_KobukiCommand;	// keep track of commands, only update ones that change



};

/**
/////////////////////////////////////////////////////////////////////////////
class CKobukiParser
{

public:
	CKobukiParser();
	~CKobukiParser();

	BOOL ParseBuffer( unsigned char* KobukiSIOBuf, int dwSIOBytesReceived );

//protected:

};
**/
