// HWInterface.cpp
// Direct interface to hardware: Arduino, Servo Controller, etc.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include "MotorControl.h"
#include "DynamixelControl.h"
#include "KerrControl.h"
#include "Module.h"
#include "Globals.h"

//#if ( ROBOT_SERVER == 1 )
#include "NMEAParser.h"
//#endif

#include "HWInterface.h"
#include "HardwareConfig.h"
#include "HeadControl.h"
#include "LaserScanner.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define GPS_BUF_LEN			 256
#define SERVO_SYNC_0		0xFF	// required sync for SSC II controller
//#define SERVO_SYNC_0		0x80	// required sync for Pololu controller
#define SERVO_CONTROLLER_ID	0x01	// only one controller attached
#define CAMERA_UNKNOWN_POSITION 0xFFFF


////////////////////////////////////////////////////////////////////////////////////////
__itt_string_handle* pshGPSReadLoop = __itt_string_handle_create("GPS Read Loop");
DWORD WINAPI GPSCommReadThreadFunc(LPVOID lpParameter)
{
	HANDLE hCommPort = (HANDLE)lpParameter;
	__itt_thread_set_name( "GPS Read Thread" );

	BYTE	GPSBuffer[GPS_BUF_LEN];
	DWORD dwBytesReceived;

	// Create the GPS Parser object
	CNMEAParser NMEAParser;
	NMEAParser.m_dGSAVDOP = 22;


	if( 1 == ROBOT_SIMULATION_MODE )
	{
		SetCommMask (g_hGPSCommPort, EV_RXCHAR | EV_BREAK);
	}
	RobotSleep(4000, pDomainGPSThread);	// Allow other startups, and for the port to be ready

	while(g_hGPSCommPort != INVALID_HANDLE_VALUE)
	{
		__itt_task_begin(pDomainGPSThread, __itt_null, __itt_null, pshGPSReadLoop);

		if( 1 == ROBOT_SIMULATION_MODE )
		{
			__itt_task_end(pDomainGPSThread);  // pshGPSReadLoop
			RobotSleep(100, pDomainGPSThread); //Use instead of SIMULATED_SIO_SLEEP_TIME for GPS
			continue;
		}		

		//SetCommMask (g_hGPSCommPort, EV_RXCHAR | EV_BREAK);
		dwBytesReceived = 0;
		GPSBuffer[0] = '\0';
		// Read ANSI characters
		if(!ReadFile(g_hGPSCommPort,		// where to read from (the open comm port)
				 GPSBuffer,			// where to put the character
				 (GPS_BUF_LEN-1),	// number of bytes to read
				 &dwBytesReceived,		// how many bytes were actually read
				 NULL))				// no overlapped I/O
		{

			ROBOT_DISPLAY( TRUE, "ERROR Reading GPS Comm port\n" )
			PurgeComm(g_hGPSCommPort, PURGE_RXABORT|PURGE_RXCLEAR);
			//PurgeComm(hCommPort, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
			DWORD dwCommError = 0;
			ClearCommError(g_hGPSCommPort, &dwCommError, NULL);
			ReportCommError("GPS SIO Read Error", dwCommError );

			__itt_task_end(pDomainGPSThread);  // pshGPSReadLoop
			return 0;		// terminate thread on error
		}
		if( 0 == dwBytesReceived )
		{
			// Just keep looking for data from the device
			__itt_task_end(pDomainGPSThread);  // pshGPSReadLoop
			RobotSleep(100, pDomainGPSThread);
			continue;
		}
		else
		{
			// Got some GPS data.  Parse it.
			if( NMEAParser.ParseBuffer(GPSBuffer, dwBytesReceived) )
			{
				// Data ready - Notify the robot control engine!
				// ROBOT_LOG( TRUE,"HWInterface - Got Data!\n")
				g_GPSSubSystemStatus = SUBSYSTEM_CONNECTED;

				// Now throw message in the queue, to indicate that the GPS data has been updated
				PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_GPS_DATA_READY), 0, 0 );
			}
		}

		__itt_task_end(pDomainGPSThread);  // pshGPSReadLoop
		RobotSleep(100, pDomainGPSThread);

	}	// while port open


	ROBOT_LOG( TRUE, "GPS COMM Read Thread exiting.\n")
	return 0;
}



////////////////////////////////////////////////////////////////////////////////////////
__itt_string_handle* pshPololuWriteLoop = __itt_string_handle_create("Pololu Write Loop");

DWORD WINAPI ServoCommWriteThreadFunc(LPVOID lpParameter)
{
	// Writes a command to the Pololu Servo controller
	// Messages are sent by SendHardwareCmd calls
	IGNORE_UNUSED_PARAM(lpParameter);
	__itt_thread_set_name( "Pololu Servo Write Thread" );

	MSG		msg;
	char	CmdBuf[16];
	DWORD	dwBytesWritten;
	BYTE	Request;
	BYTE	Index;
	BYTE	Value;
	BYTE	Option1;
	BYTE	Option2;
//	WORD	TempWord;
	CString strStatus;
	int		DbgDotCount = 0;
	BYTE ServoNumber = 0;
	BYTE Position = 0;

	RobotSleep(3000, pDomainPololuServoThread);

	// Process message loop for this thread
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		__itt_task_begin(pDomainPololuServoThread, __itt_null, __itt_null, pshPololuWriteLoop);

		Request = (BYTE)(msg.message - WM_ROBOT_MESSAGE_BASE);

		if( WM_ROBOT_THREAD_EXIT_CMD == Request )
		{
			break; // Quit command received!
		}

		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
		{
			// Ignore any messages not intended for user (where do these come from?)
			ROBOT_LOG( TRUE, "ArduinoCommWriteThread: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
			__itt_task_end(pDomainPololuServoThread);  // pshPololuWriteLoop
			continue;
		}
		// 

		// Get ready to send the message to the Servo controller

		// TODO!  TEST THIS!
		// WARNING! WARNING!  CHANGED THIS FROM UNPACKING BYTES TO USING WPARAM AND LPARAM DIRECTLY!
		// THIS MAY BREAK Arduino COMMUNICATION - NEED TO TEST!!!
//		TempWord = (WORD)(msg.lParam);
//		Index = HIBYTE(msg.wParam);
//		Value = LOBYTE(msg.wParam);
//		Option1 = HIBYTE(TempWord);
//		Option2 = LOBYTE(TempWord);
		Index = (BYTE)(msg.wParam);
		Value = (BYTE)(msg.lParam);
		Option1 = 0;
		Option2 = 0;


		// Format Servo command
		// We don't use the Pololu mode for now...
		// case HW_INITIALIZE_SERVO:
		// case HW_SET_SERVO_ZERO_POS:
		// CmdBuf[1] = SERVO_CONTROLLER_ID;


		switch( Request )
		{
			case HW_SET_MOTOR_STOP:
			{
				ServoNumber = SERVO_SPEED;
				Position = SPEED_SERVO_CENTER;	// Mini SSC II mode: 0x00 to 0xFE
				break;
			}
/*			No longer used!
			case HW_SET_MOVE_DISTANCE:
			{
				ROBOT_DISPLAY( TRUE, "ERROR! ServoCommWriteThread: HW_SET_MOVE_DISTANCE not supported!\n" )
				break;
			}
*/
			case HW_SET_TURN:
			{
				ServoNumber = SERVO_TURN;
				Position = Index;	// Mini SSC II mode: 0x00 to 0xFE
				break;
			}
/*
			case HW_SET_SPEED:
			{
				ServoNumber = SERVO_SPEED;
				Position = Index;	// Mini SSC II mode: 0x00 to 0xFE
				break;
			}
*/
			case HW_SET_GEAR:
			{
				ServoNumber = SERVO_GEAR;
				Position = Value;	// Mini SSC II mode: 0x00 to 0xFE
				ROBOT_LOG( TRUE,"Extern Servo SET GEAR: %d", Position)
				break;
			}

			case HW_SET_CAMERA_PAN_ABS:
			{
				ServoNumber = SERVO_CAMERA_PAN + SERVO_MODE_180; // See Pololu doc for SSEII mode
				Position = Index;	// Mini SSC II mode: 0x00 to 0xFE
				break;
			}
			case HW_SET_CAMERA_TILT_ABS:
			{
				ServoNumber = SERVO_CAMERA_TILT;
				Position = (255-Index);	// Tilt Servo is Inverted
				break;
			}
			case HW_SET_CAMERA_PAN_TILT_ABS:
			{
				// This function can only hand commands that control one servo at a time!
				ROBOT_DISPLAY( TRUE, "ERROR! ServoCommWriteThread: HW_SET_CAMERA_PAN_TILT_ABS not supported!\n" )
				ROBOT_ASSERT(0);
				break;
			}

			case HW_SET_SPEED_AND_TURN:
			{
				// This function can only hand commands that control one servo at a time!
				ROBOT_DISPLAY( TRUE, "ERROR! ServoCommWriteThread: HW_SET_SPEED_AND_TURN not supported!\n" )
				break;
			}
			default:
			{
				CString StrText;
				StrText.Format( "ERROR! ServoCommWriteThread: Unknown Cmd:%02X \n", Request);
				ROBOT_DISPLAY( TRUE, (StrText) )
			}

		}

		//TAL_Event("Sending Cmd");
		//-TAL_SCOPED_TASK_NAMED("Servo Write SIO");

		CmdBuf[0] = (char)(BYTE)SERVO_SYNC_0;
		CmdBuf[1] =	(char)ServoNumber;
		CmdBuf[2] =	(char)Position;

		ROBOT_LOG( SERIAL_DBG, "Servo CMD: Sending cmd %02X to Servo controller.\n", Request )
		if( 1 == ROBOT_SIMULATION_MODE )
		{
			__itt_task_end(pDomainPololuServoThread);  // pshPololuWriteLoop
			RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainPololuServoThread);
			continue;
		}
		if(!WriteFile(g_hServoCommPort,	// where to write to (the open comm port)
				CmdBuf,				// what to write
				3,					// number of bytes to be written to port
				&dwBytesWritten,	// number of bytes that were actually written
				NULL))				// overlapped I/O not supported			
		{
			ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Servo data!\n" )
			__itt_task_end(pDomainPololuServoThread);  // pshPololuWriteLoop
			continue;
		}
		else
		{
			ROBOT_LOG( SERIAL_DBG, "Sent %d bytes to Servo.\n", dwBytesWritten )
			if( SERIAL_DBG )
			{
				ROBOT_LOG( TRUE,  "Tx");	// just show activity for each write, so we know Servo is not dead.
				if( DbgDotCount++ > 30 )
				{
					ROBOT_LOG( TRUE,  "\n" );
					DbgDotCount = 0;
				}
			}

		}
		__itt_task_end(pDomainPololuServoThread);  // pshPololuWriteLoop
		Sleep(0); // force thread switch after each command
	}
	ROBOT_LOG( TRUE, "SERVO COMM WRITE: Received WM_QUIT\n")
	return 0;
}

void SendCameraCmd(char *CameraBytes, int nByteCount)
{
	DWORD dwBytesWritten;

	ASSERT(0); // Serial Camera not enabled!
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainPololuServoThread);
		return;
	}
	if(!WriteFile(g_hCameraCommPort,	// where to write to (the open comm port)
			CameraBytes,			// what to write
			nByteCount,		// number of bytes to be written to port
			&dwBytesWritten,	// number of bytes that were actually written
			NULL))				// overlapped I/O not supported			
	{
		ROBOT_LOG( TRUE,"Serial Error Sending Camera data.\n")
	}
	else
	{
		ROBOT_LOG( SERIAL_DBG, "Sent %d bytes to Camera.\n", dwBytesWritten )
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA

DWORD WINAPI CameraCommWriteThreadFunc(LPVOID lpParameter)
{
	// Writes a command to the Sony Camera serial interface
	// Messages are sent by SendHardwareCmd calls
	__itt_thread_set_name( "Sony Camera Write Thread" );


	MSG		msg;
	BYTE	Request;
	CString strStatus;
	int		DbgDotCount = 0;
	BYTE CameraNumber = 0;
	BYTE Position = 0;
	static int CurrentPanPos = 0;
	static int CurrentTiltPos = 0;
	static int AbsPanSpeed = 20;
	static int AbsTiltSpeed = 16;


	// Process message loop for this thread
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		Request = msg.message - WM_ROBOT_MESSAGE_BASE;

		if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
		{
			break; // Quit command received!
		}

		if( INVALID_HANDLE_VALUE == g_hCameraCommPort )
		{
			continue; // Camera is not connected.
		}

		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
		{
			// Ignore any messages not intended for user (where do these come from?)
			ROBOT_LOG( TRUE, "ArduinoCommWriteThread: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
			continue;
		}

		int wParam = msg.wParam;
		int lParam = msg.lParam;

		// Note that lParam is only used for Abs Pan/Tilt, and HW_SET_CAMERA_MODE commands

		switch( Request )
		{		

			case HW_SET_CAMERA_MODE:
			{
				// Put Camera in a specific mode
				// wParam = Mode, lParam = enable/disable
				if( CAMERA_MODE_DISPLAY_ENABLE == wParam)
				{
					// Set Camera Display mode

					char CameraBytes[6];
					CameraBytes[0] = (char)0x81;	// Header for Camera #1
					CameraBytes[1] = (char)0x01;	// Control Command
					CameraBytes[2] = (char)0x06;	// 0606 = Display control command 
					CameraBytes[3] = (char)0x06;	//  
									// On/Off Command will be inserted here
					CameraBytes[5] = (char)0xFF;	// Terminator

					if( 1 == lParam )
					{
						CameraBytes[4] = (char)0x02;	// Display On
					}
					else
					{
						CameraBytes[4] = (char)0x03;	// Display Off
					}
					SendCameraCmd(CameraBytes, 6);			
				}
				else if( CAMERA_MODE_TRACK_FACE == wParam )
				{
					ROBOT_LOG( TRUE,"NOT IMPLEMENTED - CAMERA_MODE_TRACK_FACE\n")
					ROBOT_ASSERT(0);	
				}
				else
				{
					ROBOT_ASSERT(0);	// ERROR BAD MODE!
				}

				break;
			}



			case HW_SET_CAMERA_ABS_PAN_TILT_SPEED:
			{
				// Just save the values for later.
				// SONY: Pan speed 0-18h (24), Tilt speed 0-14h (20)
				if( (UINT)(wParam) > SERVO_SPEED_MAX ) wParam = SERVO_SPEED_MAX;		// Can't be negative, UINT
				AbsPanSpeed = wParam + 2;
				AbsTiltSpeed = wParam - 2;
				if( AbsTiltSpeed < 0 ) AbsTiltSpeed = 0;	// Limit zero
				break;
			}

			case HW_SET_CAMERA_PAN_ABS:
			{
				// Limit values to max camera can handle
				if( wParam > SONY_CAMERA_MAX_RIGHT ) wParam = SONY_CAMERA_MAX_RIGHT;
				if( wParam < SONY_CAMERA_MAX_LEFT ) wParam = SONY_CAMERA_MAX_LEFT;

				CurrentPanPos = wParam;	// Save value in case an absolute tilt command comes in
				DWORD PanPos = (DWORD)wParam;
				DWORD TiltPos = (DWORD)CurrentTiltPos;

				char CameraAbsBytes[15];
				CameraAbsBytes[0] =  (char)0x81;	// Header for Camera #1
				CameraAbsBytes[1] =  (char)0x01;	// Control Command
				CameraAbsBytes[2] =  (char)0x06;	// 06 02 = Pan/Tilt Absolute command
				CameraAbsBytes[3] =  (char)0x02;
				CameraAbsBytes[4] =  (char)AbsPanSpeed;	// Pan speed (0-18h)
				CameraAbsBytes[5] =  (char)AbsTiltSpeed;// Tilt speed (0-14h)
				CameraAbsBytes[6] =  (char)0x00;	// Pan -----------------------------------
				CameraAbsBytes[7] =  (char)0x00;	// Range: 0xFC90 to 0x0370 = -880  <-> +880
				CameraAbsBytes[8] =  (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[9] =  (char)0x00;	// 
				CameraAbsBytes[10] = (char)0x00;	// Tilt -----------------------------------
				CameraAbsBytes[11] = (char)0x00;	// Range: 0xFED4 to 0x012C = -300 <-> +300
				CameraAbsBytes[12] = (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[13] = (char)0x00;	// 
				CameraAbsBytes[14] = (char)0xFF;	// Terminator

				// Loop and shift, copying one Nibble at a time into the buffer!
				// Note data is Sparsely Populated, and reverse-indian.  13AF ==> 0F 0A 03 01
				for(int Nibble=0; Nibble < 4; Nibble++ )
				{
					CameraAbsBytes[(9-Nibble)] = (PanPos & 0x000F);
					PanPos =  PanPos >> 4;

					CameraAbsBytes[(13-Nibble)] = (TiltPos & 0x000F);
					TiltPos = TiltPos >> 4;
				}
				
				SendCameraCmd(CameraAbsBytes, 15);
				ROBOT_LOG( TRUE,"Debug Camera - sent Pan Command, Position = %04x\n", (DWORD)wParam )
				break;
			}
			case HW_SET_CAMERA_TILT_ABS:
			{
				// Limit values to max camera can handle
				if( wParam > SONY_CAMERA_MAX_UP ) wParam = SONY_CAMERA_MAX_UP;
				if( wParam < SONY_CAMERA_MAX_DOWN ) wParam = SONY_CAMERA_MAX_DOWN;

				CurrentTiltPos = wParam;	// Save value in case an absolute tilt command comes in
				DWORD TiltPos = (DWORD)wParam;
				DWORD PanPos = (DWORD)CurrentPanPos;

				char CameraAbsBytes[15];
				CameraAbsBytes[0] =  (char)0x81;	// Header for Camera #1
				CameraAbsBytes[1] =  (char)0x01;	// Control Command
				CameraAbsBytes[2] =  (char)0x06;	// 06 02 = Pan/Tilt Absolute command
				CameraAbsBytes[3] =  (char)0x02;
				CameraAbsBytes[4] =  (char)AbsPanSpeed;	// Pan speed (0-18h)
				CameraAbsBytes[5] =  (char)AbsTiltSpeed;// Tilt speed (0-14h)
				CameraAbsBytes[6] =  (char)0x00;	// Pan -----------------------------------
				CameraAbsBytes[7] =  (char)0x00;	// Range: 0xFC90 to 0x0370 = -880  <-> +880
				CameraAbsBytes[8] =  (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[9] =  (char)0x00;	// 
				CameraAbsBytes[10] = (char)0x00;	// Tilt -----------------------------------
				CameraAbsBytes[11] = (char)0x00;	// Range: 0xFED4 to 0x012C = -300 <-> +300
				CameraAbsBytes[12] = (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[13] = (char)0x00;	// 
				CameraAbsBytes[14] = (char)0xFF;	// Terminator

				// Loop and shift, copying one Nibble at a time into the buffer!
				for(int Nibble=0; Nibble < 4; Nibble++ )
				{
					CameraAbsBytes[(9-Nibble)] = (PanPos & 0x000F);
					PanPos =  PanPos >> 4;

					CameraAbsBytes[(13-Nibble)] = (TiltPos & 0x000F);
					TiltPos = TiltPos >> 4;
				}
				
				SendCameraCmd(CameraAbsBytes, 15);
				ROBOT_LOG( TRUE,"Debug Camera - sent Tilt Command, Position = %04x\n", (DWORD)wParam )
				break;
			}
			case HW_SET_CAMERA_PAN_TILT_ABS:
			{
				// wParam = Pan, lParam = tilt
				// Limit values to max camera can handle
				if( wParam > SONY_CAMERA_MAX_RIGHT ) wParam = SONY_CAMERA_MAX_RIGHT;
				else if( wParam < SONY_CAMERA_MAX_LEFT ) wParam = SONY_CAMERA_MAX_LEFT;

				if( lParam > SONY_CAMERA_MAX_UP ) lParam = SONY_CAMERA_MAX_UP;
				else if( lParam < SONY_CAMERA_MAX_DOWN ) lParam = SONY_CAMERA_MAX_DOWN;

				CurrentPanPos = wParam;		// Save value in case an absolute pan command comes in
				CurrentTiltPos = lParam;	// Save value in case an absolute tilt command comes in
				DWORD PanPos = (DWORD)wParam;
				DWORD TiltPos = (DWORD)lParam;

				char CameraAbsBytes[15];
				CameraAbsBytes[0] =  (char)0x81;	// Header for Camera #1
				CameraAbsBytes[1] =  (char)0x01;	// Control Command
				CameraAbsBytes[2] =  (char)0x06;	// 06 02 = Pan/Tilt Absolute command
				CameraAbsBytes[3] =  (char)0x02;
				CameraAbsBytes[4] =  (char)AbsPanSpeed;	// Pan speed (0-18h)
				CameraAbsBytes[5] =  (char)AbsTiltSpeed;// Tilt speed (0-14h)
				CameraAbsBytes[6] =  (char)0x00;	// Pan -----------------------------------
				CameraAbsBytes[7] =  (char)0x00;	// Range: 0xFC90 to 0x0370 = -880  <-> +880
				CameraAbsBytes[8] =  (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[9] =  (char)0x00;	// 
				CameraAbsBytes[10] = (char)0x00;	// Tilt -----------------------------------
				CameraAbsBytes[11] = (char)0x00;	// Range: 0xFED4 to 0x012C = -300 <-> +300
				CameraAbsBytes[12] = (char)0x00;	// every high nibble is zero: 0Y 0Y 0Y 0Y
				CameraAbsBytes[13] = (char)0x00;	// 
				CameraAbsBytes[14] = (char)0xFF;	// Terminator

				// Loop and shift, copying one Nibble at a time into the buffer!
				for(int Nibble=0; Nibble < 4; Nibble++ )
				{
					CameraAbsBytes[(9-Nibble)] = (PanPos & 0x000F);
					PanPos =  PanPos >> 4;

					CameraAbsBytes[(13-Nibble)] = (TiltPos & 0x000F);
					TiltPos = TiltPos >> 4;
				}
				
				SendCameraCmd(CameraAbsBytes, 15);
//				ROBOT_LOG( TRUE,"Debug Camera - sent Pan+Tilt Command, Pan = %04d, Tilt = %04d\n", wParam, lParam )
				break;
			}

			case HW_CAMERA_INITIALIZE:
			{
				// Send Camera Initialization string
				char CameraBytes[4];
				CameraBytes[0] = (char)0x88; 
				CameraBytes[1] = (char)0x30; 
				CameraBytes[2] = (char)0x01; 
				CameraBytes[3] = (char)0xFF;

				for( int i=0; i<10; i++)
				{
					SendCameraCmd(CameraBytes, 4);		//TODO - is this really needed?
					Sleep(500);
				}
				CurrentPanPos = 0;
				CurrentTiltPos = 0;
				break;
			}
			case HW_SET_CAMERA_ZOOM:
			{
				int ZoomCmd = wParam;
				// TODO: int ZoomSpeed = lParam;
				char CameraBytes[6];
				CameraBytes[0] = (char)0x81;	// Header for Camera #1
				CameraBytes[1] = (char)0x01;	// Control Command
				CameraBytes[2] = (char)0x04;	// 04 = Zoom command
				CameraBytes[3] = (char)0x07;	// 07 = continuous zoom
								// Zoom Command will be inserted here
				CameraBytes[5] = (char)0xFF;	// Terminator

				if( CAMERA_ZOOM_STOP == ZoomCmd )
				{
						CameraBytes[4] = (char)0x00;	// Zoom Stop
						SendCameraCmd(CameraBytes, 6);
				}
				else if( CAMERA_ZOOM_IN == ZoomCmd )
				{
						CameraBytes[4] = (char)0x24;	// Zoom In TODO - Use ZoomSpeed (2-7) for low nibble
						SendCameraCmd(CameraBytes, 6);
				}
				else if( CAMERA_ZOOM_OUT == ZoomCmd )
				{
						CameraBytes[4] = (char)0x34;	// Zoom Out TODO - Use ZoomSpeed (2-7) for low nibble
						SendCameraCmd(CameraBytes, 6);
				}
				else
				{
						ROBOT_LOG( TRUE, "ERROR! Undefined WM_ROBOT_USER_CAMERA_ZOOM_CMD Option!\n")
				}
				break;
			}
///////////////////////////////////////////////////////////////////////////////////

			case HW_SET_CAMERA_ZOOM_ABS:
			{
				int ZoomLevel = wParam;		// Standard Range: 0 - 16
				// standard Zoom range is 0 to 16.  Map this to Sony range of 0 to 03FF (1023)
				ZoomLevel = ZoomLevel * 64;				
				// Limit values to max camera can handle
				if( ZoomLevel > SONY_CAMERA_MAX_ZOOM_ABS ) ZoomLevel = SONY_CAMERA_MAX_ZOOM_ABS;
				if( ZoomLevel < SONY_CAMERA_MIN_ZOOM_ABS ) ZoomLevel = SONY_CAMERA_MIN_ZOOM_ABS;
				ROBOT_LOG( TRUE,"Debug Camera - sent Abs Zoom Command, Position = %04x\n", (DWORD)ZoomLevel )

				// TODO: int ZoomSpeed = lParam;
				char CameraBytes[9];
				CameraBytes[0] = (char)0x81;	// Header for Camera #1
				CameraBytes[1] = (char)0x01;	// Control Command
				CameraBytes[2] = (char)0x04;	// 04 = Zoom command
				CameraBytes[3] = (char)0x47;	// 47 = Absolute Zoom
				CameraBytes[4] = (char)0x00;	// Absolute Zoom will be inserted here
				CameraBytes[5] = (char)0x00;	// Absolute Zoom will be inserted here
				CameraBytes[6] = (char)0x00;	// Absolute Zoom will be inserted here
				CameraBytes[7] = (char)0x00;	// Absolute Zoom will be inserted here
				CameraBytes[8] = (char)0xFF;	// Terminator

				// Loop and shift, copying one Nibble at a time into the buffer!
				// Note data is Sparsely Populated, and reverse-indian.  13AF ==> 0F 0A 03 01
				for(int Nibble=0; Nibble < 4; Nibble++ )
				{
					CameraBytes[(7-Nibble)] = (ZoomLevel & 0x000F);
					ZoomLevel =  ZoomLevel >> 4;
				}

				SendCameraCmd(CameraBytes, 9);
				break;
			}
///////////////////////////////////////////////////////////////////////////////////

			case HW_SET_CAMERA_POWER:
			{
				int PowerCmd = lParam;
				// wParam not used
				char CameraBytes[6];
				CameraBytes[0] = (char)0x81;	// Header for Camera #1
				CameraBytes[1] = (char)0x01;	// Control Command
				CameraBytes[2] = (char)0x04;	// 04 00 = Power command
				CameraBytes[3] = (char)0x00;
								// Command will be inserted here
				CameraBytes[5] = (char)0xFF;	// Terminator

				CurrentPanPos = 0;
				CurrentTiltPos = 0;

				if( POWER_ON == PowerCmd )
				{
					CameraBytes[4] = (char)0x02;	// Off
					SendCameraCmd(CameraBytes, 6);
				}
				else if( POWER_OFF == PowerCmd )
				{
					CameraBytes[4] = (char)0x03;	// On
					SendCameraCmd(CameraBytes, 6);
				}
				else if( CAMERA_RESET == PowerCmd )
				{
					CameraBytes[0] = (char)0x88;	// Header for all Cameras (broadcast message)
					CameraBytes[1] = (char)0x01;	// Control Command
					CameraBytes[2] = (char)0x00;	// 00 01 = Clear command
					CameraBytes[3] = (char)0x01;	
					CameraBytes[4] = (char)0xFF;	// Terminator
					SendCameraCmd(CameraBytes, 5);
				}
				else	
				{
						ROBOT_LOG( TRUE, "ERROR! Undefined WM_ROBOT_CAMERA_POWER_CMD Option!\n")
				}

				break;
			}
			case HW_SET_CAMERA_PAN_TILT:
			{
				int Direction = wParam;	// Pan/Tilt direction
				BYTE nPanTiltSpeed = lParam;
				char CameraBytes[9];
				CameraBytes[0] = (char)0x81;	// Header for Camera #1
				CameraBytes[1] = (char)0x01;	// Control Command
				CameraBytes[2] = (char)0x06;	// 06 01 = Pan/Tilt command
				CameraBytes[3] = (char)0x01;
				CameraBytes[4] = (char)nPanTiltSpeed;	// Pan speed
				CameraBytes[5] = (char)nPanTiltSpeed;	// Tilt speed
								// Pan Command will be inserted here
				CameraBytes[8] = (char)0xFF;	// Terminator
				CurrentPanPos = CAMERA_UNKNOWN_POSITION;	// scan does not leave us in a known position
				CurrentTiltPos = CAMERA_UNKNOWN_POSITION;

				switch( Direction )  // Which direction to Pan/Tilt
				{

					case CAMERA_PAN_STOP:
					{
						CameraBytes[6] = (char)0x03;	// No Pan
						CameraBytes[7] = (char)0x03;	// No Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					// Pan or Tilt Commands

					case CAMERA_PAN_UP:
					{
						CameraBytes[6] = (char)0x03;	// No Pan
						CameraBytes[7] = (char)0x01;	// Up
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_DOWN:
					{
						CameraBytes[6] = (char)0x03;	// No Pan
						CameraBytes[7] = (char)0x02;	// Down
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_LEFT:
					{
						CameraBytes[6] = (char)0x01;	// Pan Left
						CameraBytes[7] = (char)0x03;	// No Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_RIGHT:
					{
						CameraBytes[6] = (char)0x02;	// Pan Right
						CameraBytes[7] = (char)0x03;	// No Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

				// Combo Pan/Tilt commands

					case CAMERA_PAN_UP_LEFT:
					{
						CameraBytes[6] = (char)0x01;	// Pan
						CameraBytes[7] = (char)0x01;	// Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_UP_RIGHT:
					{
						CameraBytes[6] = (char)0x02;	// Pan
						CameraBytes[7] = (char)0x01;	// Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_DOWN_LEFT:
					{
						CameraBytes[6] = (char)0x01;	// Pan
						CameraBytes[7] = (char)0x02;	// Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_DOWN_RIGHT:
					{
						CameraBytes[6] = (char)0x02;	// Pan
						CameraBytes[7] = (char)0x02;	// Tilt
						SendCameraCmd(CameraBytes, 9);
						break;
					}

					case CAMERA_PAN_ABS_CENTER:
					{
						char CameraAbsBytes[15];
						CameraAbsBytes[0] =  (char)0x81;	// Header for Camera #1
						CameraAbsBytes[1] =  (char)0x01;	// Control Command
						CameraAbsBytes[2] =  (char)0x06;	// 06 02 = Pan/Tilt Absolute command
						CameraAbsBytes[3] =  (char)0x02;
						CameraAbsBytes[4] =  (char)0x11;	// Pan speed (0-18h)
						CameraAbsBytes[5] =  (char)0x0D;	// Tilt speed (0-14h)
						CameraAbsBytes[6] =  (char)0x00;	// Pan Center 0000
						CameraAbsBytes[7] =  (char)0x00;	// Range: 0x0370 - 0xFC90
						CameraAbsBytes[8] =  (char)0x00;	// 
						CameraAbsBytes[9] =  (char)0x00;	// 
						CameraAbsBytes[10] = (char)0x00;	// Tilt Center 0000
						CameraAbsBytes[11] = (char)0x00;	// Range: 0x012C - 0xFED4
						CameraAbsBytes[12] = (char)0x00;	// 
						CameraAbsBytes[13] = (char)0x00;	// 
						CameraAbsBytes[14] = (char)0xFF;	// Terminator
						SendCameraCmd(CameraAbsBytes, 15);
						CurrentPanPos = 0;
						CurrentTiltPos = 0;
						break;
					}

					default:
					{
						ROBOT_LOG( TRUE, "ERROR! Invalid Camera Pan Direction, wParam = 0x%08lX\n", wParam )
					}
					break;
				}
			}	
			break;

			default:
			{
				CString StrText;
				StrText.Format( "ERROR! CameraCommWriteThread: Unknown Cmd:%02X \n", Request);
				ROBOT_DISPLAY( TRUE, (StrText)
			}

		}
	}
	return 0;
}
#endif	// CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA



////////////////////////////////////////////////////////////////////////////////////////////
//                             WHEEL MOTOR CONTROL
////////////////////////////////////////////////////////////////////////////////////////////
#if MOTOR_CONTROL_TYPE == ER1_MOTOR_CONTROL
	// For ER1 Pilot motor controller
	DWORD WINAPI MotorCommThreadFunc(LPVOID lpParameter) // g_dwMotorCommThreadId
	{
		// Writes a command to the ER1 Pilot Motor controller and
		// Reads a response to each command
		// Messages are sent to this thread by SendHardwareCmd calls
		__itt_thread_set_name( "ER1 Motor Thread" );

		MSG		msg;
	//	char	CmdBuf[16];
	//	DWORD	dwBytesWritten;
		UINT	Request;
		UINT	Index;
		UINT	Value;
		UINT	Option1;
		UINT	Option2;
	//	WORD	TempWord;
		CString strStatus;
		int		DbgDotCount = 0;
		BYTE MotorNumber = 0;
		BYTE Position = 0;


		CPilotControl PilotControl;	// PMD Pilot Command handler Class
		Sleep(1000);			// Allow other initialization to complete
		PilotControl.Init();	// Send Initialization commands

		// Process message loop for this thread


	/*		if ( INVALID_HANDLE_VALUE == g_hMotorCommPort )
			{
				// Usually, we should get a WM_QUIT before the COMM port closes, 
				// but if not, go ahead and exit...
				ROBOT_LOG( TRUE, "PILOT COMM WRITE: Port INVALID HANDLE \n")
				return 0;	
			}
	*/


			// TODO!  TEST THIS!
			// WARNING! WARNING!  CHANGED THIS FROM UNPACKING BYTES TO USING WPARAM AND LPARAM DIRECTLY!
			// THIS MAY BREAK Arduino COMMUNICATION - NEED TO TEST!!!
	//		TempWord = (WORD)(msg.lParam);
	//		Index = HIBYTE(msg.wParam);
	//		Value = LOBYTE(msg.wParam);
	//		Option1 = HIBYTE(TempWord);
	//		Option2 = LOBYTE(TempWord);
			Index = (UINT)(msg.wParam);
			Value = (UINT)(msg.lParam);
			Option1 = 0;
			Option2 = 0;


			PilotControl.HandlePilotCommand( Request, Index, Value, Option1, Option2 );

			Sleep(0); // force thread switch after each command
		}
		ROBOT_LOG( TRUE, "PILOT MOTOR COMM THREAD: Received WM_QUIT\n")
		return 0;
	}


////////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == POLOLU_TREX_MOTOR_CONTROL
	__itt_string_handle* pshMotorThreadLoop = __itt_string_handle_create("Motor Loop");

	// For Pololu TReX motor controller
	DWORD WINAPI MotorCommThreadFunc(LPVOID lpParameter) //g_dwMotorCommThreadId
	{
		///TAL_SetThreadName("Motor Thread");
		// Writes a command to the Pololu TReX Motor controller and
		// Reads a responses as needed
		// Messages are sent to this thread by SendHardwareCmd calls
		IGNORE_UNUSED_PARAM(lpParameter);
		__itt_thread_set_name( "Wheel Motor Thread" );

		MSG		msg;
	//	char	CmdBuf[16];
	//	DWORD	dwBytesWritten;
		UINT	Request;
		UINT	Index;
		UINT	Value;
	//	UINT	Option1;
	//	UINT	Option2;
	//	WORD	TempWord;
		CString strStatus;
		int		DbgDotCount = 0;
		BYTE MotorNumber = 0;
		BYTE Position = 0;


		CTrexControl TrexControl;	// Pololu Trex Command handler Class
		RobotSleep(2000, pDomainMotorThread);				// Allow other initialization to complete
		TrexControl.Init();			// Send Initialization commands

		// Process message loop for this thread
		while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
			// OK to filter out Windows messages, since we don't use WM_QUIT
		{
			//TAL_Event("Sending Cmd");
			///TAL_SCOPED_TASK_NAMED("Motor CMD Loop");
			__itt_task_begin(pDomainMotorThread, __itt_null, __itt_null, pshMotorThreadLoop);


			if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
			{
				break; // Quit command received!
			}

	/*		if ( INVALID_HANDLE_VALUE == g_hMotorCommPort )
			{
				// Usually, we should get a WM_QUIT before the COMM port closes, 
				// but if not, go ahead and exit...
				ROBOT_LOG( TRUE, "TReX COMM WRITE: Port INVALID HANDLE \n")
				return 0;	
			}
	*/
			if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
			{
				// Ignore any messages not intended for user (where do these come from?)
				ROBOT_LOG( TRUE, "MotorCommThreadFunc: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
					WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
				__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
				continue;
			}

			Request = msg.message - WM_ROBOT_MESSAGE_BASE;
			Index = (UINT)(msg.wParam);
			Value = (UINT)(msg.lParam);

			{
				///TAL_SCOPED_TASK_NAMED("Trex HandleCmd");
				//TAL_Event("Handle Motor Command");
				TrexControl.HandleCommand( Request, Index, Value );
			}

			__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
			Sleep(0);
		}

		ROBOT_LOG( TRUE, "TREX MOTOR COMM THREAD: Received WM_ROBOT_THREAD_EXIT_CMD\n")
		return 0;
	}

////////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL
	__itt_string_handle* pshMotorThreadLoop = __itt_string_handle_create("Motor Loop");

	// For iRobot Create motor controller
	DWORD WINAPI MotorCommThreadFunc(LPVOID lpParameter)	// g_hMotorCommPort, g_dwMotorCommThreadId
	{
		///TAL_SetThreadName("Motor Thread");
		// Writes a command to the iRobot base controller
		// Status message are read by a separate thread
		// Messages are sent to this thread by SendHardwareCmd calls
		IGNORE_UNUSED_PARAM(lpParameter);
		__itt_thread_set_name( "Wheel Motor Thread" );

		MSG		msg;
		UINT	Request;
		UINT	Index;
		UINT	Value;
		CString strStatus;
		//int		DbgDotCount = 0;
		//BYTE	MotorNumber = 0;
		//BYTE	Position = 0;

		CiRobotControl iRobotControl;	// iRobot Command handler Class
		//Sleep(100);					// Allow other initialization to complete
		ROBOT_LOG( TRUE,"\n\n************* INIT IROBOT *****************\n\n")
		iRobotControl.Init();			// Send Initialization commands - starts the status read stream

		// Process message loop for this thread
		while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
			// OK to filter out Windows messages, since we don't use WM_QUIT
		{
			//TAL_Event("Sending Cmd");
			///TAL_SCOPED_TASK_NAMED("Motor CMD Loop");
			__itt_task_begin(pDomainMotorThread, __itt_null, __itt_null, pshMotorThreadLoop);


			if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
			{
				break; // Quit command received!
			}

	/*		if ( INVALID_HANDLE_VALUE == g_hMotorCommPort )
			{
				// Usually, we should get a WM_QUIT before the COMM port closes, 
				// but if not, go ahead and exit...
				ROBOT_LOG( TRUE, "TReX COMM WRITE: Port INVALID HANDLE \n")
				return 0;	
			}
	*/
			if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
			{
				// Ignore any messages not intended for user (where do these come from?)
				ROBOT_LOG( TRUE, "MotorCommThreadFunc: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
					WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
				__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
				continue;
			}

			Request = msg.message - WM_ROBOT_MESSAGE_BASE;
			Index = (UINT)(msg.wParam);
			Value = (UINT)(msg.lParam);

			{
				///TAL_SCOPED_TASK_NAMED("Trex HandleCmd");
				//TAL_Event("Handle Motor Command");
				iRobotControl.HandleCommand( Request, Index, Value );
			}

			__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
			Sleep(0);
		}

		ROBOT_LOG( TRUE, "TREX MOTOR COMM THREAD: Received WM_ROBOT_THREAD_EXIT_CMD\n")
		return 0;
	}

////////////////////////////////////////////////////////////////////////////////////////
__itt_string_handle* pshiRobotReadThreadLoop = __itt_string_handle_create("iRobot Read Loop");
__itt_string_handle* pshiRobotParseBuffer = __itt_string_handle_create("iRobot Parse");
DWORD WINAPI iRobotCommReadThreadFunc(LPVOID lpParameter) // g_hMotorCommPort
{
	IGNORE_UNUSED_PARAM (lpParameter);
	__itt_thread_set_name( "iRobot Read Thread" );

//	HANDLE hCommPort = (HANDLE)lpParameter;	// Use Global instead
//	DWORD	fdwCommMask;
	DWORD	dwSIOBytesReceived = 0;
	char *iRobotSIOBuf = new char[IROBOT_READ_BUF_SIZE];
	memset( iRobotSIOBuf,0,IROBOT_READ_BUF_SIZE );

	// Create the iRobot Parser object
	CiRobotParser *piRobotParser = new CiRobotParser;

	DWORD dwBytesReceived;

	if( 1 != ROBOT_SIMULATION_MODE )
	{
		SetCommMask( g_hMotorCommPort, EV_RXCHAR | EV_BREAK );
	}
	Sleep(500);	// Alow other initialization to occur

	while(g_hMotorCommPort != INVALID_HANDLE_VALUE)
	{
		///TAL_Event("Laser Read Loop Top");
		///TAL_SCOPED_TASK_NAMED("Laser Read Loop");
		__itt_task_begin(pDomainMotorThread, __itt_null, __itt_null, pshiRobotReadThreadLoop);

		dwBytesReceived = 0;
		iRobotSIOBuf[0] = '\0';
		//memset( iRobotSIOBuf,0,IROBOT_READ_BUF_SIZE );	// DEBUG

		BOOL bSucess = 0;
		{
			if( 1 == ROBOT_SIMULATION_MODE )
			{
				Sleep(SIMULATED_SIO_SLEEP_TIME);
				continue;
			}

			bSucess = ReadFile(g_hMotorCommPort,		// where to read from (the open comm port)
					 iRobotSIOBuf,						// where to put the character
					 (IROBOT_READ_BUF_SIZE-1),			// number of bytes to read
					 &dwSIOBytesReceived,				// how many bytes were actually read
					 NULL);								// no overlapped I/O
		}
		if( !bSucess ) 
		{
			PurgeComm(g_hMotorCommPort, PURGE_RXABORT|PURGE_RXCLEAR);
			//PurgeComm(hCommPort, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
			DWORD dwCommError = 0;
			ClearCommError(g_hMotorCommPort, &dwCommError, NULL);
			ReportCommError("Reading iRobot Comm port.", dwCommError );
		}
		else if( 0 != dwSIOBytesReceived )
		{
			//ROBOT_LOG( TRUE,"iROBOT Read Thread: %d bytes received\n", dwSIOBytesReceived )

			// Got some data.  Parse it.
			__itt_task_begin(pDomainMotorThread, __itt_null, __itt_null, pshiRobotParseBuffer);
			if( piRobotParser->ParseBuffer(iRobotSIOBuf, dwSIOBytesReceived) )
			{
				// Recieved binary status
				g_MotorSubSystemStatus = SUBSYSTEM_CONNECTED;
			}
			else
			{
				g_MotorSubSystemStatus = SUBSYSTEM_FAILED;
			}
			__itt_task_end(pDomainMotorThread);  // pshiRobotParseBuffer
		}

		__itt_task_end(pDomainMotorThread);  // pshLaserReadThreadLoop
		Sleep(30); // TODO!!! don't spin in a tight loop!  iRobot sends data every 15ms - IS THIS OK?  DOES ReadFile sleep?
	}	// while port open


	ROBOT_LOG( TRUE,"iROBOT COMM Read Thread exiting.\n")
	SAFE_DELETE( piRobotParser );
	SAFE_DELETE( iRobotSIOBuf );
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////
#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL
	__itt_string_handle* pshMotorThreadLoop = __itt_string_handle_create("Motor Loop");

	// For Kobuki motor controller
	DWORD WINAPI MotorCommThreadFunc(LPVOID lpParameter)	// g_hMotorCommPort, g_dwMotorCommThreadId
	{
		///TAL_SetThreadName("Motor Thread");
		// Writes a command to the Kobuki base controller
		// Status message are read by a separate thread
		// Messages are sent to this thread by SendHardwareCmd calls
		IGNORE_UNUSED_PARAM(lpParameter);
		__itt_thread_set_name( "Wheel Motor Thread" );

		MSG		msg;
		UINT	Request;
		UINT	Index;
		UINT	Value;
		CString strStatus;
		//int		DbgDotCount = 0;
		//BYTE	MotorNumber = 0;
		//BYTE	Position = 0;

		CKobukiControl KobukiControl;	// Kobuki Command handler Class
		//Sleep(100);					// Allow other initialization to complete
		ROBOT_LOG( TRUE,"\n\n************* INIT KOBUKI *****************\n\n")
		KobukiControl.Init();			// Send Initialization commands - starts the status read stream

		// Process message loop for this thread
		while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
			// OK to filter out Windows messages, since we don't use WM_QUIT
		{
			//TAL_Event("Sending Cmd");
			///TAL_SCOPED_TASK_NAMED("Motor CMD Loop");
			__itt_task_begin(pDomainMotorThread, __itt_null, __itt_null, pshMotorThreadLoop);


			if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
			{
				break; // Quit command received!
			}

			if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
			{
				// Ignore any messages not intended for user (where do these come from?)
				ROBOT_LOG( TRUE, "MotorCommThreadFunc: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
					WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
				__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
				continue;
			}

			Request = msg.message - WM_ROBOT_MESSAGE_BASE;
			Index = (UINT)(msg.wParam);
			Value = (UINT)(msg.lParam);

			{
				///TAL_SCOPED_TASK_NAMED("Trex HandleCmd");
				//TAL_Event("Handle Motor Command");
				KobukiControl.HandleCommand( Request, Index, Value );
			}

			__itt_task_end(pDomainMotorThread);  // pshMotorThreadLoop
			Sleep(0);
		}

		ROBOT_LOG( TRUE, "TREX MOTOR COMM THREAD: Received WM_ROBOT_THREAD_EXIT_CMD\n")
		return 0;
	}

#elif( MOTOR_CONTROL_TYPE == ARDUINO_MOTOR_CONTROL )
	// Do nothing, handled by the Arduino Thread

#else
	#error BAD SENSOR_CONFIG_TYPE!  
#endif  // MOTOR_CONTROL_TYPE



////////////////////////////////////////////////////////////////////////////////////////////
// For Smart Servos
__itt_string_handle* pshSmartServoThreadLoop = __itt_string_handle_create("SmartServo Loop");
__itt_string_handle* pshDynaInit = __itt_string_handle_create("Dyna Init");
__itt_string_handle* pshDynaHandleCmd = __itt_string_handle_create("Dyna HandleCmd");
__itt_string_handle* pshKerrInit = __itt_string_handle_create("Kerr Init");
__itt_string_handle* pshKerrHandleCmd = __itt_string_handle_create("Kerr HandleCmd");


DWORD WINAPI SmartServoCommThreadFunc(LPVOID lpParameter)	// g_dwSmartServoCommThreadId
{
	// Writes a command to the Dynamixel and Kerr (Shoulder) Servos
	// and reads a response to each command
	// Messages are sent to this thread by SendHardwareCmd calls
	IGNORE_UNUSED_PARAM(lpParameter);
	__itt_thread_set_name( "Smart Servo Thread" );

	MSG		msg;
	UINT	Request;
	UINT	Index;
	UINT	Value;
	UINT	Option1;
	UINT	Option2;
	CString strStatus;
	int		DbgDotCount = 0;
	BYTE	MotorNumber = 0;
	BYTE	Position = 0;
	BOOL	ServosPaused = FALSE; // for Global Pause Mode

	CDynaControl* pDynaControl = new CDynaControl(); ;	// Dynamixel Control handler

#if( TURTLE != ROBOT_TYPE )
	CKerrControl* pKerrControl = new CKerrControl();	// Kerr Control handler
	RobotSleep(100, pDomainSmartServoThread);	// Allow other initialization to complete

	__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshDynaInit);
	pDynaControl->Init();			// Send Initialization commands
	__itt_task_end(pDomainSmartServoThread);

	RobotSleep(100, pDomainSmartServoThread);					// Allow other initialization to complete

	__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshKerrInit);
	pKerrControl->Init();			// Send Initialization commands
	__itt_task_end(pDomainSmartServoThread);

#else
	RobotSleep(100, pDomainSmartServoThread);					// Allow other initialization to complete
	__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshDynaInit);
	pDynaControl->Init();			// Send Initialization commands
	__itt_task_end(pDomainSmartServoThread);

#endif

	// Process message loop for this thread
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		///TAL_SCOPED_TASK_NAMED("Smart Servo Loop");
		//TAL_Event("Sending Cmd");
		__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshSmartServoThreadLoop);

		Request = msg.message - WM_ROBOT_MESSAGE_BASE;
		// ROBOT_LOG( TRUE, "DEBUG SmartServoCommThreadFunc: msg.message = %04X \n", msg.message)

		if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
		{
			break; // Quit command received!
		}

		// Usually, we should get a WM_QUIT before the COMM port closes, but if not, go ahead and exit...
		if ( INVALID_HANDLE_VALUE == g_hDynaServoCommPort_AX12 )
		{
			ROBOT_LOG( TRUE, "SmartServoCommThreadFunc: AX12 Port INVALID HANDLE g_hSmartServoCommPort_AX12\n")
			__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
			break;	
		}
		#if( LOKI == ROBOT_TYPE )
			#if ( DYNA_SERVO_RX64_INSTALLED == 1 )
				if ( INVALID_HANDLE_VALUE == g_hDynaServoCommPort_RX64 )
				{
					ROBOT_LOG( TRUE, "SmartServoCommThreadFunc: RX64 Port INVALID HANDLE g_hSmartServoCommPort_RX64\n")
					__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
					break;	
				}
			#endif

			if ( INVALID_HANDLE_VALUE == g_hKerrServoCommPort )
			{
				ROBOT_LOG( TRUE, "SmartServoCommThreadFunc: KERR Port INVALID HANDLE g_hKerrServoCommPort\n")
				__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
				break;	
			}
		#endif

		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
		{
			// Ignore any messages not intended for user (where do these come from?)
			ROBOT_LOG( TRUE, "SmartServoCommThreadFunc: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
			__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
			continue;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Handle Global Pause Mode ("Freeze the robot")
		if( g_GlobalPause )
		{
			// Global Pause requested!
			if( !ServosPaused )
			{
				// Not yet paused, so stop movement
				ServosPaused = TRUE;
				#if( LOKI == ROBOT_TYPE )
					//pKerrControl->PauseMotion( TRUE );
				#endif
				pDynaControl->PauseMotion( TRUE );
				Sleep(0); // force thread switch
			}
			continue;
		}
		else if( ServosPaused )
		{
				// Servos were paused, now resume
				ServosPaused = FALSE;
				#if( LOKI == ROBOT_TYPE )
					//pKerrControl->PauseMotion( FALSE );
				#endif
				pDynaControl->PauseMotion( FALSE );

				Sleep(0); // force thread switch
		}
		////////////////////////////////////////////////////////////////////////////////


		Index = (UINT)(msg.wParam);
		Value = (UINT)(msg.lParam);
		Option1 = 0;
		Option2 = 0;


		if( HW_SET_CAMERA_PAN_TILT_ABS == Request )
		{
			ROBOT_ASSERT(0);
			// Not supported.
		}

		///TAL_SCOPED_TASK_NAMED("Handle SmartServoCMD");
		//TAL_Event("HandleSmartServoCommand");

		if( (HW_SET_POWER_MODE == Request) && (POWER_ON == Index) ) // "Wake Up" command  
		{
			// Wake up command needs Dynamixel first.  All others want Shoulders first
			__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshDynaHandleCmd);
			pDynaControl->HandleCommand( Request, Index, Value, Option1, Option2 );
			__itt_task_end(pDomainSmartServoThread);  // pshDynaHandleCmd

			#if( LOKI == ROBOT_TYPE )
				__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshKerrHandleCmd);
				pKerrControl->HandleCommand( Request, Index, Value, Option1, Option2 );
				__itt_task_end(pDomainSmartServoThread);  // pshKerrHandleCmd
			#endif
		}
		else if( (HW_SET_POWER_MODE == Request) && (SYSTEM_SHUT_DOWN == Index) ) // "Shut Down" command
		{
			// Move arms to "Rest" position
			#if( LOKI == ROBOT_TYPE )
				Option1 = 0; // Override: Option1 == 0 means Return to "Rest" position
				pKerrControl->HandleCommand( Request, Index, Value, Option1, Option2 );
			#endif
			pDynaControl->HandleCommand( Request, Index, Value, Option1, Option2 );

			Sleep(3000); // give time for servos to move into position

			// Turn off Servo Torque
			#if( LOKI == ROBOT_TYPE )
				Option1 = 1; // Override: Option1 == 1 means turn off torque
				pKerrControl->HandleCommand( Request, Index, Value, Option1, Option2 );
			#endif

		}
		else // Normal command
		{	
			#if( LOKI == ROBOT_TYPE )
				__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshKerrHandleCmd);
				pKerrControl->HandleCommand( Request, Index, Value, Option1, Option2 );
				__itt_task_end(pDomainSmartServoThread);  // pshKerrHandleCmd
			#endif

			__itt_task_begin(pDomainSmartServoThread, __itt_null, __itt_null, pshDynaHandleCmd);
			pDynaControl->HandleCommand( Request, Index, Value, Option1, Option2 );
			__itt_task_end(pDomainSmartServoThread);  // pshDynaHandleCmd
		}

		__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
		Sleep(0); // force thread switch after each command
	}

	__itt_task_end(pDomainSmartServoThread);  // pshSmartServoThreadLoop
	ROBOT_LOG( TRUE, "Smart Servo Control Thread: Received WM_ROBOT_THREAD_EXIT_CMD\n")
	CloseCommPort( "Dynamixel AX12" , g_hDynaServoCommPort_AX12 );
	//CloseCommPort( "Dynamixel RX64" , g_hDynaServoCommPort_RX64 );
	CloseCommPort( "Kerr" , g_hKerrServoCommPort );

	SAFE_DELETE( pDynaControl );
	#if( TURTLE != ROBOT_TYPE )
		SAFE_DELETE( pKerrControl );
	#endif

	ROBOT_LOG( TRUE, "SmartServoCommThreadFunc: Smart Servo COMM ports closed.  Exiting thread.\n")

	return 0;
}



////////////////////////////////////////////////////////////////////////////////////////////
// For Hokuyo URG-04LX-UG01 Laser Scanner
__itt_string_handle* pshLaserWriteThreadLoop = __itt_string_handle_create("Laser Write Loop");
__itt_string_handle* pshLaserHandleCommand = __itt_string_handle_create("Handle Command");
DWORD WINAPI LaserScannerCommWriteThreadFunc(LPVOID lpParameter)	// g_dwLaserScannerCommWriteThreadId, g_hLaserScannerCommPort
{
	///TAL_SetThreadName("Laser Write Thread");
	// Writes a command to the Laser Scanner
	// Messages are sent to this thread by SendHardwareCmd calls
	IGNORE_UNUSED_PARAM(lpParameter);
	__itt_thread_set_name( "Laser Write Thread" );

	MSG		msg;
//	char	CmdBuf[16];
//	DWORD	dwBytesWritten;
	UINT	Request;
	CString strStatus;

	CLaserScannerCommand LaserScannerCommand;	// Laser Scanner Control handler Class
	RobotSleep(5000, pDomainLaserThread);								// Allow other initialization to complete
	//RobotSleep(5000, pDomainLaserThread);								
	LaserScannerCommand.Init();					// Send Initialization commands (reset?)

	// Process message loop for this thread
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		///TAL_SCOPED_TASK_NAMED("Laser Write Loop");
		//TAL_Event("Sending Cmd");
		__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, pshLaserWriteThreadLoop);


		Request = msg.message - WM_ROBOT_MESSAGE_BASE;

//		ROBOT_LOG( TRUE, "DEBUG LaserScannerCommWriteThreadFunc: msg.message = %04X \n", msg.message)

		if( WM_ROBOT_THREAD_EXIT_CMD == msg.message ) 
		{
			break; // Quit command received!
		}

		if ( INVALID_HANDLE_VALUE == g_hLaserScannerCommPort )
		{
			// Usually, we should get a WM_QUIT before the COMM port closes, 
			// but if not, go ahead and exit...
			ROBOT_LOG( TRUE, "LaserScannerCommWriteThreadFunc: Port INVALID HANDLE \n")
			return 0;	
		}

		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message > (WM_ROBOT_MESSAGE_BASE+HW_MAX_MESSAGE) ) ) )
		{
			// Ignore any messages not intended for user (where do these come from?)
			ROBOT_LOG( TRUE, "LaserScannerCommWriteThreadFunc: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				WM_ROBOT_MESSAGE_BASE, msg.message, msg.wParam, msg.lParam )
			continue;
		}

		{
			//TAL_Event("Handle Laser Command");
			__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, pshLaserHandleCommand);
			LaserScannerCommand.HandleCommand( Request, (msg.wParam), (msg.lParam) );
			__itt_task_end(pDomainLaserThread);
		}

		__itt_task_end(pDomainLaserThread);  // pshLaserWriteThreadLoop
		Sleep(0); // force thread switch after each command
	}
	ROBOT_LOG( TRUE, "Laser Scanner Control Thread: Received WM_ROBOT_THREAD_EXIT_CMD\n")
	CloseCommPort( "Laser" , g_hLaserScannerCommPort );

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
__itt_string_handle* pshLaserReadThreadLoop = __itt_string_handle_create("Laser Read Loop");
__itt_string_handle* pshLaserParseBuffer = __itt_string_handle_create("Parse Buffer");
DWORD WINAPI LaserScannerCommReadThreadFunc(LPVOID lpParameter) // g_hLaserScannerCommPort
{
	///TAL_SetThreadName("Laser Read Thread");
	IGNORE_UNUSED_PARAM (lpParameter);
	__itt_thread_set_name( "Laser Read Thread" );

//	HANDLE hCommPort = (HANDLE)lpParameter;	// Use Global instead
//	DWORD	fdwCommMask;
	DWORD	dwSIOBytesReceived = 0;
	//char	LaserSIOBuf[LASER_SCANNER_READ_BUF_SIZE+1];

	char *LaserSIOBuf = new char[LASER_SCANNER_READ_BUF_SIZE+1];
	memset( LaserSIOBuf,0,LASER_SCANNER_READ_BUF_SIZE );

	// Create the Laser Parser object
	CLaserScannerParser *pLaserScannerParser = new CLaserScannerParser;
	pLaserScannerParser->Init();

	DWORD dwBytesReceived;

	if( 1 == ROBOT_SIMULATION_MODE )
	{
		SetCommMask( g_hLaserScannerCommPort, EV_RXCHAR | EV_BREAK );
	}
	RobotSleep(1000, pDomainLaserThread);	// Alow other initialization to occur first

	while(g_hLaserScannerCommPort != INVALID_HANDLE_VALUE)
	{
		///TAL_Event("Laser Read Loop Top");
		///TAL_SCOPED_TASK_NAMED("Laser Read Loop");
		__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, pshLaserReadThreadLoop);

		dwBytesReceived = 0;
		LaserSIOBuf[0] = '\0';
		//memset( LaserSIOBuf,0,LASER_SCANNER_READ_BUF_SIZE );	// DEBUG
		//memset( SioBuffer, '\0', PIC_SIO_BUF_LEN );	// clean out garbage chars


		BOOL bSucess = 0;
		{
			///TAL_SCOPED_TASK_NAMED("Read LsrSIO");

			// Read ANSI characters
			if( 1 == ROBOT_SIMULATION_MODE )
			{
				RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainLaserThread);
				continue;
			}

			bSucess = ReadFile(g_hLaserScannerCommPort,	// where to read from (the open comm port)
					 LaserSIOBuf,						// where to put the character
					 (LASER_SCANNER_READ_BUF_SIZE-1),	// number of bytes to read
					 &dwSIOBytesReceived,				// how many bytes were actually read
					 NULL);								// no overlapped I/O
		}
		if( !bSucess ) 
		{

			ROBOT_DISPLAY( TRUE, "ERROR Reading Laser Comm port\n" )
			PurgeComm(g_hLaserScannerCommPort, PURGE_RXABORT|PURGE_RXCLEAR);
			//PurgeComm(hCommPort, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
			DWORD dwCommError = 0;
			ClearCommError(g_hLaserScannerCommPort, &dwCommError, NULL);
			ReportCommError("Laser SIO Read Error", dwCommError );

			return 0;		// terminate thread on error
		}

		if( 0 != dwSIOBytesReceived )
		{
			//ROBOT_LOG( TRUE,"LASER Read Thread: {%s}", LaserSIOBuf )
			//ROBOT_LOG( TRUE,"LASER Read Thread: %d bytes received\n", dwSIOBytesReceived )

			// Got some Laser data.  Parse it.
			__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, pshLaserParseBuffer);
			if( pLaserScannerParser->ParseBuffer( LaserSIOBuf, dwSIOBytesReceived ) )
			{
				// Indicate that scan is complete
				gWaitingForLaserSingleLineScan = FALSE;

				// Now throw message in the queue, to indicate that the Laser data has been updated
				//TODO!	PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_GPS_DATA_READY), 0, 0 );
				// Post message to the GUI display (local or remote)
				SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM,	// command
					ROBOT_RESPONSE_LASER_SCANNER_DATA,			// Param1 = Bulk data command
					0 );										// Param2 = not used
			}
			__itt_task_end(pDomainLaserThread);  // pshLaserParseBuffer
		}

		__itt_task_end(pDomainLaserThread);  // pshLaserReadThreadLoop
		if( ! g_bLaserContinuousScanEnabled )
		{
			RobotSleep(100, pDomainLaserThread); // just check every now and then, since auto laser not requested
		}
		else
		{
			// NOTE: Timing verified.  Don't check faster than this, as the laser needs time to get the data after a command is sent
			RobotSleep(20, pDomainLaserThread); // don't spin in a tight loop!
		}
	}	// while port open


	ROBOT_LOG( TRUE,"LASER SCANNER COMM Read Thread exiting.\n")
	SAFE_DELETE( pLaserScannerParser );
	SAFE_DELETE( LaserSIOBuf );
	return 0;
}




#endif // ROBOT_SERVER - nothing in this file used for client 

