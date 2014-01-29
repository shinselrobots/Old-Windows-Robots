// HWInterfaceKobuki.cpp
// Interface to Kobuki interface program

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include "Globals.h"

#include "HardwareCmds.h"
#include "HWInterfaceParams.h"
#include "thread.h"
#include "module.h"


#include "MotorControl.h"
//#include "DynamixelControl.h"
#include "Module.h"


#include "HWInterface.h"
#include "HardwareConfig.h"
#include "KobukiCommon.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif




#define DEBUG_KOBUKI_SHARED_MEMORY 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread to communicate with Kobuki Control App, which runs only in non-debug mode

#if( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
DWORD WINAPI KobukiAppSharedMemoryIPCThreadProc( LPVOID NotUsed )
{
	__itt_thread_set_name( "KobukiSharedMemoryIPC Thread" );

	//	MSG	msg;
	//    HRESULT hr = E_FAIL;
	//	CString input, ResponseString, MsgString;
	unsigned int LastTimeStamp = 0;

	// wait for the other stuff to finish initializing
	RobotSleep(1000, pDomainControlThread); // allow other threads to start first

    // Create an event for synchronizing with the Kobuki App
	ROBOT_LOG( TRUE,  "Waiting for Kobuki App to start...\n" )
	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hKobukiDataEvent = CreateEvent ( NULL, bManualReset, bInitialState, KOBUKI_STATUS_EVENT_NAME );
	if ( !g_hKobukiDataEvent ) 
	{ 
		ROBOT_LOG( TRUE,  "Event creation failed!:  %s\n", KOBUKI_STATUS_EVENT_NAME )
		return 0; // exit thread
	}
	
	// Now wait for the event to be signaled by the Kobuki process, indicating that it's ok to proceed
	const DWORD msTimeOut = 10000;
	DWORD dwResult = WaitForSingleObject(g_hKobukiDataEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		if ( NULL == g_pKobukiDataSharedMemory )
		{
			ROBOT_LOG( TRUE,  "Kobuki App probably not running (g_pKobukiDataSharedMemory = NULL).  exiting IPC Thread\n" )
		}
		else
		{
			ROBOT_LOG( TRUE,  "Event Timed out or failed!:  %s, Error: %08X\n", KOBUKI_STATUS_EVENT_NAME, dwResult )
		}
		g_MotorSubSystemStatus = SUBSYSTEM_DISABLED;
		return 0; // exit thread
	}
	ROBOT_LOG( TRUE,  "Kobuki App start Success!\n" )


	// Open Memory Mapped File
	//TCHAR szKobukiInterfaceSharedFileName[]=TEXT(Kobuki_SHARED_FILE_NAME);
	char *pBuf;
//	int NextQueuedMessageFromApp = 0;	// Keep track of queue position

	HANDLE hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		_T(KOBUKI_STATUS_SHARED_FILE_NAME) );	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		//ROBOT_ASSERT(0);
		TRACE("/n/n");
		ROBOT_LOG( TRUE, "***********************************************************************" )
		ROBOT_LOG( TRUE,  "Could not open Kobuki file mapping object (%d).\n", GetLastError())
		ROBOT_LOG( TRUE, "******** FAILED TO OPEN IPC SHARED MEMORY! ********" )
		ROBOT_LOG( TRUE, "***********************************************************************\n" )
		return 0; // no sense wasting cycles, the MMF did not init correctly!
	}
	else
	{
		pBuf = (char*)MapViewOfFile(hMapFile, // handle to map object (LPCTSTR)
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,                    
			0,                    
			(sizeof(KOBUKI_BASE_STATUS_T)) );                   

		if (pBuf == NULL) 
		{ 
			ROBOT_LOG( TRUE,  "Could not map view of file (%d).\n", GetLastError());
			CloseHandle(hMapFile);
			return 0; // exit thread
		}
		else
		{
			ROBOT_DISPLAY( TRUE, "IPC Shared Memory File Opened Sucessfully!" )
		} 
	}


	///////////////////////////////////////////////////////////////////////////////////////
	// Loop forever
	ROBOT_LOG( TRUE,"Thread Loop starting.\n")
	while( g_bRunThread && (WAIT_OBJECT_0 == WaitForSingleObject(g_hKobukiDataEvent, INFINITE)) )
	{
		ROBOT_LOG( DEBUG_KOBUKI_SHARED_MEMORY,  "---------------------- g_hKobukiDataEvent Signaled ---------------------\n");

		// Read from Shared Memory
		KOBUKI_BASE_STATUS_T KobukiBaseStatus;
		CopyMemory(&KobukiBaseStatus, pBuf, (sizeof(KOBUKI_BASE_STATUS_T)));

		if( KobukiBaseStatus.TimeStamp != LastTimeStamp )
		{
			LastTimeStamp = KobukiBaseStatus.TimeStamp;


			// Now, throw the data 

			g_pKobukiStatus->BatteryChargeSourceEnum = KobukiBaseStatus.BatteryChargeSourceEnum;
			g_pKobukiStatus->BatteryChargeStateEnum =  KobukiBaseStatus.BatteryChargeStateEnum;
			g_pKobukiStatus->BatteryLevelEnum = 	   KobukiBaseStatus.BatteryLevelEnum;
			g_pKobukiStatus->BatteryPercent =		   KobukiBaseStatus.BatteryPercent;
			g_pKobukiStatus->BatteryVoltage =		   KobukiBaseStatus.BatteryVoltage;
			g_pKobukiStatus->ButtonPress =			   KobukiBaseStatus.ButtonPress;
			g_pKobukiStatus->CenterCliffA2D =		   KobukiBaseStatus.CenterCliffA2D;
			g_pKobukiStatus->GyroDegrees =			   KobukiBaseStatus.GyroDegrees;
			g_pKobukiStatus->DockCenterSignal =		   KobukiBaseStatus.DockCenterSignal;
			g_pKobukiStatus->DockLeftSignal =		   KobukiBaseStatus.DockLeftSignal;
			g_pKobukiStatus->DockRightSignal =		   KobukiBaseStatus.DockRightSignal;
			g_pKobukiStatus->LeftCliffA2D =			   KobukiBaseStatus.LeftCliffA2D;
			g_pKobukiStatus->MotorCurrentLeft =		   KobukiBaseStatus.MotorCurrentLeft;
			g_pKobukiStatus->MotorCurrentRight =	   KobukiBaseStatus.MotorCurrentRight;
			g_pKobukiStatus->OdometerLeft =			   KobukiBaseStatus.OdometerLeft;
			g_pKobukiStatus->MotorCurrentRight =	   KobukiBaseStatus.MotorCurrentRight;
			g_pKobukiStatus->MotorPWMLeft =			   KobukiBaseStatus.MotorPWMLeft;
			g_pKobukiStatus->MotorPWMRight =		   KobukiBaseStatus.MotorPWMRight;
			g_pKobukiStatus->OdometerLeft =			   KobukiBaseStatus.OdometerLeft;
			g_pKobukiStatus->OdometerRight =		   KobukiBaseStatus.OdometerRight;
			g_pKobukiStatus->OdometerTicksLeft =	   KobukiBaseStatus.OdometerTicksLeft;
			g_pKobukiStatus->OdometerTicksRight =	   KobukiBaseStatus.OdometerTicksRight;
			g_pKobukiStatus->RightCliffA2D =		   KobukiBaseStatus.RightCliffA2D;
			g_pKobukiStatus->TimeStamp = 			   KobukiBaseStatus.TimeStamp;
			g_pKobukiStatus->TurnRate =				   KobukiBaseStatus.TurnRate;

			//////////////////////////////////////////////////////////////////////////
			// Process bit fields

			// Bumpers
			g_pKobukiStatus->BumperRight = (KobukiBaseStatus.Bumper & 0x01) != 0;
			g_pKobukiStatus->BumperFront = (KobukiBaseStatus.Bumper & 0x02) != 0;
			g_pKobukiStatus->BumperLeft =  (KobukiBaseStatus.Bumper & 0x04) != 0;


			// Cliffs
			g_pKobukiStatus->CliffRight = (KobukiBaseStatus.Cliff & 0x01) != 0;
			g_pKobukiStatus->CliffFront = (KobukiBaseStatus.Cliff & 0x02) != 0;
			g_pKobukiStatus->CliffLeft =  (KobukiBaseStatus.Cliff & 0x04) != 0;

			// WheelDrop
			g_pKobukiStatus->WheelDropRight = (KobukiBaseStatus.WheelDrop & 0x01) != 0;
			g_pKobukiStatus->WheelDropLeft =  (KobukiBaseStatus.WheelDrop & 0x02) != 0;

			// Motor Over Current
			g_pKobukiStatus->MotorStallErrorRight = (KobukiBaseStatus.OverCurrentErrors & 0x01) != 0;
			g_pKobukiStatus->MotorStallErrorLeft =  (KobukiBaseStatus.OverCurrentErrors & 0x02) != 0;


			SendCommand( WM_ROBOT_SENSOR_STATUS_READY, 0, 0 );
			Sleep(100); // don't let this thread dominate
		}
		//ROBOT_LOG( TRUE,"DEBUG: Got Kobuki Data\n")

	}
	// exit when g_bRunThread flag cleared
	ROBOT_LOG( TRUE,"SharedMemoryIPC exiting.\n")

	return 0;

}	// End of Kobuki Control App IPC Thread

#endif //( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )

#endif // ( ROBOT_SERVER == 1 )