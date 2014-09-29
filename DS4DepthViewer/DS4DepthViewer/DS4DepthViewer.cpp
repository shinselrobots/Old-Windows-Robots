// DS4DepthViewer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
//#include <csignal>
#include <windows.h>
//#include <tchar.h>
//#include <stdio.h>
#include <iostream>
//#include <fstream>
//#include <sstream>

using namespace std;

#include <DepthCommon.h>
#include "DS4DepthViewer.h"



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
{

	#if (DEBUG_MEMORY_LEAKS == 1 )
		_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	// Use to detect memory leaks!
	// _CrtSetBreakAlloc( Put allocator number here);

	// Global Variables
	LPCTSTR			pDepthDataSharedMemory = NULL;
	HANDLE			hDepthDataAvailableEvent = NULL;
	LPCTSTR			pCommandSharedMemory = NULL;
	HANDLE			hCommandEvent = NULL;
	DEPTH_COMMAND_T *LastCommand = new DEPTH_COMMAND_T;
	memset( LastCommand, 0, sizeof(DEPTH_COMMAND_T) );

//	signal(SIGINT, signalHandler);
	cout << "Starting Depth Viewer..." << endl;

	cout << "Initializing shared memory..." << endl;

	// Initialize shared memory and events for communicating with the Robot control application
	int InitIPCResult = InitIPC( hCommandEvent, pCommandSharedMemory, hDepthDataAvailableEvent, pDepthDataSharedMemory  );

	if ( FAILED == InitIPCResult )
	{
		cerr << "InitIPC failed.  Exiting" << endl;
		return -1;
	}

//	signal(SIGINT, signalHandler);


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

	Sleep(100); // let robot code start up first


///////////////////////////////////////////////////////////////////////////////////
	// Command and Status Processing Loop.  
	cout << "Beginning Control Loop..." << endl;
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
				DEPTH_COMMAND_T *Command = (DEPTH_COMMAND_T*)pCommandSharedMemory;

				// Process the command block

				if( 0 != Command->bShutDown )
				{
					// Exit application.  Kobuki destructor automatically stops motors
					cout << "COMMAND SHUTDOWN received from control app" << endl;
					bRunLoop = 0;
					continue;
				}

			/*
				if( Command->LEDState != LastCommand->LEDState )
				{
					cout << "KOBUKI COMMAND LED STATE = " << Command->LEDState << endl;
					KobukiManager.SetLedState( Command->LEDState );
					LastCommand->LEDState = Command->LEDState;
				}
			*/
			}
			else
			{
				// no command received.
			}
		}
		else
		{
			// Stand Alone mode - just show status for debug testing?
		}

		// Done processing commands.
		// Now do work that needs to happend every time through the loop


		Sleep(50);  // tune this to adjust update rate and avoid over saturating the CPU

	} // Command and Status Processing Loop




	// Handle shut down clean up
	cout << "Exiting..." << endl;

	SAFE_DELETE(LastCommand);

	Sleep(1000); // for debugging
	return 0;

}




