// Utilities.cpp
// Utility funcitons used by Depth Viewer process

#include "stdafx.h"

#include <windows.h>
#include <tchar.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
#include <DepthCommon.h>

//#include "DepthCommon.h"

//#include "KobukiControl.h"

///////////////////////////////////////////////////////////////////////////////////////////////
// Initialize shared memory and events for Inter Process Communication with the Robot control application
int InitIPC( HANDLE &hCommandEvent, LPCTSTR &pCommandBuf, HANDLE &hStatusEvent, LPCTSTR &pStatusBuf )
{
	pCommandBuf = NULL;
	pStatusBuf = NULL;
	hCommandEvent = NULL;
	hStatusEvent = NULL;
	HANDLE hMapFile = NULL;

	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 


	// TODO!  DO I NEED TO RELEASE MEMORY OR HANDLES on EXIT?

	/////////////////////////////////////////////////////////////////////////////////////////
	// For sending Depth data to the robot control application
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size high
		sizeof(MAX_DEPTH_DATA_SIZE),	// buffer size  
		_T(KOBUKI_STATUS_SHARED_FILE_NAME) );// name of mapping object

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		_tprintf(TEXT("Could not create STATUS file mapping object (%d).\n"), 
			GetLastError());
		return FAILED;
	}
	pStatusBuf = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		sizeof(KOBUKI_BASE_STATUS_T) );           

	if (pStatusBuf == NULL) 
	{ 
		_tprintf(TEXT("Could not map view of Status file (%d).\n"), GetLastError()); 
		CloseHandle(hMapFile);
		return FAILED;
	}

	// Event to telling Robot that new Status data is ready
	hStatusEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(KOBUKI_STATUS_EVENT_NAME) );
	if ( !hStatusEvent ) 
	{ 
		cerr << "Event creation failed!: " << KOBUKI_STATUS_EVENT_NAME << endl;
		return FAILED;
	}
	SetEvent( hStatusEvent ); // Indicate that the app has started


	/////////////////////////////////////////////////////////////////////////////////////////
	// For getting Commands from the robot control application

	// Create Event to allow Robot to signal when a new Command is pending
	hCommandEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(KOBUKI_COMMAND_EVENT_NAME) );
	if ( !hCommandEvent ) 
	{ 
		cerr << "Event creation failed!: " << KOBUKI_COMMAND_EVENT_NAME << endl;
		return FAILED;
	}

	// Now check that the event was signaled by the Robot process, indicating that it's ok to proceed
	// If not, this program was probably launched in stand-alone mode for testing
	const DWORD msTimeOut = 100;
	DWORD dwResult = WaitForSingleObject(hCommandEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		cerr << endl;
		cerr << "==============================================================" << endl;
		cerr << "WARNING!  COMMAND Event failed, assuming STAND ALONE mode!"     << endl;
		cerr << "==============================================================" << endl << endl;
		return STAND_ALONE_MODE; 
	}

	// Open Memory Mapped File

	hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		_T(KOBUKI_COMMAND_SHARED_FILE_NAME) );	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		_tprintf(TEXT("Could not create COMMAND file mapping object (%d).\n"), 
			GetLastError());
		return FAILED;
	}
	pCommandBuf = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		sizeof(KOBUKI_COMMAND_T) );           

	if (pCommandBuf == NULL) 
	{ 
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError()); 
		CloseHandle(hMapFile);
		return FAILED;
	}
	else
	{
		cout << "COMMAND Shared Memory File Opened Sucessfully!" << endl;
	} 
	
	return SUCCESS; 
}
