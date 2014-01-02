// KobukiControl.h: Kobuki Base interface
//////////////////////////////////////////////////////////////////////

#ifndef __KOBUKI_CONTROL_H__
#define __KOBUKI_CONTROL_H__

#include "stdafx.h"

// Constants
//static char *LogFileName = "c:\\temp\\RobotKobukiLog.txt";


enum INIT_RESULT { 
		FAILED = 0, 
		SUCCESS,
		STAND_ALONE_MODE
};

// Global variables



//extern FILE* g_LogFile;		// Log file for non-debug mode
//extern int	 gStartTime;	// Time that the robot started executing.  Used as baseline for time numbers

using namespace std;


#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }


// Use the following to debug memory leaks
#define DEBUG_MEMORY_LEAKS			0

#if (DEBUG_MEMORY_LEAKS == 1)
	#ifndef _CRTDBG_MAP_ALLOC 
		#define _CRTDBG_MAP_ALLOC_NEW
		#define _CRTDBG_MAP_ALLOC
	#endif
	#include <stdlib.h>
	#include <crtdbg.h>
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility Functions Declarations
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Open up shared memory mapped file for sending data to the Robot
extern int InitIPC( HANDLE &hCommandEvent, LPCTSTR &pCommandBuf, HANDLE &hStatusEvent, LPCTSTR &pStatusBuf );

//////////////////////////////////////////////////////////////////
//extern void g_PostStatus( LPCTSTR lpszStatus, char *FunctionName );
//	void g_PostStatus();	// forward declaration, so we can use it here



/*
#define ROBOT_LOG( __test1, __format, ...) if( __test1 ) {  \
	CString LogString; \
	CString FormatString = _T( __format ); \
	LogString.Format( FormatString, ## __VA_ARGS__ );   \
	g_PostStatus( (LPCTSTR)LogString, __FUNCTION__ );    }
*/

#endif // __KOBUKI_CONTROL_H__