
#ifndef __THREAD_H__
#define __THREAD_H__

#include "RobotType.h"
//#include "device.h"
#include "module.h"
// //#include "RobotServerSock.h"
#include "RobotSharedParams.h"



	#if ( ROBOT_SERVER == 1 )
		DWORD WINAPI ControlThreadProc( LPVOID lpParameter );
		DWORD WINAPI SoundThreadProc( LPVOID lpParameter );
		DWORD WINAPI KinectAppSharedMemoryIPCThreadProc( LPVOID lpParameter );
		DWORD WINAPI CameraAppSharedMemoryIPCThreadProc( LPVOID lpParameter );
		DWORD WINAPI KobukiAppSharedMemoryIPCThreadProc( LPVOID lpParameter );
		DWORD WINAPI SpeakThreadProc( LPVOID lpParameter );

	#endif

#endif