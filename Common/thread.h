
#ifndef __THREAD_H__
#define __THREAD_H__

#include "RobotConfig.h"
//#include "device.h"
#include "module.h"
// //#include "RobotServerSock.h"
#include "RobotSharedParams.h"



	#if ( ROBOT_SERVER == 1 )
		DWORD WINAPI ControlThreadProc( LPVOID lpParameter );
		DWORD WINAPI SoundThreadProc( LPVOID lpParameter );
		DWORD WINAPI KinectSpeechThreadProc( LPVOID lpParameter );
		DWORD WINAPI CameraAppSharedMemoryIPCThreadProc( LPVOID lpParameter );
#if( MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL )
		DWORD WINAPI KobukiAppSharedMemoryIPCThreadProc( LPVOID lpParameter );
#endif
		DWORD WINAPI SpeakThreadProc( LPVOID lpParameter );

	#endif

#endif