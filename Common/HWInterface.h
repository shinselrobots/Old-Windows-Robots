#pragma once	// Only include this header once

//#include "module.h"
//#include "RobotServerSock.h"
//#include "RobotSharedParams.h"
#include "globals.h"


DWORD WINAPI ArduinoCommReadThreadFunc(LPVOID lpParameter);
DWORD WINAPI ArduinoCommWriteThreadFunc(LPVOID lpParameter); // g_dwArduinoCommWriteThreadId

DWORD WINAPI GPSCommReadThreadFunc(LPVOID lpParameter);

DWORD WINAPI ServoCommWriteThreadFunc(LPVOID lpParameter);

DWORD WINAPI SmartServoCommThreadFunc(LPVOID lpParameter);

DWORD WINAPI LaserScannerCommReadThreadFunc(LPVOID lpParameter);
DWORD WINAPI LaserScannerCommWriteThreadFunc(LPVOID lpParameter);

#if( MOTOR_CONTROL_TYPE != ARDUINO_MOTOR_CONTROL )
	// if controlled indirectly through the Arduino/Arduino, no separate motor thread needed

	DWORD WINAPI MotorCommThreadFunc(LPVOID lpParameter); //g_dwMotorCommThreadId
	#if MOTOR_CONTROL_TYPE == IROBOT_MOTOR_CONTROL
		DWORD WINAPI iRobotCommReadThreadFunc(LPVOID lpParameter);

//	#elif MOTOR_CONTROL_TYPE == KOBUKI_MOTOR_CONTROL
//		DWORD WINAPI KobukiCommReadThreadFunc(LPVOID lpParameter);

	#endif
#endif

void SendCameraCmd(char *CameraBytes, int nByteCount);

#if CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA
	DWORD WINAPI CameraCommWriteThreadFunc(LPVOID lpParameter);
#endif

