// Config.h - Configuration of debug and tracing switches
// 
#pragma once	// Only include this header once
#include "stdafx.h"
#include "RobotType.h"

//****************************************************************************
//                      SYSTEM DEBUG CONFIG SWITCHES
//****************************************************************************

// Bogus handle use for simulation
// Temp HACK - LONG_PTR is defined in DX9 SDK!
//#define SIMULATED_SIO_HANDLE	((HANDLE)(LONG_PTR)-2)
#define SIMULATED_SIO_HANDLE	((HANDLE)-2)
#define SIMULATED_SIO_SLEEP_TIME	5	// ms - time thread will sleep pretending to tx/rx data

// Simulation mode - Simulate SIO ports, etc.
#define ROBOT_SIMULATION_MODE		1

// Force Kinect scan to be far from robot, and loop for debugging object detection
#define DEBUG_KINECT_FLOOR_OBJECT_DETECTION	0


// Enable / Disable break on Asserts
// Warning, if enabled, PC will freeze on ASSERT, but robot will continue driving!
#define ASSERT_ENABLED				1

#define SPEECH_ENGINE_ENABLED		1


// SENSOR DEBUG						1 = enable, 0 = disable
#define DEBUG_SENSOR_FUSION			0
#define DEBUG_IR					0
#define DEBUG_ULTRASONIC			0
#define DEBUG_PIR					0
#define DEBUG_MOTOR_COMMANDS		1

#define REPORT_CLOSE_OBJECTS		0

#define PUBLIC_DEMO					1 // enable if doing public demos; modifies some behaviors

//****************************************************************************

// Enable / Disable break on Asserts macro implementation
#if ASSERT_ENABLED == 1
 #define ROBOT_ASSERT(a) ASSERT(a)
#else
#define ROBOT_ASSERT(a) { if(a){} else { ROBOT_LOG( TRUE,  "*****> ASSERT DISABLED! FILE: "); ROBOT_LOG( TRUE, THIS_FILE); ROBOT_LOG( TRUE,  " <*****\n" ); } }
#endif


// Delete data and set pointer to Null
#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }


// Trace Macros
#define SERIAL_DBG			0 // enable this if Arduino or servos don't work
#define CAMERA_DBG			0
#define MOTOR_DBG			0
#define NAV_DBG				1
#define MODULE_DBG			1
#define TEMP_DBG			1

 