// HardwareInterfaceParams.h
// Command interface parameters between the PC and Microcontroller.
// Appropriate file below is shared by both compilers
// Make sure syntax works with all compilers needed
#ifndef __ROBOT_HW_INTERFACE_PARAMS_H__
#define __ROBOT_HW_INTERFACE_PARAMS_H__

#include "RobotType.h"   // Change this for different robots used
#include "HWInterfaceOther.h"	// Motor, servo, etc. controllers

////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            LOKI
////////////////////////////////////////////////////////////////////////////////////////////////////////
#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI
	#include "HWInterfaceLokiArduino.h"
	//#include "HWInterfaceLokiPic.h"

#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TURTLE
	#include "HWInterfaceTurtle.h"

#elif SENSOR_CONFIG_TYPE == SENSOR_CONFIG_TELEOP
	#include "HWInterfaceTeleop.h"

#else
	#error BAD SENSOR_TYPE!
#endif

#endif // __ROBOT_HW_INTERFACE_PARAMS_H__