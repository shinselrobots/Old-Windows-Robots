#ifndef __ROBOT_SERVER_SOCK_H__
#define __ROBOT_SERVER_SOCK_H__

// Robot Server Remote Socket routines

#include "RobotConfig.h"


// Function Prototypes
DWORD WINAPI ServerSockReceiveThreadProc( LPVOID lpParameter );
DWORD WINAPI ServerSockSendThreadProc( LPVOID lpParameter );


#endif