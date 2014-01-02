#ifndef __ROBOT_CLIENT_SOCK_H__
#define __ROBOT_CLIENT_SOCK_H__

// Client Remote Socket routines

#include "RobotType.h"
#include "Globals.h"

// Function Prototypes
DWORD WINAPI ClientSockSendThreadProc( LPVOID lpParameter );
DWORD WINAPI ClientSockReceiveThreadProc( LPVOID lpParameter );

BOOL ConnectSocket( CLIENT_SOCKET_STRUCT *SocketStruct );
DWORD CloseSocket( CLIENT_SOCKET_STRUCT *SocketStruct );
DWORD SendSocketData( CLIENT_SOCKET_STRUCT *SocketStruct, 
					 DWORD CMD, DWORD Param1, DWORD Param2 );

#endif	// __ROBOT_CLIENT_SOCK_H__