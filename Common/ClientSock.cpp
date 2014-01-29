// Client Remote Socket routines
#include "stdafx.h"
#include <winsock.h>
#include "resource.h"
#include "Globals.h"
#include "RobotClientSock.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


void DisplayClientSocketError( LPCTSTR lpszStatus )
{

	CString strStatus;
	int nErr = WSAGetLastError();
	strStatus.Format("Client Socket: %s: error %d",lpszStatus, nErr );
	if( nErr != 0 )
	{
		WSASetLastError(0);  // Clear the error
	}

	ROBOT_DISPLAY( TRUE, strStatus )

}

BOOL ConnectSocket( CLIENT_SOCKET_STRUCT *SocketStruct )
{
	DWORD dwDestAddr;
	SOCKADDR_IN sockAddrDest;

	// create socket
	SocketStruct->sock = socket(AF_INET, SOCK_STREAM, 0);
	if(SocketStruct->sock == SOCKET_ERROR)
	{
		DisplayClientSocketError("Could not create socket");
		return 0;
	}

	// Turn on quick sockets
	int TcpNoDelayOn = 1; // On
	int ReceiveTimeout = 3000; // 3 seconds?
	int err = setsockopt( 
			SocketStruct->sock, 
			SOL_SOCKET,		// TODO: try IPPROTO_TCP if SOL_SOCKET does not work
//			(SO_RCVTIMEO),	// TODO: try SO_RCVTIMEO to adjust timeouts 
			(TCP_NODELAY),	// TODO: try SO_SNDTIMEO to adjust timeouts 
			(char *)&TcpNoDelayOn,	// 0 = Option off, 1 = option on.  TODO: for SO_SNDTIMEO use (char *)&timeout, 
			sizeof(TcpNoDelayOn) );

	if (err != NO_ERROR)
	{
		DisplayClientSocketError( "Setsockopt TCP_NODELAY Failed" );
	}


/*	err = setsockopt( 
			SocketStruct->sock, 
			SOL_SOCKET,		// TODO: try IPPROTO_TCP if SOL_SOCKET does not work
			(SO_RCVTIMEO),	// TODO: try SO_RCVTIMEO to adjust timeouts 
//			(TCP_NODELAY),	// TODO: try SO_SNDTIMEO to adjust timeouts 
			(char *)&ReceiveTimeout,	// 0 = Option off, 1 = option on.  TODO: for SO_SNDTIMEO use (char *)&timeout, 
			sizeof(ReceiveTimeout) );

	if (err != NO_ERROR)
	{
		DisplayClientSocketError( "Setsockopt RCVTIMEO Failed" );
	}
*/

	// convert address to in_addr (binary) form
	dwDestAddr = inet_addr(SocketStruct->szIPAddress);
	// Initialize SOCKADDR_IN with IP address, port number and address family
	memcpy(&sockAddrDest.sin_addr, &dwDestAddr, sizeof(DWORD));
	sockAddrDest.sin_port = htons(SERVER_PORT);
	sockAddrDest.sin_family = AF_INET;  // Internet address family
	// attempt to connect to server
	ROBOT_LOG( TRUE, "Attempting to connect to %s\n", SocketStruct->szIPAddress )
	if(connect(SocketStruct->sock, (LPSOCKADDR)&sockAddrDest, sizeof(sockAddrDest)) == SOCKET_ERROR)
	{
		DisplayClientSocketError("Could not connect to server socket");
		closesocket(SocketStruct->sock);
		return 0;
	}
	ROBOT_LOG( TRUE, "Socket Connected to %s\n", SocketStruct->szIPAddress )
	g_bConnectedToServer = TRUE;
	g_LastPingTime = GetTickCount();	// Initialize for round trip timing
	ROBOT_DISPLAY( TRUE, "Client Socket Connected")
	AfxMessageBox( _T("Connected To Robot!") );
	return 1;
}



/***** for reference:
SOCKET_DATABUF_HEADER_T
{
	WORD	MessageType;	// WM_ROBOT_RESPONSE_...
	DWORD	Param1;
	DWORD	Param2;
	WORD	Length;
	CHAR	Data[STATUS_DATA_BUF_LEN];
} SOCKET_DATABUF_MESSAGE_T;

#define DRAW_MODE_USER_STROKE		0x00
#define DRAW_MODE_DOOR				0x01
#define DRAW_MODE_LOW_OBSTACLE		0x02
#define DRAW_MODE_OBSTACLE			0x03
#define DRAW_MODE_WALL				0x04
#define DRAW_MODE_CLIFF				0x05
#define DRAW_MODE_WAYPOINT_LOCATION	0x06
#define DRAW_MODE_NAVIGATION_GOAL	0x07
#define DRAW_MODE_SET_LOCATION		0x08	// Set current / start locatio of Robot
#define NUMBER_OF_DRAW_MODES		0x09	// This should be number of types listed above

*****/

DWORD SendSocketData( CLIENT_SOCKET_STRUCT *SocketStruct, 
					 DWORD CMD, DWORD Param1, DWORD Param2 )
{
	if ( !g_bConnectedToServer )
	{
		ROBOT_DISPLAY( TRUE, "Error Client: Socket Not Connected" )
		return 1;
	}

	int  nSent;
//	int  nBytesToSend = 0;
	char MessageBuffer[STATUS_BUF_LEN];
	SOCKET_DATABUF_MESSAGE_T *pMessageBlock = (SOCKET_DATABUF_MESSAGE_T*)MessageBuffer;

	pMessageBlock->MessageType = CMD;
	pMessageBlock->Param1 = Param1;
	pMessageBlock->Param2 = Param2;
	pMessageBlock->Length = 0;

	// See what type of message it is
	if( WM_ROBOT_TEXT_MESSAGE_TO_SERVER == CMD )
	{
		// Sending a text message to the robot from the client. Usually used for text to speech
		// Note!  no syncronization is used, we assume the buffer does not change before being sent!

		ROBOT_ASSERT(0); // THIS IS BROKEN BY NEW SPEECH BUFFER MECHANISM!! TODO-MUST FIX THIS!

		pMessageBlock->Length = (WORD)g_ClientTextToSend.GetLength();
		if( pMessageBlock->Length < 1 )
		{
			return 0; // Nothing to do
		}
		if (pMessageBlock->Length > STATUS_DATA_BUF_LEN-1 )
		{
			pMessageBlock->Length = STATUS_DATA_BUF_LEN-1;
		}
		strncpy_s(pMessageBlock->Data, (LPCTSTR)g_ClientTextToSend, pMessageBlock->Length);
		g_ClientTextToSend.Empty();	// Reset the buffer to empty
		pMessageBlock->Data[pMessageBlock->Length] = '\0'; // for debug convenience, 0 not sent

		// Message ready to send!

	}
	else if( WM_ROBOT_BULK_DATA_TO_SERVER == CMD )
	{
		// It's a request to send bulk data (such as lines drawn on map)
		// pMessageBlock->Param1 indicates the message type
		// pMessageBlock->Param2 is an parameter such as stroke type
		// pMessageBlock->Length is the length

		if( (BULK_DATA_TYPE_MAP_STROKE	== pMessageBlock->Param1) )
		{
			// User drew line segments in a map
			pMessageBlock->Length = (WORD)g_ClientBulkDataLength;
			ASSERT(  pMessageBlock->Length < (BULK_DATA_SIZE - 1) );
			memcpy( pMessageBlock->Data, g_ClientBulkData, pMessageBlock->Length );
			g_ClientBulkDataLength = 0;	// Reset the buffer

			// Message ready to send!
		}
		else
		{
			ASSERT(0); // Unhandled message!
		}
	}
	else if( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD == CMD )
	{
		// Override Param1 with our Tick Count to the server.  
		// It will send it back in the response, so we can time roundtrip time.
		pMessageBlock->Param1 = GetTickCount();

		// Message ready to send!
	}

	// else, it's some other message, and already got setup at the top of this function!


	// Send the message to the Robot
	nSent = send(	
		SocketStruct->sock, 
		MessageBuffer, 
		pMessageBlock->Length + sizeof(SOCKET_DATABUF_HEADER_T), 0);

	if( nSent == SOCKET_ERROR )
	{
		DisplayClientSocketError( "Connection Broken" );
		g_bConnectedToServer = FALSE;
		return 1;
	} 

	if( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD != CMD )
	{
		ROBOT_LOG( TRUE, "Message %04xh Sent OK!\n", CMD )
	}

	return 0;
}


DWORD CloseSocket( CLIENT_SOCKET_STRUCT *SocketStruct )
{
	g_bConnectedToServer = FALSE;
	// close socket
	closesocket(SocketStruct->sock);
	// Clean up Winsock
	/*
	if(WSACleanup() == SOCKET_ERROR)
	{
		DisplayClientSocketError ("Could not cleanup sockets" );
		
	}
	*/
	SocketStruct->sock = INVALID_SOCKET;
	SocketStruct->szIPAddress[0] = 0;

	return 0;
}

DWORD ReceiveClientSocketData( CLIENT_SOCKET_STRUCT *SocketStruct )
{

	// read back the status messages from the Robot Server
	// The message type is defined in the first WORD.  
	// For string messages, the length of the string is indicated in the second two bytes
	// memset(cStatusBuf, '\0', STATUS_BUF_LEN );  // Clear the buffer
	int nErr, nMessageLength;
	//SOCKET_MESSAGE_T cStatusBuf;
	char	cStatusBuf[STATUS_BUF_LEN];
	SOCKET_STRING_MESSAGE_T	*pStringMessage = (SOCKET_STRING_MESSAGE_T*)cStatusBuf;	// For reading a String.  Second two bytes are the length
	SOCKET_STATUS_MESSAGE_T	*pStatusMessage = (SOCKET_STATUS_MESSAGE_T*)cStatusBuf;	// use instead of cast to make debug easier
	SOCKET_DATABUF_MESSAGE_T *pBulkDataMessage = (SOCKET_DATABUF_MESSAGE_T*)cStatusBuf;	
	int		nRecv;
	static UINT RecvCounter = 0;	// for debug

	//if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,"TIME:%ld Client Receive Start\n", GetTickCount())
	nRecv = recv(SocketStruct->sock, (char*)&cStatusBuf, STATUS_BUF_LEN, 0);
	//if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,"TIME:%ld Client Receive Done\n", GetTickCount())

	if( SOCKET_ERROR == nRecv )
	{
		nErr = WSAGetLastError();
		if( WSAEMSGSIZE == nErr )
		{
			DisplayClientSocketError(   "Socket Error:  Sock reports incoming message too big for buffer!" );
		}
		else
		{
			DisplayClientSocketError( "Socket Error during receive!" );
		}
	}
	else
	{
		// Message Received.  Reset Connection Monitor timer
		g_ConnectionMonitorTimer = 0;

		if( (WM_ROBOT_CLIENT_KEEP_ALIVE_CMD != pStatusMessage->MessageType) && (WM_ROBOT_DISPLAY_BULK_ITEMS != pStatusMessage->MessageType) )
		{
			ROBOT_LOG( TRUE,"DBG RCV SOCK: %04d - MSG = %04X, W=%04X, L=%04X\n",
				RecvCounter++, pStatusMessage->MessageType, pStatusMessage->Param1, pStatusMessage->Param2)
		}
		// See what type of message it is

		
		if( pStatusMessage->MessageType > WM_ROBOT_MAX_ROBOT_MESSAGE )
		{
			ROBOT_LOG( TRUE,"ERROR! BAD MESSAGE Received!\n" )
		}
		else if( WM_ROBOT_SEND_TEXT_MESSAGES == pStatusMessage->MessageType )
		{
			// It's a string, display the message
			if( pStringMessage->Length > (STATUS_TEXT_BUF_LEN - 1) )
			{
				DisplayClientSocketError("String Length too big for buffer, Triming..." );
				pStringMessage->Length = (STATUS_TEXT_BUF_LEN - 1);
			}
			pStringMessage->Message[pStringMessage->Length] = '\0';	// Null terminate the string
			ROBOT_DISPLAY( TRUE, pStringMessage->Message, 0, TRUE )	// from remote client
			ROBOT_LOG( TRUE,"Text Message Received.\n" )
		}
		else if( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD == pStatusMessage->MessageType )
		{
			// Server echo's the keep alive back.  Check round-trip time.
			DWORD RoundTripTime = GetTickCount() - pStatusMessage->Param1;
//			ROBOT_LOG( TRUE,"RoundTripTime = %d ms\n", RoundTripTime )
			//ROBOT_LOG( TRUE,"Time since last return ping = %d ms\n",g_RoundTripTime )
			// Everytime we get a ping, reset the timer.
			g_LastPingTime = GetTickCount();
		}
		else if( WM_ROBOT_DISPLAY_TCP_TIME == pStatusMessage->MessageType )
		{
			ROBOT_DISPLAY( TRUE, "NOT IMPLEMENTED: WM_ROBOT_DISPLAY_TCP_TIME" )
		}
		else if( WM_ROBOT_DISPLAY_SINGLE_ITEM == pStatusMessage->MessageType )
		{
			PostMessage( g_RobotCmdViewHWND, (pStatusMessage->MessageType), pStatusMessage->Param1, pStatusMessage->Param2 );
		}
		else if( WM_ROBOT_REMOTE_GUI_CMD == pStatusMessage->MessageType )
		{
			PostMessage( g_RobotCmdViewHWND, (pStatusMessage->MessageType), pStatusMessage->Param1, pStatusMessage->Param2 );
			PostMessage( g_RobotSetupViewHWND, (pStatusMessage->MessageType), pStatusMessage->Param1, pStatusMessage->Param2 );
		}
		else if( WM_ROBOT_DISPLAY_BULK_ITEMS == pStatusMessage->MessageType )
		{
			// It's bulk data (such as radar sensor data or status block), 
			if( pBulkDataMessage->Length > (BULK_DATA_SIZE - 1) )
			{
				DisplayClientSocketError("ERROR - Bulk Data Length too big for buffer, Triming..." );
				nMessageLength = (BULK_DATA_SIZE - 1);
			}
			else
			{
				nMessageLength = pBulkDataMessage->Length;
			}

			if( ROBOT_RESPONSE_RADAR_SAMPLE == pBulkDataMessage->Param1 )
			{

				// Param2 is the Sensor number
				memcpy( g_ScaledSensorData[pBulkDataMessage->Param2], pBulkDataMessage->Data, nMessageLength );

				PostMessage( g_RobotCmdViewHWND, 
					(pStatusMessage->MessageType), 
					pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
				// WARNING WARNING!  REMOVED WM_ROBOT_MESSAGE_BASE FROM THIS CODE!  NEED TO TEST!!!
			}
			if( ROBOT_RESPONSE_LASER_SCANNER_DATA == pBulkDataMessage->Param1 )
			{
				memcpy( &g_pLaserScannerData->ScanPoints, pBulkDataMessage->Data, nMessageLength );

				PostMessage( g_RobotCmdViewHWND, 
					(pStatusMessage->MessageType), 
					pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
				// WARNING WARNING!  REMOVED WM_ROBOT_MESSAGE_BASE FROM THIS CODE!  NEED TO TEST!!!
			}
			if( ROBOT_RESPONSE_PIC_STATUS == pBulkDataMessage->Param1 )
			{
				ROBOT_LOG( TRUE, "ROBOT_RESPONSE_PIC_STATUS DISABLED - To fix \n" )
				/**
				if( nMessageLength != sizeof(SENSOR_STATUS_T) )
				{
					DisplayClientSocketError("ERROR - Status Block length not correct!" );
				}
				else
				{
					memcpy( &g_pFullSensorStatus, pBulkDataMessage->Data, sizeof(SENSOR_STATUS_T) );

					PostMessage( g_RobotCmdViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotPathViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotMapViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotSetupViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
				}
				**/
			}
			else
			{
				// Error - unmapped message!
				ROBOT_LOG( TRUE, "ERROR!  Client Sock: Unmapped BULK ITEMS Type 0x%02lX!\n", pBulkDataMessage->Param1 )
			}
		}
		else
		{
			ROBOT_LOG( TRUE, "ERROR!  Client Sock: Unmapped Message Type 0x%02lX!\n", pStatusMessage->MessageType )
		}
	}

	return 0;
}


DWORD WINAPI ClientSockSendThreadProc( LPVOID lpParameter )
{
	CLIENT_SOCKET_STRUCT *ClientSockStruct = (CLIENT_SOCKET_STRUCT*)lpParameter;
	__itt_thread_set_name( "ClientSock Send Thread" );

	Sleep(500);

	// Process message loop
	MSG msg;
//	while( 0 != GetMessage( &msg, NULL, 0, 0 ) )
// TODO!!! Is this OK for Client Send messages????!!!!
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MAX_ROBOT_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		if( WM_ROBOT_THREAD_EXIT_CMD == (msg.message ) )
		{
			break; // Quit command received!
		}

		if ( !g_bConnectedToServer )
		{
			// Happens all the time until we are connected.
			// Sleep( 200 );
			continue;
		}

		// Send the command via Sockets to the Robot
		if( (msg.message ) != WM_ROBOT_CLIENT_KEEP_ALIVE_CMD )
		{
			ROBOT_LOG( TRUE, "Sending Message %04xh to Server\n", (msg.message )  )
		}

		SendSocketData( ClientSockStruct, 
			(msg.message ),	// Send just the basic Command number
			msg.wParam, msg.lParam );

	}

	// We received a WM_ROBOT_THREAD_EXIT_CMD or WM_QUIT message, exit
	ROBOT_LOG( TRUE,"ClientSockThread exiting.\n")

	return 0;
}

DWORD WINAPI ClientSockReceiveThreadProc( LPVOID lpParameter )
{
	CLIENT_SOCKET_STRUCT *ClientSockStruct = (CLIENT_SOCKET_STRUCT*)lpParameter;
	__itt_thread_set_name( "ClientSock Receive Thread" );

	Sleep(500);

	while( g_bRunThread )
	{
		if ( !g_bConnectedToServer )
		{
			// Happens all the time until we are connected.
			Sleep( 200 );
			continue;
		}

		ReceiveClientSocketData( ClientSockStruct );
	}

	// exit
	ROBOT_LOG( TRUE,"ClientSockThread exiting.\n")

	return 0;
}






