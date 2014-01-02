// Robot Server Remote Socket routines

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <winsock.h>
#include "RobotServerSock.h"

#define MAX_LOADSTRING 100
//#ifdef __WIN_CE_TARGET
//#define SD_BOTH 2
//#endif

#include "Globals.h"
#include "thread.h"
#include "module.h"
//#include "RobotSharedParams.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define SOCKETS_DISABLED	// Turn off all Socket I/O.  Stand alone Server only


// Global Variables:
//HINSTANCE			hInst;			// The current instance
//HWND				hwndCB;			// The command bar handle
//HWND				hEdit;			// The edit box for displaying information

// Foward declarations of functions included in this code module:
//ATOM				MyRegisterClass	(HINSTANCE hInstance, LPTSTR szWindowClass);
//BOOL				InitInstance	(HINSTANCE, int);
//LRESULT CALLBACK	WndProc			(HWND, UINT, WPARAM, LPARAM);
//LRESULT CALLBACK	About			(HWND, UINT, WPARAM, LPARAM);

/////////////////////////////////////////////////////////////////////////////////



void DisplaySocketStatus( LPCTSTR lpszError )
{
	ROBOT_LOG( TRUE,  "Server Socket: %s\n", lpszError )
	CString strStatus;
	strStatus.Format( "Server Socket: %s", lpszError );
	ROBOT_DISPLAY( TRUE, (LPCTSTR)strStatus )

	//WSASetLastError(0);  // Clear the error
	// MessageBox(GetFocus(), szBuffer, _T("WinSock Error"), MB_OK);

}

// displays last error associated with socket function
void DisplayServerSocketError(LPCTSTR lpszError)
{
	int nErr = WSAGetLastError();
	ROBOT_LOG( TRUE,  "Server Socket Error: %s: %d\n",lpszError, nErr )
	CString strStatus;
	strStatus.Format( "Server Socket Error: %s: %d\n",lpszError, nErr );
	ROBOT_DISPLAY( TRUE, (LPCTSTR)strStatus )

	if( nErr != 0 )
	{
		WSASetLastError(0);  // Clear the error
	} 
}



// creates a listening socket and waits for a connection. Returns a connected
// socket

SOCKET CreateListener()
{
	SOCKADDR_IN sockAddrListen;
	SOCKET sockListen;
	DWORD address;
	char szHostName[1024];
	HOSTENT* lphostent;

	// create a socket
	sockListen = socket(AF_INET, SOCK_STREAM, 0); // 0 = IP protocol
	if(sockListen == INVALID_SOCKET)
	{
		DisplayServerSocketError( "Could not create socket");
		return INVALID_SOCKET;
	}

	if(gethostname (szHostName, 1024) == SOCKET_ERROR)
	{
		DisplayServerSocketError( "Could not get host name" );
		return INVALID_SOCKET;
	}
	lphostent = gethostbyname(szHostName);
	if(lphostent == NULL)
	{
		DisplayServerSocketError( "Could not get host information");
		return INVALID_SOCKET;
	}
	memcpy(&address, lphostent->h_addr_list[0], sizeof(address));

	memset(&sockAddrListen, 0, sizeof(SOCKADDR_IN));
	// specify the port number
	sockAddrListen.sin_port = htons(SERVER_PORT);
	// specify address family as Internet
	sockAddrListen.sin_family = AF_INET;
	// specify address to bind to
	sockAddrListen.sin_addr.s_addr = address;
	
	// bind socket with the SOCKADDR_IN structure
	if(bind(sockListen, (LPSOCKADDR)&sockAddrListen, sizeof(SOCKADDR)) == SOCKET_ERROR)
	{
		DisplayServerSocketError( "Could not bind socket" );
		return INVALID_SOCKET;
	}
	// listen for a connection
	//ROBOT_LOG( TRUE,  "The following Exception comes from Socket listen, I don't know why...\n" )
	if(listen(sockListen, 1) == SOCKET_ERROR)
	{
		DisplayServerSocketError( "Could not listen on socket" );
		return INVALID_SOCKET;
	}
	return sockListen;
}

/******************************************************************************
DWORD ReceiveServerSocketData( SERVER_SOCKET_STRUCT *SocketStruct )
{

	// Receive messages from the Client
	// The message type is defined in the first WORD.  
	// For string messages, the length of the string is indicated in the second two bytes
	// memset(cStatusBuf, '\0', STATUS_BUF_LEN );  // Clear the buffer
	int nErr, nMessageLength;
	char	cStatusBuf[STATUS_BUF_LEN];
	SOCKET_STRING_MESSAGE_T	*pStringMessage = (SOCKET_STRING_MESSAGE_T*)cStatusBuf;	// For reading a String.  Second two bytes are the length
	SOCKET_STATUS_MESSAGE_T	*pStatusMessage = (SOCKET_STATUS_MESSAGE_T*)cStatusBuf;	// use instead of cast to make debug easier
	SOCKET_DATABUF_MESSAGE_T *pBulkDataMessage = (SOCKET_DATABUF_MESSAGE_T*)cStatusBuf;	
	int		nRecv;

	nRecv = recv(SocketStruct->sock, (char*)&cStatusBuf, STATUS_BUF_LEN, 0);
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

		// See what type of message it is
		if( WM_ROBOT_SEND_TEXT_MESSAGES == pStatusMessage->MessageType )
		{
			// It's a string, display the message
			if( pStringMessage->Length > (STATUS_TEXT_BUF_LEN - 1) )
			{
				DisplayClientSocketError("String Length too big for buffer, Triming..." );
				pStringMessage->Length = (STATUS_TEXT_BUF_LEN - 1);
			}
			pStringMessage->Message[pStringMessage->Length] = '\0';	// Null terminate the string
			ROBOT_DISPLAY( TRUE, ( pStringMessage->Message, TRUE )	// from remote client
			ROBOT_LOG( TRUE,  "Text Message Received.\n" )
		}
		else if( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD == pStatusMessage->MessageType )
		{
			// Server echo's the keep alive back.  Check round-trip time.
			DWORD RoundTripTime = GetTickCount() - pStatusMessage->Param1;
			ROBOT_LOG( TRUE,  "RoundTripTime = %d ms\n", RoundTripTime )
			//ROBOT_LOG( TRUE,  "Time since last return ping = %d ms\n",g_RoundTripTime )
			// Everytime we get a ping, reset the timer.
			g_LastPingTime = GetTickCount();
		}
		else if( WM_ROBOT_DISPLAY_TCP_TIME == pStatusMessage->MessageType )
		{
			ROBOT_DISPLAY( TRUE, "NOT IMPLEMENTED: WM_ROBOT_DISPLAY_TCP_TIME" );
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
			}
			if( ROBOT_RESPONSE_PIC_STATUS == pBulkDataMessage->Param1 )
			{
				if( nMessageLength != sizeof(SENSOR_STATUS_T) )
				{
					DisplayClientSocketError("ERROR - Status Block length not correct!" );
				}
				else
				{
					memcpy( &g_SensorStatus, pBulkDataMessage->Data, sizeof(SENSOR_STATUS_T) );

					PostMessage( g_RobotCmdViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotPathViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotMapViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
					PostMessage( g_RobotSetupViewHWND, (pStatusMessage->MessageType), 
						pBulkDataMessage->Param1, pBulkDataMessage->Param2 );
				}
			}
			else
			{
				// Error - unmapped message!
				ROBOT_LOG( TRUE,  "ERROR!  Client Sock: Unmapped BULK ITEMS Type 0x%02lX!\n", pBulkDataMessage->Param1 )
			}
		}
		else
		{
			ROBOT_LOG( TRUE,  "ERROR!  Client Sock: Unmapped Message Type 0x%02lX!\n", pStatusMessage->MessageType )
		}
	}

	return 0;
}
*************/




//*********************************************************************************
DWORD WINAPI ServerSockReceiveThreadProc( LPVOID lpParameter )
{
	SERVER_SOCKET_STRUCT *st = (SERVER_SOCKET_STRUCT*)lpParameter;
	__itt_thread_set_name( "Server Rx Thread" );

	Sleep(5000);

#ifdef SOCKETS_DISABLED
	CString strStatus;
	strStatus.Format( "Sockets disabled" );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)strStatus )
	return 1;
#endif	

	WSADATA wsaData;
	SOCKET sockListen;
	int nReceive;
	SOCKADDR_IN sockAddrClient;
	int nAddrLen = sizeof(SOCKADDR_IN);

	// Allocate receive buffer TODO!
/***
	typedef struct
{
	DWORD	Cmd;
	DWORD	Param1;
	DWORD	Param2;
} SOCK_DATA_T;
**/


	char	MessageBuffer[STATUS_BUF_LEN];
	SOCKET_STRING_MESSAGE_T	 *pStringMessage = (SOCKET_STRING_MESSAGE_T*)MessageBuffer;	// For reading a String.  Second two bytes are the length
	SOCKET_STATUS_MESSAGE_T	 *pStatusMessage = (SOCKET_STATUS_MESSAGE_T*)MessageBuffer;	// use instead of cast to make debug easier
	SOCKET_DATABUF_MESSAGE_T *pBulkDataMessage = (SOCKET_DATABUF_MESSAGE_T*)MessageBuffer;	
	SOCK_DATA_T				 *pSockData = (SOCK_DATA_T*)MessageBuffer;
	SOCKET_DATABUF_MESSAGE_T *pMessageBlock = (SOCKET_DATABUF_MESSAGE_T*)MessageBuffer;

	DisplaySocketStatus( "Socket Thread started..." );

	// Initialize WinSock
	if(WSAStartup(MAKEWORD(1,1), &wsaData) != 0)
	{
		DisplayServerSocketError( "Could not initialize sockets" );
		DisplayServerSocketError(_T("Socket Thread Exiting."));
		return 1;
	}
	sockListen = CreateListener();
	if( INVALID_SOCKET == sockListen )
	{
		DisplayServerSocketError(_T("Socket Thread Exiting."));
		return 1;
	}



	while( g_bRunThread )
	{
		DisplaySocketStatus(_T("Waiting for connection"));
		// block until a socket attempts to connect
		st->sockConnected = accept(sockListen, (LPSOCKADDR)&sockAddrClient, &nAddrLen);
		DisplaySocketStatus(_T("Done waiting for connection"));
		if(st->sockConnected == INVALID_SOCKET)
		{
			g_bConnectedToClient = FALSE;
			DisplayServerSocketError(_T("Could not accept a connection"));
			break;
		}
		else
		{
			DisplaySocketStatus(_T("Client Connected!"));
			// accept strings from client while connection is open
			g_bConnectedToClient = TRUE;

			while( g_bRunThread )
			{
				nReceive = recv(st->sockConnected, MessageBuffer, STATUS_BUF_LEN, 0);
				if(nReceive == 0)
				{
					DisplaySocketStatus( "Connection broken" );
					break;
				}
				else if(nReceive == SOCKET_ERROR)
				{
					DisplayServerSocketError( "Error receiving" );
					break;
				}
				// OK, got data!

				// Don't care if it's a Keep Alive, or any other command
				// Reset the TCP/IP watchdog
				g_nClientKeepAliveCount = 0;

				if( WM_ROBOT_CLIENT_KEEP_ALIVE_CMD == pSockData->Cmd )
				{
					// Send response back to the client,for measuring round-trip time
					PostThreadMessage( g_dwServerSendThreadId, (pSockData->Cmd), pSockData->Param1, pSockData->Param2 );
				}
				else if( WM_ROBOT_CLIENT_DISCONNECT == pSockData->Cmd )
				{
					// Client is about to disconnect
					ROBOT_LOG( TRUE,  "Received DISCONNECT message from Client\n" )
					PostThreadMessage( g_dwControlThreadId, pSockData->Cmd, pSockData->Param1, pSockData->Param2 );
				}
				else if( WM_ROBOT_REMOTE_GUI_CMD == pSockData->Cmd )
				{
					// Video enabling commands are sent to the Server GUI, since the windows must be created
					// by a GUI thread.
					ROBOT_LOG( TRUE,  "Received VIDEO CONFIG message from Client\n" )
//					PostThreadMessage( g_dwControlThreadId, pSockData->Cmd, pSockData->Param1, pSockData->Param2 );
					PostMessage( g_RobotCmdViewHWND, (pSockData->Cmd), pSockData->Param1, pSockData->Param2 );
//					PostMessage( g_RobotPathViewHWND, (pSockData->Cmd), pSockData->Param1, pSockData->Param2 );
//					PostMessage( g_RobotMapViewHWND, (pSockData->Cmd), pSockData->Param1, pSockData->Param2 );
					PostMessage( g_RobotSetupViewHWND, (pSockData->Cmd), pSockData->Param1, pSockData->Param2 );

				}
				else if( WM_ROBOT_TEXT_MESSAGE_TO_SERVER == pSockData->Cmd )
				{
					// It's text (usually text to speech) from the client
					// pMessageBlock->Param1 indicates the message type
					// pMessageBlock->Param2 is an optional parameter (note only one optional parameter allowed)
					// pMessageBlock->Length is the length

					ROBOT_LOG( TRUE,  "Received TEXT DATA message from Client\n" )
					ASSERT(  pMessageBlock->Length < (BULK_DATA_SIZE - 1) );
					pMessageBlock->Data[pMessageBlock->Length] = '\0'; // terminate the string, so CString can copy it
					g_ClientTextToSend = pMessageBlock->Data;	// Copy to the server's global CString
					// Now, convert to a local command, and send it (Param1--> CMD, Param2 --> Param1)
					PostThreadMessage( g_dwControlThreadId, (pSockData->Param1), pSockData->Param2, 0 );	// Control thread will know what to do with it!
				}
				else if( WM_ROBOT_BULK_DATA_TO_SERVER == pSockData->Cmd )
				{
					// It's bulk data (such as lines drawn on map) from the client
					// pMessageBlock->Param1 indicates the message type
					// pMessageBlock->Param2 is an optional parameter (note only one optional parameter allowed)
					// pMessageBlock->Length is the length

					ROBOT_LOG( TRUE,  "Received BULK_DATA message from Client\n" )
					ASSERT(  pMessageBlock->Length < (BULK_DATA_SIZE - 1) );
					memcpy( g_ClientBulkData, pMessageBlock->Data, pMessageBlock->Length );
					g_ClientBulkDataLength = pMessageBlock->Length;
					// Now, convert to a local command, and send it (Param1--> CMD, Param2 --> Param1)
					if( BULK_DATA_TYPE_MAP_STROKE == pSockData->Param1 )
					{
						PostMessage( g_RobotMapViewHWND, (WM_ROBOT_BULK_DATA_TO_SERVER), pSockData->Param1, pSockData->Param2 );	// Send directly to the Map View
					}
					else
					{
						ASSERT(0);	// Unhandled Bulk Message Type!
					}
				}
				else
				{
					// Regular Command.
					// For DEBUG:
					CString DebugMsg;
					DebugMsg.Format( "RxCmd: %04xh  %08xh (%04ld)  %08xh (%04ld)",
						pSockData->Cmd, pSockData->Param1, pSockData->Param1, pSockData->Param2, pSockData->Param2 );
					DisplaySocketStatus( (LPCTSTR)DebugMsg);

					// Send the command to the motor control
//					pSockData->Cmd+= WM_ROBOT_MESSAGE_BASE;	// Post as Windows message.  First command starts at WM_ROBOT_MESSAGE_BASE
					PostThreadMessage( g_dwControlThreadId, pSockData->Cmd, pSockData->Param1, pSockData->Param2 );

					//DisplaySocketStatus( "Received command" );
				}

			}
			// connection broken, clean up.
			g_bConnectedToClient = FALSE;
			shutdown(st->sockConnected, SD_BOTH);
			closesocket(st->sockConnected);
			// TODO?: closesocket(sockListen);

			// Send stop command to the Arduino.
			PostThreadMessage( g_dwControlThreadId, WM_ROBOT_JOYSTICK_DRIVE_CMD, 0, 0 );	// Stop
		}
		g_bConnectedToClient = FALSE;
	}


	DisplayServerSocketError( "Socket Thread Exiting." );
	// Clean up Winsock
	if(WSACleanup() == SOCKET_ERROR)
	{
		DisplayServerSocketError( "Could not cleanup sockets" );
		return 1;
	}
	return 0;
}





DWORD WINAPI ServerSockSendThreadProc( LPVOID lpParameter )
{
	SERVER_SOCKET_STRUCT *st = (SERVER_SOCKET_STRUCT*)lpParameter;
	__itt_thread_set_name( "Server Tx Thread" );

	Sleep(5000);

	int nMessageLength;
	//SOCKET_MESSAGE_T MsgToSend;
	int nSent;
	static BOOL bErrorDisplayed = FALSE;
	MSG		msg;			// Message in the windows queue
//	UINT	MessageType;	// Message ID to send to the client

	char	MessageToSend[STATUS_BUF_LEN];
	SOCKET_STRING_MESSAGE_T	 *pStringMessage =   (SOCKET_STRING_MESSAGE_T*)MessageToSend;	// For sending a String.  Second two bytes are the length
	SOCKET_STATUS_MESSAGE_T	 *pStatusMessage =   (SOCKET_STATUS_MESSAGE_T*)MessageToSend;	// use ptrs instead of cast to make debug easier
	SOCKET_DATABUF_MESSAGE_T *pBulkDataMessage = (SOCKET_DATABUF_MESSAGE_T*)MessageToSend;

	DisplaySocketStatus( "Server Socket Send Thread started." );
	//WCHAR	TempUnicodeString[(STATUS_TEXT_BUF_LEN*2)];
	int nChars=0;

	int SendCounter = 0; // for debug

	// Process message loop
	while( 0 != GetMessage( &msg, NULL, WM_ROBOT_MESSAGE_BASE, (WM_ROBOT_MAX_ROBOT_MESSAGE) ) )
		// OK to filter out Windows messages, since we don't use WM_QUIT
	{
		if( WM_ROBOT_THREAD_EXIT_CMD == (msg.message ) )
		{
			break;	// Quit command received!
		}

		if( !g_bConnectedToClient )
		{
			// Not connected.  Just pull the messages out of the queue
			// and throw them away.
			// DisplaySocketStatus( "ERROR!  Send called while not connected to client!" );
			continue;
		}
		if( (msg.message < WM_ROBOT_MESSAGE_BASE) || ((msg.message ) > WM_ROBOT_MAX_ROBOT_MESSAGE) )
		{
			// Ignore any messages not intended for user (where do these come from?)
			UINT temp = WM_ROBOT_MESSAGE_BASE;	// for debug
			ROBOT_LOG( TRUE,  "SOCKET SEND THREAD: message out of Range, WM_ROBOT_MESSAGE_BASE = 0x%08X, Msg = 0x%08lX, wParam = 0x%08lX, lParam = 0x%08lX\n", 
				temp, msg.message, msg.wParam, msg.lParam )
			continue;
		}

		// For Debug
		// See what type of message it is
/*
		if( (WM_ROBOT_CLIENT_KEEP_ALIVE_CMD != msg.message) && (WM_ROBOT_DISPLAY_BULK_ITEMS != msg.message) )
		{
			ROBOT_LOG( TRUE,  "DBG SEND SOCK: %4d - %04X = %04X, W=%04X, L=%04X\n",  
				SendCounter++, msg.message, msg.message, msg.wParam, msg.lParam)
		}
*/
//		if( 0X00D == MessageType)
//		{
//			ROBOT_LOG( TRUE,  "     KEEP ALIVE RESPONSE")
//		}



		if( 0 == msg.message )
		{
			ROBOT_LOG( TRUE,  "SEND SOCK ERROR!!! 0 MESSAGE TYPE!\n" )
		}
		else if( msg.message > WM_ROBOT_MAX_ROBOT_MESSAGE )
		{
			ROBOT_LOG( TRUE,  "SEND SOCK ERROR!!! BAD MESSAGE TYPE!\n" )
		}
		else if( WM_ROBOT_SEND_TEXT_MESSAGES == msg.message )
		{
			// It's a time to send status message strings to the client, 
			// Send contents of our String Message buffer
			pStringMessage->MessageType = msg.message;
			// ROBOT_LOG( TRUE,  "TIME:%d Server CriticalSection Start\n", GetTickCount())

			if( !g_CriticalSectionsInitialized )
			{
				ROBOT_LOG( TRUE,  " Ignoring WM_ROBOT_SEND_TEXT_MESSAGES - g_CriticalSectionsInitialized = FALSE\n" )
				continue;
			}
			__itt_task_begin(pDomainSocketThread, __itt_null, __itt_null, psh_csDisplayLock);
			EnterCriticalSection(&g_csDisplayLock);
				nMessageLength = g_StatusMessagesToSend.GetLength();
				if( nMessageLength < 1 )
				{
					// Nothing to do
					LeaveCriticalSection(&g_csDisplayLock);
					__itt_task_end(pDomainSocketThread);
					continue;
				}
				if (nMessageLength > STATUS_TEXT_BUF_LEN-1 )
				{
					nMessageLength = STATUS_TEXT_BUF_LEN-1;
				}
				// TODO; Need to break this into smaller messages!!!
				// Currently just truncates any that don't fit!!!
				strncpy_s(pStringMessage->Message, (LPCTSTR)g_StatusMessagesToSend, nMessageLength);
				g_StatusMessagesToSend.Empty();	// Reset the buffer to empty
					
			LeaveCriticalSection(&g_csDisplayLock);
			__itt_task_end(pDomainSocketThread);			

			pStringMessage->Length = nMessageLength;
			pStringMessage->Message[nMessageLength] = '\0'; // for debug convenience, 0 not sent
			// Send the message to the Client
		//	if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%d Server Send Message Start\n", GetTickCount())
			nSent = send(	
				st->sockConnected, 
				MessageToSend, 
				nMessageLength+4, 0);
		//	nSent = send(st->sockConnected, (char*)&MessageToSend, nMessageLength+4, 0);

			//if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%d Server Send Reply Done\n", GetTickCount())
		}
		else if( WM_ROBOT_DISPLAY_BULK_ITEMS == msg.message )
		{
			// It's a request to send bulk data (such as radar sensor data), 
			// Send contents of the correct data buffer
			pBulkDataMessage->MessageType = msg.message;
			pBulkDataMessage->Param1 = msg.wParam;
			pBulkDataMessage->Param2 = msg.lParam;
			pBulkDataMessage->Length = 0;

			if( ROBOT_RESPONSE_RADAR_SAMPLE == msg.wParam )
			{
				// Scanning Sensors only!  Static sensors are included in Arduino Status block
				// lParam is the Sensor number
				if( IR_ARRAY1 == msg.lParam )
					nMessageLength = IR1_SAMPLES;
				else if( US_ARRAY1 == msg.lParam )	
					nMessageLength = US1_SAMPLES;
				//else if( US_ARRAY2 == msg.lParam )
				//	nMessageLength = US2_SAMPLES;
				//else if( IR_ARRAY2 == msg.lParam )
				//	nMessageLength = IR2_SAMPLES;
				else
				{
					ROBOT_LOG( TRUE,  "ERROR - Server Sock: Bad value for ROBOT_RESPONSE_RADAR_SAMPLE lParam\n" )
					nMessageLength = US1_SAMPLES;
				}

				pBulkDataMessage->Length = (WORD)nMessageLength;
				ASSERT(  pBulkDataMessage->Length < (BULK_DATA_SIZE - 1) );
				memcpy( pBulkDataMessage->Data, g_ScaledSensorData[msg.lParam], nMessageLength );

				nSent = send(	
					st->sockConnected, 
					MessageToSend, 
					nMessageLength+8, 0);
			}
			else if( ROBOT_RESPONSE_PIC_STATUS == msg.wParam )
			{
				// Handle Status Info from the Arduino
				nMessageLength = sizeof( SENSOR_STATUS_T );
				pBulkDataMessage->Length = (WORD)nMessageLength;
				ASSERT(  pBulkDataMessage->Length < (BULK_DATA_SIZE - 1) );
				memcpy( pBulkDataMessage->Data, &g_SensorStatus, nMessageLength );

				nSent = send(	
					st->sockConnected, 
					MessageToSend, 
					nMessageLength+12, 0);
			}
			else if( ROBOT_RESPONSE_LASER_SCANNER_DATA == msg.wParam )
			{
				// Handle Laser Scanner data
				nMessageLength = sizeof(POINT2D_T) * sizeof(LASER_RANGEFINDER_MAX_SAMPLES);
				pBulkDataMessage->Length = (WORD)nMessageLength;
				ASSERT(  pBulkDataMessage->Length < (BULK_DATA_SIZE - 1) );
				memcpy( pBulkDataMessage->Data, &g_pLaserScannerData->ScanPoints, nMessageLength );
				// TODO - TEST THIS - is the size right?

				nSent = send(	
					st->sockConnected, 
					MessageToSend, 
					nMessageLength+12, 0);
			}
			else
			{
				// Error - unmapped message!
				// COULD BE GPS DATA?
				ROBOT_LOG( TRUE,  "ERROR!  Server Sock: Unmapped BULK Message Type: wParam = 0x%02lX, lParam = 0x%02lX !\n", msg.wParam, msg.wParam )
			}
		}
		else
		{
			// Non-Buffer message (including connection ACK).
			pStatusMessage->MessageType = msg.message;
			pStatusMessage->Param1 = msg.wParam;
			pStatusMessage->Param2 = msg.lParam;
			// Send the message to the Client
			//if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%d Server Send Message Start\n", GetTickCount())
			nSent = send(
				st->sockConnected, 
				MessageToSend, 
				sizeof(SOCKET_STATUS_MESSAGE_T),
				0);
			//if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%d Server Send Reply Done\n", GetTickCount())
		}
 

		if(nSent == SOCKET_ERROR)
		{
			if( !bErrorDisplayed )
			{
				DisplayServerSocketError( "Cannot send Status to Client" );
				bErrorDisplayed = TRUE;
				//break;
			}
		}
		else
		{
			//DisplaySocketStatus( "Status sent" );
			bErrorDisplayed = FALSE;	// reset for next error
		}

/*		dwElapsedTime = GetTickCount() - dwOldTime;
		if( (dwElapsedTime) > dwMaxTime )
		{
			dwMaxTime = dwElapsedTime;
			ROBOT_LOG( TRUE,  ">>> %ld Server Sock Send New Max Time: %ld\n" , GetTickCount(), dwMaxTime )
			if( dwMaxTime > 3000 )
			{
				ROBOT_LOG( TRUE,  ">>>Server Send %d Resetting Max Time after %d ms\n" , nErrorCount++, (GetTickCount() - dwLastReset) )
				dwLastReset = GetTickCount();
				dwMaxTime = 0;
			}
		}
		dwOldTime = GetTickCount();

		if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%ld Server Send Sleep Start\n", GetTickCount())
		Sleep( 200 );
		if( SHOW_TIME_DETAIL ) ROBOT_LOG( TRUE,  "TIME:%ld Server Send Sleep Done\n", GetTickCount())
*/
	}

	// We received a WM_ROBOT_THREAD_EXIT_CMD or WM_QUIT message, exit
	ROBOT_LOG( TRUE,  "ServerSockThread exiting.\n" )
	return 0;

}

#endif  // End of ifdef ROBOT_SERVER

