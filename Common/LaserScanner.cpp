// LaserScanner.cpp
// Interface for Hokuyo URG-04LX-UG01 Laser Scanner, using SCIP 2.0 Protocol
// See "Updated SCIP20.pdf" for details and www.acroname.com
// This file contains two main classes: 
//    CLaserScannerCommand, which sends commands to the Laser Scanner, called from LaserScannerCommWriteThreadFunc
//    CLaserScannerParser, which reads data from the Laser Scanner, called from LaserScannerCommReadThreadFunc

// in HWInterface.cpp.
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include "Globals.h"
//#include "LaserControl.h"
#include "LaserScanner.h"
#include "HardwareConfig.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////
//                DEBUG SWITCHES
//#define DEBUG_LASER_COMMAND_DUMP			0	   // Dump all commands
#define DEBUG_LASER_SHOW_RAW_RX_DATA		0
#define DEBUG_LASER_SHOW_FINAL_DATA			0
//#define DEBUG_LASER_SHOW_STATUS_MESSAGES	0
#define DEBUG_LASER_TIMING					0
#define DEBUG_LASER_SHOW_SCANS				0
#define DEBUG_LASER_SHOW_2D_OBJECTS_FOUND	0



// Constants
#define LASER_FINDER_OBJECT_HEIGTH_MIN		2.0	// TenthInches
#define LASER_FINDER_OBJECT_WIDTH_MIN		2.0	// TenthInches
#define LASER_FINDER_OBJECT_HEIGHT_MAX	   80.0	// TenthInches
#define LASER_FINDER_OBJECT_WIDTH_MAX	  120.0	// TenthInches



// Line Feed Positions in LfPosition Array for received Data
enum {
	//LF_POS_OPTION_STRING,
	LF_POS_STATUS = 0,
	LF_POS_TIME_STAMP,
	LF_POS_FIRST_DATA_BLOCK
};



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Laser Scanner Control Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CLaserScannerCommand::CLaserScannerCommand()
{
//	m_LaserCmd.Header = LASER_SIO_HEADER_BYTE;	// 0xAA
//	m_LaserCmd.ID = 0;			// Address of Servo to access
//	m_LaserCmd.Command = 0;		
//	memset(m_LaserCmd.Data,0,LASER_MAX_DATA_BYTES);
//	memset(m_LaserReplyBuf,0,LASER_READ_BUF_SIZE);
//	m_HomePositionInitializedRight = FALSE;
//	LaserCmdStartTime = 0;
	memset(m_CmdBuf,0,LASER_SCANNER_CMD_BUF_SIZE);

}

CLaserScannerCommand::~CLaserScannerCommand()
{
	//	SAFE_DELETE(m_pArmControlRight);
}

///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////
void CLaserScannerCommand::Init()
{
	// Reset the Scanner
	sprintf_s(m_CmdBuf,LASER_SCANNER_CMD_BUF_SIZE, "RS\n" );
	SendCmd();
}

void CLaserScannerCommand::EnableLaser( BOOL Enable )
{
	// Turn laser on or off
	if( Enable )
	{
		ROBOT_LOG( TRUE, "LASER: Enabling Laser Power\n")
		sprintf_s(m_CmdBuf,LASER_SCANNER_CMD_BUF_SIZE, "BM\n" );
	}
	else
	{
		ROBOT_LOG( TRUE, "LASER: Disabling Laser Power\n")
		sprintf_s(m_CmdBuf,LASER_SCANNER_CMD_BUF_SIZE, "QT\n" );
	}
	SendCmd();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RequestScans
// Tell the laser to perform nScans.
// Note: Laser switches on automatically before the measurement and switched off after 
// completing the number of scans defined in the command

// Format the command. MS = Request response in 2 char code format (max value 4095)
// M(4dH) S(53H), Starting Step(4bytes), End Step(4 bytes), Cluster Count(2bytes), 
// Scan Interval(1 byte), Number of Scans(2 bytes), String Characters(max 16-letters), LF(1 byte)
// NOTE: Center = step 384, full range = 239.77 degrees, Angle/step = 0.3515625 (360/1024)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CLaserScannerCommand::RequestScans( int  nClusters, int   nIntervals, int  nScans )
{
	

	int nStartingStep = 138; //44; //+43;	// 180 degree scan	 add 85
	int nEndingStep = nStartingStep + 512; // 698; //725; // +32;	// sub 85 // Must me multiple of 32!
	//char *strDebug = ";loki;";	// TODO - replace this was a command counter

	if(nScans > 99)
	{
		ROBOT_LOG( TRUE, "LASER SCANNER RequestScans: ERROR: Number of Scans requested (%d) exceeds MAX of 99!\n", nScans)
		nScans = 99;
	}

	sprintf_s(m_CmdBuf, LASER_SCANNER_CMD_BUF_SIZE, "MS%04d%04d%02d%d%02d\n", 
		nStartingStep, nEndingStep, nClusters, nIntervals, nScans );

	//ROBOT_LOG( TRUE, "\nLASER Sending Command: MS %04d %04d %02d %d %02d\n", nStartingStep, nEndingStep, nClusters, nIntervals, nScans )
	SendCmd(); 

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SendCmd
// Send command to Laser Range Finder.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CLaserScannerCommand::SendCmd()
{
	CString MsgString;
	DWORD dwBytesWritten=0;
	int  nBytesToWrite = strlen( m_CmdBuf );

	// Debug:
//	MsgString.Format( "Laser SendCmd:  Sending cmd %s to Laser Scanner\n", m_CmdBuf );
//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

	if( INVALID_HANDLE_VALUE == g_hLaserScannerCommPort )	// skip COM Read/Write if no serial hooked up
	{
		return;
	}

	// Keep the last command to compare with response
	strncpy_s( g_LaserScannerState.LastCmd, LASER_SCANNER_CMD_BUF_SIZE, m_CmdBuf, nBytesToWrite);

#if DEBUG_LASER_TIMING == 1
	unsigned long ProcessingTime = GetTickCount() - LaserCmdStartTime;
	ROBOT_LOG( TRUE,  "   Laser WriteFile Start: %4d ms\n", ProcessingTime )
#endif

	// SEND COMMAND /////////////////////////////////////////////////////////////
	//dwStartTime = GetTickCount();
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		RobotSleep(SIMULATED_SIO_SLEEP_TIME, pDomainLaserThread);
		return;
	}

	if(!WriteFile(g_hLaserScannerCommPort,	// where to write to (the open comm port)
			&m_CmdBuf,					// what to write
			nBytesToWrite,				// number of BYTEs to be written to port
			&dwBytesWritten,			// number of BYTEs that were actually written
			NULL))						// overlapped I/O not supported			
	{
		ROBOT_DISPLAY( TRUE, "SERIAL ERROR Sending Command to LaserScannerCommPort!\n" )
		DWORD ErrorCode = GetLastError();	// See Help: " System Errors "
		return;
	}

	#ifdef  DEBUG_LASER_READWRITE_TIME
		ROBOT_LOG( TRUE, "Laser Write Time: %d ms\n", (GetTickCount() - dwStartTime) )
	#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HandleCommand
// Handle Command from the command queue
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CLaserScannerCommand::HandleCommand( int  Request, int  Param1, int  Param2 )
{
	IGNORE_UNUSED_PARAM (Param1);
	//TAL_Event("Sending Cmd");
	//-TAL_SCOPED_TASK_NAMED("Handling Laser Command");

	switch( Request )
	{
		// Status Requests
		case HW_GET_STATUS:
		{
//			GetServoStatus( MotorNumber );
			break;
		}

		case HW_INITIALIZE:
		{
			Init();
			break;
		}
		case HW_SET_LASER_SCANNER_POWER:
		{
			EnableLaser( (BOOL)Param2 );
			break;
		}
		case HW_LASER_REQUEST_SCAN:
		{
			int  nClusters = 0;
			int   nIntervals = 0;
			int  nScans = Param2;

			RequestScans( nClusters, nIntervals, nScans );
			break;
		}

		default:
		{
			CString StrText;
			StrText.Format( _T("ERROR! CLaserScannerParser: Unknown Cmd:%02X \n"), Request);
			ROBOT_DISPLAY( TRUE, StrText )
		}

	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Laser Scanner Read Parser Class
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CLaserScannerParser::CLaserScannerParser()
{
		m_pParseBuf = new char[LASER_SCANNER_READ_BUF_SIZE+1];
		assert( 0 != m_pParseBuf );
		m_pDataArray = new int [LASER_RANGEFINDER_MAX_SAMPLES+1];
		assert( 0 != m_pDataArray );

		m_Initialized = FALSE;
		Init();

}

CLaserScannerParser::~CLaserScannerParser()
{
	SAFE_DELETE(m_pParseBuf);
	SAFE_DELETE(m_pDataArray);
}

void CLaserScannerParser::Init()
{
//	memset( m_InputBuf,0,LASER_SCANNER_READ_BUF_SIZE);	// input buffer from Serial I/O

	assert( 0 != m_pParseBuf );
	memset( m_pParseBuf,0,LASER_SCANNER_READ_BUF_SIZE);	// local copy for parsing
	
	assert( 0 != m_pDataArray );
	memset( m_pDataArray,0,LASER_RANGEFINDER_MAX_SAMPLES);

	memset( &m_LaserSummary, 0x00, sizeof( SCANNER_SUMMARY_T ) );

	m_LaserSummary.nLeftRearZone =			NO_OBJECT_IN_RANGE;	
	m_LaserSummary.bLeftCliff =				FALSE;
	m_LaserSummary.nLeftSideZone =			NO_OBJECT_IN_RANGE;
	m_LaserSummary.nLeftFrontSideZone =		NO_OBJECT_IN_RANGE;
	m_LaserSummary.nLeftArmZone =			NO_OBJECT_IN_RANGE;	// Object in front of Arm
	m_LaserSummary.nLeftFrontZone =			NO_OBJECT_IN_RANGE;
	m_LaserSummary.nRightFrontZone =		NO_OBJECT_IN_RANGE;
	m_LaserSummary.nRightArmZone =			NO_OBJECT_IN_RANGE; // Object in front of Arm
	m_LaserSummary.nRightFrontSideZone =	NO_OBJECT_IN_RANGE;
	m_LaserSummary.nRightSideZone =			NO_OBJECT_IN_RANGE;
	m_LaserSummary.bRightCliff =			FALSE;
	m_LaserSummary.nRightRearZone =			NO_OBJECT_IN_RANGE;	

	m_ParseBufCount = 0;
	g_LaserSubSystemStatus = SUBSYSTEM_WAITING;
	m_Initialized = TRUE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ParseBuffer
// Parse data received from the laser scanner
// ASSUME all data comes in one buffer!
// NOT: Buffers received are partial buffers, requiring multiple reads to get complete data
// NOT: Keep looking until we get 2 LF.  This terminates a response.
// Function returns true if done reading full line of valid range data
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__itt_string_handle* pshProcessScanData = __itt_string_handle_create("Process Data");
BOOL CLaserScannerParser::ParseBuffer( char *LaserSIOBuf, int  dwSIOBytesReceived )
{
	if( !m_Initialized ) return 0;
	//TAL_SCOPED_TASK_NAMED("Laser ParseBuffer called");

	CString MsgString;
	//int nScan;
	if( dwSIOBytesReceived >= LASER_SCANNER_READ_BUF_SIZE  )
	{
		ROBOT_ASSERT(0);
	}

	// Add latest string to the parser input buffer
	LaserSIOBuf[dwSIOBytesReceived] = '\0';	// should already be zero, but make sure

	///strcat_s( m_InputBuf, LASER_SCANNER_READ_BUF_SIZE, LaserSIOBuf );
	///m_InputBufCount += dwSIOBytesReceived;
	BOOL bMoreDataToRead = TRUE;
	char *pLaserSIOBuf = LaserSIOBuf;

	while( bMoreDataToRead )
	{
		// Look for double Line Feed terminator in the input buffer
		BOOL bMessageFound = FALSE;
		for( int  i=0; i< dwSIOBytesReceived; i++ )
		{
			if( ('\n' == pLaserSIOBuf[i]) && ('\n' == pLaserSIOBuf[i+1]) )
			{
				// Found LF+LF
				bMessageFound = TRUE;
				// copy off to parse buffer
				m_ParseBufCount = i;
				memcpy_s(m_pParseBuf, LASER_SCANNER_READ_BUF_SIZE, pLaserSIOBuf, m_ParseBufCount);
				m_pParseBuf[m_ParseBufCount] = 0;	// terminate
				pLaserSIOBuf += i+2;
				// WARNING! pLaserSIOBuf INVALID for further processing at this point! use m_pParseBuf until MoreDataToRead loops!
			}
		}
		if( !bMessageFound )
		{
			ROBOT_DISPLAY( TRUE, "LASER PARSE ERROR: Line recieved without 2 LF Terminator" )
			return 0;
		}

		if( dwSIOBytesReceived <= (m_ParseBufCount+2) )
		{
			bMoreDataToRead = FALSE;	// Normally, we only get one line of data at a time
		}
		else
		{
			ROBOT_LOG( TRUE,  "LASER PARSE: Note: Extra data in line.  dwSIOBytesReceived = %d,  m_ParseBufCount = %d\n", dwSIOBytesReceived, m_ParseBufCount )
		}

		// Start by getting all the line feeds, since they provide index into fields
		#define MAX_LINEFEEDS_ALLOWED	64
		int LfPosition[MAX_LINEFEEDS_ALLOWED];
		int NumOfLF = 0;
		memset(LfPosition, 0, MAX_LINEFEEDS_ALLOWED);
		for( int i=0; i< m_ParseBufCount; i++ )
		{
			if( '\n' == m_pParseBuf[i] )
			{
				if( NumOfLF >= MAX_LINEFEEDS_ALLOWED )
				{
					ROBOT_DISPLAY( TRUE, "LASER PARSE ERROR: too many LF in received buffer!" )
					return 0;
				}
				LfPosition[NumOfLF++] = i; // Found LF, save positon in the array
			}
		}

		// CHECK COMMAND ECHO
	/*	if( strncmp(m_pParseBuf, g_LaserScannerState.LastCmd, 6) )
		{
			// commands don't match, but sometimes this is normal overlap
			char tmpStr[16];
			strncpy(tmpStr, m_pParseBuf,6);
			MsgString.Format("LASER ERROR: COMMAND ECHO: %s != %s", g_LaserScannerState.LastCmd, tmpStr );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
			//return 0;
		}
	*/
		if( !strncmp( m_pParseBuf, "MS", 2) )
		{
			// "MS" (Get Scans) Command. Get # REMAINING SCANS (for multi scan commands)
			char Scan[3];
			Scan[0] = m_pParseBuf[13];
			Scan[1] = m_pParseBuf[14];
			Scan[2] = 0; // terminate
			g_LaserScansRemaining = atoi(Scan);
#if DEBUG_LASER_SHOW_SCANS == 1
			ROBOT_LOG( TRUE,  "Scan: %d\n", g_LaserScansRemaining);
			if( 0 == g_LaserScansRemaining )
			{
				ROBOT_LOG( TRUE,  "\n" );
			}
#endif
		}
		else if( !strncmp( m_pParseBuf, "RS", 2) )
		{
			// "RS" (Reset) Command.
			ROBOT_LOG( TRUE, "Laser Scanner Reset Ack received\n")
		}
		else if( !strncmp( m_pParseBuf, "QT", 2) )
		{
			// "QT" (Laser Off) Command.
		}
		else if( !strncmp( m_pParseBuf, "BM", 2) )
		{
			// "BM" (Laser On) Command.
		}
		else
		{
			char tmpStr[32];
			strncpy_s(tmpStr,32,m_pParseBuf,30);

			MsgString.Format("LASER ERROR: BAD COMMAND: (%s)", tmpStr );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
			g_LaserScansRemaining = 0;
			return 0;
		}
		// GET STRING (Optional - add here if desired)

		// GET STATUS
		char Status[3];
		Status[0] = m_pParseBuf[ (LfPosition[LF_POS_STATUS])+1 ];	// Status follows the first LF
		Status[1] = m_pParseBuf[ (LfPosition[LF_POS_STATUS])+2 ];
		Status[2] = 0; // terminate
		int nStatus = atoi(Status);

		// VERIFY STATUS *CHECKSUM*
		char StatusChecksum = m_pParseBuf[ (LfPosition[LF_POS_STATUS])+3 ];
		if( !CompareChecksum( StatusChecksum, ((LfPosition[LF_POS_STATUS])+1), 2 ) )
		{
			// Bad Checksum!  Abort.
			MsgString.Format("LASER ERROR: BAD STATUS CHECKSUM" );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
			ROBOT_ASSERT(0);
			Init();	// reset everything
			g_LaserScansRemaining = 0;
			return 0;
		}

		// REPORT STATUS
		DisplayStatus( nStatus );
		g_LaserSubSystemStatus = SUBSYSTEM_CONNECTED;

		// SEE IF THIS IS JUST THE COMMAND ECHO
		if( 0 == m_pParseBuf[ (LfPosition[LF_POS_STATUS])+4 ] )
		{
			// This is just the command echo.  No Data
			return 0;	// 0 means don't send command to other threads to act on new data
		}
		
		// GET TIMESTAMP
		char TimeStamp[5];
		memcpy_s( TimeStamp, LASER_SCANNER_READ_BUF_SIZE, &(m_pParseBuf[ (LfPosition[LF_POS_TIME_STAMP])+1 ]), 4 );
		TimeStamp[4] = 0;
		int  nTimeStamp = ConvertEncodedCharData( 4, TimeStamp );
		//ROBOT_LOG( TRUE, "LASER: TimeStamp = %d\n", nTimeStamp)

		// VERIFY TIMESTAMP CHECKSUM
		char TimeStampChecksum = m_pParseBuf[ (LfPosition[LF_POS_TIME_STAMP])+5 ];
		if( !CompareChecksum( TimeStampChecksum, ((LfPosition[LF_POS_TIME_STAMP])+1), 4 ) )
		{
			// Bad Checksum!  Abort.
			MsgString.Format("LASER ERROR: BAD TIMESTAMP CHECKSUM" );
			ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
			//ROBOT_ASSERT(0);
			Init();	// reset everything
			g_LaserScansRemaining = 0;
			return 0;
		}

		// GET DATA BLOCKS
		// Assumes 2 char encoding
		// Data blocks start at LfPosition[LF_POS_FIRST_DATA_BLOCK]+1
		int nDataValue = 0;		// Converted data byte value
		int  nDataBytesDecoded = 0;	// Data Byte iterator
		int nDataChars = 0;		// Number of characters in the data block 
		int  LargestValue = 0;

	#if ( DEBUG_LASER_SHOW_RAW_RX_DATA == 1 )
		ROBOT_LOG( TRUE,  "LASER ParseBuffer: DEBUG Data:\n" );
		ROBOT_LOG( TRUE,  "                                                      1                                                 2                                                 3                                                 4                                                 5                                                 6\n" );
		ROBOT_LOG( TRUE,  "    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6\n"    );	//    For    DEBUG
	#endif
		for( int LineFeed=LF_POS_FIRST_DATA_BLOCK; LineFeed < (NumOfLF-1); LineFeed++ )
		{
			// For each data block (64 bytes max?)
			nDataChars = (LfPosition[LineFeed+1] - LfPosition[LineFeed]) -2;	// subtract out the LF's
			int  nBlockStartPosition = (LfPosition[LineFeed]+1); // get start of current data block (first byte after the line feed)

			// Convert all the data in this block from 2 char encoded to int  
			for( int i = 0; i<nDataChars; i+=2 )
			{			
				nDataValue= ConvertEncodedCharData( 2, &(m_pParseBuf[ nBlockStartPosition+i ]) ); // 2 char decoding
				m_pDataArray[nDataBytesDecoded++] = nDataValue;

				// DEBUG
				if( nDataBytesDecoded >= LASER_RANGEFINDER_MAX_SAMPLES-1 )
				{
					ROBOT_ASSERT(0);
				}
			}
			// GET DATABLOCK CHECKSUM
			char DataChecksum = m_pParseBuf[ (LfPosition[LineFeed])+ nDataChars+1 ];
			if( !CompareChecksum( DataChecksum, nBlockStartPosition, nDataChars ) )
			{
				// Bad Checksum!  Abort.
				MsgString.Format("\nLASER ERROR: BAD DATA CHECKSUM\n" );
				ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
				//ROBOT_ASSERT(0);
				Init();	// reset everything
				g_LaserScansRemaining = 0;
				return 0;
			}

			// TODO!!! DUMP EACH DATA BLOCK (ASCII AND RESULTING VALUES)
			// FOR DEBUG!
		}
		if( LargestValue > LASER_RANGEFINDER_MM_MAX )
		{
			ROBOT_ASSERT(0);
		}

		__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, pshProcessScanData);
		ProcessScanData( nDataBytesDecoded );
		__itt_task_end(pDomainLaserThread);

	}	// while( bMoreDataToRead )

		return TRUE;	// got data!

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ProcessScanData
// Got a buffer of scan data from the laser scanner.
// Convert from mm to tenth-inches
// And calculate the X,Y,Z of each data point in reference to the Robot's center at the floor
// Results are stored in a global buffer for use by other modules
// Uses data passed in m_pDataArray
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
void CLaserScannerParser::ProcessScanData( int  nDataSamples )
{
	if( !m_Initialized ) return;

	double	RangeTenthInches = 0;
	double	LaserHeightAboveFloor = 0.0;
	double	LaserDistanceFromFront = 1.0;
	int 	TenthInches = 0;
	int 	LargestValueTenthInches = 0;
	double	FloorDistance = 0.0;
	double	LaserStepAngle = 0;
	double	LaserTiltDegrees = 0;

	// Same as LASER_RANGEFINDER_DEGREES_PER_STEP ??
	double DegreeIncrement = 180.0 / (double)nDataSamples;	// step angle of each sample
	double	X,Y;

	// Set all summary values to NO_OBJECT_IN_RANGE
	InitScannerSummaryData( &m_LaserSummary ); 

	// Save time-sensitive data
	m_LaserSummary.SampleTimeStamp = GetTickCount();	// Time laser sample was done
	m_LaserSummary.RobotLocation.x = g_SensorStatus.CurrentLocation.x;		
	m_LaserSummary.RobotLocation.y = g_SensorStatus.CurrentLocation.y;		
	m_LaserSummary.CompassHeading = g_SensorStatus.CompassHeading;		// Heading of robot at the time of the laser scan

	//ROBOT_LOG( TRUE,  "\n========= LASER PROCESSED DATA ==================\n\n" )
	__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, psh_csLaserDataLock);
	EnterCriticalSection(&g_csLaserDataLock);

		for( int  i=0; i<nDataSamples; i++ )
		{
			if( m_pDataArray[i] < 20 )	// Numbers below 20 = Laser Error Numbers.  See manual
			{
				RangeTenthInches =  (double)LASER_RANGEFINDER_TENTH_INCHES_ERROR;
			}
			else
			{
				RangeTenthInches = ((double)(m_pDataArray[i]) / 2.540);
				if( RangeTenthInches > LASER_RANGEFINDER_TENTH_INCHES_MAX ) RangeTenthInches = LASER_RANGEFINDER_TENTH_INCHES_MAX;
			}
		
			// Save Distance (in tenth inches)
			g_pLaserScannerData->ScanData[i] =  (int )(RangeTenthInches);

			// Now, convert to X,Y coordinates

			if( RangeTenthInches >= LASER_RANGEFINDER_TENTH_INCHES_MAX )
			{
				X = LASER_RANGEFINDER_TENTH_INCHES_MAX;	// multiplied by 10 before saving.  use LASER_RANGEFINDER_TENTH_INCHES_MAX to compare
				Y = LASER_RANGEFINDER_TENTH_INCHES_MAX;
			}
			else
			{
				LaserStepAngle = 90 - (DegreeIncrement * (double)i);

				// Get x,y from Laser's perspective. zero = straight forward facing.  Down = negative
				// Convert from Spherical Coordinates to X,Y coordinates for each point
				X = RangeTenthInches * sin(DEGREES_TO_RADIANS*(LaserStepAngle) );
				Y = RangeTenthInches * cos(DEGREES_TO_RADIANS*(LaserStepAngle));

				// Now, translate origin from laser position to front of robot
				if( Y > 0 )
				{
					Y = Y - LaserDistanceFromFront;
				}
				else
				{
					CString StrText;
					StrText.Format( _T("ERROR! CLaserScannerParser::ProcessScanData: Negative Y Value!:%d\n"), Y);
					ROBOT_DISPLAY( TRUE, StrText)
					ROBOT_ASSERT(0);
				}
			}

			// Save the data in Tenth Inches
			g_pLaserScannerData->ScanPoints[i].X =  (int)(X );	
			g_pLaserScannerData->ScanPoints[i].Y =  (int)(Y );	
		}

		// Now, save info on where the scan was taken (current compass heading, location on map)
		// Used by other modules to determine objects in relation to robot
		g_pLaserScannerData->NumberOfSamples = nDataSamples;
		g_pLaserScannerData->CompassHeading = m_LaserSummary.CompassHeading; // Heading of robot at the time of the laser scan
		g_pLaserScannerData->RobotLocation.x = m_LaserSummary.RobotLocation.x; // Location of robot at the time of the laser scan
		g_pLaserScannerData->RobotLocation.y = m_LaserSummary.RobotLocation.y;

	LeaveCriticalSection(&g_csLaserDataLock);
	__itt_task_end(pDomainLaserThread);


	/////////////////////////////////////////////////////
	// Update summary data, used for collision avoidance

	// Now find the minimum Y value for each zone
	for( int  i=0; i<nDataSamples; i++ )
	{
		int X = g_pLaserScannerData->ScanPoints[i].X;
		int Y = g_pLaserScannerData->ScanPoints[i].Y;

		if( (X >= LASER_RANGEFINDER_TENTH_INCHES_MAX) || (Y >= LASER_RANGEFINDER_TENTH_INCHES_MAX) )
		{
			continue; // Skip Error value
		}

		// Determine the zone.
		if( X < 0 )
		{
			// Front Left
			if( X >= -FRONT_CENTER_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_LaserSummary.nLeftFrontZone ) 
					m_LaserSummary.nLeftFrontZone = Y;
			}
			else if( X >= -FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_LaserSummary.nLeftArmZone ) 
					m_LaserSummary.nLeftArmZone = Y;
			}
			else if( X >= -FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_LaserSummary.nLeftFrontSideZone ) 
					m_LaserSummary.nLeftFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_LaserSummary.nLeftSideZone ) m_LaserSummary.nLeftSideZone = SideDist;
			}
		}
		else
		{
			// Front Right
			if( X <= FRONT_CENTER_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_LaserSummary.nRightFrontZone ) 
					m_LaserSummary.nRightFrontZone = Y;
			}
			else if( X <= FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_LaserSummary.nRightArmZone ) 
					m_LaserSummary.nRightArmZone = Y;
			}
			else if( X <= FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )  
			{
				if( Y < m_LaserSummary.nRightFrontSideZone ) 
					m_LaserSummary.nRightFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_LaserSummary.nLeftSideZone ) m_LaserSummary.nLeftSideZone = SideDist;
			}
		}
	}

	// Finally, update the global data to indicate to other modules we have a new measurement
	__itt_task_begin(pDomainLaserThread, __itt_null, __itt_null, psh_csLaserSummaryDataLock);
	EnterCriticalSection(&g_csLaserSummaryDataLock);
		memcpy_s(g_pLaserSummary, sizeof(SCANNER_SUMMARY_T), &m_LaserSummary, sizeof(SCANNER_SUMMARY_T) );
	LeaveCriticalSection(&g_csLaserSummaryDataLock);
	__itt_task_end(pDomainLaserThread);

#if ( DEBUG_LASER_SHOW_FINAL_DATA == 1 )
	ROBOT_LOG( TRUE,  "LASER ParseBuffer: FINAL Data: (%d readings)\n", nDataSamples );
	ROBOT_LOG( TRUE,  "                                                    1                                                 2                                                 3              \n" );
	ROBOT_LOG( TRUE,  "       1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    3    4    5    6    7    8    9    0    1    2    \n"    );	//    For    DEBUG

	int linecount = 0;
	for(int  i=0; i< nDataSamples; i++)
	{
		if( 0 == (i%32) )
		{
			if( ((i/32) == 7) || ((i/32+1) == 10) ) ROBOT_LOG( TRUE,  "\n" );
			ROBOT_LOG( TRUE,  "\n%02d  ", (i/32)+1);
		}
		if( g_pLaserScannerData->ScanData[i] < 100 )
			ROBOT_LOG( TRUE,  "--%02d ", g_pLaserScannerData->ScanData[i]);
		else
			ROBOT_LOG( TRUE,  "%04d ", g_pLaserScannerData->ScanData[i]);
	}
	ROBOT_LOG( TRUE,  "\n\n" );
#endif
	// TEST

	// For DEBUG testing 
	/***
	for(int i=0; i<600; i+=20)
	{
		for(int m =0; m <10; m++ )
		{
			g_LaserScannerState.Data[i+m] = 1000;
		}
		for(int m =10; m <20; m++ )
		{
			g_LaserScannerState.Data[i+m] = 500;
		}
	}
	g_LaserScannerState.NumberOfSamples = 600;
	g_LaserScannerState.LargestValue = 500;
	***/


}

///////////////////////////////////////////////////////////////////////////////
// Utilities
///////////////////////////////////////////////////////////////////////////////
BOOL CLaserScannerParser::CompareChecksum( 
	char CheckSum,		// Character to compare
	int  nStartPos,		// Starting position of data
	int  nChars )		// Number of chars to sum
{
	// Compare SCIP "SUM" of characters between two LF delimeters
	// sum defined as sum of chars, trim to 6 bits, and add 30 (to convert to an ASCII char)

	int  CalcSum = 0; 
	for( int i=0; i<nChars; i++ )
	{
		//if( i >= 62 )
		//	ROBOT_LOG( TRUE,  "near\n" );
		CalcSum += (int )m_pParseBuf[nStartPos+i];
	}

	CalcSum &= 0x3F;	// Trim to 6 bits
	CalcSum += 0x30;	// Convert to ASCII

	// Compare
	if( CalcSum != (BYTE)CheckSum )
	{
		ROBOT_LOG( TRUE, "LASER CHECKSUM FAILED: Calculated: %02x Received: %02x\n", (int )CalcSum, (int )CheckSum )
		return FALSE;
	}

	return TRUE;

}

int  CLaserScannerParser::ConvertFourCharData( char *CharData )
{
	int  nData = 0;

	for(int i=0; i<4; i++ )
	{
		int  nTemp = (BYTE)CharData[i] - 0x30;
		nData += nTemp << (6*(3-i));
	}
	return nData;
}

int  CLaserScannerParser::ConvertEncodedCharData( int  EncodeType, char *CharData )
{
	// EncodeType is 2,3,or 4 char encoding

	int  nData = 0;

	for(int  i=0; i<EncodeType; i++ )
	{
		int  nTemp = (BYTE)CharData[i] - 0x30;
		nData += nTemp << (6*((EncodeType-1)-i));
	}
	return nData;
}


void CLaserScannerParser::DisplayStatus( int  nStatus )
{
	switch( nStatus )
	{
		case 0:
			//ROBOT_LOG( TRUE, "LASER Status 0: OK (Command ACK)\n")
			return;
			break;
		case 99:
			//ROBOT_LOG( TRUE, "LASER Status 99: OK, with Data \n")
			return;
			break;
		case 1:
			ROBOT_LOG( TRUE, "LASER Status Error 1: Starting Step Non-Numeric\n")
			break;
		case 2:
			ROBOT_LOG( TRUE, "LASER Status Error 2: End Step Non-Numeric\n")
			break;
		case 3:
			ROBOT_LOG( TRUE, "LASER Status Error 3: Cluster Count Non-Numeric\n")
			break;
		case 4:
			ROBOT_LOG( TRUE, "LASER Status Error 4: End Step Out of Range\n")
			break;
		case 5:
			ROBOT_LOG( TRUE, "LASER Status Error 5: End Step Smaller then Starting Step\n")
			break;
		case 6:
			ROBOT_LOG( TRUE, "LASER Status Error 6: Scan Interval Non-Numeric\n")
			break;
		case 7:
			ROBOT_LOG( TRUE, "LASER Status Error 7: Number of Scans Non-Numeric\n")
			break;
		case 98:
			ROBOT_LOG( TRUE, "LASER Status 98: Resuming after confirming normal operation\n")
			break;
		default:
			if( (nStatus >=20) && (nStatus <=49) )
				ROBOT_LOG( TRUE, "LASER Status %d: Processing Stopped to verify Error\n", nStatus)
			else if( (nStatus >=50) && (nStatus <=97) )
				ROBOT_LOG( TRUE, "LASER Status %d: Hardware Trouble!\n", nStatus)
			else 
				ROBOT_LOG( TRUE, "LASER Status %d: UNKNOWN ERROR!\n", nStatus)
	}
	ROBOT_LOG( TRUE, "\n", nStatus)	// Only add extra line feed if an error occured
	//ROBOT_ASSERT(0); // force break on error for now
}



#endif // ROBOT_SERVER - This module used for Robot Server only

