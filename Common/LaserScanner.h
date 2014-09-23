// LaserScanner.h
// Interface for Hokuyo URG-04LX-UG01 Laser Scanner, using SCIP 2.0 Protocol
// See "Updated SCIP20.pdf" for details and www.acroname.com

#pragma once

#include "RobotConfig.h"

/////////////////////////////////////////////////////////////////////////////
class CLaserScannerCommand
{
	// Sends commands to the Hokuyo URG-04LX-UG01 Laser Scanner, using SCIP 2.0 Protocol
	// Called from LaserScannerCommWriteThreadFunc

public:
	CLaserScannerCommand();
	~CLaserScannerCommand();

void	Init();
void	SendCmd();
void	EnableLaser( BOOL Enable );
void	RequestScans( int  nClusters, int   nIntervals, int  nScans );
void	HandleCommand( int  Request, int  Param1, int  Param2 );


protected:
	char				m_CmdBuf[LASER_SCANNER_CMD_BUF_SIZE];

};


/////////////////////////////////////////////////////////////////////////////
class CLaserScannerParser
{
	// Reads data from the Hokuyo URG-04LX-UG01 Laser Scanner, using SCIP 2.0 Protocol
	// Called from LaserScannerCommReadThreadFunc

	public:
		CLaserScannerParser();
		~CLaserScannerParser();

	void	Init();
	BOOL	ParseBuffer( char *LaserSIOBuf, int  dwSIOBytesReceived );
	void	ProcessScanData( int  nDataSamples );

	private:
	void	DisplayStatus( int  nStatus );
	int 	ConvertFourCharData( char *CharData );
	int 	ConvertEncodedCharData( int  EncodeType, char *CharData );
	void	HandleCommand( int  Request, int  Param1, int  Param2 );
	BOOL	CompareChecksum(char CheckSum,		// Character to compare
							int  nStartPos,		// Starting position of data
							int  nChars );		// Number of chars to sum

	protected:
	char					*m_pParseBuf;	// local copy for parsing
	int 					*m_pDataArray;
	int						 m_ParseBufCount;
	BOOL					 m_Initialized;
	SCANNER_SUMMARY_T		 m_LaserSummary;

};
