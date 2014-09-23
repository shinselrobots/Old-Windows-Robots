///////////////////////////////////////////////////////////////////////////////
// NMEAParser.h: 
// Desctiption:	interface for the CNMEAParser class.
//
// Notes:
//		NMEA Messages parsed:
//			GPGGA, GPGSA, GPGSV, GPRMB, GPRMC, GPZDA
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1998-2002 VGPS
// All rights reserved.
//
// This copy of VGPS distributed with permission from Monte Variakojis, 
// VisualGPS, LLC.  
// For the latest version, got to www.visualgps.net and look at Whitepapers.
//
// VGPS licenses this source code for use within your application in
// object form. This source code is not to be distributed in any way without
// prior written permission from VGPS.
//
// Visual Source Safe: $Revision: 6 $
///////////////////////////////////////////////////////////////////////////////
#ifndef _NMEAPARSER_H_
#define _NMEAPARSER_H_
#include "RobotConfig.h"

//////////////////////////////////////////////////////////////////////

enum NP_STATE {
	NP_STATE_SOM =				0,		// Search for start of message
	NP_STATE_CMD,						// Get command
	NP_STATE_DATA,						// Get data
	NP_STATE_CHECKSUM_1,				// Get first checksum character
	NP_STATE_CHECKSUM_2,				// get second checksum character
};

#define NP_MAX_CMD_LEN			8		// maximum command length (NMEA address)
#define NP_MAX_DATA_LEN			256		// maximum data length
#define NP_MAX_CHAN				36		// maximum number of channels
#define NP_WAYPOINT_ID_LEN		32		// waypoint max string len

//////////////////////////////////////////////////////////////////////
class CNPSatInfo
{
public:
	WORD	m_wPRN;						//
	WORD	m_wSignalQuality;			//
	BOOL	m_bUsedInSolution;			//
	WORD	m_wAzimuth;					//
	WORD	m_wElevation;				//
};
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
class CNMEAParser  
{
private:
	NP_STATE m_nState;					// Current state protocol parser is in
	BYTE m_btChecksum;					// Calculated NMEA sentence checksum
	BYTE m_btReceivedChecksum;			// Received NMEA sentence checksum (if exists)
	WORD m_wIndex;						// Index used for command and data
	BYTE m_pCommand[NP_MAX_CMD_LEN];	// NMEA command
	BYTE m_pData[NP_MAX_DATA_LEN];		// NMEA data
	BOOL m_bDataReady;					// NMEA Data is ready to be read

public:
	DWORD m_dwCommandCount;				// number of NMEA commands received (processed or not processed)

	//
	// GPGGA Data
	//
	double m_dGGALatitude;				// < 0 = South, > 0 = North
	double m_dGGALongitude;				// < 0 = West, > 0 = East
	double m_dGGAHDOP;					//
	double m_dGGAAltitude;				// Altitude: mean-sea-level (geoid) meters
	double m_dGGAOldVSpeedAlt;			//
	double m_dGGAVertSpeed;				//
	DWORD m_dwGGACount;					//
	int m_nGGAOldVSpeedSeconds;			//
	BYTE m_btGGAHour;					//
	BYTE m_btGGAMinute;					//
	BYTE m_btGGASecond;					//
	BYTE m_btGGAGPSQuality;				// 0 = fix not available, 1 = GPS sps mode, 2 = Differential GPS, SPS mode, fix valid, 3 = GPS PPS mode, fix valid
	BYTE m_btGGANumOfSatsInUse;			//

	//
	// GPGSA
	//
	BYTE m_btGSAMode;					// M = manual, A = automatic 2D/3D
	BYTE m_btGSAFixMode;				// 1 = fix not available, 2 = 2D, 3 = 3D
	WORD m_wGSASatsInSolution[NP_MAX_CHAN]; // ID of sats in solution
	double m_dGSAPDOP;					//
	double m_dGSAHDOP;					//
	double m_dGSAVDOP;					//
	DWORD m_dwGSACount;					//

	//
	// GPGSV
	//
	BYTE m_btGSVTotalNumOfMsg;			//
	WORD m_wGSVTotalNumSatsInView;		//
	CNPSatInfo m_GSVSatInfo[NP_MAX_CHAN];	//
	DWORD m_dwGSVCount;					//

	//
	// GPRMB
	//
	double m_dRMBCrosstrackError;		// nautical miles
	double m_dRMBDestLatitude;			// destination waypoint latitude
	double m_dRMBDestLongitude;			// destination waypoint longitude
	double m_dRMBRangeToDest;			// Range to destination nautical mi
	double m_dRMBBearingToDest;			// Bearing to destination, degrees true
	double m_dRMBDestClosingVelocity;	// Destination closing velocity, knots
	DWORD m_dwRMBCount;					//
	BYTE m_btRMBDataStatus;				// A = data valid, V = navigation receiver warning
	BYTE m_btRMBDirectionToSteer;		// L/R
	BYTE m_btRMBArrivalStatus;			// A = arrival circle entered, V = not entered
	CHAR m_lpszRMBOriginWaypoint[NP_WAYPOINT_ID_LEN]; // Origin Waypoint ID
	CHAR m_lpszRMBDestWaypoint[NP_WAYPOINT_ID_LEN]; // Destination waypoint ID

	//
	// GPRMC
	//
	double m_dRMCLatitude;				// current latitude
	double m_dRMCLongitude;				// current longitude
	double m_dRMCGroundSpeed;			// speed over ground, knots
	double m_dRMCCourse;				// course over ground, degrees true
	double m_dRMCMagVar;				// magnitic variation, degrees East(+)/West(-)
	DWORD m_dwRMCCount;					//
	WORD m_wRMCYear;					//
	BYTE m_btRMCHour;					//
	BYTE m_btRMCMinute;					//
	BYTE m_btRMCSecond;					//
	BYTE m_btRMCDataValid;				// A = Data valid, V = navigation rx warning
	BYTE m_btRMCDay;					//
	BYTE m_btRMCMonth;					//

	//
	// GPZDA
	//
	BYTE m_btZDAHour;					//
	BYTE m_btZDAMinute;					//
	BYTE m_btZDASecond;					//
	BYTE m_btZDADay;					// 1 - 31
	BYTE m_btZDAMonth;					// 1 - 12
	WORD m_wZDAYear;					//
	BYTE m_btZDALocalZoneHour;			// 0 to +/- 13
	BYTE m_btZDALocalZoneMinute;		// 0 - 59
	DWORD m_dwZDACount;					//

public:
	void UpdateGlobalData();	// Copy local values to global data
	void ProcessGPZDA(BYTE *pData);
	void ProcessGPRMC(BYTE *pData);
	void ProcessGPRMB(BYTE *pData);
	void ProcessGPGSV(BYTE *pData);
	void ProcessGPGSA(BYTE *pData);
	void ProcessGPGGA(BYTE *pData);
	BOOL IsSatUsedInSolution(WORD wSatID);
	void Reset();
	BOOL GetField(BYTE *pData, BYTE *pField, int nFieldNum, int nMaxFieldLen);
	BOOL ProcessCommand(BYTE *pCommand, BYTE *pData);
	void ProcessNMEA(BYTE btData);
	BOOL ParseBuffer(BYTE *pBuff, DWORD dwLen);
	CNMEAParser();
	virtual ~CNMEAParser();
};

//////////////////////////////////////////////////////////////////////
#endif // _NMEAPARSER_H_
