// KinectInterface.h: Wii Remote Control class
//////////////////////////////////////////////////////////////////////
#pragma once	// Only include this header once

#ifdef SHOULD_I_REMOVE_THIS



#include "RobotConfig.h"

#define MAX_DEPTH_DATA_SIZE	(640*480)


typedef struct
{
	unsigned int	FrameNumber;
	unsigned int	Height;
	unsigned int	Width;
	unsigned int	PixelDataLength;
	unsigned char	DepthData[MAX_DEPTH_DATA_SIZE];

} KINECT_DATA_T; // Must match struct in Managed code!



/////////////////////////////////////////////////////////////////////////////
class KinectInterface
{

public:


	KinectInterface::KinectInterface();
	KinectInterface::~KinectInterface();

	//-----------------------------------------------------------------------------
	// Name: Initialize
	// Desc: Initialize Kinect Interface Control
	//-----------------------------------------------------------------------------
	void Initialize();

	// ------------------------------------------------------------------------------------
	// Name: Update
	// Desc: Get updated status from the Kinect
	//-------------------------------------------------------------------------------------
	void Update();



private:

	HANDLE			m_hMapFile;
	LPCTSTR			m_pBuf;	// Shared Buffer space
//	WIIMOTE_STATUS_T m_WiiMoteStatus;
	KINECT_DATA_T	m_KinectData;

	BOOL			m_bKinectSharedMemoryOpened;
//	int 			m_ManualArmControlRight;
//	int 			m_ManualArmControlLeft;
//	ArmControl	   *m_pArmControlRight;
//	ArmControl	   *m_pArmControlLeft;
//	HeadControl	   *m_pHeadControl;	// For controlling head servos

};

#endif SHOULD_I_REMOVE_THIS