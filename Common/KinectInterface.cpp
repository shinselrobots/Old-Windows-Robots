// KinectInterface.cpp: Kinect Remote Control class
//
//////////////////////////////////////////////////////////////////////


#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 ) 	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"

#include <mmsystem.h>	// for timeGetTime

//#include "..\Common\KinectInterfaceCommon.h"
#include "headcontrol.h"
#include "armcontrol.h"
#include "KinectInterface.h"


__itt_string_handle* pshKinectUpdate = __itt_string_handle_create("pshKinectUpdate");




KinectInterface::KinectInterface()
{

	m_hMapFile = INVALID_HANDLE_VALUE;
	m_pBuf = NULL;	// Shared Buffer space

	m_bKinectSharedMemoryOpened = FALSE;
//	m_pArmControlRight = new ArmControl( RIGHT_ARM );
//	m_pArmControlLeft = new ArmControl( LEFT_ARM );

	ROBOT_LOG( TRUE,  "Kinect Control constructor complete\n" )
}

KinectInterface::~KinectInterface()
{
	if( NULL != m_pBuf )
	{
		UnmapViewOfFile(m_pBuf);
	}
	if( INVALID_HANDLE_VALUE != m_hMapFile)
	{
		CloseHandle(m_hMapFile);
	}
//	SAFE_DELETE(m_pArmControlRight);
//	SAFE_DELETE(m_pArmControlLeft);
//	SAFE_DELETE(m_pHeadControl);
	ROBOT_LOG( TRUE,  "~KinectInterface done\n" )
}


//-----------------------------------------------------------------------------
// Name: Initialize
// Desc: Initialize Kinect Control
//-----------------------------------------------------------------------------
void KinectInterface::Initialize()
{
	// Initialize shared memory for getting input from KinectInterface
	TCHAR szKinectInterfaceSharedFileName[]=TEXT(KINECT_INTERFACE_SHARED_FILE_NAME);

	m_hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		szKinectInterfaceSharedFileName);	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == m_hMapFile) || (NULL == m_hMapFile)  )
	{ 
		ROBOT_LOG( TRUE,  "KinectInterface: Could not open file mapping object (%d).\n", GetLastError())
	}
	else
	{
		m_pBuf = (LPCTSTR)MapViewOfFile(m_hMapFile, // handle to map object
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,                    
			0,                    
			(sizeof(KINECT_DATA_T)) );                   

		if (m_pBuf == NULL) 
		{ 
			ROBOT_LOG( TRUE,  "KinectInterface: Could not map view of file (%d).\n", GetLastError());
			CloseHandle(m_hMapFile);
		}
		else
		{
			m_bKinectSharedMemoryOpened = TRUE;
			ROBOT_DISPLAY( TRUE, "KinectInterface: Shared Memory Sucess!" )
		}
	}

}


// ------------------------------------------------------------------------------------
// Name: Update
// Desc: Get updated status from the Kinect, and sends commands as needed
//-------------------------------------------------------------------------------------
void KinectInterface::Update()
{
	if( !m_bKinectSharedMemoryOpened )
	{
		return;
	}

	__itt_task_begin(pDomainGlobalThread, __itt_null, __itt_null, pshKinectUpdate);


	// Get updated data from the KinectInterface
//				ROBOT_LOG( TRUE,  "Message from KinectInterface\n" )
	
	KINECT_DATA_T *tempStruct = (KINECT_DATA_T*)m_pBuf;
	ROBOT_LOG( TRUE,  "KinectInterface: DirectCast   = %d, %d, %d, %d\n", 
		tempStruct->FrameNumber, tempStruct->Height, tempStruct->Width, tempStruct->PixelDataLength )
		

	TRACE( "KinectInterface: BUFFER: ");
	int offset = 16;
	for( int i = (offset); i<(offset+16); i++ )
//	for( int i = (offset); i<(tempStruct->PixelDataLength); i++ )
	{
		TRACE( "%02X, ", m_pBuf[i] );
	}
	TRACE( "\n");


	//int DepthDataCopy[MAX_DEPTH_DATA_SIZE];
	//CopyMemory( &DepthDataCopy, tempStruct->DepthData, tempStruct->PixelDataLength );

//	CopyMemory(&m_KinectData, m_pBuf, (sizeof(KINECT_DATA_T)));
//	ROBOT_LOG( TRUE,  "KinectInterface: DataCopy     = %d, %d, %d\n", m_KinectData.IntTest1, m_KinectData.IntTest2, m_KinectData.IntTest3 )
		// DO STUFF HERE

	// To WRITE to the shared file, do this:
	//	tempStruct->IntTest1++;
	//	tempStruct->IntTest3 = 1;

	__itt_task_end(pDomainGlobalThread);

}

#endif // ROBOT_SERVER - This module used for Robot Server only
