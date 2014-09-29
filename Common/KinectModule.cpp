// KinectModule.cpp: CKinectModule class implementation
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only


// THIS MUST BE IN FRONT OF module.h include
#define SHOW_XYZ_WINDOW							 0 // Show an OpenCV view of the point cloud


#include <math.h>
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "HardwareConfig.h"

//#include "opencv2/core/core.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/highgui/ 1 == ROBOT_SIMULATION_MODEhighgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

#include "kinect.h"
#include <SpeechEnums.cs>

#define USE_OPEN_CV								0


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define TRACK_TOP_OF_USERS_HEAD					 1 
#define USE_KINECT_TO_LOOKAT_HUMAN				 0 // NOTE!  REQUIRES "TRACK_TOP_OF_USERS_HEAD"
#define TILT_KINECT_TO_TRACK_CLOSE_HUMANS		 0
#define DEBUG_SHARED_MEMORY						 0 // Show source data from shared memory
#define DEBUG_FIND_OBJECTS_ON_FLOOR				 0 // For debug, Just keep targeting and showing 3D objects found

#define DEBUG_KINECT_SHOW_2D_OBJECTS_FOUND		 0
#define DEBUG_KINECT_SHOW_2D_OBJECTS_NOT_FOUND	 0
#define DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME	 0
#define DEBUG_KINECT_SHOW_3D_SLICE_OBJECTS_FOUND 0
#define DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES	 0
#define KINECT_TILT_LOOKING_AT_FLOOR			(KINECT_TILT_CENTER - 200) // KinectTiltTenthDegrees
#define KINECT_BAD_PIXELS_MAX					20
#define OBJECT_DISTANCE_COMPENSATION			60	// Require bigger objects at far distances

///////////////////////////////////////////////////////////////
// This module completely converted to new Microsoft SDK!
#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
	#error BAD KINECT_SDK_TYPE
#endif

// NOTE: Make sure to add one of these libraries to the Linker Input Dependencies:
// For Kinect SDK, add MSRKinectNUI.lib
// For OpenNI SDK, add openNI.lib
///////////////////////////////////////////////////////////////


// ITT Instrumentation
__itt_string_handle* pshKinectThreadLoop = __itt_string_handle_create("KinectThreadLoop");
//__itt_string_handle* pshKinectInit = __itt_string_handle_create("KinectInit");
//__itt_string_handle* pshKinectGetFrame = __itt_string_handle_create("KinectGetFrame");
__itt_string_handle* pshKinectGetDepthImage = __itt_string_handle_create("GetDepthImage");
__itt_string_handle* pshKinectFrameLoop = __itt_string_handle_create("KinectFrameLoop");
//__itt_string_handle* pshKinectShowFrames = __itt_string_handle_create("KinectShowFrames");
__itt_string_handle* pshKinectSleep = __itt_string_handle_create("KinectSleep");
__itt_string_handle* pshFindObjectsOnFloor = __itt_string_handle_create("Find Objects on Floor");

__itt_string_handle* pshKinectServoStart = __itt_string_handle_create("KinectServoMoveStart"); // marker
__itt_string_handle* pshKinectServoEnd = __itt_string_handle_create("KinectServoMoveEnd"); // marker

__itt_string_handle* psh_csFindWallsAnd2dMaps = __itt_string_handle_create("FindWallsAnd2dMaps"); // marker
__itt_string_handle* psh_UpdateKinectObjectSummary = __itt_string_handle_create("UpdateKinectObjectSummary"); // marker

__itt_string_handle* psh_csUpdateKinectObjectSummary = __itt_string_handle_create("ObjectSummary"); //marker

__itt_string_handle* psh_ProcessVerticalScan = __itt_string_handle_create("ProcessVerticalScan");

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

#define KINECT_VIDEO_ENABLED  1

// Calculaiton of Kinect Field Of View (FOV) in degrees
// Robot at 36.0" from wall:  
// Depth Camera FOV width = 38.0", height = 28.8"
// Video Camera FOV width = 43.0", height = 32.0"
// ArcTan = Opp/Adj. Opp = 1/2 width or height (to form right triangle)
// In Excell:  =(DEGREES(ATAN( (C8/B8) ))) * 2
// Now, adjust values as needed to compensate for mechanical position
#define KINECT_DEPTH_FOV_X	 93.10 	// 93.10 calculated.
#define KINECT_DEPTH_FOV_Y	 77.32 	// 77.32 calculated.
#define KINECT_VIDEO_FOV_X	100.13 	// 100.13 calculated.
#define KINECT_VIDEO_FOV_Y	 83.27	// 83.27 calculated.

// FINAL 3D OBJECT PARAMETERS
#define MIN_3D_OBJECT_LENGTH			15	// TenthInches - minimim size of object to detect
#define MIN_3D_OBJECT_WIDTH				10	// 
#define MAX_3D_OBJECT_LENGTH			70	// TenthInches - maximum size of object to detect
#define MAX_3D_OBJECT_WIDTH				70
#define MAX_3D_OBJECT_HEIGHT			50	// Tenthincess - catch things like a foot attached to a leg
#define MAX_3D_OBJECT_X_POSITION	   480	// TenthInches - max distance from center in front of robot (ignore objects too far to the side)
#define LEFT_WINDOW_MARGIN				12

// SLICE PARAMETERS
#define KINECT_SLICE_OBJECT_HEIGHT_MIN			 9	// TenthInches
#define KINECT_SLICE_OBJECT_WIDTH_MIN			 5	// TenthInches
#define KINECT_SLICE_OBJECT_HEIGHT_MAX		    (MAX_3D_OBJECT_HEIGHT * 2) // TenthInches - make this high, so 3D analysis function can see there is something to eliminate! (like a foot attached to a leg)
#define KINECT_SLICE_OBJECT_WIDTH_MAX		    (MAX_3D_OBJECT_LENGTH * 4)	// TenthInches - make this big, to allow post -processing to remove long shapes
#define KINECT_MAX_FLOOR_HEIGHT_TENTH_INCHES	20	// Tenthinches - prevent finding object sitting on top of steps, etc.

#define KINECT_WALL_DETECT_HEIGHT	    80	// TenthInches - Anything taller than this is considered a "wall" or unmovable object
#define MAP_NOISE_FLOOR_TENTHINCHES		10  // Tenthinches - ignore objects shorter than this when making 2D map

// Reserve top 1/4 scan lines to detect walls at the top of the frame (0 = top of frame)
#define KINECT_WALL_SCAN_ZONE		(m_FrameInfo->Height / 4)

//#define DISPLAY_MODE_OVERLAY	1
//#define DISPLAY_MODE_DEPTH		2
//#define DISPLAY_MODE_IMAGE		3
//#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

//#define MAX_DEPTH 10000

#define DEFAULT_KINECT_MOVE_TIME_LIMIT_TENTH_SECONDS	  100	// Default time for servo moves = 10 Seconds (100 Tenth Seconds)
//BOOL CALLBACK EnumWindowCallBack( HWND hwnd, LPARAM lParam );


///////////////////////////////////////////////////////////////////////////////
// Performance Tuning
//#define FRAME_SKIP_COUNT			1	// how many frames to skip between each processing (Lower number increases CPU load) 
//#define CAMERA_BLUR_SETTLE_TIME	   10	// After moving head, allow motion blur to settle before processing

//#define KINECT_FRAME_RATE_MS			500 //100 = 10fps, 200 = 5fps, 1000 = 1 fps
//#define KINECT_FRAME_MIN_SLEEP_TIME		 20 // don't allow Kinect thread to completely saturate the core
///////////////////////////////////////////////////////////////////////////////


#if (DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1)
	DEBUG_SLICE_ARRAY_T* g_pKinectDebugSliceArray;			// temp array for debugging slices
#endif


///////////////////////////////////////////////////////////////////////////////
//	Video Callback Functions for OpenCV window (used for debug)
///////////////////////////////////////////////////////////////////////////////


void OnDepthWindowMouseEvent( int event, int x, int y, int flags, void* param )
{
#if ( USE_OPEN_CV == 1 )
	IGNORE_UNUSED_PARAM (flags);

	CKinectModule* pKinectObject = (CKinectModule*)param;

    if( !pKinectObject )
        return;


    if( CV_EVENT_LBUTTONDOWN == event )
	{
		//ROBOT_LOG( TRUE,"OnDepthWindowMouseEvent: MouseDown at %d, %d\n", x, y)
		// FLIPPED VIDEO!
		int FlipX = pKinectObject->m_FrameInfo->Width - x;
		pKinectObject->m_OpenCVDepthMousePoint = cvPoint(FlipX, y);
        pKinectObject->m_OpenCVDepthMousePointSelected = TRUE;
	}

 /*   if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);
        
        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, m_pVideoFrame->width );
        selection.height = MIN( selection.height, m_pVideoFrame->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    case CV_EVENT_LBUTTONUP:
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
*/
#endif // ( USE_OPEN_CV == 1 )
}



//************************************************************************************************************************
//************************************************************************************************************************
// Name: KinectDepthThreadProc   MAIN LOOP FOR KINECT FRAME PROCESSING
// Desc: dedicated thread for grabbing Kinect Depth frames from shared memory
// This thread created by the CKinectModule class
//************************************************************************************************************************
//************************************************************************************************************************

DWORD WINAPI KinectDepthThreadProc( LPVOID lpParameter )
{
	CKinectModule* pKinectModule = (CKinectModule*)lpParameter;
	ROBOT_ASSERT( pKinectModule );

	IGNORE_UNUSED_PARAM (lpParameter);
	__itt_thread_set_name( "Kinect Thread" );

	CString MsgString;

#ifdef SPEECH_ROBOT_TYPE_LOKI
	if( ROBOT_TYPE != LOKI ) 
	{
		AfxMessageBox( _T("ALICE ERROR! Need to set RobotKinectViewer to NOT-Loki Speech (SpeechEnums.cs), and rebuild RobotKinectViewer") );
	}
#else
	if( ROBOT_TYPE == LOKI ) 
	{
		AfxMessageBox( _T("LOKI ERROR! Need to set RobotKinectViewer to Loki Speech (SpeechEnums.cs), and rebuild RobotKinectViewer") );
	}

#endif




	// Open Memory Mapped File
	if( SUBSYSTEM_DISABLED == g_KinectSubSystemStatus )
	{
		return 0; // no sense wasting cycles, the Kinect is disabled
	}

	if( pKinectModule->OpenMemoryMappedFile() )
	{
		g_KinectSubSystemStatus = SUBSYSTEM_CONNECTED;
	}
	else
	{
		//ROBOT_ASSERT(0);
		TRACE("/n/n");
		ROBOT_LOG( TRUE, "**************************************************************************" )
		ROBOT_LOG( TRUE, "******** FAILED TO OPEN KINECT SHARED MEMORY FROM C# APPLICATION! ********" )
		ROBOT_LOG( TRUE, "**************************************************************************\n" )

		g_KinectSubSystemStatus = SUBSYSTEM_FAILED;
		SpeakText( "Warning, Kenect controller initialization has failed");
		return 0; // no sense wasting cycles, the MMF did not init correctly!
	}


	#if (SHOW_XYZ_WINDOW)
		cvNamedWindow(CAMERA_WINDOW_NAME_KINECT_DEPTH, CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback( CAMERA_WINDOW_NAME_KINECT_DEPTH, OnDepthWindowMouseEvent, (void*)pKinectModule);
		//cvMoveWindow( CAMERA_WINDOW_NAME_KINECT_DEPTH, WindowPosX(m_pDepthDisplayFrame), PositionY );
	#endif
	g_Camera[KINECT_DEPTH].State = CAMERA_STATE_INITIALIZED;

	///////////////////////////////////////////////////////////////////////////////////
	// THREAD LOOP
	// gets frames when they are posted by the C# app into shared memory, when the event is signaled
	while( g_bRunThread && g_bRunKinectThread && (WAIT_OBJECT_0 == WaitForSingleObject(g_hKinectDepthReadyEvent, INFINITE)) )
	{
		//__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectThreadLoop);
		__itt_marker(pDomainKinectThread, __itt_null, pshKinectThreadLoop, __itt_marker_scope_task);

		// Get Depth Frame from Shared Memory and convert to 3D Cloud
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectGetDepthImage);
			pKinectModule->GetDepthImage(); 
		__itt_task_end(pDomainKinectThread);

		// Update 2D map with wall and object locations
		// TODO??? if( ROBOT_HAS_KINECT_SERVO )
		if( g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees <= KINECT_TILT_CENTER )
		{
			// Not in position to spot a human, so disable human tracking
			pKinectModule->m_FrameInfo->ControlFlags |= KinectControlFlag_HidePlayers; // flag for controlling the C# app

			// only enable if in position to see ahead
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csFindWallsAnd2dMaps);
				pKinectModule->FindWallsAnd2dMaps();
			__itt_task_end(pDomainKinectThread); // psh_csFindWallsAnd2dMaps

			// Update global summary of objects seen for object avoidance
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csUpdateKinectObjectSummary);
				pKinectModule->UpdateKinectObjectSummary();
			__itt_task_end(pDomainKinectThread); // psh_csUpdateKinectObjectSummary
		}
		else
		{
		}

		// Find objects on floor, if enabled by other modules
		if( ROBOT_TYPE == LOKI )
		{
			pKinectModule->FindObjectsOnFloor();
		}

		#if (SHOW_XYZ_WINDOW)
			pKinectModule->ShowDepthFrame( );		
		#endif

		if( ROBOT_TYPE == LOKI )
		{
			if( 0 != g_CurrentHumanTracked )
			{
				SendCommand( WM_ROBOT_CAMERA_LOOK_AT_PLAYER_CMD, (DWORD)(0), (DWORD)SERVO_SPEED_MED );
			}
		}
		//__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop

	}

	//////////////////////////////////////////////////////////////////////////////////////
	ROBOT_LOG( TRUE,"Kinect Thread exiting.\n")
	#if (SHOW_XYZ_WINDOW)
		cvDestroyWindow( CAMERA_WINDOW_NAME_KINECT_DEPTH );
	#endif

	return 0;
}



///////////////////////////////////////////////////////////////////////////////
//	MODULE: CKinectModule
//  NOTE: Creates its own video capture thread
///////////////////////////////////////////////////////////////////////////////


 CKinectModule::CKinectModule( CDriveControlModule *pDriveControlModule )
{
	ROBOT_LOG( TRUE, "Kinect constructor starting" )

	m_pDriveCtrl = pDriveControlModule;
	m_CurrentTask = KINECT_TASK_HUMAN_DETECTION; // by default, start looking for Humans
	m_TaskState = 1;
	m_TrackObjectX = 0;
	m_TrackObjectY = 0;
	gKinectDelayTimer = 0;
	m_FindObjectsOnFloorTrys = 0;
	m_nKinect3DObjectsFound = -1; // -1 == Result not ready
	m_KinectScanPosition = 0;
	m_FindCloseObjectsOnly = FALSE;

	m_FrameNumber = 0;
#if (SHOW_XYZ_WINDOW)
	m_pDebugZFrame = 0;
#endif
	m_DisplaySize.width = DEPTH_WINDOW_DISPLAY_SIZE_X;
	m_DisplaySize.height = DEPTH_WINDOW_DISPLAY_SIZE_Y;
	m_pKinectServoControl = new KinectServoControl;
	m_pHeadControl = new HeadControl();

	// Memory Mapped File
	m_hMapFile = INVALID_HANDLE_VALUE;
	m_pDepthInfoSharedMemory = NULL;	// Shared Buffer space LPCTSTR
	m_bKinectSharedMemoryOpened = FALSE;
	m_FrameInfo = NULL;

	m_ServoMoving = FALSE;
	m_BlurSettleTime = 0;

	// Processing variables
	m_ObjectTrackSize.width = 0;
	m_ObjectTrackSize.height = 0;
	m_ObjectTrackingCenter.x = 0;
	m_ObjectTrackingCenter.y = 0;

	m_OpenCVDepthMousePoint.x = 0;
	m_OpenCVDepthMousePoint.y = 0;
	m_KinectWindowMousePointSelected = FALSE;
	m_OpenCVDepthMousePointSelected = FALSE;
//	m_LastCameraMoveTime = GetTickCount();

/*
	// Detectors
//	m_pTrackObjectDetector	= NULL;
	// Detector Enabling flags
	m_VidCapProcessingEnabled = FALSE;
	m_ObjectTrackingEnabled = FALSE;
	m_ObjectTrackingActive = FALSE;
	m_ObjectToTrackFound = FALSE;
*/


	// Create the Point Cloud

	//__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
	//ROBOT_LOG( TRUE,"Kinect Init: EnterCriticalSection g_csKinectPointCloudLock\n")
	//EnterCriticalSection(&g_csKinectPointCloudLock);

	g_KinectPointCloud = new DEPTH_3D_CLOUD_T;
	g_KinectPointCloud->dwTimeOfSample = 0;
	g_KinectPointCloud->RobotLocation.x = 0.0;	// Location of robot at the time of depth snapshot
	g_KinectPointCloud->RobotLocation.y = 0.0;
	g_KinectPointCloud->CompassHeading = 0;		// Heading of robot at the time of depth snapshot
	for( int i=0; i < DEPTH_CAPTURE_SIZE_MAX_Y; i++ )
	{
		for( int j=0; j < DEPTH_CAPTURE_SIZE_MAX_X; j++ )
		{
			g_KinectPointCloud->Point3dArray[i][j].X = 0;
			g_KinectPointCloud->Point3dArray[i][j].Y = 0;
			g_KinectPointCloud->Point3dArray[i][j].Z = 0;
		}
	}
	for( int i=0; i < DEPTH_CAPTURE_SIZE_MAX_X; i++ )
	{
		g_KinectPointCloud->WallPoints[i].X = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
		g_KinectPointCloud->WallPoints[i].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
	}

	//LeaveCriticalSection(&g_csKinectPointCloudLock);
	//__itt_task_end(pDomainControlThread);
	//ROBOT_LOG( TRUE,"Kinect Init: LeaveCriticalSection g_csKinectPointCloudLock\n")

	m_pKinectTempObjects3D = new TEMP_OBJECT_3D_ARRAY_T;
	memset( m_pKinectTempObjects3D, 0x00, sizeof( TEMP_OBJECT_3D_ARRAY_T ) );


	// FOR DEBUG ONLY - KLUDGE
	#if DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1
		g_pKinectDebugSliceArray = new DEBUG_SLICE_ARRAY_T;
		memset( g_pKinectDebugSliceArray, 0x00, sizeof( DEBUG_SLICE_ARRAY_T ) );
	#endif

	//m_pDepthFrame = cvCreateImage( FrameSize, 8, 3 );


	// Create the Kinect video capture thread
	g_bRunKinectThread = TRUE; // When FALSE, tells thread to exit
	DWORD dwTempThreadId;
	g_hKinectThread = ::CreateThread( NULL, 0, KinectDepthThreadProc, (LPVOID)this, 0, &dwTempThreadId );
	ROBOT_LOG( TRUE, "Created Kinect Thread. ID = (0x%x)", dwTempThreadId )

	ROBOT_LOG( TRUE, "Kinect construstor complete" )

}

///////////////////////////////////////////////////////////////////////////////
CKinectModule::~CKinectModule()
{
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule.  Waiting for Kinect Thread to exit...")
	g_bRunKinectThread = FALSE; // Tell the thread to exit
	if( INVALID_HANDLE_VALUE != g_hKinectThread) 
	{
		WaitForSingleObject( g_hKinectThread, INFINITE );
		CloseHandle( g_hKinectThread );
	}
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule.  Kinect Thread exit complete.\n")
	#if DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1
		SAFE_DELETE( g_pKinectDebugSliceArray ); // TODO - WHY DOES THIS CORRUPT THE HEAP?
	#endif
	SAFE_DELETE( m_pKinectTempObjects3D );
	SAFE_DELETE( g_KinectPointCloud );
	SAFE_DELETE( m_pKinectServoControl );
	SAFE_DELETE( m_pHeadControl );

	// release Mapped File resources
	// - TODO-MUST DEBUGGING CRASH - REENALBE THIS!!!!!

	if( NULL != m_pDepthInfoSharedMemory )
	{
		UnmapViewOfFile(m_pDepthInfoSharedMemory);
	}
	/*
	if( INVALID_HANDLE_VALUE != m_hMapFile)
	{
		CloseHandle(m_hMapFile);
	}
	*/
	m_pDepthInfoSharedMemory = NULL;
	m_hMapFile = INVALID_HANDLE_VALUE;
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule done.\n")
}


//////////////////////////////////////////
// OpenMemoryMappedFile
// Initialize shared memory for getting frames from the C# code

BOOL CKinectModule::OpenMemoryMappedFile()
{
	// Create an event for synchronizing between the C# and C++ apps
	ROBOT_LOG( TRUE,  "Waiting for C# App to start...\n" )

	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hKinectDepthReadyEvent = CreateEvent ( NULL, bManualReset, bInitialState, "LokiRobotKinectDepthReadyEvent" );
	if ( !g_hKinectDepthReadyEvent ) 
	{ 
		ROBOT_LOG( TRUE,  "Kinect Event creation failed!:  LokiRobotKinectDepthReadyEvent\n" )
		return FALSE;
	}

	// Now wait for the event to be signaled by the C# process, indicating that it's ok to proceed
	const DWORD msTimeOut = 30000; // up to 30 seconds, to allow Kinect to power up!
	DWORD dwResult = WaitForSingleObject(g_hKinectDepthReadyEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		ROBOT_LOG( TRUE,  "Event Timed out or failed!:  LokiRobotKinectDepthReadyEvent, Error: %08X\n", dwResult )
		return FALSE;
	}
	ROBOT_LOG( TRUE,  "C# App start Success!\n" )

	TCHAR szKinectDepthInterfaceSharedFileName[]=TEXT(KINECT_DEPTH_INTERFACE_SHARED_FILE_NAME);

	m_hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		szKinectDepthInterfaceSharedFileName);	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == m_hMapFile) || (NULL == m_hMapFile)  )
	{ 
		ROBOT_LOG( TRUE,  "Could not open Kinect Depth file mapping object (%d).\n", GetLastError())
		return FALSE;
	}

	m_pDepthInfoSharedMemory = (int*)MapViewOfFile(m_hMapFile, // handle to map object (LPCTSTR)
		FILE_MAP_ALL_ACCESS,  // read/write permission
		0,                    
		0,                    
		(sizeof(KINECT_DATA_T)) );                   

	if (m_pDepthInfoSharedMemory == NULL) 
	{ 
		ROBOT_LOG( TRUE,  "Could not map view of file (%d).\n", GetLastError());
		CloseHandle(m_hMapFile);
		return FALSE;
	}

	m_bKinectSharedMemoryOpened = TRUE;
	ROBOT_DISPLAY( TRUE, "Kinect Depth Shared Memory File Opened Sucessfully!" )
	return TRUE; // Success!

}

void CKinectModule::ShowDepthFrame()
{	
#if ( USE_OPEN_CV == 1 )
	if( !m_pDebugZFrame )
	{
		return; // probably shutting down
	}
	// For DEBUG, draw the slices detected
	#if (DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1)
		ROBOT_ASSERT( g_pKinectDebugSliceArray );
		ROBOT_ASSERT( g_pKinectDebugSliceArray->nSlices < MAX_DEBUG_SLICES );
		for( int SliceItem =0; SliceItem < g_pKinectDebugSliceArray->nSlices; SliceItem++ )
		{
			cvLine(m_pDebugZFrame, g_pKinectDebugSliceArray->Slice[SliceItem].Pt1,  
				g_pKinectDebugSliceArray->Slice[SliceItem].Pt2, CV_RGB(255,255,0), 1, 4, 0);
		}
	#endif
	
	// Draw bounding boxes around any objects detected
	if( g_pKinectObjects3D->nObjectsDetected > 0)
	{
		//int i = 0;
		for( int i = 0; i < g_pKinectObjects3D->nObjectsDetected; i++ )
		{
			CvPoint pt1 = { (g_pKinectObjects3D->Object[i].LeftPixel), (g_pKinectObjects3D->Object[i].StartScanLine) };
			CvPoint pt2 = { (g_pKinectObjects3D->Object[i].RightPixel), (g_pKinectObjects3D->Object[i].EndScanLine) };
			cvRectangle(m_pDebugZFrame, pt1, pt2, CV_RGB(255,0,255), 1 );
		}
	}

	// KINECT_WALL_SCAN_ZONE
	CvPoint pt1, pt2;
	pt1.y = KINECT_WALL_SCAN_ZONE;
	pt1.x = 0;
	pt2.y = KINECT_WALL_SCAN_ZONE;
	pt2.x = m_pDebugZFrame->width;
	cvLine(m_pDebugZFrame, pt1, pt2, CV_RGB(0,127,0), 1, 4, 0);

	// Draw Cross-Hair lines
	CvPoint FrameCenter;
	FrameCenter.x = m_pDebugZFrame->width /2;
	FrameCenter.y = m_pDebugZFrame->height /2;

	// Vertical line
	pt1.x = FrameCenter.x;
	pt1.y = 0;
	pt2.x = FrameCenter.x;
	pt2.y = m_pDebugZFrame->height;
	cvLine(m_pDebugZFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);

	// Horizontal line
	pt1.y = FrameCenter.y;
	pt1.x = 0;
	pt2.y = FrameCenter.y;
	pt2.x = m_pDebugZFrame->width;
	cvLine(m_pDebugZFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);


/*
// Scale as needed
	if( m_CaptureSize.width != m_DisplaySize.width ) //USE THIS: m_FrameInfo->Height
	{
		// Scale down 640x480 to fit on display (easier on remote desktop too)
		cvResize( m_pDebugZFrame, m_pDebugZFrame  ); // CV_INTER_CUBIC
		//	cvResizeWindow( CAMERA_WINDOW_NAME_LEFT, 320, 240 );
	}
	if( g_Camera[KINECT_DEPTH].Flip )
	{
		// Flip video horizontally
		cvFlip( m_pDebugZFrame, 0, 1 );
	}
*/

	// Flip video horizontally
	cvFlip( m_pDebugZFrame, 0, 1 );
	cvShowImage( CAMERA_WINDOW_NAME_KINECT_DEPTH, m_pDebugZFrame );
	// KLUDGE!  REQURED DUE TO BUG IN OPENCV.		
	char c = (char)cvWaitKey(1); // NEED cvWaitKey() OR VIDEO DOES NOT DISPLAY!
	if( c == 27 )	ROBOT_LOG( TRUE,"ESC KEY Pressed!\n")

#endif // ( USE_OPEN_CV == 1 )
}


//////////////////////////////////////////
// GetDepthImage
// Get depth data from Shared Memory and convert to 3D point cloud

void CKinectModule::GetDepthImage()
{
	// TODO? Lock the 3DPointCloud

	// get data from Shared Memory
	if( NULL == m_pDepthInfoSharedMemory )
	{
		m_FrameInfo = NULL;
		return;
	}
	m_FrameInfo = (KINECT_DATA_T*)m_pDepthInfoSharedMemory;
	m_FrameInfo->ControlFlags = KinectControlFlag_None; // init flags for controlling the C# app.  These will be overridden as needed.

	#if DEBUG_SHARED_MEMORY == 1
		ROBOT_LOG( TRUE,  "m_FrameInfo  No=%4d, H=%d, W=%d, MaxDepth=%d\n", 
			m_FrameInfo->FrameNumber, m_FrameInfo->Height, m_FrameInfo->Width, m_FrameInfo->tooFarDepth )
	#endif

	#if (SHOW_XYZ_WINDOW)
		if( 0 == m_pDebugZFrame )
		{
			// first frame
			CvSize FrameSize; FrameSize.height = m_FrameInfo->Height;	FrameSize.width = m_FrameInfo->Width;
			m_pDebugZFrame = cvCreateImage( FrameSize, IPL_DEPTH_8U, 3 );
			if( !m_pDebugZFrame ) ROBOT_ASSERT(0);
			m_pDebugZFrame->origin = 0;
		}
	#endif

	// Convert depth data and map to Depth frame
	// Starts at top of frame (y=0) and scans to bottom of frame (y=FrameInfo->Height)
	// So, top of frame is farthest from the robot!

	const int tooNearDepth = 0;
	const int unknownDepth = -1;
	//int FrameWidth = m_FrameInfo->Width;
	//int FrameHeight = m_FrameInfo->Height;
	int tooFarDepth = m_FrameInfo->tooFarDepth;
	int  MouseX = 0;
	int  MouseY = 0;
//	POINT2D_T Players[KINECT_MAX_HUMANS_TO_TRACK];
//	int nPlayersFound = 0;
	m_KinectWindowMousePointSelected = FALSE;
	if( 1 == m_FrameInfo->MouseDown )
	{
		m_KinectWindowMousePointSelected = TRUE;
		MouseX = m_FrameInfo->MouseX;
		MouseY = m_FrameInfo->MouseY;
		m_FrameInfo->MouseDown = 0; // Reset the shared memory flag for next time
	}

	// Calculate degrees per pixel for the image
	double DegreeIncrementX = KINECT_DEPTH_FOV_X / (double)m_FrameInfo->Width;
	double DegreeIncrementY = KINECT_DEPTH_FOV_Y / (double)m_FrameInfo->Height;

	// Get Kinect Tilt angle, and position offset caused by tilt
	double KinectTiltTenthDegrees =	0;
	// TODO?  if( ROBOT_HAS_KINECT_SERVO )
	{
		KinectTiltTenthDegrees = g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees;
	}
	// for debug on a PC
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		KinectTiltTenthDegrees = -300; // simulate as if Arduino pointing to ground
	}

	// Handle mouse in OpenCV window (for Z debug)
	if( m_OpenCVDepthMousePointSelected )
	{
		// Scale as needed.  Mouse coordinates are always 640x480 after scaling ??
		UINT Ratio = 1; // TODO? m_CaptureSize.width / m_DisplaySize.width; USE THIS: m_FrameInfo->Height
		/*if( g_Camera[KINECT_DEPTH].Flip )
		{
			// Video flipped horizontally
			MouseX = ( (m_DisplaySize.width -1) - m_DepthMousePoint.x ) * Ratio;
		}
		else
		*/
		{
			MouseX = m_OpenCVDepthMousePoint.x * Ratio;
		}
		MouseY = m_OpenCVDepthMousePoint.y * Ratio;
	}



	if( NULL == g_KinectPointCloud )
	{
		ROBOT_ASSERT(0); // Not initilized!
		return;
	}
	g_KinectPointCloud->FrameSizeX = m_FrameInfo->Width;
	g_KinectPointCloud->FrameSizeY = m_FrameInfo->Height;

		
	// Keep a local array of humans to track in this frame (will update the global at the end)
	KINECT_HUMAN_FINDING_T HumanLocationFinding[KINECT_MAX_HUMANS_TO_TRACK]; // Location of humans being tracked
	memset( HumanLocationFinding, 0x00, (sizeof(KINECT_HUMAN_FINDING_T) * KINECT_MAX_HUMANS_TO_TRACK) );

	KINECT_HUMAN_TRACKING_T HumanLocationTracking[KINECT_MAX_HUMANS_TO_TRACK]; // Location of humans being tracked
	memset( HumanLocationTracking, 0x00, (sizeof(KINECT_HUMAN_TRACKING_T) * KINECT_MAX_HUMANS_TO_TRACK) );

	short* pDepthBuffer = m_FrameInfo->DepthData;

// TEST DEBUG CODE
			m_FrameInfo->ControlFlags |= KinectControlFlag_DisplayBoundingBox; // Draw a bounding box
			m_FrameInfo->BoundingBoxTop = 100;
			m_FrameInfo->BoundingBoxBottom = 150;
			m_FrameInfo->BoundingBoxLeft = 100;
			m_FrameInfo->BoundingBoxRight = 150;


	// From Microsoft.Kinect:
	const int PlayerIndexBitmask = 7;
	const int PlayerIndexBitmaskWidth = 3;
	int ClosestPlayerDistance = KINECT_RANGE_TENTH_INCHES_MAX;
	int ClosestPlayer = 0;
	BOOL CurrentHumanTrackedFound = FALSE;
	int LowestZValue = KINECT_RANGE_TENTH_INCHES_MAX; // track the lowest value to normalize floor


	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectFrameLoop);
	for( int y = 0; y < m_FrameInfo->Height; y++ )
	{
		LowestZValue = KINECT_RANGE_TENTH_INCHES_MAX; // track the lowest value in each scan line to normalize floor
		for( int x = 0; x < m_FrameInfo->Width; x++ )
		{
            int player = (int)(*pDepthBuffer & PlayerIndexBitmask);
            int DepthValueMM = (int)(*pDepthBuffer >> PlayerIndexBitmaskWidth); // Depth value in millimeters
			pDepthBuffer++;

			#if DEBUG_SHARED_MEMORY == 1
				if( (x < 32) && (y == 0 ) )
				{
					TRACE("%d, ", DepthValueMM);
				}

			#endif

			//////////////////////////////////////////////////////////////////////////////////////////
			// Update the 3D point cloud
			// Calculate X,Y,Z based upon tilt of Kinect and FOV of the Kinect Camera
			// x, y = pixel location
			int X=KINECT_RANGE_TENTH_INCHES_MAX,Y=KINECT_RANGE_TENTH_INCHES_MAX,Z=KINECT_RANGE_TENTH_INCHES_MAX;
			double TempX=0,TempY=0,TempZ=0,m=0,n=0,q=0,u=0, RangeTenthInches = 0;

			if( m_KinectWindowMousePointSelected || m_OpenCVDepthMousePointSelected )
			{
				if( (x == MouseX) && (y == MouseY) )
				{
					ROBOT_LOG( TRUE,"MOUSE DOWN AT %d, %d ", MouseX, MouseY )
				}
			}

			// TRAP BAD VALUES HERE!!! (too near, too far, etc.)
			if( (DepthValueMM >= KINECT_MM_MAX) || (DepthValueMM <= 0) || (tooFarDepth == DepthValueMM) )
			{
				//ROBOT_LOG( TRUE,"Kinect Read Zero: EnterCriticalSection g_csKinectPointCloudLock\n")
				//__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
				EnterCriticalSection(&g_csKinectPointCloudLock);
					g_KinectPointCloud->Point3dArray[y][x].X = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
					g_KinectPointCloud->Point3dArray[y][x].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
					g_KinectPointCloud->Point3dArray[y][x].Z = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
				LeaveCriticalSection(&g_csKinectPointCloudLock);
				//__itt_task_end(pDomainKinectThread);
				//ROBOT_LOG( TRUE,"Kinect Read Zero: LeaveCriticalSection g_csKinectPointCloudLock\n")
			}
			else
			{
				RangeTenthInches = ((double)DepthValueMM / 2.540) ;	

				double StepX = (double)x - ((double)m_FrameInfo->Width/2.0);	// goes from (-320 to + 320)
				double StepAngleX = DegreeIncrementX / 1.5 * StepX; // goes from (-46 to + 46 degrees)

				double StepY = ( ((double)m_FrameInfo->Height/2.0) - (double)y );	// goes from (+240 to -240)
				//double StepAngleY =  DegreeIncrementY * StepY; // goes from (-38.6 to + 38.6 degrees)
				double StepAngleY =  (KinectTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.6 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
				double StepAngleYZ =  (KinectTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.8 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
				//double StepAngleY =  (KinectTiltTenthDegrees/10.0); // goes from (-38.6 to + 38.6 degrees) + Tilt angle

				// Get x,y,z from Kinect's perspective. zero = straight forward facing.  Down = negative
				// Convert from Spherical Coordinates to X,Y,Z coordinates for each point
				TempX = RangeTenthInches * sin(DEGREES_TO_RADIANS*(StepAngleX) );
				TempY = RangeTenthInches * cos(DEGREES_TO_RADIANS*StepAngleY);
				TempZ = RangeTenthInches * sin(DEGREES_TO_RADIANS*StepAngleYZ * 1.1);

				// Now, translate origin from Kinect position to center of robot at floor.
				X = (int)(-TempX);	 // For Loki, Positive X is the Right side of the robot
				Y = (int)(TempY - (double)KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES);
				Z = (int)(TempZ + (double)KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES);

/*				if(Z < -60)
				{
					ROBOT_LOG( TRUE,"ERROR! Z value < -6 inches!");				
					ROBOT_LOG( TRUE,"AT: Y=%d, X=%d:  Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x, 
						(double)(Y)/10.0,
						(double)(X)/10.0,
						(double)(Z)/10.0  )
				}
*/
				if( Z < LowestZValue )
				{
					LowestZValue = Z;  // Keep track of the lowest value for floor
				}

				// Save the data in TenthInches
				//ROBOT_LOG( TRUE,"Kinect Set: EnterCriticalSection g_csKinectPointCloudLock\n")
				//__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
				if( NULL == g_KinectPointCloud ) return; // TODO Kludge for shutdown crash.  Need better solution.
				EnterCriticalSection(&g_csKinectPointCloudLock);
					g_KinectPointCloud->Point3dArray[y][x].X = (int)X;
					g_KinectPointCloud->Point3dArray[y][x].Y = (int)Y;
					g_KinectPointCloud->Point3dArray[y][x].Z = (int)Z;
				LeaveCriticalSection(&g_csKinectPointCloudLock);
				//__itt_task_end(pDomainKinectThread);
				//ROBOT_LOG( TRUE,"Kinect Set: LeaveCriticalSection g_csKinectPointCloudLock\n")




				#if ( TRACK_TOP_OF_USERS_HEAD == 1 )
					// Keep track of the highest point on the player detected in this frame (used with range data to determine player's height!)
					// This will be combined with other data later, when we know how many unique humans there are (we don't know at this point)
					if( (0 != player) && (KinectTiltTenthDegrees > KINECT_TILT_LOOKING_AT_FLOOR) )
					{
						// This pixel has player info
						int StepAngleTenthDegreesX = (int)(StepAngleX * -10.0); // Robot Right is positive
						if( !HumanLocationFinding[player].Found )
						{
							// first pixel found for this player. ASSUME it is the top of the their head (could be a hand over head, etc.)
							HumanLocationFinding[player].Found = TRUE;
							HumanLocationFinding[player].Pixel.Y = 640 - y; // y=0 is top of screen
							HumanLocationFinding[player].Pixel.X = x;
							HumanLocationFinding[player].HeadLocation_LeftX = (int)X;	 
							HumanLocationFinding[player].HeadLocation_RightX = (int)X;
							HumanLocationFinding[player].HeadLocationY = (int)Y;		// Note these come from PointCloud calculation
							HumanLocationFinding[player].HeadLocationZ = (int)Z;		// So values are in TENTH INCHES!
							HumanLocationFinding[player].AngleTenthDegrees_LeftX = StepAngleTenthDegreesX;
							HumanLocationFinding[player].AngleTenthDegrees_RightX = StepAngleTenthDegreesX;
							HumanLocationFinding[player].AngleTenthDegreesY = (int)(StepAngleY * 10.0);
							HumanLocationFinding[player].ScanLine = x;
							HumanLocationFinding[player].TopOfHeadVisable = FALSE; // default, overridden below

							if( y > 5 ) // scan lines
							{
								// TODO-MUST WARNING - I think there is a bug here.  Y=distance, Z=height!!!
								// Compensate approx from top of head to center of face
								// Only do if not top of screen (where person's head is off screen)
								HumanLocationFinding[player].HeadLocationY -= HUMAN_FACE_TO_TOP_OF_HEAD_OFFSET;		//TenthInches
								HumanLocationFinding[player].AngleTenthDegreesY -= 150; // TenthDegrees
								HumanLocationFinding[player].TopOfHeadVisable = TRUE;
								//ROBOT_LOG( TRUE, "Player Head: x=%d, y=%d, z=%d\n",  (int)X, (int)Y, (int)Z );
							}

							// Don't attempt to track players if looking at the ground! 
							// (sometimes Kinect gets confused and thin
							if( Y < ClosestPlayerDistance )
							{
								ClosestPlayer = (int)Y;
								ClosestPlayer = player;
							}
							if( player == g_CurrentHumanTracked )
							{
								CurrentHumanTrackedFound = TRUE;
							}
							//ROBOT_LOG( TRUE,"PLAYER %2d at: X=%3d, Y=%3d:   Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", player, HumanLocationFinding[player].AngleTenthDegrees.X/10, HumanLocationFinding[player].AngleTenthDegrees.Y/10, Y/10.0, X/10.0, Z/10.0  )
							//break;
						}
						else // player found previously
						{
							// Find right and left limits of the player's head
							if( x == HumanLocationFinding[player].ScanLine ) // we will find the center of player on the first line found
							{
								if( StepAngleTenthDegreesX < HumanLocationFinding[player].AngleTenthDegrees_LeftX )
								{
									HumanLocationFinding[player].AngleTenthDegrees_LeftX = StepAngleTenthDegreesX;
									HumanLocationFinding[player].HeadLocation_LeftX = X;
								}
								else if( StepAngleTenthDegreesX > HumanLocationFinding[player].AngleTenthDegrees_RightX )
								{
									HumanLocationFinding[player].AngleTenthDegrees_RightX = StepAngleTenthDegreesX;
									HumanLocationFinding[player].HeadLocation_RightX = X;
								}
							}

						}
					}
				#endif

			} 

			// Check Mouse input
			if( m_KinectWindowMousePointSelected )
			{
				if( (x == MouseX) && (y == MouseY) )
				{
					ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:   Range=%4.1f, Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x,(RangeTenthInches/10.0), Y/10.0, X/10.0, Z/10.0  )
					m_KinectWindowMousePointSelected = FALSE;
				}
			}

			#if (SHOW_XYZ_WINDOW)
				// update the Z image bitmap
				if( 0 != m_pDebugZFrame )
				{
					RGBQUAD ColorPixel = ZValueToColor( Z );
					// TODO - see if we can remove this HACK
					CvScalar	Pixel;
					Pixel.val[0] = ColorPixel.rgbBlue;		// Blue
					Pixel.val[1] = ColorPixel.rgbGreen;		// Green
					Pixel.val[2] = ColorPixel.rgbRed;		// Red
					cvSet2D( m_pDebugZFrame, y, x, Pixel );
				}

/*				if( m_OpenCVDepthMousePointSelected )
				{
					if( (x == MouseX) && (y == MouseY) )
					{
						ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:   Range=%4.1f, Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x,(RangeTenthInches/10.0), Y/10.0, X/10.0, Z/10.0  )
						m_OpenCVDepthMousePointSelected = FALSE;
					}
				}
*/

			#endif

		} // for x

		///////////////////////////////////////
		// Normalize Floor values
		if( KinectTiltTenthDegrees <= KINECT_TILT_LOOKING_AT_FLOOR ) // only enable if in position to see ahead
		{			
			if(LowestZValue < -70)
			{
				//ROBOT_LOG( TRUE,"ERROR! Lowest Z value < -7 inches! (%d)\n", LowestZValue);				
			}
			else if(LowestZValue < 60) // only do lines where floor is seen
			{
				EnterCriticalSection(&g_csKinectPointCloudLock);
				for( int x = 0; x < m_FrameInfo->Width; x++ )
				{
					int before = g_KinectPointCloud->Point3dArray[y][x].Z;
					if( KINECT_RANGE_TENTH_INCHES_MAX != g_KinectPointCloud->Point3dArray[y][x].Z )
					{
						g_KinectPointCloud->Point3dArray[y][x].Z = g_KinectPointCloud->Point3dArray[y][x].Z - LowestZValue;
					}
					int after = g_KinectPointCloud->Point3dArray[y][x].Z;
					if( m_OpenCVDepthMousePointSelected )
					{
						if( (x == MouseX) && (y == MouseY) )
						{
							ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:  Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x, 
								(double)(g_KinectPointCloud->Point3dArray[y][x].Y)/10.0,
								(double)(g_KinectPointCloud->Point3dArray[y][x].X)/10.0,
								(double)(g_KinectPointCloud->Point3dArray[y][x].Z)/10.0  )
							ROBOT_LOG( TRUE,"   LowestZValue = %d, beforeZ = %d, afterZ = %d\n", LowestZValue, before, after )
							m_OpenCVDepthMousePointSelected = FALSE;
						}
					}

				}
				LeaveCriticalSection(&g_csKinectPointCloudLock);
			}
		}

	} // for y
	__itt_task_end(pDomainKinectThread); // frameloop


	#if DEBUG_SHARED_MEMORY == 1
		TRACE("Last = %d\n", lastValue );
	#endif

	if( KinectTiltTenthDegrees <= KINECT_TILT_LOOKING_AT_FLOOR ) // only enable if in position to see a player
	{
		CurrentHumanTrackedFound = FALSE;
		g_CurrentHumanTracked = 0;
	}
	else
	{
		// Update global Human Tracking
		if( !CurrentHumanTrackedFound )
		{
			// we were tracking someone, but lost them, so track the closest player found (if any)
			g_CurrentHumanTracked = ClosestPlayer; // this will be zero if no players found
		}

		// Copy Human Location info to global
		for( int player=0; player<KINECT_MAX_HUMANS_TO_TRACK; player++ )
		{
			g_HumanLocationTracking[player].Found = FALSE;
			if( HumanLocationFinding[player].Found )
			{	
				// find center of head
				int AngleTenthDegreesX = HumanLocationFinding[player].AngleTenthDegrees_LeftX +
					( (HumanLocationFinding[player].AngleTenthDegrees_RightX - HumanLocationFinding[player].AngleTenthDegrees_LeftX) /2 ); // half the distance between the points

				int HeadLocationTenthInchesX = HumanLocationFinding[player].HeadLocation_LeftX +
					( (HumanLocationFinding[player].HeadLocation_RightX - HumanLocationFinding[player].HeadLocation_LeftX) /2 ); // half the distance between the points

				g_HumanLocationTracking[player].HeadLocation.X = HeadLocationTenthInchesX;
				g_HumanLocationTracking[player].HeadLocation.Y = HumanLocationFinding[player].HeadLocationY;	 
				g_HumanLocationTracking[player].HeadLocation.Z = HumanLocationFinding[player].HeadLocationZ;	 
				g_HumanLocationTracking[player].AngleTenthDegrees.X = AngleTenthDegreesX;
				g_HumanLocationTracking[player].AngleTenthDegrees.Y = HumanLocationFinding[player].AngleTenthDegreesY;
			}
			g_HumanLocationTracking[player].Found = HumanLocationFinding[player].Found; // do last, so we don't need a lock
		}
	}

//	__itt_task_end(pDomainKinectThread);
}

RGBQUAD CKinectModule::ZValueToColor( int Zvalue ) // for debugging Z Depth on floor
{
	RGBQUAD Pixel;
	if( KINECT_RANGE_TENTH_INCHES_MAX == Zvalue )
	{
		Pixel.rgbRed = 0;
		Pixel.rgbGreen = 0;
		Pixel.rgbBlue = 0;
		return Pixel;
	}

	// Values must be scaled between 0 to 1023
	const int MaxNegativeValueTenthInches = 60; // Max negative Z value in Tenthinches for debug
	const int MaxObjHeightTenthInches = 120; 
	const double ScaleFactor = 1024 / (MaxObjHeightTenthInches+MaxNegativeValueTenthInches);
	UINT red =0; UINT blue =0; UINT green =0; 
	signed int ScaledValue = (signed int) ((double)(Zvalue+MaxNegativeValueTenthInches) * ScaleFactor); // scale, and subtract out min range (47 ScaledValue)
	if( ScaledValue < 0) 
		ScaledValue = 0;
	if( ScaledValue > 1023) 
		ScaledValue = 1023;

	UINT modu = ScaledValue % 128;

	if( ScaledValue < 128.0 )
	{
		green = 256-modu;	// Ramp down
	}
	else if( ScaledValue < 256 )
	{
		green = 128-modu;	// Ramp down
		blue = modu;		// Ramp up
	}
	else if( ScaledValue < 384 )
	{
		blue = modu+128;	// Ramp up
	}
	else if( ScaledValue < 512 )
	{
		red = modu;			// Ramp up
		blue = 256-modu;	// Ramp down
	}
	else if( ScaledValue < 640 )
	{
		red = modu+128;		// Ramp up
		blue = 128-modu;	// Ramp down
	}
	else if( ScaledValue < 768 )
	{
		red = 256-modu;		// Ramp down
	}
	else if( ScaledValue < 896 )
	{
		red = 128-modu;		// Ramp down
	}
	else if( ScaledValue < 964 )
	{
		red = 128-modu;		// Ramp down
		green = 128-modu;	// Ramp down
		blue = 128-modu;	// Ramp down
	}
	else 
	{
		red = 60;	
		green = 60;	
		blue = 60;
	}

	Pixel.rgbRed = red;
	Pixel.rgbGreen = green;
	Pixel.rgbBlue = blue;

    return Pixel;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CKinectModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{

	CString MsgString;
	//BYTE nDistHigh, nDistLow;
	switch( uMsg )
	{
		// Process Kinect commands from the GUI or command threads

		case WM_ROBOT_KINECT_CANCEL_CMD:
		{
			// cancel any current action. Called for example, when user says "stop".
			// go back to looking for Humans
			m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
			m_TaskState = 1;
			return;
		}

		case WM_ROBOT_KINECT_SEARCH_FLOOR_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Start state machine for searching the floor
			// wParam indicates close search only or full search of floor
			if( 0 == wParam )
			{
				ROBOT_LOG( TRUE,"KINECT_SEARCH_FLOOR_CMD: Searching for Near and Far objects\n")
				m_FindCloseObjectsOnly = FALSE;
			}
			else
			{				
				ROBOT_LOG( TRUE,"KINECT_SEARCH_FLOOR_CMD: Searching for Close objects only\n")
				m_FindCloseObjectsOnly = TRUE;
			}
			m_CurrentTask = KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT;
			m_TaskState = 1;	// go to first state
			return;
		}

		case WM_ROBOT_KINECT_TRACK_OBJECT_CMD:
		{
			g_bCmdRecognized = TRUE;
			if( 0 == wParam )
			{
				// Stop Tracking, and go back to looking for Humans
				m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
				m_TaskState = 1;
			}
			else
			{
				// ASSUME (for now), that we want to track the closest object that was previously found
				// Start state machine for tracking object
				m_CurrentTask = KINECT_TASK_TRACK_CLOSEST_OBJECT;
				m_TaskState = 1;	// go to first state
			}
			return;
		}

		// ============================================================================================================
		case WM_ROBOT_SENSOR_STATUS_READY:
		case WM_ROBOT_SERVO_STATUS_READY:
		// ============================================================================================================
		{
			g_bCmdRecognized = TRUE;
			// This gets called each time the sensors are updated.


			/////////////////////////// WM_ROBOT_SENSOR_STATUS_READY //////////////////////////////////////////////////////
			// Check status of Kinect Tasks
			if( 0 != gKinectDelayTimer )
			{
				//ROBOT_LOG( TRUE,"WAITING FOR gKinectDelayTimer = %d\n", gKinectDelayTimer)
				break;
			}

			switch( m_CurrentTask )
			{
				case TASK_NONE:
				{
					break;	// Nothing to do
				}
				case KINECT_TASK_HUMAN_DETECTION:
				{
					switch( m_TaskState )
					{
						case 0:
						{
							///m_TaskState++;	// should always be looking for humans, if nothing else to do
							break;
						}
						case 1:	// Get Kinect into positon to spot humans NOTE:DISABLED FOR NOW< PROBLEM WITH PICKUP
						{	
							//ROBOT_LOG( TRUE,"KINECT_TASK_HUMAN_DETECTION START: Moving Kinect to position %d\n", KINECT_HUMAN_DETECT_START_POSITION)
							//m_pKinectServoControl->SetTiltPosition( <<KINECT_TILT_OWNER_TRACK_OBJECT??>, KINECT_HUMAN_DETECT_START_POSITION, SERVO_SPEED_MED );
							m_TaskState = 3; /// m_TaskState++
							break;
						}
						case 2:	// Wait for Servo to reach commanded position
						{	
							//ROBOT_LOG( TRUE, "DEBUG KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT : CheckServoPosition ")
							//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - Arduino\n") 
							//else if( WM_ROBOT_SERVO_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - SERVO\n")

							/*
							int ServoStatus = m_pKinectServoControl->CheckServoPosition(FALSE);
							if ( KINECT_SERVO_SUCCESS == ServoStatus )
							{
								// Kinect in position, go to next state
								m_TaskState++;	
							}
							else if( KINECT_SERVO_TIMED_OUT == ServoStatus )
							{
								ROBOT_LOG( TRUE,"ERROR: KINECT SERVO TIME OUT! \n\n")
								gKinectDelayTimer = 10; // tenth-seconds - avoid tight loop on error
								m_TaskState = 1; // Try again
							}
							*/
							m_TaskState++; // TEMP FOR DEBUG
							break;
						}
						case 3:	// See if we are positioned well to spot humans
						{	
							// If Human detected, see if too close or too far away, and tilt sensor as needed

							__itt_task_begin(pDomainModuleThread, __itt_null, __itt_null, psh_csKinectHumanTrackingLock);
							EnterCriticalSection(&g_csKinectHumanTrackingLock);
							int nHumanFound = 0;

							#if ( (USE_KINECT_TO_LOOKAT_HUMAN == 1) || (TILT_KINECT_TO_TRACK_CLOSE_HUMANS == 1) )
							for( int nHuman=0; nHuman < KINECT_MAX_HUMANS_TO_TRACK; nHuman++ )
							{
								#if ( USE_KINECT_TO_LOOKAT_HUMAN == 1 ) // if enabled, causes robot head to track human head position
									if( (0 == nHumanFound) && (0 != g_HumanLocationTracking[nHuman-1].HeadLocationZ) )
									{
										nHumanFound = nHuman; // get the first one found
									}
								#endif
								#if ( TILT_KINECT_TO_TRACK_CLOSE_HUMANS == 1 ) // If enabled, causes the Kinect sensor to Tilt to track person's head
									// Acts upon the first human found in the list that is outside of acceptable range
									if( g_HumanLocationTracking[nHuman-1].Pixel.Y > 635 ) // ASSUME 640x480 capture!
									{
										// Human head too close to top of screen, move Kinect up (tenthdegrees)
										int CurrentPos = m_pKinectServoControl->GetTiltPosition();
										//int DeltaPos = (640 - g_HumanLocationTracking[nHuman-1].Pixel.Y) / 2; // divisor is rough ratio to move tune as needed
										int DeltaPos = 20; // minimum move amount
										int NewPos = CurrentPos + DeltaPos;
										m_pKinectServoControl->SetTiltPosition( <<KINECT_TILT_OWNER_TRACK_OBJECT?>>, NewPos );
										
										ROBOT_LOG( TRUE,"KINECT Human Too Tall: Moving Kinect Up to position %d (TenthDegrees)\n", NewPos)
										gKinectDelayTimer = 10; // tenth-seconds - Don't jerk around
										break;
									}
									else if( (g_HumanLocationTracking[nHuman-1].Pixel.Y > 0) && (g_HumanLocationTracking[nHuman-1].Pixel.Y < 500) ) // ASSUME 640x480 capture!
									{
										// Human head too far down from top of screen, move Kinect down (tenthdegrees)
										int CurrentPos = m_pKinectServoControl->GetTiltPosition();
										int DeltaPos = 20; // (350 - g_HumanLocationTracking[nHuman-1].Pixel.Y) / 2; // divisor is rough ratio to move tune as needed
										int NewPos = CurrentPos - DeltaPos;
										m_pKinectServoControl->SetTiltPosition( <<KINECT_TILT_OWNER_TRACK_OBJECT?>>, NewPos );
										ROBOT_LOG( TRUE,"KINECT Human Too Short: Moving Kinect Down to position %d (TenthDegrees)\n", NewPos)
										gKinectDelayTimer = 10; // tenth-seconds - Don't jerk around
										break;
									}
								#endif
							}
							#endif

							LeaveCriticalSection(&g_csKinectHumanTrackingLock);
							__itt_task_end(pDomainModuleThread);

							// Do this outside of the CriticalSection, as it might be time intensive?
							#if ( USE_KINECT_TO_LOOKAT_HUMAN == 1 ) // if enabled, causes robot head to track human head position
								if( 0 != nHumanFound )
								{
									// Point Eyes at human for realism.
									m_pHeadControl->SetHeadSpeed( HEAD_OWNER_KINECT_HUMAN, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
									m_pHeadControl->LookAtXYZ( HEAD_OWNER_KINECT_HUMAN, g_HumanLocationTracking[nHumanFound-1].HeadLocationX, 
										g_HumanLocationTracking[nHumanFound-1].HeadLocationY, (g_HumanLocationTracking[nHumanFound-1].HeadLocationZ - 60) ); //  Look at Human eyes, not top of head
									gKinectDelayTimer = 5; // tenth-seconds - Don't jerk around (or overload the CPU)
								}
							#endif


							break; // Just stay in this state
						}
						default: ROBOT_ASSERT(0);
					}
					break;
				}
				case KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT:
				{	
					switch( m_TaskState )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// begin with scan close to robot
						{	
							//ROBOT_LOG( TRUE,"KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT: Moving Kinect to position %d\n", m_KinectScanPosition)
							#if DEBUG_KINECT_FLOOR_OBJECT_DETECTION	== 1
								m_KinectScanPosition = 2;  //******** FAR SCAN DEBUG ONLY !!!
							#else		
								m_KinectScanPosition = KINECT_SERVO_POSITION_CLOSE_SCAN;
							#endif
							ROBOT_LOG( TRUE,"KINECT Scan Position %d\n", m_KinectScanPosition)
							m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, KINECT_FLOOR_SCAN_POSITION[m_KinectScanPosition] );
							m_TaskState++;
							break;
						}
						case 2:	// Wait for Servo to reach commanded position
						{	
							//ROBOT_LOG( TRUE, "DEBUG KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT : CheckServoPosition ")
							//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - Arduino\n")
							//else if( WM_ROBOT_SERVO_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - SERVO\n")

							int ServoStatus = m_pKinectServoControl->CheckServoPosition(FALSE);
							if ( KINECT_SERVO_SUCCESS == ServoStatus )
							{
								// Kinect in position, go to next state (Kinect should have done a scan by then)
								gKinectDelayTimer = 5; // tenth-seconds - Wait for image to settle before checking 3D
								m_TaskState++;	
							}
							else if( KINECT_SERVO_TIMED_OUT == ServoStatus )
							{
								ROBOT_LOG( TRUE,"ERROR: KINECT SERVO TIME OUT!  ABORTING!\n\n")
								SendCommand( WM_ROBOT_KINECT_SEARCH_COMPLETE, 0, 0 ); // no objects found
								// Stop Tracking, and go back to looking for Humans
								m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
								m_TaskState = 1;
							}
							// else KINECT_SERVO_MOVING == ServoStatus								
							break; // keep waiting
						}
						case 3:	// Request 3D analysis at current position
						{
							ROBOT_LOG( TRUE,"KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT: Looking for Object\n")
							FindObjectsOnFloorRequest( KINECT_FIND_OBJECTS_REQUEST_RETRIES );	// Try "n" times
							m_TaskState++;	
							break;
						}
						case 4:	// Check results at current Kinect Tilt Position
						{
							if( -1 == m_nKinect3DObjectsFound )
							{
								// Kinect is still looking.  Just wait...
								//ROBOT_LOG( TRUE,"KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT Waiting...\n")
							}
							else
							{
								// if objects found, we're done, else move kinect and try again
								if( (0 == m_nKinect3DObjectsFound) && (m_KinectScanPosition < KINECT_SERVO_POSITION_FAR_SCAN) && (!m_FindCloseObjectsOnly) )
								{
									// Move Kinect to next position and try again
									m_KinectScanPosition++;
									ROBOT_LOG( TRUE,"KINECT Scan Position %d\n", m_KinectScanPosition)
									m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, KINECT_FLOOR_SCAN_POSITION[m_KinectScanPosition] );
									m_TaskState = 2; // Move Kinect to next position and try again
								}
								else
								{
									// we're done, send message
									DWORD dwLocation = 0;
									if( m_nKinect3DObjectsFound > 0 )
									{
										dwLocation = (DWORD)MAKELONG( (WORD)(g_pKinectObjects3D->Object[0].CenterX), 
																	  (WORD)(g_pKinectObjects3D->Object[0].CenterY) ); //LOWORD, HIWORD
									}
									else
									{
										ROBOT_LOG( TRUE,"**************** KINECT - NOTHING FOUND 1 ***************\n")
									}
									SendCommand( WM_ROBOT_KINECT_SEARCH_COMPLETE, (DWORD)m_nKinect3DObjectsFound, dwLocation );
									// Stop Tracking, and go back to looking for Humans
									m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
									m_TaskState = 1;
								}
							}
							break;
						}
						default: ROBOT_ASSERT(0);
					}
					break;
				}	// KINECT_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT


				case KINECT_TASK_TRACK_CLOSEST_OBJECT:
				{	
					// Used to track the closest object on the floor
					int ServoStatus = m_pKinectServoControl->CheckServoPosition(FALSE);
					if( KINECT_SERVO_TIMED_OUT == ServoStatus )
					{
						ROBOT_LOG( TRUE,"ERROR: KINECT SERVO TIME OUT!  ABORTING!\n\n")
						SendCommand( WM_ROBOT_KINECT_SEARCH_COMPLETE, 0, 0 ); // no objects found
						// Stop Tracking, and go back to looking for Humans
						m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
						m_TaskState = 1;
					}
					else if( KINECT_SERVO_MOVING == ServoStatus )
					{
						// keep waiting
						break;
					}
					// else KINECT_SERVO_SUCCESS == ServoStatus - Done!

					switch( m_TaskState )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Request 3D analysis at current position
						{
							FindObjectsOnFloorRequest( KINECT_FIND_OBJECTS_REQUEST_RETRIES );	// Try "n" times
							m_TaskState++;	
							break;
						}
						case 2:	// Check results at current Kinect Tilt Position
						{
							if( -1 == m_nKinect3DObjectsFound )
							{
								// Kinect is still looking.  Just wait...
								ROBOT_LOG( TRUE,"KINECT_TASK_TRACK_CLOSEST_OBJECT Waiting...\n")
								break;
							}
							else
							{
								// Determine if the Kinect tilt needs to be adjusted
								if ( KINECT_SERVO_POSITION_CLOSE_SCAN != m_KinectScanPosition )
								{
									// Not in Close Scan Mode
									if( 0 == m_nKinect3DObjectsFound )
									{
										ROBOT_LOG( TRUE, "TRACK_CLOSEST_OBJECT - Lost Object!  Moving to CLOSE_SCAN\n" )
										m_KinectScanPosition = KINECT_SERVO_POSITION_CLOSE_SCAN;
										m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, KINECT_FLOOR_SCAN_POSITION[m_KinectScanPosition] );
									}
									else
									{
										// Objects were found
										// See if object is close enough for close scan.  If so, swich to close scan mode
										// Note - Top of frame = 0, Bottom of frame = 480 or 240
										int FrameCenterY = m_FrameInfo->Height / 2; //m_CaptureSize.height / 2;
										int ObjectFrameCenterY =  g_pKinectObjects3D->Object[0].EndScanLine + ( (g_pKinectObjects3D->Object[0].StartScanLine - g_pKinectObjects3D->Object[0].EndScanLine) / 2 );	// StartScanLine is the bigger number 
										if( ObjectFrameCenterY > (FrameCenterY + (m_FrameInfo->Height / 5)) )	
										{
											// Object within zone where close scan can find it, so tilt Kinect down
											// Move to the close position, so we are ready for the next request
											ROBOT_LOG( TRUE, "TRACK_CLOSEST_OBJECT - Object in range, Moving to CLOSE_SCAN\n" )
											m_KinectScanPosition = KINECT_SERVO_POSITION_CLOSE_SCAN;
											m_pKinectServoControl->SetTiltPosition( KINECT_TILT_OWNER_TRACK_OBJECT, KINECT_FLOOR_SCAN_POSITION[m_KinectScanPosition] );
										}
									}
								}

								// Send result and end.
								DWORD dwLocation = 0;
								if( m_nKinect3DObjectsFound > 0 )
								{
									dwLocation = (DWORD)MAKELONG( (WORD)(g_pKinectObjects3D->Object[0].CenterX), 
																  (WORD)(g_pKinectObjects3D->Object[0].CenterY) ); //LOWORD, HIWORD
									WORD CenterX = (WORD)g_pKinectObjects3D->Object[0].CenterX;
									WORD CenterY = (WORD)g_pKinectObjects3D->Object[0].CenterY;
									DWORD dwLocationTEST = (DWORD)MAKELONG(CenterX, CenterY); //LOWORD, HIWORD
									ROBOT_ASSERT( (dwLocationTEST == dwLocation) );
								}
								else
								{
									ROBOT_LOG( TRUE,"**************** KINECT - NOTHING FOUND 2 ***************\n")
								}
								SendCommand( WM_ROBOT_KINECT_SEARCH_COMPLETE, (DWORD)m_nKinect3DObjectsFound, dwLocation );
								// Stop Tracking, and go back to looking for Humans
								m_CurrentTask = KINECT_TASK_HUMAN_DETECTION;
								m_TaskState = 1;
							}
							break;
						}
						default: ROBOT_ASSERT(0);
					}
					break;
				}	// WM_ROBOT_KINECT_TRACK_OBJECT_CMD
				default: ROBOT_ASSERT(0);
			}
			return;
		} // case WM_ROBOT_SENSOR_STATUS_READY:


/////////////// ALL THIS IS FOR FUTURE USE ON KINECT//////////////////
		case WM_ROBOT_CAMERA_POWER_CMD:
		{
	//		g_bCmdRecognized = TRUE;
			// Power Camera on/off
			// SendHardwareCmd( HW_SET_CAMERA_POWER, wParam, lParam );
			return;
		}

		case WM_ROBOT_CAMERA_POINT_TO_XYZ_CMD:
		{
	//		g_bCmdRecognized = TRUE;
			// Tell camera to go to ABSOLUTE PAN/TILT position pointing at X,Y,Z in space
			// wParam = X,Z in TenthInches
			// lParam = Y (distance from robot) in TenthInches

			// TODO REMOVE THIS COMMENT if( m_HandTrackingEnabled )
			{
/*				int Z = LOWORD(wParam);
				int X = HIWORD(wParam);
				int Y = lParam;
				m_pHeadControl->LookAtXYZ( HEAD_OWNER_USER_CONTROL, X,Y,Z );
				m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );
*/			}
			return;
		}
		
		case WM_ROBOT_CAMERA_TILT_ABS_CMD:
		{
			// Tell camera to go to ABSOLUTE TILT position specified in TENTHDEGREES
			// wParam = position to move to

	//		g_bCmdRecognized = TRUE;
	//		m_pHeadControl->SetHeadPosition( HEAD_OWNER_USER_CONTROL, NOP, wParam, NOP );	// Tilt only
	//		m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );

//			int CameraTiltPos = (int)wParam;
//			// Make sure Tilt command is within Camera limits
//			CheckPositionLimits( g_CameraPanPos, CameraTiltPos);

//			SendHardwareCmd( HW_SET_CAMERA_TILT_ABS, CameraTiltPos, 0 );
//			g_CameraTiltPos = CameraTiltPos;	// Keep track of position!
			return;
		}

		case WM_ROBOT_CAMERA_TILT_REL_CMD:
		{
			// wParam = Amount or positon to move to
			// lParam = Relative or Absolute
			// Tell camera to go to position RELATIVE to current position

	//		g_bCmdRecognized = TRUE;
	//		m_pHeadControl->SetHeadPositionRelative( HEAD_OWNER_USER_CONTROL, NOP, wParam, NOP );	// Tilt only
	//		m_pHeadControl->ExecutePosition( HEAD_OWNER_USER_CONTROL );

//			int CameraTiltPos = g_CameraTiltPos + (int)wParam;
			// Make sure Tilt command is within Camera limits
//			CheckPositionLimits( g_CameraPanPos, CameraTiltPos);
//			SendHardwareCmd( HW_SET_CAMERA_TILT_ABS, CameraTiltPos, 0 );
//			g_CameraTiltPos = CameraTiltPos;	// Keep track of position!
			return;
		}
	}
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
//	KINECT 3D OBJECT DETECTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindWallsAnd2dMaps
// updates global WallPoints array with any x,y coordinates of any walls found by Kinect
// Used to avoid large objects, and also to avoid detection of objects too close to walls
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define KINECT_WALL_OBJECT_SLOPE_THRESHOLD			0.7	// greater than .5 slope considered a wall or obstacle
#define KINECT_MIN_WALL_OBJECT_HEIGHT_TENTH_INCHES	40
#define KINECT_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES	120 // if greater than this, slope not needed, it's an object to avoid
#define KINECT_WALL_DETECT_GAP_SIZE					30 // number of pixels between front and back slope detector
 
void CKinectModule::FindWallsAnd2dMaps()
{
	// Horizontal Scan lines:  0=Right to Left
	//ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: Kinect EnterCriticalSection g_csKinectPointCloudLock\n")
	// __itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
	// EnterCriticalSection(&g_csKinectPointCloudLock);

//	TRACE("FindWallsAnd2dMaps: Edges at:");

	for( int HScan = 0; HScan < m_FrameInfo->Width; HScan++)
	{
		g_KinectPointCloud->WallPoints[HScan].X = g_KinectPointCloud->WallPoints[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR; // infinate range
		g_KinectPointCloud->MapPoints2D[HScan].X = g_KinectPointCloud->MapPoints2D[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;

//		ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: HScan = %d\n", HScan)
//		if( HScan > 162 )
	//	{
 		//	ROBOT_LOG( TRUE,"OUCH\n")
	//	}
		BOOL	LeadingEdgeFound = FALSE;
		int		ObjPeakHeight = 0;	// Height of object or cliff
		int		LeadingEdgeY = KINECT_RANGE_TENTH_INCHES_MAX;
		int		FloorValue = 0;			// floor value closest to robot
		int		LeadPos = 0;			// Leading sample.  // Training sample (separated by LeadPos by GAP_SIZE)
		int		TrailPos = 0;			// Training sample (separated by LeadPos by GAP_SIZE)

		// Walk up each VERTICAL scan line, closest to robot first (0 = top of frame)
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_ProcessVerticalScan);
		for( int LeadPos = (m_FrameInfo->Height-(KINECT_WALL_DETECT_GAP_SIZE+1)); LeadPos >0; LeadPos-- )
		{
			//ROBOT_LOG( TRUE,"             FindWallsAnd2dMaps: LeadPos = %d\n", LeadPos)

			TrailPos = LeadPos+KINECT_WALL_DETECT_GAP_SIZE;

			if( (g_KinectPointCloud->Point3dArray[LeadPos][HScan].Z >= KINECT_RANGE_TENTH_INCHES_MAX) || // Note Y,X, not X,Y!
				(g_KinectPointCloud->Point3dArray[TrailPos][HScan].Z >= KINECT_RANGE_TENTH_INCHES_MAX) )
			{
				continue; // skip invalid pixels
			}

			int LeadZ = g_KinectPointCloud->Point3dArray[LeadPos][HScan].Z;
			int TrailZ = g_KinectPointCloud->Point3dArray[TrailPos][HScan].Z;
			double HeightDelta = LeadZ - TrailZ;
			double WidthDelta = g_KinectPointCloud->Point3dArray[LeadPos][HScan].Y - g_KinectPointCloud->Point3dArray[TrailPos][HScan].Y;
			double Slope = HeightDelta / WidthDelta;
			double ObjectSlope = 0;

			// Find walls and objects to avoid
			if( (abs(Slope) >= KINECT_WALL_OBJECT_SLOPE_THRESHOLD) ||		// detect changes between average floor and object
				(LeadZ > KINECT_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES) )
			{
				// Some Object detected!
				if( !LeadingEdgeFound )
				{
					// Start of new object, wall, or cliff
					LeadingEdgeFound = TRUE;
					LeadingEdgeY = g_KinectPointCloud->Point3dArray[LeadPos][HScan].Y;
					//TrailingEdgeFound = FALSE; // Handle "bumpy" objects
					ObjectSlope = Slope;
					FloorValue = TrailZ;
					ObjPeakHeight = LeadZ;	// keep track of peak height
					if( FloorValue > KINECT_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES )
					{
						// never detected the floor, but we got a wall!
						FloorValue = 0;
					}
				}
				else
				{
					// Leading Edge found.  Possible continuation of the object, wall, or cliff
					if(ObjectSlope > 0 )
					{	// We're working on a wall or object
						if( LeadZ > ObjPeakHeight )
						{
							ObjPeakHeight = LeadZ;	// keep track of peak height
						}
						if (Slope < 0 )
						{
							// trailing edge of the object
							//TrailingEdgeFound = TRUE
							break; // stop looking in this column
						}
					} 
					else
					{
						// Cliff
						if( LeadZ < ObjPeakHeight )	 // for cliffs, Peak Height is a negative number
						{
							ObjPeakHeight = LeadZ;	// keep track of peak height
						}
						if (Slope > 0 )
						{
							// bottom of the cliff
							//TrailingEdgeFound = TRUE
							break; // stop looking in this column
						}
					}
				}
			}
			else
			{
				// No slope - Flat floor
				if( LeadingEdgeFound )
				{
					// we were working on something, but now we are done.
					break; // stop looking in this column
				}
			}

		} // for vertical scan line
		__itt_task_end(pDomainKinectThread); // 

		// Done with vertical scan line. Update summary data
		if( LeadingEdgeFound )
		{
			// MapPoints contains any blocking object 
			int detectHeight = 20;
			if( LeadingEdgeY >  detectHeight ) // cut out some noise MAP_OBJECT_DETECT_HEIGHT_TENTH_INCHES
			{
				g_KinectPointCloud->MapPoints2D[HScan].X = g_KinectPointCloud->Point3dArray[LeadPos][HScan].X;
				g_KinectPointCloud->MapPoints2D[HScan].Y = LeadingEdgeY;
				//g_KinectPointCloud->MapPoints2D[HScan].Z = ObjPeakHeight - FloorValue;
				//if( (g_KinectPointCloud->MapPoints2D[HScan].X > -60) && (g_KinectPointCloud->MapPoints2D[HScan].X < 60) )
				//	TRACE(" Y=%d, X=%d, PeakHeight=%d\n",  g_KinectPointCloud->MapPoints2D[HScan].X, g_KinectPointCloud->MapPoints2D[HScan].Y, ObjPeakHeight );
			}

			if( FloorValue < KINECT_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES )
			{
				FloorValue = 0;
			}

			if( abs(ObjPeakHeight - FloorValue) > KINECT_WALL_DETECT_HEIGHT )
			{
				// WallPoints contains tall objects (used by object detector to avoid objects close to wall)
				g_KinectPointCloud->WallPoints[HScan].X = g_KinectPointCloud->Point3dArray[LeadPos][HScan].X;
				g_KinectPointCloud->WallPoints[HScan].Y = LeadingEdgeY;
				//g_KinectPointCloud->WallPoints[HScan].Z = ObjPeakHeight - FloorValue;

				//if( (g_KinectPointCloud->MapPoints2D[HScan].X > -90) && (g_KinectPointCloud->MapPoints2D[HScan].X < 90) )
					//TRACE(" WALL: Y=%d, X=%d, PeakHeight=%d\n",  g_KinectPointCloud->WallPoints[HScan].Y, g_KinectPointCloud->WallPoints[HScan].X, ObjPeakHeight );
			}
		}

	}
	//LeaveCriticalSection(&g_csKinectPointCloudLock);
	//__itt_task_end(pDomainKinectThread);
	//ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: Kinect LeaveCriticalSection g_csKinectPointCloudLock\n")

	// Now, look for Doorways -used by Avoidance Module, and displayed in the GUI
	// (Avoidance module does not alway run, but useful to see this in the GUI anyway)

	int NumberOfKinectSamples = g_KinectPointCloud->FrameSizeX; 
	//int nMaxDoorways = MAX_DOORWAYS;
//	DOORWAY_T  *DoorWaysFound[MAX_DOORWAYS];	
//	POINT2D_T *pPointArray = g_KinectPointCloud->WallPoints;
	FindDoors( NumberOfKinectSamples, MAX_DOORWAYS, g_KinectPointCloud->MapPoints2D, g_pNavSensorSummary->DoorWaysFound );

	// Now throw message in the queue, to indicate that the Kinect 2D Map data has been updated
	//TODO!	PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_GPS_DATA_READY), 0, 0 );
	// Post message to the GUI display (local or remote)
	SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM,	// command
		ROBOT_RESPONSE_KINECT_DATA,				// Param1 = Bulk data command
		0 );									// Param2 = not used

//	LeaveCriticalSection(&g_csKinectPointCloudLock);
//	__itt_task_end(pDomainKinectThread);
/*
	// For DEBUG - Display results
	ROBOT_LOG( TRUE,"DEBUG: KINECT FindWallsAnd2dMaps: Wall points at: ")
	BOOL bWallFound = FALSE;
	for( int HScan = 0; HScan < m_FrameInfo->Width; HScan++)
	{
		if( g_KinectPointCloud->WallPoints[HScan].Y < LASER_RANGEFINDER_TENTH_INCHES_MAX )
		{
			ROBOT_LOG( TRUE,"%d, ", g_KinectPointCloud->WallPoints[HScan].Y )
		}
	}
	if( bWallFound )
		ROBOT_LOG( TRUE,"TenthInches\n")
	else
		ROBOT_LOG( TRUE,"No Walls Found\n")
*/

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UpdateKinectObjectSummary
// Updates global summary data with distance to closest objects found by Kinect in each object avoidance "zone"
// For robots equiped with fixed position Laser Scanner, this is similar info, but gives info for objects at any height
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CKinectModule::UpdateKinectObjectSummary()
{
//	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
//	EnterCriticalSection(&g_csKinectPointCloudLock);

	// Set all values to NO_OBJECT_IN_RANGE
	InitScannerSummaryData( &m_KinectSummary ); 

	// Save time-sensitive data to store with the summary
	m_KinectSummary.SampleTimeStamp = GetTickCount();	// Time laser sample was done
	m_KinectSummary.RobotLocation.x = g_pFullSensorStatus->CurrentLocation.x;		// Location of robot at the time of the laser scan
	m_KinectSummary.RobotLocation.y = g_pFullSensorStatus->CurrentLocation.y;		
	m_KinectSummary.CompassHeading = g_pFullSensorStatus->CompassHeading;		// Heading of robot at the time of the laser scan

	// Now find the minimum Y value for each zone
	for( int i = 0; i < m_FrameInfo->Width; i++ ) 
	{
		int X = g_KinectPointCloud->MapPoints2D[i].X; // tenth inches
		int Y = g_KinectPointCloud->MapPoints2D[i].Y;

		if( (X >= LASER_RANGEFINDER_TENTH_INCHES_MAX) || (Y >= LASER_RANGEFINDER_TENTH_INCHES_MAX) )
		{
			continue; // Skip out of range value
		}

		// Determine the zone.
		if( X < 0 )
		{
			// Front Left
			if( X >= -FRONT_CENTER_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_KinectSummary.nLeftFrontZone ) 
					m_KinectSummary.nLeftFrontZone = Y;
				//if( Y < 500 ) 
					//ROBOT_LOG( TRUE,"DEBUG: m_KinectSummary %d Y = %d\n", i, Y )
			}
			else if( X >= -FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_KinectSummary.nLeftArmZone ) 
					m_KinectSummary.nLeftArmZone = Y;
			}
			else if( X >= -FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_KinectSummary.nLeftFrontSideZone ) 
					m_KinectSummary.nLeftFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_KinectSummary.nLeftSideZone ) m_KinectSummary.nLeftSideZone = SideDist;
			}
		}
		else
		{
			// Front Right
			if( X <= FRONT_CENTER_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_KinectSummary.nRightFrontZone ) 
					m_KinectSummary.nRightFrontZone = Y;
			}
			else if( X <= FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_KinectSummary.nRightArmZone ) 
					m_KinectSummary.nRightArmZone = Y;
			}
			else if( X <= FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )  
			{
				if( Y < m_KinectSummary.nRightFrontSideZone ) 
					m_KinectSummary.nRightFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_KinectSummary.nLeftSideZone ) m_KinectSummary.nLeftSideZone = SideDist;
			}
		}

	}

	// Finally, update the global data to indicate to other modules we have a new measurement
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectSummaryDataLock);
	EnterCriticalSection(&g_csKinectSummaryDataLock);
		memcpy_s(g_pKinectSummary, sizeof(SCANNER_SUMMARY_T), &m_KinectSummary, sizeof(SCANNER_SUMMARY_T) );
	LeaveCriticalSection(&g_csKinectSummaryDataLock);
	__itt_task_end(pDomainKinectThread);


}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsInSingleScanLine
// Look for objects (near the center?) in a single scan line of the Kinect depth sensor
// WARNING!  Kinect sensor values are in MM!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CKinectModule::FindObjectsInSingleScanLine( int  ScanLine, int NumberOfSamples, OBJECT_2D_ARRAY_T* pKinectObjects2D )
{

#define KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM	   1
#define LEFT_SAMPLES_TO_AVE						   2
#define RIGHT_SAMPLES_TO_AVE					   2
//#define GAP_SIZE								   5	// Pixels!  For Kinect 320 / ~16" = 20 pixels/inch
#define GAP_SIZE								  10	// Pixels!  For Kinect this is 640 / ~16" = 40 pixels/inch
#define OBJECT_DETECT_HEIGHT_TENTH_INCHES		   9	// (TenthInches)
#define OBJECT_DETECT_WIDTH_TENTH_INCHES		   5	// min width to capture an object in the scan line(TenthInches)
#define MAP_OBJECT_DETECT_HEIGHT_TENTH_INCHES	  20	// objects taller than this show up on the map (TenthInches)

//ROBOT_LOG( TRUE,"ScanLine %d - FindObjectsInSingleScanLine() \n", ScanLine )

	if( (NULL == pKinectObjects2D) || (NULL == g_pKinectObjects3D) || (NULL ==g_KinectPointCloud) )
	{
		ROBOT_ASSERT(0);
	}
	
	int  SampleCount = 0;
	double AverageSum = 0;
	double FloorAverage = 0;
	//int Position = g_KinectScannerData[ScanLine].KinectAngleTenthDegrees;

	pKinectObjects2D->nObjectsDetected  = 0;

/*
	if( (0!=ScanLine) && (m_OpenCVDepthMousePoint.y == ScanLine) )
	{
		ROBOT_LOG( TRUE, "DEBUG Mouse Down\n");
	}
*/

	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: Kinect EnterCriticalSection g_csKinectPointCloudLock\n")
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
	EnterCriticalSection(&g_csKinectPointCloudLock);


	/////////////////////////////////////////////////////
	// Now find objects
	// This code looks for a step in the Z direction, by looking for 
	// N floor samples -- Gap -- N object samples
	// Note ALL VALUES IN TENTH INCHES unless otherwise indicated
		
	//int		PeakDetectOffsetIndex = 0;
	//double	RightFloorValue = 0;
	//double	LeftFloorValue = 0;
	//int		PeakHeight = 0; // keep track of highest point in the scan line
	int		StartIndex = 0;
	int		PeakHeight = 0;
	BOOL	LeadingEdgeFound = FALSE;
	BOOL	TrailingEdgeFound = FALSE;
	POINT3D_T StartPt = {0,0,0};
	POINT3D_T EndPt = {0,0,0};

	/////////////////////////////////////////////////////////////////////////////////////////////////
	for(int i = 0; i < (NumberOfSamples); i++)
	{

		int Height = g_KinectPointCloud->Point3dArray[ScanLine][i].Z; 
		if(Height >= KINECT_RANGE_TENTH_INCHES_MAX )
		{
			continue; // Invalid Pixel from deph sensor
		}

		if( !LeadingEdgeFound && (Height >= OBJECT_DETECT_HEIGHT_TENTH_INCHES) )
		{
			// Start of a new object
			StartIndex = i;
			PeakHeight = Height; // set initial peak height
			LeadingEdgeFound = TRUE;
			StartPt = g_KinectPointCloud->Point3dArray[ScanLine][i];
			//g_KinectPointCloud->Point3dArray[ScanLine][i+j].Z
			//ROBOT_LOG( TRUE,"Leading Edge Found X = %3.1f\n", (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0)
			continue;
		}
		else if( LeadingEdgeFound && !TrailingEdgeFound )
		{
			// Tracking an object
			if( Height >= OBJECT_DETECT_HEIGHT_TENTH_INCHES )
			{
				if( Height > PeakHeight)
				{
					PeakHeight = Height;
				}				
			}
			else
			{
				// found trailing edge
				//ROBOT_LOG( TRUE,"Trailing Edge Found NEAR X = %3.1f\n", (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0 )
				TrailingEdgeFound = TRUE;
				EndPt = g_KinectPointCloud->Point3dArray[ScanLine][i];
			}
		}
		if( LeadingEdgeFound && TrailingEdgeFound )
		{
			// End of object found!
			int Width = abs(StartPt.X - EndPt.X); // todo - get rid of the abs?
			int AverageY = (StartPt.Y + EndPt.Y) / 2;
			int AverageX = (StartPt.X + EndPt.X) /2;
			// Increase height requirement if the object is far away from the robot (Kinect values are less accurate, due to angle)
			int MinDetectionHeight = KINECT_SLICE_OBJECT_HEIGHT_MIN + AverageY / OBJECT_DISTANCE_COMPENSATION;

			if( (PeakHeight < KINECT_SLICE_OBJECT_HEIGHT_MAX) && (PeakHeight > MinDetectionHeight) && 
				(Width < KINECT_SLICE_OBJECT_WIDTH_MAX) && (Width > KINECT_SLICE_OBJECT_WIDTH_MIN) )	// Skip big or tiny objects
			{
				// Good object found!

				// Save Object info (in TENTH INCHES)!
				// Summary info
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].X = AverageX;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Y = AverageY;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].PeakHeight = PeakHeight;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Width = Width;
				// Details
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartX = StartPt.X;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartY = StartPt.Y;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartZ = StartPt.Z;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndX = EndPt.X;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndY = EndPt.Y;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndZ = EndPt.Z;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].LeftPixel = StartIndex;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].RightPixel = i;
				pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].ScanLine = ScanLine;


				// ROBOT_LOG( TRUE,"Small Object Found: StartX = %3.1f,  EndX = %3.1f, Height = %3.1f Y Distance = %3.1f  FloorRight = %3f, FloorLeft = %3f\n", 
				//			Objects[nDetectedObjects].StartX, Objects[nDetectedObjects].EndX, Objects[nDetectedObjects].PeakHeight, Objects[nDetectedObjects].StartY, RightFloorValue, LeftFloorValue )

				// DEBUG: Display the object found
				#if DEBUG_KINECT_SHOW_2D_OBJECTS_FOUND == 1
					ROBOT_LOG( TRUE,"Scanline %d      Found 2D Object %d at Y=%4.1f, X=%4.1f   Height = %4.1f   Width = %4.1f   StartX = %4.1f   EndX = %4.1f\n", 
						ScanLine, pKinectObjects2D->nObjectsDetected+1,
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Y)/10.0,
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].X)/10.0,
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].PeakHeight)/10.0, 
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Width)/10.0,
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartX)/10.0, 
						(double)(pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndX)/10.0 )
				#endif

				if( pKinectObjects2D->nObjectsDetected++ > DEPTH_SCAN_MAX_3D_OBJECTS )
				{
					ROBOT_LOG( TRUE, "ERROR!  Kinect Scanner LookForObjects nDetectedObjects > KINECT_SCAN_MAX_OBJECTS!  ABORTING further object search!\n")
					ROBOT_ASSERT(0);
					break;
				}
			}
			else
			{
				// DEBUG: Display the slice NOT found, on whatever line the mouse clicks on
				#if DEBUG_KINECT_SHOW_2D_OBJECTS_NOT_FOUND == 1

					if( (0!=ScanLine) && (m_OpenCVDepthMousePoint.y == ScanLine) )
					{
						ROBOT_LOG( TRUE,"Scanline %d      REJECTED 2D Object at Y=%4.1f, X=%4.1f   Height = %4.1f   Width = %4.1f   StartX = %4.1f   EndX = %4.1f\n", 
							ScanLine,
							(double)(AverageY)/10.0,
							(double)(AverageX)/10.0,
							(double)(PeakHeight)/10.0, 
							(double)(Width)/10.0,
							(double)(StartPt.X)/10.0, 
							(double)(EndPt.X)/10.0 )

						if( PeakHeight >= KINECT_SLICE_OBJECT_HEIGHT_MAX)
							ROBOT_LOG( TRUE,"PeakHeight > KINECT_SLICE_OBJECT_HEIGHT_MAX\n" )
						if( PeakHeight <= MinDetectionHeight )
							ROBOT_LOG( TRUE,"PeakHeight < MinDetectionHeight (peak %d is < %d)\n", PeakHeight, MinDetectionHeight )
						if( Width >= KINECT_SLICE_OBJECT_WIDTH_MAX )
							ROBOT_LOG( TRUE,"Width > KINECT_SLICE_OBJECT_WIDTH_MAX\n" )
						if( Width <= KINECT_SLICE_OBJECT_WIDTH_MIN )
							ROBOT_LOG( TRUE,"Width < KINECT_SLICE_OBJECT_WIDTH_MIN\n" )
					}
				#endif
			}
			StartIndex = 0;
			PeakHeight = 0;
			LeadingEdgeFound = FALSE;
			TrailingEdgeFound = FALSE;
		}
	}
	//ROBOT_LOG( TRUE,"\n")
	if( (0!=ScanLine) && (m_OpenCVDepthMousePoint.y == ScanLine) )
	{
		m_OpenCVDepthMousePoint.y = 0;
		m_OpenCVDepthMousePoint.x = 0;
	}

	LeaveCriticalSection(&g_csKinectPointCloudLock);
	__itt_task_end(pDomainKinectThread);
	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: Kinect LeaveCriticalSection g_csKinectPointCloudLock\n")
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsOnFloorRequest
// queues up a request to the Kinect Frame Grap thread to find 3D objects on the floor at the current tilt position
void CKinectModule::FindObjectsOnFloorRequest( int NumberOfTries )
{
	m_nKinect3DObjectsFound = -1; // indicate to the caller that the request is pending
	m_FindObjectsOnFloorTrys = NumberOfTries;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsOnFloor
// Parse scan lines from Kinect and find 3D objects on the floor
// Returns the number of objects detected
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OBJECT_CENTER_TOLLERANCE 30 // TenthInches - Max allowed distance between centers of subsequent scan lines (X+Y)
//#define OBJECT_CENTER_Y_WEIGHT   5 // multiplier -  Tunes for how much more important Y is than X when matching found objects
#define WALL_EDGE_ZONE_SIZE	30 // TenthInches - Objects within this distane to wall/large objects are not tracked by Kinect

__itt_string_handle* pshFindObjectsLoop = __itt_string_handle_create("FindObjectsLoop");
__itt_string_handle* pshObjectsUpdate = __itt_string_handle_create("ObjectsUpdate");
__itt_string_handle* pshFindObjectsSingleScanLine = __itt_string_handle_create("FindObjectsSingleScanLine");
__itt_string_handle* pshFind3DObjects = __itt_string_handle_create("Find3DObjects");

void CKinectModule::FindObjectsOnFloor()
{
	#if( DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1 )
		ROBOT_ASSERT( g_pKinectDebugSliceArray );
		g_pKinectDebugSliceArray->nSlices = 0;
	#endif
	if( NULL == m_FrameInfo ) return; // means we could not open the shared memory file

	#if (DEBUG_FIND_OBJECTS_ON_FLOOR == 1)
		m_FindObjectsOnFloorTrys = 1;
	#endif

	// Only look for objects if the Kinect is in position and
	// track how many times to retry finding objects on the floor
	int ServoPosition = m_pKinectServoControl->GetTiltPosition();
	if( (m_FindObjectsOnFloorTrys <= 0) || (ServoPosition > KINECT_FLOOR_SCAN_POSITION[KINECT_SERVO_POSITION_FAR_SCAN]) )
	{
		// Don't look for any objects
		m_nKinect3DObjectsFound = 0;
		return;
	}

	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshFindObjectsOnFloor);

	// Reset the number of 3D objects found
	m_pKinectTempObjects3D->nObjectsDetected = 0;
	g_pKinectObjects3D->nObjectsDetected = 0;


	// process the Kinect POINT CLOUD data, from closest to furthest
	// Go thorugh each row of POINT CLOUD data and identify potential objects then compare these rows
	// to find true 3D objects. (Note this is not the raw depth data from the sensor, it is XYZ values)
	
	// First, look for walls ( Assumes FindWallsAnd2dMaps() was called

	// Uses POINT3D_T	Point3dArray[m_FrameInfo->Width][m_FrameInfo->Height];

	// Walk through each scan line, closest to robot first
	ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"FindObjectsOnFloor: Looking for 3D Objects\n")
	OBJECT_2D_ARRAY_T* pKinectObjects2D = new OBJECT_2D_ARRAY_T;
	
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshFindObjectsLoop);
	for( int ScanLine = m_FrameInfo->Height-1; ScanLine >=0; ScanLine-- ) // Start at BOTTOM of frame
	{
		//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"DEBUG: ScanLine = %d\n", ScanLine)
		memset( pKinectObjects2D, 0x00, sizeof( OBJECT_2D_ARRAY_T ) );
		
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshFindObjectsSingleScanLine);
			FindObjectsInSingleScanLine( ScanLine, m_FrameInfo->Width, pKinectObjects2D ); // returns pKinectObjects2D
		__itt_task_end(pDomainKinectThread);  // pshFindObjectsSingleScanLine

		// Now iterate through the list of 2D objects found on this line, and associate with any current 3D objects
		if( pKinectObjects2D->nObjectsDetected > 0 )
		{
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshFind3DObjects);
			for( int Index2D = (pKinectObjects2D->nObjectsDetected-1); Index2D >= 0; Index2D-- ) // for each 2D object
			{
				// Compare to each 3D object found so far, and find the nearest.
				// If none found close enough, this 2D object will be considered the start of a new 3D object
				int Nearest3DObjectDistance = KINECT_RANGE_TENTH_INCHES_MAX;
				int Nearest3DObjectIndex = -1;
				for( int Index3D = 0; Index3D < m_pKinectTempObjects3D->nObjectsDetected; Index3D++ )
				{
					int deltaX = abs( m_pKinectTempObjects3D->Object[Index3D].LastX - pKinectObjects2D->Object[Index2D].X );	// compare AVE 3D X value with current 2D object
					int deltaY = abs( m_pKinectTempObjects3D->Object[Index3D].EndY - pKinectObjects2D->Object[Index2D].Y );		// compare LAST 3D Y value with current slice (building up the 3D image in slices)
					int Distance = deltaX + deltaY;	// approx of squares

					// Look for the 3D object that is closest to the 2D object in this slice
					if( Distance < Nearest3DObjectDistance )
					{
						// 3D object is closer to this slice center than any other object found so far
						Nearest3DObjectDistance = Distance;
						Nearest3DObjectIndex = Index3D;
					}
				}

				// Ok, checked all the 3D objects, now update the closest one found
				if( Nearest3DObjectDistance > OBJECT_CENTER_TOLLERANCE )
				{
					// Must be a new 3D object.  Initialize the 3D object.
					if( m_pKinectTempObjects3D->nObjectsDetected > DEPTH_SCAN_MAX_3D_OBJECTS )
					{
						//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"WARNING: KinectMultiLineScanFindObjects3D: m_pKinectTempObjects3D->nObjectsDetected > DEPTH_SCAN_MAX_3D_OBJECTS. Ignoring far objects\n")
					}
					else
					{
						#if (DEBUG_KINECT_SHOW_3D_SLICE_OBJECTS_FOUND == 1)

						ROBOT_LOG( TRUE,"ScanLine %d - NEW 3D Object %3d:  Y=%4.1f, X=%4.1f   Width = %4.1f,   Height = %4.1f, Nearest Dist = %d\n", ScanLine, m_pKinectTempObjects3D->nObjectsDetected, 
							(double)(pKinectObjects2D->Object[Index2D].Y)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].X)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].Width)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].PeakHeight)/10.0, Nearest3DObjectDistance )
						#endif
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].NumberOfSlices = 1;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].CenterXSum =	pKinectObjects2D->Object[Index2D].X;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].HeightSum =	pKinectObjects2D->Object[Index2D].PeakHeight;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].WidthSum =		pKinectObjects2D->Object[Index2D].Width;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].StartY =		pKinectObjects2D->Object[Index2D].Y;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].EndY =			pKinectObjects2D->Object[Index2D].Y; // Just one slice so far
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].LastX =		pKinectObjects2D->Object[Index2D].X; // Just one slice so far

						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].CenterX =		pKinectObjects2D->Object[Index2D].X;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].CenterY =		pKinectObjects2D->Object[Index2D].Y;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].Height =		pKinectObjects2D->Object[Index2D].PeakHeight;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].Width =		pKinectObjects2D->Object[Index2D].Width;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].Length = 0;

						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].LeftPixel =  pKinectObjects2D->Object[Index2D].LeftPixel;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].RightPixel = pKinectObjects2D->Object[Index2D].RightPixel;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].StartScanLine = pKinectObjects2D->Object[Index2D].ScanLine;
						m_pKinectTempObjects3D->Object[m_pKinectTempObjects3D->nObjectsDetected].EndScanLine = pKinectObjects2D->Object[Index2D].ScanLine;

						m_pKinectTempObjects3D->nObjectsDetected++;
					}
				}
				else
				{
					// this "slice" lines up with another prior "slice" of a 3D object.  Save the data.
					// NOTE!  we keep the SUM and current Average of each data value at this point.

					if( Nearest3DObjectIndex < 0 ) 
						ROBOT_ASSERT(0);

					#if DEBUG_KINECT_SHOW_3D_SLICE_OBJECTS_FOUND == 1
						ROBOT_LOG( TRUE,"    3D Slice found on ScanLine %3d:  Y=%4.1f, X=%4.1f   Width = %4.1f,   Height = %4.1f  Obj Index = %d\n", ScanLine, 
							(double)(pKinectObjects2D->Object[Index2D].Y)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].X)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].Width)/10.0, 
							(double)(pKinectObjects2D->Object[Index2D].PeakHeight)/10.0, Nearest3DObjectIndex )

					#endif
					#if( DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1 )
						ROBOT_ASSERT( g_pKinectDebugSliceArray );
						if( g_pKinectDebugSliceArray->nSlices < MAX_DEBUG_SLICES )
						{
							g_pKinectDebugSliceArray->Slice[g_pKinectDebugSliceArray->nSlices].Pt1.y = ScanLine;
							g_pKinectDebugSliceArray->Slice[g_pKinectDebugSliceArray->nSlices].Pt2.y = ScanLine;
							g_pKinectDebugSliceArray->Slice[g_pKinectDebugSliceArray->nSlices].Pt1.x = pKinectObjects2D->Object[Index2D].LeftPixel;
							g_pKinectDebugSliceArray->Slice[g_pKinectDebugSliceArray->nSlices].Pt2.x = pKinectObjects2D->Object[Index2D].RightPixel;
							g_pKinectDebugSliceArray->nSlices++;
						}

					#endif

					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices++;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].CenterXSum += pKinectObjects2D->Object[Index2D].X;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].HeightSum += pKinectObjects2D->Object[Index2D].PeakHeight;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].WidthSum += pKinectObjects2D->Object[Index2D].Width;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].EndY = pKinectObjects2D->Object[Index2D].Y; // Keep updating this until the last slice

					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].LastX = pKinectObjects2D->Object[Index2D].X; // Keep updating this until the last slice. Used to look for aligned objects

					// for bounding box on GUI
					if( pKinectObjects2D->Object[Index2D].LeftPixel < m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].LeftPixel )
					{
						m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].LeftPixel = pKinectObjects2D->Object[Index2D].LeftPixel;
					}
					if( pKinectObjects2D->Object[Index2D].RightPixel > m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].RightPixel )
					{
						m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].RightPixel = pKinectObjects2D->Object[Index2D].RightPixel;
					}
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].EndScanLine = pKinectObjects2D->Object[Index2D].ScanLine; // keep track of the last scanline

					// Now set temporary values, used to find other close slices.  When we hit the last slice, these values should be correct!
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].CenterX = m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].CenterXSum / m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].CenterY = 
						m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].StartY + ((m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].EndY - m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].StartY) / 2);
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].Height = m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].HeightSum / m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].Width = m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].WidthSum / m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].Length = abs(m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].EndY - m_pKinectTempObjects3D->Object[Nearest3DObjectIndex].StartY);
				}
			} // for
			__itt_task_end(pDomainKinectThread); // pshFind3DObjects
		} // if
	}
	__itt_task_end(pDomainKinectThread); // FindObjectsLoop
	delete pKinectObjects2D;
	

	// Now, copy objects found to global, if they pass the size test
	// AND not too close to a wall (as defined by the Laser Range Finder)
	//	int ClosestObjectDistance = KINECT_RANGE_TENTH_INCHES_MAX;
	//	int ClosestObjectIndex = 0;

	if( m_pKinectTempObjects3D->nObjectsDetected > 0)
	{
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshObjectsUpdate);
		
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "\n*************************\n")
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "FINAL 3D Objects Found:\n", g_pKinectObjects3D->nObjectsDetected )
		g_pKinectObjects3D->nObjectsDetected = 0;
		g_pKinectObjects3D->nClosestObjectIndex = 0;
		g_pKinectObjects3D->nClosestObjectDistance = KINECT_RANGE_TENTH_INCHES_MAX;

		for( int i = 0; i < m_pKinectTempObjects3D->nObjectsDetected; i++ )
		{
			// Test size of object
			if( m_pKinectTempObjects3D->Object[i].Length < MIN_3D_OBJECT_LENGTH)		// minimim size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Small: Length = %d\n", i, m_pKinectTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Width < MIN_3D_OBJECT_WIDTH)			// minimim size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Small: Width = %d\n", i, m_pKinectTempObjects3D->Object[i].Width )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Length > MAX_3D_OBJECT_LENGTH)		// max size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Big: Length = %d\n", i, m_pKinectTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Width > MAX_3D_OBJECT_WIDTH)			// max size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Big: Width = %d\n", i, m_pKinectTempObjects3D->Object[i].Width )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Height > MAX_3D_OBJECT_HEIGHT)			// max height of object to detect (catch feet attached to legs)
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too High: Height = %d\n", i, m_pKinectTempObjects3D->Object[i].Height )
				continue;
			}
			if( abs(m_pKinectTempObjects3D->Object[i].CenterX) > MAX_3D_OBJECT_X_POSITION )	// Too far to one side
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Far off center: CenterX = %d\n", i, m_pKinectTempObjects3D->Object[i].CenterX )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].RightPixel >= m_FrameInfo->Width )	// Too close to edge of frame, might be chair leg
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Far off center: CenterX = %d\n", i, m_pKinectTempObjects3D->Object[i].CenterX )
				continue;
			}

			// Passes size test.  See if it's too close to a wall (might be a bogus object, furniture, etc.)
			// NOTE: Assumes FindWallsAnd2dMaps() was called first!

			int RoughDistance = LASER_RANGEFINDER_TENTH_INCHES_MAX;
			for( int HScan = 0; HScan < m_FrameInfo->Width; HScan++)
			{
				if( g_KinectPointCloud->WallPoints[HScan].Y < LASER_RANGEFINDER_TENTH_INCHES_MAX )
				{

					int DeltaX = abs( g_KinectPointCloud->WallPoints[HScan].X - m_pKinectTempObjects3D->Object[i].CenterX );
					int DeltaY = abs( g_KinectPointCloud->WallPoints[HScan].Y - m_pKinectTempObjects3D->Object[i].CenterY );
					RoughDistance = (DeltaX + DeltaY) / 2;
					//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )

					if( RoughDistance < WALL_EDGE_ZONE_SIZE )
					{
						// too close to wall
						ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: Point=%4d   Wall Y=%4d, X=%4d,     DELTA Y=%4d, X=%4d  DIST = %d\n", HScan,  
							g_KinectPointCloud->WallPoints[HScan].Y, g_KinectPointCloud->WallPoints[HScan].X, DeltaY, DeltaX, RoughDistance )
						ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Kinect Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to Wall (%4.1f in), skipping\n", 
							(double)m_pKinectTempObjects3D->Object[i].CenterY / 10.0,
							(double)m_pKinectTempObjects3D->Object[i].CenterX / 10.0,
							(double)m_pKinectTempObjects3D->Object[i].Height / 10.0, (double)RoughDistance / 10.0 )
						break;
					}
				}
			}
			if( RoughDistance < WALL_EDGE_ZONE_SIZE )
			{
				continue;	
			}

			// Check to see if the object extends outside of the safe detection zone
			// (reserve space at the top of the scan to detect walls, if too small, wall won't be found)
			// and watch out for long objects that extend outside of view
			int WallScanZoneScanLines = m_FrameInfo->Height / 4; // reserve top 1/4 of the frame for detecting walls
			if ( m_pKinectTempObjects3D->Object[i].EndScanLine <  WallScanZoneScanLines )
			{
				// too close to top of frame
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: ObjStartScanLine =%4d   ObjEndScanLine X=%4d, ZONE=%4d,\n",  
					m_pKinectTempObjects3D->Object[i].StartScanLine, m_pKinectTempObjects3D->Object[i].EndScanLine, KINECT_WALL_SCAN_ZONE )

				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Kinect Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to top of frame, skipping\n", 
					(double)m_pKinectTempObjects3D->Object[i].CenterY / 10.0,
					(double)m_pKinectTempObjects3D->Object[i].CenterX / 10.0,
					(double)m_pKinectTempObjects3D->Object[i].Height / 10.0 )
				break;
			}


			// PLAN B  Passes size test.  See if it's too close to a wall (might be a bogus object, furniture, etc.)
			// Compare to Laser values to determine this
//#define THIS_WORKS_NOW		
#ifdef THIS_WORKS_NOW
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csLaserDataLock);
			EnterCriticalSection(&g_csLaserDataLock);
				int  NumberOfLaserSamples = g_pLaserScannerData->NumberOfSamples;
				int Distance = 0;
				if( 0 != NumberOfLaserSamples )
				{
					for( int nSample = 0; i < NumberOfLaserSamples; nSample++ ) 
					{
						int WallPointX = g_pLaserScannerData->ScanPoints[nSample].X ; // tenth inches
						int WallPointY = g_pLaserScannerData->ScanPoints[nSample].Y ;
						int DeltaX = abs( WallPointX - m_pKinectTempObjects3D->Object[i].CenterX );
						int DeltaY = abs( WallPointY - m_pKinectTempObjects3D->Object[i].CenterY );
						Distance = DeltaX + DeltaY;
						//ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )

						if( Distance < WALL_EDGE_ZONE_SIZE )
						{
							// too close to wall
							ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )
							ROBOT_LOG( TRUE, "Kinect Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to Wall, skipping\n", 
								(double)m_pKinectTempObjects3D->Object[i].CenterX / 10.0,
								(double)m_pKinectTempObjects3D->Object[i].CenterY / 10.0,
								(double)m_pKinectTempObjects3D->Object[i].Height / 10.0 )
							break;
						}
					}
				} // if( 0 != NumberOfLaserSamples )
			LeaveCriticalSection(&g_csLaserDataLock);
			__itt_task_end(pDomainKinectThread);

			if( Distance < WALL_EDGE_ZONE_SIZE )
			{
				continue;
			}
#endif
			// OK, looks good!
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterX = m_pKinectTempObjects3D->Object[i].CenterX;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterY = m_pKinectTempObjects3D->Object[i].CenterY;// -10;	// TenthInches - fudge becase it always seems to miss the first inch of the object
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Height = m_pKinectTempObjects3D->Object[i].Height;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Width = m_pKinectTempObjects3D->Object[i].Width;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Length = m_pKinectTempObjects3D->Object[i].Length;// +10;		// TenthInches - fudge becase it always seems to miss the first inch of the object

			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].LeftPixel = m_pKinectTempObjects3D->Object[i].LeftPixel;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].RightPixel = m_pKinectTempObjects3D->Object[i].RightPixel;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].StartScanLine = m_pKinectTempObjects3D->Object[i].StartScanLine;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].EndScanLine = m_pKinectTempObjects3D->Object[i].EndScanLine;


			// For debug, print out the 3D objects found
			ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %2d:  Y=%4.1f X=%4.1f,   Height=%4.1f  Width=%4.1f,  Length=%4.1f, BB = (%d,%d) (%d,%d)\n", g_pKinectObjects3D->nObjectsDetected,
				(double)g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterY / 10.0,
				(double)g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterX / 10.0,
				(double)g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Height / 10.0,
				(double)g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Width / 10.0,
				(double)g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Length / 10.0,
				g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].LeftPixel,
				g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].StartScanLine,
				g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].RightPixel,
				g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].EndScanLine )


			if( g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterY < g_pKinectObjects3D->nClosestObjectDistance )
			{
				// New Closest Object found
				g_pKinectObjects3D->nClosestObjectDistance = g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterY;
				g_pKinectObjects3D->nClosestObjectIndex = g_pKinectObjects3D->nObjectsDetected;
			}
			g_pKinectObjects3D->nObjectsDetected++;


			if( g_pKinectObjects3D->nObjectsDetected > DEPTH_SCAN_MAX_3D_OBJECTS )
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"\nWARNING: 3D Objects found > DEPTH_SCAN_MAX_3D_OBJECTS. Ignoring far objects")
				break;
			}
		}
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Found %d 3D Objects\n", g_pKinectObjects3D->nObjectsDetected )
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "*************************\n\n")

		if( g_pKinectObjects3D->nClosestObjectDistance < KINECT_RANGE_TENTH_INCHES_MAX )
		{
			// An object was found. Tell the C# code to display a bounding box
			m_FrameInfo->ControlFlags |= KinectControlFlag_DisplayBoundingBox; // Draw a bounding box
			m_FrameInfo->BoundingBoxBottom = g_pKinectObjects3D->Object[g_pKinectObjects3D->nClosestObjectIndex].StartScanLine;
			m_FrameInfo->BoundingBoxTop = g_pKinectObjects3D->Object[g_pKinectObjects3D->nClosestObjectIndex].EndScanLine;	// lines are inverted
			m_FrameInfo->BoundingBoxLeft = g_pKinectObjects3D->Object[g_pKinectObjects3D->nClosestObjectIndex].LeftPixel;
			m_FrameInfo->BoundingBoxRight = g_pKinectObjects3D->Object[g_pKinectObjects3D->nClosestObjectIndex].RightPixel;
		}

		__itt_task_end(pDomainKinectThread); // pshObjectsUpdate
	} // if( m_pKinectTempObjects3D->nObjectsDetected > 0)

	// Report out final results
	if( g_pKinectObjects3D->nObjectsDetected > 0 )
	{
		// At least one good 3D object found
		m_nKinect3DObjectsFound = g_pKinectObjects3D->nObjectsDetected;
	}
	else
	{
		// No good 3D objects found
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "KinectMultiLineScanFindObjects3D:  No 3D Objects Found in Image.\n" )
		if( KINECT_FIND_OBJECTS_REQUEST_CONTINUOUS != m_FindObjectsOnFloorTrys )
		{
			if( --m_FindObjectsOnFloorTrys <= 0 ) // decrement the retry counter
			{
				// No more retries left, tell calling routine we are done
				m_nKinect3DObjectsFound = 0;	// anthing but -1 indicates we are done looking
				m_FindObjectsOnFloorTrys = 0;
			}
		}
	}

	__itt_task_end(pDomainKinectThread); // pshFindObjectsOnFloor
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Kinect Servo Control
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
// Name: GetTiltPosition
// Desc: Gets current tilt position of the Kinect sensor in TENTHDEGREES
//-----------------------------------------------------------------------------
int KinectServoControl::GetTiltPosition(  )
{
	return g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees;
}

//-----------------------------------------------------------------------------
// Name: SetTiltPosition
// Desc: Sets tilt position of the Kinect sensor in TENTHDEGREES
//-----------------------------------------------------------------------------
void KinectServoControl::SetTiltPosition( int  nOwner, int TiltTenthDegrees, int Speed ) // Speed is optional
{
	// NOTE: KINECT TILT *SPEED* IGNORED FOR NOW!!!  TODO???


	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	if( (nOwner == gKinectCurrentOwner) &&
		(TiltTenthDegrees == g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees) )
	{
		return; // ingore repeated command
	}


	// Check servo limit
	if( TiltTenthDegrees > KINECT_TILT_TENTHDEGREES_MAX_UP )
	{
		ROBOT_DISPLAY( TRUE, "KINECT_TILT_TENTHDEGREES_MAX_UP limit" )
		TiltTenthDegrees = KINECT_TILT_TENTHDEGREES_MAX_UP;
	}
	else if( TiltTenthDegrees < KINECT_TILT_TENTHDEGREES_MAX_DOWN)
	{
		ROBOT_DISPLAY( TRUE, "KINECT_TILT_TENTHDEGREES_MAX_DOWN limit" )
		TiltTenthDegrees = KINECT_TILT_TENTHDEGREES_MAX_DOWN;
	}

	__itt_marker(pDomainControlThread, __itt_null, pshKinectServoStart, __itt_marker_scope_task);
	//		ROBOT_LOG( TRUE,"Kinect SetTiltPosition\n")

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csServoLock);
	EnterCriticalSection(&g_csServoLock);
	/*	if( 0 != Speed )
		{
			g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Speed = Speed;
		}*/
		g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees = TiltTenthDegrees;
		g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].Update = TRUE;
	LeaveCriticalSection(&g_csServoLock);
	__itt_task_end(pDomainControlThread);

	// set the "watchdog" timer, in case the servo does not reach the commanded position
	gKinectMoveTimeout = DEFAULT_KINECT_MOVE_TIME_LIMIT_TENTH_SECONDS;

	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	//SendCommand( WM_ROBOT_SET_HEAD_POSITION, 0, FALSE );	// FALSE = Don't Set Speed

	if( 0 != Speed )
	{
		PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_KINECT_POSITION), 0, (DWORD)TRUE ); // Set Speed
	}
	else
	{
		PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_KINECT_POSITION), 0, (DWORD)FALSE ); // FALSE = Don't Set Speed
	}
}



//-----------------------------------------------------------------------------
// Name: CheckServoPosition
// Desc: Are we there yet?  Check to see if servo has reached commanded position
// Verbose flag will print out the servo that is blocking move complete status
// Returns:
//		KINECT_SERVO_MOVING = 0, 
//		KINECT_SERVO_SUCCESS,
//		KINECT_SERVO_TIMED_OUT
//-----------------------------------------------------------------------------
int KinectServoControl::CheckServoPosition( BOOL verbose )
{
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		return KINECT_SERVO_SUCCESS;	// Handle simulation mode (no servos!)
	}

	int DeltaTilt = g_BulkServoCmd[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees -
		g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees;

	//ROBOT_LOG( TRUE,"Kinect CheckServoPosition: DeltaTilt = %d Degrees\n", DeltaTilt/10)

	if( abs(DeltaTilt) > KINECT_JOINT_DELTA_MAX_TENTHDEGREES )
	{
		if( verbose )
		{
			// In verbose mode, show why we are not yet in position
			ROBOT_LOG( TRUE,"Kinect CheckServoPosition: DeltaTilt = %d Degrees\n", DeltaTilt/10)
		}
		if( 0 == gKinectMoveTimeout )
		{
			//if( verbose ) 
				ROBOT_LOG( TRUE,"Kinect CheckServoPosition: Servo Move Timed Out!  Delta = %d TenthDegrees\n", DeltaTilt)
			return KINECT_SERVO_TIMED_OUT;
		}
	}
	else
	{
		__itt_marker(pDomainControlThread, __itt_null, pshKinectServoEnd, __itt_marker_scope_task);

		if( verbose )
		{
			ROBOT_LOG( TRUE,"Kinect CheckServoPosition: Servo in Target Position\n")
		}
		return KINECT_SERVO_SUCCESS;
	}

	return KINECT_SERVO_MOVING;	// Did not reach target position yet
}

/////////////////////////////////////////////////////////////////////////////////////////
// KINECT SERVO CONTROL ARBITRATOR
// Tracks who is control of the Kinect tilt position, assigning control based upon a strict priority
/////////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
// Name: Check And Set Owner
// Desc: Checks for current owner, and if requesting module is higher priority
// changes to new module.  Returns TRUE if module got ownership.
//-----------------------------------------------------------------------------
BOOL KinectServoControl::CheckAndSetOwner( int  NewOwner )
{
	if( NewOwner == gKinectCurrentOwner )
	{
		// Owner reasking for control
		// Silently reset timer for how long to maintain control
		if( gKinectOwnerTimer < 50 )
		{
			gKinectOwnerTimer = 50;	// 1/10 seconds
		}
		return TRUE;
	}

	CString MsgString, NewOwnerName, CurrentOwnerName;
	KinectOwnerNumberToName( gKinectCurrentOwner, CurrentOwnerName );
	KinectOwnerNumberToName( NewOwner, NewOwnerName );

	if( (0 == gKinectOwnerTimer) && (HEAD_OWNER_NONE != gKinectCurrentOwner) )
	{
		// ownership timed out
		MsgString.Format( "Kinect Control: Owner %s has timed out", CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		gKinectCurrentOwner = KINECT_TILT_OWNER_NONE;
	}

	if( NewOwner < gKinectCurrentOwner )
	{
		// Higher priority module has control
		if( NewOwner > KINECT_TILT_OWNER_COLLISION_AVOIDANCE ) // don't report random movement requests for control, too noisy
		{
			MsgString.Format( "Kinect Control: Request by %s Rejected: %s has control",NewOwnerName, CurrentOwnerName );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			ROBOT_LOG( TRUE,"gKinectOwnerTimer = %d\n", gKinectOwnerTimer)
		}
		return FALSE;
	}

	// No one with higher priority has control.
	if( NewOwner > gKinectCurrentOwner )
	{
		MsgString.Format( "Kinect Control: NewOwner %s has taken control from %s",NewOwnerName, CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );
		gKinectCurrentOwner = NewOwner;
	}

	// Requesting module has control now
	// Set timer for how long to maintain control
	gKinectOwnerTimer = 50;	// 1/10 seconds


	// Indicate on the GUI who is in control
	// SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_DRIVE_MODULE_OWNER, Module );
	return TRUE;

}

BOOL KinectServoControl::IsOwner( int  NewOwner )
{
	if( NewOwner != gKinectCurrentOwner )
	{
		// Not current owner
		return FALSE;
	}
	return TRUE;
}

BOOL KinectServoControl::ReleaseOwner( int  NewOwner )
{
	if( NewOwner < gKinectCurrentOwner )
	{
		// Higher priority module has control already so just ignore.
/*		CString MsgString, ModuleName, OwnerName;
		KinectOwnerNumberToName( Module, ModuleName );
		KinectOwnerNumberToName( gKinectCurrentOwner, OwnerName );
		MsgString.Format( "Module %s releasing control, but %s already has it", ModuleName, OwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
*/
		return FALSE;
	}

	// No one with higher priority has control.  Release ownership.
	CString MsgString, NewOwnerName;
	KinectOwnerNumberToName( NewOwner, NewOwnerName );
	MsgString.Format( "Kinect Control: Owner %s releasing control", NewOwnerName );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	gKinectCurrentOwner = KINECT_TILT_OWNER_NONE;
	return TRUE;

}





#endif // #if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
