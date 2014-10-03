// DepthCameraModule.cpp: CDepthCameraModule class implementation
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

//#include "kinect.h"
//#include <SpeechEnums.cs>


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define TRACK_TOP_OF_USERS_HEAD							 0
#define USE_DEPTH_CAMERA_TO_LOOKAT_HUMAN				 0 // NOTE!  REQUIRES "TRACK_TOP_OF_USERS_HEAD"
#define TILT_DEPTH_CAMERA_TO_TRACK_CLOSE_HUMANS			 0
#define DEBUG_SHARED_MEMORY								 0 // Show source data from shared memory
#define DEBUG_FIND_OBJECTS_ON_FLOOR						 0 // For debug, Just keep targeting and showing 3D objects found

#define DEBUG_DEPTH_CAMERA_SHOW_2D_OBJECTS_FOUND		 0
#define DEBUG_DEPTH_CAMERA_SHOW_2D_OBJECTS_NOT_FOUND	 0
#define DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME		 0
#define DEBUG_DEPTH_CAMERA_SHOW_3D_SLICE_OBJECTS_FOUND	 0
#define DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES			 0
#define DEPTH_CAMERA_TILT_LOOKING_AT_FLOOR				(-200) // DepthCameraTiltTenthDegrees
#define DEPTH_CAMERA_BAD_PIXELS_MAX						20
#define OBJECT_DISTANCE_COMPENSATION					60	// Require bigger objects at far distances


// ITT Instrumentation
__itt_string_handle* pshDepthCameraThreadLoop = __itt_string_handle_create("DepthCameraThreadLoop");
//__itt_string_handle* pshDepthCameraInit = __itt_string_handle_create("DepthCameraInit");
//__itt_string_handle* pshDepthCameraGetFrame = __itt_string_handle_create("DepthCameraGetFrame");
__itt_string_handle* pshDepthCameraGetDepthImage = __itt_string_handle_create("DepthCameraGetDepthImage");
__itt_string_handle* pshDepthCameraFrameLoop = __itt_string_handle_create("DepthCameraFrameLoop");
//__itt_string_handle* pshDepthCameraShowFrames = __itt_string_handle_create("DepthCameraShowFrames");
__itt_string_handle* pshDepthCameraSleep = __itt_string_handle_create("DepthCameraSleep");
__itt_string_handle* pshDepthCameraFindObjectsOnFloor = __itt_string_handle_create("DepthCameraFind Objects on Floor");

__itt_string_handle* pshDepthCameraServoStart = __itt_string_handle_create("DepthCameraServoMoveStart"); // marker
__itt_string_handle* pshDepthCameraServoEnd = __itt_string_handle_create("DepthCameraServoMoveEnd"); // marker

__itt_string_handle* psh_csDepthCameraFindWallsAnd2dMaps = __itt_string_handle_create("DepthCameraFindWallsAnd2dMaps"); // marker
__itt_string_handle* psh_UpdateDepthCameraObjectSummary = __itt_string_handle_create("DepthCameraUpdateDepthCameraObjectSummary"); // marker

__itt_string_handle* psh_csUpdateDepthCameraObjectSummary = __itt_string_handle_create("DepthCameraObjectSummary"); //marker

__itt_string_handle* psh_DepthCameraProcessVerticalScan = __itt_string_handle_create("DepthCameraProcessVerticalScan");

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

#define DEPTH_CAMERA_VIDEO_ENABLED  1

// Calculaiton of Depth Camera Field Of View (FOV) in degrees
// Robot at 36.0" from wall:  
// Depth Camera FOV width = 38.0", height = 28.8"
// Video Camera FOV width = 43.0", height = 32.0"
// ArcTan = Opp/Adj. Opp = 1/2 width or height (to form right triangle)
// In Excell:  =(DEGREES(ATAN( (C8/B8) ))) * 2
// Now, adjust values as needed to compensate for mechanical position
#define DEPTH_CAMERA_DEPTH_FOV_X	 93.10 	// 93.10 calculated.
#define DEPTH_CAMERA_DEPTH_FOV_Y	 77.32 	// 77.32 calculated.
#define DEPTH_CAMERA_VIDEO_FOV_X	100.13 	// 100.13 calculated.
#define DEPTH_CAMERA_VIDEO_FOV_Y	 83.27	// 83.27 calculated.

// FINAL 3D OBJECT PARAMETERS
#define MIN_3D_OBJECT_LENGTH					15	// TenthInches - minimim size of object to detect
#define MIN_3D_OBJECT_WIDTH						10	// 
#define MAX_3D_OBJECT_LENGTH					70	// TenthInches - maximum size of object to detect
#define MAX_3D_OBJECT_WIDTH						70
#define MAX_3D_OBJECT_HEIGHT					50	// Tenthincess - catch things like a foot attached to a leg
#define MAX_3D_OBJECT_X_POSITION			   480	// TenthInches - max distance from center in front of robot (ignore objects too far to the side)
#define LEFT_WINDOW_MARGIN						12

// SLICE PARAMETERS
#define DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MIN		 9	// TenthInches
#define DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MIN			 5	// TenthInches
#define DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MAX		(MAX_3D_OBJECT_HEIGHT * 2) // TenthInches - make this high, so 3D analysis function can see there is something to eliminate! (like a foot attached to a leg)
#define DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MAX		    (MAX_3D_OBJECT_LENGTH * 4)	// TenthInches - make this big, to allow post -processing to remove long shapes
#define DEPTH_CAMERA_MAX_FLOOR_HEIGHT_TENTH_INCHES	20	// Tenthinches - prevent finding object sitting on top of steps, etc.

#define DEPTH_CAMERA_WALL_DETECT_HEIGHT				80	// TenthInches - Anything taller than this is considered a "wall" or unmovable object
#define MAP_NOISE_FLOOR_TENTHINCHES					10  // Tenthinches - ignore objects shorter than this when making 2D map

// Reserve top 1/4 scan lines to detect walls at the top of the frame (0 = top of frame)
#define DEPTH_CAMERA_WALL_SCAN_ZONE					(m_DepthFrameInfo.Height / 4)

//#define DISPLAY_MODE_OVERLAY	1
//#define DISPLAY_MODE_DEPTH		2
//#define DISPLAY_MODE_IMAGE		3
//#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

//#define MAX_DEPTH 10000

#define DEFAULT_DEPTH_CAMERA_MOVE_TIME_LIMIT_TENTH_SECONDS	  100	// Default time for servo moves = 10 Seconds (100 Tenth Seconds)
//BOOL CALLBACK EnumWindowCallBack( HWND hwnd, LPARAM lParam );


///////////////////////////////////////////////////////////////////////////////
// Performance Tuning
//#define FRAME_SKIP_COUNT			1	// how many frames to skip between each processing (Lower number increases CPU load) 
//#define CAMERA_BLUR_SETTLE_TIME	   10	// After moving head, allow motion blur to settle before processing

//#define DEPTH_CAMERA_FRAME_RATE_MS			500 //100 = 10fps, 200 = 5fps, 1000 = 1 fps
//#define DEPTH_CAMERA_FRAME_MIN_SLEEP_TIME		 20 // don't allow DepthCamera thread to completely saturate the core
///////////////////////////////////////////////////////////////////////////////


#if (DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1)
	DEBUG_SLICE_ARRAY_T* g_pDepthCameraDebugSliceArray;			// temp array for debugging slices
#endif


///////////////////////////////////////////////////////////////////////////////
//	Video Callback Functions for OpenCV window (used for debug)
///////////////////////////////////////////////////////////////////////////////


void OnDepthCameraWindowMouseEvent( int event, int x, int y, int flags, void* param )
{
#if ( USE_OPEN_CV == 1 )
	IGNORE_UNUSED_PARAM (flags);

	CDepthCameraModule* pDepthCameraObject = (CDepthCameraModule*)param;

    if( !pDepthCameraObject )
        return;


    if( CV_EVENT_LBUTTONDOWN == event )
	{
		//ROBOT_LOG( TRUE,"OnDepthWindowMouseEvent: MouseDown at %d, %d\n", x, y)
		// FLIPPED VIDEO!
		int FlipX = pDepthCameraObject->m_DepthFrameInfo.Width - x;
		pDepthCameraObject->m_OpenCVDepthMousePoint = cvPoint(FlipX, y);
        pDepthCameraObject->m_OpenCVDepthMousePointSelected = TRUE;
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
// Name: DepthCameraDepthThreadProc   MAIN LOOP FOR DEPTH_CAMERA FRAME PROCESSING
// Desc: dedicated thread for grabbing DepthCamera Depth frames from shared memory
// This thread created by the CDepthCameraModule class
//************************************************************************************************************************
//************************************************************************************************************************

DWORD WINAPI DepthCameraDepthThreadProc( LPVOID lpParameter )
{
	CDepthCameraModule* pDepthCameraModule = (CDepthCameraModule*)lpParameter;
	ROBOT_ASSERT( pDepthCameraModule );

	IGNORE_UNUSED_PARAM (lpParameter);
	__itt_thread_set_name( "DepthCamera Thread" );

	CString MsgString;


	// Open Memory Mapped File
	/// if( SUBSYSTEM_DISABLED == g_DepthCameraSubSystemStatus )
	{
		/// return 0; // no sense wasting cycles, the DepthCamera is disabled
	}

	if( pDepthCameraModule->OpenMemoryMappedFile() )
	{
		/// g_DepthCameraSubSystemStatus = SUBSYSTEM_CONNECTED;
	}
	else
	{
		//ROBOT_ASSERT(0);
		TRACE("/n/n");
		ROBOT_LOG( TRUE, "**************************************************************************" )
		ROBOT_LOG( TRUE, "******** FAILED TO OPEN DEPTH CAMERA SHARED DEPTH FRAME MEMORY ! ********" )
		ROBOT_LOG( TRUE, "**************************************************************************\n" )

		/// g_DepthCameraSubSystemStatus = SUBSYSTEM_FAILED;
		/// SpeakText( "Warning, depth camera initialization has failed");
		return 0; // no sense wasting cycles, the MMF did not init correctly!
	}


	#if (SHOW_XYZ_WINDOW)
		cvNamedWindow(CAMERA_WINDOW_NAME_DEPTH_CAMERA_DEPTH, CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback( CAMERA_WINDOW_NAME_DEPTH_CAMERA_DEPTH, OnDepthWindowMouseEvent, (void*)pDepthCameraModule);
		//cvMoveWindow( CAMERA_WINDOW_NAME_DEPTH_CAMERA_DEPTH, WindowPosX(m_pDepthDisplayFrame), PositionY );
	#endif
	/// g_Camera[DEPTH_CAMERA_DEPTH].State = CAMERA_STATE_INITIALIZED;

	///////////////////////////////////////////////////////////////////////////////////
	// THREAD LOOP
	// gets frames when they are posted by the C# app into shared memory, when the event is signaled
	while( g_bRunThread && g_bRunDepthCameraThread && (WAIT_OBJECT_0 == WaitForSingleObject(g_hDepthFrameReadyEvent, INFINITE)) )
	{
		//__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshDepthCameraThreadLoop);
		__itt_marker(pDomainDepthCameraThread, __itt_null, pshDepthCameraThreadLoop, __itt_marker_scope_task);

		// Get Depth Frame from Shared Memory and convert to 3D Cloud
		__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshDepthCameraGetDepthImage);
			pDepthCameraModule->GetDepthImage(); 
		__itt_task_end(pDomainDepthCameraThread);

		// Update 2D map with wall and object locations
		if( 1 == DEPTH_CAMERA_INSTALLED_IN_HEAD )
		{
			if( pDepthCameraModule->m_pHeadControl->GetTiltPosition() < 0 )				
			{
				// only enable if in position to see ahead
				__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csFindWallsAnd2dMaps);
					pDepthCameraModule->FindWallsAnd2dMaps();
				__itt_task_end(pDomainDepthCameraThread); // psh_csFindWallsAnd2dMaps

				// Update global summary of objects seen for object avoidance
				__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csUpdateDepthCameraObjectSummary);
					pDepthCameraModule->UpdateDepthCameraObjectSummary();
				__itt_task_end(pDomainDepthCameraThread); // psh_csUpdateDepthCameraObjectSummary
			}
			else
			{
			}
		}
		// Find objects on floor, if enabled by other modules
		if( ROBOT_TYPE == LOKI )
		{
			pDepthCameraModule->FindObjectsOnFloor();
		}

		#if (SHOW_XYZ_WINDOW)
			pDepthCameraModule->ShowDepthFrame( );		
		#endif

		if( ROBOT_TYPE == LOKI )
		{
			/*
			if( 0 != g_CurrentHumanTracked )
			{
				SendCommand( WM_ROBOT_CAMERA_LOOK_AT_PLAYER_CMD, (DWORD)(0), (DWORD)SERVO_SPEED_MED );
			}
			*/
		}
		//__itt_task_end(pDomainDepthCameraThread);  // pshDepthCameraThreadLoop

	}

	//////////////////////////////////////////////////////////////////////////////////////
	ROBOT_LOG( TRUE,"DepthCamera Thread exiting.\n")
	#if (SHOW_XYZ_WINDOW)
		cvDestroyWindow( CAMERA_WINDOW_NAME_DEPTH_CAMERA_DEPTH );
	#endif

	return 0;
}



///////////////////////////////////////////////////////////////////////////////
//	MODULE: CDepthCameraModule
//  NOTE: Creates its own video capture thread
///////////////////////////////////////////////////////////////////////////////


 CDepthCameraModule::CDepthCameraModule( CDriveControlModule *pDriveControlModule )
{
	ROBOT_LOG( TRUE, "DepthCamera constructor starting" )

	m_pDriveCtrl = pDriveControlModule;
	m_CurrentTask = TASK_NONE; 
	m_TaskState = 1;
	m_TrackObjectX = 0;
	m_TrackObjectY = 0;
	gDepthCameraDelayTimer = 0;
	m_FindObjectsOnFloorTrys = 0;
	m_nDepthCamera3DObjectsFound = -1; // -1 == Result not ready
	m_DepthCameraScanPosition = 0;
	m_FindCloseObjectsOnly = FALSE;

	m_FrameNumber = 0;
#if (SHOW_XYZ_WINDOW)
	m_pDebugZFrame = 0;
#endif
	m_DisplaySize.width = DEPTH_WINDOW_DISPLAY_SIZE_X;
	m_DisplaySize.height = DEPTH_WINDOW_DISPLAY_SIZE_Y;
#if( 1 != DEPTH_CAMERA_INSTALLED_IN_HEAD )
	m_pDepthCameraServoControl = new DepthCameraServoControl;
#endif
	m_pHeadControl = new HeadControl();

	// Memory Mapped File
	m_hMapFile = INVALID_HANDLE_VALUE;
	m_pDepthFrameSharedMemory = NULL;	// Shared Buffer space LPCTSTR
	m_bDepthCameraSharedMemoryOpened = FALSE;

	// Depth Data
	memset( &m_DepthFrameInfo, 0x00, sizeof( DEPTH_CAMERA_FRAME_HEADER_T ) );	// Frame Header
	m_pDepthFrameData = new unsigned short [MAX_DEPTH_DATA_SIZE];				// Frame Data
	memset( m_pDepthFrameData, 0x00, MAX_DEPTH_DATA_SIZE );
	m_DepthCameraCommand.ControlFlags = DepthCameraControlFlag_None;			// Control flags back to depth camera app

	m_ServoMoving = FALSE;
	m_BlurSettleTime = 0;

	// Processing variables
	m_ObjectTrackSize.width = 0;
	m_ObjectTrackSize.height = 0;
	m_ObjectTrackingCenter.x = 0;
	m_ObjectTrackingCenter.y = 0;

	m_OpenCVDepthMousePoint.x = 0;
	m_OpenCVDepthMousePoint.y = 0;
	m_DepthCameraWindowMousePointSelected = FALSE;
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

	//__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
	//ROBOT_LOG( TRUE,"DepthCamera Init: EnterCriticalSection g_csDepthCameraPointCloudLock\n")
	//EnterCriticalSection(&g_csDepthCameraPointCloudLock);

	g_DepthCameraPointCloud = new DEPTH_3D_CLOUD_T;
	g_DepthCameraPointCloud->dwTimeOfSample = 0;
	g_DepthCameraPointCloud->RobotLocation.x = 0.0;	// Location of robot at the time of depth snapshot
	g_DepthCameraPointCloud->RobotLocation.y = 0.0;
	g_DepthCameraPointCloud->CompassHeading = 0;		// Heading of robot at the time of depth snapshot
	for( int i=0; i < DEPTH_CAPTURE_SIZE_MAX_Y; i++ )
	{
		for( int j=0; j < DEPTH_CAPTURE_SIZE_MAX_X; j++ )
		{
			g_DepthCameraPointCloud->Point3dArray[i][j].X = 0;
			g_DepthCameraPointCloud->Point3dArray[i][j].Y = 0;
			g_DepthCameraPointCloud->Point3dArray[i][j].Z = 0;
		}
	}
	for( int i=0; i < DEPTH_CAPTURE_SIZE_MAX_X; i++ )
	{
		g_DepthCameraPointCloud->WallPoints[i].X = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
		g_DepthCameraPointCloud->WallPoints[i].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
	}

	//LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
	//__itt_task_end(pDomainControlThread);
	//ROBOT_LOG( TRUE,"DepthCamera Init: LeaveCriticalSection g_csDepthCameraPointCloudLock\n")

	m_pDepthCameraTempObjects3D = new TEMP_OBJECT_3D_ARRAY_T;
	memset( m_pDepthCameraTempObjects3D, 0x00, sizeof( TEMP_OBJECT_3D_ARRAY_T ) );


	// FOR DEBUG ONLY - KLUDGE
	#if DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1
		g_pDepthCameraDebugSliceArray = new DEBUG_SLICE_ARRAY_T;
		memset( g_pDepthCameraDebugSliceArray, 0x00, sizeof( DEBUG_SLICE_ARRAY_T ) );
	#endif

	//m_pDepthFrame = cvCreateImage( FrameSize, 8, 3 );


	// Create the DepthCamera video capture thread
	g_bRunDepthCameraThread = TRUE; // When FALSE, tells thread to exit
	DWORD dwTempThreadId;
	g_hDepthCameraThread = ::CreateThread( NULL, 0, DepthCameraDepthThreadProc, (LPVOID)this, 0, &dwTempThreadId );
	ROBOT_LOG( TRUE, "Created DepthCamera Thread. ID = (0x%x)", dwTempThreadId )

	ROBOT_LOG( TRUE, "DepthCamera construstor complete" )

}

///////////////////////////////////////////////////////////////////////////////
CDepthCameraModule::~CDepthCameraModule()
{
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CDepthCameraModule.  Waiting for DepthCamera Thread to exit...")
	g_bRunDepthCameraThread = FALSE; // Tell the thread to exit
	if( INVALID_HANDLE_VALUE != g_hDepthCameraThread) 
	{
		WaitForSingleObject( g_hDepthCameraThread, INFINITE );
		CloseHandle( g_hDepthCameraThread );
	}
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CDepthCameraModule.  DepthCamera Thread exit complete.\n")
	#if DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1
		SAFE_DELETE( g_pDepthCameraDebugSliceArray ); // TODO - WHY DOES THIS CORRUPT THE HEAP?
	#endif
	SAFE_DELETE( m_pDepthCameraTempObjects3D );
	SAFE_DELETE( g_DepthCameraPointCloud );
	#if( 1 != DEPTH_CAMERA_INSTALLED_IN_HEAD )
		SAFE_DELETE( m_pDepthCameraServoControl );
	#endif
	SAFE_DELETE( m_pHeadControl );

	SAFE_DELETE( m_pDepthFrameData );

	// release Mapped File resources
	// - TODO-MUST DEBUGGING CRASH - REENALBE THIS!!!!!

	if( NULL != m_pDepthFrameSharedMemory )
	{
		if( UnmapViewOfFile(m_pDepthFrameSharedMemory) )
		{
			ROBOT_LOG( TRUE,  "ERROR Unmapping m_pDepthFrameSharedMemory!  Return Code = %04X\n", GetLastError() )
		}
	}
	if( INVALID_HANDLE_VALUE != m_hMapFile)
	{
		CloseHandle(m_hMapFile);
	}

	m_pDepthFrameSharedMemory = NULL;
	m_hMapFile = INVALID_HANDLE_VALUE;
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CDepthCameraModule done.\n")
}


//////////////////////////////////////////
// OpenMemoryMappedFile
// Initialize shared memory for getting frames from the C# code

BOOL CDepthCameraModule::OpenMemoryMappedFile()
{
	// Create an event for synchronizing between the apps
	ROBOT_LOG( TRUE,  "Waiting for Depth Camera App to start...\n" )

	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 
	g_hDepthFrameReadyEvent = CreateEvent ( NULL, bManualReset, bInitialState, DEPTH_DATA_AVAILABLE_EVENT_NAME );
	if ( !g_hDepthFrameReadyEvent ) 
	{ 
		ROBOT_LOG( TRUE,  "DepthCamera Event creation failed!:  DEPTH_DATA_AVAILABLE_EVENT_NAME\n" )
		return FALSE;
	}

	// Now wait for the event to be signaled by the Depth Camera process, indicating that it's ok to proceed
	const DWORD msTimeOut = 30000; // up to 30 seconds, to allow DepthCamera to power up!
	DWORD dwResult = WaitForSingleObject(g_hDepthFrameReadyEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		ROBOT_LOG( TRUE,  "Event Timed out or failed!:  DEPTH_DATA_AVAILABLE_EVENT_NAME, Error: %08X\n", dwResult )
		return FALSE;
	}
	ROBOT_LOG( TRUE,  "Depth Camera App start Success!\n" )

	TCHAR szDepthCameraDataSharedFileName[]=TEXT(DEPTH_CAMERA_DATA_SHARED_FILE_NAME);

	m_hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		szDepthCameraDataSharedFileName);	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == m_hMapFile) || (NULL == m_hMapFile)  )
	{ 
		ROBOT_LOG( TRUE,  "Could not open DepthCamera Depth file mapping object %s Error(%d).\n", szDepthCameraDataSharedFileName, GetLastError())
		return FALSE;
	}

	m_pDepthFrameSharedMemory = (int*)MapViewOfFile(m_hMapFile, // handle to map object (LPCTSTR)
		FILE_MAP_ALL_ACCESS,  // read/write permission
		0,                    
		0,                    
		(sizeof(DEPTH_CAMERA_FRAME_T)) );                   

	if (m_pDepthFrameSharedMemory == NULL) 
	{ 
		ROBOT_LOG( TRUE,  "Could not map view of file (%d).\n", GetLastError());
		CloseHandle(m_hMapFile);
		return FALSE;
	}

	m_bDepthCameraSharedMemoryOpened = TRUE;
	ROBOT_DISPLAY( TRUE, "DepthCamera Depth Shared Memory File Opened Sucessfully!" )
	return TRUE; // Success!

}

void CDepthCameraModule::ShowDepthFrame()
{	
#if ( USE_OPEN_CV == 1 )
	if( !m_pDebugZFrame )
	{
		return; // probably shutting down
	}
	// For DEBUG, draw the slices detected
	#if (DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1)
		ROBOT_ASSERT( g_pDepthCameraDebugSliceArray );
		ROBOT_ASSERT( g_pDepthCameraDebugSliceArray->nSlices < MAX_DEBUG_SLICES );
		for( int SliceItem =0; SliceItem < g_pDepthCameraDebugSliceArray->nSlices; SliceItem++ )
		{
			cvLine(m_pDebugZFrame, g_pDepthCameraDebugSliceArray->Slice[SliceItem].Pt1,  
				g_pDepthCameraDebugSliceArray->Slice[SliceItem].Pt2, CV_RGB(255,255,0), 1, 4, 0);
		}
	#endif
	
	// Draw bounding boxes around any objects detected
	if( g_pDepthCameraObjects3D->nObjectsDetected > 0)
	{
		//int i = 0;
		for( int i = 0; i < g_pDepthCameraObjects3D->nObjectsDetected; i++ )
		{
			CvPoint pt1 = { (g_pDepthCameraObjects3D->Object[i].LeftPixel), (g_pDepthCameraObjects3D->Object[i].StartScanLine) };
			CvPoint pt2 = { (g_pDepthCameraObjects3D->Object[i].RightPixel), (g_pDepthCameraObjects3D->Object[i].EndScanLine) };
			cvRectangle(m_pDebugZFrame, pt1, pt2, CV_RGB(255,0,255), 1 );
		}
	}

	// DEPTH_CAMERA_WALL_SCAN_ZONE
	CvPoint pt1, pt2;
	pt1.y = DEPTH_CAMERA_WALL_SCAN_ZONE;
	pt1.x = 0;
	pt2.y = DEPTH_CAMERA_WALL_SCAN_ZONE;
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
	if( m_CaptureSize.width != m_DisplaySize.width ) //USE THIS: m_DepthFrameInfo.Height
	{
		// Scale down 640x480 to fit on display (easier on remote desktop too)
		cvResize( m_pDebugZFrame, m_pDebugZFrame  ); // CV_INTER_CUBIC
		//	cvResizeWindow( CAMERA_WINDOW_NAME_LEFT, 320, 240 );
	}
	if( g_Camera[DEPTH_CAMERA_DEPTH].Flip )
	{
		// Flip video horizontally
		cvFlip( m_pDebugZFrame, 0, 1 );
	}
*/

	// Flip video horizontally
	cvFlip( m_pDebugZFrame, 0, 1 );
	cvShowImage( CAMERA_WINDOW_NAME_DEPTH_CAMERA_DEPTH, m_pDebugZFrame );
	// KLUDGE!  REQURED DUE TO BUG IN OPENCV.		
	char c = (char)cvWaitKey(1); // NEED cvWaitKey() OR VIDEO DOES NOT DISPLAY!
	if( c == 27 )	ROBOT_LOG( TRUE,"ESC KEY Pressed!\n")

#endif // ( USE_OPEN_CV == 1 )
}


//////////////////////////////////////////
// GetDepthImage
// Get depth data from Shared Memory and convert to 3D point cloud

void CDepthCameraModule::GetDepthImage()
{
	// TODO? Lock the 3DPointCloud

	// get data from Shared Memory
	if( NULL == m_pDepthFrameSharedMemory )
	{
		m_DepthFrameInfo.FrameNumber = -1; // bad frame, don;t use
		return;
	}

	// Copy from shared memory
	CopyMemory( &m_DepthFrameInfo, (PVOID)m_pDepthFrameSharedMemory, sizeof(DEPTH_CAMERA_FRAME_HEADER_T) );
	PVOID pSharedDepthData = (PVOID)( (unsigned char*)m_pDepthFrameSharedMemory + (sizeof(DEPTH_CAMERA_FRAME_HEADER_T)) ); 
	// do a fast copy from shared memory to minimize overwrite possibility
	CopyMemory( m_pDepthFrameData, pSharedDepthData, (m_DepthFrameInfo.Width * m_DepthFrameInfo.Height * 2) );

	

	#if DEBUG_SHARED_MEMORY == 1
		ROBOT_LOG( TRUE,  "m_DepthFrameInfo:  Frame=%4d, H=%d, W=%d, MaxDepth=%d\n", 
			m_DepthFrameInfo.FrameNumber, m_DepthFrameInfo.Height, m_DepthFrameInfo.Width, m_DepthFrameInfo.tooFarDepth )
	#endif

	#if (SHOW_XYZ_WINDOW)
		if( 0 == m_pDebugZFrame )
		{
			// first frame
			CvSize FrameSize; FrameSize.height = m_DepthFrameInfo.Height;	FrameSize.width = m_DepthFrameInfo.Width;
			m_pDebugZFrame = cvCreateImage( FrameSize, IPL_DEPTH_8U, 3 );
			if( !m_pDebugZFrame ) ROBOT_ASSERT(0);
			m_pDebugZFrame->origin = 0;
		}
	#endif

	// Clear flags for controlling the depth camera app.  These will be overridden as needed below.
	m_DepthCameraCommand.ControlFlags = DepthCameraControlFlag_None; 

	// Convert depth data and map to Depth frame
	// Starts at top of frame (y=0) and scans to bottom of frame (y=FrameInfo->Height)
	// So, top of frame is farthest from the robot!

	const int tooNearDepth = 0;
	const int unknownDepth = -1;
	//int FrameWidth = m_DepthFrameInfo.Width;
	//int FrameHeight = m_DepthFrameInfo.Height;
	int tooFarDepth = m_DepthFrameInfo.tooFarDepth;
	int  MouseX = 0;
	int  MouseY = 0;
//	POINT2D_T Players[DEPTH_CAMERA_MAX_HUMANS_TO_TRACK];
//	int nPlayersFound = 0;
	m_DepthCameraWindowMousePointSelected = FALSE;
	if( 1 == m_DepthFrameInfo.MouseDown )
	{
		m_DepthCameraWindowMousePointSelected = TRUE;
		MouseX = m_DepthFrameInfo.MouseX;
		MouseY = m_DepthFrameInfo.MouseY;
		m_DepthFrameInfo.MouseDown = 0; // Reset the shared memory flag for next time
	}

	// Calculate degrees per pixel for the image
	double DegreeIncrementX = DEPTH_CAMERA_DEPTH_FOV_X / (double)m_DepthFrameInfo.Width;
	double DegreeIncrementY = DEPTH_CAMERA_DEPTH_FOV_Y / (double)m_DepthFrameInfo.Height;

	// Get DepthCamera Tilt angle, and position offset caused by tilt
	double DepthCameraTiltTenthDegrees =	0;
	// TODO?  if( ROBOT_HAS_DEPTH_CAMERA_SERVO )
	{
		/// DepthCameraTiltTenthDegrees = g_BulkServoStatus[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees;
	}
	// for debug on a PC
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		DepthCameraTiltTenthDegrees = -300; // simulate as if Arduino pointing to ground
	}

	// Handle mouse in OpenCV window (for Z debug)
	if( m_OpenCVDepthMousePointSelected )
	{
		// Scale as needed.  Mouse coordinates are always 640x480 after scaling ??
		UINT Ratio = 1; // TODO? m_CaptureSize.width / m_DisplaySize.width; USE THIS: m_DepthFrameInfo.Height
		/*if( g_Camera[DEPTH_CAMERA_DEPTH].Flip )
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



	if( NULL == g_DepthCameraPointCloud )
	{
		ROBOT_ASSERT(0); // Not initilized!
		return;
	}
	g_DepthCameraPointCloud->FrameSizeX = m_DepthFrameInfo.Width;
	g_DepthCameraPointCloud->FrameSizeY = m_DepthFrameInfo.Height;

		
	// Keep a local array of humans to track in this frame (will update the global at the end)
	/** Not used with Depth Camera execpt Kinect
	DEPTH_CAMERA_HUMAN_FINDING_T HumanLocationFinding[DEPTH_CAMERA_MAX_HUMANS_TO_TRACK]; // Location of humans being tracked
	memset( HumanLocationFinding, 0x00, (sizeof(DEPTH_CAMERA_HUMAN_FINDING_T) * DEPTH_CAMERA_MAX_HUMANS_TO_TRACK) );

	DEPTH_CAMERA_HUMAN_TRACKING_T HumanLocationTracking[DEPTH_CAMERA_MAX_HUMANS_TO_TRACK]; // Location of humans being tracked
	memset( HumanLocationTracking, 0x00, (sizeof(DEPTH_CAMERA_HUMAN_TRACKING_T) * DEPTH_CAMERA_MAX_HUMANS_TO_TRACK) );

	// From Microsoft.DepthCamera:
	const int PlayerIndexBitmask = 7;
	const int PlayerIndexBitmaskWidth = 3;
	int ClosestPlayerDistance = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;
	int ClosestPlayer = 0;
	BOOL CurrentHumanTrackedFound = FALSE;

	**/

	unsigned short* pDepthBuffer = m_pDepthFrameData;

// TEST DEBUG CODE
			m_DepthCameraCommand.ControlFlags |= DepthCameraControlFlag_DisplayBoundingBox; // Draw a bounding box
			m_DepthCameraCommand.BoundingBoxBottom = 100;
			m_DepthCameraCommand.BoundingBoxTop = 150;
			m_DepthCameraCommand.BoundingBoxLeft = 100;
			m_DepthCameraCommand.BoundingBoxRight = 150;
			 
			// Send command to the depth camera app via shared memory
			if( (NULL != g_pDepthCameraCommandSharedMemory) && (NULL != g_hDepthCameraCommandEvent) && 
				(m_DepthCameraCommand.ControlFlags != DepthCameraControlFlag_None)    )
			{
				CopyMemory((PVOID)g_pDepthCameraCommandSharedMemory, &m_DepthCameraCommand, (sizeof(DEPTH_CAMERA_COMMAND_T)));
				SetEvent( g_hDepthCameraCommandEvent );  // Tell Kobuki app that a new command is pending
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR: Cant send Bounding Box command to Depth Camera App!  Did you have AUTO_LAUNCH_DEPTH_CAMERA_APP enabled?\n" )
			}



	int LowestZValue = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX; // track the lowest value to normalize floor


	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshDepthCameraFrameLoop);
	for( int y = 0; y < m_DepthFrameInfo.Height; y++ )
	{
		LowestZValue = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX; // track the lowest value in each scan line to normalize floor
		for( int x = 0; x < m_DepthFrameInfo.Width; x++ )
		{
            ///int player = (int)(*pDepthBuffer & PlayerIndexBitmask);
            int DepthValueMM = (int)(*pDepthBuffer); // >> PlayerIndexBitmaskWidth); // Depth value in millimeters
			pDepthBuffer++;

			#if DEBUG_SHARED_MEMORY == 1
				if( (x < 32) && (y == 0 ) )
				{
					TRACE("%d, ", DepthValueMM);
				}

			#endif

			//////////////////////////////////////////////////////////////////////////////////////////
			// Update the 3D point cloud
			// Calculate X,Y,Z based upon tilt of DepthCamera and FOV of the DepthCamera Camera
			// x, y = pixel location
			int X=DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX,Y=DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX,Z=DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;
			double TempX=0,TempY=0,TempZ=0,m=0,n=0,q=0,u=0, RangeTenthInches = 0;

			if( m_DepthCameraWindowMousePointSelected || m_OpenCVDepthMousePointSelected )
			{
				if( (x == MouseX) && (y == MouseY) )
				{
					ROBOT_LOG( TRUE,"MOUSE DOWN AT %d, %d ", MouseX, MouseY )
				}
			}

			// TRAP BAD VALUES HERE!!! (too near, too far, etc.)
			if( (DepthValueMM >= DEPTH_CAMERA_MM_MAX) || (DepthValueMM <= 0) || (tooFarDepth == DepthValueMM) )
			{
				//ROBOT_LOG( TRUE,"DepthCamera Read Zero: EnterCriticalSection g_csDepthCameraPointCloudLock\n")
				//__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
				EnterCriticalSection(&g_csDepthCameraPointCloudLock);
					g_DepthCameraPointCloud->Point3dArray[y][x].X = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
					g_DepthCameraPointCloud->Point3dArray[y][x].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
					g_DepthCameraPointCloud->Point3dArray[y][x].Z = LASER_RANGEFINDER_TENTH_INCHES_ERROR;
				LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
				//__itt_task_end(pDomainDepthCameraThread);
				//ROBOT_LOG( TRUE,"DepthCamera Read Zero: LeaveCriticalSection g_csDepthCameraPointCloudLock\n")
			}
			else
			{
				RangeTenthInches = ((double)DepthValueMM / 2.540) ;	

				double StepX = (double)x - ((double)m_DepthFrameInfo.Width/2.0);	// goes from (-320 to + 320)
				double StepAngleX = DegreeIncrementX / 1.5 * StepX; // goes from (-46 to + 46 degrees)

				double StepY = ( ((double)m_DepthFrameInfo.Height/2.0) - (double)y );	// goes from (+240 to -240)
				//double StepAngleY =  DegreeIncrementY * StepY; // goes from (-38.6 to + 38.6 degrees)
				double StepAngleY =  (DepthCameraTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.6 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
				double StepAngleYZ =  (DepthCameraTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.8 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
				//double StepAngleY =  (DepthCameraTiltTenthDegrees/10.0); // goes from (-38.6 to + 38.6 degrees) + Tilt angle

				// Get x,y,z from DepthCamera's perspective. zero = straight forward facing.  Down = negative
				// Convert from Spherical Coordinates to X,Y,Z coordinates for each point
				TempX = RangeTenthInches * sin(DEGREES_TO_RADIANS*(StepAngleX) );
				TempY = RangeTenthInches * cos(DEGREES_TO_RADIANS*StepAngleY);
				TempZ = RangeTenthInches * sin(DEGREES_TO_RADIANS*StepAngleYZ * 1.1);

				// Now, translate origin from DepthCamera position to center of robot at floor.
				X = (int)(-TempX);	 // For Loki, Positive X is the Right side of the robot
				Y = (int)(TempY - (double)DEPTH_CAMERA_DISTANCE_FROM_FRONT_TENTH_INCHES);
				Z = (int)(TempZ + (double)DEPTH_CAMERA_HEIGHT_ABOVE_GROUND_TENTH_INCHES);

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
				//ROBOT_LOG( TRUE,"DepthCamera Set: EnterCriticalSection g_csDepthCameraPointCloudLock\n")
				//__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
				if( NULL == g_DepthCameraPointCloud ) return; // TODO Kludge for shutdown crash.  Need better solution.
				EnterCriticalSection(&g_csDepthCameraPointCloudLock);
					g_DepthCameraPointCloud->Point3dArray[y][x].X = (int)X;
					g_DepthCameraPointCloud->Point3dArray[y][x].Y = (int)Y;
					g_DepthCameraPointCloud->Point3dArray[y][x].Z = (int)Z;
				LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
				//__itt_task_end(pDomainDepthCameraThread);
				//ROBOT_LOG( TRUE,"DepthCamera Set: LeaveCriticalSection g_csDepthCameraPointCloudLock\n")




				#if ( TRACK_TOP_OF_USERS_HEAD == 1 )
					// Keep track of the highest point on the player detected in this frame (used with range data to determine player's height!)
					// This will be combined with other data later, when we know how many unique humans there are (we don't know at this point)
					if( (0 != player) && (DepthCameraTiltTenthDegrees > DEPTH_CAMERA_TILT_LOOKING_AT_FLOOR) )
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
							// (sometimes DepthCamera gets confused and thin
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
			if( m_DepthCameraWindowMousePointSelected )
			{
				if( (x == MouseX) && (y == MouseY) )
				{
					ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:   Range=%4.1f, Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x,(RangeTenthInches/10.0), Y/10.0, X/10.0, Z/10.0  )
					m_DepthCameraWindowMousePointSelected = FALSE;
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
		if( DepthCameraTiltTenthDegrees <= DEPTH_CAMERA_TILT_LOOKING_AT_FLOOR ) // only enable if in position to see ahead
		{			
			if(LowestZValue < -70)
			{
				//ROBOT_LOG( TRUE,"ERROR! Lowest Z value < -7 inches! (%d)\n", LowestZValue);				
			}
			else if(LowestZValue < 60) // only do lines where floor is seen
			{
				EnterCriticalSection(&g_csDepthCameraPointCloudLock);
				for( int x = 0; x < m_DepthFrameInfo.Width; x++ )
				{
					int before = g_DepthCameraPointCloud->Point3dArray[y][x].Z;
					if( DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX != g_DepthCameraPointCloud->Point3dArray[y][x].Z )
					{
						g_DepthCameraPointCloud->Point3dArray[y][x].Z = g_DepthCameraPointCloud->Point3dArray[y][x].Z - LowestZValue;
					}
					int after = g_DepthCameraPointCloud->Point3dArray[y][x].Z;
					if( m_OpenCVDepthMousePointSelected )
					{
						if( (x == MouseX) && (y == MouseY) )
						{
							ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:  Y=%4.1f, X=%4.1f, Z=%4.1f inches\n", y, x, 
								(double)(g_DepthCameraPointCloud->Point3dArray[y][x].Y)/10.0,
								(double)(g_DepthCameraPointCloud->Point3dArray[y][x].X)/10.0,
								(double)(g_DepthCameraPointCloud->Point3dArray[y][x].Z)/10.0  )
							ROBOT_LOG( TRUE,"   LowestZValue = %d, beforeZ = %d, afterZ = %d\n", LowestZValue, before, after )
							m_OpenCVDepthMousePointSelected = FALSE;
						}
					}

				}
				LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
			}
		}

	} // for y
	__itt_task_end(pDomainDepthCameraThread); // frameloop


	#if DEBUG_SHARED_MEMORY == 1
		TRACE("Last = %d\n", lastValue );
	#endif


//	__itt_task_end(pDomainDepthCameraThread);
}

RGBQUAD CDepthCameraModule::ZValueToColor( int Zvalue ) // for debugging Z Depth on floor
{
	RGBQUAD Pixel;
	if( DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX == Zvalue )
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
void CDepthCameraModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{

	CString MsgString;
	//BYTE nDistHigh, nDistLow;
	switch( uMsg )
	{
		// Process DepthCamera commands from the GUI or command threads

		case WM_ROBOT_DEPTH_CAMERA_CANCEL_CMD:
		{
			// cancel any current action. Called for example, when user says "stop".
			m_CurrentTask = TASK_NONE;
			m_TaskState = 1;
			return;
		}

		case WM_ROBOT_DEPTH_CAMERA_SEARCH_FLOOR_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Start state machine for searching the floor
			// wParam indicates close search only or full search of floor
			if( 0 == wParam )
			{
				ROBOT_LOG( TRUE,"DEPTH_CAMERA_SEARCH_FLOOR_CMD: Searching for Near and Far objects\n")
				m_FindCloseObjectsOnly = FALSE;
			}
			else
			{				
				ROBOT_LOG( TRUE,"DEPTH_CAMERA_SEARCH_FLOOR_CMD: Searching for Close objects only\n")
				m_FindCloseObjectsOnly = TRUE;
			}
			m_CurrentTask = DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT;
			m_TaskState = 1;	// go to first state
			return;
		}

		case WM_ROBOT_DEPTH_CAMERA_TRACK_OBJECT_CMD:
		{
			g_bCmdRecognized = TRUE;
			if( 0 == wParam )
			{
				// Stop Tracking, and go back to looking for Humans
				m_CurrentTask = TASK_NONE;
				m_TaskState = 1;
			}
			else
			{
				// ASSUME (for now), that we want to track the closest object that was previously found
				// Start state machine for tracking object
				m_CurrentTask = DEPTH_CAMERA_TASK_TRACK_CLOSEST_OBJECT;
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
			// Check status of DepthCamera Tasks
			if( 0 != gDepthCameraDelayTimer )
			{
				//ROBOT_LOG( TRUE,"WAITING FOR gDepthCameraDelayTimer = %d\n", gDepthCameraDelayTimer)
				break;
			}

			switch( m_CurrentTask )
			{
				case TASK_NONE:
				{
					break;	// Nothing to do
				}
				case DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT:
				{	
					switch( m_TaskState )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// begin with scan close to robot
						{	
							#if( 1 == DEPTH_CAMERA_INSTALLED_IN_HEAD )
								ROBOT_LOG( TRUE,"DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT: Moving Head to position %d\n", m_DepthCameraScanPosition)
								#if DEBUG_DEPTH_CAMERA_FLOOR_OBJECT_DETECTION	== 1
									m_DepthCameraScanPosition = 2;  //******** FAR SCAN DEBUG ONLY !!!
								#else		
									m_DepthCameraScanPosition = DEPTH_CAMERA_SERVO_POSITION_CLOSE_SCAN;
								#endif
								ROBOT_LOG( TRUE,"DEPTH_CAMERA Scan Position %d\n", m_DepthCameraScanPosition)
								m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, (CAMERA_TILT_CENTER + CAMERA_TILT_CENTER+DEPTH_CAMERA_IN_HEAD_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition]), CAMERA_SIDETILT_CENTER ); // Pan, Tilt, SideTilt TENTHDEGREES
							#else
								#error DEPTH SENSOR NOT IN HEAD NOT IMPLEMENTED
								//ROBOT_LOG( TRUE,"DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT: Moving DepthCamera to position %d\n", m_DepthCameraScanPosition)
								#if DEBUG_DEPTH_CAMERA_FLOOR_OBJECT_DETECTION	== 1
									m_DepthCameraScanPosition = 2;  //******** FAR SCAN DEBUG ONLY !!!
								#else		
									m_DepthCameraScanPosition = DEPTH_CAMERA_SERVO_POSITION_CLOSE_SCAN;
								#endif
								ROBOT_LOG( TRUE,"DEPTH_CAMERA Scan Position %d\n", m_DepthCameraScanPosition)
								m_pDepthCameraServoControl->SetTiltPosition( DEPTH_CAMERA_TILT_OWNER_TRACK_OBJECT, DEPTH_CAMERA_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition] );
							#endif
							}
							m_TaskState++;
							break;
						}
						case 2:	// Wait for Servo to reach commanded position
						{	
							#if( 1 == DEPTH_CAMERA_INSTALLED_IN_HEAD )
								ROBOT_LOG( TRUE, "DEBUG DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT : CheckServoPosition ")
								//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - Arduino\n")
								//else if( WM_ROBOT_SERVO_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - SERVO\n")

								int ServoStatus = m_pHeadControl->CheckHeadPosition(FALSE);
								if ( DEPTH_CAMERA_SERVO_SUCCESS == ServoStatus )
								{
									// Head in position, go to next state (DepthCamera should have grabed a frame and scanned it by then)
									gDepthCameraDelayTimer = 5; // tenth-seconds - Wait for image to settle before checking 3D
									m_TaskState++;	
								}
								else if( DEPTH_CAMERA_SERVO_TIMED_OUT == ServoStatus )
								{
									ROBOT_LOG( TRUE,"ERROR: HEAD SERVO TIME OUT WHILE LOOKING FOR OBJECT!  ABORTING!\n\n")
									SendCommand( WM_ROBOT_DEPTH_CAMERA_SEARCH_COMPLETE, 0, 0 ); // no objects found
									// Stop Tracking, and go back to looking for Humans
									m_CurrentTask = TASK_NONE;
									m_TaskState = 1;
								}
								// else DEPTH_CAMERA_SERVO_MOVING == ServoStatus

							#else
								#error DEPTH SENSOR NOT IN HEAD NOT IMPLEMENTED
								//ROBOT_LOG( TRUE, "DEBUG DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT : CheckServoPosition ")
								//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - Arduino\n")
								//else if( WM_ROBOT_SERVO_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - SERVO\n")

								int ServoStatus = m_pDepthCameraServoControl->CheckServoPosition(FALSE);
								if ( DEPTH_CAMERA_SERVO_SUCCESS == ServoStatus )
								{
									// DepthCamera in position, go to next state (DepthCamera should have done a scan by then)
									gDepthCameraDelayTimer = 5; // tenth-seconds - Wait for image to settle before checking 3D
									m_TaskState++;	
								}
								else if( DEPTH_CAMERA_SERVO_TIMED_OUT == ServoStatus )
								{
									ROBOT_LOG( TRUE,"ERROR: DEPTH_CAMERA SERVO TIME OUT!  ABORTING!\n\n")
									SendCommand( WM_ROBOT_DEPTH_CAMERA_SEARCH_COMPLETE, 0, 0 ); // no objects found
									// Stop Tracking, and go back to looking for Humans
									m_CurrentTask = DEPTH_CAMERA_TASK_HUMAN_DETECTION;
									m_TaskState = 1;
								}
								// else DEPTH_CAMERA_SERVO_MOVING == ServoStatus

							#endif
							break; // keep waiting
						}
						case 3:	// Request 3D analysis at current position
						{
							ROBOT_LOG( TRUE,"DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT: Looking for Object\n")
							FindObjectsOnFloorRequest( DEPTH_CAMERA_FIND_OBJECTS_REQUEST_RETRIES );	// Try "n" times
							m_TaskState++;	
							break;
						}
						case 4:	// Check results at current DepthCamera Tilt Position
						{
							if( -1 == m_nDepthCamera3DObjectsFound )
							{
								// DepthCamera is still looking.  Just wait...
								//ROBOT_LOG( TRUE,"DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT Waiting...\n")
							}
							else
							{
								
								// if objects found, we're done, else move depth camera anagle and try again
								if( (0 == m_nDepthCamera3DObjectsFound) && (m_DepthCameraScanPosition < DEPTH_CAMERA_SERVO_POSITION_FAR_SCAN) && (!m_FindCloseObjectsOnly) )
								{
									// Move DepthCamera to next position and try again
									m_DepthCameraScanPosition++;
									ROBOT_LOG( TRUE,"DEPTH_CAMERA Scan Position %d\n", m_DepthCameraScanPosition)
									#if( 1 == DEPTH_CAMERA_INSTALLED_IN_HEAD )
										m_pHeadControl->SetHeadPosition( HEAD_OWNER_BEHAVIOR_P1, CAMERA_PAN_CENTER, (CAMERA_TILT_CENTER+DEPTH_CAMERA_IN_HEAD_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition]), CAMERA_SIDETILT_CENTER ); // Pan, Tilt, SideTilt TENTHDEGREES
									#else
										#error DEPTH SENSOR NOT IN HEAD NOT IMPLEMENTED
										m_pDepthCameraServoControl->SetTiltPosition( DEPTH_CAMERA_TILT_OWNER_TRACK_OBJECT, DEPTH_CAMERA_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition] );
									#endif
									m_TaskState = 2; // Move DepthCamera to next position and try again
								}
								else
								{
									// we're done, send message
									DWORD dwLocation = 0;
									if( m_nDepthCamera3DObjectsFound > 0 )
									{
										dwLocation = (DWORD)MAKELONG( (WORD)(g_pDepthCameraObjects3D->Object[0].CenterX), 
																	  (WORD)(g_pDepthCameraObjects3D->Object[0].CenterY) ); //LOWORD, HIWORD
									}
									else
									{
										ROBOT_LOG( TRUE,"**************** DEPTH_CAMERA - NOTHING FOUND 1 ***************\n")
									}
									SendCommand( WM_ROBOT_DEPTH_CAMERA_SEARCH_COMPLETE, (DWORD)m_nDepthCamera3DObjectsFound, dwLocation );
									// Stop Tracking, and go back to looking for Humans
									m_CurrentTask = TASK_NONE;
									m_TaskState = 1;
								}
								
								ROBOT_ASSERT(0); // TODO

							}
							break;
						}
						default: ROBOT_ASSERT(0);
					}
					break;
				}	// DEPTH_CAMERA_TASK_SCAN_FLOOR_FOR_CLOSEST_OBJECT


				case DEPTH_CAMERA_TASK_TRACK_CLOSEST_OBJECT:
				{	
					// Used to track the closest object on the floor
					int ServoStatus = m_pHeadControl->CheckHeadPosition(FALSE);
					//int ServoStatus = m_pDepthCameraServoControl->CheckServoPosition(FALSE);
					if( DEPTH_CAMERA_SERVO_TIMED_OUT == ServoStatus )
					{
						ROBOT_LOG( TRUE,"ERROR: DEPTH_CAMERA SERVO TIME OUT!  ABORTING!\n\n")
						SendCommand( WM_ROBOT_DEPTH_CAMERA_SEARCH_COMPLETE, 0, 0 ); // no objects found
						// Stop Tracking
						m_CurrentTask = TASK_NONE;
						m_TaskState = 1;
					}
					else if( DEPTH_CAMERA_SERVO_MOVING == ServoStatus )
					{
						// keep waiting
						break;
					}
					// else DEPTH_CAMERA_SERVO_SUCCESS == ServoStatus - Done!

					switch( m_TaskState )
					{
						case 0:
						{
							break;	// Nothing to do
						}
						case 1:	// Request 3D analysis at current position
						{
							FindObjectsOnFloorRequest( DEPTH_CAMERA_FIND_OBJECTS_REQUEST_RETRIES );	// Try "n" times
							m_TaskState++;	
							break;
						}
						case 2:	// Check results at current DepthCamera Tilt Position
						{
							if( -1 == m_nDepthCamera3DObjectsFound )
							{
								// DepthCamera is still looking.  Just wait...
								ROBOT_LOG( TRUE,"DEPTH_CAMERA_TASK_TRACK_CLOSEST_OBJECT Waiting...\n")
								break;
							}
							else
							{
								// Determine if the DepthCamera tilt needs to be adjusted
								if ( DEPTH_CAMERA_SERVO_POSITION_CLOSE_SCAN != m_DepthCameraScanPosition )
								{
									// Not in Close Scan Mode
									if( 0 == m_nDepthCamera3DObjectsFound )
									{
										ROBOT_LOG( TRUE, "TRACK_CLOSEST_OBJECT - Lost Object!  Moving to CLOSE_SCAN\n" )
										m_DepthCameraScanPosition = DEPTH_CAMERA_SERVO_POSITION_CLOSE_SCAN;
										/// m_pDepthCameraServoControl->SetTiltPosition( DEPTH_CAMERA_TILT_OWNER_TRACK_OBJECT, DEPTH_CAMERA_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition] );
									}
									else
									{
										// Objects were found
										// See if object is close enough for close scan.  If so, swich to close scan mode
										// Note - Top of frame = 0, Bottom of frame = 480 or 240
										int FrameCenterY = m_DepthFrameInfo.Height / 2; //m_CaptureSize.height / 2;
										int ObjectFrameCenterY =  g_pDepthCameraObjects3D->Object[0].EndScanLine + ( (g_pDepthCameraObjects3D->Object[0].StartScanLine - g_pDepthCameraObjects3D->Object[0].EndScanLine) / 2 );	// StartScanLine is the bigger number 
										if( ObjectFrameCenterY > (FrameCenterY + (m_DepthFrameInfo.Height / 5)) )	
										{
											// Object within zone where close scan can find it, so tilt DepthCamera down
											// Move to the close position, so we are ready for the next request
											ROBOT_LOG( TRUE, "TRACK_CLOSEST_OBJECT - Object in range, Moving to CLOSE_SCAN\n" )
											m_DepthCameraScanPosition = DEPTH_CAMERA_SERVO_POSITION_CLOSE_SCAN;
											/// m_pDepthCameraServoControl->SetTiltPosition( DEPTH_CAMERA_TILT_OWNER_TRACK_OBJECT, DEPTH_CAMERA_FLOOR_SCAN_POSITION[m_DepthCameraScanPosition] );
										}
									}
								}

								// Send result and end.
								DWORD dwLocation = 0;
								if( m_nDepthCamera3DObjectsFound > 0 )
								{
									dwLocation = (DWORD)MAKELONG( (WORD)(g_pDepthCameraObjects3D->Object[0].CenterX), 
																  (WORD)(g_pDepthCameraObjects3D->Object[0].CenterY) ); //LOWORD, HIWORD
									WORD CenterX = (WORD)g_pDepthCameraObjects3D->Object[0].CenterX;
									WORD CenterY = (WORD)g_pDepthCameraObjects3D->Object[0].CenterY;
									DWORD dwLocationTEST = (DWORD)MAKELONG(CenterX, CenterY); //LOWORD, HIWORD
									ROBOT_ASSERT( (dwLocationTEST == dwLocation) );
								}
								else
								{
									ROBOT_LOG( TRUE,"**************** DEPTH_CAMERA - NOTHING FOUND 2 ***************\n")
								}
								SendCommand( WM_ROBOT_DEPTH_CAMERA_SEARCH_COMPLETE, (DWORD)m_nDepthCamera3DObjectsFound, dwLocation );
								m_CurrentTask = TASK_NONE;
								m_TaskState = 1;
							}
							break;
						}
						default: ROBOT_ASSERT(0);
					}
					break;
				}	// WM_ROBOT_DEPTH_CAMERA_TRACK_OBJECT_CMD
				default: ROBOT_ASSERT(0);
			}
			return;
		} // case WM_ROBOT_SENSOR_STATUS_READY:


#ifdef FUTURE_DEPTH_CAMERA
////////////// ALL THIS IS FOR FUTURE USE ON DEPTH_CAMERA//////////////////
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
#endif //FUTURE_DEPTH_CAMERA

	
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
//	DEPTH_CAMERA 3D OBJECT DETECTION
/////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindWallsAnd2dMaps
// updates global WallPoints array with any x,y coordinates of any walls found by DepthCamera
// Used to avoid large objects, and also to avoid detection of objects too close to walls
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEPTH_CAMERA_WALL_OBJECT_SLOPE_THRESHOLD			0.7	// greater than .5 slope considered a wall or obstacle
#define DEPTH_CAMERA_MIN_WALL_OBJECT_HEIGHT_TENTH_INCHES	40
#define DEPTH_CAMERA_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES	120 // if greater than this, slope not needed, it's an object to avoid
#define DEPTH_CAMERA_WALL_DETECT_GAP_SIZE					30 // number of pixels between front and back slope detector
 
void CDepthCameraModule::FindWallsAnd2dMaps()
{
	// Horizontal Scan lines:  0=Right to Left
	//ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: DepthCamera EnterCriticalSection g_csDepthCameraPointCloudLock\n")
	// __itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
	// EnterCriticalSection(&g_csDepthCameraPointCloudLock);

//	TRACE("FindWallsAnd2dMaps: Edges at:");

	for( int HScan = 0; HScan < m_DepthFrameInfo.Width; HScan++)
	{
		g_DepthCameraPointCloud->WallPoints[HScan].X = g_DepthCameraPointCloud->WallPoints[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR; // infinate range
		g_DepthCameraPointCloud->MapPoints2D[HScan].X = g_DepthCameraPointCloud->MapPoints2D[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_ERROR;

//		ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: HScan = %d\n", HScan)
//		if( HScan > 162 )
	//	{
 		//	ROBOT_LOG( TRUE,"OUCH\n")
	//	}
		BOOL	LeadingEdgeFound = FALSE;
		int		ObjPeakHeight = 0;	// Height of object or cliff
		int		LeadingEdgeY = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;
		int		FloorValue = 0;			// floor value closest to robot
		int		LeadPos = 0;			// Leading sample.  // Training sample (separated by LeadPos by GAP_SIZE)
		int		TrailPos = 0;			// Training sample (separated by LeadPos by GAP_SIZE)

		// Walk up each VERTICAL scan line, closest to robot first (0 = top of frame)
		__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_ProcessVerticalScan);
		for( int LeadPos = (m_DepthFrameInfo.Height-(DEPTH_CAMERA_WALL_DETECT_GAP_SIZE+1)); LeadPos >0; LeadPos-- )
		{
			//ROBOT_LOG( TRUE,"             FindWallsAnd2dMaps: LeadPos = %d\n", LeadPos)

			TrailPos = LeadPos+DEPTH_CAMERA_WALL_DETECT_GAP_SIZE;

			if( (g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].Z >= DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX) || // Note Y,X, not X,Y!
				(g_DepthCameraPointCloud->Point3dArray[TrailPos][HScan].Z >= DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX) )
			{
				continue; // skip invalid pixels
			}

			int LeadZ = g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].Z;
			int TrailZ = g_DepthCameraPointCloud->Point3dArray[TrailPos][HScan].Z;
			double HeightDelta = LeadZ - TrailZ;
			double WidthDelta = g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].Y - g_DepthCameraPointCloud->Point3dArray[TrailPos][HScan].Y;
			double Slope = HeightDelta / WidthDelta;
			double ObjectSlope = 0;

			// Find walls and objects to avoid
			if( (abs(Slope) >= DEPTH_CAMERA_WALL_OBJECT_SLOPE_THRESHOLD) ||		// detect changes between average floor and object
				(LeadZ > DEPTH_CAMERA_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES) )
			{
				// Some Object detected!
				if( !LeadingEdgeFound )
				{
					// Start of new object, wall, or cliff
					LeadingEdgeFound = TRUE;
					LeadingEdgeY = g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].Y;
					//TrailingEdgeFound = FALSE; // Handle "bumpy" objects
					ObjectSlope = Slope;
					FloorValue = TrailZ;
					ObjPeakHeight = LeadZ;	// keep track of peak height
					if( FloorValue > DEPTH_CAMERA_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES )
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
		__itt_task_end(pDomainDepthCameraThread); // 

		// Done with vertical scan line. Update summary data
		if( LeadingEdgeFound )
		{
			// MapPoints contains any blocking object 
			int detectHeight = 20;
			if( LeadingEdgeY >  detectHeight ) // cut out some noise MAP_OBJECT_DETECT_HEIGHT_TENTH_INCHES
			{
				g_DepthCameraPointCloud->MapPoints2D[HScan].X = g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].X;
				g_DepthCameraPointCloud->MapPoints2D[HScan].Y = LeadingEdgeY;
				//g_DepthCameraPointCloud->MapPoints2D[HScan].Z = ObjPeakHeight - FloorValue;
				//if( (g_DepthCameraPointCloud->MapPoints2D[HScan].X > -60) && (g_DepthCameraPointCloud->MapPoints2D[HScan].X < 60) )
				//	TRACE(" Y=%d, X=%d, PeakHeight=%d\n",  g_DepthCameraPointCloud->MapPoints2D[HScan].X, g_DepthCameraPointCloud->MapPoints2D[HScan].Y, ObjPeakHeight );
			}

			if( FloorValue < DEPTH_CAMERA_TRIGGER_WALL_OBJECT_HEIGHT_TENTH_INCHES )
			{
				FloorValue = 0;
			}

			if( abs(ObjPeakHeight - FloorValue) > DEPTH_CAMERA_WALL_DETECT_HEIGHT )
			{
				// WallPoints contains tall objects (used by object detector to avoid objects close to wall)
				g_DepthCameraPointCloud->WallPoints[HScan].X = g_DepthCameraPointCloud->Point3dArray[LeadPos][HScan].X;
				g_DepthCameraPointCloud->WallPoints[HScan].Y = LeadingEdgeY;
				//g_DepthCameraPointCloud->WallPoints[HScan].Z = ObjPeakHeight - FloorValue;

				//if( (g_DepthCameraPointCloud->MapPoints2D[HScan].X > -90) && (g_DepthCameraPointCloud->MapPoints2D[HScan].X < 90) )
					//TRACE(" WALL: Y=%d, X=%d, PeakHeight=%d\n",  g_DepthCameraPointCloud->WallPoints[HScan].Y, g_DepthCameraPointCloud->WallPoints[HScan].X, ObjPeakHeight );
			}
		}

	}
	//LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
	//__itt_task_end(pDomainDepthCameraThread);
	//ROBOT_LOG( TRUE,"FindWallsAnd2dMaps: DepthCamera LeaveCriticalSection g_csDepthCameraPointCloudLock\n")

	// Now, look for Doorways -used by Avoidance Module, and displayed in the GUI
	// (Avoidance module does not alway run, but useful to see this in the GUI anyway)

	int NumberOfDepthCameraSamples = g_DepthCameraPointCloud->FrameSizeX; 
	//int nMaxDoorways = MAX_DOORWAYS;
//	DOORWAY_T  *DoorWaysFound[MAX_DOORWAYS];	
//	POINT2D_T *pPointArray = g_DepthCameraPointCloud->WallPoints;
	FindDoors( NumberOfDepthCameraSamples, MAX_DOORWAYS, g_DepthCameraPointCloud->MapPoints2D, g_pNavSensorSummary->DoorWaysFound );

	// Now throw message in the queue, to indicate that the DepthCamera 2D Map data has been updated
	//TODO!	PostThreadMessage( g_dwControlThreadId, (WM_ROBOT_GPS_DATA_READY), 0, 0 );
	// Post message to the GUI display (local or remote)
	SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM,	// command
		ROBOT_RESPONSE_DEPTH_CAMERA_DATA,				// Param1 = Bulk data command
		0 );									// Param2 = not used

//	LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
//	__itt_task_end(pDomainDepthCameraThread);
/*
	// For DEBUG - Display results
	ROBOT_LOG( TRUE,"DEBUG: DEPTH_CAMERA FindWallsAnd2dMaps: Wall points at: ")
	BOOL bWallFound = FALSE;
	for( int HScan = 0; HScan < m_DepthFrameInfo.Width; HScan++)
	{
		if( g_DepthCameraPointCloud->WallPoints[HScan].Y < LASER_RANGEFINDER_TENTH_INCHES_MAX )
		{
			ROBOT_LOG( TRUE,"%d, ", g_DepthCameraPointCloud->WallPoints[HScan].Y )
		}
	}
	if( bWallFound )
		ROBOT_LOG( TRUE,"TenthInches\n")
	else
		ROBOT_LOG( TRUE,"No Walls Found\n")
*/

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UpdateDepthCameraObjectSummary
// Updates global summary data with distance to closest objects found by DepthCamera in each object avoidance "zone"
// For robots equiped with fixed position Laser Scanner, this is similar info, but gives info for objects at any height
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CDepthCameraModule::UpdateDepthCameraObjectSummary()
{
//	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
//	EnterCriticalSection(&g_csDepthCameraPointCloudLock);

	// Set all values to NO_OBJECT_IN_RANGE
	InitScannerSummaryData( &m_DepthCameraSummary ); 

	// Save time-sensitive data to store with the summary
	m_DepthCameraSummary.SampleTimeStamp = GetTickCount();	// Time laser sample was done
	m_DepthCameraSummary.RobotLocation.x = g_pFullSensorStatus->CurrentLocation.x;		// Location of robot at the time of the laser scan
	m_DepthCameraSummary.RobotLocation.y = g_pFullSensorStatus->CurrentLocation.y;		
	m_DepthCameraSummary.CompassHeading = g_pFullSensorStatus->CompassHeading;		// Heading of robot at the time of the laser scan

	// Now find the minimum Y value for each zone
	for( int i = 0; i < m_DepthFrameInfo.Width; i++ ) 
	{
		int X = g_DepthCameraPointCloud->MapPoints2D[i].X; // tenth inches
		int Y = g_DepthCameraPointCloud->MapPoints2D[i].Y;

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
				if( Y < m_DepthCameraSummary.nLeftFrontZone ) 
					m_DepthCameraSummary.nLeftFrontZone = Y;
				//if( Y < 500 ) 
					//ROBOT_LOG( TRUE,"DEBUG: m_DepthCameraSummary %d Y = %d\n", i, Y )
			}
			else if( X >= -FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_DepthCameraSummary.nLeftArmZone ) 
					m_DepthCameraSummary.nLeftArmZone = Y;
			}
			else if( X >= -FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_DepthCameraSummary.nLeftFrontSideZone ) 
					m_DepthCameraSummary.nLeftFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_DepthCameraSummary.nLeftSideZone ) m_DepthCameraSummary.nLeftSideZone = SideDist;
			}
		}
		else
		{
			// Front Right
			if( X <= FRONT_CENTER_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_DepthCameraSummary.nRightFrontZone ) 
					m_DepthCameraSummary.nRightFrontZone = Y;
			}
			else if( X <= FRONT_ARM_ZONE_EDGE_TENTH_INCHES )
			{
				if( Y < m_DepthCameraSummary.nRightArmZone ) 
					m_DepthCameraSummary.nRightArmZone = Y;
			}
			else if( X <= FRONT_SIDE_ZONE_EDGE_TENTH_INCHES )  
			{
				if( Y < m_DepthCameraSummary.nRightFrontSideZone ) 
					m_DepthCameraSummary.nRightFrontSideZone = Y;
			}
			else
			{
				// Outside target areas. Save distance to the nearest object on the side
				int SideDist = (int)sqrt( (double)(X*X + Y*Y) );
				if( SideDist < m_DepthCameraSummary.nLeftSideZone ) m_DepthCameraSummary.nLeftSideZone = SideDist;
			}
		}

	}

	// Finally, update the global data to indicate to other modules we have a new measurement
	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraSummaryDataLock);
	EnterCriticalSection(&g_csDepthCameraSummaryDataLock);
		memcpy_s(g_pDepthCameraSummary, sizeof(SCANNER_SUMMARY_T), &m_DepthCameraSummary, sizeof(SCANNER_SUMMARY_T) );
	LeaveCriticalSection(&g_csDepthCameraSummaryDataLock);
	__itt_task_end(pDomainDepthCameraThread);


}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsInSingleScanLine
// Look for objects (near the center?) in a single scan line of the DepthCamera depth sensor
// WARNING!  DepthCamera sensor values are in MM!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CDepthCameraModule::FindObjectsInSingleScanLine( int  ScanLine, int NumberOfSamples, OBJECT_2D_ARRAY_T* pDepthCameraObjects2D )
{

#define DEPTH_CAMERA_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM	   1
#define LEFT_SAMPLES_TO_AVE						   2
#define RIGHT_SAMPLES_TO_AVE					   2
//#define GAP_SIZE								   5	// Pixels!  For DepthCamera 320 / ~16" = 20 pixels/inch
#define GAP_SIZE								  10	// Pixels!  For DepthCamera this is 640 / ~16" = 40 pixels/inch
#define OBJECT_DETECT_HEIGHT_TENTH_INCHES		   9	// (TenthInches)
#define OBJECT_DETECT_WIDTH_TENTH_INCHES		   5	// min width to capture an object in the scan line(TenthInches)
#define MAP_OBJECT_DETECT_HEIGHT_TENTH_INCHES	  20	// objects taller than this show up on the map (TenthInches)

//ROBOT_LOG( TRUE,"ScanLine %d - FindObjectsInSingleScanLine() \n", ScanLine )

	if( (NULL == pDepthCameraObjects2D) || (NULL == g_pDepthCameraObjects3D) || (NULL ==g_DepthCameraPointCloud) )
	{
		ROBOT_ASSERT(0);
	}
	
	int  SampleCount = 0;
	double AverageSum = 0;
	double FloorAverage = 0;
	//int Position = g_DepthCameraScannerData[ScanLine].DepthCameraAngleTenthDegrees;

	pDepthCameraObjects2D->nObjectsDetected  = 0;

/*
	if( (0!=ScanLine) && (m_OpenCVDepthMousePoint.y == ScanLine) )
	{
		ROBOT_LOG( TRUE, "DEBUG Mouse Down\n");
	}
*/

	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: DepthCamera EnterCriticalSection g_csDepthCameraPointCloudLock\n")
	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csDepthCameraPointCloudLock);
	EnterCriticalSection(&g_csDepthCameraPointCloudLock);


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

		int Height = g_DepthCameraPointCloud->Point3dArray[ScanLine][i].Z; 
		if(Height >= DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX )
		{
			continue; // Invalid Pixel from deph sensor
		}

		if( !LeadingEdgeFound && (Height >= OBJECT_DETECT_HEIGHT_TENTH_INCHES) )
		{
			// Start of a new object
			StartIndex = i;
			PeakHeight = Height; // set initial peak height
			LeadingEdgeFound = TRUE;
			StartPt = g_DepthCameraPointCloud->Point3dArray[ScanLine][i];
			//g_DepthCameraPointCloud->Point3dArray[ScanLine][i+j].Z
			//ROBOT_LOG( TRUE,"Leading Edge Found X = %3.1f\n", (double)(g_DepthCameraPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0)
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
				//ROBOT_LOG( TRUE,"Trailing Edge Found NEAR X = %3.1f\n", (double)(g_DepthCameraPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0 )
				TrailingEdgeFound = TRUE;
				EndPt = g_DepthCameraPointCloud->Point3dArray[ScanLine][i];
			}
		}
		if( LeadingEdgeFound && TrailingEdgeFound )
		{
			// End of object found!
			int Width = abs(StartPt.X - EndPt.X); // todo - get rid of the abs?
			int AverageY = (StartPt.Y + EndPt.Y) / 2;
			int AverageX = (StartPt.X + EndPt.X) /2;
			// Increase height requirement if the object is far away from the robot (DepthCamera values are less accurate, due to angle)
			int MinDetectionHeight = DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MIN + AverageY / OBJECT_DISTANCE_COMPENSATION;

			if( (PeakHeight < DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MAX) && (PeakHeight > MinDetectionHeight) && 
				(Width < DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MAX) && (Width > DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MIN) )	// Skip big or tiny objects
			{
				// Good object found!

				// Save Object info (in TENTH INCHES)!
				// Summary info
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].X = AverageX;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].Y = AverageY;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].PeakHeight = PeakHeight;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].Width = Width;
				// Details
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].StartX = StartPt.X;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].StartY = StartPt.Y;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].StartZ = StartPt.Z;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].EndX = EndPt.X;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].EndY = EndPt.Y;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].EndZ = EndPt.Z;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].LeftPixel = StartIndex;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].RightPixel = i;
				pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].ScanLine = ScanLine;


				// ROBOT_LOG( TRUE,"Small Object Found: StartX = %3.1f,  EndX = %3.1f, Height = %3.1f Y Distance = %3.1f  FloorRight = %3f, FloorLeft = %3f\n", 
				//			Objects[nDetectedObjects].StartX, Objects[nDetectedObjects].EndX, Objects[nDetectedObjects].PeakHeight, Objects[nDetectedObjects].StartY, RightFloorValue, LeftFloorValue )

				// DEBUG: Display the object found
				#if DEBUG_DEPTH_CAMERA_SHOW_2D_OBJECTS_FOUND == 1
					ROBOT_LOG( TRUE,"Scanline %d      Found 2D Object %d at Y=%4.1f, X=%4.1f   Height = %4.1f   Width = %4.1f   StartX = %4.1f   EndX = %4.1f\n", 
						ScanLine, pDepthCameraObjects2D->nObjectsDetected+1,
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].Y)/10.0,
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].X)/10.0,
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].PeakHeight)/10.0, 
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].Width)/10.0,
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].StartX)/10.0, 
						(double)(pDepthCameraObjects2D->Object[pDepthCameraObjects2D->nObjectsDetected].EndX)/10.0 )
				#endif

				if( pDepthCameraObjects2D->nObjectsDetected++ > DEPTH_SCAN_MAX_2D_OBJECTS )
				{
					ROBOT_LOG( TRUE, "ERROR!  DepthCamera Scanner LookForObjects nDetectedObjects > DEPTH_CAMERA_SCAN_MAX_OBJECTS!  ABORTING further object search!\n")
					ROBOT_ASSERT(0);
					break;
				}
			}
			else
			{
				// DEBUG: Display the slice NOT found, on whatever line the mouse clicks on
				#if DEBUG_DEPTH_CAMERA_SHOW_2D_OBJECTS_NOT_FOUND == 1

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

						if( PeakHeight >= DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MAX)
							ROBOT_LOG( TRUE,"PeakHeight > DEPTH_CAMERA_SLICE_OBJECT_HEIGHT_MAX\n" )
						if( PeakHeight <= MinDetectionHeight )
							ROBOT_LOG( TRUE,"PeakHeight < MinDetectionHeight (peak %d is < %d)\n", PeakHeight, MinDetectionHeight )
						if( Width >= DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MAX )
							ROBOT_LOG( TRUE,"Width > DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MAX\n" )
						if( Width <= DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MIN )
							ROBOT_LOG( TRUE,"Width < DEPTH_CAMERA_SLICE_OBJECT_WIDTH_MIN\n" )
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

	LeaveCriticalSection(&g_csDepthCameraPointCloudLock);
	__itt_task_end(pDomainDepthCameraThread);
	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: DepthCamera LeaveCriticalSection g_csDepthCameraPointCloudLock\n")
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsOnFloorRequest
// queues up a request to the DepthCamera Frame Grap thread to find 3D objects on the floor at the current tilt position
void CDepthCameraModule::FindObjectsOnFloorRequest( int NumberOfTries )
{
	m_nDepthCamera3DObjectsFound = -1; // indicate to the caller that the request is pending
	m_FindObjectsOnFloorTrys = NumberOfTries;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsOnFloor
// Parse scan lines from DepthCamera and find 3D objects on the floor
// Returns the number of objects detected
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OBJECT_CENTER_TOLLERANCE 30 // TenthInches - Max allowed distance between centers of subsequent scan lines (X+Y)
//#define OBJECT_CENTER_Y_WEIGHT   5 // multiplier -  Tunes for how much more important Y is than X when matching found objects
#define WALL_EDGE_ZONE_SIZE	30 // TenthInches - Objects within this distane to wall/large objects are not tracked by DepthCamera

__itt_string_handle* pshDepthCameraFindObjectsLoop = __itt_string_handle_create("DepthCameraFindObjectsLoop");
__itt_string_handle* pshDepthCameraObjectsUpdate = __itt_string_handle_create("DepthCameraObjectsUpdate");
__itt_string_handle* pshDepthCameraFindObjectsSingleScanLine = __itt_string_handle_create("DepthCameraFindObjectsSingleScanLine");
__itt_string_handle* pshDepthCameraFind3DObjects = __itt_string_handle_create("DepthCameraFind3DObjects");

void CDepthCameraModule::FindObjectsOnFloor()
{
	#if( DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1 )
		ROBOT_ASSERT( g_pDepthCameraDebugSliceArray );
		g_pDepthCameraDebugSliceArray->nSlices = 0;
	#endif
	if( m_DepthFrameInfo.FrameNumber < 0 ) return; // means we could not open the shared memory file

	#if (DEBUG_FIND_OBJECTS_ON_FLOOR == 1)
		m_FindObjectsOnFloorTrys = 1;
	#endif

	// Only look for objects if the DepthCamera is in position and
	// track how many times to retry finding objects on the floor
	/*** TODO-MUST
	int ServoPosition = m_pDepthCameraServoControl->GetTiltPosition();
	if( (m_FindObjectsOnFloorTrys <= 0) || (ServoPosition > DEPTH_CAMERA_FLOOR_SCAN_POSITION[DEPTH_CAMERA_SERVO_POSITION_FAR_SCAN]) )
	{
		// Don't look for any objects
		m_nDepthCamera3DObjectsFound = 0;
		return;
	}
	**/

	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshFindObjectsOnFloor);

	// Reset the number of 3D objects found
	m_pDepthCameraTempObjects3D->nObjectsDetected = 0;
	g_pDepthCameraObjects3D->nObjectsDetected = 0;


	// process the DepthCamera POINT CLOUD data, from closest to furthest
	// Go thorugh each row of POINT CLOUD data and identify potential objects then compare these rows
	// to find true 3D objects. (Note this is not the raw depth data from the sensor, it is XYZ values)
	
	// First, look for walls ( Assumes FindWallsAnd2dMaps() was called

	// Uses POINT3D_T	Point3dArray[m_DepthFrameInfo.Width][m_DepthFrameInfo.Height];

	// Walk through each scan line, closest to robot first
	ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"FindObjectsOnFloor: Looking for 3D Objects\n")
	OBJECT_2D_ARRAY_T* pDepthCameraObjects2D = new OBJECT_2D_ARRAY_T;
	
	__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshFindObjectsLoop);
	for( int ScanLine = m_DepthFrameInfo.Height-1; ScanLine >=0; ScanLine-- ) // Start at BOTTOM of frame
	{
		//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"DEBUG: ScanLine = %d\n", ScanLine)
		memset( pDepthCameraObjects2D, 0x00, sizeof( OBJECT_2D_ARRAY_T ) );
		
		__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshFindObjectsSingleScanLine);
			FindObjectsInSingleScanLine( ScanLine, m_DepthFrameInfo.Width, pDepthCameraObjects2D ); // returns pDepthCameraObjects2D
		__itt_task_end(pDomainDepthCameraThread);  // pshFindObjectsSingleScanLine

		// Now iterate through the list of 2D objects found on this line, and associate with any current 3D objects
		if( pDepthCameraObjects2D->nObjectsDetected > 0 )
		{
			__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshFind3DObjects);
			for( int Index2D = (pDepthCameraObjects2D->nObjectsDetected-1); Index2D >= 0; Index2D-- ) // for each 2D object
			{
				// Compare to each 3D object found so far, and find the nearest.
				// If none found close enough, this 2D object will be considered the start of a new 3D object
				int Nearest3DObjectDistance = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;
				int Nearest3DObjectIndex = -1;
				for( int Index3D = 0; Index3D < m_pDepthCameraTempObjects3D->nObjectsDetected; Index3D++ )
				{
					int deltaX = abs( m_pDepthCameraTempObjects3D->Object[Index3D].LastX - pDepthCameraObjects2D->Object[Index2D].X );	// compare AVE 3D X value with current 2D object
					int deltaY = abs( m_pDepthCameraTempObjects3D->Object[Index3D].EndY - pDepthCameraObjects2D->Object[Index2D].Y );		// compare LAST 3D Y value with current slice (building up the 3D image in slices)
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
					if( m_pDepthCameraTempObjects3D->nObjectsDetected > DEPTH_SCAN_MAX_3D_OBJECTS )
					{
						//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"WARNING: DepthCameraMultiLineScanFindObjects3D: m_pDepthCameraTempObjects3D->nObjectsDetected > DEPTH_CAMERA_SCAN_MAX_3D_OBJECTS. Ignoring far objects\n")
					}
					else
					{
						#if (DEBUG_DEPTH_CAMERA_SHOW_3D_SLICE_OBJECTS_FOUND == 1)

						ROBOT_LOG( TRUE,"ScanLine %d - NEW 3D Object %3d:  Y=%4.1f, X=%4.1f   Width = %4.1f,   Height = %4.1f, Nearest Dist = %d\n", ScanLine, m_pDepthCameraTempObjects3D->nObjectsDetected, 
							(double)(pDepthCameraObjects2D->Object[Index2D].Y)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].X)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].Width)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].PeakHeight)/10.0, Nearest3DObjectDistance )
						#endif
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].NumberOfSlices = 1;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].CenterXSum =	pDepthCameraObjects2D->Object[Index2D].X;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].HeightSum =	pDepthCameraObjects2D->Object[Index2D].PeakHeight;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].WidthSum =		pDepthCameraObjects2D->Object[Index2D].Width;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].StartY =		pDepthCameraObjects2D->Object[Index2D].Y;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].EndY =			pDepthCameraObjects2D->Object[Index2D].Y; // Just one slice so far
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].LastX =		pDepthCameraObjects2D->Object[Index2D].X; // Just one slice so far

						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].CenterX =		pDepthCameraObjects2D->Object[Index2D].X;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].CenterY =		pDepthCameraObjects2D->Object[Index2D].Y;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].Height =		pDepthCameraObjects2D->Object[Index2D].PeakHeight;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].Width =		pDepthCameraObjects2D->Object[Index2D].Width;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].Length = 0;

						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].LeftPixel =  pDepthCameraObjects2D->Object[Index2D].LeftPixel;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].RightPixel = pDepthCameraObjects2D->Object[Index2D].RightPixel;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].StartScanLine = pDepthCameraObjects2D->Object[Index2D].ScanLine;
						m_pDepthCameraTempObjects3D->Object[m_pDepthCameraTempObjects3D->nObjectsDetected].EndScanLine = pDepthCameraObjects2D->Object[Index2D].ScanLine;

						m_pDepthCameraTempObjects3D->nObjectsDetected++;
					}
				}
				else
				{
					// this "slice" lines up with another prior "slice" of a 3D object.  Save the data.
					// NOTE!  we keep the SUM and current Average of each data value at this point.

					if( Nearest3DObjectIndex < 0 ) 
						ROBOT_ASSERT(0);

					#if DEBUG_DEPTH_CAMERA_SHOW_3D_SLICE_OBJECTS_FOUND == 1
						ROBOT_LOG( TRUE,"    3D Slice found on ScanLine %3d:  Y=%4.1f, X=%4.1f   Width = %4.1f,   Height = %4.1f  Obj Index = %d\n", ScanLine, 
							(double)(pDepthCameraObjects2D->Object[Index2D].Y)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].X)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].Width)/10.0, 
							(double)(pDepthCameraObjects2D->Object[Index2D].PeakHeight)/10.0, Nearest3DObjectIndex )

					#endif
					#if( DEBUG_DEPTH_CAMERA_SHOW_3D_SLICES_ON_FRAME == 1 )
						ROBOT_ASSERT( g_pDepthCameraDebugSliceArray );
						if( g_pDepthCameraDebugSliceArray->nSlices < MAX_DEBUG_SLICES )
						{
							g_pDepthCameraDebugSliceArray->Slice[g_pDepthCameraDebugSliceArray->nSlices].Pt1.y = ScanLine;
							g_pDepthCameraDebugSliceArray->Slice[g_pDepthCameraDebugSliceArray->nSlices].Pt2.y = ScanLine;
							g_pDepthCameraDebugSliceArray->Slice[g_pDepthCameraDebugSliceArray->nSlices].Pt1.x = pDepthCameraObjects2D->Object[Index2D].LeftPixel;
							g_pDepthCameraDebugSliceArray->Slice[g_pDepthCameraDebugSliceArray->nSlices].Pt2.x = pDepthCameraObjects2D->Object[Index2D].RightPixel;
							g_pDepthCameraDebugSliceArray->nSlices++;
						}

					#endif

					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices++;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].CenterXSum += pDepthCameraObjects2D->Object[Index2D].X;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].HeightSum += pDepthCameraObjects2D->Object[Index2D].PeakHeight;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].WidthSum += pDepthCameraObjects2D->Object[Index2D].Width;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].EndY = pDepthCameraObjects2D->Object[Index2D].Y; // Keep updating this until the last slice

					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].LastX = pDepthCameraObjects2D->Object[Index2D].X; // Keep updating this until the last slice. Used to look for aligned objects

					// for bounding box on GUI
					if( pDepthCameraObjects2D->Object[Index2D].LeftPixel < m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].LeftPixel )
					{
						m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].LeftPixel = pDepthCameraObjects2D->Object[Index2D].LeftPixel;
					}
					if( pDepthCameraObjects2D->Object[Index2D].RightPixel > m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].RightPixel )
					{
						m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].RightPixel = pDepthCameraObjects2D->Object[Index2D].RightPixel;
					}
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].EndScanLine = pDepthCameraObjects2D->Object[Index2D].ScanLine; // keep track of the last scanline

					// Now set temporary values, used to find other close slices.  When we hit the last slice, these values should be correct!
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].CenterX = m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].CenterXSum / m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].CenterY = 
						m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].StartY + ((m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].EndY - m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].StartY) / 2);
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].Height = m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].HeightSum / m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].Width = m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].WidthSum / m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].NumberOfSlices;
					m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].Length = abs(m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].EndY - m_pDepthCameraTempObjects3D->Object[Nearest3DObjectIndex].StartY);
				}
			} // for
			__itt_task_end(pDomainDepthCameraThread); // pshFind3DObjects
		} // if
	}
	__itt_task_end(pDomainDepthCameraThread); // FindObjectsLoop
	delete pDepthCameraObjects2D;
	

	// Now, copy objects found to global, if they pass the size test
	// AND not too close to a wall (as defined by the Laser Range Finder)
	//	int ClosestObjectDistance = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;
	//	int ClosestObjectIndex = 0;

	if( m_pDepthCameraTempObjects3D->nObjectsDetected > 0)
	{
		__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, pshObjectsUpdate);
		
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "\n*************************\n")
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "FINAL 3D Objects Found:\n", g_pDepthCameraObjects3D->nObjectsDetected )
		g_pDepthCameraObjects3D->nObjectsDetected = 0;
		g_pDepthCameraObjects3D->nClosestObjectIndex = 0;
		g_pDepthCameraObjects3D->nClosestObjectDistance = DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX;

		for( int i = 0; i < m_pDepthCameraTempObjects3D->nObjectsDetected; i++ )
		{
			// Test size of object
			if( m_pDepthCameraTempObjects3D->Object[i].Length < MIN_3D_OBJECT_LENGTH)		// minimim size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Small: Length = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pDepthCameraTempObjects3D->Object[i].Width < MIN_3D_OBJECT_WIDTH)			// minimim size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Small: Width = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].Width )
				continue;
			}
			if( m_pDepthCameraTempObjects3D->Object[i].Length > MAX_3D_OBJECT_LENGTH)		// max size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Big: Length = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pDepthCameraTempObjects3D->Object[i].Width > MAX_3D_OBJECT_WIDTH)			// max size of object to detect
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Big: Width = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].Width )
				continue;
			}
			if( m_pDepthCameraTempObjects3D->Object[i].Height > MAX_3D_OBJECT_HEIGHT)			// max height of object to detect (catch feet attached to legs)
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too High: Height = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].Height )
				continue;
			}
			if( abs(m_pDepthCameraTempObjects3D->Object[i].CenterX) > MAX_3D_OBJECT_X_POSITION )	// Too far to one side
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Far off center: CenterX = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].CenterX )
				continue;
			}
			if( m_pDepthCameraTempObjects3D->Object[i].RightPixel >= m_DepthFrameInfo.Width )	// Too close to edge of frame, might be chair leg
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %d Too Far off center: CenterX = %d\n", i, m_pDepthCameraTempObjects3D->Object[i].CenterX )
				continue;
			}

			// Passes size test.  See if it's too close to a wall (might be a bogus object, furniture, etc.)
			// NOTE: Assumes FindWallsAnd2dMaps() was called first!

			int RoughDistance = LASER_RANGEFINDER_TENTH_INCHES_MAX;
			for( int HScan = 0; HScan < m_DepthFrameInfo.Width; HScan++)
			{
				if( g_DepthCameraPointCloud->WallPoints[HScan].Y < LASER_RANGEFINDER_TENTH_INCHES_MAX )
				{

					int DeltaX = abs( g_DepthCameraPointCloud->WallPoints[HScan].X - m_pDepthCameraTempObjects3D->Object[i].CenterX );
					int DeltaY = abs( g_DepthCameraPointCloud->WallPoints[HScan].Y - m_pDepthCameraTempObjects3D->Object[i].CenterY );
					RoughDistance = (DeltaX + DeltaY) / 2;
					//ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )

					if( RoughDistance < WALL_EDGE_ZONE_SIZE )
					{
						// too close to wall
						ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: Point=%4d   Wall Y=%4d, X=%4d,     DELTA Y=%4d, X=%4d  DIST = %d\n", HScan,  
							g_DepthCameraPointCloud->WallPoints[HScan].Y, g_DepthCameraPointCloud->WallPoints[HScan].X, DeltaY, DeltaX, RoughDistance )
						ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DepthCamera Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to Wall (%4.1f in), skipping\n", 
							(double)m_pDepthCameraTempObjects3D->Object[i].CenterY / 10.0,
							(double)m_pDepthCameraTempObjects3D->Object[i].CenterX / 10.0,
							(double)m_pDepthCameraTempObjects3D->Object[i].Height / 10.0, (double)RoughDistance / 10.0 )
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
			int WallScanZoneScanLines = m_DepthFrameInfo.Height / 4; // reserve top 1/4 of the frame for detecting walls
			if ( m_pDepthCameraTempObjects3D->Object[i].EndScanLine <  WallScanZoneScanLines )
			{
				// too close to top of frame
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DEBUG: ObjStartScanLine =%4d   ObjEndScanLine X=%4d, ZONE=%4d,\n",  
					m_pDepthCameraTempObjects3D->Object[i].StartScanLine, m_pDepthCameraTempObjects3D->Object[i].EndScanLine, DEPTH_CAMERA_WALL_SCAN_ZONE )

				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DepthCamera Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to top of frame, skipping\n", 
					(double)m_pDepthCameraTempObjects3D->Object[i].CenterY / 10.0,
					(double)m_pDepthCameraTempObjects3D->Object[i].CenterX / 10.0,
					(double)m_pDepthCameraTempObjects3D->Object[i].Height / 10.0 )
				break;
			}


			// PLAN B  Passes size test.  See if it's too close to a wall (might be a bogus object, furniture, etc.)
			// Compare to Laser values to determine this
//#define THIS_WORKS_NOW		
#ifdef THIS_WORKS_NOW
			__itt_task_begin(pDomainDepthCameraThread, __itt_null, __itt_null, psh_csLaserDataLock);
			EnterCriticalSection(&g_csLaserDataLock);
				int  NumberOfLaserSamples = g_pLaserScannerData->NumberOfSamples;
				int Distance = 0;
				if( 0 != NumberOfLaserSamples )
				{
					for( int nSample = 0; i < NumberOfLaserSamples; nSample++ ) 
					{
						int WallPointX = g_pLaserScannerData->ScanPoints[nSample].X ; // tenth inches
						int WallPointY = g_pLaserScannerData->ScanPoints[nSample].Y ;
						int DeltaX = abs( WallPointX - m_pDepthCameraTempObjects3D->Object[i].CenterX );
						int DeltaY = abs( WallPointY - m_pDepthCameraTempObjects3D->Object[i].CenterY );
						Distance = DeltaX + DeltaY;
						//ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )

						if( Distance < WALL_EDGE_ZONE_SIZE )
						{
							// too close to wall
							ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )
							ROBOT_LOG( TRUE, "DepthCamera Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to Wall, skipping\n", 
								(double)m_pDepthCameraTempObjects3D->Object[i].CenterX / 10.0,
								(double)m_pDepthCameraTempObjects3D->Object[i].CenterY / 10.0,
								(double)m_pDepthCameraTempObjects3D->Object[i].Height / 10.0 )
							break;
						}
					}
				} // if( 0 != NumberOfLaserSamples )
			LeaveCriticalSection(&g_csLaserDataLock);
			__itt_task_end(pDomainDepthCameraThread);

			if( Distance < WALL_EDGE_ZONE_SIZE )
			{
				continue;
			}
#endif
			// OK, looks good!
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterX = m_pDepthCameraTempObjects3D->Object[i].CenterX;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterY = m_pDepthCameraTempObjects3D->Object[i].CenterY;// -10;	// TenthInches - fudge becase it always seems to miss the first inch of the object
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Height = m_pDepthCameraTempObjects3D->Object[i].Height;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Width = m_pDepthCameraTempObjects3D->Object[i].Width;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Length = m_pDepthCameraTempObjects3D->Object[i].Length;// +10;		// TenthInches - fudge becase it always seems to miss the first inch of the object

			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].LeftPixel = m_pDepthCameraTempObjects3D->Object[i].LeftPixel;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].RightPixel = m_pDepthCameraTempObjects3D->Object[i].RightPixel;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].StartScanLine = m_pDepthCameraTempObjects3D->Object[i].StartScanLine;
			g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].EndScanLine = m_pDepthCameraTempObjects3D->Object[i].EndScanLine;


			// For debug, print out the 3D objects found
			ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Object %2d:  Y=%4.1f X=%4.1f,   Height=%4.1f  Width=%4.1f,  Length=%4.1f, BB = (%d,%d) (%d,%d)\n", g_pDepthCameraObjects3D->nObjectsDetected,
				(double)g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterY / 10.0,
				(double)g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterX / 10.0,
				(double)g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Height / 10.0,
				(double)g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Width / 10.0,
				(double)g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].Length / 10.0,
				g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].LeftPixel,
				g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].StartScanLine,
				g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].RightPixel,
				g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].EndScanLine )


			if( g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterY < g_pDepthCameraObjects3D->nClosestObjectDistance )
			{
				// New Closest Object found
				g_pDepthCameraObjects3D->nClosestObjectDistance = g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nObjectsDetected].CenterY;
				g_pDepthCameraObjects3D->nClosestObjectIndex = g_pDepthCameraObjects3D->nObjectsDetected;
			}
			g_pDepthCameraObjects3D->nObjectsDetected++;


			if( g_pDepthCameraObjects3D->nObjectsDetected > DEPTH_SCAN_MAX_3D_OBJECTS )
			{
				ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES,"\nWARNING: 3D Objects found > DEPTH_CAMERA_SCAN_MAX_3D_OBJECTS. Ignoring far objects")
				break;
			}
		}
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "Found %d 3D Objects\n", g_pDepthCameraObjects3D->nObjectsDetected )
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "*************************\n\n")

		if( g_pDepthCameraObjects3D->nClosestObjectDistance < DEPTH_CAMERA_RANGE_TENTH_INCHES_MAX )
		{
			// An object was found. Tell the depth app to display a bounding box
			m_DepthCameraCommand.ControlFlags |= DepthCameraControlFlag_DisplayBoundingBox; // Draw a bounding box
			m_DepthCameraCommand.BoundingBoxBottom = g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nClosestObjectIndex].StartScanLine;
			m_DepthCameraCommand.BoundingBoxTop = g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nClosestObjectIndex].EndScanLine;	// lines are inverted
			m_DepthCameraCommand.BoundingBoxLeft = g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nClosestObjectIndex].LeftPixel;
			m_DepthCameraCommand.BoundingBoxRight = g_pDepthCameraObjects3D->Object[g_pDepthCameraObjects3D->nClosestObjectIndex].RightPixel;
			 
			// Send command to the depth camera app via shared memory
			if( (NULL != g_pDepthCameraCommandSharedMemory) && (NULL != g_hDepthCameraCommandEvent) && 
				(m_DepthCameraCommand.ControlFlags != DepthCameraControlFlag_None)    )
			{
				CopyMemory((PVOID)g_pDepthCameraCommandSharedMemory, &m_DepthCameraCommand, (sizeof(DEPTH_CAMERA_COMMAND_T)));
				SetEvent( g_hDepthCameraCommandEvent );  // Tell Kobuki app that a new command is pending
			}
			else
			{
				ROBOT_LOG( TRUE, "ERROR: Cant send Bounding Box command to Depth Camera App!  Did you have AUTO_LAUNCH_DEPTH_CAMERA_APP enabled?\n" )
			}

		}

		__itt_task_end(pDomainDepthCameraThread); // pshObjectsUpdate
	} // if( m_pDepthCameraTempObjects3D->nObjectsDetected > 0)

	// Report out final results
	if( g_pDepthCameraObjects3D->nObjectsDetected > 0 )
	{
		// At least one good 3D object found
		m_nDepthCamera3DObjectsFound = g_pDepthCameraObjects3D->nObjectsDetected;
	}
	else
	{
		// No good 3D objects found
		ROBOT_LOG( DEBUG_FIND_OBJECTS_ON_FLOOR_MESSAGES, "DepthCameraMultiLineScanFindObjects3D:  No 3D Objects Found in Image.\n" )
		if( DEPTH_CAMERA_FIND_OBJECTS_REQUEST_CONTINUOUS != m_FindObjectsOnFloorTrys )
		{
			if( --m_FindObjectsOnFloorTrys <= 0 ) // decrement the retry counter
			{
				// No more retries left, tell calling routine we are done
				m_nDepthCamera3DObjectsFound = 0;	// anthing but -1 indicates we are done looking
				m_FindObjectsOnFloorTrys = 0;
			}
		}
	}

	__itt_task_end(pDomainDepthCameraThread); // pshFindObjectsOnFloor
}


#if (DEPTH_CAMERA_INSTALLED_IN_HEAD == 0)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//	DepthCamera Servo Control
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
// Name: GetTiltPosition
// Desc: Gets current tilt position of the DepthCamera sensor in TENTHDEGREES
//-----------------------------------------------------------------------------
int DepthCameraServoControl::GetTiltPosition(  )
{
	return g_BulkServoStatus[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees;
}

//-----------------------------------------------------------------------------
// Name: SetTiltPosition
// Desc: Sets tilt position of the DepthCamera sensor in TENTHDEGREES
//-----------------------------------------------------------------------------
void DepthCameraServoControl::SetTiltPosition( int  nOwner, int TiltTenthDegrees, int Speed ) // Speed is optional
{
	// NOTE: DEPTH_CAMERA TILT *SPEED* IGNORED FOR NOW!!!  TODO???


	if( !CheckAndSetOwner( nOwner ) )
	{
		return;	// Higher priority task has control
	}

	if( (nOwner == gDepthCameraCurrentOwner) &&
		(TiltTenthDegrees == g_BulkServoCmd[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees) )
	{
		return; // ingore repeated command
	}


	// Check servo limit
	if( TiltTenthDegrees > DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_UP )
	{
		ROBOT_DISPLAY( TRUE, "DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_UP limit" )
		TiltTenthDegrees = DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_UP;
	}
	else if( TiltTenthDegrees < DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_DOWN)
	{
		ROBOT_DISPLAY( TRUE, "DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_DOWN limit" )
		TiltTenthDegrees = DEPTH_CAMERA_TILT_TENTHDEGREES_MAX_DOWN;
	}

	__itt_marker(pDomainControlThread, __itt_null, pshDepthCameraServoStart, __itt_marker_scope_task);
	//		ROBOT_LOG( TRUE,"DepthCamera SetTiltPosition\n")

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csServoLock);
	EnterCriticalSection(&g_csServoLock);
	/*	if( 0 != Speed )
		{
			g_BulkServoCmd[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].Speed = Speed;
		}*/
		g_BulkServoCmd[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees = TiltTenthDegrees;
		g_BulkServoCmd[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].Update = TRUE;
	LeaveCriticalSection(&g_csServoLock);
	__itt_task_end(pDomainControlThread);

	// set the "watchdog" timer, in case the servo does not reach the commanded position
	gDepthCameraMoveTimeout = DEFAULT_DEPTH_CAMERA_MOVE_TIME_LIMIT_TENTH_SECONDS;

	// Shortcut - post directly to the Dynamixel DynaServoComm thread
	//SendCommand( WM_ROBOT_SET_HEAD_POSITION, 0, FALSE );	// FALSE = Don't Set Speed

	if( 0 != Speed )
	{
		PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_DEPTH_CAMERA_POSITION), 0, (DWORD)TRUE ); // Set Speed
	}
	else
	{
		PostThreadMessage( g_dwSmartServoCommThreadId, (WM_ROBOT_MESSAGE_BASE+HW_SET_BULK_DEPTH_CAMERA_POSITION), 0, (DWORD)FALSE ); // FALSE = Don't Set Speed
	}
}



//-----------------------------------------------------------------------------
// Name: CheckServoPosition
// Desc: Are we there yet?  Check to see if servo has reached commanded position
// Verbose flag will print out the servo that is blocking move complete status
// Returns:
//		DEPTH_CAMERA_SERVO_MOVING = 0, 
//		DEPTH_CAMERA_SERVO_SUCCESS,
//		DEPTH_CAMERA_SERVO_TIMED_OUT
//-----------------------------------------------------------------------------
int DepthCameraServoControl::CheckServoPosition( BOOL verbose )
{
	if( 1 == ROBOT_SIMULATION_MODE )
	{
		return DEPTH_CAMERA_SERVO_SUCCESS;	// Handle simulation mode (no servos!)
	}

	int DeltaTilt = g_BulkServoCmd[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees -
		g_BulkServoStatus[DYNA_DEPTH_CAMERA_SCANNER_SERVO_ID].PositionTenthDegrees;

	//ROBOT_LOG( TRUE,"DepthCamera CheckServoPosition: DeltaTilt = %d Degrees\n", DeltaTilt/10)

	if( abs(DeltaTilt) > DEPTH_CAMERA_JOINT_DELTA_MAX_TENTHDEGREES )
	{
		if( verbose )
		{
			// In verbose mode, show why we are not yet in position
			ROBOT_LOG( TRUE,"DepthCamera CheckServoPosition: DeltaTilt = %d Degrees\n", DeltaTilt/10)
		}
		if( 0 == gDepthCameraMoveTimeout )
		{
			//if( verbose ) 
				ROBOT_LOG( TRUE,"DepthCamera CheckServoPosition: Servo Move Timed Out!  Delta = %d TenthDegrees\n", DeltaTilt)
			return DEPTH_CAMERA_SERVO_TIMED_OUT;
		}
	}
	else
	{
		__itt_marker(pDomainControlThread, __itt_null, pshDepthCameraServoEnd, __itt_marker_scope_task);

		if( verbose )
		{
			ROBOT_LOG( TRUE,"DepthCamera CheckServoPosition: Servo in Target Position\n")
		}
		return DEPTH_CAMERA_SERVO_SUCCESS;
	}

	return DEPTH_CAMERA_SERVO_MOVING;	// Did not reach target position yet
}

/////////////////////////////////////////////////////////////////////////////////////////
// DEPTH_CAMERA SERVO CONTROL ARBITRATOR
// Tracks who is control of the DepthCamera tilt position, assigning control based upon a strict priority
/////////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
// Name: Check And Set Owner
// Desc: Checks for current owner, and if requesting module is higher priority
// changes to new module.  Returns TRUE if module got ownership.
//-----------------------------------------------------------------------------
BOOL DepthCameraServoControl::CheckAndSetOwner( int  NewOwner )
{
	if( NewOwner == gDepthCameraCurrentOwner )
	{
		// Owner reasking for control
		// Silently reset timer for how long to maintain control
		if( gDepthCameraOwnerTimer < 50 )
		{
			gDepthCameraOwnerTimer = 50;	// 1/10 seconds
		}
		return TRUE;
	}

	CString MsgString, NewOwnerName, CurrentOwnerName;
	DepthCameraOwnerNumberToName( gDepthCameraCurrentOwner, CurrentOwnerName );
	DepthCameraOwnerNumberToName( NewOwner, NewOwnerName );

	if( (0 == gDepthCameraOwnerTimer) && (HEAD_OWNER_NONE != gDepthCameraCurrentOwner) )
	{
		// ownership timed out
		MsgString.Format( "DepthCamera Control: Owner %s has timed out", CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
		gDepthCameraCurrentOwner = DEPTH_CAMERA_TILT_OWNER_NONE;
	}

	if( NewOwner < gDepthCameraCurrentOwner )
	{
		// Higher priority module has control
		if( NewOwner > DEPTH_CAMERA_TILT_OWNER_COLLISION_AVOIDANCE ) // don't report random movement requests for control, too noisy
		{
			MsgString.Format( "DepthCamera Control: Request by %s Rejected: %s has control",NewOwnerName, CurrentOwnerName );
			ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
			ROBOT_LOG( TRUE,"gDepthCameraOwnerTimer = %d\n", gDepthCameraOwnerTimer)
		}
		return FALSE;
	}

	// No one with higher priority has control.
	if( NewOwner > gDepthCameraCurrentOwner )
	{
		MsgString.Format( "DepthCamera Control: NewOwner %s has taken control from %s",NewOwnerName, CurrentOwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString );
		gDepthCameraCurrentOwner = NewOwner;
	}

	// Requesting module has control now
	// Set timer for how long to maintain control
	gDepthCameraOwnerTimer = 50;	// 1/10 seconds


	// Indicate on the GUI who is in control
	// SendResponse( WM_ROBOT_DISPLAY_SINGLE_ITEM, ROBOT_RESPONSE_DRIVE_MODULE_OWNER, Module );
	return TRUE;

}

BOOL DepthCameraServoControl::IsOwner( int  NewOwner )
{
	if( NewOwner != gDepthCameraCurrentOwner )
	{
		// Not current owner
		return FALSE;
	}
	return TRUE;
}

BOOL DepthCameraServoControl::ReleaseOwner( int  NewOwner )
{
	if( NewOwner < gDepthCameraCurrentOwner )
	{
		// Higher priority module has control already so just ignore.
/*		CString MsgString, ModuleName, OwnerName;
		DepthCameraOwnerNumberToName( Module, ModuleName );
		DepthCameraOwnerNumberToName( gDepthCameraCurrentOwner, OwnerName );
		MsgString.Format( "Module %s releasing control, but %s already has it", ModuleName, OwnerName );
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
*/
		return FALSE;
	}

	// No one with higher priority has control.  Release ownership.
	CString MsgString, NewOwnerName;
	DepthCameraOwnerNumberToName( NewOwner, NewOwnerName );
	MsgString.Format( "DepthCamera Control: Owner %s releasing control", NewOwnerName );
	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )
	gDepthCameraCurrentOwner = DEPTH_CAMERA_TILT_OWNER_NONE;
	return TRUE;

}
#endif 



#endif // #if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
