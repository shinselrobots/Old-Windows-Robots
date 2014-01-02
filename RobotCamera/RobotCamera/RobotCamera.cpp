// RobotCamera.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "CameraCommon.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <windows.h>
#include <tchar.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sstream>

//#include <math.h>
//#include <MMSystem.h>	// For Sound functions
#include <atltime.h>
//#include <time.h>


using namespace cv;
using namespace std;

#include "CameraDetectors.h"
#include "RobotCamera.h"

#define SHOW_VIDEO_WINDOW		1 // Turn off to lower processing overhead

//////////////////////////////////////////////////////////////////////////////////////////////
// Globals

FILE* g_LogFile = NULL;		// Log file for non-debug mode
int	  gStartTime = 0;		// Used to determine time since Robot started.  Warning, only good for 45 days! :-)

// Detectors
CFaceRecognizer *pFaceRecognizer = NULL;
CMatchObjectDetector *pObjectMatcher = NULL;

// Enabled Features
CAMERA_REQUEST_ENABLE_FEATURES_T g_EnabledFeatures;
VideoCapture *g_pVidCap = NULL;


//////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char *argv[]) 
{
	#if (DEBUG_MEMORY_LEAKS == 1 )
		_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	// Use to detect memory leaks!
	// _CrtSetBreakAlloc( Put allocator number here);

	// Global Variables
	LPCTSTR pUpdateDataSharedMemory = NULL;
	CAMERA_UPDATE_T CameraUpdate;
	LPCTSTR pRequestSharedMemory = NULL;
	//CAMERA_REQUEST_T CameraRequest;
	HANDLE hUpdateDataEvent = NULL;
	HANDLE hRequestDataEvent = NULL;
    Mat	Frame;		// Current frame from the Video 
	bool GrabSnapshot = false;
	bool bMatchObject = false;
	

	gStartTime = GetTickCount();	// Keep track of time since system startup
	g_EnabledFeatures.VideoEnable = 0;
	g_EnabledFeatures.FaceRecognition = 0;
	g_EnabledFeatures.FaceTracking = 0;
	g_EnabledFeatures.ObjectMatch = 0;
	    
	
	// Initialize shared memory and events for communicating with the Robot control application
	int InitIPCResult = InitIPC( pUpdateDataSharedMemory, hUpdateDataEvent, pRequestSharedMemory, hRequestDataEvent );
	if ( FAILED == InitIPCResult )
	{
		cerr << "InitIPC failed.  Exiting" << endl;
		return -1;
	}
	else if( STAND_ALONE_MODE == InitIPCResult )
	{
		// Running in Stand Alone (Test) mode
		// Setup the following as needed for testing:

		CAMERA_REQUEST_ENABLE_FEATURES_T TestEnabledFeatures;
		TestEnabledFeatures.VideoEnable =		1;
		TestEnabledFeatures.FaceRecognition =	1;
		TestEnabledFeatures.FaceTracking =		1;
		TestEnabledFeatures.ObjectMatch =		0; //1;
		ManageFeatures( TestEnabledFeatures );
	}



/***
	// Open log file
	//#ifndef _DEBUG // optionally, only open the log file in Release mode
	errno_t nError = fopen_s( &g_LogFile, LogFileName, "w" );
	if( 0 == nError )
	{
		fprintf(g_LogFile, "STARTING LOG FILE.  Start Time: %s\n\n", COleDateTime::GetCurrentTime().Format() );
	}
	else
	{
		g_LogFile = NULL;
		CString ErrorStr;
		ErrorStr.Format( _T("Can't open Log File\n %s Error = %d"), LogFileName, nError );
		AfxMessageBox( ErrorStr );
	}
	//#endif
***/



	cout << "Robot Camera Starting. Press Esc in the video window to exit" << endl << endl;
	#if( 1 != SHOW_VIDEO_WINDOW ) 
		cout << "WARNING : Video Window disabled to save CPU"  << endl;
	#endif
	Sleep(5000); // let robot code start up first


	///////////////////////////////////////////////////////////////////////////////////
	// Frame capture and processing Loop.  Change msTimeOut to tune Capture frame rate
	const DWORD msTimeOut = 10;  // Sleep time.  
	while( true )
	{
		if( STAND_ALONE_MODE != InitIPCResult )
		{
			DWORD dwResult = WaitForSingleObject(hRequestDataEvent, msTimeOut);
			if( WAIT_OBJECT_0 == dwResult ) 
			{
				// Event did not time out.  That means the Robot has posted a request to shared memory
				cout << "Request from Robot engine received" << endl;
				// Read from Shared Memory
				CAMERA_REQUEST_T *Request = (CAMERA_REQUEST_T*)pRequestSharedMemory;

				if( CAMERA_REQUEST_ENABLE_FEATURES == Request->RequestType )
				{
					// Features need to be turned on or off
					ManageFeatures( Request->RequestData.EnableFeatures );
				}
				else if( CAMERA_REQUEST_FACE_RECO == Request->RequestType )
				{
					// Face recognition request
					if( Request->RequestData.FaceRequest.bRequestCapture )
					{
						Request->RequestData.FaceRequest.bRequestCapture = 0; // clear the flag
						// Note: currently, the only currently supported request is to capture current person's face, and save their name and ID
						if( pFaceRecognizer ) 
						{
							pFaceRecognizer->SaveNextFace( 
								Request->RequestData.FaceRequest.PersonID, 
								Request->RequestData.FaceRequest.PersonName );
						}
					}
				}
				else if( CAMERA_REQUEST_TAKE_SNAPSHOT == Request->RequestType )
				{
					GrabSnapshot = TRUE;
					Request->RequestType = CAMERA_REQUEST_NONE; // TODO is this needed?
					cout << "Debug: Grab Snapshot requested" << endl;
				}			
				else if( CAMERA_REQUEST_OBJECT_RECO == Request->RequestType )
				{
					bMatchObject = TRUE;
					Request->RequestType = CAMERA_REQUEST_NONE; // TODO is this needed?
					cout << "Debug: Match Object requested" << endl;
				}
			}
		}


		if( g_EnabledFeatures.VideoEnable )
		{

			BOOL bPersonRecognized = FALSE;
			BOOL bFaceDetected = FALSE;
			int DetectedPersonID = 0;
			string DetectedPersonName;
			//CAMERA_FACE_RECO_UPDATE_T FaceUpdateParams;

			if( !g_pVidCap )
			{
				cout << "ERROR: g_pVidCap == NULL" << endl;
				return -1;
			}

			*g_pVidCap >> Frame;
			// Clone the current frame:
			Mat CurrentFrame = Frame.clone();

			// Save Frame Snapshot if requested
			//GrabSnapshot = TRUE;
			if( GrabSnapshot )
			{
				string LastImgFileName = "c:\\temp\\snapshot.jpg";
			
				// keep a copy of all pics on disk
				CTime Time = CTime::GetCurrentTime();
				CString ATLFileName = "c:\\temp\\Loki_Robot_" + Time.Format(_T("%a_%I_%M%p_%S")) + ".jpg";
				CT2CA pszConvertedAnsiString (ATLFileName);
				string ImgFileName = pszConvertedAnsiString;

				// Save current face info
				//PersonIDs.push_back( m_PersonID );
				//PersonNames.push_back( m_PersonName );
				//ImgFileNames.push_back( m_ImgFileName );
				//Images.push_back( face_resized );  // save person's picture
				
				// Write image (picture) to disk
				try 
				{
					imwrite( ImgFileName, CurrentFrame );
				}
				catch (runtime_error& ex) 
				{
					fprintf(stderr, "Exception writing Snapshot Image file: %s\n", ex.what());
					//return; // Abort write
				}

				// KLUDGE - just Write image (picture) to disk twice
				try 
				{
					imwrite( LastImgFileName, CurrentFrame );
				}
				catch (runtime_error& ex) 
				{
					fprintf(stderr, "Exception writing Last Snapshot Image file: %s\n", ex.what());
					//return; // Abort write
				}


				cout << "Image file" << ImgFileName << "saved"  << endl;

				GrabSnapshot = FALSE; // reset the capture request
			}

			// Process Face Recognizer
			if( pFaceRecognizer ) 
			{
				pFaceRecognizer->ProcessFrame( CurrentFrame, bFaceDetected, bPersonRecognized, DetectedPersonID, DetectedPersonName  );
				if( bFaceDetected )
				{
					CameraUpdate.UpdateType = CAMERA_UPDATE_FACE_RECO;
					CameraUpdate.UpdateData.FaceUpdate.FaceDetected = TRUE;
					if( bPersonRecognized )
					{
						// Tell the robot control if we detected a face we know
						CameraUpdate.UpdateData.FaceUpdate.PersonID = DetectedPersonID;
						strncpy_s( CameraUpdate.UpdateData.FaceUpdate.PersonName, DetectedPersonName.c_str(), DetectedPersonName.size() );
						CameraUpdate.UpdateData.FaceUpdate.PersonRecognized = TRUE;
					}
				}
			}

			// Process Object Matcher
			if( bMatchObject ) 
			{
				bool bObjectRecognized = false;
				int DetectedObjectID = -1;
				string DetectedObjectString = "";
				if( !pObjectMatcher )
				{
					cout << "ERROR: pObjectMatcher == NULL" << endl;
				}
				else
				{
					pObjectMatcher->ProcessFrame( CurrentFrame, bObjectRecognized, DetectedObjectID, DetectedObjectString );

					// Notify detection done, even if no object detected (Robot waits for a response)
					CameraUpdate.UpdateType = CAMERA_UPDATE_OBJECT_RECO;
					CameraUpdate.UpdateData.ObjectUpdate.bObjectRecognized = bObjectRecognized;

					CameraUpdate.UpdateData.ObjectUpdate.ObjectID = DetectedObjectID;
					strncpy_s( CameraUpdate.UpdateData.ObjectUpdate.ObjectName, DetectedObjectString.c_str(), DetectedObjectString.size() );
				}
				cout << "ObjectMatcher Done" << endl;

			}

 
			// Show the frame with face(s) marked
			#if( 1 == SHOW_VIDEO_WINDOW ) 
				imshow("video_capture", CurrentFrame);
			#endif
			// check for exit key
			char key = (char) waitKey(20);
			if(key == 27) 
			{
				break;  // Exit app if escape key pressed
			}

			if( key == 32 )
			{
				// Capture the next face when SPACE key pressed, for debug purposes.
				// normally, we get User ID, name, etc. from calling GUI app, but for keypress, we use hardcoded bogus data 
				if( pFaceRecognizer ) 
				{
					cout << "Space Key pressed, capturing Face" << endl;
					pFaceRecognizer->SaveNextFace( -1, "" );
				}
				else
				{
					cout << "Space Key ignored, Face Recognizer not enabled" << endl;
				}
			}

			if( bFaceDetected )
			{
				//////////////////////////////////////////////////////////////////////////////
				// Send data to Robot:
				if( (NULL != pUpdateDataSharedMemory) && (NULL != hUpdateDataEvent) )
				{
					CopyMemory((PVOID)pUpdateDataSharedMemory, &CameraUpdate, (sizeof(CAMERA_UPDATE_T)));
					SetEvent( hUpdateDataEvent ); // Indicate that new data is available
				}

				//////////////////////////////////////////////////////////////////////////////
			}
		} // if video enabled
		Sleep(100);  // 20 tune this to adjust framerate and avoid over saturating the CPU

    } // Frame Capture and Processing Loop


	///////////////////////////////////////////
	// Shut Down
	ReleaseAllDetectors();

	if( NULL != g_LogFile )
	{
		fclose( g_LogFile );
		//TRACE( " ***> View Log file at: %s\n\n", LogFileName );
		///system( "notepad c:\\temp\\RobotLog.txt" );
		///system( "type RobotLog.txt; pause" );
		///system( "pause" );
	}

    return 0;
}