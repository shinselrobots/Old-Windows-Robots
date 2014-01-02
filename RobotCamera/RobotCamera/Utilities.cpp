// Utilities.cpp
// Utility funcitons used by RobotCamera process

#include "stdafx.h"
#include "CameraCommon.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;
#include "CameraDetectors.h"
#include "RobotCamera.h"



// Read Faces config file, which contains info on each face image
// Gets the PersonID, PersonName, Image Filename, and each Image into vectors (all must stay in sync) 
int read_csv( int &NextPersonID, vector<int>& PersonIDs, vector<string>& PersonNames, vector<string>& ImageNames, vector<Mat>& Images, char separator ) 
{
	// line format: <id>;<PersonName>;<filename>
	int NumberOfFacesFound = 0;
	int nID = 0;
	string strImagePath;
	int HighestIDFound = 0;
	NextPersonID = 0;

	cout << "Loading Face Index file: " << strFaceIndexfile  << endl;

    std::ifstream file(strFaceIndexfile.c_str(), ifstream::in);
    if (!file) {
        string error_message = "Can not open trained faces input file";
		return NumberOfFacesFound;
	}

    string line, strType, strID, strImgFile, strName;
	Mat Image;

    while ( getline(file, line) ) 
	{
        stringstream liness( line );
		getline( liness, strID, separator );
		getline( liness, strName, separator );
		getline( liness, strImgFile,separator );

		if( !strID.empty() && !strName.empty() && !strImgFile.empty() ) 
		{
			cout << "Loading Face - ID: " << strID << " Name: " << strName << "File: " << strImgFile  << endl;
			nID = atoi( strID.c_str() );
            PersonIDs.push_back( nID );
            PersonNames.push_back( strName );
            ImageNames.push_back( strImgFile );
			strImagePath = strFaceDataDir + strImgFile;

			Image = imread( strImagePath, 0 );
			if( NULL == Image.data )
			{
				cerr << "Error reading image " << strImagePath << endl;
				return -1; // error
			}

            Images.push_back( Image );
			NumberOfFacesFound++;
			if( nID > HighestIDFound )
			{
				HighestIDFound = nID;  // keep track of the highest ID found, so we know what the next new one should be numbered
			}
        }
    }


	// test
	cout << "TESTING"  << endl; 
	for( int n = 0; n < NumberOfFacesFound; n++ )
	{
		string TempName = PersonNames[n];
		int nID = PersonIDs[n];
		cout << TempName  << "ID " << nID << endl; 
	}
	NextPersonID = HighestIDFound + 1;
	return NumberOfFacesFound;
}

/////////////////////////////////////////////////////////////////////
// Writes Faces config file, which contains info on each face image
// Saves the PersonID, PersonName, and Image Filename for each Image.  NOTE: Images are written as each is captured, prior to calling this function
void write_csv( vector<int>& PersonIDs, vector<string>& PersonNames, vector<string>& ImgFileNames, char separator ) 
{


	// Write the full currently known list of face images
	// line format: <id>;<PersonName>;<filename>

    std::ofstream ofile(strFaceIndexfile.c_str(), ios::out);
    if (!ofile) {
        string error_message = "Error opening output file";
        CV_Error(CV_StsBadArg, error_message);
    }

	cout << "Saving Face Index file: " << strFaceIndexfile  << endl;
    //string strID, strImgFileName, strPersonName;
	vector <int>::iterator PersonIDIt;
	vector <string>::iterator PersonNameIt;
	vector <string>::iterator ImgFileNameIt;


	// It = beginning of myVec (first value)
	// It != end of myVec (last value)
	// ++It increase It to point to the next value of the vector
	PersonIDIt = PersonIDs.begin();
	PersonNameIt = PersonNames.begin();
	ImgFileNameIt = ImgFileNames.begin();

	while ( 
		(PersonIDIt != PersonIDs.end()) &&
		(PersonNameIt != PersonNames.end()) &&
		(ImgFileNameIt != ImgFileNames.end())  )
	{
		cout << "Writing Face: " << *ImgFileNameIt  << endl;
		ofile << *PersonIDIt++ << separator;		// Person's unique ID
		ofile << *PersonNameIt++ << separator;	// Person's Name
		ofile << *ImgFileNameIt++ << separator;	// File Name of Image (Photo)
		ofile << endl; // print a new line
	}
	
	ofile.close();

}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initialize shared memory and events for Inter Process Communication with the Robot control application

int InitIPC( LPCTSTR &pDataBuf, HANDLE &hDataUpdateEvent, LPCTSTR &pRequestBuf, HANDLE &hDataRequestEvent )
{
	pDataBuf = NULL;
	pRequestBuf = NULL;
	hDataUpdateEvent = NULL;
	hDataRequestEvent = NULL;
	HANDLE hMapFile = NULL;

	static BOOL bManualReset = FALSE;
	static BOOL bInitialState = FALSE; // Not Signaled 


	// TODO!  DO I NEED TO RELEASE MEMORY OR HANDLES on EXIT?

	/////////////////////////////////////////////////////////////////////////////////////////
	// For sending data to the robot control application
	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security 
		PAGE_READWRITE,          // read/write access
		0,                       // max. object size high
		sizeof(CAMERA_UPDATE_T),	// buffer size  
		_T(CAMERA_DATA_SHARED_FILE_NAME) );// name of mapping object

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		_tprintf(TEXT("Could not create DATA UPDATE file mapping object (%d).\n"), 
			GetLastError());
		return FAILED;
	}
	pDataBuf = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		sizeof(CAMERA_UPDATE_T) );           

	if (pDataBuf == NULL) 
	{ 
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError()); 
		CloseHandle(hMapFile);
		return FAILED;
	}

	// Event to telling Robot that new data is ready
	hDataUpdateEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(CAMERA_UPDATE_EVENT_NAME) );
	if ( !hDataUpdateEvent ) 
	{ 
		cerr << "Event creation failed!: " << CAMERA_UPDATE_EVENT_NAME << endl;
		return FAILED;
	}
	SetEvent( hDataUpdateEvent ); // Indicate that the app has started


	/////////////////////////////////////////////////////////////////////////////////////////
	// For getting requests from the robot control application
	//InitSharedMemory( pRequestSharedMemory, _T(CAMERA_REQUEST_SHARED_FILE_NAME), sizeof(CAMERA_REQUEST_T));


	// Create Event from Robot that new REQUEST is ready
	hDataRequestEvent = CreateEvent ( NULL, bManualReset, bInitialState, _T(CAMERA_REQUEST_EVENT_NAME) );
	if ( !hDataUpdateEvent ) 
	{ 
		cerr << "Event creation failed!: " << CAMERA_REQUEST_EVENT_NAME << endl;
		return FAILED;
	}
	//SetEvent( hDataUpdateEvent ); // Indicate that the app has started

	// Now check that the event was signaled by the Robot process, indicating that it's ok to proceed
	// If not, this program was probably launched in stand-alone mode for testing
	const DWORD msTimeOut = 100;
	DWORD dwResult = WaitForSingleObject(hDataRequestEvent, msTimeOut);
	if( WAIT_OBJECT_0 != dwResult ) 
	{
		cerr << endl;
		cerr << "==============================================================" << endl;
		cerr << "WARNING!  REQUEST Event failed, assuming STAND ALONE mode!" << endl;
		cerr << "==============================================================" << endl << endl;
		return STAND_ALONE_MODE; 
	}

	// Open Memory Mapped File

	hMapFile = OpenFileMapping(
		FILE_MAP_ALL_ACCESS,		// read/write access
		FALSE,						// do not inherit the name
		_T(CAMERA_REQUEST_SHARED_FILE_NAME) );	// name of mapping object 

	if ( (INVALID_HANDLE_VALUE == hMapFile) || (NULL == hMapFile)  )
	{ 
		_tprintf(TEXT("Could not create REQUEST file mapping object (%d).\n"), 
			GetLastError());
		return FAILED;
	}
	pRequestBuf = (LPTSTR) MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,                   
		0,                   
		sizeof(CAMERA_REQUEST_T) );           

	if (pRequestBuf == NULL) 
	{ 
		_tprintf(TEXT("Could not map view of file (%d).\n"), GetLastError()); 
		CloseHandle(hMapFile);
		return FAILED;
	}
	else
	{
		cout << "REQUEST Shared Memory File Opened Sucessfully!"  << endl;
	} 
	
	return SUCCESS; 
}


//////////////////////////////////////////////////////////////////
// Manage Detectors
// When Enabled/Disabled by Robot App, create/destroy selected detectors
void ManageFeatures( CAMERA_REQUEST_ENABLE_FEATURES_T EnableFeatures )
{
	// Create and initialize any detectors that have just been enabled
	// For each detector type:
	// If Enabled but not created, create the object
	// If Disabled, but existing, delete the object

	g_EnabledFeatures.VideoEnable = EnableFeatures.VideoEnable;
	g_EnabledFeatures.FaceRecognition = EnableFeatures.FaceRecognition;
	g_EnabledFeatures.FaceTracking = EnableFeatures.FaceTracking;
	g_EnabledFeatures.ObjectMatch = EnableFeatures.ObjectMatch;


	// Video Enabling (nothing works if this is turned off!)
	if( (0 != EnableFeatures.VideoEnable) && (NULL == g_pVidCap) )
	{
		cout << "Starting Video Capture" << endl;

		// Get a handle to the Video device:
		int deviceId = 0;	// ID of video capture device TODO: SET THIS TO THE CORRECT CAMERA!
		g_pVidCap = new VideoCapture(deviceId);
//			VideoCapture Vidcap(deviceId);
		// Check if we can use this device at all:
		if(!g_pVidCap->isOpened()) 
		{
			cerr << "Capture Device ID " << deviceId << "cannot be opened." << endl;
			return;
		}
		cout << "Opened Capture Device ID " << deviceId  << endl;
	}
	else if( (0 == EnableFeatures.VideoEnable) && (NULL != g_pVidCap))
	{
		cout << "Stopping Video Capture" << endl;
		g_pVidCap->release();
		delete g_pVidCap;
	}

	// Face Detector
	if( (0 != EnableFeatures.FaceRecognition) && (NULL == pFaceRecognizer) )
	{
		cout << "Starting Face Recognizer" << endl;
		pFaceRecognizer = new CFaceRecognizer;
		pFaceRecognizer->Init();
	}
	else if( (0 == EnableFeatures.FaceRecognition) && (NULL != pFaceRecognizer))
	{
		cout << "Stopping Face Recognition" << endl;
		SAFE_DELETE( pFaceRecognizer );
	}


	// Object Match Detector
	if( (0 != EnableFeatures.ObjectMatch) && (NULL == pObjectMatcher) )
	{
		cout << "Starting Object Match initialization" << endl;
		pObjectMatcher = new CMatchObjectDetector;
		pObjectMatcher->Init();
	}
	else if( (0 == EnableFeatures.ObjectMatch) && (NULL != pObjectMatcher))
	{
		cout << "Stopping Object Match" << endl;
		SAFE_DELETE( pObjectMatcher );
	}

}


/////////////////////////////////////////////////////////////////////////////
// Release all detectors when vid cap is shut down
void ReleaseAllDetectors()
{


	if( NULL != pFaceRecognizer )
	{
		SAFE_DELETE( pFaceRecognizer );
	}

	if( NULL != pObjectMatcher )
	{
		SAFE_DELETE( pObjectMatcher );
	}

	if( NULL != g_pVidCap )
	{
		g_pVidCap->release();
		delete g_pVidCap;
	}

}






#ifdef BLABLAABLA

//-----------------------------------------------------------------------------
// Name: g_PostStatus
// Desc: Formats message and writes to log file and console window
//-----------------------------------------------------------------------------
void g_PostStatus( LPCTSTR lpszStatus, char *FunctionName )
{
	CString strFormattedMessage;
	CString strTime;
	static int nLineNum = 0;
	DWORD CurrentTime = GetTickCount() - gStartTime;
	// TODO - Look at COleDateTime::GetCurrentTime().Format()

	// Format the current time
	//nLineNum++;
	strTime.Format( _T("[%02d:%02d.%d] "), CurrentTime/600, (CurrentTime %600)/10, CurrentTime%10 );

	// Format the message with current time
	strFormattedMessage = strTime;

	// Add the function name, if available
	if( 0 != FunctionName )
	{
		strFormattedMessage += FunctionName;
		strFormattedMessage += ": ";
	}

	// Now, the user message
	strFormattedMessage += lpszStatus;
	strFormattedMessage += "\n"; // NOTE: automatically add a Newline


	#if TRACE_ENABLED == 1
		// Echo message to the debug window
		TRACE( (LPCTSTR)strFormattedMessage );
	#endif

	printf("%s", strFormattedMessage );

	// Write to the log file
	if( NULL != g_LogFile )
	{
		fprintf(g_LogFile, "%s", strFormattedMessage );
	}


}

#endif // #ifdef BLABLAABLA