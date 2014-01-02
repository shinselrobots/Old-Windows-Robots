// RobotCamera.h: OpenCV Camera processing
//////////////////////////////////////////////////////////////////////

#ifndef __ROBOT_CAMERA_H__
#define __ROBOT_CAMERA_H__


#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include "CameraDetectors.h"

// Constants
static char *LogFileName = "c:\\temp\\RobotCameraLog.txt";

static string face_cascade_name = "\\OpenCV2\\data\\haarcascades\\haarcascade_frontalface_default.xml";
static string strFaceDataDir = "C:\\Dev\\_Robot\\RobotCamera\\FaceData\\";
static string strFaceIndexfile = strFaceDataDir + "FaceIndex.txt";
static string HardCodedName = "Name";
// String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
// CascadeClassifier eyes_cascade;
static string window_name = "Capture - Face detection";
 //RNG rng(12345);



enum INIT_RESULT { 
		FAILED = 0, 
		SUCCESS,
		STAND_ALONE_MODE
};

// Global variables

extern VideoCapture *g_pVidCap;

extern CFaceRecognizer *pFaceRecognizer;
extern CMatchObjectDetector *pObjectMatcher;
extern CAMERA_REQUEST_ENABLE_FEATURES_T g_EnabledFeatures;


extern FILE* g_LogFile;		// Log file for non-debug mode
extern int	 gStartTime;	// Time that the robot started executing.  Used as baseline for time numbers

extern string face_cascade_name;
extern string strFaceDataDir;
extern string strFaceIndexfile;
extern string HardCodedName;
// String eyes_cascade_name;
// CascadeClassifier eyes_cascade;
extern string window_name;
 //RNG rng(12345);


using namespace cv;
using namespace std;


#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }


// Use the following to debug memory leaks
#define DEBUG_MEMORY_LEAKS			0

#if (DEBUG_MEMORY_LEAKS == 1)
	#ifndef _CRTDBG_MAP_ALLOC 
		#define _CRTDBG_MAP_ALLOC_NEW
		#define _CRTDBG_MAP_ALLOC
	#endif
	#include <stdlib.h>
	#include <crtdbg.h>
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility Functions Declarations
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
// Read Faces config file, which contains info on each face image
// Gets the PersonID, PersonName, Image Filename, and each Image into vectors (all must stay in sync) 
extern int read_csv( int &NextPersonID, vector<int>& PersonIDs, vector<string>& PersonNames, vector<string>& ImageNames, vector<Mat>& Images, char separator = ';');


// Writes Faces config file, which contains info on each face image
// Saves the PersonID, PersonName, and Image Filename for each Image.  NOTE: Images are written as each is captured, prior to calling this function
extern void write_csv( vector<int>& PersonIDs, vector<string>& PersonNames, vector<string>& ImgFileNames, char separator = ';');


//////////////////////////////////////////////////////////////////
// Open up shared memory mapped file for sending data to the Robot
//extern void InitSharedMemory( LPCTSTR &pBuf, LPCTSTR SharedFileName, int BufSize );
extern int InitIPC( LPCTSTR &pDataBuf, HANDLE &hDataUpdateEvent, LPCTSTR &pRequestBuf, HANDLE &hDataRequestEvent );


//////////////////////////////////////////////////////////////////
// Manage Detectors
// When Enabled/Disabled by Robot App, create/destroy selected detectors
void ManageFeatures( CAMERA_REQUEST_ENABLE_FEATURES_T EnableFeatures );


/////////////////////////////////////////////////////////////////////////////
// Release all detectors when vid cap is shut down
void ReleaseAllDetectors();



/*
#define ROBOT_LOG( __test1, __format, ...) if( __test1 ) {  \
	CString LogString; \
	CString FormatString = _T( __format ); \
	LogString.Format( FormatString, ## __VA_ARGS__ );   \
	g_PostStatus( (LPCTSTR)LogString, __FUNCTION__ );    }
*/

#endif // __ROBOT_CAMERA_H__