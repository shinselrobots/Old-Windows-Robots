// CameradDetectors.cpp: Video Image Object Detector classes
//
//////////////////////////////////////////////////////////////////////

//#pragma warning( disable: 4996 )	// For STEREO CALIBRATION
//#pragma warning( disable: C4786 )	// For STEREO CALIBRATION


#include "stdafx.h"
#include "CameraCommon.h"


//#include <math.h>
//#include "Globals.h"
//#include "module.h"
//#include "thread.h"
//#include "cv.h"
//#include "cvcam.h"
//#include "cxcore.h"
//#include "highgui.h"

#include "opencv2/core/core.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"


//#ifdef USING_SURF
	#include "opencv2/nonfree/nonfree.hpp"
//#endif

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

//*** FOR STEREO:
//#include "cv.h"
//#include "cxmisc.h"
//#include "highgui.h"
//#include "cvaux.h"

#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

//#include "ObjectKnowledge.h"
#include "RobotCamera.h"
#include "CameraDetectors.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#define ROBOT_DATA_PATH	"C:\\Dev\\_Robot\\LokiData"
//#define ROBOT_COMMON_PATH L"C:\\Dev\\_Robot\\Common"
#define CAMERA_FACE_DETECTOR_PATH ROBOT_DATA_PATH "\\OpenCV\\data\\haarcascades\\haarcascade_frontalface_default.xml"

//   CvSize size = cvSize(pSourceFrame->width,pSourceFrame->height); // get current frame size



//////////////////////////////////////////////////////////////////////////////
// CFaceRecognizer
//#define MIN_FACE_SIZE_DIVIDER		5	// sets how small a face to find.  5 default (big face). larger number = more CPU
//////////////////////////////////////////////////////////////////////////////

CFaceRecognizer::CFaceRecognizer()
{

//	ASSERT( m_pVideoFrame );
//	CString LogString; 
//	CString FormatStr = _T("ddd %d");


	m_PersonID = 0;
	m_NextPersonID = 0;
	m_FileNumber = 0; // guarantee file name is always unique
	m_PersonName = "Name";
	m_ImgFileName = "Name";
	m_CaptureNewFace = FALSE;
	m_NumberOfTrainedFaces = 0;

	m_FaceImageWidth = 200;
	m_FaceImageHeight = 200;
	m_PredictedID = -1;
	m_Confidence = -1;
	
	///ROBOT_LOG( TRUE, "CFaceRecognizer construction complete\n" )
}

CFaceRecognizer::~CFaceRecognizer()
{
	// Release resources allocated
	///ROBOT_LOG( TRUE,"~CFaceRecognizer done\n")
}

BOOL CFaceRecognizer::Init()
{
	// Initialize Face Recognizer


   // Read in the previously saved index file
	m_NumberOfTrainedFaces = read_csv( m_NextPersonID, PersonIDs, PersonNames, ImgFileNames, Images );
	if( -1 == m_NumberOfTrainedFaces )
 	{
        cerr << "ERROR in file " << strFaceIndexfile << endl;
		char ch;
		ch = getchar(); //allow user to read error
		return -1;		// abort on error
    }
	else if( 0 == m_NumberOfTrainedFaces )
 	{
        cerr << "No trained faces found in " << strFaceIndexfile << endl;
		m_NumberOfTrainedFaces = 0;	
		m_NextPersonID = 0;
    }
	else
	{
		m_FileNumber = m_NumberOfTrainedFaces;  
	}

    // Get the height from the first image. We'll need this
    // later in code to reshape the images to their original
    // size AND we need to reshape incoming faces to this size:
////    m_FaceImageWidth = Images[0].cols;
////    m_FaceImageHeight = Images[0].rows;
    // Create a FaceRecognizer and train it on the given images:
	model = createFisherFaceRecognizer();
	if ( m_NumberOfTrainedFaces > 0 )
	{
		model->train(Images, PersonIDs);
	}
    // That's it for learning the Face Recognition model. You now
    // need to create the classifier for the task of Face Detection.
    // We are going to use the haar cascade you have specified in the
    // command line arguments:
    //
    haar_cascade.load(face_cascade_name);

///	ROBOT_LOG( TRUE,"CFaceRecognizer Init complete\n")
	return 1;
}

void CFaceRecognizer::SaveNextFace( int RequestedPersonID, string RequestedPersonName  )
{
	// Saves the next face seen, for learning new faces
	if( -1 == RequestedPersonID )
	{
		m_PersonID = m_NextPersonID; // use the next free ID
	}
	else
	{
		m_PersonID = RequestedPersonID; // use the ID passed in from the robot app
	}

	if( RequestedPersonName.empty() )
	{
		char strPersonID[8];
		_itoa_s( m_PersonID, strPersonID,8, 10 );
		m_PersonName = (string)"Person" + strPersonID;  // name + ID for testing
	}
	else
	{
		m_PersonName = RequestedPersonName;
	}

	m_CaptureNewFace = true; // request the capture

}


void CFaceRecognizer::ProcessFrame( Mat	&original, BOOL &bFaceDetected, BOOL &bPersonRecognized, int &DetectedPersonID, string &DetectedPersonName )
{
	static int FrameNumber = 0;
	cout << "Frame " << FrameNumber << endl;
	FrameNumber++;


    // Convert the current frame to grayscale:
    Mat gray;
    cvtColor(original, gray, CV_BGR2GRAY);
    // Find the faces in the frame:
    vector< Rect_<int> > faces;
    haar_cascade.detectMultiScale(
		gray, 
		faces,
		1.1,			// double scaleFactor=1.1, 
		3,				// int minNeighbors=3, 
		0,				// int flags=0, 
		Size(100, 100)	// Size minSize=Size(), 
						// Size maxSize=Size())			
		);

    // At this point you have the position of the faces in
    // faces. Now we'll get the faces, make a prediction and
    // annotate it in the video. Cool or what?
	Mat face_resized;
    for(unsigned int i = 0; i < faces.size(); i++) 
	{
        // Process face by face:
        Rect face_i = faces[i];
        // Crop the face from the image. So simple with OpenCV C++:
        Mat face = gray(face_i);
        // Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
        // verify this, by reading through the face recognition tutorial coming with OpenCV.
        // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
        // input data really depends on the algorithm used.
        //
        // I strongly encourage you to play around with the algorithms. See which work best
        // in your scenario, LBPH should always be a contender for robust face recognition.
        //
        // Since I am showing the Fisherfaces algorithm here, I also show how to resize the
        // face you have just found:
        cv::resize(face, face_resized, Size(m_FaceImageWidth, m_FaceImageHeight), 1.0, 1.0, INTER_CUBIC);
        // Now perform the prediction, see how easy that is:
		if ( m_NumberOfTrainedFaces > 0 )
		{
//	            PredictedID = model->predict(face_resized);
	        model->predict( face_resized, m_PredictedID, m_Confidence );
		}
			
		// And finally write all we've found out to the original image!
        // First of all draw a green rectangle around the detected face:
        rectangle(original, face_i, CV_RGB(0, 255,0), 1);

		// Get user name
		string TempName = PersonNames[m_PredictedID];

        // Create the text we will annotate the box with:
//           string box_text = "ID: " + PersonID + "Name: " + TempName + "Prediction: " + Prediction;
        string box_text = cv::format("ID=%d, Name=%s, Conf=%2.1f", m_PredictedID, TempName.c_str(), m_Confidence/100.0);
        // Calculate the position for annotated text (make sure we don't put illegal values in there):
        int pos_x = std::max(face_i.tl().x - 50, 0);
        int pos_y = std::max(face_i.tl().y - 10, 0);
        // And now put it into the image:

		putText(  original, 
			box_text, 
			Point(pos_x, pos_y), 
			FONT_HERSHEY_PLAIN, 
			1.5, 
			CV_RGB(0,255,0), 
			2);	// thickness

		if( m_Confidence > 3000.0 )
		{
			DetectedPersonID = m_PredictedID;
			DetectedPersonName = TempName;
			bPersonRecognized = TRUE;
		}
		// TODO: save the index of the largest face found, assume this is the person to add 
		bFaceDetected = true;
    } 


	if( m_CaptureNewFace )
	{
		// Face capture requested from key pressed or request message from Robot app
		if( bFaceDetected )
		{
			char strFileNumber[8];
			_itoa_s( m_FileNumber, strFileNumber, 8, 10 );
			m_ImgFileName = m_PersonName + "_" + strFileNumber + ".jpg"; // Guarantee unique file name by adding a unique number
			// TODO: add User ID to the person name to guarantee unique persons?

			// Save current face info
	        PersonIDs.push_back( m_PersonID );
			PersonNames.push_back( m_PersonName );
			ImgFileNames.push_back( m_ImgFileName );
			Images.push_back( face_resized );  // save person's picture
				
			// Write image (picture) to disk
			try 
			{
				imwrite( (strFaceDataDir + m_ImgFileName), face_resized );
			}
			catch (runtime_error& ex) 
			{
				fprintf(stderr, "Exception writing Face Image file: %s\n", ex.what());
				return; // Abort write
			}

			cout << "Image file" << m_ImgFileName << "saved"  << endl;

			// Persist data - write index info for all images to disk
			write_csv( PersonIDs, PersonNames, ImgFileNames );
			m_PersonID++;
			m_FileNumber++;
			m_NumberOfTrainedFaces++;

			m_CaptureNewFace = FALSE; // reset the capture request
		}
		else
		{
			cerr << "Face Capture requested, but cannot save, no face found"  << endl;
		}
	}

}








#ifdef THIS_DETECTOR_NOT_IMPLEMENTED
//////////////////////////////////////////////////////////////////////////////
// CFaceDetector
#define MIN_FACE_SIZE_DIVIDER		5	// sets how small a face to find.  5 default (big face). larger number = more CPU
//////////////////////////////////////////////////////////////////////////////


CFaceDetector::CFaceDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{

	ASSERT( m_pVideoFrame );

	CString LogString; 
	CString FormatStr = _T("ddd %d");
	LogString.Format( FormatStr , 33);   
	g_PostStatus( (LPCTSTR)LogString, __FUNCTION__ );    
	


	// Initialize Harr Classifier Face Detector
	// Allocate memory and load profile xml from disk
	m_pFaceDetectStorage = cvCreateMemStorage(0);
	if( !m_pFaceDetectStorage )
	{
		ROBOT_LOG( TRUE, "ERROR! Unable to allocate memory for face detection!" )
		return;
	}

	// set default path for face detector XML
	m_FaceDetectorXMLPath = CAMERA_FACE_DETECTOR_PATH;

	m_pFaceDetectCascade = 
		(CvHaarClassifierCascade *)cvLoad( m_FaceDetectorXMLPath, 0, 0, 0 );
	if( !m_pFaceDetectCascade )
	{
		// Release memory storage
		if(0 != m_pFaceDetectStorage)
		{
			cvReleaseMemStorage(&m_pFaceDetectStorage);
		}

		CString MsgString;
		 
		ROBOT_LOG( TRUE, "ERROR! Unable to load Haar classifier cascade XML from: %s",  m_FaceDetectorXMLPath )
//		MsgString.Format( _T("   %s"), m_FaceDetectorXMLPath  );
//		ROBOT_LOG( TRUE, MsgString )
		return;
	}
	ROBOT_LOG( TRUE, "CFaceDetector construction complete\n" )
}

CFaceDetector::~CFaceDetector()
{
	// Release resources allocated to face detection
	if(m_pFaceDetectCascade) 
	{
		cvReleaseHaarClassifierCascade(&m_pFaceDetectCascade);
		m_pFaceDetectCascade = 0;
	}
	if(m_pFaceDetectStorage) 
	{
		cvReleaseMemStorage(&m_pFaceDetectStorage);
		m_pFaceDetectStorage = 0;
	}
	ROBOT_LOG( TRUE,"~CFaceDetector done\n")
}

CvSeq* CFaceDetector::DetectFaces()
{
	// Look for faces in the video frame
	CvSeq* pFaceRectSeq = 0;

	if( 0 == m_pFaceDetectStorage )
	{
		// Initialization failed.
		return 0;
	}

	int minFaceSize = m_pVideoFrame->height / MIN_FACE_SIZE_DIVIDER;	// 5 default
	pFaceRectSeq = cvHaarDetectObjects(
		m_pVideoFrame, 
		m_pFaceDetectCascade, 
		m_pFaceDetectStorage,
		1.1,                       // Default: 1.1 = increase search scale by 10% each pass
		8,                         // Default: 6 = require six neighbors
		CV_HAAR_DO_CANNY_PRUNING,  // skip regions unlikely to contain a face
		cvSize(minFaceSize, minFaceSize) );

	return pFaceRectSeq;

}

//////////////////////////////////////////////////////////////////////////////
// CMotionDetector
//////////////////////////////////////////////////////////////////////////////


CMotionDetector::CMotionDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Allocate and Initialize resources
//	int i;
//	int m_Last;

	ROBOT_ASSERT( m_pVideoFrame );
	m_Last = 0;
	storage = 0;
	ppMotionRingBuffer = 0; 

	// Ring Buffer of images
    ppMotionRingBuffer = (IplImage**)malloc(MOTION_RING_BUF_SIZE*sizeof(ppMotionRingBuffer[0]));
    memset( ppMotionRingBuffer, 0, MOTION_RING_BUF_SIZE*sizeof(ppMotionRingBuffer[0]));

	CvSize size = cvGetSize(m_pVideoFrame); // get current frame size
	for( int i = 0; i < MOTION_RING_BUF_SIZE; i++ ) {
        ppMotionRingBuffer[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
        cvZero( ppMotionRingBuffer[i] );
    }
    
    mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
    cvZero( mhi ); // clear MHI to start (no history)
    orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
    segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
    mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );


	ROBOT_LOG( TRUE,"CMotionDetector construction complete\n")

}

CMotionDetector::~CMotionDetector()
{
	// Release resources
	int i;   
    for( i = 0; i < MOTION_RING_BUF_SIZE; i++ ) {
        cvReleaseImage( &ppMotionRingBuffer[i] );
    }
    cvReleaseImage( &mhi );
    cvReleaseImage( &orient );
    cvReleaseImage( &segmask );
    cvReleaseImage( &mask );
    
	ROBOT_LOG( TRUE,"~CMotionDetector done\n")
}



// mhi = MotionHistoryImage
CvPoint CMotionDetector::DetectMotion( IplImage* pMotionDisplayFrame )
{
	// various tracking parameters 
	const double	MHI_DURATION =   0.5;		// seconds  default = 1
	const double	MAX_TIME_DELTA = 0.10;	// seconds	default = 0.5
	const double	MIN_TIME_DELTA = 0.05;	// seconds	default = 0.05
	const int		diff_threshold = 30;	// 	default = 30

	ROBOT_ASSERT( m_pVideoFrame );

    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
	CvSize size = cvGetSize(m_pVideoFrame); // get current frame size
    int i, idx1 = m_Last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;          
    CvScalar color;
	CvPoint MotionCenterAve;	// Center of all motion activity (location of moving person?)
	int MotionCenterSumX = 0;
	int MotionCenterSumY = 0;
	int MotionCenterCount = 0;

    cvCvtColor( m_pVideoFrame, ppMotionRingBuffer[m_Last], CV_BGR2GRAY ); // copy and convert frame to grayscale

    idx2 = (m_Last + 1) % MOTION_RING_BUF_SIZE; // index of (m_Last - (N-1))th frame
    m_Last = idx2;

    silh = ppMotionRingBuffer[idx2];
    cvAbsDiff( ppMotionRingBuffer[idx1], ppMotionRingBuffer[idx2], silh ); // get difference between frames
    
    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( pMotionDisplayFrame );
    cvCvtPlaneToPix( mask, 0, 0, 0, pMotionDisplayFrame );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);
    
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for( i = -1; i < seq->total; i++ ) 
	{

        if( i < 0 ) { // case of the whole image
            comp_rect = cvRect( 0, 0, size.width, size.height );
            color = CV_RGB(255,255,255);
            magnitude = 100;
        }
        else { // i-th motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
            if( comp_rect.width + comp_rect.height < MIN_OBJECT_SIZE ) // reject very small components
                continue;
            color = CV_RGB(255,0,0);
            magnitude = 30;
        }

        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        // calculate orientation
        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);
        angle = 360.0 - angle;  // adjust for images with top-left origin

        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // check for the case of little motion
        if( count < comp_rect.width*comp_rect.height * 0.05 )
            continue;

		// for each motion region found, determine center of the region
        center = cvPoint( (comp_rect.x + comp_rect.width/2),
                          (comp_rect.y + comp_rect.height/2) );

		// Total all the motion regions so they can be averaged
		if( i >= 0 ) 
		{	// skip case of the whole image
			MotionCenterCount++;
			MotionCenterSumX += center.x;
			MotionCenterSumY += center.y;
		}

        // draw a clock with arrow indicating the direction
		cvCircle( pMotionDisplayFrame, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        cvLine( pMotionDisplayFrame, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
                cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
    }
	// Now, calculate center of the motion area
	if( MotionCenterCount > 0 )
	{
		MotionCenterAve.x = MotionCenterSumX / MotionCenterCount;
		MotionCenterAve.y = MotionCenterSumY / MotionCenterCount;
	}
	else
	{
		MotionCenterAve.x = 0;
		MotionCenterAve.y = 0;
	}

	return MotionCenterAve;

} // DetectMotion

//////////////////////////////////////////////////////////////////////////////
// CCamShiftDetector
//////////////////////////////////////////////////////////////////////////////


CCamShiftDetector::CCamShiftDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Initialize CamShift members
	m_prevColorRect.height = 0;
	m_prevColorRect.width = 0;
	m_prevColorRect.x = 0;
	m_prevColorRect.y = 0;

	nHistBins = 30;                 // number of histogram bins
	rangesArr[0] = 0;
	rangesArr[1] = 180;				// histogram range
	vmin = 65;
	vmax = 256;
	smin = 55;					// limits for calculating hue
	m_State = 0;				// Idle

	memset( &m_FooRect, 0, sizeof(CvRect) );
	memset( &m_CamShiftCenter, 0, sizeof(CvPoint) );


	// Allocate and Initialize resources
	float * pRanges = rangesArr;
	pHSVImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 3 );
	pHueImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );
	pMask    = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );
	pProbImg = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );

	pHist = cvCreateHist( 1, &nHistBins, CV_HIST_ARRAY, &pRanges, 1 );


	ROBOT_LOG( TRUE,"CCamShiftDetector construction complete\n")
}

CCamShiftDetector::~CCamShiftDetector()
{
	// Release resources
	cvReleaseImage( &pHSVImg );
	cvReleaseImage( &pHueImg );
	cvReleaseImage( &pMask );
	cvReleaseImage( &pProbImg );
	cvReleaseHist( &pHist );

	ROBOT_LOG( TRUE,"~CCamShiftDetector done\n")
}

void CCamShiftDetector::startTracking( CvRect TrackingRect )
{
	float maxVal = 0.f;
	m_CamShiftCenter.x = 0;
	m_CamShiftCenter.y = 0;

	if( (0 == TrackingRect.width) || (0 == TrackingRect.height) )
	{
		ROBOT_LOG( TRUE, "CamShift:startTracking: Bad rectangle selected!\n" )
		m_State = 0;
		return;
	}

	m_State = 1;

	// Create a new hue image
	updateHueImage(m_pVideoFrame);

	// Create a histogram representation for the face
    cvSetImageROI( pHueImg, TrackingRect );
    cvSetImageROI( pMask,   TrackingRect );
    cvCalcHist( &pHueImg, pHist, 0, pMask );
    cvGetMinMaxHistValue( pHist, 0, &maxVal, 0, 0 );
    cvConvertScale( pHist->bins, pHist->bins, maxVal? 255.0/maxVal : 0, 0 );
    cvResetImageROI( pHueImg );
    cvResetImageROI( pMask );

	// Store the previous face location

	m_prevColorRect.height = TrackingRect.height;
	m_prevColorRect.width = TrackingRect.width;
	m_prevColorRect.x = TrackingRect.x;
	m_prevColorRect.y = TrackingRect.y;

	m_FooRect = TrackingRect;
}

CvBox2D CCamShiftDetector::track()
{
	CvConnectedComp components;
	CvBox2D CamShiftBox;
    memset( &CamShiftBox, 0, sizeof(CvBox2D) );

	// Make sure the tracker got initialized before this call
	if( m_State != 1 )
	{
		return CamShiftBox;
	}

	// Create a new hue image
	updateHueImage(m_pVideoFrame);

	// Create a probability image based on the face histogram
	cvCalcBackProject( &pHueImg, pProbImg, pHist );
    cvAnd( pProbImg, pMask, pProbImg, 0 );

	// Use CamShift to find the center of the new face probability
	CvRect TmpPrevRect = m_prevColorRect;	// try using temp, since CamShift messes this up

	//WARNING!! CURRENT STATUS - This call corrupts m_prevColorRect!!
	// I think CamShift has a bug!!
    cvCamShift( pProbImg, TmpPrevRect,
                cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
                &components, &CamShiftBox );

	// Update face location and angle

//   m_FooRect = components.rect;

    m_prevColorRect = components.rect;
	CamShiftBox.angle = -CamShiftBox.angle;

	return CamShiftBox;
}

void CCamShiftDetector::updateHueImage(const IplImage * pImg)
{
	// Convert to HSV color model
	cvCvtColor( pImg, pHSVImg, CV_BGR2HSV );

	// Mask out-of-range values
	cvInRangeS( pHSVImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
	            cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );

	// Extract the hue channel
	cvSplit( pHSVImg, pHueImg, 0, 0, 0 );
}

void CCamShiftDetector::setVmin(int _vmin)
{ vmin = _vmin; }


void CCamShiftDetector::setSmin(int _smin)
{ smin = _smin; }



//////////////////////////////////////////////////////////////////////////////
// CColorDetector
//////////////////////////////////////////////////////////////////////////////

CColorDetector::CColorDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Initialize Color members

	m_CrTarget = 0;
	m_CbTarget = 0;

	m_CrThreshold = 20;	
	m_CbThreshold = 20;	

	m_CrMax = 255;
	m_CrMin = 0;
	m_CbMax = 255;
	m_CbMin = 0;
	
	m_bInitialized = FALSE;


	// Allocate and Initialize resources
	pYCrCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 3 );
//	pCrImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );
//	pCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );


	ROBOT_LOG( TRUE,"CColorDetector construction complete\n")
}

CColorDetector::~CColorDetector()
{
	// Release resources
	cvReleaseImage( &pYCrCbImg );
//	cvReleaseImage( &pCrImg );
//	cvReleaseImage( &pCbImg );

	ROBOT_LOG( TRUE,"~CColorDetector done\n")
}

void CColorDetector::SetColor( int  Cr, int  Cb )
{

	// Manual setting of target color
	m_CrTarget = Cr;
	m_CbTarget = Cb;
	m_bInitialized = TRUE;


	// Set initial threshold (this can change while running)
	m_CrMin = m_CrTarget - m_CrThreshold;
	if( m_CrMin < 0 ) m_CrMin = 0;
	m_CrMax = m_CrTarget + m_CrThreshold;
	if( m_CrMax > 255 ) m_CrMax = 255;

	m_CbMin = m_CbTarget - m_CbThreshold;
	if( m_CbMin < 0 ) m_CbMin = 0;
	m_CbMax = m_CbTarget + m_CbThreshold;
	if( m_CbMax > 255 ) m_CbMax = 255;

	CString strStatus;
	strStatus.Format("Color Blob Target Manually Set to: Cr=%d Cb=%d", m_CrTarget, m_CbTarget );

	ROBOT_LOG( TRUE, strStatus )


}

void CColorDetector::SetColor( CvRect ColorRect )
{
	m_CrTarget = 0;
	m_CbTarget = 0;

	if( (0 == ColorRect.width) || (0 == ColorRect.height) )
	{
		ROBOT_LOG( TRUE, "Color:startTracking: Bad rectangle selected!\n" )
		m_bInitialized = FALSE;
		return;
	}
	m_bInitialized = TRUE;


	// Extract Cr and Cb images
	UpdateCrCbImages(m_pVideoFrame);

	int  CrSum = 0;
	int  CbSum = 0;
	CvScalar Pixel;
	int  PixelCount = 0;
	// Initialize the Cr and Cb targets from the selected area
	for( int X = ColorRect.x; X < (ColorRect.x+ColorRect.width); X++ )
	{
		for( int Y = ColorRect.y; Y < (ColorRect.y+ColorRect.height); Y++ )
		{
			Pixel =  cvGet2D( pYCrCbImg, X, Y );
			CrSum += (int )Pixel.val[1];	// val[0]=Y(intensity), [1]=Red, [2]=Blue
			CbSum += (int )Pixel.val[2];
			PixelCount++;
		}
	}
	// Get the average values for Red and Blue in the target area
	// This is the color we will be looking for
	m_CrTarget = CrSum / PixelCount;
	m_CbTarget = CbSum / PixelCount;

	// Set initial threshold (this can change while running)
	m_CrMin = m_CrTarget - m_CrThreshold;
	if( m_CrMin < 0 ) m_CrMin = 0;
	m_CrMax = m_CrTarget + m_CrThreshold;
	if( m_CrMax > 255 ) m_CrMax = 255;

	m_CbMin = m_CbTarget - m_CbThreshold;
	if( m_CbMin < 0 ) m_CbMin = 0;
	m_CbMax = m_CbTarget + m_CbThreshold;
	if( m_CbMax > 255 ) m_CbMax = 255;

	CString strStatus;
	strStatus.Format("Color Blob Target Set to: Cr=%d Cb=%d", m_CrTarget, m_CbTarget );

	ROBOT_LOG( TRUE, strStatus )


}


CvPoint CColorDetector::FindColorBlob( IplImage* pColorBlobDisplayFrame, 
						CvPoint &BBpt1, CvPoint &BBpt2 )
{

	// Note, This is a simple test to find "average" location of color
	// does not correct for 2 color blobs found!
	// Note that the blob "center" is average weight of the color blob,
	// NOT the center of the bounding box!

	// Bounding Box around Color Blob = BBpt1, BBpt2
	// "Center of Mass" of the color blob = ptBlobCenter
	CvPoint ptBlobCenter = {0,0};

	CvScalar	Pixel;
	int 		nPixelsFound = 0;
	int			width = GetFrameSize().width;
	int			height = GetFrameSize().height;
	int 		Cr, Cb;
	int 		SumPosX = 0;
	int 		SumPosY = 0;
//	CvPoint		pt1 = {0,0};	// edges of bounding box
//	CvPoint		pt2 = {0,0};
	CvScalar	BlankPixel = {0,0,0,0};


	m_ColorCenter.x = 0;
	m_ColorCenter.y = 0;
//	BoundingBox = {0,0,0,0};

	BBpt1.x = 0;
	BBpt1.y = 0;
	BBpt2.x = 0;
	BBpt2.y = 0;

 	// Make sure the target color got initialized before this call
	if( ! m_bInitialized )
	{
		ROBOT_LOG( TRUE, "ERROR! FindColorBlob called with no target color set!\n" )
		return ptBlobCenter;
	}

	// Create new CrCb images
	UpdateCrCbImages(m_pVideoFrame);
	int RedFound = 0;
	int BlueFound = 0;

	// Find pixels that meet Cr and Cb target values
	for( int X = 0; X < width; X++ )
	{
		for( int Y = 0; Y < height; Y++ )
		{
			Pixel =  cvGet2D( pYCrCbImg, Y, X );
			// val[0]=Y(intensity), [1]=Red, [2]=Blue
			Cr = (int )Pixel.val[1];
			Cb = (int )Pixel.val[2];

			if( (Cr > m_CrMin) && (Cr < m_CrMax) )
			{
				RedFound++;
			}
			if(	(Cb > m_CbMin) && (Cb < m_CbMax) )
			{
				BlueFound++;
			}

			if( (Cr > m_CrMin) && (Cr < m_CrMax) &&
				(Cb > m_CbMin) && (Cb < m_CbMax)  )
			{
				// Pixel is within Target color range
				if( 0 == nPixelsFound++ )
				{
					// Initialize bounding box on first pixel
					BBpt1.x = X; BBpt2.x = X;
					BBpt1.y = Y; BBpt2.y = Y;
				}
				SumPosX += X;
				SumPosY += Y;
				BBpt1.x = MIN( BBpt1.x, X);
				BBpt1.y = MIN( BBpt1.y, Y);
				BBpt2.x = MAX( BBpt2.x, X);
				BBpt2.y = MAX( BBpt2.y, Y);

			}
			else
			{
				// blank the pixel out for debug display
				cvSet2D( pYCrCbImg, Y, X, BlankPixel );
			}
		}
	}

	if( 0 != nPixelsFound )
	{
		ptBlobCenter.x = SumPosX / nPixelsFound;
		ptBlobCenter.y = SumPosY / nPixelsFound;
	}

/*	
	BoundingBox.x = pt1.x;
	BoundingBox.y = pt1.y;
	BoundingBox.width = pt2.x - pt1.x;
	BoundingBox.height = pt2.y - pt1.y;
*/

	// Mask out of range values
//	cvInRangeS( pHSVImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
//	            cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );


	// Convert image back to BGR, and copy to display frame
    cvZero( pColorBlobDisplayFrame );
	cvCvtColor( pYCrCbImg, pColorBlobDisplayFrame, CV_YCrCb2BGR );
    //cvCvtPlaneToPix( 0, pMask, 0, 0, pColorBlobDisplayFrame );

	ROBOT_LOG( TRUE,"COLORBLOB Red=%d, Blue=%d, Both=%d\n", RedFound, BlueFound, nPixelsFound)
 
	return ptBlobCenter;
}

void CColorDetector::UpdateCrCbImages(const IplImage * pImg)
{
	// Convert to CrCb color model
	cvCvtColor( pImg, pYCrCbImg, CV_BGR2YCrCb );

/*
	// Mask out-of-range values
	cvInRangeS( pYCrCbImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
	            cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );
*/

	// Extract the Cr channel
//	cvSplit( pYCrCbImg, 0, 0, pCrImg, 0 );

	// Extract the Cb channel
//	cvSplit( pYCrCbImg, 0, 0, 0, pCbImg );
}

void CColorDetector::setCrThreshold(int Threshold)
{ 
	m_CrThreshold = Threshold; 

	m_CrMin = m_CrTarget - m_CrThreshold;
	if( m_CrMin < 0 ) m_CrMin = 0;

	m_CrMax = m_CrTarget + m_CrThreshold;
	if( m_CrMax > 255 ) m_CrMax = 255;
}

void CColorDetector::setCbThreshold(int Threshold)
{ 
	m_CbThreshold = Threshold; 

	m_CbMin = m_CbTarget - m_CbThreshold;
	if( m_CbMin < 0 ) m_CbMin = 0;

	m_CbMax = m_CbTarget + m_CbThreshold;
	if( m_CbMax > 255 ) m_CbMax = 255;

}


//////////////////////////////////////////////////////////////////////////////
// CLaserSpotDetector
//////////////////////////////////////////////////////////////////////////////

#define MIN_NEIGHBORS_THRESHOLD			2	// allowed pixels in middle of donut
#define MAX_NEIGHBORS_THRESHOLD			6	// Size of donut
#define PIXEL_LIST_SIZE					64	// number of pixels to track in the frame
#define PIXEL_BRIGHTNESS_THRESHOLD		550	// 255 * 3 = 765 (max brightness a pixel can be)
//#define NEIGHBOR_PIXEL_DISTANCE			3
//#define NEIGHBOR_PIXEL_DIM_MIN			50	// Amount surrounding pixels must be dimmer then target pixel 
	typedef struct
	{
		int  value;
		int  x;
		int  y;
	} BRIGHT_PIXEL_T;


CLaserSpotDetector::CLaserSpotDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Initialize local members
	m_nPositiveFrames = 0;	// Number of frames in a row where laser was detected
	m_LastSpotPosition.x = 0;
	m_LastSpotPosition.y = 0;


	// Allocate and Initialize resources
//	pYCrCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 3 );
//	pCrImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );
//	pCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );


	ROBOT_LOG( TRUE,"CLaserSpotDetector construction complete\n")
}

CLaserSpotDetector::~CLaserSpotDetector()
{
	// Release resources
//	cvReleaseImage( &pYCrCbImg );
//	cvReleaseImage( &pCrImg );
//	cvReleaseImage( &pCbImg );

	ROBOT_LOG( TRUE,"~CLaserSpotDetector done\n")
}

CvPoint CLaserSpotDetector::FindLaserDot()
{
	// find the brightest spot in the frame


	CvPoint ptLaserCenter = {0,0};

	CvScalar	Pixel;
	//CvScalar	NeighborPixel;
//	int 		nPixelsFound = 0;
	int			width = GetFrameSize().width - MAX_NEIGHBORS_THRESHOLD;
	int			height = GetFrameSize().height - MAX_NEIGHBORS_THRESHOLD;

 	// Create new CrCb images
//	UpdateCrCbImages(m_pVideoFrame);
	int  MaxBrightness = 0;
	int  PixelBrightness = 0;
	int  q, z, X, Y, row, col;
	int  MostNeighbors = 0;
	int  BestPixel = 0;
	int  PictureBrightnessSum = 0;
	int  BrightnessThreshold = 0;
	int  NeighborBrightnessMax = 0;
	int  NeighborBrightness = 0;
	BOOL LaserSpotFound = FALSE;

	int  nNeighbors[PIXEL_LIST_SIZE];
	BRIGHT_PIXEL_T BrightPixelArray[PIXEL_LIST_SIZE];

	if( (width < 1) || (height < 1) )
	{
		// frame not initialized correctly
		ROBOT_ASSERT(0);
	}

	memset( BrightPixelArray, 0x00, (sizeof(BRIGHT_PIXEL_T) * PIXEL_LIST_SIZE) );
	memset( nNeighbors, 0x00, (sizeof(int ) * PIXEL_LIST_SIZE) );

	// Find n brightest pixels in the frame that have less bright areas around
	// to avoid false triggering on white blobs
	// [0] is brightest pixel, [n-1] is least brightest of the saved values
	// to simplify, we don't deal with pixels within n pixels of boarder



	for( X = MAX_NEIGHBORS_THRESHOLD; X < width; X++ )
	{
		for( Y = MAX_NEIGHBORS_THRESHOLD; Y < height; Y++ )
		{
			Pixel =  cvGet2D( m_pVideoFrame, Y, X );
			//Cb = Pixel.val[0];
			//Cg = Pixel.val[1];
			//Cr = Pixel.val[2];
			// Add BGR to get brightness.
			PixelBrightness = (int )(Pixel.val[0] + Pixel.val[1] + Pixel.val[2]);
			PictureBrightnessSum+= PixelBrightness;
			
			for( q=0; q < PIXEL_LIST_SIZE; q++ )
			{
				if( PixelBrightness > BrightPixelArray[q].value )
				{
					// Brighter pixel found, save the pixel
					for( z=(PIXEL_LIST_SIZE-1); z > q; z-- )
					{
						// move other saved values over one
						BrightPixelArray[z].value = BrightPixelArray[z-1].value;
						BrightPixelArray[z].x = BrightPixelArray[z-1].x;
						BrightPixelArray[z].y = BrightPixelArray[z-1].y;
					}
					// Now save the new pixel
					BrightPixelArray[q].value = PixelBrightness;
					BrightPixelArray[q].x = X;
					BrightPixelArray[q].y = Y;
					break;
				}
			}
		}
	}

	// Do a coarse test to see if any pixels are bright enough
	int  PictureBrightnessAverage = PictureBrightnessSum / ( (height - MAX_NEIGHBORS_THRESHOLD) * (width - MAX_NEIGHBORS_THRESHOLD) );
	if(BrightPixelArray[0].value < (int )((double)PictureBrightnessAverage * 1.6) )
	{
		ROBOT_LOG( TRUE,"LASER - NO BRIGHT SPOT FOUND. ")
		ROBOT_LOG( TRUE,"Brightest spot = %d, Average = %d \n", BrightPixelArray[0].value, PictureBrightnessAverage )
//		return ptLaserCenter;
	}
	BOOL PixelFailed = FALSE;
	for( q=0; q < PIXEL_LIST_SIZE; q++ )
	{
		// traverse the list from brightest pixel to least bright
		// stop searching if the pixels are not bright enough
		if(BrightPixelArray[0].value < (int )((double)PictureBrightnessAverage * 1.6) )
		{
			ROBOT_LOG( TRUE,"Pixel %d too Dim ending search\n", q)
			break;	// pixel is too dim to be considered
		}

		// Test each Pixel in the BrightPixelArray to see if it's neighbors are also bright
		// Discard any that have bright neighbors
		PixelFailed = FALSE;
		int  BrightestNeighbor = 0;
		BrightnessThreshold = BrightPixelArray[q].value - 60;

		// we test in a "ring" around the pixel of interest
		for( row = -MAX_NEIGHBORS_THRESHOLD; row < MAX_NEIGHBORS_THRESHOLD; row++ )
		{
			for( col = -MAX_NEIGHBORS_THRESHOLD; col < MAX_NEIGHBORS_THRESHOLD; col++ )
			{
				if( (col >= -MIN_NEIGHBORS_THRESHOLD) && (col <= MIN_NEIGHBORS_THRESHOLD) &&
					(row >= -MIN_NEIGHBORS_THRESHOLD) && (row <= MIN_NEIGHBORS_THRESHOLD) )
				{
//					ROBOT_LOG( TRUE,"DONUT HOLE!\n")
					continue;	// cut a donut hole out of the test box
				}
				Pixel =  cvGet2D( m_pVideoFrame, (BrightPixelArray[q].y + col),  (BrightPixelArray[q].x + row)  );
//				ROBOT_LOG( TRUE,"DEBUG - doing Row=%d, Col=%d  Pixel:  x=%d, y=%d\n", row, col, (BrightPixelArray[q].x + row), (BrightPixelArray[q].y + col) )

				PixelBrightness = (int )(Pixel.val[0] + Pixel.val[1] + Pixel.val[2]);

				// DEBUG
				if( PixelBrightness > BrightestNeighbor )
				{
					BrightestNeighbor = PixelBrightness;
				}


				if( PixelBrightness >  BrightnessThreshold )
				{
					PixelFailed = TRUE;
//					ROBOT_LOG( TRUE,"Failed at (%d, %d)\n", row, col)
					break; // inner loop
				}
			}	
			if( PixelFailed) break;	// outer loop
		}
		if( !PixelFailed)
		{
			// we found what we think is a laser spot!
			LaserSpotFound = TRUE;
			ROBOT_LOG( TRUE,"LASER Bright Spot Level %d Detected at Index %d : (%d, %d) Brightest Neighbor Delta = %d, Frames = %d\n", 
				BrightPixelArray[q].value, q, BrightPixelArray[q].x, BrightPixelArray[q].y, (BrightPixelArray[q].value - BrightestNeighbor), m_nPositiveFrames )
			break;
		}

	}
	
	if( LaserSpotFound )
	{
		int  DistanceMovedX = abs((int)(m_LastSpotPosition.x - BrightPixelArray[BestPixel].x));
		int  DistanceMovedY = abs((int)(m_LastSpotPosition.y - BrightPixelArray[BestPixel].y));

		if( (DistanceMovedX > 40) || (DistanceMovedY > 40) )
		{
			// New spot is more then n pixels away from last one.  Probably noise, not a real laser spot
			ROBOT_LOG( TRUE,"Spot moved to fast (%d,%d pixels) - rejecting\n",DistanceMovedX, DistanceMovedY )
			m_nPositiveFrames = 0;	// reset the "good frames" count
		}
		if( (DistanceMovedX < 2) && (DistanceMovedY < 2) )
		{
			// New spot is less then n pixels away from last one.  Probably reflection, not a real laser spot
			ROBOT_LOG( TRUE,"Spot moved to slow (%d,%d pixels) - rejecting\n",DistanceMovedX, DistanceMovedY )
			m_nPositiveFrames = 0;	// reset the "good frames" count
		}

		if( m_nPositiveFrames++ > 1 )
		{
			ptLaserCenter.x = BrightPixelArray[BestPixel].x;
			ptLaserCenter.y = BrightPixelArray[BestPixel].y;
		}
		else
		{
			m_LastSpotPosition.x = BrightPixelArray[BestPixel].x;
			m_LastSpotPosition.y = BrightPixelArray[BestPixel].y;
		}
	}
	else
	{
		ROBOT_LOG( TRUE,"No Laser\n")
		m_nPositiveFrames = 0;
	}

	return ptLaserCenter;
}


/*

void CLaserSpotDetector::UpdateCrCbImages(const IplImage * pImg)
{
	// Convert to CrCb color model
	cvCvtColor( pImg, pYCrCbImg, CV_BGR2YCrCb );


	// Mask out-of-range values
	//	cvInRangeS( pYCrCbImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
	//	            cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );


	// Extract the Cr channel
//	cvSplit( pYCrCbImg, 0, 0, pCrImg, 0 );

	// Extract the Cb channel
//	cvSplit( pYCrCbImg, 0, 0, 0, pCbImg );
}

*/

//////////////////////////////////////////////////////////////////////////////
// CLaserLineDetector
//////////////////////////////////////////////////////////////////////////////

#define MIN_NEIGHBORS_THRESHOLD			2	// allowed pixels in middle of donut
#define MAX_NEIGHBORS_THRESHOLD			6	// Size of donut
#define PIXEL_LIST_SIZE					64	// number of pixels to track in the frame
#define PIXEL_BRIGHTNESS_THRESHOLD		550	// 255 * 3 = 765 (max brightness a pixel can be)
//#define NEIGHBOR_PIXEL_DISTANCE			3
//#define NEIGHBOR_PIXEL_DIM_MIN			50	// Amount surrounding pixels must be dimmer then target pixel 


CLaserLineDetector::CLaserLineDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Initialize local members
	m_width = GetFrameSize().width;
	m_height = GetFrameSize().height;
	if( (m_width < 1) || (m_height < 1) )
	{
		// frame not initialized correctly
		ROBOT_ASSERT(0);
	}
	m_pLaserDotArray = new LINE_PIXEL_T[m_width+1];
	//m_pLaserDotArray = (LINE_PIXEL_T*)(malloc( (sizeof(LINE_PIXEL_T)*m_width)+1 ));
	memset( m_pLaserDotArray, 0, (sizeof(LINE_PIXEL_T)*m_width) );

	ROBOT_LOG( TRUE,"CLaserLineDetector construction complete\n")
}

CLaserLineDetector::~CLaserLineDetector()
{
	// Release resources
//	cvReleaseImage( &pYCrCbImg );
//	cvReleaseImage( &pCrImg );
//	cvReleaseImage( &pCbImg );
	SAFE_DELETE(m_pLaserDotArray);

	ROBOT_LOG( TRUE,"~CLaserLineDetector done\n")
}

BOOL CLaserLineDetector::FindLaserLines()
{
	// find laser dots for each vertical column in the frame 

	LINE_PIXEL_T BrightestPixel;

//	CvPoint ptLaserCenter = {0,0};
	CvScalar	Pixel;
	//CvScalar	NeighborPixel;
//	int 		nPixelsFound = 0;
	memset( m_pLaserDotArray, 0, (sizeof(LINE_PIXEL_T)*m_width) );

 	// Create new CrCb images
//	UpdateCrCbImages(m_pVideoFrame);
	int  MaxBrightness = 0;
	int  PixelBrightness = 0;
	int  X, Y;
	int  BestPixel = 0;
	int  PictureBrightnessSum = 0;
	int  BrightnessThreshold = 0;
	BOOL LaserLineFound = FALSE;


	// Find the brightest red pixel in each column of the frame.
	for( X = 0; X < m_width; X++ )
	{
		BrightestPixel.value = 0;
		BrightestPixel.position = 0;
		for( Y = 0; Y < m_height; Y++ )
		{
			Pixel =  cvGet2D( m_pVideoFrame, Y, X );
			//Cb = Pixel.val[0];
			//Cg = Pixel.val[1];
			//Cr = Pixel.val[2];
			// Add BGR to get brightness.
			PixelBrightness = (int )(Pixel.val[0] + Pixel.val[1] + Pixel.val[2]); // TODO - RED ONLY?
			PictureBrightnessSum+= PixelBrightness;
			if( PixelBrightness > BrightestPixel.value )
			{
				// New brightest pixel found for this column
				BrightestPixel.value = PixelBrightness;
				BrightestPixel.position = Y;
			}
		}
		// Done with column.  Record position of the brightest pixel in this column.
		m_pLaserDotArray[X].value = BrightestPixel.value;
		m_pLaserDotArray[X].position = BrightestPixel.position;
	}


	// Perform thresholding and test to see if any pixels are bright enough
	BrightestPixel.value = 0;
	BrightestPixel.position = 0;
	int  PictureBrightnessAverage = PictureBrightnessSum / ( (m_height) * (m_width) );
	int nLaserPointsFound = 0;
	for( X = 0; X < m_width; X++ )
	{
		if( m_pLaserDotArray[X].value > (int )((double)PictureBrightnessAverage * 1.6) )
		{
			nLaserPointsFound++;
			if( m_pLaserDotArray[X].value > BrightestPixel.value )
			{
				BrightestPixel.value =  m_pLaserDotArray[X].value;
				BrightestPixel.position =  m_pLaserDotArray[X].position;
			}
		}
		else
		{
			m_pLaserDotArray[X].value = 0;	// clamp to zero
			m_pLaserDotArray[X].position = 0;	// clamp to zero
		}
	}

	if( nLaserPointsFound < 5 )
	{
		ROBOT_LOG( TRUE,"LASER LINE - NO BRIGHT LINES FOUND. ")
		ROBOT_LOG( TRUE,"Brightest line = %d, Average = %d \n", BrightestPixel.value, PictureBrightnessAverage )
	}
	else
	{
		// we found what we think is some laser spots!
		LaserLineFound = TRUE;
		ROBOT_LOG( TRUE,"LASER LINE - Bright Pixels found!\n")
	}
	
	return LaserLineFound;
}


int  CLaserLineDetector::GetPositionY(int X)
{
	// For a given X position in the frame, return Y position of laser spot
	int  Y = m_pLaserDotArray[X].position;

	return Y;
}


//////////////////////////////////////////////////////////////////////////////
// CTrackObjectDetector
// Finds unique spots in image and tracks from frame to frame
// Good for keeping head tracking an object
//////////////////////////////////////////////////////////////////////////////
#define		MAX_COUNT 500
#define		ROI_SIZE	30

CTrackObjectDetector::CTrackObjectDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	ROBOT_ASSERT( m_pVideoFrame );

	m_width = GetFrameSize().width;
	m_height = GetFrameSize().height;
	if( (m_width < 1) || (m_height < 1) ) ROBOT_ASSERT(0);	// frame not initialized correctly

	m_GrayImg = 0;
	m_PreviousGrayImg = 0;
	m_Pyramid = 0;
	m_PreviousPyramid = 0;
	m_WindowSize = 10;
	m_Status = 0;
	m_FeatureCount = 0;
	m_Flags = 0;
	add_remove_pt = 0;
	m_ReferenceCloudCenter.x = 0.0;
	m_ReferenceCloudCenter.y = 0.0;

	ROBOT_LOG( TRUE,"CTrackObjectDetector construction complete\n")
}

CTrackObjectDetector::~CTrackObjectDetector()
{
	// Release resources
	cvReleaseImage( &m_PreviousGrayImg );
	cvReleaseImage( &m_Pyramid );
	cvReleaseImage( &m_PreviousPyramid );

	ROBOT_LOG( TRUE,"~CTrackObjectDetector done\n")
}

void CTrackObjectDetector::Init()
{
    m_GrayImg = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 1 );
    m_PreviousGrayImg = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 1 );
    m_Pyramid = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 1 );
    m_PreviousPyramid = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 1 );
    m_Points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(m_Points[0][0]));
    m_Points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(m_Points[0][0]));
    m_Status = (char*)cvAlloc(MAX_COUNT);
    m_Flags = 0;
}

BOOL CTrackObjectDetector::LockOn( BOOL FindWallSocket )
{
	if( FindWallSocket )
	{
		ROBOT_LOG( TRUE,"CTrackObjectDetector: Will look for WallSocket\n")
		// Get the bitmap Image of the object to find
		CString strImage = "WallSocket.jpg";
		CString strFileName = DEFAULT_KNOWN_OBJECTS_DIR + strImage;
		const char *object_filename = strFileName;

		m_GrayImg = cvLoadImage( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
		if( !m_GrayImg )
		{
			ROBOT_LOG( TRUE, "CMatchObjectDetector ERROR! Can not load image %s\n", object_filename )
			ROBOT_ASSERT(0);
			return 0;
		}
	}
	else
	{
		cvCvtColor( m_pVideoFrame, m_GrayImg, CV_BGR2GRAY );
	}

	// Get unique points to track
	IplImage* eig = cvCreateImage( cvGetSize(m_GrayImg), 32, 1 );
	IplImage* temp = cvCreateImage( cvGetSize(m_GrayImg), 32, 1 );
	IplImage* mask = cvCreateImage( cvGetSize(m_GrayImg), 8, 1 );
	double quality = 0.01;
	double min_distance = 5; // default 10

	// Mask is used to only find points near the center of the camera image
	cvZero( mask );
	CvScalar white = cvRealScalar(255);
	CvScalar black = cvRealScalar(0);
	m_CenterPt.x = m_width / 2;
	m_CenterPt.y = m_height / 2;
	m_BoxPt1.x = m_CenterPt.x - ROI_SIZE;
	m_BoxPt1.y = m_CenterPt.y - ROI_SIZE;
	m_BoxPt2.x = m_CenterPt.x + ROI_SIZE;
	m_BoxPt2.y = m_CenterPt.y + ROI_SIZE;
	CvRect ROIRect = cvRect( m_BoxPt1.x, m_BoxPt1.y, (ROI_SIZE*2), (ROI_SIZE*2) );  // x,y,w,h
	cvRectangle(mask, m_BoxPt1, m_BoxPt2, white, CV_FILLED, 4, 0); // draw white box in the middle of the mask


	m_FeatureCount = MAX_COUNT;
	cvGoodFeaturesToTrack(  
		m_GrayImg,		// image – The source 8-bit or floating-point 32-bit, single-channel image
		eig,			// eigImage – Temporary floating-point 32-bit image, the same size as image
		temp,			// tempImage – Another temporary image, the same size and format as eigImage
		m_Points[1],	// corners – Output parameter; detected corners
		&m_FeatureCount,// cornerCount – Output parameter; number of detected corners
		quality,		// qualityLevel – Multiplier for the max/min eigenvalue; specifies the minimal accepted quality of image corners
		min_distance,	// minDistance – Limit, specifying the minimum possible distance between the returned corners; Euclidian distance is used
		mask,			// mask – Region of interest. The function selects points either in the specified region or in the whole image if the mask is NULL
		3,				// blockSize – Size of the averaging block, passed to the underlying CornerMinEigenVal or CornerHarris used by the function
		0,				// useHarris – If nonzero, Harris operator (CornerHarris) is used instead of default CornerMinEigenVal
		0.04 );			// k – Free parameter of Harris detector; used only if (useHarris != 0)

//	cvGoodFeaturesToTrack( m_GrayImg, eig, temp, m_Points[1], &m_FeatureCount, quality, min_distance, mask, 3, 0, 0.04 );

	cvFindCornerSubPix( m_GrayImg, m_Points[1], m_FeatureCount,	cvSize(m_WindowSize,m_WindowSize), cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));

	cvReleaseImage( &eig );
	cvReleaseImage( &temp );
	add_remove_pt = 0;

	CV_SWAP( m_PreviousGrayImg, m_GrayImg, m_SwapTemp );
	CV_SWAP( m_PreviousPyramid, m_Pyramid, m_SwapTemp );
	CV_SWAP( m_Points[0], m_Points[1], m_SwapPoints );

	// sumarize points found for comparison later
	if( m_FeatureCount > 0 )
	{
		m_ReferenceCloudCenter.x = 0.0;
		m_ReferenceCloudCenter.y = 0.0;
		CvPoint2D32f FrameCenter = GetFrameCenterFloat();

		for( int i=0; i< m_FeatureCount; i++ )
		{
			CvPoint ToPt = cvPoint( cvRound(m_Points[0][i].x), cvRound(m_Points[0][i].y) );
			// Get the average center of the cloud of points
			m_ReferenceCloudCenter.x += FrameCenter.x - m_Points[0][i].x;
			m_ReferenceCloudCenter.y += FrameCenter.y - m_Points[0][i].y;
		}
		m_ReferenceCloudCenter.x = -m_ReferenceCloudCenter.x / (float)m_FeatureCount;
		m_ReferenceCloudCenter.y = -m_ReferenceCloudCenter.y / (float)m_FeatureCount;

		ROBOT_LOG( TRUE,"/n/nLOCK ON: Center: x=%4.1f, y=%4.1f\n", m_ReferenceCloudCenter.x, m_ReferenceCloudCenter.y)
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


BOOL CTrackObjectDetector::GetDelta( CvPoint &ObjectDelta )
{
	int i, k;

	cvCvtColor( m_pVideoFrame, m_GrayImg, CV_BGR2GRAY );
	//char FeaturesFound [MAX_COUNT];
	float FeatureErrors[MAX_COUNT];

	if( m_FeatureCount > 0 )
	{
		cvCalcOpticalFlowPyrLK( m_PreviousGrayImg, m_GrayImg, m_PreviousPyramid, m_Pyramid,
			m_Points[0], m_Points[1], m_FeatureCount, cvSize(m_WindowSize,m_WindowSize), 3, m_Status, FeatureErrors,
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), m_Flags );
		m_Flags |= CV_LKFLOW_PYR_A_READY;
		for( i = k = 0; i < m_FeatureCount; i++ )
		{
			if( !m_Status[i] )
				continue;

			m_Points[1][k++] = m_Points[1][i];
			cvCircle( m_pVideoFrame, cvPointFrom32f(m_Points[1][i]), 2, CV_RGB(0,255,0), -1, 8,0);
		}
		m_FeatureCount = k;
	}

	CV_SWAP( m_PreviousGrayImg, m_GrayImg, m_SwapTemp );
	CV_SWAP( m_PreviousPyramid, m_Pyramid, m_SwapTemp );
	CV_SWAP( m_Points[0], m_Points[1], m_SwapPoints );

	// Draw the ROI
	cvRectangle(m_pVideoFrame, m_BoxPt1, m_BoxPt2, CV_RGB(250,0,255), 1 );

	// Calculate the average cloud movement from center
	CvPoint2D32f FrameCenter = GetFrameCenterFloat();
	int GoodFeatureCount = 0;
	CvPoint2D32f CloudCenter = {0.0, 0.0};

	for( i=0; i< m_FeatureCount; i++ )
	{
		if( (0 != m_Status[i]) || (FeatureErrors[i] <= 550) )
		{
			// Get the average center of the new cloud of points
			CloudCenter.x += FrameCenter.x - m_Points[0][i].x;
			CloudCenter.y += FrameCenter.y - m_Points[0][i].y;

			// Display the delta change per frame
			CvPoint FromPt = cvPoint( cvRound(m_Points[0][i].x), cvRound(m_Points[0][i].y) );
			CvPoint ToPt = cvPoint( cvRound(m_Points[0][i].x), cvRound(m_Points[0][i].y) );
			cvLine( m_pVideoFrame, FromPt, ToPt, CV_RGB(250,0,255), 1 );

			GoodFeatureCount++;
		}
	}
	CloudCenter.x = -CloudCenter.x / (float)GoodFeatureCount;
	CloudCenter.y = -CloudCenter.y / (float)GoodFeatureCount;

	// Calculate the delta
	ObjectDelta.x = cvRound(CloudCenter.x - m_ReferenceCloudCenter.x);
	ObjectDelta.y = cvRound(CloudCenter.y - m_ReferenceCloudCenter.y);

	if( (ObjectDelta.x > 1) || (ObjectDelta.y > 1) )
	{
		ROBOT_LOG( TRUE,"GET DELTA: x=%d, y=%d,  FeatureCount = %d\n", ObjectDelta.x, ObjectDelta.y, GoodFeatureCount)
	}

	if( GoodFeatureCount < 5 )
	{
		ROBOT_LOG( TRUE,"LOST TRACK: GoodFeatureCount = %d\n", GoodFeatureCount)
		return FALSE; // Lost features to track
	}

	return TRUE;
}




//////////////////////////////////////////////////////////////////////////////
// CMatchObjectDetector
// Looks for known objects using SURF detector
//////////////////////////////////////////////////////////////////////////////

#define SURF_POINT_MATCH_THRESHOLD 12	// Number of matches required to match

static CvScalar colors[] = 
{
    {{0,0,255}},
    {{0,128,255}},
    {{0,255,255}},
    {{0,255,0}},
    {{255,128,0}},
    {{255,255,0}},
    {{255,0,0}},
    {{255,0,255}},
    {{255,255,255}}
};


CMatchObjectDetector::CMatchObjectDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{

	ROBOT_ASSERT( m_pVideoFrame );

	// Initialize local members
	m_bInitialized = FALSE;


	// Allocate and Initialize resources
//	pYCrCbImg  = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 3 );


	ROBOT_LOG( TRUE,"CMatchObjectDetector construction complete\n")
}

CMatchObjectDetector::~CMatchObjectDetector()
{
	// Release resources
///	DeleteAllSURFInfoItems();
//	cvReleaseImage( &pCbImg );

	ROBOT_LOG( TRUE,"~CMatchObjectDetector done\n")
}

////////////////////////////////////////////
BOOL CMatchObjectDetector::Initialize()
{
	// Walk through the list of known objects, and for each object image, extract SURF parameters.
	// Process each image to extract features
	// Load up data structure with object info.
	// For each Known Object, load it's image and create SURF keypoints


//	CvSURFParams params = cvSURFParams(500, 1);
//	ObjectInfo* pObjectInfo;
	
	// Load up known objects from file
	g_pObjectKnowledge = new ObjectKnowledge();
	g_pObjectKnowledge->Init();	
	ObjectInfoList* pObjectInfoList = g_pObjectKnowledge->GetObjectInfoList();

///	m_pSURFInfoList = new SURFInfoList();
	int nIndex = 0;	// Use position in the file as the ID!

	// Read list of Object images from ObjectKnowledge

	POSITION ObjectInfoListPos = pObjectInfoList->GetHeadPosition();
///	POSITION SURFInfoListPos = m_pSURFInfoList->GetHeadPosition();

	while (ObjectInfoListPos != NULL)
	{
		// Get SURF info for each Object
		ObjectInfo *pObjectInfo = pObjectInfoList->GetNext(ObjectInfoListPos);
///		ROBOT_ASSERT( nIndex = pObjectInfo->nIndex );
		CString strFileName;
		strFileName = DEFAULT_KNOWN_OBJECTS_DIR + pObjectInfo->strImageFileName;
		const char *object_filename = strFileName;

		// Get the bitmap Image of the object
		IplImage* ObjectImage = cvLoadImage( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
		if( !ObjectImage )
		{
			ROBOT_LOG( TRUE, "CMatchObjectDetector ERROR! Can not load image %s\n", object_filename )
			ROBOT_ASSERT(0);
			continue;
		}

		// Extract the SURF information
///		SURFInfo *pSURFInfo = new SURFInfo();
	    CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq *objectKeypoints = 0, *objectDescriptors = 0;
		CvSeq *imageKeypoints = 0, *imageDescriptors = 0;

/***
int extended = 500;
		int hessianThreshold = 1; 
		int nOctaves = 2; 
		int nOctaveLayers = 4;

		while(1)
		{
***/
		cvExtractSURF( ObjectImage, 0, &objectKeypoints, &objectDescriptors, storage, 
			500, 1, 2, 4 ); // extended, hessianThreshold, nOctaves, nOctaveLayers
		ROBOT_LOG( TRUE,"CMatchObjectDetector:DEBUG: Loaded Object %d: %s  file: %s Number of Object Descriptors = %d\n",
			pObjectInfo->nIndex, pObjectInfo->strObjectName, object_filename, objectDescriptors->total)
//		}

//		cvExtractSURF( ObjectImage, 0, &(pObjectInfo->Keypoints), &(pObjectInfo->Descriptors), pObjectInfo->Storage, params );
//		ROBOT_LOG( TRUE,"CMatchObjectDetector:DEBUG: Object ID %d: Number of Object Descriptors = %d\n", 
//			nID, pObjectInfo->Descriptors->total)

		// Store the SURF info into the List
		pObjectInfo->Descriptors = objectDescriptors;
		pObjectInfo->Keypoints = objectKeypoints;
	    pObjectInfo->Storage = storage;
		nIndex++;
	}


	// OK, all known object info should be loaded at this point!

	m_bInitialized = TRUE;
	ROBOT_LOG( TRUE,"CMatchObjectDetector::Initialize Done!\n")


	return TRUE;
}


////////////////////////////////////////////////////////////
POSITION CMatchObjectDetector::LookForMatchingObject()
{
	// See if any of our known images are found in the frame
	// If so, return the position of the image that has the strongest match
	
	int nBestMatchPairs = NO_MATCH;
	POSITION BestMatchPos = NULL;
//	ObjectInfo *pBestObjectInfo = NULL;
	CvSeq *imageKeypoints;		// for current Video Frame
	CvSeq *imageDescriptors;
	DWORD dwStartTime;
	CString MsgString;


//	CvSURFParams params = cvSURFParams(500, 1);
	IplImage* GrayImage = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);
	
	// Copy video frame to gray scale working image
	GrayImage  = cvCreateImage( cvGetSize(m_pVideoFrame), 8, 1 );
	GrayImage->origin = 0;	
	cvCvtColor( m_pVideoFrame, GrayImage, CV_BGR2GRAY ); // copy and convert frame to grayscale
	
	
	// Extract SURF Image Descriptors
	dwStartTime = GetTickCount();

	
//	cvExtractSURF( GrayImage, 0, &imageKeypoints, &imageDescriptors, storage, 500, 1, 2, 4 );
//	cvExtractSURF( GrayImage, 0, &imageKeypoints, &imageDescriptors, storage, params ); //500, 1, 2, 4 );
	cvExtractSURF( GrayImage, 0, &imageKeypoints, &imageDescriptors, storage, 1, 500, 3, 4 ); //500, 1, 2, 4 );
	
	ROBOT_LOG( TRUE, "---------------------------" )
	MsgString.Format( "\nCMatchObjectDetector DEBUG: STARTING NEW FRAME\n" );
	ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )
	MsgString.Format( "DEBUG: Frame Descriptors: %d, Extract time = %ld ms", imageDescriptors->total, (GetTickCount() - dwStartTime) );
	ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )
	
	/*
	CvPoint src_corners[4] = {{0,0}, {object->width,0}, {object->width, object->height}, {0, object->height}};
	CvPoint dst_corners[4];
	
	  IplImage* correspond = cvCreateImage( cvSize(GrayImage->width, object->height+GrayImage->height), 8, 1 );
	  cvSetImageROI( correspond, cvRect( 0, 0, object->width, object->height ) );
	  cvCopy( object, correspond );
	  cvSetImageROI( correspond, cvRect( 0, object->height, correspond->width, correspond->height ) );
	  cvCopy( image, correspond );
	  cvResetImageROI( correspond );
	*/
	/*
	tt = (double)cvGetTickCount();
	if( locatePlanarObject( m_objectKeypoints, m_objectDescriptors, imageKeypoints,
	imageDescriptors, src_corners, dst_corners ))
	{
	for( i = 0; i < 4; i++ )
	{
				CvPoint r1 = dst_corners[i%4];
				CvPoint r2 = dst_corners[(i+1)%4];
				cvLine( correspond, cvPoint(r1.x, r1.y+object->height ),
				cvPoint(r2.x, r2.y+object->height ), colors[8] );
				}
				}
				tt = (double)cvGetTickCount() - tt;
				ROBOT_LOG( TRUE, "\nLocate Planar time = %gms\n", tt/(cvGetTickFrequency()*1000.))
	*/
	

	
	
	
	//////////////////////////////////////////////////////////////////////
	// Find number of Matches for each Known Object
///SURFInfo* pSURFInfo;
///	POSITION SURFInfoListPos = m_pSURFInfoList->GetHeadPosition();
	ObjectInfoList* pObjectInfoList = g_pObjectKnowledge->GetObjectInfoList();
	POSITION ObjectInfoListPos = pObjectInfoList->GetHeadPosition();

	DWORD dwFullStartTime = GetTickCount();

	int nIndex = 0;
	int i = 0;
	while (ObjectInfoListPos != NULL)
	{
		// Compare video frame SURF info with info from each Known Object in the List
		POSITION CurrentPos = ObjectInfoListPos;
		ObjectInfo *pObjectInfo = pObjectInfoList->GetNext(ObjectInfoListPos);
//		nID = pSURFInfo->nID;
//		pSURFInfo->Descriptors;

		dwStartTime = GetTickCount();
		
		vector<int> ptpairs;
		findPairs( pObjectInfo->Keypoints, pObjectInfo->Descriptors, imageKeypoints, imageDescriptors, ptpairs );

//		MsgString.Format( "Index %d: nPairs = %d, Time = %ld ms", nIndex, (int)ptpairs.size(), (GetTickCount() - dwStartTime) );
//		ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )



//		tt = (double)cvGetTickCount();

		if( (int)ptpairs.size() > SURF_POINT_MATCH_THRESHOLD )
		{
			MsgString.Format( "DEBUG Object Match: [%s]  Pairs = %d, Time = %ld ms", 
				pObjectInfo->strObjectName, (int)ptpairs.size(), (GetTickCount() - dwStartTime) );
			ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )

			if( (int)ptpairs.size() > nBestMatchPairs )
			{
				nBestMatchPairs = (int)ptpairs.size();
				BestMatchPos = CurrentPos;
			}
		}

//	/*
		for( i = 0; i < (int)ptpairs.size(); i += 2 )
		{
			CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
			CvPoint center;
			int radius;
			center.x = cvRound(r->pt.x);
			center.y = cvRound(r->pt.y);
			radius = cvRound(r->size*1.2/9.*2);
			cvCircle( m_pVideoFrame, center, radius, colors[0], 2, 8, 0 );
		}
//		tt = (double)cvGetTickCount() - tt;
//		ROBOT_LOG( TRUE, "\nFind pairs draw time = %gms\n\n", tt/(cvGetTickFrequency()*1000.))
// */
/*
		for( int i = 0; i < (int)ptpairs.size(); i += 2 )
		{
//		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
		cvLine( m_pVideoFrame, cvPointFrom32f(r1->pt),
					cvPoint(cvRound(r2->pt.x), cvRound(r2->pt.y+object->height)), colors[8] );
		}
//		tt = (double)cvGetTickCount() - tt;
	//	ROBOT_LOG( TRUE, "\nFind pairs time = %gms\n\n", tt/(cvGetTickFrequency()*1000.))
*/
		
		//cvShowImage( "Object Correspond", correspond );

/*
		// Mark Key points on the video frame
		for( i = 0; i < imageKeypoints->total; i++ )
		{
			CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
			CvPoint center;
			int radius;
			center.x = cvRound(r->pt.x);
			center.y = cvRound(r->pt.y);
			radius = cvRound(r->size*1.2/9.*2);
			cvCircle( m_pVideoFrame, center, radius, colors[0], 1, 8, 0 );
		}
		//cvShowImage( "Object", object_color );
		//cvWaitKey(10);
*/
		nIndex++;

	}
	
	// cvDestroyWindow("Object");
	// cvDestroyWindow("Object SURF");
	// cvDestroyWindow("Object Correspond");
	
	ROBOT_LOG( TRUE, "---------------------------" )
	if( NULL != BestMatchPos )
	{
		ObjectInfo *pBestObjectInfo = pObjectInfoList->GetAt(BestMatchPos);
		CString MsgString;
		MsgString.Format( "OBJECT MATCH! [%s]  Pairs = %d Time = %ld" , 
			pBestObjectInfo->strObjectName, nBestMatchPairs, (GetTickCount() - dwFullStartTime) );
		ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )
		ROBOT_LOG( TRUE, "---------------------------" )

	}
	else
	{
//		ROBOT_LOG( TRUE, "CMatchObjectDetector: No Object Match found\n" )
		MsgString.Format( "No Object Match.  Time = %ld", (GetTickCount() - dwFullStartTime) );
		ROBOT_LOG( TRUE,  (LPCTSTR)MsgString )
		ROBOT_LOG( TRUE, "---------------------------" )
	}
	
	return BestMatchPos;	// return the index of the object
	
		
}

// Utilities
/***
void CMatchObjectDetector::DeleteAllSURFInfoItems()
{
	// Delete all of the objects pointed to
	// by the CTypedPtrList. Then remove all of the
	// pointers from the CTypedPtrList, which
	// is faster than removing each individually.
	POSITION pos = m_pSURFInfoList->GetHeadPosition();
	while (pos != NULL)
	{
		delete m_pSURFInfoList->GetNext(pos);
	}

	m_pSURFInfoList->RemoveAll();
	delete m_pSURFInfoList;

}
***/

double CMatchObjectDetector::compareSURFDescriptors( const double* d1, const double* d2, double best, int length )
{
	double total_cost = 0;
	assert( length % 4 == 0 );
	for( int i = 0; i < length; i += 4 )
	{
		double t0 = d1[i] - d2[i];
		double t1 = d1[i+1] - d2[i+1];
		double t2 = d1[i+2] - d2[i+2];
		double t3 = d1[i+3] - d2[i+3];
		total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
		if( total_cost > best )
			break;
	}
	return total_cost;
}

int CMatchObjectDetector::naiveNearestNeighbor( const double* vec, int laplacian,
					  const CvSeq* model_keypoints,
					  const CvSeq* model_descriptors )
{
	int length = (int)(model_descriptors->elem_size/sizeof(double));
	int i, neighbor = -1;
	double d, dist1 = 1e6, dist2 = 1e6;
	CvSeqReader reader, kreader;
	cvStartReadSeq( model_keypoints, &kreader, 0 );
	cvStartReadSeq( model_descriptors, &reader, 0 );

	for( i = 0; i < model_descriptors->total; i++ )
	{
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
		const double* mvec = (const double*)reader.ptr;
		CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
		if( laplacian != kp->laplacian )
			continue;
		d = compareSURFDescriptors( vec, mvec, dist2, length );
		if( d < dist1 )
		{
			dist2 = dist1;
			dist1 = d;
			neighbor = i;
		}
		else if ( d < dist2 )
			dist2 = d;
	}
	if ( dist1 < 0.6*dist2 )
		return neighbor;
	return -1;
}

void CMatchObjectDetector::findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
		   const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	int i;
	CvSeqReader reader, kreader;
	cvStartReadSeq( objectKeypoints, &kreader );
	cvStartReadSeq( objectDescriptors, &reader );
	ptpairs.clear();

	for( i = 0; i < objectDescriptors->total; i++ )
	{
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
		const double* descriptor = (const double*)reader.ptr;
		CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
		int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
		if( nearest_neighbor >= 0 )
		{
			ptpairs.push_back(i);
			ptpairs.push_back(nearest_neighbor);
		}
	}
}

/* a rough implementation for object location */
int CMatchObjectDetector::locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
					const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
					const CvPoint src_corners[4], CvPoint dst_corners[4] )
{
	double h[9];
	CvMat _h = cvMat(3, 3, CV_64F, h);
	vector<int> ptpairs;
	vector<CvPoint2D32f> pt1, pt2;
	CvMat _pt1, _pt2;
	int i, n;

	findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
	n = ptpairs.size()/2;
	if( n < 4 )
		return 0;

	pt1.resize(n);
	pt2.resize(n);
	for( i = 0; i < n; i++ )
	{
		pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
		pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
	}

	_pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
	_pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
	if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
		return 0;

	for( i = 0; i < 4; i++ )
	{
		double x = src_corners[i].x, y = src_corners[i].y;
		double Z = 1./(h[6]*x + h[7]*y + h[8]);
		double X = (h[0]*x + h[1]*y + h[2])*Z;
		double Y = (h[3]*x + h[4]*y + h[5])*Z;
		dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
	}

	return 1;
}




//////////////////////////////////////////////////////////////////////////////
// HiGUI Tracker Bar Callback functions


// pre-filtering (normalization of input images)
void OnChange_preFilterSize(int value )
{
	// BMState->preFilterSize=41;		// averaging window size: ~5x5..21x21
	BMState->preFilterSize = value;
	if( 0 == (BMState->preFilterSize %2) ) BMState->preFilterSize += 1;	// must be an odd number
	if( BMState->preFilterSize < 5 ) BMState->preFilterSize = 5;
	if( BMState->preFilterSize > 255 ) BMState->preFilterSize = 175; // 75 weird, doc says max is 21, but default is 41
}
void OnChange_preFilterCap(int value )
{
	// BMState->preFilterCap=31;		// the output of pre-filtering is clipped by [-preFilterCap,preFilterCap]
	BMState->preFilterCap = value;
	if( BMState->preFilterCap < 1 ) BMState->preFilterCap = 1;
	if( BMState->preFilterCap > 163 ) BMState->preFilterCap = 163;	// 63	
}


// correspondence using Sum of Absolute Difference (SAD)
void OnChange_SADWindowSize(int value )
{
	// BMState->SADWindowSize=41;		// ~5x5..21x21
	BMState->SADWindowSize = value;
	if( 0 == (BMState->SADWindowSize %2) ) BMState->SADWindowSize += 1;	// must be an odd number
	if( BMState->SADWindowSize < 5 ) BMState->SADWindowSize = 5;
	if( BMState->SADWindowSize > 155 ) BMState->SADWindowSize = 155; //55
}
void OnChange_minDisparity(int value )
{
	// BMState->minDisparity=-64;		// minimum disparity (can be negative)
	BMState->minDisparity= value *32 - 64;
	if( BMState->minDisparity < -64 ) BMState->minDisparity = -64;
	if( BMState->minDisparity > 64 ) BMState->minDisparity = 64;
}
void OnChange_numberOfDisparities(int value )
{
	// BMState->numberOfDisparities=128;// maximum disparity - minimum disparity (> 0)
	BMState->numberOfDisparities = value *64 + 64;
	if( BMState->numberOfDisparities < 64 ) BMState->numberOfDisparities = 64;
	if( BMState->numberOfDisparities > 256 ) BMState->numberOfDisparities = 256;
}
void OnChange_textureThreshold(int value )
{
	// BMState->textureThreshold=10;	// the disparity is only computed for pixels with textured enough neighborhood
	BMState->textureThreshold = value;
	if( BMState->textureThreshold < 3 ) BMState->textureThreshold = 3;
	if( BMState->textureThreshold > 15 ) BMState->textureThreshold = 15;
}
void OnChange_uniquenessRatio(int value )
{
	// BMState->uniquenessRatio=15;		
	// accept the computed disparity d* only if streamSAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
	// for any d != d*+/-1 within the search range.
	BMState->uniquenessRatio = value;
	if( BMState->uniquenessRatio < 5 ) BMState->uniquenessRatio = 5;
	if( BMState->uniquenessRatio > 125 ) BMState->uniquenessRatio = 125; //25
}


void OnChange_speckleWindowSize(int value )
{
	// BMState->speckleWindowSize=9;			// 5x5..21x21 Edge of objects disparity variation window
	BMState->speckleWindowSize = value;
	if( 0 == (BMState->speckleWindowSize %2) ) BMState->speckleWindowSize += 1;	// must be an odd number
	if( BMState->speckleWindowSize < 5 ) BMState->speckleWindowSize = 5;
	if( BMState->speckleWindowSize > 55 ) BMState->speckleWindowSize = 55;
}
void OnChange_speckleRange(int value )
{
	// BMState->speckleRange=4;				// acceptable range of variation in window 
	BMState->speckleRange = value;
	if( BMState->speckleRange < 1 ) BMState->speckleRange = 1;
	if( BMState->speckleRange > 112 ) BMState->speckleRange = 112; //12
}


//////////////////////////////////////////////////////////////////////////////
// CStereoDetector
//////////////////////////////////////////////////////////////////////////////

CStereoDetector::CStereoDetector(IplImage* pVideoFrame) :
	CCameraDetector(pVideoFrame)	// Call base class constructor
{
	// Initialize local members
//	m_nPositiveFrames = 0;	// Number of frames in a row where laser was detected
//	m_LastSpotPosition.x = 0;
//	m_LastSpotPosition.y = 0;


	// Allocate and Initialize resources
//	pYCrCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 3 );
//	pCrImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );
//	pCbImg  = cvCreateImage( cvGetSize(pVideoFrame), 8, 1 );

	BMState = cvCreateStereoBMState();
    imageSize.height = 0;
    imageSize.width = 0;


	ROBOT_LOG( TRUE,"CStereoDetector construction complete\n")
}

CStereoDetector::~CStereoDetector()
{
	// Release resources
//	cvReleaseImage( &pYCrCbImg );
//	cvReleaseImage( &pCrImg );
//	cvReleaseImage( &pCbImg );

/* TODO

	cvReleaseStereoBMState(&BMState);
	cvReleaseMat( &mx1 );
	cvReleaseMat( &my1 );
	cvReleaseMat( &mx2 );
	cvReleaseMat( &my2 );
	cvReleaseMat( &img1r );
	cvReleaseMat( &img2r );
	cvReleaseMat( &disp );

*/
	cvDestroyWindow( "rectified" );
	cvDestroyWindow( "disparity" );


	ROBOT_LOG( TRUE,"~CStereoDetector done\n")
}

//  StereoDetector->Calibrate("calib_list.txt", 9, 6, 1);
void CStereoDetector::Calibrate(const char* imageList, int nx, int ny, int useUncalibrated)
{
	// Calibrate the stereo camera
    int displayCorners = 0;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 1;
    const double squareSize = 1.f; //Set this to your actual square size
    FILE *pfile;
	errno_t nError;
	nError = fopen_s( &pfile, imageList, "rt" );
	if( 0 != nError )
	{
		ROBOT_ASSERT(0);
	}
    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    CvMat _Mat1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _Mat2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
// READ IN THE LIST OF CHESSBOARDS:
// READ IN THE LIST OF CHESSBOARDS:
    if( !pfile )
    {
        ROBOT_LOG( TRUE,"CStereoDetector: ERROR: Can not open file %s\n", imageList )
        return;
    }
    for(i=0;;i++)
    {
        char buf[1024];
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, pfile ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);

		//FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            ROBOT_LOG( TRUE,"%s\n", buf)
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            ROBOT_LOG( TRUE,".")
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);

		//assert( result != 0 );
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    fclose(pfile);
    ROBOT_LOG( TRUE,"\n")

	// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] =
        cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_Mat1);
    cvSetIdentity(&_Mat2);
    cvZero(&_D1);
    cvZero(&_D2);

	// CALIBRATE THE STEREO CAMERAS
    ROBOT_LOG( TRUE,"CStereoDetector: Running stereo calibration ...\n")
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_Mat1, &_D1, &_Mat2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH );
	ROBOT_LOG( TRUE,"CStereoDetector: stereo calibration DONE\n")

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);

	//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_Mat1, &_D1, 0, &_Mat1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_Mat2, &_D2, 0, &_Mat2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    ROBOT_LOG( TRUE, "CStereoDetector: Average Error = %g\n", avgErr/(nframes*n) )


	//COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
// CvMat* 
		mx1 =   cvCreateMat( imageSize.height, imageSize.width, CV_32F );
		my1 =   cvCreateMat( imageSize.height, imageSize.width, CV_32F );
		mx2 =   cvCreateMat( imageSize.height, imageSize.width, CV_32F );
		my2 =   cvCreateMat( imageSize.height, imageSize.width, CV_32F );
		img1r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
		img2r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
		disp =  cvCreateMat( imageSize.height, imageSize.width, CV_16S );
		vdisp = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
		disp3d= cvCreateMat( imageSize.height, imageSize.width, CV_16SC3 ); // CV_16SC3 or CV_32FC3
		disp =  cvCreateMat( imageSize.height, imageSize.width, CV_16S );
		pair =  cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );

        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);

		// IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_Mat1, &_Mat2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &_Q,
                0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);

			//Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_Mat1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_Mat2,&_D2,&_R2,&_P2,mx2,my2);
        }
		//OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
		 // use intrinsic parameters of each camera, but
		 // compute the rectification transformation directly
		 // from the fundamental matrix
		{
            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);

	    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
                &_imagePoints2, &_F);
            cvStereoRectifyUncalibrated( &_imagePoints1,
                &_imagePoints2, &_F,
                imageSize,
                &_H1, &_H2, 3);
            cvInvert(&_Mat1, &_iM);
            cvMatMul(&_H1, &_Mat1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);
            cvInvert(&_Mat2, &_iM);
            cvMatMul(&_H2, &_Mat2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);

			//Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_Mat1,&_D1,&_R1,&_Mat1,mx1,my1);
            cvInitUndistortRectifyMap(&_Mat2,&_D1,&_R2,&_Mat2,mx2,my2);
        }
        else
		{
            assert(0);
		}

        cvNamedWindow( "rectified", 1 );

		// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
         pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );

		//Setup for finding stereo corrrespondences
        assert(BMState != 0);
        BMState->preFilterSize=41;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=41;
        BMState->minDisparity=-64;
        BMState->numberOfDisparities=128;
        BMState->textureThreshold=10;
        BMState->uniquenessRatio=15;
        BMState->speckleWindowSize=9;
        BMState->speckleRange=4;


		// Global Slider Variables for HiGui
		preFilterSizeSlider = BMState->preFilterSize;
		preFilterCapSlider = BMState->preFilterCap;
		SADWindowSizeSlider = BMState->SADWindowSize;

		minDisparitySlider = 0;			;
		numberOfDisparitiesSlider = 1;

		textureThresholdSlider = BMState->textureThreshold;
		uniquenessRatioSlider = BMState->uniquenessRatio;
		speckleWindowSizeSlider = BMState->speckleWindowSize;
		speckleRangeSlider = BMState->speckleRange;



        for( i = 0; i < nframes; i++ )
        {
            IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
            IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
            if( img1 && img2 )
            {
                CvMat part;
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
                if( !isVerticalStereo || useUncalibrated != 0 )
                {
              // When the stereo camera is oriented vertically,
              // useUncalibrated==0 does not transpose the
              // image, so the epipolar lines in the rectified
              // images are vertical. Stereo correspondence
              // function does not support such a case.
                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                        BMState);
                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                    cvNamedWindow( "disparity" );
                    cvShowImage( "disparity", vdisp );
                }
                cvGetCols( pair, &part, 0, imageSize.width );
                cvCvtColor( img1r, &part, CV_GRAY2BGR );
                cvGetCols( pair, &part, imageSize.width,
                    imageSize.width*2 );
                cvCvtColor( img2r, &part, CV_GRAY2BGR );
                for( j = 0; j < imageSize.height; j += 16 )
				{
                    cvLine( pair, cvPoint(0,j),
                    cvPoint(imageSize.width*2,j),
                    CV_RGB(0,255,0));
				}

                cvShowImage( "rectified", pair );

				// KLUDGE!  REQURED DUE TO BUG IN OPENCV.
				// NEED cvWaitKey() OR VIDEO DOES NOT DISPLAY!
				char c = (char)cvWaitKey(1);	
				if( c == 27 )
				{
					ROBOT_LOG( TRUE,"ESC KEY Pressed!\n")
					break;
				}
				Sleep(10);
            }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );


        }
		cvDestroyWindow( "rectified" );
		//cvDestroyWindow( "disparity" );

		// Now, add tracker bars to adjust parameters
#define USE_SLIDERS_TO_TUNE_3D
#ifdef USE_SLIDERS_TO_TUNE_3D 
		cvCreateTrackbar( "FilterSize", "disparity", &preFilterSizeSlider, 175, OnChange_preFilterSize );
		cvCreateTrackbar( "FilterCap", "disparity", &preFilterCapSlider, 163, OnChange_preFilterCap );
		cvCreateTrackbar( "SADSize", "disparity", &SADWindowSizeSlider, 155, OnChange_SADWindowSize );
		cvCreateTrackbar( "minDisparity", "disparity", &minDisparitySlider, 6, OnChange_minDisparity );
		cvCreateTrackbar( "nDisparities", "disparity", &numberOfDisparitiesSlider, 3, OnChange_numberOfDisparities );
		cvCreateTrackbar( "textThresh", "disparity", &textureThresholdSlider, 15, OnChange_textureThreshold );
		cvCreateTrackbar( "uniqueRatio", "disparity", &uniquenessRatioSlider, 125, OnChange_uniquenessRatio );

		cvCreateTrackbar( "speckleWinow", "disparity", &speckleWindowSizeSlider, 55, OnChange_speckleWindowSize );
		cvCreateTrackbar( "speckleRange", "disparity", &speckleRangeSlider, 112, OnChange_speckleRange );

#endif


/*

        BMState->preFilterSize=41;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=41;
        BMState->minDisparity=-64;
        BMState->numberOfDisparities=128;
        BMState->textureThreshold=10;
        BMState->uniquenessRatio=15;
	BMState->speckleWindowSize=9;			// 5x5..21x21 Edge of objects disparity variation window
	BMState->speckleRange=4;				// acceptable range of variation in window 
*/

    }

}
void CStereoDetector::ProcessFrames( IplImage* pVideoFrameLeft, IplImage* pVideoFrameRight )
{
	// Handle stereo image set

	ROBOT_ASSERT(BMState != 0);
	ROBOT_ASSERT( pVideoFrameLeft );
	ROBOT_ASSERT( pVideoFrameRight );

	IplImage* img1  = cvCreateImage( cvGetSize(pVideoFrameLeft), 8, 1 );
	IplImage* img2  = cvCreateImage( cvGetSize(pVideoFrameRight), 8, 1 );

	// Convert from color to Grayscale
	cvCvtColor( pVideoFrameLeft,  img1, CV_BGR2GRAY ); // copy and convert frame to grayscale
	cvCvtColor( pVideoFrameRight, img2, CV_BGR2GRAY ); // copy and convert frame to grayscale


//	CvMat part;
	cvRemap( img1, img1r, mx1, my1 );
	cvRemap( img2, img2r, mx2, my2 );

	cvFindStereoCorrespondenceBM( img1r, img2r, disp, BMState);
	cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
	//cvNamedWindow( "disparity" );
	cvShowImage( "disparity", vdisp );


//	cvReprojectImageTo3D( vdisp, disp3d, &_Q);
//	cvShowImage( "disparity", disp3d );


/***
	cvGetCols( pair, &part, 0, imageSize.width );
	cvCvtColor( img1r, &part, CV_GRAY2BGR );
	cvGetCols( pair, &part, imageSize.width, imageSize.width*2 );
	cvCvtColor( img2r, &part, CV_GRAY2BGR );

	for( int j = 0; j < imageSize.height; j += 16 )
	{
		cvLine( pair, cvPoint(0,j),
		cvPoint(imageSize.width*2,j),
		CV_RGB(0,255,0));
	}
***/
	//cvShowImage( "rectified", pair );

	// KLUDGE!  REQURED DUE TO BUG IN OPENCV. NEED cvWaitKey() OR VIDEO DOES NOT DISPLAY!
	char c = (char)cvWaitKey(1);	
	cvReleaseImage( &img1 );
	cvReleaseImage( &img2 );

 

}

#endif // THIS_DETECTOR_NOT_IMPLEMENTED
