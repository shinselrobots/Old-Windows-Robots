// KinectDetectors.cpp: Kinect Depth Image Object Detector classes
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

using namespace std;

#include "cxmisc.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

#include "ObjectKnowledge.h"

#include "Kinect.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//double MyQ[4][4];/
//CvMat _Q =  cvMat(4, 4, CV_64F, MyQ);

//#define USE_SLIDERS_TO_TUNE_3D	// enable this to have sliders appear on 3D GUI
//   CvSize size = cvSize(pSourceFrame->width,pSourceFrame->height); // get current frame size



//////////////////////////////////////////////////////////////////////////////
// CKinectDetector Base Class
//////////////////////////////////////////////////////////////////////////////

CKinectDetector::CKinectDetector(IplImage* pVideoFrame)
{
	ASSERT_POINTER( pVideoFrame, IplImage );
	m_pVideoFrame = pVideoFrame;
}



//////////////////////////////////////////////////////////////////////////////
// CKinectTrackObjectDetector
// Finds unique spots in image and tracks from frame to frame
// Good for keeping head tracking an object
//////////////////////////////////////////////////////////////////////////////
#define		MAX_COUNT 500
#define		ROI_SIZE	30

CKinectTrackObjectDetector::CKinectTrackObjectDetector(IplImage* pVideoFrame) :
	CKinectDetector(pVideoFrame)	// Call base class constructor
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

	ROBOT_LOG( TRUE,"CKinectTrackObjectDetector construction complete\n")
}

CKinectTrackObjectDetector::~CKinectTrackObjectDetector()
{
	// Release resources
	cvReleaseImage( &m_PreviousGrayImg );
	cvReleaseImage( &m_Pyramid );
	cvReleaseImage( &m_PreviousPyramid );

	ROBOT_LOG( TRUE,"~CKinectTrackObjectDetector done\n")
}

void CKinectTrackObjectDetector::Init()
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

BOOL CKinectTrackObjectDetector::LockOn()
{
	// Gets unique points to track
	cvCvtColor( m_pVideoFrame, m_GrayImg, CV_BGR2GRAY );

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


BOOL CKinectTrackObjectDetector::GetDelta( CvPoint &ObjectDelta )
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




#endif // ROBOT_SERVER