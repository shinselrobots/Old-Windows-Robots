#ifndef __CAMERA_DETECTORS_MODULE_H__
#define __CAMERA_DETECTORS_MODULE_H__


//#include "ObjectKnowledge.h"


/////////////////////////////////////////////////////////////////////////////
/*
class CCameraDetector
{

public:
	CCameraDetector( IplImage* pVideoFrame );	// May be Left or Right camera frame!
	~CCameraDetector(){};

	// Common Member valiables
	IplImage*	m_pVideoFrame;	// Pointer to the current Video Frame to process

	// Common Functions handled by base class
	CvSize	GetFrameSize() 
	{ return cvGetSize(m_pVideoFrame); }

	CvPoint	GetFrameCenter()
	{ return cvPoint( (m_pVideoFrame->width / 2), (m_pVideoFrame->height / 2) ); }

	CvPoint2D32f GetFrameCenterFloat()	// float version
	{ return cvPoint2D32f( ((double)m_pVideoFrame->width / 2.0), ((double)m_pVideoFrame->height / 2.0) ); }

	double	GetFrameAspectRatio()
	{ return ( (double)m_pVideoFrame->height/(double)m_pVideoFrame->width ); }

	// Overrides
	//virtual void Initialize();

};
*/

/////////////////////////////////////////////////////////////////////////////
class CFaceRecognizer
{

//#define	MIN_OBJECT_SIZE			100	// component size
//#define MOTION_RING_BUF_SIZE	  3	// number of cyclic frame buffer used for motion detection
									// (should, probably, depend on FPS)

#define CAMERA_FACE_DETECTOR_PATH ROBOT_DATA_PATH "\\OpenCV\\data\\haarcascades\\haarcascade_frontalface_default.xml"
// Not used (yet):#define CAMERA_UPPER_BODY_DETECTOR_PATH "C:/RobotData/OpenCV/data/haarcascades/haarcascade_upperbody.xml"


public:
	CFaceRecognizer();
	~CFaceRecognizer();
	BOOL Init();
	void SaveNextFace( int RequestedPersonID, string RequestedPersonName  );
	void ProcessFrame( Mat	&original, BOOL &bFaceDetected, BOOL &bPersonRecognized, int &DetectedPersonID, string &DetectedPersonName );

protected:
	int		m_PersonID;
	int		m_NextPersonID;
	int		m_FileNumber; // guarantee file name is always unique
	string	m_PersonName;
	string	m_ImgFileName;
	BOOL	m_CaptureNewFace;

	int		m_NumberOfTrainedFaces;
	int		m_FaceImageWidth;
	int		m_FaceImageHeight;
	int		m_PredictedID;
	double	m_Confidence;

    CascadeClassifier haar_cascade;


    //string fn_haar = string(argv[1]); - replaced wtih face_cascade_name
    // These vectors hold the images and corresponding labels:
    vector<int>		PersonIDs;
    vector<string>	PersonNames;
    vector<string>	ImgFileNames;
    vector<Mat>		Images;

	Ptr<FaceRecognizer> model;



//	CvHaarClassifierCascade*	m_pFaceDetectCascade;	// the face detector
//	CvMemStorage*				m_pFaceDetectStorage;	// memory for the face detector to use
//	CvSeq*						m_pFaceRectSeq;	// memory-access interface
//	char*						m_FaceDetectorXMLPath;
//	int 						m_FaceTrackingState;
//	CvRect						m_FaceRect;

public:
//	CvSeq*	DetectFaces();


};


/////////////////////////////////////////////////////////////////////////////
class CMatchObjectDetector
{



public:
	CMatchObjectDetector();
	~CMatchObjectDetector();

	int  Init();
	void ProcessFrame( Mat &original, bool &ObjectRecognized, int &DetectedObjectID, string &DetectedObjectString );

protected:

	void maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask );
	void readTrainFilenames( const string& filename, string& dirName, vector<string>& trainFilenames );
	bool createDetectorDescriptorMatcher( const string& detectorType, const string& descriptorType, const string& matcherType,
                                      Ptr<FeatureDetector>& featureDetector,
                                      Ptr<DescriptorExtractor>& descriptorExtractor,
                                      Ptr<DescriptorMatcher>& descriptorMatcher );
	bool readTrainImages( const string& trainFilename,
		             vector <Mat>& trainImages, vector<string>& trainImageNames );

	void detectQueryKeypoints( const Mat& queryImage, vector<KeyPoint>& queryKeypoints,
                                   Ptr<FeatureDetector>& featureDetector );

	void detectTrainKeypoints( 
                      const vector<Mat>& trainImages, vector<vector<KeyPoint> >& trainKeypoints,
                      Ptr<FeatureDetector>& featureDetector );

	void computeQueryDescriptors( const Mat& queryImage, vector<KeyPoint>& queryKeypoints, Mat& queryDescriptors,
                                      Ptr<DescriptorExtractor>& descriptorExtractor );


	void computeTrainDescriptors( 
                         const vector<Mat>& trainImages, vector<vector<KeyPoint> >& trainKeypoints, vector<Mat>& trainDescriptors,
                         Ptr<DescriptorExtractor>& descriptorExtractor );

	void matchQueryDescriptors( const Mat& queryDescriptors, const vector<Mat>& trainDescriptors,
                       vector<DMatch>& matches, Ptr<DescriptorMatcher>& descriptorMatcher );

	void loadTrainDescriptors( const vector<Mat>& trainDescriptors,
                       Ptr<DescriptorMatcher>& descriptorMatcher );

	void saveResultImages( const Mat& queryImage, const vector<KeyPoint>& queryKeypoints,
                       const vector<Mat>& trainImages, const vector<vector<KeyPoint> >& trainKeypoints,
                       const vector<DMatch>& matches, const vector<string>& trainImagesNames, const string& resultDir );


	int nTrainingImages;
	Mat queryImage;
	Mat captureImage;
    Ptr<FeatureDetector> featureDetector;
    Ptr<DescriptorExtractor> descriptorExtractor;
    Ptr<DescriptorMatcher> descriptorMatcher;
    vector<Mat> trainDescriptors;
    vector<DMatch> matches;
    vector<Mat> trainImages;
    vector<string> trainImagesNames;
    vector<vector<KeyPoint> > trainKeypoints;




};



#ifdef THIS_DETECTOR_NOT_IMPLEMENTED
/////////////////////////////////////////////////////////////////////////////
class CFaceDetector : public CCameraDetector
{

//#define	MIN_OBJECT_SIZE			100	// component size
//#define MOTION_RING_BUF_SIZE	  3	// number of cyclic frame buffer used for motion detection
									// (should, probably, depend on FPS)

#define CAMERA_FACE_DETECTOR_PATH ROBOT_DATA_PATH "\\OpenCV\\data\\haarcascades\\haarcascade_frontalface_default.xml"
// Not used (yet):#define CAMERA_UPPER_BODY_DETECTOR_PATH "C:/RobotData/OpenCV/data/haarcascades/haarcascade_upperbody.xml"


public:
	CFaceDetector(IplImage* pVideoFrame);
	~CFaceDetector();

protected:
	CvHaarClassifierCascade*	m_pFaceDetectCascade;	// the face detector
	CvMemStorage*				m_pFaceDetectStorage;	// memory for the face detector to use
//	CvSeq*						m_pFaceRectSeq;	// memory-access interface
	char*						m_FaceDetectorXMLPath;
	int 						m_FaceTrackingState;
	CvRect						m_FaceRect;

public:
	CvSeq*	DetectFaces();


};

/////////////////////////////////////////////////////////////////////////////
class CMotionDetector : public CCameraDetector
{

#define	MIN_OBJECT_SIZE			100	// component size
#define MOTION_RING_BUF_SIZE	  3	// (4) number of cyclic frame buffer used for motion detection
									// (should, probably, depend on FPS)
public:
	CMotionDetector(IplImage* pVideoFrame);
	~CMotionDetector();

//	void	Initialize();
//	void	InitMotionDetector();
//	void	ReleaseMotionDetector();

	CvPoint	DetectMotion( IplImage* pMotionDisplayFrame );


protected:

	// ring image buffer
	IplImage **ppMotionRingBuffer; 
	int m_Last;

	// temporary images
	IplImage*		mhi;		// MHI
	IplImage*		orient;		// orientation
	IplImage*		mask;		// valid orientation mask
	IplImage*		segmask;	// motion segmentation map
	CvMemStorage*	storage;	// temporary storage

};

/////////////////////////////////////////////////////////////////////////////
class CCamShiftDetector : public CCameraDetector
{

									// (should, probably, depend on FPS)
public:
	CCamShiftDetector(IplImage* pVideoFrame);
	~CCamShiftDetector();

protected:
	int					nHistBins;				// number of histogram bins
	float				rangesArr[2];			// histogram range
	CvRect				m_prevColorRect;		// location of color blob in previous frame
	int					vmin;
	int					vmax; 
	int					smin;				// limits for calculating hue
	CvRect				m_FooRect;			// location of color blob in previous frame

	IplImage*			pHSVImg;			// the input image converted to HSV color mode
	IplImage*			pHueImg;			// the Hue channel of the HSV image
	IplImage*			pMask;				// this image is used for masking pixels
	IplImage*			pProbImg;			// the face probability estimates for each pixel
	CvHistogram*		pHist;				// histogram of hue in the original face image

	CvPoint				m_CamShiftCenter;	// current face-location estimate from CamShift
	int 				m_State;			// State of the tracker
	//	int					nFrames;


public:
	void	startTracking( CvRect ColorRect );
	CvBox2D track();
	void	updateHueImage(const IplImage * pImg);
	void	setVmin(int _vmin);
	void	setSmin(int _smin);
	int 	GetState() { return m_State; }


protected:


};


/////////////////////////////////////////////////////////////////////////////
class CColorDetector : public CCameraDetector
{

public:
	CColorDetector(IplImage* pVideoFrame);
	~CColorDetector();

protected:

	int 				m_CrTarget;			// Red Target value
	int 				m_CbTarget;			// Blue Target value
	int					m_CrThreshold;
	int					m_CbThreshold; 

	IplImage*			pYCrCbImg;			// the YCrCb version of the image
//	IplImage*			pCrImg;				// the Cr channel of the YCrCb image
//	IplImage*			pCbImg;				// the Cb channel of the YCrCb image

	CvPoint				m_ColorCenter;	// current face-location estimate from Color
	BOOL				m_bInitialized;
	int 				m_CrMax;
	int 				m_CrMin;
	int 				m_CbMax;
	int 				m_CbMin;

public:
	int 	GetCrTarget() { return m_CrTarget; }
	int 	GetCbTarget() { return m_CbTarget; }

	void	SetColor( CvRect ColorRect );	// Auto set Cr, Cb from current image
	void	SetColor( int  Cr, int  Cb );	// Manually set Cr, Cb


	// Returns Center of Color Blob
	CvPoint FindColorBlob( IplImage* pColorBlobDisplayFrame, // debug display image
				CvPoint &BBpt1, CvPoint &BBpt2 );			 // bounding box of Color Blob

	void	setCrThreshold(int Threshold);
	void	setCbThreshold(int Threshold);
	BOOL	Initialized() { return m_bInitialized; }

	void	UpdateCrCbImages(const IplImage * pImg); // CrCb based

protected:


};

/////////////////////////////////////////////////////////////////////////////
class CLaserSpotDetector : public CCameraDetector
{

public:
	CLaserSpotDetector(IplImage* pVideoFrame);
	~CLaserSpotDetector();

protected:

	int					m_nPositiveFrames;	// Number of frames in a row where laser was detected
	CvPoint				m_LastSpotPosition; // Position of laser found in prior frame

//	IplImage*			pYCrCbImg;			// the YCrCb version of the image
//	IplImage*			pCrImg;				// the Cr channel of the YCrCb image
//	IplImage*			pCbImg;				// the Cb channel of the YCrCb image

//	CvPoint				m_ColorCenter;	// current face-location estimate from Color
//	BOOL				m_bInitialized;

public:

	// Returns position of Laser Spot. If not found, returns {0,0}
	CvPoint CLaserSpotDetector::FindLaserDot();

//	BOOL	Initialized() { return m_bInitialized; }
//	void	UpdateCrCbImages(const IplImage * pImg); // CrCb based

protected:


};

/////////////////////////////////////////////////////////////////////////////
typedef struct
{
	int  value;
	int  position;
} LINE_PIXEL_T;

class CLaserLineDetector : public CCameraDetector
{

public:
	CLaserLineDetector(IplImage* pVideoFrame);
	~CLaserLineDetector();

protected:
	int					m_nPositiveFrames;	// Number of frames in a row where laser was detected
	CvPoint				m_LastSpotPosition; // Position of laser found in prior frame
	int					m_width;			// Frame Width and Height
	int					m_height;

	LINE_PIXEL_T*		m_pLaserDotArray;	// Value and positon of laser points for each column in the frame

//	IplImage*			pYCrCbImg;			// the YCrCb version of the image
//	IplImage*			pCrImg;				// the Cr channel of the YCrCb image
//	IplImage*			pCbImg;				// the Cb channel of the YCrCb image

//	CvPoint				m_ColorCenter;	// current face-location estimate from Color
//	BOOL				m_bInitialized;

public:

	// Find Laser lines in the frame. If none found, returns FALSE
	BOOL CLaserLineDetector::FindLaserLines();
	int  CLaserLineDetector::GetPositionY(int  X);

protected:


};

/////////////////////////////////////////////////////////////////////////////
class CTrackObjectDetector : public CCameraDetector
{

public:
	CTrackObjectDetector(IplImage* pVideoFrame);
	~CTrackObjectDetector();

protected:

	IplImage			*m_GrayImg;
	IplImage			*m_PreviousGrayImg;
	IplImage			*m_Pyramid;
	IplImage			*m_PreviousPyramid;
	IplImage			*m_SwapTemp;

	int					m_width;
	int					m_height;
	int					m_WindowSize;
	CvPoint2D32f		*m_Points[2];
	CvPoint2D32f		*m_SwapPoints;
	char* 				m_Status;
	int 				m_FeatureCount;
	int 				m_Flags;
	int 				add_remove_pt;
	CvPoint 			m_Pt;
	CvPoint				m_BoxPt1;
	CvPoint				m_BoxPt2;
	CvPoint				m_CenterPt;
	CvPoint2D32f		m_ReferenceCloudCenter;


public:

	// Find Laser lines in the frame. If none found, returns FALSE
	void Init();
	BOOL LockOn( BOOL FindWallSocket = FALSE );
	BOOL GetDelta( CvPoint &ObjectDelta );

protected:


};





/////////////////////////////////////////////////////////////////////////////
class CStereoDetector : public CCameraDetector
{

public:
	CStereoDetector(IplImage* pVideoFrame);
	~CStereoDetector();


protected:


    CvSize imageSize;

	CvMat* mx1;
	CvMat* my1;
	CvMat* mx2;
	CvMat* my2;
	CvMat* img1r;
	CvMat* img2r;
	CvMat* disp;
	CvMat* disp3d;
	CvMat* vdisp;
	CvMat* pair;
//	double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
//	CvMat _R1 = cvMat(3, 3, CV_64F, R1);
//	CvMat _R2 = cvMat(3, 3, CV_64F, R2);




//	int					m_nPositiveFrames;	// Number of frames in a row where laser was detected
//	CvPoint				m_LastSpotPosition; // Position of laser found in prior frame

//	IplImage*			pYCrCbImg;			// the YCrCb version of the image
//	IplImage*			pCrImg;				// the Cr channel of the YCrCb image
//	IplImage*			pCbImg;				// the Cb channel of the YCrCb image

//	CvPoint				m_ColorCenter;	// current face-location estimate from Color
//	BOOL				m_bInitialized;

public:

	//void CStereoDetector::Calibrate();
	void CStereoDetector::Calibrate( const char* imageList, int nx, int ny, int useUncalibrated );
	void CStereoDetector::ProcessFrames( IplImage* pVideoFrameLeft, IplImage* pVideoFrameRight );


//	BOOL	Initialized() { return m_bInitialized; }
//	void	UpdateCrCbImages(const IplImage * pImg); // CrCb based

protected:


};


#endif // THIS_DETECTOR_NOT_IMPLEMENTED



#endif // __CAMERA_DETECTORS_MODULE_H__