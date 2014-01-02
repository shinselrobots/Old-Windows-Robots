// ObjectDetector.cpp: Video Image Object Detector classes
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CameraCommon.h"

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


//#define ROBOT_DATA_PATH	"C:\\Dev\\_Robot\\LokiData"
//#define ROBOT_COMMON_PATH L"C:\\Dev\\_Robot\\Common"
//#define CAMERA_FACE_DETECTOR_PATH ROBOT_DATA_PATH "\\OpenCV\\data\\haarcascades\\haarcascade_frontalface_default.xml"




#if DETECTOR_METHOD == USING_SURF
	const string defaultDetectorType = "SURF";
	const string defaultDescriptorType = "SURF";
	const string defaultMatcherType = "FlannBased";
#elif DETECTOR_METHOD == USING_BRISK
	const string defaultDetectorType = "BRISK";
	const string defaultDescriptorType = "BRISK";
	const string defaultMatcherType = "BruteForce-Hamming";
#else
	const string defaultDetectorType = "ORB";
	const string defaultDescriptorType = "ORB";
	const string defaultMatcherType = "BruteForce";
#endif

const string defaultQueryImageName = "C:/temp/matching_to_many_images/query.jpg";
const string defaultFileWithTrainImages = "C:/temp/matching_to_many_images/train/trainImages.txt";
const string defaultDirToSaveResImages = "C:/temp/matching_to_many_images/results";


string detectorType = defaultDetectorType;
string descriptorType = defaultDescriptorType;
string matcherType = defaultMatcherType;
string queryImageName = defaultQueryImageName;
string fileWithTrainImages = defaultFileWithTrainImages;
string dirToSaveResImages = defaultDirToSaveResImages;




//////////////////////////////////////////////////////////////////////////////
// CMatchObjectDetector
// Looks for known objects using SURF detector
//////////////////////////////////////////////////////////////////////////////


CMatchObjectDetector::CMatchObjectDetector()
{
	int   nTrainingImages = 0;
}

CMatchObjectDetector::~CMatchObjectDetector()
{
	// Release resources
///	DeleteAllSURFInfoItems();
//	cvReleaseImage( &pCbImg );
}

////////////////////////////////////////////
int CMatchObjectDetector::Init()
{
	/// Walk through the list of known objects, and for each object image, extract SURF parameters.
	/// Process each image to extract features
	/// Load up data structure with object info.
	/// For each Known Object, load it's image and create SURF keypoints


//#if DETECTOR_METHOD == USING_SURF
	cv::initModule_nonfree(); //THIS LINE IS IMPORTANT   
//#endif

	////////////////////////////////////////////////////////////////////////////
	// Train images


	cout << "Training Images..." << endl;
    if( !createDetectorDescriptorMatcher( detectorType, descriptorType, matcherType, featureDetector, descriptorExtractor, descriptorMatcher ) )
    {
		cout << "createDetectorDescriptorMatcher Failed!" << endl;
        return -1;
    }

	// Load up and train the training images
    TickMeter tm;
    tm.start(); // track how long it takes to train images

    vector<Mat> trainImages;
    vector<string> trainImagesNames;
    if( !readTrainImages( fileWithTrainImages, trainImages, trainImagesNames ) )
    {
		cout << "readTrainImages Failed!" << endl;
        return -1;
    }

	cout << "Detect Keypoints..." << endl;

    detectTrainKeypoints( trainImages, trainKeypoints, featureDetector );

    computeTrainDescriptors( 
                        trainImages, trainKeypoints, trainDescriptors,
                        descriptorExtractor );

    loadTrainDescriptors( trainDescriptors,  descriptorMatcher );

	cout << "Done Training.  ";

	// Done training
    tm.stop();
    double trainTime = tm.getTimeMilli();
    cout << "TOTAL TRAINING TIME: " << trainTime << " ms" << endl;

	return 1;
}


void CMatchObjectDetector::ProcessFrame( Mat &original, bool &ObjectRecognized, int &DetectedObjectID, string &DetectedObjectString )
{

//    cout << "Best Image Index: " << BestMatchIndex << " Best Image matching points: " << BestScore << " Dist: " << ClosestMatchDistance << endl;

	// Set a sub-set of the frame as the Region Of Interest
	int ROI_Height = 250;
	int ROI_Width =  250;
	Rect ROIrect;
	ROIrect.x = 320 - (ROI_Width/2);
	ROIrect.y = 240 - (ROI_Height/2);
	ROIrect.width = ROI_Width;
	ROIrect.height = ROI_Height;

	queryImage = captureImage( ROIrect ); // limit to just the ROI
	// draw ROI for the display
	//rectangle(captureImage, ROIrect, CV_RGB(0, 0, 255), 2);


	// Now Match Query Image to the training images
	cout << endl << "************************************************" << endl;
	TickMeter qm;
	qm.start(); // track how long it takes to match new query image

	vector<KeyPoint> queryKeypoints;
	detectQueryKeypoints( queryImage, queryKeypoints,  featureDetector );

	Mat queryDescriptors;
	computeQueryDescriptors( queryImage, queryKeypoints, queryDescriptors,
						descriptorExtractor );

	matchQueryDescriptors( queryDescriptors, trainDescriptors, matches, descriptorMatcher );

	// Done matching
	qm.stop();
	double matchTime = qm.getTimeMilli();
	cout << "TOTAL MATCHING TIME: " << matchTime << " ms" << endl;

	// Save Results

	saveResultImages( queryImage, queryKeypoints, trainImages, trainKeypoints,
						matches, trainImagesNames, dirToSaveResImages );

}


void CMatchObjectDetector::maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask )
{
    mask.resize( matches.size() );
    fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == trainImgIdx )
            mask[i] = 1;
    }
}


void CMatchObjectDetector::readTrainFilenames( const string& filename, string& dirName, vector<string>& trainFilenames )
{
    trainFilenames.clear();

    ifstream file( filename.c_str() );
    if ( !file.is_open() )
        return;

    size_t pos = filename.rfind('\\');
    char dlmtr = '\\';
    if (pos == String::npos)
    {
        pos = filename.rfind('/');
        dlmtr = '/';
    }
    dirName = pos == string::npos ? "" : filename.substr(0, pos) + dlmtr;

    while( !file.eof() )
    {
        string str; getline( file, str );
        if( str.empty() ) break;
        trainFilenames.push_back(str);
    }
    file.close();
}


bool CMatchObjectDetector::createDetectorDescriptorMatcher( const string& detectorType, const string& descriptorType, const string& matcherType,
                                      Ptr<FeatureDetector>& featureDetector,
                                      Ptr<DescriptorExtractor>& descriptorExtractor,
                                      Ptr<DescriptorMatcher>& descriptorMatcher )
{
    cout << "< Creating feature detector, descriptor extractor and descriptor matcher ..." << endl;
    featureDetector = FeatureDetector::create( detectorType );
    descriptorExtractor = DescriptorExtractor::create( descriptorType );
    descriptorMatcher = DescriptorMatcher::create( matcherType );
    cout << ">" << endl;

    bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );
    if( !isCreated )
        cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;

    return isCreated;
}

bool CMatchObjectDetector::readTrainImages( const string& trainFilename,
                 vector <Mat>& trainImages, vector<string>& trainImageNames )
{
    cout << "< Reading Training images..." << endl;
    string trainDirName;
    readTrainFilenames( trainFilename, trainDirName, trainImageNames );
    if( trainImageNames.empty() )
    {
        cout << "Train image filenames can not be read." << endl << ">" << endl;
        return false;
    }
    int readImageCount = 0;
    for( size_t i = 0; i < trainImageNames.size(); i++ )
    {
        string filename = trainDirName + trainImageNames[i];
        Mat img = imread( filename, CV_LOAD_IMAGE_GRAYSCALE );
        if( img.empty() )
            cout << "Train image " << filename << " can not be read." << endl;
        else
		{
            cout << readImageCount << " - " << filename << endl;
            readImageCount++;
		}
		trainImages.push_back( img );
    }
    if( !readImageCount )
    {
        cout << "All train images can not be read." << endl << ">" << endl;
        return false;
    }
    else
        cout << readImageCount << " train images were read." << endl;
    cout << ">" << endl;

	nTrainingImages = readImageCount;
    return true;
}

void CMatchObjectDetector::detectQueryKeypoints( const Mat& queryImage, vector<KeyPoint>& queryKeypoints,
                                   Ptr<FeatureDetector>& featureDetector )
{
    cout << endl << "< Extracting keypoints from Query image..." << endl;
    featureDetector->detect( queryImage, queryKeypoints );
    cout << ">" << endl;
}
void CMatchObjectDetector::detectTrainKeypoints( 
                      const vector<Mat>& trainImages, vector<vector<KeyPoint> >& trainKeypoints,
                      Ptr<FeatureDetector>& featureDetector )
{
    cout << endl << "< Extracting keypoints from images..." << endl;
    featureDetector->detect( trainImages, trainKeypoints );
    cout << ">" << endl;
}

void CMatchObjectDetector::computeQueryDescriptors( const Mat& queryImage, vector<KeyPoint>& queryKeypoints, Mat& queryDescriptors,
                                      Ptr<DescriptorExtractor>& descriptorExtractor )
{
    cout << "< Computing descriptors for Query keypoints..." << endl;
    descriptorExtractor->compute( queryImage, queryKeypoints, queryDescriptors );
    cout << "Query descriptors count: " << queryDescriptors.rows << endl;
    cout << ">" << endl;
}
void CMatchObjectDetector::computeTrainDescriptors( 
                         const vector<Mat>& trainImages, vector<vector<KeyPoint> >& trainKeypoints, vector<Mat>& trainDescriptors,
                         Ptr<DescriptorExtractor>& descriptorExtractor )
{
    cout << "< Computing descriptors for Train keypoints..." << endl;
    descriptorExtractor->compute( trainImages, trainKeypoints, trainDescriptors );

    int totalTrainDesc = 0;
    for( vector<Mat>::const_iterator tdIter = trainDescriptors.begin(); tdIter != trainDescriptors.end(); tdIter++ )
        totalTrainDesc += tdIter->rows;

    cout << "Total train descriptors count: " << totalTrainDesc << endl;
    cout << ">" << endl;
}


void CMatchObjectDetector::matchQueryDescriptors( const Mat& queryDescriptors, const vector<Mat>& trainDescriptors,
                       vector<DMatch>& matches, Ptr<DescriptorMatcher>& descriptorMatcher )
{
    cout << "< Match query descriptors to train descriptors collection ..." << endl;
    TickMeter tm;

    tm.start();
    descriptorMatcher->match( queryDescriptors, matches );
    tm.stop();
    double matchTime = tm.getTimeMilli();

    CV_Assert( queryDescriptors.rows == (int)matches.size() || matches.empty() );

    cout << "Number of matches: " << matches.size() << endl;
    cout << "Match time: " << matchTime << " ms" << endl;


	// Find closest image
	int imgScore[100]; // array of scores for each image
	float ClosestMatchDistance = 1000.0;
	for(int i=0; i<100; i++)
	{
		imgScore[i]=0;
	}

    for( size_t i = 0; i < matches.size(); i++ )
    {
//        if( matches[i].distance < 0.4 ) // TUNE THIS VALUE
		{
			imgScore[matches[i].imgIdx] += 1; // give that image a point!
			if( matches[i].distance < ClosestMatchDistance )
			{
				ClosestMatchDistance = matches[i].distance;
			}
		}
    }

	// which image got the best score?
	int BestMatchIndex = -1;
	int BestScore = 0;
	for( int i=0; i< nTrainingImages; i++ )
	{
		if( imgScore[i] > BestScore )
		{
			BestScore = imgScore[i];
			BestMatchIndex = i;
		}
	}
    cout << "Best Image Index: " << BestMatchIndex << " Best Image matching points: " << BestScore << " Dist: " << ClosestMatchDistance << endl;

    cout << ">" << endl;
}


void CMatchObjectDetector::loadTrainDescriptors( const vector<Mat>& trainDescriptors,
                       Ptr<DescriptorMatcher>& descriptorMatcher )
{
    cout << "< Set train descriptors collection in the matcher..." << endl;
    TickMeter tm;

    tm.start();
    descriptorMatcher->add( trainDescriptors );
    descriptorMatcher->train();
    tm.stop();
    double buildTime = tm.getTimeMilli();

    cout << "Build time: " << buildTime << " ms" << endl;
    cout << ">" << endl;
}


void CMatchObjectDetector::saveResultImages( const Mat& queryImage, const vector<KeyPoint>& queryKeypoints,
                       const vector<Mat>& trainImages, const vector<vector<KeyPoint> >& trainKeypoints,
                       const vector<DMatch>& matches, const vector<string>& trainImagesNames, const string& resultDir )
{
    cout << "< Save results..." << endl;
    Mat drawImg;
    vector<char> mask;
    for( size_t i = 0; i < trainImages.size(); i++ )
    {
        if( !trainImages[i].empty() )
        {
            maskMatchesByTrainImgIdx( matches, (int)i, mask );
            drawMatches( queryImage, queryKeypoints, trainImages[i], trainKeypoints[i],
                         matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask );
            string filename = resultDir + "/res_" + trainImagesNames[i];
            if( !imwrite( filename, drawImg ) )
                cout << "Image " << filename << " can not be saved (may be because directory " << resultDir << " does not exist)." << endl;
        }
    }
    cout << ">" << endl;
}


