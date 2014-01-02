// KinectModule.cpp: CKinectModule class implementation
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
#include "Globals.h"
#include "module.h"
#include "thread.h"
//#include "videoInput.h" 
#include "cv.h"
//#include "cvcam.h"
#include "highgui.h"
#include "HardwareConfig.h"
#include "kinect.h"
#include <math.h>
#define TRACK_TOP_OF_USERS_HEAD				0 
#define USE_KINECT_TO_LOOKAT_HUMAN			0 // NOTE!  REQUIRES "TRACK_TOP_OF_USERS_HEAD"
#define TILT_KINECT_TO_TRACK_CLOSE_HUMANS	0

// NOTE: Make sure to add one of these libraries to the Linker Input Dependencies:
// For Kinect SDK, add MSRKinectNUI.lib
// For OpenNI SDK, add openNI.lib


#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
	// For Kinect OpenNI library
	// #include <XnOS.h>
	// #include <XnOpenNI.h>
	// #include <XnCodecIDs.h>
	#include <XnCppWrapper.h>
	// #include "SceneDrawer.h"
	#include <XnPropNames.h>
	#include <map>
	std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;

	using namespace xn;
#endif

// ITT Instrumentation
__itt_string_handle* pshKinectThreadLoop = __itt_string_handle_create("KinectThreadLoop");
__itt_string_handle* pshKinectInit = __itt_string_handle_create("KinectInit");
__itt_string_handle* pshKinectGetFrame = __itt_string_handle_create("KinectGetFrame");
__itt_string_handle* pshKinectGetDepthImage = __itt_string_handle_create("KinectGetDepthImage");
__itt_string_handle* pshKinectGetVideoImage = __itt_string_handle_create("KinectGetVideoImage");
__itt_string_handle* pshKinectShowFrames = __itt_string_handle_create("KinectShowFrames");
__itt_string_handle* pshKinectSleep = __itt_string_handle_create("KinectSleep");
__itt_string_handle* pshFindObjectsOnFloor = __itt_string_handle_create("Find Objects on Floor");

__itt_string_handle* pshKinectServoStart = __itt_string_handle_create("KinectServoMoveStart"); // marker
__itt_string_handle* pshKinectServoEnd = __itt_string_handle_create("KinectServoMoveEnd"); // marker



///////////////////////////////////////////////////////////////////////////////
// OpenNI Stuff

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
//#define SAMPLE_XML_PATH "../../../Data/SamplesConfig.xml"
#define SAMPLE_XML_PATH "C:\\Program Files (x86)\\OpenNI\\Data\\SamplesConfig.xml"
//#define SAMPLE_XML_PATH "C:\\Program Files\\OpenNI\\Data\\SamplesConfig.xml"

#define KINECT_VIDEO_ENABLED  1

// SLICE PARAMETERS
#define KINECT_SLICE_OBJECT_HEIGHT_MIN			 5.0	// TenthInches
#define KINECT_SLICE_OBJECT_WIDTH_MIN			 5.0	// TenthInches
#define KINECT_SLICE_OBJECT_HEIGHT_MAX		    80.0	// TenthInches
#define KINECT_SLICE_OBJECT_WIDTH_MAX		    70.0	// TenthInches
#define KINECT_MAX_FLOOR_HEIGHT_TENTH_INCHES	20.0	// Tenthinches - prevent finding object sitting on top of steps, etc.

// FINAL 3D OBJECT PARAMETERS
#define MIN_3D_OBJECT_LENGTH			15	// TenthInches - minimim size of object to detect
#define MIN_3D_OBJECT_WIDTH				10	// 
#define MAX_3D_OBJECT_LENGTH			65	// TenthInches - maximum size of object to detect
#define MAX_3D_OBJECT_WIDTH				65
#define MAX_3D_OBJECT_X_POSITION	   480	// TenthInches - max distance from center in front of robot (ignore objects too far to the side)
#define LEFT_WINDOW_MARGIN				12

#define KINECT_WALL_DETECT_HEIGHT	    80	// TenthInches - Anything taller than this is considered a "wall" or unmovable object
#define MAP_NOISE_FLOOR_TENTHINCHES		10  // Tenthinches - ignore objects shorter than this when making 2D map

// Reserve top 1/3 scan lines to detect walls at the top of the frame (0 = top of frame)
#define KINECT_WALL_SCAN_ZONE		(KINECT_CAPTURE_SIZE_Y / 3)


///#define GL_WIN_SIZE_X 1280
///#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MAX_DEPTH 10000

#define DEFAULT_KINECT_MOVE_TIME_LIMIT_TENTH_SECONDS	  100	// Default time for servo moves = 10 Seconds (100 Tenth Seconds)



//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE ) // USING OPENNI API!

	Context g_Context;
	ScriptNode g_scriptNode;
	DepthGenerator g_DepthGenerator;
	ImageGenerator g_ImageGenerator;
	UserGenerator g_UserGenerator;
	Player g_Player;

	DepthMetaData pDepthMD;
	SceneMetaData pSceneMD;
	ImageMetaData g_imageMD;

	XnBool g_bNeedPose = FALSE;
	XnChar g_strPose[20] = "";
	XnBool g_bDrawBackground = TRUE;
	XnBool g_bDrawPixels = TRUE;
	XnBool g_bDrawSkeleton = TRUE;
	XnBool g_bPrintID = TRUE;
	XnBool g_bPrintState = TRUE;

	KINECT_HUMAN_TRACKING_T HumanLocationTracking[KINECT_MAX_HUMANS_TO_TRACK];


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		DWS -  NOTE!!! IF THIS IS NOT WORKING ROBUSTLY, SEE NiUserTracker sample.  it works really well!!!
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////


//	void DrawDepthMap(const DepthMetaData& pDepthMD, const SceneMetaData& pSceneMD);
//	void XN_CALLBACK_TYPE MyCalibrationInProgress(SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie);
//	void XN_CALLBACK_TYPE MyPoseInProgress(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie);

	void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie)
	{
		m_Errors[id].first = calibrationError;
	}
	void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie)
	{
		m_Errors[id].second = poseError;
	}


	void glPrintString(void *font, char *str)
	{
		ROBOT_ASSERT(0); // TODO implement some sort of drawing mechanis to show payer number
		int i,l = strlen(str);
		for(i=0; i<l; i++)
		{
			//glutBitmapCharacter(font,*str++);
		}
	}

	void DrawLimb( XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2 )
	{

		if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
		{
			ROBOT_DISPLAY( TRUE, "KINECT ERROR: DrawLimb Called,but Player not tracked!\n" )
			return;
		}

		XnSkeletonJointPosition joint1, joint2;
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

		if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
		{
			return;
		}

		XnPoint3D pt[2];
		pt[0] = joint1.position;
		pt[1] = joint2.position;

		g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);

		// Get a pointer to the KinectModule Object
		CKinectModule* pKinectModule = (CKinectModule*) g_pKinectModule;
		ROBOT_ASSERT( pKinectModule );
		CvPoint pt1, pt2;

		pt1.x = (int)pt[0].X;
		pt1.y = (int)pt[0].Y;
		pt2.x = (int)pt[1].X;
		pt2.y = (int)pt[1].Y;
		cvLine( pKinectModule->m_pDepthFrame, pt1, pt2, CV_RGB(255,255,255), 2, 4, 0);

/**
	#ifndef USE_GLES
		glVertex3i(pt[0].X, pt[0].Y, 0);
		glVertex3i(pt[1].X, pt[1].Y, 0);
	#else
		GLfloat verts[4] = {pt[0].X, pt[0].Y, pt[1].X, pt[1].Y};
		glVertexPointer(2, GL_FLOAT, 0, verts);
		glDrawArrays(GL_LINES, 0, 2);
		glFlush();
	#endif
	**/

	}

	const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
	{
		switch (error)
		{
		case XN_CALIBRATION_STATUS_OK:
			return "OK";
		case XN_CALIBRATION_STATUS_NO_USER:
			return "NoUser";
		case XN_CALIBRATION_STATUS_ARM:
			return "Arm";
		case XN_CALIBRATION_STATUS_LEG:
			return "Leg";
		case XN_CALIBRATION_STATUS_HEAD:
			return "Head";
		case XN_CALIBRATION_STATUS_TORSO:
			return "Torso";
		case XN_CALIBRATION_STATUS_TOP_FOV:
			return "Top FOV";
		case XN_CALIBRATION_STATUS_SIDE_FOV:
			return "Side FOV";
		case XN_CALIBRATION_STATUS_POSE:
			return "Pose";
		default:
			return "Unknown";
		}
	}
	const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
	{
		switch (error)
		{
		case XN_POSE_DETECTION_STATUS_OK:
			return "OK";
		case XN_POSE_DETECTION_STATUS_NO_USER:
			return "NoUser";
		case XN_POSE_DETECTION_STATUS_TOP_FOV:
			return "Top FOV";
		case XN_POSE_DETECTION_STATUS_SIDE_FOV:
			return "Side FOV";
		case XN_POSE_DETECTION_STATUS_ERROR:
			return "General error";
		default:
			return "Unknown";
		}
	}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	ROBOT_LOG( TRUE, "%d New User %d\n", epochTime, nId)
	// New user found
	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	ROBOT_LOG( TRUE, "***************** %d Lost user %d\n", epochTime, nId )
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	ROBOT_LOG( TRUE, "%d Pose %s detected for user %d\n", epochTime, strPose, nId )
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	ROBOT_LOG( TRUE, "%d Calibration started for user %d\n", epochTime, nId )
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (bSuccess)
	{
		// Calibration succeeded
		ROBOT_DISPLAY( TRUE, "%d Calibration complete, start tracking user %d\n", epochTime, nId )
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		ROBOT_LOG( TRUE, "%d Calibration failed for user %d\n", epochTime, nId )
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		// Calibration succeeded
		ROBOT_DISPLAY( TRUE, "%d Calibration complete, start tracking user %d\n", epochTime, nId )		
		ROBOT_LOG( TRUE, "Calibration complete, start tracking user %d\n", nId )
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		ROBOT_LOG( TRUE, "%d Calibration failed for user %d\n", epochTime, nId )
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

// Save calibration to file
void SaveCalibration()
{
	XnUserID aUserIDs[20] = {0};
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUserIDs, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		// Find a user who is already calibrated
		if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i]))
		{
			// Save user's calibration to file
			g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
			break;
		}
	}
}
// Load calibration from file
void LoadCalibration()
{
	XnUserID aUserIDs[20] = {0};
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUserIDs, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		// Find a user who isn't calibrated or currently in pose
		if (g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) continue;
		if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUserIDs[i])) continue;

		// Load user's calibration from file
		XnStatus rc = g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
		if (rc == XN_STATUS_OK)
		{
			// Make sure state is coherent
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
			g_UserGenerator.GetSkeletonCap().StartTracking(aUserIDs[i]);
		}
		break;
	}
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LabelPlayers( IplImage* pDisplayFrame )
{
	// Label Players that are identifed, and show current level of Tracking/Pose recognition

	if( !pDisplayFrame )
	{
		return; // probably shutting down...
	}

	#define STR_LABEL_SIZE 50
	CvFont font;
	CvPoint ptLocation;
	ptLocation.x = 0;
	ptLocation.y = 0;
	int line_type = CV_AA; // change it to 8 to see non-antialiased graphics
	cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, //CV_FONT_HERSHEY_COMPLEX, 
		2.0,		// Horz Scale
		2.0,		// Vert Scale
		0.0,		// Shear
		2,			// Thickness
		line_type ); 


	char strLabel[STR_LABEL_SIZE] = "";
	XnUserID UserIDArray[KINECT_MAX_HUMANS_TO_TRACK];
	XnUInt16 NumberOfUsers = KINECT_MAX_HUMANS_TO_TRACK; 
	// NumberOfUsers initially specifies max, but the next function changes this to number of users found
	g_UserGenerator.GetUsers(UserIDArray, NumberOfUsers);
	for (int nHuman = 0; nHuman < NumberOfUsers; ++nHuman) // TODO-MUST - CHECK OUT THIS ++ does it make first nHuman = 1?  <**************************************************************************
	{
		XnUserID nHumanID = UserIDArray[nHuman];
		ROBOT_ASSERT( nHumanID > 0 ); // The id Should always be non-zero

		if (g_bPrintID)
		{
			XnPoint3D Com3D, ComProj;
			g_UserGenerator.GetCoM(nHumanID, Com3D);
			g_DepthGenerator.ConvertRealWorldToProjective(1, &Com3D, &ComProj); // Count, RealWorld, Projective(out)

			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState) // use this if we only want to see a number, no state info
			{
				// Tracking
				sprintf_s( strLabel, STR_LABEL_SIZE, "%d", nHumanID);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsTracking(nHumanID))
			{
				// Tracking
				sprintf_s( strLabel, STR_LABEL_SIZE, "%d - Tracking", nHumanID );
			}
			else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(nHumanID))
			{
				// Calibrating
				sprintf_s( strLabel, STR_LABEL_SIZE, "%d - Calibrating [%s]", nHumanID, GetCalibrationErrorString(m_Errors[nHumanID].first));
			}
			else
			{
				// Nothing
				sprintf_s( strLabel, STR_LABEL_SIZE, "%d - Looking for pose [%s]", nHumanID, GetPoseErrorString(m_Errors[nHumanID].second));
			}

			if( (ComProj.X > 0) && (ComProj.Y > 0) )
			{
				ptLocation.x = (int)ComProj.X;
				ptLocation.y = (int)ComProj.Y;

				// Calculate Human position relative to Robot front.  
				// Convert from MM to TenthInches, and compensate for Kinect Position on the Robot relative to Front Bumper Center
				// Kinect Z => Distance from front of Robot (Our Y)
				// Kinect Y => Distance from ground (our Z).  X ==> -X
				int HumanCom_Y_TenthInches = (int)((Com3D.Z / (float)2.54) - KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES);
				int HumanCom_Z_TenthInches = (int)((Com3D.Y / 2.54) + KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES);
				int HumanCom_X_TenthInches = (int)(-Com3D.X / 2.54);
				// DEBUG: (overwrites normally displayed string)
				//sprintf_s( strLabel, STR_LABEL_SIZE, "Y=%d, X=%d, Z=%d", HumanCom_Y_TenthInches, HumanCom_X_TenthInches, HumanCom_Z_TenthInches );
				cvPutText( pDisplayFrame, strLabel, ptLocation, &font, CV_RGB(255,255,255) );

				// Add CenterOfMass info to Human Tracking
				HumanLocationTracking[nHuman].nHumanID = nHumanID;
				HumanLocationTracking[nHuman].CenterOfMass.X = HumanCom_X_TenthInches;
				HumanLocationTracking[nHuman].CenterOfMass.Y = HumanCom_Y_TenthInches;
				HumanLocationTracking[nHuman].CenterOfMass.Z = HumanCom_Z_TenthInches;

				#if ( TRACK_TOP_OF_USERS_HEAD == 1 )
					// Label the user's Head and save head position info
					if( 0 != HumanLocationTracking[nHumanID-1].HeadLocation.z )
					{
						ptLocation.y = 640 - (HumanLocationTracking[nHumanID-1].Pixel.Y - 50); // anchor label to top of head
						ptLocation.x = HumanLocationTracking[nHumanID-1].Pixel.X; // anchor label to top of head
						sprintf_s( strLabel, STR_LABEL_SIZE, "%d - Height: %3.1f", nHumanID, (float)(HumanLocationTracking[nHumanID-1].HeadLocation.z) / 10 ); // Note; value comes from Point Cloud in TenthInches
						cvPutText( pDisplayFrame, strLabel, ptLocation, &font, CV_RGB(255,255,255) );
					}
				#endif
			}
		}

		// Draw Skeletons of Players that are identifed

		if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(nHumanID))
		{
			//cvPutText( pDisplayFrame, "SKELETON!", ptLocation, &font2, CV_RGB(255,255,255) );

			DrawLimb(nHumanID, XN_SKEL_HEAD, XN_SKEL_NECK);

			DrawLimb(nHumanID, XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
			DrawLimb(nHumanID, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
			DrawLimb(nHumanID, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);

			DrawLimb(nHumanID, XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
			DrawLimb(nHumanID, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
			DrawLimb(nHumanID, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);

			DrawLimb(nHumanID, XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
			DrawLimb(nHumanID, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

			DrawLimb(nHumanID, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
			DrawLimb(nHumanID, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
			DrawLimb(nHumanID, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

			DrawLimb(nHumanID, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
			DrawLimb(nHumanID, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
			DrawLimb(nHumanID, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

			DrawLimb(nHumanID, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
		}
	}

	// Update Global human position info
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectHumanTrackingLock);
	EnterCriticalSection(&g_csKinectHumanTrackingLock);
	g_nHumansTracked = NumberOfUsers;
	for( int nHuman=0; nHuman < NumberOfUsers; nHuman++ )
	{
		XnUserID nHumanID = UserIDArray[nHuman];
		ROBOT_ASSERT( nHumanID > 0 ); // The id Should always be non-zero
		g_HumanLocationTracking[nHuman].nHumanID	   = HumanLocationTracking[nHuman].nHumanID; // Used to find the unique human in this array
		g_HumanLocationTracking[nHuman].CenterOfMass.X = HumanLocationTracking[nHuman].CenterOfMass.X; // in TenthInches
		g_HumanLocationTracking[nHuman].CenterOfMass.Y = HumanLocationTracking[nHuman].CenterOfMass.Y;
		g_HumanLocationTracking[nHuman].CenterOfMass.Z = HumanLocationTracking[nHuman].CenterOfMass.Z;
		g_HumanLocationTracking[nHuman].Pixel.X = HumanLocationTracking[nHuman].Pixel.X; // in Pixels
		g_HumanLocationTracking[nHuman].Pixel.Y = HumanLocationTracking[nHuman].Pixel.Y;
	#if ( TRACK_TOP_OF_USERS_HEAD == 1 )
		g_HumanLocationTracking[nHuman].HeadLocation.x = HumanLocationTracking[nHuman].HeadLocation.x; // in TenthInches
		g_HumanLocationTracking[nHuman].HeadLocation.y = HumanLocationTracking[nHuman].HeadLocation.y;
		g_HumanLocationTracking[nHuman].HeadLocation.z = HumanLocationTracking[nHuman].HeadLocation.z;
	#endif
	}
	LeaveCriticalSection(&g_csKinectHumanTrackingLock);
	__itt_task_end(pDomainKinectThread);


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RGBQUAD DepthValueToColor( UINT DepthValueMM , XnLabel nPlayer  )
{
	// Convert the supplied depth to a color value for showing depth in the image
	// DAVES: Uses my unique method for color depth visualization
    RGBQUAD Pixel;

	double ScaleFactor = 5;
    Pixel.rgbRed = Pixel.rgbBlue = Pixel.rgbGreen = 0;

	if( 0 != nPlayer )
	{
		// Use solid colors for human players!
		nPlayer = nPlayer % 6;
		switch( nPlayer )
		{
			case 1:
				Pixel.rgbRed = 255;
				break;
			case 2:
				Pixel.rgbGreen = 255;
				break;
			case 3:
				Pixel.rgbBlue = 255;
				break;
			case 4:
				Pixel.rgbRed = 127;
				Pixel.rgbGreen = 127;
				Pixel.rgbBlue = 0;
				break;
			case 5:
				Pixel.rgbRed = 127;
				Pixel.rgbGreen = 0;
				Pixel.rgbBlue = 127;
				break;
			case 6:
				Pixel.rgbRed = 0;
				Pixel.rgbGreen = 127;
				Pixel.rgbBlue = 127;
				break;
			default:		// handle 0 mod?
				Pixel.rgbRed = 100; 
				Pixel.rgbGreen = 64;
				Pixel.rgbBlue = 100;
				break;
		}

	}
	else if( 0 != DepthValueMM  )	
	{
		// Values must be scaled between 0 to 1023
		UINT red =0; UINT blue =0; UINT green =0; 
		signed int ScaledValue = (signed int) (((double)DepthValueMM / ScaleFactor) -90.0); // scale, and subtract out min range (47 ScaledValue)
		if( ScaledValue < 0) ScaledValue = 0;
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
	}

    return Pixel;
}

BOOL CheckRetVal( XnStatus nRetVal, const char* Message )
{
	if (nRetVal != XN_STATUS_OK)
	{
		ROBOT_DISPLAY( TRUE, "KINECT ERROR: %s: %s\n", Message, xnGetStatusString(nRetVal) )
		g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
		g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;
		__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
		return FALSE;
	}
	return TRUE;
}


#endif // USING OPENNI API

///////////////////////////////////////////////////////////////////////////////
// Performance Tuning
//#define FRAME_SKIP_COUNT			1	// how many frames to skip between each processing (Lower number increases CPU load) 
//#define CAMERA_BLUR_SETTLE_TIME	   10	// After moving head, allow motion blur to settle before processing

#define KINECT_FRAME_RATE_MS			30 //100 = 10fps, 200 = 5fps, 1000 = 1 fps
///////////////////////////////////////////////////////////////////////////////


// FACE TRACKING STATE MACHINE
#define STATE_IDLE							0
#define STATE_LOOK_FOR_FACE					1
#define STATE_FACE_FOUND					2 
#define STATE_CAMSHIFT_TRACKING				3

// Window Positions

#define WINDOW_TOP_OFFSET					50	// move down for Loki
#define WINDOW_BOARDER_SIZE					14
#define RAW_WINDOW_POSITION_Y				  0+WINDOW_TOP_OFFSET

// Use these values if you want to see the Raw window.
//   I just cover it up with the processed window to save space
//#define VIEW_RAW_WINDOW

	#define DEPTH_WINDOW_POSITION_Y_SMALL		158+WINDOW_TOP_OFFSET
	#define DEPTH_WINDOW_POSITION_Y_MED			320+WINDOW_TOP_OFFSET
	#define DEPTH_WINDOW_POSITION_Y_LARGE		480+WINDOW_TOP_OFFSET

	#define VIDEO_WINDOW_POSITION_Y_SMALL		158+DEPTH_WINDOW_POSITION_Y_SMALL
	#define VIDEO_WINDOW_POSITION_Y_MED			290+DEPTH_WINDOW_POSITION_Y_MED
	#define VIDEO_WINDOW_POSITION_Y_LARGE		480+DEPTH_WINDOW_POSITION_Y_LARGE




///////////////////////////////////////////////////////////////////////////////
//	Video Callback Functions
///////////////////////////////////////////////////////////////////////////////

void OnDepthWindowMouseEvent( int event, int x, int y, int flags, void* param )
{
	IGNORE_UNUSED_PARAM (flags);

	CKinectModule* pKinectObject = (CKinectModule*)param;

    if( !pKinectObject )
        return;


    if( CV_EVENT_LBUTTONDOWN == event )
	{
		ROBOT_LOG( TRUE,"OnDepthWindowMouseEvent: MouseDown at %d, %d\n", x, y)
		pKinectObject->m_DepthMousePoint = cvPoint(x,y);
        pKinectObject->m_DepthMousePointSelected = TRUE;
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
}



//************************************************************************************************************************
//************************************************************************************************************************
// Name: KinectThreadProc   MAIN LOOP FOR KINECT FRAME PROCESSING
// Desc: dedicated thread for grabbing Kinect video frames
// This thread created by the CKinectModule class
//************************************************************************************************************************
//************************************************************************************************************************

DWORD WINAPI KinectThreadProc( LPVOID lpParameter )
{
	IGNORE_UNUSED_PARAM (lpParameter);
	__itt_thread_set_name( "Kinect Thread" );


	BOOL bFrameGrabSucessLeft = FALSE;
	BOOL bFrameGrabSucessRight = FALSE;
	CString MsgString;
	CKinectModule* pKinectModule = 0;

#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
	XnStatus nRetVal = XN_STATUS_OK;
#endif


	char c = '\0';	// used to get key input from video frame
	DWORD dwFrameStartTime = 0;	// track how long each frame takes to process

	int KinectInitialized = FALSE;
	Sleep(500); // Let other modules init first

	while( g_bRunThread && g_bRunKinectThread )
	{
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectThreadLoop);

		if( !g_pKinectModule )
		{
			__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
			Sleep( 200 ); // Kinect Module not yet initialized
			continue;
		}

		// Get a pointer to the KinectModule Object
		pKinectModule = (CKinectModule*) g_pKinectModule;
		ROBOT_ASSERT( pKinectModule );

		dwFrameStartTime = GetTickCount();	// track how long each frame takes to process

		///////////////////////////////////////////////////////////////////////////////////
		// SEE IF CAMERAS NEED TO BE SHUT DOWN
		///////////////////////////////////////////////////////////////////////////////////

		if( (CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_DEPTH].State) ||
			(CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_VIDEO].State) )
		{
			// SHUTDOWN ONE OR BOTH KINECT CAMERAS
			if( ((CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_DEPTH].State) ||
				 (CAMERA_STATE_NOT_ENABLED ==		 g_Camera[KINECT_DEPTH].State))		&&						 
				((CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_VIDEO].State) ||
				 (CAMERA_STATE_NOT_ENABLED ==		 g_Camera[KINECT_VIDEO].State))		)
			{
				// Last camera being shut down.  Shutdown Kinect
				if( KinectInitialized )
				{
					/***
					if( !StopNUICamera(KinectCam) )
					{
						MsgString.Format( "\nKINECT FAIL: StopNUICamera error" );
						ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
					}
					if( !DestroyNUICamera(KinectCam) )
					{
						MsgString.Format( "\nKINECT FAIL: DestroyNUICamera error" );
						ROBOT_DISPLAY( TRUE, (LPCTSTR)MsgString )
					}
					***/
				}
			}

			if( CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_DEPTH].State )
			{
				pKinectModule->ReleaseKinectDepthWorkingImages();
				///cvReleaseImage( &pInputFrameKinectDepth );	// Safe to call without checking pointer
				cvDestroyWindow( CAMERA_WINDOW_NAME_KINECT_DEPTH );
				KinectInitialized = FALSE;
				g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
				g_KinectSubSystemStatus = SUBSYSTEM_DISABLED;
			}

			#if ( KINECT_VIDEO_ENABLED == 1 )
			if( CAMERA_STATE_SHUTDOWN_REQUESTED == g_Camera[KINECT_VIDEO].State )
			{
				pKinectModule->ReleaseKinectVideoWorkingImages();
				///cvReleaseImage( &pInputFrameKinectVideo );	// Safe to call without checking pointer
				cvDestroyWindow( CAMERA_WINDOW_NAME_KINECT_VIDEO );
				g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;
			}
			#endif
		}

		///////////////////////////////////////////////////////////////////////////////////
		// INITIALIZE NEW KINECT DEPTH AND IMAGE CAMERAS
		///////////////////////////////////////////////////////////////////////////////////

		if( (CAMERA_STATE_INIT_REQUESTED == g_Camera[KINECT_DEPTH].State) ||
			(CAMERA_STATE_INIT_REQUESTED == g_Camera[KINECT_VIDEO].State) )
		{
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectInit);
			CString strStatus;

			// Initialize Kinect if it has not already been initialized
			if ( !KinectInitialized )
			{
				// INITIALIZE KINECT
				#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
					// Microsoft Kinect SDK
					continue;
				#elif ( KINECT_SDK_TYPE == KINECT_MICROSOFT_1_0 )
					continue; 

				#else ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
					// OpenNI Interface

					nRetVal = XN_STATUS_OK;

					xn::EnumerationErrors errors;
					nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
					if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
					{
						XnChar strError[1024];
						errors.ToString(strError, 1024);
						ROBOT_DISPLAY( TRUE, "KINECT ERROR: InitFromXmlFile Error: %s\n", strError )
						g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
						g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;
						__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
						continue;
					}
					else if( !CheckRetVal(nRetVal, "SAMPLE_XML_PATH Open Failed") )	
						continue; // nRetVal != XN_STATUS_OK

					// DEPTH /////////////////////////////////////////////////
					nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
					if (nRetVal != XN_STATUS_OK)
					{
						ROBOT_DISPLAY( TRUE, "KINECT WARNING: No depth generator found. Using a default one...")
						xn::MockDepthGenerator mockDepth;
						nRetVal = mockDepth.Create(g_Context);

						if( !CheckRetVal(nRetVal, "Create mock depth failed") )	
							continue; // nRetVal != XN_STATUS_OK

						// set some defaults
						XnMapOutputMode defaultMode;
						defaultMode.nXRes = 640;
						defaultMode.nYRes = 480;
						defaultMode.nFPS = 30;
						nRetVal = mockDepth.SetMapOutputMode(defaultMode);
						if( !CheckRetVal(nRetVal, "Mock depth set default mode failed") )	
							continue; // nRetVal != XN_STATUS_OK

						// set FOV
						XnFieldOfView fov;
						fov.fHFOV = 1.0225999419141749;
						fov.fVFOV = 0.79661567681716894;
						nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
						if( !CheckRetVal(nRetVal, "set FOV") )	
							continue; // nRetVal != XN_STATUS_OK

						XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
						XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

						nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
						if( !CheckRetVal(nRetVal, "set empty depth map") )	
							continue; // nRetVal != XN_STATUS_OK

						g_DepthGenerator = mockDepth;
					}
					// VIDEO /////////////////////////////////////////////////
					#if (KINECT_VIDEO_ENABLED == 1 )
					nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
					if (nRetVal != XN_STATUS_OK)
					{
						ROBOT_DISPLAY( TRUE, "KINECT ERROR: No image generator. Check the XML File")
						ROBOT_ASSERT(0);
/***
						ROBOT_DISPLAY( TRUE, "KINECT WARNING: No image generator found. Using a default one...")
						xn::MockImageGenerator mockImage;
						nRetVal = mockImage.Create(g_Context);

						if( !CheckRetVal(nRetVal, "Create mock Image failed") )	
							continue; // nRetVal != XN_STATUS_OK

						// set some defaults
						XnMapOutputMode defaultMode;
						defaultMode.nXRes = 640;
						defaultMode.nYRes = 480;
						defaultMode.nFPS = 30;
						nRetVal = mockImage.SetMapOutputMode(defaultMode);
						if( !CheckRetVal(nRetVal, "Mock Image set default mode failed") )	
							continue; // nRetVal != XN_STATUS_OK

						// set FOV
						XnFieldOfView fov;
						fov.fHFOV = 1.0225999419141749;
						fov.fVFOV = 0.79661567681716894;
						nRetVal = mockImage.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
						if( !CheckRetVal(nRetVal, "set FOV") )	
							continue; // nRetVal != XN_STATUS_OK

						XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnImagePixel);
						XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

						nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
						if( !CheckRetVal(nRetVal, "set empty depth map") )	
							continue; // nRetVal != XN_STATUS_OK

						g_DepthGenerator = mockDepth;
**/
					}

					#endif


					nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
					if (nRetVal != XN_STATUS_OK)
					{
						nRetVal = g_UserGenerator.Create(g_Context);
						if( !CheckRetVal(nRetVal, "Find user generator") )	
							continue; // nRetVal != XN_STATUS_OK
					}

					XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
					if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
					{
						ROBOT_DISPLAY( TRUE, "KINECT ERROR: Supplied user generator doesn't support skeleton\n" )
						g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
						g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;
						g_KinectSubSystemStatus = SUBSYSTEM_FAILED;
						__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
						continue;
					}

					nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
					if( !CheckRetVal(nRetVal, "Register to user callbacks") )	
						continue; // nRetVal != XN_STATUS_OK

					nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
					if( !CheckRetVal(nRetVal, "Register to calibration start") )	
						continue; // nRetVal != XN_STATUS_OK

					nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
					if( !CheckRetVal(nRetVal, "Register to calibration complete") )	
						continue; // nRetVal != XN_STATUS_OK

					if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
					{
						g_bNeedPose = TRUE;
						if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
						{
							ROBOT_DISPLAY( TRUE, "KINECT ERROR: Pose required, but not supported\n" )
							g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
							g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;
							g_KinectSubSystemStatus = SUBSYSTEM_FAILED;
							__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
							continue;
						}
						nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
						if( !CheckRetVal(nRetVal, "Register to Pose Detected") )	
							continue; // nRetVal != XN_STATUS_OK

						g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
					}

					g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

					nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
					if( !CheckRetVal(nRetVal, "Register to calibration in progress") )	
						continue; // nRetVal != XN_STATUS_OK

					nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
					if( !CheckRetVal(nRetVal, "Register to pose in progress") )	
						continue; // nRetVal != XN_STATUS_OK

					nRetVal = g_Context.StartGeneratingAll();
					if( !CheckRetVal(nRetVal, "StartGenerating") )	
						continue; // nRetVal != XN_STATUS_OK

					KinectInitialized = TRUE;

				#endif // #if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
			}

			// KINECT VIDEO CAMERA
		#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
			#if (KINECT_VIDEO_ENABLED == 1 )
			if( CAMERA_STATE_INIT_REQUESTED == g_Camera[KINECT_VIDEO].State )
			{
				g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;		// in case of error initializing
				pKinectModule->m_DisplaySize.width = g_Camera[KINECT_DEPTH].DisplaySize.cx;
				pKinectModule->m_DisplaySize.height = g_Camera[KINECT_DEPTH].DisplaySize.cy;
				cvNamedWindow(CAMERA_WINDOW_NAME_KINECT_VIDEO, CV_WINDOW_AUTOSIZE);

				// Initialize working frames
				pKinectModule->InitKinectVideoWorkingImages( pKinectModule->m_CaptureSize, pKinectModule->m_DisplaySize );					
				if( !pKinectModule->m_pVideoFrame ) ROBOT_ASSERT(0); // Confirm that pKinectModule->m_pVideoFrame was initialized

				// Grab first video frame from Kinect	
				g_ImageGenerator.GetMetaData(g_imageMD);
				pKinectModule->GetVideoImage(); // Copy from Kinect frame to video input frame

				///////////////////////////////
				// Show first frame
				pKinectModule->MoveVideoWindow();
				pKinectModule->ShowVideoFrame( pKinectModule->m_DisplaySize );

				c = (char)cvWaitKey(1); // Kludge to assure CV draws the image
				g_Camera[KINECT_VIDEO].State = CAMERA_STATE_INITIALIZED;
				Sleep(KINECT_FRAME_RATE_MS);

				
				__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
				continue;

			}
			#endif

			// KINECT DEPTH CAMERA
			if( CAMERA_STATE_INIT_REQUESTED == g_Camera[KINECT_DEPTH].State )
			{
				g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;		// in case of error initializing

				pKinectModule->m_DisplaySize.width = g_Camera[KINECT_DEPTH].DisplaySize.cx;
				pKinectModule->m_DisplaySize.height = g_Camera[KINECT_DEPTH].DisplaySize.cy;
				cvNamedWindow(CAMERA_WINDOW_NAME_KINECT_DEPTH, CV_WINDOW_AUTOSIZE);
				cvSetMouseCallback( CAMERA_WINDOW_NAME_KINECT_DEPTH, OnDepthWindowMouseEvent, (void*)g_pKinectModule);

				// Initialize working frames
				pKinectModule->InitKinectDepthWorkingImages( pKinectModule->m_CaptureSize, pKinectModule->m_DisplaySize );
				if( !pKinectModule->m_pDepthFrame ) ROBOT_ASSERT(0);  // Confirm that pKinectModule->m_pDepthFrame was initialized

				g_Camera[KINECT_DEPTH].State = CAMERA_STATE_INITIALIZED;
				g_KinectSubSystemStatus = SUBSYSTEM_CONNECTED;

				pKinectModule->MoveDepthWindow();

				Sleep(KINECT_FRAME_RATE_MS);
				__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
				continue;
			}
		#endif // #if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )

			__itt_task_end(pDomainKinectThread);  // pshKinectInit
		}		// end of Vidcap Init


		///////////////////////////////////////////////////////////////////////////////////
		// Sleep if no cameras are enabled
		if(	(CAMERA_STATE_NOT_ENABLED == g_Camera[KINECT_DEPTH].State) &&
			(CAMERA_STATE_NOT_ENABLED == g_Camera[KINECT_VIDEO].State) )
		{			
			__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
			Sleep( 200 ); // Cameras not enabled
			continue;
		}

		///////////////////////////////////////////////////////////////////////////////////
		// NORMAL OPERATION
		///////////////////////////////////////////////////////////////////////////////////
		if( (CAMERA_STATE_INITIALIZED == g_Camera[KINECT_DEPTH].State) ||
			(CAMERA_STATE_INITIALIZED == g_Camera[KINECT_VIDEO].State) )
		{
			///TAL_SCOPED_TASK_NAMED("Kinect VidCap Grab");
			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectGetFrame);
			#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
				continue;
			#elif ( KINECT_SDK_TYPE == KINECT_MICROSOFT_1_0 )
				continue; 

			#else ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )

				// Do an general processing for both Video and Depth here (if any)


				////////////////////////////////////////////////////////////////////////////////
				// KINECT DEPTH
				if( CAMERA_STATE_INITIALIZED == g_Camera[KINECT_DEPTH].State )
				{
					// Get Depth Frame from raw data and convert to image and 3D Cloud

					__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectGetDepthImage);
					g_DepthGenerator.GetMetaData(pDepthMD);

					// Read next available data
					// Old:rc = g_context.WaitAnyUpdateAll();
					// Use this instead? XN_C_API XnStatus XN_C_DECL xnWaitNoneUpdateAll(XnContext* pContext);
					nRetVal = g_Context.WaitOneUpdateAll(g_UserGenerator);
					if( !CheckRetVal(nRetVal, "**** KINECT ERROR CAN'T GRAB FRAME FROM KINECT!") )
					{
						Sleep(500); // Keep this sleep, only occurs on error (so we don't spin in a tight loop)
						__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
						continue; // nRetVal != XN_STATUS_OK
					}

					// Process the data
					g_DepthGenerator.GetMetaData(pDepthMD); // get it again?
					g_UserGenerator.GetUserPixels(0, pSceneMD);



					//////////////////////////////////////////////////////////////////
					// Get Depth Frame from raw data and convert to image AND 3D CLOUD
					// Similar to DrawDepthMap(depthMD, sceneMD) in sample
					pKinectModule->GetDepthImage(); 
					__itt_task_end(pDomainKinectThread);

					// See if other processing of the image is needed

					// Find objects on floor, if enabled by other modules
					pKinectModule->FindObjectsOnFloor();

				}
			
				#if ( KINECT_VIDEO_ENABLED == 1 )
				////////////////////////////////////////////////////////////////////////////////
				// KINECT VIDEO
				if( CAMERA_STATE_INITIALIZED == g_Camera[KINECT_VIDEO].State )
				{
					// Copy from Kinect frame to video input frame
					__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectGetVideoImage);

					/* nRetVal = g_Context.WaitAnyUpdateAll();
					if( !CheckRetVal(nRetVal, "**** KINECT ERROR CAN'T GRAB FRAME FROM KINECT VIDEO!") )
					{
						Sleep(500); // Keep this sleep, only occurs on error (so we don't spin in a tight loop)
						__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop
						continue; // nRetVal != XN_STATUS_OK
					}
					*/

					// Process the data
					g_ImageGenerator.GetMetaData(g_imageMD);
					pKinectModule->GetVideoImage();
					__itt_task_end(pDomainKinectThread);
				}
				#endif

			#endif // ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
		}	// Normal Operation
			
		/////////////////////////////////////////////////////////////////////////////////
		// Show Kinect Video frames 
		if( g_bRunKinectThread ) // check in case we are shutting down
		{
			///TAL_SCOPED_TASK_NAMED("Kinect VidCap Show");

			__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectShowFrames);
			pKinectModule->ShowDepthFrame( pKinectModule->m_DisplaySize );
			pKinectModule->ShowVideoFrame( pKinectModule->m_DisplaySize );
			pKinectModule->m_DepthMousePointSelected = FALSE;
			__itt_task_end(pDomainKinectThread);
		}

		// Calculate how long the frame took to process and display results
		DWORD dwFrameProcessingTime = GetTickCount() - dwFrameStartTime;

		// KLUDGE!  REQURED DUE TO BUG IN OPENCV.		
		char c = (char)cvWaitKey(1); // NEED cvWaitKey() OR VIDEO DOES NOT DISPLAY!
		if( c == 27 )	ROBOT_LOG( TRUE,"ESC KEY Pressed!\n")
		
		__itt_task_end(pDomainKinectThread);  // pshKinectThreadLoop

		// Sleep for up to KINECT_FRAME_RATE_MS.  Subtract out processing time
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinectSleep);
		int SleepTime = KINECT_FRAME_RATE_MS - dwFrameProcessingTime;
		if( SleepTime > 0 )
		{
			// Only sleep if we are not busy processing.
			Sleep( SleepTime );
		}
		else
		{
			SleepTime = 0;
		}
		__itt_task_end(pDomainKinectThread); // Sleep

		//g_SensorStatus.VideoFPS = 1000.0 / (dwFrameProcessingTime+SleepTime);
		//g_SensorStatus.KinectFPS = 1000.0 / (dwFrameProcessingTime+SleepTime);


	}

	//////////////////////////////////////////////////////////////////////////////////////
	ROBOT_LOG( TRUE,"Kinect Thread exiting.\n")

	if( CAMERA_STATE_INITIALIZED == g_Camera[KINECT_DEPTH].State )
	{
#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
		g_scriptNode.Release();
		g_DepthGenerator.Release();
		g_UserGenerator.Release();
		g_Player.Release();
		g_Context.Release();
#endif

		pKinectModule->ReleaseKinectDepthWorkingImages();
		//cvReleaseImage( &pInputFrameKinectDepth );	// Safe to call without checking pointer
		cvDestroyWindow( CAMERA_WINDOW_NAME_KINECT_DEPTH );
		KinectInitialized = FALSE;
		g_Camera[KINECT_DEPTH].State = CAMERA_STATE_NOT_ENABLED;
	}

	#if ( KINECT_VIDEO_ENABLED == 1 )
	if( CAMERA_STATE_INITIALIZED == g_Camera[KINECT_VIDEO].State )
	{
		pKinectModule->ReleaseKinectVideoWorkingImages();
		//cvReleaseImage( &pInputFrameKinectVideo );	// Safe to call without checking pointer
		cvDestroyWindow( CAMERA_WINDOW_NAME_KINECT_VIDEO );
		g_Camera[KINECT_VIDEO].State = CAMERA_STATE_NOT_ENABLED;

	}
	#endif

	if( KinectInitialized )
	{
		//	DestroyNUICamera(KinectCam);
	}
	return 0;
}



///////////////////////////////////////////////////////////////////////////////
//	MODULE: CKinectModule
//  NOTE: Creates its own video capture thread
///////////////////////////////////////////////////////////////////////////////


 CKinectModule::CKinectModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_CurrentTask = KINECT_TASK_HUMAN_DETECTION; // by default, start looking for Humans
	m_TaskState = 1;
	m_TrackObjectX = 0;
	m_TrackObjectY = 0;
	gKinectDelayTimer = 0;
	m_FindObjectsOnFloorRequest = 0;
	m_nKinect3DObjectsFound = -1; // -1 == Result not ready
	m_KinectScanPosition = 0;
	m_FindCloseObjectsOnly = FALSE;

	m_FrameNumber = 0;
	m_pDepthFrame = 0;
	m_pVideoFrame = 0;
	m_pDepthDisplayFrame = 0;
	m_pVideoDisplayFrame = 0;
	m_CaptureSize.width = KINECT_CAPTURE_SIZE_X;	// Defaults
	m_CaptureSize.height = KINECT_CAPTURE_SIZE_Y;
	m_DisplaySize.width = KINECT_WINDOW_DISPLAY_SIZE_X;
	m_DisplaySize.height = KINECT_WINDOW_DISPLAY_SIZE_Y;
	m_pKinectServoControl = new KinectServoControl;
	m_pHeadControl = new HeadControl();

	// Detectors
	m_pTrackObjectDetector	= NULL;

	m_ServoMoving = FALSE;
	m_BlurSettleTime = 0;

	// Detector Enabling flags
	m_VidCapProcessingEnabled = FALSE;
	m_ObjectTrackingEnabled = FALSE;
	m_ObjectTrackingActive = FALSE;
	m_ObjectToTrackFound = FALSE;

	// Processing variables
	m_ObjectTrackSize.width = 0;
	m_ObjectTrackSize.height = 0;
	m_ObjectTrackingCenter.x = 0;
	m_ObjectTrackingCenter.y = 0;

	m_DepthMousePoint.x = 0;
	m_DepthMousePoint.y = 0;
	m_DepthMousePointSelected = FALSE;
//	m_LastCameraMoveTime = GetTickCount();

	// Create the Point Cloud

	__itt_task_begin(pDomainControlThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
	//ROBOT_LOG( TRUE,"Kinect Init: EnterCriticalSection g_csKinectPointCloudLock\n")
	EnterCriticalSection(&g_csKinectPointCloudLock);

	g_KinectPointCloud = new KINECT_3D_CLOUD_T;
	g_KinectPointCloud->dwTimeOfSample = 0;
	g_KinectPointCloud->RobotLocation.x = 0.0;	// Location of robot at the time of depth snapshot
	g_KinectPointCloud->RobotLocation.y = 0.0;
	g_KinectPointCloud->CompassHeading = 0;		// Heading of robot at the time of depth snapshot
	for( int i=0; i < KINECT_CAPTURE_SIZE_Y; i++ )
	{
		for( int j=0; j < KINECT_CAPTURE_SIZE_X; j++ )
		{
			g_KinectPointCloud->Point3dArray[i][j].X = 0;
			g_KinectPointCloud->Point3dArray[i][j].Y = 0;
			g_KinectPointCloud->Point3dArray[i][j].Z = 0;
		}
	}
	for( int i=0; i < KINECT_CAPTURE_SIZE_X; i++ )
	{
		g_KinectPointCloud->WallPoints[i].X = LASER_RANGEFINDER_TENTH_INCHES_MAX;
		g_KinectPointCloud->WallPoints[i].Y = LASER_RANGEFINDER_TENTH_INCHES_MAX;
	}

	LeaveCriticalSection(&g_csKinectPointCloudLock);
	__itt_task_end(pDomainControlThread);
	//ROBOT_LOG( TRUE,"Kinect Init: LeaveCriticalSection g_csKinectPointCloudLock\n")

	m_pKinectTempObjects3D = new TEMP_OBJECT_3D_ARRAY_T;
	memset( m_pKinectTempObjects3D, 0x00, sizeof( TEMP_OBJECT_3D_ARRAY_T ) );


	// FOR DEBUG ONLY - KLUDGE
	g_pKinectDebugSliceArray = new DEBUG_SLICE_ARRAY_T;
	memset( g_pKinectDebugSliceArray, 0x00, sizeof( DEBUG_SLICE_ARRAY_T ) );


	// Create the Kinect video capture thread
	g_bRunKinectThread = TRUE; // When FALSE, tells Vidcap thread to exit
	DWORD dwTempThreadId;
	g_hKinectThread = ::CreateThread( NULL, 0, KinectThreadProc, (LPVOID)0, 0, &dwTempThreadId );
	ROBOT_LOG( TRUE, "Created Kinect Thread. ID = (0x%x)\n", dwTempThreadId )

}

///////////////////////////////////////////////////////////////////////////////
CKinectModule::~CKinectModule()
{
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule.  Waiting for Kinect Thread to exit...\n")
	g_bRunKinectThread = FALSE; // Tell the thread to exit
	if( INVALID_HANDLE_VALUE != g_hKinectThread) 
	{
		WaitForSingleObject( g_hKinectThread, INFINITE );
		CloseHandle( g_hKinectThread );
	}
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule.  Kinect Thread exit complete.\n")
	SAFE_DELETE( g_pKinectDebugSliceArray );
	SAFE_DELETE( m_pKinectTempObjects3D );
	SAFE_DELETE( g_KinectPointCloud );
	SAFE_DELETE( m_pKinectServoControl );
	SAFE_DELETE( m_pHeadControl );
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CKinectModule done.\n")
}

///////////////////////////////////////////////////////////////////////////////
// Kinect Depth Camera
void CKinectModule::InitKinectDepthWorkingImages( CvSize FrameSize, CvSize DisplaySize )
{
	// On first frame, allocate "Working" frames of the correct size

	m_pDepthFrame = cvCreateImage( FrameSize, IPL_DEPTH_8U, 3 );
	//m_pDepthFrame = cvCreateImage( FrameSize, 8, 3 );
	if( !m_pDepthFrame ) ROBOT_ASSERT(0);
	m_pDepthFrame->origin = 0;

	m_pDepthDisplayFrame = cvCreateImage( DisplaySize, 8, 3 );
	if( !m_pDepthDisplayFrame ) ROBOT_ASSERT(0);
	m_pDepthDisplayFrame->origin = 0;

}

void CKinectModule::ShowDepthFrame( CvSize DisplaySize )
{	
	if( !m_pDepthFrame )
	{
		return; // probably shutting down
	}
	// For DEBUG, draw the slices detected
	ROBOT_ASSERT( g_pKinectDebugSliceArray );
	ROBOT_ASSERT( g_pKinectDebugSliceArray->nSlices < MAX_DEBUG_SLICES );
	for( int SliceItem =0; SliceItem < g_pKinectDebugSliceArray->nSlices; SliceItem++ )
	{
		cvLine(m_pDepthFrame, g_pKinectDebugSliceArray->Slice[SliceItem].Pt1,  g_pKinectDebugSliceArray->Slice[SliceItem].Pt2, CV_RGB(0,127,127), 1, 4, 0);
	}
	
	// Draw bounding boxes around any objects detected
	if( g_pKinectObjects3D->nObjectsDetected > 0)
	{
		//int i = 0;
		for( int i = 0; i < g_pKinectObjects3D->nObjectsDetected; i++ )
		{
			CvPoint pt1 = { (g_pKinectObjects3D->Object[i].LeftPixel), (g_pKinectObjects3D->Object[i].StartScanLine) };
			CvPoint pt2 = { (g_pKinectObjects3D->Object[i].RightPixel), (g_pKinectObjects3D->Object[i].EndScanLine) };
			cvRectangle(m_pDepthFrame, pt1, pt2, CV_RGB(255,0,255), 1 );
		}
	}

	// Label Players that are identifed, and show current level of Tracking/Pose recognition
	// Includig Skeletons of Players that are identifed
#if ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
	LabelPlayers( m_pDepthFrame );
#endif

	// KINECT_WALL_SCAN_ZONE
	CvPoint pt1, pt2;
	pt1.y = KINECT_WALL_SCAN_ZONE;
	pt1.x = 0;
	pt2.y = KINECT_WALL_SCAN_ZONE;
	pt2.x = m_pDepthFrame->width;
	cvLine(m_pDepthFrame, pt1, pt2, CV_RGB(0,127,0), 1, 4, 0);

	// Draw Cross-Hair lines
	CvPoint FrameCenter = GetFrameCenter(m_pDepthFrame);

	// Vertical line
	pt1.x = FrameCenter.x;
	pt1.y = 0;
	pt2.x = FrameCenter.x;
	pt2.y = m_pDepthFrame->height;
	cvLine(m_pDepthFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);

	// Horizontal line
	pt1.y = FrameCenter.y;
	pt1.x = 0;
	pt2.y = FrameCenter.y;
	pt2.x = m_pDepthFrame->width;
	cvLine(m_pDepthFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);


	// Scale as needed
	if( m_CaptureSize.width != m_DisplaySize.width )
	{
		// Scale down 640x480 to fit on display (easier on remote desktop too)
		cvResize( m_pDepthFrame, m_pDepthDisplayFrame  ); // CV_INTER_CUBIC
		//	cvResizeWindow( CAMERA_WINDOW_NAME_LEFT, 320, 240 );
	}
	else
	{
		cvCopy( m_pDepthFrame, m_pDepthDisplayFrame, 0 );
	}

	if( g_Camera[KINECT_DEPTH].Flip )
	{
		// Flip video horizontally
		cvFlip( m_pDepthDisplayFrame, 0, 1 );
	}

	cvShowImage( CAMERA_WINDOW_NAME_KINECT_DEPTH, m_pDepthDisplayFrame );

}

void CKinectModule::ReleaseKinectDepthWorkingImages()
{
	// These are safe to call without validating the pointers
    cvReleaseImage( &m_pDepthDisplayFrame );
    cvReleaseImage( &m_pDepthFrame );
}


//////////////////////////////////////////
// GetDepthImage
// Copy image to something viewable, look for Users, and also convert to 3 point cloud
// See "DrawDepthMap" from the OPENNI demo

__itt_string_handle* pshKinect3DCloud = __itt_string_handle_create("3D Cloud"); //marker

void CKinectModule::GetDepthImage()
	// USES: pSceneMD, pDepthMD - TODO make these non-global?
{
	// TODO? Lock the 3DPointCloud

	// Convert depth data to colors and map to Depth frame
	// Starts at top of frame (y=0) and scans to bottom of frame (y=frameheight)
	// So, top of frame is farthest from the robot!
	#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
		return;
	#elif ( KINECT_SDK_TYPE == KINECT_MICROSOFT_1_0 )
		return; 

	#else ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )

		const XnDepthPixel* pDepthRow = pDepthMD.Data();
		const XnLabel* pLabels = pSceneMD.Data(); /// comes from User Generator

		CvScalar	Pixel;
		UINT FrameWidth = pDepthMD.XRes();
		UINT FrameHeight = pDepthMD.YRes();

		// Calculate degrees per pixel for the image
		double DegreeIncrementX = KINECT_DEPTH_FOV_X / (double)FrameWidth;
		double DegreeIncrementY = KINECT_DEPTH_FOV_Y / (double)FrameHeight;

		// Get Kinect Tilt angle, and position offset caused by tilt
		double KinectTiltTenthDegrees =	g_BulkServoStatus[DYNA_KINECT_SCANNER_SERVO_ID].PositionTenthDegrees - 10.0;	// Add a small fudge factor
		//double HeightAboveFloor = KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES;
		//double DistanceFromFront = KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES; 

		// Reset all human viewed player data at the start of each frame
		memset( HumanLocationTracking, 0x00, (sizeof(KINECT_HUMAN_TRACKING_T) * KINECT_MAX_HUMANS_TO_TRACK) );


		UINT MouseX = 0;
		UINT MouseY = 0;
		UINT nHumanIndex = 0; // Index into the Human table used for tracking person height

		if( NULL == g_KinectPointCloud )
		{
			ROBOT_ASSERT(0); // Not initilized!
			return;
		}

		if( m_DepthMousePointSelected )
		{
			// Scale as needed.  Mouse coordinates are always 640x480 after scaling
			UINT Ratio = m_CaptureSize.width / m_DisplaySize.width;
			if( g_Camera[KINECT_DEPTH].Flip )
			{
				// Video flipped horizontally
				MouseX = ( (m_DisplaySize.width -1) - m_DepthMousePoint.x ) * Ratio;
			}
			else
			{
				MouseX = m_DepthMousePoint.x * Ratio;
			}
			MouseY = m_DepthMousePoint.y * Ratio;
		}

		double ScaleFactor = 5.0;

		/** Optional
		// Determine max range for scale
		int MaxDepthMM = 0;
		for (XnUInt y = 0; y < FrameHeight; ++y)
		{
			const XnDepthPixel* pDepth = pDepthRow;
			for (XnUInt x = 0; x < FrameWidth; ++x, ++pDepth)
			{
				int DepthValueMM = 0;	// Depth value in millimeters
				if (*pDepth != 0)
				{
					DepthValueMM = *pDepth;
				}

				if( (DepthValueMM > MaxDepthMM) && (DepthValueMM < KINECT_MM_MAX) )
				{
					MaxDepthMM = DepthValueMM;	// keep largest value
				}
			}
			pDepthRow += pDepthMD.XRes();
		}
		// Determine Scale factor
		ScaleFactor = (double)MaxDepthMM / 900.0;
		if( ScaleFactor > 5.0 ) ScaleFactor = 5.0;
		**/
		

		pDepthRow = pDepthMD.Data(); // Reset it back to the start of the buffer (after optional range test)
		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshKinect3DCloud);
		for( XnUInt y = 0; y < FrameHeight; y++ )
		{
			const XnDepthPixel* pDepth = pDepthRow;
			for( XnUInt x = 0; x < FrameWidth; x++, pDepth++, pLabels++ )
			{
				USHORT DepthValueMM = 0;	// Depth value in millimeters
				if (*pDepth != 0)
				{
					DepthValueMM = *pDepth;
				}

				//////////////////////////////////////////////////////////////////////////////////////////
				// Update the 3D point cloud
				// Calculate X,Y,Z based upon tilt of Kinect and FOV of the Kinect Camera
				// x, y = pixel location
				double X=0,Y=0,Z=0,TempX=0,TempY=0,TempZ=0,m=0,n=0,q=0,u=0, RangeTenthInches = 0;

				if( m_DepthMousePointSelected )
				{
					if( (x == MouseX) && (y == MouseY) )
					{
						ROBOT_LOG( TRUE,"MOUSE DOWN  ")
					}
				}

				if( (DepthValueMM >= KINECT_MM_MAX) || (DepthValueMM <= 0) )
				{
					//ROBOT_LOG( TRUE,"Kinect Read Zero: EnterCriticalSection g_csKinectPointCloudLock\n")
					__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
					EnterCriticalSection(&g_csKinectPointCloudLock);
						g_KinectPointCloud->Point3dArray[y][x].X = KINECT_RANGE_TENTH_INCHES_MAX;
						g_KinectPointCloud->Point3dArray[y][x].Y = KINECT_RANGE_TENTH_INCHES_MAX;
						g_KinectPointCloud->Point3dArray[y][x].Z = KINECT_RANGE_TENTH_INCHES_MAX;
					LeaveCriticalSection(&g_csKinectPointCloudLock);
					__itt_task_end(pDomainKinectThread);
					//ROBOT_LOG( TRUE,"Kinect Read Zero: LeaveCriticalSection g_csKinectPointCloudLock\n")
				}
				else
				{
					RangeTenthInches = ((double)DepthValueMM / 2.540) ;	

					double StepX = (double)x - ((double)FrameWidth/2.0);	// goes from (-320 to + 320)
					double StepAngleX = DegreeIncrementX / 1.5 * StepX; // goes from (-46 to + 46 degrees)

					double StepY = ( ((double)FrameHeight/2.0) - (double)y );	// goes from (+240 to -240)
					//double StepAngleY =  DegreeIncrementY * StepY; // goes from (-38.6 to + 38.6 degrees)
					double StepAngleY =  (KinectTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.6 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
					double StepAngleYZ =  (KinectTiltTenthDegrees/10.0) + (DegreeIncrementY / 1.8 * StepY); // goes from (-38.6 to + 38.6 degrees) + Tilt angle
					//double StepAngleY =  (KinectTiltTenthDegrees/10.0); // goes from (-38.6 to + 38.6 degrees) + Tilt angle

					// Get x,y,z from Kinect's perspective. zero = straight forward facing.  Down = negative
					// Convert from Spherical Coordinates to X,Y,Z coordinates for each point
					TempX = RangeTenthInches * sin(DEGREES_TO_RADIANS*(StepAngleX) );// TODO! THIS SEEMS WRONG!!!
					TempY = RangeTenthInches * cos(DEGREES_TO_RADIANS*StepAngleY);// * cos(DEGREES_TO_RADIANS*(StepAngleX));
					TempZ = RangeTenthInches * sin(DEGREES_TO_RADIANS*StepAngleYZ * 1.1); // * cos(DEGREES_TO_RADIANS*(StepAngleX));

					// Now, translate origin from Kinect position to center of robot at floor.
					X = -TempX;	 // For Loki, Positive X is the Right side of the robot
					Y = TempY - (double)KINECT_DISTANCE_FROM_FRONT_TENTH_INCHES;
					Z = TempZ + (double)KINECT_HEIGHT_ABOVE_GROUND_TENTH_INCHES;


					// Save the data in TenthInches
					//ROBOT_LOG( TRUE,"Kinect Set: EnterCriticalSection g_csKinectPointCloudLock\n")
					__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
					if( NULL == g_KinectPointCloud ) return; // TODO Kludge for shutdown crash.  Need better solution.
					EnterCriticalSection(&g_csKinectPointCloudLock);
						g_KinectPointCloud->Point3dArray[y][x].X = (int)X;
						g_KinectPointCloud->Point3dArray[y][x].Y = (int)Y;
						g_KinectPointCloud->Point3dArray[y][x].Z = (int)Z;
					LeaveCriticalSection(&g_csKinectPointCloudLock);
					__itt_task_end(pDomainKinectThread);
					//ROBOT_LOG( TRUE,"Kinect Set: LeaveCriticalSection g_csKinectPointCloudLock\n")

					#if ( TRACK_TOP_OF_USERS_HEAD == 1 )
						// *** TODO IN THE FUTURE, TRACK TOP OF USERS HEAD.
						// Try this:  See if we can move the code to determine number of users ahead of this routine, so we know the number of users and can populate the table below correctly
						// Keep track of the highest point on the player detected in this frame (used with range data to determine player's height!)
						// This will be combined with other data later, when we know how many unique humans there are (we don't know at this point)
						if( 0 != nHumanID )
						{
							int Pixel_Y = 640 - y; // y=0 is top of screen
							if( Pixel_Y > HumanLocationTracking[nHumanID-1].Pixel.Y )
							{
								HumanLocationTracking[nHumanIndex].nHumanID = nHumanID;
								HumanLocationTracking[nHumanIndex].Pixel.Y = Pixel_Y;
								HumanLocationTracking[nHumanIndex].Pixel.X = x;
								HumanLocationTracking[nHumanIndex].HeadLocation.x = (int)X;   // Note these come from PointCloud calculation
								HumanLocationTracking[nHumanIndex].HeadLocation.y = (int)Y;	  // So values are in TENTH INCHES!
								HumanLocationTracking[nHumanIndex].HeadLocation.z = (int)Z;
							}
						}
					#endif
				}		

				//////////////////////////////////////////////////////////////////////////////////////////
				// Determine color of Pixel. Either a depth color or solid color related to a Human Player
				XnLabel nHumanID = *pLabels;

				RGBQUAD ColorPixel = DepthValueToColor( DepthValueMM, nHumanID );
				// For DEBUG, you can use Z Value View:
				// RGBQUAD ColorPixel = DepthValueToColor( (USHORT)Z, nHumanID );

				// TODO - see if we can remove this HACK
				Pixel.val[0] = ColorPixel.rgbBlue;		// Blue
				Pixel.val[1] = ColorPixel.rgbGreen;		// Green
				Pixel.val[2] = ColorPixel.rgbRed;		// Red
				cvSet2D( m_pDepthFrame, y, x, Pixel );

				// Check Mouse input
				if( m_DepthMousePointSelected )
				{
					if( (x == MouseX) && (y == MouseY) )
					{
						ROBOT_LOG( TRUE,"MOUSE: Y=%d, X=%d:   Range=%4.1f, Y=%4.1f, X=%4.1f, Z=%4.1f\n", y, x,(RangeTenthInches/10.0), Y/10.0, X/10.0, Z/10.0  )
						m_DepthMousePointSelected = FALSE;
					}
				}

			}
			pDepthRow += pDepthMD.XRes();
		}

		// Update 2D map with wall and object locations
		FindWallsAnd2dMaps();

		// Update global summary of objects seen for object avoidance
		UpdateKinectObjectSummary();

		__itt_task_end(pDomainKinectThread);
	#endif // ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )
}

///////////////////////////////////////////////////////////////////////////////
// Kinect Video Camera
void CKinectModule::InitKinectVideoWorkingImages( CvSize FrameSize, CvSize DisplaySize )
{
	// On first frame, allocate "Working" frames of the correct size
	m_pVideoFrame = cvCreateImage( FrameSize, IPL_DEPTH_8U, 3 );
	//m_pVideoFrame = cvCreateImage( FrameSize, 8, 3 );
	if( !m_pVideoFrame ) ROBOT_ASSERT(0);
	m_pVideoFrame->origin = 0;

	m_pVideoDisplayFrame = cvCreateImage( DisplaySize, 8, 3 );
	if( !m_pVideoDisplayFrame ) ROBOT_ASSERT(0);
	m_pVideoDisplayFrame->origin = 0;
}

void CKinectModule::ShowVideoFrame( CvSize DisplaySize )
{
	#if ( KINECT_VIDEO_ENABLED == 1 )


	if( m_CaptureSize.width != m_DisplaySize.width )
	{
		// Scale down 640x480 to fit on display (easier on remote desktop too)
		cvResize( m_pVideoFrame, m_pVideoDisplayFrame  ); // CV_INTER_CUBIC
		//	cvResizeWindow( CAMERA_WINDOW_NAME_LEFT, 320, 240 );
	}
	else
	{
		cvCopy( m_pVideoFrame, m_pVideoDisplayFrame, 0 );
	}

	// Draw Cross-Hair lines
	CvPoint pt1, pt2;
	CvPoint FrameCenter = GetFrameCenter(m_pVideoDisplayFrame);

	// Vertical line
	pt1.x = FrameCenter.x + 5;	// Offset to match Depth Frame
	pt1.y = 0;
	pt2.x = FrameCenter.x + 5;	// Offset to match Depth Frame
	pt2.y = m_pVideoDisplayFrame->height;
	cvLine(m_pVideoDisplayFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);

	// Horizontal line
	pt1.y = FrameCenter.y + 20;	// Offset to match Depth Frame
	pt1.x = 0;
	pt2.y = FrameCenter.y + 20;	// Offset to match Depth Frame
	pt2.x = m_pVideoDisplayFrame->width;
	cvLine(m_pVideoDisplayFrame, pt1, pt2, CV_RGB(0,255,0), 1, 4, 0);

	if( g_Camera[KINECT_VIDEO].Flip )
	{
		// Flip video horizontally
		cvFlip( m_pVideoDisplayFrame, 0, 1 );
	}
	cvShowImage( CAMERA_WINDOW_NAME_KINECT_VIDEO, m_pVideoDisplayFrame );
	#endif

}

void CKinectModule::ReleaseKinectVideoWorkingImages()
{
	// These are safe to call without validating the pointers
    cvReleaseImage( &m_pVideoDisplayFrame );
    cvReleaseImage( &m_pVideoFrame );
}


//////////////////////////////////////////
void CKinectModule::GetVideoImage()
{
#if ( KINECT_VIDEO_ENABLED == 1 )

	if( !g_bRunKinectThread ) // check in case we are shutting down
		return;

	#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
		return;
	#elif ( KINECT_SDK_TYPE == KINECT_MICROSOFT_1_0 )
		return; 

	#else ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )

		CvScalar Pixel;
		const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();

		for (XnUInt y = 0; y < g_imageMD.YRes(); ++y)
		{
			if( !g_bRunKinectThread ) // check in case we are shutting down
				return;
			const XnRGB24Pixel* pImage = pImageRow;
			//XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

			for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage) //, ++pTex)
			{
				//*pTex = *pImage;
				// convert RGB to BGR
				Pixel.val[0] = pImage->nBlue;	// Blue
				Pixel.val[1] = pImage->nGreen;	// Blue
				Pixel.val[2] = pImage->nRed;	// Blue
				cvSet2D( m_pVideoFrame, y, x, Pixel );
			}
			pImageRow += g_imageMD.XRes();
			//pTexRow += g_nTexMapX;
		}
	#endif	// ( KINECT_SDK_TYPE == KINECT_OPEN_SOURCE )

#endif

}



///////////////////////////////////////////////////////////////////////////////
//	Video Helper Functions
///////////////////////////////////////////////////////////////////////////////

int CKinectModule::WindowPosX( IplImage* pInputFrame )
{
	int DisplayWidth = GetSystemMetrics(SM_CXSCREEN);
	int DisplayHeight = GetSystemMetrics(SM_CYSCREEN);


//	CvSize FrameSize = cvGetSize(pInputFrame);	// LAPTOP_DISPLAY_SIZE_WIDTH
	int PosX =  DisplayWidth - (pInputFrame->width + WINDOW_BOARDER_SIZE);
	return PosX;
}

///////////////////////////////////////////////////////////////////////////////
void CKinectModule::CopyImage( IplImage* pInputFrame, IplImage* pOutputFrame )
{

	// Not used

	ASSERT(0);

	// Copy frame to the "working" image, inverting it if needed
	//cvCopy( pInputFrame, pOutputFrame, 0 );
	int FlipMode = -1;

	//cvFlip( pInputFrame, pOutputFrame, FlipMode );

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


			///////////////////////////WM_ROBOT_SENSOR_STATUS_READYWM_ROBOT_SENSOR_STATUS_READY//////////////////////////////////////////////////////////////
			// Check status of Kinect Tasks
			if( 0 != gKinectDelayTimer )
			{
				//ROBOT_LOG( TRUE,"WAITING FOR gKinectDelayTimer = %d\n", gKinectDelayTimer)
				break;
			}

			switch( m_CurrentTask )
			{
				case OBJECT_TASK_NONE:
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
							//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - PIC\n")
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
							for( int nHuman=0; nHuman < g_nHumansTracked; nHuman++ )
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
							//if( WM_ROBOT_SENSOR_STATUS_READY == uMsg ) ROBOT_LOG( TRUE," - PIC\n")
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
							FindObjectsOnFloorRequest( KINECT_FIND_OBJECTS_REQUEST_ONCE );	// Just look once
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
							FindObjectsOnFloorRequest( KINECT_FIND_OBJECTS_REQUEST_ONCE );	// Just look once
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
										// Note - Top of frame = 0, Bottom of frame = 480 
										int FrameCenterY = m_CaptureSize.height / 2;
										int ObjectFrameCenterY =  g_pKinectObjects3D->Object[0].EndScanLine + ( (g_pKinectObjects3D->Object[0].StartScanLine - g_pKinectObjects3D->Object[0].EndScanLine) / 2 );	// StartScanLine is the bigger number 
										if( ObjectFrameCenterY > (FrameCenterY + (m_CaptureSize.height / 5)) )	
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



void CKinectModule::MoveDepthWindow()
{

	// Move frames based upon frame size
	// Get Width/Height of laptop screen
	//SIZE ScreenSize;
	//ZeroMemory( &s, sizeof(SIZE) );
	//ScreenSize.cx = (LONG)::GetSystemMetrics( SM_CXFULLSCREEN );
	//ScreenSize.cy = (LONG)::GetSystemMetrics( SM_CYFULLSCREEN );



	// Handle DEPTH CAMERA
	if( m_pDepthFrame )
	{
		int PositionY = 0;
		CvSize FrameSizeDepth = cvGetSize(m_pDepthDisplayFrame);
		if( 0 == FrameSizeDepth.width ) ROBOT_ASSERT(0);

		if( 120 == FrameSizeDepth.height )
		{	
			PositionY = DEPTH_WINDOW_POSITION_Y_SMALL; // 160x120
		}
		else if( 240 == FrameSizeDepth.height )
		{
			PositionY = DEPTH_WINDOW_POSITION_Y_MED; // 320x240
		}
		else if( 480 == FrameSizeDepth.height )
		{			
			PositionY = DEPTH_WINDOW_POSITION_Y_LARGE; // 640x480
		}
		else
		{
			ROBOT_ASSERT(0);
		}

		cvMoveWindow( CAMERA_WINDOW_NAME_KINECT_DEPTH, WindowPosX(m_pDepthDisplayFrame), PositionY );
	}
}

void CKinectModule::MoveVideoWindow()
{

	// Handle VIDEO CAMERA
	if( m_pVideoFrame )
	{
		int PositionY = 0;
		CvSize FrameSizeVideo = cvGetSize(m_pVideoDisplayFrame);
		if( 0 == FrameSizeVideo.width ) ROBOT_ASSERT(0);

		if( 120 == FrameSizeVideo.height )
		{	
			PositionY = VIDEO_WINDOW_POSITION_Y_SMALL; // 160x120
		}
		else if( 240 == FrameSizeVideo.height )
		{
			PositionY = VIDEO_WINDOW_POSITION_Y_MED; // 320x240
		}
		else if( 480 == FrameSizeVideo.height )
		{			
			PositionY = VIDEO_WINDOW_POSITION_Y_LARGE; // 640x480
		}
		else
		{
			ROBOT_ASSERT(0);
		}

		cvMoveWindow( CAMERA_WINDOW_NAME_KINECT_VIDEO, WindowPosX(m_pVideoDisplayFrame), PositionY );
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

	for( int HScan = 0; HScan < KINECT_CAPTURE_SIZE_X; HScan++)
	{
		g_KinectPointCloud->WallPoints[HScan].X = g_KinectPointCloud->WallPoints[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_MAX; // infinate range
		g_KinectPointCloud->MapPoints2D[HScan].X = g_KinectPointCloud->MapPoints2D[HScan].Y = LASER_RANGEFINDER_TENTH_INCHES_MAX;

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
		for( int LeadPos = (KINECT_CAPTURE_SIZE_Y-(KINECT_WALL_DETECT_GAP_SIZE+1)); LeadPos >0; LeadPos-- )
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
		}

		// Done with vertical scan line. Update summary data
		if( LeadingEdgeFound )
		{
			// MapPoints contains any blocking object 
			g_KinectPointCloud->MapPoints2D[HScan].X = g_KinectPointCloud->Point3dArray[LeadPos][HScan].X;
			g_KinectPointCloud->MapPoints2D[HScan].Y = LeadingEdgeY;
			//g_KinectPointCloud->MapPoints2D[HScan].Z = ObjPeakHeight - FloorValue;
			//if( (g_KinectPointCloud->MapPoints2D[HScan].X > -60) && (g_KinectPointCloud->MapPoints2D[HScan].X < 60) )
			//	TRACE(" Y=%d, X=%d, PeakHeight=%d\n",  g_KinectPointCloud->MapPoints2D[HScan].X, g_KinectPointCloud->MapPoints2D[HScan].Y, ObjPeakHeight );


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
	for( int HScan = 0; HScan < KINECT_CAPTURE_SIZE_X; HScan++)
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
	m_KinectSummary.RobotLocation.x = g_SensorStatus.CurrentLocation.x;		// Location of robot at the time of the laser scan
	m_KinectSummary.RobotLocation.y = g_SensorStatus.CurrentLocation.y;		
	m_KinectSummary.CompassHeading = g_SensorStatus.CompassHeading;		// Heading of robot at the time of the laser scan

	// Now find the minimum Y value for each zone
	for( int i = 0; i < KINECT_CAPTURE_SIZE_X; i++ ) 
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
#define DEBUG_KINECT_SHOW_2D_OBJECTS_FOUND 0
#define DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME 1
#define DEBUG_KINECT_SHOW_3D_SLICE_OBJECTS_FOUND 0
#define KINECT_BAD_PIXELS_MAX  20

void CKinectModule::FindObjectsInSingleScanLine( int  ScanLine, int NumberOfSamples, OBJECT_2D_ARRAY_T* pKinectObjects2D )
{

#define KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM	   1
#define LEFT_SAMPLES_TO_AVE						   1
#define RIGHT_SAMPLES_TO_AVE					   1
#define GAP_SIZE								  10	// Pixels!  For Kinect this is 640 / ~16" = 40 pixels/inch

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

	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: Kinect EnterCriticalSection g_csKinectPointCloudLock\n")
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, psh_csKinectPointCloudLock);
	EnterCriticalSection(&g_csKinectPointCloudLock);


	/////////////////////////////////////////////////////
	// Now find objects
	// This code looks for a step in the Z direction, by looking for 
	// N floor samples -- Gap -- N object samples
	// Note ALL VALUES IN TENTH INCHES unless otherwise indicated

	int		BigObjStartIndex = 0;
	int		BigObjPeakHeight = 0;
	BOOL	BigObjLeadingEdgeFound = FALSE;
	BOOL	BigObjTrailingEdgeFound = FALSE;

	int		SmallObjStartIndex = 0;
	int		SmallObjPeakHeight = 0;
//	int 	nDetectedObjects = 0;
	BOOL	LeadingEdgeFound = FALSE;
	BOOL	TrailingEdgeFound = FALSE;

	double	LeftSum = 0;
	int		nLeftSamples = 0;
	double	RightSum = 0;
	int		nRightSamples = 0;
	int		PeakDetectOffsetIndex = 0;
	double	RightFloorValue = 0;
	double	LeftFloorValue = 0;

//	ROBOT_LOG( TRUE,"Min = %d, Ave = %d, Max = %d\n", MinValue[ScanLine], AveValue[ScanLine], MaxValue[ScanLine])
	//ROBOT_LOG( TRUE,"%02d %03d:O                          ", ScanLine, Position)	// print line number and tilt position

	// Scan Data:  0=Right to NumberOfSamples=Left
	for(int i = KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM; i < (NumberOfSamples-KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM); i++)
	{
		RightSum = 0.0;
		nRightSamples = 0;
		LeftSum = 0.0;
		nLeftSamples = 0;
		double RightAve = 0;
		double LeftAve = 0;

		for( int j=0; j<KINECT_BAD_PIXELS_MAX; j++ )
		{
			if( (i+j) >= (NumberOfSamples-KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM) )
			{
				break; // don't look at more than "n" pixels, or go off the edge of the frame
			}
			if( g_KinectPointCloud->Point3dArray[ScanLine][i+j].Z < KINECT_RANGE_TENTH_INCHES_MAX )
			{
				// Valid Pixel from deph sensor
				RightSum += (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+j].Z);
				if( ++nRightSamples >= RIGHT_SAMPLES_TO_AVE )
				{
					break;
				}
			}
		}


		for( int j=0; j<KINECT_BAD_PIXELS_MAX; j++ )
		{
			if( (i+j+(RIGHT_SAMPLES_TO_AVE + GAP_SIZE)) >= (NumberOfSamples-KINECT_SCAN_LOOK_FOR_OBJECT_EDGE_TRIM) )
			{
				break; // don't look at more than "n" pixels, or go off the edge of the frame
			}
			if( g_KinectPointCloud->Point3dArray[ScanLine][i+j+(RIGHT_SAMPLES_TO_AVE + GAP_SIZE)].Z < KINECT_RANGE_TENTH_INCHES_MAX )
			{
				// Valid Pixel from deph sensor
				LeftSum += (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+j+(RIGHT_SAMPLES_TO_AVE + GAP_SIZE)].Z);
				if( ++nLeftSamples >= LEFT_SAMPLES_TO_AVE )
				{
					break;
				}
			}
		}


		if( 0 != nRightSamples) RightAve = RightSum / (double)nRightSamples;
		if( 0 != nLeftSamples) LeftAve =  LeftSum /  (double)nLeftSamples;
		double HeightDelta = LeftAve - RightAve;
		double AbsHeightDelta = abs(HeightDelta); // tenth inches
		if( (abs(HeightDelta) >= KINECT_SLICE_MIN_OBJECT_HEIGHT_TENTH_INCHES) &&				// detect changes between average floor and object
			( (RightAve < KINECT_MAX_FLOOR_HEIGHT_TENTH_INCHES) || (LeftAve < KINECT_MAX_FLOOR_HEIGHT_TENTH_INCHES) )	 )	// Make sure one of the two values is close to the floor
		{
			// Some Object detected!
			if( !LeadingEdgeFound && (HeightDelta > 0) )
			{
				// Start of a new object (positive slope)
				SmallObjStartIndex = i + RIGHT_SAMPLES_TO_AVE + GAP_SIZE;
				RightFloorValue = RightAve;
				LeadingEdgeFound = TRUE;
				//g_KinectPointCloud->Point3dArray[ScanLine][i+j].Z
				//ROBOT_LOG( TRUE,"Leading Edge Found X = %3.1f\n", (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0)
			}
			else if( LeadingEdgeFound && !TrailingEdgeFound && (HeightDelta < 0)  )
			{
				// found start of trailing edge (negative slope)
				//ROBOT_LOG( TRUE,"Trailing Edge Found NEAR X = %3.1f\n", (double)(g_KinectPointCloud->Point3dArray[ScanLine][i+(RIGHT_SAMPLES_TO_AVE+1)].X) / 10.0 )
				TrailingEdgeFound = TRUE;
			}

			PeakDetectOffsetIndex = i + nRightSamples + GAP_SIZE + 1;
			if( LeadingEdgeFound && !TrailingEdgeFound )
			{
				if( (g_KinectPointCloud->Point3dArray[ScanLine][PeakDetectOffsetIndex].Z < KINECT_RANGE_TENTH_INCHES_MAX) &&
					(g_KinectPointCloud->Point3dArray[ScanLine][PeakDetectOffsetIndex].Z > SmallObjPeakHeight) )
				{
					SmallObjPeakHeight = g_KinectPointCloud->Point3dArray[ScanLine][PeakDetectOffsetIndex].Z;	// keep track of peak height
				}
			}
		}
		else
		{
			// object not detected, or no longer detected (close out the object detection)
			if( LeadingEdgeFound && TrailingEdgeFound )
			{
				// We were tracking a trailing edge, and it fell below minimum height.  End of object found!
				LeftFloorValue = LeftAve;
				double AveFloor = (RightFloorValue + LeftFloorValue) / 2.0;
				double PeakHeight = ((double)(SmallObjPeakHeight) - AveFloor);

				int StartX =  (g_KinectPointCloud->Point3dArray[ScanLine][SmallObjStartIndex].X)  ;
				int StartY =  (g_KinectPointCloud->Point3dArray[ScanLine][SmallObjStartIndex].Y) ;
				int StartZ = ((g_KinectPointCloud->Point3dArray[ScanLine][SmallObjStartIndex].Z) - (int)AveFloor) ; //- (FloorAverage * 0.66);// subtract out noise floor
				int EndX = (g_KinectPointCloud->Point3dArray[ScanLine][i].X) ;
				int EndY = (g_KinectPointCloud->Point3dArray[ScanLine][i].Y) ;
				int EndZ = ((g_KinectPointCloud->Point3dArray[ScanLine][i].Z) - (int)AveFloor) ; // - (FloorAverage * 0.66);// subtract out noise floor;
				int Width = abs( (StartX - EndX) );
				int AverageY = (StartY + EndY) / 2;
				int AverageX = (StartX + EndX) /2;


				if( (PeakHeight < KINECT_SLICE_OBJECT_HEIGHT_MAX) && (PeakHeight > KINECT_SLICE_OBJECT_HEIGHT_MIN) && 
					(Width < KINECT_SLICE_OBJECT_WIDTH_MAX) && (Width > KINECT_SLICE_OBJECT_WIDTH_MIN) )	// Skip big or tiny objects
				{
					// Save Object info (in TENTH INCHES)!
					// Summary info
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].X = AverageX;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Y = AverageY;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].PeakHeight = (int)PeakHeight;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].Width = Width;
					// Details
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartX = StartX;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartY = StartY;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].StartZ = StartZ;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndX = EndX;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndY = EndY;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].EndZ = EndZ;
					pKinectObjects2D->Object[pKinectObjects2D->nObjectsDetected].LeftPixel = SmallObjStartIndex;
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

					if( pKinectObjects2D->nObjectsDetected++ > KINECT_SCAN_MAX_2D_OBJECTS )
					{
						ROBOT_LOG( TRUE, "ERROR!  Kinect Scanner LookForObjects nDetectedObjects > KINECT_SCAN_MAX_OBJECTS!  ABORTING further object search!\n")
						ROBOT_ASSERT(0);
						break;
					}
				}
				SmallObjPeakHeight = 0;
				LeadingEdgeFound = FALSE;
				TrailingEdgeFound = FALSE;
			}
		}
	}
	//ROBOT_LOG( TRUE,"\n")

	LeaveCriticalSection(&g_csKinectPointCloudLock);
	__itt_task_end(pDomainKinectThread);
	//ROBOT_LOG( TRUE,"FindObjectsInSingleScanLine: Kinect LeaveCriticalSection g_csKinectPointCloudLock\n")
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RequestFindObjectsOnFloor
// queues up a request to the Kinect Frame Grap thread to find 3D objects on the floor at the current tilt position
void CKinectModule::FindObjectsOnFloorRequest( UINT Request )
{
	m_nKinect3DObjectsFound = -1; // indicate to the caller that the request is pending
	m_FindObjectsOnFloorRequest = Request;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FindObjectsOnFloor
// Parse scan lines from Kinect and find 3D objects on the floor
// Returns the number of objects detected
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OBJECT_CENTER_TOLLERANCE 30 // TenthInches - Max allowed distance between centers of subsequent scan lines (X+Y)
//#define OBJECT_CENTER_Y_WEIGHT   5 // multiplier -  Tunes for how much more important Y is than X when matching found objects

__itt_string_handle* pshSingleScanLine = __itt_string_handle_create("Single Scan Line");

void CKinectModule::FindObjectsOnFloor()
{
	ROBOT_ASSERT( g_pKinectDebugSliceArray );
	g_pKinectDebugSliceArray->nSlices = 0;

	if( KINECT_FIND_OBJECTS_REQUEST_STOP == m_FindObjectsOnFloorRequest )
	{
		// Don't look for any objects
		m_nKinect3DObjectsFound = 0;
		return;
	}
	else if( KINECT_FIND_OBJECTS_REQUEST_ONCE == m_FindObjectsOnFloorRequest )
	{
		// Look only once for any objects
		m_FindObjectsOnFloorRequest = KINECT_FIND_OBJECTS_REQUEST_STOP;
	}
	else if( KINECT_FIND_OBJECTS_REQUEST_CONTINUOUS != m_FindObjectsOnFloorRequest )
	{
		// bad parameter
		ROBOT_ASSERT(0);
	}
	__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshFindObjectsOnFloor);

	// Reset the number of 3D objects found
	m_pKinectTempObjects3D->nObjectsDetected = 0;
	g_pKinectObjects3D->nObjectsDetected = 0;


	// process the Kinect POINT CLOUD data, from closest to furthest
	// Go thorugh each row of POINT CLOUD data and identify potential objects then compare these rows
	// to find true 3D objects. (Note this is not the raw depth data from the sensor, it is XYZ values)
	
	// First, look for walls ( Assumes FindWallsAnd2dMaps() was called

	// Uses POINT3D_T	Point3dArray[KINECT_CAPTURE_SIZE_X][KINECT_CAPTURE_SIZE_Y];

	// Walk through each scan line, closest to robot first
	ROBOT_LOG( TRUE,"FindObjectsOnFloor: Looking for 3D Objects\n")
	OBJECT_2D_ARRAY_T* pKinectObjects2D = new OBJECT_2D_ARRAY_T;
	
	for( int ScanLine = KINECT_CAPTURE_SIZE_Y-1; ScanLine >=0; ScanLine-- ) // Start at BOTTOM of frame
	{
		//ROBOT_LOG( TRUE,"DEBUG: ScanLine = %d\n", ScanLine)
		memset( pKinectObjects2D, 0x00, sizeof( OBJECT_2D_ARRAY_T ) );

		__itt_task_begin(pDomainKinectThread, __itt_null, __itt_null, pshSingleScanLine);
		FindObjectsInSingleScanLine( ScanLine, KINECT_CAPTURE_SIZE_X, pKinectObjects2D ); // returns pKinectObjects2D
		__itt_task_end(pDomainKinectThread);

		// Now iterate through the list of 2D objects found on this line, and associate with any current 3D objects
		if( pKinectObjects2D->nObjectsDetected > 0 )
		{
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
					if( m_pKinectTempObjects3D->nObjectsDetected > KINECT_SCAN_MAX_3D_OBJECTS )
					{
						ROBOT_LOG( TRUE,"WARNING: KinectMultiLineScanFindObjects3D: m_pKinectTempObjects3D->nObjectsDetected > KINECT_SCAN_MAX_3D_OBJECTS. Ignoring far objects\n")
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
					#if DEBUG_KINECT_SHOW_3D_SLICES_ON_FRAME == 1
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
		} // if
	}


	// Now, copy objects found to global, if they pass the size test
	// AND not too close to a wall (as defined by the Laser Range Finder)
//	int ClosestObjectDistance = KINECT_RANGE_TENTH_INCHES_MAX;
//	int ClosestObjectIndex = 0;
#define WALL_EDGE_ZONE_SIZE	30 // TenthInches - Objects within this distane to wall/large objects are not tracked by Kinect

	if( m_pKinectTempObjects3D->nObjectsDetected > 0)
	{
		ROBOT_LOG( TRUE, "\n*************************\n")
		ROBOT_LOG( TRUE, "FINAL 3D Objects Found:\n", g_pKinectObjects3D->nObjectsDetected )
		g_pKinectObjects3D->nObjectsDetected = 0;
		g_pKinectObjects3D->nClosestObjectIndex = 0;
		g_pKinectObjects3D->nClosestObjectDistance = KINECT_RANGE_TENTH_INCHES_MAX;

		for( int i = 0; i < m_pKinectTempObjects3D->nObjectsDetected; i++ )
		{
			// Test size of object
			if( m_pKinectTempObjects3D->Object[i].Length < MIN_3D_OBJECT_LENGTH)		// minimim size of object to detect
			{
				ROBOT_LOG( TRUE, "Object %d Too Small: Length = %d\n", i, m_pKinectTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Width < MIN_3D_OBJECT_WIDTH)			// minimim size of object to detect
			{
				ROBOT_LOG( TRUE, "Object %d Too Small: Width = %d\n", i, m_pKinectTempObjects3D->Object[i].Width )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Length > MAX_3D_OBJECT_LENGTH)		// max size of object to detect
			{
				ROBOT_LOG( TRUE, "Object %d Too Big: Length = %d\n", i, m_pKinectTempObjects3D->Object[i].Length )
				continue;
			}
			if( m_pKinectTempObjects3D->Object[i].Width > MAX_3D_OBJECT_WIDTH)			// max size of object to detect
			{
				ROBOT_LOG( TRUE, "Object %d Too Big: Width = %d\n", i, m_pKinectTempObjects3D->Object[i].Width )
				continue;
			}
			if( abs(m_pKinectTempObjects3D->Object[i].CenterX) > MAX_3D_OBJECT_X_POSITION )	// Too far to one side
			{
				ROBOT_LOG( TRUE, "Object %d Too Far off center: CenterX = %d\n", i, m_pKinectTempObjects3D->Object[i].CenterX )
				continue;
			}

			// Passes size test.  See if it's too close to a wall (might be a bogus object, furniture, etc.)
			// NOTE: Assumes FindWallsAnd2dMaps() was called first!

			int RoughDistance = LASER_RANGEFINDER_TENTH_INCHES_MAX;
			for( int HScan = 0; HScan < KINECT_CAPTURE_SIZE_X; HScan++)
			{
				if( g_KinectPointCloud->WallPoints[HScan].Y < LASER_RANGEFINDER_TENTH_INCHES_MAX )
				{

					int DeltaX = abs( g_KinectPointCloud->WallPoints[HScan].X - m_pKinectTempObjects3D->Object[i].CenterX );
					int DeltaY = abs( g_KinectPointCloud->WallPoints[HScan].Y - m_pKinectTempObjects3D->Object[i].CenterY );
					RoughDistance = (DeltaX + DeltaY) / 2;
					//ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Point X=%4d, Y=%4d,     DELTA X=%4d, Y=%4d  DIST = %d\n", nSample, WallPointX, WallPointY, DeltaX, DeltaY, Distance )

					if( RoughDistance < WALL_EDGE_ZONE_SIZE )
					{
						// too close to wall
						ROBOT_LOG( TRUE, "DEBUG: Point=%4d   Wall Y=%4d, X=%4d,     DELTA Y=%4d, X=%4d  DIST = %d\n", HScan,  
							g_KinectPointCloud->WallPoints[HScan].Y, g_KinectPointCloud->WallPoints[HScan].X, DeltaY, DeltaX, RoughDistance )
						ROBOT_LOG( TRUE, "Kinect Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to Wall (%4.1f in), skipping\n", 
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
			if ( m_pKinectTempObjects3D->Object[i].EndScanLine <  KINECT_WALL_SCAN_ZONE )
			{
				// too close to top of frame
				ROBOT_LOG( TRUE, "DEBUG: ObjStartScanLine =%4d   ObjEndScanLine X=%4d, ZONE=%4d,\n",  
					m_pKinectTempObjects3D->Object[i].StartScanLine, m_pKinectTempObjects3D->Object[i].EndScanLine, KINECT_WALL_SCAN_ZONE )

				ROBOT_LOG( TRUE, "Kinect Object Y=%4.1f X=%4.1f Height=%4.1f   Too close to top of frame, skipping\n", 
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
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].CenterY = m_pKinectTempObjects3D->Object[i].CenterY -10;	// TenthInches - fudge becase it always seems to miss the first inch of the object
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Height = m_pKinectTempObjects3D->Object[i].Height;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Width = m_pKinectTempObjects3D->Object[i].Width;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].Length = m_pKinectTempObjects3D->Object[i].Length +10;		// TenthInches - fudge becase it always seems to miss the first inch of the object

			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].LeftPixel = m_pKinectTempObjects3D->Object[i].LeftPixel;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].RightPixel = m_pKinectTempObjects3D->Object[i].RightPixel;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].StartScanLine = m_pKinectTempObjects3D->Object[i].StartScanLine;
			g_pKinectObjects3D->Object[g_pKinectObjects3D->nObjectsDetected].EndScanLine = m_pKinectTempObjects3D->Object[i].EndScanLine;


			// For debug, print out the 3D objects found
			ROBOT_LOG( TRUE, "Object %2d:  Y=%4.1f X=%4.1f,   Height=%4.1f  Width=%4.1f,  Length=%4.1f, BB = (%d,%d) (%d,%d)\n", g_pKinectObjects3D->nObjectsDetected,
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


			if( g_pKinectObjects3D->nObjectsDetected > KINECT_SCAN_MAX_3D_OBJECTS )
			{
				ROBOT_LOG( TRUE,"\nWARNING: 3D Objects found > KINECT_SCAN_MAX_3D_OBJECTS. Ignoring far objects")
				break;
			}
		}
		ROBOT_LOG( TRUE, "Found %d 3D Objects\n", g_pKinectObjects3D->nObjectsDetected )
		ROBOT_LOG( TRUE, "*************************\n\n")
	}
	else
	{
		ROBOT_LOG( TRUE, "KinectMultiLineScanFindObjects3D:  No 3D Objects Found in Image.\n" )
	}

	m_nKinect3DObjectsFound = g_pKinectObjects3D->nObjectsDetected;
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
	if( SIMULATED_SIO_HANDLE == g_hPicCommPort )
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
		gKinectOwnerTimer = 50;	// 1/10 seconds
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
		gKinectCurrentOwner = HEAD_OWNER_NONE;
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
