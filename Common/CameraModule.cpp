// CameraModule.cpp: CCameraModule class implementation
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ClientOrServer.h"
#if ( WILLNEVERHAPPEN )	// This module used for Robot Server only
//#if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only

#include <math.h>
//#include <MMSystem.h>	// For Sound functions
#include "Globals.h"
#include "module.h"
#include "thread.h"
#include "videoInput.h" 
#include "cv.h"
//#include "cvcam.h"
#include "highgui.h"
#include "RobotSharedParams.h"
//#include <CLNUIDevice.h>	// Kinect Sensor

// For Kinect OpenNI library
//#include <XnOS.h>
//#include <math.h>
//#include <XnCppWrapper.h>
//using namespace xn;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



///////////////////////////////////////////////////////////////////////////////
//	MODULE: CameraModule
///////////////////////////////////////////////////////////////////////////////


CCameraModule::CCameraModule( CDriveControlModule *pDriveControlModule )
{
	m_pDriveCtrl = pDriveControlModule;
	m_pHeadControl = new HeadControl();	// For controlling head servos

	m_FrameNumber = 0;
	m_pVideoFrameLeft = 0;		// Working copy of current LEFT video frame
	m_pVideoFrameRight = 0;		// Working copy of current RIGHT video frame
	m_pKinectDepthFrame = 0;
	m_pKinectVideoFrame = 0;
	m_pMotionFrame = 0;			// for displaying motion view
	m_pColorBlobFrame = 0;		// for displaying color blob view
	m_pDisplayFrameLeft = 0;	// Scaled Frame to display
	m_pDisplayFrameRight = 0;	// Scaled Frame to display
	m_pDisplayFrameKinectDepth = 0;
	m_pDisplayFrameKinectVideo = 0;


//	m_nCameraPanTiltCmd = 0;	// User commands
//	m_nUserCameraPanTiltSpeed = 0;
//	g_CameraPanPos = CAMERA_PAN_TENTHDEGREES_REST_POSITION;
//	g_CameraTiltPos = CAMERA_TILT_TENTHDEGREES_REST_POSITION;
//	g_CameraSideTiltPos = CAMERA_SIDETILT_TENTHDEGREES_REST_POSITION;
//	g_CameraNeckPos = CAMERA_NECK_TENTHDEGREES_REST_POSITION;
	g_CameraZoomPos	= MIN_ZOOM_LEVEL;
	g_CameraServoTorqueEnabled = FALSE;

	m_ShowPropDialog = FALSE;
	m_ShowFormatDialog = FALSE;

	// Detectors
	m_pFaceDetector			= NULL;
	m_pMotionDetector		= NULL;
	m_pCamShiftDetector		= NULL;
	m_pColorDetector		= NULL;
	m_pLaserSpotDetector	= NULL;
	m_pLaserLineDetector	= NULL;
	m_pStereoDetector		= NULL;
	m_pMatchObjectDetector	= NULL;
	m_pTrackObjectDetector	= NULL;


//	m_pConeDetector		= NULL;
//	m_pObjectDetector	= NULL;

	m_HeadMoving = FALSE;
	m_BlurSettleTime = 0;
	m_MotionDetectorTime = 0;
	m_StereoDetectorTime = 0;
	m_FaceLostTime = 0;
	m_FramesSinceMotionLost = 100;


	m_LastMotionCenter.x = 0;
	m_LastMotionCenter.y = 0;
	m_IdleBehavior = BEHAVIOR_FRIENDLY;

	// Detector Enabling flags
	m_VidCapProcessingEnabled = FALSE;
	m_FaceTrackingEnabled = FALSE;
	m_ColorTrackingEnabled = FALSE;		// Initialize the color tracking object
	m_ColorTrackingActive = FALSE;		// Actively track colors
	m_CamShiftTrackingEnabled = FALSE;
	m_ConeTrackingEnabled = FALSE;		// for tracking Cone SHAPE

	m_MotionTrackingEnabled = FALSE;
	m_IRTrackingEnabled = FALSE;
	m_MotionTrackingViewEnabled = FALSE;
	m_LaserSpotTrackingEnabled = FALSE;	
	m_LaserLineTrackingEnabled = FALSE;	

	m_ObjectTrackingEnabled = FALSE;
	m_ObjectTrackingActive = FALSE;
	m_ObjectToTrackFound = FALSE;

	m_StereoEnabled = FALSE;	
	m_HandTrackingEnabled = FALSE;
	m_TakeSnapshot = FALSE;


	// Processing variables
	m_FaceTrackingFrameCount = 0;
	m_CamShiftCenter.x = 0;
	m_CamShiftCenter.y = 0;
	m_FaceTrackCenter.x = 0;
	m_FaceTrackCenter.y = 0;
	m_FaceTrackSize.width = 0;
	m_FaceTrackSize.height = 0;
	m_MotionCenter.x = 0;
	m_MotionCenter.y = 0;
	m_ObjectTrackingCenter.x = 0;
	m_ObjectTrackingCenter.y = 0;
	m_ColorBlobCenter.x = 0;
	m_ColorBlobCenter.y = 0;
	m_LaserSpot.x = 0;
	m_LaserSpot.y = 0;
	m_IRTrackCenter.x = 0;
	m_IRTrackCenter.y = 0;
	m_PIR_Position.x = 0;
	m_PIR_Position.y = 0;
//	m_HandTrackPosition.x = 0;
//	m_HandTrackPosition.y = 0;

    memset( &m_CamShiftBox, 0, sizeof(CvBox2D) );

	m_FaceTrackingState = STATE_LOOK_FOR_FACE;
	ROBOT_LOG( TRUE,"m_FaceTrackingState = STATE_LOOK_FOR_FACE\n")
	m_CamShiftState = STATE_IDLE;
	ROBOT_LOG( TRUE,"m_CamShiftState = STATE_IDLE\n")


	// Color Blob Tracking
	m_ColorScanningEnabled = FALSE;
	m_ColorSearchingState = STATE_IDLE;	// If not Idle, looking for a color, but one not found yet
	m_FrameRetryCount = 0;
	m_ServoDelayTime = SERVO_DELAY_DISABLED;	// target system tic count, in ms. 0=disabled
	m_nPScanPosLeft = CAMERA_PAN_CENTER;
	m_nPScanPosRight = CAMERA_PAN_CENTER;

	m_VideoMousePoint.x = 0;
	m_VideoMousePoint.y = 0;
	m_VideoMousePointSelected = FALSE;
	//m_RandomHeadCounter = 0;
	g_LastCameraMoveTime = GetTickCount();

	// Ask the GUI to send the last saved values for Camera settings
//	SendResponse( WM_ROBOT_GET_CAMERA_SETTINGS, 0, 0 );

	// Create the video capture thread
	g_bRunVidCapThread = TRUE; // When FALSE, tells Vidcap thread to exit
	DWORD dwTempThreadId;
	g_hCameraVidCapThread = ::CreateThread( NULL, 0, VidCapThreadProc, (LPVOID)0, 0, &dwTempThreadId );
	ROBOT_LOG( TRUE, "Created VidCap Thread. ID = (0x%x)", dwTempThreadId )


}

///////////////////////////////////////////////////////////////////////////////
CCameraModule::~CCameraModule()
{
	SAFE_DELETE(m_pHeadControl);
	ROBOT_LOG( TRUE,"SHUT DOWN: ~CCameraModule done.\n")
}


#define PLAYER_LOCK_ON_ANGLE_MAX		100 // Tenth Degrees
///////////////////////////////////////////////////////////////////////////////
void CCameraModule::ProcessMessage(
		UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	if( ROBOT_TYPE == TURTLE)
	{
		// No "Head" on turtle, all video done by Kinect
		// TODO - Enable this someday if we use OpenCV on Turtle?
		g_bCmdRecognized = TRUE;
		return;
	}

	CString MsgString;
	//BYTE nDistHigh, nDistLow;
	switch( uMsg )
	{

		// Camera VidCap Commands
		case WM_ROBOT_CAMERA_ENABLE_FEATURE:
		{
			g_bCmdRecognized = TRUE;
			return;
		}
		break;


		// Process Camera commands from the GUI or command threads

		case WM_ROBOT_CAMERA_BEHAVIOR_MODE:
		{
			g_bCmdRecognized = TRUE;
			m_IdleBehavior = wParam;
			if( m_IdleBehavior >= BEHAVIOR_MAX )
			{
				ROBOT_LOG( TRUE,"CAMERA_BEHAVIOR_MODE - ERROR! Illegal value! %d\n", wParam)
				m_IdleBehavior = BEHAVIOR_FRIENDLY;
				ROBOT_LOG( TRUE,"Behavior set to mode %d\n", m_IdleBehavior)
			}
			return;
		}

		case WM_ROBOT_CAMERA_MODE_CMD:
		{
			// wParam = Mode, lParam = Value
			g_bCmdRecognized = TRUE;
			return;
		}

		case WM_ROBOT_CAMERA_TAKE_SNAPSHOT:
		{
			g_bCmdRecognized = TRUE;
			return;
		}
		case WM_ROBOT_CAMERA_RECORD_VIDEO:
		{
			g_bCmdRecognized = TRUE;
			return;
		}
		

		case WM_ROBOT_CAMERA_INITIALIZE_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Reset the camera
			SendHardwareCmd( HW_CAMERA_INITIALIZE, 0, 0 );
			return;
		}

		case WM_ROBOT_CAMERA_POWER_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Power Camera on/off
			SendHardwareCmd( HW_SET_CAMERA_POWER, wParam, lParam );

			return;
		}

		case WM_ROBOT_USER_CAMERA_ZOOM_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Zoom in/out and speed of zoom
			SendHardwareCmd( (BYTE)HW_SET_CAMERA_ZOOM, wParam, lParam );
			return;
		}

		case WM_ROBOT_CAMERA_ZOOM_ABS_CMD:
		{
			// Set camera absolute zoom position.  Range is 0 - 16.
			g_bCmdRecognized = TRUE;
			int nZoomLevel = wParam;

			// Zooms camera to level specified
			if( nZoomLevel > 16 )
			{
				ROBOT_LOG( TRUE, "WM_ROBOT_CAMERA_ZOOM_ABS_CMD: BAD Zoom Level: %d \n", nZoomLevel )
				nZoomLevel = 16;
			}
			else if( nZoomLevel < 0 )
			{
				ROBOT_LOG( TRUE, "WM_ROBOT_CAMERA_ZOOM_ABS_CMD: BAD Zoom Level: %d \n", nZoomLevel )
				nZoomLevel = 0;
			}

			#if( CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA )
				if( nZoomLevel != g_CameraZoomPos )
				{
					// Automatically handled by Video Capture loop, except for Sony Camera
					SendHardwareCmd( (BYTE)HW_SET_CAMERA_ZOOM_ABS, nZoomLevel, lParam );
				}
			#endif

			g_CameraZoomPos = nZoomLevel;	// NOTE!  used by Video Capture loop to detect when zoom change is needed
			return;
		}

		case WM_ROBOT_USER_CAMERA_FOWARD:
		{
			// Set camera neck Absolute up/forward position.
			// Note when we move the neck, we need to tilt the head too!
			g_bCmdRecognized = TRUE;

			// Special case "Rest" Position.
			/**
			if( CAMERA_NECK_TENTHDEGREES_REST_POSITION == wParam )
			{
				// Send  commands to move the Camera / Head to "Home" or "Rest" position
				SendHardwareCmd( HW_SET_CAMERA_POWER, POWER_ON, 0 );
				return;
			}
			**/
			// Otherwise, send the command to move the Camera Neck
			SendHardwareCmd( HW_SET_CAMERA_FORWARD_ABS, wParam, 0 ); //TODO - Servo Speed?
			return;
		}



/***
		case WM_ROBOT_USER_CAMERA_PAN_CMD:
		{
			g_bCmdRecognized = TRUE;
			// Send Command to tell camera to move Pan and Tilt
			// This command controls both Pan and Tilt
			UINT m_nCameraPanTiltCmd = wParam;	// Tells info on movement direction


			// Set default to current position
			int NewPanPosition, NewTiltPosition, SideTilt;
			m_pHeadControl->GetHeadPosition( NewPanPosition, NewTiltPosition, SideTilt );


			// Convert command to servo movement
			
			UINT nPanTiltIncrement = m_nUserCameraPanTiltSpeed;	// in TenthDegrees - (example: speed 5 = 50 tenthdegrees = 5.0 degrees)
			
			switch( m_nCameraPanTiltCmd )  // Which direction to Pan/Tilt
			{

				case CAMERA_PAN_STOP:
				{
					break;
				}
				case CAMERA_PAN_UP:
				{
					NewTiltPosition += nPanTiltIncrement;
					break;
				}
				case CAMERA_PAN_DOWN:
				{
					NewTiltPosition -= nPanTiltIncrement;
					break;
				}

				case CAMERA_PAN_LEFT:
				{
					NewPanPosition -= nPanTiltIncrement;
					break;
				}

				case CAMERA_PAN_RIGHT:
				{
					NewPanPosition += nPanTiltIncrement;
					break;
				}

				// Combo Pan/Tilt commands
				case CAMERA_PAN_UP_LEFT:
				{
					NewTiltPosition += nPanTiltIncrement;
					NewPanPosition -= nPanTiltIncrement;
					break;
				}
				case CAMERA_PAN_UP_RIGHT:
				{
					NewTiltPosition += nPanTiltIncrement;
					NewPanPosition += nPanTiltIncrement;
					break;
				}
				case CAMERA_PAN_DOWN_LEFT:
				{
					NewTiltPosition -= nPanTiltIncrement;
					NewPanPosition -= nPanTiltIncrement;
					break;
				}
				case CAMERA_PAN_DOWN_RIGHT:
				{
					NewTiltPosition -= nPanTiltIncrement;
					NewPanPosition += nPanTiltIncrement;
					break;
				}
				case CAMERA_PAN_ABS_CENTER:
				{
					NewTiltPosition = 0;
					NewPanPosition = 0;
					break;
				}
				default:
				{
					ROBOT_LOG( TRUE, "ERROR! Invalid Camera Pan Direction, m_nCameraPanTiltCmd = 0x%08lX\n", m_nCameraPanTiltCmd )
				}
				break;
			}

			m_pHeadControl->SetHeadSpeed( HEAD_OWNER_USER_CONTROL, SERVO_SPEED_MED, SERVO_SPEED_MED, SERVO_SPEED_MED );
			m_pHeadControl->SetHeadPosition( HEAD_OWNER_USER_CONTROL, NewPanPosition, NewTiltPosition, NOP );
			m_pHeadControl->ExecutePositionAndSpeed( HEAD_OWNER_USER_CONTROL );

			return;
		}
		***/


		case WM_ROBOT_SENSOR_STATUS_READY:
		{
			//g_bCmdRecognized = TRUE;
			//CString MsgString;

			#if SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI // Head mounted IR is only on Loki

#ifdef IR_SENSORS_ADDED_BACK_TO_HEAD	// all this disabled for now

				if( m_IRTrackingEnabled && (!m_VidCapProcessingEnabled) )
				{
					// only do this if IR head tracking is enabled, but VidCapProcessing is NOT!
					// (if VidCapProcessingEnabled, the frame callback thread will prioritize IR tracking with other tracking)
					/*
						DWORD dwCurrTime = GetTickCount();
						if( (0 != m_CameraMoveTime)								&&
							((dwCurrTime - m_CameraMoveTime) > CAMERA_MOVE_TIME_IR) )
						{
							m_CameraMoveTime = 0; // Reset 
							//Beep(500, 100);	// Beep to indicate Motor done moving
						}
						else
					*/

					// Track closest object
					if( (g_SensorStatus.IR[IR_SENSOR_HEAD_LEFT]  <= IR_HEAD_TRACKING_RANGE ) ||		
						(g_SensorStatus.IR[IR_SENSOR_HEAD_RIGHT] <= IR_HEAD_TRACKING_RANGE ) )		
					{
						// An object found within range of IR sensors.  Track it

						if( (g_SensorStatus.IR[IR_SENSOR_HEAD_LEFT] + IR_TRACKING_FUDGE) < g_SensorStatus.IR[IR_SENSOR_HEAD_RIGHT] )
						{
							m_IRTrackCenter.x = -30;	// Object is to the Left
						}
						else if( (g_SensorStatus.IR[IR_SENSOR_HEAD_RIGHT] + IR_TRACKING_FUDGE) < g_SensorStatus.IR[IR_SENSOR_HEAD_LEFT] )
						{
							m_IRTrackCenter.x = 30;	// Object is to the Right
						}
						// Debug
						ROBOT_LOG( TRUE,"Camera IRTracking: Left = %d, Right = %d, Command = %d\n", 
							g_SensorStatus.IR[IR_SENSOR_HEAD_LEFT], g_SensorStatus.IR[IR_SENSOR_HEAD_RIGHT], m_IRTrackCenter.x )

						if( (0 != m_IRTrackCenter.x ) )	// && (0 != m_IRTrackCenter.y)  && (0 == m_CameraMoveTime) )
						{
							//	m_IRTrackCenter.y = 20;	// Tilt head up a bit to look people in the eye :-)
							//	PositionCamera( m_IRTrackCenter );
							//	m_IRTrackCenter.x = 0;

							int CameraTiltPos = 200;	// Tilt up to look at person (TenthDegrees)
							int CameraPanPos = m_IRTrackCenter.x;

							m_pHeadControl->SetHeadSpeed(SERVO_SPEED_MED, SERVO_SPEED_MED SERVO_SPEED_MED )
							m_pHeadControl->SetHeadPosition( Owner, NOP, CameraTiltPos, NOP );				// Pan, Tilt, SideTilt
							m_pHeadControl->SetHeadPositionRelative( Owner, m_IRTrackCenter.x, NOP, NOP );	// Pan, Tilt, SideTilt
							m_pHeadControl->ExecutePositionAndSpeed( );
							ROBOT_LOG( TRUE,"CAMERA IR TRACK: Tracking Point: Pan = %d  Tilt = %d\n", CameraPanPos, CameraTiltPos )

							//m_pHeadControl->SetHeadPositionRelative( Owner, m_IRTrackCenter.x, m_IRTrackCenter.y, NOP, TRUE )
							// ROBOT_LOG( TRUE,"CAMERA IR TRACK: Tracking Point: Relative Pan = %d  Tilt = %d\n", m_IRTrackCenter.x, m_IRTrackCenter.y )
							// m_CameraMoveTime = GetTickCount();	// tell detectors that camera is moving
							// m_MotionDetectorTime = GetTickCount();	// tell motion detector to allow settle time
							m_IRTrackCenter.x = 0;
						}
					}
				}
#endif //IR_SENSORS_ADDED_BACK_TO_HEAD
			#endif // SENSOR_CONFIG_TYPE == SENSOR_CONFIG_LOKI
		}	// end case WM_ROBOT_SENSOR_STATUS_READY
	}	// end Switch
} // end process message



void CCameraModule::CenterCamera()
{
	SendCommand( WM_ROBOT_CAMERA_PAN_ABS_CMD, (DWORD)CAMERA_PAN_CENTER, 0 );
	SendCommand( WM_ROBOT_CAMERA_TILT_ABS_CMD, (DWORD)CAMERA_TILT_CENTER, 0 );
	m_ServoDelayTime = SERVO_DELAY_MEDIUM; // Give Servo time to move
	m_ColorSearchingState = IDLE;	// Go to idle state

}



void CCameraModule::CameraIdleBehavior()
{
	__itt_task_begin(pDomainVidCapThread, __itt_null, __itt_null, pshCameraIdle);

	m_LastFaceCenter.x = 0;
	m_LastFaceCenter.y = 0;
	m_LastMotionCenter.x = 0;
	m_LastMotionCenter.y = 0;

	if(0 != m_MotionDetectorTime )
	{
		__itt_task_end(pDomainVidCapThread);
		return;	// camera moving
	}

	if( !gHeadIdle )
	{		
		__itt_task_end(pDomainVidCapThread);
		return; // Idle behavior disabled
	}

	// Haven't seen anyone in a while. 
	if( MIN_ZOOM_LEVEL != g_CameraZoomPos )
	{
		ZoomAbs(MIN_ZOOM_LEVEL);	// go to wide mode (minimum zoom)
		ROBOT_LOG( TRUE,"Camera Idle, so Zooming Out!\n")
	}

	__itt_task_end(pDomainVidCapThread);	
}


/* void CCameraModule::WaitingForSomethingInteresting()
{
	// Only called if nothing detected for a while.

}
*/
void CCameraModule::ZoomAbs( int nZoomLevel )
{
	// Zooms camera to level specified
	if( nZoomLevel > 16 )
	{
		ROBOT_LOG( TRUE, "ZoomAbs - BAD Zoom Level: %d \n", nZoomLevel )
		nZoomLevel = 16;
	}
	else if( nZoomLevel < 0 )
	{
		ROBOT_LOG( TRUE, "ZoomAbs - BAD Zoom Level: %d \n", nZoomLevel )
		nZoomLevel = 0;
	}

	if( nZoomLevel != g_CameraZoomPos )
	{
		SendCommand( WM_ROBOT_CAMERA_ZOOM_ABS_CMD, (DWORD)nZoomLevel, 0 );
		// Since Zoom is slow, allow extra time before enabling motion detector
		g_LastCameraMoveTime = GetTickCount(); // And delay IDLE behavior a bit longer
		m_MotionDetectorTime = g_LastCameraMoveTime + MOTION_DETECTOR_TIME;
	}
}

void CCameraModule::ZoomIn( int ZoomDelta )
{
	// Zooms camera in by Delta specified
	int  nZoomLevel = g_CameraZoomPos + ZoomDelta;
	if( nZoomLevel > 16 )
	{
		ROBOT_LOG( TRUE, "ZoomIn - BAD Zoom Level: %d \n", nZoomLevel )
		nZoomLevel = 16;
	}

	if( g_CameraZoomPos < 16 )
	{
		ROBOT_LOG( TRUE, "CCamera Requesting Zoom Level: %d \n", nZoomLevel )
		SendCommand( WM_ROBOT_CAMERA_ZOOM_ABS_CMD, (DWORD)nZoomLevel, 0 );
		g_LastCameraMoveTime = GetTickCount(); // And delay IDLE behavior a bit longer
		m_MotionDetectorTime = g_LastCameraMoveTime + MOTION_DETECTOR_TIME;
	}
}

void CCameraModule::ZoomOut( int ZoomDelta )
{
	// Zooms camera in by Delta specified
	int  nZoomLevel = g_CameraZoomPos - ZoomDelta;
	if( nZoomLevel < MIN_ZOOM_LEVEL )
	{
		// Not an error, just hit max zoom out
		return;
	}

	if( g_CameraZoomPos > 0 )
	{
		ROBOT_LOG( TRUE, "CCamera Requesting Zoom Level: %d \n", nZoomLevel )
		SendCommand( WM_ROBOT_CAMERA_ZOOM_ABS_CMD, (DWORD)nZoomLevel, 0 );
		g_LastCameraMoveTime = GetTickCount(); // And delay IDLE behavior a bit longer
		m_MotionDetectorTime = g_LastCameraMoveTime + MOTION_DETECTOR_TIME;
	}
}

void CCameraModule::ZoomCameraToWindowSize( CvSize TargetWindowSize )
{
	// Given current window size and target size, find correct zoom amount

	
}

void CCameraModule::PositionCamera( int nOwner, CvPoint CameraPanTiltPoint)
{
	// Adjust camera position to point at target

	ROBOT_LOG( TRUE,"DEBUG: PositionCamera: CameraPanTiltPoint = %d,%d\n",CameraPanTiltPoint.x, CameraPanTiltPoint.y )

	CString	MsgString;
	BOOL	MovingServo = FALSE;
	int		TenthDegreeMoveX = 0;
	int		TenthDegreeMoveY = 0;

	g_LastCameraMoveTime = GetTickCount(); // indicate when to execute IDLE behavior

	// Pan Camera to center on item
	// Global Zoom value g_CameraZoomPos ranges from 0 - 16,   

#if( CAMERA_CONTROL_TYPE == SONY_SERIAL_CAMERA )
	// This table is for SONY camera:
	// Table to map Zoom level to camera Field Of View
	// Camera values are non-linear, so table is used instead of formula.  See CameraZoom.xls spreadsheet for details
	const double FovDegreesTable[17] =
	{
		//0		  1		  2		  3		  4		  5		  6		  7		  8		// Zoom Level
// 		77.77,	74.33,	68.23,	64.34,	58.89,	52.39,	48.61,	42.32,	37.42,	//FovDegrees Calculated
 		79.0,	73.0,	66.0,	60.0,	54.0,	48.0,	42.0,	37.0,	32.0,	//FovDegrees Trimmed

		//9		 10		 11		 12		 13		 14		 15		 16				// Zoom Level
//		32.38,	28.94,	23.68,	19.22,	15.61,	12.88,	10.14,	8.30			//FovDegrees Calculated
		27.0,	24.0,	20.5,	16.5,	14.0,	11.5,	9.0,	7.5				//FovDegrees Trimmed
	};

	double DegreesPerFrameX = FovDegreesTable[g_CameraZoomPos];	// 8.3 - 78 ZOOM
	// Factor in Zoom level and ratio of Frame Height to Frame Width
	double DegreesPerFrameY = (FovDegreesTable[g_CameraZoomPos]);
	DegreesPerFrameY = DegreesPerFrameY * GetFrameAspectRatio();


#elif( CAMERA_CONTROL_TYPE == DYNA_SERVO_CONTROL )
	// This table is for LOKI, using "Logitech Quickcam Pro for Notebooks"
	// WARNING!!! ZOOM NOT SUPPORTED!
	// Calculation used: Robot at 52" from wall.  FOV width = 59", height = 43.8"
	// ArcTan = Opp/Adj. Opp = 1/2 width or height (to form right triangle)
	// Width  HALF FOV = ArcTan( (59/2) / 52 )   => HALF_FOV_X = 29.567.  FOV_X = 59.133 degrees
	// Height HALF FOV = ArcTan( (43.8/2) / 52 ) => HALF_FOV_Y = 22.839.  FOV_Y = 45.677 degrees
	// Now, adjust values as needed, since camera not mounted exactly over center of servos!
	#define FOV_X	64.0 	// 59 calculated.  Non-linear due to camera lens
	#define FOV_Y	45.0	// 45.677 degrees calculated

	double DegreesPerFrameX = FOV_X;
	double DegreesPerFrameY = FOV_Y;

//#elif( CAMERA_CONTROL_TYPE == PIC_SERVO_CONTROL )
//#elif( CAMERA_CONTROL_TYPE == EXTERN_SERVO_CONTROL )
	//TODO-CARBOT-MUST - Add FOV table here!
#else
	#error BAD CAMERA_CONTROL_TYPE
#endif


	// Camera Pan
	int PixelDeltaX = CameraPanTiltPoint.x - GetFrameCenter(m_pVideoFrameLeft).x;
	if( abs(PixelDeltaX) > CAMERA_PIXEL_FUDGE )
	{
		// Calculate Pixels per 1/10 Degree.
		double PixelsPerTenthDegreeX = m_pVideoFrameLeft->width / (DegreesPerFrameX * 10);
//		Old:		double ServoTicksPerPixelX = 5.0 / PixelsPerDegreeX;	// 5.0?  7.8?
//		double ServoTicksPerPixelX = CAMERA_PAN_PER_DEGREE / PixelsPerDegreeX;
		// KLUDGE FOR CAR TODO-CAR-MUST REMOVE / FIX THIS
//		ServoTicksPerPixelX =  0.3; // HARD CODED!!!
//		ServoMoveX = int(PixelDeltaX * ServoTicksPerPixelX);

		TenthDegreeMoveX = (int)((double)PixelDeltaX / PixelsPerTenthDegreeX );
		MovingServo = TRUE;
//		Sleep(10);

		// Limit how far from center Camera tracking will go (avoid la la land!)

	}

	// Camera Tilt
	int PixelDeltaY =  GetFrameCenter(m_pVideoFrameLeft).y - CameraPanTiltPoint.y;
	if( abs(PixelDeltaY) > CAMERA_PIXEL_FUDGE )
	{
		// Calculate Pixels per 1/10 Degree.
		double PixelsPerTenthDegreeY = m_pVideoFrameLeft->height / (DegreesPerFrameY *10);
//		double ServoTicksPerPixelY = 7.0 / PixelsPerDegreeY; 
//		double ServoTicksPerPixelY = CAMERA_TILT_PER_DEGREE / PixelsPerDegreeY; 
		// KLUDGE FOR CAR TODO-CAR-MUST REMOVE / FIX THIS
//		ServoTicksPerPixelY =  -0.3; // HARD CODED!!!
		
		TenthDegreeMoveY = (int)((double)PixelDeltaY / PixelsPerTenthDegreeY);
		MovingServo = TRUE;
	}

	if( MovingServo )
	{
		m_pHeadControl->SetHeadSpeed( nOwner, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW, SERVO_SPEED_MED_SLOW );
		m_pHeadControl->SetHeadPosition( nOwner, NOP, NOP, 0 ); // No Side-Tilt!
		m_pHeadControl->SetHeadPositionRelative( nOwner, TenthDegreeMoveX, TenthDegreeMoveY, NOP, TRUE ); // TRUE = Limit to front of robot
		m_pHeadControl->ExecutePositionAndSpeed( nOwner );
		ROBOT_LOG( TRUE,"CAMERA: Tracking Point: Relative Pan = %d  Tilt = %d\n", (int)(TenthDegreeMoveX/10), (int)(TenthDegreeMoveY/10) )
		ROBOT_LOG( TRUE,"***************> TRACKING SPEED FAST\n")
	}
}




#endif // #if ( ROBOT_SERVER == 1 )	// This module used for Robot Server only
