
// Shared file between main Robot control and handler for depth camera
// Note that this is NOT accessed by the C# Kinect depth camera, but does has defines
// for that code that are defined here.
#pragma once



#define DEPTH_CAMERA_DATA_SHARED_FILE_NAME		"RobotDepthDataMappingObject"
#define DEPTH_CAMERA_COMMAND_SHARED_FILE_NAME	"RobotDepthCommandMappingObject"

#define DEPTH_DATA_AVAILABLE_EVENT_NAME		"RobotDepthDataAvailableEvent"
#define DEPTH_CAMERA_COMMAND_EVENT_NAME		"RobotDepthCommandEvent"

//#define KOBUKI_MAX_NAME_LEN			 31
//#define KOBUKI_REQUEST_BUFFER_SIZE  128
//#define KOBUKI_RESPONSE_BUFFER_SIZE 128


// Max depth image size
#define MAX_DEPTH_DATA_SIZE	(640 * 480 * 2)	// 2 bytes per 16 bit depth value

// Max KINECT depth image size
#define MAX_KINECT_DEPTH_DATA_SIZE	((640*480)+4)	// 4 bytes per 32 bit value


const int DepthCameraControlFlag_None = 0;
const int DepthCameraControlFlag_DisplayBoundingBox = 1;
const int DepthCameraControlFlag_HidePlayers = 2;
const int DepthCameraControlFlag_All = 3;				// enables all flags


typedef struct
{									// Use 16 bit values, same as the data, for easier debug of copy issues
	unsigned short	Height;
	unsigned short	Width;
	unsigned short	FrameNumber;
	unsigned short	tooFarDepth;	// max depth as reported by SDK
	unsigned short MouseDown;		// User clicked a mouse in the depth applicaiton
	unsigned short MouseX;
	unsigned short MouseY;
} DEPTH_CAMERA_FRAME_HEADER_T; 

typedef struct
{
	DEPTH_CAMERA_FRAME_HEADER_T FrameHeader;
	unsigned short DepthData[MAX_DEPTH_DATA_SIZE]; // 16 bit values (2 bytes)
} DEPTH_CAMERA_FRAME_T; 



typedef struct
{
//	unsigned int	bShutDown;		// Boolean - shut down if set
	int	ControlFlags;			// OUT - flags to control behavior of the depth camera applicaiton code
	int	BoundingBoxTop;			// OUT
	int	BoundingBoxBottom;		// OUT
	int	BoundingBoxLeft;		// OUT
	int	BoundingBoxRight;		// OUT

} DEPTH_CAMERA_COMMAND_T; 