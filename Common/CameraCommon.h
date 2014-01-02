// CameraCommon.h
// Common defines used by Robot and Camera process
#ifndef __ROBOT_CAMERA_SHARED_PARAMS_H__
#define __ROBOT_CAMERA_SHARED_PARAMS_H__

//#include "stdafx.h"
//#include "TCHAR.H"

//#include "RobotType.h"

#define CAMERA_DATA_SHARED_FILE_NAME	"RobotCameraDataMappingObject"
#define CAMERA_REQUEST_SHARED_FILE_NAME	"RobotCameraRequestMappingObject"

#define CAMERA_UPDATE_EVENT_NAME "RobotCameraAppUpdateEvent"
#define CAMERA_REQUEST_EVENT_NAME "RobotCameraAppRequestEvent"

#define CAMERA_MAX_NAME_LEN			 31
#define CAMERA_REQUEST_BUFFER_SIZE  128
#define CAMERA_RESPONSE_BUFFER_SIZE 128
 

//////////////////////////////////////////////////////////////////////////////////
// Messages from Robot app to Camera app

enum CAMERA_REQUEST {
		CAMERA_REQUEST_NONE = 0,
		CAMERA_REQUEST_ENABLE_FEATURES,
		CAMERA_REQUEST_FACE_RECO,		
		CAMERA_REQUEST_OBJECT_RECO,
		CAMERA_REQUEST_TAKE_SNAPSHOT,
};

// Command from Robot to capture a frame, and use name supplied
typedef struct
{
	int		bRequestCapture;
	int		PersonID;
	char	PersonName[CAMERA_MAX_NAME_LEN+1];
} CAMERA_FACE_RECO_REQUEST_T;

// Command to enable/disable the selected detectors
typedef struct
{
	int	VideoEnable;
	int	FaceTracking;
	int	FaceRecognition;
	int ObjectMatch;
} CAMERA_REQUEST_ENABLE_FEATURES_T;

// Shared data area for all requests
typedef union CameraRequestData
{
	CAMERA_REQUEST_ENABLE_FEATURES_T	EnableFeatures;
	CAMERA_FACE_RECO_REQUEST_T			FaceRequest;
} CAMERA_REQUEST_DATA_T;

typedef struct
{
	int	 RequestType;
	CAMERA_REQUEST_DATA_T RequestData;
} CAMERA_REQUEST_T;


//////////////////////////////////////////////////////////////////////////////////
// Messages from Camera app to Robot app

enum CAMERA_UPDATE {
		CAMERA_UPDATE_NONE = 0,
		CAMERA_UPDATE_FACE_RECO,		
		CAMERA_UPDATE_OBJECT_RECO,		
};

// Face detected message
typedef struct
{
	int		FaceDetected;
	int		PersonRecognized;
	int		PersonID;
	char	PersonName[CAMERA_MAX_NAME_LEN+1];
} CAMERA_FACE_RECO_UPDATE_T;

// Object detected message
typedef struct
{
	int		bObjectRecognized;
	int		ObjectID;
	char	ObjectName[CAMERA_MAX_NAME_LEN+1];
} CAMERA_OBJECT_RECO_UPDATE_T;

typedef union //CameraUpdateData
{
	CAMERA_FACE_RECO_UPDATE_T		FaceUpdate;
	CAMERA_OBJECT_RECO_UPDATE_T		ObjectUpdate;
} CAMERA_UPDATE_DATA_T;


typedef struct
{
	CAMERA_UPDATE			UpdateType;
	CAMERA_UPDATE_DATA_T	UpdateData;
} CAMERA_UPDATE_T;


#endif // __ROBOT_CAMERA_SHARED_PARAMS_H__