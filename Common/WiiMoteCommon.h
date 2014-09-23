// WiiMoteCommon.h
// Common defines used by Robot and WiiMote input
#ifndef __ROBOT_WIIMOTE_SHARED_PARAMS_H__
#define __ROBOT_WIIMOTE_SHARED_PARAMS_H__

#include "RobotConfig.h"

#define WIIMOTE_SHARED_FILE_NAME	"Global\\MyFileMappingObject"

//TCHAR szWiiMoteSharedFileName[]=TEXT(WIIMOTE_SHARED_FILE_NAME);

#define DEFAULT_PATH_FILE			ROBOT_DATA_PATH "\\DefaultPath.pat"
#define DEFAULT_MAP_FILE			ROBOT_DATA_PATH "\\BlankMap.rmp"

#define WIIMOTE_A		0x0800
#define WIIMOTE_B		0x0400
#define WIIMOTE_1		0x0200
#define WIIMOTE_2		0x0100
#define WIIMOTE_MINUS	0x1000
#define WIIMOTE_PLUS	0x0010
#define WIIMOTE_UP		0x0008
#define WIIMOTE_DOWN	0x0004
#define WIIMOTE_RIGHT	0x0002
#define WIIMOTE_LEFT	0x0001
#define WIIMOTE_HOME	0x8000

typedef struct
{
	BOOL	Visible;
	UINT	Size;
	double	X;
	double	Y;
} WIIMOTE_IR_DOT_T;



typedef struct
{
	UINT	Battery;
	UINT	bOrientationValid;
	int		Pitch;
	int		Roll;
	double	Accel_X;
	double	Accel_Y;
	double	Accel_Z;
	UINT	Buttons;
	WIIMOTE_IR_DOT_T	IrDot[4];
} WIIMOTE_STATUS_T;



#endif // __ROBOT_WIIMOTE_SHARED_PARAMS_H__