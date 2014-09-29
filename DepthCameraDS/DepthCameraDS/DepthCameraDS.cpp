// DepthCameraDS.cpp : Depth Camera capture and viewer
//

#include "stdafx.h"
//#include <csignal>
#include <windows.h>
//#include <tchar.h>
//#include <stdio.h>
#include <iostream>
//#include <fstream>
//#include <sstream>

using namespace std;

#include <DepthCommon.h>
#include "DepthCameraDS.h"

#include "DSAPI.h"
#include "DSAPIUtil.h"
#include "DSCommonUtils.h"

//#include "DSSampleCommon/Common.h"

#include <cassert>
#include <cctype>
#include <algorithm>
#include <sstream>

// Delete data and set pointer to Null
#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
#define SAFE_DELETE_CS(p)  { if(p) { DeleteCriticalSection(&p);     (p)=NULL; } }


// Global Variables
LPCTSTR			pDepthDataSharedMemory = NULL;
HANDLE			hDepthDataAvailableEvent = NULL;
LPCTSTR			pCommandSharedMemory = NULL;
HANDLE			hCommandEvent = NULL;
DEPTH_CAMERA_COMMAND_T g_Command;

int				g_InitIPCResult = 0;

DSAPI			* g_dsapi;
DSThird			* g_third;
DSHardware		* g_hardware;
bool			g_paused, g_showOverlay = true, g_showImages = true, g_stopped = true;
GlutWindow		g_depthWindow, g_leftWindow, g_rightWindow, g_thirdWindow; // Windows where we will display depth, left, right, third images
Timer			g_timer, g_thirdTimer;
int				g_lastThirdFrame;
float			g_exposure, g_gain;

uint8_t			g_zImageRGB[640 * 480 * 3];

void DrawGLImage(GlutWindow & window, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid * pixels);

void OnIdle()
{
    if (g_stopped)
        return;

    if (!g_paused)
    {
        DS_CHECK_ERRORS(g_dsapi->grab());
    }


	///////////////////////////////////////////////////////////////////////////////////
	// Copy data to shared memory buffer
	// TODO:  Don't send an update every frame!
	static int SendDataIncrement = 0; 

	DEPTH_CAMERA_FRAME_HEADER_T FrameHeader;

	//if( SendDataIncrement++ > 5 )
	{

		FrameHeader.Height = g_dsapi->zHeight();
		FrameHeader.Width = g_dsapi->zWidth();
		FrameHeader.FrameNumber = g_dsapi->getFrameNumber();
		FrameHeader.tooFarDepth = 0;
		FrameHeader.MouseDown = 0;
		FrameHeader.MouseX = 1;
		FrameHeader.MouseY = 2;

		uint16_t* pZImageDebug = g_dsapi->getZImage();
		// Debug / test
		
		for(int i =0; i<16; i++)
		{
			pZImageDebug[i] = i;
		}

		// Send data to Robot:
		if( (NULL != pDepthDataSharedMemory) && (NULL != hDepthDataAvailableEvent) )
		{
			CopyMemory( (PVOID)pDepthDataSharedMemory, &FrameHeader, sizeof(DEPTH_CAMERA_FRAME_HEADER_T) );
			PVOID pSharedDepthData = (PVOID)( (unsigned char*)pDepthDataSharedMemory + (sizeof(DEPTH_CAMERA_FRAME_HEADER_T)) ); 

			CopyMemory( pSharedDepthData, (PVOID)g_dsapi->getZImage(), (FrameHeader.Width * FrameHeader.Height * 2) );
			SetEvent( hDepthDataAvailableEvent ); // Indicate that new data is available
		}


		///////////////////////////////////////////////////////////////////////////////////
		// Check for Commands from the robot control code
		// so far, this is only a command for when to draw a bounding box

		if( STAND_ALONE_MODE != g_InitIPCResult )
		{
			DWORD dwResult = WaitForSingleObject(hCommandEvent, 0);
			if( WAIT_OBJECT_0 == dwResult ) 
			{
				// Event did not time out.  That means the Robot has posted a command to shared memory
				//std::cout << "Command Received: " << endl;
				// Read from Shared Memory
				CopyMemory( (PVOID)&g_Command, pCommandSharedMemory, sizeof(DEPTH_CAMERA_COMMAND_T) );

				// Process the command block
				/*
				if( 0 != Command->bShutDown )
				{
					// Exit application.  Kobuki destructor automatically stops motors
					cout << "COMMAND SHUTDOWN received from control app" << endl;
					bRunLoop = 0;
					continue;
				} */


			}
			else
			{
				// no command received.  Just leave the last command active?
				//g_Command.ControlFlags = DepthCameraControlFlag_None;
			}
		}
		else
		{
			// Stand Alone mode - just show status for debug testing?
		}

		SendDataIncrement = 0;
	}


	///////////////////////////////////////////////////////////////////////////////////
	// Display local images

    // Display Z image, if it is enabled
    if (g_dsapi->isZEnabled())
    {
        const uint8_t nearColor[] = {255, 0, 0}, farColor[] = {20, 40, 255};
        if (g_showImages) ConvertDepthToRGBUsingHistogram(g_dsapi->getZImage(), g_dsapi->zWidth(), g_dsapi->zHeight(), nearColor, farColor, g_zImageRGB);
        DrawGLImage(g_depthWindow, g_dsapi->zWidth(), g_dsapi->zHeight(), GL_RGB, GL_UNSIGNED_BYTE, g_zImageRGB);
    }

    // Display left/right images, if they are enabled
    if (g_dsapi->isLeftEnabled() || g_dsapi->isRightEnabled())
    {
        assert(g_dsapi->getLRPixelFormat() == DS_LUMINANCE8);
        if (g_dsapi->isLeftEnabled()) DrawGLImage(g_leftWindow, g_dsapi->lrWidth(), g_dsapi->lrHeight(), GL_LUMINANCE, GL_UNSIGNED_BYTE, g_showImages ? g_dsapi->getLImage() : nullptr);
        if (g_dsapi->isRightEnabled()) DrawGLImage(g_rightWindow, g_dsapi->lrWidth(), g_dsapi->lrHeight(), GL_LUMINANCE, GL_UNSIGNED_BYTE, g_showImages ? g_dsapi->getRImage() : nullptr);
    }

    // Display third image, if it is enabled
    if (g_third && g_third->isThirdEnabled() && g_third->getThirdFrameNumber() != g_lastThirdFrame)
    {
        assert(g_third->getThirdPixelFormat() == DS_RGB8);
        DrawGLImage(g_thirdWindow, g_third->thirdWidth(), g_third->thirdHeight(), GL_RGB, GL_UNSIGNED_BYTE, g_showImages ? g_third->getThirdImage() : nullptr);
        g_lastThirdFrame = g_third->getThirdFrameNumber();

        g_thirdTimer.OnFrame();
    }

    g_timer.OnFrame();
}

void PrintControls()
{
    std::cout << "\nProgram controls:" << std::endl;
    std::cout << "  (space) Pause/unpause" << std::endl;
    std::cout << "  (d) Toggle display of overlay" << std::endl;
    std::cout << "  (i) Toggle display of images" << std::endl;
    std::cout << "  (s) Take snapshot" << std::endl;
    std::cout << "  (e/E) Modify exposure" << std::endl;
    std::cout << "  (g/G) Modify gain" << std::endl;
    std::cout << "  (l) Toggle emitter" << std::endl;
    std::cout << "  (q) Quit application" << std::endl;
}

void OnKeyboard(unsigned char key, int x, int y)
{
	float a = 1.0;
	float b = 1.5;
	float foo = __min(a, b);


    switch (std::tolower(key))
    {
    case ' ':
        g_paused = !g_paused;
        break;
    case 'd':
        g_showOverlay = !g_showOverlay;
        break;
    case 'i':
        g_showImages = !g_showImages;
        break;
    case 's':
        g_dsapi->takeSnapshot();
        break;
    case 'e':
		//float sign = (key == 'E' ? 0.1f : -0.1f);
//        g_exposure = std::min(std::max(g_exposure + (key == 'E' ? 0.1f : -0.1f), 0.0f), 16.3f);
        std::cout << "Setting exposure to " << std::fixed << std::setprecision(1) << g_exposure << std::endl;
        DS_CHECK_ERRORS(g_hardware->setImagerExposure(g_exposure, DS_BOTH_IMAGERS));
        break;
    case 'g':
//        g_gain = std::min(std::max(g_gain + (key == 'G' ? 0.1f : -0.1f), 1.0f), 64.0f);
        std::cout << "Setting gain to " << std::fixed << std::setprecision(1) << g_gain << std::endl;
        DS_CHECK_ERRORS(g_hardware->setImagerGain(g_gain, DS_BOTH_IMAGERS));
        break;
    case 'l':
        if (auto emitter = g_dsapi->accessEmitter())
        {
            DS_CHECK_ERRORS(emitter->enableEmitter(!emitter->isEmitterEnabled()));
        }
        break;
    case 'q':
        glutLeaveMainLoop();
        break;
    case 'r':
        if (g_dsapi->startCapture())
            g_stopped = false;
        break;
    case 't':
        if (g_dsapi->stopCapture())
            g_stopped = true;
        break;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
//int _tmain(int argc, _TCHAR* argv[])
{

	#if (DEBUG_MEMORY_LEAKS == 1 )
		_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	// Use to detect memory leaks!
	// _CrtSetBreakAlloc( Put allocator number here);

//	signal(SIGINT, signalHandler);
	cout << "Starting Depth Viewer..." << endl;

	cout << "Initializing shared memory..." << endl;

	// Initialize shared memory and events for communicating with the Robot control application
	g_InitIPCResult = InitIPC( hCommandEvent, pCommandSharedMemory, hDepthDataAvailableEvent, pDepthDataSharedMemory  );

	if ( FAILED == g_InitIPCResult )
	{
		cerr << "InitIPC failed.  Exiting" << endl;
		Sleep(10000);
		return -1;
	}

//	signal(SIGINT, signalHandler);


/***
	// Open log file
	//#ifndef _DEBUG // optionally, only open the log file in Release mode
	errno_t nError = fopen_s( &g_LogFile, LogFileName, "w" );
	if( 0 == nError )
	{
		fprintf(g_LogFile, "STARTING LOG FILE.  Start Time: %s\n\n", COleDateTime::GetCurrentTime().Format() );
	}
	else
	{
		g_LogFile = NULL;
		CString ErrorStr;
		ErrorStr.Format( _T("Can't open Log File\n %s Error = %d"), LogFileName, nError );
		AfxMessageBox( ErrorStr );
	}
	//#endif
***/

	///////////////////////////////////////////////////////////////////////////////////
	Sleep(100); // Let robot code start up first

	///////////////////////////////////////////////////////////////////////////////////
	// Kick off Control thread to receive commands from the robot control
	// TODO-MUST see code at bottom of this file


	///////////////////////////////////////////////////////////////////////////////////
	// Initialize Depth Camera
    g_dsapi = DSCreate(DS_DS4_PLATFORM);
    g_third = g_dsapi->accessThird();
    g_hardware = g_dsapi->accessHardware();

    DS_CHECK_ERRORS(g_dsapi->probeConfiguration());

    std::cout << "Firmware version: " << g_dsapi->getFirmwareVersionString() << std::endl;
    std::cout << "Software version: " << g_dsapi->getSoftwareVersionString() << std::endl;
    uint32_t serialNo;
    DS_CHECK_ERRORS(g_dsapi->getCameraSerialNumber(serialNo));
    std::cout << "Camera serial no: " << serialNo << std::endl;

    // Check calibration data
    if (!g_dsapi->isCalibrationValid())
    {
        std::cout << "Calibration data on camera is invalid" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Configure core Z-from-stereo capabilities
    DS_CHECK_ERRORS(g_dsapi->enableZ(true));
    DS_CHECK_ERRORS(g_dsapi->enableLeft(false));
    DS_CHECK_ERRORS(g_dsapi->enableRight(false));
    DS_CHECK_ERRORS(g_dsapi->setLRZResolutionMode(true, 480, 360, 60, DS_LUMINANCE8)); // Valid resolutions: 628x468, 480x360
    g_dsapi->enableLRCrop(true);

    // Configure third camera
    if (g_third)
    {
        DS_CHECK_ERRORS(g_third->enableThird(true));
        DS_CHECK_ERRORS(g_third->setThirdResolutionMode(true, 640, 480, 30, DS_RGB8)); // Valid resolutions: 1920x1080, 640x480
    }

    // Change exposure and gain values
    g_exposure = 16.3f;
    g_gain = 2.0f;
    DS_CHECK_ERRORS(g_hardware->setImagerExposure(g_exposure, DS_BOTH_IMAGERS));
    DS_CHECK_ERRORS(g_hardware->setImagerGain(g_gain, DS_BOTH_IMAGERS));

    //float exposure, gain;
    //DS_CHECK_ERRORS(g_hardware->getImagerExposure(exposure, DS_BOTH_IMAGERS));
    //DS_CHECK_ERRORS(g_hardware->getImagerGain(gain, DS_BOTH_IMAGERS));
    //std::cout << "(Before startCapture()) Initial exposure: " << exposure << ", initial gain: " << gain << std::endl;

    // Begin capturing images
    DS_CHECK_ERRORS(g_dsapi->startCapture());
    g_stopped = false;

    //DS_CHECK_ERRORS(g_hardware->getImagerExposure(exposure, DS_BOTH_IMAGERS));
    //DS_CHECK_ERRORS(g_hardware->getImagerGain(gain, DS_BOTH_IMAGERS));
    //std::cout << "(After startCapture()) Initial exposure: " << exposure << ", initial gain: " << gain << std::endl;

    // Open some GLUT windows to display the incoming images
    glutInit(&argc, argv);
    glutIdleFunc(OnIdle);

    if (g_dsapi->isZEnabled())
    {
        std::ostringstream title;
        title << "Z: " << g_dsapi->zWidth() << "x" << g_dsapi->zHeight() << "@" << g_dsapi->getLRZFramerate() << " FPS";
        g_depthWindow.Open(title.str(), 300 * g_dsapi->zWidth() / g_dsapi->zHeight(), 300, 60, 60, OnKeyboard);
    }

    if (g_dsapi->isLeftEnabled())
    {
        std::ostringstream title;
        title << "Left (" << DSPixelFormatString(g_dsapi->getLRPixelFormat()) << "): " << g_dsapi->lrWidth() << "x" << g_dsapi->lrHeight() << "@" << g_dsapi->getLRZFramerate() << " FPS";
        g_leftWindow.Open(title.str(), 300 * g_dsapi->lrWidth() / g_dsapi->lrHeight(), 300, 60, 420, OnKeyboard);
    }

    if (g_dsapi->isRightEnabled())
    {
        std::ostringstream title;
        title << "Right (" << DSPixelFormatString(g_dsapi->getLRPixelFormat()) << "): " << g_dsapi->lrWidth() << "x" << g_dsapi->lrHeight() << "@" << g_dsapi->getLRZFramerate() << " FPS";
        g_rightWindow.Open(title.str(), 300 * g_dsapi->lrWidth() / g_dsapi->lrHeight(), 300, 520, 420, OnKeyboard);
    }

    if (g_third->isThirdEnabled())
    {
        std::ostringstream title;
        title << "Third (" << DSPixelFormatString(g_third->getThirdPixelFormat()) << "): " << g_third->thirdWidth() << "x" << g_third->thirdHeight() << "@" << g_third->getThirdFramerate() << " FPS";
        g_thirdWindow.Open(title.str(), 300 * g_third->thirdWidth() / g_third->thirdHeight(), 300, 520, 60, OnKeyboard);
    }



	///////////////////////////////////////////////////////////////////////////////////
    // Turn control over to GLUT (blocks)
    PrintControls();
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutMainLoop();


	///////////////////////////////////////////////////////////////////////////////////
	// Handle shut down clean up
	cout << "Exiting..." << endl;
    std::cout << "Destroying DSAPI instance." << std::endl;
    DSDestroy(g_dsapi);

	//Sleep(10000); // for debugging
	return 0;

}



static void OnDisplay() {}

int CreateGLWindow(std::string name, int width, int height, int startX, int startY)
{
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(startX, startY);
    auto windowNum = glutCreateWindow(name.c_str());

    glutDisplayFunc(OnDisplay);
    glutKeyboardFunc(OnKeyboard);
    glutIdleFunc(OnIdle);
    return windowNum;
}



void DrawBoundingBox( int x1, int y1, int x2, int y2, float ColorR, float ColorG, float ColorB  )
{
 
    // Set up a pixel-aligned orthographic coordinate space
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 0, -1, +1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Draw a box
    glBegin(GL_LINE_LOOP);
    glColor4f( ColorR, ColorG, ColorB, 1.0f);
    //glColor4f(0.0, 1.0f, 0.0, 1.0f);
    glVertex2i(x1, y1);
    glVertex2i(x1, y2);
    glVertex2i(x2, y2);
    glVertex2i(x2, y1);
    glEnd();
    glDisable(GL_BLEND);

    // Restore GL state to what it was prior to this call
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}


void DrawGLImage(GlutWindow & window, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid * pixels)
{
    window.ClearScreen(0.1f, 0.1f, 0.15f);

    if (g_showImages)
    {
        window.DrawImage(width, height, format, type, pixels, 1);
    }

    if (g_showOverlay)
    {
        DrawString(10, 10, "FPS: %0.1f", (&window == &g_thirdWindow ? g_thirdTimer : g_timer).GetFramesPerSecond());
        DrawString(10, 28, "Frame #: %d", &window == &g_thirdWindow ? g_third->getThirdFrameNumber() : g_dsapi->getFrameNumber());
        DrawString(10, 46, "Frame time: %s", GetHumanTime(&window == &g_thirdWindow ? g_third->getThirdFrameTime() : g_dsapi->getFrameTime()).c_str());
    }

	// Debug
	//DrawBoundingBox(80, 80, 170, 170, 1.0f, 0.0, 0.0 ); // Red - Debug

    // See if robot control app as requested a bounding box to be displayed over the depth image
    if ((DepthCameraControlFlag_DisplayBoundingBox == g_Command.ControlFlags) || (DepthCameraControlFlag_All == g_Command.ControlFlags))
    {
        DrawBoundingBox(g_Command.BoundingBoxLeft, g_Command.BoundingBoxTop, g_Command.BoundingBoxRight, g_Command.BoundingBoxBottom, 0.0, 1.0f, 0.0 ); // Green
	}

    glutSwapBuffers();
}



