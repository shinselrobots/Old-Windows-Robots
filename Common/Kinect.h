// Kinect.h
// Interface for Kinect / PrimeSense sensor. Uses OpenNI library
// See:
// http://www.codeproject.com/Articles/148251/How-to-Successfully-Install-Kinect-on-Windows-Open.aspx
// http://groups.google.com/group/openni-dev
// http://www.openni.org/

#pragma once

#include "../Common/HardwareCmds.h"
#include "Globals.h"

//#include "cv.h"
//#include "highgui.h"
//#include "cxmisc.h"

//using namespace std;


#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )
#include "MSR_NuiApi.h"
#endif
#include "DrawDevice.h"


#if ( KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA )

	/////////////////////////////////////////////////////////////////////////////
	class CKinectNui
	{
	public:
		HRESULT					Nui_Init();
		void                    Nui_UnInit( );
		void                    Nui_GotDepthAlert( );
		void                    Nui_GotVideoAlert( );
		void                    Nui_GotSkeletonAlert( );
		void                    Nui_Zero();
		void                    Nui_BlankSkeletonScreen( HWND hWnd );
		void                    Nui_DoDoubleBuffer(HWND hWnd,HDC hDC);
		void                    Nui_DrawSkeleton( bool bBlank, NUI_SKELETON_DATA * pSkel, HWND hWnd, int WhichSkeletonColor );
		void                    Nui_DrawSkeletonSegment( NUI_SKELETON_DATA * pSkel, int numJoints, ... );

		RGBQUAD                 Nui_ShortToQuad_Depth( USHORT s );
		RGBQUAD                 Nui_ShortToColorQuad_Depth( USHORT s );
		void					Nui_UpdatePointCloud( UINT x, UINT y, int DepthValueMM, KINECT_SURFACE_DESC FrameDesc );

		//static LONG CALLBACK    WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

		HWND m_hWnd;

	private:
		static DWORD WINAPI     Nui_ProcessThread(LPVOID pParam);


		// NUI and draw stuff
		DrawDevice    m_DrawDepth;
		DrawDevice    m_DrawVideo;

		// thread handling
		//HANDLE        m_hThNuiProcess;
		HANDLE        m_hEvNuiProcessStop;

		HANDLE        m_hNextDepthFrameEvent;
		HANDLE        m_hNextVideoFrameEvent;
		HANDLE        m_hNextSkeletonEvent;
		HANDLE        m_pDepthStreamHandle;
		HANDLE        m_pVideoStreamHandle;
		//HFONT         m_hFontFPS;
		HPEN          m_Pen[6];
		HDC           m_SkeletonDC;
		HBITMAP       m_SkeletonBMP;
		HGDIOBJ       m_SkeletonOldObj;
		int           m_PensTotal;
		POINT         m_Points[NUI_SKELETON_POSITION_COUNT];
		RGBQUAD       m_rgbWk[640*480];
		int           m_LastSkeletonFoundTime;
		bool          m_bScreenBlanked;
		int           m_FramesTotal;
		int           m_LastFPStime;
		int           m_LastFramesTotal;
	};

#endif // KINECT_SDK_TYPE == KINECT_MICROSOFT_BETA


/////////////////////////////////////////////////////////////////////////////
	/*
class CKinectDetector
{

public:

	CKinectDetector( IplImage* pVideoFrame );	// May be Depth or Video frame!
	~CKinectDetector(){};


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


/////////////////////////////////////////////////////////////////////////////
class CKinectTrackObjectDetector : public CKinectDetector
{

public:
	CKinectTrackObjectDetector(IplImage* pVideoFrame);
	~CKinectTrackObjectDetector();

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

	void Init();
	BOOL LockOn();
	BOOL GetDelta( CvPoint &ObjectDelta );


protected:


};
*/

