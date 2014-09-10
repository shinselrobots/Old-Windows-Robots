// RobotCreativeDepth.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include <wchar.h>
#include <vector>
#include <tchar.h>

#include "pxcsession.h"
#include "pxcsmartptr.h"
#include "pxccapture.h"
#include "util_render.h"
#include "util_capture_file.h"
#include "util_cmdline.h"
#include "..\framework\common\pxcupipeline\include\pxcupipeline.h"

class MyPipeline: public UtilPipeline {

public:

   MyPipeline(void): UtilPipeline() {

       // Select a depth stream with resolution

       EnableImage(PXCImage::COLOR_FORMAT_DEPTH,320,240);

       // Set a filter to only allow 60FPS

       PXCSizeU32 size={0,0};

       QueryCapture()->SetFilter(PXCImage::IMAGE_TYPE_DEPTH,size,60);

 

   }

   virtual void OnImage(PXCImage *image) {

      // ...  // work on the image

   }

};

 


int _tmain(int argc, _TCHAR* argv[])
{
   MyPipeline pp;

   pp.LoopFrames();

   return 0;
}

