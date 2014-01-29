//------------------------------------------------------------------------------
// <copyright file="KinectDepthViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
#define SHOW_KINECT_SERVO_CALIBRATION_LINE


namespace KinectWpfViewers
{
    using System;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
//    using NetSharedMemory;

    using System.IO;
    using System.Collections.Generic;
    using System.IO.MemoryMappedFiles;
    using System.Threading;

    /// <summary>
    /// Interaction logic for KinectDepthViewer.xaml
    /// </summary>
    /// 

    public partial class KinectDepthViewer : ImageViewer
    {

        // color divisors for tinting depth pixels
        private static readonly int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
        private static readonly int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
        private static readonly int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

        private const int RedIndex = 2;
        private const int GreenIndex = 1;
        private const int BlueIndex = 0;
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        private DepthImageFormat lastImageFormat;
        private short[] pixelData;

        // We want to control how depth data gets converted into false-color data
        // for more intuitive visualization, so we keep 32-bit color frame buffer versions of
        // these, to be updated whenever we receive and process a 16-bit frame.
        private byte[] depthFrame32;
        private WriteableBitmap outputBitmap;

        /////////////////////////////////////////////////////////////////////////
        // Memory Mapped File for sharing Depth data with the Robot C++ program
        const int mmfOffsetControlFlags =     0;
        const int mmfOffsetBoundingBoxTop =      4;
        const int mmfOffsetBoundingBoxBottom =   8;
        const int mmfOffsetBoundingBoxLeft =    12;
        const int mmfOffsetBoundingBoxRight =   16;
        const int mmfOffsetFrameNumber =        20;
        const int mmfOffsetHeight =             24;
        const int mmfOffsetWidth =              28;
        const int mmfOffsettooFarDepth =        32;
        const int mmfOffsetMouseDown =          36;
        const int mmfOffsetMouseX =             40;
        const int mmfOffsetMouseY =             44;
        const int mmfOffsetBitmapStart =        48;

        const int MAX_SHARED_DATA_SIZE = (640 * 480 * 2) + mmfOffsetBitmapStart + 4; // 2 bytes per Int16
        private bool SharedDataFileInitialized = true;
        private int FrameLoopCount = 1;
        private int CopiedFrameNumber = 0; // number of frames copied to shared memory
        private string smName = "RobotKinectDepthMappingFile";
        private Mutex smLock;
        private long smSize = MAX_SHARED_DATA_SIZE;
        private long offset = 0;
        private bool locked;
        private MemoryMappedFile mmf;
        private MemoryMappedViewAccessor accessor;
        private EventWaitHandle KinectDepthWaitHandle;

        // mmfOffsetControlFlags (This must match C++ code)
        const int ControlFlag_None = 0;
        const int ControlFlag_DisplayBoundingBox = 1;
        const int ControlFlag_HidePlayers = 2;
        const int ControlFlag_All = 3;

        private bool DrawBoundingBox = false;
        private bool HidePlayers = false;
        private int BoundingBoxTop = 0;
        private int BoundingBoxBottom = 0;
        private int BoundingBoxLeft = 0;
        private int BoundingBoxRight = 0;       

        public KinectDepthViewer()
        {

            /////////////////////////////////////////////////////////////////////////
            // Create Memory Mapped File for sharing data with the Robot C++ program
            //
            // File format is:
            // OUT:
            //   Message Queue:
            //      3 ints each (12 bytes) * 16 messages max
            // IN:
            //   Speech Queue (speak requests back from the C++ process code)
            //      128 bytes each * 16 messages max

            try
            {
                // Create named MMF
                mmf = MemoryMappedFile.CreateOrOpen(smName, smSize);

                // Create accessors to MMF
                accessor = mmf.CreateViewAccessor(offset, smSize,
                                MemoryMappedFileAccess.ReadWrite);

                // Create lock
                smLock = new Mutex(true, "SM_LOCK", out locked);
            }
            catch
            {
                SharedDataFileInitialized = false;
                Console.WriteLine("*** Shared Memory Open Failed! ***");
                MessageBox.Show("Shared Memory Open Failed!");
            }

            // Create an event for synchronizing between the C# and C++ apps
            try
            {
                bool bInitialState = true; // signaled, to indicate the process is up
                KinectDepthWaitHandle = new EventWaitHandle(bInitialState, EventResetMode.AutoReset, "LokiRobotKinectDepthReadyEvent");
                if (null == KinectDepthWaitHandle)
                {
                    SharedDataFileInitialized = false;
                    Console.WriteLine("*** KinectDepthWaitHandle creation Failed! ***");
                    MessageBox.Show("KinectDepthWaitHandle creation Failed!");
                }
            }
            catch
            {
                SharedDataFileInitialized = false;
                Console.WriteLine("*** KinectDepthWaitHandle creation Exception! ***");
                MessageBox.Show("KinectDepthWaitHandle creation Exception!");
            }

           InitializeComponent();
        }

        ~KinectDepthViewer()
        {
            if (SharedDataFileInitialized)
            {
                accessor.Dispose();
                mmf.Dispose();
                smLock.Close();
            }
        }

        protected override void OnKinectChanged(KinectSensor oldKinectSensor, KinectSensor newKinectSensor)
        {
            if (oldKinectSensor != null)
            {
                oldKinectSensor.DepthFrameReady -= this.DepthImageReady;
                kinectDepthImage.Source = null;
                this.lastImageFormat = DepthImageFormat.Undefined;
            }

            if (newKinectSensor != null && newKinectSensor.Status == KinectStatus.Connected)
            {
                ResetFrameRateCounters();

                newKinectSensor.DepthFrameReady += this.DepthImageReady;
            }

        }

        private void DepthImageReady(object sender, DepthImageFrameReadyEventArgs e)
        {

            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    // We need to detect if the format has changed.
                    bool haveNewFormat = this.lastImageFormat != imageFrame.Format;

                    if (haveNewFormat)
                    {
                        this.pixelData = new short[imageFrame.PixelDataLength];
                        this.depthFrame32 = new byte[imageFrame.Width * imageFrame.Height * Bgr32BytesPerPixel];

                        if (imageFrame.PixelDataLength > (640 * 480))
                        {
                            SharedDataFileInitialized = false;
                            Console.WriteLine("*** Shared Memory ERROR: PixelDataLength too big ***");
                            MessageBox.Show("Shared Memory ERROR: PixelDataLength too big!");
                            return;
                        }
                    }

                    bool bWriteFrameToSharedMemory = false;
                    if (FrameLoopCount++ >= 15)
                    {
                        bWriteFrameToSharedMemory = true; // tell ConvertDepthFrame to write the bitmap data to shared memory
                        FrameLoopCount = 1;
                    }


                    imageFrame.CopyPixelDataTo(this.pixelData);

                    byte[] convertedDepthBits = this.ConvertDepthFrame( bWriteFrameToSharedMemory, imageFrame.Height, imageFrame.Width, imageFrame.PixelDataLength, 
                                                                        this.pixelData, ((KinectSensor)sender).DepthStream);

                    // A WriteableBitmap is a WPF construct that enables resetting the Bits of the image.
                    // This is more efficient than creating a new Bitmap every frame.
                    if (haveNewFormat)
                    {
                        this.outputBitmap = new WriteableBitmap(
                            imageFrame.Width, 
                            imageFrame.Height, 
                            96,  // DpiX
                            96,  // DpiY
                            PixelFormats.Bgr32, 
                            null);

                        this.kinectDepthImage.Source = this.outputBitmap;
                    }

                    this.outputBitmap.WritePixels(
                        new Int32Rect(0, 0, imageFrame.Width, imageFrame.Height), 
                        convertedDepthBits,
                        imageFrame.Width * Bgr32BytesPerPixel,
                        0);

                    this.lastImageFormat = imageFrame.Format;

                    UpdateFrameRate();
                }
            }
        }

        // Converts a 16-bit grayscale depth frame which includes player indexes into a 32-bit frame
        // that displays different players in different colors
        private byte[] ConvertDepthFrame(bool bWriteFrameToSharedMemory, int Height, int Width, int PixelDataLength, 
                                         short[] depthFrame, DepthImageStream depthStream)
        {
            int tooNearDepth = depthStream.TooNearDepth;
            int tooFarDepth = depthStream.TooFarDepth;
            int unknownDepth = depthStream.UnknownDepth;

            Point[] PlayerLocationTracking = new Point[16];
            for (int i = 0; i < 16; i++)
            {
                PlayerLocationTracking[i].X = 0;
                PlayerLocationTracking[i].Y = 0;
            }

            //////////////////////////////////////////////////////////////////////////////////
            // Get BoundingBox for object that the C++ code wants to highlight
            // bWriteFrameToSharedMemory says Don't read/write every frame, just every "N" frames
            if (SharedDataFileInitialized && bWriteFrameToSharedMemory)
            {
                int ControlFlags = (int)accessor.ReadInt64(mmfOffsetControlFlags);
                // 1 = Display Bounding Box
                // 2 = Hide Players
                // 3 = Do both

                // See if C++ app as requested a bounding box to be displayed over the depth image
                if ((ControlFlag_DisplayBoundingBox == ControlFlags) || (ControlFlag_All == ControlFlags))
                {
                    BoundingBoxTop = (int)accessor.ReadInt64(mmfOffsetBoundingBoxTop);
                    BoundingBoxBottom = (int)accessor.ReadInt64(mmfOffsetBoundingBoxBottom);
                    BoundingBoxLeft = (int)accessor.ReadInt64(mmfOffsetBoundingBoxLeft);
                    BoundingBoxRight = (int)accessor.ReadInt64(mmfOffsetBoundingBoxRight);
                    DrawBoundingBox = true;
                }
                else
                {
                    DrawBoundingBox = false;
                }

                // See if C++ app as requested Players to not be shown in the depth image
                if ((ControlFlag_HidePlayers == ControlFlags) || (ControlFlag_All == ControlFlags))
                {
                    HidePlayers = true;
                }
                else
                {
                    HidePlayers = false;
                }
            }


            int lastValue = 0;
            #if SHOW_KINECT_SERVO_CALIBRATION_LINE
                //const int TargetLinePad = 80;
                const int TargetLineGapPad = 3;
                const int TargetLineWidth = 50;
                int CenterRowStart = ((Height * Width) / 2) + ((Width / 2) - TargetLineWidth);
                int CenterRowEnd = CenterRowStart + (TargetLineWidth * 2);
                int CenterRowGapStart = CenterRowStart + (TargetLineWidth - TargetLineGapPad);
                int CenterRowGapEnd = CenterRowEnd - (TargetLineWidth - TargetLineGapPad);

/*
                const int TargetLinePad = 80;
                const int TargetLineGapPad = 75;
                int CenterRowStart = ((Height * Width) / 2) + TargetLinePad + 1;
                int CenterRowEnd = (CenterRowStart + Width) - (TargetLinePad * 2);
                int CenterRowGapStart = CenterRowStart + TargetLineGapPad;
                int CenterRowGapEnd = CenterRowEnd - TargetLineGapPad;
 */

            #endif
            //////////////////////////////////////////////////////////////////////////////////
            // Save Memory Mapped Shared data Header
            // bWriteFrameToSharedMemory says Don't write every frame, just every "N" frames
            if (SharedDataFileInitialized && bWriteFrameToSharedMemory)
            {
                CopiedFrameNumber++;
                if (CopiedFrameNumber >= 0xFFFE) CopiedFrameNumber = 0;

                // Write the header info to shared memory
                accessor.Write(mmfOffsetFrameNumber, CopiedFrameNumber);
                accessor.Write(mmfOffsetHeight, Height);
                accessor.Write(mmfOffsetWidth, Width);
                accessor.Write(mmfOffsettooFarDepth, tooFarDepth);
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Loop through all the depth values
            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < this.depthFrame32.Length; i16++, i32 += 4)
            {
                int rawValue = depthFrame[i16]; // Data as stored in the bitmap, including player data
                int player = depthFrame[i16] & DepthImageFrame.PlayerIndexBitmask;
                int realDepth = (depthFrame[i16] >> DepthImageFrame.PlayerIndexBitmaskWidth); // Depth value in millimeters
                lastValue = realDepth;


                //////////////////////////////////////////////////////////////////////////////////
                // Save Depth data in mm to share data with the C++ app for creating a point cloud
                if (SharedDataFileInitialized && bWriteFrameToSharedMemory)
                {
                    /* FOR DEBUG
                    if (i16 == 0)
                    {
                        realDepth = 26;
                    }
                    if (i16 < 32)
                    {
                        Console.Write(realDepth); Console.Write(", ");
                    }
                    */ 
                    // write out the 16 bit depth data
                    accessor.Write((mmfOffsetBitmapStart + (i16*2)), rawValue);
                }


                //////////////////////////////////////////////////////////////////////////////////
                // Display Depth image                
                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(~(realDepth >> 4));

                #if SHOW_KINECT_SERVO_CALIBRATION_LINE

                    // DEBUG: show horizontal line for calibrating servo
                    if ( ((i16 > CenterRowStart) && (i16 < CenterRowGapStart)) ||
                         ((i16 > CenterRowGapEnd) && (i16 < CenterRowEnd))      )
                    {
                        // Draw a center line on the display
                        this.depthFrame32[i32 + RedIndex] = (byte)(255);
                        this.depthFrame32[i32 + GreenIndex] = (byte)(255);
                        this.depthFrame32[i32 + BlueIndex] = (byte)(255);
                    }
                    else 
                #endif
                if ( (0 != player) && (!HidePlayers) )
                {
                    // Use solid colors for human players!
                    // tint the intensity by dividing by per-player values
                    this.depthFrame32[i32 + RedIndex] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                    this.depthFrame32[i32 + GreenIndex] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                    this.depthFrame32[i32 + BlueIndex] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                }
                else
                {
                    if (realDepth == 0)
                    {
                        // white 
                        this.depthFrame32[i32 + RedIndex] = 255;
                        this.depthFrame32[i32 + GreenIndex] = 255;
                        this.depthFrame32[i32 + BlueIndex] = 255;
                    }
                    else if (realDepth == tooFarDepth)
                    {
                        // dark brown
                        this.depthFrame32[i32 + RedIndex] = 66;
                        this.depthFrame32[i32 + GreenIndex] = 66;
                        this.depthFrame32[i32 + BlueIndex] = 33;
                    }
                    else if (realDepth == unknownDepth)
                    {
                        // black
                        this.depthFrame32[i32 + RedIndex] = 0;
                        this.depthFrame32[i32 + GreenIndex] = 0;
                        this.depthFrame32[i32 + BlueIndex] = 0;
                    }
                    else
                    {
                        // Show color Gradiant
                        // DAVES: Uses my unique method for color depth visualization
                        int red = 0; int blue = 0; int green = 0;
                        int DepthValueMM = realDepth;	// Depth value in millimeters


                        int ScaledValue = (DepthValueMM / 4) - 90; // scale, and subtract out min range (47 ScaledValue)
                        //int ScaledValue = (int)(((double)DepthValueMM / ScaleFactor) - 90.0); // scale, and subtract out min range (47 ScaledValue)
                        if (ScaledValue < 0) ScaledValue = 0;
                        int modu = ScaledValue % 128;

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


                        this.depthFrame32[i32 + RedIndex] = (byte)(red);
                        this.depthFrame32[i32 + GreenIndex] = (byte)(green);
                        this.depthFrame32[i32 + BlueIndex] = (byte)(blue);
                    }
                }
            }
            // Indicate to the C++ process that the frame is ready
            if (SharedDataFileInitialized && bWriteFrameToSharedMemory)
            {
                KinectDepthWaitHandle.Set(); // Set the event
            }
            // Decorate the bitmap before display
            // draw bounding box, if any
            // DEBUG:::
            //BoundingBoxTop = 100;
            //BoundingBoxBottom = 150;
            //BoundingBoxLeft = 140;
            //BoundingBoxRight = 180;
            //DrawBoundingBox = true;

            if( DrawBoundingBox )
            {
                for (int x = BoundingBoxLeft; x < BoundingBoxRight; x++)
                {
                    // Top line
                    int TopRowPixel = ((Width * BoundingBoxTop) + x) * 4;   // 4 bytes per int
                    this.depthFrame32[TopRowPixel + RedIndex] = 64;
                    this.depthFrame32[TopRowPixel + GreenIndex] = 255;
                    this.depthFrame32[TopRowPixel + BlueIndex] = 64;

                    // Bottom line
                    int BottomRowPixel = ((Width * BoundingBoxBottom) + x) * 4;
                    this.depthFrame32[BottomRowPixel + RedIndex] = 64;
                    this.depthFrame32[BottomRowPixel + GreenIndex] = 255;
                    this.depthFrame32[BottomRowPixel + BlueIndex] = 64;
                }
            
                //Draw the sides of the box
                for (int y = BoundingBoxTop; y < BoundingBoxBottom; y++)
                {
                    // Left line
                    int LeftPixel = ((Width * y) + BoundingBoxLeft) * 4;   // 4 bytes per int
                    this.depthFrame32[LeftPixel + RedIndex] = 64;
                    this.depthFrame32[LeftPixel + GreenIndex] = 255;
                    this.depthFrame32[LeftPixel + BlueIndex] = 64;

                    // Right line
                    int RightPixel = ((Width * y) + BoundingBoxRight) * 4;
                    this.depthFrame32[RightPixel + RedIndex] = 64;
                    this.depthFrame32[RightPixel + GreenIndex] = 255;
                    this.depthFrame32[RightPixel + BlueIndex] = 64;
                }
            }

            return this.depthFrame32;
        }

        private void DepthMouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Point position = System.Windows.Input.Mouse.GetPosition(this);
            Int32 MouseX = (Int32)(position.X+.5);
            Int32 MouseY = (Int32)(position.Y+.5);

            Console.WriteLine("MOUSE DOWN AT: x= {0}, y= {1}", MouseX, MouseY);

            if (SharedDataFileInitialized)
            {
                // Write Mouse info to shared memory
                // this gets read the next time the C++ app gets a depth frame
                accessor.Write(mmfOffsetMouseDown, (Int32)1);
                accessor.Write(mmfOffsetMouseX, MouseX);
                accessor.Write(mmfOffsetMouseY, MouseY);
            }

        }
    }
}
