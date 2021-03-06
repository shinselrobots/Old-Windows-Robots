//------------------------------------------------------------------------------
// <copyright file="KinectDiagnosticViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
//#define AUTO_START_LOKI 

namespace KinectWpfViewers
{
    using System;
    using System.Collections.Generic;
    using System.IO;
    using System.Windows;
    using System.Windows.Controls;
    using System.IO.MemoryMappedFiles;
    using System.Threading;
    using System.Diagnostics;
    using Microsoft.Kinect;
    using RobotKinect.Speech;


    /// <summary>
    /// Interaction logic for KinectDiagnosticViewer.xaml
    /// </summary>
    /// 

    public enum MESSAGE_QUEUE_FLAGS
    {
        MESSAGE_QUEUE_OPEN_SLOT = 0,
        MESSAGE_QUEUE_WRITE_IN_PROGRESS,
        MESSAGE_QUEUE_MESSAGE_READY,
    };
    public enum MESSAGE_CMDS
    {
        SPEECH_CMD = 0,
        OTHER_CMD,
    };
    
 

    public partial class KinectDiagnosticViewer : UserControl
    {
        private readonly KinectSettings kinectSettings;
        private readonly Dictionary<KinectSensor, bool> sensorIsInitialized = new Dictionary<KinectSensor, bool>();
        private KinectSensor kinect;
        private bool kinectAppConflict;
        private SpeechRecognizer mySpeechRecognizer;
//        private SpeechTalker mySpeechTalker;aaaaa

        /////////////////////////////////////////////////////////////////////////
        // Create Memory Mapped File for sharing data with the Robot C++ program
        // SharedMemory Structure:

        //
        // File format is:
        // OUT: Descrete Data:
        //const int SHARED_MEMORY_CHANGED_FLAG_INDEX = 0;  // NOT USED, this first INT available for future use
        const int SPEECH_RECO_DISABLE_REQUEST_FLAG_INDEX = 4;
        const int DESCRETE_DATA_SIZE = (2 * 4); // 4 bytes per int

        // OUT: Message Queue:
        const int MESSAGE_QUEUE_OFFSET = DESCRETE_DATA_SIZE; // starts immediately after descrete data
        const int MESSAGE_QUEUE_FLAG_OFFSET = 0;  // flag
        const int MESSAGE_QUEUE_RECO_TYPE_OFFSET = 4;
        const int MESSAGE_QUEUE_RECO_PARAM1_OFFSET = 8;  
        const int MESSAGE_QUEUE_RECO_PARAM2_OFFSET = 12;   
        const int MESSAGE_QUEUE_RECO_PARAM3_OFFSET = 16;
        const int MESSAGE_QUEUE_RECO_PARAM4_OFFSET = 20;
        const int MESSAGE_QUEUE_RECO_CONFIDENCE_OFFSET = 24;     // Confidence
        const int MESSAGE_QUEUE_BEAM_DIRECTION_OFFSET = 28;     // Direction

        const int MESSAGE_QUEUE_STEP = 64; // total bytes: 8 ints each x 8 items 

        const int MESSAGE_QUEUE_MAX_MESSAGES = 4; // messages max
        const int MESSAGE_QUEUE_SIZE = (MESSAGE_QUEUE_STEP * MESSAGE_QUEUE_MAX_MESSAGES);

        // IN: Speech Input Queue (speak requests back from the C++ process code)
        //      128 bytes each * 16 messages max
        //const int SPEECH_INPUT_QUEUE_OFFSET = DESCRETE_DATA_SIZE + MESSAGE_QUEUE_SIZE;
        //const int SPEECH_INPUT_PHRASE_MAX_LENGTH = 128; // Max lenth of a phrase to say
        //const int SPEECH_INPUT_QUEUE_MAX_MESSAGES = 16; // messages max
        //const int SPEECH_INPUT_QUEUE_SIZE = (SPEECH_INPUT_PHRASE_MAX_LENGTH * SPEECH_INPUT_QUEUE_MAX_MESSAGES);

        const int MM_FILE_SIZE = DESCRETE_DATA_SIZE + MESSAGE_QUEUE_SIZE; // + SPEECH_INPUT_QUEUE_SIZE;


        private bool SharedDataFileInitialized = true;
        ///private int FrameLoopCount = 1;
        ///private int CopiedFrameNumber = 0; // number of frames copied to shared memory
        private string smName = "RobotKinectSpeechMappingFile";
        private Mutex smLock;
        private long smSize = MM_FILE_SIZE;
        private long offset = 0;
        private bool locked;
        private MemoryMappedFile mmf;
        private MemoryMappedViewAccessor accessor;
        private int MessageQueueNextSlot = 0; // next available slot in the message queue
        ///private int TestMessage = 0;
        private EventWaitHandle SpeechRecoWaitHandle;
        /////////////////////////////////////////////////////////////////////////

        private Process ChildProcess;
        private IntPtr ChildProcessHandle;
          

        public KinectDiagnosticViewer()
        {

            //SpeechTalker mySpeechTalker = new SpeechTalker();
           // mySpeechTalker.SayThis("Starting Up");

            /////////////////////////////////////////////////////////////////////////
            // Create Memory Mapped File for sharing data with the Robot C++ program
            try
            {
                // Create named MMF
                mmf = MemoryMappedFile.CreateOrOpen(smName, smSize);

                // Create accessors to MMF
                accessor = mmf.CreateViewAccessor(offset, smSize,
                                MemoryMappedFileAccess.ReadWrite);

                // Create lock
                smLock = new Mutex(true, "SM_LOCK", out locked);

                // Indicate to receiving process that there is no data to read
                //accessor.Write(SHARED_MEMORY_CHANGED_FLAG_INDEX, (int)0); // NOT USED
                accessor.Write(SPEECH_RECO_DISABLE_REQUEST_FLAG_INDEX, (int)0);

                // Initialize the Message Queue
                MessageQueueNextSlot = 0;
                for (int i = 0; i < MESSAGE_QUEUE_MAX_MESSAGES; i++)
                {
                    accessor.Write(
                        (MESSAGE_QUEUE_OFFSET + (MessageQueueNextSlot*MESSAGE_QUEUE_STEP) + MESSAGE_QUEUE_FLAG_OFFSET), 
                        (Int32)(MESSAGE_QUEUE_FLAGS.MESSAGE_QUEUE_OPEN_SLOT) );
                }
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
                SpeechRecoWaitHandle = new EventWaitHandle(bInitialState, EventResetMode.AutoReset, "LokiRobotSpeechRecoEvent");
                if (null == SpeechRecoWaitHandle)
                {
                    SharedDataFileInitialized = false;
                    Console.WriteLine("*** SpeechRecoWaitHandle creation Failed! ***");
                    MessageBox.Show("SpeechRecoWaitHandle creation Failed!");
                }
            }
            catch
            {
                SharedDataFileInitialized = false;
                Console.WriteLine("*** SpeechRecoWaitHandle creation Exception! ***");
                MessageBox.Show("SpeechRecoWaitHandle creation Exception!");
            }


            /////////////////////////////////////////////////////////////////////////
            // Launch the Robot C++ program
            // start the MFC application as a process in the .NET app
#if AUTO_START_LOKI

            // NOT USED. Instead the core robot C++ code starts this applicaiton
            ChildProcess = new Process();
            
            try
            {
                ChildProcess.StartInfo.UseShellExecute = false;
                ChildProcess.StartInfo.FileName = "C:\\Dev\\Robots\\Debug\\Loki.exe";
                //ChildProcess.StartInfo.CreateNoWindow = true;
                ChildProcess.Start();
                // This code assumes the process you are starting will terminate itself. 
                // Given that is is started without a window so you cannot terminate it 
                // on the desktop, it must terminate itself or you can do it programmatically
                // from this application using the Kill method.
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
#endif
            /////////////////////////////////////////////////////////////////////////


            InitializeComponent();
            this.kinectSettings = new KinectSettings(this);
            this.kinectSettings.PopulateComboBoxesWithFormatChoices();
            Settings.Content = this.kinectSettings;
            this.KinectColorViewer = this.colorViewer;

            // Just to make sure, signal the C++ app
            SpeechRecoWaitHandle.Set(); // Set the event
            this.StatusChanged();
        }

        ~KinectDiagnosticViewer()
        {
            if (SharedDataFileInitialized)
            {
                accessor.Dispose();
                mmf.Dispose();
                smLock.Close();
            }
        }

        public ImageViewer KinectColorViewer { get; set; }

         public KinectSensor Kinect
        {
            get
            {
                return this.kinect;
            }

            set
            {
                if (this.kinect != null)
                {
                    bool wasInitialized;
                    this.sensorIsInitialized.TryGetValue(this.kinect, out wasInitialized);
                    if (wasInitialized)
                    {
                        this.UninitializeKinectServices(this.kinect);
                        this.sensorIsInitialized[this.kinect] = false;
                    }
                }

                this.kinect = value;
                this.kinectSettings.Kinect = value;
                if (this.kinect != null)
                {
                    if (this.kinect.Status == KinectStatus.Connected)
                    {
                        this.kinect = this.InitializeKinectServices(this.kinect);

                        if (this.kinect != null)
                        {
                            this.sensorIsInitialized[this.kinect] = true;
                        }
                    }
                }

                this.StatusChanged(); // update the UI about this sensor
            }
        }

        public void StatusChanged()
        {
            if (this.kinectAppConflict)
            {
                _DisplayStatus.Text = "KinectAppConflict";
            }
            else if (this.Kinect == null)
            {
                _DisplayStatus.Text = "Kinect initialize failed";
            }
            else
            {
                this._DisplayStatus.Text = this.Kinect.Status.ToString();

                if (this.Kinect.Status == KinectStatus.Connected)
                {
                    // Update comboboxes' selected value based on stream isenabled/format.
                    this.kinectSettings.colorFormats.SelectedValue = this.Kinect.ColorStream.Format;
                    this.kinectSettings.depthFormats.SelectedValue = this.Kinect.DepthStream.Format;

                    // NOT SUPPORTED ON ROBOT (BASE REMOVED!) 
                    //      this.kinectSettings.UpdateUiElevationAngleFromSensor();

                    // Select Near mode if possible
                    try
                    {
                        this.Kinect.DepthStream.Range = DepthRange.Near; // Choose Near Mode by default
                    }
                    catch (InvalidOperationException)
                    {
                        // Oops, must be the XBox360 sensor, not the PC sensor.
                        Console.WriteLine("*** XBOX 360 SENSOR:  NEAR MODE DISABLED ***");
                    }

                    this.kinectSettings.trackingModes.SelectedValue = KinectSkeletonViewerOnDepth.TrackingMode;

                }
            }
        }

        // Kinect enabled apps should customize which Kinect services it initializes here.
        private KinectSensor InitializeKinectServices(KinectSensor sensor)
        {
            // Centralized control of the formats for Color/Depth and enabling skeletalViewer
            // DAVES: ENABLE THIS FOR COLOR VIDEO: 
            //sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            //this.DisplayColumnBasedOnIsChecked(false, 1, 2);
            //this.DisplayPanelBasedOnIsChecked(false, this.DiagViewer.colorPanel);
            this.LayoutRoot.ColumnDefinitions[1].Width = new GridLength(0);
            //this.panel.Visibility = Visibility.Collapsed;


            sensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30); // Resolution320x240Fps30 or SLOW! Resolution640x480Fps30
            this.Kinect.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated; // Choose Seated mode by default                        
            this.kinectSettings.SkeletonStreamEnable.IsChecked = true; // will enable SkeletonStream if available
            sensor.SkeletonStream.Enable();
            
            // Inform the viewers of the Kinect KinectSensor.
            this.KinectColorViewer.Kinect = sensor;
            KinectDepthViewer.Kinect = sensor;
            KinectSkeletonViewerOnColor.Kinect = sensor;
            KinectSkeletonViewerOnDepth.Kinect = sensor;
            kinectAudioViewer.Kinect = sensor;

            // Start streaming
            try
            {
                sensor.Start();
                this.kinectAppConflict = false;
            }
            catch (IOException)
            {
                this.kinectAppConflict = true;
                return null;
            }

            /*
            ////////////////////////////////////////////////
            // Get child window handle to send messages
            try
            {
                if (ChildProcess.WaitForInputIdle())
                {
                    while ( 0 == ChildProcess.MainWindowHandle.ToInt32() )
                    {
                        System.Threading.Thread.Sleep(200);
                    }

                    ChildProcessHandle = ChildProcess.MainWindowHandle;
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
*/

            //sensor.AudioSource.Start();
            //var audioSource = sensor.AudioSource;
            //audioSource.BeamAngleMode = BeamAngleMode.Adaptive;

            // Start speech recognizer after KinectSensor.Start() is called
            // returns null if problem with speech prereqs or instantiation.
            this.mySpeechRecognizer = SpeechRecognizer.Create();
            //this.mySpeechRecognizer.SetProcessHandle(ChildProcessHandle);
            this.mySpeechRecognizer.SetProcessInfo(ChildProcess);
            this.mySpeechRecognizer.SaidSomething += this.RecognizerSaidSomething;
            this.mySpeechRecognizer.Start(sensor.AudioSource);

            // ***** ENABLE AEC HERE!!! ****
            this.kinect.AudioSource.EchoCancellationMode = EchoCancellationMode.CancellationAndSuppression; // or EchoCancellationMode.None

            return sensor;
        }

        // Kinect enabled apps should uninitialize all Kinect services that were initialized in InitializeKinectServices() here.
        private void UninitializeKinectServices(KinectSensor sensor)
        {
            //sensor.AudioSource.Stop();

            // Stop streaming
            sensor.Stop();

            if (this.mySpeechRecognizer != null)
            {
                this.mySpeechRecognizer.Stop();
                this.mySpeechRecognizer.SaidSomething -= this.RecognizerSaidSomething;
                this.mySpeechRecognizer.Dispose();
                this.mySpeechRecognizer = null;
            }

 ///           enableAec.Visibility = Visibility.Collapsed;


            // Inform the viewers that they no longer have a Kinect KinectSensor.
            this.KinectColorViewer.Kinect = null;
            KinectDepthViewer.Kinect = null;
            KinectSkeletonViewerOnColor.Kinect = null;
            KinectSkeletonViewerOnDepth.Kinect = null;
            kinectAudioViewer.Kinect = null;

            // Disable skeletonengine, as only one Kinect can have it enabled at a time.
            if (sensor.SkeletonStream != null)
            {
                sensor.SkeletonStream.Disable();
            }
        }

 
        #region Kinect Speech processing
        private void RecognizerSaidSomething(object sender, SpeechRecognizer.SaidSomethingEventArgs EventArgs)
        {

            if (EventArgs.WaitingForName)
            {
                // Loki's name not heard in a while.  Update the GUI
                this._DisplayStatus.Text = "  WAITING FOR NAME ";
                return;
            }

            Console.Write("*** RECOGNIZED: "); Console.Write(EventArgs.Phrase); Console.WriteLine();
            string sConf = string.Format("({0:0.00})  ", EventArgs.Confidence);
            this._DisplayStatus.Text = "  Recognized: " + sConf + EventArgs.Phrase;

            float BeamAngle =  (float)kinectAudioViewer.BeamAngleInDegrees;
            float AudioAngle = (float)kinectAudioViewer.SoundSourceAngleInDegrees;


            //////////////////////////////////////////////////////////////////////////////////
            // Save to Memory Mapped
            if (SharedDataFileInitialized)
            {
                // Write the info to shared memory
                //accessor.Write(SPEECH_RECO_ITEM_INDEX, (Int32)EventArgs.Cmd);
                //accessor.Write(SPEECH_RECO_CONFIDENCE_INDEX, (float)EventArgs.Confidence);
                //accessor.Write(AUDIO_BEAM_DIRECTION_INDEX, (float)BeamAngle);
                //accessor.Write(AUDIO_RAW_DIRECTION_INDEX, (float)AudioAngle);

                // Test WM_ message slots by writing the slot number as the message number

                bool messageWritten = false;
                for (int MessageQueueSlot = 0; MessageQueueSlot < MESSAGE_QUEUE_MAX_MESSAGES; MessageQueueSlot++)
                {
                    // Find the first available slot.  Receiving queue will look in same order.
                    int MessageSlotOffset = MESSAGE_QUEUE_OFFSET + (MessageQueueSlot * MESSAGE_QUEUE_STEP);
                    Int32 MessageQueueReadFlag = accessor.ReadInt32(MessageSlotOffset + MESSAGE_QUEUE_FLAG_OFFSET);

                    if ((Int32)(MESSAGE_QUEUE_FLAGS.MESSAGE_QUEUE_OPEN_SLOT) != MessageQueueReadFlag)
                    {
                        // slot full, try the next one
                        continue;
                    }
                    else
                    {
                        Console.WriteLine("*** DEBUG: writing to SLOT {0} ***", MessageQueueSlot);

                        // Flag that we are writing to the Message Queue
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_FLAG_OFFSET), (Int32)(MESSAGE_QUEUE_FLAGS.MESSAGE_QUEUE_WRITE_IN_PROGRESS));

                        // Write the message
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_TYPE_OFFSET), (Int32)(EventArgs.RecoType));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_PARAM1_OFFSET), (Int32)(EventArgs.Param1));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_PARAM2_OFFSET), (Int32)(EventArgs.Param2));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_PARAM3_OFFSET), (Int32)(EventArgs.Param3));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_PARAM4_OFFSET), (Int32)(EventArgs.Param4));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_RECO_CONFIDENCE_OFFSET), (float)(EventArgs.Confidence));
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_BEAM_DIRECTION_OFFSET), (float)(BeamAngle));

                        // Flag that we are done writing the Message to the Queue
                        accessor.Write((MessageSlotOffset + MESSAGE_QUEUE_FLAG_OFFSET), (Int32)(MESSAGE_QUEUE_FLAGS.MESSAGE_QUEUE_MESSAGE_READY));
                        messageWritten = true;
                        // Indicate that there is data to read
                        //accessor.Write(SHARED_MEMORY_CHANGED_FLAG_INDEX, (int)1); // NOT USED
                        SpeechRecoWaitHandle.Set(); // Set the event
                        Console.WriteLine("*** DEBUG: writing to SLOT {0} *** DONE", MessageQueueSlot);
                        break;
                    }
                }
                if (!messageWritten)
                {
                    Console.WriteLine("*** BAD MESSAGE QUEUE SLOT! ALL SLOTS FULL? Message Discarded ***");
                    //MessageBox.Show("Bad Message Queue Slot! (all slots full?");                   
                }

            }

        }

        #endregion Kinect Speech processing

        private void UserControl_Loaded(object sender, RoutedEventArgs e)
        {

        }
/*
        private void DepthMouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {

            Point position = System.Windows.Input.Mouse.GetPosition(this);
//            Point position = System.Windows.Input.Mouse.GetPosition(Window.GetWindow(this));
            Console.WriteLine("MOUSE DOWN AT: x= {0}, y= {1}", position.X, position.Y);

//            Console.WriteLine("MOUSE DOWN!");

        }
 */

    }
}
