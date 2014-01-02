namespace RobotKinect.Speech
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Threading;
    using System.Diagnostics;
    using Microsoft.Kinect;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Synthesis;


    public class SpeechTalker //: IDisposable
    {

        private SpeechSynthesizer speech;

        public SpeechTalker()
        {
            speech = new SpeechSynthesizer(); // params not needed?
            //speech.SpeakCompleted += new EventHandler<SpeakCompletedEventArgs>(speech_SpeakCompleted);

            string text = "hi";
            //string language = "en";
            speech.Speak(text);    // This method returns immediately
            //speech.SpeakAsync(text);    // This method returns immediately 
        }

       public void SayThis(string TextToSay)
       {
            //string text = "Have a nice day!";
            //string language = "en";
           speech.Speak(TextToSay);

            //speech.SpeakAsync(TextToSay);  // This method returns immediately 
       }


 /*
        private static 
        void speech_SpeakCompleted(object sender, SpeakCompletedEventArgs e)
        {
            Console.WriteLine("Operation completed.");
        }
        */

	}
}


