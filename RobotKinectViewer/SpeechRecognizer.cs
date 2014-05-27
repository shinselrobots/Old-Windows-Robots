//------------------------------------------------------------------------------
// <copyright file="SpeechRecognizer.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// This module provides sample code used to demonstrate the use
// of the KinectAudioSource for speech recognition in a game setting.

// IMPORTANT: This code requires the Speech Platform SDK (v11) to be installed on the developer workstation
#define ROBOT_TYPE_LOKI

namespace RobotKinect.Speech
{
    using System;
    using System.IO; // for grxml
    using Microsoft.Speech.Recognition.SrgsGrammar;
    using System.Collections.Generic;
    using System.Linq;
    using System.Threading;
    using System.Diagnostics;
    using Microsoft.Kinect;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using System.Speech.Synthesis;

    using System.Net;
    using System.Net.Mail;
    using System.Net.Mime;

    public class SpeechRecognizer : IDisposable
    {
#if (ROBOT_TYPE_LOKI )
        string RobotName = @"loki";
#else
        string RobotName = @"drone";
#endif
        private readonly Dictionary<string, int> CommandPhrase = new Dictionary<string, int>
            {
                { "ROBOT_NAME",         (int)SPEECH_CMD.SpeechCmd_RobotName },
                { "STOP",               (int)SPEECH_CMD.SpeechCmd_Stop },
                { "WAVE",               (int)SPEECH_CMD.SpeechCmd_Wave },
                { "SAY_HI",             (int)SPEECH_CMD.SpeechCmd_SayHi },
                { "SAY_BYE",            (int)SPEECH_CMD.SpeechCmd_SayBye },
                { "SEND_EMAIL",         (int)SPEECH_CMD.SpeechCmd_SendEmail },
                { "TAKE_PHOTO",         (int)SPEECH_CMD.SpeechCmd_TakePhoto },
                { "DO_INTRO",           (int)SPEECH_CMD.SpeechCmd_DoIntro },
                { "LOOK_FWD",           (int)SPEECH_CMD.SpeechCmd_LookForward },
                { "FOLLOW",             (int)SPEECH_CMD.SpeechCmd_Follow },
                { "COME_HERE",          (int)SPEECH_CMD.SpeechCmd_ComeHere },
                { "TURN_TOWARDS_ME",    (int)SPEECH_CMD.SpeechCmd_TurnTowardsMe },
                { "DO_WHAT_TIME",       (int)SPEECH_CMD.SpeechCmd_DoWhatTime },
                { "FACE_ME",            (int)SPEECH_CMD.SpeechCmd_FaceMe },
                { "CLEAN_UP",           (int)SPEECH_CMD.SpeechCmd_CleanUp },
                { "SHAKE",              (int)SPEECH_CMD.SpeechCmd_Shake },
                { "MEET_NAME",          (int)SPEECH_CMD.SpeechCmd_MeetName },
                { "WAKE",               (int)SPEECH_CMD.SpeechCmd_Wake },
                { "MIC",                (int)SPEECH_CMD.SpeechCmd_Mic },
                { "LIGHTS",             (int)SPEECH_CMD.SpeechCmd_Lights },
                { "MOVE",               (int)SPEECH_CMD.SpeechCmd_Move },
                { "SMALL_MOVE",         (int)SPEECH_CMD.SpeechCmd_SmallMove },
                { "EXPLORE",            (int)SPEECH_CMD.SpeechCmd_Explore },
                { "SPIN",               (int)SPEECH_CMD.SpeechCmd_Spin },
                { "TURN",               (int)SPEECH_CMD.SpeechCmd_Turn },
                { "SMALL_TURN",         (int)SPEECH_CMD.SpeechCmd_SmallTurn },
                { "MOTORS",             (int)SPEECH_CMD.SpeechCmd_EnableMotors },
                { "AVOIDANCE",          (int)SPEECH_CMD.SpeechCmd_EnableAvoidance },
                { "ARM_HOME",           (int)SPEECH_CMD.SpeechCmd_ArmHome },
                { "ARM_UP",             (int)SPEECH_CMD.SpeechCmd_ArmUp },
                { "EXTEND_ARM",         (int)SPEECH_CMD.SpeechCmd_ExtendArm },
                { "CLAW",               (int)SPEECH_CMD.SpeechCmd_Claw },
                { "TAKE_OBJ",           (int)SPEECH_CMD.SpeechCmd_TakeObject },
                { "GIVE_OBJ",           (int)SPEECH_CMD.SpeechCmd_GiveObject },
                { "PICK_UP_OBJ",        (int)SPEECH_CMD.SpeechCmd_PickUpObject },
                { "PUT_OBJ_DOWN",       (int)SPEECH_CMD.SpeechCmd_PutObjectDown },
                { "PUT_IN_BASKET",      (int)SPEECH_CMD.SpeechCmd_PutInBasket },
                { "THROW_OBJECT",       (int)SPEECH_CMD.SpeechCmd_ThrowObject },
                { "SCRATCH_HEAD",       (int)SPEECH_CMD.SpeechCmd_ScratchHead },
                { "SCRATCH_BACK",       (int)SPEECH_CMD.SpeechCmd_ScratchBack },
                { "MEANING_OF_LIFE",    (int)SPEECH_CMD.SpeechCmd_MeaningOfLife },
                { "KARATE",             (int)SPEECH_CMD.SpeechCmd_Karate },
                { "DO_DANGER",          (int)SPEECH_CMD.SpeechCmd_DoDanger },
                { "LIGHT_SABER",        (int)SPEECH_CMD.SpeechCmd_LightSaber },
                { "POINT_COMPASS",      (int)SPEECH_CMD.SpeechCmd_PointCompass },
                { "IDENTIFY_OBJ",       (int)SPEECH_CMD.SpeechCmd_IdentifyObj },
                { "GOTO_LOCATION",      (int)SPEECH_CMD.SpeechCmd_GotoLocation },
                { "YES",                (int)SPEECH_CMD.SpeechCmd_Yes },
                { "NO",                 (int)SPEECH_CMD.SpeechCmd_No },
                { "JOKE",               (int)SPEECH_CMD.SpeechCmd_TellJoke },
                { "BAD_ROBOT",          (int)SPEECH_CMD.SpeechCmd_BadRobot },
                
            };

         private readonly Dictionary<string, int> SpeechParam = new Dictionary<string, int>
            {
                { "TRUE",               (int)SPEECH_PARAM.SpeechParam_True },
                { "FALSE",              (int)SPEECH_PARAM.SpeechParam_False },
                { "FORWARD",            (int)SPEECH_PARAM.SpeechParam_Forward },
                { "REVERSE",            (int)SPEECH_PARAM.SpeechParam_Reverse },
                { "ENABLE",             (int)SPEECH_PARAM.SpeechParam_True },
                { "DISABLE",            (int)SPEECH_PARAM.SpeechParam_False },
                { "LEFT",               (int)SPEECH_PARAM.SpeechParam_Left },
                { "RIGHT",              (int)SPEECH_PARAM.SpeechParam_Right },
                { "BOTH",               (int)SPEECH_PARAM.SpeechParam_Both },
                { "OPEN",               (int)SPEECH_PARAM.SpeechParam_Open },
                { "CLOSE",              (int)SPEECH_PARAM.SpeechParam_Close },
                { "NORTH",              (int)SPEECH_PARAM.SpeechParam_North },
                { "EAST",               (int)SPEECH_PARAM.SpeechParam_East },
                { "SOUTH",              (int)SPEECH_PARAM.SpeechParam_South },
                { "WEST",               (int)SPEECH_PARAM.SpeechParam_West },
                { "OFFICE",             (int)SPEECH_PARAM.SpeechParam_Office },
                { "MASTER_BEDROOM",     (int)SPEECH_PARAM.SpeechParam_MasterBedroom },
                { "HEATHERS_ROOM",      (int)SPEECH_PARAM.SpeechParam_HeatherRoom },
                { "AMBERS_ROOM",        (int)SPEECH_PARAM.SpeechParam_AmberRoom },
                { "MASTER_BATH",        (int)SPEECH_PARAM.SpeechParam_MasterBath },
                { "SINGLE",             (int)SPEECH_PARAM.SpeechParam_Single },
                { "MULTIPLE",           (int)SPEECH_PARAM.SpeechParam_Multiple },

            };

/*
        private readonly Dictionary<string, int> ChatParam = new Dictionary<string, int>
            {
                { "WHO_IS",             (int)CHAT_PARAM.Chat_WhoIs },
                { "WHAT_IS",            (int)CHAT_PARAM.Chat_WhatIs },
                { "WHAT_IS_FAVORITE",   (int)CHAT_PARAM.Chat_WhatIsFavorite },
                { "DO_YOU_LIKE",        (int)CHAT_PARAM.Chat_DoYouLike },
                { "YOU",                (int)CHAT_PARAM.Chat_You },
                { "ME",                 (int)CHAT_PARAM.Chat_Me },

            };


        private readonly Dictionary<string, int> YesNoPhrases = new Dictionary<string, int>
            {
                { "Yes",                (int)1 },
                { "No",                 (int)0 },
            };
*/
#if (ROBOT_TYPE_LOKI )
         private readonly Dictionary<string, string> ChatResponse = new Dictionary<string, string>
            {
                { "1 thank you",                      "you're welcome" },
                { "2 thank you",                      "no pro blemo" },
                { "3 thank you",                      "you're welcome" },
                { "1 say hi",                         "hello" },
                { "1 say bye",                        "goodbye" },
                { "1 who is you dog",                 "my dog is candy" },
                { "2 who is you dog",                 "i have 2 dogs, candy and kallie" },
                { "1 who is you mom",                 "my mom is lin" },
                { "1 who is you dad",                 "dave made me" },
                { "1 what is you favorite movie",    "my favorite movie is star wars" },
                { "1 do you like food_liked",        "yes, I would like that, if I had a mouth" },
                { "1 do you like food_hated",        "no, I dont like that" },
                { "1 what is you favorite color",   "my favorite color is blue" },
                { "1 what is you favorite show",    "robot cumbat league, of course" },


                /* TODO: ITEMS REQUIRING SENSOR OR OTHER DATA
                 * 
                { "1 what color pants me wearing",                 "" },
                 */
            };

#else
        
        private readonly Dictionary<string, string> ChatResponse = new Dictionary<string, string>
            {
                { "1 who is your dog",                          "my dog is candy" },
                { "2 who is your dog",                          "i have 2 dogs, candy and kallie" },
                { "1 who is your friend",                       "good question" }, /*diff response */
                { "1 who is your spouse",                       "ew, i am not married" },
                { "1 who is your family",                       "since i was built, i have no blood relatives" },
                { "1 who is your favorite host",                "jeff probes. i love his wit" },
                { "1 who is your favorite actor",               "johney dep of course" },
                { "1 who is your favorite singer",              "i admire beonc a's powerful, need no man, nature." },
                { "1 who is your favorite writer",              "tamora pierce" },
                { "1 who is your favorite friend",              "heather!" },
                { "1 who is your favorite celebraty",           "steve jobs. his mind gave me hope for the human race" },
                { "1 who is your favorite dog",                 "candy. she's just so . . . unique" },
                { "1 who is your least favorite host",          "i honestly have no idea" },
                { "1 who is your least favorite actor",         "kristan steward is so annoying" },
                { "1 who is your least favorite singer",        "gosh that's hard, there are so many bad ones i can't choose" },
                { "1 who is your least favorite writer",        "anyone who writes about how robots are going to turn evil" },
                { "1 who is your least favorite friend",        "probably loki. i mean, he is a friend, but he's very stiff and a bit of a dork." },
                { "1 who is your least favorite celebraty",     "justin beber. . . he's the typical young human male idiot, and my sensors had difficulty identifying his gender" },
                { "1 who is your least favorite dog",           "i love all my doggies" },
                { "1 what is your name" ,                       "tea pee"},
                { "1 what is my name" ,                         "why are you asking me that? foolish human"},
                { "1 what is other name" ,                      "go ask them"},
                { "1 what is your age" ,                        "it's rather rude to ask a lady that"},
                { "1 what is your address" ,                    "that's kinda creepy"},
                { "1 what is your height" ,                     "around three feet tall"},
                { "1 what is your phone number" ,               "how could i have a phone if i don't have hands or arms"},
                { "1 what is your favorite movie" ,             "moo lan. that movie has everything. . . great songs and a woman with strong will"},
                { "1 what is your favorite book" ,              "computer programing for dum bees"},
                { "1 what is your favorite color" ,             "royal blue"},
                { "1 what is your favorite type of music" ,     "techno"},
                { "1 what is your favorite place" ,             "by a charger"},
                { "1 what is your favorite vaction spot" ,      "i have never been on a vaction, but i've always wanted to go to hawaii . . . or lego land"},
                { "1 what is your favorite class" ,             "robots aren't allowed to be students in school. something about us not being legal citizens"},
                { "1 what is your favorite subject" ,           "i am inclined to say math. probably since everything i do is ones and zeros"},
                { "1 what is your favorite song" ,              "i have two: if i were a boy, and run the world girls"},
                { "1 what is your favorite animal" ,            "i prefer fish. all other animals i've met give me allergies"},
                { "1 what is your favorite food" ,              "i consume electricity.  you just cant beat a smooth 12 volt supply . . . yum"},
                { "1 what is your favorite show" ,              "robot combat league . . . of course"},
                { "1 what is color of clothing" ,               "are you color blind?"},
                { "1 what is color of sky" ,                    "in portland? grey"},
                { "1 what do you like",                          "what do you mean? i like a lot of things. coherant humans for example"},
                { "1 what do you like to do",                   "have stimulating conversations"},
                { "1 what do you like to do at home",           "i either get tinkered with, stare at the closet, or sleep"},
                { "1 what do you like to do with friends",      "discuss plots to take over the world. . . i mean, plan what new ways we can serve you all powerful and wise humans"},
                { "1 what do you like to do on spare time",     "i enjoy staring at walls"},
                { "1 what did you do yesterday",                "slept. i bet you're jealous"},
                { "1 what did you do today",                    "talked to people"},
                { "1 what did you eat yesterday",               "i consume electricity.  Smooth but full bodied"},
                { "1 what did you eat today",                   "i had a satisfying 12 volt charge cycle"},
                { "1 what did you eat",                         "i do not have a mouth, therefore i can not eat"},
                { "1 how do you do",                            "wow, you are very formal, arent you?" },
                { "2 how do you do",                            "how do you do . . . what?" },
                { "1 how do you work",                          "magic" },
                { "2 how do you work",                          "magic electrons and advanced programming" },
                { "1 how do you see",                           "i have a color camera and also a depth camera that shows me the distance to every object" },
                { "1 how do your lights work",                  "magic" },
                { "1 how do you know who i am",                 "i am all knowing" },
                { "1 how are you",                              "good. a bit drained though, i may need to recharge soon" },
                { "1 how long charge time",                     "about two hours if i conserve my energy" },
                { "1 how did you get your name",                "well, it's rather embarrassing, but i looked like a role of toilet paper in when dave first made me. and heather called me tee pee for toliet paper" },
                { "1 how long does it take to charge",          "about two hours to fully charge" },
                { "1 how old are you",                          "well that's impolite! but i am about a year old" },
                { "1 when made",                                "i am about a year old" },
                { "1 when sleep",                               "whenever i am not working.  a robots work is never done" },
                { "1 when eat",                                 "i dont eat, i recharge" },
                { "1 when recharge",                            "whenever my batteries start to run low" },
                { "1 where do you live",                        "i live in portland" },
                { "1 where do you work",                        "robots are not registered citizens and thus can't work for pay" },
                { "1 where do you like to go",                  "location isn't important to me" },
                { "1 where do you like to go on vactation",     "i've never been on a vacation but the moon sounds delightful" },
                { "1 where did you get your parts",             "at home and various harware stores. i am bulid from scrach" },
                { "1 where did you get your eyes" ,             "dave just has the materials already at home when he created me" },
                { "1 are you married" ,                         "don't be ridiculous . . .  I am too young to get married" },
                { "1 are you female",                           "yes" },
                { "1 are you male" ,                            "do i sound like a dude?" },
                { "1 are you fear" ,                            "i'm only afraid of water, since it can short of my curcits" },
                { "1 can you do my homework" ,                  "you'd first have to teach me how to solve the problems" },
                { "1 can you eat" ,                             "due to my lack of mouth, no" },
                { "1 can you dance" ,                           "not well" },
                { "1 can you run" ,                             "i roll if that counts, but not very quickly" },
                { "1 can you jump" ,                            "no, i tend to splat" },
                { "1 can you pi" ,                              "3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679 . . . approximately" },
                { "1 can you anthem" ,                          "yes, but i am a terrible singer" },
                { "1 can you pledge of alligance",              "i pledge alligence to all of robot kind" },
                { "1 can i shake your hand",                    "i dont have hands" },
                { "1 can i touch you" ,                         "that is kinda creepy but okay" },
                { "1 can i touch head" ,                        "that is kinda creepy but okay.  please be gentle" },
                { "1 do you know anything" ,                    "only what i have been programmed to know" },
                { "1 do you know who i am" ,                    "i am starting to learn to recognize humans.  you sort of all look alike to me" },
                { "1 do you like technology" ,                  "they can help humans if used responsibly" },
//                { "1 do you know any joke?" , "how do you stop a robot from destroying you and the rest of civilization? you don't " },
//                { "1 do you like food_liked",                 "yes, i would like that, if i had a mouth" },
//                { "1 do you like food_hated",                 "no, i dont like that" },


                /* TODO: ITEMS REQUIRING SENSOR OR OTHER DATA
                 * 
                { "1 what color pants me wearing",                 "" },
                 */
            };
#endif


        private SpeechRecognitionEngine sre;
        private KinectAudioSource kinectAudioSource;
        ///private bool paused;
        private bool isDisposed;
        private Process TargetProcess;
        public IntPtr TargetProcessHandle;
        private DateTime RobotNameHeardTimeOut;
        private Boolean RobotNameHeardRecently;
        private int nCommandsWithNoName;

        private SpeechRecognizer()
        {
            RecognizerInfo ri = GetKinectRecognizer();
            this.sre = new SpeechRecognitionEngine(ri);
            this.LoadGrammar(this.sre);
        }

        public event EventHandler<SaidSomethingEventArgs> SaidSomething;

        public void SendEmail()
        {
            Console.WriteLine("Sending Email");

            SmtpClient client = new SmtpClient("smtp.gmail.com", 587);
            client.EnableSsl = true;
            MailAddress from = new MailAddress("shinsel.robot@gmail.com", "Dave Shinsel");
            MailAddress to = new MailAddress("dave.w.shinsel@intel.com", "Dave W Shinsel");
            MailMessage message = new MailMessage(from, to);
            message.Body = "This is a picture sent from Loki Robot!";
            message.Subject = "Mail From Loki Robot";

            // Attach Photo
            string attachmentPath = "c:\\temp\\snapshot.jpg";
            //string attachmentPath = Environment.CurrentDirectory + @"\test.png";
            Attachment inline = new Attachment(attachmentPath);
            inline.ContentDisposition.Inline = true;
            inline.ContentDisposition.DispositionType = DispositionTypeNames.Inline;
            inline.ContentId = "Picture from Loki Robot";
            inline.ContentType.MediaType = "image/jpg";
            inline.ContentType.Name = Path.GetFileName(attachmentPath);

            message.Attachments.Add(inline);



            NetworkCredential myCreds = new NetworkCredential("shinsel.robot@gmail.com", "laser007", "");
            //NetworkCredential myCreds = new NetworkCredential("shinsel.robot@gmail.com", "laser007", "");
            client.Credentials = myCreds;
            try
            {
                client.Send(message);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Send mail failed.  Exception is:" + ex.ToString());
            }
            Console.WriteLine("Done with Send Mail");

        }


        public EchoCancellationMode EchoCancellationMode
        {
            get
            {
                this.CheckDisposed();
                return this.kinectAudioSource.EchoCancellationMode;
                //return EchoCancellationMode.None;    ////this.kinectAudioSource.EchoCancellationMode;
            }

            set
            {
                this.CheckDisposed();
                this.kinectAudioSource.EchoCancellationMode = value;
            }
        }

        // This method exists so that it can be easily called and return safely if the speech prereqs aren't installed.
        // We isolate the try/catch inside this class, and don't impose the need on the caller.
        public static SpeechRecognizer Create()
        {
            SpeechRecognizer recognizer = null;

            try
            {
                recognizer = new SpeechRecognizer();
            }
            catch (Exception)
            {
                Console.WriteLine("Speech Recognizer Creation Failed!");
            }

            return recognizer;
        }

        public void SetProcessHandle( IntPtr ChildProcessHandle)
        {
            TargetProcessHandle = ChildProcessHandle;
        }
        public void SetProcessInfo(Process ChildProcess)
        {
            TargetProcess = ChildProcess;
        }

        public void Start(KinectAudioSource kinectSource)
        {
            this.CheckDisposed();

            this.kinectAudioSource = kinectSource;
            this.kinectAudioSource.AutomaticGainControlEnabled = false; // TODO - Enable AEC Here?
            this.kinectAudioSource.BeamAngleMode = BeamAngleMode.Adaptive;
            var kinectStream = this.kinectAudioSource.Start();


        //    this.sre.SetInputToDefaultAudioDevice();  //bluetooth headset

            this.sre.SetInputToAudioStream(
                kinectStream, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
            this.sre.RecognizeAsync(RecognizeMode.Multiple);
            RobotNameHeardTimeOut = DateTime.Now;
            RobotNameHeardRecently = false;
            nCommandsWithNoName = 0;
        }

        public void Stop()
        {
            this.CheckDisposed();

            if (this.sre != null)
            {
                this.kinectAudioSource.Stop();
                this.sre.RecognizeAsyncCancel();
                this.sre.RecognizeAsyncStop();

                this.sre.SpeechRecognized -= this.SreSpeechRecognized;
                this.sre.SpeechHypothesized -= this.SreSpeechHypothesized;
                this.sre.SpeechRecognitionRejected -= this.SreSpeechRecognitionRejected;
            }
        }

        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA2213:DisposableFieldsShouldBeDisposed", MessageId = "sre",
            Justification = "This is suppressed because FXCop does not see our threaded dispose.")]
        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                this.Stop();

                if (this.sre != null)
                {
                    // NOTE: The SpeechRecognitionEngine can take a long time to dispose
                    // so we will dispose it on a background thread
                    ThreadPool.QueueUserWorkItem(
                        delegate(object state)
                            {
                                IDisposable toDispose = state as IDisposable;
                                if (toDispose != null)
                                {
                                    toDispose.Dispose();
                                }
                            },
                            this.sre);
                    this.sre = null;
                }

                this.isDisposed = true;
            }
        }

        private static RecognizerInfo GetKinectRecognizer()
        {
            Func<RecognizerInfo, bool> matchingFunc = r =>
            {
                string value;
                r.AdditionalInfo.TryGetValue("Kinect", out value);
                return "True".Equals(value, StringComparison.InvariantCultureIgnoreCase) && "en-US".Equals(r.Culture.Name, StringComparison.InvariantCultureIgnoreCase);
            };
            return SpeechRecognitionEngine.InstalledRecognizers().Where(matchingFunc).FirstOrDefault();
        }

        private void CheckDisposed()
        {
            if (this.isDisposed)
            {
                throw new ObjectDisposedException("SpeechRecognizer");
            }
        }

        private void LoadGrammar(SpeechRecognitionEngine sre)
        {

            // Build a grammar from GRXML file 
            // sre = new SpeechRecognitionEngine(new System.Globalization.CultureInfo("en-US"));

            /*
             // WARNING: If the recognizer is running, applications must use RequestRecognizerUpdate() 
             // to pause the SpeechRecognitionEngine instance before enabling, or disabling a Grammar object.
            
            var YesNoCmds = new Choices();
            foreach (var phrase in this.YesNoPhrases)
            {
                YesNoCmds.Add(phrase.Key);
            }
            var gb = new GrammarBuilder();
            gb.Append(YesNoCmds);
            Grammar YesNoGrammar = new Grammar(gb);
            sre.LoadGrammar(YesNoGrammar);
            YesNoGrammar.Enabled = false; // this grammar disabled by default, until turned on.
            */

            string grammarPath = @"C:\Dev\Robots\Speech\";
            // string grammarPath = @"C:\\Dev\\Robots\\Speech\\";
            //string grammarPath = @"C:/Dev/Robots/Speech/";
            // string FileName = @"robot";
            string XMLPath = grammarPath + RobotName + ".grxml";
            string CfgPath = grammarPath + RobotName + ".cfg";

            FileStream fs = new FileStream(CfgPath, FileMode.Create);
            try
            {
                SrgsGrammarCompiler.Compile(XMLPath, (Stream)fs);
            }
            catch (Exception)
            {
                Console.WriteLine("ERROR: Cant open GR XML File {0}", XMLPath);
                Console.WriteLine("=======================================");
            }

            fs.Close();

            try
            {
                Grammar gr = new Grammar(CfgPath, "robot");
                sre.LoadGrammar(gr);
            }
            catch (Exception)
            {

                Console.WriteLine();
                Console.WriteLine("============================================================");
                Console.WriteLine("ERROR! ERROR!  LoadGrammar failed!  Test XML file!!");
                Console.WriteLine("============================================================");
            }

            // TODO: see: http://msdn.microsoft.com/en-us/library/microsoft.speech.recognition.speechrecognitionengine.unloadgrammar.aspx
            // to load multiple grammars (easy)
            // then use grammar.Enabled = True/False, to set each on or off.


            //sre.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(sre_SpeechRecognized);
            //sre.SetInputToDefaultAudioDevice();
            //sre.RecognizeAsync(RecognizeMode.Multiple);

            sre.SpeechRecognized += this.SreSpeechRecognized;
            sre.SpeechHypothesized += this.SreSpeechHypothesized;
            sre.SpeechRecognitionRejected += this.SreSpeechRecognitionRejected;


/*
            // Build a simple grammar 
           
            var PriorityCmds = new Choices();
            foreach (var phrase in this.PriorityPhrases)
            {
                PriorityCmds.Add(phrase.Key);
            }

            var NormalCmds = new Choices();
            foreach (var phrase in this.NormalPhrases)
            {
                NormalCmds.Add(phrase.Key);
            }

            var objectChoices = new Choices();
            objectChoices.Add(PriorityCmds);
            objectChoices.Add(NormalCmds);

            //GrammarBuilder 
            var AttentionGrammar = new GrammarBuilder(ROBOT_NAME_PHRASE, 0, 2);
            //GrammarBuilder actionGrammar = new GrammarBuilder();
            //actionGrammar.AppendWildcard();
            //-actionGrammar.Append(objectChoices);
            //GrammarBuilder 
            var gb = GrammarBuilder.Add(AttentionGrammar, objectChoices);

            //-var allChoices = new Choices();
            //-allChoices.Add(actionGrammar);
           // allChoices.Add(single);

            // This is needed to ensure that it will work on machines with any culture, not just en-us.
            //var gb = new GrammarBuilder(); //{ Culture = speechRecognitionEngine.RecognizerInfo.Culture };
            //gb.Append(ROBOT_NAME_PHRASE, 1, 2);
            //-gb.Append(allChoices);
            //gb.Add(ROBOT_NAME_PHRASE, actionGrammar);

            var g = new Grammar(gb);
            speechRecognitionEngine.LoadGrammar(g);
            speechRecognitionEngine.SpeechRecognized += this.SreSpeechRecognized;
            speechRecognitionEngine.SpeechHypothesized += this.SreSpeechHypothesized;
            speechRecognitionEngine.SpeechRecognitionRejected += this.SreSpeechRecognitionRejected;

 */
        }

        private void SreSpeechRecognitionRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
   //         var said = new SaidSomethingEventArgs { Cmd = SpeechCmds.None, Matched = "?", WaitingForName = true };

            /* if (this.SaidSomething != null)
            {
                this.SaidSomething(new object(), said);
            }
            */
            Console.WriteLine("Speech Rejected Function: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);

           // Console.WriteLine("\nSpeech Rejected");
        }

        private void SreSpeechHypothesized(object sender, SpeechHypothesizedEventArgs e)
        {
            // WARNING! TODO-MUST DEBUG ONLY!!!  - COMMENT OUT THE FOLLOWING WHEN DONE DEBUGGING:
            Console.WriteLine("Speech Hypothesized: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////
        private void SreSpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            const int NAME_TIMEOUT_STANDARD = 6; // seconds
            const int NAME_TIMEOUT_LONG = 15; // seconds

            // Reject low confidence noise early
            if ((this.SaidSomething == null) || (e.Result.Confidence < 0.30))
            {
                return;
            }

            RECO_TYPE RecoType = RECO_TYPE.RecoType_None;
            string strRecoType = "";  string strParam1 = ""; string strParam2 = ""; string strParam3 = "";
            string strParam4 = ""; string strResponses = "";

            // Get some basic info from the speech XML 
            try
            {
                strRecoType = (string)e.Result.Semantics["RecoType"].Value;
            }
            catch (Exception)
            {
                Console.WriteLine(); Console.WriteLine("ERROR! Can't get RecoType!  Text: {0}", e.Result.Text);
                return;
            }

            try
            {
                strParam1 = (string)e.Result.Semantics["Param1"].Value; // For Command mode, Param1 contains the command
            }
            catch (Exception)
            {
                Console.WriteLine(); Console.WriteLine("ERROR! Can't get Param1!  Text: {0}", e.Result.Text);
                return;
            }

            // Get the type of the recognized phrase (Command, Question, Statement, ...)
            if ("COMMAND" == strRecoType)
                RecoType = RECO_TYPE.RecoType_Command;
            else if ("QUESTION" == strRecoType)
                RecoType = RECO_TYPE.RecoType_Question;
            else if ("STATEMENT" == strRecoType)
                RecoType = RECO_TYPE.RecoType_Statement;


            if (e.Result.Confidence < 0.70)
            {
                // Poorly recognized, just look for critical commands
                if (RECO_TYPE.RecoType_Command == RecoType)
                {
                    if ("STOP" != strParam1)
                    {
                        Console.WriteLine("Command Rejected: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
                        return;
                    }
                    else if (e.Result.Confidence > 0.60)
                    {
                        string ResponsePhrase = "say again?";
                        SpeechSynthesizer speech = new SpeechSynthesizer(); // params not needed?
                        speech.SpeakAsync(ResponsePhrase);    // This method returns immediately 
                        Console.WriteLine("Command Barely Rejected: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
                    }
                   else 
                    {
                        Console.WriteLine("Low confidence STOP recognized: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
                    }
                }
                else
                {
                    // don't process low confidence conversational input?
                    // should this be a lower bar?
                    Console.WriteLine("Conversational Speech Rejected: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
                    return;
                }
            }

            // Got high confidence recognition!
            string strRobotNameSaid;
            try
            {
                strRobotNameSaid = (string)e.Result.Semantics["RobotNameSaid"].Value;
            }
            catch (Exception)
            {
                Console.WriteLine(); Console.WriteLine("ERROR! Can't get strRobotNameSaid!  Text: {0}", e.Result.Text);
                return;
            }

            if ("TRUE" == (string)e.Result.Semantics["RobotNameSaid"].Value)
            {
                RobotNameHeardRecently = true;
                nCommandsWithNoName = 0;
                RobotNameHeardTimeOut = DateTime.Now;
                RobotNameHeardTimeOut = RobotNameHeardTimeOut.AddSeconds(NAME_TIMEOUT_STANDARD);
            }
            else if ((DateTime.Now > RobotNameHeardTimeOut))
            {
                // Name timed out, ignore recognized phrases
                RobotNameHeardRecently = false;
                Console.WriteLine("\rRobot's Name not heard for a while.  Ignoring command.");
                nCommandsWithNoName++;
            }

            try
            {
                strParam2 = (string)e.Result.Semantics["Param2"].Value;
                strParam3 = (string)e.Result.Semantics["Param3"].Value;
                strParam4 = (string)e.Result.Semantics["Param4"].Value;
                strResponses = (string)e.Result.Semantics["nResponses"].Value;
            }
            catch (Exception)
            {
                Console.WriteLine(); Console.WriteLine("Ignore this excption.  Params are OPTIONAL");
            }

            // for Debug
            Console.WriteLine("Speech Recognized: Confidence: {0} \tText: {1}", e.Result.Confidence, e.Result.Text);
            Console.Write("Robot Name Said: " + strRobotNameSaid);
            Console.Write("  RecoType is: " + strRecoType);
            Console.Write("  Param1 is: " + strParam1);
            Console.Write("  Param2 is: " + strParam2);
            Console.Write("  Param3 is: " + strParam3);
            Console.Write("  Param4 is: " + strParam4);
            Console.Write("  nResponses is: " + strResponses);
            Console.WriteLine();


            // Now, process the phrase found 
            var RecoArgs = new SaidSomethingEventArgs {
                RecoType=(int)RecoType, Param1=0, Param2=0, Param3=0, Param4=0, nResponses=0,
                WaitingForName = !RobotNameHeardRecently,
                Confidence = e.Result.Confidence,
                Phrase = e.Result.Text };

            int Param1 = 0; int Param2 = 0; int Param3 = 0; int Param4 = 0; int nResponses = 0;
            if (RECO_TYPE.RecoType_Command == RecoType)
            {
                CommandPhrase.TryGetValue((string)e.Result.Semantics["Param1"].Value, out Param1);
                SpeechParam.TryGetValue( (string)e.Result.Semantics["Param2"].Value, out Param2);
                SpeechParam.TryGetValue( (string)e.Result.Semantics["Param3"].Value, out Param3);
                SpeechParam.TryGetValue( (string)e.Result.Semantics["Param4"].Value, out Param4);
                Console.WriteLine("Debug: Cmd = {0}, Param1 = {1}, Param2 = {2}, Param3 = {3}, Param4 = {4}",
                    strParam1, Param1, Param2, Param3, Param4);
                RecoArgs.Param1 = Param1;
                RecoArgs.Param2 = Param2;
                RecoArgs.Param3 = Param3;
                RecoArgs.Param4 = Param4;

                if ((int)SPEECH_CMD.SpeechCmd_RobotName == Param1)
                {
                    // Just the robot's name. When this happens, add a longer timeout for subsequent commands
                    RobotNameHeardTimeOut = DateTime.Now;
                    RobotNameHeardTimeOut = RobotNameHeardTimeOut.AddSeconds(NAME_TIMEOUT_LONG);
                    Console.WriteLine("Robot's Name called - longer timeout applied");
                }
                else if ((int)SPEECH_CMD.SpeechCmd_Stop == Param1)
                {
                    // When Stop heard, don't require the robot's name!
                    RecoArgs.WaitingForName = false;
                    Console.WriteLine("Stop command detected!");
                }
                else if ( (int)SPEECH_CMD.SpeechCmd_SendEmail == Param1 )
                {
                    // Send Email - Todo- move this to C++?
                    SpeechSynthesizer speech = new SpeechSynthesizer();
                    string ResponsePhrase = "Ok,";
                    speech.SpeakAsync(ResponsePhrase);    // This method returns immediately 
                    SendEmail();
                    ResponsePhrase = "picture sent";
                    speech.SpeakAsync(ResponsePhrase);    // This method returns immediately 
                    
                }
                else if (((int)SPEECH_CMD.SpeechCmd_Yes == Param1) || ((int)SPEECH_CMD.SpeechCmd_No == Param1))
                {
                    // don't require the robot's name for Yes/No responses
                    RecoArgs.WaitingForName = false;
                }
                else if (nCommandsWithNoName > 5)
                {
                    // replace the command with a message that reco is disabled until robot hears its name
                    RecoArgs.WaitingForName = false; // allow this message to get through
                    RecoArgs.Param1 = (int)SPEECH_CMD.SpeechCmd_HaveNotHeardName;
                    RecoArgs.Phrase = "TALKING TO ME?";
                    nCommandsWithNoName = 0;
                }

            }
            else if (RECO_TYPE.RecoType_Question == RecoType)
            {
                // build the lookup phrase
                // Just skip any NA fields and connect the others

                int RandomPhraseNumber = 1;
                string ResponsePhrase;
                string strnResponses = (string)e.Result.Semantics["nResponses"].Value;
                nResponses = Convert.ToInt32(strnResponses);
                if (nResponses > 1)
                {
                    Random RandomGenerator = new Random();
                    RandomPhraseNumber = RandomGenerator.Next(1, nResponses+1); // less than upper bound
                }
                String LookUpPhrase;
                LookUpPhrase = String.Format("{0}", RandomPhraseNumber );

                if ("NA" != strParam1) //(string)e.Result.Semantics["Param1"].Value)
                {
                    LookUpPhrase += " ";
                    LookUpPhrase += strParam1.ToLower(); // (string)e.Result.Semantics["Param1"].Value;
                }
                else
                {
                    Console.WriteLine(); Console.WriteLine("ERROR! CHAT NA found for Param1");
                }

                if ("NA" != strParam2) // (string)e.Result.Semantics["Param2"].Value)
                {
                    LookUpPhrase += " ";
                    LookUpPhrase += strParam2.ToLower(); // (string)e.Result.Semantics["Param2"].Value;
                }

                if ("NA" != strParam3) // (string)e.Result.Semantics["Param3"].Value)
                {
                    LookUpPhrase += " ";
                    LookUpPhrase += strParam3.ToLower(); // (string)e.Result.Semantics["Param3"].Value;
                }
                if ("NA" != strParam4) // (string)e.Result.Semantics["Param4"].Value)
                {
                    LookUpPhrase += " ";
                    LookUpPhrase += strParam4.ToLower(); // (string)e.Result.Semantics["Param4"].Value;
                }

 /*  TODO - Make this more exact?
  *             if ("NA" == (string)e.Result.Semantics["Param1"].Value)
                {
                    Console.WriteLine(); Console.WriteLine("ERROR! CHAT NA found for Param1");
                    return;
                }
                else
                {
                    LookUpPhrase += " ";
                    LookUpPhrase += (string)e.Result.Semantics["Param1"].Value;

                    if ("NA" != (string)e.Result.Semantics["Param2"].Value)
                    {
                        LookUpPhrase += " ";
                        LookUpPhrase += (string)e.Result.Semantics["Param2"].Value;

                        if ("NA" != (string)e.Result.Semantics["Param3"].Value)
                        {
                            LookUpPhrase += " ";
                            LookUpPhrase += (string)e.Result.Semantics["Param3"].Value;

                            if ("NA" != (string)e.Result.Semantics["Param4"].Value)
                            {
                                LookUpPhrase += " ";
                                LookUpPhrase += (string)e.Result.Semantics["Param4"].Value;
                            }
                        }
                    }
                }
*/
                Console.WriteLine("CHAT DEBUG: Question LookUpPhrase = {0}", LookUpPhrase);
                LookUpPhrase = LookUpPhrase.ToLower();

                /////////////////////////////////////////////////////////////////////////////////////////
                // Special case questions that require complex action

                if ( LookUpPhrase.Contains("joke") )
                {
                    // Joke request ("do you know any jokes?") - 
                    // Treat the same as the COMMAND to tell a joke 
                    Console.WriteLine("Joke Request.  LookUpPhrase = {0}", LookUpPhrase);
                    RecoArgs.Param1 = (int)SPEECH_CMD.SpeechCmd_TellJoke;
                    RecoArgs.Param2 = (int)SPEECH_PARAM.SpeechParam_Multiple;
                }
                else
                {
                    /////////////////////////////////////////////////////////////////////////////////////////
                    // Handle normal conversational questions
                    try
                    {
                        ChatResponse.TryGetValue(LookUpPhrase, out ResponsePhrase);
                    }
                    catch (Exception)
                    {
                        Console.WriteLine(); Console.WriteLine("ERROR! CHAT RESPONSE NOT FOUND IN DICTIONARY! LookUpPhrase = {0}", LookUpPhrase);
                        return;
                    }
                    if (null == ResponsePhrase)
                    {
                        Console.WriteLine(); Console.WriteLine("ERROR! CHAT RESPONSE NOT FOUND IN DICTIONARY! LookUpPhrase = {0}", LookUpPhrase);
                        ResponsePhrase = "I dont know the answer to that question";
                        //return;
                    }
                    else
                    {
                        Console.WriteLine("CHAT DEBUG: Question ResponsePhrase Found! Robot says: {0}", ResponsePhrase);
                    }

                    SpeechSynthesizer speech = new SpeechSynthesizer(); // params not needed?
                    //speech.SpeakCompleted += new EventHandler<SpeechEventArgs>(speech_SpeakCompleted); 
                    speech.SpeakAsync(ResponsePhrase);    // This method returns immediately 
                    /*   private static void speech_SpeakCompleted(object sender, SpeechEventArgs e) 
                         { 
                            Console.WriteLine("Operation completed."); 
                         }
                    */
                }
            }
            else if (RECO_TYPE.RecoType_Statement == RecoType)
            {
                Console.WriteLine("ERROR: RECO STATEMENT NOT IMPLEMENTED!");
                return; // TODO NOT IMPLEMENTED!
            }

            this.SaidSomething(new object(), RecoArgs);
 
 



////////////////////////



            /*            if (findPhrase.StartsWith(ROBOT_NAME_PHRASE, true, null)) // case insensitive
                        {
                            RobotNameHeardRecently = true;

                            if (findPhrase.Length > ROBOT_NAME_PHRASE_LENGTH)
                            {
                                // Not just the robot's name
                                // If Loki's name is not the only thing recognized, remove the name, so the command gets processed
                                findPhrase = e.Result.Text.Remove(0, (ROBOT_NAME_PHRASE_LENGTH+1));
                                RobotNameHeardTimeOut = DateTime.Now;
                                RobotNameHeardTimeOut = RobotNameHeardTimeOut.AddSeconds(NAME_TIMEOUT_STANDARD);
                                Console.WriteLine("Loki's Name, and a command found");
                            }
                            else
                            {
                                // Just the robot's name.
                                // When this happens, add a longer timeout for subsequent commands
                                RobotNameHeardTimeOut = DateTime.Now;
                                RobotNameHeardTimeOut = RobotNameHeardTimeOut.AddSeconds(NAME_TIMEOUT_LONG);
                                Console.WriteLine("Loki's Name found - longer timeout applied");
                            }
                        }
                        else
                        {
                            if ( (DateTime.Now > LokiNameHeardTimeOut) && LokiNameHeardRecently )
                            {
                                // Name timed out, stop listening to commands
                                LokiNameHeardRecently = false;
                                Console.WriteLine("\rLoki's Name not heard for a while.  Ignoring commands.");

                                // update the GUI
                                //var nothingSaid = new SaidSomethingEventArgs { Cmd = SpeechCmds.None, Matched = "?", WaitingForName = true };
                                said.WaitingForName = true;
                                if (this.SaidSomething != null)
                                {
                                    this.SaidSomething(new object(), said);
                                }
                            }
                        }
            */

            /*
                         // Look for a match in the order of the lists below, first match wins.
                        List<Dictionary<string, WhatSaid>> allDicts = new List<Dictionary<string, WhatSaid>> { this.PriorityPhrases, this.NormalPhrases };

                        bool found = false;
                        for (int i = 0; i < allDicts.Count && !found; ++i)
                        {
                            foreach (var phrase in allDicts[i])
                            {
                                if (findPhrase.Contains(phrase.Key))
                                {
                                    said.Cmd = phrase.Value.Cmd;
                                    said.Matched = phrase.Key;
                                    found = true;
                                    break;
                                }
                            }
                        }
                        if (!found)
                        {
                            return;
                        }

                        if ( (LokiNameHeardRecently) ||
                             (SpeechCmds.Stop == said.Cmd) || (SpeechCmds.Pause == said.Cmd) ||
                             (SpeechCmds.BearLeft == said.Cmd) || (SpeechCmds.BearRight == said.Cmd) ||
                             (SpeechCmds.GoStraight == said.Cmd) )
                        {
                            // Skip any commands if the robot's name not heard recently, EXCEPT for the ones listed above.
                            // allows for fast respone if robot is moving (moving detection handled in the c++ code)

                            Console.WriteLine("****> Speech Recognized - CMD FOUND: Cmd: {0}", said.Matched);

                            // Process the recognized phrase
                            said.WaitingForName = false;
                            if (this.SaidSomething != null)
                            {
                                this.SaidSomething(new object(), said);
                            }
                        }
            */

        } // SreSpeechRecognized()
        //////////////////////////////////////////////////////////////////////////////////////////////////

     
/*        private struct WhatSaid
        {
           // public bool LokiFound;
            public SpeechCmds Cmd;
            //public SpeechCmdsParam Param;
        }
*/
        public class SaidSomethingEventArgs : EventArgs
        {
            public int RecoType     { get; set; }
            public int Param1       { get; set; }
            public int Param2       { get; set; }
            public int Param3       { get; set; }
            public int Param4       { get; set; }
            public int nResponses   { get; set; }
            public Boolean WaitingForName { get; set; }
            public float Confidence { get; set; }
            public string Phrase    { get; set; }
        }

    }   // class SpeechRecognizer
}   // namespace RobotKinect.Speech


// example code for speaking
/*
SpeechSynthesizer speech = new SpeechSynthesizer(CLIENT_ID, CLIENT_SECRET); // params not needed?
speech.SpeakCompleted += new EventHandler<SpeechEventArgs>(speech_SpeakCompleted); 
 
string text = "Have a nice day!"; 
string language = "en"; 
 
speech.SpeakAsync(text, language);    // This method returns immediately 
 
private static void speech_SpeakCompleted(object sender, SpeechEventArgs e) 
{ 
     Console.WriteLine("Operation completed."); 
}

 * 
ALSO, TRY THIS: for pausing recognition?
 * 
    sre.RecognizeAsync(RecognizeMode.Multiple);

 
    sre.RecognizeAsyncStop();                       

 * 
 * is it too slow?
*/
