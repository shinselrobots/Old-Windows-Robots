
#if !__cplusplus  // make this file work for C++ as well as C#

namespace RobotKinect.Speech
{
    public
#endif

 enum RECO_TYPE // What kind of speech mode are we seeing?
    {
        RecoType_None = 0,
        RecoType_Command,
        RecoType_Question,
        RecoType_Statement
    };

    enum SPEECH_CMD // Recognized Commands
    {
        SpeechCmd_None = 0,
        SpeechCmd_RobotName,
        SpeechCmd_Stop,
        SpeechCmd_Thanks,
        SpeechCmd_Yes,
        SpeechCmd_No,
        SpeechCmd_Wave,
        SpeechCmd_SayHi,
        SpeechCmd_SayBye,
        SpeechCmd_SendEmail,
        SpeechCmd_TakePhoto,
        SpeechCmd_DoIntro,
        SpeechCmd_LookForward,
        SpeechCmd_Follow,
        SpeechCmd_ComeHere,
        SpeechCmd_TurnTowardsMe,
        SpeechCmd_DoWhatTime,
        SpeechCmd_FaceMe,
        SpeechCmd_CleanUp,
        SpeechCmd_Shake,
        SpeechCmd_MeetName,
        SpeechCmd_Wake,
        SpeechCmd_Mic,
        SpeechCmd_Lights,
        SpeechCmd_Move, 
        SpeechCmd_SmallMove,
        SpeechCmd_Explore,
        SpeechCmd_Spin,
        SpeechCmd_Turn,
        SpeechCmd_SmallTurn,
        SpeechCmd_EnableMotors,
        SpeechCmd_EnableAvoidance,
        SpeechCmd_ArmHome,
        SpeechCmd_ArmUp,
        SpeechCmd_ExtendArm,
        SpeechCmd_Claw,
        SpeechCmd_TakeObject,
        SpeechCmd_GiveObject,
        SpeechCmd_PickUpObject,
        SpeechCmd_PutObjectDown,
        SpeechCmd_PutInBasket,
        SpeechCmd_ThrowObject,
        SpeechCmd_ScratchHead,
        SpeechCmd_ScratchBack,
        SpeechCmd_MeaningOfLife,
        SpeechCmd_Karate,
        SpeechCmd_DoDanger,
        SpeechCmd_LightSaber,
        SpeechCmd_FaceCompass,
        SpeechCmd_PointCompass,
        SpeechCmd_IdentifyObj,
        SpeechCmd_GotoLocation,
        SpeechCmd_HaveNotHeardName,
        SpeechCmd_TellJoke,
        SpeechCmd_BadRobot,



    };

    enum SPEECH_PARAM // Recognized Command Parameters
    {
        SpeechParam_False = 0,  // and Disable
        SpeechParam_True,       // and Enable
        SpeechParam_Forward,
        SpeechParam_Reverse,
        SpeechParam_Left,
        SpeechParam_Right,
        SpeechParam_Both,
        SpeechParam_Open,
        SpeechParam_Close,
        SpeechParam_North,
        SpeechParam_East,
        SpeechParam_South,
        SpeechParam_West,
        SpeechParam_Full,
        SpeechParam_Office,
        SpeechParam_MasterBedroom,
        SpeechParam_HeatherRoom,
        SpeechParam_AmberRoom,
        SpeechParam_MasterBath,
        SpeechParam_Single,
        SpeechParam_Multiple,

    };

    enum SPEECH_QUESTION // Special Questions needing advanced processing
    {
        //None = 0,
        // 5 W's
        Question_Joke = 1,
        
    };






#if !__cplusplus  // make this file work for C++ as well as C#
}
#endif