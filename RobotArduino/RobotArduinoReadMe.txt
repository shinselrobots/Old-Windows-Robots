
1. Make sure you configure Arduio as follows:

File-> Preferences -> Sketchbook location: C:\Dev\Arduino

2.  Set COM Port to port where Arduino is connected

3.  Pair Phone with BlueSmirf module.

4.  In Android "BotControl" project, set DEVICE_ADDRESS to device address
    To find device address, pair bluetooth device with a PC, then check
    properties of the driver in device manager. (Details -> Hardware Ids)
    Hint: If you see "Firefly ABCD" the <ABCD> is the last two digits of
    the device; eg. 00:06:66:1F:AB:CD

TO SETUP A NEW DEVICE:
1.  On the Android phone, install Amarino!  
    From command line:  "adb install Amarino_2_xxx.apk"
2.  Run Amarino and add BT devices!
(if you skip these steps, the App will never connect)




If you get this error (or similar):
	In file included from RobotArduino.ino:3:
	RobotConstants.h:37: error: 'byte' does not name a type
	RobotUtilities:216: error: 'struct ARDUINO_CMD_T' has no member named 'Param4'

You need to set the Sketchbook location above, exit and restart Arduino IDE! 