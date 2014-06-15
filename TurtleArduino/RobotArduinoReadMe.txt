
1. Make sure you configure Arduio as follows:

File-> Preferences -> Sketchbook location: C:\Dev\Arduino

2.  Set COM Port to port where Arduino is connected

3.  Pair Phone with BlueSmirf module.

4.  In Android "BotControl" project, set DEVICE_ADDRESS to device address
    To find device address, pair bluetooth device with a PC, then check
    properties of the driver in device manager. (Details -> Hardware Ids)
    Hint: If you see "Firefly ABCD" the <ABCD> is the last two digits of
    the device; eg. 00:06:66:1F:AB:CD

