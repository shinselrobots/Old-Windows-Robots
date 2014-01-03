@ECHO Making Backup Copy of Loki project

@Echo Making backup structure
rmdir  /s /q .\_Backup
mkdir .\_Backup
mkdir .\_Backup\Common
mkdir .\_Backup\Loki
mkdir .\_Backup\Turtle
mkdir .\_Backup\Speech
mkdir .\_Backup\RobotArduino
mkdir .\_Backup\RobotArduino\Tests
mkdir .\_Backup\RobotCamera
mkdir .\_Backup\RobotCamera\RobotCamera
mkdir .\_Backup\KobukiControl
mkdir .\_Backup\KobukiControl\KobukiControl
date /T > .\_Backup\BackupDate.txt

@Echo Copying Common Folders...
xcopy .\Common 		  	     .\_Backup\Common		 	    /H/R/I/G/K/Y  
xcopy .\Speech	 		     .\_Backup\Speech	 		    /H/R/I/G/K/Y  
xcopy .\Robots.* 		     .\_Backup		 		    /H/R/G/K/Y  

@Echo Copying Projects...
xcopy .\Loki 		  	     .\_Backup\Loki          		    /H/R/I/G/K/Y  
xcopy .\Loki\res 	  	     .\_Backup\Loki\res  	  	    /H/R/I/G/K/Y  

xcopy .\Turtle 		  	     .\_Backup\Turtle        		    /H/R/I/G/K/Y  
xcopy .\Turtle\res 	  	     .\_Backup\Turtle\res 	  	    /H/R/I/G/K/Y  

xcopy .\RobotArduino		     .\_Backup\RobotArduino		    /H/R/I/G/K/Y  
xcopy .\RobotArduino\Tests	     .\_Backup\RobotArduino\Tests 	    /H/R/I/G/K/Y/E  

xcopy .\RobotKinectViewer	     .\_Backup\RobotKinectViewer	    /H/R/I/G/K/Y  
xcopy .\RobotKinectViewer\Images     .\_Backup\RobotKinectViewer\Images	    /H/R/I/G/K/Y  
xcopy .\RobotKinectViewer\Properties .\_Backup\RobotKinectViewer\Properties /H/R/I/G/K/Y  

xcopy .\RobotCamera		     .\_Backup\RobotCamera	    	    /H/R/I/G/K/Y  
xcopy .\RobotCamera\RobotCamera	     .\_Backup\RobotCamera\RobotCamera 	    /H/R/I/G/K/Y  
xcopy .\RobotCamera\FaceData         .\_Backup\RobotCamera\FaceData	    /H/R/I/G/K/Y  

xcopy .\KobukiControl		     .\_Backup\KobukiControl	    	    /H/R/I/G/K/Y  
xcopy .\KobukiControl\KobukiControl  .\_Backup\KobukiControl\KobukiControl  /H/R/I/G/K/Y  

@Echo ---------------------------------------------------------------
@ECHO Done!
@Pause
