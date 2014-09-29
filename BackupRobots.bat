@ECHO Making Backup Copy of Loki project

@Echo Making backup structure
rmdir  /s /q ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Common
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Loki
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Turtle
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Speech
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotArduino
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotArduino\Tests
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotCamera
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotCamera\RobotCamera
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\KobukiControl
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\KobukiControl\KobukiControl
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\DepthCameraDS
mkdir ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\DepthCameraDS\DepthCameraDS


@Echo Copying Common Folders...
xcopy .\Robots.* 		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup		 		    /H/R/G/K/Y  
xcopy .\Common 		  	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Common  			    /H/R/I/G/K/Y  
xcopy .\Speech	 		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Speech	 		    /H/R/I/G/K/Y  

@Echo Copying Projects...
xcopy .\Loki 		  	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Loki          		    /H/R/I/G/K/Y  
xcopy .\Loki\res 	  	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Loki\res  	  	    /H/R/I/G/K/Y  

xcopy .\Turtle 		  	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Turtle        		    /H/R/I/G/K/Y  
xcopy .\Turtle\res 	  	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\Turtle\res 	  	    /H/R/I/G/K/Y  

xcopy .\RobotArduino		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotArduino		    /H/R/I/G/K/Y  
xcopy .\RobotArduino\Tests	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotArduino\Tests 	    /H/R/I/G/K/Y/E  

xcopy .\RobotKinectViewer	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotKinectViewer	    	    /H/R/I/G/K/Y  
xcopy .\RobotKinectViewer\Images     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotKinectViewer\Images	    /H/R/I/G/K/Y  
xcopy .\RobotKinectViewer\Properties ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotKinectViewer\Properties  /H/R/I/G/K/Y  

xcopy .\RobotCamera		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotCamera	    	    /H/R/I/G/K/Y  
xcopy .\RobotCamera\RobotCamera	     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotCamera\RobotCamera 	    /H/R/I/G/K/Y  
xcopy .\RobotCamera\FaceData         ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\RobotCamera\FaceData	    /H/R/I/G/K/Y  

xcopy .\KobukiControl		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\KobukiControl	    	    /H/R/I/G/K/Y  
xcopy .\KobukiControl\KobukiControl  ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\KobukiControl\KobukiControl   /H/R/I/G/K/Y  

xcopy .\DepthCameraDS		     ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\DepthCameraDS	            /H/R/I/G/K/Y  
xcopy .\DepthCameraDS\DepthCameraDS  ..\Backups\%date:~-4,4%-%date:~-10,2%-%date:~7,2%-Backup\DepthCameraDS\DepthCameraDS   /H/R/I/G/K/Y  

@Echo ---------------------------------------------------------------
@ECHO Done!
@Pause
