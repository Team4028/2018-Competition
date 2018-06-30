package org.usfirst.frc.team4028.robot.sensors;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Paths;

import org.usfirst.frc.team4028.util.CircularQueue;

public class SwitchableCameraServer
{
	// ======
	//  We can use /dev/v4l/by-id/ since they are different types of cameras
	// ======
	// Note: Sonix (bottom)
	private static final String USB1_NAME = "driver camera";
	//private static final String USB1_DEVICE_PATH = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0";
	private static final String USB1_DEVICE_PATH =  "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0";
	
	// Note: LifeCAM (top)
	private static final String USB2_NAME = "carriage camera";	
	private static final String USB2_DEVICE_PATH = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0";
	
    private static final int CAMERA_TCP_PORT = 1180;
	
    private UsbCamera _camera0;
	private UsbCamera _camera1;
	private UsbCamera _currentCamera;

	private MjpegServer _rawVideoServer;
    
    private CircularQueue<UsbCamera> _camList;
    
    // ==========================
    // Singleton Pattern
    // ==========================
    private static SwitchableCameraServer _instance = new SwitchableCameraServer();

	public static SwitchableCameraServer getInstance() 
	{
		return _instance;
	}

	// private constructor
	private SwitchableCameraServer() 
	{
		//C920
		//640x480 10FPS ~5.4 MB/S OK YELLOW
		//640x480 20FPS ~14.44 MB/S OK RED
		//320x240 20FPS BAD
		//640x360 20FPS (GOT 15-20FPS) 10-12 MB/S OK RED
		//432x240 20FPS BAD
		//432x240 30FPS (25-30 FPS) ~9MBPS OK RED
		//432x240 24FPS BAD
		//432x240 15FPS BAD
		//352x288 30FPS BAD
		//352x288 24FPS BAD
		//352x288 20FPS BAD
		//352x288 15FPS BAD
		
		//ELP
		//640x480 10FPS (6.6-7.1 MB/S) OK RED
		//640x480 20FPS (7.6-8.3 MB/S) OK RED
		//640x360 15FPS (~9.4 MB/S) OK RED
		//320x240 20PS (~5.0 MB/S) OK RED
		//320x240 15FPS ~5.0MB/S OK GREEN (NOT PERFECT GREEN)
		//320x180 20FPS ~5.0MB/S OK YELLOW
		
		//LIFECAM 
		//640x480 20FPS 9.52MB/S OK RED
		//640x480 10FPS 6.7MB/S OK RED
		//432x240 15FPS 3.9MB/S OK GREEN
		//640x360 24FPS 9.76MB/S OK RED
		//352x288 24FPS 7.6MB/S OK RED
		//176x144 30FPS 2.68MB/S OK GREEN
		// =============
		// option 2: (
		// =============
		/* Open connection to USB Camera (video device 0 [/dev/video0]) */
		int width = 320; // 160; // 320; //640;
		int height = 240; //90; //180; //480;
		int frames_per_sec = 15; //10; //20; //15;
		
		_rawVideoServer = new MjpegServer("raw_video_server", CAMERA_TCP_PORT);    	
		
		// build list of available cameras
		_camList = new CircularQueue<UsbCamera>();
		
		// =======================
		// driver camera (1st camera added to _camList will be the default camera) 
		// =======================
		if (Files.exists(Paths.get(USB1_DEVICE_PATH), LinkOption.NOFOLLOW_LINKS)) 
		{
			System.out.println ("...camera0 exists");
			_camera0 = new UsbCamera(USB1_NAME, USB1_DEVICE_PATH);
			_camera0.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera0.setExposureManual(5);
			_camera0.setWhiteBalanceManual(50);
			_camList.add(_camera0);
		}
		
		// =======================
		//  carriage camera
		// ======================= 
		if (Files.exists(Paths.get(USB2_DEVICE_PATH), LinkOption.NOFOLLOW_LINKS)) 
		{
			System.out.println ("...camera1 exists");
			_camera1 = new UsbCamera(USB2_NAME, USB2_DEVICE_PATH);
			_camera1.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera1.setExposureManual(60);
			_camera1.setWhiteBalanceManual(50);
			_camList.add(_camera1);
		}
		
		/* Configure Camera */
		/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
		/* Start raw Video Streaming Server */
		if(!_camList.isEmpty()) 
		{
			_currentCamera = _camList.get(0);
			_rawVideoServer.setSource(_currentCamera);
			System.out.println ("current camera ==> " + _currentCamera.getName());
			
			_rawVideoServer.setSource(_currentCamera);
		}
	}
	
	public void SwitchCamera() 
	{		
		if(!_camList.isEmpty()) 
		{
			_currentCamera = _camList.getNext();
			_rawVideoServer.setSource(_currentCamera);
			System.out.println ("current camera ==> " + _currentCamera.getName());
			
			_rawVideoServer.setSource(_currentCamera);
		} 
		else  
		{
			DriverStation.reportError("No Cameras Available", false);
		}
	}
	
/*
  Code to config necessary network tables entries for Shuffleboard Camera Viewer 
 	{
	//connect to Network Tables
	NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
	if(networkTables == null)
	{
		System.out.println ("networkTables does not exist");
	}
	
	//get the necessary table
	NetworkTable table = networkTables.getTable("CameraPublisher/USB Camera 0");
	if (table == null)
	{
		System.out.println ("table does not exist");
	}
	
	//get the necessary entry
	NetworkTableEntry entry = table.getEntry("streams");
	if (entry == null)
	{
		System.out.println ("entry does not exist");
	}
	
	//build camera address list
	String[] urls = new String[]
	{
			//options for wired or tethered
			//		mjpg:http://roboRIO-4028-FRC.local:1181/?action=stream
			//		mjpg:http://10.0.1.55:1181/?action=stream
			//		mjpg:http://172.22.11.2:1181/?action=stream
					"mjpg:http://172.22.11.2:" + Integer.toString(CAMERA_TCP_PORT) + "/stream.mjpg",
					"mjpg:http://10.40.28.2:" + Integer.toString(CAMERA_TCP_PORT) + "/stream.mjpg"
	};
			} */
	
}

