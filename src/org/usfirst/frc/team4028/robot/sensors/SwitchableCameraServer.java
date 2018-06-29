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

public class SwitchableCameraServer {
	private static final String CAM0_NAME = "ceiling camera";
	private static final String CAM1_NAME = "mechanical camera";
	private static final String CAM2_NAME = "camera2";
	private static final String CAM3_NAME = "camera3";
	UsbCamera _camera0;
	UsbCamera _camera1;
	UsbCamera _camera2;
	UsbCamera _camera3;
    MjpegServer _rawVideoServer;
    private CircularQueue<UsbCamera> _camList;
    
    private static final int CAMERA_TCP_PORT = 1180;
	
    private static SwitchableCameraServer _instance = new SwitchableCameraServer();
	String _currentCamera;
	
	public static SwitchableCameraServer getInstance() {
		return _instance;
	}

	private SwitchableCameraServer() {
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
		
		if (Files.exists(Paths.get("/dev/video0"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera0 exists");
			_camera0 = new UsbCamera(CAM0_NAME, 0);
			_camera0.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera0.setExposureManual(60);
			_camera0.setWhiteBalanceManual(50);
			_camList.add(_camera0);
		}
		if (Files.exists(Paths.get("/dev/video1"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera1 exists");
			_camera1 = new UsbCamera(CAM1_NAME, 1);
			_camera1.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera1.setExposureManual(5);
			_camera1.setWhiteBalanceManual(50);
			_camList.add(_camera1);
		}
		if (Files.exists(Paths.get("/dev/video2"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera2 exists");
			_camera2 = new UsbCamera(CAM2_NAME, 2);
			_camera2.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera2.setExposureManual(60);
			_camera2.setWhiteBalanceManual(50);
			_camList.add(_camera2);
		}
		if (Files.exists(Paths.get("/dev/video3"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera3 exists");
			_camera3 = new UsbCamera(CAM3_NAME, 3);
			_camera3.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camera3.setExposureManual(5);
			_camera3.setWhiteBalanceManual(50);
			_camList.add(_camera3);
		}
		
		/* Configure Camera */
		/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
	
		/* Start raw Video Streaming Server */
		
		_rawVideoServer.setSource(_camList.get(0));
		_currentCamera = _camList.get(0).toString(); 

		SwitchCamera();
	}
	
	public void SwitchCamera() {
		UsbCamera nextCamera = null;
		if(!_camList.isEmpty()) {
			nextCamera = _camList.getNext();
			_rawVideoServer.setSource(nextCamera);
			System.out.println ("	New camera = " + nextCamera.getName());
		} else  {
			DriverStation.reportError("No Cameras Available", false);
		}
		_rawVideoServer.setSource(nextCamera);
	}
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
			}
}