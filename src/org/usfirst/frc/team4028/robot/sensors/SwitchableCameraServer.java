package org.usfirst.frc.team4028.robot.sensors;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
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
		// =============
		// option 2: (
		// =============
		/* Open connection to USB Camera (video device 0 [/dev/video0]) */
		int width = 640; // 160; // 320; //640;
		int height = 480; //90; //180; //480;
		int frames_per_sec = 10; //20; //15;
		
		_rawVideoServer = new MjpegServer("raw_video_server", CAMERA_TCP_PORT);    	
		
		// build list of available cameras
		_camList = new CircularQueue<UsbCamera>();
		
		if (Files.exists(Paths.get("/dev/video0"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera0 exists");
			_camera0 = new UsbCamera(CAM0_NAME, 0);
			_camera0.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camList.add(_camera0);
		}
		if (Files.exists(Paths.get("/dev/video1"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera1 exists");
			_camera1 = new UsbCamera(CAM1_NAME, 1);
			_camera1.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camList.add(_camera1);
		}
		if (Files.exists(Paths.get("/dev/video2"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera2 exists");
			_camera2 = new UsbCamera(CAM2_NAME, 2);
			_camera2.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camList.add(_camera2);
		}
		if (Files.exists(Paths.get("/dev/video3"), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("		camera3 exists");
			_camera3 = new UsbCamera(CAM3_NAME, 3);
			_camera3.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_camList.add(_camera3);
		}
		
		/* Configure Camera */
		/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
	
		/* Start raw Video Streaming Server */
		_rawVideoServer.setSource(null);
		_currentCamera = null; 

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
}