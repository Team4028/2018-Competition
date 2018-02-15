package org.usfirst.frc.team4028.robot.subsystems;

public class CubeHandler {
	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
			
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================	
	private static CubeHandler _instance = new CubeHandler();
	
	public static CubeHandler getInstance() {
		return _instance;
	}
	
	//=====================================================================================
	//Private Constructor for Singleton pattern
	//=====================================================================================	
	private CubeHandler() {}
	
	//=====================================================================================
	//Methods for Handling Interactions between multiple Subsystems
	//=====================================================================================	
	public void runInfeedCubePlusCarriage(double joystickCommand) {
		if(_elevator.isElevatorAtFloorPosition()) {
			_infeed.driveInfeedWheelsVBus(joystickCommand);
			_carriage.infeedCarriageMotorsVBus(joystickCommand);
		}
	}
	
	public void runInfeedSpinManuverPlusCarriage() {
		
		_infeed.spinManuverInfeedWheels();
		
	}
	
	public void ejectCube(double joystickCommand) {
		_carriage.ejectCubeVBus(joystickCommand);
	}
	
	public void stop() {
		_infeed.stopDriveMotors();
		_carriage.stop();
	}
}