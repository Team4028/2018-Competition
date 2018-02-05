package org.usfirst.frc.team4028.robot.subsystems;

public class CubeHandler {

	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
			
	// singleton pattern
	private static CubeHandler _instance = new CubeHandler();
	
	public static CubeHandler getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private CubeHandler() {
	
	}
}
