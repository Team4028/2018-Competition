package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.util.LogDataBE;

public class Carriage implements Subsystem {

	// singleton pattern
	private static Carriage _instance = new Carriage();
	
	public static Carriage getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Carriage() {
	
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void zeroSensors() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void outputToShuffleboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		// TODO Auto-generated method stub
		
	}
}

