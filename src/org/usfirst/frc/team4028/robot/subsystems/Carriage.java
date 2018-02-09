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
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void outputToShuffleboard() {
	}

	@Override
	public void updateLogData(LogDataBE logData) {
	}
}