package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.util.LogDataBE;

public class Climber implements Subsystem {
	private static Climber _instance = new Climber();
	
	public static Climber getInstance() {
		return _instance;
	}
	
	private Climber() {
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