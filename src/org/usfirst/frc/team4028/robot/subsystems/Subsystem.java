package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.util.LogDataBE;

public interface Subsystem {
	public void stop();
	
	public void zeroSensors();
	
	public void outputToShuffleboard();
	
	public void updateLogData(LogDataBE logData);
}