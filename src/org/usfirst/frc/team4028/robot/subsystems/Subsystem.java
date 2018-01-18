package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.util.LogDataBE;

public interface ISubsystem {
	public void stop();
	
	public void zeroSensors();
	
	public void outputToDashboard();
	
	public void updateLogData(LogDataBE logData);
}