package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.util.LogDataBE;

public abstract class Subsystem {
	public abstract void stop();
	
	public abstract void zeroSensors();
	
	public abstract void outputToSmartDashboard();
	
	public abstract void updateLogData(LogDataBE logData);
}