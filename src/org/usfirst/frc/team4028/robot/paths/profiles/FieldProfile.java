package org.usfirst.frc.team4028.robot.paths.profiles;

/** Interface that holds all the field measurements required by the PathAdapter */
public interface FieldProfile {
	public double getLeftSwitchX();
	
	public double getLeftSwitchY();
	
	public double getRightSwitchX();
	
	public double getRightSwitchY();
	
	public double getLeftScaleX();
	
	public double getLeftScaleY();
	
	public double getRightScaleX();
	
	public double getRightScaleY();
}