package org.usfirst.frc.team4028.robot.paths.profiles;

/** Interface that holds all the field measurements required by the PathAdapter */
public interface FieldProfile {
	public double getLSwitchX();
	
	public double getLSwitchY();
	
	public double getRSwitchX();
	
	public double getRSwitchY();
	
	public double getLScaleX();
	
	public double getLScaleY();
	
	public double getRScaleX();
	
	public double getRScaleY();
}