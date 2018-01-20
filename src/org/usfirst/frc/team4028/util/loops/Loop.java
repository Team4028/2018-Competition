package org.usfirst.frc.team4028.util.loops;

public interface Loop {
	public void onStart(double timestamp);
	
	public void onLoop(double timestamp);
	
	public void onStop(double timestamp);
}