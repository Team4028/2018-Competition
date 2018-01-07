package org.usfirst.frc.team4028.robot.auton.actions;

public interface Action {
	public abstract void start();
	
	public abstract void update();
	
	public abstract void done();
	
	public abstract boolean isFinished();
}