package org.usfirst.frc.team4028.robot.auton;

import org.usfirst.frc.team4028.robot.auton.actions.Action;

import edu.wpi.first.wpilibj.Timer;

public abstract class AutonBase {
	protected boolean _active = false;
	protected double _startTime;
	
	/** This contains all the runAction methods in the auton */
	public abstract void routine();
	
	public void run() {
		_active = true;
		routine();
	}
	
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}
	
	public void stop() {
		_active = false;
	}
	
	/** Runs an action until isFinished() returns true */
	public void runAction(Action action) {
		action.start();
		while (_active && !action.isFinished()) {
			action.update();
		}
		action.done();
	}
}