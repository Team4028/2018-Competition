package org.usfirst.frc.team4028.robot.auton.actions;

import edu.wpi.first.wpilibj.Timer;

// Waits for a set amount of time
public class WaitAction implements Action{
	private double _startTime, _waitTime;
	
	public WaitAction(double waitTime) {
		_waitTime = waitTime;
	}
	
	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - _startTime > _waitTime;
	}
}