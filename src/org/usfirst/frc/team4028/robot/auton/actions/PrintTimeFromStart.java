package org.usfirst.frc.team4028.robot.auton.actions;

import edu.wpi.first.wpilibj.Timer;

public class PrintTimeFromStart implements Action {
	private double _startTime;
	
	public PrintTimeFromStart(double startTime) {
		_startTime = startTime;
	}
	
	@Override
	public void start() {}

	@Override
	public void update() {}

	@Override
	public void done() {
		System.out.println(Timer.getFPGATimestamp() - _startTime);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}