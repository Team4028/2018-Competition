package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;

public class ArcadeDriveAction implements Action {
	private double _throttle;
	private double _startTime, _waitTime;
	
	public ArcadeDriveAction(double throttle, double waitTime) {
		_throttle = throttle;
		_waitTime = waitTime;
	}
	
	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		Chassis.getInstance().arcadeDrive(-_throttle, 0);
	}

	@Override
	public void done() {
		Chassis.getInstance().stop();
	}

	@Override
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() - _startTime) > _waitTime;
	}
	
}