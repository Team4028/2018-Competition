package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class DriveInfeedWheelsAction implements Action {
	Infeed _infeed = Infeed.getInstance();
	
	@Override
	public void start() {
	}

	@Override
	public void update() {
		_infeed.driveInfeedWheels();
	}

	@Override
	public void done() {
		_infeed.stopDriveMotors();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}