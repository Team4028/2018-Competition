package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

public class DriveSetDistanceAction implements Action{
	Chassis _chassis = Chassis.getInstance();
	double _distanceInInches;
	
	public DriveSetDistanceAction(double distanceInInches) {
		_distanceInInches = distanceInInches;
	}
	
	@Override
	public void start() {
		_chassis.setTargetPos(_distanceInInches);
	}

	@Override
	public void update() {}

	@Override
	public void done() {
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		return _chassis.atTargetPos();
	}
}