package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

public class WaitUntilRemainingDistanceAction implements Action {
	double _targetDistance;
	
	public WaitUntilRemainingDistanceAction(double distance) {
		_targetDistance = distance;
	}
	
	@Override
	public void start() {}

	@Override
	public void update() {}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return Chassis.getInstance().getRemainingPathDistance() < _targetDistance;
	}
}