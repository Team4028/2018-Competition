package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

// Turns the chassis to a specified angle
public class TurnAction implements Action{
	private Chassis _chassis = Chassis.getInstance();
	
	private double _targetAngle;
	private boolean _isTurnRight;
	
	public TurnAction(double angle, boolean isTurnRight) {
		_targetAngle = angle;
		_isTurnRight = isTurnRight;
	}
	
	@Override
	public void start() {
		_chassis.setTargetAngle(_targetAngle, _isTurnRight);
	}

	@Override
	public void update() {}

	@Override
	public void done() {	
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		if(_targetAngle==180||_targetAngle==-180)
		{
			return _chassis.getHeading()>178 || _chassis.getHeading()<-178;	
		}
		else
		{
			return Math.abs(_targetAngle - _chassis.getHeading()) < 2.0;		// Returns true when chassis is within angle deadband
		}
	}
}