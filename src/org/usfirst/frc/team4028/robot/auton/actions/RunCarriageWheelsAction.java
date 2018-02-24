package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage;

public class RunCarriageWheelsAction implements Action{
	Carriage _carriage = Carriage.getInstance();
	private boolean _isInfeed;
	
	public RunCarriageWheelsAction(boolean isInfeed) {
		isInfeed=_isInfeed;
	} 

	@Override
	public void start() {}

	@Override
	public void update() {
		if(_isInfeed) {
			_carriage.FeedIn();
		} else {
			_carriage.FeedOut();
		}
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}