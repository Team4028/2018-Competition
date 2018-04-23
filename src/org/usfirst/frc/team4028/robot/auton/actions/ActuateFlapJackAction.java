package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;

public class ActuateFlapJackAction implements Action{
	private CubeHandler _cubeHandler = CubeHandler.getInstance();
	private boolean _isActuated;
	
	public ActuateFlapJackAction(boolean isActuated) {
		_isActuated = isActuated;
	}
	
	@Override
	public void start() {}

	@Override
	public void update() {
		if (_isActuated)
			_cubeHandler.carriage_FlapUp();
		else 
			_cubeHandler.carriage_FlapDown();
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}