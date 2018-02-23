package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class OutfeedCubeAction implements Action {
	CubeHandler _cubeHandler = CubeHandler.getInstance();
	Carriage _carriage = Carriage.getInstance();
	
	@Override
	public void start() {
	}

	@Override
	public void update() {
		_carriage.ejectCubeVBus(1.0);
	}

	@Override
	public void done() {
		_cubeHandler.stop();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
