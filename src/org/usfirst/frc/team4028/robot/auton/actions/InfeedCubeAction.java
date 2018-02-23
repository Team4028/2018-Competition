package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class InfeedCubeAction implements Action {
	CubeHandler _cubeHandler = CubeHandler.getInstance();
	Infeed _infeed = Infeed.getInstance();
	Carriage _carriage = Carriage.getInstance();
	
	@Override
	public void start() {
	}

	@Override
	public void update() {
		_cubeHandler.runInfeedCubePlusCarriage(0.5);
		_infeed.moveArmsToSqueezeInfeedPosition();
	}

	@Override
	public void done() {
		_cubeHandler.stop();
	}

	@Override
	public boolean isFinished() {
		return _carriage.isCubeInCarriage();
	}
}
