package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class InfeedCubeAction implements Action {
	CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	Infeed _infeed = Infeed.getInstance();
	Carriage _carriage = Carriage.getInstance();
	
	@Override
	public void start() {
	}

	@Override
	public void update() {
		_cubeHandler.acquireCube_InfeedPlusCarriage();
	}

	@Override
	public void done() {
		_cubeHandler.stop();
	}

	@Override
	public boolean isFinished() {
		return _cubeHandler.isCubeInCarriage();
	}
}
