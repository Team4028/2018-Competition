package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;

public class OutfeedCubeAction implements Action {
	CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	
	@Override
	public void start() {}

	@Override
	public void update() {
		_cubeHandler.ejectCube_InfeedPlusCarriage();
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