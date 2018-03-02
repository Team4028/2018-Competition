package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;

public class OutfeedCubeAction implements Action {
	CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	
	@Override
	public void start() {}

	@Override
	public void update() {
		_cubeHandler.carriage_FeedOut(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100);
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