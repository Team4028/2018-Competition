package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;

import edu.wpi.first.wpilibj.Timer;

public class InfeedCubeAction implements Action {
	CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	double _startTime;
	
	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		if ((Timer.getFPGATimestamp() - _startTime) < 2) {
			_cubeHandler.acquireCube_InfeedAndCarriage();;
		} 
		else if ((Timer.getFPGATimestamp() - _startTime) < 3) {
			_cubeHandler.stopInfeedAndCarriage();
		}
		else {
			_cubeHandler.acquireCube_InfeedAndCarriage();
		}
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