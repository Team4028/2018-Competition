package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

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
		if ((Timer.getFPGATimestamp() - _startTime) < 1) {
			_cubeHandler.acquireCube_InfeedAndCarriage();
			_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
		} 
		else if ((Timer.getFPGATimestamp() - _startTime) < 1.5) {
			_cubeHandler.stop_InfeedAndCarriage();
			_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
		} else {
			_startTime = Timer.getFPGATimestamp();
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