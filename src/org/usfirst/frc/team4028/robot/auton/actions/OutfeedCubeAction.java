package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;

public class OutfeedCubeAction implements Action {
	CubeHandler _cubeHandler = CubeHandler.getInstance();
	CARRIAGE_WHEELS_OUT_VBUS_INDEX _speed;
	double _startTime;
	double _duration;
	
	public OutfeedCubeAction() {
		_speed = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
		_duration = 0.2;
	}
	
	public OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX speed) {
		_speed = speed;
		_duration = 0.2;
	}
	
	public OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX speed, double duration) {
		_speed = speed;
		_duration = duration;
	}
	
	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		_cubeHandler.carriage_FeedOut(_speed);
	}

	@Override
	public void done() {
		_cubeHandler.stop();
	}

	@Override
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() - _startTime) > _duration;
	}
}