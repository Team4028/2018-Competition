package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;

public class MoveElevatorToPosAction implements Action {
	private CubeHandler _cubeHandler = CubeHandler.getInstance();
	private ELEVATOR_PRESET_POSITION _targetPos;
	
	public MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION targetPos) {
		_targetPos = targetPos;
	}
	
	@Override
	public void start() {
		_cubeHandler.elevator_MoveToPresetPosition(_targetPos);
	}

	@Override
	public void update() {}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return _cubeHandler.isElevatorAtTargetPos();
	}
}