package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;

public class MoveElevatorToPosAction implements Action {
	private CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	private ELEVATOR_PRESET_POSITION _targetPos;
	
	public MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION targetPos) {
		_targetPos = targetPos;
	}
	
	@Override
	public void start() {}

	@Override
	public void update() {
		_cubeHandler.elevator_MoveToPresetPosition(_targetPos);
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return _cubeHandler.isElevatorAtTargetPos();
	}
}