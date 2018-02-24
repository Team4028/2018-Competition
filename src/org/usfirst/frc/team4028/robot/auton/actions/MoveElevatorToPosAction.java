package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;

public class MoveElevatorToPosAction implements Action {
	private Elevator _elevator = Elevator.getInstance();
	private ELEVATOR_PRESET_POSITION _targetPos;
	
	public MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION targetPos) {
		_targetPos = targetPos;
	}
	
	@Override
	public void start() {
	}

	@Override
	public void update() {
		_elevator.MoveToPresetPosition(_targetPos);
	}

	@Override
	public void done() {
	}

	@Override
	public boolean isFinished() {
		return _elevator.isElevatorAtInfeedPosition();
	}
}