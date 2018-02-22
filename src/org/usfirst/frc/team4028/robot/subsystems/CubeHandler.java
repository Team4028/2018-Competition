package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;

public class CubeHandler {
	private enum CUBE_HANDLER_STATE {
		NEED_TO_MOVE_ARMS,
		ELEVATOR_MOVING_TO_POSITION,
		ELEVATOR_IN_POSITION,
		UNDEFINED
	}
	
	private CUBE_HANDLER_STATE _cubeHandlerState;
	
	private ELEVATOR_PRESET_POSITION _targetElevatorPosition;
	
	private boolean _isCubeHandlerStateMachineRunning = false;
	
	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
			
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================	
	private static CubeHandler _instance = new CubeHandler();
	
	public static CubeHandler getInstance() {
		return _instance;
	}
	
	//=====================================================================================
	//Private Constructor for Singleton pattern
	//=====================================================================================	
	private CubeHandler() {
		_cubeHandlerState = CUBE_HANDLER_STATE.UNDEFINED;
	}
	
	//=====================================================================================
	//Methods for Handling Interactions between multiple Subsystems
	//=====================================================================================	
	public void manageMoveElevatorToPresetPosition() {
		System.out.println("p");
		switch(_cubeHandlerState) {
			case NEED_TO_MOVE_ARMS:
				System.out.print("t");
				if(_infeed.areArmsInSafePosition()) {
					_cubeHandlerState = CUBE_HANDLER_STATE.ELEVATOR_MOVING_TO_POSITION;
					System.out.println("CUBE_HANDLER_STATE_MACHINE: NEED_TO_MOVE_ARMS ==> ELEVATOR_MOVING_TO_POSITION");
				} else {
					_infeed.moveArmsToSafePosition();
				}
				break;
			case ELEVATOR_MOVING_TO_POSITION:
				_elevator.MoveToPresetPosition(_targetElevatorPosition);
				if(_elevator.IsAtTargetPosition() || _targetElevatorPosition == ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR) {
					_cubeHandlerState = CUBE_HANDLER_STATE.ELEVATOR_IN_POSITION;
					System.out.println("CUBE_HANDLER_STATE_MACHINE: ELEVATOR_MOVING_TO_POSITION ==> ELEVATOR_IN_POSITION");
				}
				break;
			case ELEVATOR_IN_POSITION:
				_isCubeHandlerStateMachineRunning = false;
				break;
			case UNDEFINED:
				break;
		}
		System.out.println("y");
	}
	
	public void moveElevatorToPresetPosition(ELEVATOR_PRESET_POSITION commandedPosition) {
		_targetElevatorPosition = commandedPosition;
		_cubeHandlerState = CUBE_HANDLER_STATE.NEED_TO_MOVE_ARMS;
		_isCubeHandlerStateMachineRunning = true;
		System.out.println("CUBE_HANDLER_STATE_MACHINE: ????? ==> NEED_TO_MOVE_ARMS");
	}
	
	public boolean isStateMachineCurrentlyRunning() {
		if (_isCubeHandlerStateMachineRunning) {
			return true;
		} else {
			return false;
		}
	}
	
	public void runInfeedCubePlusCarriage(double joystickCommand) {
		if(_elevator.isElevatorAtFloorPosition()) {
			_infeed.driveInfeedWheelsVBus(joystickCommand);
			_carriage.infeedCarriageMotorsVBus(joystickCommand);
		}
	}
	
	public void runInfeedSpinManuver() {
		_infeed.spinManuverInfeedWheels();
	}
	
	public void ejectCube(double joystickCommand) {
		_carriage.ejectCubeVBus(joystickCommand);
	}
		
	public void doNothing() {
		System.out.println("Do Nothing");
		_infeed.doNothing();
		_elevator.doNothing();
	}
	
	public void stop() {
		_infeed.stopDriveMotors();
		_carriage.stop();
	}
}