package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;

public class CubeHandler {
	private enum CUBE_HANDLER_STATE {
		NEED_TO_MOVE_ARMS,
		ARMS_ARE_SAFE,
		ELEVATOR_MOVING_TO_POSITION,
		UNDEFINED
	}
	
	private CUBE_HANDLER_STATE _cubeHandlerState;
	
	private ELEVATOR_PRESET_POSITION _targetElevatorPosition;
	
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
	//Methods for Handling Interactions between infeed Subsystems
	//=====================================================================================	
	public void manageMoveElevatorToPresetPosition() {
		System.out.println("p");
		switch(_cubeHandlerState) {
			case NEED_TO_MOVE_ARMS:
				System.out.print("t");
				if(_infeed.areArmsInSafePosition()) {
					_cubeHandlerState = CUBE_HANDLER_STATE.ARMS_ARE_SAFE;
					System.out.println("CUBE_HANDLER_STATE_MACHINE: NEED_TO_MOVE_ARMS ==> ARMS_ARE_SAFE");
				} else {
					_infeed.moveArmsToSafePosition();
				}
				break;
			case ARMS_ARE_SAFE:
				System.out.println("j");
				_elevator.MoveToPresetPosition(_targetElevatorPosition);
				break;
			case UNDEFINED:
			default:
				System.out.println("o");
				break;
		}
		System.out.println("y");
	}
	
	public void moveElevatorToPresetPosition(ELEVATOR_PRESET_POSITION commandedPosition) {
		_targetElevatorPosition = commandedPosition;
		_cubeHandlerState = CUBE_HANDLER_STATE.NEED_TO_MOVE_ARMS;
		manageMoveElevatorToPresetPosition();
		System.out.println("CUBE_HANDLER_STATE_MACHINE: ARMS_ARE_SAFE ==> NEED_TO_MOVE_ARMS");
	}
	
	public boolean isStateMachineCurrentlyRunning() {
		if (_cubeHandlerState == CUBE_HANDLER_STATE.NEED_TO_MOVE_ARMS || !_elevator.IsAtTargetPosition()) {
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
		_infeed.doNothing();
		_elevator.doNothing();
		_carriage.stop();
	}
	
	public void stop() {
		_infeed.stopDriveMotors();
		_carriage.stop();
		_elevator.stop();
	}
	


}