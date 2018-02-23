package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_WHEELS_STATE;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DriverStation;

public class CubeHandler2 implements Subsystem {

	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static CubeHandler2 _instance = new CubeHandler2();
	
	public static CubeHandler2 getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private CubeHandler2() {
	}
	
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (CubeHandler2.this) {
			}
		}
		
		//=====================================================================================
		//Looper and State Machine for Commanding Infeed Axis
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) {
			synchronized (CubeHandler2.this) {	
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (CubeHandler2.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}

	
	// =====================================================================================
	
	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void zeroSensors() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void outputToShuffleboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		// TODO Auto-generated method stub
		
	}

	//=====================================================================================
	//Methods for original cube handler
	//=====================================================================================	
	public void ejectCube(double joystickCommand) {
		_carriage.ejectCubeVBus(joystickCommand);
	}
	
	public void runInfeedCubePlusCarriage(double joystickCommand) {
		if(_elevator.isElevatorAtFloorPosition()) {
			_infeed.driveInfeedWheelsVBus(joystickCommand);
			_carriage.infeedCarriageMotorsVBus(joystickCommand);
		}
	}
	
	public void doNothing() {
		_infeed.doNothing();
		_elevator.doNothing();
		_carriage.stop();
	}
	//=====================================================================================
	//Methods for Handling Interactions between Elevator Subsystems
	//=====================================================================================	
	public void JogAxis(double speedCmd) {
		_elevator.JogAxis(speedCmd);
	}

	public void MoveToPresetPosition(ELEVATOR_PRESET_POSITION presetPosition) {
		_elevator.MoveToPresetPosition(presetPosition);		
	}
	
	//=====================================================================================
	//Methods for Handling Interactions between Infeed Subsystems
	//=====================================================================================	
	public void engrGamepadB_FeedIn() {
		_infeed.engrGamepadB_FeedIn();
	}
	
	public void engrGamepadB_FeedOut() {
		_infeed.engrGamepadB_FeedOut();
	}
	
	public void engrGamepadB_SpinCounterClockwise() {
		_infeed.engrGamepadB_SpinCounterClockwise();
	}
	
	public void engrGamepadB_SpinClockwise() {
		_infeed.engrGamepadB_SpinClockwise();
	}
	
	public void engrGamepadB_InfeedVBUS_BumpUp() {
		_infeed.engrGamepadB_InfeedVBUS_BumpUp();
	}
	
	public void engrGamepadB_InfeedVBUS_BumpDown() {		
		_infeed.engrGamepadB_InfeedVBUS_BumpDown();
	}
	
	public void engrGamepadB_SqueezeAngle_BumpNarrower() {
		_infeed.engrGamepadB_SqueezeAngle_BumpNarrower();
	}
	
	public void engrGamepadB_SqueezeAngle_BumpWider() {
		_infeed.engrGamepadB_SqueezeAngle_BumpWider();
	}
	
	public void reZeroArms() {
		_infeed.reZeroArms();
	}
	
	public void moveArmsToWideInfeedPosition() {
		_infeed.moveArmsToWideInfeedPosition();
	}
	
	public void moveArmsToSqueezeInfeedPosition() {
		_infeed.moveArmsToSqueezeInfeedPosition();
	}
	
	public void storeArms() {
		_infeed.storeArms();
	}	
}