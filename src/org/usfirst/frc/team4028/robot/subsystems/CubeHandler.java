package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the Cube Handler SuperSubsystem
//
// This superclass manages the multi-subsystem interactions between the following
//	interrelated classes:
//		- Elevator
//		- Infeed
//		- Carriage
//
//=====> For Changes see Patrick Bruns
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		PatB		???			Initial Version
//	1		TomB,Nick	24-Feb		Implement Elevator, Infeed Interlock
//-------------------------------------------------------------
public class CubeHandler implements Subsystem {

	private enum CUBE_HANDLER_STATE {
		UNDEFINED,
		STOPPED,
		WANT_TO_MOVE_ELEVATOR_TO_PRESET,
		SAFE_TO_MOVE_ELEVATOR_TO_PRESET,
		WANT_TO_MOVE_ELEVATOR_JOYSTICK,		// Note: Separate state since the cmd method is different
		SAFE_TO_MOVE_ELEVATOR_JOYSTICK		// Note: Separate state since the cmd method is different
	}
	
	// Subsystems
	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
	
	// define class level working variables
	private CUBE_HANDLER_STATE _cubeHandlerState = CUBE_HANDLER_STATE.UNDEFINED;
	private ELEVATOR_PRESET_POSITION _requestedPresetPosition;
	private double _requestedElevatorSpeedCmd;
	
	// define class level constants
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = false;
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static CubeHandler _instance = new CubeHandler();
	
	public static CubeHandler getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private CubeHandler() {
	}
	
	//=====================================================================================
	//Looper
	//=====================================================================================
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (CubeHandler.this) {
			}
		}
		
		//=====================================================================================
		//State Machine for Coordinating subsystem actions
		//	(in this case it is making sure the infeed arms are in a safe position
		//		before allowing the elevator to move to the infeed position)
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) 
		{
			synchronized (CubeHandler.this) 
			{	
				switch(_cubeHandlerState) 
				{
					case STOPPED:
						break;
						
					case WANT_TO_MOVE_ELEVATOR_TO_PRESET:
						
						if(!_infeed.areArmsInSafePosition()) {							
							_infeed.moveArmsToSafePosition(); // make sure arms are safe before moving elevator
						}
						else {
							ReportStateChg("Infeed Arm (State) " + _cubeHandlerState.toString() + " ==> [SAFE_TO_MOVE_ELEVATOR_PRESET]");
							_cubeHandlerState = CUBE_HANDLER_STATE.SAFE_TO_MOVE_ELEVATOR_TO_PRESET;
						}
						break;
					
					case SAFE_TO_MOVE_ELEVATOR_TO_PRESET:
						_elevator.MoveToPresetPosition(_requestedPresetPosition); // move elevator to requested position	
						break;
						
					case WANT_TO_MOVE_ELEVATOR_JOYSTICK:
						
						if(!_infeed.areArmsInSafePosition()) {
							_infeed.moveArmsToSafePosition(); // make sure arms are safe before moving elevator
						}
						else {
							ReportStateChg("Infeed Arm (State) " + _cubeHandlerState.toString() + " ==> [SAFE_TO_MOVE_ELEVATOR_JOYSTICK]");
							_cubeHandlerState = CUBE_HANDLER_STATE.SAFE_TO_MOVE_ELEVATOR_JOYSTICK;
						}
						break;
					
					case SAFE_TO_MOVE_ELEVATOR_JOYSTICK:
						_elevator.JogAxis(_requestedElevatorSpeedCmd); // move elevator to requested position
						break;
							
					default:
						break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (CubeHandler.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}

	
	//=====================================================================================
	//
	//=====================================================================================	
	
	@Override
	public void stop() {
		stop_InfeedAndCarriage();
		stopElevator();
	}
	
	@Override
	public void zeroSensors() {}

	//=====================================================================================
	//Methods for Handling Interactions with Elevator Subsystem
	//=====================================================================================	
	public void elevator_JogAxis(double speedCmd) {		
		_requestedElevatorSpeedCmd = speedCmd;
		ReportStateChg("Cube Handler (State) " + _cubeHandlerState.toString() + " ==> [WANT_TO_MOVE_ELEVATOR_JOYSTICK]");
		_cubeHandlerState = CUBE_HANDLER_STATE.WANT_TO_MOVE_ELEVATOR_JOYSTICK;
	}

	public void elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION presetPosition) {
		_elevator.resetElevatorScaleHeightBump(); // always reset bump when we move to a position
		_requestedPresetPosition = presetPosition;
		ReportStateChg("Cube Handler (State) " + _cubeHandlerState.toString() + " ==> [WANT_TO_MOVE_ELEVATOR_TO_PRESET]");
		_cubeHandlerState = CUBE_HANDLER_STATE.WANT_TO_MOVE_ELEVATOR_TO_PRESET;
	}
	
	public boolean isElevatorAtTargetPos() {
		return _elevator.IsAtTargetPosition();
	}
	
	public void stopElevator() {
		_elevator.stop();
	}
	
	public void elevator_ScaleHeight_BumpPositionUp() {
		if(_requestedPresetPosition == ELEVATOR_PRESET_POSITION.NEUTRAL_SCALE_HEIGHT) {
			if(_elevator.getElevatorScaleHeightBumpInches() < 11.9) {
				_elevator.elevatorScaleHeightBumpPositionUp();
			}
			else {
				System.out.println("Elevator Scale Position Bump Tooooooo Large");
			}
		}
		else {
			System.out.println("Bump Up Only honored when requested position is Scale ");
		}
	}
	
	public void elevator_ScaleHeight_BumpPositionDown() {
		if(_requestedPresetPosition == ELEVATOR_PRESET_POSITION.NEUTRAL_SCALE_HEIGHT) {
			if(_elevator.getElevatorScaleHeightBumpInches() > -11.9) {
				_elevator.elevatorScaleHeightBumpPositionDown();
			}
			else {
				System.out.println("Elevator Scale Position Bump Tooooooo Large");
			}
		}
		else {
			System.out.println("Bump Down Only honored when requested position is Scale ");
		}
	}
	
	//=====================================================================================
	//Methods for Handling Interactions with Multiple Subsystem
	//=====================================================================================	
	public void acquireCube_InfeedAndCarriage() {
		_infeed.feedIn();
		_carriage.feedIn();
	}
	
	public void ejectCube_InfeedAndCarriage() {
		if(_infeed.getInfeedArmsTargetPosition() == INFEED_ARM_TARGET_POSITION.SQUEEZE) { // only run infeed wheels if we are at squeeze position
			_infeed.feedOut();
		}
		_carriage.feedOut(); // always run carriage wheels
	}
	
	public void stop_InfeedAndCarriage() {
		_infeed.stopDriveMotors();
		_carriage.stop();
	}
	
	//=====================================================================================
	//Methods for Handling Interactions with Infeed Subsystem
	//=====================================================================================	
	public void infeedArms_SpinCube_CCW() {
		_infeed.infeedWheels_SpinCube_CCW();
	}
	
	public void infeedArms_SpinCube_CW() {
		_infeed.infeedWheels_SpinCube_CW();
	}
	
	public void infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION presetTargetPostion) { // used by auton only
		switch(presetTargetPostion)	{
			case SQUEEZE:
				infeedArms_moveToSqueezePosition();
				break;
				
			case HOME:
			case STORE:
				infeedArms_moveToStorePosition();
				break;
				
			case INFEED:
			case WIDE:
				infeedArms_moveToWidePosition();
				break;
				
			default:
				break;
		}
	}
	
	public void infeedArms_moveToSqueezePosition() {
		if(_elevator.isElevatorAtInfeedPosition()) {
			_infeed.moveArmsToSqueezeInfeedPosition();
		}
		else {
			DriverStation.reportError("Elevator must be at Infeed Position!", false);
		}
	}
	
	public void infeedArms_moveToStorePosition() {
		_infeed.storeArms();
	}	
	
	public void infeedArms_moveToWidePosition() {
		_infeed.moveArmsToWideInfeedPosition();
	}
	
	public void infeedArms_SqueezeAngle_BumpNarrower() {
		_infeed.infeedArms_SqueezeAngle_BumpNarrower();
	}
	
	public void infeedArms_SqueezeAngle_BumpWider() {
		_infeed.infeedArms_SqueezeAngle_BumpWider();
	}
	
	public void infeedArms_Rezero() {
		_infeed.reZeroArms();
	}
	
	public void infeedWheels_VBusCmd_BumpUp() {
		_infeed.infeedWheels_VBusCmd_BumpUp();
	}
	
	public void infeedWheels_VBusCmd_BumpDown() {		
		_infeed.infeedWheels_VBusCmd_BumpDown();
	}
	
	//=====================================================================================	
	//Methods for Handling Interactions with Carriage Subsystem
	//=====================================================================================	
	public void carriage_FeedOut(CARRIAGE_WHEELS_OUT_VBUS_INDEX setSpeed) {
		_carriage.feedOut(setSpeed);
	}
	
	public void carriage_FeedIn_VBusCmd_BumpDown() {
		_carriage.carriageWheels_FeedIn_VBusCmd_BumpDown();
	}
	
	public void carriage_FeedIn_VBusCmd_BumpUp() {
		_carriage.carriageWheels_FeedIn_VBusCmd_BumpUp();
	}
	
	public void carriage_FeedOut_VBusCmd_IndexBumpUp() {
		_carriage.carriageWheels_FeedOut_VBusCmd_IndexBumpUp();
	}
	
	public void carriage_FeedOut_VBusCmd_IndexBumpDown() {
		_carriage.carriage_FeedOut_VBusCmd_IndexBumpDown();	
	}
	
	public void carriage_MoveSolenoidToSqueeze() {
		_carriage.moveCarriageToSqueezeWidth();
	}
	
	public void carriage_MoveSolenoidToWide() {
		_carriage.moveCarriageToWideWidth();
	}
	
	public boolean isCubeInCarriage() {
		return _carriage.isCubeInCarriage();
	}
	
	//=====================================================================================	
	// Utility Methods
	//=====================================================================================	
	@Override
	public void outputToShuffleboard() {
		SmartDashboard.putString("Cube Handler:State:", _cubeHandlerState.toString() );
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("CubeHandler:State", _cubeHandlerState.toString());
	}
	
	private void ReportStateChg(String message) {
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}