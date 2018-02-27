package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_STATE;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_STATE;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_WHEELS_STATE;
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
public class CubeHandler2 implements Subsystem {

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
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = true;
	
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
	
	//=====================================================================================
	//Looper
	//=====================================================================================
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (CubeHandler2.this) {
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
			synchronized (CubeHandler2.this) 
			{	
				switch(_cubeHandlerState) 
				{
					case STOPPED:
						break;
						
					case WANT_TO_MOVE_ELEVATOR_TO_PRESET:
						
						if(!_infeed.areArmsInSafePosition())
						{
							// make sure arms are safe before moving elevator
							_infeed.moveArmsToSafePosition();
						}
						else 
						{
							ReportStateChg("Infeed Arm (State) " + _cubeHandlerState.toString() + " ==> [SAFE_TO_MOVE_ELEVATOR_PRESET]");
							_cubeHandlerState = CUBE_HANDLER_STATE.SAFE_TO_MOVE_ELEVATOR_TO_PRESET;
						}
						break;
					
					case SAFE_TO_MOVE_ELEVATOR_TO_PRESET:
						// move elevator to requested position
						_elevator.MoveToPresetPosition(_requestedPresetPosition);	
						
						break;
						
					case WANT_TO_MOVE_ELEVATOR_JOYSTICK:
						
						if(!_infeed.areArmsInSafePosition())
						{
							// make sure arms are safe before moving elevator
							_infeed.moveArmsToSafePosition();
						}
						else 
						{
							ReportStateChg("Infeed Arm (State) " + _cubeHandlerState.toString() + " ==> [SAFE_TO_MOVE_ELEVATOR_JOYSTICK]");
							_cubeHandlerState = CUBE_HANDLER_STATE.SAFE_TO_MOVE_ELEVATOR_JOYSTICK;
						}
						break;
					
					case SAFE_TO_MOVE_ELEVATOR_JOYSTICK:
						// move elevator to requested position
						_elevator.JogAxis(_requestedElevatorSpeedCmd);
						break;
							
					default:
						break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) 
		{
			synchronized (CubeHandler2.this) 
			{
				stop();
			}
		}
	};
	
	public Loop getLoop() 
	{
		return _loop;
	}

	
	// =====================================================================================
	
	@Override
	public void stop() 
	{
		stopInfeedAndCarriage();
		stopElevator();
	}
	
	@Override
	public void zeroSensors() 
	{
		// N/A for this class
	}

	//=====================================================================================
	//Methods from original cube handler
	//=====================================================================================	
	public void ejectCube(double joystickCommand) 
	{
		_carriage.ejectCubeVBus(joystickCommand);
	}
	
	public void runInfeedCubePlusCarriage(double joystickCommand) 
	{
		if(_elevator.isElevatorAtInfeedPosition()) 
		{
			_infeed.driveInfeedWheelsVBus(joystickCommand);
			_carriage.infeedCarriageMotorsVBus(joystickCommand);
		}
	}

	public boolean isCubeInCarriage() {
		return _carriage.isCubeInCarriage();
	}
	
	//=====================================================================================
	//Methods for Handling Interactions with Elevator Subsystem
	//=====================================================================================	
	public void elevator_JogAxis(double speedCmd) 
	{
		// TODO: Intercept
		//_elevator.JogAxis(speedCmd);
		
		_requestedElevatorSpeedCmd = speedCmd;
		ReportStateChg("Cube Handler (State) " + _cubeHandlerState.toString() + " ==> [WANT_TO_MOVE_ELEVATOR_JOYSTICK]");
		_cubeHandlerState = CUBE_HANDLER_STATE.WANT_TO_MOVE_ELEVATOR_JOYSTICK;
	}

	public void elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION presetPosition) 
	{
		// TODO: Intercept
		//_elevator.MoveToPresetPosition(presetPosition);	
		
		_requestedPresetPosition = presetPosition;
		ReportStateChg("Cube Handler (State) " + _cubeHandlerState.toString() + " ==> [WANT_TO_MOVE_ELEVATOR_TO_PRESET]");
		_cubeHandlerState = CUBE_HANDLER_STATE.WANT_TO_MOVE_ELEVATOR_TO_PRESET;
	}
	
	public boolean isElevatorAtTargetPos() {
		return _elevator.IsAtTargetPosition();
	}

	public void elevator_SafeStartup()
	{		
		// in ONLY this special case we bypass looper and talk directly to elevator class
		// we want to avoid 1 scan cycle when elevator looper can still have a cached target
		_elevator.rezeroElevator();
		
		// reset any cached target position 
		_requestedPresetPosition = ELEVATOR_PRESET_POSITION.HOME;
	}
	
	public void stopElevator()
	{
		_elevator.stop();
	}
	
	//=====================================================================================
	//Methods for Handling Interactions with Multiple Subsystem
	//=====================================================================================	
	public void acquireCube_InfeedAndCarriage() 
	{
		_infeed.infeedWheels_FeedIn();
		_carriage.FeedIn();
	}
	
	public void ejectCube_InfeedAndCarriage() 
	{
		// only run infeed wheels if we are at squeeze position
		if(_infeed.getInfeedArmTargetPosition() == INFEED_ARM_TARGET_POSITION.SQUEEZE)
		{
			_infeed.infeedWheels_FeedOut();
		}
		
		_carriage.FeedOut();
	}
	
	public void stopInfeedAndCarriage() 
	{
		_infeed.stopDriveMotors();
		_carriage.stop();
	}
	
	//=====================================================================================
	//Methods for Handling Interactions with Infeed Subsystem
	//=====================================================================================	
	public void infeedArms_SpinCube_CCW() 
	{
		_infeed.infeedWheels_SpinCube_CCW();
	}
	
	public void infeedArms_SpinCube_CW() 
	{
		_infeed.infeedWheels_SpinCube_CW();
	}
	
	public void infeedWheels_VBusCmd_BumpUp() 
	{
		_infeed.infeedWheels_VBusCmd_BumpUp();
	}
	
	public void infeedWheels_VBusCmd_BumpDown() 
	{		
		_infeed.infeedWheels_VBusCmd_BumpDown();
	}
	
	// used by auton only
	public void infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION presetTargetPostion)
	{
		switch(presetTargetPostion)
		{
			case SQUEEZE:
				infeedArms_moveToSqueezePosition();
				break;
			case HOME:
			case STORE:
				infeedArms_moveToStorePosition();
				break;
			case INFEED:
			case WIDE:
				_infeed.moveArmsToWideInfeedPosition();
				break;
			default:
				break;
		}
	}
	
	public void infeedArms_SqueezeAngle_BumpNarrower() 
	{
		_infeed.infeedArms_SqueezeAngle_BumpNarrower();
	}
	
	public void infeedArms_SqueezeAngle_BumpWider() 
	{
		_infeed.infeedArms_SqueezeAngle_BumpWider();
	}
	
	public void infeedArms_Rezero() 
	{
		_infeed.reZeroArms();
	}
	
	public void infeedArms_moveToWidePosition() 
	{
		_infeed.moveArmsToWideInfeedPosition();
	}
	
	public void infeedArms_moveToSqueezePosition() 
	{
		// TODO: Intercept
		if(_elevator.isElevatorAtInfeedPosition()) 
		{
			_infeed.moveArmsToSqueezeInfeedPosition();
		}
		else 
		{
			DriverStation.reportError("Elevator must be at Infeed Position!", false);
		}
	}
	
	public void infeedArms_moveToStorePosition() 
	{
		_infeed.storeArms();
	}	
	
	public void infeedArms_SafeStartup()
	{
		if(_infeed.getInfeedArmState() == INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD)
		{
			_infeed.reZeroArms();
		}
	}
	
	//=====================================================================================	
	//Methods for Handling Interactions with Carriage Subsystem
	//=====================================================================================	
	public void carriage_FeedIn_VBusCmd_BumpDown() 
	{
		_carriage.carriageWheels_FeedIn_VBusCmd_BumpDown();
	}
	
	public void carriage_FeedIn_VBusCmd_BumpUp() 
	{
		_carriage.carriageWheels_FeedIn_VBusCmd_BumpUp();
	}
	
	public void carriage_FeedOut_VBusCmd_BumpDown() 
	{
		_carriage.carriageWheels_FeedOut_VBusCmd_BumpDown();
	}
	
	public void carriage_FeedOut_VBusCmd_BumpUp() 
	{
		_carriage.carriageWheels_FeedOut_VBusCmd_BumpUp();
	}
	
	public void carriage_FeedOut_VBusCmd_IndexBumpUp()
	{
		_carriage.carriage_FeedOut_VBusCmd_IndexBumpUp();
	}
	
	public void carriage_FeedOut_VBusCmd_IndexBumpDown()
	{
		_carriage.carriage_FeedOut_VBusCmd_IndexBumpDown();	
	}
	
	//=====================================================================================	
	// Utility Methods
	//=====================================================================================	
	@Override
	public void outputToShuffleboard() 
	{
		SmartDashboard.putString("Cube Handler:State:", _cubeHandlerState.toString() );
	}

	@Override
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("CubeHandler:State", _cubeHandlerState.toString());
	}
	
	private void ReportStateChg(String message) 
	{
		if(IS_VERBOSE_LOGGING_ENABLED) 
		{
			System.out.println(message);
		}
	}
}