package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the carriage Subsystem
//=====> For Changes see Patrick Bruns
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		PatB		???			Initial Version
//	1		TomB		18.Feb		Cleanup Looper
//  2		TomB		25.Feb		Code Cleanup
//	3		TomB		26.Feb		Initial support for Indexed Outfeed Cmds
//-------------------------------------------------------------
public class Carriage implements Subsystem {
	
	private enum CARRIAGE_WHEELS_STATE {
		STOPPED,
		FEED_IN,
		FEED_OUT,
		JOYSTICK
	}
	
	public enum CARRIAGE_WHEELS_OUT_VBUS_INDEX {
		VBUS_10,
		VBUS_20,
		VBUS_30,
		VBUS_40,
		VBUS_50,
		VBUS_60,
		VBUS_70,
		VBUS_80,
		VBUS_90,
		VBUS_100
	}
	
	// define class level working variables
	private TalonSRX _carriageLeftMotor; 
	private TalonSRX _carriageRightMotor;
	
	private DigitalInput _carriageLimitSwitch;
	private DoubleSolenoid _squeezeCylinder;
	private DoubleSolenoid _tiltCylinder;
	
	private CARRIAGE_WHEELS_STATE _carriageWheelsState;
	private CARRIAGE_WHEELS_OUT_VBUS_INDEX _currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
	private double _currentCarriageWheelsFeedInVBusCmd = .45;
	private double _currentCarriageWheelsJoystickVBusCmd = 0;
	
	private static final double CARRIAGE_WHEELS_IN_VBUS_COMMAND_BUMP = 0.05;
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = true;
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static Carriage _instance = new Carriage();
	
	public static Carriage getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Carriage() {
		//====================================================================================
		//	config master & slave talon objects
		//====================================================================================
		_carriageLeftMotor = new TalonSRX(Constants.CARRIAGE_LEFT_CAN_ADDRESS);
		_carriageRightMotor = new TalonSRX(Constants.CARRIAGE_RIGHT_CAN_ADDRESS);
		
		// set motor phasing
		_carriageLeftMotor.setInverted(false);
		_carriageRightMotor.setInverted(true);
		
		// config limit switches
		_carriageLeftMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_carriageLeftMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_carriageRightMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0, 0);
		_carriageRightMotor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0, 0);
		
		// turn off all soft limits
		_carriageLeftMotor.configForwardSoftLimitEnable(false, 0);
		_carriageLeftMotor.configReverseSoftLimitEnable(false, 0);
		_carriageRightMotor.configForwardSoftLimitEnable(false, 0);
		_carriageRightMotor.configReverseSoftLimitEnable(false, 0);
		
		// config brake mode
		_carriageLeftMotor.setNeutralMode(NeutralMode.Coast);
		_carriageRightMotor.setNeutralMode(NeutralMode.Coast);
		
		//Enable Current Limiting
		_carriageLeftMotor.enableCurrentLimit(true);
		_carriageLeftMotor.configPeakCurrentDuration(200, 0);
		_carriageLeftMotor.configPeakCurrentLimit(17, 0);
		
		// config quad encoder & phase (invert = true)
		_carriageLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
		
		//configure the peak and nominal output voltages in both directions for both Talons
		_carriageLeftMotor.configNominalOutputForward(0, 0);
		_carriageLeftMotor.configNominalOutputReverse(0, 0);
		_carriageLeftMotor.configPeakOutputForward(1, 0);
		_carriageLeftMotor.configPeakOutputReverse(-1, 0);
		
		_carriageRightMotor.configNominalOutputForward(0, 0);
		_carriageRightMotor.configNominalOutputReverse(0, 0);
		_carriageRightMotor.configPeakOutputForward(1, 0);
		_carriageRightMotor.configPeakOutputReverse(-1, 0);
		
		// set motor mode
		_carriageLeftMotor.set(ControlMode.PercentOutput, 0, 0);
		_carriageRightMotor.set(ControlMode.PercentOutput, 0, 0);
	
		// DisableSoftLimits
		_carriageLeftMotor.configReverseSoftLimitEnable(false, 0);
		_carriageLeftMotor.configForwardSoftLimitEnable(false, 0);
		_carriageRightMotor.configReverseSoftLimitEnable(false, 0);
		_carriageRightMotor.configForwardSoftLimitEnable(false, 0);
		
		//Setup Limit Switch
		_carriageLimitSwitch = new DigitalInput(Constants.CARRIAGE_LIMIT_SWITCH_DIO_PORT);
		
		//Setup Solenoid for Cylinder
		_squeezeCylinder = new DoubleSolenoid(Constants.PCM_CAN_ADDR, Constants.CARRIAGE_SQUEEZE_PCM_PORT, Constants.CARRIAGE_WIDE_PCM_PORT);
		_squeezeCylinder.set(Constants.CARRIAGE_WIDE_POS);
		
		//Setup Solenoid for Tilt
		_tiltCylinder = new DoubleSolenoid(Constants.PCM_CAN_ADDR, Constants.CARRIAGE_FLAP_UP_PCM_PORT, Constants.CARRIAGE_FLAP_DOWN_PCM_PORT);
		_tiltCylinder.set(Constants.CARRIAGE_FLAP_DOWN);
		
		_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED;
	}

	//=====================================================================================
	//Set Up Looper to run loop at 10ms interval (2x RoboRio Cycle Time)
	//=====================================================================================
	private final Loop _loop = new Loop() { // called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (Carriage.this) {
				_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED; // reset to default startup start
				_squeezeCylinder.set(Constants.CARRIAGE_WIDE_POS);
			}
		}
		
		//=====================================================================================
		//Looper and State Machine for Commanding Infeed Axis
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) {
			synchronized (Carriage.this) {		
				switch(_carriageWheelsState) {
					case STOPPED:
						_carriageLeftMotor.set(ControlMode.PercentOutput, 0, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, 0, 0);
						break;
						
					case FEED_IN:
						_carriageLeftMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsFeedInVBusCmd, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsFeedInVBusCmd, 0);
						break;
						
					case FEED_OUT:
						_carriageLeftMotor.set(ControlMode.PercentOutput, -1 * getCurrentCarriageWheelsFeedOutVBusCmd(), 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, -1 * getCurrentCarriageWheelsFeedOutVBusCmd(), 0);
						break;
						
					case JOYSTICK:
						_carriageLeftMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsJoystickVBusCmd, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsJoystickVBusCmd, 0);
						break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Carriage.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	//=====================================================================================
	// Public Methods to control motors
	//=====================================================================================
	@Override
	public void stop() {
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.STOPPED) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [STOPPED]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED;
		}
	}

	//=== Feed In =================================================================
	public void feedIn() {
		if(!isCubeInCarriage()) {
			if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.FEED_IN) {
				ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [FEED_IN]");
				_carriageWheelsState = CARRIAGE_WHEELS_STATE.FEED_IN;
			}			
		} else {
			stop();
		}
	}
	
	public void feedIn(double joystickCommand) {
		if(!isCubeInCarriage()) {
			_currentCarriageWheelsJoystickVBusCmd = joystickCommand * 0.5;
			
			if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.JOYSTICK) {
				ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [JOYSTICK][IN]");
				_carriageWheelsState = CARRIAGE_WHEELS_STATE.JOYSTICK;
			}			
		} else {
			stop();
		}
	}
	
	//=== Feed Out =================================================================
	public void feedOut() {
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.FEED_OUT) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [FEED_OUT]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.FEED_OUT;
		}
	}
			
	public void feedOut(CARRIAGE_WHEELS_OUT_VBUS_INDEX setSpeed) {
		_currentCarriageWheelsFeedOutVBusIndex = setSpeed;
		
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.FEED_OUT) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [FEED_OUT]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.FEED_OUT;
		}
	}
	
	public void feedOut(double joystickCommand) {
		_currentCarriageWheelsJoystickVBusCmd = -1 * joystickCommand; // invert cmd
		
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.JOYSTICK) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [JOYSTICK][OUT]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.JOYSTICK;
		}
	}
	
	//=== Bump Feed In Cmd =================================================================
	public void carriageWheels_FeedIn_VBusCmd_BumpUp() {
		double newCmd = _currentCarriageWheelsFeedInVBusCmd + CARRIAGE_WHEELS_IN_VBUS_COMMAND_BUMP;
		
		if(newCmd <= 1.0) { // only bump if new cmd is not over max
			_currentCarriageWheelsFeedInVBusCmd = newCmd;
		}
	}
	
	public void carriageWheels_FeedIn_VBusCmd_BumpDown() {		
		double newCmd = _currentCarriageWheelsFeedInVBusCmd - CARRIAGE_WHEELS_IN_VBUS_COMMAND_BUMP;
		
		if(newCmd >= 0.0) { // only bump if new cmd is not under min
			_currentCarriageWheelsFeedInVBusCmd = newCmd;
		}
	}
	
	//=== Bump Feed Out Cmd =================================================================
	public void carriageWheels_FeedOut_VBusCmd_IndexBumpUp() {
		switch (_currentCarriageWheelsFeedOutVBusIndex)	{
			case VBUS_100:
				// do nothing
				break;
	
			case VBUS_90:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
				break;
				
			case VBUS_80:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_90;
				break;
	
			case VBUS_70:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_80;
				break;
				
			case VBUS_60:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				break;
	
			case VBUS_50:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				break;
				
			case VBUS_40:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				break;
	
			case VBUS_30:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_40;
				break;
	
			case VBUS_20:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_30;
				break;
				
			case VBUS_10:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_20;
				break;
				
			default:
				break;
		}
	}
	
	public void carriage_FeedOut_VBusCmd_IndexBumpDown() {
		switch (_currentCarriageWheelsFeedOutVBusIndex)	{
			case VBUS_100:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_90;
				break;

			case VBUS_90:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_80;
				break;
				
			case VBUS_80:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				break;

			case VBUS_70:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				break;
				
			case VBUS_60:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				break;

			case VBUS_50:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_40;
				break;
				
			case VBUS_40:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_30;
				break;

			case VBUS_30:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_20;
				break;

			case VBUS_20:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_10;
				break;
				
			case VBUS_10:
				// do nothing
				break;
				
			default:
				break;
		}
	}
	
	//=== Handle Carriage Squeeze Solenoid =================================================================
	public void moveCarriageToSqueezeWidth() {
		if(isCarriageInSqueezePosition()) {
			System.out.println("Carriage Already In Thin Position");
		} else {
			_squeezeCylinder.set(Constants.CARRIAGE_SQUEEZE_POS);
		}
	}
	
	public void moveCarriageToWideWidth() {
		if(isCarriageInSqueezePosition()) {
			_squeezeCylinder.set(Constants.CARRIAGE_WIDE_POS);
		} else {
			System.out.println("Carriage Already In Wide Position");
		}	
	}
	
	//=== Handle Carriage Tilt Solenoid =================================================================
	public void tiltCarriageUp() {
		_tiltCylinder.set(Constants.CARRIAGE_FLAP_UP);
	}
	
	public void tiltCarriageDown() {
		_tiltCylinder.set(Constants.CARRIAGE_FLAP_DOWN);	
	}
	
	@Override
	public void zeroSensors() {}
	
	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	public boolean isCubeInCarriage() {
		return _carriageLimitSwitch.get(); // normally closed switch, input is pulled low
	} 
	
	public boolean isCarriageInSqueezePosition() {
		return _squeezeCylinder.get() == Constants.CARRIAGE_SQUEEZE_POS;
	}
	
	private double getCurrentCarriageWheelsFeedOutVBusCmd() {
		switch (_currentCarriageWheelsFeedOutVBusIndex)	{
			case VBUS_100:
				return 1;
			
			case VBUS_90:
				return .9;
				
			case VBUS_80:
				return .8;
				
			case VBUS_70:
				return .7;
				
			case VBUS_60:
				return .6;
				
			case VBUS_50:
				return .5;
				
			case VBUS_40:
				return .4;
				
			case VBUS_30:
				return .3;
				
			case VBUS_20:
				return .2;
				
			case VBUS_10:
				return .1;
				
			default:
				return 0;
		}
	}
	
	//=====================================================================================
	// Utility Methods
	//=====================================================================================
	@Override
	public void outputToShuffleboard() {
		SmartDashboard.putNumber("Carriage: Wheels Feed In %VBus", _currentCarriageWheelsFeedInVBusCmd);
		SmartDashboard.putNumber("Carriage: Wheels Feed Out %VBus", getCurrentCarriageWheelsFeedOutVBusCmd());
		SmartDashboard.putBoolean("Carriage: Is Cube In Carriage?", isCubeInCarriage());
		SmartDashboard.putBoolean("Carriage: Is Squeezed", isCarriageInSqueezePosition());
		SmartDashboard.putString("State: Carriage", _carriageWheelsState.toString());
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Carriage: LimitSwitch", String.valueOf(isCubeInCarriage()));
		logData.AddData("Carriage: VBus Cmd", String.valueOf(getCurrentCarriageWheelsFeedOutVBusCmd()));
		logData.AddData("State: Carriage", _carriageWheelsState.toString());
	}
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message) {
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}