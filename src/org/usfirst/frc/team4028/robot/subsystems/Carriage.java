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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the carriage Subsystem
//=====> For Changes see Patrick Bruns
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		PatB		???			Initial Version
//	1		TomB		18-Feb		Cleanup Looper
//-------------------------------------------------------------
public class Carriage implements Subsystem {
	
	private enum CARRIAGE_WHEELS_STATE {
		STOPPED,
		FEED_IN,
		FEED_OUT,
		JOYSTICK
	}
	
	// define class level working variables
	private TalonSRX _carriageLeftMotor; 
	private TalonSRX _carriageRightMotor;
	
	private DigitalInput _carriageLimitSwitch;
	
	//private Servo _carriageSqueezeServo;
	
	private CARRIAGE_WHEELS_STATE _carriageWheelsState;
	
	private double _currentCarriageWheelsVBusCmd = .45;
	//private double _servoTargetPosition = 0.1;
	
	// private double _currentInFeedWheelsVBusCmd = INFEED_DRIVE_WHEELS_VBUS_COMMAND;
	
	private static final double CARRIAGE_WHEELS_INFEED_COMMAND = 0.5;
	private static final double CARRIAGE_WHEELS_VBUS_COMMAND_BUMP = 0.05;
	
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
		
		// Setup Carriage Servo Motors
		//_carriageSqueezeServo = new Servo(Constants.CARRIAGE_SERVO_PWM_ADDRESS);
		//_carriageSqueezeServo.set(0);
		
		//Setup Limit Switch
		_carriageLimitSwitch = new DigitalInput(Constants.CARRIAGE_LIMIT_SWITCH_DIO_PORT);
		
		_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED;
	}

	//=====================================================================================
	//Set Up Looper to run loop at 10ms interval (2x RoboRio Cycle Time)
	//=====================================================================================
	private final Loop _loop = new Loop() 
	{
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) 
		{
			synchronized (Carriage.this) 
			{
			}
		}
		
		//=====================================================================================
		//Looper and State Machine for Commanding Infeed Axis
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) 
		{
			synchronized (Carriage.this)
			{	
				
				switch(_carriageWheelsState)
				{
					case STOPPED:
						_carriageLeftMotor.set(ControlMode.PercentOutput, 0, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, 0, 0);
						break;
						
					case FEED_IN:
						_carriageLeftMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsVBusCmd, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsVBusCmd, 0);
						break;
						
					case FEED_OUT:
						_carriageLeftMotor.set(ControlMode.PercentOutput, -1 * _currentCarriageWheelsVBusCmd, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, -1 * _currentCarriageWheelsVBusCmd, 0);
						break;
						
					case JOYSTICK:
						_carriageLeftMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsVBusCmd, 0);
						_carriageRightMotor.set(ControlMode.PercentOutput, _currentCarriageWheelsVBusCmd, 0);
						break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) 
		{
			synchronized (Carriage.this) 
			{
				stop();
			}
		}
	};
	
	public Loop getLoop() 
	{
		return _loop;
	}
	
	@Override
	public void stop() 
	{
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.STOPPED) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [STOPPED]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED;
		}
	}

	public void infeedCarriageMotorsVBus(double vbusCmd) 
	{
		if(!isCubeInCarriage()) {
			// scale cmd
			_currentCarriageWheelsVBusCmd = vbusCmd * 0.5;
			FeedIn();
		} 
		else {
			stop();
		}
		
		ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [JOYSTICK]");
		_carriageWheelsState = CARRIAGE_WHEELS_STATE.JOYSTICK;
	}
	
	public void FeedIn() 
	{
		if(!isCubeInCarriage()) {
			//_carriageDriveCmd = CARRIAGE_WHEELS_INFEED_COMMAND;
			if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.FEED_IN) {
				ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [FEED_IN]");
				_carriageWheelsState = CARRIAGE_WHEELS_STATE.FEED_IN;
			}			
		} 
		else {
			if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.STOPPED) {
				ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [STOPPED]");
				_carriageWheelsState = CARRIAGE_WHEELS_STATE.STOPPED;
			}
		}
	}
	
	public void FeedOut() 
	{
		//_carriageDriveCmd = -1  * CARRIAGE_WHEELS_INFEED_COMMAND;
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.FEED_OUT) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [FEED_OUT]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.FEED_OUT;
		}
	}
	
	public void ejectCubeVBus(double joystickCommand) 
	{
		_currentCarriageWheelsVBusCmd = -1 * joystickCommand;
		
		if(_carriageWheelsState != CARRIAGE_WHEELS_STATE.JOYSTICK) {
			ReportStateChg("Carriage Wheels (State) " + _carriageWheelsState.toString() + " ==> [JOYSTICK]");
			_carriageWheelsState = CARRIAGE_WHEELS_STATE.JOYSTICK;
		}
	}
	
	//public void moveCarriageServosWider() {
	//	if(_servoTargetPosition != 1) {
	//		_servoTargetPosition = _servoTargetPosition + 0.05;
	//	}
	//}
	
	//public void moveCarriageServosCloser() {
	//	if(_servoTargetPosition != 0) {
	//		_servoTargetPosition = _servoTargetPosition - 0.05;
	//	}
	//}
	
	public void carriageWheels_VBusCmd_BumpUp() 
	{
		double newCmd = _currentCarriageWheelsVBusCmd + CARRIAGE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not over max
		if(newCmd <= 1.0) {
			_currentCarriageWheelsVBusCmd = newCmd;
		}
	}
	
	public void carriageWheels_VBusCmd_BumpDown() 
	{		
		double newCmd = _currentCarriageWheelsVBusCmd - CARRIAGE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not under min
		if(newCmd >= 0.0) {
			_currentCarriageWheelsVBusCmd = newCmd;
		}
	}
	
	public boolean isCubeInCarriage() 
	{
		return _carriageLimitSwitch.get();
	} 
	
	@Override
	public void zeroSensors() 
	{
		// N/A on this subsystem
	}

	private double getCarriageMotorCurrent()
	{
		return _carriageLeftMotor.getOutputCurrent();
	}
	
	@Override
	public void outputToShuffleboard() 
	{
		SmartDashboard.putNumber("Carriage Current:", getCarriageMotorCurrent());
		SmartDashboard.putNumber("Carriage Wheels %VBus", _currentCarriageWheelsVBusCmd);
		SmartDashboard.putString("Carriage State", _carriageWheelsState.toString());
		SmartDashboard.putBoolean("Carriage LimitSwitch", isCubeInCarriage());
	}

	@Override
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Carriage:LimitSwitch", String.valueOf(isCubeInCarriage()));
	}
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message) 
	{
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}