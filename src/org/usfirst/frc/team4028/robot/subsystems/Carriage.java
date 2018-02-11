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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;

public class Carriage implements Subsystem {
	
	// define class level working variables
	private TalonSRX _carriageLeftMotor; 
	private VictorSPX _carriageRightMotor;
	
	private double _carriageVBusCmd;
	
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
		_carriageRightMotor = new VictorSPX(Constants.CARRIAGE_RIGHT_CAN_ADDRESS);
		
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
	}

	//=====================================================================================
	//Set Up Looper to run loop at 10ms interval (2x RoboRio Cycle Time)
	//=====================================================================================
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (Carriage.this) {
			}
		}
		
		//=====================================================================================
		//Looper and State Machine for Commanding Infeed Axis
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) {
			synchronized (Carriage.this) {	
				// joystick mode
				_carriageLeftMotor.set(ControlMode.PercentOutput, _carriageVBusCmd, 0);
				_carriageRightMotor.set(ControlMode.PercentOutput, _carriageVBusCmd, 0);
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
	
	@Override
	public void stop() {
		_carriageVBusCmd = 0.0;
		_carriageLeftMotor.set(ControlMode.PercentOutput, _carriageVBusCmd, 0);
		_carriageRightMotor.set(ControlMode.PercentOutput, _carriageVBusCmd, 0);
	}

	public void RunCarriageMotors(double vbusCmd)
	{
		// scale cmd
		_carriageVBusCmd = vbusCmd * 0.75;
	}
	
	@Override
	public void zeroSensors() {
	}

	@Override
	public void outputToShuffleboard() {
	}

	@Override
	public void updateLogData(LogDataBE logData) {
	}
}