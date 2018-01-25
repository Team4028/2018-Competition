package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;

public class Infeed {	
	
	// define enum for infeed axis
	private enum INFEED_STATE {
		NEED_TO_HOME,
		MOVING_TO_HOME,
		AT_HOME,
		MOVE_TO_INFEED,
		INFEED_CUBE,
		STORE_ARMS,
		TIMEOUT,
	} 
		
	// define class level working variables
	private INFEED_STATE _infeedState;
	
	Boolean _isLeftArmHomed;
	Boolean _isRightArmHomed;
	Boolean _areArmsHomed;
	Boolean _areArmsInInfeedPosition;
	
	TalonSRX _leftArmRotatorMotor; 
	TalonSRX _rightArmRotatorMotor;
	VictorSP _leftInfeedDriveMotor;
	VictorSP _rightInfeedDriveMotor;
	
	//singleton pattern 
	private static Infeed _instance = new Infeed();
	
	public static Infeed getInstance() {
		return _instance;
	}
	
	private Infeed() {
		
		//====================================================================================
		//	Begin Setting Up Motors
		//====================================================================================
		
		//Left Arm Rotator Motor
		_leftArmRotatorMotor = new TalonSRX(Constants.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
		
		_leftArmRotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		_leftArmRotatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_leftArmRotatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_leftArmMotor.configOpenloopRamp(5, 0);
		
		_leftArmRotatorMotor.setNeutralMode(NeutralMode.Brake);
		
		_leftArmRotatorMotor.configForwardSoftLimitEnable(false, 0);
		//_leftArmMotor.configForwardSoftLimitThreshold(2048, 20);
		
		_leftArmRotatorMotor.setInverted(false);
		
		_leftArmRotatorMotor.selectProfileSlot(0, 0);
		_leftArmRotatorMotor.config_kF(0, Constants.INFEED_MOTION_MAGIC_F, 0);
		_leftArmRotatorMotor.config_kP(0, Constants.INFEED_MOTION_MAGIC_P, 0);
		_leftArmRotatorMotor.config_kI(0, Constants.INFEED_MOTION_MAGIC_I, 0);
		_leftArmRotatorMotor.config_kD(0, Constants.INFEED_MOTION_MAGIC_D, 0);
		
		_leftArmRotatorMotor.configMotionCruiseVelocity(Constants.INFEED_MOTION_MAGIC_MAX_VEL, 0);
		_leftArmRotatorMotor.configMotionAcceleration(Constants.INFEED_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Right Arm Rotator Motor
		_rightArmRotatorMotor = new TalonSRX(Constants.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
		
		_rightArmRotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		_rightArmRotatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_rightArmRotatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_rightArmMotor.configOpenloopRamp(5, 0);
		
		_rightArmRotatorMotor.setNeutralMode(NeutralMode.Brake);
		
		_rightArmRotatorMotor.configForwardSoftLimitEnable(false, 0);
		//_rightArmMotor.configForwardSoftLimitThreshold(2048, 20);
		
		_rightArmRotatorMotor.setInverted(true);
		
		_rightArmRotatorMotor.config_kF(0, Constants.INFEED_MOTION_MAGIC_F, 0);
		_rightArmRotatorMotor.config_kP(0, Constants.INFEED_MOTION_MAGIC_P, 0);
		_rightArmRotatorMotor.config_kI(0, Constants.INFEED_MOTION_MAGIC_I, 0);
		_rightArmRotatorMotor.config_kD(0, Constants.INFEED_MOTION_MAGIC_D, 0);
		
		_rightArmRotatorMotor.configMotionCruiseVelocity(Constants.INFEED_MOTION_MAGIC_MAX_VEL, 0);
		_rightArmRotatorMotor.configMotionAcceleration(Constants.INFEED_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Left Arm Drive Motor
		_leftInfeedDriveMotor = new VictorSP(Constants.LEFT_INFEED_DRIVE_PWM_ADDRESS); 
			
		//=====================================================================================
		//Right Arm Drive Motor
		_rightInfeedDriveMotor = new VictorSP(Constants.RIGHT_INFEED_DRIVE_PWM_ADDRESS);
				
		//=====================================================================================
		
		//Initially Configure Booleans
		_isLeftArmHomed = false;
		_isRightArmHomed = false;
		_areArmsHomed = false;
		_areArmsInInfeedPosition = false;
		
		_infeedState = INFEED_STATE.NEED_TO_HOME;
	}
	
	
	// this is run by Looper typically at a 10mS interval (or 2x the RoboRio Scan time)
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (Infeed.this) {
			}
		}
		
		// the goal is to have ALL motion controlled thru here
		// ie this is the place where in the case stmts, mtr ctrls are commanded to move
		@Override
		public void onLoop(double timestamp) {
			synchronized (Infeed.this) {				
				switch(_infeedState) {
					case NEED_TO_HOME:
						_infeedState = INFEED_STATE.MOVING_TO_HOME;
						DriverStation.reportWarning("InfeedAxis (State) [NEED_TO_HOME] ==> [MOVING_TO_HOME]", false);
						break;
						
					case MOVING_TO_HOME:
						homeArms();
						break;
					
					case AT_HOME:
						break;
						
					case MOVE_TO_INFEED:
						_leftArmRotatorMotor.set(ControlMode.MotionMagic, Constants.INFEED_POSITION);
						_rightArmRotatorMotor.set(ControlMode.MotionMagic, Constants.INFEED_POSITION);
						
						if(Constants.INFEED_MINIMUM_ALLOWED_ERROR_POSITION 
								< _leftArmRotatorMotor.getSelectedSensorPosition(0) && 
								_leftArmRotatorMotor.getSelectedSensorPosition(0) < 
								Constants.INFEED_MAXIMUM_ALLOWED_ERROR_POSITION) {
							_areArmsInInfeedPosition = true;
							_infeedState = INFEED_STATE.INFEED_CUBE;
						}
					
						_infeedState = INFEED_STATE.NEED_TO_HOME;
						break;
						
					case INFEED_CUBE:
						_leftInfeedDriveMotor.set(0.5);
						_rightInfeedDriveMotor.set(0.5);
						break;
						
					case STORE_ARMS:
						
						break;
						
					case TIMEOUT:
						DriverStation.reportWarning("InfeedAxis (State) [TIMEOUT] error homing axis", false);
						break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Infeed.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private void homeArms() {
		System.out.println("Is reverse Limit Switch Closed: " + _leftArmRotatorMotor.getSensorCollection().isRevLimitSwitchClosed());
	
		if (_leftArmRotatorMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_leftArmRotatorMotor.setSelectedSensorPosition(0, 0, 0);
			_leftArmRotatorMotor.set(ControlMode.PercentOutput, 0);
			_isLeftArmHomed = true;
		}
		
		else if (_isLeftArmHomed == false) {
			_leftArmRotatorMotor.set(ControlMode.PercentOutput, -.15);
		}
		
		else {
			_leftArmRotatorMotor.set(ControlMode.PercentOutput, 0);
		}
		
		
		if (_rightArmRotatorMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_rightArmRotatorMotor.setSelectedSensorPosition(0, 0, 0);
			_rightArmRotatorMotor.set(ControlMode.PercentOutput, 0);
			_isRightArmHomed = true;
		}
		
		else if (_isRightArmHomed == false) {
			_rightArmRotatorMotor.set(ControlMode.PercentOutput, -.15);
		}
		
		else {
			_rightArmRotatorMotor.set(ControlMode.PercentOutput, 0);
		}
		
		if (_isRightArmHomed && _isLeftArmHomed) {
			_areArmsHomed = true;
			_infeedState = INFEED_STATE.AT_HOME;
		}
	}
	
//	private void displayVelocity() {
//		double armVelocity = _leftArmRotatorMotor.getSelectedSensorVelocity(0);
//		//SmartDashboard.putNumber("Left Arm Velocity:", armVelocity);
//	}
	
	public void moveArmsToInfeedPosition() {
		if (_areArmsHomed) {
			_infeedState = INFEED_STATE.MOVE_TO_INFEED;
		}
		else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void reZeroArms() {
		_areArmsHomed = false;
		_infeedState = INFEED_STATE.NEED_TO_HOME;
	}
	
//	public void MoveArms(double throttleCmd) {
//		if (_areArmsHomed == true) {
//			_leftArmRotatorMotor.set(ControlMode.PercentOutput, throttleCmd * Constants.INPUT_SCALE_FACTOR);
//		}
//		else {
//			_leftArmRotatorMotor.set(ControlMode.PercentOutput, 0);
//			DriverStation.reportWarning("Home Arms first", false);
//		}
//	}
	
	private void stop() {
		_leftArmRotatorMotor.set(ControlMode.PercentOutput, 0);
		_rightArmRotatorMotor.set(ControlMode.PercentOutput, 0);
		_leftInfeedDriveMotor.stopMotor();
		_rightInfeedDriveMotor.stopMotor();
	}
}

