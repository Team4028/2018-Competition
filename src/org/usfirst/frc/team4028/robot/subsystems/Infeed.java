package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.sensors.UltrasonicSensor;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infeed {	
	//=====================================================================================
	//Define the Class Level Variables/Enums
	//=====================================================================================
	private enum INFEED_STATE {
		NEED_TO_HOME,
		MOVING_TO_HOME,
		MOVE_TO_POSITION_AND_HOLD,
		AUTO_ACQUIRE_MANUVER,
		STAGGER_INFEED_MANUVER,
		JOYSTICK_POSITION_CONTROL,
		DO_NOTHING,
		TIMEOUT,
	} 
	
	public enum INFEED_TARGET_POSITION {
		HOME,
		INFEED,
		WIDE,
		SQUEEZE,
		THIN_SIDE,
		STORE,
	}
		
	// define class level working variables
	private INFEED_STATE _infeedState;
	
	private UltrasonicSensor _ultrasonic;
	
	private boolean _isLeftArmHomed;
	private boolean _isRightArmHomed;
	private boolean _areArmsHomed;
	private boolean _isStaggerAtInitialPosition;
	
	private double _targetInfeedPosition;
	private double _leftSwitchbladeActualPosition;
	private double _rightSwitchbladeActualPosition;
	
	TalonSRX _leftSwitchbladeMotor; 
	TalonSRX _rightSwitchbladeMotor;
	TalonSRX _leftInfeedDriveMotor;
	TalonSRX _rightInfeedDriveMotor;
	
	//====================================================================================
	//	Closed Loop Gains for Infeed Motors
	//====================================================================================
	// PID gains for infeed
	private static final int INFEED_POSITIONS_PID_SLOT_INDEX = 0; //Infeed
	private static final int STORING_ARMS_PID_SLOT_INDEX = 1; //Store
	private static final int STAGGER_MANUVER_PID_SLOT_INDEX = 2;
	
	private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_P = 1.5;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_D = 0;			// |
    																			// |
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_P = 1.5;
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_D = 0;
    
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_P = 3.0;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_D = 0;			// |
    																			// |
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_P = 3.0; 
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_D = 0;
    
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P = 1.8;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D = 0;			// |
    																			// |
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P = 1.8;
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D = 0;
            
    private static final int INFEED_MOTION_MAGIC_MAX_VEL = 3000;
    private static final int INFEED_MOTION_MAGIC_MAX_ACC = 2000;
    
    private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_VEL = 7000;
    private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_ACC = 6000;
    
    private static final int INFEED_SWITCHBLADE_FORWARD_SOFT_LIMIT = 2000;
	
	// Infeed Position Constants [THESE ARE ANGLE MEASURES IN DEGREES]
	private static final double HOME_POSITION_ANGLE = 0; //Is Home
    private static final double INFEED_POSITION_ANGLE = 160;	
	private static final double WIDE_INFEED_POSITION_ANGLE = 140;
	private static final double SQUEEZE_INFEED_POSITION_ANGLE = 180;
	private static final double STORE_POSITION_ANGLE = 10;
	private static final double THIN_SIDE_POSITION_ANGLE = 200;
	private static final double STAGGER_POSITION_ANGLE = 185;
	
	private static final double INFEED_ALLOWED_ERROR_ANGLE = 20;
	
	// Infeed Drive Wheel Constant
	public static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND = 1.0;
	public static final double INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND = 0.2;
	
	//Conversion Constant
	public static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096/360);
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static Infeed _instance = new Infeed();
	
	public static Infeed getInstance() {
		return _instance;
	}
	
	private Infeed() {
		//====================================================================================
		//	Begin Setting Up Motors
		//====================================================================================
		
		//Left Arm Rotator Motor
		_leftSwitchbladeMotor = new TalonSRX(Constants.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
				
		_leftSwitchbladeMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_leftSwitchbladeMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_leftArmMotor.configOpenloopRamp(5, 0);
		
		_leftSwitchbladeMotor.setNeutralMode(NeutralMode.Brake);
		
		_leftSwitchbladeMotor.configForwardSoftLimitEnable(false, 0);
		_leftSwitchbladeMotor.configReverseSoftLimitEnable(false, 0);
		_leftSwitchbladeMotor.configForwardSoftLimitThreshold(INFEED_SWITCHBLADE_FORWARD_SOFT_LIMIT, 20);
		
		_leftSwitchbladeMotor.setInverted(false);
		
		_leftSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_leftSwitchbladeMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_leftSwitchbladeMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_leftSwitchbladeMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_leftSwitchbladeMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		
		_leftSwitchbladeMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_leftSwitchbladeMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_leftSwitchbladeMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_leftSwitchbladeMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		
		_leftSwitchbladeMotor.config_kF(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F, 0);
		_leftSwitchbladeMotor.config_kP(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P, 0);
		_leftSwitchbladeMotor.config_kI(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I, 0);
		_leftSwitchbladeMotor.config_kD(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D, 0);
		
		_leftSwitchbladeMotor.configMotionCruiseVelocity(INFEED_MOTION_MAGIC_MAX_VEL, 0);
		_leftSwitchbladeMotor.configMotionAcceleration(INFEED_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Right Arm Rotator Motor
		_rightSwitchbladeMotor = new TalonSRX(Constants.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
		
		_rightSwitchbladeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		_rightSwitchbladeMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_rightSwitchbladeMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_rightArmMotor.configOpenloopRamp(5, 0);
		
		_rightSwitchbladeMotor.setNeutralMode(NeutralMode.Brake);
		
		_rightSwitchbladeMotor.configForwardSoftLimitEnable(false, 0);
		_rightSwitchbladeMotor.configReverseSoftLimitEnable(false, 0);
		_rightSwitchbladeMotor.configForwardSoftLimitThreshold(INFEED_SWITCHBLADE_FORWARD_SOFT_LIMIT, 20);
		
		_rightSwitchbladeMotor.setInverted(true);
		
		_rightSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_rightSwitchbladeMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_rightSwitchbladeMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_rightSwitchbladeMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_rightSwitchbladeMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		
		_rightSwitchbladeMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_rightSwitchbladeMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_rightSwitchbladeMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_rightSwitchbladeMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		
		_rightSwitchbladeMotor.config_kF(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F, 0);
		_rightSwitchbladeMotor.config_kP(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P, 0);
		_rightSwitchbladeMotor.config_kI(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I, 0);
		_rightSwitchbladeMotor.config_kD(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D, 0);
		
		_rightSwitchbladeMotor.configMotionCruiseVelocity(INFEED_MOTION_MAGIC_MAX_VEL, 0);
		_rightSwitchbladeMotor.configMotionAcceleration(INFEED_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Left Arm Drive Motor
		_leftInfeedDriveMotor = new TalonSRX(Constants.LEFT_INFEED_DRIVE_CAN_ADDRESS);
		_leftInfeedDriveMotor.setInverted(false);
			
		//=====================================================================================
		//Right Arm Drive Motor
		_rightInfeedDriveMotor = new TalonSRX(Constants.RIGHT_INFEED_DRIVE_CAN_ADDRESS);
		_rightInfeedDriveMotor.setInverted(true);
				
		//=====================================================================================
		//Set up Ultrasonic Sensor
		_ultrasonic = UltrasonicSensor.getInstance();
		
		_leftSwitchbladeMotor.configPeakOutputForward(1, 0);
		_leftSwitchbladeMotor.configPeakOutputReverse(-1, 0);
		_rightSwitchbladeMotor.configPeakOutputForward(1, 0);
		_rightSwitchbladeMotor.configPeakOutputReverse(-1, 0);
		
		//Initially Configure Booleans
		_isLeftArmHomed = false;
		_isRightArmHomed = false;
		_areArmsHomed = false;
		_isStaggerAtInitialPosition = false;
		
		_infeedState = INFEED_STATE.NEED_TO_HOME;
	}
	
	//=====================================================================================
	//Set Up Looper to run loop at 10ms interval (2x RoboRio Cycle Time)
	//=====================================================================================
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (Infeed.this) {
				
			}
		}
		
		//=====================================================================================
		//Looper and State Machine for Commanding Infeed Axis
		//=====================================================================================
		@Override
		public void onLoop(double timestamp) {
			synchronized (Infeed.this) {	
				switch(_infeedState) {
					case NEED_TO_HOME:
						_infeedState = INFEED_STATE.MOVING_TO_HOME;
						
						_areArmsHomed = false;
						_isLeftArmHomed = false;
						_isRightArmHomed = false;
						
						_leftSwitchbladeMotor.configForwardSoftLimitEnable(false, 20);
						_rightSwitchbladeMotor.configForwardSoftLimitEnable(false, 20);

						
						DriverStation.reportWarning("InfeedAxis (State) [NEED_TO_HOME] ==> [MOVING_TO_HOME]", false);
						break;
						
					case MOVING_TO_HOME:
						homeArms();
						break;
											
					case MOVE_TO_POSITION_AND_HOLD:				
						_leftSwitchbladeMotor.configMotionCruiseVelocity(INFEED_MOTION_MAGIC_MAX_VEL, 0);
						_rightSwitchbladeMotor.configMotionCruiseVelocity(INFEED_MOTION_MAGIC_MAX_VEL, 0);
						_leftSwitchbladeMotor.configMotionAcceleration(INFEED_MOTION_MAGIC_MAX_ACC, 0);
						_rightSwitchbladeMotor.configMotionAcceleration(INFEED_MOTION_MAGIC_MAX_ACC, 0);
						
						//set appropriate gain slot in use
						if(_targetInfeedPosition == STORE_POSITION_ANGLE) {
							_leftSwitchbladeMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
						}
						else {
							_leftSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
						}
						
						_leftSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedPosition));
						_rightSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedPosition));
					 	break;
					 	
					case AUTO_ACQUIRE_MANUVER:
						if(_ultrasonic.getIsCubeInRange()) {
							_infeedState = INFEED_STATE.STAGGER_INFEED_MANUVER;
						}
						break;
					 	
					case STAGGER_INFEED_MANUVER:						
						_leftSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(STAGGER_POSITION_ANGLE));
						double positionLeftError = Math.abs(nativeUnitsToDegrees(_leftSwitchbladeMotor.getSelectedSensorPosition(0)) - STAGGER_POSITION_ANGLE);
						double positionRightError = Math.abs(nativeUnitsToDegrees(_rightSwitchbladeMotor.getSelectedSensorPosition(0)) - STAGGER_POSITION_ANGLE);
						if(positionLeftError < INFEED_ALLOWED_ERROR_ANGLE) {
							_rightSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(STAGGER_POSITION_ANGLE));
						}
						
						if (positionLeftError < INFEED_ALLOWED_ERROR_ANGLE &&
								positionRightError < INFEED_ALLOWED_ERROR_ANGLE) {
							driveInfeedWheels();
						}
						
						if (_ultrasonic.getIsCubeInRobot()) {
							MoveToPresetPosition(INFEED_TARGET_POSITION.INFEED);
							_infeedState = INFEED_STATE.MOVE_TO_POSITION_AND_HOLD;
						}
					
						break;
						
					case JOYSTICK_POSITION_CONTROL:
						break;
					
					case DO_NOTHING:
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
	
	//=====================================================================================
	//Supports Button Mapping to Pre-Set Positions while staying in Same State
	//=====================================================================================
	public void MoveToPresetPosition(INFEED_TARGET_POSITION presetPosition) {
		_infeedState = INFEED_STATE.MOVE_TO_POSITION_AND_HOLD;
		_isStaggerAtInitialPosition = false;
		switch(presetPosition) {
			case HOME:
				_targetInfeedPosition = HOME_POSITION_ANGLE;
				break;
			case INFEED:
				 _targetInfeedPosition = INFEED_POSITION_ANGLE;
				 break;
			case WIDE:
				 _targetInfeedPosition = WIDE_INFEED_POSITION_ANGLE;
				 break;
			case SQUEEZE:
				 _targetInfeedPosition = SQUEEZE_INFEED_POSITION_ANGLE;
				 break;
			case THIN_SIDE:
				_targetInfeedPosition = THIN_SIDE_POSITION_ANGLE;
				break;
			case STORE:
				 _targetInfeedPosition = STORE_POSITION_ANGLE;
				 break;
			}
	}
	
	//=====================================================================================
	//Method for Homing Arms
	//=====================================================================================
	private void homeArms() {
		if (_leftSwitchbladeMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_leftSwitchbladeMotor.setSelectedSensorPosition(0, 0, 0);
			_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
			_isLeftArmHomed = true;
		}
		else if (_isLeftArmHomed == false) {
			_leftSwitchbladeMotor.set(ControlMode.PercentOutput, -.1);
		}
		else {
			_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
		}
		
		if (_rightSwitchbladeMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_rightSwitchbladeMotor.setSelectedSensorPosition(0, 0, 0);
			_rightSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
			_isRightArmHomed = true;
		}
		else if (_isRightArmHomed == false) {
			_rightSwitchbladeMotor.set(ControlMode.PercentOutput, -.1);
		}
		else {
			_rightSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
		}
		
		if (_isRightArmHomed && _isLeftArmHomed) {
			_leftSwitchbladeMotor.configForwardSoftLimitEnable(true, 20);
			_rightSwitchbladeMotor.configForwardSoftLimitEnable(true, 20);
			
			_areArmsHomed = true;
			MoveToPresetPosition(INFEED_TARGET_POSITION.HOME);
			_infeedState = INFEED_STATE.MOVE_TO_POSITION_AND_HOLD;
		}
	}
	
	//=====================================================================================
	//Methods for Calling Positions of Infeed Arms
	//=====================================================================================
	public void storeArms() {
		if (_areArmsHomed) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.STORE);
		}else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToInfeedPosition() {
		if (_areArmsHomed) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.INFEED);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToWideInfeedPosition() {
		if (_areArmsHomed) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.WIDE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToSqueezeInfeedPosition() {
		if (_areArmsHomed) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.SQUEEZE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToThinSideInfeedPosition() {
		if (_areArmsHomed) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.THIN_SIDE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	//=====================================================================================
	//Methods for Handling Special Cases
	//=====================================================================================	
	public void staggerInfeedManuver() {
		if(_areArmsHomed && isStaggerManuverSetup()) {
			_infeedState = INFEED_STATE.STAGGER_INFEED_MANUVER;
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void autoInfeedManuver() {
		if(_areArmsHomed && isStaggerManuverSetup()) {
			_infeedState = INFEED_STATE.AUTO_ACQUIRE_MANUVER;
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void infeedJoystickCommandedPosition(double joystickYAxis, double joystickXAxis) {
		if (_areArmsHomed) {
			double newJoystickYAxis = 0;
			double newJoystickXAxis = 0;
			
			if (joystickYAxis > 0 && joystickXAxis < 0) {
				newJoystickYAxis = -1 * joystickYAxis;
				newJoystickXAxis = -1 * joystickXAxis;
			}
			else {
				newJoystickYAxis = joystickYAxis;
				newJoystickXAxis = joystickXAxis;
			}
			//_infeedState = INFEED_STATE.JOYSTICK_POSITION_CONTROL;
			
			double commandedAngle = (Math.atan(newJoystickYAxis)/(newJoystickXAxis));
			double commandedPosition = (degreesToNativeUnits(Math.toDegrees(commandedAngle)));
			
			_leftSwitchbladeMotor.set(ControlMode.MotionMagic, commandedPosition);
			_rightSwitchbladeMotor.set(ControlMode.MotionMagic, commandedPosition);
			
		}
	}
	
	public void reZeroArms() {
		_infeedState = INFEED_STATE.NEED_TO_HOME;
	}
	
	//=====================================================================================
	//Method for Driving Infeed Wheels
	//=====================================================================================
	public void driveInfeedWheels() {
		/*
		if(areArmsInPosition() || _infeedState == INFEED_STATE.STAGGER_INFEED_MANUVER) {
			_leftInfeedDriveMotor.setSpeed(-1*INFEED_DRIVE_WHEELS_VBUS_COMMAND);
			_rightInfeedDriveMotor.setSpeed(-1*-1*INFEED_DRIVE_WHEELS_VBUS_COMMAND);
		} */
		_leftInfeedDriveMotor.set(ControlMode.PercentOutput, -1*INFEED_DRIVE_WHEELS_VBUS_COMMAND);
		_rightInfeedDriveMotor.set(ControlMode.PercentOutput,-1*-1*INFEED_DRIVE_WHEELS_VBUS_COMMAND);
	}
	
	public void driveInfeedWheelsVBus(double joystickCommand) {
		if(areArmsInPosition()) {
			_leftInfeedDriveMotor.set(ControlMode.PercentOutput, joystickCommand);
			_rightInfeedDriveMotor.set(ControlMode.PercentOutput, -1 * joystickCommand);
		}
	}
	
	public void spinManuverInfeedWheels() {
		if(areArmsInPosition()) {
			_leftInfeedDriveMotor.set(ControlMode.PercentOutput,INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
			_rightInfeedDriveMotor.set(ControlMode.PercentOutput, INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
		}
	}
	
	//=====================================================================================
	//Method for determining if Arms are In Position
	//=====================================================================================
	private boolean isStaggerManuverSetup() {
		if (_targetInfeedPosition != INFEED_POSITION_ANGLE) {
			MoveToPresetPosition(INFEED_TARGET_POSITION.INFEED);
			return false;
		}
		double errorLeftSwitchblade = Math.abs(nativeUnitsToDegrees(_leftSwitchbladeMotor.getSelectedSensorPosition(0)) - INFEED_POSITION_ANGLE);
		double errorRightSwitchblade = Math.abs(nativeUnitsToDegrees(_rightSwitchbladeMotor.getSelectedSensorPosition(0)) - INFEED_POSITION_ANGLE);
		
		if(errorLeftSwitchblade < INFEED_ALLOWED_ERROR_ANGLE && errorRightSwitchblade< INFEED_ALLOWED_ERROR_ANGLE) {
			_isStaggerAtInitialPosition = true;
		}
		
		if (_isStaggerAtInitialPosition) {			
			_leftSwitchbladeMotor.selectProfileSlot(STAGGER_MANUVER_PID_SLOT_INDEX, 0);
			_rightSwitchbladeMotor.selectProfileSlot(STAGGER_MANUVER_PID_SLOT_INDEX, 0);
			_leftSwitchbladeMotor.configMotionCruiseVelocity(INFEED_STAGGER_MOTION_MAGIC_MAX_VEL, 0);
			_rightSwitchbladeMotor.configMotionCruiseVelocity(INFEED_STAGGER_MOTION_MAGIC_MAX_VEL, 0);
			_leftSwitchbladeMotor.configMotionAcceleration(INFEED_STAGGER_MOTION_MAGIC_MAX_ACC, 0);
			_rightSwitchbladeMotor.configMotionAcceleration(INFEED_STAGGER_MOTION_MAGIC_MAX_ACC, 0);
			
			_infeedState = INFEED_STATE.STAGGER_INFEED_MANUVER;
			return true;
		} else {
			return false;
		}
	}
		
	public boolean areArmsInPosition() {
		double currentError = Math.abs(nativeUnitsToDegrees(_leftSwitchbladeMotor.getSelectedSensorPosition(0)) - _targetInfeedPosition);
		if(currentError < INFEED_ALLOWED_ERROR_ANGLE && _targetInfeedPosition != HOME_POSITION_ANGLE
				&& _targetInfeedPosition != STORE_POSITION_ANGLE) {
			return true;
		} else if(_targetInfeedPosition == SQUEEZE_INFEED_POSITION_ANGLE) { 
			return true;
		} else {
			return false;
		}
	}
	
	//=====================================================================================
	//Methods for Commanding the Motors to Stop
	//=====================================================================================
	public void stopDriveMotors() {
		_leftInfeedDriveMotor.set(ControlMode.PercentOutput,0);
		_rightInfeedDriveMotor.set(ControlMode.PercentOutput,0);
	}
	
	public void stop() {
		_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
		_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
		stopDriveMotors();
	}
	
	//=====================================================================================
	//Methods for Exposing Properties of Infeed Motors
	//=====================================================================================
	public void doNothing() {
		_infeedState = INFEED_STATE.DO_NOTHING;
	}
	//=====================================================================================
	//Methods for Conversions between Native Units and Degrees
	//=====================================================================================
	private double degreesToNativeUnits(double degreeMeasure) {
		double nativeUnits = degreeMeasure * DEGREES_TO_NATIVE_UNITS_CONVERSION;
		return nativeUnits;
	}
	
	private double nativeUnitsToDegrees(double nativeUnitsMeasure) {
		double degrees = nativeUnitsMeasure / DEGREES_TO_NATIVE_UNITS_CONVERSION;
		return degrees;
	}
	
	//=====================================================================================
	//Methods for Logging and Outputting to Shuffleboard
	//=====================================================================================	
	public void outputToShuffleboard() {
		SmartDashboard.putBoolean("Is the Infeed in Position?", areArmsInPosition());
	}
	
	// add data elements to be logged  to the input param (which is passed by ref)
	public void updateLogData(LogDataBE logData) {	
		_leftSwitchbladeActualPosition = _leftSwitchbladeMotor.getSelectedSensorPosition(0);
		_rightSwitchbladeActualPosition = _rightSwitchbladeMotor.getSelectedSensorPosition(0);
		
		logData.AddData("Left Infeed Position:", String.valueOf(_leftSwitchbladeActualPosition));
		logData.AddData("Left Infeed Position:", String.valueOf(_rightSwitchbladeActualPosition));
	} 
}