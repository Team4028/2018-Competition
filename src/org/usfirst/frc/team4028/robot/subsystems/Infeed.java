package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
//import org.usfirst.frc.team4028.robot.sensors.UltrasonicSensor;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Infeed {	
	//=====================================================================================
	//Define the Class Level Variables/Enums
	//=====================================================================================
	private enum INFEED_ARM_STATE {
		STOPPED,
		NEED_TO_HOME,
		MOVING_TO_HOME,
		AT_HOME,
		MOVE_TO_POSITION_AND_HOLD,
		TIMEOUT,
	} 

	public enum INFEED_ARM_TARGET_POSITION {
		HOME,
		INFEED,
		WIDE,
		SQUEEZE,
		STORE,
	}
		
	private enum INFEED_WHEELS_STATE {
		STOPPED,
		FEED_IN,
		FEED_OUT,
		SPIN_COUNTER_CLOCKWISE,
		SPIN_CLOCKWISE,
	}
		
	public enum INFEED_DRIVE_DIRECTION{
		UNDEFINED,
		IN,
		OUT
	}
	
	// define class level working variables
	private INFEED_ARM_STATE _infeedArmState;
	private INFEED_WHEELS_STATE _infeedWheelsState;
	
	//private UltrasonicSensor _ultrasonic;
	
	private boolean _hasLeftArmBeenHomed;
	private boolean _hasRightArmBeenHomed;
	//private boolean _isStaggerAtInitialPosition;
	
	private double _targetInfeedArmPosition = SQUEEZE_INFEED_POSITION_ANGLE;
	
	// use default value
	private double _currentInFeedWheelsVBusCmd = INFEED_DRIVE_WHEELS_VBUS_COMMAND;
	
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
            
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_VEL = 3000;
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_ACC = 2000;
    
    //private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_VEL = 7000;
    //private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_ACC = 6000;
    
    private static final int INFEED_ARM_FORWARD_SOFT_LIMIT = 3000;
	
	// Infeed Position Constants [THESE ARE ANGLE MEASURES IN DEGREES]
	private static final double HOME_POSITION_ANGLE = 0; //Is Home
    private static final double INFEED_POSITION_ANGLE = 160;	
	private static final double WIDE_INFEED_POSITION_ANGLE = 140;
	private static final double SQUEEZE_INFEED_POSITION_ANGLE = 190;
	private static final double STORE_POSITION_ANGLE = 10;
//	private static final double THIN_SIDE_POSITION_ANGLE = 230;
//	private static final double STAGGER_POSITION_ANGLE = 185;
	
	private static final double SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP = 1.0;
	
	private static final double INFEED_ALLOWED_ERROR_ANGLE = 5;
	
	// Infeed Drive Wheel Constant
	private static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND = 1.0;
	private static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP = 0.05;
	
	private static final double INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND = 0.2;
	
	//INfeed Homing Speed
	private static final double INFEED_HOMING_VBUS_COMMAND = 0.2;
	
	//Conversion Constant
	private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096/360);
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = true;
	
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
		_leftSwitchbladeMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
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
		
		_leftSwitchbladeMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_leftSwitchbladeMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
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
		_rightSwitchbladeMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
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
		
		_rightSwitchbladeMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_rightSwitchbladeMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Left Arm Drive Motor
		_leftInfeedDriveMotor = new TalonSRX(Constants.LEFT_INFEED_DRIVE_CAN_ADDRESS);
		_leftInfeedDriveMotor.setInverted(true);
			
		//=====================================================================================
		//Right Arm Drive Motor
		_rightInfeedDriveMotor = new TalonSRX(Constants.RIGHT_INFEED_DRIVE_CAN_ADDRESS);
		_rightInfeedDriveMotor.setInverted(true);
				
		//=====================================================================================
		//Set up Ultrasonic Sensor
		//_ultrasonic = UltrasonicSensor.getInstance();
		
		_leftSwitchbladeMotor.configPeakOutputForward(1, 0);
		_leftSwitchbladeMotor.configPeakOutputReverse(-1, 0);
		_rightSwitchbladeMotor.configPeakOutputForward(1, 0);
		_rightSwitchbladeMotor.configPeakOutputReverse(-1, 0);
		
		//Initially Configure Booleans
		_hasLeftArmBeenHomed = false;
		_hasRightArmBeenHomed = false;
		//_isStaggerAtInitialPosition = false;
		
		_infeedArmState = INFEED_ARM_STATE.NEED_TO_HOME;
		_infeedWheelsState = INFEED_WHEELS_STATE.STOPPED;
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
				switch(_infeedArmState) {
					case STOPPED:
						//_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
						//_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
						_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
						_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
						break;
						
					case NEED_TO_HOME:						
						_hasLeftArmBeenHomed = false;
						_hasRightArmBeenHomed = false;
						
						ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [MOVING_TO_HOME]");
						_infeedArmState = INFEED_ARM_STATE.MOVING_TO_HOME;						
						break;
						
					case MOVING_TO_HOME:
						// ==== left side ====
						// are we on the home l/s?
						if (_leftSwitchbladeMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
							// zero encoder
							_leftSwitchbladeMotor.setSelectedSensorPosition(0, 0, 0);
							// stop motor
							_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
							
							if(!_hasLeftArmBeenHomed)
							{
								ReportStateChg("Left Arm [AT_HOME]");
							}
							
							_hasLeftArmBeenHomed = true;
						}
						// only run this side if home not reached, may still be running the other side
						else if (_hasLeftArmBeenHomed == false) {	
							// run motor "backwards"
							_leftSwitchbladeMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND);
						}
						//else {
						//	_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
						//}
						
						// ==== right side ====
						// are we on the home l/s?
						if (_rightSwitchbladeMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
							// zero encoder
							_rightSwitchbladeMotor.setSelectedSensorPosition(0, 0, 0);
							// stop motor
							_rightSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
							
							if(!_hasRightArmBeenHomed)
							{
								ReportStateChg("Right Arm [AT_HOME]");
							}
							
							_hasRightArmBeenHomed = true;
						}
						// only run this side if home not reached, may still be running the other side
						else if (_hasRightArmBeenHomed == false) {
							// run motor "backwards"
							_rightSwitchbladeMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND);
						}
						
						// only when both arms have been homed
						if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
							ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [AT_HOME]");
							_infeedArmState = INFEED_ARM_STATE.AT_HOME;
						}

						break;
						
					case AT_HOME:
						// enable fwd soft limits
						_leftSwitchbladeMotor.configForwardSoftLimitEnable(true, 20);
						_rightSwitchbladeMotor.configForwardSoftLimitEnable(true, 20);
						
						// hold arms at home
						ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [MOVE_TO_POSITION_AND_HOLD]:[HOME]");
						_infeedArmState = INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD;
						MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.HOME);
						break;
											
					case MOVE_TO_POSITION_AND_HOLD:				
						_leftSwitchbladeMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
						_leftSwitchbladeMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
						
						_rightSwitchbladeMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
						_rightSwitchbladeMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
						
						//set appropriate gain slot in use
						if(_targetInfeedArmPosition == STORE_POSITION_ANGLE) {
							_leftSwitchbladeMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
						} else {
							_leftSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
						}
						
						// set target angle for both infeed arms
						_leftSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedArmPosition));
						_rightSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedArmPosition));
					 	break;
					 	
					//case AUTO_ACQUIRE_MANUVER:
					//	if(_ultrasonic.getIsCubeInRange()) {
					//		_infeedArmState = INFEED_ARM_STATE.STAGGER_INFEED_MANUVER;
					//	}
					//	break;
					 	
					//case STAGGER_INFEED_MANUVER:						
					//	_leftSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(STAGGER_POSITION_ANGLE));
					//	double positionLeftError = Math.abs(nativeUnitsToDegrees(_leftSwitchbladeMotor.getSelectedSensorPosition(0)) - STAGGER_POSITION_ANGLE);
					//	double positionRightError = Math.abs(nativeUnitsToDegrees(_rightSwitchbladeMotor.getSelectedSensorPosition(0)) - STAGGER_POSITION_ANGLE);
					//	if(positionLeftError < INFEED_ALLOWED_ERROR_ANGLE) {
					//		_rightSwitchbladeMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(STAGGER_POSITION_ANGLE));
					//	}
						
					//	if (positionLeftError < INFEED_ALLOWED_ERROR_ANGLE &&
					//			positionRightError < INFEED_ALLOWED_ERROR_ANGLE) {
					//		driveInfeedWheels();
					//	}
						
					//	if (_ultrasonic.getIsCubeInRobot()) {
					//		MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.INFEED);
							
					//		ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [MOVE_TO_POSITION_AND_HOLD]");
					//		_infeedArmState = INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD;
					//	}
					
					//	break;
						
					//case JOYSTICK_POSITION_CONTROL:
					//	break;
						
//					case FEED_IN:
//						_leftSwitchbladeMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
//						_rightSwitchbladeMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
//						break;
//						
//					case FEED_OUT:
//						_leftSwitchbladeMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
//						_rightSwitchbladeMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
//						break;
											
					//case DO_NOTHING:
					//	if(_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
					//		_leftSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
					//		_rightSwitchbladeMotor.set(ControlMode.PercentOutput, 0);
					//	}
					//	else {
					//		ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [NEED_TO_HOME]");
					//		_infeedArmState = INFEED_ARM_STATE.NEED_TO_HOME;
					//	}
					//	break;
						
					case TIMEOUT:
						DriverStation.reportWarning("InfeedAxis (State) [TIMEOUT] error homing axis", false);
						break;
				}
				
				switch (_infeedWheelsState)	{
					case STOPPED:
						_leftInfeedDriveMotor.set(ControlMode.PercentOutput,0);
						_rightInfeedDriveMotor.set(ControlMode.PercentOutput,0);
						break;
						
					case FEED_IN:
						_leftInfeedDriveMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						_rightInfeedDriveMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						break;
						
					case FEED_OUT:
						_leftInfeedDriveMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						_rightInfeedDriveMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						break;
						
					case SPIN_COUNTER_CLOCKWISE:
						_leftInfeedDriveMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						_rightInfeedDriveMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						break;
						
					case SPIN_CLOCKWISE:
						_leftInfeedDriveMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						_rightInfeedDriveMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
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
	public void MoveToPresetPosition(INFEED_ARM_TARGET_POSITION presetPosition) {
		ReportStateChg("Infeed Arm (State) " + _infeedArmState.toString() + " ==> [MOVE_TO_POSITION_AND_HOLD]:[" + presetPosition.toString() + "]");
		
		//_isStaggerAtInitialPosition = false;
		switch(presetPosition) {
			case HOME:
				_targetInfeedArmPosition = HOME_POSITION_ANGLE;
				break;
			case INFEED:
				 _targetInfeedArmPosition = INFEED_POSITION_ANGLE;
				 break;
			case WIDE:
				 _targetInfeedArmPosition = WIDE_INFEED_POSITION_ANGLE;
				 break;
			case SQUEEZE:
				 //_targetInfeedPosition = SQUEEZE_INFEED_POSITION_ANGLE;
				 _targetInfeedArmPosition = SQUEEZE_INFEED_POSITION_ANGLE;
				 break;
			//case THIN_SIDE:
			//	_targetInfeedPosition = THIN_SIDE_POSITION_ANGLE;
			//	break;
			case STORE:
				 _targetInfeedArmPosition = STORE_POSITION_ANGLE;
				 break;
		}
		
		_infeedArmState = INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD;
	}
	
	//=====================================================================================
	//Methods for Calling Positions of Infeed Arms
	//=====================================================================================
	public void storeArms() {
		if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
			MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.STORE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToInfeedPosition() {
		if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
			MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.INFEED);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToWideInfeedPosition() {
		if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
			MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	public void moveArmsToSqueezeInfeedPosition() {
		if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
			MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
		} else {
			DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
		}
	}
	
	//public void moveArmsToThinSideInfeedPosition() {
	//	if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
	//		MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.THIN_SIDE);
	//	} else {
	//		DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
	//	}
	//}
	
	//=====================================================================================
	//Methods for Handling Special Cases
	//=====================================================================================	
	//public void staggerInfeedManuver() {
	//	if(_areArmsHomed && isStaggerManuverSetup()) {
	//		_infeedArmState = INFEED_ARM_STATE.STAGGER_INFEED_MANUVER;
	//	} else {
	//		DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
	//	}
	//}
	
	//public void autoInfeedManuver() {
	//	if(_areArmsHomed && isStaggerManuverSetup()) {
	//		_infeedArmState = INFEED_ARM_STATE.AUTO_ACQUIRE_MANUVER;
	//	} else {
	//		DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
	//	}
	//}
	
	//public void infeedJoystickCommandedPosition(double joystickYAxis, double joystickXAxis) {
	//	if(_areArmsHomed && isStaggerManuverSetup()) {
	//		double newJoystickYAxis = 0;
	//		double newJoystickXAxis = 0;
			
	//		if (joystickYAxis > 0 && joystickXAxis < 0) {
	//			newJoystickYAxis = -1 * joystickYAxis;
	//			newJoystickXAxis = -1 * joystickXAxis;
	//		}
	//		else {
	//			newJoystickYAxis = joystickYAxis;
	//			newJoystickXAxis = joystickXAxis;
	//		}
			//_infeedState = INFEED_STATE.JOYSTICK_POSITION_CONTROL;
			
	//		double commandedAngle = (Math.atan(newJoystickYAxis)/(newJoystickXAxis));
	//		double commandedPosition = (degreesToNativeUnits(Math.toDegrees(commandedAngle)));
			
	//		_leftSwitchbladeMotor.set(ControlMode.MotionMagic, commandedPosition);
	//		_rightSwitchbladeMotor.set(ControlMode.MotionMagic, commandedPosition);
			
	//	}
	//	else {
	//		DriverStation.reportWarning("Function Not Avaliable until Arms are Homed", false);
	//	}
	//}
	
	public void moveArmsToSafePosition() {
		MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE);
	}
	
	public boolean areArmsInSafePosition() {
		if (getCurrentLeftInfeedPosition() <= (degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE) + 100) 
				&& getCurrentLeftInfeedPosition() <= (degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE) + 100)) {
			return true;
		} else {
			return false;
		}
	}
	
	public void reZeroArms() {
		_hasLeftArmBeenHomed = false;
		_hasRightArmBeenHomed = false;
		_infeedArmState = INFEED_ARM_STATE.NEED_TO_HOME;
	}
	
	//=====================================================================================
	//Method for Driving Infeed Wheels
	//=====================================================================================
	public void driveInfeedWheels() {
		_leftInfeedDriveMotor.set(ControlMode.PercentOutput, INFEED_DRIVE_WHEELS_VBUS_COMMAND);
		_rightInfeedDriveMotor.set(ControlMode.PercentOutput, INFEED_DRIVE_WHEELS_VBUS_COMMAND);
	}
	
	public void driveInfeedWheelsVBus(double joystickCommand) {
		if(areArmsInPosition()) {
			_leftInfeedDriveMotor.set(ControlMode.PercentOutput, joystickCommand);
			_rightInfeedDriveMotor.set(ControlMode.PercentOutput, -1 * joystickCommand);
		}
	}
	
	public void spinManuverInfeedWheels() {
		if(areArmsInPosition()) {
			_leftInfeedDriveMotor.set(ControlMode.PercentOutput, INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
			_rightInfeedDriveMotor.set(ControlMode.PercentOutput, INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
		}
	}
	
	//=====================================================================================
	//Method for Engr GamePad B
	//=====================================================================================
	
	public void engrGamepadB_FeedIn()
	{
		if(_infeedWheelsState != INFEED_WHEELS_STATE.FEED_IN) {
			ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_IN_MODE]");
			_infeedWheelsState = INFEED_WHEELS_STATE.FEED_IN;
		}
	}
	
	public void engrGamepadB_FeedOut()
	{
		if(_infeedWheelsState != INFEED_WHEELS_STATE.FEED_OUT) {
				ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_OUT_MODE]");
				_infeedWheelsState = INFEED_WHEELS_STATE.FEED_OUT;
		}
	}
	
	public void engrGamepadB_SpinLeft()
	{
		if(_infeedWheelsState != INFEED_WHEELS_STATE.SPIN_COUNTER_CLOCKWISE) {
			ReportStateChg("Infeed Wheel (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_SPIN_LEFT_MODE]");
		_infeedWheelsState = INFEED_WHEELS_STATE.SPIN_COUNTER_CLOCKWISE;
		}
	}
	
	public void engrGamepadB_SpinRight()
	{
		if(_infeedWheelsState != INFEED_WHEELS_STATE.SPIN_CLOCKWISE) {
			ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_SPIN_RIGHT_MODE]");
			_infeedWheelsState = INFEED_WHEELS_STATE.SPIN_CLOCKWISE;
		}
	}
	
	public void engrGamepadB_InfeedVBUS_BumpUp()
	{
		double newCmd = _currentInFeedWheelsVBusCmd + INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not over max
		if(newCmd <= 1.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}
	
	public void engrGamepadB_InfeedVBUS_BumpDown()
	{		
		double newCmd = _currentInFeedWheelsVBusCmd - INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not under min
		if(newCmd >= 0.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}
	
	public void engrGamepadB_SqueezeAngle_BumpNarrower()
	{
		double newTarget = _targetInfeedArmPosition + SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP;
		_targetInfeedArmPosition = newTarget;
	}
	
	public void engrGamepadB_SqueezeAngle_BumpWider()
	{
		double newTarget = _targetInfeedArmPosition - SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP;
		_targetInfeedArmPosition = newTarget;
	}
	
	//=====================================================================================
	//Method for determining if Arms are In Position
	//=====================================================================================
	/*
	private boolean isStaggerManuverSetup() {
		if (_targetInfeedPosition != INFEED_POSITION_ANGLE) {
			MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.INFEED);
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
			
			ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [STAGGER_INFEED_MANUVER]");
			_infeedArmState = INFEED_ARM_STATE.STAGGER_INFEED_MANUVER;
			return true;
		} else {
			return false;
		}
	}
	*/
		
	public boolean areArmsInPosition() {
		double currentErrorL = Math.abs(nativeUnitsToDegrees(getCurrentLeftInfeedPosition()) - _targetInfeedArmPosition);
		double currentErrorR = Math.abs(nativeUnitsToDegrees(getCurrentRightInfeedPosition()) - _targetInfeedArmPosition);
		
		if(currentErrorL < INFEED_ALLOWED_ERROR_ANGLE && currentErrorR < INFEED_ALLOWED_ERROR_ANGLE
				&& _targetInfeedArmPosition != HOME_POSITION_ANGLE
				&& _targetInfeedArmPosition != STORE_POSITION_ANGLE) {
			return true;
		} else if(_targetInfeedArmPosition == SQUEEZE_INFEED_POSITION_ANGLE) { 
			return true;
		} else {
			return false;
		}
	}
	
	//=====================================================================================
	//Methods for Commanding the Motors to Stop
	//=====================================================================================
	public void stopDriveMotors() {
		if(_infeedWheelsState != INFEED_WHEELS_STATE.STOPPED) {
			ReportStateChg("Infeed Wheels (State) " + _infeedWheelsState.toString() + " ==> [STOP]");
			_infeedWheelsState = INFEED_WHEELS_STATE.STOPPED;
		}
		
		//_leftInfeedDriveMotor.set(ControlMode.PercentOutput,0);
		//_rightInfeedDriveMotor.set(ControlMode.PercentOutput,0);
	}
	
	public void stop() {
		if(_infeedArmState != INFEED_ARM_STATE.STOPPED) {			
			if(_hasLeftArmBeenHomed 
					&& _hasRightArmBeenHomed
					&& _infeedArmState != INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD) {
				ReportStateChg("Infeed Arm (State) " + _infeedArmState.toString() + " ==> [STOP]");
				_infeedArmState = INFEED_ARM_STATE.STOPPED;
			}			
		}
		
		
		//_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
		//_leftSwitchbladeMotor.set(ControlMode.MotionMagic, 0);
		stopDriveMotors();
	}
	
	//=====================================================================================
	//Methods for Exposing Properties of Infeed Motors
	//=====================================================================================
	//public void doNothing() {
	//	_infeedArmState = INFEED_ARM_STATE.DO_NOTHING;
	//}
	
	public double getCurrentLeftInfeedPosition() {
		return _leftSwitchbladeMotor.getSelectedSensorPosition(0);
	}
	
	public double getCurrentRightInfeedPosition() {
		return _rightSwitchbladeMotor.getSelectedSensorPosition(0);
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
		SmartDashboard.putNumber("RP:", getCurrentRightInfeedPosition());
		SmartDashboard.putNumber("LP:", getCurrentLeftInfeedPosition());
		SmartDashboard.putNumber("Wide Infeed Position:", degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE));
		SmartDashboard.putBoolean("Are Arms Safe?", areArmsInSafePosition());
		SmartDashboard.putString("Infeed Wheels State", _infeedWheelsState.toString());
		SmartDashboard.putNumber("Infeed Wheels %VBus", _currentInFeedWheelsVBusCmd * 100);
		SmartDashboard.putNumber("Infeed Arm Target Angle", _targetInfeedArmPosition);
		SmartDashboard.putString("Infeed Arm State", _infeedArmState.toString());
	}
	
	// add data elements to be logged  to the input param (which is passed by ref)
	public void updateLogData(LogDataBE logData) {			
		logData.AddData("Left Infeed Position:", String.valueOf(getCurrentLeftInfeedPosition()));
		logData.AddData("Right Infeed Position:", String.valueOf(getCurrentRightInfeedPosition()));
	} 
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message) {
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}