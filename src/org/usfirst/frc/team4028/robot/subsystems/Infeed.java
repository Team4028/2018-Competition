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

public class Infeed  implements Subsystem {	
	// define the Class Level Enums
	public enum INFEED_ARM_STATE {
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
		
	public enum INFEED_WHEELS_STATE {
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
	private INFEED_ARM_TARGET_POSITION _infeedArmTargetPosition;
	
	private boolean _hasLeftArmBeenHomed;
	private boolean _hasRightArmBeenHomed;
	//private boolean _isStaggerAtInitialPosition;
	private double _targetInfeedArmPosition;
	
	// supports bumping
	private double _currentInFeedArmSqueezeTargetAngle = INFEED_POSITION_ANGLE; // 198;
	private double _currentInFeedWheelsVBusCmd = .50; //.45;
	
	// motor controllers
	TalonSRX _leftSwitchbladeArmMotor; 
	TalonSRX _rightSwitchbladeArmMotor;
	TalonSRX _leftInfeedWheelMotor;
	TalonSRX _rightInfeedWheelMotor;
	
	//private UltrasonicSensor _ultrasonic;
	
	//====================================================================================
	//	Constants for Closed Loop Gains for Infeed Motors
	//====================================================================================
	// PID gains for infeed
	private static final int INFEED_POSITIONS_PID_SLOT_INDEX = 0; //Infeed
	private static final int STORING_ARMS_PID_SLOT_INDEX = 1; //Store
	private static final int STAGGER_MANUVER_PID_SLOT_INDEX = 2;
	
	private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_P = 1.5;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_D = 0;			// |
    private static final int LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE = 0;

    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_P = 1.5;
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_D = 0;
    private static final int RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE = 0;
    
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_P = 3.0;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_D = 0;			// |
    private static final int LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_IZONE = 0;																			
    
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_P = 3.0; 
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_D = 0;
    private static final int RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_IZONE = 0;
    
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P = 1.8;			//for testing, the F-values are very different
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I = 0;			// |
    private static final double LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D = 0;			// |
    private static final int LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_IZONE = 0;																			
    
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F = 0.3354098361;// V
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P = 1.8;
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I = 0;
    private static final double RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D = 0;
    private static final int RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_IZONE = 0;
            
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_VEL = 3000;
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_ACC = 2000;
    
    //private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_VEL = 7000;
    //private static final int INFEED_STAGGER_MOTION_MAGIC_MAX_ACC = 6000;
    
    private static final int INFEED_ARM_FORWARD_SOFT_LIMIT = 3000;
	
	// Infeed Position Constants [THESE ARE ANGLE MEASURES IN DEGREES]
	private static final double HOME_POSITION_ANGLE = 0; //Is Home
    private static final double INFEED_POSITION_ANGLE = 184; //160;	
	private static final double WIDE_INFEED_POSITION_ANGLE = 140;
	private static final double SQUEEZE_INFEED_POSITION_ANGLE = 190;

	private static final double STORE_POSITION_ANGLE = 10;
//	private static final double THIN_SIDE_POSITION_ANGLE = 230;
//	private static final double STAGGER_POSITION_ANGLE = 185;
	
	private static final double SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP = 1.0;
	
	private static final double INFEED_ALLOWED_ERROR_ANGLE = 5;
	
	// Infeed Drive Wheel Constant
	private static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND = 0.5;
	private static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP = 0.05;
	
	private static final double INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND = 0.2;
	
	//Infeed Homing Speed
	private static final double INFEED_HOMING_VBUS_COMMAND = 0.2;
	
	//Conversion Constant
	private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096/360);
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = false;
	
	// handles issue that arms on practice robot do not hit home limit switches
	//	at the same place
	private static final int LEFT_INFEED_ARM_ZERO_OFFSET = 96; //75;  // 150 > 200 > 100 > 50 > 75
	private static final int RIGHT_INFEED_ARM_ZERO_OFFSET = 0;
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static Infeed _instance = new Infeed();
	
	public static Infeed getInstance() {
		return _instance;
	}
	
	// private constructor
	private Infeed() {
		//====================================================================================
		//	Begin Setting Up Motors
		//====================================================================================
		
		//Left Arm Rotator Motor
		_leftSwitchbladeArmMotor = new TalonSRX(Constants.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
				
		_leftSwitchbladeArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_leftSwitchbladeArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_leftArmMotor.configOpenloopRamp(5, 0);
		
		_leftSwitchbladeArmMotor.setNeutralMode(NeutralMode.Brake);
		
		_leftSwitchbladeArmMotor.configForwardSoftLimitEnable(false, 0);
		_leftSwitchbladeArmMotor.configReverseSoftLimitEnable(false, 0);
		_leftSwitchbladeArmMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
		_leftSwitchbladeArmMotor.setInverted(false);
		
		_leftSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_leftSwitchbladeArmMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_leftSwitchbladeArmMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_leftSwitchbladeArmMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_leftSwitchbladeArmMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		_leftSwitchbladeArmMotor.config_IntegralZone(INFEED_POSITIONS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE, 0);
		
		_leftSwitchbladeArmMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_leftSwitchbladeArmMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_leftSwitchbladeArmMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_leftSwitchbladeArmMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		_leftSwitchbladeArmMotor.config_IntegralZone(STORING_ARMS_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STORE_MOTION_MAGIC_IZONE, 0);
		
		_leftSwitchbladeArmMotor.config_kF(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F, 0);
		_leftSwitchbladeArmMotor.config_kP(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P, 0);
		_leftSwitchbladeArmMotor.config_kI(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I, 0);
		_leftSwitchbladeArmMotor.config_kD(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D, 0);
		_leftSwitchbladeArmMotor.config_IntegralZone(STAGGER_MANUVER_PID_SLOT_INDEX, LEFT_SWITCHBLADE_STAGGER_MOTION_MAGIC_IZONE, 0);
		
		_leftSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_leftSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Right Arm Rotator Motor
		_rightSwitchbladeArmMotor = new TalonSRX(Constants.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
		
		_rightSwitchbladeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		_rightSwitchbladeArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_rightSwitchbladeArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		//_rightArmMotor.configOpenloopRamp(5, 0);
		
		_rightSwitchbladeArmMotor.setNeutralMode(NeutralMode.Brake);
		
		_rightSwitchbladeArmMotor.configForwardSoftLimitEnable(false, 0);
		_rightSwitchbladeArmMotor.configReverseSoftLimitEnable(false, 0);
		_rightSwitchbladeArmMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
		_rightSwitchbladeArmMotor.setInverted(true);
		
		_rightSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_rightSwitchbladeArmMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_rightSwitchbladeArmMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_rightSwitchbladeArmMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_rightSwitchbladeArmMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		_rightSwitchbladeArmMotor.config_IntegralZone(INFEED_POSITIONS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE, 0);
		
		_rightSwitchbladeArmMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_rightSwitchbladeArmMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_rightSwitchbladeArmMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_rightSwitchbladeArmMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		_rightSwitchbladeArmMotor.config_IntegralZone(STORING_ARMS_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STORE_MOTION_MAGIC_IZONE, 0);
		
		_rightSwitchbladeArmMotor.config_kF(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_F, 0);
		_rightSwitchbladeArmMotor.config_kP(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_P, 0);
		_rightSwitchbladeArmMotor.config_kI(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_I, 0);
		_rightSwitchbladeArmMotor.config_kD(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_D, 0);
		_rightSwitchbladeArmMotor.config_IntegralZone(STAGGER_MANUVER_PID_SLOT_INDEX, RIGHT_SWITCHBLADE_STAGGER_MOTION_MAGIC_IZONE, 0);
		
		_rightSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_rightSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Left Arm Drive Motor
		_leftInfeedWheelMotor = new TalonSRX(Constants.LEFT_INFEED_DRIVE_CAN_ADDRESS);
		_leftInfeedWheelMotor.setInverted(true);
			
		//=====================================================================================
		//Right Arm Drive Motor
		_rightInfeedWheelMotor = new TalonSRX(Constants.RIGHT_INFEED_DRIVE_CAN_ADDRESS);
		_rightInfeedWheelMotor.setInverted(true);
				
		//=====================================================================================
		//Set up Ultrasonic Sensor
		//_ultrasonic = UltrasonicSensor.getInstance();
		
		_leftSwitchbladeArmMotor.configPeakOutputForward(1, 0);
		_leftSwitchbladeArmMotor.configPeakOutputReverse(-1, 0);
		_rightSwitchbladeArmMotor.configPeakOutputForward(1, 0);
		_rightSwitchbladeArmMotor.configPeakOutputReverse(-1, 0);
		
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
						_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
						_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
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
						if (_leftSwitchbladeArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
							// zero encoder
							_leftSwitchbladeArmMotor.setSelectedSensorPosition(0, 0, 0);
							// stop motor
							_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
							
							if(!_hasLeftArmBeenHomed) {
								ReportStateChg("Left Arm [AT_HOME]");
							}
							
							_hasLeftArmBeenHomed = true;
						}
						// only run this side if home not reached, may still be running the other side
						else if (_hasLeftArmBeenHomed == false) {	
							// run motor "backwards"
							_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND);
						}
						// ==== right side ====
						// are we on the home l/s?
						if (_rightSwitchbladeArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
							// zero encoder
							_rightSwitchbladeArmMotor.setSelectedSensorPosition(0, 0, 0);
							// stop motor
							_rightSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
							
							if(!_hasRightArmBeenHomed)
							{
								ReportStateChg("Right Arm [AT_HOME]");
							}
							
							_hasRightArmBeenHomed = true;
						}
						// only run this side if home not reached, may still be running the other side
						else if (_hasRightArmBeenHomed == false) {
							// run motor "backwards"
							_rightSwitchbladeArmMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND);
						}
						
						// only when both arms have been homed
						if (_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
							ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [AT_HOME]");
							_infeedArmState = INFEED_ARM_STATE.AT_HOME;
						}
						break;
						
					case AT_HOME:
						_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
						_rightSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0);
						zeroSensors();
						
						// enable fwd soft limits
						_leftSwitchbladeArmMotor.configForwardSoftLimitEnable(true, 40);
						_rightSwitchbladeArmMotor.configForwardSoftLimitEnable(true, 40);
						
						// hold arms at home
						ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [MOVE_TO_POSITION_AND_HOLD]:[HOME]");
						_infeedArmState = INFEED_ARM_STATE.MOVE_TO_POSITION_AND_HOLD;
						MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.HOME);
						break;
											
					case MOVE_TO_POSITION_AND_HOLD:				
						_leftSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
						_leftSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
						
						_rightSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
						_rightSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
						
						//set appropriate gain slot in use
						if(_targetInfeedArmPosition == STORE_POSITION_ANGLE) {
							_leftSwitchbladeArmMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeArmMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
						} else {
							_leftSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
							_rightSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
						}
						
						// set target angle for both infeed arms
						if(_infeedArmTargetPosition == INFEED_ARM_TARGET_POSITION.SQUEEZE) {
							// update _targetInfeedArmPosition since it might have been bumped
							_targetInfeedArmPosition = _currentInFeedArmSqueezeTargetAngle;
						}
							
						_leftSwitchbladeArmMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedArmPosition));
						_rightSwitchbladeArmMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetInfeedArmPosition));
					 	break;
						
					case TIMEOUT:
						DriverStation.reportWarning("InfeedAxis (State) [TIMEOUT] error homing axis", false);
						break;
				}
				
				switch (_infeedWheelsState)	{
					case STOPPED:
						_leftInfeedWheelMotor.set(ControlMode.PercentOutput,0);
						_rightInfeedWheelMotor.set(ControlMode.PercentOutput,0);
						break;
						
					case FEED_IN:
						_leftInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						_rightInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						break;
						
					case FEED_OUT:
						_leftInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						_rightInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						break;
						
					case SPIN_COUNTER_CLOCKWISE:
						_leftInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						_rightInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
						break;
						
					case SPIN_CLOCKWISE:
						_leftInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
						_rightInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
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
		
		_infeedArmTargetPosition = presetPosition;
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
				 _targetInfeedArmPosition = _currentInFeedArmSqueezeTargetAngle;
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
	
	//=====================================================================================
	//Methods for Handling Special Cases
	//=====================================================================================	
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
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, INFEED_DRIVE_WHEELS_VBUS_COMMAND);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, INFEED_DRIVE_WHEELS_VBUS_COMMAND);
	}
	
	public void driveInfeedWheelsVBus(double joystickCommand) {
		if(areArmsInPosition()) {
			_leftInfeedWheelMotor.set(ControlMode.PercentOutput, joystickCommand);
			_rightInfeedWheelMotor.set(ControlMode.PercentOutput, -1 * joystickCommand);
		}
	}
	
	public void spinManuverInfeedWheels() {
		if(areArmsInPosition()) {
			_leftInfeedWheelMotor.set(ControlMode.PercentOutput, INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
			_rightInfeedWheelMotor.set(ControlMode.PercentOutput, INFEED_SPIN_CUBE_WHEELS_VBUS_COMMAND);
		}
	}
	
	//=====================================================================================
	//Method for Engr GamePad B
	//=====================================================================================
	public void infeedWheels_FeedIn() {
		if(_infeedWheelsState != INFEED_WHEELS_STATE.FEED_IN) {
			ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_IN_MODE]");
			_infeedWheelsState = INFEED_WHEELS_STATE.FEED_IN;
		}
	}
	
	public void infeedWheels_FeedOut() {
		if(_infeedWheelsState != INFEED_WHEELS_STATE.FEED_OUT) {
				ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_OUT_MODE]");
				_infeedWheelsState = INFEED_WHEELS_STATE.FEED_OUT;
		}
	}
	
	public void infeedWheels_SpinCube_CCW() {
		if(_infeedWheelsState != INFEED_WHEELS_STATE.SPIN_COUNTER_CLOCKWISE) {
			ReportStateChg("Infeed Wheel (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_SPIN_LEFT_MODE]");
		_infeedWheelsState = INFEED_WHEELS_STATE.SPIN_COUNTER_CLOCKWISE;
		}
	}
	
	public void infeedWheels_SpinCube_CW() {
		if(_infeedWheelsState != INFEED_WHEELS_STATE.SPIN_CLOCKWISE) {
			ReportStateChg("Infeed Arm (State) " + _infeedWheelsState.toString() + " ==> [ENGR_GAMEPAD_B_SPIN_RIGHT_MODE]");
			_infeedWheelsState = INFEED_WHEELS_STATE.SPIN_CLOCKWISE;
		}
	}
	
	public void infeedWheels_VBusCmd_BumpUp() {
		double newCmd = _currentInFeedWheelsVBusCmd + INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not over max
		if(newCmd <= 1.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}
	
	public void infeedWheels_VBusCmd_BumpDown() {		
		double newCmd = _currentInFeedWheelsVBusCmd - INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not under min
		if(newCmd >= 0.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}
	
	public void infeedArms_SqueezeAngle_BumpNarrower() {
		double newTarget = _currentInFeedArmSqueezeTargetAngle + SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP;
		_currentInFeedArmSqueezeTargetAngle = newTarget;
	}
	
	public void infeedArms_SqueezeAngle_BumpWider() {
		double newTarget = _currentInFeedArmSqueezeTargetAngle - SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP;
		_currentInFeedArmSqueezeTargetAngle = newTarget;
	}
	
	//=====================================================================================
	//Method for determining if Arms are In Position
	//=====================================================================================	
	public boolean areArmsInPosition() {
		double currentErrorL = Math.abs(nativeUnitsToDegrees(getCurrentLeftInfeedPosition()) - _targetInfeedArmPosition);
		double currentErrorR = Math.abs(nativeUnitsToDegrees(getCurrentRightInfeedPosition()) - _targetInfeedArmPosition);
		
		if(currentErrorL < INFEED_ALLOWED_ERROR_ANGLE && currentErrorR < INFEED_ALLOWED_ERROR_ANGLE
				&& _targetInfeedArmPosition != HOME_POSITION_ANGLE
				&& _targetInfeedArmPosition != STORE_POSITION_ANGLE) {
			return true;
		} else if(_targetInfeedArmPosition == _currentInFeedArmSqueezeTargetAngle) { 
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
		stopDriveMotors();
	}
	
	public void doNothing() {
		if(_hasLeftArmBeenHomed && _hasRightArmBeenHomed) {
			ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [STOPPED]");
			_infeedArmState = INFEED_ARM_STATE.STOPPED;
		} else {
			ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [NEED_TO_HOME]");
			_infeedArmState = INFEED_ARM_STATE.NEED_TO_HOME;
		}
	}
	
	//=====================================================================================
	//Methods for Exposing Properties of Infeed Motors
	//=====================================================================================
	private double getCurrentLeftInfeedPosition() {
		return _leftSwitchbladeArmMotor.getSelectedSensorPosition(0);
	}
	
	private double getCurrentRightInfeedPosition() {
		return _rightSwitchbladeArmMotor.getSelectedSensorPosition(0);
	}
	
	public INFEED_ARM_STATE getInfeedArmState() {
		return _infeedArmState;
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
	public void outputToShuffleboard() 
	{
		SmartDashboard.putString("InfeedWheels:State", _infeedWheelsState.toString());
		SmartDashboard.putNumber("InfeedWheels:%VBus", _currentInFeedWheelsVBusCmd);
		
		SmartDashboard.putNumber("InfeedArms:Target Wide Angle:", degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE));
		SmartDashboard.putNumber("InfeedArms:Target Squeeze Angle", _currentInFeedArmSqueezeTargetAngle);
		
		SmartDashboard.putBoolean("InfeedArms:Left Homed?", _hasLeftArmBeenHomed);
		SmartDashboard.putBoolean("InfeedArms:Right Homed?", _hasRightArmBeenHomed);
		SmartDashboard.putBoolean("InfeedArms:Are Safe?", areArmsInSafePosition());		
		SmartDashboard.putBoolean("InfeedArms:InPosition?", areArmsInPosition());
		SmartDashboard.putString("InfeedArms:State", _infeedArmState.toString());
		
		SmartDashboard.putNumber("InfeedArms:Left Current PositionNU", getCurrentLeftInfeedPosition());
		SmartDashboard.putNumber("InfeedArms:Right Current PositionNU:", getCurrentRightInfeedPosition());
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

	@Override
	public void zeroSensors() {
		_leftSwitchbladeArmMotor.setSelectedSensorPosition(LEFT_INFEED_ARM_ZERO_OFFSET, 0, 0);
		_rightSwitchbladeArmMotor.setSelectedSensorPosition(RIGHT_INFEED_ARM_ZERO_OFFSET, 0, 0);
	}
}