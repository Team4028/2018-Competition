package org.usfirst.frc.team4028.robot.subsystems;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the elevator Subsystem
//=====> For Changes see Cade Reinberger
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		CadeR		22-Jan		Initial Version
//  1		TomB		24-Jan		Convert to use looper, implement state machine
//	2		TomB		25-Jan		Added Cade's feedback: error tol, inpos property
//	3		TomB		4-Feb		Adapted to real elevator
//	4		TomB		25.Feb		Resolve Elevator move on enable after disable when elevator was not at home
//-------------------------------------------------------------
public class Elevator implements Subsystem {
	// define enums for the elevator axis
	public enum ELEVATOR_STATE {
		NEED_TO_HOME,
		MOVING_TO_HOME,
		AT_HOME,
		TIMEOUT,
		GOTO_TARGET_POSITION,
		HOLD_TARGET_POSITION,
		MOVE_BELOW_SOFT_LIMIT,
		JOG_AXIS,
		//DO_NOTHING
	}
	
	public enum ELEVATOR_PRESET_POSITION {
		HOME,
		INFEED_HEIGHT,
		CUBE_ON_PYRAMID_LEVEL_1,
		SWITCH_HEIGHT,
		SCALE_HEIGHT,
		OTHER
	}
	
	// define class level working variables
	private TalonSRX _elevatorMasterMotor; //, _elevatorSlaveMotor;
	private ELEVATOR_STATE _elevatorState;
	private long _elevatorHomeStartTime;
	private int _targetElevatorPosition;
	private double _targetElevatorVelocity;
		
	private double _actualPositionNU = 0;
	private double _actualVelocityNU_100mS = 0;
	private double _actualAccelerationNU_100mS_mS = 0;
	private long _lastScanTimeStamp = 0;
	private double _lastScanActualVelocityNU_100mS = 0;
	private int _pidSlotInUse = -1;
	private boolean _isSoftLimitsEnabled = false;
	
	// define general constants
	public static final double NU_PER_INCH = 106.040268456;
	
	// hardcoded preset jogging velocities
	private static final double JOG_UP_VELOCITY = 0.5; //0.4; //0.70;
	private static final double JOG_DOWN_VELOCITY = -0.25; //-0.50;
	private static final double ELEVATOR_MOVE_TO_HOME_VELOCITY_CMD = -0.20;   
	private static final long ELEVATOR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC = 5000;	// 5 sec
	
	private static final double ELEVATOR_POS_ALLOWABLE_ERROR_IN_INCHES = 0.5;	// +/- 0.25
	private static final int ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU = InchesToNativeUnits(ELEVATOR_POS_ALLOWABLE_ERROR_IN_INCHES);
	
	//Conversion Constant
	public static final double NATIVE_UNITS_PER_INCH_CONVERSION = (28510/78.75);//(29638 / 41);
	
	// hardcoded preset positions (in native units, 0 = home position)
	private static final int SCALE_HEIGHT_POSITION = InchesToNativeUnits(80);
	private static final int SWITCH_HEIGHT_POSITION = InchesToNativeUnits(28);
	private static final int CUBE_ON_PYRAMID_LEVEL_1_POSITION = InchesToNativeUnits(24);
	private static final int CUBE_ON_FLOOR_POSITION = InchesToNativeUnits(0);
	private static final int INFEED_POSITION = 0;
	private static final int HOME_POSITION = 0;

//	private static final int ELEVATOR_MAX_TRAVEL = InchesToNativeUnits(41);
	private static final int UP_SOFT_LIMIT = InchesToNativeUnits(90.0);
	private static final int DOWN_SOFT_LIMIT = InchesToNativeUnits(3.0);
	
	/*
	 * Moveable Slide Top to Bottom = 48.5 in
	 * 
	 * Travel (in)	Floor to bottom of Slide	Floor to Top Stop (in)	Encoder Native Units
	 * ===========	========================	======================	====================
	 * 		40.75		42.75					95						29638
	 * 			
	 * 		12			14						66.5					8736
	 * 		3			5						57.5				
	 * 		0			2						54.5					0
	 */
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = false;
	
	private static final int HOLDING_PID_SLOT_INDEX = 2;
	private static final int MOVING_UP_PID_SLOT_INDEX = 1;
	private static final int MOVING_DOWN_PID_SLOT_INDEX = 0;
	
	// define PID Constants
	public static final int UP_CRUISE_VELOCITY = 4061; // native units per 100 mSec 50% of max
	public static final int UP_ACCELERATION = 4500; //3061; 	// native units per 100 mSec per sec
	
	public static final int DOWN_CRUISE_VELOCITY = 3000;//2000; // native units per 100 mSec 50% of max
	public static final int DOWN_ACCELERATION = 2500; //1500; 	// native units per 100 mSec per sec
	
	public static final double FEED_FORWARD_GAIN_HOLD = 1.0; //1.5; //3.4074425;
	public static final double PROPORTIONAL_GAIN_HOLD  = 0.65; //.4; //0.0731; //3.0;
	public static final double INTEGRAL_GAIN_HOLD  = 0; //0.03; //0.0; 
	public static final int INTEGRAL_ZONE_HOLD = 0; //0.0; 
	public static final double DERIVATIVE_GAIN_HOLD  = 40; //4.0; //0.7;
	
	public static final double FEED_FORWARD_GAIN_UP = 0.4; //3.4074425;
	public static final double PROPORTIONAL_GAIN_UP = 1.5; //.4; //0.0731; //3.0;
	public static final double INTEGRAL_GAIN_UP = 0; //0.03; //0.0; 
	public static final int INTEGRAL_ZONE_UP = 0; //0.0; 
	public static final double DERIVATIVE_GAIN_UP = 75; //4.0; //0.7;
	
	public static final double FEED_FORWARD_GAIN_DOWN = 0.2;// 1.0; //3.4074425;
	public static final double PROPORTIONAL_GAIN_DOWN = 0.1; //.14; //2.0;
	public static final double INTEGRAL_GAIN_DOWN = 0; //0.03; //0.0; 
	public static final int INTEGRAL_ZONE_DOWN = 0; //200; //0.0; 
	public static final double DERIVATIVE_GAIN_DOWN = 2; //3.0; // 0.7;
	
	// singleton pattern
	private static Elevator _instance = new Elevator();
	
	public static Elevator getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private Elevator() {
		// config master & slave talon objects
		_elevatorMasterMotor = new TalonSRX(Constants.ELEVATOR_LIFT_MASTER_CAN_ADDRESS);

		// set motor phasing
		_elevatorMasterMotor.setInverted(false);
		
		// config limit switches
		_elevatorMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_elevatorMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

		// turn off all soft limits
		_elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
		_elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);

		// config brake mode
		_elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
		
		// config quad encoder & phase (invert = true)
		_elevatorMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		_elevatorMasterMotor.setSensorPhase(true);
		_elevatorMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		
		//configure the peak and nominal output voltages in both directions for both Talons
		_elevatorMasterMotor.configNominalOutputForward(0, 0);	// elimate rattle w/i deadband
		_elevatorMasterMotor.configNominalOutputReverse(0, 0);
		_elevatorMasterMotor.configPeakOutputForward(1, 0);
		_elevatorMasterMotor.configPeakOutputReverse(-1, 0);
		
		// config velocity measurement (2x of scan time, looper is 10 mS)
		_elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
		_elevatorMasterMotor.configVelocityMeasurementWindow(32, 0);
		
		// Setup MotionMagic Mode
		SetPidSlotToUse("constr", MOVING_DOWN_PID_SLOT_INDEX);
		
		// set closed loop gains		
		_elevatorMasterMotor.config_kF(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kP(MOVING_DOWN_PID_SLOT_INDEX, PROPORTIONAL_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kI(MOVING_DOWN_PID_SLOT_INDEX, INTEGRAL_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kD(MOVING_DOWN_PID_SLOT_INDEX, DERIVATIVE_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_IntegralZone(MOVING_DOWN_PID_SLOT_INDEX, INTEGRAL_ZONE_DOWN, 0);
		
		_elevatorMasterMotor.config_kF(MOVING_UP_PID_SLOT_INDEX, FEED_FORWARD_GAIN_UP, 0);
		_elevatorMasterMotor.config_kP(MOVING_UP_PID_SLOT_INDEX, PROPORTIONAL_GAIN_UP, 0);
		_elevatorMasterMotor.config_kI(MOVING_UP_PID_SLOT_INDEX, INTEGRAL_GAIN_UP, 0);
		_elevatorMasterMotor.config_kD(MOVING_UP_PID_SLOT_INDEX, DERIVATIVE_GAIN_UP, 0);
		_elevatorMasterMotor.config_IntegralZone(MOVING_UP_PID_SLOT_INDEX, INTEGRAL_ZONE_UP, 0);
		
		_elevatorMasterMotor.config_kF(HOLDING_PID_SLOT_INDEX, FEED_FORWARD_GAIN_HOLD, 0);
		_elevatorMasterMotor.config_kP(HOLDING_PID_SLOT_INDEX, PROPORTIONAL_GAIN_HOLD, 0);
		_elevatorMasterMotor.config_kI(HOLDING_PID_SLOT_INDEX, INTEGRAL_GAIN_HOLD, 0);
		_elevatorMasterMotor.config_kD(HOLDING_PID_SLOT_INDEX, DERIVATIVE_GAIN_HOLD, 0);
		_elevatorMasterMotor.config_IntegralZone(HOLDING_PID_SLOT_INDEX, INTEGRAL_ZONE_HOLD, 0);
		
		// set accel and cruise velocities
		_elevatorMasterMotor.configMotionCruiseVelocity(UP_CRUISE_VELOCITY, 0);
		_elevatorMasterMotor.configMotionAcceleration(UP_ACCELERATION, 0);
		
		// set allowable closed loop gain
		// +/- 0.25"
		_elevatorMasterMotor.configAllowableClosedloopError(0, ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU, 0);
		
		// set initial elevator state
		ReportStateChg("ElevatorAxis (State) [Startup] ==> [NEED_TO_HOME]");
		_elevatorState = ELEVATOR_STATE.NEED_TO_HOME;
		
		DisableSoftLimits();
	}
	
	// this is run by Looper typically at a 10mS interval (or 2x the RoboRio Scan time)
	private final Loop _loop = new Loop() {
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) {
			synchronized (Elevator.this) {}
		}
		
		// the goal is to have ALL motion controlled thru here
		// ie this is the place where in the case stmts, mtr ctrls are commanded to move
		@Override
		public void onLoop(double timestamp) {
			synchronized (Elevator.this) {
				long deltatime = 0;
				
				switch(_elevatorState) {
					case NEED_TO_HOME:
						// snapshot start time
						_elevatorHomeStartTime = System.currentTimeMillis();
						DisableSoftLimits();
						_elevatorMasterMotor.set(ControlMode.PercentOutput, 0);
						
						// change state
						ReportStateChg("ElevatorAxis (State) " + _elevatorState.toString() + "==> [MOVING_TO_HOME]");
						_elevatorState = ELEVATOR_STATE.MOVING_TO_HOME;
						break;
						
					case MOVING_TO_HOME:
						// are we on the rev (home) limit switch
						if(!_elevatorMasterMotor.getSensorCollection().isRevLimitSwitchClosed()) {
							
							// change state
							ReportStateChg("ElevatorAxis (State) " + _elevatorState.toString() + "==> [AT_HOME]");
							EnableSoftLimits();
							_elevatorState = ELEVATOR_STATE.AT_HOME;
						} 
						else {
		    				// check for timeout
		    				long elapsedTime = System.currentTimeMillis() - _elevatorHomeStartTime;
		    				if (elapsedTime >= ELEVATOR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC) {
								// change state
		    					ReportStateChg("ElevatorAxis (State) " + _elevatorState.toString() + "==> [TIMEOUT]");
		    					_elevatorState = ELEVATOR_STATE.TIMEOUT;
		    				} 
		    				else {
		    					// drive axis down in % vbus mode
		    					DisableSoftLimits();
		    					_elevatorMasterMotor.set(ControlMode.PercentOutput, ELEVATOR_MOVE_TO_HOME_VELOCITY_CMD);
		    				}
						}
						break;
						
					case AT_HOME:
						_elevatorMasterMotor.set(ControlMode.PercentOutput, 0);
						zeroSensors();
							
						// set default position after homing
						_targetElevatorPosition = CUBE_ON_FLOOR_POSITION;
						
						// change state
						ReportStateChg("ElevatorAxis (State) " + _elevatorState.toString() + " ==> [GOTO_TARGET_POSTION]:[CUBE_ON_FLOOR_POSITION]");
    					_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
						break;
											
					case GOTO_TARGET_POSITION:
						deltatime = (new Date().getTime()) - _lastScanTimeStamp;
						DisableSoftLimits();
						
						if(IsAtTargetPosition(_targetElevatorPosition)) {
							// change state
							ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [HOLD_TARGET_POSTION]:[" + _targetElevatorPosition +"]");
	    					_elevatorState = ELEVATOR_STATE.HOLD_TARGET_POSITION;
						} 
						else {
							_actualPositionNU = _elevatorMasterMotor.getSelectedSensorPosition(0);
							_actualVelocityNU_100mS  = _elevatorMasterMotor.getSelectedSensorVelocity(0);
							_actualAccelerationNU_100mS_mS = (_actualVelocityNU_100mS - _lastScanActualVelocityNU_100mS) / deltatime;
		
							// set appropriate gain slot to use
							if(_targetElevatorPosition > _actualPositionNU) {
								SetPidSlotToUse("GotoUp", MOVING_UP_PID_SLOT_INDEX);
							}
							else {
								SetPidSlotToUse("GotoDown", MOVING_DOWN_PID_SLOT_INDEX);
							}					
							_elevatorMasterMotor.set(ControlMode.MotionMagic, _targetElevatorPosition, 0);
						}
						_lastScanTimeStamp = new Date().getTime();
						_lastScanActualVelocityNU_100mS = _actualVelocityNU_100mS;					
						
						break;
						
					case HOLD_TARGET_POSITION:
						if(!IsAtTargetPosition(_targetElevatorPosition))
						{
							// change state
							ReportStateChg("ElevatorAxis (State) " + _elevatorState.toString() + " ==> [GOTO_TARGET_POSTION]:[" + _targetElevatorPosition  +"]");
	    					_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
						}
						//else if (_targetElevatorPosition == DOWN_SOFT_LIMIT) {
						//	_elevatorState = ELEVATOR_STATE.MOVE_BELOW_SOFT_LIMIT;
						//	ReportStateChg("ElevatorAxis (State) [GOTO_TARGET_POSTION] ==> [MOVE_BELOW_SOFT_LIMIT]");
						//}
						else
						{
							// set appropriate gain slot to use	    	
							SetPidSlotToUse("Hold", HOLDING_PID_SLOT_INDEX);
							_elevatorMasterMotor.set(ControlMode.MotionMagic, _targetElevatorPosition, 0);
						}
						
						break;
						
					//case MOVE_BELOW_SOFT_LIMIT:
					//	_elevatorMasterMotor.configReverseSoftLimitEnable(false, 20);
					//	_elevatorMasterMotor.set(ControlMode.PercentOutput, -0.1);
					//	break;
					
					case JOG_AXIS:
						deltatime = (new Date().getTime()) - _lastScanTimeStamp;
						
						EnableSoftLimits();
						
						_actualPositionNU = _elevatorMasterMotor.getSelectedSensorPosition(0);
						_actualVelocityNU_100mS  = _elevatorMasterMotor.getSelectedSensorVelocity(0);
						_actualAccelerationNU_100mS_mS = (_actualVelocityNU_100mS - _lastScanActualVelocityNU_100mS) / deltatime;
						
						_elevatorMasterMotor.set(ControlMode.PercentOutput, _targetElevatorVelocity);					
						
						_lastScanTimeStamp = new Date().getTime();
						_lastScanActualVelocityNU_100mS = _actualVelocityNU_100mS;
						
						if(_isSoftLimitsEnabled && (_actualPositionNU >= UP_SOFT_LIMIT)) {
							ReportStateChg("Elevator At Up Soft Limit!");
						} 
						else if (_isSoftLimitsEnabled && (_actualPositionNU <= DOWN_SOFT_LIMIT)) {
							ReportStateChg("Elevator At Down Soft Limit!");
						}
						break;
						
					//case DO_NOTHING:
					//	if(_elevatorState != ELEVATOR_STATE.NEED_TO_HOME) {
					//		_elevatorMasterMotor.set(ControlMode.PercentOutput, 0);
					//	}
					//	else {
					//		_elevatorState = ELEVATOR_STATE.NEED_TO_HOME;
					//	}
						
					//	break;
						
					case TIMEOUT:
						_elevatorMasterMotor.set(ControlMode.PercentOutput, 0);
						// add logic to control spamming
						ReportStateChg("ElevatorAxis (State) [TIMEOUT] error homing axis");
						break;
				}
			}
		}
		
		// run logic when the looper stops
		@Override
		public void onStop(double timestamp) {
			synchronized (Elevator.this) {
				stop();
			}
		}
	};
	
	// return the internal loop reference
	public Loop getLoop() {
		return _loop;
	}
	
	// =================================================================================================================
	// Public methods to move the elevator
	// =================================================================================================================
	
	// Support Operators Gamepad Buttons mapped to discrete positions
	public void MoveToPresetPosition(ELEVATOR_PRESET_POSITION presetPosition) 
	{
		// ignore move requests while in homing process
		if(_elevatorState != ELEVATOR_STATE.NEED_TO_HOME 
				&& _elevatorState != ELEVATOR_STATE.MOVING_TO_HOME)
		{
			switch(presetPosition) {
				case HOME:
					// limit spamming from CubeHandler
					if((_elevatorState !=ELEVATOR_STATE.GOTO_TARGET_POSITION && _elevatorState !=ELEVATOR_STATE.HOLD_TARGET_POSITION)
							|| _targetElevatorPosition != HOME_POSITION)
					{
						_targetElevatorPosition = HOME_POSITION;
						ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_TARGET_POSTION]:[HOME_POSITION]");
						_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
					}
					break;
					
				case INFEED_HEIGHT:
					if((_elevatorState !=ELEVATOR_STATE.GOTO_TARGET_POSITION && _elevatorState !=ELEVATOR_STATE.HOLD_TARGET_POSITION)
							|| _targetElevatorPosition != INFEED_POSITION)
					{
						_targetElevatorPosition = INFEED_POSITION;
						//if(_elevatorState != ELEVATOR_STATE.MOVE_BELOW_SOFT_LIMIT) {
						//	_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSTION;
						//}	
						ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_TARGET_POSTION]:[INFEED_POSITION]");
						_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
					}
					break;
					
				case CUBE_ON_PYRAMID_LEVEL_1:
					if((_elevatorState !=ELEVATOR_STATE.GOTO_TARGET_POSITION && _elevatorState !=ELEVATOR_STATE.HOLD_TARGET_POSITION)
							|| _targetElevatorPosition != CUBE_ON_PYRAMID_LEVEL_1_POSITION)
					{
						_targetElevatorPosition = CUBE_ON_PYRAMID_LEVEL_1_POSITION;
						ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_TARGET_POSTION]:[CUBE_ON_PYRAMID_LEVEL_1_POSITION]");
						_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
					}
					break;
					
				case SWITCH_HEIGHT:
					if((_elevatorState !=ELEVATOR_STATE.GOTO_TARGET_POSITION && _elevatorState !=ELEVATOR_STATE.HOLD_TARGET_POSITION)
							|| _targetElevatorPosition != SWITCH_HEIGHT_POSITION)
					{
						_targetElevatorPosition = SWITCH_HEIGHT_POSITION;
						ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_TARGET_POSTION]:[SWITCH_HEIGHT_POSITION]");
						_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
					}
					break;
					
				case SCALE_HEIGHT:
					if((_elevatorState !=ELEVATOR_STATE.GOTO_TARGET_POSITION && _elevatorState !=ELEVATOR_STATE.HOLD_TARGET_POSITION)
							|| _targetElevatorPosition != SCALE_HEIGHT_POSITION)
					{
						_targetElevatorPosition = SCALE_HEIGHT_POSITION;
						ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_TARGET_POSTION]:[SCALE_HEIGHT_POSITION]");
						_elevatorState = ELEVATOR_STATE.GOTO_TARGET_POSITION;
					}
					break;
					
				case OTHER:
					DriverStation.reportWarning("ElevatorAxis (State) ILLEGAL value for presetPosition", false);
					break;
			}
			
			// set appropriate gain slot to use (only flip if outside deadband
			int currentError = Math.abs(_elevatorMasterMotor.getSelectedSensorPosition(0) - _targetElevatorPosition);
            if (currentError > ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU) 
			{
				if(_targetElevatorPosition > _actualPositionNU) 
				{
					SetPidSlotToUse("MoveUp", MOVING_UP_PID_SLOT_INDEX);
				}
				else 
				{
					SetPidSlotToUse("MoveDown", MOVING_DOWN_PID_SLOT_INDEX);
				}
			}
		}
	}
	
	// support joystick like jogging but at a fixed velocity
	public void JogAxis(double speedCmd) {
		if(_elevatorState == ELEVATOR_STATE.GOTO_TARGET_POSITION 
				|| _elevatorState == ELEVATOR_STATE.HOLD_TARGET_POSITION 
				|| _elevatorState == ELEVATOR_STATE.AT_HOME
				|| _elevatorState == ELEVATOR_STATE.JOG_AXIS) {
			if(_elevatorState != ELEVATOR_STATE.JOG_AXIS) {
				// change the state
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [JOG_AXIS]");
				_elevatorState = ELEVATOR_STATE.JOG_AXIS;
			}
			
			if(_isSoftLimitsEnabled && ((_actualPositionNU >= UP_SOFT_LIMIT) || (_actualPositionNU <= DOWN_SOFT_LIMIT))) {
				_targetElevatorVelocity = 0;
			} else {
				if(speedCmd > 0) {
					// jog at a fixed rate in a + direction
					_targetElevatorVelocity = JOG_UP_VELOCITY;
				}
				else if(speedCmd < 0) {
					// jog at a fixed rate in a - direction
					_targetElevatorVelocity = JOG_DOWN_VELOCITY;
				}
			}
			
		} else {
			DriverStation.reportWarning("Wait until elevator is homed to jog", false);
		}
	}
	
	// implemented as active hold in place for now (vs just turning motors off)
	@Override
	public void stop() {		
		if(_elevatorState != ELEVATOR_STATE.GOTO_TARGET_POSITION
				&& _elevatorState != ELEVATOR_STATE.HOLD_TARGET_POSITION
				&& _elevatorState != ELEVATOR_STATE.NEED_TO_HOME
				&& _elevatorState != ELEVATOR_STATE.MOVING_TO_HOME
				&& _elevatorState != ELEVATOR_STATE.MOVE_BELOW_SOFT_LIMIT
				//&& _elevatorState != ELEVATOR_STATE.DO_NOTHING
				&& _elevatorState != ELEVATOR_STATE.TIMEOUT) {
			// set target to current location
			_targetElevatorPosition = _elevatorMasterMotor.getSelectedSensorPosition(0);
			
			// flip back to hold position mode using the current position
			if(_elevatorState != ELEVATOR_STATE.HOLD_TARGET_POSITION)
			{
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [HOLD_TARGET_POSTION]");
				_elevatorState = ELEVATOR_STATE.HOLD_TARGET_POSITION;
			}
			
			// cancel joystick cmd
			_targetElevatorVelocity = 0.0;
		}
	}


	public void rezeroElevator() {
		ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [NEED_TO_HOME]");
		_elevatorState = ELEVATOR_STATE.NEED_TO_HOME;
	}
	
	// reset (rzero) all sensors
	@Override
	public void zeroSensors() {
		_elevatorMasterMotor.setSelectedSensorPosition(0, 0, 0);
	}
	
	//public void doNothing() {
	//	ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [DO_NOTHING]");
	//	_elevatorState = ELEVATOR_STATE.DO_NOTHING;
	//}
	
	// =================================================================================================================
	// Private helper methods for the elevator
	// =================================================================================================================
	

	private void SetPidSlotToUse(String ref, int pidSlot)
	{
		if(pidSlot != _pidSlotInUse)
		{
			ReportStateChg("Chg Pid Slot: Ref: [" + ref + "] [" + _pidSlotInUse + "] => [" + pidSlot + "]");
			_pidSlotInUse = pidSlot;
			_elevatorMasterMotor.selectProfileSlot(_pidSlotInUse, 0);
			
			if(pidSlot == MOVING_UP_PID_SLOT_INDEX) {
				_elevatorMasterMotor.configMotionCruiseVelocity(UP_CRUISE_VELOCITY, 0);
				_elevatorMasterMotor.configMotionAcceleration(UP_ACCELERATION, 0);
			}
			else if(pidSlot == MOVING_DOWN_PID_SLOT_INDEX) {
				_elevatorMasterMotor.configMotionCruiseVelocity(DOWN_CRUISE_VELOCITY, 0);
				_elevatorMasterMotor.configMotionAcceleration(DOWN_ACCELERATION, 0);
			}
		}
	}
	
	private void EnableSoftLimits() {
		_elevatorMasterMotor.configReverseSoftLimitThreshold(DOWN_SOFT_LIMIT, 0);
		_elevatorMasterMotor.configForwardSoftLimitThreshold(UP_SOFT_LIMIT, 0);	
		
		_elevatorMasterMotor.configReverseSoftLimitEnable(true, 0);
		_elevatorMasterMotor.configForwardSoftLimitEnable(true, 0);		
		
		_isSoftLimitsEnabled = true;
	}
	
	private void DisableSoftLimits() {
		_elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);
		_elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
		
		_isSoftLimitsEnabled = false;
	}
	
	//=====================================================================================
	// Public Methods for Exposing Elevator Properties
	//=====================================================================================
	
	public ELEVATOR_STATE getElevatorState()
	{
		return _elevatorState;
	}
	
	// this property indicates if the elevator is w/i the position deadband of the target position
	public boolean IsAtTargetPosition(int targetPosition) {
        if (_elevatorState == ELEVATOR_STATE.GOTO_TARGET_POSITION
        		|| _elevatorState == ELEVATOR_STATE.HOLD_TARGET_POSITION) 
        {
        	int currentError = Math.abs(_elevatorMasterMotor.getSelectedSensorPosition(0) - targetPosition);

            if (currentError <= ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU) {
               	return true;
            } else {
            	return false;
            }
        } 
        else if (_elevatorState == ELEVATOR_STATE.JOG_AXIS) {
        	return false;
        } else {
        	return false;
        }
    }
	
	public boolean IsAtTargetPosition() {
		return IsAtTargetPosition(_targetElevatorPosition);
	}
	
	private double getElevatorActualPositionNU() {
		return _elevatorMasterMotor.getSelectedSensorPosition(0);
	}
	
	public boolean isElevatorAtInfeedPosition() {
		if(getElevatorActualPositionNU() < 100) {
			return true;
		}
		else {
			return false;
		}
	}
	
	// ===============================================================================================================
	// General Purpose Utility Methods
	// ===============================================================================================================
	private static int InchesToNativeUnits(double positionInInches) {
		int nativeUnits = (int)(positionInInches * NATIVE_UNITS_PER_INCH_CONVERSION);
		return nativeUnits;
	}
	
//	private static double NativeUnitsToInches(double nativeUnitsMeasure) {
//		double positionInInches = nativeUnitsMeasure / NATIVE_UNITS_PER_INCH_CONVERSION;
//		return positionInInches;
//	}
		
	// output data to the dashboard on the drivers station
	@Override
	public void outputToShuffleboard() {
		double actualPosition = 0;
		double actualVelocity = 0;
		double actualAcceleration = 0;
		
		boolean isDisplayNativeUnits = true;
		if(!isDisplayNativeUnits) {
			actualPosition =  _actualPositionNU / NU_PER_INCH;
			actualVelocity = 10 * _actualVelocityNU_100mS / NU_PER_INCH;
			actualAcceleration = 1000 * 10 * (_actualAccelerationNU_100mS_mS / NU_PER_INCH);
		} else {
			actualPosition = _actualPositionNU;
			actualVelocity = _actualVelocityNU_100mS;
			actualAcceleration = _actualAccelerationNU_100mS_mS;			
		}
				
		SmartDashboard.putNumber("Elevator:Current", _elevatorMasterMotor.getOutputCurrent());
		//SmartDashboard.putNumber("Elevator:VoltageCmd", elevatorCommandedVoltage);
		SmartDashboard.putNumber("Elevator:VoltageActual", _elevatorMasterMotor.getMotorOutputVoltage());
		
		SmartDashboard.putNumber("Elevator:Position", actualPosition);
		SmartDashboard.putNumber("Elevator:Velocity", GeneralUtilities.RoundDouble(actualVelocity, 2));
		SmartDashboard.putNumber("Elevator:Acceleration", GeneralUtilities.RoundDouble(actualAcceleration, 2));

		SmartDashboard.putNumber("Elevator:TargetPosition",_targetElevatorPosition);
		SmartDashboard.putBoolean("Elevator:IsInPosition", IsAtTargetPosition());
		SmartDashboard.putString("Elevator:State", _elevatorState.toString());
	}
	
	// add data elements to be logged  to the input param (which is passed by ref)
	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Elevator:PostionNu", String.valueOf(_actualPositionNU));	
		logData.AddData("Elevator:VelocityNu", String.valueOf(_actualVelocityNU_100mS));	
		logData.AddData("Elevator:AccelNu", String.valueOf(_actualAccelerationNU_100mS_mS));	
		logData.AddData("Elevator:State", _elevatorState.toString());	
	}
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message) {
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}