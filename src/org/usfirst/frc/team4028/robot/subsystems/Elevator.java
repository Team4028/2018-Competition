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
//-------------------------------------------------------------
public class Elevator implements Subsystem 
{
	// define enums for the elevator axis
	private enum ELEVATOR_STATE {
		NEED_TO_HOME,
		MOVING_TO_HOME,
		AT_HOME,
		TIMEOUT,
		GOTO_AND_HOLD_TARGET_POSTION,
		JOG_AXIS,
	}
	
	public enum ELEVATOR_PRESET_POSITION {
		HOME,
		CUBE_ON_FLOOR,
		CUBE_ON_PYRAMID_LEVEL_1,
		SWITCH_HEIGHT,
		SCALE_HEIGHT,
		OTHER
	}
	
	// define class level working variables
	private TalonSRX _elevatorMasterMotor, _elevatorSlaveMotor;
	private ELEVATOR_STATE _elevatorState;
	private long _elevatorHomeStartTime;
	private int _targetElevatorPosition;
	private double _targetElevatorVelocity;
	
	private double _actualPositionNU = 0;
	private double _actualVelocityNU_100mS = 0;
	private double _actualAccelerationNU_100mS_mS = 0;
	private long _lastScanTimeStamp = 0;
	private double _lastScanActualVelocityNU_100mS = 0;
	
	// define general constants
	public static final double NU_PER_INCH = 106.040268456;
	
	private static final double ELEVATOR_MOVE_TO_HOME_VELOCITY_CMD = -0.25;   
	private static final long ELEVATOR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC = 5000;	// 5 sec
	
	private static final double ELEVATOR_POS_ALLOWABLE_ERROR_IN_INCHES = 0.25;	// +/- 0.25
	private static final int ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU = (int)Math.round(ELEVATOR_POS_ALLOWABLE_ERROR_IN_INCHES * NU_PER_INCH);
	
	// hardcoded preset positions (in native units, 0 = home position)
	private static final int HOME_POSITION = 0;
	private static final int CUBE_ON_FLOOR_POSITION = 100;
	private static final int CUBE_ON_PYRAMID_LEVEL_1_POSITION = 200;
	private static final int SWITCH_HEIGHT_POSITION = 1000;
	private static final int SCALE_HEIGHT_POSITION = 3605;
	
	// hardcoded preset jogging velocities
	private static final double JOG_UP_VELOCITY = 0.35;
	private static final double JOG_DOWN_VELOCITY = -0.25;
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = true;
	
	private static final int MOVING_UP_PID_SLOT_INDEX = 1;
	private static final int MOVING_DOWN_PID_SLOT_INDEX = 0;
	
	// define PID Constants
	public static final int CRUISE_VELOCITY = 30000;			// native units per 100 mSec
	public static final int ACCELERATION = 20000;				// native units per 100 mSec per sec
	
	public static final double FEED_FORWARD_GAIN_UP = 3.4074425;
	public static final double PROPORTIONAL_GAIN_UP = 3.0;
	public static final double INTEGRAL_GAIN_UP = 0.0; 
	public static final double DERIVATIVE_GAIN_UP = 0.7;
	
	public static final double FEED_FORWARD_GAIN_DOWN = 3.4074425;
	public static final double PROPORTIONAL_GAIN_DOWN = 2.0;
	public static final double INTEGRAL_GAIN_DOWN = 0.0; 
	public static final double DERIVATIVE_GAIN_DOWN = 0.7;
	
	// singleton pattern
	private static Elevator _instance = new Elevator();
	
	public static Elevator getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private Elevator()
	{
		// config master & slave talon objects
		_elevatorMasterMotor = new TalonSRX(Constants.ELEVATOR_MASTER_CAN_BUS_ADDR);
		_elevatorSlaveMotor = new TalonSRX(Constants.ELEVATOR_SLAVE_CAN_BUS_ADDR);
		
		// config slave mode
		_elevatorSlaveMotor.follow(_elevatorMasterMotor);
		
		// set motor phasing
		_elevatorMasterMotor.setInverted(false);
		_elevatorSlaveMotor.setInverted(false);
		
		// config limit switches
		_elevatorMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_elevatorMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_elevatorSlaveMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_elevatorSlaveMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		// turn off all soft limits
		_elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
		_elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);
		_elevatorSlaveMotor.configForwardSoftLimitEnable(false, 0);
		_elevatorSlaveMotor.configReverseSoftLimitEnable(false, 0);
		
		// config brake mode
		_elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
		_elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);
		
		// config quad encoder & phase (invert = true)
		_elevatorMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		_elevatorMasterMotor.setSensorPhase(true);
		_elevatorMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		
		//configure the peak and nominal output voltages in both directions for both Talons
		_elevatorMasterMotor.configNominalOutputForward(0, 0);
		_elevatorMasterMotor.configNominalOutputReverse(0, 0);
		_elevatorMasterMotor.configPeakOutputForward(1, 0);
		_elevatorMasterMotor.configPeakOutputReverse(-1, 0);
		_elevatorSlaveMotor.configNominalOutputForward(0, 0);
		_elevatorSlaveMotor.configNominalOutputReverse(0, 0);
		_elevatorSlaveMotor.configPeakOutputForward(1, 0);
		_elevatorSlaveMotor.configPeakOutputReverse(-1, 0);
		
		// config velocity measurement (2x of scan time, looper is 10 mS)
		_elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
		_elevatorMasterMotor.configVelocityMeasurementWindow(32, 0);
		
		// Setup MotionMagic Mode
		_elevatorMasterMotor.selectProfileSlot(MOVING_DOWN_PID_SLOT_INDEX, 0);
		
		// set closed loop gains
		_elevatorMasterMotor.config_kF(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kP(MOVING_DOWN_PID_SLOT_INDEX, PROPORTIONAL_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kI(MOVING_DOWN_PID_SLOT_INDEX, INTEGRAL_GAIN_DOWN, 0);
		_elevatorMasterMotor.config_kD(MOVING_DOWN_PID_SLOT_INDEX, DERIVATIVE_GAIN_DOWN, 0);
		
		_elevatorMasterMotor.config_kF(MOVING_UP_PID_SLOT_INDEX, FEED_FORWARD_GAIN_UP, 0);
		_elevatorMasterMotor.config_kP(MOVING_UP_PID_SLOT_INDEX, PROPORTIONAL_GAIN_UP, 0);
		_elevatorMasterMotor.config_kI(MOVING_UP_PID_SLOT_INDEX, INTEGRAL_GAIN_UP, 0);
		_elevatorMasterMotor.config_kD(MOVING_UP_PID_SLOT_INDEX, DERIVATIVE_GAIN_UP, 0);
		
		// set accel and cruise velocities
		_elevatorMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY, 0);
		_elevatorMasterMotor.configMotionAcceleration(ACCELERATION, 0);
		
		// set allowable closed loop gain
		// +/- 0.25"
		_elevatorMasterMotor.configAllowableClosedloopError(0, ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU, 0);
		
		// set initial elevator state
		_elevatorState = ELEVATOR_STATE.NEED_TO_HOME;
		ReportStateChg("ElevatorAxis (State) initial ==> [NEED_TO_HOME]");
	}
	
	// this is run by Looper typically at a 10mS interval (or 2x the RoboRio Scan time)
	private final Loop _loop = new Loop() 
	{
		// called in Telop & Auton Init
		@Override
		public void onStart(double timestamp) 
		{
			synchronized (Elevator.this) 
			{
			}
		}
		
		// the goal is to have ALL motion controlled thru here
		// ie this is the place where in the case stmts, mtr ctrls are commanded to move
		@Override
		public void onLoop(double timestamp) 
		{
			synchronized (Elevator.this) 
			{
				long deltatime = 0;
				
				switch(_elevatorState) 
				{
					case NEED_TO_HOME:
						// snapshot start time
						_elevatorHomeStartTime = System.currentTimeMillis();
						// change state
						_elevatorState = ELEVATOR_STATE.MOVING_TO_HOME;
						ReportStateChg("ElevatorAxis (State) [NEED_TO_HOME] ==> [MOVING_TO_HOME]");
						break;
						
					case MOVING_TO_HOME:
						// are we on the rev (home) limit switch
						if(!_elevatorMasterMotor.getSensorCollection().isRevLimitSwitchClosed())
						{
							// change state
							_elevatorState = ELEVATOR_STATE.AT_HOME;
							ReportStateChg("ElevatorAxis (State) [MOVING_TO_HOME] ==> [AT_HOME]");
						} 
						else 
						{
		    				// check for timeout
		    				long elapsedTime = System.currentTimeMillis() - _elevatorHomeStartTime;
		    				if (elapsedTime >= ELEVATOR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC) 
		    				{
								// change state
		    					_elevatorState = ELEVATOR_STATE.TIMEOUT;
		    					ReportStateChg("ElevatorAxis (State) [MOVING_TO_HOME] ==> [TIMEOUT]");
		    				} 
		    				else 
		    				{
		    					// drive axis down in % vbus mode
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
    					_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
    					ReportStateChg("ElevatorAxis (State) [AT_HOME] ==> [GOTO_AND_HOLD_TARGET_POSTION]");
						break;
											
					case GOTO_AND_HOLD_TARGET_POSTION:
						deltatime = (new Date().getTime()) - _lastScanTimeStamp;
						
						_actualPositionNU = _elevatorMasterMotor.getSelectedSensorPosition(0);
						_actualVelocityNU_100mS  = _elevatorMasterMotor.getSelectedSensorVelocity(0);
						_actualAccelerationNU_100mS_mS = (_actualVelocityNU_100mS - _lastScanActualVelocityNU_100mS) / deltatime;
	
						// set appropriate gain slot to use
						if(_targetElevatorPosition > _actualPositionNU) {
							_elevatorMasterMotor.selectProfileSlot(MOVING_UP_PID_SLOT_INDEX, 0);
						}
						else {
							_elevatorMasterMotor.selectProfileSlot(MOVING_DOWN_PID_SLOT_INDEX, 0);
						}
						
						_elevatorMasterMotor.set(ControlMode.MotionMagic, _targetElevatorPosition, 0);
						
						_lastScanTimeStamp = new Date().getTime();
						_lastScanActualVelocityNU_100mS = _actualVelocityNU_100mS;
						break;
							
					case JOG_AXIS:
						deltatime = (new Date().getTime()) - _lastScanTimeStamp;
						
						_actualPositionNU = _elevatorMasterMotor.getSelectedSensorPosition(0);
						_actualVelocityNU_100mS  = _elevatorMasterMotor.getSelectedSensorVelocity(0);
						_actualAccelerationNU_100mS_mS = (_actualVelocityNU_100mS - _lastScanActualVelocityNU_100mS) / deltatime;
						
						_elevatorMasterMotor.set(ControlMode.PercentOutput, _targetElevatorVelocity);
						
						_lastScanTimeStamp = new Date().getTime();
						_lastScanActualVelocityNU_100mS = _actualVelocityNU_100mS;
						break;
						
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
		public void onStop(double timestamp) 
		{
			synchronized (Elevator.this) 
			{
				stop();
			}
		}
	};
	
	// return the internal loop reference
	public Loop getLoop() {
		return _loop;
	}
	
	// Support Operators Gamepad Buttons mapped to discrete positions
	public void MoveToPresetPosition(ELEVATOR_PRESET_POSITION presetPosition)
	{
		switch(presetPosition)
		{
			case HOME:
				_targetElevatorPosition = HOME_POSITION;
				_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_AND_HOLD_TARGET_POSTION:HOME]");
				break;
				
			case CUBE_ON_FLOOR:
				_targetElevatorPosition = CUBE_ON_FLOOR_POSITION;
				_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_AND_HOLD_TARGET_POSTION:CUBE_ON_FLOOR]");
				break;
				
			case CUBE_ON_PYRAMID_LEVEL_1:
				_targetElevatorPosition = CUBE_ON_PYRAMID_LEVEL_1_POSITION;
				_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_AND_HOLD_TARGET_POSTION:PYR1]");
				break;
				
			case SWITCH_HEIGHT:
				_targetElevatorPosition = SWITCH_HEIGHT_POSITION;
				_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_AND_HOLD_TARGET_POSTION:SWITCH]");
				break;
				
			case SCALE_HEIGHT:
				_targetElevatorPosition = SCALE_HEIGHT_POSITION;
				_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
				ReportStateChg("ElevatorAxis (State) [" + _elevatorState.toString() + "] ==> [GOTO_AND_HOLD_TARGET_POSTION:SCALE]");
				break;
				
			case OTHER:
				DriverStation.reportWarning("ElevatorAxis (State) ILLEGAL value for presetPosition", false);
				break;
		}
	}
	
	// support joystick like jogging but at a fixed velocity
	public void JogAxis(double speedCmd)
	{
		if(_elevatorState == ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION 
				|| _elevatorState == ELEVATOR_STATE.AT_HOME
				|| _elevatorState == ELEVATOR_STATE.JOG_AXIS)
		{
			if(_elevatorState != ELEVATOR_STATE.JOG_AXIS)
			{
				// change the state
				_elevatorState = ELEVATOR_STATE.JOG_AXIS;
				ReportStateChg("ElevatorAxis (State) ==> [JOG_AXIS]");	
			}

			if(speedCmd > 0)
			{
				// jog at a fixed rate in a + direction
				_targetElevatorVelocity = JOG_UP_VELOCITY;
			}
			else if(speedCmd < 0)
			{
				// jog at a fixed rate in a - direction
				_targetElevatorVelocity = JOG_DOWN_VELOCITY;
			}
		}
		else
		{
			DriverStation.reportWarning("Wait until elevator is homed to jog", false);
		}
	}
	
	// implemented as active hold in place for now (vs just turning motors off)
	@Override
	public void stop() 
	{		
		if(_elevatorState != ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION
				&& _elevatorState != ELEVATOR_STATE.NEED_TO_HOME
				&& _elevatorState != ELEVATOR_STATE.MOVING_TO_HOME
				&& _elevatorState != ELEVATOR_STATE.TIMEOUT)
		{
			// set target to current location
			_targetElevatorPosition = _elevatorMasterMotor.getSelectedSensorPosition(0);
			
			// flip back to hold position mode using the current position
			_elevatorState = ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION;
			ReportStateChg("ElevatorAxis (State) stop ==> [GOTO_AND_HOLD_TARGET_POSTION]");
		}
	}

	// reset (rzero) all sensors
	@Override
	public void zeroSensors() 
	{
		_elevatorMasterMotor.setSelectedSensorPosition(0, 0, 0);
	}
	
	// output data to the dashboard on the drivers station
	@Override
	public void outputToSmartDashboard() 
	{
		double actualPosition = 0;
		double actualVelocity = 0;
		double actualAcceleration = 0;
		
		boolean isDisplayNativeUnits = true;
		if(!isDisplayNativeUnits)
		{
			actualPosition =  _actualPositionNU / NU_PER_INCH;
			actualVelocity = 10 * _actualVelocityNU_100mS / NU_PER_INCH;
			actualAcceleration = 1000 * 10 * (_actualAccelerationNU_100mS_mS / NU_PER_INCH);
		}
		else
		{
			actualPosition = _actualPositionNU;
			actualVelocity = _actualVelocityNU_100mS;
			actualAcceleration = _actualAccelerationNU_100mS_mS;			
		}
			
		SmartDashboard.putNumber("Elevator:Position", actualPosition);
		SmartDashboard.putNumber("Elevator:Velocity", GeneralUtilities.RoundDouble(actualVelocity, 2));
		SmartDashboard.putNumber("Elevator:Acceleration", GeneralUtilities.RoundDouble(actualAcceleration, 2));
		SmartDashboard.putBoolean("Elevator:InPosition", IsAtTargetPosition());
		SmartDashboard.putString("Elevator:State", _elevatorState.toString());
	}

	// this property indicates if the elevator is w/i the position deadband of the target position
	public boolean IsAtTargetPosition() 
	{
        if (_elevatorState == ELEVATOR_STATE.GOTO_AND_HOLD_TARGET_POSTION)
        {
        	int currentError = Math.abs(_elevatorMasterMotor.getSelectedSensorPosition(0) - _targetElevatorPosition);
            if ( currentError < ELEVATOR_POS_ALLOWABLE_ERROR_IN_NU) 
            {
            	return true;
            } 
            else
            {
            	return false;
            }
        } 
        else if (_elevatorState == ELEVATOR_STATE.JOG_AXIS)
        {
        	return false;
        } 
        else
        {
        	return false;
        }
    }

	// add data elements to be logged  to the input param (which is passed by ref)
	@Override
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Elevator:PostionNu", String.valueOf(_actualPositionNU));	
		logData.AddData("Elevator:VelocityNu", String.valueOf(_actualVelocityNU_100mS));	
		logData.AddData("Elevator:AccelNu", String.valueOf(_actualAccelerationNU_100mS_mS));	
		logData.AddData("Elevator:State", _elevatorState.toString());	
	} 
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message)
	{
		if(IS_VERBOSE_LOGGING_ENABLED)
		{
			System.out.println(message);
		}
	}
}
