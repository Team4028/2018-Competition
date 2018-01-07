package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.Kinematics;
import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.util.DriveCommand;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathFollower;
import org.usfirst.frc.team4028.util.loops.Loop;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Twist;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Chassis extends Subsystem{
	private static Chassis _instance = new Chassis();
	public static Chassis getInstance() {
		return _instance;
	}
	
	private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
	
	// define class level variables for Robot objects
	private CANTalon _leftMaster, _leftSlave, _rightMaster, _rightSlave;
	private DoubleSolenoid _shifterSolenoid;
	private NavXGyro _navX = NavXGyro.getInstance();
	
	// Controllers
	private RobotState _robotState = RobotState.getInstance();
	private PathFollower _pathFollower;
	
	private Path _currentPath = null;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	
	private ChassisState _chassisState;
	
	private double _targetAngle;
	
	// acc/dec variables
	private boolean _isAccelDecelEnabled = true;
	private double _currentThrottleCmdScaled, _previousThrottleCmdScaled;
	private double _currentThrottleCmdAccDec;
	
	private double _arcadeDriveThrottleCmdAdj;
	private double _arcadeDriveTurnCmdAdj;
	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOTAL_TIME_SECS = 0.8;
	
	private static final double _turnSpeedScalingFactor = 0.7;
	
	private static final double SHIFT_UP_VEL = 75;
	private static final double SHIFT_DOWN_VEL= 70;
	
	// shifter positions
	public enum GearShiftPosition {
		UNKNOWN,
		HIGH_GEAR,
		LOW_GEAR
	}
	
	// Chassis various states
	public enum ChassisState {
		PERVENT_VBUS, 
		AUTO_TURN, 
		FOLLOW_PATH,
		VELOCITY_SETPOINT
	}
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Chassis.this) {
				_chassisState = ChassisState.PERVENT_VBUS;
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Chassis.this) {
				switch(_chassisState) {
					case AUTO_TURN:
						ShiftGear(GearShiftPosition.LOW_GEAR);
						_leftMaster.setProfile(kLowGearPositionControlSlot);
						_rightMaster.setProfile(kLowGearPositionControlSlot);
						moveToTarget();
						return;
						
					case FOLLOW_PATH:
						ShiftGear(GearShiftPosition.HIGH_GEAR);
						_leftMaster.setProfile(kHighGearVelocityControlSlot);
						_rightMaster.setProfile(kHighGearVelocityControlSlot);
						if (_pathFollower != null) 
							updatePathFollower(timestamp);
						return;
						
					case PERVENT_VBUS:
						return;
						
					case VELOCITY_SETPOINT:
						return;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Chassis.this) {
				ShiftGear(GearShiftPosition.HIGH_GEAR);
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	private Chassis() {
		/* Drive Motors */
		_leftMaster = new CANTalon(Constants.LEFT_DRIVE_MASTER_CAN_BUS_ADDR);
		_leftSlave = new CANTalon(Constants.LEFT_DRIVE_SLAVE_CAN_BUS_ADDR);
		_rightMaster = new CANTalon(Constants.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR);
		_rightSlave = new CANTalon(Constants.RIGHT_DRIVE_SLAVE_CAN_BUS_ADDR);
		
		_leftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		_leftMaster.configEncoderCodesPerRev(1097);
		_leftMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
		_rightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		_rightMaster.configEncoderCodesPerRev(1097);
		_rightMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
		
		_leftMaster.changeControlMode(TalonControlMode.PercentVbus);
		_leftSlave.changeControlMode(TalonControlMode.Follower);
		_leftSlave.set(Constants.LEFT_DRIVE_MASTER_CAN_BUS_ADDR);
		_rightMaster.changeControlMode(TalonControlMode.PercentVbus);
		_rightSlave.changeControlMode(TalonControlMode.Follower);
		_rightSlave.set(Constants.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR);
		
		_leftMaster.reverseSensor(false);
		_leftMaster.reverseOutput(false);
		_rightMaster.reverseSensor(true);	// reverse these to ensure encoder counts and closed loop output are in same direction
		_rightMaster.reverseOutput(true);
		
		_leftMaster.enableLimitSwitch(false,  false);
		_leftSlave.enableLimitSwitch(false, false);
		_rightMaster.enableLimitSwitch(false, false);
		_rightSlave.enableLimitSwitch(false, false);
		
		_leftMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        _leftMaster.SetVelocityMeasurementWindow(32);
        _rightMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        _rightMaster.SetVelocityMeasurementWindow(32);
		
		// Motion Magic Constants
		_leftMaster.setMotionMagicAcceleration(Constants.DRIVE_TURN_MAX_ACC);
		_rightMaster.setMotionMagicAcceleration(Constants.DRIVE_TURN_MAX_ACC);
		_leftMaster.setMotionMagicCruiseVelocity(Constants.DRIVE_TURN_MAX_VEL);
		_rightMaster.setMotionMagicCruiseVelocity(Constants.DRIVE_TURN_MAX_VEL);
		
		reloadGains();
		
		/* Shifter */
		_shifterSolenoid = new DoubleSolenoid(Constants.PCM_CAN_BUS_ADDR, Constants.SHIFTER_SOLENOID_EXTEND_PCM_PORT, 
												Constants.SHIFTER_SOLENOID_RETRACT_PCM_PORT);
		
		enableBrakeMode(false);
		
		_driveSpeedScalingFactorClamped = 1.0;
	}
	
	/**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(ChassisState state) {
        if (state == ChassisState.VELOCITY_SETPOINT || state == ChassisState.FOLLOW_PATH) {
            return true;
        }
        return false;
    }
	
	public synchronized void arcadeDrive(double throttle, double turn) {
		_chassisState = ChassisState.PERVENT_VBUS;
		
		enablePercentVBusMode();
		// calc scaled throttle cmds
		double newThrottleCmdScaled = throttle * _driveSpeedScalingFactorClamped;
		double newTurnCmdScaled = turn * _turnSpeedScalingFactor;
		
		// if the cmd just chg'd reset 
		if(newThrottleCmdScaled != _previousThrottleCmdScaled) {
			_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			_lastCmdChgTimeStamp = System.currentTimeMillis();
		}
			
		// if acc/dec mode is enabled
		if(_isAccelDecelEnabled) {
			//implement speed scaling
			_arcadeDriveThrottleCmdAdj = calcAccelDecelThrottleCmd(_currentThrottleCmdScaled, _previousThrottleCmdScaled, _lastCmdChgTimeStamp);
			
			_currentThrottleCmdAccDec = _arcadeDriveThrottleCmdAdj;
			
			if(Math.abs(_arcadeDriveThrottleCmdAdj - _currentThrottleCmdScaled) < 0.1)
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
		} else {
			_arcadeDriveThrottleCmdAdj = newThrottleCmdScaled;
		}
		
		_arcadeDriveTurnCmdAdj = newTurnCmdScaled;
		
		// send cmd to mtr controllers
		_leftMaster.set(_arcadeDriveThrottleCmdAdj - 0.7 * _arcadeDriveTurnCmdAdj);
		_rightMaster.set(-_arcadeDriveThrottleCmdAdj - 0.7 * _arcadeDriveTurnCmdAdj);
	}
	
	public synchronized void tankDrive(DriveCommand command) {
		enablePercentVBusMode();
		_leftMaster.set(command.leftCmd);
		_rightMaster.set(command.rightCmd);
	}
	
	private void setMotionMagicTargetPosition(double leftPosition, double rightPosition) {
		enableMotionMagicMode();
		_leftMaster.set(leftPosition);
		_rightMaster.set(rightPosition);
	}
	
	public synchronized void enablePercentVBusMode() {
		if (_leftMaster.getControlMode() != TalonControlMode.PercentVbus)
			_leftMaster.changeControlMode(TalonControlMode.PercentVbus);
			_rightMaster.changeControlMode(TalonControlMode.PercentVbus);
	}
	
	public synchronized void enableMotionMagicMode() {
		if (_leftMaster.getControlMode() != TalonControlMode.MotionMagic)
			_leftMaster.changeControlMode(TalonControlMode.MotionMagic);
			_rightMaster.changeControlMode(TalonControlMode.MotionMagic);
	}
	
	/**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        _chassisState = ChassisState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(_chassisState)) {
            // We entered a velocity control state.
            _leftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            _leftMaster.setNominalClosedLoopVoltage(12.0);
            _leftMaster.setProfile(kHighGearVelocityControlSlot);
            _leftMaster.configNominalOutputVoltage(Constants.DriveHighGearNominalOutput,
                    -Constants.DriveHighGearNominalOutput);
            _rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            _rightMaster.setNominalClosedLoopVoltage(12.0);
            _rightMaster.setProfile(kHighGearVelocityControlSlot);
            _rightMaster.configNominalOutputVoltage(Constants.DriveHighGearNominalOutput,
                    -Constants.DriveHighGearNominalOutput);
            enableBrakeMode(true);
        }
    }
    
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(_chassisState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.DriveHighGearMaxSetpoint
                    ? Constants.DriveHighGearMaxSetpoint / max_desired : 1.0;
            _leftMaster.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            _rightMaster.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            _leftMaster.set(0);
            _rightMaster.set(0);
        }
    }
	
	public synchronized void enableBrakeMode(boolean isEnabled) {
		_leftMaster.enableBrakeMode(isEnabled);
		_leftSlave.enableBrakeMode(isEnabled);
		_rightMaster.enableBrakeMode(isEnabled);
		_rightSlave.enableBrakeMode(isEnabled);
	}
	
	public synchronized void setTargetAngle(double target) {
		setAutoAimTarget(target);
	}
	
	private synchronized void setAutoAimTarget(double target) {
		_targetAngle = target;
		_chassisState = ChassisState.AUTO_TURN;
	}
	
	private synchronized void moveToTarget() {
		enableMotionMagicMode();
		
		double angleError;
		
		angleError = _targetAngle - _navX.getYaw();
		
		double encoderError = GeneralUtilities.degreesToEncoderRotations(angleError);
		
		double leftDriveTargetPosition = getLeftEncPosition() - encoderError;
		double rightDriveTargetPosition = getRightEncPosition() + encoderError;
		
		setMotionMagicTargetPosition(leftDriveTargetPosition, rightDriveTargetPosition);
	}
	
	public synchronized double autoAimError() {
		return _targetAngle - _navX.getYaw();
	}
	
	public synchronized double errorToTarget() {
		return _targetAngle - _navX.getYaw();
	}
	
	private void updatePathFollower(double timestamp) {
		RigidTransform _robotPose = _robotState.getLatestFieldToVehicle().getValue();
		Twist command = _pathFollower.update(timestamp, _robotPose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!_pathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updateVelocitySetpoint(setpoint.left, setpoint.right);
		} else {
			updateVelocitySetpoint(0.0, 0.0);
		}
	}
	
	/**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (_currentPath != path || _chassisState != ChassisState.FOLLOW_PATH) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            _pathFollower = new PathFollower(path, reversed);
            _chassisState = ChassisState.FOLLOW_PATH;
            _currentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
            return _pathFollower.isFinished();
        else
            System.out.println("Robot is not in path following mode");
            return true;
    }

    public synchronized void forceDoneWithPath() {
        if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
            _pathFollower.forceFinish();
        else
            System.out.println("Robot is not in path following mode");
    }
	
	// shifts between high & low gear
	public synchronized void ShiftGear(GearShiftPosition gear) {
		// send cmd to to solenoids
		switch(gear) {
			case HIGH_GEAR:
				_shifterSolenoid.set(Constants.SHIFTER_SOLENOID_HIGH_GEAR_POSITION);
				_shifterSolenoidPosition = Constants.SHIFTER_SOLENOID_HIGH_GEAR_POSITION;
				break;
			
			case LOW_GEAR:
				_shifterSolenoid.set(Constants.SHIFTER_SOLENOID_LOW_GEAR_POSITION);
				_shifterSolenoidPosition = Constants.SHIFTER_SOLENOID_LOW_GEAR_POSITION;
				break;
		}
	}
	
	private void autoShift() {
		if (Math.max(getLeftVelocityInchesPerSec(), getRightVelocityInchesPerSec()) > SHIFT_UP_VEL){
			ShiftGear(GearShiftPosition.HIGH_GEAR);
		} else if (Math.max(getLeftVelocityInchesPerSec(), getRightVelocityInchesPerSec()) < SHIFT_DOWN_VEL) {
			ShiftGear(GearShiftPosition.LOW_GEAR);
		} else {
		}
	}

	public synchronized void ToggleShiftGear() {
		if (_shifterSolenoidPosition == Constants.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) 
			ShiftGear(GearShiftPosition.LOW_GEAR);
		else 
			ShiftGear(GearShiftPosition.HIGH_GEAR);
	}
	
	public void zeroEncoders() {
		_leftMaster.setEncPosition(0);
		_rightMaster.setEncPosition(0);
		
		_leftMaster.setPosition(0);
		_rightMaster.setPosition(0);
	}
	
	public double getLeftEncPosition() {
		return _leftMaster.getPosition();
	}
	
	public double getRightEncPosition() {
		return _rightMaster.getPosition();
	}

	public void zeroGyro() {
		_navX.zeroYaw();
	}
	
	public double getHeading() {
		return _navX.getYaw();
	}
	
	public void setGyroAngle(double yaw) {
		_navX.zeroYaw();
		_navX.setAngleAdjustment(yaw);
	}
	
	public double getLeftDistanceInches() {
        return rotationsToInches(_leftMaster.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(_rightMaster.getPosition());
    }
    
    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(_leftMaster.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(_rightMaster.getSpeed());
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.DriveWheelDiameterInches * Math.PI);
    }
    
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
	
	private static double inchesToRotations(double inches) {
        return inches / (Constants.DriveWheelDiameterInches * Math.PI);
    }
	
	private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
	
	public synchronized void reloadGains() {
		_leftMaster.setPID(Constants.DRIVE_LOW_GEAR_POSITION_P, Constants.DRIVE_LOW_GEAR_POSITION_I,
				Constants.DRIVE_LOW_GEAR_POSITION_D, Constants.DRIVE_LOW_GEAR_POSITION_F,
				Constants.DRIVE_LOW_GEAR_POSITION_I_ZONE, Constants.DRIVE_LOW_GEAR_POSITION_RAMP_RATE, 
				kLowGearPositionControlSlot);
		_rightMaster.setPID(Constants.DRIVE_LOW_GEAR_POSITION_P, Constants.DRIVE_LOW_GEAR_POSITION_I,
				Constants.DRIVE_LOW_GEAR_POSITION_D, Constants.DRIVE_LOW_GEAR_POSITION_F,
				Constants.DRIVE_LOW_GEAR_POSITION_I_ZONE, Constants.DRIVE_LOW_GEAR_POSITION_RAMP_RATE,
				kLowGearPositionControlSlot);
        _leftMaster.setPID(Constants.DriveHighGearVelocityKp, Constants.DriveHighGearVelocityKi,
                Constants.DriveHighGearVelocityKd, Constants.DriveHighGearVelocityKf,
                Constants.DriveHighGearVelocityIZone, Constants.DriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        _rightMaster.setPID(Constants.DriveHighGearVelocityKp, Constants.DriveHighGearVelocityKi,
                Constants.DriveHighGearVelocityKd, Constants.DriveHighGearVelocityKf,
                Constants.DriveHighGearVelocityIZone, Constants.DriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        _leftMaster.setVoltageCompensationRampRate(Constants.DriveVoltageCompensationRampRate);
        _rightMaster.setVoltageCompensationRampRate(Constants.DriveVoltageCompensationRampRate);
	}

	@Override
	public synchronized void stop() {
		enableBrakeMode(true);
		arcadeDrive(0.0, 0.0);
	}

	@Override
	public synchronized void zeroSensors() {
		zeroEncoders();
		zeroGyro();
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Chassis Vel: ", getLeftVelocityInchesPerSec());
	}

	@Override
	public void updateLogData(LogData logData) {}
	
	//============================================================================================
	// Utility Helper Methods
	//============================================================================================
	// implement s-curve accel / decel
	private double calcAccelDecelThrottleCmd(double currentThrottleCmd, double previousThrottleCmd, long lastCmdChgTimeStamp) {
		double accDecMidpointTimeSecs = ACC_DEC_TOTAL_TIME_SECS / 2.0;    // a

        double minusK = -1.0 * ACC_DEC_RATE_FACTOR;
        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - _lastCmdChgTimeStamp) / 1000.0; // x
        double xMinusA = elapsedSecsSinceLastChg - accDecMidpointTimeSecs;

        double scaleFactor = 1.0 / ( 1.0 + Math.exp(minusK * xMinusA) );

        // finally calc the adj cmd
        double accDecCmd = previousThrottleCmd + ((_currentThrottleCmdScaled - previousThrottleCmd) * scaleFactor);
        
        return accDecCmd;
	}	
}