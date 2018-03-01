package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.Kinematics;
import org.usfirst.frc.team4028.robot.sensors.RobotState;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathFollower;
import org.usfirst.frc.team4028.util.loops.Loop;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Twist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team4028.util.GeneralUtilities.setPIDFGains;
import static org.usfirst.frc.team4028.util.GeneralUtilities.setMotionMagicConstants;

public class Chassis implements Subsystem {
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() {
		return _instance;
	}
	
	private TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;
	private DoubleSolenoid _shifter;
	
	private NavXGyro _navX = NavXGyro.getInstance();
	
	private PathFollower _pathFollower;
	private Path _currentPath = null;
	
	private ChassisState _chassisState;
	
	private double _targetAngle = 0;
	private double _angleError = 180;
	private boolean _isTurnRight;
	private double _leftTargetPos, _rightTargetPos;
	private double _leftTargetVelocity, _rightTargetVelocity;

	private static final double CODES_PER_REV = 30725.425;
	private static final double ENCODER_ROTATIONS_PER_DEGREE = 46.15/3600;
	
	private static final double[] MOTION_MAGIC_TURN_PIDF_GAINS = {0.3, 0.0, 40.0, 0.095};
	private static final double[] MOTION_MAGIC_STRAIGHT_PIDF_GAINS = {0.15, 0.0, 20.0, 0.095};
	private static final double[] LOW_GEAR_VELOCITY_PIDF_GAINS = {0.15, 0.0, 1.5, 0.085}; 
	private static final double[] HIGH_GEAR_VELOCITY_PIDF_GAINS = {0.065, 0.0, 1.0, 0.044}; 
    
    private static final int[] MOTION_MAGIC_TURN_VEL_ACC = {80 * 150, 150 * 150};
    private static final int[] MOTION_MAGIC_STRAIGHT_VEL_ACC = {80 * 150, 140 * 150};
	
	private enum ChassisState {
		PERCENT_VBUS, 
		AUTO_TURN, 
		FOLLOW_PATH,
		DRIVE_SET_DISTANCE
	}
	
	private Chassis() {
		_leftMaster = new TalonSRX(Constants.LEFT_DRIVE_MASTER_CAN_ADDR);
		_leftSlave = new TalonSRX(Constants.LEFT_DRIVE_SLAVE_CAN_ADDR);
		_rightMaster = new TalonSRX(Constants.RIGHT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave = new TalonSRX(Constants.RIGHT_DRIVE_SLAVE_CAN_ADDR);
		
		_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		
		_leftSlave.set(ControlMode.Follower, Constants.LEFT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave.set(ControlMode.Follower, Constants.RIGHT_DRIVE_MASTER_CAN_ADDR);

		_leftMaster.setInverted(true);
		_leftSlave.setInverted(true);
		_rightMaster.setInverted(false);
		_rightSlave.setInverted(false);
		
		_leftMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_leftSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_leftMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_leftSlave.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightSlave.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
	
        _leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        _leftMaster.configVelocityMeasurementWindow(32, 0);
        _rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        _rightMaster.configVelocityMeasurementWindow(32, 0);
        
        _leftMaster.enableCurrentLimit(false);
        _leftSlave.enableCurrentLimit(false);
        _rightMaster.enableCurrentLimit(false);
        _rightSlave.enableCurrentLimit(false);
        
        _leftMaster.configOpenloopRamp(0.5, 10);
        _rightMaster.configOpenloopRamp(0.5, 10);
        
        _leftMaster.configClosedloopRamp(0.0, 0);
        _rightMaster.configClosedloopRamp(0.0, 0);
        
        _leftMaster.configPeakOutputForward(1.0, 10);
        _leftMaster.configPeakOutputReverse(-1.0, 10);
        _leftSlave.configPeakOutputForward(1.0, 10);
        _leftSlave.configPeakOutputReverse(-1.0, 10);
        _rightMaster.configPeakOutputForward(1.0, 10);
        _rightMaster.configPeakOutputReverse(-1.0, 10);
        _rightSlave.configPeakOutputForward(1.0, 10);
        _rightSlave.configPeakOutputReverse(-1.0, 10);
        _leftMaster.configNominalOutputForward(0, 10);
        _leftSlave.configNominalOutputForward(0, 10);
        _rightMaster.configNominalOutputForward(0, 10);
        _rightSlave.configNominalOutputForward(0, 10);
        _leftMaster.configNominalOutputReverse(0, 10);
        _leftSlave.configNominalOutputReverse(0, 10);
        _rightMaster.configNominalOutputReverse(0, 10);
        _leftMaster.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
        _leftSlave.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
        _rightMaster.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
        _rightSlave.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);

		_shifter = new DoubleSolenoid(Constants.PCM_CAN_ADDR, Constants.SHIFTER_EXTEND_PCM_PORT, Constants.SHIFTER_RETRACT_PCM_PORT);
	}
	
	private final Loop _loop = new Loop() {
		@Override
		public void onStart(double timestamp) {
			synchronized (Chassis.this) {
				_chassisState = ChassisState.PERCENT_VBUS;
			}
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized (Chassis.this) {
				switch(_chassisState) {
					case PERCENT_VBUS:
						return;
						
					case AUTO_TURN:
						moveToTargetAngle();
						return;
						
					case DRIVE_SET_DISTANCE:
						moveToTargetPos();
						return;
						
					case FOLLOW_PATH:
						if (isHighGear()) {
							setPIDFGains(_leftMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
							setPIDFGains(_rightMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
						} else {
							setPIDFGains(_leftMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
							setPIDFGains(_rightMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
						}
						
						if (_pathFollower != null) 
							updatePathFollower(timestamp);
						return;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			synchronized (Chassis.this) {
				stop();
			}
		}
	};
	
	public Loop getLoop() {
		return _loop;
	}
	
	/* ===== Chassis State: PERCENT VBUS ===== */
	/** Arcade drive with throttle and turn inputs. Includes anti-tipping. */
	public synchronized void arcadeDrive(double throttle, double turn) {
		_chassisState = ChassisState.PERCENT_VBUS;
		
		if(_navX.isPitchPastThreshhold()) {
			_leftMaster.set(ControlMode.PercentOutput, 0.0);
			_rightMaster.set(ControlMode.PercentOutput, 0.0);
			DriverStation.reportError("Tipping Threshold", false);
		} else {
			_leftMaster.set(ControlMode.PercentOutput, -throttle + 0.5 * turn);
			_rightMaster.set(ControlMode.PercentOutput, -throttle - 0.5 * turn);
		} 
	}
	
	/* ===== Chassis State: AUTO TURN ===== */
	/** Set the target gyro angle for the robot to turn to. */
	public synchronized void setTargetAngleAndTurnDirection(double targetAngle, boolean isTurnRight) {
		_targetAngle = targetAngle;
		_isTurnRight = isTurnRight;
		setHighGear(false);
		setMotionMagicTurnGains();
		_chassisState = ChassisState.AUTO_TURN;
	} 
	
	/** Updates target position every cycle while using MotionMagic to turn to heading goal */
	private synchronized void moveToTargetAngle() {
		if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() > _targetAngle) ||
			(_navX.getYaw() >= 0 && _targetAngle < 0 && _isTurnRight) ||
			(_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
			_angleError = 360 - _navX.getYaw() + _targetAngle;
		}
		else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() < _targetAngle)||
				(_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() > _targetAngle)||
				(_navX.getYaw() >= 0 && _targetAngle < 0 && !_isTurnRight) ||
				(_navX.getYaw() < 0 && _targetAngle >= 0 && _isTurnRight) ||
				(_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle)) ||
				(_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
			_angleError = _targetAngle - _navX.getYaw();
		}		
		else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() < _targetAngle)||
				(_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle))||
				(_navX.getYaw() < 0 && _targetAngle >= 0 && !_isTurnRight)) {
			_angleError = _targetAngle - _navX.getYaw() - 360;
		}			
		
		double encoderError = ENCODER_ROTATIONS_PER_DEGREE * _angleError;		
		double leftDriveTargetPosition = (getLeftPosInRot() + encoderError) * CODES_PER_REV;
		double rightDriveTargetPosition = (getRightPosInRot() - encoderError) * CODES_PER_REV;
		
		_leftMaster.set(ControlMode.MotionMagic, leftDriveTargetPosition);
		_rightMaster.set(ControlMode.MotionMagic, rightDriveTargetPosition);
	}
	
	/* ===== Chassis State: DRIVE SET DISTANCE ===== */
	public synchronized void setTargetPos(double targetPos) {
		_leftTargetPos = inchesToNativeUnits(getLeftPosInches() + targetPos);
		_rightTargetPos = inchesToNativeUnits(getRightPosInches() + targetPos);
		setHighGear(false);
		setMotionMagicStraightGains();
		_chassisState = ChassisState.DRIVE_SET_DISTANCE;
	}
	
	private synchronized void moveToTargetPos() {
		_leftMaster.set(ControlMode.MotionMagic, _leftTargetPos);
		_rightMaster.set(ControlMode.MotionMagic, _rightTargetPos);
	}
	
	public synchronized boolean atTargetPos() {
		return (Math.abs(_leftTargetPos - inchesToNativeUnits(getLeftPosInches())) < 2500) && (Math.abs(_rightTargetPos - inchesToNativeUnits(getRightPosInches())) < 2500);
	} 

	/* ===== Chassis State: FOLLOW PATH ===== */
	/** Set path for PathFollower controller to follow */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (_currentPath != path || _chassisState != ChassisState.FOLLOW_PATH) {
            RobotState.getInstance().resetDistanceDriven();
            _pathFollower = new PathFollower(path, reversed, path.maxAccel, path.maxDecel, path.inertiaSteeringGain);
            _chassisState = ChassisState.FOLLOW_PATH;
            _currentPath = path;
        } else {
        	_leftMaster.set(ControlMode.Velocity, 0.0);
        	_rightMaster.set(ControlMode.Velocity, 0.0);
        }
    }
    
    /** Update PathFollower with latest pose estimate to get new target velocity */
    private void updatePathFollower(double timestamp) {
		RigidTransform _robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		Twist command = _pathFollower.update(timestamp, _robotPose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!_pathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			final double max_desired = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
            final double scale = max_desired > Constants.DRIVE_VELOCITY_MAX_SETPOINT ? Constants.DRIVE_VELOCITY_MAX_SETPOINT / max_desired : 1.0;
            _leftMaster.set(ControlMode.Velocity, inchesPerSecondToNativeUnits(setpoint.left * scale));
            _rightMaster.set(ControlMode.Velocity, inchesPerSecondToNativeUnits(setpoint.right * scale));
			_leftTargetVelocity = setpoint.left;
			_rightTargetVelocity = setpoint.right;
		} else {
			_leftMaster.set(ControlMode.Velocity, 0.0);
			_rightMaster.set(ControlMode.Velocity, 0.0);
		}
	}

    /** Returns whether Chassis has completed the set path */
    public synchronized boolean isDoneWithPath() {
        if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
            return _pathFollower.isFinished();
        else
            System.out.println("Robot is not in path following mode");
            return true;
    }

    /** Path following e-stop */
    public synchronized void forceDoneWithPath() {
        if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
            _pathFollower.forceFinish();
        else
            System.out.println("Robot is not in path following mode");
    }
    
    /* SHIFTER */
    public synchronized void toggleShifter() {
    	setHighGear(!isHighGear());	// Inverse of current solenoid state
    }
	
	public synchronized void setHighGear(boolean isHighGear) {
		if (isHighGear) 
			_shifter.set(Constants.SHIFTER_HIGH_GEAR_POS);
		else 
			_shifter.set(Constants.SHIFTER_LOW_GEAR_POS);
	}
	
	private synchronized boolean isHighGear() {
		return _shifter.get() == Constants.SHIFTER_HIGH_GEAR_POS;
	}
	
	/* SENSORS */
	public void zeroEncoders() {
		_leftMaster.getSensorCollection().setQuadraturePosition(0, 10);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, 10);
	}
	
	public void zeroGyro() {
		_navX.zeroYaw();
	}
	
	public double getHeading() {
		return _navX.getYaw();
	}
	
	public double getLeftPosInRot() {
		return _leftMaster.getSelectedSensorPosition(0) / CODES_PER_REV;
	}
	
	public double getRightPosInRot() {
		return _rightMaster.getSelectedSensorPosition(0) / CODES_PER_REV;
	}
	
	public double getLeftSpeed() {
		return _leftMaster.getSelectedSensorVelocity(0) * (600 / CODES_PER_REV);
	}
	
	public double getRightSpeed() {
		return -_rightMaster.getSelectedSensorVelocity(0) * (600 / CODES_PER_REV);
	}
	
	public double getLeftPosInches() {
        return rotationsToInches(getLeftPosInRot());
    }

    public double getRightPosInches() {
        return rotationsToInches(getRightPosInRot());
    }
    
    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getLeftSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getRightSpeed());
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI);
    } 
    
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
    
    private static double inchesToNativeUnits(double inches) {
    	return inches * 1540.95;
    }
	
	private static double inchesPerSecondToNativeUnits(double inches_per_second) {
        return inches_per_second * 148.2;
    }
	
	public synchronized void setBrakeMode(boolean isBrakeMode) {
		NeutralMode mode = (isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		
		_leftMaster.setNeutralMode(mode);
		_leftSlave.setNeutralMode(mode);
		_rightMaster.setNeutralMode(mode);
		_rightSlave.setNeutralMode(mode);
	}
	
	public synchronized void setMotionMagicTurnGains() {
		setPIDFGains(_leftMaster, MOTION_MAGIC_TURN_PIDF_GAINS);
		setPIDFGains(_rightMaster, MOTION_MAGIC_TURN_PIDF_GAINS);
        
		setMotionMagicConstants(_leftMaster, MOTION_MAGIC_TURN_VEL_ACC);
		setMotionMagicConstants(_rightMaster, MOTION_MAGIC_TURN_VEL_ACC);
	}
	
	public synchronized void setMotionMagicStraightGains() {
		setPIDFGains(_leftMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
		setPIDFGains(_rightMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
        
		setMotionMagicConstants(_leftMaster, MOTION_MAGIC_STRAIGHT_VEL_ACC);
		setMotionMagicConstants(_rightMaster, MOTION_MAGIC_STRAIGHT_VEL_ACC);
	}

	@Override
	public synchronized void stop() {
		arcadeDrive(0.0, 0.0);
	}

	@Override
	public synchronized void zeroSensors() {
		zeroEncoders();
		zeroGyro();
	}
	
	@Override
	public void outputToShuffleboard() {
		SmartDashboard.putNumber("Chassis: Left Velocity", GeneralUtilities.RoundDouble(getLeftVelocityInchesPerSec(), 2));
		SmartDashboard.putNumber("Chassis: Right Velocity", GeneralUtilities.RoundDouble(getLeftVelocityInchesPerSec(), 2));
		
		SmartDashboard.putNumber("Chassis: Left Wheel Target Velocity", GeneralUtilities.RoundDouble(_leftTargetVelocity, 2));
		SmartDashboard.putNumber("Chasiss: Right Wheel Target Velocity", GeneralUtilities.RoundDouble(_leftTargetVelocity, 2));
		
		SmartDashboard.putNumber("Chassis: Angle", GeneralUtilities.RoundDouble(getHeading(), 2));
		SmartDashboard.putString("Chassis: Robot Pose", RobotState.getInstance().getLatestFieldToVehicle().getValue().toString());
	}
	
	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Left Actual Velocity [in/s]", String.valueOf(GeneralUtilities.RoundDouble(getLeftVelocityInchesPerSec(), 2)));
		logData.AddData("Left Target Velocity [in/s]", String.valueOf(GeneralUtilities.RoundDouble(_leftTargetVelocity, 2)));
		
		logData.AddData("Right Actual Velocity [in/s]", String.valueOf(GeneralUtilities.RoundDouble(-getRightVelocityInchesPerSec(), 2)));
		logData.AddData("Right Target Velocity [in/s]", String.valueOf(GeneralUtilities.RoundDouble(_rightTargetVelocity, 2)));
	}
}