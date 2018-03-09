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
	
	private enum ChassisState {
		PERCENT_VBUS, 
		AUTO_TURN, 
		FOLLOW_PATH,
		DRIVE_SET_DISTANCE
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
	
	private Chassis() {
		_leftMaster = new TalonSRX(Constants.LEFT_DRIVE_MASTER_CAN_ADDR);
		_leftSlave = new TalonSRX(Constants.LEFT_DRIVE_SLAVE_CAN_ADDR);
		_rightMaster = new TalonSRX(Constants.RIGHT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave = new TalonSRX(Constants.RIGHT_DRIVE_SLAVE_CAN_ADDR);
		
		_leftSlave.set(ControlMode.Follower, Constants.LEFT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave.set(ControlMode.Follower, Constants.RIGHT_DRIVE_MASTER_CAN_ADDR);
		
		configMasterMotors(_leftMaster);
		configMasterMotors(_rightMaster);
        
        configDriveMotors(_leftMaster);
        configDriveMotors(_rightMaster);
        configDriveMotors(_leftSlave);
        configDriveMotors(_rightSlave);

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
		_leftTargetPos = inchesToNU(getLeftPosInches() + targetPos);
		_rightTargetPos = inchesToNU(getRightPosInches() + targetPos);
		setHighGear(false);
		setMotionMagicStraightGains();
		_chassisState = ChassisState.DRIVE_SET_DISTANCE;
	}
	
	private synchronized void moveToTargetPos() {
		_leftMaster.set(ControlMode.MotionMagic, _leftTargetPos);
		_rightMaster.set(ControlMode.MotionMagic, _rightTargetPos);
	}
	
	public synchronized boolean atTargetPos() {
		return (Math.abs(_leftTargetPos - inchesToNU(getLeftPosInches())) < 2500) && (Math.abs(_rightTargetPos - inchesToNU(getRightPosInches())) < 2500);
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
            _leftMaster.set(ControlMode.Velocity, inchesPerSecToNU(setpoint.left * scale));
            _rightMaster.set(ControlMode.Velocity, inchesPerSecToNU(setpoint.right * scale));
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
    
    private static double rotationsToInches(double rot) {
        return rot * (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI);
    } 
    
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
    
    private static double inchesToNU(double inches) {
    	return inches * 1540.95;
    }
	
	private static double inchesPerSecToNU(double inches_per_second) {
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
	
	private void configMasterMotors(TalonSRX talon) {
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
	
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        talon.configVelocityMeasurementWindow(32, 0);
        
        talon.configOpenloopRamp(0.5, 10);
        talon.configClosedloopRamp(0.0, 0);
	}
	
	private void configDriveMotors(TalonSRX talon) {
		talon.setInverted(true);
		
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
        
        talon.enableCurrentLimit(false);
        
        talon.configPeakOutputForward(1.0, 10);
        talon.configPeakOutputReverse(-1.0, 10);
        talon.configNominalOutputForward(0, 10);
        talon.configNominalOutputReverse(0, 10);
        talon.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
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
		logData.AddData("Chassis: Angle", String.valueOf(GeneralUtilities.RoundDouble(getHeading(), 2)));
	}
}