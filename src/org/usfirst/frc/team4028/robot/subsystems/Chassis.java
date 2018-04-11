package org.usfirst.frc.team4028.robot.subsystems;

import static org.usfirst.frc.team4028.util.GeneralUtilities.setMotionMagicConstants;
import static org.usfirst.frc.team4028.util.GeneralUtilities.setPIDFGains;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.RobotState;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.Kinematics;
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
	private double _leftTargetVelocity, _rightTargetVelocity, _centerTargetVelocity;

	private static final double CODES_PER_REV = 30725.425;
	private static final double ENCODER_ROTATIONS_PER_DEGREE = 46.15/3600;
	
	private static final double[] MOTION_MAGIC_TURN_PIDF_GAINS = {0.25, 0.0, 30.0, 0.095};
	private static final double[] MOTION_MAGIC_STRAIGHT_PIDF_GAINS = {0.15, 0.0, 20.0, 0.095};
	private static final double[] LOW_GEAR_VELOCITY_PIDF_GAINS = {0.15, 0.0, 1.5, 0.085}; 
	private static final double[] HIGH_GEAR_VELOCITY_PIDF_GAINS = {0.09, 0.0, 1.3, 0.044}; 
    
    private static final int[] MOTION_MAGIC_TURN_VEL_ACC = {80 * 150, 170 * 150};
    private static final int[] MOTION_MAGIC_STRAIGHT_VEL_ACC = {80 * 150, 170 * 150};
	
	private Chassis() {
		_leftMaster = new TalonSRX(Constants.LEFT_DRIVE_MASTER_CAN_ADDR);
		_leftSlave = new TalonSRX(Constants.LEFT_DRIVE_SLAVE_CAN_ADDR);
		_rightMaster = new TalonSRX(Constants.RIGHT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave = new TalonSRX(Constants.RIGHT_DRIVE_SLAVE_CAN_ADDR);
		
		_leftSlave.follow(_leftMaster);
		_rightSlave.follow(_rightMaster);
		
		_leftMaster.setInverted(true);
		_leftSlave.setInverted(true);
		_rightMaster.setInverted(false);
		_rightSlave.setInverted(false);
		
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
			setLeftRightCommand(ControlMode.PercentOutput, 0.0, 0.0);
			DriverStation.reportError("Tipping Threshold", false);
		} else if ((Math.abs(getLeftVelocityInchesPerSec() - getRightVelocityInchesPerSec())) < 5.0) {
			setLeftRightCommand(ControlMode.PercentOutput, -throttle + 0.7 * turn, -throttle - 0.7 * turn);
		} else {
			setLeftRightCommand(ControlMode.PercentOutput, -throttle + 0.5 * turn, -throttle - 0.5 * turn);
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
	private void moveToTargetAngle() {
		// TODO: This code needs to be simplified. Should convert angles to vectors and use dot product to get angle difference.
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
		double leftDriveTargetPos = (getLeftPosInRot() + encoderError) * CODES_PER_REV;
		double rightDriveTargetPos = (getRightPosInRot() - encoderError) * CODES_PER_REV;
		
		setLeftRightCommand(ControlMode.MotionMagic, leftDriveTargetPos, rightDriveTargetPos);
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
		setLeftRightCommand(ControlMode.MotionMagic, _leftTargetPos, _rightTargetPos);
	}
	
	public synchronized boolean atTargetPos() {
		return (Math.abs(_leftTargetPos - inchesToNU(getLeftPosInches())) < inchesToNU(2))
				&& (Math.abs(_rightTargetPos - inchesToNU(getRightPosInches())) < inchesToNU(2));
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
        	setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
        }
    }
    
    /** Update PathFollower with latest pose estimate to get new target velocity */
    private void updatePathFollower(double timestamp) {
		RigidTransform _robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		Twist command = _pathFollower.update(timestamp, _robotPose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!_pathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			final double maxDesired = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
            final double scale = maxDesired > Constants.DRIVE_VELOCITY_MAX_SETPOINT ? Constants.DRIVE_VELOCITY_MAX_SETPOINT / maxDesired : 1.0;
            setLeftRightCommand(ControlMode.Velocity, inchesPerSecToNU(setpoint.left * scale), inchesPerSecToNU(setpoint.right * scale));
            _centerTargetVelocity = command.dx;
			_leftTargetVelocity = setpoint.left;
			_rightTargetVelocity = setpoint.right;
		} else {
			setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
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
	
	
	public synchronized double getRemainingPathDistance() {
		if (_pathFollower != null) {
			return _pathFollower.remainingPathLength();
		} 
		return 0;
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
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
        
        talon.enableCurrentLimit(false);
        
        talon.configPeakOutputForward(1.0, 10);
        talon.configPeakOutputReverse(-1.0, 10);
        talon.configNominalOutputForward(0, 10);
        talon.configNominalOutputReverse(0, 10);
        talon.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
	}
	
	private void setLeftRightCommand(ControlMode mode, double leftCommand, double rightCommand) {
		_leftMaster.set(mode, leftCommand);
		_rightMaster.set(mode, rightCommand);
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
		SmartDashboard.putNumber("Chassis: Left Velocity", GeneralUtilities.roundDouble(getLeftVelocityInchesPerSec(), 2));
		SmartDashboard.putNumber("Chassis: Right Velocity", GeneralUtilities.roundDouble(getLeftVelocityInchesPerSec(), 2));
		
		SmartDashboard.putNumber("Chassis: Left Wheel Target Velocity", GeneralUtilities.roundDouble(_leftTargetVelocity, 2));
		SmartDashboard.putNumber("Chasiss: Right Wheel Target Velocity", GeneralUtilities.roundDouble(_leftTargetVelocity, 2));
		
		SmartDashboard.putNumber("Chassis: Angle", GeneralUtilities.roundDouble(getHeading(), 2));
		SmartDashboard.putString("Chassis: Robot Pose", RobotState.getInstance().getLatestFieldToVehicle().getValue().toString());
	}
	
	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Left Actual Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(getLeftVelocityInchesPerSec(), 2)));
		logData.AddData("Left Target Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(_leftTargetVelocity, 2)));
		logData.AddData("Left Output Current", String.valueOf(GeneralUtilities.roundDouble(_leftMaster.getOutputCurrent(), 2)));
		
		logData.AddData("Right Actual Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(-getRightVelocityInchesPerSec(), 2)));
		logData.AddData("Right Target Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(_rightTargetVelocity, 2)));
		logData.AddData("Right Output Current", String.valueOf(GeneralUtilities.roundDouble(_rightMaster.getOutputCurrent(), 2)));
		
		logData.AddData("Pose X", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x()));
		logData.AddData("Pose Y", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y()));
		logData.AddData("Pose Angle", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees()));
		logData.AddData("Remaining Distance", String.valueOf(getRemainingPathDistance()));
		
		logData.AddData("Center Target Velocity", String.valueOf(GeneralUtilities.roundDouble(_centerTargetVelocity, 2)));
	}
}