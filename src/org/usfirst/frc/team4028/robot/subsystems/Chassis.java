package org.usfirst.frc.team4028.robot.subsystems;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.Kinematics;
import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.util.DriveCommand;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathFollower;
import org.usfirst.frc.team4028.util.loops.ILoop;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Chassis extends Subsystem{
	
	// singleton pattern
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() {
		return _instance;
	}
	
	private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
	
	// define class level variables for Robot objects
	private TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;
	private DoubleSolenoid _shifterSolenoid;
	private NavXGyro _navX = NavXGyro.getInstance();
	
	// Controllers
	private RobotState _robotState = RobotState.getInstance();
	private PathFollower _pathFollower;
	private double _setpointright;
	
	private Path _currentPath = null;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	private ChassisDrivePerfMetrics _lastScanPerfMetricsSnapShot;
	
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
	

	private static final double CODES_PER_REV = 4590;
	public static final double CODES_PER_METER = 1367.18;
	
	// shifter positions
	public enum GearShiftPosition {
		UNKNOWN,
		HIGH_GEAR,
		LOW_GEAR
	}
	
	// Chassis various states
	public enum ChassisState {
		PERCENT_VBUS, 
		AUTO_TURN, 
		FOLLOW_PATH,
		VELOCITY_SETPOINT
	}
	
	// private constructor for singleton pattern
	private Chassis() {
		/* Drive Motors */
		_leftMaster = new TalonSRX(Constants.LEFT_DRIVE_MASTER_CAN_BUS_ADDR);
		_leftSlave = new TalonSRX(Constants.LEFT_DRIVE_SLAVE_CAN_BUS_ADDR);
		
		_rightMaster = new TalonSRX(Constants.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR);
		_rightSlave = new TalonSRX(Constants.RIGHT_DRIVE_SLAVE_CAN_BUS_ADDR);
		
		_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		
		_leftSlave.set(ControlMode.Follower, Constants.LEFT_DRIVE_MASTER_CAN_BUS_ADDR);
		_rightSlave.set(ControlMode.Follower, Constants.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR);

		_leftMaster.setInverted(false);
		_leftSlave.setInverted(false);
		_rightMaster.setInverted(true);
		_rightSlave.setInverted(true);

		
		_leftMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_leftSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_rightSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
	
        _leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        _leftMaster.configVelocityMeasurementWindow(32, 0);
        _rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        _rightMaster.configVelocityMeasurementWindow(32, 0);
        
		// Motion Magic Constants
		_leftMaster.configMotionAcceleration(Constants.DRIVE_TURN_MAX_ACC, 0);
		_rightMaster.configMotionAcceleration(Constants.DRIVE_TURN_MAX_ACC, 0);
		_leftMaster.configMotionCruiseVelocity(Constants.DRIVE_TURN_MAX_VEL, 0);
		_rightMaster.configMotionCruiseVelocity(Constants.DRIVE_TURN_MAX_VEL, 0);
		
		reloadGains();
		
		/* Shifter */
		_shifterSolenoid = new DoubleSolenoid(Constants.PCM_CAN_BUS_ADDR, Constants.SHIFTER_SOLENOID_EXTEND_PCM_PORT, 
												Constants.SHIFTER_SOLENOID_RETRACT_PCM_PORT);
		
		setBrakeMode(false);
		
		_driveSpeedScalingFactorClamped = 1.0;
		
		_lastScanPerfMetricsSnapShot = new ChassisDrivePerfMetrics();
	}
	
	private final ILoop _loop = new ILoop() 
	{
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
					case AUTO_TURN:
						ShiftGear(GearShiftPosition.LOW_GEAR);
						_leftMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
						_rightMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
						moveToTarget();
						return;
						
					case FOLLOW_PATH:
						ShiftGear(GearShiftPosition.HIGH_GEAR);
						_leftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
						_rightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
						if (_pathFollower != null) 
							updatePathFollower(timestamp);
						return;
						
					case PERCENT_VBUS:
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
	
	public ILoop getLoop() {
		return _loop;
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
		_chassisState = ChassisState.PERCENT_VBUS;
	
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
		_leftMaster.set(ControlMode.PercentOutput, _arcadeDriveThrottleCmdAdj - 0.7 * _arcadeDriveTurnCmdAdj);
		_rightMaster.set(ControlMode.PercentOutput, _arcadeDriveThrottleCmdAdj + 0.7 * _arcadeDriveTurnCmdAdj);
	}
	
	public synchronized void tankDrive(DriveCommand command) {
		_leftMaster.set(ControlMode.PercentOutput, command.leftCmd);
		_rightMaster.set(ControlMode.PercentOutput, command.rightCmd);
	}
	
	private void setMotionMagicTargetPosition(double leftPosition, double rightPosition) {
		_leftMaster.set(ControlMode.MotionMagic, leftPosition);
		_rightMaster.set(ControlMode.MotionMagic, rightPosition);
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
            _leftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
            _rightMaster.configNominalOutputForward(Constants.DriveHighGearNominalOutput, 0);
            _rightMaster.configNominalOutputReverse(-Constants.DriveHighGearNominalOutput, 0);
            
            _rightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
            _rightMaster.configNominalOutputForward(Constants.DriveHighGearNominalOutput, 0);
            _rightMaster.configNominalOutputReverse(-Constants.DriveHighGearNominalOutput, 0);
            
            setBrakeMode(true);
        }
    }
    
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(_chassisState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.DriveHighGearMaxSetpoint
                    ? Constants.DriveHighGearMaxSetpoint / max_desired : 1.0;
            _setpointright = inchesPerSecondToNativeUnits(left_inches_per_sec * scale);
            _leftMaster.set(ControlMode.Velocity, inchesPerSecondToNativeUnits(left_inches_per_sec * scale));
            _rightMaster.set(ControlMode.Velocity, inchesPerSecondToNativeUnits(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            _leftMaster.set(ControlMode.Velocity, 0);
            _rightMaster.set(ControlMode.Velocity, 0);
        }
    }
	
	public synchronized void setBrakeMode(boolean isBrakeMode) {
		NeutralMode mode;
		if (isBrakeMode) {
			mode = NeutralMode.Brake;
		} else {
			mode = NeutralMode.Coast;
		}
		
		_leftMaster.setNeutralMode(mode);
		_leftSlave.setNeutralMode(mode);
		_rightMaster.setNeutralMode(mode);
		_rightSlave.setNeutralMode(mode);
	}
	
	public synchronized void setTargetAngle(double target) {
		setAutoAimTarget(target);
	}
	
	private synchronized void setAutoAimTarget(double target) {
		_targetAngle = target;
		_chassisState = ChassisState.AUTO_TURN;
	}
	
	private synchronized void moveToTarget() {
		double angleError;
		
		angleError = _targetAngle - _navX.getYaw();
		
		double encoderError = GeneralUtilities.degreesToEncoderRotations(angleError);
		
		double leftDriveTargetPosition = getLeftPosInRot() - encoderError;
		double rightDriveTargetPosition = getRightPosInRot() + encoderError;
		
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
			case UNKNOWN:
				break;
		}
	}

	public synchronized void ToggleShiftGear() {
		if (_shifterSolenoidPosition == Constants.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) 
			ShiftGear(GearShiftPosition.LOW_GEAR);
		else 
			ShiftGear(GearShiftPosition.HIGH_GEAR);
	}
	
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
	
	public void setGyroAngle(double yaw) {
		_navX.zeroYaw();
		_navX.setAngleAdjustment(yaw);
	}
	
	public double getLeftPosInRot() {
		return _leftMaster.getSelectedSensorPosition(0) / CODES_PER_REV;
	}
	
	public double getRightPosInRot() {
		return _rightMaster.getSelectedSensorPosition(0) / CODES_PER_REV;
	}
	
	public double getLeftSpeed() {
		return _leftMaster.getSelectedSensorVelocity(0) / (600 * CODES_PER_REV);
	}
	
	public double getRightSpeed() {
		return -_rightMaster.getSelectedSensorVelocity(0) / (600 * CODES_PER_REV);
	}
	
	public double getLeftSpeedInMPS() {
		return _leftMaster.getSelectedSensorVelocity(0) / CODES_PER_METER;
	}
	
	public double getRightSpeedInMPS() {
		return _rightMaster.getSelectedSensorVelocity(0) / CODES_PER_METER;
	}
	
	public double getLeftDistanceInches() {
        return rotationsToInches(getLeftPosInRot());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(getRightPosInRot());
    }
    
    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getLeftSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getRightSpeed());
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
	
	private static double inchesPerSecondToNativeUnits(double inches_per_second) {
        return inches_per_second * 34.7;
    }
	
	public synchronized void reloadGains() {
        // Low Gear
        _leftMaster.config_kP(0, Constants.DRIVE_LOW_GEAR_POSITION_P, 0);
        _leftMaster.config_kI(0, Constants.DRIVE_LOW_GEAR_POSITION_I, 0);
        _leftMaster.config_kD(0, Constants.DRIVE_LOW_GEAR_POSITION_D, 0);
        _leftMaster.config_kF(0, Constants.DRIVE_LOW_GEAR_POSITION_F, 0);
        _leftMaster.config_IntegralZone(0, Constants.DRIVE_LOW_GEAR_POSITION_I_ZONE, 0);
        
        _rightMaster.config_kP(0, Constants.DRIVE_LOW_GEAR_POSITION_P, 0);
        _rightMaster.config_kI(0, Constants.DRIVE_LOW_GEAR_POSITION_I, 0);
        _rightMaster.config_kD(0, Constants.DRIVE_LOW_GEAR_POSITION_D, 0);
        _rightMaster.config_kF(0, Constants.DRIVE_LOW_GEAR_POSITION_F, 0);
        _rightMaster.config_IntegralZone(0, Constants.DRIVE_LOW_GEAR_POSITION_I_ZONE, 0);
        
        // High Gear
        _leftMaster.config_kP(1, Constants.DriveHighGearVelocityKp, 0);
        _leftMaster.config_kI(1, Constants.DriveHighGearVelocityKi, 0);
        _leftMaster.config_kD(1, Constants.DriveHighGearVelocityKd, 0);
        _leftMaster.config_kF(1, Constants.DriveHighGearVelocityKf, 0);
        _leftMaster.config_IntegralZone(1, Constants.DriveHighGearVelocityIZone, 0);
        
        _rightMaster.config_kP(1, Constants.DriveHighGearVelocityKp, 0);
        _rightMaster.config_kI(1, Constants.DriveHighGearVelocityKi, 0);
        _rightMaster.config_kD(1, Constants.DriveHighGearVelocityKd, 0);
        _rightMaster.config_kF(1, Constants.DriveHighGearVelocityKf, 0);
        _rightMaster.config_IntegralZone(1, Constants.DriveHighGearVelocityIZone, 0);
        
        _leftMaster.configClosedloopRamp(Constants.DRIVE_CLOSED_LOOP_RAMP_RATE, 0);
        _rightMaster.configClosedloopRamp(Constants.DRIVE_CLOSED_LOOP_RAMP_RATE, 0);
	}

	@Override
	public synchronized void stop() {
		setBrakeMode(true);
		arcadeDrive(0.0, 0.0);
	}

	@Override
	public synchronized void zeroSensors() {
		zeroEncoders();
		zeroGyro();
	}

	public void UpdateDriveTrainPerfMetrics()
	{
		_lastScanPerfMetricsSnapShot = CalcCurrentDriveMetrics(_lastScanPerfMetricsSnapShot);
	}
	
	// Publish Data to the Dashboard
	@Override
	public void outputToSmartDashboard() {
		//SmartDashboard.putNumber("Left Position", getLeftPosInRot());
		//SmartDashboard.putNumber("Left Drive Inches/Sec", getLeftSpeed());
		//SmartDashboard.putNumber("Right Position", getRightPosInRot());
		//SmartDashboard.putNumber("Right Drive Inches/Sec", getLeftSpeed());
		
		//SmartDashboard.putNumber("Left Position in Inches", getLeftDistanceInches());
		//SmartDashboard.putNumber("Right Position in Inches", getRightDistanceInches());
		SmartDashboard.putNumber("Left Target Velocity", _setpointright);
		//SmartDashboard.putNumber("Left Position", _leftMaster.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("Left Position Quadruature", _leftMaster.getSensorCollection().getQuadraturePosition());
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Chassis:LeftDriveMtr%VBus", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.LeftDriveMtrPercentVBus, 2)));
		logData.AddData("Chassis:LeftDriveMtrPos [m]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.LeftDriveMtrPos, 2)));
		logData.AddData("Chassis:LeftDriveMtrVel [m/s]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.LeftDriveMtrMPS, 2)));
		logData.AddData("Chassis:LeftDriveMtrAccel [m/s/s]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.LeftDriveMtrMPSPerSec, 2)));

		logData.AddData("Chassis:RightDriveMtr%VBus", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.RightDriveMtrPercentVBus, 2)));
		logData.AddData("Chassis:RightDriveMtrPos [m]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.RightDriveMtrPos, 2)));
		logData.AddData("Chassis:RightDriveMtrVel [m/s]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.RightDriveMtrMPS, 2)));
		logData.AddData("Chassis:RightDriveMtrAccel [m/s/s]", String.valueOf(GeneralUtilities.RoundDouble(_lastScanPerfMetricsSnapShot.RightDriveMtrMPSPerSec, 2)));
	}
	
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
	
	private ChassisDrivePerfMetrics CalcCurrentDriveMetrics(ChassisDrivePerfMetrics previousScanMetrics)
	{
		ChassisDrivePerfMetrics currentScanMetrics = new ChassisDrivePerfMetrics();
		
		currentScanMetrics.LastSampleTimestampInMSec = new Date().getTime();
		
		currentScanMetrics.LeftDriveMtrPercentVBus = _leftMaster.getMotorOutputVoltage()/_leftMaster.getBusVoltage();
		currentScanMetrics.LeftDriveMtrPos = _leftMaster.getSelectedSensorPosition(0) / (10 * CODES_PER_METER); //Native Units
		currentScanMetrics.LeftDriveMtrMPS = getLeftSpeedInMPS(); //Native units per 100ms
		
		if(previousScanMetrics != null)
		{
			currentScanMetrics.LeftDriveMtrMPSPerSec = CalcAccDec(previousScanMetrics.LeftDriveMtrMPS,
																	previousScanMetrics.LastSampleTimestampInMSec,
																	currentScanMetrics.LeftDriveMtrMPS,
																	currentScanMetrics.LastSampleTimestampInMSec);
		} 
		else {
			currentScanMetrics.LeftDriveMtrMPSPerSec = 0.0;
		}
		currentScanMetrics.RightDriveMtrPercentVBus = -1 * (_rightMaster.getMotorOutputVoltage()/_rightMaster.getBusVoltage()); 
		currentScanMetrics.RightDriveMtrPos = _rightMaster.getSelectedSensorPosition(0) / (10 * CODES_PER_METER);
		currentScanMetrics.RightDriveMtrMPS = getRightSpeedInMPS(); 
		
		if(previousScanMetrics != null)
		{
			currentScanMetrics.RightDriveMtrMPSPerSec = CalcAccDec(previousScanMetrics.RightDriveMtrMPS,
																	previousScanMetrics.LastSampleTimestampInMSec,
																	currentScanMetrics.RightDriveMtrMPS,
																	currentScanMetrics.LastSampleTimestampInMSec);
		}
		else {
			currentScanMetrics.RightDriveMtrMPSPerSec = 0.0;
		}
		
		return currentScanMetrics;
	}
	
	private double CalcAccDec(double previousMtrMPS, long prevTimeStampInMSec, double currMtrMPS, long currTimeStampInMSec)
	{
		double deltaVInRPM = currMtrMPS - previousMtrMPS;
		
		double deltaTInMsec = currTimeStampInMSec - prevTimeStampInMSec;
		
		double deltaTInSec = deltaTInMsec / 1000.0;

		double accDecInRPMPerSec = deltaVInRPM / deltaTInSec;
		
		return accDecInRPMPerSec;
	}
	
	class ChassisDrivePerfMetrics
	{
		public double LeftDriveMtrPercentVBus;
		public double LeftDriveMtrPos;
		public double LeftDriveMtrMPS;			// velocity
		public double LeftDriveMtrMPSPerSec;	// acc/dec
		

		public double RightDriveMtrPercentVBus;
		public double RightDriveMtrPos;
		public double RightDriveMtrMPS;
		public double RightDriveMtrMPSPerSec;	// acc/dec
		
		public long LastSampleTimestampInMSec;
	}
}