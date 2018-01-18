package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.sensors.Ultrasonic;
import org.usfirst.frc.team4028.robot.subsystems.*;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.util.DataLogger;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.MovingAverage;
import org.usfirst.frc.team4028.util.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private static final String ROBOT_NAME = "2018 COMPETITION";
	
	// Subsystems
	private Chassis _chassis = Chassis.getInstance();
	
	// class level private variables for all Subsystems & Operator Interface Instances
	private DashboardInputs _dashboardInputs;
	private DriverOperationStation _dos;
	
	// Other class level private instances 
	private AutonExecuter _autonExecuter;
	private DataLogger _dataLogger;
	private Looper _enabledLooper;
	private Ultrasonic _ultrasonic;
	
	// class level working variables
	String _buildMsg = "?";
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
 	
 	TalonSRX talon = new TalonSRX(11);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// initialize Instance Variables
		_chassis = Chassis.getInstance();
		_dashboardInputs = DashboardInputs.getInstance();
		_dos = DriverOperationStation.getInstance();
		_ultrasonic = Ultrasonic.getInstance();
					
		// init looper
		_enabledLooper = new Looper();
		_enabledLooper.register(_chassis.getLoop());
		_enabledLooper.register(RobotStateEstimator.getInstance().getLoop());
			
		// create class to hold Scan Times moving Average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan

		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
		_dashboardInputs.printStartupMessage();
		outputAllToDashboard(true);
	}

  /* ================================================================================
   * Initialization code which will be called each time the robot enters disabled mode.
   * ================================================================================ */
	@Override
	public void disabledInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.stop();
		
		if (_dataLogger != null) {
			_dataLogger.close();
			_dataLogger = null;
		}
				
		stopAll();
	}

	/* ================================================================================
	 * Periodic code for while the robot is in disabled mode.
   	 * ================================================================================*/
	@Override
	public void disabledPeriodic() {
				
		stopAll();
	}
	
	/* ================================================================================
	 * Initialization code which will be called each time the robot enters Autonomous mode.
   	 * ================================================================================*/
	@Override
	public void autonomousInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		_autonExecuter = new AutonExecuter();
		_autonExecuter.setAutoMode(_dashboardInputs.getSelectedAuton());
		_autonExecuter.start();
		
		_dataLogger = GeneralUtilities.setupLogging("auton");
		
		_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	/* ================================================================================
	 * Periodic code for while the robot is in Autonomous mode.
	 *   This is called approx every 20 mSec
   	 * ================================================================================*/
	@Override
	public void autonomousPeriodic() {	

		_chassis.arcadeDrive(.1, 0);
		
		talon.set(ControlMode.PercentOutput, .1);
		
		logAllData();
		outputAllToDashboard();
	}

	/* ================================================================================
	 * Initialization code which will be called each time the robot enters Telop mode.
   	 * ================================================================================*/
	@Override
	public void teleopInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		stopAll();
		
		// set chassis telop defaults
		_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
		_chassis.setBrakeMode(false);
		_chassis.zeroSensors();
		
		_dataLogger = GeneralUtilities.setupLogging("teleop");
		_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	/* ================================================================================
	 * Periodic code for while the robot is in Periodic mode.
	 *   This is called approx every 20 mSec
   	 * ================================================================================*/
	@Override
	public void teleopPeriodic() {
		// handle Chassis Throttle & Turn commands
		_chassis.arcadeDrive(_dos.getThrottleCmd(), _dos.getTurnCmd());

		//Handle Ultrasonic Sensor
		_ultrasonic.calculateDistanceReadings();
		
		// Refresh Dashboard
		outputAllToDashboard();
		
		// Optionally Log Data
		_chassis.UpdateDriveTrainPerfMetrics();
		logAllData();
	}
	
	private void stopAll() {
		_chassis.stop();
	}
	
	// simulate default parameter values with a method overload
	private void outputAllToDashboard() {
		this.outputAllToDashboard(false);
	}
	
	private void outputAllToDashboard(Boolean isInRobotInit) {

		if(isInRobotInit)
		{
			// push this value at startup so the field appears on the dashboard
			// otherwise it would not be written until the 10th scan
			SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		}
		
		// calc scan time
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
		// limit spamming (only write every 100 mSec)
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		_chassis.outputToSmartDashboard();
    		_ultrasonic.outputToDashboard();
	    	
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	SmartDashboard.putString("FMS Debug Msg", _fmsDebugMsg);
	    	
	    	BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	DecimalFormat df = new DecimalFormat("####");
	    	SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
	    				
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
	}
	
	private void logAllData() {
		// always call this 1st to calc drive metrics
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
	    	
	    	// ask each subsystem that exists to add its data
	    	_chassis.updateLogData(logData);
	    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}