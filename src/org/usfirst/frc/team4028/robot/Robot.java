package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.subsystems.*;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.util.DataLogger;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogData;
import org.usfirst.frc.team4028.util.MovingAverage;
import org.usfirst.frc.team4028.util.loops.Looper;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private static final String ROBOT_NAME = "2018 COMPETITION";
	
	// Subsystems
	private Chassis _chassis = Chassis.getInstance();
	
	// Other
	private ControlBoard _controlBoard = ControlBoard.getInstance();
	private AutonExecuter _autonExecuter = null;
	private SmartDashboardInputs _smartDashboard = SmartDashboardInputs.getInstance();
	private DataLogger _dataLogger = null;
	
	Looper _enabledLooper = new Looper();
	
	String _buildMsg = "?";
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
 	long _autonStartTime;
	
	@Override
	public void robotInit() {
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
		
		_enabledLooper.register(_chassis.getLoop());
		_enabledLooper.register(RobotStateEstimator.getInstance().getLoop());
		
		_smartDashboard.printStartupMessage();
		
		// create class to hold Scan Times moving Average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		outputAllToSmartDashboard();
	}

	@Override
	public void disabledInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.stop();
		stopAll();
	}

	@Override
	public void disabledPeriodic() {
		if (_dataLogger != null) {
			_dataLogger.close();
			_dataLogger = null;
		}
		stopAll();
	}
	
	@Override
	public void autonomousInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		_autonExecuter = new AutonExecuter();
		_autonExecuter.setAutoMode(_smartDashboard.getSelectedAuton());
		_autonExecuter.start();
		
		_chassis.zeroSensors();
		
		_dataLogger = GeneralUtilities.setupLogging("auton");
		
		_lastDashboardWriteTimeMSec = new Date().getTime();
		_autonStartTime=System.currentTimeMillis();
	}

	@Override
	public void autonomousPeriodic() {	
		logAllData();
		outputAllToSmartDashboard();
		
	}

	@Override
	public void teleopInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		stopAll();
		
		_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
		_chassis.setBrakeMode(false);
		_chassis.zeroSensors();
		
		_dataLogger = GeneralUtilities.setupLogging("teleop");
	}

	@Override
	public void teleopPeriodic() {
		// Chassis Throttle & Turn
		if ((Math.abs(_controlBoard.getThrottleCmd()) > 0.05) 
				|| (Math.abs(_controlBoard.getTurnCmd()) > 0.05)) {
			_chassis.arcadeDrive(-1.0 * _controlBoard.getThrottleCmd(), _controlBoard.getTurnCmd());
		} else {
			_chassis.stop();
		}
		
		if (_controlBoard.getIsShiftGearJustPressed()) 
		{
			_chassis.ToggleShiftGear();
		}
		// Refresh Dashboard
		outputAllToSmartDashboard();
		
		// Optionally Log Data
		logAllData();
	}
	
	private void stopAll() {
		_chassis.stop();
	}
	
	private void outputAllToSmartDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		_chassis.outputToSmartDashboard(); 
	    	
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
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data
	    	_chassis.updateLogData(logData);
	    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}