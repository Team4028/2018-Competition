package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.sensors.RobotStateEstimator;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.sensors.UltrasonicSensor;
import org.usfirst.frc.team4028.robot.subsystems.*;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.DataLogger;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;
import org.usfirst.frc.team4028.util.MovingAverage;
import org.usfirst.frc.team4028.util.loops.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This is the main class for the Robot
//	It contains all startup, init & periodic entry points
public class Robot extends IterativeRobot {
	private static final String ROBOT_NAME = "2018 COMPETITION";
	
	// Subsystems
	private Chassis _chassis = Chassis.getInstance();
	private Infeed _infeed = Infeed.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Carriage _carriage = Carriage.getInstance();
	private CubeHandler _cubeHandler = CubeHandler.getInstance();
	
	// Sensors
	private UltrasonicSensor _ultrasonic = UltrasonicSensor.getInstance();
	private SwitchableCameraServer _switchableCameraServer = SwitchableCameraServer.getInstance();
	//private PDPMonitor _pdpm = PDPMonitor.getInstance();
	
	// Other
	private DriverOperatorStation _dos = DriverOperatorStation.getInstance();
	private AutonExecuter _autonExecuter = null;
	private Dashboard _dashboard = Dashboard.getInstance();
	private DataLogger _dataLogger = null;
	
	private Looper _enabledLooper = new Looper();
	
	// Class level variables
	String _buildMsg = "?";
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
 	long _autonStartTime;
	
 	// ================================================================
 	// Robot-wide initialization code should go here
 	// called once, each time the robot powers up or the roborio is reset
 	// ================================================================
	@Override
	public void robotInit() {
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
		
		_enabledLooper.register(_chassis.getLoop());
		_enabledLooper.register(_infeed.getLoop());
		_enabledLooper.register(_elevator.getLoop());
		_enabledLooper.register(_carriage.getLoop());
		_enabledLooper.register(RobotStateEstimator.getInstance().getLoop());
		
		_dashboard.printStartupMessage();
		
		// Hold scan times moving average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		outputAllToDashboard();
	}

	// ================================================================
	// called once, each time the robot enters disabled mode from
    // either a different mode or from power-on
	// ================================================================
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
		
		_chassis.setBrakeMode(false);
		
		
		
		stopAll();
	}

	// ================================================================
	// called each loop (approx every 20mS) in disabled mode
	// Note: outputs (ex: Motors, PWM, PCM etc are disabled
	//			but you can perform internal "reset state" actions
	// ================================================================
	@Override
	public void disabledPeriodic() {
		stopAll();
	}
	
	// ================================================================
	// called once, each time the robot enters autonomous mode.
	// ================================================================
	@Override
	public void autonomousInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		_enabledLooper.start();
		
		int retries = 100;
		
		while(!_dashboard.isGameDataReceived() && retries > 0) {
			retries--;
			try { 
				Thread.sleep(5);
			} catch (InterruptedException ie) {
				// Ignore InterruptedException
			}
		}
		
		if (retries == 0) {
			DriverStation.reportError("Failed To Receive Game Data", false);
		}
		
		_chassis.zeroGyro();
		
		_autonExecuter = new AutonExecuter();
		_autonExecuter.setAutoMode(_dashboard.getSelectedAuton());
		_autonExecuter.start();

		_chassis.zeroGyro();
		_chassis.setBrakeMode(true);
		
		_cubeHandler.doNothing();

		_infeed.reZeroArms();

		// init data logging
		_dataLogger = GeneralUtilities.setupLogging("auton");
		// snapshot time to control spamming
		_lastDashboardWriteTimeMSec = new Date().getTime();
		
		_autonStartTime = System.currentTimeMillis();
	}

	// ================================================================
	// called each loop (approx every 20mS) in autonomous mode
	// ================================================================
	@Override
	public void autonomousPeriodic() {	
		// Refresh Dashboard
		outputAllToDashboard();
		
		_chassis.setBrakeMode(true);
		
		// Optionally Log Data
		logAllData();
	}

	// ================================================================
	// called once, each time the robot enters teleop mode.
	// ================================================================
	@Override
	public void teleopInit() {		
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		
		stopAll();
		
		_chassis.zeroSensors();
		_chassis.setHighGear(false);
		_chassis.setBrakeMode(false);  
				
		_cubeHandler.doNothing();
		
		_enabledLooper.start();
		// init data logging
		_dataLogger = GeneralUtilities.setupLogging("auton");
		// snapshot time to control spamming
		_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	// ================================================================
	// called each loop (approx every 20mS) in telop mode
	// ================================================================
	@Override
	public void teleopPeriodic() {		
		_ultrasonic.refreshUltrasonicValues();
		
		// =============  CHASSIS ============= 
		
		if ((Math.abs(_dos.getDriver_Throttle_JoystickCmd()) != 0) || (Math.abs(_dos.getDriver_Turn_JoystickCmd()) != 0)) {
			_chassis.arcadeDrive(-1 * _dos.getDriver_Throttle_JoystickCmd(), -1 * _dos.getDriver_Turn_JoystickCmd());
		} else {
			_chassis.stop();
		}

		if (_dos.getIsDriver_ShiftGear_BtnJustPressed()) {
			_chassis.toggleShifter();
		}
	
		//=============  INFEED ============= 
		if (_dos.getIsDriver_RezeroInfeed_BtnJustPressed() || _dos.getIsEngineering_ReZeroInfeed_BtnJustPressed()) {
			_infeed.reZeroArms();
		}		
		else if (_dos.getIsDriver_WideInfeed_BtnJustPressed() || _dos.getIsEngineering_WideInfeed_BtnPressed()) {
			_infeed.moveArmsToWideInfeedPosition();
		}
		else if (_dos.getIsDriver_SqueezeInfeed_BtnJustPressed() || _dos.getIsEngineering_SqueezeInfeed_BtnPressed()) {
			_infeed.moveArmsToSqueezeInfeedPosition();
		}
		else if (_dos.getIsDriver_StoreInfeed_BtnJustPressed() || _dos.getIsEngineering_StoreInfeed_BtnPressed()) {
			_infeed.storeArms();
		}
		else if (_dos.getIsEngineering_StaggerInfeed_BtnPressed()) {
			_infeed.staggerInfeedManuver();
		}
//		else if (_dos.getOperator_InfeedPositionX_JoystickCmd() > 0.5 
//				|| _dos.getOperator_InfeedPositionY_JoystickCmd() > 0.5) {
//			_infeed.infeedJoystickCommandedPosition(_dos.getOperator_InfeedPositionY_JoystickCmd(), 
//					_dos.getOperator_InfeedPositionX_JoystickCmd());
//		}
//		else if (_dos.getIsDriver_AutoAcquire_BtnJustPressed()) {
//			_infeed.autoInfeedManuver();
//		}

		// =============  ELEVATOR ============= 
		
		if (_dos.getOperator_Elevator_JoystickCmd() != 0) {
			_elevator.JogAxis(_dos.getOperator_Elevator_JoystickCmd());
		}
		else if (_dos.getEngineering_Elevator_JoystickCmd() != 0) {
			_elevator.JogAxis(_dos.getEngineering_Elevator_JoystickCmd());
		}
		else if (_dos.getIsOperator_ElevatorCubeOnFloorHgt_BtnJustPressed() || _dos.getIsEngineering_ElevatorCubeOnFloorHgt_BtnJustPressed()) {
			_elevator.MoveToPresetPosition(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR);
		}	
		else if (_dos.getIsOperator_ElevatorScaleHgt_BtnJustPressed() || _dos.getIsEngineering_ElevatorScaleHgt_BtnJustPressed()) {
			_elevator.MoveToPresetPosition(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT);
		} 
		else if (_dos.getIsOperator_ElevatorSwitchHgt_BtnJustPressed() || _dos.getIsEngineering_ElevatorSwitchHgt_BtnJustPressed()) {
			_elevator.MoveToPresetPosition(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT);
		}
		else if (_dos.getIsOperator_ElevatorPyrmdLvl1Hgt_BtnJustPressed() || _dos.getIsEngineering_ElevatorPyramidHgt_BtnJustPressed()) {
			_elevator.MoveToPresetPosition(ELEVATOR_PRESET_POSITION.CUBE_ON_PYRAMID_LEVEL_1);
		}		
		else if (_dos.getIsOperator_ElevatorHome_BtnJustPressed()) {
			_elevator.MoveToPresetPosition(ELEVATOR_PRESET_POSITION.HOME);
		} 
//		else if(_cubeHandler.isStateMachineCurrentlyRunning()) {
//			System.out.println("State Machine Check");
//			_cubeHandler.manageMoveElevatorToPresetPosition();
//		}
		else {
			_elevator.stop();
		} 
		
		// ============= CARRIAGE =============
		if (Math.abs(_dos.getDriver_InfeedCube_JoystickCmd()) != 0) {
			_cubeHandler.runInfeedCubePlusCarriage(_dos.getDriver_InfeedCube_JoystickCmd());
		}
		else if (Math.abs(_dos.getEngineering_InfeedCube_JoystickCmd()) != 0) {
			_cubeHandler.runInfeedCubePlusCarriage(_dos.getEngineering_InfeedCube_JoystickCmd());
		}			
		else if (Math.abs(_dos.getDriver_EjectCube_JoystickCmd()) != 0) {
			_cubeHandler.ejectCube(_dos.getDriver_EjectCube_JoystickCmd());
		} 
		else if (Math.abs(_dos.getEngineering_EjectCube_JoystickCmd()) != 0) {
			_cubeHandler.ejectCube(_dos.getEngineering_EjectCube_JoystickCmd());
		} 
		else if (_dos.getIsDriver_SpinCubeManuver_BtnPressed() || _dos.getIsEngineering_SpinCubeManuver_BtnPressed()){
			_cubeHandler.runInfeedSpinManuver();	
		} else {
			_cubeHandler.stop();			
		}
		
		//if(_dos.getIsDriver_MoveCarriageCloser_BtnJustPressed()) {
		//	_carriage.moveCarriageServosCloser();
		//}
		//else if(_dos.getIsDriver_MoveCarriageWider_BtnJustPressed()) {
		//	_carriage.moveCarriageServosWider();
		//}
				
		// ============= Camera Switch ============= 
		if (_dos.getIsOperator_SwitchCamera_BtnJustPressed() == true) {
			_switchableCameraServer.SwitchCamera();
		}
		
		// ============= Refresh Dashboard =============
		outputAllToDashboard();
		
		// ============= Optionally Log Data =============
		logAllData();
	}
	
	//=====================================================================================
	//Methods for Stopping All Motors on Every Subsystem (Every Subsystem w/ Motors needs a method here)
	//=====================================================================================
	private void stopAll() {
		_chassis.stop();
		//_elevator.stop();
		_infeed.stop();
		_carriage.stop();
	}
	
	//=====================================================================================
	//Method to Push Data to ShuffleBoard
	//=====================================================================================
	private void outputAllToDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	// add scan time sample to calc scan time rolling average
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		// each subsystem should add a call to a outputToSmartDashboard method
    		// to push its data out to the dashboard

    		_chassis.outputToShuffleboard(); 
    		_elevator.outputToShuffleboard();
    		_infeed.outputToShuffleboard();
    		_carriage.outputToShuffleboard();
    		_ultrasonic.outputToShuffleboard();
	    	
    		// write the overall robot dashboard info
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
	
	//=====================================================================================
	//Method for Logging Data to the USB Stick plugged into the RoboRio
	//=====================================================================================
	private void logAllData() { 
		// always call this 1st to calc drive metrics
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
	    	
	    	// ask each subsystem that exists to add its data
	    	_chassis.updateLogData(logData);
	    	//_elevator.updateLogData(logData);
	    	_infeed.updateLogData(logData);
	    	_carriage.updateLogData(logData);
	    	_ultrasonic.updateLogData(logData);
	    	
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}