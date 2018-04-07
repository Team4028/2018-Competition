package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.paths.AdaptedPaths;
import org.usfirst.frc.team4028.robot.sensors.PressureSensor;
import org.usfirst.frc.team4028.robot.sensors.RobotStateEstimator;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
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
	private Climber _climber = Climber.getInstance();
	
	// Sensors
	private PressureSensor _pressureSensor = PressureSensor.getInstance();
	private SwitchableCameraServer _switchableCameraServer = SwitchableCameraServer.getInstance();
	
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
		_enabledLooper.register(_cubeHandler.getLoop());
		_enabledLooper.register(RobotStateEstimator.getInstance().getLoop());
		
		_dashboard.printStartupMessage();
		
		// Hold scan times moving average samples
		_scanTimeSamples = new MovingAverage(100);
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		outputAllToDashboard();
		
		System.out.println(payRespectsToRNGesus());
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
		
		if (_dataLogger != null) {
			_dataLogger.close();
			_dataLogger = null;
		}
		
		_dashboard.outputToDashboard();
	}
	
	/** Called once, each time the robot enters autonomous mode. */
	@Override
	public void autonomousInit() {
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;
		_dos.clearGamepadsCachedBtnPresses();
		
		_enabledLooper.start();
		
		int retries = 100;
		
		while(!_dashboard.isGameDataReceived() && retries > 0) {
			retries--;
			try { 
				Thread.sleep(5);
			} catch (InterruptedException ie) {}
		}
		
		if (retries == 0) {
			DriverStation.reportError("Failed To Receive Game Data", false);
		}
		
		_chassis.zeroGyro();
		
		AdaptedPaths.locateFlavorTownUSA();
		
		_autonExecuter = new AutonExecuter();
		_autonExecuter.setAutoMode(_dashboard.getSelectedAuton());
		_autonExecuter.start();

		_chassis.zeroGyro();
		_chassis.setBrakeMode(true);
		
		_dataLogger = GeneralUtilities.setupLogging("Auton"); // init data logging
		
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		
		_dashboard.outputToDashboard();
		
		_autonStartTime = System.currentTimeMillis();
		
		System.out.println(payRespectsToRNGesus());
	}

	/** Called each loop (approx every 20mS) in autonomous mode */
	@Override
	public void autonomousPeriodic() {	
		outputAllToDashboard(); // Refresh Dashboard
		
		_chassis.setBrakeMode(true);
		
		logAllData(); // Optionally Log Data
	}

	/** Called once, each time the robot enters teleop mode. */
	@Override
	public void teleopInit() {		
		if (_autonExecuter != null) {
			_autonExecuter.stop();
		}
		_autonExecuter = null;

		_enabledLooper.start();
		
		stopAll();
		
		// setup chassis default state
		_chassis.zeroSensors();
		_chassis.setHighGear(false);
		_chassis.setBrakeMode(true);  
		_chassis.stop();
		
		_dos.clearGamepadsCachedBtnPresses();
		
		_dataLogger = GeneralUtilities.setupLogging("Teleop"); // init data logging
		
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		
		System.out.println(payRespectsToRNGesus());
	}


	/** Called each loop (approx every 20mS) in telop mode */
	@Override
	public void teleopPeriodic() {
		// =============  CHASSIS ============= 
		if ((Math.abs(_dos.getDriver_Throttle_JoystickCmd()) != 0) || (Math.abs(_dos.getDriver_Turn_JoystickCmd()) != 0)) {
			if (_elevator.isElevatorAtUnsafeHeight()) {
				_chassis.arcadeDrive(-0.5 * _dos.getDriver_Throttle_JoystickCmd(), -0.8 * _dos.getDriver_Turn_JoystickCmd());
			} 
			else {
				_chassis.arcadeDrive(-1 * _dos.getDriver_Throttle_JoystickCmd(), -1 * _dos.getDriver_Turn_JoystickCmd());
			}
		} else {
			_chassis.stop();
		}

		if (_dos.getIsDriver_ShiftGear_BtnJustPressed()) {
			_chassis.toggleShifter();
		}
	
		//=============  INFEED ============= 
		if(!_dos.IsEngineeringGamepadBAvailable()
				&& !_dos.IsEngineeringGamepadAAvailable()) {
			// =============================
			// ENGR GamePad A and B ARE NOT plugged in
			// =============================
			if (_dos.getIsDriver_RezeroInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_Rezero();
			}		
			else if (_dos.getIsDriver_WideInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToWidePosition();
			}
			else if (_dos.getIsDriver_SqueezeInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToSqueezePosition();
			}
			else if (_dos.getIsDriver_StoreInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToStorePosition();
			}
			
			// ============= CARRIAGE =============
			// adjust Carriage Wheel Feed Out Speeds
			if(_dos.getIsDriver_Carriage_FeedOut_IndexUp_BtnJustPressed()
					|| _dos.getIsOperator_Carriage_FeedOut_IndexUp_BtnJustPressed()) {
				_cubeHandler.carriage_FeedOut_VBusCmd_IndexBumpUp();
			}
			else if(_dos.getIsDriver_Carriage_FeedOut_IndexDown_BtnJustPressed()
					|| _dos.getIsOperator_Carriage_FeedOut_IndexDown_BtnJustPressed()) {
				_cubeHandler.carriage_FeedOut_VBusCmd_IndexBumpDown();
			}
			
			if (_dos.getIsDriver_SpinCubeCounterClockwise_BtnPressed()){
				_cubeHandler.infeedArms_SpinCube_CCW();
			}
			else if (_dos.getIsDriver_SpinCubeClockwise_BtnPressed()) {
				_cubeHandler.infeedArms_SpinCube_CW();
			} 
			else if (Math.abs(_dos.getDriver_InfeedCube_JoystickCmd()) != 0) {
				_cubeHandler.acquireCube_InfeedAndCarriage();
			}			
			else if ((Math.abs(_dos.getDriver_EjectCube_JoystickCmd()) != 0) || _dos.getOperator_EjectCube_JoystickCmd() != 0) {
				_cubeHandler.ejectCube_InfeedAndCarriage();
			}
			else {
				_cubeHandler.stop_InfeedAndCarriage();			
			}
			
			if(_dos.getIsOperator_SqueezeCarriage_BtnPressed()) {
				_cubeHandler.carriage_MoveSolenoidToSqueeze();
			}
			else if (_dos.getIsOperator_WideCarriage_BtnPressed()) {
				_cubeHandler.carriage_MoveSolenoidToWide();
			}		
		} 
		else if(_dos.IsEngineeringGamepadBAvailable()) {
			// =============================
			// ENGR GamePad B is plugged In
			// =============================
			// ignore Driver Gamepad if Engineering B is plugged in
			// adjust Infeed Arm Width
			if(_dos.getIsEngrB_SqueezeBumpWider_BtnJustPressed()) {
				_cubeHandler.infeedArms_SqueezeAngle_BumpWider();
			}
			else if(_dos.getIsEngrB_SqueezeBumpNarrower_BtnJustPressed()) {
				_cubeHandler.infeedArms_SqueezeAngle_BumpNarrower();
			}
			
			// adjust Infeed Wheel speeds
			if(_dos.getIsEngrB_InfeedVBusBumpDown_BtnJustPressed())	{
				_cubeHandler.infeedWheels_VBusCmd_BumpDown();
			}
			else if(_dos.getIsEngrB_InfeedVBusBumpUp_BtnJustPressed()) {
				_cubeHandler.infeedWheels_VBusCmd_BumpUp();
			}
			
			// adjust Carriage Wheel Speeds
			if(_dos.getIsEngrB_Carriage_FeedIn_VBusBumpDown_BtnJustPressed()) {
				_cubeHandler.carriage_FeedIn_VBusCmd_BumpDown();
			}
			else if(_dos.getIsEngrB_Carriage_FeedIn_VBusBumpUp_BtnJustPressed()) {
				_cubeHandler.carriage_FeedIn_VBusCmd_BumpUp();
			}
			
			if(_dos.getIsEngrB_Carriage_FeedOut_IndexBumpDown_BtnJustPressed()) {
				_cubeHandler.carriage_FeedOut_VBusCmd_IndexBumpDown();
			}
			else if(_dos.getIsEngrB_Carriage_FeedOut_IndexBumpUp_BtnJustPressed()) {
				_cubeHandler.carriage_FeedOut_VBusCmd_IndexBumpUp();
			}
			
			// infeed arm positions
			if (_dos.getIsEngrB_RezeroInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_Rezero();
			}
			else if (_dos.getIsEngrB_WideInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToWidePosition();
			}
			else if (_dos.getIsEngrB_SqueezeInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToSqueezePosition();
			}
			else if (_dos.getIsEngrB_StoreInfeed_BtnJustPressed()) {
				_cubeHandler.infeedArms_moveToStorePosition();
			}
			
			// infeed wheel control
			if (_dos.getEngrB_InfeedSpin_JoystickCmd() == 1.0) {
				_cubeHandler.infeedArms_SpinCube_CCW();
			}
			else if (_dos.getEngrB_InfeedSpin_JoystickCmd() == -1.0) {
				_cubeHandler.infeedArms_SpinCube_CW();
			}
			else if (_dos.getEngrB_InfeedAndCarriage_JoystickCmd() == 1.0) {
				_cubeHandler.ejectCube_InfeedAndCarriage();
			}
			else if (_dos.getEngrB_InfeedAndCarriage_JoystickCmd() == -1.0) {
				_cubeHandler.acquireCube_InfeedAndCarriage();
			}
			else {
				_cubeHandler.stop_InfeedAndCarriage();
			}
		}
		
		// =============  ELEVATOR ============= 		
		if (_dos.getIsOperator_ElevatorInfeedHgt_BtnJustPressed()) {
			_cubeHandler.elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT);
		}
		else if (_dos.getIsOperator_ElevatorSwitchHgt_BtnJustPressed()) {
			_cubeHandler.elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT);
		}
		else if (_dos.getIsOperator_ElevatorNeutralScaleHgt_BtnJustPressed()) {
			_cubeHandler.elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION.NEUTRAL_SCALE_HEIGHT);
		}
//		else if (_dos.getIsOperator_ElevatorReHome_BtnJustPressed()) {
//			_cubeHandler.elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION.HOME);
//		} 
		else if (_dos.getIsOperator_ElevatorClimbHgt_BtnJustPressed()){
			_cubeHandler.elevator_MoveToPresetPosition(ELEVATOR_PRESET_POSITION.CLIMB_SCALE_HEIGHT);
		} else {
			_cubeHandler.stopElevator();
		} 
		
		// Neutral Scale Height / Climb Scale Height Bumps
		if(_dos.getIsOperator_ElevatorScaleHeightBumpUp_BtnJustPressed()) {
			_cubeHandler.elevator_ScaleHeight_BumpPositionUp();
		}
		else if (_dos.getIsOperator_ElevatorScalePositionBumpDown_BtnJustPressed()) {
			_cubeHandler.elevator_ScaleHeight_BumpPositionDown();
		}
				
		// =============  CLIMBER ============= 
		_climber.runMotor(_dos.getOperator_Climber_JoystickCmd());

		// =============  CLIMBER SERVO ============= 
		if(_dos.getIsOperator_ToggleClimberServo_BtnJustPressed()) {
			_climber.toggleClimberServo();
			_cubeHandler.infeedArm_moveRightInfeedArmToClimbPosition();
		}
		
		// ============= Camera Switch ============= 
		if (_dos.getIsOperator_SwitchCamera_BtnJustPressed() == true) {
			_switchableCameraServer.SwitchCamera();
		}
		
		// ============= Refresh Dashboard =============
		outputAllToDashboard();
		
		// ============= Optionally Log Data =============
		logAllData();
	}
	
	/** Methods for Stopping All Motors on Every Subsystem (Every Subsystem w/ Motors needs a method here) */
	private void stopAll() {
		_chassis.stop();
		_elevator.stop();
		_infeed.stop();
		_carriage.stop();
		_climber.stop();
	}
	
	/** Method to Push Data to ShuffleBoard */
	private void outputAllToDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	// add scan time sample to calc scan time rolling average
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		// each subsystem should add a call to a outputToSmartDashboard method
    		// to push its data out to the dashboard

    		//_chassis.outputToShuffleboard(); 
    		_elevator.outputToShuffleboard();
    		//_infeed.outputToShuffleboard();
    		//_carriage.outputToShuffleboard();
	    	//_cubeHandler.outputToShuffleboard();
	    	//_climber.outputToShuffleboard();
	    	//_pressureSensor.outputToShuffleboard();
	    	
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
	
	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
	private void logAllData() { 
		// always call this 1st to calc drive metrics
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
	    	
	    	// ask each subsystem that exists to add its data
	    	_chassis.updateLogData(logData);
	    	_elevator.updateLogData(logData);
	    	_infeed.updateLogData(logData);
	    	_carriage.updateLogData(logData);
	    	_cubeHandler.updateLogData(logData);
	    	_pressureSensor.updateLogData(logData);
	    	
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
	
	private String payRespectsToRNGesus() {
		return "RNGesus, please provide us with a favorable qualification schedule and ideal scoring plate assignment of LLL";
	}
}