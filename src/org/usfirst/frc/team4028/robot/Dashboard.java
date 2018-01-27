package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.*;

// This class contains code to interface with the Dashboard on the Driver's Station
//	We read values from 
//		- Sendable Choosers to control Auton
//		- FMS Data for real time game data
//  We write values to
//		- provide real-time info to the drive team
public class Dashboard {
	// singleton pattern
	private static Dashboard _instance = new Dashboard();
	
	public static Dashboard getInstance() {
		return _instance;
	}
	
	public enum AUTON_MODE {
		UNDEFINED,
		DO_NOTHING,
		AUTO_RUN,
		SWITCH,
		DOUBLE_SWITCH,
		TRIPLE_SWITCH,
		SCALE,
		DOUBLE_SCALE,
		SCALE_THEN_SWITCH
	}
	
	// Define all Dashboard Sendable Choosers (use generic types based on enums)
	private SendableChooser<AUTON_MODE> _autonModeChooser = new SendableChooser<>();
	
	private boolean _isSwitchLeft, _isScaleLeft;
	
	// private constructor for singleton pattern
	private Dashboard() {
		// setup Auton Mode Chooser
		_autonModeChooser.addDefault("Do Nothing", AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Auto Run", AUTON_MODE.AUTO_RUN);
		_autonModeChooser.addObject("Switch", AUTON_MODE.SWITCH);
		_autonModeChooser.addObject("Double Switch", AUTON_MODE.DOUBLE_SWITCH);
		_autonModeChooser.addObject("Triple Switch", AUTON_MODE.TRIPLE_SWITCH);
		_autonModeChooser.addObject("Scale", AUTON_MODE.SCALE);
		_autonModeChooser.addObject("Double Scale", AUTON_MODE.DOUBLE_SCALE);
		_autonModeChooser.addObject("Scale then Switch", AUTON_MODE.SCALE_THEN_SWITCH);
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
	}
	
	private boolean getIsFMSAttached() {
		return DriverStation.getInstance().isFMSAttached();
	}
	
	public void getGameData() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		_isSwitchLeft = (gameData.charAt(0) == 'L');
		_isScaleLeft = (gameData.charAt(1) == 'L');
	}
	
	public void printStartupMessage() {
		// This prints once during robotInit
		boolean isFMSAttached = getIsFMSAttached();
		
		DriverStation.reportWarning(">>>>> Is FMS Attached : [" + isFMSAttached + "] <<<<<<", false);
	}
	
	public AutonBase getSelectedAuton() {
		// Returns the autonBase object associated with the auton selected on the dashboard 
		switch(_autonModeChooser.getSelected()) {
			case DO_NOTHING:
				return new DoNothing();
			case AUTO_RUN:
				return new AutoRun();
			case SWITCH:
				return new Switch(_isSwitchLeft);
			case DOUBLE_SWITCH:
				return new DoubleSwitch(_isSwitchLeft);
			case TRIPLE_SWITCH:
				return new TripleSwitch(_isSwitchLeft);
			case SCALE:
				return new Scale(_isScaleLeft);
			case DOUBLE_SCALE:
				return new DoubleScale(_isScaleLeft);
			case SCALE_THEN_SWITCH:
				return new ScaleThenSwitch(_isSwitchLeft, _isScaleLeft);
			default:
				return new DoNothing();
		}
	}
}