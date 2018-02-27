package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.center.*;
import org.usfirst.frc.team4028.robot.auton.modes.side.*;

/**
 *  This class contains code to interface with the Dashboard on the Driver's Station
 *	We read values from 
 *		- Sendable Choosers to control Auton
 *		- FMS Data for real time game data
 *  We write values to
 *		- provide real-time info to the drive team
 */
public class Dashboard {
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
		TRIPLE_SCALE,
		SCALE_THEN_SWITCH,
		DOUBLE_SCALE_THEN_SWITCH
	}
	
	private SendableChooser<AUTON_MODE> _autonModeChooser = new SendableChooser<>();
	
	private boolean _isSwitchLeft, _isScaleLeft;
	
	private Dashboard() {
		_autonModeChooser.addDefault("Do Nothing", AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Auto Run", AUTON_MODE.AUTO_RUN);
		_autonModeChooser.addObject("Switch", AUTON_MODE.SWITCH);
		_autonModeChooser.addObject("Double Switch", AUTON_MODE.DOUBLE_SWITCH);
		_autonModeChooser.addObject("Triple Switch", AUTON_MODE.TRIPLE_SWITCH);
		_autonModeChooser.addObject("Scale", AUTON_MODE.SCALE);
		_autonModeChooser.addObject("Double Scale", AUTON_MODE.DOUBLE_SCALE);
		_autonModeChooser.addObject("Triple Scale", AUTON_MODE.TRIPLE_SCALE);
		_autonModeChooser.addObject("Scale then Switch", AUTON_MODE.SCALE_THEN_SWITCH);
		_autonModeChooser.addObject("Double Scale Then Switch", AUTON_MODE.DOUBLE_SCALE_THEN_SWITCH);
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
	}
	
	public boolean isGameDataReceived() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0) {
			_isSwitchLeft = (gameData.charAt(0) == 'L');
			_isScaleLeft = (gameData.charAt(1) == 'L');
			return true;
		} else {
			return false;
		}
	}
	
	/** This prints once during robotInit */
	public void printStartupMessage() {
		boolean isFMSAttached = DriverStation.getInstance().isFMSAttached();
		
		DriverStation.reportWarning(">>>>> Is FMS Attached : [" + isFMSAttached + "] <<<<<<", false);
	}
	
	/** Returns the autonBase object associated with the auton selected on the dashboard */
	public AutonBase getSelectedAuton() {
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
				if(_isScaleLeft == _isSwitchLeft) {
					return new ScaleThenSwitchSameSide(_isScaleLeft);
				} else {
					return new ScaleThenSwitchOppositeSide(_isScaleLeft);
				}
			case DOUBLE_SCALE_THEN_SWITCH:
				return new DoubleScaleAndSwitch(_isScaleLeft);
			case TRIPLE_SCALE:
				if (_isScaleLeft) {
					return new TripleScale();
				} else {
					return new DoubleScale(_isScaleLeft);
				}
			default:
				return new DoNothing();
		}
	}
}