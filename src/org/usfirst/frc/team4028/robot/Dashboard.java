package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4028.robot.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.*;

public class Dashboard {
	private static Dashboard _instance = new Dashboard();
	
	public static Dashboard getInstance() {
		return _instance;
	}
	
	// Define all Dashboard Sendable Choosers (use generic types based on enums)
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	
	// private constructor for singleton pattern
	private Dashboard() {
		// Auton Mode Chooser
		_autonModeChooser.addDefault("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Auto Run", GeneralEnums.AUTON_MODE.AUTO_RUN);
		_autonModeChooser.addObject("Switch", GeneralEnums.AUTON_MODE.SWITCH);
		_autonModeChooser.addObject("Double Switch", GeneralEnums.AUTON_MODE.DOUBLE_SWITCH);
		_autonModeChooser.addObject("Triple Switch", GeneralEnums.AUTON_MODE.TRIPLE_SWITCH);
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
	}
	
	public boolean getIsFMSAttached() {
		return DriverStation.getInstance().isFMSAttached();
	}
	
	public void printStartupMessage() {
		// This prints once during robotInit
		boolean isFMSAttached = getIsFMSAttached();
		String _fmsDebugMsg = "?";
		
		_fmsDebugMsg = "Is FMS Attached: [" + isFMSAttached + "]";
		
		DriverStation.reportWarning(">>>>> " + _fmsDebugMsg + " <<<<<<", false);
	}
	
	public AutonBase getSelectedAuton() {
		// Returns the autonBase object associated with the auton selected on the dashboard 
		switch(_autonModeChooser.getSelected()) {
			case UNDEFINED:
				return new DoNothing();
			case DO_NOTHING:
				return new DoNothing();
			case AUTO_RUN:
				return new AutoRun();
			case SWITCH:
				return new Switch();
			case DOUBLE_SWITCH:
				return new DoubleSwitch();
			case TRIPLE_SWITCH:
				return new TripleSwitch();
			default:
				return new DoNothing();
		}
	}
}