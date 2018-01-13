package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInputs {
	private static SmartDashboardInputs _instance = new SmartDashboardInputs();
	public static SmartDashboardInputs getInstance() {
		return _instance;
	}
	
	// Enums
	private AUTON_MODE _autonModeChoice;
	private ALLIANCE_COLOR _allianceColor;
	
	// Choosers
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE_COLOR> _allianceChooser;
	
	private String _fmsDebugMsg = "?";
	
	private SmartDashboardInputs() {
		ConfigAutonModeChoosers();
	}
	
	private void ConfigAutonModeChoosers() {
		// Auton Mode Chooser
		_autonModeChooser = new SendableChooser<AUTON_MODE>();
		_autonModeChooser.addDefault("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addDefault("Auto Run", GeneralEnums.AUTON_MODE.AUTO_RUN);
		_autonModeChooser.addDefault("Switch", GeneralEnums.AUTON_MODE.SWITCH);
		_autonModeChooser.addDefault("Double Switch", GeneralEnums.AUTON_MODE.DOUBLE_SWITCH);
		
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
		
		// Alliance Color Chooser
		_allianceChooser = new SendableChooser<ALLIANCE_COLOR>();
		_allianceChooser.addDefault("FMS", GeneralEnums.ALLIANCE_COLOR.USE_FMS);
		_allianceChooser.addObject("Red Alliance", GeneralEnums.ALLIANCE_COLOR.RED_ALLIANCE);
		_allianceChooser.addObject("Blue Alliance", GeneralEnums.ALLIANCE_COLOR.BLUE_ALLIANCE);
		
		SmartDashboard.putData("Alliance Chooser" , _allianceChooser);		
	}
	
	public boolean getIsFMSAttached() {
		return DriverStation.getInstance().isFMSAttached();
	}
	
	public void printStartupMessage() {
		// This prints once during robotInit
		boolean isFMSAttached = getIsFMSAttached();
		ALLIANCE_COLOR allianceColor = _allianceChooser.getSelected();
		_fmsDebugMsg = "Is FMS Attached: [" + isFMSAttached + "] Alliance: [" + allianceColor + "]";
		DriverStation.reportWarning(">>>>> " + _fmsDebugMsg + " <<<<<<", false);
	}
	
	public AutonBase getSelectedAuton() {
		// Returns the autonBase object associated with the auton selected on the dashboard 
		_autonModeChoice = _autonModeChooser.getSelected();
		switch(_autonModeChoice) {
			case UNDEFINED:
				return new DoNothing();
			case AUTO_RUN:
				return new AutoRun();
			case SWITCH:
				return new Switch();
			case DOUBLE_SWITCH:
				return new TwoSwitch();
			default:
				return new DoNothing();
		}
	}
	
	public boolean getIsBlueAlliance() {
		_allianceColor = _allianceChooser.getSelected();
		
		switch (_allianceColor) {
		case BLUE_ALLIANCE:
			return true;
			
		case RED_ALLIANCE:
			return false;
			
		case USE_FMS:
			if(getIsFMSAttached()) {
				Alliance fmsAlliance = DriverStation.getInstance().getAlliance();
				
				switch(fmsAlliance) {
					case Blue:
						return true;
						
					case Red:
						return false;
						
					default:
						return true;	// force default of blue alliance
				}
			} else {
				return true;	// force default of blue alliance
			}
		
		default:
			return true;
		}
	}
}