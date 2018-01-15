package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4028.robot.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.modes.*;

//This class encapsulates all the interaction with the Operator Station Dashboard
//	for 2018 season this will probably be SuffleBoard instead of Smart DashBoard
//	(FYI: this driver's station client side decision should have no impact on this server side code)
//	https://wpilib.screenstepslive.com/s/currentCS/m/shuffleboard
public class DashboardInputs 
{
	// singleton pattern{
	private static DashboardInputs _instance = new DashboardInputs();
	
	public static DashboardInputs getInstance() {
		return _instance;
	}
	
	// Define all Dashboard Sendable Choosers (use generic types based on enums)
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE_COLOR> _allianceChooser;
	
	private String _fmsDebugMsg = "?";
	
	// private constructor for singleton pattern
	private DashboardInputs() 
	{
		// configure all the sendable choosers
		
		// Auton Mode Chooser
		_autonModeChooser = new SendableChooser<AUTON_MODE>();
		_autonModeChooser.addDefault("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
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
		switch(_autonModeChooser.getSelected()) {
			case UNDEFINED:
				return new DoNothing();
			default:
				return new DoNothing();
		}
	}
	
	public boolean getIsBlueAlliance() {		
		switch ( _allianceChooser.getSelected()) {
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