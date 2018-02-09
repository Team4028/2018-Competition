package org.usfirst.frc.team4028.robot.sensors;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDPMonitor {
	PowerDistributionPanel _pdp;
	//Singleton Pattern
	private static PDPMonitor _instance = new PDPMonitor();
	public static PDPMonitor getInstance() {
		return _instance;
	}
	
	//Private Constructor to enforce Singleton
	private PDPMonitor() {
		_pdp = new PowerDistributionPanel();
	}
	
	public double MonitorTotalCurrent() {
		 return _pdp.getTotalCurrent();
	}
	
	public double MonitorTotalEnergy() {
		return _pdp.getTotalEnergy();
	}
	
	public double MonitorTotalPower() {
		return _pdp.getTotalPower();
	}	
	
	public double MonitorVoltage() {
		return _pdp.getVoltage();
	}
	

	public void outputToSmartDashboard() {
		/*
		SmartDashboard.putNumber("Chassis Current: ", MonitorTotalCurrent());
		SmartDashboard.putNumber("Chassis Energy:", MonitorTotalEnergy());
		SmartDashboard.putNumber("Chassis Power:", MonitorTotalPower());
		SmartDashboard.putNumber("Chassis Voltage:", MonitorVoltage());
		*/
	}
}