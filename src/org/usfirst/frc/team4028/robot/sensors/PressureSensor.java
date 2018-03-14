package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PressureSensor {
	private static PressureSensor _instance = new PressureSensor();
	
	public static PressureSensor getInstance() {
		return _instance;
	}
	
	private AnalogInput _pressureSensor = new AnalogInput(0);
	
	private PressureSensor() {}
	
	public double getPressure() {
		double voltage = _pressureSensor.getAverageVoltage();
		double pressure = 250*(voltage/4.9) -25;
		return GeneralUtilities.RoundDouble(pressure, 2);
	}
	
	public void outputToShuffleboard() {
		SmartDashboard.putNumber("Analog Pressure", getPressure());
	}

	public void updateLogData(LogDataBE logData) {
		SmartDashboard.putNumber("Analog Pressure", getPressure());
	}
}
