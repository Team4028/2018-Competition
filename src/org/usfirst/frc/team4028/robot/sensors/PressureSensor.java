package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import org.usfirst.frc.team4028.util.LogDataBE;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the onboard pressure sensor
//=====> For Changes see Patrick Bruns
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		Prat		???			Initial Version
//	1		TomB		17.Mar		Code Cleanup, added comments
//-------------------------------------------------------------
public class PressureSensor {
	// define class level working variables
	private AnalogInput _pressureSensor;
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static PressureSensor _instance = new PressureSensor();
	
	public static PressureSensor getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private PressureSensor() {
		_pressureSensor = new AnalogInput(Constants.STORED_PRESSURE_SENSOR_AIO_PORT);
	}
	
	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	public double getPressure() {
		double voltage = _pressureSensor.getAverageVoltage();
		double pressure = 250*(voltage/4.9) -25;
		return GeneralUtilities.RoundDouble(pressure, 2);
	}
	
	//=====================================================================================
	// Utility Methods
	//=====================================================================================
	public void outputToShuffleboard() {
		SmartDashboard.putNumber("Stored Pressure (PSI)", getPressure());
	}

	public void updateLogData(LogDataBE logData) {
		logData.AddData("Stored Pressure (PSI)", String.valueOf(getPressure()));
	}
}
