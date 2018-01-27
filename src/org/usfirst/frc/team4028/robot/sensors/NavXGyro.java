package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;

/* This class encapsulates interactions with the NavX Sensor
   setup path to libraries using these instructions:
   http://www.pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
   http://www.pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/ */
public class NavXGyro {
	// singleton pattern
	private static NavXGyro _instance = new NavXGyro();
	
	public static NavXGyro getInstance() {
		return _instance;
	}
	
	private AHRS _navXSensor;
	private double _angleAdjustment;
	
	// private constructor for singleton pattern
	private NavXGyro() {
        try {          
        	_navXSensor = new AHRS(Constants.NAVX_PORT); // Communication via RoboRIO MXP (SPI) 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
	}
	
	public double getYaw() { 
		return _angleAdjustment + _navXSensor.getYaw(); 
	}
	
	public void zeroYaw() { 
		_navXSensor.zeroYaw(); 
	}
	
	public boolean isPitchPastThreshhold() {
		if(_navXSensor.getPitch()>=Constants.MAX_PITCH_POSITIVE || _navXSensor.getPitch()<= Constants.MAX_PITCH_NEGATIVE) {
			return true;
		} else {
			return false;
		}
	}
	
	public void setAngleAdjustment(double angle) {
		_angleAdjustment = angle;
	}
}