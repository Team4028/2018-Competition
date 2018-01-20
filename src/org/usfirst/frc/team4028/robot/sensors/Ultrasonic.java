package org.usfirst.frc.team4028.robot.sensors;

import java.text.DecimalFormat;

import org.usfirst.frc.team4028.robot.Constants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ultrasonic {
		
	private static Ultrasonic _instance = new Ultrasonic();
	
	public static Ultrasonic getInstance() {
		return _instance;
	}
	
	AnalogInput ultrasonicSensor;
	
	private double _ultrasonicSensorOutputVoltage;
	private double _distanceReadInInches;
	private double _distanceReadInMeters;
	
	boolean _isCubeInRobot = false;
	
	static final double VOLTS_PER_INCH = 6.7138665; //0.0098;
	static final double INCHES_PER_METER = 39.3700787;
	
	static final double MINIMUM_DISTANCE_FOR_CUBE = 10;
	
	private Ultrasonic() {
		ultrasonicSensor = new AnalogInput(Constants.ULTRASONIC_PORT);
	}
		
	public void calculateDistanceReadings() {
			
		_ultrasonicSensorOutputVoltage = (ultrasonicSensor.getVoltage()*1000); // puts value in mV
		//System.out.println("Voltage Read:" + ultrasonicSensorOutputVoltage);
		//System.out.println("Value: " + ultrasonicSensorOutputVoltage/16);
		_distanceReadInInches = _ultrasonicSensorOutputVoltage/VOLTS_PER_INCH;
		_distanceReadInMeters = _distanceReadInInches/INCHES_PER_METER;
		
		if (_distanceReadInInches <= MINIMUM_DISTANCE_FOR_CUBE) {
			_isCubeInRobot = true; 
		} else {
			_isCubeInRobot = false;
		}
		
		DecimalFormat df=new DecimalFormat("0.00");
		String formate = df.format(_distanceReadInInches);
		_distanceReadInInches = Double.parseDouble(formate) ;
			
		String formate1 = df.format(_distanceReadInMeters);
		_distanceReadInMeters = Double.parseDouble(formate1) ;
		
		String formate2 = df.format(_ultrasonicSensorOutputVoltage);
		_ultrasonicSensorOutputVoltage = Double.parseDouble(formate2) ;
			
		//System.out.println("Inches: " + finalValue + "      Meters: " + finalValue1);
	}
	
	public boolean getIsCubeInRange() {
		return _isCubeInRobot;
	}
	
	public void outputToDashboard() {
		SmartDashboard.putBoolean("Is The Cube In The Robot?", _isCubeInRobot);
		SmartDashboard.putNumber("Distance Read in Inches:", _distanceReadInInches);
		SmartDashboard.putNumber("Distance Read In Meters:", _distanceReadInMeters);
		SmartDashboard.putNumber("Voltage [mV]:", _ultrasonicSensorOutputVoltage);
	}
}
