package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.GeneralUtilities;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This class encapsulates interactions with a ultrasonic (distance) sensor plugging into
//	a Analog Input Port. (the sensor output a analog voltage proportional to the distance
//	the object is away from the front of the sensor
// Note there is approx a 6 inch deadband in front of the sensor wher readings are unuseable
public class UltrasonicSensor {
	
	// singleton pattern
	private static UltrasonicSensor _instance = new UltrasonicSensor();
	public static UltrasonicSensor getInstance() {
		return _instance;
	}
	
	// class level working variables
	AnalogInput _ultrasonicSensor;
	
	private double _ultrasonicSensorOutputVoltage;
	private double _ultrasonicSensorOutputVoltageAVG;
	private double _distanceReadInInches;
	private double _distanceReadInMeters;
	
	boolean _isCubeInRobot = false;
	boolean _isCubeInRange = false;
	
	// constants determined from bench testing
	static final double VOLTS_PER_INCH = 0.0233; //0.0061742804; //0.0200271621; //0.0098;
	static final double INCHES_PER_METER = 39.3700787;
	
	static final double MINIMUM_DISTANCE_FOR_CUBE_IN_ROBOT = 10;
	static final double MINIMUM_DISTANCE_TO_GRAB_CUBE = 40;
	
	// private constructor for singleton pattern
	private UltrasonicSensor() {
		_ultrasonicSensor = new AnalogInput(Constants.ULTRASONIC_PORT);
	}
		
	public void calculateDistanceReadings() 
	{
		// convert value from mV to V
		_ultrasonicSensorOutputVoltageAVG = (_ultrasonicSensor.getAverageVoltage()); 
		_ultrasonicSensorOutputVoltage = (_ultrasonicSensor.getVoltage()); 
		//System.out.println("Voltage Read:" + ultrasonicSensorOutputVoltage);
		//System.out.println("Value: " + ultrasonicSensorOutputVoltage/16);
		
		// convert analog voltage to distance
		_distanceReadInInches = _ultrasonicSensorOutputVoltage/VOLTS_PER_INCH;
		_distanceReadInMeters = _distanceReadInInches/INCHES_PER_METER;
				
		// 
		//DecimalFormat df=new DecimalFormat("0.00");
		//String formate = df.format(_distanceReadInInches);
		//_distanceReadInInches = Double.parseDouble(formate) ;
		_distanceReadInInches = GeneralUtilities.RoundDouble(_distanceReadInInches, 2);
		
		//String formate1 = df.format(_distanceReadInMeters);
		//_distanceReadInMeters = Double.parseDouble(formate1) ;
		_distanceReadInMeters = GeneralUtilities.RoundDouble(_distanceReadInMeters, 2);
		
		//String formate2 = df.format(_ultrasonicSensorOutputVoltage);
		//_ultrasonicSensorOutputVoltage = Double.parseDouble(formate2) ;
		_ultrasonicSensorOutputVoltage = GeneralUtilities.RoundDouble(_ultrasonicSensorOutputVoltage, 3);
		
		//System.out.println("Inches: " + finalValue + "      Meters: " + finalValue1);
	}
	
	public boolean getIsCubeInRobot() {
		calculateDistanceReadings();
		if (_distanceReadInInches <= MINIMUM_DISTANCE_FOR_CUBE_IN_ROBOT) {
			_isCubeInRobot = true; 
		} else {
			_isCubeInRobot = false;
		}
		return _isCubeInRobot;
	}
	
	public boolean getIsCubeInRange() {
		calculateDistanceReadings();
		if (_distanceReadInInches <= MINIMUM_DISTANCE_TO_GRAB_CUBE) {
			_isCubeInRange = true; 
		} else {
			_isCubeInRange = false;
		}
		return _isCubeInRange;	
	}
	
	public void outputToDashboard() {
		SmartDashboard.putBoolean("Is The Cube In The Robot?", _isCubeInRobot);
		SmartDashboard.putNumber("Cube Distance in Inches:", _distanceReadInInches);
		SmartDashboard.putNumber("Cube Distance In Meters:", _distanceReadInMeters);
		SmartDashboard.putNumber("Voltage AVG [V]:", _ultrasonicSensorOutputVoltageAVG);
		SmartDashboard.putNumber("Voltage [V]", _ultrasonicSensorOutputVoltage);
	}
}
