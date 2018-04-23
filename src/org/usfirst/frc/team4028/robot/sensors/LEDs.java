package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.DriverOperatorStation;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.Subsystem;
import org.usfirst.frc.team4028.util.LogDataBE;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs implements Subsystem{
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static LEDs _instance = new LEDs();
	
	public static LEDs getInstance() {
		return _instance;
	}
	
	//=====================================================================================
	//Define Class Level Objects
	//=====================================================================================
//	private DigitalOutput _arduinoPort1 = new DigitalOutput(1);
//	private DigitalOutput _arduinoPort2 = new DigitalOutput(2);
//	private DigitalOutput _arduinoPort3 = new DigitalOutput(3);
//	private DigitalOutput _arduinoPort4 = new DigitalOutput(4);
//	private DigitalOutput _arduinoPort5 = new DigitalOutput(5);
//	private DigitalOutput _arduinoPort6 = new DigitalOutput(6);
	private DigitalOutput _arduinoPort7 = new DigitalOutput(Constants.ARDUINO_DIO_PORT_SEVEN);
	private DigitalOutput _arduinoPort8 = new DigitalOutput(Constants.ARDUINO_DIO_PORT_EIGHT);
	private DigitalOutput _arduinoPort9 = new DigitalOutput(Constants.ARDUINO_DIO_PORT_NINE);

		//=================================================================================
		//Corresponding Pins on Arduino: 7 = 11; 8 = 12; 9 = 10
		//=================================================================================
		

	private Climber _climber = Climber.getInstance();
	private DriverOperatorStation _dos = DriverOperatorStation.getInstance();
	private CubeHandler _cubeHandler = CubeHandler.getInstance();
	
	//=====================================================================================
	//Private Constructor
	//=====================================================================================
	private LEDs() {
		_arduinoPort7.set(false);
		_arduinoPort8.set(false);
		_arduinoPort9.set(false);
	}
	
	//=====================================================================================
	//Operational Method
	//=====================================================================================
	public void commandLEDToLight() {
		if(_climber.isClimberServoOpen()) {
			_arduinoPort7.set(true);
		} else {
			_arduinoPort7.set(false);
		}
		
		if(_dos.getIsDriver_Strobe_BtnPressed()) {
			_arduinoPort8.set(true);
			_arduinoPort9.set(true);
		}
		else if(_cubeHandler.isCubeInCarriage()) {
			_arduinoPort8.set(false);
			_arduinoPort9.set(true);
		} 
		else if(_cubeHandler.areInfeedArmsInfeeding()) {
			_arduinoPort8.set(true);
			_arduinoPort9.set(false);
		} else {
			_arduinoPort8.set(false);
			_arduinoPort9.set(false);
		}
	}

	//=====================================================================================
	//Required Subsystem Methods (Not applicable)
	//=====================================================================================
	@Override
	public void stop() {}

	@Override
	public void zeroSensors() {}

	@Override
	public void outputToShuffleboard() {
		SmartDashboard.putBoolean("LED", _arduinoPort7.get());
		SmartDashboard.putBoolean("LEDButton", _dos.getIsDriver_Strobe_BtnPressed());
	}

	@Override
	public void updateLogData(LogDataBE logData) {}
}

