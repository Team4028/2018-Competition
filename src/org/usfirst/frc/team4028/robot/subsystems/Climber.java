package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogDataBE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the climber Subsystem
//=====> For Changes see Nathen Worthington
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		NW			???			Initial Version
//	1		TomB		27.Feb		Add Run Cmd
//  2		TomB		27.Feb		Add 2 Speed Ctrl, Added Servo Support
//-------------------------------------------------------------
public class Climber implements Subsystem 
{
	// define class level working variables
	private TalonSRX _climberMotor; 
	private Servo _climberServo;
	private Elevator _elevator = Elevator.getInstance();
	
	private double _targetServoPosition = 0.0;
	private boolean _isClimberServoOpen = false;
	
	private static final double SERVO_OPEN_POSITION = 1;
	private static final double SERVO_CLOSED_POSITION = 0;
	
	public static final double CLIMBER_MOTOR_HIGH_VBUS = 1.0;
	public static final double CLIMBER_MOTOR_LOW_VBUS = 0.40;
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static Climber _instance = new Climber();
	
	public static Climber getInstance() {
		return _instance;
	}
	
	private Climber() { // private constructor for singleton pattern
		//====================================================================================
		//	config master & slave talon objects
		//====================================================================================
		_climberMotor = new TalonSRX(Constants.CLIMBER_CAN_ADDRESS);
		
		// set motor phasing
		_climberMotor.setInverted(false);
		
		// config limit switches
		_climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		// turn off all soft limits
		_climberMotor.configForwardSoftLimitEnable(false, 0);
		_climberMotor.configReverseSoftLimitEnable(false, 0);
		
		// config brake mode
		_climberMotor.setNeutralMode(NeutralMode.Brake);
	
		//Enable Current Limiting
		_climberMotor.enableCurrentLimit(false);
			
		//configure the peak and nominal output voltages in both directions for both Talons
		_climberMotor.configNominalOutputForward(0, 0);
		_climberMotor.configNominalOutputReverse(0, 0);
		_climberMotor.configPeakOutputForward(1, 0);
		_climberMotor.configPeakOutputReverse(-1, 0);
		
		// set motor mode
		_climberMotor.set(ControlMode.PercentOutput, 0, 0);
	
		// DisableSoftLimits
		_climberMotor.configReverseSoftLimitEnable(false, 0);
		_climberMotor.configForwardSoftLimitEnable(false, 0);
		
		// Setup Carriage Servo Motors
		_climberServo = new Servo(Constants.CLIMBER_SERVO_PWM_ADDRESS);
		_climberServo.set(_targetServoPosition);
		_isClimberServoOpen = false;
	}

	public void runMotor(double vbusCmd)
	{
		if(vbusCmd > 0.8) { // more than 1/2 way, climb @ high speed
			_climberMotor.set(ControlMode.PercentOutput, CLIMBER_MOTOR_HIGH_VBUS, 0);
		}
		else if(vbusCmd > 0.05)	{ // more than 1/2 way, climb @ low speed
			_climberMotor.set(ControlMode.PercentOutput, CLIMBER_MOTOR_LOW_VBUS, 0);
		}
		else {
			_climberMotor.set(ControlMode.PercentOutput, 0.0, 0);
		}
	}
	
	public void toggleClimberServo() {
		double servoCurrentPosition = _climberServo.get();
		if(servoCurrentPosition <= 0.5  && _elevator.isClimberServoEnabledHeight()) {
			_targetServoPosition = SERVO_OPEN_POSITION;
			_isClimberServoOpen = true;
		}
		else {
			_targetServoPosition = SERVO_CLOSED_POSITION;
			_isClimberServoOpen = false;
		}
		_climberServo.set(_targetServoPosition);
	}
	
	@Override
	public void stop() {
		_climberMotor.set(ControlMode.PercentOutput, 0, 0);
	}

	@Override
	public void zeroSensors() {}
	
	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	private double getClimberMotorCurrent()	{
		return _climberMotor.getOutputCurrent();
	}
	
	//=====================================================================================
	// Utility Methods
	//=====================================================================================
	@Override
	public void outputToShuffleboard() {
		SmartDashboard.putNumber("Climber:Current:", getClimberMotorCurrent());
		SmartDashboard.putBoolean("Is Climber Servo Open?:", _isClimberServoOpen);
	}

	@Override
	public void updateLogData(LogDataBE logData) {
		logData.AddData("Climber: Current", String.valueOf(getClimberMotorCurrent()));
	}
}