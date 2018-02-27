package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogDataBE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the climber Subsystem
//=====> For Changes see Nathen Worthington
//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		NW			???			Initial Version
//	1		TomB		27.Feb		Add Run Cmd
//-------------------------------------------------------------
public class Climber implements Subsystem 
{
	// define class level working variables
	private TalonSRX _climberMotor; 
	
	//=====================================================================================
	//Define Singleton Pattern
	//=====================================================================================
	private static Climber _instance = new Climber();
	
	public static Climber getInstance() 
	{
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Climber() 
	{
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
		//_climberMotor.configPeakCurrentDuration(200, 0);
		//_climberMotor.configPeakCurrentLimit(17, 0);
			
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
	}

	public void runMotor(double vbusCmd)
	{
		_climberMotor.set(ControlMode.PercentOutput, vbusCmd, 0);		
	}
	
	@Override
	public void stop() 
	{
		_climberMotor.set(ControlMode.PercentOutput, 0, 0);
	}

	@Override
	public void zeroSensors()
	{
		// N/A on this subsystem
	}
	
	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	private double getClimberMotorCurrent()
	{
		return _climberMotor.getOutputCurrent();
	}
	
	//=====================================================================================
	// Utility Methods
	//=====================================================================================
	@Override
	public void outputToShuffleboard() 
	{
		SmartDashboard.putNumber("Climber:Current:", getClimberMotorCurrent());
	}

	@Override
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Climber:Current", String.valueOf(getClimberMotorCurrent()));
	}
}