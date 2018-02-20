package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.LogDataBE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber implements Subsystem {
	private static Climber _instance = new Climber();
	
	private TalonSRX _climbMotor;
	
	public static Climber getInstance() {
		return _instance;
	}
	
	private Climber() {
		_climbMotor = new TalonSRX(Constants.CLIMBER_CAN_ADDRESS);
	}
	
	public void ClimbOn(double climb) {
		_climbMotor.set(ControlMode.PercentOutput, climb);
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void outputToShuffleboard() {
	}

	@Override
	public void updateLogData(LogDataBE logData) {
	}
}