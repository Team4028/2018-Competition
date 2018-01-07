package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.util.LatchedBoolean;
import org.usfirst.frc.team4028.util.LogitechF310;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
	private static ControlBoard _instance = new ControlBoard();
	public static ControlBoard getInstance() {
		return _instance;
	}
	
	private final Joystick _driverGamepad;
	private final Joystick _operatorGamepad;
	
	// Latched Boolean Buttons (to check when a button is just pressed)
	private LatchedBoolean _shiftGearBtn = new LatchedBoolean();
	
	private ControlBoard() {
		_driverGamepad = new Joystick(0);
		_operatorGamepad = new Joystick(1);
	}
	
	/* Axis Inputs */
	// Driver
	public double getThrottleCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
	}
	
	public double getTurnCmd() {
		return _driverGamepad.getRawAxis(LogitechF310.RIGHT_X_AXIS);
	}
	
	/* Button Inputs */
	// Driver
	public boolean getIsShiftGearJustPressed() {
		return _shiftGearBtn.isJustPressed(_driverGamepad.getRawButton(LogitechF310.START_BUTTON));
	}
}