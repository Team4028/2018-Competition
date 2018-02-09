package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class Switch extends AutonBase {
	Path toSwitch;
	
	public Switch(boolean isSwitchLeft) {
		if (isSwitchLeft) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 90.0, 90.0,-0.01);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 90.0, 90.0,-0.0085);
		}
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toSwitch));
		runAction(new PrintTimeFromStart(_startTime));
	}
}