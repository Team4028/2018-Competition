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
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 120.0, 60.0);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 120.0, 60.0);
		}
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toSwitch));
		runAction(new PrintTimeFromStart(_startTime));
	}
}