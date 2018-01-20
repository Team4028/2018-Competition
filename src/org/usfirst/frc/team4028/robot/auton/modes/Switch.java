package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class Switch extends AutonBase {
	Path path = Paths.getPath(PATHS.L_SWITCH);
	
	@Override
	public void routine() {
		runAction(new ResetPoseFromPathAction(path));
		runAction(new RunMotionProfileAction(path));
		runAction(new PrintTimeFromStart(_startTime));
	}
}
