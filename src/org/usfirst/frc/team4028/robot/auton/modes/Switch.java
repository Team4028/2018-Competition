package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.LeftSwitchPath;
import org.usfirst.frc.team4028.robot.paths.PathContainer;

public class Switch extends AutonBase {
	PathContainer path = new LeftSwitchPath();
	
	@Override
	public void routine() {
		runAction(new ResetPoseFromPathAction(path));
		runAction(new RunMotionProfileAction(path));
	}
}
