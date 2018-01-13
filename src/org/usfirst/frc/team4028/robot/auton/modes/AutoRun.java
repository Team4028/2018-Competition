package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.AutoRunPath;
import org.usfirst.frc.team4028.robot.paths.PathContainer;

public class AutoRun extends AutonBase {
	PathContainer path = new AutoRunPath();
	
	@Override
	public void routine() {
		runAction(new ResetPoseFromPathAction(path));
		runAction(new RunMotionProfileAction(path));
	}
}