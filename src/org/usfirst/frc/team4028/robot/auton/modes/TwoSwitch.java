package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.LeftSwitchPath;
import org.usfirst.frc.team4028.robot.paths.PathContainer;
import org.usfirst.frc.team4028.robot.paths.RightSwitchPath;
import org.usfirst.frc.team4028.robot.paths.RightSwitchtoFrontofPyramidPath;

public class TwoSwitch extends AutonBase {
	PathContainer toRightSwitch = new RightSwitchPath();
	PathContainer fromRightSwitchToFrontOfPyramidPath = new RightSwitchtoFrontofPyramidPath();
	
	@Override
	public void routine() {
		runAction(new ResetPoseFromPathAction(toRightSwitch));
		runAction(new RunMotionProfileAction(toRightSwitch));
	}
}
