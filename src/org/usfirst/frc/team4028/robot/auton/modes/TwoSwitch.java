package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.AwayFromPyramidPath;
import org.usfirst.frc.team4028.robot.paths.InFrontOfPyramidtoRightSwitch;
import org.usfirst.frc.team4028.robot.paths.LeftSwitchPath;
import java.util.concurrent.*;
import org.usfirst.frc.team4028.robot.paths.PathContainer;
import org.usfirst.frc.team4028.robot.paths.RightSwitchPath;
import org.usfirst.frc.team4028.robot.paths.RightSwitchtoFrontofPyramidPath;
import org.usfirst.frc.team4028.robot.paths.ToThePyramidPath;

public class TwoSwitch extends AutonBase {
	PathContainer toRightSwitch = new RightSwitchPath();
	PathContainer fromRightSwitchToFrontOfPyramidPath = new RightSwitchtoFrontofPyramidPath();
	PathContainer toThePyramid = new ToThePyramidPath();
	PathContainer AwayFromPyramid = new AwayFromPyramidPath();
	PathContainer InFrontOfPyramidToRightSwitch = new InFrontOfPyramidtoRightSwitch();
	
	@Override
	public void routine() {
		runAction(new ResetPoseFromPathAction(toRightSwitch));
		runAction(new RunMotionProfileAction(toRightSwitch));
		runAction(new ResetPoseFromPathAction(fromRightSwitchToFrontOfPyramidPath));
		runAction(new RunMotionProfileAction(fromRightSwitchToFrontOfPyramidPath));
		runAction(new ResetPoseFromPathAction(toThePyramid));
		runAction(new RunMotionProfileAction(toThePyramid));
		runAction(new ResetPoseFromPathAction(AwayFromPyramid));
		runAction(new RunMotionProfileAction(AwayFromPyramid));
		runAction(new ResetPoseFromPathAction(InFrontOfPyramidToRightSwitch));
		runAction(new RunMotionProfileAction(InFrontOfPyramidToRightSwitch));
	}
}
