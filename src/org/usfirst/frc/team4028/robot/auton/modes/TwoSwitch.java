package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class TwoSwitch extends AutonBase {
	Path toRightSwitch = Paths.getPath(PATHS.R_SWITCH);
	Path fromRightSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
	Path toThePyramid = Paths.getPath(PATHS.TO_PYRAMID);
	Path AwayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_PYRAMID);
	Path InFrontOfPyramidToRightSwitch = Paths.getPath(PATHS.IN_FRONT_OF_PYRAMID_TO_RIGHT_SWITCH);
	
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
		runAction(new PrintTimeFromStart(_startTime));
	}
}