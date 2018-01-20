package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class TripleSwitch extends AutonBase {
	Path toRightSwitch = Paths.getPath(PATHS.R_SWITCH);
	Path fromRightSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
	Path toThePyramid = Paths.getPath(PATHS.TO_PYRAMID);
	Path AwayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_PYRAMID);
	Path InFrontOfPyramidToRightSwitch = Paths.getPath(PATHS.IN_FRONT_OF_PYRAMID_TO_RIGHT_SWITCH);
	Path AwayFromRightSwitch = Paths.getPath(PATHS.AWAY_FROM_RIGHT_SWITCH);
	Path ToRightSideofPyramid = Paths.getPath(PATHS.PYRAMID_FOR_SECOND_CUBE_FROM_RIGHT);
	Path AwayFromRightPyramid = Paths.getPath(PATHS.FROM_RIGHT_OF_PYRAMID_TO_FRONT_OF_SWITCH);
	Path FromFrontofRightSwitchtoSwitch = Paths.getPath(PATHS.FRONT_OF_R_SWITCH_TO_SWITCH);
	
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
		runAction(new ResetPoseFromPathAction(AwayFromRightSwitch));
		runAction(new RunMotionProfileAction(AwayFromRightSwitch));
		runAction(new ResetPoseFromPathAction(ToRightSideofPyramid));
		runAction(new RunMotionProfileAction(ToRightSideofPyramid));
		runAction(new ResetPoseFromPathAction(AwayFromRightPyramid));
		runAction(new RunMotionProfileAction(AwayFromRightPyramid));
		runAction(new ResetPoseFromPathAction(FromFrontofRightSwitchtoSwitch));
		runAction(new RunMotionProfileAction(FromFrontofRightSwitchtoSwitch));
		runAction(new PrintTimeFromStart(_startTime));
		
		/*
		System.out.println(toRightSwitch.getStartPose().toString());
		System.out.println(fromRightSwitchToFrontOfPyramidPath.getStartPose().toString());
		System.out.println(toThePyramid.getStartPose().toString());
		System.out.println(AwayFromPyramid.getStartPose().toString());
		System.out.println(InFrontOfPyramidToRightSwitch.getStartPose().toString());
		System.out.println(AwayFromRightSwitch.getStartPose().toString());
		System.out.println(ToRightSideofPyramid.getStartPose().toString());
		System.out.println(AwayFromRightPyramid.getStartPose().toString());
		System.out.println(FromFrontofRightSwitchtoSwitch.getStartPose().toString());
		*/
	}
}
