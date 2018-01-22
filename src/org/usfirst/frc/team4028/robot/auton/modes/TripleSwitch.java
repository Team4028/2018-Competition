package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class TripleSwitch extends AutonBase {
	Path toRightSwitch = Paths.getPath(PATHS.R_SWITCH);
	Path fromRightSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
	Path toThePyramid = Paths.getPath(PATHS.TO_PYRAMID);
	Path STurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT);
	Path ToSwitch = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN);
	Path AwayFromRightSwitch = Paths.getPath(PATHS.AWAY_FROM_RIGHT_SWITCH);
	Path ToRightSideofPyramid = Paths.getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT);
	Path AwayFromRightPyramid = Paths.getPath(PATHS.AWAY_FROM_R_PYRAMID);
	Path FromFrontofRightSwitchtoSwitch = Paths.getPath(PATHS.TO_R_SWITCH_WITH_CUBE_3);
	
	@Override
	public void routine() {	
		runAction(new RunMotionProfileAction(toRightSwitch));
		runAction(new RunMotionProfileAction(fromRightSwitchToFrontOfPyramidPath));
		runAction(new RunMotionProfileAction(toThePyramid));
		runAction(new RunMotionProfileAction(STurnAwayFromPyramid));
		runAction(new RunMotionProfileAction(ToSwitch));
		runAction(new RunMotionProfileAction(AwayFromRightSwitch));
		runAction(new RunMotionProfileAction(ToRightSideofPyramid));
		runAction(new RunMotionProfileAction(AwayFromRightPyramid));
		runAction(new RunMotionProfileAction(FromFrontofRightSwitchtoSwitch));
		runAction(new PrintTimeFromStart(_startTime));
	}
}