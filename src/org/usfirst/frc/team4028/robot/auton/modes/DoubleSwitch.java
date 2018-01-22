package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class DoubleSwitch extends AutonBase {
	Path toRightSwitch = Paths.getPath(PATHS.R_SWITCH);
	Path fromRightSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
	Path toThePyramid = Paths.getPath(PATHS.TO_PYRAMID);
	//Path AwayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_PYRAMID);
	//Path InFrontOfPyramidToRightSwitch = Paths.getPath(PATHS.IN_FRONT_OF_PYRAMID_TO_RIGHT_SWITCH);
	Path STurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT);
	Path ToSwitch = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN);
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toRightSwitch));
		runAction(new RunMotionProfileAction(fromRightSwitchToFrontOfPyramidPath));
		runAction(new RunMotionProfileAction(toThePyramid));
		runAction(new RunMotionProfileAction(STurnAwayFromPyramid));
		runAction(new RunMotionProfileAction(ToSwitch));
		runAction(new PrintTimeFromStart(_startTime));
	}
}