package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class TripleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toThePyramid, sTurnAwayFromPyramid, toSwitchAfterSTurn; // Cube 2
	Path awayFromSwitch, toSideOfPyramid, awayFromPyramid, fromFrontofSwitchtoSwitch; // Cube 3
	
	public TripleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_L_SWITCH_AFTER_S_TURN);
			awayFromSwitch = Paths.getPath(PATHS.AWAY_FROM_LEFT_SWITCH);
			toSideOfPyramid = Paths.getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT);
			awayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_L_PYRAMID);
			fromFrontofSwitchtoSwitch = Paths.getPath(PATHS.TO_L_SWITCH_WITH_CUBE_3);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN);
			awayFromSwitch = Paths.getPath(PATHS.AWAY_FROM_RIGHT_SWITCH);
			toSideOfPyramid = Paths.getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT);
			awayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_R_PYRAMID);
			fromFrontofSwitchtoSwitch = Paths.getPath(PATHS.TO_R_SWITCH_WITH_CUBE_3);
		}
		toThePyramid = Paths.getPath(PATHS.TO_PYRAMID);
	}
	
	@Override
	public void routine() {	
		runAction(new RunMotionProfileAction(toSwitch));
		runAction(new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath));
		runAction(new RunMotionProfileAction(toThePyramid));
		runAction(new RunMotionProfileAction(sTurnAwayFromPyramid));
		runAction(new RunMotionProfileAction(toSwitchAfterSTurn));
		runAction(new RunMotionProfileAction(awayFromSwitch));
		runAction(new RunMotionProfileAction(toSideOfPyramid));
		runAction(new RunMotionProfileAction(awayFromPyramid));
		runAction(new RunMotionProfileAction(fromFrontofSwitchtoSwitch));
		runAction(new PrintTimeFromStart(_startTime));
	}
}