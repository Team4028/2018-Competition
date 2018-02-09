package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;

public class DoubleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toThePyramid, sTurnAwayFromPyramid, toSwitchAfterSTurn; // Cube 2
	
	public DoubleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 120.0, 90.0);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID, 90.0, 70.0);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT, 90.0, 120.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_L_SWITCH_AFTER_S_TURN, 120.0, 120.0);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 90.0, 90.0);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID, 90.0, 90.0);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT, 90.0, 90.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN, 100.0, 100.0);
		}
		toThePyramid = Paths.getPath(PATHS.TO_PYRAMID, 100.0, 100.0);
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toSwitch));
		runAction(new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath));
		runAction(new RunMotionProfileAction(toThePyramid));
		runAction(new RunMotionProfileAction(sTurnAwayFromPyramid));
		runAction(new RunMotionProfileAction(toSwitchAfterSTurn));
		runAction(new PrintTimeFromStart(_startTime));
	}
}