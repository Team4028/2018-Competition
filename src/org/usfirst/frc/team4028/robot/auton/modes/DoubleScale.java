package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.TurnAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale;
	
	public DoubleScale(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0.00175);
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, -0.00575);
		}
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale, true));
		runAction(new RunMotionProfileAction(fromScaleToSwitch));
		runAction(new TurnAction(160.0));
		runAction(new WaitAction(0.5));
		runAction(new TurnAction(0.0));
		runAction(new RunMotionProfileAction(fromSwitchToScale));
		runAction(new PrintTimeFromStart(_startTime));
	}
}