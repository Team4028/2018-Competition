package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitch extends AutonBase {
	Path toScale = Paths.getPath(PATHS.L_SCALE);
	Path fromLScaleToRSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_1);
	Path fromLScaleToRSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_2);
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale, true));
		runAction(new RunMotionProfileAction(fromLScaleToRSwitchPt1));
		runAction(new RunMotionProfileAction(fromLScaleToRSwitchPt2));
		runAction(new PrintTimeFromStart(_startTime));
	}
}