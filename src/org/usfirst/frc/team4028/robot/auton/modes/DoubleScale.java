package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase{
	Path toScale = Paths.getPath(PATHS.L_SCALE);
	Path fromLScaleToLSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_1);
	Path fromLScaleToLSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_2);
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale));
		runAction(new RunMotionProfileAction(fromLScaleToLSwitchPt1));
		runAction(new RunMotionProfileAction(fromLScaleToLSwitchPt2));
		runAction(new PrintTimeFromStart(_startTime));
	}
}