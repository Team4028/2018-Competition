package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase {
	Path toScale;
	Path fromScaleToSwitchPt1, fromScaleToSwitchPt2, fromSwitchToScalePt1, fromSwitchToScalePt2;
	
	public DoubleScale(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0.00175);
			fromScaleToSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_1, 100, 120, -0.0075);
			fromScaleToSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_2,100, 120, -0.004);
			//fromSwitchToScalePt1 = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE_PT_1, 100, 120);
			//fromSwitchToScalePt2 = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE_PT_2,100, 120);
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, -0.00575);
			fromScaleToSwitchPt1 = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_1, 100, 120);
			fromScaleToSwitchPt2 = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_2,100,120);
			fromSwitchToScalePt1 = Paths.getPath(PATHS.R_SWITCH_TO_R_SCALE_PT_1,100,120);
			fromSwitchToScalePt2 = Paths.getPath(PATHS.R_SWITCH_TO_R_SCALE_PT_2, 100, 120);
		}
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale, true));
		runAction(new RunMotionProfileAction(fromScaleToSwitchPt1));
		runAction(new RunMotionProfileAction(fromScaleToSwitchPt2));
		//runAction(new RunMotionProfileAction(fromSwitchToScalePt1));
		//runAction(new RunMotionProfileAction(fromSwitchToScalePt2));
		runAction(new PrintTimeFromStart(_startTime));
	}
}