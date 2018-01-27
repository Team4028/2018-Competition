package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitch extends AutonBase {
	Path toScale = Paths.getPath(PATHS.L_SCALE);
	Path fromScaleToSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_1);
	Path fromScaleToSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_2);
	
	public ScaleThenSwitch(boolean isLeftSwitch, boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE);
			
			if (isLeftSwitch) {
				fromScaleToSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_1);
				fromScaleToSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_2);
			} else {
				fromScaleToSwitchPt1 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_1);
				fromScaleToSwitchPt2 = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_2);
			}
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE);
			
			if (isLeftSwitch) {
				fromScaleToSwitchPt1 = Paths.getPath(PATHS.R_SCALE_TO_L_SWITCH_PT_1);
				fromScaleToSwitchPt2 = Paths.getPath(PATHS.R_SCALE_TO_L_SWITCH_PT_2);
			} else {
				fromScaleToSwitchPt1 = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_1);
				fromScaleToSwitchPt2 = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_2);
			}
		}
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale, true));
		runAction(new RunMotionProfileAction(fromScaleToSwitchPt1));
		runAction(new RunMotionProfileAction(fromScaleToSwitchPt2));
		runAction(new PrintTimeFromStart(_startTime));
	}
}