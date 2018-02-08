package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class Scale extends AutonBase{
	Path toScale;
	
	public Scale(boolean isScaleLeft) {
		if (isScaleLeft) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0.00175);
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, -0.00575);
		}
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale));
		runAction(new PrintTimeFromStart(_startTime));
	}
}