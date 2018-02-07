package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class AutoRun extends AutonBase {
	Path path = Paths.getPath(PATHS.AUTO_RUN, 120.0, 120.0);
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(path, false));
		runAction(new PrintTimeFromStart(_startTime));
	}
}