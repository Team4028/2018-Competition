package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.util.control.Path;

public class TokyoDriftTest extends AutonBase {
	Path TokyoDrift1 = Paths.getPath(PATHS.TOKYO_DRIFT_PT_1);
	Path TokyoDrift2 = Paths.getPath(PATHS.TOKYO_DRIFT_PT_2);
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(TokyoDrift1));
		runAction(new RunMotionProfileAction(TokyoDrift2));
		runAction(new PrintTimeFromStart(_startTime));
	}
}