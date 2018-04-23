package org.usfirst.frc.team4028.robot.auton.modes.center;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;

public class AutoRun extends AutonBase {
	@Override
	public void routine() {
		runAction(new ArcadeDriveAction(0.8, 2.0));
		runAction(new PrintTimeFromStart(_startTime));
	}
}