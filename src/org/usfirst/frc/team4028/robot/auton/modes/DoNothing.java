package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.paths.AdaptedPaths;

public class DoNothing extends AutonBase {
	@Override
	public void routine() {
		AdaptedPaths.locateFlavorTownUSA();
	}
}