package org.usfirst.frc.team4028.robot.auton.modes.center;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.AdaptedPaths;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

public class DoNothing extends AutonBase {
	@Override
	public void routine() {
		//AdaptedPaths.locateFlavorTownUSA();
		/*(
		runAction(new InfeedCubeAction());
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
				new OutfeedCubeAction(),
				new WaitAction(5)
		})));
		*/
	}
}