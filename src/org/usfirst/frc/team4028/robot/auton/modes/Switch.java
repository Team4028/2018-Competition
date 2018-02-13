package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.Action;
import org.usfirst.frc.team4028.robot.auton.actions.ParallelAction;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.SetInfeedPosAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class Switch extends AutonBase {
	Path toSwitch;
	
	public Switch(boolean isSwitchLeft) {
		if (isSwitchLeft) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 100.0, 120.0, 0.004); 
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 100.0, 120.0, 0.0065);
		}
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitch),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
					}
		)));
		runAction(new PrintTimeFromStart(_startTime));
	}
}