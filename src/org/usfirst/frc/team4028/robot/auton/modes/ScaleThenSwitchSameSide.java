package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchSameSide extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double targetTurnAngle;
	
	public ScaleThenSwitchSameSide(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE);
			fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100, 120);
			targetTurnAngle = 136.27;
		} 
		else {
			toScale = Paths.getPath(PATHS.R_SCALE);
			fromScaleToSwitch = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH);
			targetTurnAngle = -160;
		}
	}
	
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
	})));
	runAction(new TurnAction(targetTurnAngle, true));
	runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(fromScaleToSwitch),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.65),
						new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
				}))
	})));
	runAction(new ParallelAction(Arrays.asList(new Action[] {
				new WaitAction(0.65),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
				new DriveInfeedWheelsAction()
	})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(8),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}