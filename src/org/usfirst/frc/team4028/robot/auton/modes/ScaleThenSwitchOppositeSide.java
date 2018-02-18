package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchOppositeSide extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double turnTargetAngle;
	boolean isTurnRight;
	
	public ScaleThenSwitchOppositeSide(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE);
			fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH, 100,120,0.00);
			turnTargetAngle=150;
			isTurnRight=true;
			
		} 
		else {
			toScale = Paths.getPath(PATHS.R_SCALE);
			fromScaleToSwitch = Paths.getPath(PATHS.R_SCALE_TO_L_SWITCH,100,120,0.006);
			turnTargetAngle=-150;
			isTurnRight=false;
		}
	}
	
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toScale));
		runAction(new TurnAction(turnTargetAngle,true));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(fromScaleToSwitch),
				new SeriesAction(Arrays.asList(new Action[] {
						//new WaitAction(2),
						new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
				}))
	})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
				new TurnAction(180,isTurnRight),
				new WaitAction(0.5)
				})),
				new DriveInfeedWheelsAction(),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.5),
						new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE)
				}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(14),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}
