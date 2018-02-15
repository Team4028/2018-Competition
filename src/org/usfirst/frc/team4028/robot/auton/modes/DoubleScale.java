package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.Action;
import org.usfirst.frc.team4028.robot.auton.actions.DriveInfeedWheelsAction;
import org.usfirst.frc.team4028.robot.auton.actions.DriveSetDistanceAction;
import org.usfirst.frc.team4028.robot.auton.actions.ParallelAction;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.SeriesAction;
import org.usfirst.frc.team4028.robot.auton.actions.SetInfeedPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.TurnAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale;
	double targetTurnAngle;
	
	public DoubleScale(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0); // 0.0065
			targetTurnAngle = 151.1;
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, 0.0065);
			targetTurnAngle = -160;
		}
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0, 0.001);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new TurnAction(targetTurnAngle));		
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitch),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.65),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new RunMotionProfileAction(fromSwitchToScale)
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction()
		}))); 
		runAction(new TurnAction(0.0));
		runAction(new PrintTimeFromStart(_startTime));
	}
}