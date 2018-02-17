package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class LeftDoubleScale extends AutonBase{
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale;
	double targetTurnAngle,endTargetTurnAngle;
	
	public LeftDoubleScale() {
		toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0);
		targetTurnAngle = 160;//137.862405226;
		endTargetTurnAngle = 30;
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0, 0.005);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
	}

	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new TurnAction(targetTurnAngle, true));
		//runAction(new DriveSetDistanceAction(30));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitch),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.15),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.45),
							new RunMotionProfileAction(fromSwitchToScale)
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction()
		}))); 
		runAction(new TurnAction(endTargetTurnAngle, false));
		runAction(new PrintTimeFromStart(_startTime));
	}
}