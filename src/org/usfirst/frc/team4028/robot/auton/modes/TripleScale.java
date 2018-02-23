package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class TripleScale extends AutonBase{
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale, ScaleToSwitch2, SwitchtoScale2;
	double targetTurnAngle1, targetTurnAngle2, endTargetTurnAngle;
	
	public TripleScale() {
		toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0);
		targetTurnAngle1 = 160;
		targetTurnAngle2 = 145;
		endTargetTurnAngle = 30;
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0, 0.005);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
		ScaleToSwitch2 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_2,100,120);
		SwitchtoScale2 = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE_2);
	}

	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SetInfeedPosAction(Infeed.INFEED_ARM_TARGET_POSITION.STORE)
	})));
	runAction(new TurnAction(targetTurnAngle1, true));
	//runAction(new DriveSetDistanceAction(30));
	runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(fromScaleToSwitch),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.15),
						new SetInfeedPosAction(Infeed.INFEED_ARM_TARGET_POSITION.WIDE)
				}))
	})));
	runAction(new ParallelAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.50),
						new RunMotionProfileAction(fromSwitchToScale)
				})),
				new SetInfeedPosAction(Infeed.INFEED_ARM_TARGET_POSITION.SQUEEZE),
				new DriveInfeedWheelsAction()
	}))); 
	runAction(new TurnAction(endTargetTurnAngle, false));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new SetInfeedPosAction(Infeed.INFEED_ARM_TARGET_POSITION.STORE),
				new WaitAction(0.5)
		})));
		runAction(new TurnAction(targetTurnAngle2,true));
		/*runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(ScaleToSwitch2),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.75),
						new ParallelAction(Arrays.asList(new Action[] {
								new WaitAction(1),
								new DriveInfeedWheelsAction(),
								new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE)
						}))
				})),
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(SwitchtoScale2),
				new DriveInfeedWheelsAction()
		})));
		runAction(new TurnAction(0, false));*/
		runAction(new RunMotionProfileAction(ScaleToSwitch2));
		//runAction(new DriveSetDistanceAction(30));
		runAction(new PrintTimeFromStart(_startTime));
	}
}