package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class RightDoubleScale extends AutonBase {
	Path toScale,ScaletoSwitch,SwitchtoScale;
	double targetTurnAngle,endTargetTurnAngle;
	
	public RightDoubleScale() {
		toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, 0.007);
		ScaletoSwitch = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH,100,120);
		SwitchtoScale= Paths.getPath(PATHS.R_SWITCH_TO_R_SCALE, 100, 120);
		
		targetTurnAngle = -170;
		endTargetTurnAngle = 0;
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new TurnAction(targetTurnAngle, true));	
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(ScaletoSwitch),
				new SeriesAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE),
					new WaitAction(0.75),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE)
				}))
				
		}))); 	
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new RunMotionProfileAction(SwitchtoScale),
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction()
		}))); 
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new TurnAction(endTargetTurnAngle, false),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}