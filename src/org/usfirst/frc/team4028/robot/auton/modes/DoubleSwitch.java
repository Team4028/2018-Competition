package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.Action;
import org.usfirst.frc.team4028.robot.auton.actions.DriveInfeedWheelsAction;
import org.usfirst.frc.team4028.robot.auton.actions.ParallelAction;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.SeriesAction;
import org.usfirst.frc.team4028.robot.auton.actions.SetInfeedPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class DoubleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toThePyramid, sTurnAwayFromPyramid, toSwitchAfterSTurn; // Cube 2
	
	public DoubleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 100.0, 120.0, 0.004);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID, 100.0, 120.0, 0.00);
			toThePyramid = Paths.getPath(PATHS.TO_PYRAMID_L, 100.0, 120.0);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT, 100.0, 120.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_L_SWITCH_AFTER_S_TURN, 100.0, 120.0, 0.005);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 100.0, 120.0, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID, 100.0, 120.0, 0.008);
			toThePyramid = Paths.getPath(PATHS.TO_PYRAMID_R, 100.0, 120.0);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT, 100.0, 120.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN, 100.0, 120.0,0.009);
		}
		
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitch),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1.2),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toThePyramid),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE),
					new DriveInfeedWheelsAction()
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new RunMotionProfileAction(sTurnAwayFromPyramid)
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction()
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitchAfterSTurn),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
			}
		))); 
		runAction(new PrintTimeFromStart(_startTime));  
	}
}