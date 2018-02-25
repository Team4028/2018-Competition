package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchSameSide extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double targetTurnAngle, elevatorWaitTime;
	
	public ScaleThenSwitchSameSide(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0.0055);
			fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100, 120, 0.005);
			targetTurnAngle = 162;
			elevatorWaitTime = 2.0;
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 100.0, 0.005);
			fromScaleToSwitch = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH);
			targetTurnAngle = -170;
			elevatorWaitTime = 4.0;
		}
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
							//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Lower Elevator to Switch before turn
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new TurnAction(targetTurnAngle, true)	
		})));
		// Drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.65),
						new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
				})),
				new DriveSetDistanceAction(40.0)
		})));	
		// Infeed cube while sitting in place for 0.65s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.65),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
					new InfeedCubeAction()
		})));
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(12),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[ ] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}