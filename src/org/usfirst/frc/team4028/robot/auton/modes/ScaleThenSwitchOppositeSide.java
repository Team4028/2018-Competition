package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchOppositeSide extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double turnTargetAngle, elevatorWaitTime;
	boolean isTurnRight;
	
	public ScaleThenSwitchOppositeSide(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 100.0, 0.0055);
			fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_R_SWITCH, 100, 120, 0.00);
			turnTargetAngle = 150;
			isTurnRight = true;
			elevatorWaitTime = Math.E;
			
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 100.0, 0.005);
			fromScaleToSwitch = Paths.getPath(PATHS.R_SCALE_TO_L_SWITCH,100,120,0.006);
			turnTargetAngle = -150;
			isTurnRight = false;
			elevatorWaitTime = Math.E;
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
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Turn to switch while lowering elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(turnTargetAngle,true),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
		// Drive to 2nd cube while storing infeed and moving elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitch),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		// Turn to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(180, isTurnRight),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
									new OutfeedCubeAction()
							}))
					}))
		})));
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(14),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Move elevator to floor
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
		runAction(new PrintTimeFromStart(_startTime));
	}
}