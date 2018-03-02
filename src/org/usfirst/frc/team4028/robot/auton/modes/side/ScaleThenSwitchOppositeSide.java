package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.LeftSide;
import org.usfirst.frc.team4028.robot.paths.Paths.RightSide;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchOppositeSide extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double turnTargetAngle, elevatorWaitTime;
	boolean isTurnRight;
	
	public ScaleThenSwitchOppositeSide(boolean isLeftScale, boolean isStartingLeft) {
		if (isLeftScale) {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.L_SCALE);
				fromScaleToSwitch = Paths.getPath(LeftSide.L_SCALE_TO_R_SWITCH);
				turnTargetAngle = 150;
				isTurnRight = true;
				elevatorWaitTime = 2.0;
			} else {
				toScale = Paths.getPath(RightSide.R_SCALE);
				fromScaleToSwitch = Paths.getPath(RightSide.L_SCALE_TO_R_SWITCH);
				turnTargetAngle = 150;
				isTurnRight = true;
				elevatorWaitTime = 4.5;
			}
		} else {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.R_SCALE);
				fromScaleToSwitch = Paths.getPath(LeftSide.R_SCALE_TO_L_SWITCH);
				turnTargetAngle = -150;
				isTurnRight = false;
				elevatorWaitTime = 4.5;
			} else {
				toScale = Paths.getPath(RightSide.R_SCALE);
				fromScaleToSwitch = Paths.getPath(RightSide.R_SCALE_TO_L_SWITCH);
				turnTargetAngle = -150;
				isTurnRight = false;
				elevatorWaitTime = 2.0;
			}
		}
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Turn then driveto switch while lowering elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new TurnAction(turnTargetAngle, true),
							new RunMotionProfileAction(fromScaleToSwitch)
					}))
		})));
		// Turn to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(180, isTurnRight),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.8),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new InfeedCubeAction(), 
					new DriveSetDistanceAction(6)
		})));
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(12),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		runAction(new PrintTimeFromStart(_startTime));
		// Move elevator to floor
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)); 
		runAction(new PrintTimeFromStart(_startTime));
	}
}