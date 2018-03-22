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

public class ScaleThenSwitchSameSide extends AutonBase {
	Path toScale;
	double targetTurnAngle, elevatorWaitTime, driveToSwitchDistance;
	boolean isTurnRight;
	
	public ScaleThenSwitchSameSide(boolean isLeftScale, boolean isStartingLeft) {
		if (isLeftScale) {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.L_SCALE);
				targetTurnAngle = 163;
				elevatorWaitTime = 2.0;
				driveToSwitchDistance = 36.0;
				isTurnRight = true;
			} else {
				toScale = Paths.getPath(RightSide.L_SCALE);
				targetTurnAngle = 168;
				elevatorWaitTime = 4.25;
				driveToSwitchDistance = 32.0;
				isTurnRight = false;
			}
		} else {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.R_SCALE);
				targetTurnAngle = -166;
				elevatorWaitTime = 4.25;	
				driveToSwitchDistance = 39.0;
				isTurnRight = true;
			} else {
				toScale = Paths.getPath(RightSide.R_SCALE);
				targetTurnAngle = -165;
				elevatorWaitTime = 1.75;
				driveToSwitchDistance = 32.0;
				isTurnRight = false;
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
		// Lower Elevator to Switch during turn, then drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.3),
							new TurnAction(targetTurnAngle, isTurnRight),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new DriveSetDistanceAction(driveToSwitchDistance)
							}))
					}))	
		})));
		// Infeed cube while sitting in place
		runAction(new InfeedCubeAction());
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(14),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[ ] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		}))); 
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
			new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
			new DriveSetDistanceAction(-20.0)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}