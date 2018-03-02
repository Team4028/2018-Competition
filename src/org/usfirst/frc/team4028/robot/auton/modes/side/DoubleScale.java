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

public class DoubleScale extends AutonBase{
	Path toScale;
	
	double toSwitchDistance, toScaleAgainDistance;
	double targetTurnAngle, endTargetTurnAngle;
	double finalTurnTargetAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	
	boolean isRightTurnToSwitch;
	
	public DoubleScale(boolean isLeftScale, boolean isStartingLeft) {
		if (isLeftScale) {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.L_SCALE);
				toSwitchDistance = 36.0;
				toScaleAgainDistance = -38.0;
				targetTurnAngle = 165;
				endTargetTurnAngle = 25;
				finalTurnTargetAngle = 135;
				elevatorWaitTime1 = 2.0;
				elevatorWaitTime2 = 0.5;
				isRightTurnToSwitch = true;
			} else {
				toScale = Paths.getPath(RightSide.L_SCALE);
				toSwitchDistance = 36.0;
				toScaleAgainDistance = -36.0;
				targetTurnAngle = 168;
				endTargetTurnAngle = 15;
				finalTurnTargetAngle = 135;
				elevatorWaitTime1 = 4.25;
				elevatorWaitTime2 = 1.0;
				isRightTurnToSwitch = false;
			}
		} else {
			if (isStartingLeft) {
				toScale = Paths.getPath(LeftSide.R_SCALE);
				toSwitchDistance = 39.0;
				toScaleAgainDistance = -39.0;
				targetTurnAngle = -168;
				endTargetTurnAngle = -20.0;
				finalTurnTargetAngle = -135;
				elevatorWaitTime1 = 4.25;
				elevatorWaitTime2 = 1.0;
				isRightTurnToSwitch = true;
			} else {
				toScale = Paths.getPath(RightSide.R_SCALE);
				toSwitchDistance = 36.0;
				toScaleAgainDistance = -40.0;
				targetTurnAngle = -165;
				endTargetTurnAngle = -25;
				finalTurnTargetAngle = -135;
				elevatorWaitTime1 = 2.0;
				elevatorWaitTime2 = 0.5;
				isRightTurnToSwitch = false;
			}
		}
	}

	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime1),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
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
							new WaitAction(0.7),
							new TurnAction(targetTurnAngle, isRightTurnToSwitch),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new DriveSetDistanceAction(toSwitchDistance)
							}))
					}))	
		})));
		// Infeed cube while sitting in place
		runAction(new InfeedCubeAction());
		// Drive back to scale and turn while raising elevator to scale height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new DriveSetDistanceAction(toScaleAgainDistance),
							new TurnAction(endTargetTurnAngle, !isRightTurnToSwitch)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime2),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		}))); 
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new DriveSetDistanceAction(-10.0));
		// Move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.7),
						new TurnAction(finalTurnTargetAngle, isRightTurnToSwitch),
						new DriveSetDistanceAction(40.0)
				}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}