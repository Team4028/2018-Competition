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

public class DoubleScaleAndSwitch extends AutonBase{
	Path toScale;
	Path fromScaleToSwitchSecondCube;
	double targetTurnAngle, endTargetTurnAngle;
	double elevatorWaitTime;
	
	boolean isRightTurnToSwitch;
	
	public DoubleScaleAndSwitch(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(LeftSide.L_SCALE);
			targetTurnAngle = 163;
			endTargetTurnAngle = 17;
			fromScaleToSwitchSecondCube = Paths.getPath(LeftSide.L_SWITCH_TO_L_SCALE_SECOND_CUBE);
			isRightTurnToSwitch = true;
		} else {
			toScale = Paths.getPath(LeftSide.R_SCALE);
			targetTurnAngle = -165;
			endTargetTurnAngle = -25;
			fromScaleToSwitchSecondCube = Paths.getPath(RightSide.R_SWITCH_TO_R_SCALE_SECOND_CUBE);
			isRightTurnToSwitch = false;
		}
		elevatorWaitTime = 1.75;
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
							new TurnAction(targetTurnAngle, isRightTurnToSwitch),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new DriveSetDistanceAction(36.0)
							}))
					}))	
		})));
		// Infeed cube while sitting in place
		runAction(new InfeedCubeAction());
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(14),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(30)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[ ] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		}))); 
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new DriveSetDistanceAction(-13.0),
							new TurnAction(130.0, false)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(14.0),
					new InfeedCubeAction()
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(-37.0, false),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(36)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitchSecondCube),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[ ] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		}))); 
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new DriveSetDistanceAction(-10.0));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
		runAction(new PrintTimeFromStart(_startTime));
	}
}