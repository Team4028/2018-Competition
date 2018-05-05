package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScaleAndSwitch extends AutonBase{
	Path toScale;
	Path scaleToSwitch;
	Path fromSwitchToScaleSecondCube;
	double targetTurnAngle, endTargetTurnAngle;
	double elevatorWaitTime;
	
	boolean isRightTurnToSwitch;
	
	public DoubleScaleAndSwitch(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(Left.L_SCALE);
			scaleToSwitch = Paths.getPath(Left.L_SCALE_TO_L_SWITCH);
			targetTurnAngle = 163;
			endTargetTurnAngle = 17;
			fromSwitchToScaleSecondCube = Paths.getPath(Left.L_SWITCH_TO_L_SCALE_SECOND_CUBE);
			isRightTurnToSwitch = true;
		} else {
			toScale = Paths.getPath(Left.R_SCALE);
			targetTurnAngle = -165;
			endTargetTurnAngle = -25;
			fromSwitchToScaleSecondCube = Paths.getPath(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE);
			isRightTurnToSwitch = false;
		}
		elevatorWaitTime = 1.25;
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(elevatorWaitTime),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT),
						new WaitUntilRemainingDistanceAction(18),
						new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50)
				}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
		// Lower Elevator to Switch during turn, then drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new TurnAction(targetTurnAngle, isRightTurnToSwitch),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new RunMotionProfileAction(scaleToSwitch),
									new SeriesAction(Arrays.asList(new Action[] {
											new WaitAction(0.55),
											new InfeedCubeAction()
									}))
							}))
					}))	
		})));
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(16),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(30)
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new DriveSetDistanceAction(-13.0),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
							new TurnAction(130.0, false)
					})),
					new SimultaneousAction(Arrays.asList(new Action[] {
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(15.0),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(.4),
						new InfeedCubeAction()
				}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(-37.0, false),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(30)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromSwitchToScaleSecondCube),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
		})));
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new DriveSetDistanceAction(-10.0));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
	}
}