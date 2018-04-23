package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class FarSwitchCloseScale extends AutonBase {
	Path toScale;
	Path fromScaleToSwitch;
	double turnTargetAngle, elevatorWaitTime;
	boolean isTurnRight;
	
	public FarSwitchCloseScale(boolean isStartingLeft) {
		if (isStartingLeft) {
			toScale = Paths.getPath(Left.L_SCALE);
			fromScaleToSwitch = Paths.getPath(Left.L_SCALE_TO_R_SWITCH);
			turnTargetAngle = 150;
			isTurnRight = true;
			elevatorWaitTime = 1.2;
		} else {
			toScale = Paths.getPath(Right.R_SCALE);
			fromScaleToSwitch = Paths.getPath(Right.R_SCALE_TO_L_SWITCH);
			turnTargetAngle = -150;
			isTurnRight = false;
			elevatorWaitTime = 1.5;
		}
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
						new OutfeedCubeAction(Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50)
				}))
		})));
		// Turn then drive to switch while lowering elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new TurnAction(turnTargetAngle, true),
							new RunMotionProfileAction(fromScaleToSwitch)
					}))
		})));
		// Turn to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(-160, isTurnRight),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.8),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(18),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.3),
							new InfeedCubeAction()
					}))
		})));
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new ArcadeDriveAction(0.7, 0.4),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		// Move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new DriveSetDistanceAction(-15)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(-125, true),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE)
		})));
		runAction(new DriveSetDistanceAction(15));
		runAction(new InfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
	}
}