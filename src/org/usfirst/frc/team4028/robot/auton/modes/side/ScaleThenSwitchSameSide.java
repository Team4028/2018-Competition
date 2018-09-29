package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleThenSwitchSameSide extends AutonBase {
	Path toScale;
	Path scaleToSwitch;
	double targetTurnAngle, elevatorWaitTime, driveToSwitchDistance;
	double remainingDistance;
	boolean isTurnRight;
	
	public ScaleThenSwitchSameSide(boolean isLeftScale) {
		if (isLeftScale) {
			toScale = Paths.getPath(Left.L_SCALE);
			scaleToSwitch = Paths.getPath(Left.L_SCALE_TO_L_SWITCH);
			targetTurnAngle = 160; //163
			elevatorWaitTime = 1.2;
			remainingDistance = 18;
			isTurnRight = true;
		} else {
			toScale = Paths.getPath(Left.R_SCALE);
			scaleToSwitch = Paths.getPath(Left.R_SCALE_TO_R_SWITCH);
			targetTurnAngle = -155;
			elevatorWaitTime = 2.8;	
			remainingDistance = 12;
			isTurnRight = true; 
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
						new WaitUntilRemainingDistanceAction(remainingDistance),
						new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50, 0.3)
				}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new DriveSetDistanceAction(-10));
		// Lower Elevator to Switch during turn, then drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new TurnAction(targetTurnAngle, isTurnRight),
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
					new ArcadeDriveAction(0.7, 0.4),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					//new ArcadeDriveAction(-1, 0.2)
		})));
	}
}