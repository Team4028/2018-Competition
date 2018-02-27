package org.usfirst.frc.team4028.robot.auton.modes.left;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class TripleScale extends AutonBase{
	Path toScale;
	double targetTurnAngle, targetTurnAngle2, endTargetTurnAngle;
	double driveToSwitchDistance;
	double elevatorWaitTime;
	
	public TripleScale() {
		toScale = Paths.getPath(PATHS.L_SCALE, 0.0055);
		targetTurnAngle = 160;
		targetTurnAngle2 = 145;
		endTargetTurnAngle = 30;
		
		driveToSwitchDistance = 38.0;
		
		elevatorWaitTime = 2.0;
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
		// Lower Elevator to Switch during turn, then drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new TurnAction(targetTurnAngle, true),
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
					new DriveSetDistanceAction(12),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[ ] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		}))); 
		runAction(new PrintTimeFromStart(_startTime));
	}
}