package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class RightDoubleScale extends AutonBase {
	Path toScale, scaleToSwitch, switchToScale;
	double targetTurnAngle, endTargetTurnAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	
	public RightDoubleScale() {
		toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 100.0, 0.005);
		scaleToSwitch = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH, 100, 120);
		switchToScale= Paths.getPath(PATHS.R_SWITCH_TO_R_SCALE, 100, 120);
		
		targetTurnAngle = -168;
		endTargetTurnAngle = 0;
		elevatorWaitTime1 = 4.7;
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
							new WaitAction(0.8),
							new TurnAction(targetTurnAngle, true),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new DriveSetDistanceAction(38.0)
							}))
					}))	
		})));
		// Infeed cube while sitting in place for 0.65s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
					new InfeedCubeAction()
		})));
		// Drive back to scale and raise elevator to switch height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
				new SeriesAction(Arrays.asList(new Action[] {
						new DriveSetDistanceAction(-38.0),
						new TurnAction(0.0, false)
				})),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(1.0),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
				}))
		}))); 
		// Turn while raising elevator to scale height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(-15.0, false),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
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