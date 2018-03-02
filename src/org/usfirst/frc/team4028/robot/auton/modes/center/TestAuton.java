package org.usfirst.frc.team4028.robot.auton.modes.center;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

public class TestAuton extends AutonBase {
	@Override
	public void routine() {
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(4.0),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(2.0),
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
							new WaitAction(2.0),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new DriveSetDistanceAction(38)
							}))
					}))	
		})));
		// Infeed cube while sitting in place
		runAction(new InfeedCubeAction());
		// Drive back to scale and turn while raising elevator to scale height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new DriveSetDistanceAction(-38),
							new WaitAction(1.5)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.7),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
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
						new WaitAction(2.0),
						new DriveSetDistanceAction(40.0)
				}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}