package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.LeftSide;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class BeefSquadDLCTripleScaleMrBrunsDontReadSuperSecretEncrypted extends AutonBase {
	Path toLeftSwitchOnSide = Paths.getPath(LeftSide.L_SWITCH_SIDE_2);
	Path toRightScalefromLeftSwitchSide = Paths.getPath(LeftSide.L_SWITCH_SIDE_TO_R_SCALE_2);
	double _firstWaitTime = 1.0;
	
	@Override
	public void routine() {
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(_firstWaitTime),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
				})),
				new RunTimedMotionProfileAction(toLeftSwitchOnSide, 2.6)
		})));
		runAction(new PrintTimeFromStart(_startTime));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Turn to 0 degrees to be parallel to switch
		runAction(new TurnAction(0, false));
		// Drive to 2nd cube while lowering cube to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toRightScalefromLeftSwitchSide),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
		// Turn to cube and set infeeds wide
		runAction(new TurnAction(-160, true));
		// Drive to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(4.0),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new InfeedCubeAction()
					}))
		})));
		// Drive to switch while raising elevator and storing infeed
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(-34),
					//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(0, false),
					//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					//new OutfeedCubeAction()
		}))); 
		runAction(new PrintTimeFromStart(_startTime));
	}
}