package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.auton.actions.*;

public class LeftSwitchtoRightScaleExperimental extends AutonBase {
	Path _toleftSwitchonSide = Paths.getPath(PATHS.L_SWITCH_TO_SIDE);
	Path _toRightScalefromLeftSwitchside = Paths.getPath(PATHS.L_SWITCH_SIDE_TO_R_SCALE);
	double _firstWaitTime= 0.5;
	
	@Override
	public void routine() {
		// Drive to switch at an angle while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action [] {
							new WaitAction(_firstWaitTime),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					})),
					new RunMotionProfileAction(_toleftSwitchonSide),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Turn to 0 degrees to be parallel to switch
		runAction(new TurnAction(0, false));
		// Drive to 2nd cube while lowering cube to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(_toRightScalefromLeftSwitchside),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
		// Turn to cube and set infeeds wide
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(180, true),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
		})));
		// Drive to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(20.0),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
									new InfeedCubeAction()
							}))
					}))
		})));
		// Drive to switch while raising elevator and storing infeed
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(12),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
	}
}