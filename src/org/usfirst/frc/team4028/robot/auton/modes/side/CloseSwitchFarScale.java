package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.auton.actions.*;

public class CloseSwitchFarScale extends AutonBase {
	Path toLeftSwitchSide = Paths.getPath(Left.L_SWITCH_SIDE);
	Path toRightScalefromLeftSwitchSide = Paths.getPath(Left.L_SWITCH_SIDE_TO_R_SCALE);
	Path toRightScaleSecondCube = Paths.getPath(Left.TO_R_SCALE_SECOND_CUBE);
	
	@Override
	public void routine() {
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.8),
						new MoveElevatorToPosAction(40)
				})),
				new RunTimedMotionProfileAction(toLeftSwitchSide, 2.6)
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		// Turn to 0 degrees to be parallel to switch and lower infeed to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new TurnAction(0, false),
						new RunMotionProfileAction(toRightScalefromLeftSwitchSide)
				})),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
		// Turn to cube and set infeeds wide
		runAction(new TurnAction(-162, true));
		// Drive to cube and infeed it
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(5.0),
					new SeriesAction(Arrays.asList(new Action[] {
							//new WaitAction(0.5),
							new InfeedCubeAction()
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new TurnAction(37.6, false),
					new MoveElevatorToPosAction(44),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toRightScaleSecondCube),
				new ActuateFlapJackAction(true),
				new MoveElevatorToPosAction(76)
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_80));
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new DriveSetDistanceAction(-15.0));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
	}
}