package org.usfirst.frc.team4028.robot.auton.modes;

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
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action [] {
						new WaitAction(_firstWaitTime),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
				})),
				new RunMotionProfileAction(_toleftSwitchonSide)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		runAction(new TurnAction(0, false));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(_toRightScalefromLeftSwitchside),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR)
		})));
		runAction(new TurnAction(180, true));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(20.0),
				new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
		})));
	}
}