package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.LeftSide;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleOutside extends AutonBase{
	Path toScale;
	double elevatorWaitTime;
	
	public ScaleOutside() {
		toScale = Paths.getPath(LeftSide.L_SCALE_OUTSIDE);
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(elevatorWaitTime),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
				}))
		})));
		// Turn to scale while raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new TurnAction(80.0, true),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
		})));
		runAction(new DriveSetDistanceAction(15.0));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		// Drive backwards 20in and move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(-20.0),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
	}
}