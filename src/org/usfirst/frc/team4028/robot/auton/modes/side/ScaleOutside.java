package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.LeftSide;
import org.usfirst.frc.team4028.robot.paths.Paths.RightSide;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleOutside extends AutonBase{
	Path toScale;
	double targetTurnAngle;
	double elevatorWaitTime;
	
	boolean isTurnRight;
	
	public ScaleOutside(boolean isStartingLeft) {
		if (isStartingLeft) {
			toScale = Paths.getPath(LeftSide.L_SCALE_OUTSIDE);
			targetTurnAngle = 80;
			isTurnRight = true;
		} else {
			toScale = Paths.getPath(RightSide.R_SCALE_OUTSIDE);
			targetTurnAngle = -80;
			isTurnRight = false;
		}
		elevatorWaitTime = 3.0;
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(elevatorWaitTime),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
				}))
		})));
		// Turn to scale while raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.7),
						new TurnAction(targetTurnAngle, isTurnRight)
				})),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
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