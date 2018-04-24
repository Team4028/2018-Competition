package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class ScaleOutside extends AutonBase{
	Path toScale;
	double targetTurnAngle;
	double elevatorWaitTime;
	
	boolean isTurnRight;
	
	public ScaleOutside(boolean isStartingLeft) {
		if (isStartingLeft) {
			toScale = Paths.getPath(Left.L_SCALE_OUTSIDE);
			targetTurnAngle = 80;
			isTurnRight = true;
		} else {
			toScale = Paths.getPath(Right.R_SCALE_OUTSIDE);
			targetTurnAngle = -80;
			isTurnRight = false;
		}
		elevatorWaitTime = 1.0;
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
				new TurnAction(targetTurnAngle, isTurnRight),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		// Drive backwards 20in and move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(-10.0),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
	}
}