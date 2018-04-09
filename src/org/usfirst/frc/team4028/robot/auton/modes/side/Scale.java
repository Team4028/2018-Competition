package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class Scale extends AutonBase {
	Path toScale;
	double elevatorWaitTime;
	
	public Scale(boolean isScaleLeft, boolean isStartingLeft) {
		if (isStartingLeft) {
			if (isScaleLeft) {
				toScale = Paths.getPath(Left.L_SCALE);
				elevatorWaitTime = 1.5;
			} else {
				toScale = Paths.getPath(Left.R_SCALE);
				elevatorWaitTime = 4.0;
			}
		} else {
			if (isScaleLeft) {
				toScale = Paths.getPath(Right.L_SCALE);
				elevatorWaitTime = 4.0;
			} else {
				toScale = Paths.getPath(Right.R_SCALE);
				elevatorWaitTime = 1.5;
			}
		}
	}
	
	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		// Drive backwards 10in and move elevator to floor
		runAction(new DriveSetDistanceAction(-10.0));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)); 
		runAction(new PrintTimeFromStart(_startTime));
	}
}