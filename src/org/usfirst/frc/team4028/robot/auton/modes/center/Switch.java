package org.usfirst.frc.team4028.robot.auton.modes.center;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Center;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class Switch extends AutonBase {
	Path toSwitch;
	double elevatorWaitTime;
	
	public Switch(boolean isSwitchLeft) {
		if (isSwitchLeft) 
			toSwitch = Paths.getPath(Center.L_SWITCH);
		else
			toSwitch = Paths.getPath(Center.R_SWITCH);

		elevatorWaitTime = 1.0;
	}
	
	@Override
	public void routine() {
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunTimedMotionProfileAction(toSwitch, 2.7),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction());
		runAction(new PrintTimeFromStart(_startTime));
		// Move Elevator back to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new DriveSetDistanceAction(-20.0)
		})));
	}
}