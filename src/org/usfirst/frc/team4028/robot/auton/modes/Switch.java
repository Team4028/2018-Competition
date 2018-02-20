package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class Switch extends AutonBase {
	Path toSwitch;
	double elevatorWaitTime;
	
	public Switch(boolean isSwitchLeft) {
		if (isSwitchLeft) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 100.0, 120.0, 0.004);
			elevatorWaitTime = 1.75;
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 100.0, 120.0, 0.0065);
			elevatorWaitTime = 1.75;
		}
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitch),
				//	new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
			//				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new WaitAction(0.5),
			//	new RunCarriageWheelsAction(false)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(-20.0),
				//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}