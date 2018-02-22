package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class Scale extends AutonBase{
	Path toScale;
	double elevatorWaitTime;
	
	public Scale(boolean isScaleLeft) {
		if (isScaleLeft) {
			toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0.0055);
			elevatorWaitTime = 3.0;
		} else {
			toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, 0.005);
			elevatorWaitTime = 5.0;
		}
	}
	
	@Override
	public void routine() {
		runAction(new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					//new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime),
						//	new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new WaitAction(0.5),
				//new RunCarriageWheelsAction(false)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				//new DriveSetDistanceAction(-30.0),
				//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}