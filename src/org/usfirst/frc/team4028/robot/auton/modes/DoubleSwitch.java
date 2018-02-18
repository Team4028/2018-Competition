package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

public class DoubleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toThePyramid, sTurnAwayFromPyramid, toSwitchAfterSTurn; // Cube 2
	double elevatorWaitTime1, elevatorWaitTime2;
	public DoubleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 100.0, 120.0, 0.004);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID, 100.0, 120.0, 0.002);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT, 100.0, 120.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_L_SWITCH_AFTER_S_TURN, 100.0, 120.0, 0.009);
			elevatorWaitTime1=1.75;
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 100.0, 120.0, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID, 100.0, 120.0, 0.008);
			sTurnAwayFromPyramid = Paths.getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT, 100.0, 120.0);
			toSwitchAfterSTurn = Paths.getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN, 100.0, 120.0,0.009);
		}
		toThePyramid = Paths.getPath(PATHS.TO_PYRAMID, 100.0, 120.0);
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitch),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime1),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new WaitAction(0.5),
				new RunCarriageWheelsAction(false)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1.4),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toThePyramid),
					new RunCarriageWheelsAction(true),
					new DriveInfeedWheelsAction(),
					new SeriesAction(Arrays.asList(new Action[] {
							new ParallelAction(Arrays.asList(new Action[] {
									new WaitAction(0.5),
									new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE),
							})),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE)
					}))
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.2),
							new RunMotionProfileAction(sTurnAwayFromPyramid)
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction(),
					new RunCarriageWheelsAction(true)
			}
		)));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(toSwitchAfterSTurn),
							new ParallelAction(Arrays.asList(new Action[] {
								new RunCarriageWheelsAction(false),
								new WaitAction(0.5)
							}))
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime2),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
					}))
			}
		))); 
		runAction(new PrintTimeFromStart(_startTime));  
	}
}