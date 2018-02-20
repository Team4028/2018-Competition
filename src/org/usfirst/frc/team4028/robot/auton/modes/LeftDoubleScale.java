package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.util.control.Path;

public class LeftDoubleScale extends AutonBase{
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale;
	double targetTurnAngle,endTargetTurnAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	
	public LeftDoubleScale() {
		toScale = Paths.getPath(PATHS.L_SCALE, 100.0, 120.0, 0);
		targetTurnAngle = 160;//137.862405226;
		endTargetTurnAngle = 30;
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0, 0.005);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
		elevatorWaitTime1 = 3.0;
		elevatorWaitTime2 = 0.75;
	}

	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
				//	new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime1),
						//	new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT),
							new ParallelAction(Arrays.asList(new Action[] {
							//		new RunCarriageWheelsAction(false),
									new WaitAction(0.5)
							}))
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new TurnAction(targetTurnAngle, true),
				//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitch),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.65),
						//	new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.65),
							new ParallelAction(Arrays.asList(new Action[] {
									new RunMotionProfileAction(fromSwitchToScale),
									new SeriesAction(Arrays.asList(new Action[] {
											new WaitAction(elevatorWaitTime2),
								//			new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
									}))
							}))
					})),
				//	new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
				//	new DriveInfeedWheelsAction(),
				//	new RunCarriageWheelsAction(true)
		}))); 
		runAction(new TurnAction(endTargetTurnAngle, false));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				//new RunCarriageWheelsAction(false),
				new WaitAction(0.5)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new DriveSetDistanceAction(-30.0),
				//new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}