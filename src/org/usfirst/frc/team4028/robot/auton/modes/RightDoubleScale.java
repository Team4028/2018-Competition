package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class RightDoubleScale extends AutonBase {
	Path toScale,ScaletoSwitch,SwitchtoScale;
	double targetTurnAngle,endTargetTurnAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	
	public RightDoubleScale() {
		toScale = Paths.getPath(PATHS.R_SCALE, 100.0, 120.0, 0.007);
		ScaletoSwitch = Paths.getPath(PATHS.R_SCALE_TO_R_SWITCH,100,120);
		SwitchtoScale= Paths.getPath(PATHS.R_SWITCH_TO_R_SCALE, 100, 120);
		
		targetTurnAngle = -170;
		endTargetTurnAngle = 0;
		elevatorWaitTime1 = 5.0;
		elevatorWaitTime2 = 2.0;
	}
	
	@Override
	public void routine() {
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime1),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new TurnAction(targetTurnAngle, true),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR)
		})));	
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(ScaletoSwitch),
				new SeriesAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE),
					new WaitAction(0.75),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE)
				}))
				
		}))); 	
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.5),
							new ParallelAction(Arrays.asList(new Action[] {
									new RunMotionProfileAction(SwitchtoScale),
									new SeriesAction(Arrays.asList(new Action[] {
											new WaitAction(elevatorWaitTime2),
											new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
									}))
							}))
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.SQUEEZE),
					new DriveInfeedWheelsAction(),
					new RunCarriageWheelsAction(true)
		}))); 
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new TurnAction(endTargetTurnAngle, false),
				new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE)
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new RunCarriageWheelsAction(false),
				new WaitAction(0.5)
		})));
		runAction(new RunCarriageWheelsAction(false));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(-30.0),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
		})));
		runAction(new PrintTimeFromStart(_startTime));
	}
}