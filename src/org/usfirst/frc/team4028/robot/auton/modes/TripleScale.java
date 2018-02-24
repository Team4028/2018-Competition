package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class TripleScale extends AutonBase{
	Path toScale;
	Path fromScaleToSwitch, fromSwitchToScale, ScaleToSwitch2, SwitchtoScale2;
	double targetTurnAngle1, targetTurnAngle2, endTargetTurnAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	
	public TripleScale() {
		toScale = Paths.getPath(PATHS.L_SCALE, 0.0055);
		targetTurnAngle1 = 160;
		targetTurnAngle2 = 145;
		endTargetTurnAngle = 30;
		fromScaleToSwitch = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH, 100.0, 120.0, 0.005);
		fromSwitchToScale = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE, 100.0, 120.0);
		ScaleToSwitch2 = Paths.getPath(PATHS.L_SCALE_TO_L_SWITCH_2,100,120);
		SwitchtoScale2 = Paths.getPath(PATHS.L_SWITCH_TO_L_SCALE_2);
		
		elevatorWaitTime1 = 2.0;
	}

	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toScale),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime1),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.2),
					new OutfeedCubeAction()
		})));
		// Turn to switch while lowering elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new TurnAction(targetTurnAngle1, true),
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT)
		})));
		// Drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromScaleToSwitch),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(0.65),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		// Infeed cube while sitting in place for 0.65s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new WaitAction(0.65),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
					new InfeedCubeAction()
		})));
		// Continue infeeding while driving back to scale and turning
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(fromSwitchToScale),
							new TurnAction(30, false)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new InfeedCubeAction(),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SCALE_HEIGHT),
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
							}))
					}))
		}))); 
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
	runAction(new PrintTimeFromStart(_startTime));
	}
}