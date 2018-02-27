package org.usfirst.frc.team4028.robot.auton.modes.center;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.Action;
import org.usfirst.frc.team4028.robot.auton.actions.InfeedCubeAction;
import org.usfirst.frc.team4028.robot.auton.actions.MoveElevatorToPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.OutfeedCubeAction;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.SeriesAction;
import org.usfirst.frc.team4028.robot.auton.actions.SetInfeedPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.SimultaneousAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

public class TripleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toPyramid, fromPyramid, sTurnToSwitch; // Cube 2
	Path awayFromSwitch, toSideOfPyramid, awayFromPyramid, fromFrontofSwitchtoSwitch; // Cube 3
	double elevatorWaitTimeFirstCube, elevatorWaitTimeSecondCube;
	
	public TripleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID, 0.002);
			sTurnToSwitch = Paths.getPath(PATHS.S_TURN_TO_L_SWITCH, 0.009);
			awayFromSwitch = Paths.getPath(PATHS.AWAY_FROM_LEFT_SWITCH);
			toSideOfPyramid = Paths.getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT);
			awayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_L_PYRAMID);
			fromFrontofSwitchtoSwitch = Paths.getPath(PATHS.TO_L_SWITCH_WITH_CUBE_3);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID, 0.002);
			sTurnToSwitch = Paths.getPath(PATHS.S_TURN_TO_R_SWITCH, 0.009);
			awayFromSwitch = Paths.getPath(PATHS.AWAY_FROM_RIGHT_SWITCH);
			toSideOfPyramid = Paths.getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT);
			awayFromPyramid = Paths.getPath(PATHS.AWAY_FROM_R_PYRAMID);
			fromFrontofSwitchtoSwitch = Paths.getPath(PATHS.TO_R_SWITCH_WITH_CUBE_3);
		}
		toPyramid = Paths.getPath(PATHS.TO_PYRAMID);
		fromPyramid = Paths.getPath(PATHS.FROM_PYRAMID);
		
		elevatorWaitTimeFirstCube = 1.0;
		elevatorWaitTimeSecondCube = 0.8;
	}
	
	@Override
	public void routine() {	
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {	
					new RunMotionProfileAction(toSwitch),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTimeFirstCube),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
		})));
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		// Drive to front of pyramid while moving elevator to floor and swinging out infeeds
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1.4),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		// Drive into pyramid to acquire 2nd cube
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toPyramid),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.SQUEEZE),
									new InfeedCubeAction()
							}))
					}))
		})));
		runAction(new WaitAction(0.5));
		// Move back from pyramid while continuing to infeed
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromPyramid),
					new InfeedCubeAction()
		})));
		// Drive back to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(sTurnToSwitch),		
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTimeSecondCube),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
					}))
		}))); 
		// Outfeed cube for 0.2s
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		/*
		runAction(new RunMotionProfileAction(awayFromSwitch));
		runAction(new RunMotionProfileAction(toSideOfPyramid));
		runAction(new RunMotionProfileAction(awayFromPyramid));
		runAction(new RunMotionProfileAction(fromFrontofSwitchtoSwitch));
		*/
		runAction(new PrintTimeFromStart(_startTime));
	}
}