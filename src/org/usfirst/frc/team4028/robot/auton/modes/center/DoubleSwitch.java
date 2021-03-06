package org.usfirst.frc.team4028.robot.auton.modes.center;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.Center;
import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

public class DoubleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toPyramid, fromPyramid, sTurnToSwitch; // Cube 2
	Path awayFromSwitch, pyramidAgain;
	double elevatorWaitTimeFirstCube, elevatorWaitTimeSecondCube;
	
	public DoubleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(Center.L_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(Center.L_SWITCH_TO_PYRAMID_FRONT);
			sTurnToSwitch = Paths.getPath(Center.S_TURN_TO_L_SWITCH);
			awayFromSwitch = Paths.getPath(Center.AWAY_FROM_L_SWITCH);
			pyramidAgain = Paths.getPath(Center.PYRAMID_AGAIN_FROM_L);
		} else {
			toSwitch = Paths.getPath(Center.R_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(Center.R_SWITCH_TO_PYRAMID_FRONT);
			sTurnToSwitch = Paths.getPath(Center.S_TURN_TO_R_SWITCH);
			awayFromSwitch = Paths.getPath(Center.AWAY_FROM_R_SWITCH);
			pyramidAgain = Paths.getPath(Center.PYRAMID_AGAIN_FROM_R);
		}
		
		elevatorWaitTimeFirstCube = 1.0;
		elevatorWaitTimeSecondCube = 0.8;
		
		toPyramid = Paths.getPath(Center.TO_PYRAMID);
		fromPyramid = Paths.getPath(Center.FROM_PYRAMID);
	}
	
	@Override
	public void routine() {
		// Drive to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {	
					new RunTimedMotionProfileAction(toSwitch, 2.6),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTimeFirstCube),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
		// Outfeed cube for 0.2s
		runAction(new OutfeedCubeAction(Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_35, .2));
		// Drive to front of pyramid while moving elevator to floor and swinging out infeeds
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),	
				new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),	
				new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1.7),
							new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
		})));
		// Drive into pyramid to acquire 2nd cube
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunTimedMotionProfileAction(toPyramid,2.0),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new InfeedCubeAction()
							}))
					}))
		})));
		// Move back from pyramid while continuing to infeed
		runAction(new RunMotionProfileAction(fromPyramid));
		// Drive back to switch while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunTimedMotionProfileAction(sTurnToSwitch, 2.7),
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
		// Move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new RunMotionProfileAction(awayFromSwitch)
		})));
		/*runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(pyramidAgain),
				new InfeedCubeAction()
		})));*/
		runAction(new PrintTimeFromStart(_startTime));  
	}
}