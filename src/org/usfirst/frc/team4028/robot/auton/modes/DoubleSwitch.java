package org.usfirst.frc.team4028.robot.auton.modes;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.robot.paths.Paths.PATHS;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_TARGET_POSITION;

public class DoubleSwitch extends AutonBase {
	Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toPyramid, fromPyramid, sTurnToSwitch; // Cube 2
	double elevatorWaitTimeFirstCube, elevatorWaitTimeSecondCube;
	public DoubleSwitch(boolean isLeftSwitch) {
		if (isLeftSwitch) {
			toSwitch = Paths.getPath(PATHS.L_SWITCH, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID, 0.002);
			sTurnToSwitch = Paths.getPath(PATHS.S_TURN_TO_L_SWITCH, 0.009);
		} else {
			toSwitch = Paths.getPath(PATHS.R_SWITCH, 100.0, 120.0, 0.0065);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID, 0.008);
			sTurnToSwitch = Paths.getPath(PATHS.S_TURN_TO_R_SWITCH, 0.009);
		}
		
		elevatorWaitTimeFirstCube = 1.0;
		elevatorWaitTimeSecondCube = 0.8;
		
		toPyramid = Paths.getPath(PATHS.TO_PYRAMID);
		fromPyramid = Paths.getPath(PATHS.FROM_PYRAMID);
	}
	
	@Override
	public void routine() {
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toSwitch),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTimeFirstCube),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT)
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(1.4),
							new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.WIDE)
					}))
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(toPyramid),
					new SimultaneousAction(Arrays.asList(new Action[] {
							new DriveInfeedWheelsAction(),
							new SeriesAction(Arrays.asList(new Action[] {
									new WaitAction(1),
									new SetInfeedPosAction(INFEED_TARGET_POSITION.SQUEEZE)
							})),
							new RunCarriageWheelsAction(true),
							new WaitAction(2)
					})),
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							//new WaitAction(0.2),
							new RunMotionProfileAction(fromPyramid)
					})),
					new DriveInfeedWheelsAction(),
					new RunCarriageWheelsAction(true)
		})));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(sTurnToSwitch),
							new SimultaneousAction(Arrays.asList(new Action[] {
					//			new RunCarriageWheelsAction(false),
								new WaitAction(0.5)
							}))
					})),
					new SetInfeedPosAction(Infeed.INFEED_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTimeSecondCube),
							new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.SWITCH_HEIGHT),
					}))
		}))); 
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new WaitAction(0.2),
				new OutfeedCubeAction()
		})));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.CUBE_ON_FLOOR));
		runAction(new PrintTimeFromStart(_startTime));  
	}
}