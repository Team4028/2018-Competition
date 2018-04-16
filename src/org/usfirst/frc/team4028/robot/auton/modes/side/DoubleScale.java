package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase{
	Path toScale;
	Path scaleToSwitch, switchToScale;
	
	double toScaleRemainingDistance, toSwitchDistance, toScaleAgainDistance;
	double targetTurnAngle, endTargetTurnAngle;
	double finalTurnTargetAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	CARRIAGE_WHEELS_OUT_VBUS_INDEX _carriageVBUSCube1, carriageVBUSCube2;
	
	boolean isRightTurnToSwitch;
	
	public DoubleScale(boolean isLeftScale, boolean isStartingLeft) {
		if (isLeftScale) {
			if (isStartingLeft) {
				toScale = Paths.getPath(Left.L_SCALE);
				scaleToSwitch = Paths.getPath(Left.L_SCALE_TO_L_SWITCH);
				switchToScale = Paths.getPath(Left.L_SWITCH_TO_L_SCALE);
				_carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_90;
				toScaleRemainingDistance = 18;
				toSwitchDistance = 40.0;
				toScaleAgainDistance = -41.0;
				targetTurnAngle = 163;
				endTargetTurnAngle = 30;
				finalTurnTargetAngle = 144;
				elevatorWaitTime1 = 1.25;
				elevatorWaitTime2 = 0.9;
				isRightTurnToSwitch = true;
			} else {
				toScale = Paths.getPath(Right.L_SCALE); 
				toSwitchDistance = 37.0;
				toScaleAgainDistance = -37.0;
				targetTurnAngle = 168;
				endTargetTurnAngle = 15;
				finalTurnTargetAngle = 135;
				elevatorWaitTime1 = 4.0;
				elevatorWaitTime2 = 1.0;
				isRightTurnToSwitch = false;
			}
		} else {
			if (isStartingLeft) {
				toScale = Paths.getPath(Left.R_SCALE);
				scaleToSwitch = Paths.getPath(Left.R_SCALE_TO_R_SWITCH);
				switchToScale = Paths.getPath(Left.R_SWITCH_TO_R_SCALE);
				_carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
				toScaleRemainingDistance = 12;
				toSwitchDistance = 37.0;
				toScaleAgainDistance = -38.0;
				targetTurnAngle = -160;
				endTargetTurnAngle = -10.0;
				finalTurnTargetAngle = -135;
				elevatorWaitTime1 = 3.7;
				elevatorWaitTime2 = 1.0;
				isRightTurnToSwitch = true;
			} else {
				toScale = Paths.getPath(Right.R_SCALE);
				toSwitchDistance = 40.0;
				toScaleAgainDistance = -40.0;
				targetTurnAngle = -165;
				endTargetTurnAngle = -25;
				finalTurnTargetAngle = -135;
				elevatorWaitTime1 = 2.0;
				elevatorWaitTime2 = 0.5;
				isRightTurnToSwitch = false;
			}
		}
	}

	@Override
	public void routine() {
		// Drive to scale while storing infeed and raising elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new RunMotionProfileAction(toScale),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(elevatorWaitTime1),
						new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT),
						new WaitUntilRemainingDistanceAction(toScaleRemainingDistance),
						new OutfeedCubeAction(_carriageVBUSCube1)
				}))
		})));
		// Outfeed cube for 0.2s
		runAction(new PrintTimeFromStart(_startTime));
		// Lower Elevator to Switch during turn, then drive to 2nd cube while setting infeed wide and continuing to lower elevator
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
					new SeriesAction(Arrays.asList(new Action[] {
							new TurnAction(targetTurnAngle, isRightTurnToSwitch),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
									new RunMotionProfileAction(scaleToSwitch),
									new SeriesAction(Arrays.asList(new Action [] {
											new WaitAction(0.55),
											new InfeedCubeAction()
									}))
							}))
					}))
		})));
		// Infeed cube while sitting in place
		//runAction(new InfeedCubeAction());
		// Drive back to scale and turn while raising elevator to scale height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(switchToScale),
							new TurnAction(endTargetTurnAngle, !isRightTurnToSwitch),
							//new PrintTimeFromStart(_startTime)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime2),
							new SimultaneousAction(Arrays.asList(new Action[] {
								new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT),
								new SeriesAction(Arrays.asList(new Action[] {
										new WaitAction(1.3),
										//new PrintTimeFromStart(_startTime),
										new OutfeedCubeAction(carriageVBUSCube2)
								})),

							})) 
					}))
		}))); 
		// Outfeed cube for 0.2s
		//runAction(new OutfeedCubeAction(carriageVBUSCube2));
		runAction(new PrintTimeFromStart(_startTime));
		// Move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.3),
						new TurnAction(finalTurnTargetAngle, isRightTurnToSwitch),
						/*new SimultaneousAction(Arrays.asList(new Action[] {
								new DriveSetDistanceAction(50.0),
								new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
						}))*/
				}))
		})));
		/*runAction(new InfeedCubeAction());
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new DriveSetDistanceAction(-50.0),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		runAction(new TurnAction(0.0, false));*/
		runAction(new PrintTimeFromStart(_startTime));
	}
}