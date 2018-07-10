package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class DoubleScale extends AutonBase{
	Path toScale;
	Path scaleToSwitch, switchToScale;
	Path scaleToSwitchThirdCube, switchToScaleThirdCube;
	
	double toScaleRemainingDistance, toSwitchDistance, toScaleAgainDistance;
	double targetTurnAngle, endTargetTurnAngle;
	double finalTurnTargetAngle;
	double elevatorWaitTime1, elevatorWaitTime2;
	CARRIAGE_WHEELS_OUT_VBUS_INDEX carriageVBUSCube1, carriageVBUSCube2;
	
	boolean isRightTurnToSwitch;
	boolean actuateFlapJack;
	
	public DoubleScale(boolean isLeftScale, boolean isStartingLeft) {
		if (isLeftScale) {
			if (isStartingLeft) {
				toScale = Paths.getPath(Left.L_SCALE);
				carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_40;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				scaleToSwitch = Paths.getPath(Left.L_SCALE_TO_L_SWITCH_EXP);//Paths.getPath(Left.L_SCALE_TO_L_SWITCH);
				switchToScale = Paths.getPath(Left.L_SWITCH_TO_L_SCALE_EXP);//Paths.getPath(Left.L_SWITCH_TO_L_SCALE);
				scaleToSwitchThirdCube = Paths.getPath(Left.L_SCALE_TO_L_SWITCH_THIRD_CUBE);
				switchToScaleThirdCube = Paths.getPath(Left.L_SWITCH_TO_L_SCALE_THIRD_CUBE);
				toScaleRemainingDistance = 18;
				elevatorWaitTime1 = 1.25;
				elevatorWaitTime2 = 0.7;
				isRightTurnToSwitch = true;
				actuateFlapJack = false;
			} else {
				toScale = Paths.getPath(Right.L_SCALE); 
				carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
				scaleToSwitch = Paths.getPath(Right.L_SCALE_TO_L_SWITCH);
				switchToScale = Paths.getPath(Right.L_SWITCH_TO_L_SCALE);
				scaleToSwitchThirdCube = Paths.getPath(Left.L_SCALE_TO_L_SWITCH_THIRD_CUBE);
				switchToScaleThirdCube = Paths.getPath(Left.L_SWITCH_TO_L_SCALE_THIRD_CUBE);
				toScaleRemainingDistance = 12;
				elevatorWaitTime1 = 3.2;
				elevatorWaitTime2 = 0.9;
				isRightTurnToSwitch = false;
				actuateFlapJack = true;
			}
			targetTurnAngle = 150;//was 160
			endTargetTurnAngle = 30;
			finalTurnTargetAngle = 144;
		} else {
			if (isStartingLeft) {
				toScale = Paths.getPath(Left.R_SCALE);
				carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
				scaleToSwitch = Paths.getPath(Left.R_SCALE_TO_R_SWITCH);
				switchToScale = Paths.getPath(Left.R_SWITCH_TO_R_SCALE);
				scaleToSwitchThirdCube = Paths.getPath(Right.R_SCALE_TO_R_SWITCH_THIRD_CUBE);
				switchToScaleThirdCube = Paths.getPath(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE);
				toScaleRemainingDistance = 12;
				targetTurnAngle = -130;
				endTargetTurnAngle = -10.0;
				finalTurnTargetAngle = -144;
				elevatorWaitTime1 = 3.2;
				elevatorWaitTime2 = 0.9;
				isRightTurnToSwitch = true;
				actuateFlapJack = true;
			} else {
				toScale = Paths.getPath(Right.R_SCALE);
				carriageVBUSCube1 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				carriageVBUSCube2 = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				scaleToSwitch = Paths.getPath(Left.R_SCALE_TO_R_SWITCH);
				switchToScale = Paths.getPath(Left.R_SWITCH_TO_R_SCALE);
				targetTurnAngle = -160;
				endTargetTurnAngle = -25;
				finalTurnTargetAngle = -135;
				elevatorWaitTime1 = 1.25;
				elevatorWaitTime2 = 0.5;
				isRightTurnToSwitch = false;
				actuateFlapJack = false;
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
						new SimultaneousAction(Arrays.asList(new Action[] {
								new ActuateFlapJackAction(true),
								new MoveElevatorToPosAction(Elevator.ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
						})),
						new WaitUntilRemainingDistanceAction(toScaleRemainingDistance),
						new OutfeedCubeAction(carriageVBUSCube1,0.3)
				}))
		})));
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
											new WaitAction(1.25),
											new InfeedCubeAction()
									}))
							}))
					}))
		})));
		// Drive back to scale and turn while raising elevator to scale height
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(switchToScale),
							new TurnAction(endTargetTurnAngle, !isRightTurnToSwitch)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime2),
							new SimultaneousAction(Arrays.asList(new Action[] {
								new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT),
								new ActuateFlapJackAction(true)
							})) 
					}))
		}))); 
		runAction(new OutfeedCubeAction());
		// Outfeed cube for 0.2s
		runAction(new DriveSetDistanceAction(-20));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
		runAction(new PrintTimeFromStart(_startTime));
		// Move elevator to floor
		/*runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new ActuateFlapJackAction(false),
				new SeriesAction(Arrays.asList(new Action[] {
						new WaitAction(0.3),
						new TurnAction(finalTurnTargetAngle, isRightTurnToSwitch),
						new SimultaneousAction(Arrays.asList(new Action[] {
								new RunMotionProfileAction(scaleToSwitchThirdCube),
								new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE)
						}))
				}))
		})));
		runAction(new InfeedCubeAction());
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new RunMotionProfileAction(switchToScaleThirdCube),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE)
		})));
		runAction(new TurnAction(0.0, false));
		runAction(new PrintTimeFromStart(_startTime));*/
	}
}