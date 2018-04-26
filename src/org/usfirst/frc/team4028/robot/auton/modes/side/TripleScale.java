package org.usfirst.frc.team4028.robot.auton.modes.side;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.Action;
import org.usfirst.frc.team4028.robot.auton.actions.ActuateFlapJackAction;
import org.usfirst.frc.team4028.robot.auton.actions.DriveSetDistanceAction;
import org.usfirst.frc.team4028.robot.auton.actions.InfeedCubeAction;
import org.usfirst.frc.team4028.robot.auton.actions.MoveElevatorToPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.OutfeedCubeAction;
import org.usfirst.frc.team4028.robot.auton.actions.PrintTimeFromStart;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.auton.actions.SeriesAction;
import org.usfirst.frc.team4028.robot.auton.actions.SetInfeedPosAction;
import org.usfirst.frc.team4028.robot.auton.actions.SimultaneousAction;
import org.usfirst.frc.team4028.robot.auton.actions.TurnAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitAction;
import org.usfirst.frc.team4028.robot.auton.actions.WaitUntilRemainingDistanceAction;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.util.control.Path;

public class TripleScale extends AutonBase {
	Path toScale;
	Path scaleToSwitch, switchToScale;
	Path scaleToSwitchThirdCube, switchToScaleThirdCube;
	
	double toSwitchDistance, toScaleAgainDistance;
	double targetTurnAngle, endTargetTurnAngle;
	double finalTurnTargetAngle;
	double elevatorWaitTime1, elevatorWaitTime2, elevatorWaitTime3;
	
	boolean isRightTurnToSwitch;
	
	public TripleScale(boolean isStartingLeft) {
		if (isStartingLeft) {
			toScale = Paths.getPath(Left.L_SCALE);
			scaleToSwitch = Paths.getPath(Left.L_SCALE_TO_L_SWITCH);
			switchToScale = Paths.getPath(Left.L_SWITCH_TO_L_SCALE);
			scaleToSwitchThirdCube = Paths.getPath(Left.L_SCALE_TO_L_SWITCH_THIRD_CUBE);
			switchToScaleThirdCube = Paths.getPath(Left.L_SWITCH_TO_L_SCALE_THIRD_CUBE);
			targetTurnAngle = 160;
			endTargetTurnAngle = 30;
			finalTurnTargetAngle = 144;
			elevatorWaitTime1 = 1.25;
			elevatorWaitTime2 = 0.5;
			elevatorWaitTime3 = 0.5;
			isRightTurnToSwitch = true;
		} else {
			toScale = Paths.getPath(Right.R_SCALE);
			scaleToSwitch = Paths.getPath(Right.R_SCALE_TO_R_SWITCH);
			switchToScale = Paths.getPath(Right.R_SWITCH_TO_R_SCALE);
			scaleToSwitchThirdCube = Paths.getPath(Right.R_SCALE_TO_R_SWITCH_THIRD_CUBE);
			switchToScaleThirdCube = Paths.getPath(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE);
			targetTurnAngle = -163;
			endTargetTurnAngle = -30;
			finalTurnTargetAngle = -144;
			elevatorWaitTime1 = 1.25;
			elevatorWaitTime2 = 0.5;
			elevatorWaitTime3 = 0.5;
			isRightTurnToSwitch = false;
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
						new WaitUntilRemainingDistanceAction(18),
						new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50)
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
											new WaitAction(0.7),
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
							new SeriesAction(Arrays.asList(new Action[] {
									new WaitAction(elevatorWaitTime2),
									new SimultaneousAction(Arrays.asList(new Action[] {
										new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT),
										new SeriesAction(Arrays.asList(new Action[] {
												new WaitAction(1.5),
												new SimultaneousAction(Arrays.asList(new Action[] {
														new ActuateFlapJackAction(true),
														new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50, 0.25)
												}))
												
										})),

									})) 
							}))
					}))
		}))); 
		runAction(new PrintTimeFromStart(_startTime));
		// Move elevator to floor
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
				new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT),
				new ActuateFlapJackAction(false),
				new SeriesAction(Arrays.asList(new Action[] {
						new TurnAction(finalTurnTargetAngle, isRightTurnToSwitch),
						new SimultaneousAction(Arrays.asList(new Action[] {
								new RunMotionProfileAction(scaleToSwitchThirdCube),
								new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.WIDE),
								new SeriesAction(Arrays.asList(new Action[] {
										new WaitAction(.75),
										new InfeedCubeAction()
								}))
						}))
				}))
		})));
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new SimultaneousAction(Arrays.asList(new Action[] {
					new SeriesAction(Arrays.asList(new Action[] {
							new RunMotionProfileAction(switchToScaleThirdCube),
							new TurnAction(30, !isRightTurnToSwitch)
					})),
					new SeriesAction(Arrays.asList(new Action[] {
							new WaitAction(elevatorWaitTime3),
							new SimultaneousAction(Arrays.asList(new Action[] {
									new ActuateFlapJackAction(true),
									new MoveElevatorToPosAction(Elevator.ELEVATOR_PRESET_POSITION.HIGH_SCALE_HEIGHT)
							}))
					})),
					new SetInfeedPosAction(INFEED_ARM_TARGET_POSITION.STORE),
		})));
		runAction(new OutfeedCubeAction(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60));		
		runAction(new PrintTimeFromStart(_startTime));
		runAction(new ActuateFlapJackAction(false));
		runAction(new DriveSetDistanceAction(-20));
		runAction(new MoveElevatorToPosAction(ELEVATOR_PRESET_POSITION.INFEED_HEIGHT));
		runAction(new PrintTimeFromStart(_startTime));
	}
}