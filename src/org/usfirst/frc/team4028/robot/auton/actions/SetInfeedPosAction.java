package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler2;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

public class SetInfeedPosAction implements Action{
	Infeed _infeed = Infeed.getInstance();
	INFEED_ARM_TARGET_POSITION _infeedTargetPos;
	CubeHandler2 _cubeHandler = CubeHandler2.getInstance();
	
	public SetInfeedPosAction(INFEED_ARM_TARGET_POSITION infeedTargetPos) {
		_infeedTargetPos = infeedTargetPos;
	} 

	@Override
	public void start() {}

	@Override
	public void update() {
		_infeed.MoveToPresetPosition(_infeedTargetPos);
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}