package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_TARGET_POSITION;

public class SetInfeedPosAction implements Action{
	Infeed _infeed = Infeed.getInstance();
	INFEED_TARGET_POSITION _infeedTargetPos;
	
	public SetInfeedPosAction(INFEED_TARGET_POSITION infeedTargetPos) {
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