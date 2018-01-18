package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.util.control.Path;

import edu.wpi.first.wpilibj.Timer;

public class ResetPoseFromPathAction implements Action {
	protected Path _path;

    public ResetPoseFromPathAction(Path path) {
        _path = path;
    }

    @Override
    public synchronized void done() {
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), _path.getStartPose());
    }

	@Override
	public void start() {}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}