package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.robot.paths.PathContainer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.motion.RigidTransform;

import edu.wpi.first.wpilibj.Timer;

public class ResetPoseFromPathAction implements Action {
	protected PathContainer _pathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        _pathContainer = pathContainer;
    }

    @Override
    public synchronized void done() {
        RigidTransform startPose = _pathContainer.getStartPose();
        //Chassis.getInstance().zeroGyro();
        //Chassis.getInstance().setGyroAngle(0.0);
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        //Chassis.getInstance().setGyroAngle(startPose.getRotation().getDegrees());
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