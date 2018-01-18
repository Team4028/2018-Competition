package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;

import edu.wpi.first.wpilibj.Timer;

public class ResetPoseReverse implements Action{
	protected Path _path;

    public ResetPoseReverse(Path pathContainer) {
        _path = pathContainer;
    }

    @Override
    public synchronized void done() {
        RigidTransform startPose = _path.getStartPose();
        //Chassis.getInstance().zeroGyro();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        Chassis.getInstance().setGyroAngle(startPose.getRotation().getDegrees());
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