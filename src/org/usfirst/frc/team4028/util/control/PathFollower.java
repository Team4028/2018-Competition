package org.usfirst.frc.team4028.util.control;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Twist;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileConstraints;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGoal;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGoal.CompletionBehavior;
import org.usfirst.frc.team4028.util.motionProfile.MotionState;
import org.usfirst.frc.team4028.util.motionProfile.ProfileFollower;

public class PathFollower {
	private static final double kReallyBigNumber = 1E6;

    AdaptivePurePursuitController mSteeringController;
    Twist mLastSteeringDelta;
    ProfileFollower mVelocityController;
    boolean overrideFinished = false;
    boolean doneSteering = false;

    /**
     * Create a new PathFollower for a given path.
     */
    public PathFollower(Path path, boolean reversed) {
        mSteeringController = new AdaptivePurePursuitController(path, reversed);
        mLastSteeringDelta = Twist.identity();
        mVelocityController = new ProfileFollower();
        mVelocityController.setConstraints(
                new MotionProfileConstraints(Constants.PATH_FOLLOWING_MAX_VEL, Constants.PATH_FOLLOWING_MAX_ACCEL));
    }

    /**
     * Get new velocity commands to follow the path.
     * 
     * @param t
     *            The current timestamp
     * @param pose
     *            The current robot pose
     * @param displacement
     *            The current robot displacement (total distance driven).
     * @param velocity
     *            The current robot velocity.
     * @return The velocity command to apply
     */
    public synchronized Twist update(double t, RigidTransform pose, double displacement, double velocity) {
        if (!mSteeringController.isFinished()) {
            final AdaptivePurePursuitController.Command steering_command = mSteeringController.update(pose);
            mLastSteeringDelta = steering_command.delta;
            mVelocityController.setGoalAndConstraints(
                    new MotionProfileGoal(displacement + steering_command.delta.dx,
                            Math.abs(steering_command.end_velocity), CompletionBehavior.VIOLATE_MAX_ACCEL,
                            Constants.PATH_FOLLOWING_GOAL_POS_TOLERANCE, Constants.PATH_FOLLOWING_GOAL_VEL_TOLERANCE),
                    new MotionProfileConstraints(Math.min(Constants.PATH_FOLLOWING_MAX_VEL, steering_command.max_velocity),
                            Constants.PATH_FOLLOWING_MAX_ACCEL));

            if (steering_command.remaining_path_length < Constants.PATH_STOP_STEERING_DISTANCE) {
                doneSteering = true;
            }
        }

        final double velocity_command = mVelocityController.update(new MotionState(t, displacement, velocity, 0.0), t);
        final double curvature = mLastSteeringDelta.dtheta / mLastSteeringDelta.dx;
        double dtheta = mLastSteeringDelta.dtheta;
        if (!Double.isNaN(curvature) && Math.abs(curvature) < kReallyBigNumber) {
            // Regenerate angular velocity command from adjusted curvature.
            final double abs_velocity_setpoint = Math.abs(mVelocityController.getSetpoint().vel());
            dtheta = mLastSteeringDelta.dx * curvature * (1.0 + Constants.INERTIA_STEERING_GAIN * abs_velocity_setpoint);
        }
        final double scale = velocity_command / mLastSteeringDelta.dx;
        final Twist rv = new Twist(mLastSteeringDelta.dx * scale, 0.0, dtheta * scale);

        return rv;
    }
    
    public boolean isFinished() {
        return (mSteeringController.isFinished() && mVelocityController.isFinishedProfile()
                && mVelocityController.onTarget()) || overrideFinished;
    }

    public void forceFinish() {
        overrideFinished = true;
    }
}