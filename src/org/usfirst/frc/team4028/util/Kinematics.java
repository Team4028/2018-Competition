package org.usfirst.frc.team4028.util;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Twist;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */
public class Kinematics {
	private static final double kEpsilon = 1E-9;

    /** Using only encoders, rotation is implicit (less accurate than below, but useful for predicting motion) */
	public static Twist forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double delta_v = (right_wheel_delta - left_wheel_delta) / 2 * Constants.TRACK_SCRUBBING_FACTOR;
        double delta_rotation = delta_v * 2 / Constants.TRACK_WIDTH_INCHES;
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    /** Using encoders and explicitly measured rotation (ex. from gyro) */
    public static Twist forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist(dx, 0, delta_rotation_rads);
    }

    /** With absolute rotation and previous rotation. */
    public static Twist forwardKinematics(Rotation prev_heading, double left_wheel_delta, double right_wheel_delta,
            Rotation current_heading) {
        return forwardKinematics(left_wheel_delta, right_wheel_delta,
                prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /** Integrate forward kinematics with a Twist2d and previous rotation. */
    public static RigidTransform integrateForwardKinematics(RigidTransform current_pose,
            Twist forward_kinematics) {
        return current_pose.transformBy(RigidTransform.exp(forward_kinematics));
    }

    /** Class that contains left and right wheel velocities */
    public static class DriveVelocity {
        public final double left, right;

        public DriveVelocity(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    /** Uses inverse kinematics to convert a Twist2d into left and right wheel velocities */
    public static DriveVelocity inverseKinematics(Twist velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveVelocity(velocity.dx, velocity.dx);
        }
        double delta_v = Constants.TRACK_WIDTH_INCHES * velocity.dtheta / (2 * Constants.TRACK_SCRUBBING_FACTOR);
        return new DriveVelocity(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}