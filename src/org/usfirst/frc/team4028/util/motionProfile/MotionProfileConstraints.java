package org.usfirst.frc.team4028.util.motionProfile;

/** Constraints for constructing a MotionProfile */
public class MotionProfileConstraints {
	protected double max_abs_vel = Double.POSITIVE_INFINITY;
	protected double max_abs_acc = Double.POSITIVE_INFINITY;
	
	public MotionProfileConstraints(double max_vel, double max_acc) {
		this.max_abs_acc = Math.abs(max_acc);
		this.max_abs_vel = Math.abs(max_vel);
	}
	
	/** @return The (positive) maximum allowed velocity */
    public double max_abs_vel() {
        return max_abs_vel;
    }

    /** @return The (positive) maximum allowed acceleration */
    public double max_abs_acc() {
        return max_abs_acc;
    }
}