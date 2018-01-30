package org.usfirst.frc.team4028.util.motionProfile;

/** Constraints for constructing a MotionProfile */
public class MotionProfileConstraints {
	protected double maxAbsVel = Double.POSITIVE_INFINITY;
	protected double maxAcc = Double.POSITIVE_INFINITY;
	protected double maxDecel = Double.POSITIVE_INFINITY;
	
	public MotionProfileConstraints(double max_vel, double max_acc, double max_decel) {
		this.maxAcc = max_acc;
		this.maxAbsVel = Math.abs(max_vel);
		this.maxDecel = max_decel;
	}
	
	/** @return The (positive) maximum allowed velocity */
    public double maxAbsVel() {
        return maxAbsVel;
    }

    /** @return The (positive) maximum allowed acceleration */
    public double maxAcc() {
        return maxAcc;
    }
    
    public double maxDecel() {
    	return maxDecel;
    }
}