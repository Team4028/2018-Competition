package org.usfirst.frc.team4028.util.motionProfile;

import static org.usfirst.frc.team4028.util.GeneralUtilities.epsilonEquals;

public class MotionProfileGoal {
	/**
     * A goal consists of a desired position and specified maximum velocity magnitude. But what should we do if we would
     * reach the goal at a velocity greater than the maximum? This enum allows a user to specify a preference on
     * behavior in this case.
     * 
     * Example use-cases of each:
     * 
     * OVERSHOOT - Generally used with a goal max_abs_vel of 0.0 to stop at the desired pos without violating any
     * constraints.
     * 
     * VIOLATE_MAX_ACCEL - If we absolutely do not want to pass the goal and are unwilling to violate the max_abs_vel
     * (for example, there is an obstacle in front of us - slam the brakes harder than we'd like in order to avoid
     * hitting it).
     * 
     * VIOLATE_MAX_ABS_VEL - If the max velocity is just a general guideline and not a hard performance limit, it's
     * better to slightly exceed it to avoid skidding wheels.
     */
	public static enum CompletionBehavior {
        // Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back.
        // Only valid if the goal velocity is 0.0 (otherwise VIOLATE_MAX_ACCEL will be used).
        OVERSHOOT,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max accel
        // constraint.
        VIOLATE_MAX_ACCEL,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the goal velocity.
        VIOLATE_MAX_ABS_VEL
    }
	
	protected double pos;
	protected double maxAbsVel;
	protected CompletionBehavior completionBehavior = CompletionBehavior.OVERSHOOT;
	protected double posTolerance = 1E-3;
	protected double velTolerance = 1E-2;
	
	public MotionProfileGoal(double pos, double max_abs_vel) {
        this.pos = pos;
        this.maxAbsVel = max_abs_vel;
        sanityCheck();
	}

    public MotionProfileGoal(double pos, double max_abs_vel, CompletionBehavior completion_behavior,
            double pos_tolerance, double vel_tolerance) {
        this.pos = pos;
        this.maxAbsVel = max_abs_vel;
        this.completionBehavior = completion_behavior;
        this.posTolerance = pos_tolerance;
        this.velTolerance = vel_tolerance;
        sanityCheck();
    }
    
    public MotionProfileGoal(MotionProfileGoal other) {
        this(other.pos, other.maxAbsVel, other.completionBehavior, other.posTolerance, other.velTolerance);
    }
    
    /** @return A flipped MotionProfileGoal (where the position is negated, but all other attributes remain the same) */
    public MotionProfileGoal flipped() {
        return new MotionProfileGoal(-pos, maxAbsVel, completionBehavior, posTolerance, velTolerance);
    }
    
    public double pos() {
    	return pos;
    }
    
    public double getMaxAbsVel() {
    	return maxAbsVel;
    }
    
    public double getPosTolerance() {
    	return posTolerance;
    }
    
    public double getVelTolerance() {
    	return velTolerance;
    }
    
    public CompletionBehavior getCompletionBehavior() {
    	return completionBehavior;
    }
    
    public boolean atGoalState(MotionState state) {
    	return atGoalPos(state.pos()) && (Math.abs(state.vel()) < (maxAbsVel + velTolerance)
                || completionBehavior == CompletionBehavior.VIOLATE_MAX_ABS_VEL);
    }
    
    public boolean atGoalPos(double pos) {
        return epsilonEquals(pos, this.pos, posTolerance);
    }
    
    /** This method makes sure that the completion behavior is compatible with the max goal velocity. */
    protected void sanityCheck() {
        if (maxAbsVel > velTolerance && completionBehavior == CompletionBehavior.OVERSHOOT) {
            completionBehavior = CompletionBehavior.VIOLATE_MAX_ACCEL;
        }
    }
}