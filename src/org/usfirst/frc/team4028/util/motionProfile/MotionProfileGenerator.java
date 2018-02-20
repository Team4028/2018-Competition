package org.usfirst.frc.team4028.util.motionProfile;

import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGoal.CompletionBehavior;

public class MotionProfileGenerator {
    private MotionProfileGenerator() {}

    protected static MotionProfile generateFlippedProfile(MotionProfileConstraints constraints,
            MotionProfileGoal goal_state, MotionState prev_state) {
        MotionProfile profile = generateProfile(constraints, goal_state.flipped(), prev_state.flipped());
        for (MotionSegment s : profile.segments()) {
            s.setStart(s.start().flipped());
            s.setEnd(s.end().flipped());
        }
        return profile;
    }

    /**
     * Generate a motion profile.
     * 
     * @param constraints
     *            The constraints to use.
     * @param goal_state
     *            The goal to use.
     * @param prev_state
     *            The initial state to use.
     * @return A motion profile from prev_state to goal_state that satisfies constraints.
     */
    public synchronized static MotionProfile generateProfile(MotionProfileConstraints constraints,
            MotionProfileGoal goalState,
            MotionState prevState) {
    	double deltaPos = goalState.pos() - prevState.pos(); 
    	if (deltaPos < 0.0 || (deltaPos == 0.0 && prevState.vel() < 0.0)) {
            // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
            // flip the solution.
            return generateFlippedProfile(constraints, goalState, prevState);
        }
    	// Invariant from this point on: deltaPos >= 0.0
        // Clamp the start state to be valid.
        MotionState startState = new MotionState(prevState.t(), prevState.pos(),
                Math.signum(prevState.vel()) * Math.min(Math.abs(prevState.vel()), constraints.maxAbsVel),
                getMaxAccDec(prevState.acc(), constraints));
        MotionProfile profile = new MotionProfile();
        profile.reset(startState);
        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (startState.vel() < 0.0 && deltaPos > 0.0) {
            final double stopping_time = Math.abs(startState.vel() / constraints.maxDecel);
            profile.appendControl(constraints.maxDecel, stopping_time);
            startState = profile.endState();
            deltaPos = goalState.pos() - startState.pos();
        }
        // Invariant from this point on: start_state.vel() >= 0.0
        final double minAbsVelAtGoalSqr = startState.vel2() - 2.0 * constraints.maxDecel * deltaPos;
        final double minAbsVelAtGoal = Math.sqrt(Math.abs(minAbsVelAtGoalSqr));
        final double maxAbsVelAtGoal = Math.sqrt(startState.vel2() + 2.0 * constraints.maxAcc * deltaPos);
        double goalVel = goalState.getMaxAbsVel();
        double maxAcc = constraints.maxAcc;
        double maxDecel = constraints.maxDecel;
        if (minAbsVelAtGoalSqr > 0.0
                && minAbsVelAtGoal > (goalState.getMaxAbsVel() + goalState.getVelTolerance())) {
            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goalState.getCompletionBehavior() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
                // Adjust the goal velocity.
                goalVel = minAbsVelAtGoal;
            } else if (goalState.getCompletionBehavior() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                if (Math.abs(deltaPos) < goalState.getPosTolerance()) {
                    // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                    // which will result in NaNs below, so we can return the profile immediately.
                    profile.appendSegment(new MotionSegment(
                            new MotionState(profile.endTime(), profile.endPos(), profile.endState().vel(),
                                    Double.NEGATIVE_INFINITY),
                            new MotionState(profile.endTime(), profile.endPos(), goalVel, Double.NEGATIVE_INFINITY)));
                    profile.consolidate();
                    return profile;
                }
                // Adjust the max deceleration.
                maxDecel = Math.abs(goalVel * goalVel - startState.vel2()) / (2.0 * deltaPos);
            } else {
                // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                final double stopping_time = Math.abs(startState.vel() / constraints.maxDecel);
                profile.appendControl(-constraints.maxDecel, stopping_time);
                // Now we need to travel backwards, so generate a flipped profile.
                profile.appendProfile(generateFlippedProfile(constraints, goalState, profile.endState()));
                profile.consolidate();
                return profile;
            }
        }
        goalVel = Math.min(goalVel, maxAbsVelAtGoal);
        // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
        // max_acc.

        // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
        // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
        // is greater than constraints.max_abs_vel, we will clamp and cruise.
        // Solve the following three equations to find Vmax (by substitution):
        // Vmax^2 = Vstart^2 + 2*a*d_accel
        // Vgoal^2 = Vmax^2 - 2*a*d_decel
        // delta_pos = d_accel + d_decel
        final double vMax = Math.min(constraints.maxAbsVel,
                Math.sqrt((startState.vel2() + goalVel * goalVel) / 2.0 + deltaPos * maxAcc));

        // Accelerate to v_max
        if (vMax > startState.vel()) {
            final double accelTime = (vMax - startState.vel()) / maxAcc;
            profile.appendControl(maxAcc, accelTime);
            startState = profile.endState();
        }
        // Figure out how much distance will be covered during deceleration.
        final double distanceDecel = Math.max(0.0,
                (startState.vel2() - goalVel * goalVel) / (2.0 * maxDecel));
        final double distanceCruise = Math.max(0.0, goalState.pos() - startState.pos() - distanceDecel);
        //System.out.println(distanceDecel);
        // Cruise at constant velocity.
        if (distanceCruise > 0.0) {
            final double cruise_time = distanceCruise / startState.vel();
            profile.appendControl(0.0, cruise_time);
            startState = profile.endState();
        }
        // Decelerate to goal velocity.
        if (distanceDecel > 0.0) {
            final double decelTime = (startState.vel() - goalVel) / maxDecel;
            profile.appendControl(-maxDecel, decelTime);
        }
        
        profile.consolidate();
        return profile;
    }
    
    private static double getMaxAccDec(double prevStateAcc, MotionProfileConstraints constraints) {
    	if (Math.signum(prevStateAcc) == 1.0) {
    		return Math.min(prevStateAcc, constraints.maxAcc);
    	} 
    	else if (Math.signum(prevStateAcc) == -1.0) {
    		return -1.0 * Math.max(prevStateAcc, constraints.maxDecel);
    	} else {
    		return 0.0;
    	}
    }
}