package org.usfirst.frc.team4028.util.motionProfile;

import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGoal.CompletionBehavior;

public class MotionProfileGenerator {
	 // Static class.
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
    /*
    public synchronized static MotionProfile generateProfile(MotionProfileConstraints constraints,
            MotionProfileGoal goal_state,
            MotionState prev_state) {
        double delta_pos = goal_state.pos() - prev_state.pos();
        if (delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.vel() < 0.0)) {
            // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
            // flip the solution.
            return generateFlippedProfile(constraints, goal_state, prev_state);
        }
        // Invariant from this point on: delta_pos >= 0.0
        // Clamp the start state to be valid.
        MotionState start_state = new MotionState(prev_state.t(), prev_state.pos(),
                Math.signum(prev_state.vel()) * Math.min(Math.abs(prev_state.vel()), constraints.maxAbsVel()),
                Math.signum(prev_state.acc()) * Math.min(Math.abs(prev_state.acc()), constraints.maxAcc));
        MotionProfile profile = new MotionProfile();
        profile.reset(start_state);
        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (start_state.vel() < 0.0 && delta_pos > 0.0) {
            final double stopping_time = Math.abs(start_state.vel() / constraints.maxAcc);
            profile.appendControl(constraints.maxAcc, stopping_time);
            start_state = profile.endState();
            delta_pos = goal_state.pos() - start_state.pos();
        }
        // Invariant from this point on: start_state.vel() >= 0.0
        final double min_abs_vel_at_goal_sqr = start_state.vel2() - 2.0 * constraints.maxAcc() * delta_pos;
        final double min_abs_vel_at_goal = Math.sqrt(Math.abs(min_abs_vel_at_goal_sqr));
        final double max_abs_vel_at_goal = Math.sqrt(start_state.vel2() + 2.0 * constraints.maxAcc() * delta_pos);
        double goal_vel = goal_state.max_abs_vel();
        double max_acc = constraints.maxAcc;
        if (min_abs_vel_at_goal_sqr > 0.0
                && min_abs_vel_at_goal > (goal_state.max_abs_vel() + goal_state.vel_tolerance())) {
            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
                // Adjust the goal velocity.
                goal_vel = min_abs_vel_at_goal;
            } else if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                if (Math.abs(delta_pos) < goal_state.pos_tolerance()) {
                    // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                    // which will result in NaNs below, so we can return the profile immediately.
                    profile.appendSegment(new MotionSegment(
                            new MotionState(profile.endTime(), profile.endPos(), profile.endState().vel(),
                                    Double.NEGATIVE_INFINITY),
                            new MotionState(profile.endTime(), profile.endPos(), goal_vel, Double.NEGATIVE_INFINITY)));
                    profile.consolidate();
                    return profile;
                }
                // Adjust the max acceleration.
                max_acc = Math.abs(goal_vel * goal_vel - start_state.vel2()) / (2.0 * delta_pos);
            } else {
                // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                final double stopping_time = Math.abs(start_state.vel() / constraints.maxAcc);
                profile.appendControl(-constraints.maxAcc, stopping_time);
                // Now we need to travel backwards, so generate a flipped profile.
                profile.appendProfile(generateFlippedProfile(constraints, goal_state, profile.endState()));
                profile.consolidate();
                return profile;
            }
        }
        goal_vel = Math.min(goal_vel, max_abs_vel_at_goal);
        // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
        // max_acc.

        // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
        // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
        // is greater than constraints.max_abs_vel, we will clamp and cruise.
        // Solve the following three equations to find Vmax (by substitution):
        // Vmax^2 = Vstart^2 + 2*a*d_accel
        // Vgoal^2 = Vmax^2 - 2*a*d_decel
        // delta_pos = d_accel + d_decel
        final double v_max = Math.min(constraints.maxAbsVel(),
                Math.sqrt((start_state.vel2() + goal_vel * goal_vel) / 2.0 + delta_pos * max_acc));

        // Accelerate to v_max
        if (v_max > start_state.vel()) {
            final double accel_time = (v_max - start_state.vel()) / max_acc;
            profile.appendControl(max_acc, accel_time);
            start_state = profile.endState();
        }
        // Figure out how much distance will be covered during deceleration.
        final double distance_decel = Math.max(0.0,
                (start_state.vel2() - goal_vel * goal_vel) / (2.0 * constraints.maxAcc));
        final double distance_cruise = Math.max(0.0, goal_state.pos() - start_state.pos() - distance_decel);
        // Cruise at constant velocity.
        if (distance_cruise > 0.0) {
            final double cruise_time = distance_cruise / start_state.vel();
            profile.appendControl(0.0, cruise_time);
            start_state = profile.endState();
        }
        // Decelerate to goal velocity.
        if (distance_decel > 0.0) {
            final double decel_time = (start_state.vel() - goal_vel) / max_acc;
            profile.appendControl(-max_acc, decel_time);
        }

        profile.consolidate();
        return profile;
    }
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
                Math.signum(prevState.vel()) * Math.min(Math.abs(prevState.vel()), constraints.maxAbsVel()),
                getMaxAccDec(prevState.acc(), constraints));
        MotionProfile profile = new MotionProfile();
        profile.reset(startState);
        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (startState.vel() < 0.0 && deltaPos > 0.0) {
            final double stopping_time = Math.abs(startState.vel() / constraints.maxDecel());
            profile.appendControl(constraints.maxDecel(), stopping_time);
            startState = profile.endState();
            deltaPos = goalState.pos() - startState.pos();
        }
        // Invariant from this point on: start_state.vel() >= 0.0
        final double minAbsVelAtGoalSqr = startState.vel2() - 2.0 * constraints.maxDecel() * deltaPos;
        final double minAbsVelAtGoal = Math.sqrt(Math.abs(minAbsVelAtGoalSqr));
        final double maxAbsVelAtGoal = Math.sqrt(startState.vel2() + 2.0 * constraints.maxAcc() * deltaPos);
        double goalVel = goalState.max_abs_vel();
        double maxAcc = constraints.maxAcc;
        double maxDecel = constraints.maxDecel;
        if (minAbsVelAtGoalSqr > 0.0
                && minAbsVelAtGoal > (goalState.max_abs_vel() + goalState.vel_tolerance())) {
            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goalState.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {
                // Adjust the goal velocity.
                goalVel = minAbsVelAtGoal;
            } else if (goalState.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                if (Math.abs(deltaPos) < goalState.pos_tolerance()) {
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
                final double stopping_time = Math.abs(startState.vel() / constraints.maxDecel());
                profile.appendControl(-constraints.maxDecel(), stopping_time);
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
        final double vMax = Math.min(constraints.maxAbsVel(),
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