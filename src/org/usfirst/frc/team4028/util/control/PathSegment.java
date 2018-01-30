package org.usfirst.frc.team4028.util.control;

import java.util.Optional;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfile;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileConstraints;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGenerator;
import org.usfirst.frc.team4028.util.motionProfile.MotionProfileGoal;
import org.usfirst.frc.team4028.util.motionProfile.MotionState;

public class PathSegment {
	private Translation start;
	private Translation end;
	private Translation center;
	private Translation deltaStart;
	private Translation deltaEnd;
	private double maxSpeed;
	private boolean isLine;
	private MotionProfile speedController;
	private boolean extrapolateLookahead;
	private String marker;
	
	/**
     * Constructor for a linear segment
     * 
     * @param x1
     *            start x
     * @param y1
     *            start y
     * @param x2
     *            end x
     * @param y2
     *            end y
     * @param maxSpeed
     *            maximum speed allowed on the segment
     */
    public PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState,
            double endSpeed) {
        this.start = new Translation(x1, y1);
        this.end = new Translation(x2, y2);

        this.deltaStart = new Translation(start, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = true;
        createMotionProfiler(startState, endSpeed);
    }

    public PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState,
            double endSpeed, String marker) {
        this.start = new Translation(x1, y1);
        this.end = new Translation(x2, y2);

        this.deltaStart = new Translation(start, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = true;
        this.marker = marker;
        createMotionProfiler(startState, endSpeed);
    }

    /**
     * Constructor for an arc segment
     * 
     * @param x1
     *            start x
     * @param y1
     *            start y
     * @param x2
     *            end x
     * @param y2
     *            end y
     * @param cx
     *            center x
     * @param cy
     *            center y
     * @param maxSpeed
     *            maximum speed allowed on the segment
     */
    public PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
            MotionState startState, double endSpeed) {
        this.start = new Translation(x1, y1);
        this.end = new Translation(x2, y2);
        this.center = new Translation(cx, cy);

        this.deltaStart = new Translation(center, start);
        this.deltaEnd = new Translation(center, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = false;
        createMotionProfiler(startState, endSpeed);
    }

    public PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
            MotionState startState, double endSpeed, String marker) {
        this.start = new Translation(x1, y1);
        this.end = new Translation(x2, y2);
        this.center = new Translation(cx, cy);

        this.deltaStart = new Translation(center, start);
        this.deltaEnd = new Translation(center, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = false;
        this.marker = marker;
        createMotionProfiler(startState, endSpeed);
    }
    
    public double getMaxSpeed() {
    	return maxSpeed;
    }
    
    public void createMotionProfiler(MotionState start_state, double end_speed) {
    	MotionProfileConstraints motionConstraints = new MotionProfileConstraints(maxSpeed, 
    			Constants.PATH_FOLLOWING_MAX_ACCEL, Constants.PATH_FOLLOWING_MAX_DECEL);
    	MotionProfileGoal goal_state = new MotionProfileGoal(getLength(), end_speed);
    	speedController = MotionProfileGenerator.generateProfile(motionConstraints, goal_state, start_state);
    }
    
    /** @return starting point of the segment */
    public Translation getStart() {
        return start;
    }

    /** @return end point of the segment */
    public Translation getEnd() {
        return end;
    }

    /** @return the total length of the segment */
    public double getLength() {
        if (isLine) {
            return deltaStart.norm();
        } else {
            return deltaStart.norm() * Translation.getAngle(deltaStart, deltaEnd).getRadians();
        }
    }

    /**
     * Set whether or not to extrapolate the lookahead point. Should only be true for the last segment in the path
     * 
     * @param val
     */
    public void extrapolateLookahead(boolean val) {
        extrapolateLookahead = val;
    }

    /**
     * Gets the point on the segment closest to the robot
     * 
     * @param position
     *            the current position of the robot
     * @return the point on the segment closest to the robot
     */
    public Translation getClosestPoint(Translation position) {
    	if (isLine) {
            Translation delta = new Translation(start, end);
            double u = ((position.x() - start.x()) * delta.x() + (position.y() - start.y()) * delta.y())
                    / (delta.x() * delta.x() + delta.y() * delta.y());
            if (u >= 0 && u <= 1)
                return new Translation(start.x() + u * delta.x(), start.y() + u * delta.y());
            return (u < 0) ? start : end;
        } else {
            Translation deltaPosition = new Translation(center, position);
            deltaPosition = deltaPosition.scale(deltaStart.norm() / deltaPosition.norm());
            if (Translation.cross(deltaPosition, deltaStart) * Translation.cross(deltaPosition, deltaEnd) < 0) {
                return center.translateBy(deltaPosition);
            } else {
                Translation startDist = new Translation(position, start);
                Translation endDist = new Translation(position, end);
                return (endDist.norm() < startDist.norm()) ? end : start;
            }
        }
    }
    
    /**
     * Calculates the point on the segment <code>dist</code> distance from the starting point along the segment.
     * 
     * @param dist
     *            distance from the starting point
     * @return point on the segment <code>dist</code> distance from the starting point
     */
    public Translation getPointByDistance(double dist) {
        double length = getLength();
        if (!extrapolateLookahead && dist > length) {
            dist = length;
        }
        if (isLine) {
            return start.translateBy(deltaStart.scale(dist / length));
        } else {
            double deltaAngle = Translation.getAngle(deltaStart, deltaEnd).getRadians()
                    * ((Translation.cross(deltaStart, deltaEnd) >= 0) ? 1 : -1);
            deltaAngle *= dist / length;
            Translation t = deltaStart.rotateBy(Rotation.fromRadians(deltaAngle));
            return center.translateBy(t);
        }
    }

    /**
     * Gets the remaining distance left on the segment from point <code>point</code>
     * 
     * @param point
     *            result of <code>getClosestPoint()</code>
     * @return distance remaining
     */
    public double getRemainingDistance(Translation position) {
        if (isLine) {
            return new Translation(end, position).norm();
        } else {
            Translation deltaPosition = new Translation(center, position);
            double angle = Translation.getAngle(deltaEnd, deltaPosition).getRadians();
            double totalAngle = Translation.getAngle(deltaStart, deltaEnd).getRadians();
            return angle / totalAngle * getLength();
        }
    }

    private double getDistanceTravelled(Translation robotPosition) {
        Translation pathPosition = getClosestPoint(robotPosition);
        return getLength() - getRemainingDistance(pathPosition);
    }

    public double getSpeedByDistance(double dist) {
        if (dist < speedController.startPos()) {
            dist = speedController.startPos();
        } else if (dist > speedController.endPos()) {
            dist = speedController.endPos();
        }
        Optional<MotionState> state = speedController.firstStateByPos(dist);
        if (state.isPresent()) {
            return state.get().vel();
        } else {
            System.out.println("Velocity does not exist at that position!");
            return 0.0;
        }
    } 

    public double getSpeedByClosestPoint(Translation robotPosition) {
        return getSpeedByDistance(getDistanceTravelled(robotPosition));
    }

    public MotionState getEndState() {
        return speedController.endState();
    }

    public MotionState getStartState() {
        return speedController.startState();
    }

    public String getMarker() {
        return marker;
    }

    public String toString() {
        if (isLine) {
            return "(" + "start: " + start + ", end: " + end + ", speed: " + maxSpeed + ")";
        } else {
            return "(" + "start: " + start + ", end: " + end + ", center: " + center + ", speed: " + maxSpeed + ")"; 
        }
    }
}