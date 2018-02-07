package org.usfirst.frc.team4028.util.control;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.util.motionProfile.MotionState;

public class Path {
	List<PathSegment> segments;
	PathSegment prevSegment;
	HashSet<String> mMarkersCrossed = new HashSet<String>();
	boolean isReversed;
	public double maxAccel, maxDecel;
	public double inertiaSteeringGain;
	
	public void extrapolateLast() {
		PathSegment last = segments.get(segments.size() - 1);
		last.extrapolateLookahead(true);
	}
	
	public Path() {
		segments = new ArrayList<PathSegment>();
	}
	
	/**
     * add a segment to the Path
     * 
     * @param segment
     *            the segment to add
     */
    public void addSegment(PathSegment segment) {
        segments.add(segment);
    }
    
    public void setAccDec(double maxAccel, double maxDecel) {
    	this.maxAccel = maxAccel;
    	this.maxDecel = maxDecel;
    }
    
    public void setInertiaGain(double inertiaGain) {
    	inertiaSteeringGain = inertiaGain;
    }

    /** @return the last MotionState in the path */
    public MotionState getLastMotionState() {
        if (segments.size() > 0) {
            MotionState endState = segments.get(segments.size() - 1).getEndState();
            return new MotionState(0.0, 0.0, endState.vel(), endState.acc());
        } else {
            return new MotionState(0, 0, 0, 0);
        }
    }
    
    public RigidTransform getStartPose() {
		return new RigidTransform(segments.get(0).getStart(), new Rotation(Rotation.fromDegrees(0.0)));
    }

    /**
     * get the remaining distance left for the robot to travel on the current segment
     * 
     * @param robotPos
     *            robot position
     * @return remaining distance on current segment
     */
    public double getSegmentRemainingDist(Translation robotPos) {
        PathSegment currentSegment = segments.get(0);
        return currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
    }
    
    /** @return the length of the current segment */
    public double getSegmentLength() {
        PathSegment currentSegment = segments.get(0);
        return currentSegment.getLength();
    }

    public static class TargetPointReport {
        public Translation closest_point;
        public double closest_point_distance;
        public double closest_point_speed;
        public Translation lookahead_point;
        public double max_speed;
        public double lookahead_point_speed;
        public double remaining_segment_distance;
        public double remaining_path_distance;

        public TargetPointReport() {}
    }

    /**
     * Gives the position of the lookahead point (and removes any segments prior to this point).
     * 
     * @param robot
     *            Translation of the current robot pose.
     * @return report containing everything we might want to know about the target point.
     */
    public TargetPointReport getTargetPoint(Translation robot, Lookahead lookahead) {
        TargetPointReport rv = new TargetPointReport();
        PathSegment currentSegment = segments.get(0);
        rv.closest_point = currentSegment.getClosestPoint(robot);
        rv.closest_point_distance = new Translation(robot, rv.closest_point).norm();
        rv.remaining_segment_distance = currentSegment.getRemainingDistance(rv.closest_point);
        rv.remaining_path_distance = rv.remaining_segment_distance;
        for (int i = 1; i < segments.size(); ++i) {
            rv.remaining_path_distance += segments.get(i).getLength();
        }
        rv.closest_point_speed = currentSegment
                .getSpeedByDistance(currentSegment.getLength() - rv.remaining_segment_distance);
        double lookahead_distance = lookahead.getLookaheadForSpeed(rv.closest_point_speed) + rv.closest_point_distance;
        if (rv.remaining_segment_distance < lookahead_distance && segments.size() > 1) {
            lookahead_distance -= rv.remaining_segment_distance;
            for (int i = 1; i < segments.size(); ++i) {
                currentSegment = segments.get(i);
                final double length = currentSegment.getLength();
                if (length < lookahead_distance && i < segments.size() - 1) {
                    lookahead_distance -= length;
                } else {
                    break;
                }
            }
        } else {
            lookahead_distance += (currentSegment.getLength() - rv.remaining_segment_distance);
        }
        rv.max_speed = currentSegment.getMaxSpeed();
        rv.lookahead_point = currentSegment.getPointByDistance(lookahead_distance);
        rv.lookahead_point_speed = currentSegment.getSpeedByDistance(lookahead_distance);
        checkSegmentDone(rv.closest_point);
        return rv;
    }

    /**
     * Gives the speed the robot should be traveling at the given position
     * 
     * @param robotPos
     *            position of the robot
     * @return speed robot should be traveling
     */
    public double getSpeed(Translation robotPos) {
        PathSegment currentSegment = segments.get(0);
        return currentSegment.getSpeedByClosestPoint(robotPos);
    }
    
    public void setIsReversed(boolean isReversed) {
    	this.isReversed = isReversed;
    }
    
    public boolean isReversed() {
    	return isReversed;
    }
    
    /**
     * Checks if the robot has finished traveling along the current segment then removes it from the path if it has
     * 
     * @param robotPos
     *            robot position
     */
    public void checkSegmentDone(Translation robotPos) {
        PathSegment currentSegment = segments.get(0);
        double remainingDist = currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
        if (remainingDist < Constants.SEGMENT_COMPLETION_TOLERANCE) {
            removeCurrentSegment();
        }
    }

    public void removeCurrentSegment() {
        prevSegment = segments.remove(0);
        String marker = prevSegment.getMarker();
        if (marker != null)
            mMarkersCrossed.add(marker);
    }

    /** Ensures that all speeds in the path are attainable and robot can slow down in time */
    public void verifySpeeds() {
        double maxStartSpeed = 0.0;
        double[] startSpeeds = new double[segments.size() + 1];
        startSpeeds[segments.size()] = 0.0;
        for (int i = segments.size() - 1; i >= 0; i--) {
            PathSegment segment = segments.get(i);
            maxStartSpeed += Math
                    .sqrt(maxStartSpeed * maxStartSpeed + 2 * maxAccel * segment.getLength());
            startSpeeds[i] = segment.getStartState().vel();
            // System.out.println(maxStartSpeed + ", " + startSpeeds[i]);
            if (startSpeeds[i] > maxStartSpeed) {
                startSpeeds[i] = maxStartSpeed;
                // System.out.println("Segment starting speed is too high!");
            }
            maxStartSpeed = startSpeeds[i];
        }
        for (int i = 0; i < segments.size(); i++) {
            PathSegment segment = segments.get(i);
            double endSpeed = startSpeeds[i + 1];
            MotionState startState = (i > 0) ? segments.get(i - 1).getEndState() : new MotionState(0, 0, 0, 0);
            startState = new MotionState(0, 0, startState.vel(), startState.vel());
            segment.createMotionProfiler(startState, endSpeed);
        }
    }

    public boolean hasPassedMarker(String marker) {
        return mMarkersCrossed.contains(marker);
    }

    public String toString() {
        String str = "";
        for (PathSegment s : segments) {
            str += s.toString() + "\n";
        }
        return str;
    }
}