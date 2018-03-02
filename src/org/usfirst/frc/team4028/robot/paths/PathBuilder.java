package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathSegment;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;

public class PathBuilder {
	private static final double kEpsilon = 1E-9;
    private static final double kReallyBigNumber = 1E9;
    private static double maxAccel, maxDecel;

    public static Path buildPathFromWaypoints(List<Waypoint> w, double max_Accel, double max_Decel, double inertiaSteeringGain) {
        Path p = new Path();
        maxAccel = max_Accel;
        maxDecel = max_Decel;
        if (w.size() < 2)
            throw new Error("Path must contain at least 2 waypoints");
        int i = 0;
        if (w.size() > 2) {
            do {
                new Arc(getPoint(w, i), getPoint(w, i + 1), getPoint(w, i + 2)).addToPath(p);
                i++;
            } while (i < w.size() - 2);
        }
        new Line(w.get(w.size() - 2), w.get(w.size() - 1)).addToPath(p, 0);
        p.extrapolateLast();
        p.verifySpeeds();
        p.setAccDec(maxAccel, maxDecel);
        p.setInertiaGain(inertiaSteeringGain);
        return p;
    }

    private static Waypoint getPoint(List<Waypoint> w, int i) {
        if (i > w.size())
            return w.get(w.size() - 1);
        return w.get(i);
    }
    
    public static ArrayList<Waypoint> flipPath(ArrayList<Waypoint> sWaypoints) {
    	ArrayList<Waypoint> flippedWaypoints = new ArrayList<Waypoint>();
    	for (int point = 0; point < sWaypoints.size(); point++) {
    		sWaypoints.get(point).flipWaypoint();
    		flippedWaypoints.add(sWaypoints.get(point));   
    	}
    	return flippedWaypoints;
    }
    
    public static ArrayList<Waypoint> reversePath(ArrayList<Waypoint> sWaypoints) {
    	ArrayList<Waypoint> reversedWaypoints = new ArrayList<Waypoint>();
    	Waypoint firstWaypoint = sWaypoints.get(0);
    	Waypoint lastWaypoint = sWaypoints.get(sWaypoints.size() - 1);
    	
    	reversedWaypoints.add(new Waypoint(lastWaypoint.pos, lastWaypoint.radius, 0.0));
    	for (int point = sWaypoints.size() - 2; point > 0; point--) {
    		reversedWaypoints.add(sWaypoints.get(point));
    	}
    	reversedWaypoints.add(new Waypoint(firstWaypoint.pos, firstWaypoint.radius, lastWaypoint.speed));
    	return reversedWaypoints;
    }


    /**
     * A waypoint along a path. Contains a position, radius (for creating curved paths), and speed. The information from
     * these waypoints is used by the PathBuilder class to generate Paths. Waypoints also contain an optional marker
     * that is used by the WaitForPathMarkerAction.
     *
     * @see PathBuilder
     * @see WaitForPathMarkerAction
     */
    public static class Waypoint {
        Translation pos;
        double radius, speed;
        String marker;

        public Waypoint(Waypoint other) {
            this(other.pos.x(), other.pos.y(), other.radius, other.speed, other.marker);
        }

        public Waypoint(double x, double y, double r, double s) {
            pos = new Translation(x, y);
            radius = r;
            speed = s;
        }

        public Waypoint(Translation pos, double r, double s) {
            this.pos = pos;
            radius = r;
            speed = s;
        }

        public Waypoint(double x, double y, double r, double s, String m) {
            pos = new Translation(x, y);
            radius = r;
            speed = s;
            marker = m;
        }
        public void adjustWaypoint(double x, double y) {
        	pos = new Translation(pos.x()+x,pos.y()+y);
        }
          
        public void flipWaypoint() {
        	pos = new Translation(pos.x(), 324 - pos.y());
        }
        
        @Override
        public String toString() {
        	return "(" + pos.x() + "," + pos.y() + "," + radius + "," + speed + ")";
        }
    }

    /** A Line object is formed by two Waypoints. Contains a start and end position, slope, and speed. */
    static class Line {
        Waypoint a;
        Waypoint b;
        Translation start;
        Translation end;
        Translation slope;
        double speed;

        public Line(Waypoint a, Waypoint b) {
            this.a = a;
            this.b = b;
            slope = new Translation(a.pos, b.pos);
            speed = b.speed;
            start = a.pos.translateBy(slope.scale(a.radius / slope.norm()));
            end = b.pos.translateBy(slope.scale(-b.radius / slope.norm()));
        }

        private void addToPath(Path p, double endSpeed) {
            double pathLength = new Translation(end, start).norm();
            if (pathLength > kEpsilon) {
                if (b.marker != null) {
                    p.addSegment(new PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed,
                            p.getLastMotionState(), endSpeed, maxAccel, maxDecel, b.marker));
                } else {
                    p.addSegment(new PathSegment(start.x(), start.y(), end.x(), end.y(), b.speed,
                            p.getLastMotionState(), endSpeed, maxAccel, maxDecel));
                }
            }
        }
    }

    /** An Arc object is formed by two Lines that share a common Waypoint. Contains a center position, radius, and speed. */
    static class Arc {
        Line a;
        Line b;
        Translation center;
        double radius;
        double speed;

        public Arc(Waypoint a, Waypoint b, Waypoint c) {
            this(new Line(a, b), new Line(b, c));
        }

        public Arc(Line a, Line b) {
            this.a = a;
            this.b = b;
            this.speed = (a.speed + b.speed) / 2;
            this.center = intersect(a, b);
            this.radius = new Translation(center, a.end).norm();
        }

        private void addToPath(Path p) {
            a.addToPath(p, speed);
            if (radius > kEpsilon && radius < kReallyBigNumber) {
                p.addSegment(new PathSegment(a.end.x(), a.end.y(), b.start.x(), b.start.y(), center.x(), center.y(),
                        speed, p.getLastMotionState(), b.speed, maxAccel, maxDecel));
            }
        }

        private static Translation intersect(Line l1, Line l2) {
            final RigidTransform lineA = new RigidTransform(l1.end, new Rotation(l1.slope, true).normal());
            final RigidTransform lineB = new RigidTransform(l2.start, new Rotation(l2.slope, true).normal());
            return lineA.intersection(lineB);
        }
    }
}