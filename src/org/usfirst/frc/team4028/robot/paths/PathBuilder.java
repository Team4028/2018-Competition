package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.control.PathSegment;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.usfirst.frc.team4028.util.motion.Translation.getAngle;

public class PathBuilder {
	private static final double kEpsilon = 1E-9;
    private static final double kReallyBigNumber = 1E9;
    private static double maxAccel, maxDecel;

    public static Path buildPathFromWaypoints(double max_Accel, double max_Decel, double inertiaSteeringGain, boolean isReversed, List<Waypoint> w) {
        Path p = new Path(max_Accel, max_Decel, inertiaSteeringGain);
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
        p.setStartingAngle(getAngle(w.get(0).pos, w.get(1).pos).getDegrees());
        p.extrapolateLast();
        p.verifySpeeds();
        p.setIsReversed(isReversed);
        return p;
    }
    
    public static Path buildPathFromWaypoints(double inertiaGain, boolean isReversed, List<Waypoint> w) {
    	return buildPathFromWaypoints(Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, inertiaGain, isReversed, w);
    }
    
    public static Path buildPathFromWaypoints(double inertiaGain, List<Waypoint> w) {
    	return buildPathFromWaypoints(inertiaGain, false, w);
    }
    
    public static Path buildPathFromWaypoints(boolean isReversed, List<Waypoint> w) {
    	return buildPathFromWaypoints(0.0, isReversed, w);
    }
    
    public static Path buildPathFromWaypoints(List<Waypoint> w) {
    	return buildPathFromWaypoints(false, w);
    }
    
    public static ArrayList<Waypoint> getStraightPathWaypoints(Translation startPose, double startAngle, double distance) {
    	Translation endPose = startPose.extrapolate(startPose.translateBy(Rotation.fromDegrees(startAngle).toTranslation()), distance);
    	//System.out.println(endPose.toString());
    	ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
    	sWaypoints.add(new Waypoint(startPose, 0, 0));
    	sWaypoints.add(new Waypoint(endPose, 0, 140));
    	return sWaypoints;
    }

    private static Waypoint getPoint(List<Waypoint> w, int i) {
    	return i > w.size() ? w.get(w.size() - 1) : w.get(i);
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
        public Translation pos;
        public double radius, speed;
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
        
        public double x() {
        	return pos.x();
        }
        
        public double y() {
        	return pos.y();
        }
        
        @Override
        public String toString() {
        	return "(" + pos.x() + "," + pos.y() + "," + radius + "," + speed + ")";
        }
    }

    /** A Line object is formed by two Waypoints. Contains a start and end position, slope, and speed. */
    static class Line {
        Waypoint a, b;
        Translation start;
        Translation end;
        Translation slope;
        double speed;

        public Line(Waypoint a, Waypoint b) {
            this.a = a;
            this.b = b;
            slope = new Translation(a.pos, b.pos);
            speed = b.speed;
            
            if ((a.radius + b.radius) > slope.norm()) {
            	if ((a.radius > slope.norm()/2) && (b.radius > slope.norm()/2)) {
            		a.radius = 0.5 * slope.norm();
            		b.radius = 0.5 * slope.norm();
            	}
            	else if ((a.radius > slope.norm()/2) && (b.radius < slope.norm()/2)) {
            		a.radius = slope.norm() - b.radius;
            	}
            	else if ((a.radius < slope.norm()/2) && (b.radius > slope.norm()/2)) {
            		b.radius = slope.norm() - a.radius;
            	}
            }
            
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
    public static class Arc {
        Line a, b;
        public Translation center;
        public double radius;
        public double speed;
        public double tStart = 0;
        public double tEnd = 1;

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
        
        public Arc(Translation a, Translation b, Translation c, double speed) {
        	double quart = Math.PI / 2;
    		
    		double dx1 = b.x() - a.x(),
    			   dy1 = b.y() - a.y(),
    			   dx2 = c.x() - b.x(),
    			   dy2 = c.y() - b.y();
    		
    		double dx1p = dx1 * cos(quart) - dy1 * sin(quart),
    			   dy1p = dx1 * sin(quart) + dy1 * cos(quart),
    		       dx2p = dx2 * cos(quart) - dy2 * sin(quart),
    		       dy2p = dx2 * sin(quart) + dy2 * cos(quart);
    		
    		// chord midpoints
    		double mx1 = (a.x() + b.x())/2,
    			   my1 = (a.y() + b.y())/2,
    			   mx2 = (b.x() + c.x())/2,
    			   my2 = (b.y() + c.y())/2;
    		
    		// midpoint offsets
    		double mx1n = mx1 + dx1p,
    			   my1n = my1 + dy1p,
    		       mx2n = mx2 + dx2p,
    		       my2n = my2 + dy2p;
    		
    		center = getCenter(mx1, my1, mx1n, my1n, mx2, my2, mx2n, my2n);
    		radius = getDist(center, a);
    		this.speed = speed;
    		
    		// arc start/end values over mid point
    		double s = atan2(a.y() - center.y(), a.x() - center.x()),
    			   m = atan2(b.y() - center.y(), b.x() - center.x()),
    			   e = atan2(c.y() - center.y(), c.x() - center.x()),
    			   ph = 0;
    		
    		// determine arc direction (cw/ccw correction)
    		if (s < e) {
    			// if s<m<e, arc(s, e)
    			// if m<s<e, arc(e, s + tau)
    			// if s<e<m, arc(e, s + tau)
    			if (s>m || m>e) 
    				s += 2*Math.PI;
    			
    			if (s>e)
    				ph=e; e=s; s=ph; // swap e and s
    		} else {
    			// if e<m<s, arc(e, s)
    	        // if m<e<s, arc(s, e + tau)
    	        // if e<s<m, arc(s, e + tau)
    			if (e<m && m<s) {
    				ph=e; e=s; s=ph;
    			} else {
    				e += 2*Math.PI;
    			}
    		}
    		
    		setInterval(s, e);
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
        
        public void setInterval(double s, double e) {
        	tStart = s;
        	tEnd = e;
        }
        
        public Translation getCenter(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
			double nx=(x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4),
			       ny=(x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4),
			       d=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
			return new Translation(nx/d, ny/d);
		}
    }
    
    public static double getDist(Translation a, Translation b) {
		return Math.hypot(b.x() - a.x(), b.y() - a.y());
	}
}