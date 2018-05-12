package org.usfirst.frc.team4028.util;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.paths.PathBuilder.Arc;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

import org.usfirst.frc.team4028.util.motion.Translation;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.getDist;

public class CubicBezierCurve {
	public static final double CURVE_APPROXIMATION_THRESHOLD_IN_INCHES = 0.25;
	
	Waypoint a, b, c, d;
	double speed;
	
	public CubicBezierCurve(Waypoint start, Waypoint center, Waypoint end, double curvatureFactor) {
		a = start;
		d = end;
		b = new Waypoint(center.pos.x() + curvatureFactor * (a.pos.x() - center.pos.x()), 
				center.pos.y() + curvatureFactor * (a.pos.y() - center.pos.y()), center.radius, center.speed);
		c = new Waypoint(center.pos.x() + curvatureFactor * (d.pos.x() - center.pos.x()), 
				center.pos.y() + curvatureFactor * (d.pos.y() - center.pos.y()), center.radius, center.speed);
		speed = b.speed;
	}
	
	private org.usfirst.frc.team4028.util.motion.Translation getPoint(double t) {
		double x, y;
		x = a.x() * pow(1-t, 3) + 3 * b.x() * t * pow(1-t, 2) + 3 * c.x() * pow(1-t, 2)*t*t + d.x() * pow(t, 3);
		y = a.y() * pow(1-t, 3) + 3 * b.y() * t * pow(1-t, 2) + 3 * c.y() * pow(1-t, 2)*t*t + d.y() * pow(t, 3);
		return new Translation(x, y);
	}
	
	public ArrayList<Arc> approximateWithArcs() {
		double tStart, tEnd;
		double safety;
		ArrayList<Arc> arcs = new ArrayList<Arc>();
		
		tStart = 0;
		tEnd = 1;
		
		do {
			safety = 0;
			
			// start with max possible arc
			tEnd = 1;
			
			Translation np1, np2, np3;
			np1 = getPoint(tStart);
			Arc arc = new Arc(new Translation(0.0, 0.0), new Translation(0.0, 0.0), new Translation(0.0, 0.0), speed), prevArc;
			boolean isCurrGood = false, isPrevGood = false;
			boolean done;
			double tMiddle = tEnd;
			double prevEnd = 1;
			
			// find the best possible arc
			do {
				isPrevGood = isCurrGood;
				prevArc = arc;
				tMiddle = (tStart + tEnd)/2;
				
				np2 = getPoint(tMiddle);
				np3 = getPoint(tEnd);
				
				arc = new Arc(np1, np2, np3, speed);
				
				arc.setInterval(tStart, tEnd);
				
				double error = getError(arc, np1, tStart, tEnd);
				isCurrGood = (error <= CURVE_APPROXIMATION_THRESHOLD_IN_INCHES);
				
				done = isPrevGood && !isCurrGood;
				
				if(!done) prevEnd = tEnd;
				
				// this arc is fine, we can increase e to see if we can find a wider arc
				if(isCurrGood) {
					// if e is already at max, we're done
					if (tEnd >= 1) {
						arc.tEnd = 1;
						prevEnd = 1;
						prevArc = arc;
						
						if (tEnd > 1) {
							Translation d = new Translation(arc.center.x() + arc.radius * cos(arc.tEnd), arc.center.y() + arc.radius * sin(arc.tEnd));
							arc.tEnd += getAngle(arc.center, d, getPoint(1));
						}
						break;
					}
					tEnd = tEnd + (tEnd - tStart)/2;
				}
				// this is a bad arc, need to decrease e to find a good arc
				else {
					tEnd = tMiddle;
				}
				
			} while (!done && safety++ < 100);
			
			if (safety >= 100) {
				break;
			}
			
			arcs.add(prevArc);
			tStart = prevEnd;
		} while (tEnd < 1);
		return arcs;
	}
	
	private double getError(Arc arc, Translation np1, double start, double end) {
		double q = (end - start)/4;
		Translation c1 = getPoint(start + q);
		Translation c2 = getPoint(end - q);
		double ref = getDist(arc.center, np1);
		double d1 = getDist(arc.center, c1);
		double d2 = getDist(arc.center, c2);
		return Math.abs(d1 - ref) + Math.abs(d2 - ref);
	}
	
	public void addToPath(Path p) {
		
	}
	
	public static double getAngle(Translation o, Translation v1, Translation v2) {
		double dx1 = v1.x() - o.x(),
			   dy1 = v1.y() - o.y(),
			   dx2 = v2.x() - o.x(),
			   dy2 = v2.y() - o.y(),
			   cross = dx1*dy2 - dy1*dx2,
			   dot = dx1*dx2 + dy1*dy2;
		return atan2(cross, dot);
	}
}