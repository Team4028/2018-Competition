package org.usfirst.frc.team4028.util.motion;

public class Twist {
	protected static final Twist _identity = new Twist(0.0, 0.0, 0.0);
	
	public static final Twist identity() {
		return _identity;
	}
	
	public final double dx;
	public final double dy;
	public final double dtheta; // Radians
	
	public Twist(double dx, double dy, double dtheta) {
		this.dx = dx;
		this.dy = dy;
		this.dtheta = dtheta;
	}
	
	public Twist scaled(double scale) {
		return new Twist(dx * scale, dy * scale, dtheta * scale);
	}
}