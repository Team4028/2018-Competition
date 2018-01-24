package org.usfirst.frc.team4028.util.control;

/**
 * A utility class for interpolating lookahead distance based on current speed.
 */
public class Lookahead {
	public final double minDistance, maxDistance;
	public final double minSpeed, maxSpeed;
	
	protected final double deltaDistance;
	protected final double deltaSpeed;
	
	public Lookahead(double minDistance, double maxDistance, double minSpeed, double maxSpeed) {
		this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        deltaDistance = maxDistance - minDistance;
        deltaSpeed = maxSpeed - minSpeed;
	}
	
	public double getLookaheadForSpeed(double speed) {
		double lookahead = deltaDistance * (speed - minSpeed) / deltaSpeed + minDistance;
		return Double.isNaN(lookahead) ? minDistance : Math.max(minDistance, Math.min(maxDistance, lookahead));
	}
}