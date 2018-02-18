package org.usfirst.frc.team4028.util.motion;

import static org.usfirst.frc.team4028.util.GeneralUtilities.epsilonEquals;

import org.usfirst.frc.team4028.robot.Constants;

public class Rotation {
	protected static final Rotation kIdentity = new Rotation();

    public static final Rotation identity() {
        return kIdentity;
    }

    protected double cosAngle, sinAngle;

    public Rotation() {
        this(1, 0, false);
    }

    public Rotation(double x, double y, boolean normalize) {
        cosAngle = x;
        sinAngle = y;
        if (normalize) {
            normalize();
        }
    }

    public Rotation(Rotation other) {
        cosAngle = other.cosAngle;
        sinAngle = other.sinAngle;
    }

    public Rotation(Translation direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation fromRadians(double angle_radians) {
        return new Rotation(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    public static Rotation fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
     * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
     */
    public void normalize() {
        double magnitude = Math.hypot(cosAngle, sinAngle);
        if (magnitude > Constants.EPSILON_NEGATIVE_9) {
            sinAngle /= magnitude;
            cosAngle /= magnitude;
        } else {
            sinAngle = 0;
            cosAngle = 1;
        }
    }

    public double cos() {
        return cosAngle;
    }

    public double sin() {
        return sinAngle;
    }

    public double tan() {
        if (Math.abs(cosAngle) < Constants.EPSILON_NEGATIVE_9) {
            if (sinAngle >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sinAngle / cosAngle;
    }

    public double getRadians() {
        return Math.atan2(sinAngle, cosAngle);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     * 
     * @param other
     *            The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation rotateBy(Rotation other) {
        return new Rotation(cosAngle * other.cosAngle - sinAngle * other.sinAngle,
                cosAngle * other.sinAngle + sinAngle * other.cosAngle, true);
    }

    public Rotation normal() {
        return new Rotation(-sinAngle, cosAngle, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     * 
     * @return The opposite of this rotation.
     */
    public Rotation inverse() {
        return new Rotation(cosAngle, -sinAngle, false);
    }

    public boolean isParallel(Rotation other) {
        return epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0, Constants.EPSILON_NEGATIVE_9);
    }

    public Translation toTranslation() {
        return new Translation(cosAngle, sinAngle);
    }
    
    @Override
    public String toString() {
    	return "Angle: " + Double.toString(getDegrees());
    }
}