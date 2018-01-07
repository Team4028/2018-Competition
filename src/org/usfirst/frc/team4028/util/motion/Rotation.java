package org.usfirst.frc.team4028.util.motion;

import static org.usfirst.frc.team4028.util.GeneralUtilities.epsilonEquals;

public class Rotation {
	protected static final Rotation kIdentity = new Rotation();

    public static final Rotation identity() {
        return kIdentity;
    }

    protected static final double kEpsilon = 1E-9;

    protected double cos_angle_;
    protected double sin_angle_;

    public Rotation() {
        this(1, 0, false);
    }

    public Rotation(double x, double y, boolean normalize) {
        cos_angle_ = x;
        sin_angle_ = y;
        if (normalize) {
            normalize();
        }
    }

    public Rotation(Rotation other) {
        cos_angle_ = other.cos_angle_;
        sin_angle_ = other.sin_angle_;
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
        double magnitude = Math.hypot(cos_angle_, sin_angle_);
        if (magnitude > kEpsilon) {
            sin_angle_ /= magnitude;
            cos_angle_ /= magnitude;
        } else {
            sin_angle_ = 0;
            cos_angle_ = 1;
        }
    }

    public double cos() {
        return cos_angle_;
    }

    public double sin() {
        return sin_angle_;
    }

    public double tan() {
        if (Math.abs(cos_angle_) < kEpsilon) {
            if (sin_angle_ >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle_ / cos_angle_;
    }

    public double getRadians() {
        return Math.atan2(sin_angle_, cos_angle_);
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
        return new Rotation(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
    }

    public Rotation normal() {
        return new Rotation(-sin_angle_, cos_angle_, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     * 
     * @return The opposite of this rotation.
     */
    public Rotation inverse() {
        return new Rotation(cos_angle_, -sin_angle_, false);
    }

    public boolean isParallel(Rotation other) {
        return epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0, kEpsilon);
    }

    public Translation toTranslation() {
        return new Translation(cos_angle_, sin_angle_);
    }

    public Rotation interpolate(Rotation other, double x) {
        if (x <= 0) {
            return new Rotation(this);
        } else if (x >= 1) {
            return new Rotation(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation.fromRadians(angle_diff * x));
    }
    
    @Override
    public String toString() {
    	return "Angle: " + Double.toString(getDegrees());
    }
}