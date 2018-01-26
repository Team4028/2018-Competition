package org.usfirst.frc.team4028.util.motion;

import static org.usfirst.frc.team4028.util.GeneralUtilities.epsilonEquals;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.Interpolable;

public class RigidTransform implements Interpolable<RigidTransform>{
    protected static final RigidTransform kIdentity = new RigidTransform();

    public static final RigidTransform identity() {
        return kIdentity;
    }

    protected Translation _translation;
    protected Rotation _rotation;

    public RigidTransform() {
        _translation = new Translation();
        _rotation = new Rotation();
    }

    public RigidTransform(Translation translation, Rotation rotation) {
        _translation = translation;
        _rotation = rotation;
    }

    public RigidTransform(RigidTransform other) {
        _translation = new Translation(other._translation);
        _rotation = new Rotation(other._rotation);
    } 

    /**
     * Obtain a new RigidTransform2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static RigidTransform exp(Twist delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < Constants.EPSILON_NEGATIVE_9) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new RigidTransform(new Translation(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation(cos_theta, sin_theta, false));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist log(RigidTransform transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < Constants.EPSILON_NEGATIVE_9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final Translation translation_part = transform.getTranslation()
                .rotateBy(new Rotation(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist(translation_part.x(), translation_part.y(), dtheta);
    }

    public Translation getTranslation() {
        return _translation;
    }

    public Rotation getRotation() {
        return _rotation;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     * 
     * @param other
     *            The other transform.
     * @return This transform * other
     */
    public RigidTransform transformBy(RigidTransform other) {
        return new RigidTransform(_translation.translateBy(other._translation.rotateBy(_rotation)),
                _rotation.rotateBy(other._rotation));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     * 
     * @return The opposite of this transform.
     */
    public RigidTransform inverse() {
        Rotation rotation_inverted = _rotation.inverse();
        return new RigidTransform(_translation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    } 

    public RigidTransform normal() {
        return new RigidTransform(_translation, _rotation.normal());
    }

    /**
     * Finds the point where the heading of this transform intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation intersection(RigidTransform other) {
        final Rotation other_rotation = other.getRotation();
        if (_rotation.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(_rotation.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if the heading of this transform is colinear with the heading of another.
     */
    public boolean isColinear(RigidTransform other) {
        final Twist twist = log(inverse().transformBy(other));
        return (epsilonEquals(twist.dy, 0.0, Constants.EPSILON_NEGATIVE_9) && epsilonEquals(twist.dtheta, 0.0, Constants.EPSILON_NEGATIVE_9));
    }

    private static Translation intersectionInternal(RigidTransform a, RigidTransform b) {
        final Rotation a_r = a.getRotation();
        final Rotation b_r = b.getRotation();
        final Translation a_t = a.getTranslation();
        final Translation b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this transform assuming constant curvature.
     */
    @Override
    public RigidTransform interpolate(RigidTransform other, double x) {
        if (x <= 0) {
            return new RigidTransform(this);
        } else if (x >= 1) {
            return new RigidTransform(other);
        }
        final Twist twist = RigidTransform.log(inverse().transformBy(other));
        return transformBy(RigidTransform.exp(twist.scaled(x)));
    }
    
    @Override
    public String toString() {
    	return _translation.toString() + " | " + _rotation.toString();
    }
}