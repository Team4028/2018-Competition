package org.usfirst.frc.team4028.util.motion;

import java.text.DecimalFormat;

public class Translation {
	protected static final Translation IDENTITY = new Translation();

    public static final Translation identity() {
        return IDENTITY;
    }

    protected double _x, _y;

    public Translation() {
        this(0, 0);
    }

    public Translation(double x, double y) {
        _x = x;
        _y = y;
    }

    public Translation(Translation other) {
        this(other._x, other._y);
    }

    public Translation(Translation start, Translation end) {
        _x = end._x - start._x;
        _y = end._y - start._y;
    }

    /** The "norm" of a transform is the Euclidean distance in x and y. */
    public double norm() {
        return Math.hypot(_x, _y);
    }
    
    public double x() {
    	return _x;
    }
    
    public double y() {
    	return _y;
    }

    /**
     * We can add Translation's by adding together the x and y shifts.
     * 
     * @param other
     *            The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation translateBy(Translation other) {
        return new Translation(_x + other._x, _y + other._y);
    }

    /**
     * We can also rotate Translation's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * 
     * @param rotation
     *            The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation rotateBy(Rotation rotation) {
        return new Translation(_x * rotation.cos() - _y * rotation.sin(), _x * rotation.sin() + _y * rotation.cos());
    }

    public Rotation direction() {
        return new Rotation(_x, _y, true);
    }

    /**
     * The inverse simply means a Translation that "undoes" this object.
     * 
     * @return Translation by -x and -y.
     */
    public Translation inverse() {
        return new Translation(-_x, -_y);
    }

    public Translation interpolate(Translation other, double x) {
        if (x <= 0) {
            return new Translation(this);
        } else if (x >= 1) {
            return new Translation(other);
        }
        return extrapolate(other, x);
    }

    public Translation extrapolate(Translation other, double x) {
        return new Translation(x * (other._x - _x) + _x, x * (other._y - _y) + _y);
    }

    public Translation scale(double s) {
        return new Translation(_x * s, _y * s);
    }

    public static Rotation getAngle(Translation a, Translation b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation();
        }
        return Rotation.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }
    
    public static double dot(Translation a, Translation b) {
        return a._x * b._x + a._y * b._y;
    }

    public static double cross(Translation a, Translation b) {
        return a._x * b._y - a._y * b._x;
    }
    
    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(_x) + "," + fmt.format(_y) + ")";
    }
}