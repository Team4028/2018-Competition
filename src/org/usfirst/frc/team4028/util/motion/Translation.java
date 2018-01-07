package org.usfirst.frc.team4028.util.motion;

import java.text.DecimalFormat;

public class Translation {
	protected static final Translation IDENTITY = new Translation();

    public static final Translation identity() {
        return IDENTITY;
    }

    protected double x_, y_;

    public Translation() {
        this(0, 0);
    }

    public Translation(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public Translation(Translation other) {
        this(other.x_, other.y_);
    }

    public Translation(Translation start, Translation end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     * 
     * @return sqrt(x^2 + y^2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }

    public void setX(double x) {
        x_ = x;
    }

    public void setY(double y) {
        y_ = y;
    }

    /**
     * We can add Translation's by adding together the x and y shifts.
     * 
     * @param other
     *            The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation translateBy(Translation other) {
        return new Translation(x_ + other.x_, y_ + other.y_);
    }

    /**
     * We can also rotate Translation's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * 
     * @param rotation
     *            The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation rotateBy(Rotation rotation) {
        return new Translation(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public Rotation direction() {
        return new Rotation(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation that "undoes" this object.
     * 
     * @return Translation by -x and -y.
     */
    public Translation inverse() {
        return new Translation(-x_, -y_);
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
        return new Translation(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public Translation scale(double s) {
        return new Translation(x_ * s, y_ * s);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }

    public static Rotation getAngle(Translation a, Translation b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation();
        }
        return Rotation.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }
    
    public static double dot(Translation a, Translation b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    public static double cross(Translation a, Translation b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }
}