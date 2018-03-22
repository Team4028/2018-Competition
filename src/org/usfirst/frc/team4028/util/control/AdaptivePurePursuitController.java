package org.usfirst.frc.team4028.util.control;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Translation;
import org.usfirst.frc.team4028.util.motion.Twist;

public class AdaptivePurePursuitController {
    public static class Command {
        public Twist delta = Twist.identity();
        public double crossTrackError;
        public double maxVelocity;
        public double endVelocity;
        public Translation lookaheadPoint;
        public double remainingPathLength;

        public Command(Twist delta, double crossTrackError, double maxVelocity, double endVelocity,
                Translation lookaheadVelocity, double remainingPathLength) {
            this.delta = delta;
            this.crossTrackError = crossTrackError;
            this.maxVelocity = maxVelocity;
            this.endVelocity = endVelocity;
            this.lookaheadPoint = lookaheadVelocity;
            this.remainingPathLength = remainingPathLength;
        }
    }

    Path path;
    boolean atEndOfPath = false;
    final boolean reversed;

    public AdaptivePurePursuitController(Path path, boolean reversed) {
        this.path = path;
        this.reversed = reversed;
    }

    /**
     * Gives the RigidTransform2d.Delta that the robot should take to follow the path
     * 
     * @param pose
     *            robot pose
     * @return movement command for the robot to follow
     */
    public Command update(RigidTransform pose) {
        if (reversed) {
            pose = new RigidTransform(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation.fromRadians(Math.PI)));
        }

        final Path.TargetPointReport report = path.getTargetPoint(pose.getTranslation());
        if (isFinished()) {
            // Stop.
            return new Command(Twist.identity(), report.closest_point_distance, report.max_speed, 0.0,
                    report.lookahead_point, report.remaining_path_distance);
        }

        final Arc arc = new Arc(pose, report.lookahead_point);
        double scale_factor = 1.0;
        // Ensure we don't overshoot the end of the path (once the lookahead speed drops to zero).
        if (report.lookahead_point_speed < 1E-6 && report.remaining_path_distance < arc.length) {
            scale_factor = Math.max(0.0, report.remaining_path_distance / arc.length);
            atEndOfPath = true;
        } else {
            atEndOfPath = false;
        }
        if (reversed) {
            scale_factor *= -1;
        }

        return new Command(
                new Twist(scale_factor * arc.length, 0.0,
                        arc.length * getDirection(pose, report.lookahead_point) * Math.abs(scale_factor) / arc.radius),
                report.closest_point_distance, report.max_speed,
                report.lookahead_point_speed * Math.signum(scale_factor), report.lookahead_point,
                report.remaining_path_distance);
    }

    public boolean hasPassedMarker(String marker) {
        return path.hasPassedMarker(marker);
    }

    public static class Arc {
        public Translation center;
        public double radius;
        public double length;

        public Arc(RigidTransform pose, Translation point) {
            center = getCenter(pose, point);
            radius = new Translation(center, point).norm();
            length = getLength(pose, point, center, radius);
        }
    }

    /**
     * Gives the center of the circle joining the lookahead point and robot pose
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return center of the circle joining the lookahead point and robot pose
     */
    public static Translation getCenter(RigidTransform pose, Translation point) {
        final Translation poseToPointHalfway = pose.getTranslation().interpolate(point, 0.5);
        final Rotation normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction().normal();
        final RigidTransform perpendicularBisector = new RigidTransform(poseToPointHalfway, normal);
        final RigidTransform normalFromPose = new RigidTransform(pose.getTranslation(),
                pose.getRotation().normal());
        if (normalFromPose.isColinear(perpendicularBisector.normal())) {
            // Special case: center is poseToPointHalfway.
            return poseToPointHalfway;
        }
        return normalFromPose.intersection(perpendicularBisector);
    }

    /**
     * Gives the radius of the circle joining the lookahead point and robot pose
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return radius of the circle joining the lookahead point and robot pose
     */
    public static double getRadius(RigidTransform pose, Translation point) {
        Translation center = getCenter(pose, point);
        return new Translation(center, point).norm();
    }

    /**
     * Gives the length of the arc joining the lookahead point and robot pose (assuming forward motion).
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return the length of the arc joining the lookahead point and robot pose
     */
    public static double getLength(RigidTransform pose, Translation point) {
        final double radius = getRadius(pose, point);
        final Translation center = getCenter(pose, point);
        return getLength(pose, point, center, radius);
    }

    public static double getLength(RigidTransform pose, Translation point, Translation center, double radius) {
        if (radius < Constants.BIG_NUMBER) {
            final Translation centerToPoint = new Translation(center, point);
            final Translation centerToPose = new Translation(center, pose.getTranslation());
            // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
            // check the sign of the cross-product between the normal vector and the vector from pose to point.
            final boolean behind = Math.signum(
                    Translation.cross(pose.getRotation().normal().toTranslation(),
                            new Translation(pose.getTranslation(), point))) > 0.0;
            final Rotation angle = Translation.getAngle(centerToPose, centerToPoint);
            return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
        } else {
            return new Translation(pose.getTranslation(), point).norm();
        }
    }

    /**
     * Gives the direction the robot should turn to stay on the path
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return the direction the robot should turn: -1 is left, +1 is right
     */
    public static int getDirection(RigidTransform pose, Translation point) {
        Translation poseToPoint = new Translation(pose.getTranslation(), point);
        Translation robot = pose.getRotation().toTranslation();
        double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
        return (cross < 0) ? -1 : 1; // if robot < pose turn left
    }

    /** @return has the robot reached the end of the path */
    public boolean isFinished() {
        return atEndOfPath;
    }
}