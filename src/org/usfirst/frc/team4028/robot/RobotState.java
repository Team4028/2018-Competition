package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.util.motion.RigidTransform;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Twist;

import java.util.Map;

import org.usfirst.frc.team4028.util.*;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system. Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 *
 * Robot frames of interest (from parent to child):
 *
 * 1. Field frame: origin is where the robot is turned on
 *
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing forwards
 *
 * 3. Camera frame: origin is the center of the camera imager relative to the robot base.
 *
 * 4. Goal frame: origin is the center of the boiler (note that orientation in this frame is arbitrary). Also note that
 * there can be multiple goal frames.
 *
 * As a kinematic chain with 4 frames, there are 3 transforms of interest:
 *
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and gyro measurements. It will inevitably
 * drift, but is usually accurate over short time periods.
 *
 * 2. Vehicle-to-camera: This is a constant.
 *
 * 3. Camera-to-goal: This is a pure translation, and is measured by the vision system.
 */
public class RobotState {
	private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    private static final int kObservationBufferSize = 100;
    
    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform> _fieldToVehicle;
    private Twist _vehicleVelocityPredicted;
    private Twist _vehicleVelocityMeasured;
    private double _distanceDriven;
    
    private RobotState() {
        reset(0, new RigidTransform());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, RigidTransform initial_field_to_vehicle) {
        _fieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        _fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        _vehicleVelocityPredicted = Twist.identity();
        _vehicleVelocityMeasured = Twist.identity();
        _distanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        _distanceDriven = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized RigidTransform getFieldToVehicle(double timestamp) {
        return _fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform> getLatestFieldToVehicle() {
        return _fieldToVehicle.lastEntry();
    }

    public synchronized RigidTransform getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(RigidTransform.exp(_vehicleVelocityPredicted.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform observation) {
        _fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist measured_velocity,
            Twist predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        _vehicleVelocityMeasured = measured_velocity;
        _vehicleVelocityPredicted = predicted_velocity;
    }

    public synchronized Twist generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation current_gyro_angle) {
        final RigidTransform last_measurement = getLatestFieldToVehicle().getValue();
        final Twist delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
        _distanceDriven += delta.dx;
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return _distanceDriven;
    }

    public synchronized Twist getPredictedVelocity() {
        return _vehicleVelocityPredicted;
    }

    public synchronized Twist getMeasuredVelocity() {
        return _vehicleVelocityMeasured;
    }
}