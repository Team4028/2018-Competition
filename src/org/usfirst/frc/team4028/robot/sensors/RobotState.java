package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.util.motion.*;
import org.usfirst.frc.team4028.util.*;

import java.util.Map;

/**
 * RobotState keeps track of the robot pose relative to its start point throughout the match for use in autonomous.
 *
 * Field-to-vehicle is tracked over time by integrating encoder and gyro measurements. It will inevitably
 * drift, but is usually accurate over short time periods.
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
    private double _distanceDriven;
    
    private RobotState() {
        reset(0, new RigidTransform());
    }

    /** Resets the field to robot transform (robot's position on the field) */
    public synchronized void reset(double start_time, RigidTransform initial_field_to_vehicle) {
        _fieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        _fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        _vehicleVelocityPredicted = Twist.identity();
        _distanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        _distanceDriven = 0.0;
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform> getLatestFieldToVehicle() {
        return _fieldToVehicle.lastEntry();
    }

    private synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform observation) {
        _fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist measured_velocity,
            Twist predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
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
}