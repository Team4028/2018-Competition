package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.loops.ILoop;
import org.usfirst.frc.team4028.util.motion.Rotation;
import org.usfirst.frc.team4028.util.motion.Twist;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator {
	static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {}

    RobotState robotState = RobotState.getInstance();
    Chassis chassis = Chassis.getInstance();
    double leftEncoderPrevDistance = 0;
    double rightEncoderPrevDistance = 0;
    
    private final ILoop _loop = new ILoop() {
	    @Override
	    public synchronized void onStart(double timestamp) {
	        leftEncoderPrevDistance = chassis.getLeftDistanceInches();
	        rightEncoderPrevDistance = chassis.getRightDistanceInches();
	    }
	
	    @Override
	    public synchronized void onLoop(double timestamp) {
	        final double left_distance = chassis.getLeftDistanceInches();
	        final double right_distance = chassis.getRightDistanceInches();
	        final Rotation gyro_angle = Rotation.fromDegrees(chassis.getHeading());
	        final Twist odometry_velocity = robotState.generateOdometryFromSensors(
	                left_distance - leftEncoderPrevDistance, right_distance - rightEncoderPrevDistance, gyro_angle);
	        final Twist predicted_velocity = Kinematics.forwardKinematics(chassis.getLeftVelocityInchesPerSec(),
	                chassis.getRightVelocityInchesPerSec());
	        robotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
	        leftEncoderPrevDistance = left_distance;
	        rightEncoderPrevDistance = right_distance;
	    }
	
	    @Override
	    public void onStop(double timestamp) {}
    };
    
    public ILoop getLoop() {
    	return _loop;
    }
}