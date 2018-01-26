package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// This class contains id values for the physical elements of the robot so we can use names 
//	in the code instead of hardcoded constants
public class Constants {
	// Drivers Station Gamepad USB Ports
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int ENGINEERING_GAMEPAD_USB_PORT = 2;
	
	// PCM Can Bus Address
	public static final int PCM_CAN_BUS_ADDR = 0;	
	
	// Talons Can Bus Address
	public static final int LEFT_DRIVE_MASTER_CAN_BUS_ADDR = 11;
	public static final int LEFT_DRIVE_SLAVE_CAN_BUS_ADDR = 12;
	public static final int RIGHT_DRIVE_MASTER_CAN_BUS_ADDR = 9;
	public static final int RIGHT_DRIVE_SLAVE_CAN_BUS_ADDR = 10;
	
	public static final int ELEVATOR_MASTER_CAN_BUS_ADDR = 7;
	public static final int ELEVATOR_SLAVE_CAN_BUS_ADDR = 8;
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	
	// Analog In Ports on RoboRIO
	public static final int ULTRASONIC_PORT = 0;
	
	// PCM Ports
	public static final int SHIFTER_SOLENOID_RETRACT_PCM_PORT = 7;
	public static final int SHIFTER_SOLENOID_EXTEND_PCM_PORT = 3;
	
	// Solenoid Positions
	public static final Value SHIFTER_LOW_GEAR_POS = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_HIGH_GEAR_POS = DoubleSolenoid.Value.kForward;
	
	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  
	// You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
	public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
	
	/* CONTROL LOOP GAINS */
	// PID gains for motion magic loop (LOW GEAR)
	public static final double DRIVE_MOTION_MAGIC_P = 1.0; //4.0
    public static final double DRIVE_MOTION_MAGIC_I = 0.002; // 0.0
    public static final double DRIVE_MOTION_MAGIC_D = 100.0; // 95.0
    public static final double DRIVE_MOTION_MAGIC_F = 0.45; // 0.5
    public static final int DRIVE_MOTION_MAGIC_I_ZONE = 700;
    
    public static final int DRIVE_MOTION_MAGIC_MAX_VEL = 360; //120.0;
    public static final int DRIVE_MOTION_MAGIC_MAX_ACC = 720; //1200.0;

    // Units: setpoint, error, and output are in inches per second.
    // PID gains for drive velocity loop (LOW_GEAR)
    public static final double DRIVE_LOW_GEAR_VELOCITY_KP = 0.8; //0.45;
    public static final double DRIVE_LOW_GEAR_VELOCITY_KI = 0.0;
    public static final double DRIVE_LOW_GEAR_VELOCITY_KD = 5.0; //2.0;
    public static final double DRIVE_LOW_GEAR_VELOCITY_KF = 0.38; //0.5;
    public static final int DRIVE_LOW_GEAR_VELOCITY_I_ZONE = 0;
    
    // PID gains for drive velocity loop (HIGH GEAR)
    public static final double DRIVE_HIGH_GEAR_VELOCITY_KP = 0.45; //1.2;
    public static final double DRIVE_HIGH_GEAR_VELOCITY_KI = 0.0;
    public static final double DRIVE_HIGH_GEAR_VELOCITY_KD = 2.0; //6.0;
    public static final double DRIVE_HIGH_GEAR_VELOCITY_KF = 0.5;
    public static final int DRIVE_HIGH_GEAR_VELOCITY_I_ZONE = 0;
    
    public static final double DRIVE_VELOCITY_NOMINAL_OUTPUT = 0.05;
    public static final double DRIVE_VELOCITY_MAX_SETPOINT = 13.97 * 12.0; // 14 fps
    
    public static final double DRIVE_VOLTAGE_COMPENSATION_RAMPRATE = 0.0;
    
    public static final double DRIVE_CLOSED_LOOP_RAMP_RATE = 0.05;
    
	/* Robot Physical Constants */
	// Wheels
	public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4.05;
	public static final double TRACK_WIDTH_INCHES = 25;
	public static final double TRACK_SCRUBBING_FACTOR = 0.9;
	
	// Geometry
	public static final double CENTER_TO_FRONT_BUMPER_DISTANCE = 16.33;
    public static final double CENTER_TO_INTAKE_DISTANCE = 26.33;
    public static final double CENTER_TO_REAR_BUMPER_DISTANCE = 16.33;
    public static final double CENTER_TO_SIDE_BUMPER_DISTANCE = 20.0;
    
    // Path Following Constants
    public static final double MIN_LOOKAHEAD = 12.0; // inches
    public static final double MIN_LOOKAHEAD_SPEED = 9.0; // inches per second
    public static final double MAX_LOOKAHEAD = 24.0; // inches
    public static final double MAX_LOOKAHEAD_SPEED = 120.0; // inches per second
    public static final double DELTA_LOOKAHEAD = MAX_LOOKAHEAD - MIN_LOOKAHEAD;
    public static final double DELTA_LOOKAHEAD_SPEED = MAX_LOOKAHEAD_SPEED - MIN_LOOKAHEAD_SPEED;

    public static final double INERTIA_STEERING_GAIN = 0.0; // angular velocity command is multiplied by this gain *
                            
    public static final double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches
    public static final double PATH_FOLLOWING_MAX_ACCEL = 120.0; // inches per second^2
    public static final double PATH_FOLLOWING_MAX_VEL = 120.0; // inches per second
    public static final double PATH_FOLLOWING_PROFILE_KP = 5.0;	//5.00;
    public static final double PATH_FOLLOWING_PROFILE_KI = 0.0;	//0.03;
    public static final double PATH_FOLLOWING_PROFILE_KV = 0.1; //0.02;
    public static final double PATH_FOLLOWING_PROFILE_KFFV = 1.0;
    public static final double PATH_FOLLOWING_PROFILE_KFFA = 0.1;
    public static final double PATH_FOLLOWING_GOAL_POS_TOLERANCE = 0.75;
    public static final double PATH_FOLLOWING_GOAL_VEL_TOLERANCE = 12.0;
    public static final double PATH_STOP_STEERING_DISTANCE = 9.0;
    
    public static final double CELERY_SPEED = 0 - 1e-9;
    public static final double TURTLE_SPEED = 20;
    public static final double WILD_TURTLE_SPEED = 40;
    public static final double NORMAL_SPEED = 60;
    public static final double FLOOR_IT_SPEED = 80;
    public static final double WARP_SPEED = 100;
    public static final double KEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEENS_SPEED = 120;
}