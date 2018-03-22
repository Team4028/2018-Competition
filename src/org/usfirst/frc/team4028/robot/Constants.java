package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.paths.profiles.MiamiValleyField;
import org.usfirst.frc.team4028.robot.paths.profiles.PracticeField;

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
	public static final int ENGINEERING_GAMEPAD_B_USB_PORT = 3;
	
	// PCM Can Bus Address
	public static final int PCM_CAN_ADDR = 0;	
	
	// Talons Can Bus Address
	public static final int LEFT_DRIVE_MASTER_CAN_ADDR = 1;
	public static final int LEFT_DRIVE_SLAVE_CAN_ADDR = 2;
	public static final int RIGHT_DRIVE_MASTER_CAN_ADDR = 3;
	public static final int RIGHT_DRIVE_SLAVE_CAN_ADDR = 4;
	public static final int LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS = 5;
	public static final int RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS = 6;
	public static final int ELEVATOR_LIFT_MASTER_CAN_ADDRESS = 7;
	public static final int CARRIAGE_LEFT_CAN_ADDRESS = 8;
	public static final int CARRIAGE_RIGHT_CAN_ADDRESS = 9;
	public static final int RIGHT_INFEED_DRIVE_CAN_ADDRESS = 10;
	public static final int LEFT_INFEED_DRIVE_CAN_ADDRESS = 11;
	public static final int CLIMBER_CAN_ADDRESS = 12;
	
	// DIO Ports
	public static final int CARRIAGE_LIMIT_SWITCH_DIO_PORT = 0;
	
	// Analog Ports
	public static final int STORED_PRESSURE_SENSOR_AIO_PORT = 0;	
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	public static final double MAX_PITCH_POSITIVE = 25.0;
	public static final double MAX_PITCH_NEGATIVE = -25.0;
	
	// PWM Ports on RoboRIO
	public static final int CLIMBER_SERVO_PWM_ADDRESS = 0;
	
	// PCM Ports
	public static final int CARRIAGE_SQUEEZE_PCM_PORT = 0;
	public static final int CARRIAGE_WIDE_PCM_PORT = 1;
	public static final int SHIFTER_EXTEND_PCM_PORT = 3;
	public static final int SHIFTER_RETRACT_PCM_PORT = 7;
	
	// Solenoid Positions
	public static final Value SHIFTER_LOW_GEAR_POS = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_HIGH_GEAR_POS = DoubleSolenoid.Value.kForward;

	public static final Value CARRIAGE_SQUEEZE_POS = DoubleSolenoid.Value.kForward;
	public static final Value CARRIAGE_WIDE_POS = DoubleSolenoid.Value.kReverse;
	
	// Infeed Position Constants
	public static final double INFEED_POSITION = 2300;
	public static final double INFEED_MINIMUM_ALLOWED_ERROR_POSITION = 2200;
	public static final double INFEED_MAXIMUM_ALLOWED_ERROR_POSITION = 2400;
	
	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  
	// You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
	public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";

    // Units: setpoint, error, and output are in inches per second. 
    public static final double DRIVE_VELOCITY_NOMINAL_OUTPUT = 0.05;
    public static final double DRIVE_VELOCITY_MAX_SETPOINT = 15 * 12.0; // 15 fps
    
	/* Robot Physical Constants */
	// Wheels
	public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.35;
	public static final double TRACK_WIDTH_INCHES = 24.25;
	public static final double TRACK_SCRUBBING_FACTOR = 0.9;
	
	// Geometry
	public static final double CENTER_TO_FRONT_BUMPER_DISTANCE = 16.33;
    public static final double CENTER_TO_INTAKE_DISTANCE = 26.33;
    public static final double CENTER_TO_REAR_BUMPER_DISTANCE = 16.33;
    public static final double CENTER_TO_SIDE_BUMPER_DISTANCE = 20.0;
    
    // Path Following Constants
    public static final double MIN_LOOKAHEAD = 10.0; // inches
    public static final double MIN_LOOKAHEAD_SPEED = 9.0; // inches per second
    public static final double MAX_LOOKAHEAD = 20; // inches
    public static final double MAX_LOOKAHEAD_SPEED = 150.0; // inches per second
    public static final double DELTA_LOOKAHEAD = MAX_LOOKAHEAD - MIN_LOOKAHEAD;
    public static final double DELTA_LOOKAHEAD_SPEED = MAX_LOOKAHEAD_SPEED - MIN_LOOKAHEAD_SPEED;

    public static final double INERTIA_STEERING_GAIN = 0; // angular velocity command is multiplied by this gain *
                            
    public static final double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches
    public static final double PATH_DEFAULT_ACCEL = 100.0; // inches per second^2
    public static final double PATH_DEFAULT_DECEL = 120.0;
    public static final double PATH_FOLLOWING_MAX_VEL = 180.0; // inches per second
    public static final double PATH_FOLLOWING_PROFILE_KP = 6.0; 
    public static final double PATH_FOLLOWING_PROFILE_KI = 0.01;	//0.03;
    public static final double PATH_FOLLOWING_PROFILE_KV = 0.02; //0.02;
    public static final double PATH_FOLLOWING_PROFILE_KFFV = 1.0;
    public static final double PATH_FOLLOWING_PROFILE_KFFA = 0.1;
    public static final double PATH_FOLLOWING_GOAL_POS_TOLERANCE = 0.75;
    public static final double PATH_FOLLOWING_GOAL_VEL_TOLERANCE = 12.0;
    public static final double PATH_STOP_STEERING_DISTANCE = 2.0;
    
    public static final double CELERY_SPEED = 0.0000001;
    public static final double TURTLE_SPEED = 20;
    public static final double WILD_TURTLE_SPEED = 40;
    public static final double NORMAL_SPEED = 60;
    public static final double FLOOR_IT_SPEED = 80;
    public static final double WARP_SPEED = 100;
    public static final double KEEEEEEEEEEEEEEEEEEENS_SPEED = 120;
    
    public static final double BLUE_LEFT_SWITCH_FRONT_X_DELTA = MiamiValleyField._leftSwitchFrontBlueX-PracticeField._leftSwitchFrontBlueX;
    public static final double BLUE_LEFT_SWITCH_FRONT_Y_DELTA = MiamiValleyField._leftSwitchFrontBlueY-PracticeField._leftSwitchFrontBlueY;
    public static final double BLUE_LEFT_SWITCH_BACK_X_DELTA = MiamiValleyField._leftSwitchBackBlueX-PracticeField._leftSwitchBackBlueX;
    public static final double BLUE_LEFT_SCALE_X_DELTA = MiamiValleyField._leftScaleBlueX-PracticeField._leftScaleBlueX;
    public static final double BLUE_LEFT_SCALE_Y_DELTA = MiamiValleyField._leftScaleBlueY-PracticeField._leftScaleBlueY;
    public static final double BLUE_LEFT_SWITCH_BACK_DELTA_Y = MiamiValleyField._leftSwitchBackBlueY-PracticeField._leftSwitchBackBlueY;
    public static final double BLUE_RIGHT_SWITCH_FRONT_X_DELTA = MiamiValleyField._rightSwitchFrontBlueX-PracticeField._rightSwitchFrontBlueX;
    public static final double BLUE_RIGHT_SWITCH_FRONT_Y_DELTA = MiamiValleyField._rightSwitchFrontBlueY-PracticeField._rightSwitchFrontBlueY;
    public static final double BLUE_RIGHT_SWITCH_BACK_X_DELTA = MiamiValleyField._rightSwitchBackBlueX-PracticeField._rightSwitchBackBlueX;
    public static final double BLUE_RIGHT_SCALE_X_DELTA = MiamiValleyField._rightScaleBlueX-PracticeField._rightScaleBlueX;
    public static final double BLUE_RIGHT_SCALE_Y_DELTA = MiamiValleyField._rightScaleBlueY-PracticeField._rightScaleBlueY;
    public static final double BLUE_RIGHT_SWITCH_BACK_DELTA_Y = MiamiValleyField._rightSwitchBackBlueY-PracticeField._rightSwitchBackBlueY;
    
    public static final double RED_LEFT_SWITCH_FRONT_X_DELTA = MiamiValleyField._leftSwitchFrontRedX-PracticeField._leftSwitchFrontRedX;
    public static final double RED_LEFT_SWITCH_FRONT_Y_DELTA = MiamiValleyField._leftSwitchFrontRedY-PracticeField._leftSwitchFrontRedY;
    public static final double RED_LEFT_SWITCH_BACK_X_DELTA = MiamiValleyField._leftSwitchBackRedX-PracticeField._leftSwitchBackRedX;
    public static final double RED_LEFT_SCALE_X_DELTA = MiamiValleyField._leftScaleRedX-PracticeField._leftScaleRedX;
    public static final double RED_LEFT_SCALE_Y_DELTA = MiamiValleyField._leftScaleRedY-PracticeField._leftScaleRedY;
    public static final double RED_LEFT_SWITCH_BACK_DELTA_Y = MiamiValleyField._leftSwitchBackRedY-PracticeField._leftSwitchBackRedY;
    public static final double RED_RIGHT_SWITCH_FRONT_X_DELTA = MiamiValleyField._rightSwitchFrontRedX-PracticeField._rightSwitchFrontRedX;
    public static final double RED_RIGHT_SWITCH_FRONT_Y_DELTA = MiamiValleyField._rightSwitchFrontRedY-PracticeField._rightSwitchFrontRedY;
    public static final double RED_RIGHT_SWITCH_BACK_X_DELTA = MiamiValleyField._rightSwitchBackRedX-PracticeField._rightSwitchBackRedX;
    public static final double RED_RIGHT_SCALE_X_DELTA = MiamiValleyField._rightScaleRedX-PracticeField._rightScaleRedX;
    public static final double RED_RIGHT_SCALE_Y_DELTA = MiamiValleyField._rightScaleRedY-PracticeField._rightScaleRedY;
    public static final double RED_RIGHT_SWITCH_BACK_DELTA_Y = MiamiValleyField._rightSwitchBackRedY-PracticeField._rightSwitchBackRedY;
    
    public static final int BIG_NUMBER = (int)1e6;
    public static final double EPSILON_NEGATIVE_6 = 1e-6;
    public static final double EPSILON_NEGATIVE_9 = 1e-9;
}