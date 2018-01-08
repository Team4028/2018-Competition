package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// This class contains id values for the physical elements of the robot so we can use names in the code instead of hardcoded constants
public class Constants {
	// Gamepad USB Port
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	
	// PCM Can Bus
	public static final int PCM_CAN_BUS_ADDR = 0;	
	
	// Talons Can Bus
	public static final int LEFT_DRIVE_MASTER_CAN_BUS_ADDR = 11;
	public static final int LEFT_DRIVE_SLAVE_CAN_BUS_ADDR = 12;
	public static final int RIGHT_DRIVE_MASTER_CAN_BUS_ADDR = 9;
	public static final int RIGHT_DRIVE_SLAVE_CAN_BUS_ADDR = 10;
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	
	public static final int SHIFTER_SOLENOID_RETRACT_PCM_PORT = 7;
	public static final int SHIFTER_SOLENOID_EXTEND_PCM_PORT = 6;
	
	// Solenoid Positions
	public static final Value SHIFTER_SOLENOID_LOW_GEAR_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_SOLENOID_HIGH_GEAR_POSITION = DoubleSolenoid.Value.kReverse;
	
	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
	public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
	
	
	/* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double DriveHighGearVelocityKp = 0.5; //1.2;
    public static double DriveHighGearVelocityKi = 0.0;
    public static double DriveHighGearVelocityKd = 3.0; //6.0;
    public static double DriveHighGearVelocityKf = 0.5;
    public static int DriveHighGearVelocityIZone = 0;
    public static double DriveHighGearNominalOutput = 0.05;
    public static double DriveHighGearMaxSetpoint = 13.97 * 12.0; // 17 fps
    
    public static double DRIVE_LOW_GEAR_POSITION_P = 1.0; //4.0
    public static double DRIVE_LOW_GEAR_POSITION_I = 0.002; // 0.0
    public static double DRIVE_LOW_GEAR_POSITION_D = 100.0; // 0.95
    public static double DRIVE_LOW_GEAR_POSITION_F = 0.45; // 0.5
    public static int DRIVE_LOW_GEAR_POSITION_I_ZONE = 700;
    
    public static int DRIVE_TURN_MAX_VEL = 360; //120.0;
    public static int DRIVE_TURN_MAX_ACC = 720; //1200.0
    
    public static double DriveVoltageCompensationRampRate = 0.0;
    
    public static double DRIVE_CLOSED_LOOP_RAMP_RATE = 0.05;
    
	/* Robot Physical Constants */
	// Wheels
	public static double DriveWheelDiameterInches = 4.0;
	public static double TrackWidthInches = 29.04;
	public static double TrackScrubFactor = 0.9;
	
	// Geometry
	public static double CenterToFrontBumperDistance = 16.33;
    public static double CenterToIntakeDistance = 26.33;
    public static double CenterToRearBumperDistance = 16.33;
    public static double CenterToSideBumperDistance = 20.0;
    
    // Path Following Constants
    public static double MinLookAhead = 12.0; // inches
    public static double MinLookAheadSpeed = 9.0; // inches per second
    public static double MaxLookAhead = 24.0; // inches
    public static double MaxLookAheadSpeed = 120.0; // inches per second
    public static double DeltaLookAhead = MaxLookAhead - MinLookAhead;
    public static double DeltaLookAheadSpeed = MaxLookAheadSpeed - MinLookAheadSpeed;

    public static double InertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double SegmentCompletionTolerance = 0.1; // inches
    public static double PathFollowingMaxAccel = 80.0; // inches per second^2
    public static double PathFollowingMaxVel = 120.0; // inches per second
    public static double PathFollowingProfileKp = 6.0;	//5.00;
    public static double PathFollowingProfileKi = 0.0;	//0.03;
    public static double PathFollowingProfileKv = 0.8; //0.02;
    public static double PathFollowingProfileKffv = 1.0;
    public static double PathFollowingProfileKffa = 0.05;
    public static double PathFollowingGoalPosTolerance = 0.75;
    public static double PathFollowingGoalVelTolerance = 12.0;
    public static double PathStopSteeringDistance = 9.0;
}