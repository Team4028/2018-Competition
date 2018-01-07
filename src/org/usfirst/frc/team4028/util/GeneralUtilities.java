package org.usfirst.frc.team4028.util;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;

public class GeneralUtilities {	
    // This method writes general info about the build to the Operator's Console
	public static String WriteBuildInfoToDashboard(String robotName) {
		String buildMsg = "?";
		try {
    		//DriverStation.reportError("** Team 4028 The Beak Squad **", false);
    		
    		//get the path of the currently executing jar file
			String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();		
			Path filePath = Paths.get(currentJarFilePath);
			
			//get file system details from current file
			BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
			Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());
	
			// convert from UTC to local time zone
			SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
			outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
			String newDateString = outputFormatter.format(utcFileDate);
			
			// write the build date & time to the operator's console log window
			buildMsg = "== Robot Name == " + robotName + "| Build Date and Time: " + newDateString + "|";
			DriverStation.reportWarning(buildMsg, false);
		} catch (URISyntaxException e) {
    		DriverStation.reportWarning("Error determining filename of current JAR file", true);
			//e.printStackTrace();
		} catch (IOException e) {	
    		DriverStation.reportWarning("General Error trying to determine current JAR file", true);
			//e.printStackTrace();
		}
		
		return buildMsg;
	}
	
    /**
    / This method optionally sets up logging
    /	if return object is null, logger is disabled
	**/
	public static DataLogger setupLogging(String mode) {
		DataLogger dataLogger;
		
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(Constants.PRIMARY_LOG_FILE_PATH);
		Path alternatePath = Paths.get(Constants.ALTERNATE_LOG_FILE_PATH);
    	if (Files.exists(path)) {
    		try {
				dataLogger = new DataLogger(Constants.PRIMARY_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} 
    		catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + Constants.PRIMARY_LOG_FILE_PATH);
			}
    	}
    	else if (Files.exists(alternatePath)) {
    		try {
				dataLogger = new DataLogger(Constants.ALTERNATE_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} 
    		catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + Constants.ALTERNATE_LOG_FILE_PATH);
    		}
    	} else {
    		dataLogger = null;
    		
    		System.out.println("..Logging Disabled!");
    	}
    	
    	return dataLogger;
	}
    
    // This method rounds a double to the specified # of decimal places
	public static double RoundDouble(Double originalValue, int decimalPlaces) {
		BigDecimal bd = new BigDecimal(originalValue).setScale(decimalPlaces, RoundingMode.HALF_EVEN);
		
		return bd.doubleValue();
	}
	
    // This method makes sure a value is between a max & min value
	public static double ClampValue(double originalValue, double minValue, double maxValue) {
		double clampedValue = originalValue;
		
		if (clampedValue > maxValue) {
			clampedValue = maxValue;
		}
		else if (clampedValue < minValue) {
			clampedValue = minValue;
		}
		
		return clampedValue;
	}
	
	private static final double ENCODER_ROTATIONS_PER_DEGREE = 77.371/3600;
	
	public static double arctan(double heading) {
		return Math.toDegrees(Math.atan(heading));
    }
  
    public static double degreesToEncoderRotations(double degrees) {
	    return ENCODER_ROTATIONS_PER_DEGREE * degrees;
    }
    
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}