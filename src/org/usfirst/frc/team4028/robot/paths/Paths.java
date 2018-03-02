package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.reversePath;

public class Paths {
	public enum PATHS {
		AUTO_RUN,
		
		// First Switch Cube
		L_SWITCH,
		R_SWITCH,
		
		// Second Switch Cube
		L_SWITCH_TO_FRONT_OF_PYRAMID,
		R_SWITCH_TO_FRONT_OF_PYRAMID,
		
		TO_PYRAMID,
		FROM_PYRAMID,
		
		S_TURN_TO_L_SWITCH,
		S_TURN_TO_R_SWITCH,
		
		// Third Switch Cube
		AWAY_FROM_LEFT_SWITCH,
		PYRAMID_FOR_THIRD_CUBE_FROM_LEFT,
		AWAY_FROM_L_PYRAMID,
		TO_L_SWITCH_WITH_CUBE_3,

		AWAY_FROM_RIGHT_SWITCH,
		PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT,
		AWAY_FROM_R_PYRAMID,
		TO_R_SWITCH_WITH_CUBE_3,
		
		L_SCALE,
		L_SCALE_TO_R_SWITCH,
		
		R_SCALE,
		R_SCALE_TO_L_SWITCH,
		
		L_SWITCH_TO_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		L_SWITCH_TO_L_SCALE_SECOND_CUBE
	}
	
	public enum Center {
		AUTO_RUN,
		
		// First Cube
		L_SWITCH,
		R_SWITCH,
		
		// Second Cube
		L_SWITCH_TO_FRONT_OF_PYRAMID,
		R_SWITCH_TO_FRONT_OF_PYRAMID,
		
		TO_PYRAMID,
		FROM_PYRAMID,
		
		S_TURN_TO_L_SWITCH,
		S_TURN_TO_R_SWITCH,
		
		// Third Cube
		AWAY_FROM_LEFT_SWITCH,
		PYRAMID_FOR_THIRD_CUBE_FROM_LEFT,
		AWAY_FROM_L_PYRAMID,
		TO_L_SWITCH_WITH_CUBE_3,

		AWAY_FROM_RIGHT_SWITCH,
		PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT,
		AWAY_FROM_R_PYRAMID,
		TO_R_SWITCH_WITH_CUBE_3,
	}
	
	public enum LeftSide {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		L_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_R_SWITCH,
		R_SCALE_TO_L_SWITCH,
		
		L_SWITCH_TO_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		// Third Cube
		L_SWITCH_TO_L_SCALE_SECOND_CUBE
	}
	
	public enum RightSide {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_R_SWITCH,
		R_SCALE_TO_L_SWITCH,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_SECOND_CUBE
	}
	
	public static Path getPath(PATHS pathName) {
		return getPath(pathName, Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0);
	}
	
	public static Path getPath(PATHS pathName, double inertiaSteeringGain) {
		return getPath(pathName, Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, inertiaSteeringGain);
	}
	
	public static Path getPath(PATHS pathName, double max_Accel, double max_Decel, double inertiaSteeringGain) {
		Path path;
		double maxAccel = max_Accel;
		double maxDecel = max_Decel;
		switch(pathName) {	
			case AUTO_RUN:
				path = buildPathFromWaypoints(getAutoRunWaypoints(), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH:
		        path = buildPathFromWaypoints(getLeftSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
		        path.setIsReversed(false);
		        return path;
			case R_SWITCH:
				path = buildPathFromWaypoints(getRightSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(getLeftSwitchtoFrontofPyramidWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(getRightSwitchtoFrontofPyramidWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
				
			case TO_PYRAMID:
				path = buildPathFromWaypoints(getToPyramidWaypoints(), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case FROM_PYRAMID:
				path = buildPathFromWaypoints(reversePath(getToPyramidWaypoints()), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
				
			case AWAY_FROM_LEFT_SWITCH:
				path = buildPathFromWaypoints(getAwayFromLeftSwitchForThirdCubeWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_LEFT:
				path = buildPathFromWaypoints(gettoLeftPyramidForThirdCubeWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_RIGHT_SWITCH:
				path = buildPathFromWaypoints(flipPath(getAwayFromLeftSwitchForThirdCubeWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT:
				path = buildPathFromWaypoints(flipPath(gettoLeftPyramidForThirdCubeWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case S_TURN_TO_L_SWITCH:
				path = buildPathFromWaypoints(getFrontofPyramidtoLeftSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_L_PYRAMID:
				path = buildPathFromWaypoints(reversePath(gettoLeftPyramidForThirdCubeWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case TO_L_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case S_TURN_TO_R_SWITCH:
				path = buildPathFromWaypoints(getFrontofPyramidtoRightSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_R_PYRAMID:
				path = buildPathFromWaypoints(reversePath(flipPath(gettoLeftPyramidForThirdCubeWaypoints())), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case TO_R_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(flipPath(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints())), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE:
				path = buildPathFromWaypoints(getLeftScaleFromLeftWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			
			case L_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchLeftSideWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			
				
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getRightScaletoLeftSwitchRightSideWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getLeftSwitchToLeftScaleSecondCubeWaypoints(), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
				
			case L_SWITCH_TO_SIDE:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints(), max_Accel, max_Decel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_SIDE_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), max_Accel, max_Decel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
		}
	}
	
	public static Path getPath(Center pathName) {
		Path path;
		
		switch (pathName) {
			case AUTO_RUN:
				path = buildPathFromWaypoints(getAutoRunWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptLeftSwitch(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0065);
				path.setIsReversed(false);
				return path;
			case R_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptRightSwitch(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0065);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(AdaptedPaths.adaptLSwitchtoFrontofPyramid(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.006);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(AdaptedPaths.adaptRSwitchtoFrontofPyramid(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.008);
				path.setIsReversed(true);
				return path;
				
			case TO_PYRAMID:
				path = buildPathFromWaypoints(getToPyramidWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case FROM_PYRAMID:
				path = buildPathFromWaypoints(reversePath(getToPyramidWaypoints()), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
				
			case S_TURN_TO_L_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoLeft(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.01);
				path.setIsReversed(false);
				return path;
			case S_TURN_TO_R_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoRight(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.01);
				path.setIsReversed(false);
				return path;
				
			case AWAY_FROM_LEFT_SWITCH:
				path = buildPathFromWaypoints(getAwayFromLeftSwitchForThirdCubeWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_LEFT:
				path = buildPathFromWaypoints(gettoLeftPyramidForThirdCubeWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_RIGHT_SWITCH:
				path = buildPathFromWaypoints(flipPath(getAwayFromLeftSwitchForThirdCubeWaypoints()), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT:
				path = buildPathFromWaypoints(flipPath(gettoLeftPyramidForThirdCubeWaypoints()), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
		
			case AWAY_FROM_L_PYRAMID:
				path = buildPathFromWaypoints(reversePath(gettoLeftPyramidForThirdCubeWaypoints()), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
			case TO_L_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints()), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_R_PYRAMID:
				path = buildPathFromWaypoints(reversePath(flipPath(gettoLeftPyramidForThirdCubeWaypoints())), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
			case TO_R_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(flipPath(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints())), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
		}
	}
	
	// Do Nothing (default)
	protected static ArrayList<Waypoint> getDoNothingWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(31,162,0,0));
        sWaypoints.add(new Waypoint(32,162,0,Constants.CELERY_SPEED));
        return sWaypoints;
	}
	
	// Auto Run
	protected static ArrayList<Waypoint> getAutoRunWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
		sWaypoints.add(new Waypoint(140,46,0,120));
        return sWaypoints;
	}
	
	// Switch (First Cube)
	protected static ArrayList<Waypoint> getLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,166,0,0));
        sWaypoints.add(new Waypoint(50,166,25,80));
        sWaypoints.add(new Waypoint(90,107,30,80));
        sWaypoints.add(new Waypoint(125,107,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,166,0,0));
        sWaypoints.add(new Waypoint(50,166,25,80));
        sWaypoints.add(new Waypoint(90,217,30,80));
        sWaypoints.add(new Waypoint(125,217,0,80));
        return sWaypoints;
	}
	
	// Switch (Second Cube)
	protected static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,217,0,0));
        sWaypoints.add(new Waypoint(89,217,20,70));
        sWaypoints.add(new Waypoint(60,155,15,50));
        sWaypoints.add(new Waypoint(45,155,0,40));
        return sWaypoints;
	}
	protected static ArrayList<Waypoint> getLeftSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,107,0,0));
        sWaypoints.add(new Waypoint(95,107,20,70));
        sWaypoints.add(new Waypoint(65,168,20,50));
        sWaypoints.add(new Waypoint(45,168,0,40));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		sWaypoints.add(new Waypoint(65,162,20,80));
		sWaypoints.add(new Waypoint(95,107,20,80));
        sWaypoints.add(new Waypoint(126,107,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		 sWaypoints.add(new Waypoint(60,162,15,80));
		 sWaypoints.add(new Waypoint(89,217,20,80));
        sWaypoints.add(new Waypoint(130,217,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45,162,0,0));
        sWaypoints.add(new Waypoint(80,162,0,60));
        return sWaypoints;
	}
	
	// Switch (Third Cube)
	protected static ArrayList<Waypoint> getAwayFromLeftSwitchForThirdCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(122,115,0,0));
        sWaypoints.add(new Waypoint(60,115,0,Constants.FLOOR_IT_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> gettoLeftPyramidForThirdCubeWaypoints(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(60,115,0,0));
        sWaypoints.add(new Waypoint(77,115,15,Constants.FLOOR_IT_SPEED));
        sWaypoints.add(new Waypoint(102,140,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	public static Path getPath(LeftSide pathName) {
		Path path;
		
		switch (pathName) {
			case L_SCALE:
				path = buildPathFromWaypoints(getLeftScaleFromLeftWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.005);
				path.setIsReversed(false);
				return path;	
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0045);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE_OUTSIDE:
				path = buildPathFromWaypoints(getLeftScaleOutsideWaypoints(), Constants.PATH_DEFAULT_ACCEL,Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchLeftSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.007);
				path.setIsReversed(false);
				return path;
			case R_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getRightScaletoLeftSwitchLeftSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.007);
				path.setIsReversed(false);
				return path; 
				
			case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getLeftScaleToLeftSwitch2Waypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
				
			case L_SWITCH_TO_SIDE:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_SIDE_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path; 
		}
	}
	
	// Scale
	protected static ArrayList<Waypoint>  getLeftScaleFromLeftWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(221,46,30,120));
        sWaypoints.add(new Waypoint(273,74,0,100));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(195,46,0,120));
        sWaypoints.add(new Waypoint(240,46,40,120));
        sWaypoints.add(new Waypoint(240,96,0,120));
        sWaypoints.add(new Waypoint(240,110,0,30));
        sWaypoints.add(new Waypoint(240,217,0,120));
        sWaypoints.add(new Waypoint(240,252,35,80));
        sWaypoints.add(new Waypoint(275,252,0,40));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleOutsideWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
	    sWaypoints.add(new Waypoint(300,46,0,120));
	    return sWaypoints;
	}
	
	// Scale Then Switch	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitchLeftSideWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(273,74,0,0));
        sWaypoints.add(new Waypoint(247,87,28,80));
        sWaypoints.add(new Waypoint(247,204,0,120));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaletoLeftSwitchLeftSideWaypoints(){
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(274,244,0,0));
        sWaypoints.add(new Waypoint(247,232,25,80));
        sWaypoints.add(new Waypoint(247,114,0,120));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitch2Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(243,60,0,0));
        sWaypoints.add(new Waypoint(243,190,20,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(213,190,0,Constants.WILD_TURTLE_SPEED));
        return sWaypoints;
	}
	
	// Triple Scale
	protected static ArrayList<Waypoint> getLeftScaleToLeftSwitch2Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	    sWaypoints.add(new Waypoint(281,74,0,0));
	    sWaypoints.add(new Waypoint(261,84,20,60));
	    sWaypoints.add(new Waypoint(229,107,0,60));
	    return sWaypoints;
	}
	
	// Experimental Right Scale Left Switch
	protected static ArrayList<Waypoint> getLeftSwitchBeforeRightScaleWaypoints(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(135,46,20,60));
        sWaypoints.add(new Waypoint(150,63,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftSwitchWaypoints(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(150,63,0,0));
        sWaypoints.add(new Waypoint(240,63,40,60));
        sWaypoints.add(new Waypoint(240,235,0,60));
        return sWaypoints;
	}
	
	// Second cube switch to scale
	protected static ArrayList<Waypoint> getLeftSwitchToLeftScaleSecondCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(220,92,0,0));
	    sWaypoints.add(new Waypoint(240,68,30,70));
	    sWaypoints.add(new Waypoint(270,80,0,70));
	    return sWaypoints;
	}
	
	public static Path getPath(RightSide pathName) {
		Path path;
		
		switch(pathName) {
			case L_SCALE:
				path = buildPathFromWaypoints(getLeftScaleFromRightWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0045);
				path.setIsReversed(false);
				return path;
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromRightWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.005);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE_OUTSIDE:
				path = buildPathFromWaypoints(getRightScaleOutsideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchRightSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.007);
				path.setIsReversed(false);
				return path;
			case R_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getRightScaletoLeftSwitchRightSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.007);
				path.setIsReversed(false);
				return path;
				
			case R_SWITCH_TO_R_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getRightSwitchToRightScaleSecondCubeWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path; 
		}
	}
	
	// Scale
	protected static ArrayList<Waypoint> getLeftScaleFromRightWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,278,0,0));
        sWaypoints.add(new Waypoint(191,278,0,120));
        sWaypoints.add(new Waypoint(232,278,40,120));
        sWaypoints.add(new Waypoint(232,228,0,120));
        sWaypoints.add(new Waypoint(232,214,0,20));
        sWaypoints.add(new Waypoint(232,109,0,100));
        sWaypoints.add(new Waypoint(232,74,35,80));
        sWaypoints.add(new Waypoint(268,74,0,40));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint>  getRightScaleFromRightWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,278,0,0));
        sWaypoints.add(new Waypoint(229,278,30,120));
        sWaypoints.add(new Waypoint(281,250,0,100));
    	return sWaypoints;
	}
	
	// Scale outside
	protected static ArrayList<Waypoint> getRightScaleOutsideWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,278,0,0));
	    sWaypoints.add(new Waypoint(300,278,0,120));
	    return sWaypoints;
	}
	
	// Scale to switch
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitchRightSideWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(279,74,0,0));
        sWaypoints.add(new Waypoint(251,88,30,80));
        sWaypoints.add(new Waypoint(251,208,0,120));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaletoLeftSwitchRightSideWaypoints(){
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(279,244,0,0));
        sWaypoints.add(new Waypoint(247,228,25,80));
        sWaypoints.add(new Waypoint(247,117,0,120));
    	return sWaypoints;
	}
	
	// Second cube switch to scale
	protected static ArrayList<Waypoint> getRightSwitchToRightScaleSecondCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(220,232,0,0));
	    sWaypoints.add(new Waypoint(240,256,30,60));
	    sWaypoints.add(new Waypoint(270,244,0,60));
	    return sWaypoints;
	}
}