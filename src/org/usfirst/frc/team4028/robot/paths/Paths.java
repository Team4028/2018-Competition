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
		L_SCALE_TO_L_SWITCH,
		L_SWITCH_TO_L_SCALE,
		L_SCALE_TO_R_SWITCH,
		L_SCALE_TO_R_SWITCH_PT_2,
		
		R_SCALE,
		R_SCALE_TO_L_SWITCH,
		R_SCALE_TO_R_SWITCH,
		R_SWITCH_TO_R_SCALE, 
		
		L_SCALE_TO_L_SWITCH_2,
		L_SWITCH_TO_L_SCALE_2,
		L_SWITCH_TO_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		L_SWITCH_TO_L_SCALE_SECOND_CUBE
	}
	
	public static Path getPath(PATHS pathName) {
		return getPath(pathName, Constants.PATH_FOLLOWING_STANDARD_ACCEL, Constants.PATH_FOLLOWING_STANDARD_DECEL, 0);
	}
	
	public static Path getPath(PATHS pathName, double inertiaSteeringGain) {
		return getPath(pathName, Constants.PATH_FOLLOWING_STANDARD_ACCEL, Constants.PATH_FOLLOWING_STANDARD_DECEL, inertiaSteeringGain);
	}
	
	public static Path getPath(PATHS pathName, double max_Accel, double max_Decel) {
		return getPath(pathName,max_Accel, max_Decel,0);
	}
	
	public static Path getPath(PATHS pathName, double max_Accel, double max_Decel, double inertiaSteeringGain) {
		Path path;
		double maxAccel = max_Accel;
		double maxDecel = max_Decel;
		switch(pathName) {	
			case L_SWITCH_TO_SIDE:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints(), max_Accel, max_Decel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			
			case L_SWITCH_SIDE_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), max_Accel, max_Decel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case AUTO_RUN:
				path = buildPathFromWaypoints(getAutoRunWaypoints(), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH:
		        path = buildPathFromWaypoints(getLeftSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
		        path.setIsReversed(false);
		        return path;
			case R_SWITCH:
				path = buildPathFromWaypoints(flipPath(getLeftSwitchWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
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
				path = buildPathFromWaypoints(getLeftScaleExperimentalWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToLeftSwitchExperimentalWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_TO_L_SCALE:
				path = buildPathFromWaypoints(reversePath(getLeftScaleToLeftSwitchExperimentalWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case L_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SCALE_TO_R_SWITCH_PT_2:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitch2Waypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getRightScaleToRightSwitchExperimentalWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case R_SWITCH_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightSwitchtoRightScaleExperimentalWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case R_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getRightScaletoLeftSwitchWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SCALE_TO_L_SWITCH_2:
				path = buildPathFromWaypoints(getLeftScaleSecondCubeToSwitchWaypoints(), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_TO_L_SCALE_2:
				path = buildPathFromWaypoints(reversePath(getLeftScaleSecondCubeToSwitchWaypoints()), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
				
			case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(flipPath(getLeftScaleSecondCubeToSwitchWaypoints()), maxAccel, maxDecel, inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
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
		sWaypoints.add(new Waypoint(20,162,0,0));
        sWaypoints.add(new Waypoint(50,162,25,80));
        sWaypoints.add(new Waypoint(90,107,30,80));
        sWaypoints.add(new Waypoint(120,107,0,80));
        return sWaypoints;
	}
	
	// Switch (Second Cube)
	protected static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,217,0,0));
        sWaypoints.add(new Waypoint(89,217,20,70));
        sWaypoints.add(new Waypoint(60,162,15,50));
        sWaypoints.add(new Waypoint(45,162,0,40));
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
        sWaypoints.add(new Waypoint(118,107,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		 sWaypoints.add(new Waypoint(60,162,15,80));
		 sWaypoints.add(new Waypoint(89,217,20,80));
        sWaypoints.add(new Waypoint(119,217,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45,162,0,0));
        sWaypoints.add(new Waypoint(75,162,0,60));
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
	
	// Scale
	protected static ArrayList<Waypoint>  getLeftScaleExperimentalWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(226,46,30,120));
        sWaypoints.add(new Waypoint(278,74,0,100));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(195,46,0,120));
        sWaypoints.add(new Waypoint(240,46,40,120));
        sWaypoints.add(new Waypoint(240,96,0,120));
        sWaypoints.add(new Waypoint(240,110,0,30));
        sWaypoints.add(new Waypoint(240,215,0,120));
        sWaypoints.add(new Waypoint(240,250,35,80));
        sWaypoints.add(new Waypoint(275,250,0,40));
        return sWaypoints;
	}
	
	// Scale Then Switch
	protected static ArrayList<Waypoint> getLeftScaleToLeftSwitchExperimentalWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(281,74,0,0));
        sWaypoints.add(new Waypoint(258,94,15,60));
        sWaypoints.add(new Waypoint(238,94,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleToRightSwitchExperimentalWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(282,247,0,0));
        sWaypoints.add(new Waypoint(245,238,0,80));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightSwitchtoRightScaleExperimentalWaypoints(){
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(245,242,0,0));
		sWaypoints.add(new Waypoint(277,246,0,80));
		return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(281,74,0,0));
        sWaypoints.add(new Waypoint(253,88,30,80));
        sWaypoints.add(new Waypoint(253,208,0,120));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleSecondCubeToSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(281,74,0,0));
        sWaypoints.add(new Waypoint(242,101,0,80));
        //sWaypoints.add(new Waypoint(242,105,0,60));
	    return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaletoLeftSwitchWaypoints(){
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(279,244,0,0));
        sWaypoints.add(new Waypoint(247,228,25,80));
        sWaypoints.add(new Waypoint(247,117,0,120));
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
	    sWaypoints.add(new Waypoint(240,68,30,60));
	    sWaypoints.add(new Waypoint(270,80,0,60));
	    return sWaypoints;
	}
}