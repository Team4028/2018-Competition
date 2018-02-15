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
		
		S_TURN_FROM_PYRAMID_TO_LEFT,
		TO_L_SWITCH_AFTER_S_TURN,
		
		S_TURN_FROM_PYRAMID_TO_RIGHT,
		TO_R_SWITCH_AFTER_S_TURN,
		
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
		L_SCALE_TO_R_SWITCH_PT_1,
		L_SCALE_TO_R_SWITCH_PT_2,
		
		R_SCALE,
		R_SCALE_TO_L_SWITCH_PT_1,
		R_SCALE_TO_L_SWITCH_PT_2,
		R_SCALE_TO_R_SWITCH,
		R_SWITCH_TO_R_SCALE,
	}
	
	public static Path getPath(PATHS pathName) {
		return getPath(pathName, Constants.PATH_FOLLOWING_STANDARD_ACCEL, Constants.PATH_FOLLOWING_STANDARD_DECEL, 0);
	}
	
	public static Path getPath(PATHS pathName, double max_Accel, double max_Decel) {
		return getPath(pathName,max_Accel, max_Decel,0);
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
				path = buildPathFromWaypoints(getToPyramidWaypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
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
			case S_TURN_FROM_PYRAMID_TO_LEFT:
				path = buildPathFromWaypoints(reversePath(getToPyramidWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case TO_L_SWITCH_AFTER_S_TURN:
				path = buildPathFromWaypoints(reversePath(getLeftSwitchtoFrontofPyramidWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
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
			case S_TURN_FROM_PYRAMID_TO_RIGHT:
				path = buildPathFromWaypoints(flipPath(reversePath(getToPyramidWaypoints())), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case TO_R_SWITCH_AFTER_S_TURN:
				path = buildPathFromWaypoints(reversePath(getRightSwitchtoFrontofPyramidWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
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
			case L_SCALE_TO_R_SWITCH_PT_1:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitch1Waypoints(), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
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
				path = buildPathFromWaypoints(flipPath(getLeftScaleToLeftSwitchExperimentalWaypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_R_SCALE:
				path = buildPathFromWaypoints(reversePath(flipPath(getLeftScaleToLeftSwitchExperimentalWaypoints())), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
				return path;
			case R_SCALE_TO_L_SWITCH_PT_1:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToRightSwitch1Waypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(true);
				return path;
			case R_SCALE_TO_L_SWITCH_PT_2:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToRightSwitch2Waypoints()), maxAccel, maxDecel,inertiaSteeringGain);
				path.setIsReversed(false);
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
        sWaypoints.add(new Waypoint(31,162,0,Constants.CELERY_SPEED));
        return sWaypoints;
	}
	
	// Auto Run
	protected static ArrayList<Waypoint> getAutoRunWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
		sWaypoints.add(new Waypoint(140,46,0,Constants.KEEEEEEEEEEEEEEEEEEENS_SPEED));
        return sWaypoints;
	}
	
	// Switch (First Cube)
	protected static ArrayList<Waypoint> getLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,162,0,0));
        sWaypoints.add(new Waypoint(50,162,25,80));
        sWaypoints.add(new Waypoint(90,115,30,80));
        sWaypoints.add(new Waypoint(124,115,0,80));
        return sWaypoints;
	}
	
	// Switch (Second Cube)
	protected static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(124,209,0,0));
        sWaypoints.add(new Waypoint(95,209,20,80));
        sWaypoints.add(new Waypoint(75,158,20,60));
        sWaypoints.add(new Waypoint(50,158,0,40));
        return sWaypoints;
	}
	protected static ArrayList<Waypoint> getLeftSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(124,115,0,0));
        sWaypoints.add(new Waypoint(95,115,20,80));
        sWaypoints.add(new Waypoint(75,172,20,60));
        sWaypoints.add(new Waypoint(50,172,0,40));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(50,166,0,0));
        sWaypoints.add(new Waypoint(82,166,0,70));
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
	protected static ArrayList<Waypoint>  getLeftScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(180,46,30,120));
        sWaypoints.add(new Waypoint(250,82,30,120));
        sWaypoints.add(new Waypoint(281,82,0,60));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint>  getLeftScaleExperimentalWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(235,46,30,120));
        sWaypoints.add(new Waypoint(281,78,0,60));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
       
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(195,46,0,120));
        sWaypoints.add(new Waypoint(236,46,40,50));
        sWaypoints.add(new Waypoint(236,87,0,50));
    	sWaypoints.add(new Waypoint(236,253,30,120));
    	sWaypoints.add(new Waypoint(271,253,0,40));
        return sWaypoints;
	}
	
	// Scale Then Switch
	protected static ArrayList<Waypoint> getLeftScaleToLeftSwitchExperimentalWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(281,74,0,0));
        sWaypoints.add(new Waypoint(252,90,15,60));
        sWaypoints.add(new Waypoint(237,90,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitch1Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(285,80,0,0));
        sWaypoints.add(new Waypoint(243,80,10,60));
        sWaypoints.add(new Waypoint(243,60,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitch2Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(243,60,0,0));
        sWaypoints.add(new Waypoint(243,190,20,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(213,190,0,Constants.WILD_TURTLE_SPEED));
        return sWaypoints;
	}
}