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
		L_SCALE_TO_L_SWITCH_PT_1,
		L_SCALE_TO_L_SWITCH_PT_2,
		L_SWITCH_TO_L_SCALE_PT_1,
		L_SWITCH_TO_L_SCALE_PT_2,
		L_SCALE_TO_R_SWITCH_PT_1,
		L_SCALE_TO_R_SWITCH_PT_2,
		
		R_SCALE,
		R_SCALE_TO_L_SWITCH_PT_1,
		R_SCALE_TO_L_SWITCH_PT_2,
		R_SCALE_TO_R_SWITCH_PT_1,
		R_SCALE_TO_R_SWITCH_PT_2,
		R_SWITCH_TO_R_SCALE_PT_1,
		R_SWITCH_TO_R_SCALE_PT_2,
	}
	
	public static Path getPath(PATHS pathName) {
		return getPath(pathName, Constants.PATH_FOLLOWING_STANDARD_ACCEL, Constants.PATH_FOLLOWING_STANDARD_DECEL);
	}
	
	public static Path getPath(PATHS pathName, double max_Accel, double max_Decel) {
		Path path;
		double maxAccel = max_Accel;
		double maxDecel = max_Decel;
		switch(pathName) {		
			case AUTO_RUN:
				path = buildPathFromWaypoints(getAutoRunWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH:
		        path = buildPathFromWaypoints(getLeftSwitchWaypoints(), maxAccel, maxDecel);
		        path.setIsReversed(false);
		        return path;
			case R_SWITCH:
				path = buildPathFromWaypoints(flipPath(getLeftSwitchWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(flipPath(getRightSwitchtoFrontofPyramidWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(getRightSwitchtoFrontofPyramidWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
				
			case TO_PYRAMID:
				path = buildPathFromWaypoints(getToPyramidWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			
			case AWAY_FROM_LEFT_SWITCH:
				path = buildPathFromWaypoints(getAwayFromLeftSwitchForThirdCubeWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_LEFT:
				path = buildPathFromWaypoints(gettoLeftPyramidForThirdCubeWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_RIGHT_SWITCH:
				path = buildPathFromWaypoints(flipPath(getAwayFromLeftSwitchForThirdCubeWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT:
				path = buildPathFromWaypoints(flipPath(gettoLeftPyramidForThirdCubeWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case S_TURN_FROM_PYRAMID_TO_LEFT:
				path = buildPathFromWaypoints(getSTurnFromPyramidtoLeftWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case TO_L_SWITCH_AFTER_S_TURN:
				path = buildPathFromWaypoints(gettoLeftSwitchAfterSTurn(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_L_PYRAMID:
				path = buildPathFromWaypoints(reversePath(gettoLeftPyramidForThirdCubeWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case TO_L_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case S_TURN_FROM_PYRAMID_TO_RIGHT:
				path = buildPathFromWaypoints(flipPath(getSTurnFromPyramidtoLeftWaypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case TO_R_SWITCH_AFTER_S_TURN:
				path = buildPathFromWaypoints(flipPath(gettoLeftSwitchAfterSTurn()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_R_PYRAMID:
				path = buildPathFromWaypoints(reversePath(flipPath(gettoLeftPyramidForThirdCubeWaypoints())), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case TO_R_SWITCH_WITH_CUBE_3:
				path = buildPathFromWaypoints(flipPath(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints())), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE:
				path = buildPathFromWaypoints(getLeftScaleWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case L_SCALE_TO_L_SWITCH_PT_1:
				path = buildPathFromWaypoints(getLeftScaleToLeftSwitch1Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case L_SCALE_TO_L_SWITCH_PT_2:
				path = buildPathFromWaypoints(getLeftScaleToLeftSwitch2Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_TO_L_SCALE_PT_1:
				path = buildPathFromWaypoints(getLeftSwitchToLeftScale1Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case L_SWITCH_TO_L_SCALE_PT_2:
				path = buildPathFromWaypoints(getLeftSwitchToLeftScale2Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case L_SCALE_TO_R_SWITCH_PT_1:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitch1Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case L_SCALE_TO_R_SWITCH_PT_2:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitch2Waypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			case R_SCALE_TO_R_SWITCH_PT_1:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToLeftSwitch1Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case R_SCALE_TO_R_SWITCH_PT_2:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToLeftSwitch2Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case R_SWITCH_TO_R_SCALE_PT_1:
				path = buildPathFromWaypoints(flipPath(getLeftSwitchToLeftScale1Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_R_SCALE_PT_2:
				path = buildPathFromWaypoints(flipPath(getLeftSwitchToLeftScale2Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
			case R_SCALE_TO_L_SWITCH_PT_1:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToRightSwitch1Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
			case R_SCALE_TO_L_SWITCH_PT_2:
				path = buildPathFromWaypoints(flipPath(getLeftScaleToRightSwitch2Waypoints()), maxAccel, maxDecel);
				path.setIsReversed(false);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), maxAccel, maxDecel);
				path.setIsReversed(true);
				return path;
		}
	}
	// Do Nothing (default)
	protected static ArrayList<Waypoint> getDoNothingWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(30,162,0,0));
        sWaypoints.add(new Waypoint(31,162,0,Constants.CELERY_SPEED));
        return sWaypoints;
	}
	
	// Auto Run
	protected static ArrayList<Waypoint> getAutoRunWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,50,0,0));
		sWaypoints.add(new Waypoint(300,50,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	// Switch (First Cube)
	protected static ArrayList<Waypoint> getLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(18,162,0,0));
        sWaypoints.add(new Waypoint(40,162,20,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(75,100,30,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(122,100,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	// Switch (Second Cube)
	protected static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,224,0,0));
        sWaypoints.add(new Waypoint(85,224,20,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(65,162,12,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(50,162,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(50,162,0,0));
        sWaypoints.add(new Waypoint(91,162,0,Constants.FLOOR_IT_SPEED));
        return sWaypoints;
	}
	
	// Switch (Third Cube)
	protected static ArrayList<Waypoint> getSTurnFromPyramidtoLeftWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(91,162,0,0));
        sWaypoints.add(new Waypoint(70,162,15,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(65,121,20,Constants.WILD_TURTLE_SPEED));
        sWaypoints.add(new Waypoint(40,121,0,Constants.WILD_TURTLE_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> gettoLeftSwitchAfterSTurn() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(40,121,0,0));
        sWaypoints.add(new Waypoint(122,121,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getAwayFromLeftSwitchForThirdCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(122,121,0,0));
        sWaypoints.add(new Waypoint(60,121,0,Constants.FLOOR_IT_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> gettoLeftPyramidForThirdCubeWaypoints(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(60,121,0,0));
        sWaypoints.add(new Waypoint(77,121,15,Constants.FLOOR_IT_SPEED));
        sWaypoints.add(new Waypoint(102,143,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	// Scale
	protected static ArrayList<Waypoint>  getLeftScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,50,0,0));
        sWaypoints.add(new Waypoint(195,50,20,120));
        sWaypoints.add(new Waypoint(245,80,20,60));
        sWaypoints.add(new Waypoint(285,80,0,60));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(18,50,0,0));
        sWaypoints.add(new Waypoint(160,50,0,120));
        sWaypoints.add(new Waypoint(232,50,40,100));
        sWaypoints.add(new Waypoint(232,100,0,80));
        sWaypoints.add(new Waypoint(232,220,0,120));
        sWaypoints.add(new Waypoint(232,243,15,60));
        sWaypoints.add(new Waypoint(280,243,0,40));
        return sWaypoints;
	}
	
	// Scale Then Switch
	protected static ArrayList<Waypoint> getLeftScaleToLeftSwitch1Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(285,80,0,0));
        sWaypoints.add(new Waypoint(265,80,10,60));
        sWaypoints.add(new Waypoint(265,60,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToLeftSwitch2Waypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(265,60,0,0));
        sWaypoints.add(new Waypoint(265,75,8,60));
        sWaypoints.add(new Waypoint(245,92,8,60));
        sWaypoints.add(new Waypoint(213,92,0,50));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitch1Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(285,80,0,0));
        sWaypoints.add(new Waypoint(243,80,10,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(243,60,0,Constants.NORMAL_SPEED));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleToRightSwitch2Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(243,60,0,0));
        sWaypoints.add(new Waypoint(243,190,20,Constants.NORMAL_SPEED));
        sWaypoints.add(new Waypoint(213,190,0,Constants.WILD_TURTLE_SPEED));
        return sWaypoints;
	}
	
	// Double Scale
	protected static ArrayList<Waypoint> getLeftSwitchToLeftScale1Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(213,92,0,0));
        sWaypoints.add(new Waypoint(233,92,10,60));
        sWaypoints.add(new Waypoint(233,62,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftSwitchToLeftScale2Waypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(233,62,0,0));
        sWaypoints.add(new Waypoint(233,80,10,60));
        sWaypoints.add(new Waypoint(285,80,0,50));
        return sWaypoints;
	}
}