package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.reversePath;

public class Paths {
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
		
		L_SWITCH_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		L_SWITCH_SIDE_2,
		L_SWITCH_SIDE_TO_R_SCALE_2,
		
		// Third Cube
		L_SWITCH_TO_L_SCALE_SECOND_CUBE
	}
	
	public enum RightSide {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		R_SWITCH_BEFORE_L_SCALE,
		
		// Second Cube
		L_SCALE_TO_R_SWITCH,
		R_SCALE_TO_L_SWITCH,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_SECOND_CUBE
	}
	
	public static Path getPath(Center pathName) {
		Path path;
		
		switch (pathName) {
			case AUTO_RUN:
				path = buildPathFromWaypoints(getAutoRunWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH:
				path = buildPathFromWaypoints(getLeftSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(false);
				return path;
			case R_SWITCH:
				path = buildPathFromWaypoints(getRightSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0025);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(getLeftSwitchtoFrontofPyramidWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(true);
				return path;
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				path = buildPathFromWaypoints(getRightSwitchtoFrontofPyramidWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
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
				path = buildPathFromWaypoints(getFrontofPyramidtoLeftSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case S_TURN_TO_R_SWITCH:
				path = buildPathFromWaypoints(getFrontofPyramidtoRightSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
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
        sWaypoints.add(new Waypoint(90,107,30,70));
        sWaypoints.add(new Waypoint(125,107,0,70));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,166,0,0));
        sWaypoints.add(new Waypoint(50,166,25,80));
        sWaypoints.add(new Waypoint(90,217,30,70));
        sWaypoints.add(new Waypoint(125,217,0,70));
        return sWaypoints;
	}
	
	// Switch (Second Cube)
	protected static ArrayList<Waypoint> getLeftSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,107,0,0));
        sWaypoints.add(new Waypoint(95,107,20,70));
        sWaypoints.add(new Waypoint(65,170,20,60));
        sWaypoints.add(new Waypoint(45,170,0,50));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,217,0,0));
        sWaypoints.add(new Waypoint(95,217,20,70));
        sWaypoints.add(new Waypoint(65,158,20,60));
        sWaypoints.add(new Waypoint(45,158,0,50));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		sWaypoints.add(new Waypoint(65,162,20,70));
		sWaypoints.add(new Waypoint(95,107,30,60));
        sWaypoints.add(new Waypoint(126,107,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		 sWaypoints.add(new Waypoint(65,162,20,70));
		 sWaypoints.add(new Waypoint(95,217,30,60));
        sWaypoints.add(new Waypoint(126,217,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45,162,0,0));
        sWaypoints.add(new Waypoint(82,162,0,60));
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
				path = buildPathFromWaypoints(getLeftScaleFromLeftWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(false);
				return path;	
			case R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE_OUTSIDE:
				path = buildPathFromWaypoints(getLeftScaleOutsideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SCALE_TO_R_SWITCH:
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchLeftSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(false);
				return path;
			case R_SCALE_TO_L_SWITCH:
				path = buildPathFromWaypoints(getRightScaletoLeftSwitchLeftSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.00);
				path.setIsReversed(false);
				return path; 
				
			case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getLeftScaleToLeftSwitch2Waypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
				
			case L_SWITCH_SIDE:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_SIDE_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(true);
				return path;
				
			case L_SWITCH_SIDE_2:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints2(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_SIDE_TO_R_SCALE_2:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints2(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
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
        sWaypoints.add(new Waypoint(235,46,40,140));
        sWaypoints.add(new Waypoint(279,68,0,120));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(198,46,0,120));
        sWaypoints.add(new Waypoint(238,46,40,80));
        sWaypoints.add(new Waypoint(238,86,0,80));
        sWaypoints.add(new Waypoint(238,98,0,30));
        sWaypoints.add(new Waypoint(238,205,0,120));
        sWaypoints.add(new Waypoint(238,246,35,80));
        sWaypoints.add(new Waypoint(274,246,0,40));
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
        sWaypoints.add(new Waypoint(100,46,15,80));
        sWaypoints.add(new Waypoint(140,40,15,80));
        sWaypoints.add(new Waypoint(175,40,20,60));
        sWaypoints.add(new Waypoint(175,63,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftSwitchWaypoints(){
	    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(175,63,0,0));
	    sWaypoints.add(new Waypoint(175,43,15,60));
	    sWaypoints.add(new Waypoint(245,43,40,60));
	    sWaypoints.add(new Waypoint(245,103,0,60));
	    sWaypoints.add(new Waypoint(245,133,0,20));
	    sWaypoints.add(new Waypoint(245,242,0,100));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftSwitchBeforeRightScaleWaypoints2(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(12,46,0,0));
        sWaypoints.add(new Waypoint(126,46,15,80));
        sWaypoints.add(new Waypoint(137,58,0,70));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftSwitchWaypoints2(){
	    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	    sWaypoints.add(new Waypoint(140,61,0,0));
        sWaypoints.add(new Waypoint(237,61,40,80));
        sWaypoints.add(new Waypoint(237,102,0,80));
        sWaypoints.add(new Waypoint(237,112,0,30));
        sWaypoints.add(new Waypoint(237,246,0,120));
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
        sWaypoints.add(new Waypoint(242,278,40,120));
        sWaypoints.add(new Waypoint(242,228,0,120));
        sWaypoints.add(new Waypoint(242,214,0,20));
        sWaypoints.add(new Waypoint(242,109,0,100));
        sWaypoints.add(new Waypoint(242,74,35,80));
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
	//Switch Then Scale
	protected static ArrayList<Waypoint> getRightSwitchBeforeLeftScaleWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,278,0,0));
        sWaypoints.add(new Waypoint(165,278,15,60));
        sWaypoints.add(new Waypoint(165,261,0,60));

        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftScaleAfterRightSwitchWaypoints()
	{

	    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(165,261,0,0));
        sWaypoints.add(new Waypoint(165,281,15,120));
        sWaypoints.add(new Waypoint(242,281,40,120));
        sWaypoints.add(new Waypoint(242,80,0,60));

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