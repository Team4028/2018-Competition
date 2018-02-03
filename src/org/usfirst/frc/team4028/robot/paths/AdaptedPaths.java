package org.usfirst.frc.team4028.robot.paths;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.reversePath;

import java.util.*;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.robot.paths.profiles.PracticeField;
import org.usfirst.frc.team4028.util.control.Path;

public class AdaptedPaths extends Paths {
	private PracticeField _practiceField = new PracticeField();
	public static Path getAdaptedPath(PATHS pathName) {
		return getPath(pathName, Constants.PATH_FOLLOWING_STANDARD_ACCEL, Constants.PATH_FOLLOWING_STANDARD_DECEL);
	}
	
	public static Path getAdaptedPath(PATHS pathName, double maxAccel, double maxDecel) {
		switch (pathName) {
			case AUTO_RUN:
				return getPath(PATHS.AUTO_RUN);
				
			case L_SWITCH:
		        return getPath(PATHS.L_SWITCH);
			case R_SWITCH:
				return getPath(PATHS.R_SWITCH);
				
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				return getPath(PATHS.L_SWITCH_TO_FRONT_OF_PYRAMID);
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				return getPath(PATHS.R_SWITCH_TO_FRONT_OF_PYRAMID);
				
			case TO_PYRAMID:
				return getPath(PATHS.TO_PYRAMID);
			
			case AWAY_FROM_LEFT_SWITCH:
				return getPath(PATHS.AWAY_FROM_LEFT_SWITCH);
			case PYRAMID_FOR_THIRD_CUBE_FROM_LEFT:
				return getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT);
			case AWAY_FROM_RIGHT_SWITCH:
				return getPath(PATHS.AWAY_FROM_RIGHT_SWITCH);
			case PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT:
				return getPath(PATHS.PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT);
			case S_TURN_FROM_PYRAMID_TO_LEFT:
				return getPath(PATHS.S_TURN_FROM_PYRAMID_TO_LEFT);
			case TO_L_SWITCH_AFTER_S_TURN:
				return getPath(PATHS.TO_L_SWITCH_AFTER_S_TURN);
			case AWAY_FROM_L_PYRAMID:
				return getPath(PATHS.AWAY_FROM_L_PYRAMID);
			case TO_L_SWITCH_WITH_CUBE_3:
				return getPath(PATHS.TO_L_SWITCH_WITH_CUBE_3);
			case S_TURN_FROM_PYRAMID_TO_RIGHT:
				return getPath(PATHS.S_TURN_FROM_PYRAMID_TO_RIGHT);
			case TO_R_SWITCH_AFTER_S_TURN:
				return getPath(PATHS.TO_R_SWITCH_AFTER_S_TURN);
			case AWAY_FROM_R_PYRAMID:
				return getPath(PATHS.AWAY_FROM_R_PYRAMID);
			case TO_R_SWITCH_WITH_CUBE_3:
				return getPath(PATHS.TO_R_SWITCH_WITH_CUBE_3);
				
			case L_SCALE:
				return getPath(PATHS.L_SCALE);
				
			case L_SCALE_TO_L_SWITCH_PT_1:
				return getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_1);
			case L_SCALE_TO_L_SWITCH_PT_2:
				return getPath(PATHS.L_SCALE_TO_L_SWITCH_PT_2);
			case L_SWITCH_TO_L_SCALE_PT_1:
				return getPath(PATHS.L_SWITCH_TO_L_SCALE_PT_1);
			case L_SWITCH_TO_L_SCALE_PT_2:
				return getPath(PATHS.L_SWITCH_TO_L_SCALE_PT_2);
			case L_SCALE_TO_R_SWITCH_PT_1:
				return getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_1);
			case L_SCALE_TO_R_SWITCH_PT_2:
				return getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_2);
				
			case R_SCALE:
				return getPath(PATHS.R_SCALE);
				
			case R_SCALE_TO_R_SWITCH_PT_1:
				return getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_1);
			case R_SCALE_TO_R_SWITCH_PT_2:
				return getPath(PATHS.R_SCALE_TO_R_SWITCH_PT_2);
			case R_SWITCH_TO_R_SCALE_PT_1:
				return getPath(PATHS.R_SWITCH_TO_R_SCALE_PT_1);
			case R_SWITCH_TO_R_SCALE_PT_2:
				return getPath(PATHS.R_SWITCH_TO_R_SCALE_PT_2);
			case R_SCALE_TO_L_SWITCH_PT_1:
				return getPath(PATHS.R_SCALE_TO_L_SWITCH_PT_1);
			case R_SCALE_TO_L_SWITCH_PT_2:
				return getPath(PATHS.R_SCALE_TO_L_SWITCH_PT_2);
				
			default:
				return getPath(PATHS.AUTO_RUN);
		}
	}
	
	public static ArrayList<Waypoint> adaptLeftSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftSwitchWaypoints();
		sWaypoints.get(2).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		sWaypoints.get(3).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLSwitchtoFrontOfPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.getRightSwitchtoFrontofPyramidWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptSTurnFromPyramidtoLeft() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints= Paths.getSTurnFromPyramidtoLeftWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToLSwitchAfterSTurn() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.gettoLeftSwitchAfterSTurn();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromLeftSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getAwayFromLeftSwitchForThirdCubeWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
}