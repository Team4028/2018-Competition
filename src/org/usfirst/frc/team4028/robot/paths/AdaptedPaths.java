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
				
			case L_SCALE_TO_L_SWITCH:
				return getPath(PATHS.L_SCALE_TO_L_SWITCH);
			case L_SWITCH_TO_L_SCALE:
				return getPath(PATHS.L_SWITCH_TO_L_SCALE);
			case L_SCALE_TO_R_SWITCH_PT_1:
				return getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_1);
			case L_SCALE_TO_R_SWITCH_PT_2:
				return getPath(PATHS.L_SCALE_TO_R_SWITCH_PT_2);
				
			case R_SCALE:
				return getPath(PATHS.R_SCALE);
				
			case R_SCALE_TO_R_SWITCH:
				return getPath(PATHS.R_SCALE_TO_R_SWITCH);
			case R_SWITCH_TO_R_SCALE:
				return getPath(PATHS.R_SWITCH_TO_R_SCALE);
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
	
	public static ArrayList<Waypoint> adaptPyramidForThirdCubeFromLeft() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.gettoLeftPyramidForThirdCubeWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromLPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToLSwitchWithCubeThree() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(Paths.getAwayFromLeftSwitchForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRightSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=reversePath(Paths.getLeftSwitchWaypoints());
		for(int point=2;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRSwitchtoFrontofPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getRightSwitchtoFrontofPyramidWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptSTurnToPyramidFromRight() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.getSTurnFromPyramidtoLeftWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToRightSwitchAfterSTurn() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.gettoLeftSwitchAfterSTurn());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromRSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getAwayFromLeftSwitchForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptPyramidForThirdCubeFromRight() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromRPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(PathBuilder.flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints()));
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToRSwitchWithCubeThree() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_FRONT_X_DELTA, Constants.RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLeftScale() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftScaleWaypoints();
		for(int point=2;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SCALE_X_DELTA, Constants.LEFT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLScaletoLSwitchPt1() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftScaleToLeftSwitchWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SCALE_X_DELTA, Constants.LEFT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLScaletoRSwitchPt1() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftScaleToRightSwitch1Waypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SCALE_X_DELTA, Constants.LEFT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLScaletoRSwitchPt2() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftScaleToRightSwitch2Waypoints();
		sWaypoints.get(0).adjustWaypoint(Constants.LEFT_SCALE_X_DELTA, Constants.LEFT_SCALE_Y_DELTA);
		for(int point=1;point<sWaypoints.size()-1;point++)
		{
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_BACK_X_DELTA,Constants.RIGHT_SWITCH_BACK_DELTA_Y);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRightScale() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getLeftScaleWaypoints());
		for(int point=2;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SCALE_X_DELTA, Constants.RIGHT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRScaletoRSwitchPt1() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getLeftScaleToLeftSwitchWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SCALE_X_DELTA, Constants.RIGHT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRScaletoLSwitchPt1() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getLeftScaleToRightSwitch1Waypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SCALE_X_DELTA, Constants.RIGHT_SCALE_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRScaletoLSwitchPt2() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getLeftScaleToRightSwitch2Waypoints());
		sWaypoints.get(0).adjustWaypoint(Constants.RIGHT_SCALE_X_DELTA, Constants.RIGHT_SCALE_Y_DELTA);
		for(int point=1;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.RIGHT_SWITCH_BACK_X_DELTA, Constants.RIGHT_SWITCH_BACK_DELTA_Y);
		}
		return sWaypoints;
	}
}