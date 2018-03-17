package org.usfirst.frc.team4028.robot.paths;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.reversePath;

import java.util.*;
import java.awt.datatransfer.FlavorMap;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.Dashboard;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("unused")// nailed it

public class AdaptedPaths extends Paths {//oh God we're here
	public static Path getAdaptedPath(Center pathName) {
		Path path;
		
		switch (pathName) {
			case AUTO_RUN:
				return getPath(Center.AUTO_RUN);
			
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
				return getPath(Center.TO_PYRAMID);
			case FROM_PYRAMID:
				return getPath(Center.FROM_PYRAMID);
				
			case S_TURN_TO_L_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoLeft(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.01);
				path.setIsReversed(false);
				return path;
			case S_TURN_TO_R_SWITCH:
				path = buildPathFromWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoRight(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.01);
				path.setIsReversed(false);
				return path;
				
			case AWAY_FROM_LEFT_SWITCH:
				return getPath(Center.AWAY_FROM_LEFT_SWITCH);
			case PYRAMID_FOR_THIRD_CUBE_FROM_LEFT:
				return getPath(Center.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT);
			case AWAY_FROM_RIGHT_SWITCH:
				return getPath(Center.AWAY_FROM_LEFT_SWITCH);
			case PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT:
				return getPath(Center.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT);
				
			case AWAY_FROM_L_PYRAMID:
				return getPath(Center.AWAY_FROM_L_PYRAMID);
			case TO_L_SWITCH_WITH_CUBE_3:
				return getPath(Center.TO_L_SWITCH_WITH_CUBE_3);
			case AWAY_FROM_R_PYRAMID:
				return getPath(Center.AWAY_FROM_R_PYRAMID);
			case TO_R_SWITCH_WITH_CUBE_3:
				return getPath(Center.TO_R_SWITCH_WITH_CUBE_3);
			
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path; 
		}
	}
	
	static Dashboard _dashboard = Dashboard.getInstance();
	
	public static ArrayList<Waypoint> adaptLeftSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftSwitchWaypoints();
		if(_dashboard.isBlueAlliance()) {
			sWaypoints.get(2).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, 0);
			sWaypoints.get(3).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, 0);
		} else {
			sWaypoints.get(2).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, 0);
			sWaypoints.get(3).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, 0);
		}

		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLSwitchtoFrontofPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftSwitchtoFrontofPyramidWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, 0);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, 0);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptSTurnFromPyramidtoLeft() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints= getFrontofPyramidtoLeftSwitchWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, 0);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, 0);
			}
		}
		return sWaypoints;
	}
/*	
	public static ArrayList<Waypoint> adaptAwayFromLeftSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getAwayFromLeftSwitchForThirdCubeWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptPyramidForThirdCubeFromLeft() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.gettoLeftPyramidForThirdCubeWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromLPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToLSwitchWithCubeThree() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(Paths.getAwayFromLeftSwitchForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}*/
	
	public static ArrayList<Waypoint> adaptRightSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(getLeftSwitchWaypoints());
		for(int point=2;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRSwitchtoFrontofPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=getRightSwitchtoFrontofPyramidWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			}
			
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptSTurnFromPyramidtoRight() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=getFrontofPyramidtoRightSwitchWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SWITCH_FRONT_X_DELTA, 0);
			}
		}
		return sWaypoints;
	}
	
/*	public static ArrayList<Waypoint> adaptAwayFromRSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.getAwayFromLeftSwitchForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, Constants.BLUE_RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptPyramidForThirdCubeFromRight() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, Constants.BLUE_RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromRPyramid() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.reversePath(PathBuilder.flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints()));
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, Constants.BLUE_RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToRSwitchWithCubeThree() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.gettoLeftPyramidForThirdCubeWaypoints());
		for(int point=0;point<sWaypoints.size();point++) {
			sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_FRONT_X_DELTA, Constants.BLUE_RIGHT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}*/
	
	public static Path getAdaptedPath(LeftSide pathName) {
		Path path;
		
		switch (pathName) {
		case L_SCALE:
			path = buildPathFromWaypoints(adaptLeftScale(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.005);
			path.setIsReversed(false);
			return path;	
		case R_SCALE:
			path = buildPathFromWaypoints(adaptRightScale(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0045);
			path.setIsReversed(false);
			return path;
			
		case L_SCALE_OUTSIDE:
			return getPath(LeftSide.L_SCALE_OUTSIDE);
			
		case L_SCALE_TO_R_SWITCH:
			path = buildPathFromWaypoints(adaptLScaletoRSwitch(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.007);
			path.setIsReversed(false);
			return path;
		case R_SCALE_TO_L_SWITCH:
			return getPath(LeftSide.R_SCALE_TO_L_SWITCH);
			
		case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
			path = buildPathFromWaypoints(adaptLSwitchtoLScaleforThirdCube(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
			path.setIsReversed(true);
			return path;
			
		case L_SWITCH_SIDE:
			path = buildPathFromWaypoints(adaptLeftSwitchBeforeRightScale(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
			path.setIsReversed(false);
			return path;
		case L_SWITCH_SIDE_TO_R_SCALE:
			path = buildPathFromWaypoints(adaptRightScaleAfterLeftSwitch(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
			path.setIsReversed(false);
			return path;
			
		default:
			path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
			path.setIsReversed(false);
			return path;
		}
	}
	
	public static ArrayList<Waypoint> adaptLeftScale() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=getLeftScaleFromLeftWaypoints();
		for(int point=1;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SCALE_X_DELTA, Constants.BLUE_LEFT_SCALE_Y_DELTA);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_LEFT_SCALE_X_DELTA, Constants.RED_LEFT_SCALE_Y_DELTA);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLScaletoRSwitch() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftScaleToRightSwitchLeftSideWaypoints();
		if(_dashboard.isBlueAlliance()) {
			sWaypoints.get(0).adjustWaypoint(Constants.BLUE_LEFT_SCALE_X_DELTA, Constants.BLUE_LEFT_SCALE_Y_DELTA);
			for(int point=1;point<sWaypoints.size();point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SWITCH_BACK_X_DELTA, Constants.BLUE_RIGHT_SWITCH_BACK_DELTA_Y);
			}
		} else {
			sWaypoints.get(0).adjustWaypoint(Constants.RED_LEFT_SCALE_X_DELTA, Constants.RED_LEFT_SCALE_Y_DELTA);
			for(int point=1;point<sWaypoints.size();point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SWITCH_BACK_X_DELTA, Constants.RED_RIGHT_SWITCH_BACK_DELTA_Y);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRightScale() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=getRightScaleFromLeftWaypoints();
		for(int point=2;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SCALE_X_DELTA, Constants.BLUE_RIGHT_SCALE_Y_DELTA);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SCALE_X_DELTA, Constants.RED_RIGHT_SCALE_Y_DELTA);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLeftSwitchBeforeRightScale() {
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=getLeftSwitchBeforeRightScaleWaypoints();
		for(int point=0;point<sWaypoints.size();point++) {
			if(_dashboard.isBlueAlliance()) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
			} else {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, Constants.RED_LEFT_SWITCH_FRONT_Y_DELTA);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptRightScaleAfterLeftSwitch() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints = getRightScaleFromLeftSwitchWaypoints();
		if(_dashboard.isBlueAlliance()) {
			sWaypoints.get(0).adjustWaypoint(Constants.BLUE_LEFT_SWITCH_FRONT_X_DELTA, Constants.BLUE_LEFT_SWITCH_FRONT_Y_DELTA);
			for(int point =1; point<sWaypoints.size(); point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_RIGHT_SCALE_X_DELTA, Constants.BLUE_RIGHT_SCALE_Y_DELTA);
			}
		} else {
			sWaypoints.get(0).adjustWaypoint(Constants.RED_LEFT_SWITCH_FRONT_X_DELTA, Constants.RED_LEFT_SWITCH_FRONT_Y_DELTA);
			for(int point =1; point<sWaypoints.size(); point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_RIGHT_SCALE_X_DELTA, Constants.RED_RIGHT_SCALE_Y_DELTA);
			}
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLSwitchtoLScaleforThirdCube() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints = getRightScaleFromLeftSwitchWaypoints();
		if(_dashboard.isBlueAlliance()) {
			for(int point =0; point<sWaypoints.size(); point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.BLUE_LEFT_SCALE_X_DELTA, Constants.BLUE_LEFT_SCALE_Y_DELTA);
			}
		} else {
			for(int point =0; point<sWaypoints.size(); point++) {
				sWaypoints.get(point).adjustWaypoint(Constants.RED_LEFT_SCALE_X_DELTA, Constants.RED_LEFT_SCALE_Y_DELTA);
			}
		}
		return sWaypoints;
	}
	
	private static void printWaypoints(ArrayList<Waypoint> waypoints) {
		for (int point = 0; point < waypoints.size(); point++) {
			System.out.println(waypoints.get(point).toString());
		}
	}
	
	public static void printWaypointReport() {
		System.out.println("Left Switch: ");
		printWaypoints(AdaptedPaths.adaptLeftSwitch());
		System.out.println("Right Switch: ");
		printWaypoints(AdaptedPaths.adaptRightSwitch());
		
		System.out.println("Left Switch to Front of Pyramid: ");
		printWaypoints(AdaptedPaths.adaptLSwitchtoFrontofPyramid());
		System.out.println("Right Switch to Front of Pyramid: ");
		printWaypoints(AdaptedPaths.adaptRSwitchtoFrontofPyramid());
		System.out.println("S Turn To Left Switch");
		printWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoLeft());
		System.out.println("S Turn To Right Switch");
		printWaypoints(AdaptedPaths.adaptSTurnFromPyramidtoRight());
		
		System.out.println("Left Scale: ");
		printWaypoints(AdaptedPaths.adaptLeftScale());
		System.out.println("Right Scale: ");
		printWaypoints(AdaptedPaths.adaptRightScale());
		System.out.println("Left Scale to Right Switch: ");
		printWaypoints(AdaptedPaths.adaptLScaletoRSwitch());
		System.out.println("Right Scale to Left Switch: ");
		//printWaypoints(AdaptedPaths.adaptRScale)
	}
	
	public static void locateFlavorTownUSA() {
		double flavorTownUSAX = Math.random()*360-180;
		double flavorTownUSAY = Math.random()*360-180;
		String ourLordAndSavior = "Guy Fieri";
		String whatAreWeDoing = "Rolling Out to Find America's Greatest Diners, Drive-ins and Dives";
		System.out.println("Flavor Town USA Located: " + flavorTownUSAX + ", " + flavorTownUSAY);
	}
}