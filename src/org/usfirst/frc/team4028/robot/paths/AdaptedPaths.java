package org.usfirst.frc.team4028.robot.paths;

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