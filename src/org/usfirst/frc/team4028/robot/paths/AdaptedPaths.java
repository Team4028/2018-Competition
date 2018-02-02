package org.usfirst.frc.team4028.robot.paths;
import java.util.*;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
public class AdaptedPaths extends Paths
{
	public static ArrayList<Waypoint> adaptLeftSwitch()
	{
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getLeftSwitchWaypoints();
		sWaypoints.get(2).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		sWaypoints.get(3).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptLSwitchtoFrontOfPyramid()
	{
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=PathBuilder.flipPath(Paths.getRightSwitchtoFrontofPyramidWaypoints());
		for(int point=0;point<sWaypoints.size();point++)
		{
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptSTurnFromPyramidtoLeft()
	{
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints= Paths.getSTurnFromPyramidtoLeftWaypoints();
		for(int point=0;point<sWaypoints.size();point++)
		{
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptToLSwitchAfterSTurn()
	{
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.gettoLeftSwitchAfterSTurn();
		for(int point=0;point<sWaypoints.size();point++)
		{
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
	
	public static ArrayList<Waypoint> adaptAwayFromLeftSwitch()
	{
		ArrayList<Waypoint> sWaypoints= new ArrayList<Waypoint>();
		sWaypoints=Paths.getAwayFromLeftSwitchForThirdCubeWaypoints();
		for(int point=0;point<sWaypoints.size();point++)
		{
			sWaypoints.get(point).adjustWaypoint(Constants.LEFT_SWITCH_FRONT_X_DELTA, Constants.LEFT_SWITCH_FRONT_Y_DELTA);
		}
		return sWaypoints;
	}
}