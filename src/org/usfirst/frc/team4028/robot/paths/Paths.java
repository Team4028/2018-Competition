package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

public class Paths {
	public enum PATHS {
		AUTO_RUN,
		
		L_SWITCH,
		L_SWITCH_TO_FRONT_OF_PYRAMID,
		IN_FRONT_OF_PYRAMID_TO_LEFT_SWITCH,
		AWAY_FROM_LEFT_SWITCH,
		FROM_LEFT_OF_PYRAMID_TO_FRONT_OF_SWITCH,
		FRONT_OF_L_SWITCH_TO_SWITCH,
		
		R_SWITCH,
		R_SWITCH_TO_FRONT_OF_PYRAMID,
		IN_FRONT_OF_PYRAMID_TO_RIGHT_SWITCH,
		AWAY_FROM_RIGHT_SWITCH,
		FROM_RIGHT_OF_PYRAMID_TO_FRONT_OF_SWITCH,
		FRONT_OF_R_SWITCH_TO_SWITCH,
		
		TO_PYRAMID,
		AWAY_FROM_PYRAMID,
		PYRAMID_FOR_SECOND_CUBE_FROM_LEFT,
		PYRAMID_FOR_SECOND_CUBE_FROM_RIGHT
		
	}
	
	public static Path getPath(PATHS pathName) {
		Path path;
		switch(pathName) {		
			case AUTO_RUN:
				path = PathBuilder.buildPathFromWaypoints(getAutoRunWaypoints());
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_PYRAMID:
				path = PathBuilder.buildPathFromWaypoints(getAwayFromPyramidWaypoints());
				path.setIsReversed(true);
				return path;
			case IN_FRONT_OF_PYRAMID_TO_LEFT_SWITCH:
				path=PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getFrontofPyramidtoRightSwitchWaypoints()));
				path.setIsReversed(false);
				return path;
			case IN_FRONT_OF_PYRAMID_TO_RIGHT_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(getFrontofPyramidtoRightSwitchWaypoints());
				path.setIsReversed(false);
				return path;
			case L_SWITCH:
		        path = PathBuilder.buildPathFromWaypoints(getLeftSwitchWaypoints());
		        path.setIsReversed(false);
		        return path;
			case L_SWITCH_TO_FRONT_OF_PYRAMID:
				path = PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getRightSwitchtoFrontofPyramidWaypoints()));
				path.setIsReversed(true);
				return path;
			case R_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getLeftSwitchWaypoints()));
				path.setIsReversed(false);
				return path;
			case R_SWITCH_TO_FRONT_OF_PYRAMID:
				path = PathBuilder.buildPathFromWaypoints(getRightSwitchtoFrontofPyramidWaypoints());
				path.setIsReversed(true);
				return path;
			case TO_PYRAMID:
				path = PathBuilder.buildPathFromWaypoints(getToPyramidWaypoints());
				path.setIsReversed(false);
				return path;
			case AWAY_FROM_RIGHT_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(getAwayFromRightSwitchWaypoints());
				path.setIsReversed(true);
				return path;
			case AWAY_FROM_LEFT_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getAwayFromRightSwitchWaypoints()));
				path.setIsReversed(true);
				return path;
			case PYRAMID_FOR_SECOND_CUBE_FROM_RIGHT:
				path=PathBuilder.buildPathFromWaypoints(getToPyramidForSecondCubeFromRightWaypoints());
				path.setIsReversed(false);
				return path;
			case PYRAMID_FOR_SECOND_CUBE_FROM_LEFT:
				path=PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getToPyramidForSecondCubeFromRightWaypoints()));
				path.setIsReversed(false);
				return path;
			case FROM_LEFT_OF_PYRAMID_TO_FRONT_OF_SWITCH:
				path=PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getToPyramidForSecondCubeFromRightWaypoints()));
				path.setIsReversed(true);
				return path;
			case FROM_RIGHT_OF_PYRAMID_TO_FRONT_OF_SWITCH:
				path=PathBuilder.buildPathFromWaypoints(getToPyramidForSecondCubeFromRightWaypoints());
				path.setIsReversed(true);
				return path;
			case FRONT_OF_R_SWITCH_TO_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(getAwayFromRightSwitchWaypoints());
				path.setIsReversed(false);
				return path;
			case FRONT_OF_L_SWITCH_TO_SWITCH:
				path = PathBuilder.buildPathFromWaypoints(PathBuilder.flipPath(getAwayFromRightSwitchWaypoints()));
				path.setIsReversed(false);
				return path;
			default:
				path = PathBuilder.buildPathFromWaypoints(getDoNothingWaypoints());
				path.setIsReversed(true);
				return path;
		}
	}
	
	
	private static ArrayList<Waypoint> getLeftSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(18,162,0,0));
        sWaypoints.add(new Waypoint(50,162,30,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(75,100,30,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(122,100,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getAutoRunWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(18,50,0,0));
        sWaypoints.add(new Waypoint(150,50,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getAwayFromPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(95,162,0,0));
        sWaypoints.add(new Waypoint(45,162,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getRightSwitchtoFrontofPyramidWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(123,224,0,0));
        sWaypoints.add(new Waypoint(85,224,20,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(65,162,20,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(30,162,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getToPyramidWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(30,162,0,0));
        sWaypoints.add(new Waypoint(95,162,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getFrontofPyramidtoRightSwitchWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(45,162,0,0));
        sWaypoints.add(new Waypoint(65,162,18,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(95,224,20,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(122,224,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getDoNothingWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(30,162,0,0));
        sWaypoints.add(new Waypoint(30,162,0,Constants.CelerySpeed));
        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getAwayFromRightSwitchWaypoints() {
	        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	        sWaypoints.add(new Waypoint(122,224,0,0));
	        sWaypoints.add(new Waypoint(65,224,0,Constants.WildTurtleSpeed));
	        return sWaypoints;
	}
	
	private static ArrayList<Waypoint> getToPyramidForSecondCubeFromRightWaypoints() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(65,224,0,0));
        sWaypoints.add(new Waypoint(75,224,10,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(87,198,10,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(105,184,0,Constants.WildTurtleSpeed));
        return sWaypoints;
	}
	
}