package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.flipPath;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.reversePath;

public class Paths {
	private static Hashtable<Center, Path> centerPaths = new Hashtable<Center, Path>();
	private static Hashtable<Left, Path> leftPaths = new Hashtable<Left, Path>();
	private static Hashtable<Right, Path> rightPaths = new Hashtable<Right, Path>();
	
	public enum Center {
		AUTO_RUN,
		
		// First Cube
		L_SWITCH,
		R_SWITCH,
		
		// Second Cube
		L_SWITCH_TO_PYRAMID_FRONT,
		R_SWITCH_TO_PYRAMID_FRONT,
		
		TO_PYRAMID,
		FROM_PYRAMID,
		
		S_TURN_TO_L_SWITCH,
		S_TURN_TO_R_SWITCH,
		
		// Third Cube
		AWAY_FROM_L_SWITCH,
		PYRAMID_AGAIN_FROM_L,

		AWAY_FROM_R_SWITCH,
		PYRAMID_AGAIN_FROM_R,
	}
	
	private static Path autoRunPath;
	private static Path lSwitchPath, rSwitchPath;
	private static Path lSwitchToPyramidFrontPath, rSwitchToPyramidFrontPath;
	private static Path toPyramidPath, fromPyramidPath;
	private static Path sTurnToLSwitchPath, sTurnToRSwitchPath;
	private static Path awayFromLSwitchPath, awayFromRSwitchPath;
	private static Path pyramidAgainFromLeftPath, pyramidAgainFromRightPath;
	
	public enum Left {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		L_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_R_SWITCH,
		
		L_SWITCH_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		TO_R_SCALE_SECOND_CUBE,
		
		// Third Cube
		L_SWITCH_TO_L_SCALE_THIRD_CUBE
	}
	
	private static Path lScalePathL, rScalePathL;
	private static Path lScaleOutsidePathL;
	private static Path lScaleToRSwitchPathL;
	private static Path lSwitchSidePathL, lSwitchSideToRScalePathL;
	private static Path toRScaleSecondCubeL;
	private static Path lSwitchToLScaleThirdCubeL;
	
	public enum Right {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		
		// Second Cube
		R_SCALE_TO_L_SWITCH,
		
		R_SWITCH_SIDE,
		R_SWITCH_SIDE_TO_L_SCALE,
		
		TO_L_SCALE_SECOND_CUBE,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_THIRD_CUBE
	}
	
	private static Path lScalePathR, rScalePathR;
	private static Path rScaleOutsidePathR;
	private static Path rScaleToLSwitchPathR;
	private static Path rSwitchSidePathR, rSwitchSideToLScalePathR;
	private static Path toLScaleSecondCubeR;
	private static Path rSwitchToRScaleThirdCubeR;
	
	public static void buildPaths() {
		buildCenterPaths();
		buildLeftPaths();
		buildRightPaths();
	}
	
	public static Path getPath(Center pathName) {
		return centerPaths.get(pathName);
	}
	
	public static Path getPath(Left pathName) {
		return leftPaths.get(pathName);
	}
	
	public static Path getPath(Right pathName) {
		return rightPaths.get(pathName);
	}
	
	private static void buildCenterPaths() {
		// Auto Run
		autoRunPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(140,46,0,120)));
		centerPaths.put(Center.AUTO_RUN, autoRunPath);
		
		// First Cube
		lSwitchPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,166,0,0),
						new Waypoint(50,166,25,80),
						new Waypoint(90,107,30,70),
						new Waypoint(128,107,0,70)));
		centerPaths.put(Center.L_SWITCH, lSwitchPath);
		
		rSwitchPath = buildPathFromWaypoints(0.0025, Arrays.asList(
						new Waypoint(20,166,0,0),
						new Waypoint(50,166,25,80),
						new Waypoint(90,217,30,70),
						new Waypoint(128,217,0,70)));
		centerPaths.put(Center.R_SWITCH, rSwitchPath);
		
		// Second Cube
		lSwitchToPyramidFrontPath = buildPathFromWaypoints(true, Arrays.asList(
						new Waypoint(123,107,0,0),
						new Waypoint(95,107,20,70),
						new Waypoint(65,170,20,60),
						new Waypoint(45,170,0,50)));
		centerPaths.put(Center.L_SWITCH_TO_PYRAMID_FRONT, lSwitchToPyramidFrontPath);
		
		rSwitchToPyramidFrontPath = buildPathFromWaypoints(true, Arrays.asList(
						new Waypoint(123,217,0,0),
						new Waypoint(95,217,20,70),
						new Waypoint(65,158,20,60),
						new Waypoint(45,158,0,50)));
		centerPaths.put(Center.R_SWITCH_TO_PYRAMID_FRONT, rSwitchToPyramidFrontPath);
		
		ArrayList<Waypoint> toPyramidWaypoints = new ArrayList<Waypoint>();
		toPyramidWaypoints.add(new Waypoint(45,162,0,0));
		toPyramidWaypoints.add(new Waypoint(82,162,0,60));
		toPyramidPath = buildPathFromWaypoints(toPyramidWaypoints);
		centerPaths.put(Center.TO_PYRAMID, toPyramidPath);
		
		fromPyramidPath = buildPathFromWaypoints(true, reversePath(toPyramidWaypoints));
		centerPaths.put(Center.FROM_PYRAMID, fromPyramidPath);
		
		sTurnToLSwitchPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(45,162,0,0),
						new Waypoint(65,162,20,70),
						new Waypoint(95,107,30,60),
						new Waypoint(128,107,0,60)));
		centerPaths.put(Center.S_TURN_TO_L_SWITCH, sTurnToLSwitchPath);
		
		sTurnToRSwitchPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(45,162,0,0),
						new Waypoint(65,162,20,70),
						new Waypoint(95,217,30,60),
						new Waypoint(128,217,0,60)));
		centerPaths.put(Center.S_TURN_TO_R_SWITCH, sTurnToRSwitchPath);
		
		// Third Cube
		ArrayList<Waypoint> awayFromLSwitchWaypoints = new ArrayList<Waypoint>();
		awayFromLSwitchWaypoints.add(new Waypoint(122,115,0,0));
		awayFromLSwitchWaypoints.add(new Waypoint(60,115,0,Constants.FLOOR_IT_SPEED));
		awayFromLSwitchPath = buildPathFromWaypoints(awayFromLSwitchWaypoints);
		centerPaths.put(Center.AWAY_FROM_L_SWITCH, awayFromLSwitchPath);
		
		awayFromRSwitchPath = buildPathFromWaypoints(flipPath(awayFromLSwitchWaypoints));
		centerPaths.put(Center.AWAY_FROM_R_SWITCH, awayFromRSwitchPath);
		
		ArrayList<Waypoint> pyramidAgainFromLeftWaypoints = new ArrayList<Waypoint>();
		pyramidAgainFromLeftWaypoints.add(new Waypoint(60,115,0,0));
		pyramidAgainFromLeftWaypoints.add(new Waypoint(77,115,15,Constants.FLOOR_IT_SPEED));
		pyramidAgainFromLeftWaypoints.add(new Waypoint(102,140,0,Constants.NORMAL_SPEED));
		pyramidAgainFromLeftPath = buildPathFromWaypoints(pyramidAgainFromLeftWaypoints);
		centerPaths.put(Center.PYRAMID_AGAIN_FROM_L, pyramidAgainFromLeftPath);
		
		pyramidAgainFromRightPath= buildPathFromWaypoints(flipPath(pyramidAgainFromLeftWaypoints));
		centerPaths.put(Center.PYRAMID_AGAIN_FROM_R, pyramidAgainFromRightPath);
	}
	
	private static void buildLeftPaths() {
		// First Cube
		lScalePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(233,46,40,120),
						new Waypoint(278,68,0,120)));
		leftPaths.put(Left.L_SCALE, lScalePathL);
		
		rScalePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(198,46,0,120),
						new Waypoint(240,46,40,80),
						new Waypoint(240,86,0,80),
						new Waypoint(240,98,0,30),
						new Waypoint(240,205,0,120),
						new Waypoint(240,248,32,80),
						new Waypoint(274,248,0,40)));
		leftPaths.put(Left.R_SCALE, rScalePathL);
		
		lScaleOutsidePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(300,46,0,120)));
		leftPaths.put(Left.L_SCALE_OUTSIDE, lScaleOutsidePathL);
		
		// Second Cube
		lScaleToRSwitchPathL = buildPathFromWaypoints(0.001, Arrays.asList(
						new Waypoint(273,74,0,0),
						new Waypoint(247,87,28,80),
						new Waypoint(250,256,0,120)));
		leftPaths.put(Left.L_SCALE_TO_R_SWITCH, lScaleToRSwitchPathL);
		
		lSwitchSidePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,46,0,0),
						new Waypoint(126,46,15,80),
       					new Waypoint(137,59,0,70)));
        leftPaths.put(Left.L_SWITCH_SIDE, lSwitchSidePathL);
        
        lSwitchSideToRScalePathL = buildPathFromWaypoints(Arrays.asList(
        				new Waypoint(140,61,0,0),
        				new Waypoint(243,61,40,80),
        				new Waypoint(243,102,0,80),
        				new Waypoint(243,114,0,25),
        				new Waypoint(243,260,0,120)));
		leftPaths.put(Left.L_SWITCH_SIDE_TO_R_SCALE, lSwitchSideToRScalePathL);
		
		
		toRScaleSecondCubeL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(235,241,0,0),
						new Waypoint(248,251,13,60),
						new Waypoint(266,246,0,60)));
		leftPaths.put(Left.L_SWITCH_SIDE_TO_R_SCALE, toRScaleSecondCubeL);
		
		// Third Cube
		lSwitchToLScaleThirdCubeL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(229,96,0,0),
						new Waypoint(251,77,20,70),
						new Waypoint(273,80,0,70)));
		leftPaths.put(Left.L_SWITCH_TO_L_SCALE_THIRD_CUBE, lSwitchToLScaleThirdCubeL);
	}
	
	private static void buildRightPaths() {
		// First Cube
		lScalePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(191,278,0,120),
						new Waypoint(242,278,40,120),
						new Waypoint(242,228,0,120),
						new Waypoint(242,214,0,20),
						new Waypoint(242,109,0,100),
						new Waypoint(242,74,35,80),
						new Waypoint(268,74,0,40)));
		rightPaths.put(Right.L_SCALE, lScalePathR);
		
		rScalePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(229,278,30,120),
						new Waypoint(281,250,0,100)));
		rightPaths.put(Right.R_SCALE, rScalePathR);
		
		rScaleOutsidePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(300,278,0,120)));
		rightPaths.put(Right.R_SCALE_OUTSIDE, rScaleOutsidePathR);
		
		// Second Cube
		rScaleToLSwitchPathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(279,244,0,0),
						new Waypoint(247,228,25,80),
						new Waypoint(247,117,0,120)));
		rightPaths.put(Right.R_SCALE_TO_L_SWITCH, rScaleToLSwitchPathR);
		
		rSwitchSidePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(165,278,15,60),
						new Waypoint(165,261,0,60)));
        rightPaths.put(Right.R_SWITCH_SIDE, rSwitchSidePathR);
        
        rSwitchSideToLScalePathR = buildPathFromWaypoints(Arrays.asList(
        				new Waypoint(165,261,0,0),
       					new Waypoint(165,281,15,120),
        				new Waypoint(242,281,40,120),
        				new Waypoint(242,80,0,60)));
		rightPaths.put(Right.R_SWITCH_SIDE_TO_L_SCALE, rSwitchSideToLScalePathR);
		
		
		toLScaleSecondCubeR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(235,83,0,0),
						new Waypoint(248,73,13,60),
						new Waypoint(266,78,0,60)));
		rightPaths.put(Right.TO_L_SCALE_SECOND_CUBE, toLScaleSecondCubeR);
		
		// Third Cube
		rSwitchToRScaleThirdCubeR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(220,232,0,0),
						new Waypoint(240,256,30,60),
						new Waypoint(270,244,0,60)));
		rightPaths.put(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE, rSwitchToRScaleThirdCubeR);
	}
	/*
	private static Hashtable<Center, Path> centerPaths = new Hashtable<Center, Path>();
	private static Hashtable<Left, Path> leftPaths = new Hashtable<Left, Path>();
	private static Hashtable<Right, Path> rightPaths = new Hashtable<Right, Path>();
	
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
	
	public enum Left {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		L_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_R_SWITCH,
		
		L_SWITCH_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		TO_R_SCALE_SECOND_CUBE,
		
		// Third Cube
		L_SWITCH_TO_L_SCALE_SECOND_CUBE
	}
	
	public enum Right {
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		
		// Second Cube
		R_SCALE_TO_L_SWITCH,
		
		R_SWITCH_SIDE,
		R_SWITCH_SIDE_TO_L_SCALE,
		
		TO_L_SCALE_SECOND_CUBE,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_SECOND_CUBE
	}
	
	/*
	public static void buildPaths() {
		buildCenterPaths();
		buildLeftPaths();
		buildRightPaths();
	}
	
	public static Path getPath(Center pathName) {
		return centerPaths.get(pathName);
	}
	
	public static Path getPath(Left pathName) {
		return leftPaths.get(pathName);
	}
	
	public static Path getPath(Right pathName) {
		return rightPaths.get(pathName);
	}
	
	private static void buildCenterPaths() {
		// Auto Run
		centerPaths.put(Center.AUTO_RUN, buildPathFromWaypoints(getAutoRunWaypoints()));
		
		// First Cube
		centerPaths.put(Center.L_SWITCH, buildPathFromWaypoints(getLeftSwitchWaypoints()));
		centerPaths.put(Center.R_SWITCH, buildPathFromWaypoints(getRightSwitchWaypoints(), 0.0025));
		
		// Second Cube
		centerPaths.put(Center.L_SWITCH_TO_FRONT_OF_PYRAMID, buildPathFromWaypoints(getLeftSwitchtoFrontofPyramidWaypoints(), true));
		centerPaths.put(Center.R_SWITCH_TO_FRONT_OF_PYRAMID, buildPathFromWaypoints(getRightSwitchtoFrontofPyramidWaypoints(), true));
		
		centerPaths.put(Center.TO_PYRAMID, buildPathFromWaypoints(getToPyramidWaypoints()));
		centerPaths.put(Center.FROM_PYRAMID, buildPathFromWaypoints(reversePath(getToPyramidWaypoints()), true));
		
		centerPaths.put(Center.S_TURN_TO_L_SWITCH, buildPathFromWaypoints(getFrontofPyramidtoLeftSwitchWaypoints()));
		centerPaths.put(Center.S_TURN_TO_R_SWITCH, buildPathFromWaypoints(getFrontofPyramidtoRightSwitchWaypoints()));
		
		// Third Cube
		centerPaths.put(Center.AWAY_FROM_LEFT_SWITCH, buildPathFromWaypoints(getAwayFromLeftSwitchForThirdCubeWaypoints(), true));
		centerPaths.put(Center.PYRAMID_FOR_THIRD_CUBE_FROM_LEFT, buildPathFromWaypoints(gettoLeftPyramidForThirdCubeWaypoints()));
		centerPaths.put(Center.AWAY_FROM_L_PYRAMID, buildPathFromWaypoints(reversePath(gettoLeftPyramidForThirdCubeWaypoints()), true));
		centerPaths.put(Center.TO_L_SWITCH_WITH_CUBE_3, buildPathFromWaypoints(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints())));
		
		centerPaths.put(Center.AWAY_FROM_RIGHT_SWITCH, buildPathFromWaypoints(flipPath(getAwayFromLeftSwitchForThirdCubeWaypoints()), true));
		centerPaths.put(Center.PYRAMID_FOR_THIRD_CUBE_FROM_RIGHT, buildPathFromWaypoints(flipPath(gettoLeftPyramidForThirdCubeWaypoints())));
		centerPaths.put(Center.AWAY_FROM_R_PYRAMID, buildPathFromWaypoints(flipPath(reversePath(gettoLeftPyramidForThirdCubeWaypoints())), true));
		centerPaths.put(Center.TO_R_SWITCH_WITH_CUBE_3, buildPathFromWaypoints(flipPath(reversePath(getAwayFromLeftSwitchForThirdCubeWaypoints()))));
	}
	
	private static void buildLeftPaths() {
		// First Cube
		leftPaths.put(Left.L_SCALE, buildPathFromWaypoints(getLeftScaleFromLeftWaypoints()));
		leftPaths.put(Left.R_SCALE, buildPathFromWaypoints(getRightScaleFromLeftWaypoints()));
		leftPaths.put(Left.L_SCALE_OUTSIDE, buildPathFromWaypoints(getLeftScaleOutsideWaypoints()));
		
		// Second Cube
		leftPaths.put(Left.L_SCALE_TO_R_SWITCH, buildPathFromWaypoints(getLeftScaleToRightSwitchLeftSideWaypoints(), 0.001));
		leftPaths.put(Left.L_SWITCH_SIDE, buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints()));
		leftPaths.put(Left.L_SWITCH_SIDE_TO_R_SCALE, buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), 0.001));
		leftPaths.put(Left.TO_R_SCALE_SECOND_CUBE, buildPathFromWaypoints(getRightScalewithSecondCube()));
		
		// Third Cube
		leftPaths.put(Left.L_SWITCH_TO_L_SCALE_SECOND_CUBE, buildPathFromWaypoints(getLeftSwitchToLeftScaleThirdCubeWaypoints()));
	}
	
	private static void buildRightPaths() {
		// First Cube
		rightPaths.put(Right.L_SCALE, buildPathFromWaypoints(getLeftScaleFromRightWaypoints()));
		rightPaths.put(Right.R_SCALE, buildPathFromWaypoints(getRightScaleFromRightWaypoints()));
		rightPaths.put(Right.R_SCALE_OUTSIDE, buildPathFromWaypoints(getRightScaleOutsideWaypoints()));
		
		// Second Cube
		rightPaths.put(Right.R_SCALE_TO_L_SWITCH, buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints()));
		rightPaths.put(Right.R_SWITCH_SIDE, buildPathFromWaypoints(getRightSwitchBeforeLeftScaleWaypoints()));
		rightPaths.put(Right.R_SWITCH_SIDE_TO_L_SCALE, buildPathFromWaypoints(getLeftScaleAfterRightSwitchWaypoints()));
		rightPaths.put(Right.TO_L_SCALE_SECOND_CUBE, buildPathFromWaypoints(getRightScalewithSecondCube()));
		
		// Third Cube
		rightPaths.put(Right.R_SWITCH_TO_R_SCALE_SECOND_CUBE, buildPathFromWaypoints(getRightSwitchToRightScaleThirdCubeWaypoints()));
	}
	
	/*
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
	*/
	/*
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
        sWaypoints.add(new Waypoint(128,107,0,70));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,166,0,0));
        sWaypoints.add(new Waypoint(50,166,25,80));
        sWaypoints.add(new Waypoint(90,217,30,70));
        sWaypoints.add(new Waypoint(128,217,0,70));
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
        sWaypoints.add(new Waypoint(128,107,0,60));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getFrontofPyramidtoRightSwitchWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(45,162,0,0));
		 sWaypoints.add(new Waypoint(65,162,20,70));
		 sWaypoints.add(new Waypoint(95,217,30,60));
        sWaypoints.add(new Waypoint(128,217,0,60));
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
	
	/*
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
				path = buildPathFromWaypoints(getLeftScaleToRightSwitchLeftSideWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.001);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_TO_L_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getLeftSwitchToLeftScaleThirdCubeWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			case L_SWITCH_SIDE:
				path = buildPathFromWaypoints(getLeftSwitchBeforeRightScaleWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
			case L_SWITCH_SIDE_TO_R_SCALE:
				path = buildPathFromWaypoints(getRightScaleFromLeftSwitchWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.001);
				path.setIsReversed(false);
				return path;
				
			case TO_R_SCALE_SECOND_CUBE:
				path = buildPathFromWaypoints(getRightScalewithSecondCube(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path;
				
			default:
				path = buildPathFromWaypoints(getDoNothingWaypoints(), Constants.PATH_DEFAULT_ACCEL, Constants.PATH_DEFAULT_DECEL, 0.0);
				path.setIsReversed(false);
				return path; 
		}
	} */
	
	/*
	// Scale
	protected static ArrayList<Waypoint>  getLeftScaleFromLeftWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(233,46,40,120));
        sWaypoints.add(new Waypoint(278,68,0,120));
    	return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,46,0,0));
        sWaypoints.add(new Waypoint(198,46,0,120));
        sWaypoints.add(new Waypoint(240,46,40,80));
        sWaypoints.add(new Waypoint(240,86,0,80));
        sWaypoints.add(new Waypoint(240,98,0,30));
        sWaypoints.add(new Waypoint(240,205,0,120));
        sWaypoints.add(new Waypoint(240,248,32,80));
        sWaypoints.add(new Waypoint(274,248,0,40));
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
        sWaypoints.add(new Waypoint(250,256,0,120));
        return sWaypoints;
	}
	
	// Triple Scale
	protected static ArrayList<Waypoint> getLeftSwitchToLeftScaleThirdCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(229,96,0,0));
        sWaypoints.add(new Waypoint(251,77,20,70));
        sWaypoints.add(new Waypoint(273,80,0,70));
	    return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getLeftSwitchBeforeRightScaleWaypoints(){
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(12,46,0,0));
        sWaypoints.add(new Waypoint(126,46,15,80));
        sWaypoints.add(new Waypoint(137,59,0,70));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScaleFromLeftSwitchWaypoints(){
	    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	    sWaypoints.add(new Waypoint(140,61,0,0));
        sWaypoints.add(new Waypoint(243,61,40,80));
        sWaypoints.add(new Waypoint(243,102,0,80));
        sWaypoints.add(new Waypoint(243,114,0,25));
        sWaypoints.add(new Waypoint(243,260,0,120));
        return sWaypoints;
	}
	
	protected static ArrayList<Waypoint> getRightScalewithSecondCube(){
	    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
	    sWaypoints.add(new Waypoint(235,241,0,0));
        sWaypoints.add(new Waypoint(248,251,13,60));
        sWaypoints.add(new Waypoint(266,246,0,60));
        return sWaypoints;
	}
	
	/*
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
	} */
	
	/*
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
	
	protected static ArrayList<Waypoint> getLeftScaleAfterRightSwitchWaypoints(){
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
	protected static ArrayList<Waypoint> getRightScaletoLeftSwitchRightSideWaypoints(){
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(279,244,0,0));
        sWaypoints.add(new Waypoint(247,228,25,80));
        sWaypoints.add(new Waypoint(247,117,0,120));
    	return sWaypoints;
	}
	
	// Second cube switch to scale
	protected static ArrayList<Waypoint> getRightSwitchToRightScaleThirdCubeWaypoints() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(220,232,0,0));
	    sWaypoints.add(new Waypoint(240,256,30,60));
	    sWaypoints.add(new Waypoint(270,244,0,60));
	    return sWaypoints;
	}
	*/
}