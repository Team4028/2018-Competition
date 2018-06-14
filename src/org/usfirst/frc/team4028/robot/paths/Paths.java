package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.Translation;

import static org.usfirst.frc.team4028.robot.paths.PathBuilder.buildPathFromWaypoints;
import static org.usfirst.frc.team4028.robot.paths.PathBuilder.getStraightPathWaypoints;
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
		TO_BACK_CENTER,
		
		// First Cube
		L_SCALE,
		R_SCALE,
		
		L_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_L_SWITCH,
		L_SWITCH_TO_L_SCALE,
		
		R_SCALE_TO_R_SWITCH,
		R_SWITCH_TO_R_SCALE,
		
		L_SCALE_TO_R_SWITCH,
		
		L_SWITCH_SIDE,
		L_SWITCH_SIDE_TO_R_SCALE,
		
		TO_R_SCALE_SECOND_CUBE,
		
		TO_BACK_LEFT,
		
		// Third Cube
		L_SWITCH_TO_L_SCALE_SECOND_CUBE,
		
		L_SCALE_TO_L_SWITCH_THIRD_CUBE,
		L_SWITCH_TO_L_SCALE_THIRD_CUBE
	}
	
	private static Path toBackCenterL;
	private static Path lScalePathL, rScalePathL;
	private static Path lScaleOutsidePathL;
	private static Path lScaleToLSwitchPathL, lSwitchToLScalePathL;
	private static Path rScaleToRSwitchPathL, rSwitchToRScalePathL;
	private static Path lScaleToRSwitchPathL;
	private static Path lSwitchSidePathL, lSwitchSideToRScalePathL;
	private static Path toRScaleSecondCubeL;
	private static Path toBackLeft;
	private static Path lSwitchToLScaleSecondCubeL;
	private static Path lScaleToLSwitchThirdCube, lSwitchToLScaleThirdCube;
	
	public enum Right {
		TO_BACK_CENTER,
		TO_BACK_RIGHT,
		
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_L_SWITCH,
		L_SWITCH_TO_L_SCALE,
		
		R_SCALE_TO_R_SWITCH,
		R_SWITCH_TO_R_SCALE,
		
		R_SWITCH_SIDE,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_SECOND_CUBE,
		
		R_SCALE_TO_R_SWITCH_THIRD_CUBE,
		R_SWITCH_TO_R_SCALE_THIRD_CUBE
	}
	
	private static Path toBackCenterR, toBackR;
	private static Path lScalePathR, rScalePathR;
	private static Path rScaleOutsidePathR;
	private static Path lScaleToLSwitchPathR, lSwitchToLScalePathR;
	private static Path rScaleToRSwitchPathR, rSwitchToRScalePathR;
	private static Path rSwitchSidePathR;
	private static Path rSwitchToRScaleThirdCubeR;
	private static Path rScaleToRSwitchThirdCube, rSwitchToRScaleThirdCube;
	
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
		autoRunPath = buildPathFromWaypoints(getStraightPathWaypoints(new Translation(20, 46), 0, 120));
		centerPaths.put(Center.AUTO_RUN, autoRunPath);
		
		// First Cube
		lSwitchPath = buildPathFromWaypoints(0.0015, Arrays.asList(
						new Waypoint(20,166,0,0),
						new Waypoint(50,166,25,80),
						new Waypoint(90,107,30,80),
						new Waypoint(128,107,0,80)));
		centerPaths.put(Center.L_SWITCH, lSwitchPath);
		
		rSwitchPath = buildPathFromWaypoints(0.0030, Arrays.asList(
						new Waypoint(20,166,0,0),
						new Waypoint(50,166,25,60),
						new Waypoint(90,217,30,60),
						new Waypoint(128,217,0,60)));
		centerPaths.put(Center.R_SWITCH, rSwitchPath);
		
		// Second Cube
		lSwitchToPyramidFrontPath = buildPathFromWaypoints(true, Arrays.asList(
						new Waypoint(123,107,0,0),
						new Waypoint(95,107,20,60),
						new Waypoint(65,170,20,60),
						new Waypoint(45,170,0,50)));
		centerPaths.put(Center.L_SWITCH_TO_PYRAMID_FRONT, lSwitchToPyramidFrontPath);
		
		rSwitchToPyramidFrontPath = buildPathFromWaypoints(true, Arrays.asList(
						new Waypoint(123,217,0,0),
						new Waypoint(95,217,20,60),
						new Waypoint(65,158,20,60),
						new Waypoint(45,158,0,50)));
		centerPaths.put(Center.R_SWITCH_TO_PYRAMID_FRONT, rSwitchToPyramidFrontPath);
		
		ArrayList<Waypoint> toPyramidWaypoints = new ArrayList<Waypoint>();
		toPyramidWaypoints.add(new Waypoint(45,162,0,0));
		toPyramidWaypoints.add(new Waypoint(82,162,0,40));
		toPyramidPath = buildPathFromWaypoints(toPyramidWaypoints);
		centerPaths.put(Center.TO_PYRAMID, toPyramidPath);
		
		fromPyramidPath = buildPathFromWaypoints(true, reversePath(toPyramidWaypoints));
		centerPaths.put(Center.FROM_PYRAMID, fromPyramidPath);
		
		sTurnToLSwitchPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(45,162,0,0),
						new Waypoint(65,162,20,60),
						new Waypoint(95,107,30,60),
						new Waypoint(128,107,0,60)));
		centerPaths.put(Center.S_TURN_TO_L_SWITCH, sTurnToLSwitchPath);
		
		sTurnToRSwitchPath = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(45,162,0,0),
						new Waypoint(65,162,20,60),
						new Waypoint(95,217,30,60),
						new Waypoint(128,217,0,60)));
		centerPaths.put(Center.S_TURN_TO_R_SWITCH, sTurnToRSwitchPath);
		
		// Third Cube
		ArrayList<Waypoint> awayFromLSwitchWaypoints = new ArrayList<Waypoint>();
		awayFromLSwitchWaypoints.add(new Waypoint(122,115,0,0));
		awayFromLSwitchWaypoints.add(new Waypoint(60,115,0,80));
		awayFromLSwitchPath = buildPathFromWaypoints(true, awayFromLSwitchWaypoints);
		centerPaths.put(Center.AWAY_FROM_L_SWITCH, awayFromLSwitchPath);
		
		awayFromRSwitchPath = buildPathFromWaypoints(true, flipPath(awayFromLSwitchWaypoints));
		centerPaths.put(Center.AWAY_FROM_R_SWITCH, awayFromRSwitchPath);
		
		ArrayList<Waypoint> pyramidAgainFromLeftWaypoints = new ArrayList<Waypoint>();
		pyramidAgainFromLeftWaypoints.add(new Waypoint(60,115,0,0));
		pyramidAgainFromLeftWaypoints.add(new Waypoint(77,115,15,60));
		pyramidAgainFromLeftWaypoints.add(new Waypoint(102,140,0,60));
		pyramidAgainFromLeftPath = buildPathFromWaypoints(pyramidAgainFromLeftWaypoints);
		centerPaths.put(Center.PYRAMID_AGAIN_FROM_L, pyramidAgainFromLeftPath);
		
		pyramidAgainFromRightPath= buildPathFromWaypoints(Arrays.asList(
				new Waypoint(60,209,0,0),
				new Waypoint(77,209,15,60),
				new Waypoint(106,184,0,60)));
		centerPaths.put(Center.PYRAMID_AGAIN_FROM_R, pyramidAgainFromRightPath);
	}
	
	private static void buildLeftPaths() {
		toBackCenterL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,46,0,0),
				        new Waypoint(240,46,50,60),
				        new Waypoint(240,120,0,60)));
		leftPaths.put(Left.TO_BACK_CENTER, toBackCenterL);
		
		// First Cube
		lScalePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(220,46,50,140),
						new Waypoint(275,73,0,140)));
		leftPaths.put(Left.L_SCALE, lScalePathL);
		
		rScalePathL = buildPathFromWaypoints(0.002, Arrays.asList(
						new Waypoint(20,46,0,0),
						new Waypoint(190,46,0,140),//120
						new Waypoint(240,46,50,90),//90
						new Waypoint(240,96,0,90),//90
						new Waypoint(240,104,0,80),//40
						new Waypoint(240,202,0,140),//120
						new Waypoint(240,210,0,60),
						new Waypoint(240,257,34,80),//60
						new Waypoint(275,257,0,40)));
		leftPaths.put(Left.R_SCALE, rScalePathL);
		
		lScaleOutsidePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,46,0,0),
						new Waypoint(120,46,40,100),
						new Waypoint(220,34,40,100),
						new Waypoint(320,34,0,100)));
		leftPaths.put(Left.L_SCALE_OUTSIDE, lScaleOutsidePathL);
		
		// Second Cube
		ArrayList<Waypoint> lScaleToLSwitchWaypoints = getStraightPathWaypoints(new Translation(277, 72), 160, 39);
		lScaleToLSwitchPathL = buildPathFromWaypoints(lScaleToLSwitchWaypoints);
		leftPaths.put(Left.L_SCALE_TO_L_SWITCH, lScaleToLSwitchPathL);
		
		lSwitchToLScalePathL = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(lScaleToLSwitchWaypoints.get(1).pos), 160, -39));
		leftPaths.put(Left.L_SWITCH_TO_L_SCALE, lSwitchToLScalePathL);
		
		ArrayList<Waypoint> rScaleToRSwitchWaypoints = getStraightPathWaypoints(new Translation(278, 252), -160, 44);
		rScaleToRSwitchPathL = buildPathFromWaypoints(rScaleToRSwitchWaypoints);
		leftPaths.put(Left.R_SCALE_TO_R_SWITCH, rScaleToRSwitchPathL);
		
		rSwitchToRScalePathL = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(rScaleToRSwitchWaypoints.get(1).pos), -160, -42));
		leftPaths.put(Left.R_SWITCH_TO_R_SCALE, rSwitchToRScalePathL);
		
		lScaleToRSwitchPathL = buildPathFromWaypoints(0.006, Arrays.asList(
						new Waypoint(273,74,0,0),
						new Waypoint(247,87,28,80),//80
						new Waypoint(250,260,0,120)));//120
		leftPaths.put(Left.L_SCALE_TO_R_SWITCH, lScaleToRSwitchPathL);
		
		lSwitchSidePathL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,46,0,0),
						new Waypoint(126,46,15,80),
       					new Waypoint(137,59,0,70)));
        leftPaths.put(Left.L_SWITCH_SIDE, lSwitchSidePathL);
        
        lSwitchSideToRScalePathL = buildPathFromWaypoints(0.001, Arrays.asList(
        				new Waypoint(140,61,0,0),
        				new Waypoint(240,61,40,80),
        				new Waypoint(240,102,0,80),
        				new Waypoint(240,114,0,25),
        				new Waypoint(240,263,0,120)));
		leftPaths.put(Left.L_SWITCH_SIDE_TO_R_SCALE, lSwitchSideToRScalePathL);
		
		toRScaleSecondCubeL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(235,241,0,0),
						new Waypoint(248,251,13,60),
						new Waypoint(266,246,0,60)));
		leftPaths.put(Left.TO_R_SCALE_SECOND_CUBE, toRScaleSecondCubeL);
		
		toBackLeft = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(137,59,0,0),
					    new Waypoint(201,59,30,100),
					    new Waypoint(236,87,0,100)));
		leftPaths.put(Left.TO_BACK_LEFT, toBackLeft);
		
		// Third Cube
		lSwitchToLScaleSecondCubeL = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(229,96,0,0),
						new Waypoint(251,77,20,70),
						new Waypoint(273,80,0,70)));
		leftPaths.put(Left.L_SWITCH_TO_L_SCALE_SECOND_CUBE, lSwitchToLScaleSecondCubeL);
		
		ArrayList<Waypoint> lScaleToLSwitchThirdCubeWaypoints = getStraightPathWaypoints(new Translation(278, 68), 140, 52);
		lScaleToLSwitchThirdCube = buildPathFromWaypoints(lScaleToLSwitchThirdCubeWaypoints);
		leftPaths.put(Left.L_SCALE_TO_L_SWITCH_THIRD_CUBE, lScaleToLSwitchThirdCube);
		
		lSwitchToLScaleThirdCube = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(lScaleToLSwitchThirdCubeWaypoints.get(1).pos), 140, -52));
		leftPaths.put(Left.L_SWITCH_TO_L_SCALE_THIRD_CUBE, lSwitchToLScaleThirdCube);
	}
	
	private static void buildRightPaths() {
		toBackCenterR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,278,0,0),
				        new Waypoint(240,278,50,60),
				        new Waypoint(240,204,0,60)));
		rightPaths.put(Right.TO_BACK_CENTER, toBackCenterR);
		// First Cube
		lScalePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(190,278,0,120),
						new Waypoint(240,278,40,80),
						new Waypoint(240,228,0,80),
						new Waypoint(240,220,0,80),
						new Waypoint(240,114,0,120),
						new Waypoint(240,72,35,100),
						new Waypoint(278,72,0,80)));
		rightPaths.put(Right.L_SCALE, lScalePathR);
		
		rScalePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(20,278,0,0),
						new Waypoint(225,278,50,140),
						new Waypoint(278,253,0,140)));
		rightPaths.put(Right.R_SCALE, rScalePathR);
		
		rScaleOutsidePathR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(12,278,0,0),
						new Waypoint(120,278,40,100),
						new Waypoint(220,290,40,100),
						new Waypoint(320,290,0,100)));
		rightPaths.put(Right.R_SCALE_OUTSIDE, rScaleOutsidePathR);
		
		// Second Cube
		ArrayList<Waypoint> lScaleToLSwitchWaypoints = getStraightPathWaypoints(new Translation(280, 74), 160, 38);
		lScaleToLSwitchPathR = buildPathFromWaypoints(lScaleToLSwitchWaypoints);
		rightPaths.put(Right.L_SCALE_TO_L_SWITCH, lScaleToLSwitchPathR);
		
		lSwitchToLScalePathR = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(lScaleToLSwitchWaypoints.get(1).pos), 160, -38));
		rightPaths.put(Right.L_SWITCH_TO_L_SCALE, lSwitchToLScalePathR);
		
		ArrayList<Waypoint> rScaleToRSwitchWaypoints = getStraightPathWaypoints(new Translation(281, 256), -163, 36);
		rScaleToRSwitchPathR = buildPathFromWaypoints(rScaleToRSwitchWaypoints);
		rightPaths.put(Right.R_SCALE_TO_R_SWITCH, rScaleToRSwitchPathR);
		
		rSwitchToRScalePathR = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(rScaleToRSwitchWaypoints.get(1).pos), -163, -36));
		rightPaths.put(Right.R_SWITCH_TO_R_SCALE, rSwitchToRScalePathR);
		
		rSwitchSidePathR = buildPathFromWaypoints(Arrays.asList(
					new Waypoint(12,278,0,0),
					new Waypoint(126,278,15,80),
					new Waypoint(137,260,0,70)));
        rightPaths.put(Right.R_SWITCH_SIDE, rSwitchSidePathR);
        
        toBackR = buildPathFromWaypoints(Arrays.asList(
				new Waypoint(137,265,0,0),
			    new Waypoint(201,265,30,100),
			    new Waypoint(236,237,0,100)));
        rightPaths.put(Right.TO_BACK_RIGHT, toBackR);
        
		
		// Third Cube
		rSwitchToRScaleThirdCubeR = buildPathFromWaypoints(Arrays.asList(
						new Waypoint(220,232,0,0),
						new Waypoint(240,256,30,60),
						new Waypoint(270,244,0,60)));
		rightPaths.put(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE, rSwitchToRScaleThirdCubeR);
		
		ArrayList<Waypoint> rScaleToRSwitchThirdCubeWaypoints = getStraightPathWaypoints(new Translation(281, 56), -144, 50);
		rScaleToRSwitchThirdCube = buildPathFromWaypoints(rScaleToRSwitchThirdCubeWaypoints);
		rightPaths.put(Right.R_SCALE_TO_R_SWITCH_THIRD_CUBE, rScaleToRSwitchThirdCube);
		
		rSwitchToRScaleThirdCube = buildPathFromWaypoints(true, getStraightPathWaypoints(new Translation(rScaleToRSwitchThirdCubeWaypoints.get(1).pos), -144, -50));
		rightPaths.put(Right.R_SWITCH_TO_R_SCALE_THIRD_CUBE, rSwitchToRScaleThirdCube);
	}
}