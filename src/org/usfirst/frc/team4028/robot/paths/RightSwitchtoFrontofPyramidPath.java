package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.*;

public class RightSwitchtoFrontofPyramidPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		        sWaypoints.add(new Waypoint(122,224,0,0));
        sWaypoints.add(new Waypoint(85,224,20,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(65,162,20,Constants.WildTurtleSpeed));
        sWaypoints.add(new Waypoint(20,162,0,Constants.WildTurtleSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(122, 224), Rotation.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}
