package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.*;

public class RightSwitchtoFrontofPyramidPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		        sWaypoints.add(new Waypoint(123,224,0,0));
        sWaypoints.add(new Waypoint(93,224,20,60));
        sWaypoints.add(new Waypoint(73,162,10,60));
        sWaypoints.add(new Waypoint(53,162,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(123, 224), Rotation.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}
