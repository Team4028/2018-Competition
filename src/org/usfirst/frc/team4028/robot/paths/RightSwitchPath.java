package org.usfirst.frc.team4028.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team4028.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.*;

public class RightSwitchPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		        sWaypoints.add(new Waypoint(18,162,0,0));
        sWaypoints.add(new Waypoint(50,162,30,60));
        sWaypoints.add(new Waypoint(90,224,30,60));
        sWaypoints.add(new Waypoint(123,224,0,60));
       

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform getStartPose() {
        return new RigidTransform(new Translation(18, 162), Rotation.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}
