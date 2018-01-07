package org.usfirst.frc.team4028.robot.paths;

import org.usfirst.frc.team4028.util.control.Path;
import org.usfirst.frc.team4028.util.motion.RigidTransform;

public interface PathContainer {
	Path buildPath();
	
	RigidTransform getStartPose();
	
	boolean isReversed();
}