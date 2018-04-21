package org.usfirst.frc.team4028.robot.auton.modes.side;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.*;
import org.usfirst.frc.team4028.robot.paths.Paths;
import org.usfirst.frc.team4028.robot.paths.Paths.Left;
import org.usfirst.frc.team4028.robot.paths.Paths.Right;
import org.usfirst.frc.team4028.util.control.Path;

public class ToBackCenter extends AutonBase{
	Path toBackCenter;
	
	public ToBackCenter(boolean isStartingLeft) {
		if (isStartingLeft) 
			toBackCenter = Paths.getPath(Left.TO_BACK_CENTER);
		else
			toBackCenter = Paths.getPath(Right.TO_BACK_CENTER);
	}
	
	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(toBackCenter));
		runAction(new PrintTimeFromStart(_startTime));
	}
}