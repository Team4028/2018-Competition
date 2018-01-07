package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.paths.PathContainer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.util.control.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/* Runs a motion profile */
public class RunMotionProfileAction implements Action {
	private Chassis _chassis = Chassis.getInstance();
	private PathContainer _pathContainer;
	private Path _path;
	private double _startTime;
	
	public RunMotionProfileAction(PathContainer p) {
		_pathContainer = p;
		_path = _pathContainer.buildPath();
	}
	
	@Override
	public void start() {
		_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
		_chassis.setWantDrivePath(_path, _pathContainer.isReversed());
		DriverStation.reportError("Running Motion Profile", false);
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {}	// Nothing here since trajController updates in its own thread

	@Override
	public void done() {	
		_chassis.stop();
	}

	@Override
	public boolean isFinished() {
		if ((Timer.getFPGATimestamp() - _startTime) > 2.8) {
			_chassis.forceDoneWithPath();
			return true;
		} else {
			return _chassis.isDoneWithPath();
		}
	}
}