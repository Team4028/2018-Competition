package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.util.control.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/* Runs a motion profile */
public class RunMotionProfileAction implements Action {
	private Chassis _chassis = Chassis.getInstance();
	private Path _path;
	private double _startTime;
	
	public RunMotionProfileAction(Path p) {
		_path = p;
	}
	
	@Override
	public void start() {
		_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
		_chassis.setWantDrivePath(_path, _path.isReversed());
		DriverStation.reportError("Running Motion Profile", false);
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		if(Timer.getFPGATimestamp()-_startTime>.25) {
			if(_chassis.getLeftPosInRot()==0 || _chassis.getRightPosInRot()==0){
				_chassis.forceDoneWithPath();
				System.out.println("Attention Idiots: You Morons Forgot to Plug in The Encoder");
			}
		}
	}	// Nothing here since trajController updates in its own thread

	@Override
	public void done() {	
		_chassis.stop();
	}
	
	
	@Override
	public boolean isFinished() {
		if ((Timer.getFPGATimestamp() - _startTime) > 1000) {
			_chassis.forceDoneWithPath();
			return true;
		} else {
			return _chassis.isDoneWithPath();
		}
	}
}