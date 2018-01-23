package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.RobotState;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.util.control.Path;

import edu.wpi.first.wpilibj.Timer;

/* Runs a motion profile */
public class RunMotionProfileAction implements Action {
	private Chassis _chassis = Chassis.getInstance();
	private Path _path;
	private double _startTime;
	private boolean _isShiftingEnabled;
	
	public RunMotionProfileAction(Path p, boolean isShiftingEnabled) {
		_isShiftingEnabled = isShiftingEnabled;
		_path = p;
	}
	
	public RunMotionProfileAction(Path p) {
		this(p, false);
	}
	
	@Override
	public void start() {
		RobotState.getInstance().reset(Timer.getFPGATimestamp(), _path.getStartPose());
		_chassis.setWantDrivePath(_path, _path.isReversed(), _isShiftingEnabled);
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		if(Timer.getFPGATimestamp() - _startTime > 0.25) {
			if(_chassis.getLeftPosInRot() == 0 || _chassis.getRightPosInRot() == 0){
				_chassis.forceDoneWithPath();
				System.out.println("Attention Idiots: You Morons Forgot to Plug in The Encoder");
			}
		}
	}

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