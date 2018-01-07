package org.usfirst.frc.team4028.robot.auton;

import edu.wpi.first.wpilibj.DriverStation;

/* Selects, runs and stops auton */
public class AutonExecuter {
	private AutonBase _autoMode;
	private Thread _autoThread = null;
	
	public void setAutoMode(AutonBase autoMode) {
		_autoMode = autoMode;
	}
	
	public void start() {
		// Creates a new threat to run auton in
		if (_autoThread == null) {
			_autoThread = new Thread(new Runnable() {
				@Override
				public void run() {
					if (_autoMode != null) {
						_autoMode.run();
					}
				}
			});
			_autoThread.start();
		}
		DriverStation.reportError("Starting Auto", false);
	}
	
	public void stop() {
		if (_autoMode != null) {
			_autoMode.stop();
		}
		_autoThread = null;
	}
}