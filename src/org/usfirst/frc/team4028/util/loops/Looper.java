package org.usfirst.frc.team4028.util.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

// Creates a thread which runs all subsystems. Controls subsystem implementation of loop interface
public class Looper {
	public final double PERIOD = 0.01;	// Running at 100hz
	
	private boolean _running;
	
	private final Notifier _notifier;
	private final List<Loop> _loops;
	private final Object _taskRunningLock = new Object();
	private double _timestamp = 0.0;
	private double _dt = 0.0;
	
	private final Runnable _runnable = new Runnable() {
		@Override
        public void run() {
            synchronized (_taskRunningLock) {
                if (_running) {
                    double now = Timer.getFPGATimestamp();
                    for (Loop loop : _loops) {
                        loop.onLoop(now);
                    }
                    
                    _dt = now - _timestamp;
                    _timestamp = now;
                }
            }
        }
	};
	
	public Looper() {
		_notifier = new Notifier(_runnable);
		_running = false;
		_loops = new ArrayList<>();
	}
	
	// Adds subsystem to the looper
	public synchronized void register(Loop loop) {
		synchronized (_taskRunningLock) {
			_loops.add(loop);
		}
	}
	
	// Calls the onStart() method in every subsystem on thread startup
	public synchronized void start() {
        if (!_running) {
            System.out.println("Starting loops");
            synchronized (_taskRunningLock) {
                _timestamp = Timer.getFPGATimestamp();
                for (Loop loop : _loops) {
                    loop.onStart(_timestamp);
                }
                _running = true;
            }
            _notifier.startPeriodic(PERIOD);
        }
    }
	
	// Calls the onStop() method in every subsystem on thread termination
	public synchronized void stop() {
		if (_running) {
			System.out.println("Stopping loops");
			_notifier.stop();
			synchronized (_taskRunningLock) {
				_running = false;
				_timestamp = Timer.getFPGATimestamp();
				for (Loop loop : _loops) {
					System.out.println("Stopping" + loop);
					loop.onStop(_timestamp);
				}
			}
		}
	}
}