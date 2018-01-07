package org.usfirst.frc.team4028.robot.auton.actions;

import java.util.ArrayList;
import java.util.List;

// Runs an list of actions in a sequence
public class SeriesAction implements Action {
	private Action _curAction;
	private final ArrayList<Action> _remainingActions;
	
	public SeriesAction(List<Action> actions) {
		_remainingActions = new ArrayList<>(actions.size());
		for (Action action : actions) {
			_remainingActions.add(action);
			_curAction = null;
		}
	}
	
	@Override
	public void start() {}

	@Override
	public void update() {
		 if (_curAction == null) {
	            if (_remainingActions.isEmpty()) {
	                return;
	            }
	            _curAction = _remainingActions.remove(0);
	            _curAction.start();
	        }
	        _curAction.update();
	        if (_curAction.isFinished()) {
	            _curAction.done();
	            _curAction = null;
	        }
	}

	@Override
	public void done() {}

	@Override
	public boolean isFinished() {
		return _remainingActions.isEmpty() && _curAction == null;	// Returns true when final sequence is complete
	}
}