package org.usfirst.frc.team4028.robot.auton.actions;

import java.util.ArrayList;
import java.util.List;

/* Allows for multiple actions to run in parallel */
public class ParallelAction implements Action{
	private final ArrayList<Action> _actionList;
	
	public ParallelAction(List<Action> actionList) {
		_actionList = new ArrayList<>(actionList.size());
		for (Action action : actionList) {
			_actionList.add(action);
		}
	}
	
	@Override
	public void start() {
		for (Action action : _actionList) {
			action.start();	// Start all actions
		}
	}

	@Override
	public void update() {
		for (Action action : _actionList) {
			action.update();	// Update all actions every cycle
		}
	}

	@Override
	public void done() {	
		for (Action action : _actionList) {
			action.done();	// Call when ALL actions are finished
		}
	}

	@Override
	public boolean isFinished() {	// Returns true when ALL actions are finished
		boolean isAllFinished = true;
		for (Action action : _actionList) {
			if (!action.isFinished()) {
				isAllFinished = false;	
			}
		}
		
		return isAllFinished;
	}
}