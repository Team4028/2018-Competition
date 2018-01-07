package org.usfirst.frc.team4028.util;

public class LatchedBoolean {
	private boolean _isPressedLastCycle = false;
	
	public boolean isJustPressed(boolean isCurrentlyPressed) {	
		boolean ret = isCurrentlyPressed && !_isPressedLastCycle;
		
		_isPressedLastCycle = isCurrentlyPressed;
		
		return ret;
	}
}