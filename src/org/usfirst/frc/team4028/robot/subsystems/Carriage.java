package org.usfirst.frc.team4028.robot.subsystems;

public class Carriage {

	// singleton pattern
	private static Carriage _instance = new Carriage();
	
	public static Carriage getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Carriage() {
	
	}
}
