package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/*
========================================================================
=======================	 HOW TO USE THIS CLASS	========================
========================================================================
	1)Select which button/joystick you want to use
		a) You will know if it is available by looking at Table 1.1 and seeing if there is a listed function next to the button/joystick
	2) Claim the button/joystick by updating the proposed functionality in Table 1.1
		EXAMPLE:     DRIVER_BACK_BUTTON		Cosmic Ray Gun Fire
	3) Uncomment & Change the name of the method used to access the properties of said button/joystick
		EXAMPLE:	 getIsDriver_Back_BtnJustPressed()     -changes to-		getIsDriver_CosmicRayGunFire_BtnJustPressed()

		/* Table 1.1:
		==== DRIVER ==========================================================
		--- Driver Joysticks --------
		DRIVER_LEFT_X_AXIS							xxxxxxxxxxxx
		DRIVER_LEFT_Y_AXIS							Throttle Cmd
		DRIVER_LEFT_TRIGGER			
		DRIVER_RIGHT_TRIGGER		
		DRIVER_RIGHT_X_AXIS							Turn Cmd
		DRIVER_RIGHT_Y_AXIS							xxxxxxxxxxxx
		
		--- Driver Buttons --------
		DRIVER_GREEN_BUTTON_A						Store Arms
		DRIVER_RED_BUTTON_B			 				Stagger Infeed
		DRIVER_BLUE_BUTTON_X						Wide Infeed
		DRIVER_YELLOW_BUTTON_Y						Squeeze Infeed
		DRIVER_LEFT_BUMPER							Infeed Cube
		DRIVER_RIGHT_BUMPER							Auto Acquire
		
							ReZero Infeed
		DRIVER_START_BUTTON							Shift Gear	
		DRIVER_LEFT_THUMBSTICK
		DRIVER_RIGHT_THUMBSTICK
								
		==== OPERATOR ==========================================================
			
		--- Operator Joysticks --------
		OPERATOR_LEFT_X_AXIS						*************
		OPERATOR_LEFT_Y_AXIS						Elevator
		OPERATOR_LEFT_TRIGGER			
		OPERATOR_RIGHT_TRIGGER			
		OPERATOR_RIGHT_X_AXIS						*************
		OPERATOR_RIGHT_Y_AXIS						Carriage
		
		--- Operator Buttons --------
		OPERATOR_GREEN_BUTTON_A						Elevator Floor
		OPERATOR_RED_BUTTON_B						Elevator Prymd
		OPERATOR_BLUE_BUTTON_X						Elevator Switch
		OPERATOR_YELLOW_BUTTON_Y					Elevator Scale
		OPERATOR_LEFT_BUMPER			
		OPERATOR_RIGHT_BUMPER						Elevator Home
		OPERATOR_BACK_BUTTON			
		OPERATOR_START_BUTTON						Switch Camera
		OPERATOR_LEFT_THUMBSTICK		
		OPERATOR_RIGHT_THUMBSTICK	
													
		==== ENGINEERING  ==========================================================
					
		--- Engineer Joysticks --------
		ENGINEER_LEFT_X_AXIS			
		ENGINEER_LEFT_Y_AXIS			
		ENGINEER_LEFT_TRIGGER			
		ENGINEER_RIGHT_TRIGGER			
		ENGINEER_RIGHT_X_AXIS
		ENGINEER_RIGHT_Y_AXIS			
		
		--- Engineer Buttons --------
		ENGINEER_GREEN_BUTTON_A			
		ENGINEER_RED_BUTTON_B			
		ENGINEER_BLUE_BUTTON_X			
		ENGINEER_YELLOW_BUTTON_Y		
		ENGINEER_LEFT_BUMPER			
		ENGINEER_RIGHT_BUMPER			
		ENGINEER_BACK_BUTTON			
		ENGINEER_START_BUTTON			
		ENGINEER_LEFT_THUMBSTICK		
		ENGINEER_RIGHT_THUMBSTICK						
		*/

public class DriverOperatorStation {
	// class level private variables
	private XboxController _driverGamepad;
	private XboxController _operatorGamepad;
	private XboxController _engineeringGamepadA;
	private XboxController _engineeringGamepadB;
		
	private DriverStation _driverStation;
	
	private static final double JOYSTICK_DEADBAND = 0.05;
	
	// singleton pattern
	private static DriverOperatorStation _instance = new DriverOperatorStation();

	public static DriverOperatorStation getInstance() {
		return _instance;
	}
	
	private Boolean _isDriverGamepad_PluggedIn = false;
	private Boolean _isOperatorGamepad_PluggedIn = false;
	private Boolean _isEngineeringGamepadA_PluggedIn = false;
	private Boolean _isEngineeringGamepadB_PluggedIn = false;
	
	// private constructor for singleton pattern
	private DriverOperatorStation() {
		_driverStation = DriverStation.getInstance();
		
		_isDriverGamepad_PluggedIn = _driverStation.getJoystickIsXbox(Constants.DRIVER_GAMEPAD_USB_PORT);
		_driverGamepad = new XboxController(Constants.DRIVER_GAMEPAD_USB_PORT);					// std Logitech F310 Gamepad  
		
		_isOperatorGamepad_PluggedIn = _driverStation.getJoystickIsXbox(Constants.OPERATOR_GAMEPAD_USB_PORT);
		_operatorGamepad = new XboxController(Constants.OPERATOR_GAMEPAD_USB_PORT);				// std Logitech F310 Gamepad  
		
		_isEngineeringGamepadA_PluggedIn = _driverStation.getJoystickIsXbox(Constants.ENGINEERING_GAMEPAD_USB_PORT);
		_engineeringGamepadA = new XboxController(Constants.ENGINEERING_GAMEPAD_USB_PORT);	// std Logitech F310 Gamepad  
		
		_isEngineeringGamepadB_PluggedIn = _driverStation.getJoystickIsXbox(Constants.ENGINEERING_GAMEPAD_B_USB_PORT);
		_engineeringGamepadB = new XboxController(Constants.ENGINEERING_GAMEPAD_B_USB_PORT);
		
		if(_isEngineeringGamepadA_PluggedIn) {
			System.out.println("Enginnering GamePad A Plugged In");
		}

		if(_isEngineeringGamepadB_PluggedIn) {
			System.out.println("Enginnering GamePad B Plugged In");
		}		
	}
	
	public void clearGamepadsCachedBtnPresses() {
		// Note: 2018 WPI code seems to have an issue that caches joystick button 
		//			pressed while the robot is on but disabled
		// 			so we eat cached button presses by reading them
		if(_isDriverGamepad_PluggedIn) {
			// eat cached button presses
			resetGamepad(_driverGamepad);
		}
		
		if(_isOperatorGamepad_PluggedIn) {
			// eat cached button presses
			resetGamepad(_operatorGamepad);
		}
		
		if(_isEngineeringGamepadA_PluggedIn) {
			// eat cached button presses
			resetGamepad(_engineeringGamepadA);
		}
		
		if(_isEngineeringGamepadB_PluggedIn) {
			// eat cached button presses
			resetGamepad(_engineeringGamepadB);
		}
	}
		
	// eat cached button presses
	private void resetGamepad(XboxController gamepad) {
		gamepad.getBackButtonPressed();
		gamepad.getStartButtonPressed();
		gamepad.getYButtonPressed();
		gamepad.getBButtonPressed();
		gamepad.getAButtonPressed();
		gamepad.getXButtonPressed();
		gamepad.getBumperPressed(Hand.kLeft);
		gamepad.getBumperPressed(Hand.kRight);
		gamepad.getStickButtonPressed(Hand.kLeft);
		gamepad.getStickButtonPressed(Hand.kRight);
	}
	
	// =========================================================================================================
	// DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER
	// =========================================================================================================
	
	// ===================================
	// === Driver Just Pressed buttons ===
	// ===================================

//		public boolean getIsDriver_Back_BtnJustPressed() {
//			return _driverGamepad.getBackButtonPressed();
//		}

	public boolean getIsDriver_ShiftGear_BtnJustPressed() {
		return _driverGamepad.getStartButtonPressed();
	}
	
	public boolean getIsDriver_RezeroInfeed_BtnJustPressed() {
		return _driverGamepad.getYButtonPressed();
	}
	 
	public boolean getIsDriver_WideInfeed_BtnJustPressed() {
		return _driverGamepad.getBButtonPressed();
	}

	public boolean getIsDriver_SqueezeInfeed_BtnJustPressed() {
		return _driverGamepad.getAButtonPressed();
	}	

	public boolean getIsDriver_StoreInfeed_BtnJustPressed() {
		return _driverGamepad.getXButtonPressed();
	}
	
//		public boolean getIsDriver_LeftBumper_BtnJustPressed() {
//			return _driverGamepad.getBumperPressed(Hand.kLeft);
//		}
	
//		public boolean getIsDriver_AutoAcquire_BtnJustPressed() {
//			return _driverGamepad.getBumperPressed(Hand.kRight);
//		}
	
//		public boolean getIsDriver_LeftStick_BtnJustPressed() {
//			return _driverGamepad.getStickButtonPressed(Hand.kLeft);
//		}
	
//		public boolean getIsDriver_RightStick_BtnJustPressed() {
//			return _driverGamepad.getStickButtonPressed(Hand.kRight);
//		}
	
	// ===================================
	// === Driver Just Released buttons ==
	// ===================================

//		public boolean getIsDriver_Back_BtnJustReleased() {
//			return _driverGamepad.getBackButtonReleased();
//		}

//		public boolean getIsDriver_Start_BtnJustReleased() {
//			return _driverGamepad.getStartButtonReleased();
//		}
	
//		public boolean getIsDriver_YellowY_BtnJustReleased() {
//			return _driverGamepad.getYButtonReleased();
//		}
	 
//		public boolean getIsDriver_RedB_BtnJustReleased() {
//			return _driverGamepad.getBButtonReleased();
//		}
	
//		public boolean getIsDriver_GreenA_BtnJustReleased() {
//			return _driverGamepad.getAButtonReleased();
//		}	

//		public boolean getIsDriver_BlueX_BtnJustReleased() {
//			return _driverGamepad.getXButtonReleased();
//		}
	
//		public boolean getIsDriver_LeftBumper_BtnJustReleased() {
//			return _driverGamepad.getBumperReleased(Hand.kLeft);
//		}
	
//		public boolean getIsDriver_RightBumper_BtnJustReleased() {
//			return _driverGamepad.getBumperReleased(Hand.kRight);
//		}
	
//		public boolean getIsDriver_LeftStick_BtnJustReleased() {
//			return _driverGamepad.getStickButtonReleased(Hand.kLeft);
//		}
	
//		public boolean getIsDriver_RightStick_BtnJustReleased() {
//			return _driverGamepad.getStickButtonReleased(Hand.kRight);
//		}
	
	// ===================================
	// === Driver Is Pressed buttons =====
	// ===================================

//		public boolean getIsDriver_Back_BtnPressed() {
//			return _driverGamepad.getBackButton();
//		}
	
//		public boolean getIsDriver_Start_BtnPressed() {
//			return _driverGamepad.getStartButton();
//		}
	
//		public boolean getIsDriver_YellowY_BtnPressed() {
//			return _driverGamepad.getYButton();
//		}
	
//		public boolean getIsDriver_RedB_BtnPressed() {
//			return _driverGamepad.getBButton();
//		}
	
//		public boolean getIsDriver_GreenA_BtnPressed() {
//			return _driverGamepad.getAButton();
//		}	
	
//		public boolean getIsDriver_BlueX_BtnPressed() {
//			return _driverGamepad.getXButton();
//		} 
	
		public boolean getIsDriver_SpinCubeCounterClockwise_BtnPressed() {
			return _driverGamepad.getBumper(Hand.kLeft);
		}
	
		public boolean getIsDriver_SpinCubeClockwise_BtnPressed() {
			return _driverGamepad.getBumper(Hand.kRight);
		} 
			
//		public boolean getIsDriver_LeftStick_BtnPressed() {
//			return _driverGamepad.getStickButton(Hand.kLeft);
//		}
	
//		public boolean getIsDriver_RightStick_BtnPressed() {
//			return _driverGamepad.getStickButton(Hand.kRight);
//		}

	// ===================================
	// ======== Driver Joysticks =========
	// ===================================

	public double getDriver_Throttle_JoystickCmd() {
		if(Math.abs(_driverGamepad.getY(Hand.kLeft)) >= JOYSTICK_DEADBAND){
			// flip the sign, pushing the joystick up is a # < 0
			return _driverGamepad.getY(Hand.kLeft) * -1.0;
		} 
		else {
			return 0.0;
		}
	}
	
//		public double getDriver_LeftX_JoystickCmd() {
//			return _driverGamepad.getX(Hand.kLeft);
//		}
	
//		public double getDriver_RightY_JoystickCmd() {
//			return _driverGamepad.getY(Hand.kRight);
//		}
	
	public double getDriver_Turn_JoystickCmd() {
		if(Math.abs(_driverGamepad.getX(Hand.kRight)) >= JOYSTICK_DEADBAND){
			// flip the sign, pushing the joystick up is a # < 0
			return _driverGamepad.getX(Hand.kRight) * -1.0;
		} 
		else {
			return 0.0;
		}
	}

	public double getDriver_InfeedCube_JoystickCmd() {
		return  _driverGamepad.getTriggerAxis(Hand.kLeft);
	}
	
	public double getDriver_EjectCube_JoystickCmd() {
		return _driverGamepad.getTriggerAxis(Hand.kRight);
	}
	
	// =========================================================================================================
	// OPERATOR		OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	
	// =========================================================================================================
	
	// ===================================
	// == Operator Just Pressed buttons ==
	// ===================================
	public boolean getIsOperator_ElevatorReHome_BtnJustPressed() {
		return _operatorGamepad.getBackButtonPressed();
	}

	public boolean getIsOperator_SwitchCamera_BtnJustPressed() {
		return _operatorGamepad.getStartButtonPressed();
	}
	
	public boolean getIsOperator_ElevatorScaleHgt_BtnJustPressed() {
		return _operatorGamepad.getYButtonPressed();
	}
	 
	public boolean getIsOperator_ElevatorPyrmdLvl1Hgt_BtnJustPressed() {
		return _operatorGamepad.getBButtonPressed();
	}
	
	public boolean getIsOperator_ElevatorInfeedHgt_BtnJustPressed() {
		return _operatorGamepad.getAButtonPressed();
	}	

	public boolean getIsOperator_ElevatorSwitchHgt_BtnJustPressed() {
		return _operatorGamepad.getXButtonPressed();
	}
	
//		public boolean getIsOperator_LeftBumper_BtnJustPressed() {
//			return _operatorGamepad.getBumperPressed(Hand.kLeft);
//		}

//		public boolean getIsOperator_RightBumper_BtnJustPressed() {
//			return _operatorGamepad.getBumperPressed(Hand.kRight);
//		}
	
//		public boolean getIsOperator_LeftStick_BtnJustPressed() {
//			return _operatorGamepad.getStickButtonPressed(Hand.kLeft);
//		}
	
//		public boolean getIsOperator_RightStick_BtnJustPressed() {
//			return _operatorGamepad.getStickButtonPressed(Hand.kRight);
//		}
	
	// ====================================
	// == Operator Just Released buttons ==
	// ====================================		
//		public boolean getIsOperator_Back_BtnJustReleased() {
//			return _operatorGamepad.getBackButtonReleased();
//		}

//		public boolean getIsOperator_Start_BtnJustReleased() {
//			return _operatorGamepad.getStartButtonReleased();
//		}
	
//		public boolean getIsOperator_YellowY_BtnJustReleased() {
//			return _operatorGamepad.getYButtonReleased();
//		}
	 
//		public boolean getIsOperator_RedB_BtnJustReleased() {
//			return _operatorGamepad.getBButtonReleased();
//		}
	
//		public boolean getIsOperator_GreenA_BtnJustReleased() {
//			return _operatorGamepad.getAButtonReleased();
//		}	

//		public boolean getIsOperator_BlueX_BtnJustReleased() {
//			return _operatorGamepad.getXButtonReleased();
//		}
	
//		public boolean getIsOperator_LeftBumper_BtnJustReleased() {
//			return _operatorGamepad.getBumperReleased(Hand.kLeft);
//		}
	
//		public boolean getIsOperator_RightBumper_BtnJustReleased() {
//			return _operatorGamepad.getBumperReleased(Hand.kRight);
//		}
	
//		public boolean getIsOperator_LeftStick_BtnJustReleased() {
//			return _operatorGamepad.getStickButtonReleased(Hand.kLeft);
//		}
	
//		public boolean getIsOperator_RightStick_BtnJustReleased() {
//			return _operatorGamepad.getStickButtonReleased(Hand.kRight);
//		}
	
	// ===================================
	// === Operator Is Pressed buttons ===
	// ===================================

//		public boolean getIsOperator_Back_BtnPressed() {
//			return _operatorGamepad.getBackButton();
//		}
	
//		public boolean getIsOperator_Start_BtnPressed() {
//			return _operatorGamepad.getStartButton();
//		}
	
//		public boolean getIsOperator_YellowY_BtnPressed() {
//			return _operatorGamepad.getYButton();
//		}
	
//		public boolean getIsOperator_RedB_BtnPressed() {
//			return _operatorGamepad.getBButton();
//		}
	
//		public boolean getIsOperator_GreenA_BtnPressed() {
//			return _operatorGamepad.getAButton();
//		}	
	
//		public boolean getIsOperator_BlueX_BtnPressed() {
//			return _operatorGamepad.getXButton();
//		}
	
//	public boolean getIsOperator_ElevatorSafety_BtnPressed() {
//		return _operatorGamepad.getBumper(Hand.kLeft);
//	}
	
//		public boolean getIsOperator_RightBumper_BtnPressed() {
//			return _operatorGamepad.getBumper(Hand.kRight);
//		}
	
//		public boolean getIsOperator_LeftStick_BtnPressed() {
//			return _operatorGamepad.getStickButton(Hand.kLeft);
//		}
	
//		public boolean getIsOperator_RightStick_BtnPressed() {
//			return _operatorGamepad.getStickButton(Hand.kRight);
//		}

	// ===================================
	// ======== Operator Joysticks =========
	// ===================================

	public double getOperator_Elevator_JoystickCmd() {
		if(Math.abs(_operatorGamepad.getY(Hand.kLeft)) >= JOYSTICK_DEADBAND){
			// flip the sign, pushing the joystick up is a # < 0
			return _operatorGamepad.getY(Hand.kLeft) * -1.0;
		} 
		else {
			return 0.0;
		}
	}
	
//		public double getOperator_LeftX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kLeft);
//		}
	
	public double getOperator_Climb_JoystickCmd() {
		if(Math.abs(_operatorGamepad.getY(Hand.kRight)) >= JOYSTICK_DEADBAND){
			// flip the sign, pushing the joystick up is a # < 0
			return _operatorGamepad.getY(Hand.kRight) * -1.0;
		} 
		else {
			return 0.0;
		}
	}
	
//		public double getOperator_RightX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kRight);
//		}

//		public double getOperator_LeftTrigger_JoystickCmd() {
//			return  _operatorGamepad.getTriggerAxis(Hand.kLeft);
//		}
	
		public double getOperator_EjectCube_JoystickCmd() {
			return _operatorGamepad.getTriggerAxis(Hand.kRight);
		}
	
	// =========================================================================================================
	// ENGINEER		ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER
	// =========================================================================================================
	public boolean IsEngineeringGamepadAAvailable()
	{
		return _isEngineeringGamepadA_PluggedIn;
	}
	
	// ======================================
	// == Engineering Just Pressed buttons ==
	// ======================================

	public boolean getIsEngineering_ReZeroInfeed_BtnJustPressed() {
		if(_isEngineeringGamepadA_PluggedIn) {
			return _engineeringGamepadA.getBackButtonPressed();
		}
		else {
			return false; 
		}
	}

//		public boolean getIsEngineering_Start_BtnJustPressed() {
//			return _engineeringGamepad.getStartButtonPressed();
//		}
	
	public boolean getIsEngineering_ElevatorScaleHgt_BtnJustPressed() {
		return _engineeringGamepadA.getYButtonPressed();
	}
	 
	public boolean getIsEngineering_ElevatorPyramidHgt_BtnJustPressed() {
		return _engineeringGamepadA.getBButtonPressed();
	}
	
	public boolean getIsEngineering_ElevatorCubeOnFloorHgt_BtnJustPressed() {
		return _engineeringGamepadA.getAButtonPressed();
	}	

	public boolean getIsEngineering_ElevatorSwitchHgt_BtnJustPressed() {
		return _engineeringGamepadA.getXButtonPressed();
	}
	
//		public boolean getIsEngineering_LeftBumper_BtnJustPressed() {
//			return _engineeringGamepad.getBumperPressed(Hand.kLeft);
//		}
	
//		public boolean getIsEngineering_RightBumper_BtnJustPressed() {
//			return _engineeringGamepad.getBumperPressed(Hand.kRight);
//		}
	
//		public boolean getIsEngineering_LeftStick_BtnJustPressed() {
//			return _engineeringGamepad.getStickButtonPressed(Hand.kLeft);
//		}
	
//		public boolean getIsEngineering_RightStick_BtnJustPressed() {
//			return _engineeringGamepad.getStickButtonPressed(Hand.kRight);
//		}
	
	// =======================================
	// == Engineering Just Released buttons ==
	// =======================================

//		public boolean getIsEngineering_Back_BtnJustReleased() {
//			return _engineeringGamepad.getBackButtonReleased();
//		}

//		public boolean getIsEngineering_Start_BtnJustReleased() {
//			return _engineeringGamepad.getStartButtonReleased();
//		}
	
//		public boolean getIsEngineering_YellowY_BtnJustReleased() {
//			return _engineeringGamepad.getYButtonReleased();
//		}
	 
//		public boolean getIsEngineering_RedB_BtnJustReleased() {
//			return _engineeringGamepad.getBButtonReleased();
//		}
	
//		public boolean getIsEngineering_GreenA_BtnJustReleased() {
//			return _engineeringGamepad.getAButtonReleased();
//		}	

//		public boolean getIsEngineering_BlueX_BtnJustReleased() {
//			return _engineeringGamepad.getXButtonReleased();
//		}
	
//		public boolean getIsEngineering_LeftBumper_BtnJustReleased() {
//			return _engineeringGamepad.getBumperReleased(Hand.kLeft);
//		}
	
//		public boolean getIsEngineering_RightBumper_BtnJustReleased() {
//			return _engineeringGamepad.getBumperReleased(Hand.kRight);
//		}
	
//		public boolean getIsEngineering_LeftStick_BtnJustReleased() {
//			return _engineeringGamepad.getStickButtonReleased(Hand.kLeft);
//		}
	
//		public boolean getIsEngineering_RightStick_BtnJustReleased() {
//			return _engineeringGamepad.getStickButtonReleased(Hand.kRight);
//		}
	
	// ======================================
	// === Engineering Is Pressed buttons ===
	// ======================================

//		public boolean getIsEngineering_Back_BtnPressed() {
//			return _engineeringGamepad.getBackButton();
//		}
	
//		public boolean getIsEngineering_Start_BtnPressed() {
//			return _engineeringGamepad.getStartButton();
//		}
	
//		public boolean getIsEngineering_YellowY_BtnPressed() {
//			return _engineeringGamepad.getYButton();
//		}
	
//		public boolean getIsEngineering_RedB_BtnPressed() {
//			return _engineeringGamepad.getBButton();
//		}
	
//		public boolean getIsEngineering_GreenA_BtnPressed() {
//			return _engineeringGamepad.getAButton();
//		}	
	
//		public boolean getIsEngineering_BlueX_BtnPressed() {
//			return _engineeringGamepad.getXButton();
//		}
	
	public boolean getIsEngineering_SpinCubeManuver_BtnPressed() {
		return _engineeringGamepadA.getBumper(Hand.kLeft);
	}
	
//		public boolean getIsEngineering_RightBumper_BtnPressed() {
//			return _engineeringGamepad.getBumper(Hand.kRight);
//		}
	
//		public boolean getIsEngineering_LeftStick_BtnPressed() {
//			return _engineeringGamepad.getStickButton(Hand.kLeft);
//		}
	
//		public boolean getIsEngineering_RightStick_BtnPressed() {
//			return _engineeringGamepad.getStickButton(Hand.kRight);
//		}

	// ===================================
	// ====== Engineering Joysticks ======
	// ===================================

	public double getEngineering_Elevator_JoystickCmd() {
		if(_isEngineeringGamepadA_PluggedIn) {
			if(Math.abs(_engineeringGamepadA.getY(Hand.kLeft)) >= JOYSTICK_DEADBAND){
				// flip the sign, pushing the joystick up is a # < 0
				return _engineeringGamepadA.getY(Hand.kLeft) * -1.0;
			} 
			else {
				return 0.0;
			}
		} 
		else {
			return 0;
		}
	}

//		public double getOperator_LeftX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kLeft);
//		}

//		public double getEngineering_InfeedPositionY_JoystickCmd() {
//			return _operatorGamepad.getY(Hand.kRight);
//		}
			
//		public double getOperator_RightX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kRight);
//		}
	
	public double getEngineering_InfeedPositionX_JoystickCmd() {
		if(_isEngineeringGamepadA_PluggedIn) {
			return _engineeringGamepadA.getX(Hand.kRight);
		} 
		else {
			return 0.0;
		}
	}

	public double getEngineering_InfeedCube_JoystickCmd() {
		if(_isEngineeringGamepadA_PluggedIn) {
			return _engineeringGamepadA.getTriggerAxis(Hand.kLeft);
		} 
		else {
			return 0.0;
		}
	}
	
	public double getEngineering_EjectCube_JoystickCmd() {
		if(_isEngineeringGamepadA_PluggedIn) {
			return _engineeringGamepadA.getTriggerAxis(Hand.kRight);
		} 
		else {
			return 0.0;
		}
	}
	
	public boolean getIsEngineering_SqueezeInfeed_BtnPressed() {
		if(_isEngineeringGamepadA_PluggedIn) {
			if(_engineeringGamepadA.getPOV(0) == 0) {
				return true;
			} 
			else {
				return false;
			}
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngineering_StaggerInfeed_BtnPressed() {
		if(_isEngineeringGamepadA_PluggedIn) {
			if(_engineeringGamepadA.getPOV(0) == 90) {
				return true;
			} 
			else {
				return false;
			}
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngineering_StoreInfeed_BtnPressed() {
		if(_isEngineeringGamepadA_PluggedIn) {
			if(_engineeringGamepadA.getPOV(0) == 180) {
				return true;
			} 
			else {
				return false;
			}
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngineering_WideInfeed_BtnPressed() {
		if(_isEngineeringGamepadA_PluggedIn) {
			if(_engineeringGamepadA.getPOV(0) == 270) {
				return true;
			} else {
				return false;
			}
		} 
		else {
			return false;
		}
	}
	
	// =========================================================================================================
	// Engineering B	Engineering B	Engineering B	Engineering B	Engineering B	Engineering B
	// =========================================================================================================
	public boolean IsEngineeringGamepadBAvailable() {
		return _isEngineeringGamepadB_PluggedIn;
	}
	
	// ===================================
	// === Engineering B Just Pressed buttons ===
	// ===================================
	public boolean getIsEngrB_InfeedVBusBumpDown_BtnJustPressed() {		
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getBackButtonPressed();
		} 
		else {
			return false;
		}
	}

	public boolean getIsEngrB_InfeedVBusBumpUp_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getStartButtonPressed();
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngrB_StoreInfeed_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getYButtonPressed();
		} 
		else {
			return false;
		}
	}
	 
	public boolean getIsEngrB_WideInfeed_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getBButtonPressed();
		} 
		else {
			return false;
		}
	}

	public boolean getIsEngrB_SqueezeInfeed_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getAButtonPressed();
		} 
		else {
			return false;
		}
	}	

	public boolean getIsEngrB_RezeroInfeed_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getXButtonPressed();
		} else {
			return false;
		}
	}
	
	public boolean getIsEngrB_Carriage_FeedIn_VBusBumpDown_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getBumperPressed(Hand.kLeft);
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngrB_Carriage_FeedIn_VBusBumpUp_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getBumperPressed(Hand.kRight);
		} 
		else {
			return false;
		}
	}
		
	// implement just pressed for POV
	boolean wasEngrBPOV180PresssedLastScan = false;
	
	public boolean getIsEngrB_Carriage_FeedOut_VBusBumpDown_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			if(_engineeringGamepadB.getPOV(0) == 180) {
				boolean isJustPressed = true && !wasEngrBPOV180PresssedLastScan;
				wasEngrBPOV180PresssedLastScan = true;
				return isJustPressed;
			} 
			else {
				wasEngrBPOV180PresssedLastScan = false;
				return false;
			}
		} 
		else {
			return false;
		}
	}
	
	boolean wasEngrBPOV0PresssedLastScan = false;
	
	public boolean getIsEngrB_Carriage_FeedOut_VBusBumpUp_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			if(_engineeringGamepadB.getPOV(0) == 0) {
				boolean isJustPressed = true && !wasEngrBPOV0PresssedLastScan;
				wasEngrBPOV0PresssedLastScan = true;
				return isJustPressed;
			} 
			else {
				wasEngrBPOV0PresssedLastScan = false;
				return false;
			}
		} 
		else {
			return false;
		}
	}
	 
	
	public boolean getIsEngrB_SqueezeBumpNarrower_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getStickButtonPressed(Hand.kLeft);
		} 
		else {
			return false;
		}
	}
	
	public boolean getIsEngrB_SqueezeBumpWider_BtnJustPressed() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return _engineeringGamepadB.getStickButtonPressed(Hand.kRight);
		} 
		else {
			return false;
		}
	}
	
	
	// ===================================
	// === Engineering B Just Released buttons ==
	// ===================================

//		public boolean getIsEngrB_Back_BtnJustReleased() {
//			return _engineeringGamepadB.getBackButtonReleased();
//		}

//		public boolean getIsEngrB_Start_BtnJustReleased() {
//			return _engineeringGamepadB.getStartButtonReleased();
//		}
	
//		public boolean getIsEngrB_YellowY_BtnJustReleased() {
//			return _engineeringGamepadB.getYButtonReleased();
//		}
	 
//		public boolean getIsEngrB_RedB_BtnJustReleased() {
//			return _engineeringGamepadB.getBButtonReleased();
//		}
	
//		public boolean getIsEngrB_GreenA_BtnJustReleased() {
//			return _engineeringGamepadB.getAButtonReleased();
//		}	

//		public boolean getIsEngrB_BlueX_BtnJustReleased() {
//			return _engineeringGamepadB.getXButtonReleased();
//		}
	
//		public boolean getIsEngrB_LeftBumper_BtnJustReleased() {
//			return _engineeringGamepadB.getBumperReleased(Hand.kLeft);
//		}
	
//		public boolean getIsEngrB_RightBumper_BtnJustReleased() {
//			return _engineeringGamepadB.getBumperReleased(Hand.kRight);
//		}
	
//		public boolean getIsEngrB_LeftStick_BtnJustReleased() {
//			return _engineeringGamepadB.getStickButtonReleased(Hand.kLeft);
//		}
	
//		public boolean getIsEngrB_RightStick_BtnJustReleased() {
//			return _engineeringGamepadB.getStickButtonReleased(Hand.kRight);
//		}
	
	// ===================================
	// === Engineering B Is Pressed buttons =====
	// ===================================

//		public boolean getIsEngrB_Back_BtnPressed() {
//			return _engineeringGamepadB.getBackButton();
//		}
	
//		public boolean getIsEngrB_Start_BtnPressed() {
//			return _engineeringGamepadB.getStartButton();
//		}
	
//		public boolean getIsEngrB_YellowY_BtnPressed() {
//			return _engineeringGamepadB.getYButton();
//		}
	
//		public boolean getIsEngrB_RedB_BtnPressed() {
//			return _engineeringGamepadB.getBButton();
//		}
	
//		public boolean getIsEngrB_GreenA_BtnPressed() {
//			return _drive_engineeringGamepadBrGamepad.getAButton();
//		}	
	
//		public boolean getIsEngrB_BlueX_BtnPressed() {
//			return _engineeringGamepadB.getXButton();
//		} 
	
//		public boolean getIsEngrB_SpinCubeManuver_BtnPressed() {
//			return _engineeringGamepadB.getBumper(Hand.kLeft);
//		}
	
//		public boolean getIsEngrB_EjectCube_BtnPressed() {
//			return _engineeringGamepadB.getBumper(Hand.kRight);
//		} 
			
//		public boolean getIsEngrB_LeftStick_BtnPressed() {
//			return _engineeringGamepadB.getStickButton(Hand.kLeft);
//		}
	
//		public boolean getIsEngrB_RightStick_BtnPressed() {
//			return _engineeringGamepadB.getStickButton(Hand.kRight);
//		}

	// ===================================
	// ======== Engineering B Joysticks =========
	// ===================================

	public double getEngrB_InfeedAndCarriage_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn) {
			if(Math.abs(_engineeringGamepadB.getY(Hand.kRight)) >= JOYSTICK_DEADBAND) {
				if(_engineeringGamepadB.getY(Hand.kRight) > 0){
					return -1.0;	// down
				} 
				else if(_engineeringGamepadB.getY(Hand.kRight) < 0){
					return 1.0;	// up
				}
			}
			else {
				return 0.0;		
			}
		}

		return 0.0;
	}
	
	// joystick acting as button
	/*public Boolean getEngrB_InfeedIn_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
		
			// flip the sign, pushing the joystick up is a # < 0
			if(Math.abs(_engineeringGamepadB.getY(Hand.kRight)) >= JOYSTICK_DEADBAND
					&& _engineeringGamepadB.getY(Hand.kRight) > 0){
				return true;
			} else {
				return false;
			}
		}
		else {
			return false;
		}
	}*/
	
	// joystick acting as button
	/*public Boolean getEngrB_InfeedOut_JoystickCmd() {
		// flip the sign, pushing the joystick up is a # < 0
		if(Math.abs(_engineeringGamepadB.getY(Hand.kRight)) >= JOYSTICK_DEADBAND
			&& _engineeringGamepadB.getY(Hand.kRight) < 0){
			return true;
		} else {
			return false;
		}
	}*/
	
	// joystick acting as button
	/*public Boolean getEngrB_InfeedStop_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			if(Math.abs(_engineeringGamepadB.getY(Hand.kRight)) < JOYSTICK_DEADBAND) {
				return true;
			}
			else {
				return false;					
			}
		}
		else {
			return false;
		}
	}*/
	
	/*public double getEngrB_LeftX_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			return _engineeringGamepadB.getX(Hand.kLeft);
		}
		else {
			return 0;
		}
	}*/
	
	/*public double getEngrB_RightY_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			return _engineeringGamepadB.getY(Hand.kRight);
		}
		else {
			return 0;
		}
	}*/
	
	public double getEngrB_InfeedSpin_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn) {
			if(Math.abs(_engineeringGamepadB.getX(Hand.kLeft)) >= JOYSTICK_DEADBAND) {
				if(_engineeringGamepadB.getX(Hand.kLeft) > 0){
					return 1.0;	// left
				} 
				else if(_engineeringGamepadB.getX(Hand.kLeft) < 0){
					return -1.0;	// right
				}
			}
			else {
				return 0.0;		
			}
		}

		return 0.0;
	}
	
	// joystick acting as button
	/*public Boolean getEngrB_InfeedSpinLeft_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			// flip the sign, pushing the joystick up is a # < 0
			if(Math.abs(_engineeringGamepadB.getX(Hand.kLeft)) >= JOYSTICK_DEADBAND
					&& _engineeringGamepadB.getX(Hand.kLeft) > 0){
				return true;
			} else {
				return false;
			}
		}
		else {
			return false;
		}
	}*/
	
	// joystick acting as button
	/*public Boolean getEngrB_InfeedSpinRight_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			// flip the sign, pushing the joystick up is a # < 0
			if(Math.abs(_engineeringGamepadB.getX(Hand.kLeft)) >= JOYSTICK_DEADBAND
				&& _engineeringGamepadB.getX(Hand.kLeft) < 0){
				return true;
			} else {
				return false;
			}
		}
		else {
			return false;
		}
	}*/

	// joystick acting as button
	/*public Boolean getEngrB_InfeedSpinStop_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn)
		{
			if(Math.abs(_engineeringGamepadB.getY(Hand.kLeft)) < JOYSTICK_DEADBAND) {
				return true;
			}
			else {
				return false;					
			}
		}
		else {
			return false;
		}
	}*/
	
	/*public double getEngrB_LeftTrigger_JoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return  _engineeringGamepadB.getTriggerAxis(Hand.kLeft);
		}
		else {
			return 0;
		}
	}*/
	
	/*public double getEngrB_RightTriggerJoystickCmd() {
		if(_isEngineeringGamepadB_PluggedIn) {
			return  _engineeringGamepadB.getTriggerAxis(Hand.kRight);
		}
		else {
			return 0;
		}
	}*/
}