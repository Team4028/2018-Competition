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
		DRIVER_LEFT_X_AXIS							Throttle Cmd
		DRIVER_LEFT_Y_AXIS				
		DRIVER_LEFT_TRIGGER			
		DRIVER_RIGHT_TRIGGER		
		DRIVER_RIGHT_X_AXIS							Turn Cmd
		DRIVER_RIGHT_Y_AXIS
		
		--- Driver Buttons --------
		DRIVER_GREEN_BUTTON_A		
		DRIVER_RED_BUTTON_B			 
		DRIVER_BLUE_BUTTON_X		
		DRIVER_YELLOW_BUTTON_Y		
		DRIVER_LEFT_BUMPER			
		DRIVER_RIGHT_BUMPER			
		DRIVER_BACK_BUTTON			
		DRIVER_START_BUTTON			
		DRIVER_LEFT_THUMBSTICK
		DRIVER_RIGHT_THUMBSTICK
								
		==== OPERATOR ==========================================================
			
		--- Operator Joysticks --------
		OPERATOR_LEFT_X_AXIS			
		OPERATOR_LEFT_Y_AXIS			
		OPERATOR_LEFT_TRIGGER			
		OPERATOR_RIGHT_TRIGGER			
		OPERATOR_RIGHT_X_AXIS
		OPERATOR_RIGHT_Y_AXIS			
		
		--- Operator Buttons --------
		OPERATOR_GREEN_BUTTON_A			
		OPERATOR_RED_BUTTON_B			
		OPERATOR_BLUE_BUTTON_X			
		OPERATOR_YELLOW_BUTTON_Y		
		OPERATOR_LEFT_BUMPER			
		OPERATOR_RIGHT_BUMPER			
		OPERATOR_BACK_BUTTON			
		OPERATOR_START_BUTTON			
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

public class DriverOperationStation 
{
		// class level private variables
		private XboxController _driverGamepad;
		private XboxController _operatorGamepad;
		private XboxController _engineeringGamepad;
			
		// singleton pattern
		private static DriverOperationStation _instance = new DriverOperationStation();

		public static DriverOperationStation getInstance() 
		{
			return _instance;
		}
		
		// private constructor for singleton pattern
		private DriverOperationStation() 
		{
			_driverGamepad = new XboxController(Constants.DRIVER_GAMEPAD_USB_PORT);				// std Logitech F310 Gamepad  
			_operatorGamepad = new XboxController(Constants.OPERATOR_GAMEPAD_USB_PORT);			// std Logitech F310 Gamepad  
			_engineeringGamepad = new XboxController(Constants.ENGINEERING_GAMEPAD_USB_PORT);	// std Logitech F310 Gamepad  
			
			if(_driverGamepad == null) {
				DriverStation.reportError("DriverGamepad not present!", false);
			}
			if(_operatorGamepad == null) {
				DriverStation.reportError("OperatorGamepad not present!", false);
			}
			if(_engineeringGamepad == null) {
				DriverStation.reportWarning("EngineeringGamepad not present!", false);
			}
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
	
//		public boolean getIsDriver_Start_BtnJustPressed() {
//			return _driverGamepad.getStartButtonPressed();
//		}
		
//		public boolean getIsDriver_YellowY_BtnJustPressed() {
//			return _driverGamepad.getYButtonPressed();
//		}
		 
//		public boolean getIsDriver_RedB_BtnJustPressed() {
//			return _driverGamepad.getBButtonPressed();
//		}
	
		public boolean getIsDriver_MoveToInfeedPosition_BtnJustPressed() {
			return _driverGamepad.getAButtonPressed();
		}	

		public boolean getIsDriver_ReZeroInfeed_BtnJustPressed() {
			return _driverGamepad.getXButtonPressed();
		}
		
//		public boolean getIsDriver_LeftBumper_BtnJustPressed() {
//			return _driverGamepad.getBumperPressed(Hand.kLeft);
//		}
		
//		public boolean getIsDriver_RightBumper_BtnJustPressed() {
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
	
		public boolean getIsShiftGearJustPressed() {
			return _driverGamepad.getStartButtonReleased();
		}
		
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
		
//		public boolean getIsDriver_LeftBumper_BtnPressed() {
//			return _driverGamepad.getBumper(Hand.kLeft);
//		}
		
//		public boolean getIsDriver_RightBumper_BtnPressed() {
//			return _driverGamepad.getBumper(Hand.kRight);
//		}
		
//		public boolean getIsDriver_LeftStick_BtnPressed() {
//			return _driverGamepad.getStickButton(Hand.kLeft);
//		}
		
//		public boolean getIsDriver_RightStick_BtnPressed() {
//			return _driverGamepad.getStickButton(Hand.kRight);
//		}
	
		// ===================================
		// ======== Driver Joysticks =========
		// ===================================
	
		public double getThrottleCmd() {
			// invert command since we want up to be fwd
			return (_driverGamepad.getY(Hand.kLeft) * -1.0);
		}
		
//		public double getDriver_LeftX_JoystickCmd() {
//			return _driverGamepad.getX(Hand.kLeft);
//		}
		
//		public double getDriver_RightY_JoystickCmd() {
//			return _driverGamepad.getY(Hand.kRight);
//		}
		
		public double getTurnCmd() {
			return _driverGamepad.getX(Hand.kRight);
		}

//		public double getDriver_LeftTrigger_JoystickCmd() {
//			return _driverGamepad.getTriggerAxis(Hand.kLeft);
//		}
		
//		public double getDriver_RightTrigger_JoystickCmd() {
//			return _driverGamepad.getTriggerAxis(Hand.kRight);
//		}
		
		// =========================================================================================================
		// OPERATOR		OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	
		// =========================================================================================================
		
		// ===================================
		// == Operator Just Pressed buttons ==
		// ===================================
	
//		public boolean getIsOperator_Back_BtnJustPressed() {
//			return _operatorGamepad.getBackButtonPressed();
//		}
	
//		public boolean getIsOperator_Start_BtnJustPressed() {
//			return _operatorGamepad.getStartButtonPressed();
//		}
		
//		public boolean getIsOperator_YellowY_BtnJustPressed() {
//			return _operatorGamepad.getYButtonPressed();
//		}
		 
//		public boolean getIsOperator_RedB_BtnJustPressed() {
//			return _operatorGamepad.getBButtonPressed();
//		}
		
//		public boolean getIsOperator_GreenA_BtnJustPressed() {
//			return _operatorGamepad.getAButtonPressed();
//		}	
	
//		public boolean getIsOperator_BlueX_BtnJustPressed() {
//			return _operatorGamepad.getXButtonPressed();
//		}
		
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
		
//		public boolean getIsOperator_LeftBumper_BtnPressed() {
//			return _operatorGamepad.getBumper(Hand.kLeft);
//		}
		
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
	
//		public double getOperator_LeftY_JoystickCmd() {
//			return _operatorGamepad.getY(Hand.kLeft);
//		}
		
//		public double getOperator_LeftX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kLeft);
//		}
		
//		public double getOperator_RightY_JoystickCmd() {
//			return _operatorGamepad.getY(Hand.kRight);
//		}
		
//		public double getOperator_RightX_JoystickCmd() {
//			return _operatorGamepad.getX(Hand.kRight);
//		}

//		public double getOperator_LeftTrigger_JoystickCmd() {
//			return _operatorGamepad.getTriggerAxis(Hand.kLeft);
//		}
		
//		public double getOperator_RightTrigger_JoystickCmd() {
//			return _operatorGamepad.getTriggerAxis(Hand.kRight);
//		}
		
		// =========================================================================================================
		// ENGINEER		ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER	ENGINEER
		// =========================================================================================================
		
		// ======================================
		// == Engineering Just Pressed buttons ==
		// ======================================
	
//		public boolean getIsEngineering_Back_BtnJustPressed() {
//			return _engineeringGamepad.getBackButtonPressed();
//		}
	
//		public boolean getIsEngineering_Start_BtnJustPressed() {
//			return _engineeringGamepad.getStartButtonPressed();
//		}
		
//		public boolean getIsEngineering_YellowY_BtnJustPressed() {
//			return _engineeringGamepad.getYButtonPressed();
//		}
		 
//		public boolean getIsEngineering_RedB_BtnJustPressed() {
//			return _engineeringGamepad.getBButtonPressed();
//		}
		
//		public boolean getIsEngineering_GreenA_BtnJustPressed() {
//			return _engineeringGamepad.getAButtonPressed();
//		}	
	
//		public boolean getIsEngineering_BlueX_BtnJustPressed() {
//			return _engineeringGamepad.getXButtonPressed();
//		}
		
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
		
//		public boolean getIsEngineering_LeftBumper_BtnPressed() {
//			return _engineeringGamepad.getBumper(Hand.kLeft);
//		}
		
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
	
//		public double getEngineering_LeftY_JoystickCmd() {
//			return _engineeringGamepad.getY(Hand.kLeft);
//		}
		
//		public double getEngineering_LeftX_JoystickCmd() {
//			return _engineeringGamepad.getX(Hand.kLeft);
//		}
		
//		public double getEngineering_RightY_JoystickCmd() {
//			return _engineeringGamepad.getY(Hand.kRight);
//		}
		
//		public double getEngineering_RightX_JoystickCmd() {
//			return _engineeringGamepad.getX(Hand.kRight);
//		}

//		public double getEngineering_LeftTrigger_JoystickCmd() {
//			return _engineeringGamepad.getTriggerAxis(Hand.kLeft);
//		}
		
//		public double getEngineering_RightTrigger_JoystickCmd() {
//			return _engineeringGamepad.getTriggerAxis(Hand.kRight);
//		}
	}