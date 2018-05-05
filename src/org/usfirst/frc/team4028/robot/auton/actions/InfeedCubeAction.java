package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.CubeHandler;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

import edu.wpi.first.wpilibj.Timer;


public class InfeedCubeAction implements Action {
	CubeHandler _cubeHandler = CubeHandler.getInstance();
	//Infeed _infeed = Infeed.getInstance();	conform to society and use cube handler everywhere

	Chassis _chassis = Chassis.getInstance();
	@SuppressWarnings("unused")
	private double _startTime, _startAngle, _otherTime;
	private boolean _isComplete;
	private enum INFEED_CUBE_AUTON_STATE
	{
		EVERYTHING_HAS_GONE_ACCORDING_TO_PLAN,
		JAM_CENTER,
		LEFT_OR_RIGHT,
		SHORT,
		THAT_ONE_OTHER_JAM_THAT_I_COULDNT_FIGURE_OUT_BEFORE_AND_HAS_NO_NAME,
		UNDEFINED
	}
	private INFEED_CUBE_AUTON_STATE _infeedCubeState;
	
	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
		_startAngle = _chassis.getHeading();
		_infeedCubeState = INFEED_CUBE_AUTON_STATE.EVERYTHING_HAS_GONE_ACCORDING_TO_PLAN;
	}

	@Override
	public void update() {
		switch(_infeedCubeState)
		{
			case EVERYTHING_HAS_GONE_ACCORDING_TO_PLAN:
				//System.out.println("Test");
				_cubeHandler.acquireCube_InfeedAndCarriage();
				_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
				_isComplete = true;
				break;
			
			case JAM_CENTER:
				if(Timer.getFPGATimestamp()-_startTime<1.315)
				{
					_cubeHandler.ejectCube_InfeedAndCarriage();
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
				}
				else if(Timer.getFPGATimestamp()-_startTime<1.45)
				{
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE);
					_cubeHandler.acquireCube_InfeedAndCarriage();
					
					//_cubeHandler.carriage_FeedOut(CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60);
				}
				else
				{
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
				}
				if(Timer.getFPGATimestamp()-_startTime<1.65)
				{
					_cubeHandler.infeedWheels_SpinAuton();
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
					_isComplete = false;
				}
				else
				{
					_startTime= Timer.getFPGATimestamp();
					_isComplete = true;
				}
				break;
				
			case LEFT_OR_RIGHT:
				if(Timer.getFPGATimestamp()-_startTime<1.45)
				{
					_chassis.arcadeDrive(1, 0);
					_isComplete=false;
				}
				else
				{
					_startTime = Timer.getFPGATimestamp();
					_chassis.stop();
					_isComplete = true;
				}
				break;
			case SHORT:
				if(Timer.getFPGATimestamp()-_startTime<1.55)
				{
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE);
					_chassis.arcadeDrive(-1, 0);
					_isComplete=false;
				}
				else
				{
					_startTime = Timer.getFPGATimestamp();
					_chassis.stop();
					_isComplete = true;
				}
				break;	
			
			case THAT_ONE_OTHER_JAM_THAT_I_COULDNT_FIGURE_OUT_BEFORE_AND_HAS_NO_NAME:
				if(Timer.getFPGATimestamp()- _startTime<1.45)
				{
					_cubeHandler.infeedArms_MoveToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE);
					_cubeHandler.ejectCube_InfeedAndCarriage();
					_isComplete = false;
				}
				else
				{
					_startTime = Timer.getFPGATimestamp();
					_isComplete = true;
				}
				break;
				
				
			case UNDEFINED:
				break;
		}
		
		
		
		if ((Timer.getFPGATimestamp() - _startTime) < 1.25) 
		{
			_infeedCubeState= INFEED_CUBE_AUTON_STATE.EVERYTHING_HAS_GONE_ACCORDING_TO_PLAN;
		} 
		else 
		{
			if(_isComplete)
			{
				//System.out.println("We made it");
				if(Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos()))<=190 && 
					Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos()))>150 &&
					Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos()))<=190 &&
					Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos()))>150)
				{
					_infeedCubeState = INFEED_CUBE_AUTON_STATE.JAM_CENTER;				
				}
			
				else if(Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos()))<150 &&
						Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos()))>170 ||
						Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos()))>170 &&
						Math.abs(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos()))<150
						|| _cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos())>230
						|| _cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos())>230)
				{
					_infeedCubeState = INFEED_CUBE_AUTON_STATE.LEFT_OR_RIGHT;
				}
				
				else if(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos())>190 && 
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos())<230 && 
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos())>190 &&
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos())<230
						)
				{
					_infeedCubeState=INFEED_CUBE_AUTON_STATE.SHORT;		
				}
				
				/*else if(_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos())>160 && 
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getRightInfeedArmPos())<200 &&
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos())>140 && 
						_cubeHandler.infeedArm_nativeUnitstoDegrees(_cubeHandler.getLeftInfeedArmPos())<190)
				{
					_infeedCubeState = INFEED_CUBE_AUTON_STATE.THAT_ONE_OTHER_JAM_THAT_I_COULDNT_FIGURE_OUT_BEFORE_AND_HAS_NO_NAME;
				}*/
				else 
				{
					_infeedCubeState= INFEED_CUBE_AUTON_STATE.EVERYTHING_HAS_GONE_ACCORDING_TO_PLAN;
					_startTime = Timer.getFPGATimestamp();
				}
			}
		}
			
	}

	@Override
	public void done() {
		_cubeHandler.stop();
	}

	@Override
	public boolean isFinished() {
		return _cubeHandler.isCubeInCarriage();
	}
}