package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.cyborgcats.reusable.Phoenix.R_Encoder;
import com.cyborgcats.reusable.Phoenix.R_Talon;
import com.cyborgcats.reusable.Phoenix.R_Victor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class R_Clamp {
	private static final DoubleSolenoid.Value CloseState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OpenState = DoubleSolenoid.Value.kReverse;
	private static final DoubleSolenoid.Value UpState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OutState = DoubleSolenoid.Value.kReverse;
	private R_Victor intakeLeft;
	private R_Victor intakeRight;
	private DoubleSolenoid clamp;
	private R_Talon rotator;
	private AnalogInput ultrasonic;
	private static int counter;
	
	private final double intakeConstant = 0.85;
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final int rotatorID, final int ultrasonicPort) {
		intakeLeft = new R_Victor(intakeLeftID, R_Victor.percent);
		intakeRight = new R_Victor(intakeRightID, R_Victor.percent);
		rotator = new R_Talon(rotatorID, 1.0, ControlMode.PercentOutput, R_Encoder.CTRE_MAG_ABSOLUTE, false);
		this.clamp = clamp;
		ultrasonic = new AnalogInput(ultrasonicPort);
	}
	
	public void init() {
		intakeLeft.init();
		intakeRight.init();
		counter = 0;
	}
	
	
	/**
	 * This function attempts to "slurp" nearby cubes into the clamp.
	**/
	public void slurp() {
		if (!cubeInReach()) {
			clamp.set(OpenState);
			setWheelSpeed(-intakeConstant);
		} else {
			clamp.set(CloseState);
			if (hasCube()) {
				setWheelSpeed(0.0);
			}
		}
	}
	
	
	/**
	 * This function attempts to "spit" cubes out of the clamp.
	**/
	public void spit() {
		setWheelSpeed(intakeConstant);
	}
	
	
	/**
	 * This function stops the clamp motors (thereby stopping slurp and spit).
	**/
	public void stop() {
		setWheelSpeed(0.0);
	}
	
	
	/**
	 * 
	**/
	private void setWheelSpeed(final double percent) {
		intakeLeft.quickSet(percent);
		intakeRight.quickSet(-percent);
	}
	
	
	/**
	 * This function opens the clamp 
	**/
	public void open() {
		clamp.set(OpenState);
	}
	
	
	/**
	 * This function closes the clamp 
	**/
	public void close() {
		clamp.set(CloseState);
	}
	
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isOpen() {
		return clamp.get().equals(OpenState);
	}
	
	
	/**
	 * This function returns if the cube is in the clamp or not
	 **/
	public boolean hasCube() {
		if (ultrasonic.getAverageValue() < 55) {
			counter++;
			return counter > 10;
		} else {
			counter = 0;
			return false;
		}
	}
	
	
	/**
	 * This function returns if the cube is in reach of the clamp or not
	 **/
	public boolean cubeInReach() {
		return ultrasonic.getAverageValue() < 70;
	}
	
	/**
	 * 
	**/
	public void extend() {
	}
	
	/**
	 * 
	**/
	public void retract() {
	}
	
	public void rotatorStop() {
	}
	
	
	/**
	 * This function returns if the clamp is looking up or straight
	 **/
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	public void completeLoopUpdate() {
		intakeLeft.completeLoopUpdate();
		intakeRight.completeLoopUpdate();
		rotator.completeLoopUpdate();//TODO maybe not?
		SmartDashboard.putNumber("rotator REVS", rotator.getCurrentRevs());
		SmartDashboard.putNumber("ultrasonic", ultrasonic.getAverageValue());
		SmartDashboard.putBoolean("in reach", cubeInReach());
	}
}