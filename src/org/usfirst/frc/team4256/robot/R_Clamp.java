package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.cyborgcats.reusable.Phoenix.R_Victor;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Clamp {
	private static final DoubleSolenoid.Value CloseState = DoubleSolenoid.Value.kForward;//TODO test all of these
	private static final DoubleSolenoid.Value OpenState = DoubleSolenoid.Value.kReverse;
	private static final DoubleSolenoid.Value UpState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OutState = DoubleSolenoid.Value.kReverse;
	private R_Victor intakeLeft;
	private R_Victor intakeRight;
	private DoubleSolenoid clamp;
	private DoubleSolenoid extender;
	
	private final double intakeConstant = 0.85;
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final DoubleSolenoid extended) {
		intakeLeft = new R_Victor(intakeLeftID, R_Victor.percent);
		intakeRight = new R_Victor(intakeRightID, R_Victor.percent);
		this.clamp = clamp;
		this.extender = extended;
	}
	
	public void init() {
		intakeLeft.init();
		intakeRight.init();
	}
	
	
	/**
	 * This function attempts to "slurp" nearby cubes into the clamp.
	**/
	public void slurp() {
		setWheelSpeed(-intakeConstant);
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
	 * 
	**/
	public void extend() {
		extender.set(OutState);
	}
	
	
	/**
	 * 
	**/
	public void retract() {
		extender.set(UpState);
	}
	
	
	/**
	 * This function returns if the clamp is looking up or straight
	 **/
	public boolean isExtended() {
		return extender.get().equals(OutState);
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	public void completeLoopUpdate() {
		intakeLeft.completeLoopUpdate();
		intakeRight.completeLoopUpdate();
	}
}