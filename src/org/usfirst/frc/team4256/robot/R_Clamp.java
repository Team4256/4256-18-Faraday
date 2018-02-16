package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Clamp {
	private static final DoubleSolenoid.Value OpenState = DoubleSolenoid.Value.kForward;//TODO test all of these
	private static final DoubleSolenoid.Value CloseState = DoubleSolenoid.Value.kReverse;
	private static final DoubleSolenoid.Value UpState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OutState = DoubleSolenoid.Value.kReverse;
	private VictorSPX intakeLeft;
	private VictorSPX intakeRight;
	private DoubleSolenoid clamp;
	private DoubleSolenoid extender;
	
	private final double intakeConstant = 0.7;
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final DoubleSolenoid extended) {
		intakeLeft = new VictorSPX(intakeLeftID);
		intakeRight = new VictorSPX(intakeRightID);
		this.clamp = clamp;
		this.extender = extended;
	}
	
	public void init() {
		//TODO
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
		intakeLeft.set(ControlMode.PercentOutput, percent);
		intakeRight.set(ControlMode.PercentOutput, percent);
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
}