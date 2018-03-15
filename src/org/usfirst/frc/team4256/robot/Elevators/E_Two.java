package org.usfirst.frc.team4256.robot.Elevators;

import com.cyborgcats.reusable.Phoenix.R_Encoder;
import com.cyborgcats.reusable.Phoenix.R_Talon;

public class E_Two implements Elevator {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 1.29*Math.PI;//inches
	protected static final double maximumHeight = 40.5;//inches
	protected static final double climbingHeight = 0.0;//inches
	private R_Talon master;
	private int maximumEncoderValue;
	public boolean knowsZero = false;
	
	public E_Two(final int masterID) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.CTRE_MAG_ABSOLUTE, true);
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * This function prepares the motor by enabling soft limits and setting PID values.
	**/
	public void init() {
		master.init();
		master.configAllowableClosedloopError(0, (int)master.convert.from.REVS.afterGears(inchesToRevs(0.5)), R_Talon.kTimeoutMS);//TODO
		
		master.setNeutralMode(R_Talon.brake);
		master.setInverted(true);
		enableSoftLimits();

		master.config_kP(0, 0.17, R_Talon.kTimeoutMS);
		master.config_kI(0, 0.0, R_Talon.kTimeoutMS);
		master.config_kD(0, 1.7, R_Talon.kTimeoutMS);
	}
	
	
	private void enableSoftLimits() {
		master.configForwardSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitThreshold(0, R_Talon.kTimeoutMS);//assuming negative motor voltage results in downward motion
		master.configForwardSoftLimitThreshold(maximumEncoderValue, R_Talon.kTimeoutMS);
	}
	
	
	/**
	 * This function sets the elevator to a certain revolution value using PID.
	**/
	private void setRevs(final double revs) {
		master.quickSet(revs, false);
	}
	
	
	/**
	 * A shortcut to call getCurrentRevs on the motor.
	**/
	private double getRevs() {
		return master.getCurrentRevs();
	}
	
	
	private double validateInches(final double inches) {
		if (inches > maximumHeight) {
			return maximumHeight;
		}else if (inches < 0.0) {
			return 0.0;
		}else {
			return inches;
		}
	}
	
	
	/**
	 * This function sends the elevator to a certain height after clipping the input.
	**/
	public void setInches(final double inches) {
		setRevs(inchesToRevs(validateInches(inches)));
	}
	
	
	/**
	 * 
	**/
	public double getInches() {
		return revsToInches(getRevs());
	}
	
	
	/**
	 * 
	**/
	public void increment(final double inches, final boolean startingAtPreviousSetpoint) {
		double newSetpoint = getInches() + inches;
		if (startingAtPreviousSetpoint) newSetpoint += master.getCurrentError(false);
		setInches(newSetpoint);
	}
	
	
	/**
	 * Threshold should be specified in inches. If the elevator is within that many inches of its target, this function returns true.
	**/
	public boolean isThere(final double threshold) {
		return Math.abs(revsToInches(master.getCurrentError(false))) <= threshold;
	}
	
	
	/**
	 * A shortcut to call overrideSoftLimits on all the Talons in the elevator.
	**/
	public void overrideSoftLimits(final boolean enable) {
		master.overrideSoftLimitsEnable(enable);
	}
	
	
	public void setZero(final double offsetInchesFromCurrent) {
		master.setSelectedSensorPosition(0 + (int)master.convert.from.REVS.afterGears(inchesToRevs(offsetInchesFromCurrent)), 0, R_Talon.kTimeoutMS);
		knowsZero = true;
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	public void completeLoopUpdate() {
		master.completeLoopUpdate();
	}
	
	
	/**
	 * This function converts inches to revolutions.
	**/
	private static double inchesToRevs(final double inches) {
		return inches/sprocketCircumference;
	}
	
	
	/**
	 * This functions converts revolutions to inches.
	**/
	private static double revsToInches(final double revs) {
		return sprocketCircumference*revs;
	}
}
