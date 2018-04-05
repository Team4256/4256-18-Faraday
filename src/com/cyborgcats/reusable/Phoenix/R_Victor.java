package com.cyborgcats.reusable.Phoenix;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class R_Victor extends VictorSPX {
	
	public static final ControlMode follower = ControlMode.Follower;
	public static final ControlMode percent = ControlMode.PercentOutput;
	public static final ControlMode disabled = ControlMode.Disabled;
	public static final NeutralMode brake = NeutralMode.Brake;
	public static final NeutralMode coast = NeutralMode.Coast;
	
	private ControlMode controlMode;
	public static final int kTimeoutMS = 10;
	private Double lastSetpoint = 0.0;
	private boolean updated = false;
	
	public R_Victor(int deviceID, final ControlMode controlMode) {
		super(deviceID);
		this.controlMode = controlMode;
	}
	
	
	public void init() {
		init(1.0);
	}
	
	
	public void init(final R_Talon master) {
		init(1.0);
		follow(master);
	}
	
	
	/**
	 * This function prepares a motor by setting the minimum and maximum percentages.
	**/
	public void init(final double maxPercent) {
		clearStickyFaults(kTimeoutMS);//TODO everywhere where we have kTimeoutMS, do error handling
		
		configNominalOutputForward(0.0, kTimeoutMS);
		configNominalOutputReverse(0.0, kTimeoutMS);
		configPeakOutputForward(Math.abs(maxPercent), kTimeoutMS);
		configPeakOutputReverse(-Math.abs(maxPercent), kTimeoutMS);
		
		quickSet(0.0);
	}
	
	
	/**
	 * This function sets the motor's output based on the control mode.
	 * Percent: -1 to 1
	**/
	public void quickSet(final double value) {
		try {
			this.set(value, true);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
	}
	
	
	public void set(final double value, final boolean updateSetPoint) throws IllegalAccessException {
		double currentSetPoint = lastSetpoint;
		switch (controlMode) {
		case Current:throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is incompatible with Victors");
		case Follower:break;
		case PercentOutput:currentSetPoint = setPercent(value);break;
		case Position:throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is incompatible with Victors");
		case Velocity:throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is incompatible with Victors");
		case Disabled:break;
		default:throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + "'s mode is unimplemented.");
		}
		
		updated = true;
		if (updateSetPoint) lastSetpoint = currentSetPoint;
	}
	
	
	private double setPercent(final double percentage) throws IllegalAccessException {
		if (controlMode == percent) {
			super.set(controlMode, percentage);
		}else {
			throw new IllegalAccessException("Victor " + Integer.toString(getDeviceID()) + " was given percentage in " + controlMode.name() + " mode.");
		}return percentage;
	}
	
	
	public void enterNeutral() {
		neutralOutput();
		updated = true;
		lastSetpoint = null;
	}
	
	
	/**
	 * Run this after all other commands in a system level loop to make sure the Talon receives a command.
	**/
	public void completeLoopUpdate() {
		if (!updated) {
			if (lastSetpoint != null) super.set(controlMode, lastSetpoint);//send a command if there hasn't yet been one, using raw encoder units
			else neutralOutput();
		}
		
		if (getControlMode() != follower) {updated = false;}//loop is over, reset updated for use in next loop (followers excluded)
	}
}
