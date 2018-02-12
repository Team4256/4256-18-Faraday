package org.usfirst.frc.team4256.robot.Elevators;

import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;

public class R_ElevatorTwo {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 1.29*Math.PI;//inches
	public static final double maximumHeight = 42.5;//inches
	private R_Talon master;
	private DigitalInput sensor;
	private boolean knowsZero = false;
	private int maximumEncoderValue;
	
	public R_ElevatorTwo(final int masterID, final int sensorID) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.CTRE_MAG_ABSOLUTE, true);
		sensor = new DigitalInput(sensorID);
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * This function prepares the motor by enabling soft limits and setting PID values.
	**/
	public void init() {
		master.init();
		
		master.setNeutralMode(R_Talon.coast);
		master.setInverted(true);
		enableSoftLimits();
		
		master.config_kP(0, 0.1, R_Talon.kTimeoutMS);
		master.config_kI(0, 0.0, R_Talon.kTimeoutMS);
		master.config_kD(0, 0.0, R_Talon.kTimeoutMS);
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
	public void increment(final double inches) {//TODO make a boolean to determine whether we are incrementing from current height, or the previous target height
		setInches(getInches() + inches);
	}
	
	/**
	 *
	**/
	public void findZero() {
		if(!sensor.get()) {//not at zero
			master.overrideSoftLimitsEnable(true);
			knowsZero = false;
			increment(-0.3);
		}else {//at zero
			setZero(0.0);
			master.overrideSoftLimitsEnable(false);
			
			knowsZero = true;
		}
	}
	
	public void setZero(final double offsetInchesFromCurrent) {
		master.setSelectedSensorPosition(0 + (int)master.convert.from.REVS.afterGears(inchesToRevs(offsetInchesFromCurrent)), 0, R_Talon.kTimeoutMS);
	}
	
	/**
	 * 
	**/
	public boolean knowsZero() {
		return knowsZero;
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
