package org.usfirst.frc.team4256.robot.Elevators;

import com.cyborgcats.reusable.Phoenix.R_Encoder;
import com.cyborgcats.reusable.Phoenix.R_Talon;
import com.cyborgcats.reusable.Phoenix.R_Victor;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_ElevatorOne {
	private static final DoubleSolenoid.Value HighGear = DoubleSolenoid.Value.kReverse;
	private static final DoubleSolenoid.Value LowGear = DoubleSolenoid.Value.kForward;
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 2.873*Math.PI;//inches
	protected static final double maximumHeight = 37.0;//inches
	protected static final double hookBaseline = 44.0;//inches
	private R_Talon master;
	private R_Victor followerA;
	private R_Victor followerB;
	private DoubleSolenoid shifter;
	private int maximumEncoderValue;
	public boolean knowsZero = false;

	public R_ElevatorOne(final int masterID, final int followerAID, final int followerBID, final DoubleSolenoid shifter) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, true);//practice: false, comp: true
		followerA = new R_Victor(followerAID, R_Victor.follower);
		followerB = new R_Victor(followerBID, R_Victor.follower);
		this.shifter = shifter;	
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * 
	**/
	public void setTorque(final boolean aWholeLot) {
		if (aWholeLot) {
			shifter.set(LowGear);
			master.selectProfileSlot(0, 0);
		}
		else {
			shifter.set(HighGear);
			master.selectProfileSlot(0, 1);
		}
	}
	
	
	/**
	 * 
	**/
	public boolean hasLotsOfTorque() {
		return shifter.get().equals(LowGear);
	}
	
	
	/**
	 * This function prepares each motor individually by enabling soft limits, setting PID values, and commanding followers.
	**/
	public void init() {
		master.init();
		
		master.setNeutralMode(R_Talon.brake);
		enableSoftLimits();
		
		master.config_kP(0, 0.7, R_Talon.kTimeoutMS);
		master.config_kI(0, 0.0, R_Talon.kTimeoutMS);
		master.config_kD(0, 0.0, R_Talon.kTimeoutMS);
		master.config_kP(1, .45, R_Talon.kTimeoutMS);
		master.config_kI(1, 0.0, R_Talon.kTimeoutMS);
		master.config_kD(1, 10.0, R_Talon.kTimeoutMS);

		followerA.init(master);
		followerB.init(master);
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
	 * A shortcut to call getCurrentRevs on the master motor.
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
		enableSoftLimits();
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
