package org.usfirst.frc.team4256.robot.Elevators;

import com.cyborgcats.reusable.Phoenix.Encoder;
import com.cyborgcats.reusable.Phoenix.Talon;
import com.cyborgcats.reusable.Phoenix.Victor;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class L_One extends Lift {
	private static final DoubleSolenoid.Value HighGear = DoubleSolenoid.Value.kReverse;//should be kForward, but shifter is broken
	private static final DoubleSolenoid.Value LowGear = DoubleSolenoid.Value.kReverse;
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 2.873*Math.PI;//inches
	protected static final double maximumHeight = 42.5;//inches
	protected static final double hookBaseline = 44.0;//inches
	private Talon master;
	private Victor followerA;
	private DoubleSolenoid shifter;
	private int maximumEncoderValue;
	public boolean knowsZero = false;

	public L_One(final int masterID, final int followerAID, final int followerBID, final DoubleSolenoid shifter) {
		master = new Talon(masterID, gearRatio, Talon.position, Encoder.OEM_QUAD, true);//practice: true, comp: true
		followerA = new Victor(followerAID, Victor.follower);
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
			master.selectProfileSlot(1, 0);
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
	@Override
	public void init() {
		master.init();
		
		master.setNeutralMode(Talon.brake);
		enableSoftLimits();
		
		master.config_kP(0, 0.7, Talon.kTimeoutMS);
		master.config_kI(0, 0.0, Talon.kTimeoutMS);
		master.config_kD(0, 0.0, Talon.kTimeoutMS);
		master.config_kP(1, .45, Talon.kTimeoutMS);
		master.config_kI(1, 0.0, Talon.kTimeoutMS);
		master.config_kD(1, 10.0, Talon.kTimeoutMS);

		followerA.init(master);
	}
	
	
	private void enableSoftLimits() {
		master.configForwardSoftLimitEnable(true, Talon.kTimeoutMS);
		master.configReverseSoftLimitEnable(true, Talon.kTimeoutMS);
		master.configReverseSoftLimitThreshold(0, Talon.kTimeoutMS);//assuming negative motor voltage results in downward motion
		master.configForwardSoftLimitThreshold(maximumEncoderValue, Talon.kTimeoutMS);
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
	@Override
	public void setInches(final double inches) {
		setRevs(inchesToRevs(validateInches(inches)));
	}
	
	
	/**
	 * 
	**/
	@Override
	public double getInches() {
		return revsToInches(getRevs());
	}
	
	
	/**
	 * 
	**/
	@Override
	public void increment(final double inches, final boolean startingAtPreviousSetpoint) {
		double newSetpoint = getInches() + inches;
		if (startingAtPreviousSetpoint) newSetpoint += master.getCurrentError(false);
		setInches(newSetpoint);
	}
	
	
	/**
	 * Threshold should be specified in inches. If the elevator is within that many inches of its target, this function returns true.
	**/
	@Override
	public boolean isThere(final double threshold) {
		return Math.abs(revsToInches(master.getCurrentError(false))) <= threshold;
	}
	
	
	/**
	 * A shortcut to call overrideSoftLimits on all the Talons in the elevator.
	**/
	@Override
	public void overrideSoftLimits(final boolean enable) {
		master.overrideSoftLimitsEnable(enable);
	}
	
	@Override
	public void setZero(final double offsetInchesFromCurrent) {
		master.setSelectedSensorPosition(-(int)master.convert.from.REVS.afterGears(inchesToRevs(offsetInchesFromCurrent)), 0, Talon.kTimeoutMS);
		enableSoftLimits();
		knowsZero = true;
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	@Override
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
