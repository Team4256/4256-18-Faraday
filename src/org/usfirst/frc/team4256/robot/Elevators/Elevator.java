package org.usfirst.frc.team4256.robot.Elevators;

public abstract class Elevator {
	public abstract void init();
	public abstract void setInches(final double inches);
	public abstract double getInches();
	public abstract void increment(final double inches, final boolean startingAtPreviousSetpoint);
	public abstract boolean isThere(final double threshold);
	public abstract void overrideSoftLimits(final boolean enable);
	public abstract void setZero(final double offsetInchesFromCurrent);
	public abstract void completeLoopUpdate();
}
