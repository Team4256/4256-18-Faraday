package org.usfirst.frc.team4256.robot.Elevators;

public interface Elevator {
	public void init();
	public void setInches(final double inches);
	public double getInches();
	public void increment(final double inches, final boolean startingAtPreviousSetpoint);
	public boolean isThere(final double threshold);
	public void overrideSoftLimits(final boolean enable);
	public void setZero(final double offsetInchesFromCurrent);
	public void completeLoopUpdate();
}
