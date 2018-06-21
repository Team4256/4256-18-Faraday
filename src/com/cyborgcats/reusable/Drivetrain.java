package com.cyborgcats.reusable;

public interface Drivetrain {
	public void init();
	
	public void setSpeed(final double speed);
	public void setSpin(final double spin);
	public void travelTowards(final double heading);
	
	public void correctFor(final double errorDirection, final double errorMagnitude);
	public double face(final double heading, double maximumOutput);
	
	public void completeLoopUpdate();
}
