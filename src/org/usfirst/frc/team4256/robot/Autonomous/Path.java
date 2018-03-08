package org.usfirst.frc.team4256.robot.Autonomous;

public interface Path {
	public boolean increment(final double amount);
	public double getX();
	public double getY();
	public double getIndependentVariable();
}
