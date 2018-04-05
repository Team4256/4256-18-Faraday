package com.cyborgcats.reusable.Autonomous;

public interface Path {
	public boolean increment(final double amount);
	public double getX();
	public double getY();
	public double getIndependentVariable();
}
