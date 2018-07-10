package com.cyborgcats.reusable;

/* This interface is designed such that its implementations can be put into an array and easily passed around during autonomous.
 * Essentially it represents a way for our normally iterative code to become temporarily command based. */
public interface Subsystem {
	public void init();
	public boolean perform(final String action, final double[] data);
	public void completeLoopUpdate();
}
