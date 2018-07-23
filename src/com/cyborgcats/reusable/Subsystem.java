package com.cyborgcats.reusable;

/**
 * A set of common methods which can be implemented for many different mechanisms.<br>
 * Having them all under one interface makes it easier to write reusable autonomous code.
 */
public interface Subsystem {
	public void init();
	public boolean perform(final String action, final double[] data);
	public void completeLoopUpdate();
}
