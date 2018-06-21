package com.cyborgcats.reusable;

public interface Subsystem {
	public void init();
	public boolean perform(final String action, final double[] data);
	public void completeLoopUpdate();
}
