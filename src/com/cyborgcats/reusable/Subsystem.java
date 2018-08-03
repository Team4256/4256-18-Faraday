package com.cyborgcats.reusable;

/**
 * A set of common methods which can be implemented for many different mechanisms.<br>
 * Having them all under one interface makes it easier to write reusable autonomous code.
 */
public interface Subsystem {
	public void init();//not necessary, but common in our code
	/**
	 * A very generic function. Example:<br>
	 * <code>
	 * public boolean perform(String action, double[] data) {<br>
	 * 		switch(Abilities.valueOf(action)) {<br>
	 * 		case SET: setInches(data[0]); break;<br>
	 * 		case INCREMENT: increment(data[0]); break;<br>
	 * 		default: throw new IllegalStateException("The elevator cannot " + action);<br>
	 * 		}<br>
	 * 		return (one.isThere(2.0) && two.isThere(2.0));<br>
	 * }<br>
	 * <br>
	 * public static enum Abilities {SET, INCREMENT}
	 * </code>
	 * @param action name of the action
	 * @param data any additional parameters that are necessary to perform the action
	 * @return whether the action has completed successfully
	 */
	public boolean perform(final String action, final double[] data);
	public void completeLoopUpdate();//not necessary, but common in our code
}
