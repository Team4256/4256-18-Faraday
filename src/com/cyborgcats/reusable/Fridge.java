package com.cyborgcats.reusable;//COMPLETE 2017

import java.util.HashMap;
import java.util.Map;

/**
 * Helps manage fancy boolean states like toggles.
 */
public final class Fridge {
	private Fridge() {}
	
	private static final Map<String, Boolean> previousStates = new HashMap<String, Boolean>();
	public static final Map<String, Boolean> toggleStates = new HashMap<String, Boolean>();
	public static final Map<String, Long> toggledTimes = new HashMap<String, Long>();
	
	/**
	 * Ensures that all three HashMaps contain the specified key and a corresponding value
	 * @param key a name like "Button A"
	 * @param initState the initial state
	 */
	public static void initialize(final String key, final boolean initState) {
		if (previousStates.get(key) == null) {
			previousStates.put(key, initState);
		}if (toggleStates.get(key) == null) {
			toggleStates.put(key, false);
		}if (toggledTimes.get(key) == null) {
			toggledTimes.put(key, System.currentTimeMillis());
		}
	}
	
	/**
	 * Runs {@link #initialize(key, initState)} with initState being false
	 * @param key a name like "Button A"
	 */
	private static void initialize(final String key) {initialize(key, false);}
	
	/**
	 * A traditional toggle. Update currentState as often as possible.
	 * @param key a name like "Button A"
	 * @param currentState the current state
	 * @return a boolean alternating between true and false each time currentState becomes true
	 */
	public static boolean freeze(final String key, final boolean currentState) {
		initialize(key);
		if (currentState && (currentState != previousStates.get(key))) {
			boolean toggleBool = !toggleStates.get(key);
			toggleStates.replace(key, toggleBool);
			toggledTimes.replace(key, System.currentTimeMillis());
		}
		previousStates.replace(key, currentState);
		return toggleStates.get(key);
	}
	
	/**
	 * Basically returns true if <code>((current != previous) && current)</code>.<br>
	 * Update currentState as often as possible.
	 * @param key a name like "Button A"
	 * @param currentState the current state
	 * @return true only when currentState has just become true
	 */
	public static boolean becomesTrue(final String key, final boolean currentState) {
		initialize(key);
		if (currentState && (currentState != previousStates.get(key))) {
			boolean toggleBool = !toggleStates.get(key);
			toggleStates.replace(key, toggleBool);
			toggledTimes.replace(key, System.currentTimeMillis());
			previousStates.replace(key, currentState);
			return true;
		}else {
			previousStates.replace(key, currentState);
			return false;
		}
	}
	
	/**
	 * Records the time whenever <code>((current != previous) && current)</code> is true.<br>
	 * Returns true if <code>(currentTime - recordedTime <= timeoutMS)</code>.<br>
	 * Update currentState as often as possible.
	 * @param key a name like "Button A"
	 * @param currentState the current state
	 * @param timeoutMS how long (in milliseconds) to return true after a button press
	 * @return true for <code>timeoutMS</code> after currentState becomes true
	 */
	public static boolean chill(final String key, final boolean currentState, final double timeoutMS) {
		freeze(key, currentState);
		return System.currentTimeMillis() - toggledTimes.get(key) <= timeoutMS;
	}
	
	/**
	 * Identifies which key corresponds to the state that became true most recently.<br>
	 * freeze() must be called in order to update states right before this function runs.
	 * @param keys an array of names that have been used in other functions of this class
	 * @param onlyLookAtFrozen specifies whether to ignore keys with a false result from freeze(). returns null if all are false
	 * @return the key with the state that became true most recently
	 */
	public static String youngest(final String[] keys, final boolean onlyLookAtFrozen) {
		String youngestKey = null;
		for (String key : keys) {
			Long time = toggledTimes.get(key);
			if ((!onlyLookAtFrozen || toggleStates.get(key)) && time.compareTo(toggledTimes.get(youngestKey)) > 0) youngestKey = key;
		}return youngestKey;
	}
}