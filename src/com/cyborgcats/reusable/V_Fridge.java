package com.cyborgcats.reusable;//COMPLETE 2017

import java.util.HashMap;
import java.util.Map;

public class V_Fridge {//fridges and freezers control the state of matter (solid, liquid) as this class helps with boolean states
	private static final Map<String, Boolean> previousStates = new HashMap<String, Boolean>();
	public static final Map<String, Boolean> toggleStates = new HashMap<String, Boolean>();
	public static final Map<String, Long> toggledTimes = new HashMap<String, Long>();
	/**
	 * Run this to ensure that the specified key is available in all the class' HashMaps.
	**/
	public static void initialize(final String key, final boolean initState) {
		if (previousStates.get(key) == null) {
			previousStates.put(key, initState);
		}if (toggleStates.get(key) == null) {
			toggleStates.put(key, false);
		}if (toggledTimes.get(key) == null) {
			toggledTimes.put(key, System.currentTimeMillis());
		}
	}
	private static void initialize(final String key) {
		initialize(key, false);
	}
	/**
	 * This function's return value alternates between true and false each time currentState becomes true.
	 * Update currentState as often as possible.
	**/
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
	 * This function returns true only once when currentState becomes true.
	 * Update currentState as often as possible.
	**/
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
	 * This function returns true for a given number of milliseconds after the currentState becomes true.
	 * Update currentState as often as possible.
	**/
	public static boolean chill(final String key, final boolean currentState, final double timeoutMS) {
		freeze(key, currentState);
		return System.currentTimeMillis() - toggledTimes.get(key) <= timeoutMS;
	}
	
	/**
	 * Use this function to figure out which state corresponding to the given array of keys became true most recently.
	 * onlyLookAtFrozen specifies whether to ignore keys with a false result from freeze(). returns null if all are false
	 * freeze() must be called in order to update states right before this function runs.
	**/
	public static String youngest(final String[] keys, final boolean onlyLookAtFrozen) {
		String youngestKey = null;
		for (String key : keys) {
			Long time = toggledTimes.get(key);
			if ((!onlyLookAtFrozen || toggleStates.get(key)) && time.compareTo(toggledTimes.get(youngestKey)) > 0) {
				youngestKey = key;
			}
		}return youngestKey;
	}
}