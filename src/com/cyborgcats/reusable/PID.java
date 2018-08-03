package com.cyborgcats.reusable;//COMPLETE 2017

import java.util.HashMap;
import java.util.Map;

public final class PID {
	private PID() {}
	
	private static final Map<String, double[]> PIDSystems = new HashMap<String, double[]>();
	
	public static void set(final String key, final double P, final double I, final double D) {
		if (PIDSystems.get(key) == null) {
			PIDSystems.put(key, new double[] {P, I, D, 0, 0});
		}else {
			double previousKerr = PIDSystems.get(key)[3];
			double previousIerr = PIDSystems.get(key)[4];
			PIDSystems.replace(key, new double[] {P, I, D, previousKerr, previousIerr});
		}
	}
	
	/**
	 * Wipes the i and d errors without messing up p, i, and d values.
	 * @param key which PID to adjust
	 */
	public static void clear(final String key) {
		if (PIDSystems.get(key) == null) {
			PIDSystems.put(key, new double[] {0, 0, 0, 0, 0});
		}else {
			double p = PIDSystems.get(key)[0], i = PIDSystems.get(key)[1], d = PIDSystems.get(key)[2];
			PIDSystems.replace(key, new double[] {p, i, d, 0, 0});
		}
	}
	
	/**
	 * Calculates PID output for the specified key and error
	 * @param key which PID values to use
	 * @param error difference between target and actual (position, angle, speed, etc)
	 * @return PID output
	 */
	public static double get(final String key, final double error) {
		if (PIDSystems.get(key) == null) {
			return 0;
		}else {
			double[] tempArr = PIDSystems.get(key);
			double pOut = error*tempArr[0];
			double iErr = error + tempArr[4];
			double iOut = iErr*tempArr[1];
			double dErr = error - tempArr[3];
			double dOut = dErr*tempArr[2];
			PIDSystems.replace(key, new double[] {tempArr[0], tempArr[1], tempArr[2], error, iErr});
			return pOut + iOut + dOut;
		}
	}
	
	/**
	 * Sets P, I, and D values on the fly, then calculates PID output based on the error
	 * @param key which PID values to update and then use for calculations
	 * @param error difference between target and actual (position, angle, speed, etc)
	 * @param P new P value
	 * @param I new I value
	 * @param D new D value
	 * @return PID output
	 */
	public static double get(final String key, final double error, final double P, final double I, final double D) {
		set(key, P, I, D);
		return get(key, error);
	}
}
