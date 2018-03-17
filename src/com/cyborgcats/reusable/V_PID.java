package com.cyborgcats.reusable;//COMPLETE 2017

import java.util.HashMap;
import java.util.Map;

public class V_PID {
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
	 * This function wipes the i and d errors without messing up p, i, and d values.
	 * Useful when beginning a new turn to a new angle.
	**/
	public static void clear(final String key) {
		if (PIDSystems.get(key) == null) {
			PIDSystems.put(key, new double[] {0, 0, 0, 0, 0});
		}else {
			double p = PIDSystems.get(key)[0], i = PIDSystems.get(key)[1], d = PIDSystems.get(key)[2];
			PIDSystems.replace(key, new double[] {p, i, d, 0, 0});
		}
	}
	/**
	 * This function calculates and returns the result of PID for the specified key with the specified error.
	**/
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
	 * This function calculates and returns the result of PID for the specified key with the specified error.
	 * Also sets new p, i, and d values on the fly.
	**/
	public static double get(final String key, final double error, final double P, final double I, final double D) {
		set(key, P, I, D);
		return get(key, error);
	}
}
