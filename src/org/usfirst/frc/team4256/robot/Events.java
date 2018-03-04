package org.usfirst.frc.team4256.robot;

public class Events {
	public static final double[] oneScaleAuto = new double[] {0.0, 3.0*Math.PI/4.0, Math.PI};//list of values where independentVariable triggers an action
	
	public static int counter = -1;
	public static boolean doneRunning = false;
	
	public static void init() {counter = -1;}
	
	public static void check(final double independentVariable) {
		if (!doneRunning && independentVariable >= oneScaleAuto[counter + 1]) {
			if (counter + 2 < oneScaleAuto.length) counter++;
			else {
				counter++;
				doneRunning = true;
			}
		}
	}
}
