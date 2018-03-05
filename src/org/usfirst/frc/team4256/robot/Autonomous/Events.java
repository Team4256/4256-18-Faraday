package org.usfirst.frc.team4256.robot.Autonomous;

public class Events {
	public static final double[] oneScaleAuto = new double[] {0.0, 1.0, 5.4};//list of values where independentVariable triggers an action
	
	public static int counter = -1;
	public static boolean doneRunning = false;
	
	public static void init() {
		counter = -1;
		doneRunning = false;
	}
	
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
