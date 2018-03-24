package org.usfirst.frc.team4256.robot;//COMPLETE 2017

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.V_PID;

public abstract class V_Instructions {
	private static int previousStep = -1;//ensures it always starts fresh
	private static Long stepStart = System.currentTimeMillis();
	private static double[] currentInstructions;
	private static boolean canMoveOn = true;
	private static Long autoTimer = System.currentTimeMillis();
	private static boolean startedTimer;
	/**
	 * This function requires a multi-dimensional array of instructions containing a duration, direction, speed, and orientation.
	 * The array is saved inside the V_Instructions class for use by its other members.
	 * It then runs through the specified step in the array, commanding swerve to move accordingly.
	**/
	public static void follow(final double[][] instructions, final int autoStep, final R_Drivetrain swerve, final R_Gyro gyro) {
		if (autoStep != previousStep) {
			stepStart = System.currentTimeMillis();
			if (canMoveOn) {
				currentInstructions = instructions[autoStep];
			}canMoveOn = autoStep + 1 < instructions.length;
			previousStep = autoStep;
			V_PID.clear("spin");
		}
		if (System.currentTimeMillis() - stepStart < currentInstructions[0]) {
			double spinError = gyro.wornPath(currentInstructions[3]);
			if (Math.abs(spinError) < 3) {spinError = 0;}
			swerve.holonomic_encoderAware(currentInstructions[1], currentInstructions[2], V_PID.get("spin", spinError));
		}
	}
	/**
	 * This function returns true when the current step has run to completion.
	**/
	public static boolean readyToMoveOn() {
		return System.currentTimeMillis() - stepStart >= currentInstructions[0];
	}
	/**
	 * This function indicates whether there is another step to follow.
	**/
	public static boolean canMoveOn() {
		return canMoveOn;
	}
	
	public static void startTimer() {
		autoTimer = System.currentTimeMillis();
		startedTimer = true;
	}
	
	public static void resetTimer() {
		startedTimer = false;
	}
	
	public static boolean startedTimer() {
		return startedTimer;
	}
	
	public static double getSeconds() {
		return (double)(System.currentTimeMillis() - autoTimer)/1000;
	}
}