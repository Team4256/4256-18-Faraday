package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_DriveTrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

public class V_Events {
	private final Command[] events;
	private final double[] at;//list of values where independentVariable triggers an action
	
	public V_Events(final Command[] events, final double[] at) {
		this.events = events;
		this.at = at;
	}
	
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
	
	public interface Command {
		void execute(final R_DriveTrain swerve, final R_Clamp clamp, final R_Combined elevator);
	}
}
