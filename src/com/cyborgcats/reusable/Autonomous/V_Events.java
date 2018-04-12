package com.cyborgcats.reusable.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

import com.cyborgcats.reusable.R_Gyro;

public class V_Events {
	private final Command[] commands;//list of actions
	private final double[] triggers;//list of values where independentVariable triggers an action
	
	private int step = -1;
	private boolean doneRunning = false;
	
	public V_Events(final Command[] commands, final double[] triggers) {
		this.commands = commands;
		this.triggers = triggers;
		if (commands.length != triggers.length) throw new IllegalStateException("Each autonomous event must have a corresponding trigger point.");
	}
	
	public void reinit() {
		step = -1;
		doneRunning = false;
	}
	
	public void check(final double independentVariable) {
		//if done running, don't bother checking [counter + 1] because it will be out of bounds
		if (!doneRunning && independentVariable >= triggers[step + 1]) {
			//move on when the independent variable is greater than the trigger value
			step++;
			if (step + 2 > triggers.length) doneRunning = true;
		}
	}
	
	public double execute(final R_Clamp clamp, final R_Combined elevator, final R_Gyro gyro) {
		if (step > -1) return commands[step].execute(clamp, elevator, gyro);//calls methods on clamp and elevator, then returns spin for swerve
		else return 0.0;
	}
	
	public interface Command {
		//should call methods on clamp and elevator, then return spin for swerve
		double execute(final R_Clamp clamp, final R_Combined elevator, final R_Gyro gyro);
	}
	
	
	
	/*
	 * Instructions is a 2D array of ints. It can have any number of rows, and should have 3 columns.
	 * The first column determines clamp state (0 is slurp, 1 is spit, 2 is open).
	 * The second column determines elevator height (in inches).
	 * The third column represents the desired robot orientation (angle in degrees).
	 * The fourth column determines the maximum spin speed (as a percent).
	 * 
	 * An array of executable commands is returned.
	*/
	public static Command[] getFromArray(final int[][] instructions) {
		Command[] commands = new Command[instructions.length];
		
		for (int i = 0; i < instructions.length; i++) {
			final int[] instruction = instructions[i];
			
			commands[i] = (c, e, g) -> {//c is for clamp, e is for elevator, g is for gyro
				final int clampState = instruction[0],
						  elevatorHeight = instruction[1],
						  desiredAngle = instruction[2];
			
				switch(clampState) {
				case 0: c.slurp();break;
				case 1: c.spit(0.5);break;
				case 2: c.open();break;
				case 3: c.rotateTo(0.0);break;
				case 4: c.close();break;
				default: break;
				}
			
				e.setInches((double)elevatorHeight);
			
				final double error = g.wornPath((double)desiredAngle);
				return Math.abs(error) > 5.0 ? Math.signum(error)*(double)instruction[3]/100.0 : 0.0;
			};
		}
		
		return commands;
	}
}
