package com.cyborgcats.reusable.Autonomous;

import java.util.Map;

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;

public class Events {
	private final Command[] commands;//list of actions
	private final double[] triggers;//list of values where independentVariable triggers an action
	
	private int step = -1;
	private boolean doneRunning = false;
	
	/**
	 * @param	commands	a list of lambda functions that update the state of the drivetrain and other subsystems
	 * @param	triggers	a list of percentages that indicate when/where to execute the commands
	 * @throws	IllegalStateException if <code>commands</code> and <code>triggers</code> have different lengths
	 * @see Command
	 */
	public Events(final Command[] commands, final double[] triggers) {
		this.commands = commands;
		this.triggers = triggers;
		if (commands.length != triggers.length) throw new IllegalStateException("Each command must have a corresponding trigger.");
		if (commands.length == 0) doneRunning = true;
	}
	
	/**
	 * Resets private instance variables <code>step</code> and <code>doneRunning</code> to their initial values.<br>
	 * This should be called to start over, for example, at the beginning of autonomous.
	 */
	public void reinit() {
		step = -1;
		doneRunning = commands.length == 0 ? true : false;
	}
	
	/**
	 * If <code>!doneRunning</code>, increments <code>step</code> when <code>independentVariable</code> has reached a trigger<br>
	 * If <code>step</code> is the last index in <code>commands</code> and <code>triggers</code>, <code>doneRunning</code> will be set to true
	 * @param independentVariable should increase as autonomous progresses
	 */
	public void check(final double independentVariable) {
		//if done running, don't bother checking [counter + 1] because it will be out of bounds
		if (!doneRunning && independentVariable >= triggers[step + 1]) {
			step++;//move on when the independent variable is greater than the trigger value
			if (step + 2 > triggers.length) doneRunning = true;
		}
	}
	
	/**
	 * Runs the lambda function in <code>commands[step]</code>
	 * @param drivetrain a {@link Drivetrain} implementation that can be used in the lambda function
	 * @param subsystems a list of {@link Subsystem} implementations that can be used in the lambda function
	 */
	public void execute(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems) {//TODO have another variable that determines whether the command is executed once, or over and over until the next trigger is reached
		if (step > -1) commands[step].execute(drivetrain, subsystems);
	}
	
	public static interface Command {void execute(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems);}
}
