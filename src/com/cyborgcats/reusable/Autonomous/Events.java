package com.cyborgcats.reusable.Autonomous;

import java.util.Map;

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;

/**
 * Allows discrete actions to be performed at designated points along an autonomous path<br>
 * Unlike a {@link Leash} object, which updates the drivetrain every loop, an Events object only runs <code>commands</code> when <code>triggers</code> are reached.
 */
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
	 * Should be called to start over, for example, at the beginning of autonomous.
	 */
	public void reinit() {
		step = -1;
		doneRunning = commands.length == 0 ? true : false;
	}
	
	/**
	 * If <code>!doneRunning</code>, increments <code>step</code> when <code>independentVariable</code> has reached a trigger<br>
	 * If <code>step</code> is the last index in <code>commands</code> and <code>triggers</code>, <code>doneRunning</code> will be set to true<br><br>
	 * Should be called every loop
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
	
	/**
	 * As a single method interface, Command defines a signature against which lambda functions can be matched.<br><br>
	 * When creating a list of Commands to be used in {@link Events}, many of them start to look very similar.
	 * To save space, states and parameters can be placed into one big array and passed into a helper function that
	 * converts them into the corresponding lambda expressions.<br><br>
	 * 
	 * Usage:<br>
	 * <code>Command command = (drive, sys) -> {
	 * <pre>drive.foo();</pre>
	 * <pre>sys.get("baz").perform(bar, xyzzy);</pre>
	 * };</code>
	 */
	public static interface Command {
		/**
		 * This should update the desired state of one or more subsystems and possibly use <code>drivetrain</code> to set the robot's orientation.<br>
		 * Unless the drivetrain is swerve AND the current {@link Odometer} can track through a change in orientation, a while loop should be used to pause other code during the change.
		 * @param drivetrain a {@link Drivetrain} implementation
		 * @param subsystems a list of {@link Subsystem} implementations
		 */
		void execute(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems);
	}
}
