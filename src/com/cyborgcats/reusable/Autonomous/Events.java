package com.cyborgcats.reusable.Autonomous;

import java.util.Map;

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;

public class Events {
	private final Command[] commands;//list of actions
	private final double[] triggers;//list of values where independentVariable triggers an action
	
	private int step = -1;
	private boolean doneRunning = false;
	
	public Events(final Command[] commands, final double[] triggers) {
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
	
	public void execute(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems) {
		if (step > -1) commands[step].execute(drivetrain, subsystems);
	}
	
	public interface Command {void execute(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems);}
}
