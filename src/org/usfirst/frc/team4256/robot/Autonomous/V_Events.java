package org.usfirst.frc.team4256.robot.Autonomous;

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
			if (step + 2 >= triggers.length) doneRunning = true;
		}
	}
	
	public double execute(final R_Clamp clamp, final R_Combined elevator, final R_Gyro gyro) {
		if (step > -1) return commands[step].execute(clamp, elevator, gyro);//calls methods on clamp and elevator, then returns spin for swerve
		else return 0.0;
	}
	
	public interface Command {
		double execute(final R_Clamp clamp, final R_Combined elevator, final R_Gyro gyro);//should call methods on clamp and elevator, then return spin for swerve
	}
}
