package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Strategy;
import com.cyborgcats.reusable.Autonomous.Events.Command;

public abstract class Strategy2018 extends Strategy {
	public static final double switchX = 4.25, cubeX = 5.42, scaleX = 6.18;
	public static final double switchY = 10.0, cubeY = 17.5, scaleY = 23.0;
	
	protected FieldPieceConfig switchTarget, scaleTarget;
	protected StartingPosition startingPosition;
	
	@Override
	public double initialY() {return 2.82;}//feet
	@Override
	public double initialX() {
		switch(startingPosition) {
		case LEFT: return -11.38;
		case CENTER: return 0.184;
		case RIGHT: return 8.16;
		default: return 0.184;
		}
	}
	
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceConfig {LEFT, RIGHT};

	
	
	/*
	 * Instructions is a 2D array of ints. It can have any number of rows, and should have 3 columns.
	 * The first column determines clamp state (0 is slurp, 1 is spit, 2 is open).
	 * The second column determines elevator height (in inches).
	 * The third column represents the desired robot orientation (angle in degrees).
	 * The fourth column determines the maximum spin speed (as a percent).
	 * The fifth column determines pause state (1 is wait for orient, 0 is continue regardless).
	 * 
	 * An array of executable commands is returned.
	*/
	public static Command[] getFromArray(final String[][] instructions) {
		Command[] commands = new Command[instructions.length];
		
		for (int i = 0; i < instructions.length; i++) {
			final String[] instruction = instructions[i];
			
			commands[i] = (drive, sys) -> {
				final String clampAction = instruction[0];
				final double elevatorHeight = Double.parseDouble(instruction[1]),
							 desiredAngle = Double.parseDouble(instruction[2]),
							 maxSpin = Double.parseDouble(instruction[3]);
				final boolean wait = instruction[4] == "wait";
				
				if (wait) {
					drive.setSpeed(0.0);
					while (Math.abs(drive.face((double)desiredAngle, maxSpin)) > 5.0) drive.completeLoopUpdate();
				}else drive.face((double)desiredAngle, maxSpin);
				
				sys.get("Clamp").perform(clampAction, null);
				sys.get("Elevator").perform("SET", new double[] {elevatorHeight});
			};
		}
		
		return commands;
	}
}
