package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Events;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.Strategy;

import org.usfirst.frc.team4256.robot.Elevator.Elevator.Abilities;

import com.cyborgcats.reusable.Autonomous.Events.Command;

public abstract class Strategy2018 extends Strategy {
	public static final double Yi = 2.82;//initial y value
	public static final double switchX = 4.25, cubeX = 5.42, scaleX = 6.18;
	public static final double switchY = 10.0, cubeY = 17.5, scaleY = 23.0;
	
	protected final FieldPieceConfig switchTarget, scaleTarget;
	protected final StartingPosition posI;
	
	protected Strategy2018(final StartingPosition posI, final char[] gameData, final Odometer odometer) {
		super(odometer);
		this.posI = posI;
		odometer.setOrigin(odometer.getX(false) - posI.x, odometer.getY(false) - Yi);
		
		if (gameData.length != 3) throw new IllegalStateException("Strategies only work with valid game data.");
		switchTarget = gameData[0] == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData[1] == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
	}
	
	
	@Override
	protected Leash getLeash() {
		switch (posI) {
		case LEFT: return leftLeash();
		case CENTER: return centerLeash();
		case RIGHT: return rightLeash();
		default: return centerLeash();
		}
	}
	@Override
	protected Events getEvents() {
		switch (posI) {
		case LEFT: return leftEvents();
		case CENTER: return centerEvents();
		case RIGHT: return rightEvents();
		default: return centerEvents();
		}
	}
	
	protected Leash leftLeash() {return super.getLeash();}
	protected Leash centerLeash() {return super.getLeash();}
	protected Leash rightLeash() {return super.getLeash();}
	protected Events leftEvents() {return super.getEvents();}
	protected Events centerEvents() {return super.getEvents();}
	protected Events rightEvents() {return super.getEvents();}
	
	
	public static enum StartingPosition {
		LEFT(-11.38),
		CENTER(0.184),
		RIGHT(8.16);
		
		private final double x;
		
		StartingPosition(final double initialX) {this.x = initialX;}
		
		protected double x() {return x;}
	}
	public static enum FieldPieceConfig {LEFT, RIGHT}

	
	
	/**
	 * @param instructions a 2D array of Strings with any number of rows and 3 columns<br>
	 * first column: clamp action ({@linkplain org.usfirst.frc.team4256.robot.Clamp.Abilities Abilities})<br>
	 * second column: elevator height (inches)<br>
	 * third column: robot orientation (degrees)<br>
	 * fourth column: maximum spin speed (percent)<br>
	 * fifth column: whether to pause code until actual orientation matches desired orientation (<code>"wait"</code> or <code>"pass"</code>)
	 * 
	 * @return an array of executable commands
	 * @see Command
	*/
	public static Command[] getFromArray(final String[][] instructions) {
		Command[] commands = new Command[instructions.length];
		
		for (int i = 0; i < instructions.length; i++) {
			final String[] instruction = instructions[i];
			
			commands[i] = (drive, sys) -> {
				final String clampAction = instruction[0];
				final double elevatorHeight = Double.parseDouble(instruction[1]),
							 desiredAngle = Double.parseDouble(instruction[2]),
							 maxSpin = Double.parseDouble(instruction[3])/100.0;
				final boolean wait = instruction[4] == "wait";
				
				if (wait) {
					drive.setSpeed(0.0);
					while (Math.abs(drive.face((double)desiredAngle, maxSpin)) > 5.0) drive.completeLoopUpdate();
				}else drive.face((double)desiredAngle, maxSpin);
				
				sys.get("Clamp").perform(clampAction, null);
				sys.get("Elevator").perform(Abilities.SET.name(), new double[] {elevatorHeight});
			};
		}
		
		return commands;
	}
}
