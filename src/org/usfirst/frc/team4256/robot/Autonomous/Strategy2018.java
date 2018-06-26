package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Events;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.Strategy;

import org.usfirst.frc.team4256.robot.Elevator.Elevator.Abilities;

import com.cyborgcats.reusable.Autonomous.Events.Command;

public abstract class Strategy2018 extends Strategy {
	public static final double leftXi = -11.38, centerXi = 0.184, rightXi = 8.16, Yi = 2.82;//initial values
	public static final double switchX = 4.25, cubeX = 5.42, scaleX = 6.18;
	public static final double switchY = 10.0, cubeY = 17.5, scaleY = 23.0;
	
	protected final FieldPieceConfig switchTarget, scaleTarget;
	protected final StartingPosition startingPosition;
	
	protected Strategy2018(final int startingPosition, final String gameData, final Odometer odometer) {
		super(odometer);
		if (gameData.length() != 3) throw new IllegalStateException("Strategies only work with valid game data.");
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {
		case(0):this.startingPosition = StartingPosition.LEFT;  break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT; break;
		default:this.startingPosition = StartingPosition.CENTER;break;
		}
	}
	
	
	@Override
	protected Leash getLeash() {
		switch (startingPosition) {
		case LEFT: return leftLeash();
		case CENTER: return centerLeash();
		case RIGHT: return rightLeash();
		default: return centerLeash();
		}
	}
	@Override
	protected Events getEvents() {
		switch (startingPosition) {
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
	
	
	public static enum StartingPosition {LEFT, CENTER, RIGHT};
	public static enum FieldPieceConfig {LEFT, RIGHT};

	
	
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
