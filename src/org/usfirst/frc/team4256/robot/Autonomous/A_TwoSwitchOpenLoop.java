package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Elevators.R_Combined;
import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_Drivetrain;

public class A_TwoSwitchOpenLoop implements Autonomous {
	public final FieldPieceConfig switchTarget;
	public final FieldPieceConfig scaleTarget;
	public final StartingPosition startingPosition;
	
	private Long start = null;
	public double initOdometerPosX = 0.0;
	
	public A_TwoSwitchOpenLoop(final int startingPosition, final String gameData) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;  break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT; break;
		default:this.startingPosition = StartingPosition.CENTER;break;
		}
	}
	
	public void run(final R_Drivetrain swerve, final R_Clamp clamp, final R_Combined elevator) {
		ensureTimerHasStarted();
		final double direction = switchTarget.equals(FieldPieceConfig.RIGHT) ? 15.0 : -15.0;
		if (System.currentTimeMillis() - start < 1500) {
			swerve.holonomic_encoderIgnorant(direction, 0.5, 0.0);
			clamp.close();
			elevator.setInches(3.0);
		}else if (System.currentTimeMillis() - start < 3000) {
			swerve.holonomic_encoderIgnorant(direction, 0.5, 0.0);
			elevator.setInches(ElevatorPresets.SWITCH.height());
		}else {
			swerve.holonomic_encoderIgnorant(0.0, 0.0, 0.0);
			clamp.spit(0.5);
		}
	}
	
	public double initOdometerPosX() {return initOdometerPosX;}
	
	private void ensureTimerHasStarted() {if (start == null) start = System.currentTimeMillis();}
}
