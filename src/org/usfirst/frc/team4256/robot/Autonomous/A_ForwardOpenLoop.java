package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Elevators.R_Combined;
import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_DriveTrain;

public class A_ForwardOpenLoop implements Autonomous{
	public final FieldPieceConfig switchTarget;
	public final FieldPieceConfig scaleTarget;
	public final StartingPosition startingPosition;
	
	private Long start = null;
	public double initOdometerPosX = 0.0;
	
	public A_ForwardOpenLoop(final int startingPosition, final String gameData) {
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
		
	public void run(final R_DriveTrain swerve, final R_Clamp clamp, final R_Combined elevator) {
		startTimer();//if necessary
		if (System.currentTimeMillis() - start < 2000) {
			swerve.holonomic_encoderAware(0.0, 0.0, 0.0);
			clamp.close();
			elevator.setInches(3.0);
		}else if (System.currentTimeMillis() - start < 5000) {
			swerve.holonomic_encoderAware(0.0, 0.5, 0.0);
			elevator.setInches(ElevatorPresets.SWITCH.height());
		}else {
			swerve.holonomic_encoderAware(0.0, 0.0, 0.0);
			if ((startingPosition.equals(StartingPosition.LEFT) && switchTarget.equals(FieldPieceConfig.LEFT)) ||
				(startingPosition.equals(StartingPosition.RIGHT) && switchTarget.equals(FieldPieceConfig.RIGHT))) {
				clamp.spit(R_Clamp.intakeConstant);
			}
		}
	}
	
	public double initOdometerPosX() {return initOdometerPosX;}
	
	private void startTimer() {if (start == null) start = System.currentTimeMillis();}
}
