package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

import edu.wpi.first.networktables.NetworkTable;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_Drivetrain;

public class A_TwoSwitchOpenLoop implements Autonomous {
	public final FieldPieceConfig switchTarget, scaleTarget;
	public final StartingPosition startingPosition;
	private final NetworkTable visionTable;
	
	private Long start = null;
	public double initOdometerPosX = 0.0;
	
	public A_TwoSwitchOpenLoop(final int startingPosition, final String gameData, final NetworkTable visionTable) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;  break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT; break;
		default:this.startingPosition = StartingPosition.CENTER;break;
		}
		
		this.visionTable = visionTable;
	}
	
	public void run(final R_Drivetrain swerve, final R_Clamp clamp, final R_Combined elevator) {
		ensureTimerHasStarted();
		
		final double error = swerve.gyro.wornPath(0.0);
		final double spin = Math.abs(error) > 1.0 ? 0.07*Math.signum(error) : 0.0;
		
		final double direction = switchTarget.equals(FieldPieceConfig.RIGHT) ? 25.0 : -32.0;
		if (System.currentTimeMillis() - start < 1200) {//BEGIN DRIVING TO SWITCH
			swerve.holonomic_encoderIgnorant(direction, 0.6, spin);
			clamp.close();
			elevator.setInches(3.0);
		}else if (System.currentTimeMillis() - start < 2200) {//FINISH DRIVING TO SWITCH AND RAISE ELVATOR
			swerve.holonomic_encoderIgnorant(direction, 0.5, spin);
			elevator.setInches(ElevatorPresets.SWITCH.height());
		}else if (System.currentTimeMillis() - start < 2300) {
			swerve.holonomic_encoderIgnorant(direction, 0.0, 0.0);//STOP DRIVING AND SPIT
			clamp.spit(0.5);
		}else if (System.currentTimeMillis() - start < 2600) {//DRIVE BACK TOWARD MIDDLE
			swerve.holonomic_encoderIgnorant(180.0, 0.5, spin);
		}else {
			swerve.holonomic_encoderIgnorant(0.0, 0.0, 0.0);//STOP DRIVING AND SPIT
			clamp.spit(0.5);
		}
	}
	
	public double initOdometerPosX() {return initOdometerPosX;}
	
	private void ensureTimerHasStarted() {if (start == null) start = System.currentTimeMillis();}
}
