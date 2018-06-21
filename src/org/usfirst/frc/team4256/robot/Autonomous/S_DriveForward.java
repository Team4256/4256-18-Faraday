package org.usfirst.frc.team4256.robot.Autonomous;//COMPLETE MARCH

import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Clamp;
import org.usfirst.frc.team4256.robot.D_Swerve;

public class S_DriveForward extends Strategy2018 {
	private Long start = null;
	
	public S_DriveForward(final int startingPosition, final String gameData) {
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
	
	@Override
	public void use(final Drivetrain drivetrain, final Subsystem[] subsystems) {
		ensureTimerHasStarted();
		if (System.currentTimeMillis() - start < 2000) {
			swerve.holonomic_encoderIgnorant(0.0, 0.0, 0.0);
			clamp.close();
			elevator.setInches(3.0);
		}else if (System.currentTimeMillis() - start < 5000) {
			swerve.holonomic_encoderIgnorant(0.0, 0.5, 0.0);
			elevator.setInches(ElevatorPresets.SWITCH.height());
		}else {
			swerve.holonomic_encoderIgnorant(0.0, 0.0, 0.0);
			if ((startingPosition.equals(StartingPosition.LEFT) && switchTarget.equals(FieldPieceConfig.LEFT)) ||
				(startingPosition.equals(StartingPosition.RIGHT) && switchTarget.equals(FieldPieceConfig.RIGHT))) {
				clamp.spit(0.5);
			}
		}
	}
	
	private void ensureTimerHasStarted() {if (start == null) start = System.currentTimeMillis();}
}
