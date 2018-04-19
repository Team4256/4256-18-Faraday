package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_Drivetrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

public interface Autonomous {
	public void run(final R_Drivetrain swerve, final R_Clamp clamp, final R_Combined elevator);
	public double initOdometerPosX();
	
	public static final double leftStartX = -127.54, centerStartX = 11.21, rightStartX = 106.94;
	public static final double switchX = 53.0, cubeX = 65.0, scaleX = 74.11;
	public static final double startY = 28.75, switchY = 120.0, cubeY = 210.0, scaleY = 276.0;
	//switchY = 166.6
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceConfig {LEFT, RIGHT};
}
