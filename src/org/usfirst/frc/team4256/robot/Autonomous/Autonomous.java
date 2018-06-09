package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_Drivetrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

public interface Autonomous {
	public void run(final R_Drivetrain swerve, final R_Clamp clamp, final R_Combined elevator);
	public double initX();
	//starting positions are measured from field origin to ZED's left camera
	public static final double leftStartX = -127.54 - 9.0, centerStartX = 11.21 - 9.0, rightStartX = 106.94 - 9.0;
	public static final double switchX = 51.0, cubeX = 65.0, scaleX = 74.11;
	public static final double initY = 28.75, switchY = 120.0, cubeY = 210.0, scaleY = 276.0;
	
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceConfig {LEFT, RIGHT};
}
