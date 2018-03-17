package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_DriveTrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

public interface Autonomous {
	public void run(final R_DriveTrain swerve, final R_Clamp clamp, final R_Combined elevator);
	public double initOdometerPosX();
	
	public static final double leftStartX = -112.33, centerStartX = 7.19, rightStartX = 115.0;
	public static final double switchX = 101.0, cubeX = 65.0, scaleX = 83.11;
	public static final double startY = 13.32, switchY = 166.6, cubeY = 210.0, scaleY = 296.13;
	
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceConfig {LEFT, RIGHT};
}
