package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.P_Curve;
import com.cyborgcats.reusable.Autonomous.P_Curve.Function;
import com.cyborgcats.reusable.Autonomous.Path;

public final class S_Mickey extends Strategy2018 {
	private static final double faceRadius = 3.0;
	private static final double earRadius = 1.5;
	private static final double earOffsetX = 2.5;
	private static final double earOffsetY = 3.5;
	private static final double earAngle = 10.0;
	
	public S_Mickey(final StartingPosition posI, final char[] gameData, final Odometer odometer) {super(posI, gameData, odometer);}
	public S_Mickey(final Odometer odometer) {super(StartingPosition.CENTER, new char[] {'R', 'R', 'R'}, odometer);}
	
	@Override
	protected Leash getLeash() {
		final Function faceX = (t) -> faceRadius*Math.cos(t) + posI.x();
		final Function faceY = (t) -> faceRadius*Math.sin(t) + Yi;
		final Function earRightX = (t) -> earRadius*Math.cos(t) + posI.x() + earOffsetX;
		final Function earRightY = (t) -> earRadius*Math.sin(t) + Yi + earOffsetY;
		final Function earLeftX = (t) -> earRadius*Math.cos(t) + posI.x() - earOffsetX;
		final Function earLeftY = (t) -> earRadius*Math.sin(t) + Yi + earOffsetY;
		
		final Path faceA = new P_Curve(faceX, faceY, -0.5*Math.PI, Math.atan(earOffsetY/earOffsetX) - earAngle);
		final Path rightEar = new P_Curve(earRightX, earRightY, Math.atan(earOffsetY/earOffsetX) - earAngle, Math.atan(earOffsetY/earOffsetX) + earAngle);
		final Path faceB = new P_Curve(faceX, faceY, Math.atan(earOffsetY/earOffsetX) + earAngle, Math.PI - Math.atan(earOffsetY/earOffsetX) - earAngle);
		final Path leftEar = new P_Curve(earLeftX, earLeftY, Math.PI - Math.atan(earOffsetY/earOffsetX) - earAngle, Math.PI - Math.atan(earOffsetY/earOffsetX) + earAngle);
		final Path faceC = new P_Curve(faceX, faceY, Math.PI - Math.atan(earOffsetY/earOffsetX) + earAngle, 1.5*Math.PI);
		
		
		
		
		final Path[] path = new Path[] {faceA, rightEar, faceB, leftEar, faceC};
		return new Leash(path, /*leash length*/0.75, /*growth rate*/0.03);
	}
}
