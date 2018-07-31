package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Clamp.Abilities;

import com.cyborgcats.reusable.Autonomous.Events;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.P_Curve;
import com.cyborgcats.reusable.Autonomous.P_Curve.Function;
import com.cyborgcats.reusable.Autonomous.Path;

public final class S_Mickey extends Strategy2018 {
	private static final double faceRadius = 6.0;
	private static final double faceEarRatio = 3.0;
	
	private static final double offset = (faceRadius + faceRadius/faceEarRatio)/Math.sqrt(2.0);
	
	public S_Mickey(final StartingPosition posI, final char[] gameData, final Odometer odometer) {super(posI, gameData, odometer);}
	public S_Mickey(final Odometer odometer) {super(StartingPosition.CENTER, new char[] {'R', 'R', 'R'}, odometer);}
	
	@Override
	protected Leash getLeash() {
		final Function faceX = (t) -> faceRadius*Math.cos(t) + posI.x() - faceRadius;
		final Function faceY = (t) -> faceRadius*Math.sin(t) + Yi;
		
		final Function earRightX = (t) -> (faceRadius/faceEarRatio)*Math.cos(t) + offset + posI.x() - faceRadius;
		final Function earRightY = (t) -> (faceRadius/faceEarRatio)*Math.sin(t) + offset + Yi;
		
		final Function earLeftX = (t) -> (faceRadius/faceEarRatio)*Math.cos(t) - offset + posI.x() - faceRadius;
		final Function earLeftY = (t) -> (faceRadius/faceEarRatio)*Math.sin(t) + offset + Yi;
		
		final Path faceA = new P_Curve(faceX, faceY, 0, Math.PI/4.0);
		final Path rightEar = new P_Curve(earRightX, earRightY, (-3.0/4.0)*Math.PI, (5.0/4.0)*Math.PI);
		final Path faceB = new P_Curve(faceX, faceY, Math.PI/4.0, (3.0/4.0)*Math.PI);
		final Path leftEar = new P_Curve(earLeftX, earLeftY, -Math.PI/4.0, (7.0/4.0)*Math.PI);
		final Path faceC = new P_Curve(faceX, faceY, (3.0/4.0)*Math.PI, 2.0*Math.PI);
		
		
		
		
		final Path[] path = new Path[] {faceA, rightEar, faceB, leftEar, faceC};
		return new Leash(path, /*leash length*/0.75, /*growth rate*/0.03);
	}
	
	@Override
	protected Events getEvents() {
		final String[][] instructions = new String[][] {
			{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
			{Abilities.CLOSE.name(), "3"/*inches*/, "90"/*degrees*/, "50"/*percent*/, "wait"},
			{Abilities.CLOSE.name(), "3"/*inches*/, "90"/*degrees*/, "5"/*percent*/, "pass"}
		};
		
		return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.5, 0.6});		
	}
}
