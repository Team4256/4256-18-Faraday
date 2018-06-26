package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.Path;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;

public class S_PassLine extends Strategy2018 {
	
	public S_PassLine(final int startingPosition, final String gameData, final Odometer odometer) {super(startingPosition, gameData, odometer);}
	public S_PassLine(final int startingPosition, final Odometer odometer) {super(startingPosition, "RRR", odometer);}
	
	@Override
	protected Leash leftLeash() {
		final Path a = new P_Bezier(leftXi, Yi, -9.17, 7.75, -7.75, 7.75, -switchX, switchY, 0.0);//get to nearest switch
		final Path[] path = new Path[] {a};
		return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	@Override
	protected Leash centerLeash() {
		final Path a = new P_Bezier(centerXi, Yi, 7, 11, 8.58, 7.08, switchX, switchY, 0.0);//get to right switch
		final Path[] path = new Path[] {a};
		return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	@Override
	protected Leash rightLeash() {
		final Path a = new P_Bezier(rightXi, Yi, 9.67, 7.42, 8.25, 7.42, switchX, switchY, 0.0);//get to nearest switch
		final Path[] path = new Path[] {a};
		return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
}
