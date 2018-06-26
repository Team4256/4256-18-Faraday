package org.usfirst.frc.team4256.robot.Autonomous;

import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.P_Curve;
import com.cyborgcats.reusable.Autonomous.P_Curve.Function;
import com.cyborgcats.reusable.Autonomous.Path;

public class S_Slither extends Strategy2018 {
	
	public S_Slither(final int startingPosition, final String gameData, final Odometer odometer) {super(startingPosition, gameData, odometer);}
	public S_Slither(final Odometer odometer) {super(1, "RRR", odometer);}
	
	@Override
	protected Leash getLeash() {
		final Function x = (t) -> 2.0*Math.sin(t);
		final Function y = (t) -> t;
		
		final Path a = new P_Curve(x, y, 0.0, 2.0*Math.PI);
		final Path[] path = new Path[] {a};
		return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
}
