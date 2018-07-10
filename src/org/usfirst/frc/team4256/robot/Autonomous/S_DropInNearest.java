package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Clamp.Abilities;
import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;

import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.Path;
import com.cyborgcats.reusable.Autonomous.Events;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;

public final class S_DropInNearest extends Strategy2018 {
	public S_DropInNearest(final StartingPosition posI, final char[] gameData, final Odometer odometer) {super(posI, gameData, odometer);}

	@Override
	protected Leash leftLeash() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final Path a = new P_Bezier(posI.x(), Yi, -10, 17.92, -7.17, 20.17, -scaleX, scaleY, 0.0);//get to easy scale

			final Path[] path = new Path[] {a};
			return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final Path a = new P_Bezier(posI.x(), Yi, -9.17, 7.75, -7.75, 7.75, -switchX, switchY, 0.0);//get to easy switch
				
				final Path[] path = new Path[] {a};
				return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final Path a = new P_Bezier(posI.x(), Yi, posI.x(), 7.75, posI.x(), 7.75, posI.x(), switchY, 0.0);//drive forward
				return new Leash(new Path[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}
	@Override
	protected Events leftEvents() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final String[][] instructions = new String[][] {
				{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "3"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"},
				{Abilities.SPIT.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "0"/*percent*/, "pass"}
			};
			
			return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final String[][] instructions = new String[][] {
					{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "90"/*degrees*/, "100"/*percent*/, "wait"}
				};
				
				return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final String[][] instructions = new String[][] {{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"}};
				return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1});
			}
		}
	}


	@Override
	protected Leash centerLeash() {
		Path bezier;

		if (switchTarget.equals(FieldPieceConfig.LEFT)) bezier = new P_Bezier(posI.x(), Yi, -2.5, 6.83, -4.33, 5, -switchX, switchY, 0.0);//get to left switch
		else bezier = new P_Bezier(posI.x(), Yi, 2.5, 6.83, 4.33, 5, switchX, switchY, 0.0);//get to right switch
		
		return new Leash(new Path[] {bezier}, /*leash length*/1.5, /*growth rate*/0.1);
	}
	@Override
	protected Events centerEvents() {
		// at 1.0, reaches switch
		final String[][] instructions = new String[][] {
			{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
			{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "3"/*percent*/, "pass"},
			{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"}
		};
		
		return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.2, 1.0});
	}


	@Override
	protected Leash rightLeash() {
		if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			final Path a = new P_Bezier(posI.x(), Yi, 10, 17.92, 7.17, 20.17, scaleX, scaleY, 0.0);//get to easy scale

			final Path[] path = new Path[] {a};
			return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				final Path a = new P_Bezier(posI.x(), Yi, 9.67, 7.42, 8.25, 7.42, switchX, switchY, 0.0);//get to easy switch

				final Path[] path = new Path[] {a};
				return new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final Path a = new P_Bezier(posI.x(), Yi, posI.x(), 7.42, posI.x(), 7.42, posI.x(), switchY, 0.0);//drive forward
				return new Leash(new Path[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}
	@Override
	protected Events rightEvents() {
		if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			final String[][] instructions = new String[][] {
				{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "3"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"},
				{Abilities.SPIT.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "0"/*percent*/, "pass"}
			};
			
			return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				final String[][] instructions = new String[][] {
					{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "270"/*degrees*/, "100"/*percent*/, "wait"}
				};
				
				return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final String[][] instructions = new String[][] {{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"}};
				return new Events(Strategy2018.getFromArray(instructions), new double[] {0.1});
			}
		}
	}
}
