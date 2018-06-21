package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Clamp.Abilities;
import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;

import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.Events;
import com.cyborgcats.reusable.Autonomous.Leash;

import com.cyborgcats.reusable.Autonomous.Odometer;

public class S_DropInNearest extends Strategy2018 {

	public S_DropInNearest(final int startingPosition, final String gameData, final Odometer odometer) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;  break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT; break;
		default:this.startingPosition = StartingPosition.CENTER;break;
		}

		this.odometer = odometer;

		switch (this.startingPosition) {
		case LEFT:  useLeash_left();
					useEvents_left();  break;
		case CENTER:useLeash_center();
					useEvents_center();break;
		case RIGHT: useLeash_right();
					useEvents_right(); break;
		default:    useLeash_center();
					useEvents_center();break;
		}
	}


	//------------------------------------------------------------------------------------------
	private void useLeash_left() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final P_Bezier a = new P_Bezier(initialX(), initialY(), -120, 215, -86, 242, -scaleX, scaleY, 0.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a};
			leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final P_Bezier a = new P_Bezier(initialX(), initialY(), -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
				
				final P_Bezier[] path = new P_Bezier[] {a};
				leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final P_Bezier a = new P_Bezier(initialX(), initialY(), initialX(), 93, initialX(), 93, initialX(), switchY, 0.0);//drive forward
				leash = new Leash(new P_Bezier[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}
	
	private void useEvents_left() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final String[][] instructions = new String[][] {
				{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "3"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"},
				{Abilities.SPIT.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "0"/*percent*/, "pass"}
			};
			
			events = new Events(getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final String[][] instructions = new String[][] {
					{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "90"/*degrees*/, "100"/*percent*/, "wait"}
				};
				
				events = new Events(getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final String[][] instructions = new String[][] {{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"}};
				events = new Events(getFromArray(instructions), new double[] {0.1});
			}
		}
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private Leash useLeash_center() {
		P_Bezier bezier;

		if (switchTarget.equals(FieldPieceConfig.LEFT)) bezier = new P_Bezier(initialX(), initialY(), -30, 82, -52, 60, -switchX, switchY, 0.0);//get to left switch
		else bezier = new P_Bezier(initialX(), initialY(), 30, 82, 52, 60, switchX, switchY, 0.0);//get to right switch
		
		leash = new Leash(new P_Bezier[] {bezier}, /*leash length*/1.5, /*growth rate*/0.1);
		return leash;
	}

	private Events useEvents_center() {
		// at 1.0, reaches switch
		final String[][] instructions = new String[][] {
			{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
			{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "3"/*percent*/, "pass"},
			{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"}
		};
		
		events = new Events(getFromArray(instructions), new double[] {0.1, 0.2, 1.0});
		return events;
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private void useLeash_right() {
		if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			final P_Bezier a = new P_Bezier(initialX(), initialY(), 120, 215, 86, 242, scaleX, scaleY, 0.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a};
			leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				final P_Bezier a = new P_Bezier(initialX(), initialY(), 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch

				final P_Bezier[] path = new P_Bezier[] {a};
				leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final P_Bezier a = new P_Bezier(initialX(), initialY(), initialX(), 89, initialX(), 89, initialX(), switchY, 0.0);//drive forward
				leash = new Leash(new P_Bezier[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}

	private void useEvents_right() {
		if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			final String[][] instructions = new String[][] {
				{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "3"/*percent*/, "pass"},
				{Abilities.CLOSE.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "1"/*percent*/, "pass"},
				{Abilities.SPIT.name(), ElevatorPresets.SCALE_HIGH.heightString(), "0"/*degrees*/, "0"/*percent*/, "pass"}
			};
			
			events = new Events(getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				final String[][] instructions = new String[][] {
					{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.EXTEND.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"},
					{Abilities.SPIT.name(), ElevatorPresets.SWITCH.heightString(), "270"/*degrees*/, "100"/*percent*/, "wait"}
				};
				
				events = new Events(getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final String[][] instructions = new String[][] {{Abilities.CLOSE.name(), "3"/*inches*/, "0"/*degrees*/, "5"/*percent*/, "pass"}};
				events = new Events(getFromArray(instructions), new double[] {0.1});
			}
		}
	}
	//------------------------------------------------------------------------------------------
}
