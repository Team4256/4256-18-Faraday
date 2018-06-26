package org.usfirst.frc.team4256.robot.Autonomous;//COMPLETE MARCH

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;
import com.cyborgcats.reusable.Autonomous.Odometer;
import com.cyborgcats.reusable.Autonomous.Strategy;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;

import java.util.Map;

import org.usfirst.frc.team4256.robot.Clamp;
import org.usfirst.frc.team4256.robot.Elevator.Elevator;

public class S_DriveForward extends Strategy2018 {
	private static Long start = null;
	
	public S_DriveForward(final int startingPosition, final String gameData, final Odometer odometer) {super(startingPosition, gameData, odometer);}
	public S_DriveForward(final int startingPosition, final String gameData) {super(startingPosition, gameData, new Strategy.O_Useless());}
	
	@Override
	public void use(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems) {
		ensureTimerHasStarted();
		if (System.currentTimeMillis() - start < 2000) {
			drivetrain.travelTowards(0.0);
			drivetrain.setSpeed(0.0);
			drivetrain.setSpeed(0.0);
			subsystems.get("Clamp").perform(Clamp.Abilities.CLOSE.name(), null);
			subsystems.get("Elevator").perform(Elevator.Abilities.SET.name(), new double[] {3.0});
		}else if (System.currentTimeMillis() - start < 5000) {
			drivetrain.setSpeed(0.5);
			subsystems.get("Elevator").perform(Elevator.Abilities.SET.name(), new double [] {ElevatorPresets.SWITCH.height()});
		}else {
			drivetrain.setSpeed(0.0);
			if ((startingPosition.equals(StartingPosition.LEFT) && switchTarget.equals(FieldPieceConfig.LEFT)) ||
				(startingPosition.equals(StartingPosition.RIGHT) && switchTarget.equals(FieldPieceConfig.RIGHT))) {
				subsystems.get("Clamp").perform(Clamp.Abilities.SPIT.name(), null);
			}
		}
		drivetrain.completeLoopUpdate();
	}
	
	private static void ensureTimerHasStarted() {if (start == null) start = System.currentTimeMillis();}
}
