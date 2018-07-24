package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Autonomous.Strategy2018.StartingPosition;

import com.cyborgcats.reusable.Autonomous.Odometer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Coach {
	private Coach() {}
	
	public static final SendableChooser<Strategies> strategyChooser = new SendableChooser<Strategies>();
	public static final SendableChooser<StartingPosition> positionChooser = new SendableChooser<StartingPosition>();
	private static String oldGameData = "", newGameData = "";
	private static boolean hasGameData = false;
	
	public static void init() {
		oldGameData = DriverStation.getInstance().getGameSpecificMessage();
		listOptions();
		SmartDashboard.putData("Strategy", strategyChooser);
		SmartDashboard.putData("Position", positionChooser);
	}
	
	private static void listOptions() {
		boolean addedDefault = false;
		for (Strategies strategy : Strategies.values()) {
			if (addedDefault) strategyChooser.addObject(strategy.toString(), strategy);
			else {
				strategyChooser.addDefault(strategy.toString(), strategy);
				addedDefault = true;
			}
		}
		positionChooser.addObject(StartingPosition.LEFT.name(), StartingPosition.LEFT);
		positionChooser.addDefault(StartingPosition.CENTER.name(), StartingPosition.CENTER);
		positionChooser.addObject(StartingPosition.RIGHT.name(), StartingPosition.RIGHT);
	}
	
	public static Strategy2018 selectedStrategy(final Odometer odometer) {
		switch(strategyChooser.getSelected()) {
		case DriveForward: if (hasGameData) return new S_DriveForward(positionChooser.getSelected(), gameData());
		case DropInNearest: if (hasGameData) return new S_DropInNearest(positionChooser.getSelected(), gameData(), odometer);
		//if no game data, DriveForward and DropInNearest cases fall into PassLine
		case PassLine: return new S_PassLine(positionChooser.getSelected(), odometer);
		case Slither: return new S_Slither(odometer);
		case Mickey: return new S_Mickey(odometer);
		
		default: return new S_PassLine(positionChooser.getSelected(), odometer);
		}
	}
	
	public static void waitForGameData(final double timeoutMS) {
		final long start = System.currentTimeMillis();
		while (!hasGameData && (System.currentTimeMillis() - start <= timeoutMS)) pollGameData();
	}
	
	public static void pollGameData() {
		newGameData = DriverStation.getInstance().getGameSpecificMessage();
		if ((newGameData != null) && (newGameData.length() == 3) && (newGameData != oldGameData)) hasGameData = true;
		else hasGameData = false;
	}
	
	public static boolean hasGameData() {return hasGameData;}
	
	public static char[] gameData() {
		if (hasGameData) return new char[] {newGameData.charAt(0), newGameData.charAt(1), newGameData.charAt(2)};
		else return new char[] {};
	}
	


	public static enum Strategies {
		DriveForward("Open-Loop Forward"),
		DropInNearest("Smart Forward"),
		PassLine("No Elevator"),
		Slither("Slither Drive"),
		Mickey("Mickey Mouse");
		
		private final String readableName;
		Strategies(final String readableName) {this.readableName = readableName;}
		@Override
		public String toString() {return readableName;}
	}
}
