package org.usfirst.frc.team4256.robot;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Elevators.*;

import com.cyborgcats.reusable.Gyro;
import com.cyborgcats.reusable.Xbox;
import com.cyborgcats.reusable.Fridge;
import com.cyborgcats.reusable.PID;
import com.cyborgcats.reusable.Subsystem;
import com.cyborgcats.reusable.Autonomous.Odometer;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.usfirst.frc.team4256.robot.Clamp;
import org.usfirst.frc.team4256.robot.Autonomous.S_DriveForward;
import org.usfirst.frc.team4256.robot.Autonomous.S_PassLine;
import org.usfirst.frc.team4256.robot.Autonomous.S_DropInNearest;
import org.usfirst.frc.team4256.robot.Autonomous.Strategy2018;
import org.usfirst.frc.team4256.robot.Autonomous.O_Encoder;
import org.usfirst.frc.team4256.robot.Autonomous.O_ZED;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.hal.PowerJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final Xbox driver = new Xbox(0), gunner = new Xbox(1);
	//{Robot Input}
	private static final Gyro gyro = new Gyro(Parameters.Gyrometer_updateHz);
	public static double robotHeading = 0.0;
	private static final AnalogInput pressureGauge = new AnalogInput(Parameters.PRESSURE_GAUGE);
	private static final DigitalInput tx2PowerSensor = new DigitalInput(Parameters.TX2_POWER_SENSOR);
	
	private static Strategy2018 strategy;
	private static NetworkTableInstance nt;
	private static NetworkTable faraday, zed;
	private static Odometer odometer;
	
	//{Game Input}
	private static String gameData_old = "", gameData_new = "";
	private static boolean haveGameData = false;

	//{Robot Output}
	private static final SwerveModule
	moduleA = new SwerveModule(Parameters.SWERVE_ROTATOR_A,/*flipped sensor*/ false, Parameters.SWERVE_DRIVE_A, Parameters.SWERVE_MAGNET_A),
	moduleB = new SwerveModule(Parameters.SWERVE_ROTATOR_B,/*flipped sensor*/ false, Parameters.SWERVE_DRIVE_B, Parameters.SWERVE_MAGNET_B),
	moduleC = new SwerveModule(Parameters.SWERVE_ROTATOR_C,/*flipped sensor*/ false, Parameters.SWERVE_DRIVE_C, Parameters.SWERVE_MAGNET_C),
	moduleD = new SwerveModule(Parameters.SWERVE_ROTATOR_D,/*flipped sensor*/ false, Parameters.SWERVE_DRIVE_D, true, Parameters.SWERVE_MAGNET_D);
	private static final D_Swerve swerve = new D_Swerve(moduleA, moduleB, moduleC, moduleD);
	
	private static final DoubleSolenoid elevatorOneShifter = new DoubleSolenoid(Parameters.ELEVATOR_ONE_SHIFTER_MODULE, Parameters.ELEVATOR_ONE_SHIFTER_FORWARD, Parameters.ELEVATOR_ONE_SHIFTER_REVERSE);
	private static final E_One elevatorOne = new E_One(Parameters.ELEVATOR_ONE_MASTER, Parameters.ELEVATOR_ONE_FOLLOWER_A, Parameters.ELEVATOR_ONE_FOLLOWER_B, elevatorOneShifter);
	private static final E_Two elevatorTwo = new E_Two(Parameters.ELEVATOR_TWO_MASTER);
	private static final R_Combined elevator = new R_Combined(elevatorOne, elevatorTwo);
	
	private static final DoubleSolenoid clampShifter = new DoubleSolenoid(Parameters.CLAMP_MODUlE, Parameters.CLAMP_FORWARD, Parameters.CLAMP_REVERSE);
	private static final Clamp clamp = new Clamp(Parameters.INTAKE_LEFT, Parameters.INTAKE_RIGHT, clampShifter, Parameters.CLAMPY_ROTATOR, Parameters.ULTRASONIC);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(Parameters.TX2_POWER_CONTROL);
	
	private static final Map<String, Subsystem> subsystems = new HashMap<String, Subsystem>();

	private static final Logger logger = Logger.getLogger("Robot");
	
	@Override
	public void robotInit() {
		//{Robot Input}
		gyro.reset();
		nt = NetworkTableInstance.getDefault();
		faraday = nt.getTable("Faraday");
		zed = nt.getTable("ZED");
		odometer = new O_Encoder(moduleD);//new O_ZED(zed);
		odometer.init();
		
		//{Human Input}
		faraday.getEntry("Starting Position").setNumber(1);
		faraday.getEntry("Simple Auto").setBoolean(false);
		
		//{Game Input}
		final DriverStation ds = DriverStation.getInstance();
		gameData_old = ds.getGameSpecificMessage();
		
		//{Robot Output}
		subsystems.put("Clamp", clamp);
		subsystems.put("Elevator", elevator);
		swerve.init();
		elevator.init();
		clamp.init();

		setupLogging(ds);
		
		moduleA.setTareAngle(-78.0);moduleB.setTareAngle(-28.0);moduleC.setTareAngle(45.0);moduleD.setTareAngle(-77.0);
		moduleA.setParentLogger(logger);moduleB.setParentLogger(logger);moduleC.setParentLogger(logger);moduleD.setParentLogger(logger);
		
		elevatorOne.setZero(0.5);
		elevatorTwo.setZero(1.0);
		clamp.setZero();

		PID.set("zed", Parameters.ZED_P, Parameters.ZED_I, Parameters.ZED_D);
		PID.set("spin", Parameters.SPIN_P, Parameters.SPIN_I, Parameters.SPIN_D);
		
		if (!tx2PowerSensor.get()) {
			tx2PowerControl.set(true);
			try {Thread.sleep(50);}//milliseconds
			catch (InterruptedException e) {Thread.currentThread().interrupt();}
			tx2PowerControl.set(false);
		}
	}

	@Override
	public void autonomousInit() {
		//{Human Input}
		final int startingPosition = Math.round(faraday.getEntry("Starting Position").getNumber(1).floatValue());
		final boolean simpleAuto = faraday.getEntry("Simple Auto").getBoolean(false);
		
		//{Game Input}
		final long start = System.currentTimeMillis();
		while (!haveGameData && (System.currentTimeMillis() - start <= 2500)) pollGameData();
		if(!simpleAuto) {
			if (haveGameData) strategy = new S_DropInNearest(startingPosition, gameData_new, odometer);
			else strategy = new S_PassLine(startingPosition, odometer);
		}else {
			strategy = new S_DriveForward(startingPosition, gameData_new);
		}
		
		//{Robot Input}
		zed.getEntry("Enable Odometry").setBoolean(false);//TODO if we want to switch back to ZED make sure to remove this line
		odometer.setOrigin(odometer.getX(false) - strategy.initialX(), odometer.getY(false) - strategy.initialY());
		
		//{Robot Output}
		swerve.autoMode(true);
		
		PID.clear("zed");
		PID.clear("spin");
	}
	
	@Override
	public void teleopInit() {
		//{Robot Input}
		if (odometer.getClass().isInstance(O_ZED.class)) ((O_ZED) odometer).disable();
		//{Robot Output}
		swerve.autoMode(false);
		
		PID.clear("spin");
		
		Fridge.initialize("Button LB", true);
		Fridge.initialize("Button RB", true);
	}
	
	@Override
	public void testInit() {}
	
	@Override
	public void disabledInit() {}
	
	@Override
	public void robotPeriodic() {
		robotHeading = gyro.getCurrentAngle();
		faraday.getEntry("Gyro").setNumber(robotHeading);
		faraday.getEntry("Pressure").setNumber(pressureGauge.getAverageVoltage()*39.875);
		faraday.getEntry("Clamp Open").setBoolean(clamp.isOpen());
		faraday.getEntry("Climbing Mode").setBoolean(elevator.inClimbingMode());
		faraday.getEntry("Browning Out").setBoolean(RobotController.isBrownedOut());
		faraday.getEntry("TX2 Powered On").setBoolean(tx2PowerSensor.get());
		faraday.getEntry("Has Cube").setBoolean(clamp.hasCube());
		faraday.getEntry("Match Timer").setNumber(DriverStation.getInstance().getMatchTime());
		faraday.getEntry("Battery Voltage").setNumber(PowerJNI.getVinVoltage());
		faraday.getEntry("Received Field").setBoolean(haveGameData);
		if (gameData_new != null) faraday.getEntry("Field Data").setString(gameData_new);
		faraday.getEntry("Angle A").setNumber(moduleA.rotationMotor().getCurrentAngle(true));
		faraday.getEntry("Angle B").setNumber(moduleB.rotationMotor().getCurrentAngle(true));
		faraday.getEntry("Angle C").setNumber(moduleC.rotationMotor().getCurrentAngle(true));
		faraday.getEntry("Angle D").setNumber(moduleD.rotationMotor().getCurrentAngle(true));

		odometer.completeLoopUpdate();
		faraday.getEntry("X").setNumber(odometer.getX(/*markAsRead*/false));
		faraday.getEntry("Y").setNumber(odometer.getY(/*markAsRead*/false));
	}
	
	@Override
	public void autonomousPeriodic() {strategy.use(swerve, subsystems);}
	
	@Override
	public void teleopPeriodic() {
		clamp.beginLoopUpdate();
		
		//{speed multipliers}
		final boolean turbo = driver.getRawButton(Xbox.BUTTON_STICK_LEFT);
		final boolean snail = driver.getRawButton(Xbox.BUTTON_STICK_RIGHT);
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(Xbox.STICK_LEFT, true);//turbo mode
		if (!turbo) speed *= 0.7;//---------------------------------------normal mode
		if (snail)  speed *= 0.5;//---------------------------------------snail mode
		speed *= speed;
		
		//{calculating spin}
		double spin = 0.7*driver.getDeadbandedAxis(Xbox.AXIS_RIGHT_X);//normal mode
		if (snail) spin  *= 0.7;//----------------------------------------snail mode
		spin *= spin*Math.signum(spin);
		
		if (driver.getRawButton(Xbox.BUTTON_X)) swerve.formX();//X lock
		else {//SWERVE DRIVE
			swerve.travelTowards(driver.getCurrentAngle(Xbox.STICK_LEFT, true));
			swerve.setSpeed(speed);
			swerve.setSpin(spin);
//			swerve.holonomic_encoderIgnorant(driver.getCurrentAngle(Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		}
		
		
		if (!elevator.inClimbingMode()) {
				 if (gunner.getRawButton(Xbox.BUTTON_A)) {elevator.setInches(ElevatorPresets.FLOOR.height());}//ELEVATOR PRESETS
			else if (gunner.getRawButton(Xbox.BUTTON_X)) {elevator.setInches(ElevatorPresets.SWITCH.height());}
			else if (gunner.getRawButton(Xbox.BUTTON_B)) {elevator.setInches(ElevatorPresets.SCALE_LOW.height());}
			else if (gunner.getRawButton(Xbox.BUTTON_Y)) {elevator.setInches(ElevatorPresets.SCALE_HIGH.height());}
		
				 if (Fridge.becomesTrue("Button LB", gunner.getRawButton(Xbox.BUTTON_LB))) elevator.increment(-11.0);//ELEVATOR INCREMENTS (gunner)
			else if (Fridge.becomesTrue("Button RB", gunner.getRawButton(Xbox.BUTTON_RB))) elevator.increment(11.0);
			else if (gunner.getAxisPress(Xbox.AXIS_LT, 0.1)) elevator.increment(-0.7*gunner.getRawAxis(Xbox.AXIS_LT));
			else if (gunner.getAxisPress(Xbox.AXIS_RT, 0.1)) elevator.increment(0.7*gunner.getRawAxis(Xbox.AXIS_RT));
				 
				 
				 if (driver.getAxisPress(Xbox.AXIS_RT, 0.5)) clamp.slurp();//CLAMP SLURP AND SPIT
			else if (driver.getAxisPress(Xbox.AXIS_LT, 0.2)) clamp.spit(driver.getRawAxis(Xbox.AXIS_LT));
			else 											   clamp.stop();
			
				 if (driver.getRawButton(Xbox.BUTTON_LB)) clamp.open();//CLAMP OPEN AND CLOSE OVERRIDE
			else if (driver.getRawButton(Xbox.BUTTON_RB)) clamp.close();
		
		}else {
				 if (driver.getAxisPress(Xbox.AXIS_LT, 0.1)) elevator.increment(-0.7*driver.getRawAxis(Xbox.AXIS_LT));//ELEVATOR INCREMENTS (driver)
			else if (driver.getAxisPress(Xbox.AXIS_RT, 0.1)) elevator.increment(0.7*driver.getRawAxis(Xbox.AXIS_RT));
		}
		
		if (elevator.inClimbingMode() != Fridge.freeze("Button Start", driver.getRawButton(Xbox.BUTTON_START))) {//CLIMBING MODE
			if (elevator.inClimbingMode()) elevator.disableClimbMode();
			else elevator.enableClimbMode(clamp);
		}
		
		final double gunnerLeftX = gunner.getRawAxis(Xbox.AXIS_LEFT_X);
		if (Math.abs(gunnerLeftX) > 0.5) clamp.rotateTo(45.0*(Math.signum(gunnerLeftX) + 1.0));
		else clamp.increment(-2.0*gunner.getDeadbandedAxis(Xbox.AXIS_LEFT_Y));
		
		
		if (gunner.getRawButton(Xbox.BUTTON_START)) swerve.align();//SWERVE ALIGNMENT

		if (Fridge.becomesTrue("gyro reset", driver.getRawButton(Xbox.BUTTON_BACK))) {//GYRO RESET
			gyro.setTareAngle(robotHeading, true);
			PID.clear("spin");
		}
		
		
		final double rumbleAmount = clamp.hasCube() ? 0.4 : 0.0;
		driver.setRumble(RumbleType.kLeftRumble, rumbleAmount); driver.setRumble(RumbleType.kRightRumble, rumbleAmount);
		gunner.setRumble(RumbleType.kLeftRumble, rumbleAmount); gunner.setRumble(RumbleType.kRightRumble, rumbleAmount);
//		if (gyro.netAcceleration() >= 1) driver.setRumble(RumbleType.kLeftRumble, 1.0);//DANGER RUMBLE
//		else driver.setRumble(RumbleType.kLeftRumble, 0.0);
		
		
		//{completing motor controller updates}
		swerve.completeLoopUpdate();
		elevator.completeLoopUpdate();
		clamp.completeLoopUpdate();
	}
	
	@Override
	public void testPeriodic() {
		moduleA.swivelTo(0.0);
		moduleB.swivelTo(0.0);
		moduleC.swivelTo(0.0);
		moduleD.swivelTo(0.0);
	}
	
	@Override
	public void disabledPeriodic() {
		pollGameData();
		driver.setRumble(RumbleType.kLeftRumble, 0.0); driver.setRumble(RumbleType.kRightRumble, 0.0);
		gunner.setRumble(RumbleType.kLeftRumble, 0.0); gunner.setRumble(RumbleType.kRightRumble, 0.0);
	}
	

	private static void pollGameData() {
		gameData_new = DriverStation.getInstance().getGameSpecificMessage();
		if ((gameData_new != null) && (gameData_new.length() == 3) && (gameData_new != gameData_old)) haveGameData = true;
		else haveGameData = false;
	}
	
	private static void setupLogging(final DriverStation ds) {
		String logFileName = "/U/", eventName = ds.getEventName(), matchNumber = Integer.toString(ds.getMatchNumber()), replayNumber = Integer.toString(ds.getReplayNumber());
		
		switch (ds.getMatchType()) {
		case None:
			logFileName += "Unknown.log";
			eventName = "None"; matchNumber = "0"; replayNumber = "0";
			break;
		case Practice: logFileName += "Practice_";break;
		case Qualification: logFileName += "Qualification_";break;
		case Elimination: logFileName += "Elimination_";break;
		}
		
		logFileName += eventName + "_" + matchNumber + "_" + replayNumber + ".log"; 
		
		try {
			logger.addHandler(new FileHandler(logFileName));
			logger.setLevel(Level.FINE);
			logger.log(Level.CONFIG, "Event:" + eventName);
			logger.log(Level.CONFIG,  "Match:" + matchNumber);
			logger.log(Level.CONFIG,  "Replay:" + replayNumber);
		}catch (SecurityException | IOException e1) {/*ignore exceptions*/}
	}
}
