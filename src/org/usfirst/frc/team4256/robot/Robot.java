package org.usfirst.frc.team4256.robot;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Elevators.*;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.R_Xbox;
import com.cyborgcats.reusable.V_Fridge;
import com.cyborgcats.reusable.V_PID;
import com.cyborgcats.reusable.Autonomous.V_Odometer;

import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.Autonomous.A_ForwardOpenLoop;
import org.usfirst.frc.team4256.robot.Autonomous.A_OneSwitchOneScale;
import org.usfirst.frc.team4256.robot.Autonomous.A_PassLine;
import org.usfirst.frc.team4256.robot.Autonomous.Autonomous;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.hal.PowerJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final R_Xbox driver = new R_Xbox(0), gunner = new R_Xbox(1);
//	private static double lockedAngle = 0;
	//{Robot Input}
	private static final R_Gyro gyro = new R_Gyro(Parameters.Gyrometer_updateHz);
	private static final AnalogInput pressureGauge = new AnalogInput(Parameters.pressureGauge);
	private static final DigitalInput tx2PowerSensor = new DigitalInput(Parameters.tx2PowerSensor);
	
	private static Autonomous autonomous;
	private static NetworkTableInstance nt;
	private static NetworkTable faraday, zed;
	private static V_Odometer odometer;
	
	//{Game Input}
	private static String gameData_old = "", gameData_new = "";
	private static boolean haveGameData = false;

	//{Robot Output}
	private static final R_SwerveModule
	moduleA = new R_SwerveModule(Parameters.Swerve_rotatorA,/*flipped sensor*/ false, Parameters.Swerve_driveA),
	moduleB = new R_SwerveModule(Parameters.Swerve_rotatorB,/*flipped sensor*/ false, Parameters.Swerve_driveB),
	moduleC = new R_SwerveModule(Parameters.Swerve_rotatorC,/*flipped sensor*/ false, Parameters.Swerve_driveC),
	moduleD = new R_SwerveModule(Parameters.Swerve_rotatorD,/*flipped sensor*/ false, Parameters.Swerve_driveD, true);
	private static final R_Drivetrain swerve = new R_Drivetrain(gyro, moduleA, moduleB, moduleC, moduleD);
	
	private static final DoubleSolenoid elevatorOneShifter = new DoubleSolenoid(Parameters.ElevatorOne_shifterModule, Parameters.ElevatorOne_shifterForward, Parameters.ElevatorOne_shifterReverse);
	private static final E_One elevatorOne = new E_One(Parameters.ElevatorOne_master, Parameters.ElevatorOne_followerA, Parameters.ElevatorOne_followerB, elevatorOneShifter);
	private static final E_Two elevatorTwo = new E_Two(Parameters.ElevatorTwo_master);
	private static final R_Combined elevator = new R_Combined(elevatorOne, elevatorTwo);
	
	private static final DoubleSolenoid clampShifter = new DoubleSolenoid(Parameters.Clamp_module, Parameters.Clamp_forward, Parameters.Clamp_reverse);
	private static final R_Clamp clamp = new R_Clamp(Parameters.Intake_left, Parameters.Intake_right, clampShifter, Parameters.clampyRotator, Parameters.ultrasonic);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(Parameters.tx2PowerControl);

	private static final Logger logger = Logger.getLogger("Robot");
	
	@Override
	public void robotInit() {
		//{Robot Input}
		gyro.reset();
		nt = NetworkTableInstance.getDefault();
		faraday = nt.getTable("Faraday");
		zed = nt.getTable("ZED");
		odometer = new V_Odometer(zed);
		odometer.init();
		//{Human Input}
		faraday.getEntry("Starting Position").setNumber(0);
		faraday.getEntry("Simple Auto").setBoolean(false);
		//{Game Input}
		final DriverStation ds = DriverStation.getInstance();
		gameData_old = ds.getGameSpecificMessage();
		//{Robot Output}
		swerve.init();
		elevator.init();
		clamp.init();

		setupLogging(ds);
		
		moduleA.setTareAngle(35.0);moduleB.setTareAngle(-31.0);moduleC.setTareAngle(86.0);moduleD.setTareAngle(-73.0);
		//-67.0, -51.0, 85.0, -78.0
		//competition robot: -64.0, 80.0, -10.0, 25.0
		//practice robot:	 -26.0,	-104.0, 75.0, 48.0
		moduleA.setParentLogger(logger);moduleB.setParentLogger(logger);moduleC.setParentLogger(logger);moduleD.setParentLogger(logger);
		elevatorOne.setZero(0.0);
		elevatorTwo.setZero(0.0);
		clamp.setZero();

		V_PID.set("zed", Parameters.zedP, Parameters.zedI, Parameters.zedD);
		V_PID.set("spin", Parameters.spinP, Parameters.spinI, Parameters.spinD);
		
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
		final int startingPosition = Math.round(faraday.getEntry("Starting Position").getNumber(0).floatValue());
		final boolean simpleAuto = faraday.getEntry("Simple Auto").getBoolean(false);
		
		//{Game Input}
		final long start = System.currentTimeMillis();
		while (!haveGameData && (System.currentTimeMillis() - start <= 5000)) pollGameData();
		if(!simpleAuto) {
			if (haveGameData) {
				autonomous = new A_OneSwitchOneScale(startingPosition, gameData_new, odometer);
//				gyro.setTareAngle(-90.0, false);
			}else {
				autonomous = new A_PassLine(startingPosition, odometer);
//				gyro.setTareAngle(-90.0, false);
			}
		}else {
			autonomous = new A_ForwardOpenLoop(startingPosition, gameData_new);
//			gyro.setTareAngle(0.0, false);
		}
		
		//{Robot Input}
		odometer.setOrigin(odometer.getX() - autonomous.initOdometerPosX()/12.0, odometer.getY() - Autonomous.startY/12.0);
		
		//{Robot Output}
		swerve.autoMode(true);
		
		V_PID.clear("zed");
		V_PID.clear("spin");
	}
	
	@Override
	public void teleopInit() {
		//{Robot Input}
		odometer.disable();
//		lockedAngle = gyro.getCurrentAngle();
		//{Robot Output}
		swerve.autoMode(false);
		
		V_PID.clear("spin");
		
		V_Fridge.initialize("Button LB", true);
		V_Fridge.initialize("Button RB", true);
	}
	
	@Override
	public void testInit() {}
	
	@Override
	public void disabledInit() {}
	
	@Override
	public void robotPeriodic() {
		faraday.getEntry("Gyro").setNumber(gyro.getCurrentAngle());
		faraday.getEntry("Pressure").setNumber(pressureGauge.getAverageVoltage()*39.875);
		faraday.getEntry("Clamp Open").setBoolean(clamp.isOpen());
		faraday.getEntry("Climbing Mode").setBoolean(elevator.inClimbingMode());
		faraday.getEntry("Browning Out").setBoolean(RobotController.isBrownedOut());
		faraday.getEntry("TX2 Powered On").setBoolean(tx2PowerSensor.get());
		faraday.getEntry("Has Cube").setBoolean(clamp.hasCube());
		faraday.getEntry("Match Timer").setNumber(DriverStation.getInstance().getMatchTime());
		faraday.getEntry("Battery Voltage").setNumber(PowerJNI.getVinVoltage());
		faraday.getEntry("Received Field").setBoolean(haveGameData);
		SmartDashboard.putNumber("a", moduleA.rotationMotor().getCurrentAngle(true));
		SmartDashboard.putNumber("b", moduleB.rotationMotor().getCurrentAngle(true));
		SmartDashboard.putNumber("c", moduleC.rotationMotor().getCurrentAngle(true));
		SmartDashboard.putNumber("d", moduleD.rotationMotor().getCurrentAngle(true));
	}
	
	@Override
	public void autonomousPeriodic() {autonomous.run(swerve, clamp, elevator);}
	
	@Override
	public void teleopPeriodic() {
		//{speed multipliers}
		final boolean turbo = driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT);
		final boolean snail = driver.getRawButton(R_Xbox.BUTTON_STICK_RIGHT);
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(R_Xbox.STICK_LEFT, true);//turbo mode
		if (!turbo) speed *= 0.7;//---------------------------------------normal mode
		if (snail)  speed *= 0.5;//---------------------------------------snail mode
		speed *= speed;
		
		//{calculating spin}
		double spin = 0.7*driver.getDeadbandedAxis(R_Xbox.AXIS_RIGHT_X);//normal mode
		if (snail) spin  *= 0.7;//----------------------------------------snail mode
		spin *= spin*Math.signum(spin);
//		final boolean handsOffSpinStick = spin == 0.0;
//		
//		//{adding driver aids}
//		if (handsOffSpinStick) {
//			double spinError = 0;
//			//stop rotation drift at high speeds
//			if (speed >= 0.6) spinError = gyro.wornPath(lockedAngle);
//			if (Math.abs(spinError) > 3.0) spin = V_PID.get("spin", spinError);
//			if (Math.abs(spin) > 1.0) spin = Math.signum(spin);
//		}
//		if (V_Fridge.becomesTrue("hands off", handsOffSpinStick)) {
//			//remember angle when driver stops rotating
//			lockedAngle = gyro.getCurrentAngle();
//			V_PID.clear("spin");
//		}
		
		if (driver.getRawButton(R_Xbox.BUTTON_X)) swerve.formX();//X lock
		else swerve.holonomic_encoderIgnorant(driver.getCurrentAngle(R_Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		
		
		if (!elevator.inClimbingMode()) {
				 if (gunner.getRawButton(R_Xbox.BUTTON_A)) {elevator.setInches(ElevatorPresets.FLOOR.height());}//ELEVATOR PRESETS
			else if (gunner.getRawButton(R_Xbox.BUTTON_X)) {elevator.setInches(ElevatorPresets.SWITCH.height());}
			else if (gunner.getRawButton(R_Xbox.BUTTON_B)) {elevator.setInches(ElevatorPresets.SCALE_LOW.height());}
			else if (gunner.getRawButton(R_Xbox.BUTTON_Y)) {elevator.setInches(ElevatorPresets.SCALE_HIGH.height());}
		
				 if (V_Fridge.becomesTrue("Button LB", gunner.getRawButton(R_Xbox.BUTTON_LB))) elevator.increment(-11.0);//ELEVATOR INCREMENTS (gunner)
			else if (V_Fridge.becomesTrue("Button RB", gunner.getRawButton(R_Xbox.BUTTON_RB))) elevator.increment(11.0);
			else if (gunner.getAxisPress(R_Xbox.AXIS_LT, 0.1)) elevator.increment(-0.7*gunner.getRawAxis(R_Xbox.AXIS_LT));
			else if (gunner.getAxisPress(R_Xbox.AXIS_RT, 0.1)) elevator.increment(0.7*gunner.getRawAxis(R_Xbox.AXIS_RT));
				 
				 
				 if (driver.getAxisPress(R_Xbox.AXIS_RT, 0.5)) clamp.slurp();//CLAMP SLURP AND SPIT
			else if (driver.getAxisPress(R_Xbox.AXIS_LT, 0.2)) clamp.spit(driver.getRawAxis(R_Xbox.AXIS_LT));
			else 											   clamp.stop();
			
				 if (driver.getRawButton(R_Xbox.BUTTON_LB)) clamp.open();//CLAMP OPEN AND CLOSE OVERRIDE
			else if (driver.getRawButton(R_Xbox.BUTTON_RB)) clamp.close();
		
		}else {
				 if (driver.getAxisPress(R_Xbox.AXIS_LT, 0.1)) elevator.increment(-0.7*driver.getRawAxis(R_Xbox.AXIS_LT));//ELEVATOR INCREMENTS (driver)
			else if (driver.getAxisPress(R_Xbox.AXIS_RT, 0.1)) elevator.increment(0.7*driver.getRawAxis(R_Xbox.AXIS_RT));
		}
		
		if (elevator.inClimbingMode() != V_Fridge.freeze("Button Start", driver.getRawButton(R_Xbox.BUTTON_START))) {//CLIMBING MODE
			if (elevator.inClimbingMode()) elevator.disableClimbMode();
			else elevator.enableClimbMode(clamp);
		}
		
		final double gunnerLeftX = gunner.getRawAxis(R_Xbox.AXIS_LEFT_X);
		if (Math.abs(gunnerLeftX) > 0.5) clamp.rotateTo(45.0*(Math.signum(gunnerLeftX) + 1.0));
		else clamp.increment(-2.0*gunner.getDeadbandedAxis(R_Xbox.AXIS_LEFT_Y));

		
		if (V_Fridge.becomesTrue("gyro reset", driver.getRawButton(R_Xbox.BUTTON_BACK))) {//GYRO RESET
			gyro.setTareAngle(gyro.getCurrentAngle(), true);
//			lockedAngle = 0.0;
			V_PID.clear("spin");
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
