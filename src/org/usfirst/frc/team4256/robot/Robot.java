package org.usfirst.frc.team4256.robot;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Elevators.*;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.R_Xbox;
import com.cyborgcats.reusable.V_Fridge;
import com.cyborgcats.reusable.V_PID;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.Autonomous.Events;
import org.usfirst.frc.team4256.robot.Autonomous.Instructions;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final R_Xbox driver = new R_Xbox(0);
	private static final R_Xbox gunner = new R_Xbox(1);
	private static double lockedAngle = 0;
	//{Robot Input}
	private static final R_Gyro gyro = new R_Gyro(Parameters.Gyrometer_updateHz, 0, 0);
	private static final AnalogInput pressureGauge = new AnalogInput(0);
	private static final DigitalInput tx2PowerSensor = new DigitalInput(8);
	private static Instructions instructions;
	private static NetworkTableInstance nt;
	private static NetworkTable faraday;
	private static NetworkTable zed;
	private static boolean updatedFeetX = false;
	private static boolean updatedFeetY = false;
	private static double actualFeetX = 0.0;
	private static double actualFeetY = 0.0;

	//{Robot Output}
	private static final R_SwerveModule moduleA = new R_SwerveModule(Parameters.Swerve_rotatorA,/*flipped sensor*/ false, Parameters.Swerve_driveA);
	private static final R_SwerveModule moduleB = new R_SwerveModule(Parameters.Swerve_rotatorB,/*flipped sensor*/ false, Parameters.Swerve_driveB);
	private static final R_SwerveModule moduleC = new R_SwerveModule(Parameters.Swerve_rotatorC,/*flipped sensor*/ false, Parameters.Swerve_driveC);
	private static final R_SwerveModule moduleD = new R_SwerveModule(Parameters.Swerve_rotatorD,/*flipped sensor*/ false, Parameters.Swerve_driveD, false);
	private static final R_DriveTrain swerve = new R_DriveTrain(gyro, moduleA, moduleB, moduleC, moduleD);
	
	private static final DoubleSolenoid elevatorOneShifter = new DoubleSolenoid(Parameters.ElevatorOne_shifterModule, Parameters.ElevatorOne_shifterForward, Parameters.ElevatorOne_shifterReverse);
	private static final R_ElevatorOne elevatorOne = new R_ElevatorOne(Parameters.ElevatorOne_master, Parameters.ElevatorOne_followerA, Parameters.ElevatorOne_followerB, elevatorOneShifter);
	private static final R_ElevatorTwo elevatorTwo = new R_ElevatorTwo(Parameters.ElevatorTwo_master);
	private static final R_Elevators elevators = new R_Elevators(elevatorOne, elevatorTwo);
	
	private static final DoubleSolenoid clampShifter = new DoubleSolenoid(Parameters.Clamp_module, Parameters.Clamp_forward, Parameters.Clamp_reverse);
	private static final R_Clamp clamp = new R_Clamp(Parameters.Intake_left, Parameters.Intake_right, clampShifter, Parameters.clampyRotator, Parameters.ultrasonic);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(9);
	
	@Override
	public void robotInit() {
		//{Robot Input}
		gyro.reset();
		nt = NetworkTableInstance.getDefault();
		faraday = nt.getTable("Faraday");
		zed = nt.getTable("ZED").getSubTable("Position");
		//{Robot Output}
		swerve.init();
		elevators.init();
		clamp.init();
		
		moduleA.setTareAngle(-66.0);	moduleB.setTareAngle(-43.0);	moduleC.setTareAngle(5.0);	moduleD.setTareAngle(45.0);
		//competition robot: -68.0							 59.0						 -3.0						 56.0
		//practice robot:	 -66.0,						 	 -43.0,							 5.0,						 45.0

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
		V_PID.clear("zed");
		V_PID.clear("spin");
		
		swerve.autoMode(true);
		
		zed.addEntryListener("X", (zed, key, entry, value, flags) -> {
			actualFeetX = value.getDouble();
			updatedFeetX = true;
		}, EntryListenerFlags.kUpdate);
		zed.addEntryListener("Y", (zed, key, entry, value, flags) -> {
			actualFeetY = value.getDouble();
			updatedFeetY = true;
		}, EntryListenerFlags.kUpdate);
		
		
		final String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) instructions = new Instructions(gameData);
	}
	
	@Override
	public void teleopInit() {
		V_PID.clear("spin");
		swerve.autoMode(false);
		lockedAngle = gyro.getCurrentAngle();
		
		V_Fridge.initialize("!Button LT", true);
		V_Fridge.initialize("!Button RT", true);
	}
	
	@Override
	public void testInit() {
	}
	
	@Override
	public void disabledInit() {
	}
	
	@Override
	public void robotPeriodic() {
		faraday.getEntry("Gyro").setNumber(gyro.getCurrentAngle());
		faraday.getEntry("Pressure").setNumber(pressureGauge.getAverageVoltage()*39.875 - 54.375);//TODO are these constants correct?
		faraday.getEntry("Zeroed Elevators").setBoolean(elevatorOne.knowsZero && elevatorTwo.knowsZero);
		faraday.getEntry("Clamp Open").setBoolean(clamp.isOpen());
		faraday.getEntry("Climbing Mode").setBoolean(elevators.inClimbingMode());
		faraday.getEntry("Browning Out").setBoolean(RobotController.isBrownedOut());
		faraday.getEntry("TX2 Powered On").setBoolean(tx2PowerSensor.get());
		//TODO put whether we have a cube or not
	}
	
	@Override
	public void autonomousPeriodic() {
		if (updatedFeetX && updatedFeetY) {
			final double actualX = actualFeetX,		  actualY = actualFeetY;
			instructions.getLeash().maintainLength(actualX, actualY);
			final double desiredX = instructions.getLeash().getX(), 	  desiredY = instructions.getLeash().getY();
			final double errorX = desiredX - actualX, errorY = desiredY - actualY;
			
			final double errorDirection = Math.toDegrees(Math.atan2(errorX, errorY));
			final double errorMagnitude = Math.sqrt(errorX*errorX + errorY*errorY);
			double speed = V_PID.get("zed", errorMagnitude);
			if (Math.abs(speed) > 0.7) speed = 0.7*Math.signum(speed);
			
			
			double desiredOrientation = gyro.getCurrentAngle();
			
			
			Events.check(instructions.getLeash().getIndependentVariable());
			
			switch(Events.counter) {
			case(0):
				desiredOrientation = 0.0;
				clamp.close();
				break;
			case(1): 
				elevators.setInches(ElevatorPresets.SCALE_HIGH.height());
				break;
			case(2): clamp.spit();break;
			}
			
			
			final double errorOrientation = gyro.wornPath(desiredOrientation)/180.0;
			double spin = V_PID.get("spin", errorOrientation);
			if (Math.abs(spin) > 0.5) spin = 0.5*Math.signum(spin);
			
			
			swerve.holonomic_encoderIgnorant(errorDirection, speed, spin);
			
			updatedFeetX = false; updatedFeetY = false;
		}
	}
	
	@Override
	public void teleopPeriodic() {
		//{speed multipliers}
		final boolean turbo = driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT);
		final boolean snail = driver.getRawButton(R_Xbox.BUTTON_STICK_RIGHT);
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(R_Xbox.STICK_LEFT, true);//turbo mode
		if (!turbo) {speed *= 0.7;}//-------------------------------------normal mode
		if (snail)  {speed *= 0.5;}//-------------------------------------snail mode
		speed *= speed;
		
		//{calculating spin}
		double spin = 0.7*driver.getDeadbandedAxis(R_Xbox.AXIS_RIGHT_X);//normal mode
		if (snail)  {spin  *= 0.7;	/*lockWheelsInPlace = true;*/}//------snail mode//TODO this boolean should face all wheels forward or in an X position rather than coasting
		spin *= spin*Math.signum(spin);
		final boolean handsOffSpinStick = spin == 0.0;
		
		//{adding driver aids}
		if (handsOffSpinStick) {
			double spinError = 0;
			//stop rotation drift at high speeds
			if (speed >= 0.6) {spinError = gyro.wornPath(lockedAngle);}
			if (Math.abs(spinError) > 3.0) {spin = V_PID.get("spin", spinError);}
			if (Math.abs(spin) > 1.0) spin = Math.signum(spin);
		}
		if (V_Fridge.becomesTrue("hands off", handsOffSpinStick)) {
			//remember angle when driver stops rotating
			lockedAngle = gyro.getCurrentAngle();
			V_PID.clear("spin");
		}
		
		swerve.holonomic_encoderAware(driver.getCurrentAngle(R_Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		
		
		if (!elevators.inClimbingMode()) {
				 if (gunner.getRawButton(R_Xbox.BUTTON_A)) {elevators.setInches(ElevatorPresets.FLOOR.height());}//ELEVATOR PRESETS
			else if (gunner.getRawButton(R_Xbox.BUTTON_X)) {elevators.setInches(ElevatorPresets.SWITCH.height());}
			else if (gunner.getRawButton(R_Xbox.BUTTON_B)) {elevators.setInches(ElevatorPresets.SCALE_LOW.height());}
			else if (gunner.getRawButton(R_Xbox.BUTTON_Y)) {elevators.setInches(ElevatorPresets.SCALE_HIGH.height());}
		
				 if (V_Fridge.becomesTrue("Button LB", gunner.getRawButton(R_Xbox.BUTTON_LB))) elevators.increment(-11.0);//ELEVATOR INCREMENTS (gunner)
			else if (V_Fridge.becomesTrue("Button RB", gunner.getRawButton(R_Xbox.BUTTON_RB))) elevators.increment(11.0);
			else if (gunner.getAxisPress(R_Xbox.AXIS_LT, 0.1)) elevators.increment(-0.7*gunner.getRawAxis(R_Xbox.AXIS_LT));
			else if (gunner.getAxisPress(R_Xbox.AXIS_RT, 0.1)) elevators.increment(0.7*gunner.getRawAxis(R_Xbox.AXIS_RT));
				 
				 
				 if (driver.getAxisPress(R_Xbox.AXIS_RT, 0.5) && !clamp.hasCube()) clamp.slurp();//CLAMP SLURP AND SPIT
			else if (driver.getAxisPress(R_Xbox.AXIS_LT, 0.5)) 					   clamp.spit();
			else 																   clamp.stop();
			
				 if (driver.getRawButton(R_Xbox.BUTTON_LB)) clamp.open();//CLAMP OPEN AND CLOSE OVERRIDE
			else if (driver.getRawButton(R_Xbox.BUTTON_RB)) clamp.close();
		
		}else {
				 if (driver.getAxisPress(R_Xbox.AXIS_LT, 0.1)) elevators.increment(-0.7*driver.getRawAxis(R_Xbox.AXIS_LT));//ELEVATOR INCREMENTS (driver)
			else if (driver.getAxisPress(R_Xbox.AXIS_RT, 0.1)) elevators.increment(0.7*driver.getRawAxis(R_Xbox.AXIS_RT));
		}

		
		if (elevators.inClimbingMode() != V_Fridge.freeze("Button Start", driver.getRawButton(R_Xbox.BUTTON_START))) {//CLIMBING MODE
			if (elevators.inClimbingMode()) elevators.disableClimbMode(clamp);
			else elevators.enableClimbMode(clamp);
		}

		
		if (V_Fridge.becomesTrue("gyro reset", driver.getRawButton(R_Xbox.BUTTON_BACK))) {//GYRO RESET
			gyro.reset();
			lockedAngle = gyro.getCurrentAngle();
			V_PID.clear("spin");
		}
		
		
		if (gyro.netAcceleration() >= 1) {//DANGER RUMBLE
			driver.setRumble(RumbleType.kLeftRumble, 1.0);
		}else {
			driver.setRumble(RumbleType.kLeftRumble, 0.0);
		}
		
		
		//{completing motor controller updates}
		swerve.completeLoopUpdate();
		elevators.completeLoopUpdate();
		clamp.completeLoopUpdate();
	}
	
	@Override
	public void testPeriodic() {
		moduleA.swivelTo(0);
		moduleB.swivelTo(0);
		moduleC.swivelTo(0);
		moduleD.swivelTo(0);
		elevatorOne.setZero(-0.5);
		elevatorTwo.setZero(0.0);
		elevators.setInches(0.0);
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
