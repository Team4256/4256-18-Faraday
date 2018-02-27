package org.usfirst.frc.team4256.robot;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Elevators.*;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.R_Xbox;
import com.cyborgcats.reusable.V_Fridge;
import com.cyborgcats.reusable.V_PID;
import org.usfirst.frc.team4256.robot.R_Clamp;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final R_Xbox driver = new R_Xbox(0);
	private static double lockedAngle = 0;
	//{Robot Input}
	private static final R_Gyro gyro = new R_Gyro(Parameters.Gyrometer_updateHz, 0, 0);
	private static final AnalogInput pressureGauge = new AnalogInput(0);
	private static final DigitalInput tx2PowerSensor = new DigitalInput(8);
	private static NetworkTableInstance nt;
	private static NetworkTable faraday;
	private static NetworkTable zed;
	private static boolean updatedFeetX = false;
	private static boolean updatedFeetY = false;
	private static double actualFeetX = 0.0;
	private static double actualFeetY = 0.0;
	
	//{Robot Output}
//	private static final Compressor compressor = new Compressor(0);
	
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
	private static final DoubleSolenoid extenderShifter = new DoubleSolenoid(Parameters.Extender_module, Parameters.Extender_forward, Parameters.Extender_reverse);
	private static final R_Clamp clamp = new R_Clamp(Parameters.Intake_left, Parameters.Intake_right, clampShifter, extenderShifter, Parameters.ultrasonic);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(9);
	
	@Override
	public void robotInit() {
		//{Robot Input}
		gyro.reset();
		nt = NetworkTableInstance.getDefault();
		faraday = nt.getTable("Faraday");
		zed = nt.getTable("ZED").getSubTable("Position");
		//{Robot Output}
//		compressor.start();
//		compressor.setClosedLoopControl(true);
//		compressor.clearAllPCMStickyFaults();
		swerve.init();
		elevators.init();
		V_Fridge.initialize("!Button LB", true);
		V_Fridge.initialize("!Button RB", true);
		clamp.init();
//		lift.setVoltageRampRate(8);//TODO
		
		moduleA.setTareAngle(62.0);	moduleB.setTareAngle(-14.0);	moduleC.setTareAngle(0.0);	moduleD.setTareAngle(50.0);
		//competition robot: -68.0							 59.0						 -3.0						 56.0
		//practice robot:	 62.0,						 -14.0,							 0.0,						 50.0

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
//		gyro.reset();
		V_PID.clear("forward");
		V_PID.clear("strafe");
		V_PID.clear("spin");
		
		swerve.autoMode(true);
		
		V_Instructions.resetTimer();
		
		zed.addEntryListener("X", (zed, key, entry, value, flags) -> {
			actualFeetX = value.getDouble();
			updatedFeetX = true;
		}, EntryListenerFlags.kUpdate);
		zed.addEntryListener("Y", (zed, key, entry, value, flags) -> {
			actualFeetY = value.getDouble();
			updatedFeetY = true;
		}, EntryListenerFlags.kUpdate);
	}
	
	@Override
	public void teleopInit() {
//		gyro.reset();
		
		swerve.autoMode(false);
		elevatorOne.setInches(elevatorOne.getInches());
		elevatorTwo.setInches(elevatorTwo.getInches());
		lockedAngle = gyro.getCurrentAngle();
		V_PID.clear("spin");
	}
	
	@Override
	public void testInit() {
	}
	
	@Override
	public void disabledInit() {
	}
	
	@Override
	public void robotPeriodic() {
		faraday.getEntry("gyro").setNumber(gyro.getCurrentAngle());
		faraday.getEntry("pressure").setNumber(pressureGauge.getAverageVoltage()*39.875 - 54.375);
		faraday.getEntry("zeroed elevators").setBoolean(elevatorOne.knowsZero && elevatorTwo.knowsZero);
		faraday.getEntry("clamp open").setBoolean(clamp.isOpen());
		faraday.getEntry("climbing mode").setBoolean(elevators.inClimbingMode());
		faraday.getEntry("browning out").setBoolean(RobotController.isBrownedOut());//TODO use this elsewhere
		faraday.getEntry("TX2 Powered On").setBoolean(tx2PowerSensor.get());
		//TODO put whether we have a cube or not
	}
	
	
	private static double prev_x = 0.0;
	private static double prev_y = 0.0;
	private static double prev_orient = 0.0;
	private static int counter = 0;
	@Override
	public void autonomousPeriodic() {
		if (updatedFeetX && updatedFeetY) {
			counter++;
			SmartDashboard.putNumber("counter", counter);
			
			double desiredX = prev_x;//feet
			double desiredY = prev_y;//feet
			double desiredOrientation = prev_orient;//degrees
			
			final double errorY = desiredY - actualFeetY;//feet
			final double errorX = desiredX - actualFeetX;//feet
			final double errorOrientation = gyro.wornPath(desiredOrientation);//degrees
			
			final double errorDirection = Math.toDegrees(Math.atan2(errorX, errorY));
			final double errorMagnitude = Math.sqrt(errorX*errorX + errorY*errorY);
			
			SmartDashboard.putNumber("errorY", errorY);
			SmartDashboard.putNumber("errorX", errorX);
			SmartDashboard.putNumber("errorOrient", errorOrientation);
			
			double speed = V_PID.get("zed", errorMagnitude);
			double spin = V_PID.get("spin", errorOrientation);
			
			if (Math.abs(speed) > 0.7) speed = 0.7*Math.signum(speed);
			if (Math.abs(spin) > 0.5) spin = 0.5*Math.signum(spin);
			SmartDashboard.putNumber("direction", errorDirection);
			
			swerve.holonomicPlain(errorDirection, speed, spin);
			
			
			prev_x = desiredX;
			prev_y = desiredY;
			prev_orient = desiredOrientation;
			updatedFeetX = false; updatedFeetY = false;
		}
		
//		final double time = V_Instructions.getSeconds();
//		if (time < 3) {
//			desiredX = 6.75 - 6.75*Math.pow(Math.cos(time*Math.PI/6.0), 2);//feet
//			desiredY = 7.0*Math.pow(Math.sin(time*Math.PI/6.0), 2);//feet
//			desiredOrientation = 0;//degrees
//		}else if (time < 6) {
//			desiredX = 5.5*Math.cos(time*Math.PI/6.0) + 6.75;//feet
//			desiredY = -7.0*Math.sin(time*Math.PI/6.0) + 7.0;//feet
//			desiredOrientation = 0;//-60.0*(time - 3.0);//degrees
//		}
	}
	
	@Override
	public void teleopPeriodic() {
		//{speed multipliers}
		final boolean turbo = driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT);
		final boolean snail = driver.getRawButton(R_Xbox.BUTTON_STICK_RIGHT);
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(R_Xbox.STICK_LEFT, true);//turbo mode
		if (!turbo) {speed *= 1.0/*0.7*/;}//-------------------------------------normal mode
		if (snail)  {speed *= 0.5;}//-------------------------------------snail mode
		speed *= speed;
		
		//{calculating spin}
		double spin = 0.7*driver.getDeadbandedAxis(R_Xbox.AXIS_RIGHT_X);//normal mode
		if (snail)  {spin  *= 0.7;	/*lockWheelsInPlace = true;*/}//----------snail mode//TODO this boolean should do what the hacked holonomic thing did
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
		
		swerve.holonomic(driver.getCurrentAngle(R_Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		
		
		//{sending to preset heights}
			 if (driver.getRawButton(R_Xbox.BUTTON_A)) {elevators.setInches(ElevatorPresets.FLOOR.height());}
		else if (driver.getRawButton(R_Xbox.BUTTON_X)) {elevators.setInches(ElevatorPresets.SWITCH.height());}
		else if (driver.getRawButton(R_Xbox.BUTTON_B)) {elevators.setInches(ElevatorPresets.SCALE_LOW.height());}
		else if (driver.getRawButton(R_Xbox.BUTTON_Y)) {elevators.setInches(ElevatorPresets.SCALE_HIGH.height());}
		
		//{incrementing downward}
		final boolean buttonLT = driver.getAxisPress(R_Xbox.AXIS_LT, 0.9);
		final boolean chillLT = V_Fridge.chill("Button LB", buttonLT, 200.0);
		if (!chillLT) {
			//it's been held down for a while, increment
			elevators.increment(-0.7*driver.getRawAxis(R_Xbox.AXIS_LT));
		}else if (V_Fridge.becomesTrue("!Button LB", !buttonLT) && chillLT) {
			elevators.increment(-11.0);//inches
		}
		
		//{incrementing upward}
		final boolean buttonRT = driver.getAxisPress(R_Xbox.AXIS_RT, 0.9);
		final boolean chillRT = V_Fridge.chill("Button RB", buttonRT, 200.0);
		if (!chillRT) {
			//it's been held down for a while, increment
			elevators.increment(0.7*driver.getRawAxis(R_Xbox.AXIS_RT));
		}else if (V_Fridge.becomesTrue("!Button RB", !buttonRT) && chillRT) {
			elevators.increment(11.0);//inches
		}

		
		if (elevators.inClimbingMode() != V_Fridge.freeze("Button Start", driver.getRawButton(R_Xbox.BUTTON_START))) {//CLIMBING MODE
			if (elevators.inClimbingMode()) elevators.disableClimbMode(clamp);
			else elevators.enableClimbMode(clamp);
		}
		
		
		if (driver.getRawButton(R_Xbox.BUTTON_LB)/* && !clamp.hasCube()*/) {//CLAMP SLURP AND SPIT
			clamp.slurp();
		}else if (driver.getRawButton(R_Xbox.BUTTON_RB)) {
			clamp.spit();
		}else {
			clamp.stop();
		}

		if (!elevators.inClimbingMode()) {
			if (V_Fridge.freeze("STICKLEFTBUTTON", driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT))) {//CLAMP OPEN AND CLOSE
				clamp.open();
			}else { 
				clamp.close();
			}
		}
		
		
		if (driver.getRawButton(R_Xbox.BUTTON_BACK)) {//GYRO RESET
			gyro.reset();
			lockedAngle = gyro.getCurrentAngle();
			V_PID.clear("spin");
		}
		
		if (gyro.netAcceleration() >= 1) {//DANGER RUMBLE
			driver.setRumble(RumbleType.kLeftRumble, 1);
		}else {
			driver.setRumble(RumbleType.kLeftRumble, 0);
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
		elevatorOne.setZero(0.0);
		elevatorTwo.setZero(0.0);
		elevators.setInches(0.0);
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
