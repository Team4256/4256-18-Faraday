//DRIVER
//start + back: align
//left stick, both axis: raw speed and direction
//right stick, x axis: raw spin
//left stick, press: snail mode drive
//right stick, press: snail mode spin
//LB: 
//LT: increment elevator down
//RB: turbo mode drive
//RT: increment elevator up
//X: elevator low scale preset
//A: elevator floor preset
//B: elevator switch preset
//Y: elevator high scale preset
//dpad down: 

//GUNNER
//start + back: gyro reset

package org.usfirst.frc.team4256.robot;

import org.usfirst.frc.team4256.robot.Parameters.ElevatorPresets;
import org.usfirst.frc.team4256.robot.Elevators.*;

import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.R_Xbox;
import com.cyborgcats.reusable.V_Fridge;
import com.cyborgcats.reusable.V_PID;
import org.usfirst.frc.team4256.robot.R_Clamp;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final R_Xbox driver = new R_Xbox(0);
	private static double lockedAngle = 0;
	//{Robot Input}
	private static final R_Gyro gyro = new R_Gyro(Parameters.Gyrometer_updateHz, 0, 0);
	private static NetworkTableInstance nt;
	private static NetworkTable faraday;
	private static NetworkTable targeting;
	private static NetworkTable zed;
//	private static double metersX = 0;
//	private static double metersY = 0;
	
	//{Robot Output}
	private static final Compressor compressor = new Compressor(0);
	
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
	private static final R_Clamp clamp = new R_Clamp(Parameters.Intake_left, Parameters.Intake_right, clampShifter, extenderShifter);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(9);
	
	@Override
	public void robotInit() {
		//{Robot Input}
		nt = NetworkTableInstance.getDefault();
		faraday = nt.getTable("Faraday");
		targeting = nt.getTable("Targeting");
		zed = nt.getTable("ZED");
		//{Robot Output}
		compressor.clearAllPCMStickyFaults();
		swerve.init();
		elevators.init();
		clamp.init();
//		climberB.setVoltageCompensationRampRate(24); //TODO
//		lift.setVoltageRampRate(8);
		
		moduleA.setTareAngle(60.0);	moduleB.setTareAngle(-14.0);	moduleC.setTareAngle(0.0);	moduleD.setTareAngle(50.0);
		//practice robot:	 60.0,						 -14.0,							 0.0,						 50.0

		tx2PowerControl.set(true);
		try {Thread.sleep(50);}//milliseconds
		catch (InterruptedException e) {Thread.currentThread().interrupt();}
		tx2PowerControl.set(false);
	}

	@Override
	public void autonomousInit() {
		elevatorOne.setInches(elevatorOne.getInches());
		elevatorTwo.setInches(elevatorTwo.getInches());
		gyro.reset();
		V_PID.clear("spin");
//		autoMode = (int)faraday.getNumber("auto mode", 1);
//		autoStep = 0;
//		V_PID.clear("forward");
//		V_PID.clear("strafe");
//		V_Instructions.resetTimer();
	}
	
	@Override
	public void teleopInit() {
		elevatorOne.setInches(elevatorOne.getInches());
		elevatorTwo.setInches(elevatorTwo.getInches());
		lockedAngle = gyro.getCurrentAngle();
		V_PID.clear("spin");
	}
	
	@Override
	public void testInit() {//TODO these numbers should come from the ZED/TK1
	}
	
	@Override
	public void disabledInit() {
	}
	
	@Override
	public void robotPeriodic() {
		//faraday.getEntry("aligned").setBoolean(elevators.knowsZero);
		faraday.getEntry("clamp open").setBoolean(clamp.isOpen());
		faraday.getEntry("climbing mode").setBoolean(elevatorOne.hasLotsOfTorque());
	}
	
	@Override
	public void autonomousPeriodic() {
		elevatorOne.setZero(0.0);//TODO zero elevators at some point
		elevatorTwo.setZero(0.0);
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
		if (snail)  {spin  *= 0.7;	/*lockWheelsInPlace = true;*/}//----------snail mode//TODO this boolean should do what the hacked holonomic thing did
		spin *= spin*Math.signum(spin);
		final boolean handsOffSpinStick = spin == 0.0;
		
		//{adding driver aids}
		if (handsOffSpinStick) {
			double spinError = 0;
			//stop rotation drift at high speeds
			if (speed >= 0.3) {spinError = gyro.wornPath(lockedAngle);}
			if (Math.abs(spinError) > 3.0) {spin = V_PID.get("spin", spinError);}
		}
		if (V_Fridge.becomesTrue("hands off", handsOffSpinStick)) {
			//remember angle when driver stops rotating
			lockedAngle = gyro.getCurrentAngle();
			V_PID.clear("spin");
		}
		
		swerve.holonomic(driver.getCurrentAngle(R_Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		
		
//		double desiredElevatorHeight = elevators.getInches();
		//{sending to preset heights}
		if (driver.getRawButton(R_Xbox.BUTTON_A)) {elevators.setInches(ElevatorPresets.FLOOR.height());}
		if (driver.getRawButton(R_Xbox.BUTTON_X)) {elevators.setInches(ElevatorPresets.SWITCH.height());}
		if (driver.getRawButton(R_Xbox.BUTTON_B)) {elevators.setInches(ElevatorPresets.SCALE_LOW.height());}
		if (driver.getRawButton(R_Xbox.BUTTON_Y)) {elevators.setInches(ElevatorPresets.SCALE_HIGH.height());}
//		elevators.setInches(desiredElevatorHeight);
		
		//{incrementing downward}
		final boolean buttonLT = driver.getAxisPress(R_Xbox.AXIS_LT, 0.9);
		final boolean chillLT = V_Fridge.chill("Button LB", buttonLT, 200.0);
		if (!chillLT) {
			//it's been held down for a while, increment
			elevators.increment(-0.75*driver.getRawAxis(R_Xbox.AXIS_LT));
		}else if (V_Fridge.becomesTrue("!Button LB", !buttonLT) && chillLT) {
			elevators.increment(-11.0);//inches
		}
		
		//{incrementing upward}
		final boolean buttonRT = driver.getAxisPress(R_Xbox.AXIS_RT, 0.9);
		final boolean chillRT = V_Fridge.chill("Button RB", buttonRT, 200.0);
		if (!chillRT) {//it's been held down for a while, increment
			elevators.increment(0.75*driver.getRawAxis(R_Xbox.AXIS_RT));
		}else if (V_Fridge.becomesTrue("!Button RB", !buttonRT) && chillRT) {
			elevators.increment(11.0);//inches
		}

		
		if (V_Fridge.freeze("Button Start", driver.getRawButton(R_Xbox.BUTTON_START))) {
			if (!elevators.inClimbingMode()) {
				elevators.enableClimbMode(clamp);
			}
		}else {
			elevators.disableClimbMode(clamp);
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
		
		//{completing Talon updates}
		swerve.completeLoopUpdate();
		elevators.completeLoopUpdate();
		//clamp.completeLoopUpdate//TODO
	}
	
	@Override
	public void testPeriodic() {
		moduleA.swivelTo(0);
		moduleB.swivelTo(0);
		moduleC.swivelTo(0);
		moduleD.swivelTo(0);
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
