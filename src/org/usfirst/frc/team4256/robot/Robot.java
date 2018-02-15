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
	private static double desiredElevatorHeight = ElevatorPresets.FLOOR.height();
	private static double lockedAngle = 0;
	//{Robot Input}
	private static final R_Gyro gyro = new R_Gyro(Parameters.Gyrometer_updateHz, 0, 0);
//	private static NetworkTableInstance nt;
//	private static NetworkTable faraday;
//	private static NetworkTable targeting;
//	private static NetworkTable zed;
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
	private static final R_ElevatorOne elevatorOne = new R_ElevatorOne(Parameters.ElevatorOne_master, Parameters.ElevatorOne_followerA, Parameters.ElevatorOne_followerB, elevatorOneShifter, Parameters.ElevatorOne_calibrator);
	private static final R_ElevatorTwo elevatorTwo = new R_ElevatorTwo(Parameters.ElevatorTwo_master, Parameters.ElevatorTwo_calibrator);
	private static final R_Elevators elevators = new R_Elevators(elevatorOne, elevatorTwo);
	
	private static final DoubleSolenoid clampShifter = new DoubleSolenoid(Parameters.Clamp_module, Parameters.Clamp_forward, Parameters.Clamp_reverse);
	private static final R_Clamp clamp = new R_Clamp(Parameters.Intake_left, Parameters.Intake_right, clampShifter, 0);
	
	private static final DigitalOutput tx2PowerControl = new DigitalOutput(9);
//	private Thread thread;
	@Override
	public void robotInit() {
//		nt = NetworkTableInstance.getDefault();
		//{Robot Input}
//		faraday = nt.getTable("Faraday");
//		targeting = nt.getTable("Targeting");
//		zed = nt.getTable("ZED");
		//{Robot Output}
		compressor.clearAllPCMStickyFaults();
		swerve.init();
		elevators.init();
		clamp.init();
//		climberB.setVoltageCompensationRampRate(24); //TODO
//		lift.setVoltageRampRate(8);

		tx2PowerControl.set(true);
		try {Thread.sleep(35);}//milliseconds
		catch (InterruptedException e) {Thread.currentThread().interrupt();}
		tx2PowerControl.set(false);
	}

	@Override
	public void autonomousInit() {
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
//		faraday.putBoolean("old gear out", gearer.get().equals(DoubleSolenoid.Value.kForward));
//		faraday.putBoolean("clamp open", clamp.get().equals(DoubleSolenoid.Value.kForward));
//		faraday.putBoolean("aligning", swerve.isAligning());
//		faraday.putBoolean("aligned", swerve.isAligned());
//		faraday.putNumber("match timer", V_Instructions.getSeconds());
	}
	
	@Override
	public void autonomousPeriodic() {
//		elevatorOne.setZero(0.0);//TODO zero elevators at some point
//		elevatorTwo.setZero(0.0);
	}
	
	@Override
	public void teleopPeriodic() {
		//{speed multipliers}
		final boolean turbo = driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT);
		final boolean snail = driver.getRawButton(R_Xbox.BUTTON_STICK_RIGHT);
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(R_Xbox.STICK_LEFT, true);//turbo mode
		if (!turbo) {speed *= 0.6;}//-------------------------------------normal mode
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
		
		
		if (driver.getRawButton(R_Xbox.BUTTON_A)) {desiredElevatorHeight = ElevatorPresets.FLOOR.height();}//ELEVATOR PRESETS
		if (driver.getRawButton(R_Xbox.BUTTON_X)) {desiredElevatorHeight = ElevatorPresets.SWITCH.height();}
		if (driver.getRawButton(R_Xbox.BUTTON_B)) {desiredElevatorHeight = ElevatorPresets.SCALE_LOW.height();}
		if (driver.getRawButton(R_Xbox.BUTTON_Y)) {desiredElevatorHeight = ElevatorPresets.SCALE_HIGH.height();}
		
		if (driver.getPOV() == R_Xbox.POV_NORTH) {//ELEVATOR FINE-TUNING
			desiredElevatorHeight += 0.25;
		}else if (driver.getPOV() == R_Xbox.POV_SOUTH) {
			desiredElevatorHeight -= 0.25;
		}
		
		elevators.setInches(desiredElevatorHeight);
		
		
		if (driver.getAxisPress(R_Xbox.AXIS_RT, 0.5) && !clamp.hasCube()) {//CLAMP SLURP AND SPIT
			clamp.slurp();
		}else if (driver.getAxisPress(R_Xbox.AXIS_LT, 0.5)) {
			clamp.spit();
		}else {
			clamp.stop();
		}

		if (V_Fridge.freeze("BUTTON_RB", driver.getRawButton(R_Xbox.BUTTON_RB))) {//
			clamp.close();
		}else { 
			clamp.open();
		}
		
		if (driver.getRawButton(R_Xbox.BUTTON_START)) {//SWERVE ALIGNMENT
			moduleA.setTareAngle(100.5);	moduleB.setTareAngle(39.0);	moduleC.setTareAngle(-36.5);	moduleD.setTareAngle(-8.0);
			//practice robot:	 100.5,							 39.0,						 -36.5,							 -8.0
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
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
