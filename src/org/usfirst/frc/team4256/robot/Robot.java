//DRIVER//TODO document new driver commands
//start + back: align
//left stick, both axis: raw speed and direction
//right stick, x axis: raw spin
//left stick, press: snail mode drive
//right stick, press: snail mode spin
//LB: boolean climber
//LT: toggle clamp
//RB: turbo mode (drive and climber)
//RT: toggle lift
//X: left gear orientation
//A: center gear orientation
//B: right gear orientation
//Y: loading station orientation
//dpad down: toggle gearer

//GUNNER
//start + back: gyro reset
//LT: reverse driver's climbing commands

package org.usfirst.frc.team4256.robot;

import java.util.HashMap;
import java.util.Map;

import com.cyborgcats.reusable.Talon.R_Talon;
import com.cyborgcats.reusable.R_Gyro;
import com.cyborgcats.reusable.R_Xbox;
import com.cyborgcats.reusable.V_Fridge;
import com.cyborgcats.reusable.V_PID;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {
	//{Human Input}
	private static final R_Xbox driver = new R_Xbox(0);
	private static final R_Xbox gunner = new R_Xbox(1);
	private static final Map<Integer, Double> buttons2angle = new HashMap<Integer, Double>();
	private static final int[] gearButtons = new int[] {R_Xbox.BUTTON_X, R_Xbox.BUTTON_A, R_Xbox.BUTTON_B, R_Xbox.BUTTON_Y};
	private static Long handsOffTime = System.currentTimeMillis();
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
//	private static final Compressor compressor = new Compressor(0);
	
	private static final R_SwerveModule moduleA = new R_SwerveModule(Parameters.Swerve_rotatorA,/*flipped sensor*/ false, Parameters.Swerve_driveA);
	private static final R_SwerveModule moduleB = new R_SwerveModule(Parameters.Swerve_rotatorB,/*flipped sensor*/ false, Parameters.Swerve_driveB);
	private static final R_SwerveModule moduleC = new R_SwerveModule(Parameters.Swerve_rotatorC,/*flipped sensor*/ false, Parameters.Swerve_driveC);
	private static final R_SwerveModule moduleD = new R_SwerveModule(Parameters.Swerve_rotatorD,/*flipped sensor*/ false, Parameters.Swerve_driveD, false);
	private static final R_DriveTrain swerve = new R_DriveTrain(gyro, moduleA, moduleB, moduleC, moduleD);
	
//	private static final R_Talon lift = new R_Talon(Parameters.Lift,/*gear ratio*/ 1, R_Talon.percent);//TODO may not function the same as the voltage mode
//	private static final DoubleSolenoid clamp = new DoubleSolenoid(Parameters.Clamp_module, Parameters.Clamp_forward, Parameters.Clamp_reverse);
//	private static final DoubleSolenoid gearer = new DoubleSolenoid(Parameters.Gearer_module, Parameters.Gearer_forward, Parameters.Gearer_reverse);
	
	@Override
	public void robotInit() {
//		nt = NetworkTableInstance.getDefault();
//		//{Robot Input}
//		faraday = nt.getTable("Faraday");
//		targeting = nt.getTable("Targeting");
//		zed = nt.getTable("ZED");
//		//{Robot Output}
//		compressor.clearAllPCMStickyFaults();
		swerve.init();
//		V_PID.set("forward", Parameters.forwardP, Parameters.forwardI, Parameters.forwardD);
//		V_PID.set("strafe", Parameters.strafeP, Parameters.strafeI, Parameters.strafeD);
//		V_PID.set("spin", Parameters.spinP, Parameters.spinI, Parameters.spinD);
//		climberA.init();
//		climberA.setVoltageCompensationRampRate(24);
//		climberB.init(climberA.getDeviceID(), 12f);
//		climberB.setVoltageCompensationRampRate(24);
//		lift.init();
//		lift.setVoltageRampRate(8);
	}

	@Override
	public void autonomousInit() {
		gyro.reset();
//		autoMode = (int)faraday.getNumber("auto mode", 1);
//		autoStep = 0;
//		V_PID.clear("forward");
//		V_PID.clear("strafe");
//		V_PID.clear("spin");
//		V_Instructions.resetTimer();
	}
	
	@Override
	public void teleopInit() {
//		if (DriverStation.getInstance().getAlliance() != DriverStation.Alliance.Red) {//TODO override all brake modes
//			Parameters.loadingStation += 90;
//		}
//		buttons2angle.put(R_Xbox.BUTTON_X, Parameters.leftGear);
//		buttons2angle.put(R_Xbox.BUTTON_A, Parameters.centerGear);
//		buttons2angle.put(R_Xbox.BUTTON_B, Parameters.rightGear);
//		buttons2angle.put(R_Xbox.BUTTON_Y, Parameters.loadingStation);
		V_PID.clear("spin");
		lockedAngle = gyro.getCurrentAngle();
	}
	
	@Override
	public void testInit() {//TODO these numbers should come from the ZED/TK1
//		zed.putNumber("x", 0);
//		zed.putNumber("y", 0);
//		zed.putNumber("expected x", 0);
//		zed.putNumber("expected y", 0);
//		zed.putNumber("expected angle", 0);
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
	}
	
	@Override
	public void teleopPeriodic() {
		if (driver.getRawButton(R_Xbox.BUTTON_START) && driver.getRawButton(R_Xbox.BUTTON_BACK)) {//SWERVE ALIGNMENT
			moduleA.setTareAngle(99);	moduleB.setTareAngle(39);	moduleC.setTareAngle(127);	moduleD.setTareAngle(0);
			moduleA.swivelTo(0);	moduleB.swivelTo(0);	moduleC.swivelTo(0);	moduleD.swivelTo(0);
		}
		
		if (gunner.getRawButton(R_Xbox.BUTTON_START) && gunner.getRawButton(R_Xbox.BUTTON_BACK)) {//GYRO RESET
			gyro.reset();
			lockedAngle = gyro.getCurrentAngle();
			V_PID.clear("spin");
		}
		
		//{calculating speed}
		double speed = driver.getCurrentRadius(R_Xbox.STICK_LEFT, true);//--turbo mode
		if (!driver.getRawButton(R_Xbox.BUTTON_RB)) {speed *= .6;}//--normal mode
		if (driver.getRawButton(R_Xbox.BUTTON_STICK_LEFT)) {speed *= .5;}//--snail mode
		speed *= speed;
		//{calculating raw spin}
		double spin = driver.getDeadbandedAxis(R_Xbox.AXIS_RIGHT_X);
		spin *= spin*Math.signum(spin)*.5;//--normal mode
		if (driver.getRawButton(R_Xbox.BUTTON_STICK_RIGHT)) {
			spin *= .5;//--snail mode
			if (speed == 0) {speed = .01;}//.01 restrains coast after spinning by hacking holonomic
		}
		//{adding driver aids}
		if (V_Fridge.becomesTrue("hands off", spin == 0)) {
			handsOffTime = System.currentTimeMillis();
			lockedAngle = gyro.getCurrentAngle();//remember angle when driver stops rotating
			V_PID.clear("spin");
		}if (spin == 0) {
			double spinError = 0;
			if (speed >= .3) {spinError = gyro.wornPath(lockedAngle);}//stop rotation drift at high speeds
//			int gearButton = driver.mostRecentButton(gearButtons);
//			if (driver.lastPress(gearButton) > handsOffTime) {spinError = gyro.wornPath(buttons2angle.get(gearButton));}
			if (Math.abs(spinError) > 3) {spin = V_PID.get("spin", spinError);}
		}
		
		swerve.holonomic(driver.getCurrentAngle(R_Xbox.STICK_LEFT, true), speed, spin);//SWERVE DRIVE
		
//		if (driver.getRawButton(R_Xbox.BUTTON_LB)) {//CLIMBER
//			double climbSpeed = driver.getRawButton(R_Xbox.BUTTON_RB) ? 1 : .6;
//			if (gunner.getAxisPress(R_Xbox.AXIS_LT, .5)) {climbSpeed *= -1;}
//			climberA.quickSet(climbSpeed);
//		}else {
//			climberA.quickSet(0);
//		}
//		
//		if (V_Fridge.freeze("POVSOUTH", driver.getPOV(0) == R_Xbox.POV_SOUTH)) {//GEARER
//			gearer.set(DoubleSolenoid.Value.kForward);
//		}else {
//			gearer.set(DoubleSolenoid.Value.kReverse);
//		}
//		
//		if (V_Fridge.freeze("AXISRT", driver.getAxisPress(R_Xbox.AXIS_RT, .5))) {//CLAMPER
//			clamp.set(DoubleSolenoid.Value.kForward);
//		}else {
//			clamp.set(DoubleSolenoid.Value.kReverse);
//		}
		
		
		if (gyro.netAcceleration() >= 1) {
			driver.setRumble(RumbleType.kLeftRumble, 1);//DANGER RUMBLE
		}else {
			driver.setRumble(RumbleType.kLeftRumble, 0);
		}
		
		//{completing Talon updates}
		moduleA.completeLoopUpdate();
		moduleB.completeLoopUpdate();
		moduleC.completeLoopUpdate();
		moduleD.completeLoopUpdate();
//		lift.completeLoopUpdate();
	}
	
	@Override
	public void testPeriodic() {
//		metersX = zed.getNumber("x", metersX);
//		metersY = zed.getNumber("y", metersY);
//		double expectedX = zed.getNumber("expected x", metersX);
//		double expectedY = zed.getNumber("expected y", metersY);
//		double expectedAngle = zed.getNumber("expected angle", gyro.getCurrentAngle());
//		double xError = expectedX - metersX;
//		double yError = expectedY - metersY;
//		double spinError = gyro.wornPath(expectedAngle);
//		swerve.holonomicCartesian(V_PID.get("forward", yError), V_PID.get("strafe", xError), V_PID.get("spin", spinError));
	}
	
	@Override
	public void disabledPeriodic() {
	}
}
