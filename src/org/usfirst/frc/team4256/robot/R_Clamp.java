package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.cyborgcats.reusable.Phoenix.R_Encoder;
import com.cyborgcats.reusable.Phoenix.R_Talon;
import com.cyborgcats.reusable.Phoenix.R_Victor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Clamp {
	private enum CubePosition {Absent, WithinReach, Present;}
	private static final DoubleSolenoid.Value CloseState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OpenState = DoubleSolenoid.Value.kReverse;
	private CubePosition cubePosition = CubePosition.Absent;
	private R_Victor intakeLeft;
	private R_Victor intakeRight;
	private DoubleSolenoid clamp;
	private R_Talon rotator;
	private AnalogInput ultrasonic;
	private static int counter = 0;
	private static double currentSetpoint = 0.0;
	public boolean knowsZero = false;
	
	public static final double intakeConstant = 0.85;
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final int rotatorID, final int ultrasonicPort) {
		intakeLeft = new R_Victor(intakeLeftID, R_Victor.percent);
		intakeRight = new R_Victor(intakeRightID, R_Victor.percent);
		this.clamp = clamp;
		rotator = new R_Talon(rotatorID, 1.0, ControlMode.Position, R_Encoder.CTRE_MAG_ABSOLUTE, true, 110.0, 230.0);//false for practice
		ultrasonic = new AnalogInput(ultrasonicPort);
	}
	
	public void init() {
		intakeLeft.init();
		intakeRight.init();
		rotator.setInverted(false);
		rotator.setNeutralMode(R_Talon.brake);
		
		rotator.config_kP(0, 0.2, R_Talon.kTimeoutMS);
		rotator.config_kI(0, 0.0, R_Talon.kTimeoutMS);
		rotator.config_kD(0, 0.0, R_Talon.kTimeoutMS);
		rotator.config_kP(1, 2.5, R_Talon.kTimeoutMS);
		rotator.config_kI(1, 0.0, R_Talon.kTimeoutMS);
		rotator.config_kD(1, 80.0, R_Talon.kTimeoutMS);
	}
	
	
	/**
	 * This function attempts to "slurp" nearby cubes into the clamp.
	**/
	public void slurp() {
		//{do stuff based on the enum}
		switch (cubePosition) {
		case Absent://open and begin intaking
			open();//TODO theoretically will open, but if ultrasonic sensor doesn't update instantaneously then it will close again right away
			setWheelSpeed(-intakeConstant);
			break;
		case WithinReach://hug cube
			close();
			break;
		case Present://stop intaking
			stop();
			break;
		}
		//if the ultrasonic sensor says the cube is in reach at least once, update the enum
		if (cubeInReach()) cubePosition = CubePosition.WithinReach;
		//if the cube was previously in reach and the ultrasonic sensor says the cube is all the way in, update the enum
		if (cubePosition.equals(CubePosition.WithinReach) && cubeLikelyPresent()) cubePosition = CubePosition.Present;
	}
	
	
	/**
	 * This function attempts to "spit" cubes out of the clamp.
	**/
	public void spit(final double strength) {
		setWheelSpeed(strength);
		cubePosition = CubePosition.Absent;
	}
	
	
	/**
	 * This function stops the clamp motors (thereby stopping slurp and spit).
	**/
	public void stop() {
		setWheelSpeed(0.0);
	}
	
	
	/**
	 * 
	**/
	private void setWheelSpeed(final double percent) {
		intakeLeft.quickSet(-percent);
		intakeRight.quickSet(percent);
	}
	
	
	/**
	 * This function opens the clamp 
	**/
	public void open() {
		clamp.set(OpenState);
		cubePosition = CubePosition.Absent;
	}
	
	
	/**
	 * This function closes the clamp 
	**/
	public void close() {
		clamp.set(CloseState);
		cubePosition = CubePosition.Present;
	}
	
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isOpen() {
		return clamp.get().equals(OpenState);
	}
	
	public boolean hasCube() {return cubePosition.equals(CubePosition.Present);}
	
	
	/**
	 * This function returns if the cube is in the clamp or not.
	 **/
	private boolean cubeLikelyPresent() {
		if (ultrasonic.getAverageValue() <= 55) counter++;
		else counter = 0;
		return counter > 10;
	}
	
	
	/**
	 * This function returns if the cube is in reach of the clamp or not.
	**/
	private boolean cubeInReach() {
		return ultrasonic.getAverageValue() <= 70;
	}
	
	/**
	 * This function checks if the rotator is within a threshold of the desired angle.
	 * Threshold should be specified in degrees.
	**/
	public boolean isThere(final double threshold) {
		return Math.abs(rotator.getCurrentError(true)) <= threshold;
	}
	
	/**
	 * This function defines zero for the rotator.
	**/
	public void setZero() {
		rotator.setSelectedSensorPosition((int)rotator.convert.from.DEGREES.afterGears(90.0), 0, R_Talon.kTimeoutMS);
		knowsZero = true;
	}
	
	public void increment(final double degrees) {
		rotateTo(currentSetpoint + degrees);
	}
	
	public void rotateTo(double desiredAngle) {
		desiredAngle = Math.min(Math.max(0.0, desiredAngle), 90.0);//clips values outside of [0.0, 90.0]
		if (desiredAngle <= 10.0) rotator.selectProfileSlot(0, 0);
		else rotator.selectProfileSlot(1, 0);
		rotator.quickSet(desiredAngle, true);
		currentSetpoint = desiredAngle;
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	public void completeLoopUpdate() {
		intakeLeft.completeLoopUpdate();
		intakeRight.completeLoopUpdate();
	}
}
