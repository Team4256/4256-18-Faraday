package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.cyborgcats.reusable.Subsystem;
import com.cyborgcats.reusable.Phoenix.Encoder;
import com.cyborgcats.reusable.Phoenix.Talon;
import com.cyborgcats.reusable.Phoenix.Victor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Clamp implements Subsystem {//TODO make a private subclass for the rotator and a private subclass for intaking and combine them; then double check auto-slurp
	private enum CubePosition {Absent, WithinReach, Present;}
	private static final DoubleSolenoid.Value CloseState = DoubleSolenoid.Value.kForward;
	private static final DoubleSolenoid.Value OpenState = DoubleSolenoid.Value.kReverse;
	private CubePosition cubePosition = CubePosition.Absent;
	private Victor intakeLeft;
	private Victor intakeRight;
	private DoubleSolenoid clamp;
	private Talon rotator;
	private AnalogInput ultrasonic;
	private int counter_hasCube = 0;
	private static double currentSetpoint = 0.0;
	public boolean knowsZero = false;
	
	public static final double intakeConstant = 0.85;
	
	public Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final int rotatorID, final int ultrasonicPort) {
		intakeLeft = new Victor(intakeLeftID, Victor.percent);
		intakeRight = new Victor(intakeRightID, Victor.percent);
		this.clamp = clamp;
		rotator = new Talon(rotatorID, 1.0, ControlMode.Position, Encoder.CTRE_MAG_ABSOLUTE, true, 110.0, 230.0);
		ultrasonic = new AnalogInput(ultrasonicPort);
	}
	
	public void init() {
		intakeLeft.init();
		intakeRight.init();
		rotator.setInverted(false);
		rotator.setNeutralMode(Talon.brake);
		
		rotator.config_kP(0, 0.2, Talon.kTimeoutMS);
		rotator.config_kI(0, 0.0, Talon.kTimeoutMS);
		rotator.config_kD(0, 0.0, Talon.kTimeoutMS);
		rotator.config_kP(1, 2.5, Talon.kTimeoutMS);
		rotator.config_kI(1, 0.0, Talon.kTimeoutMS);
		rotator.config_kD(1, 80.0, Talon.kTimeoutMS);
	}
	
	
	/**
	 * This function attempts to "slurp" nearby cubes into the clamp.
	**/
	public void slurp() {
		//{do stuff based on the enum}
		switch (cubePosition) {
		case Absent://open and begin intaking
			open();
			setWheelSpeed(-intakeConstant);
			break;
		case WithinReach://hug cube
			close();
			break;
		case Present://stop intaking
			stop();
			break;
		}
	}
	
	
	/**
	 * This function attempts to "spit" cubes out of the clamp.
	**/
	public void spit(final double strength) {
		setWheelSpeed(Math.abs(strength));
	}
	
	
	/**
	 * This function stops the clamp motors (thereby stopping slurp and spit).
	**/
	public void stop() {setWheelSpeed(0.0);}
	
	
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
		counter_hasCube = 0;
	}
	
	
	/**
	 * This function closes the clamp
	**/
	public void close() {clamp.set(CloseState);}
	
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isOpen() {return clamp.get().equals(OpenState);}
	
	public boolean hasCube() {return cubePosition.equals(CubePosition.Present);}
	
	
	/**
	 * This function returns if the cube is in the clamp or not.
	 **/
	private boolean cubeLikelyPresent() {
		if (ultrasonic.getAverageValue() <= 65) counter_hasCube++;
		else counter_hasCube -= 2;
		counter_hasCube = Math.max(0, Math.min(counter_hasCube, 25));
		return counter_hasCube == 25;
	}
	
	
	/**
	 * This function returns if the cube is in reach of the clamp or not.
	**/
	private boolean cubeInReach() {return ultrasonic.getAverageValue() <= 80;}
	
	/**
	 * This function checks if the rotator is within a threshold of the desired angle.
	 * Threshold should be specified in degrees.
	**/
	public boolean isThere(final double threshold) {return Math.abs(rotator.getCurrentError(true)) <= threshold;}
	
	/**
	 * This function defines zero for the rotator.
	**/
	public void setZero() {
		rotator.setSelectedSensorPosition((int)rotator.convert.from.DEGREES.afterGears(90.0), 0, Talon.kTimeoutMS);
		knowsZero = true;
	}
	
	public void increment(final double degrees) {rotateTo(currentSetpoint + degrees);}
	
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
	
	public void beginLoopUpdate() {
		//if the cube was previously in reach and the ultrasonic sensor says the cube is all the way in, update the enum
		if (cubeLikelyPresent() && !isOpen()) cubePosition = CubePosition.Present;
		//if the ultrasonic sensor says the cube is in reach at least once, update the enum
		else if (cubeInReach()) cubePosition = CubePosition.WithinReach;
		else cubePosition = CubePosition.Absent;
	}

	@Override
	public boolean perform(final String action, final double[] data) {
		switch(Abilities.valueOf(action)) {
		case OPEN: open(); return isOpen();
		case CLOSE: close(); return !isOpen();
		case SLURP: slurp(); return hasCube();
		case SPIT: spit(0.5); return true;
		case EXTEND: rotateTo(0.0); return true;
		default: throw new IllegalStateException("The clamp cannot " + action);
		}
	}
	
	public static enum Abilities {OPEN, CLOSE, SLURP, SPIT, EXTEND}
}
