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
	
	private final double intakeConstant = 0.85;
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final int rotatorID, final int ultrasonicPort) {
		intakeLeft = new R_Victor(intakeLeftID, R_Victor.percent);
		intakeRight = new R_Victor(intakeRightID, R_Victor.percent);
		this.clamp = clamp;
		rotator = new R_Talon(rotatorID, 1.0, ControlMode.PercentOutput, R_Encoder.CTRE_MAG_ABSOLUTE, false);
		ultrasonic = new AnalogInput(ultrasonicPort);
	}
	
	public void init() {
		intakeLeft.init();
		intakeRight.init();
	}
	
	
	/**
	 * This function attempts to "slurp" nearby cubes into the clamp.
	**/
	public void slurp() {
		//if the ultrasonic sensor says the cube is in reach at least once, update the enum
		if (cubeInReach()) cubePosition = CubePosition.WithinReach;
		//if the cube was previously in reach and the ultrasonic sensor says the cube is all the way in, update the enum
		if (cubePosition.equals(CubePosition.WithinReach) && hasCube()) cubePosition = CubePosition.Present;
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
	public void spit() {
		setWheelSpeed(intakeConstant);
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
		intakeLeft.quickSet(percent);
		intakeRight.quickSet(-percent);
	}
	
	
	/**
	 * This function opens the clamp 
	**/
	public void open() {
		clamp.set(OpenState);
	}
	
	
	/**
	 * This function closes the clamp 
	**/
	public void close() {
		clamp.set(CloseState);
	}
	
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isOpen() {
		return clamp.get().equals(OpenState);
	}
	
	
	/**
	 * This function returns if the cube is in the clamp or not.
	 **/
	public boolean hasCube() {
		if (ultrasonic.getAverageValue() <= 55) counter++;
		else counter = 0;
		return counter > 10;
	}
	
	
	/**
	 * This function returns if the cube is in reach of the clamp or not.
	**/
	public boolean cubeInReach() {
		return ultrasonic.getAverageValue() <= 70;
	}
	
	/**
	 * 
	**/
	public void extend() {
	}
	
	/**
	 * 
	**/
	public void retract() {
	}
	
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the elevator.
	**/
	public void completeLoopUpdate() {
		intakeLeft.completeLoopUpdate();
		intakeRight.completeLoopUpdate();
		rotator.completeLoopUpdate();
	}
}