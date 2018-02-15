package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Clamp {
	private VictorSPX intakeLeft;
	private VictorSPX intakeRight;
	private DoubleSolenoid clamp;
	private DoubleSolenoid look;
	private DigitalInput sensor;
	
	private boolean isClosed = true;
	private boolean hasCube = true;
	private boolean isLookingUp = true;
	private double intakeConstant = 0.5;//TODO test
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clamp, final int sensorID) {
		intakeLeft = new VictorSPX(intakeLeftID);
		intakeRight = new VictorSPX(intakeRightID);
		this.clamp = clamp;
		sensor = new DigitalInput(sensorID);
	}
	
	public void init() {
		//TODO
	}
	
	/**
	 * This function "slurps" (intakes) the cube into "clampy"
	**/
	public void slurp() {
		intakeLeft.set(ControlMode.PercentOutput, -intakeConstant);//TODO negative?
		intakeRight.set(ControlMode.PercentOutput, -intakeConstant);//TODO negative?
		
		hasCube = sensor.get();//TODO may need to be !
	}
	
	/**
	 * This function "spits" (outakes) the cube out of "clampy"
	**/
	public void spit() {
		intakeLeft.set(ControlMode.PercentOutput, intakeConstant);//TODO positive?
		intakeRight.set(ControlMode.PercentOutput, intakeConstant);//TODO positive?
		
		hasCube = sensor.get();
	}
	
	public void stop() {
		intakeLeft.set(ControlMode.PercentOutput, 0);//TODO positive?
		intakeRight.set(ControlMode.PercentOutput, 0);//TODO positive?
	}
	
	/**
	 * This function closes the clamp 
	**/
	public void close() {
		clamp.set(DoubleSolenoid.Value.kReverse);//TODO test
		isClosed = true;
	}
	
	/**
	 * This function opens the clamp 
	**/
	public void open() {
		clamp.set(DoubleSolenoid.Value.kForward);//TODO test
		isClosed = false;
	}
	
	/**
	 * This function makes the clamp look up
	 **/
	public void lookUp() {
		look.set(DoubleSolenoid.Value.kForward);
		isLookingUp = true;
	}
	
	/**
	 * This function makes the clamp look straight
	 **/
	public void lookStraight() {
		look.set(DoubleSolenoid.Value.kReverse);
		isLookingUp = false;
	}
	
	/**
	 * This function returns if the clamp is looking up or straight
	 **/
	public boolean isLookingUp() {
		return isLookingUp;
	}
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isClosed() {
		return isClosed;
	}
	
	/**
	 * This function returns it has a cube or not
	**/
	public boolean hasCube() {
		return hasCube;
	}

}