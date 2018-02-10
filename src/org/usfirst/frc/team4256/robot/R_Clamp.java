package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Clamp {
	
	private VictorSPX intakeLeft;
	private VictorSPX intakeRight;
	private DoubleSolenoid clampRight;
	private DoubleSolenoid clampLeft;
	private DigitalInput sensor;
	private boolean isClampClosed = true;//TODO private?
	private boolean isSlurped = true;//TODO private?
	private double intakeConstant = 0.5;//TODO test
	
	public R_Clamp(final int intakeLeftID, final int intakeRightID, final DoubleSolenoid clampLeft, final DoubleSolenoid clampRight, final int sensorID) {
		intakeLeft = new VictorSPX(intakeLeftID);
		intakeRight = new VictorSPX(intakeRightID);
		this.clampRight = clampRight;
		this.clampLeft = clampLeft;
		sensor = new DigitalInput(sensorID);
	}
	
	/**
	 * This function "slurps" (intakes) the cube into "clampy"
	**/
	public void slurp() {
		if(!sensor.get()) {
			intakeLeft.set(ControlMode.PercentOutput, -intakeConstant);//TODO negative?
			intakeRight.set(ControlMode.PercentOutput, -intakeConstant);//TODO negative?
			isSlurped = false;
		}else {
			intakeLeft.set(ControlMode.PercentOutput, 0);
			intakeRight.set(ControlMode.PercentOutput, 0);
			isSlurped = true;
		}
		
	}
	
	/**
	 * This function "spits" (outakes) the cube out of "clampy"
	**/
	public void spit() {
		intakeLeft.set(ControlMode.PercentOutput, intakeConstant);//TODO positive?
		intakeRight.set(ControlMode.PercentOutput, intakeConstant);//TODO positive?
	}
	
	/**
	 * This function closes the clamp 
	**/
	public void closeClamp() {
		clampLeft.set(DoubleSolenoid.Value.kReverse);//TODO test
		clampRight.set(DoubleSolenoid.Value.kReverse);//TODO test
		isClampClosed = false;
	}
	
	/**
	 * This function opens the clamp 
	**/
	public void openClamp() {
		clampLeft.set(DoubleSolenoid.Value.kForward);//TODO test
		clampRight.set(DoubleSolenoid.Value.kForward);//TODO test
		isClampClosed = true;
	}
	
	/**
	 * This function returns if the clamp is closed or not
	**/
	public boolean isClampClosed() {
		return isClampClosed;
	}
	
	/**
	 * This function returns if a cube is slurped or not
	**/
	public boolean isSlurped() {
		return isSlurped;
	}

}
