package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_Elevator {
	
	private R_Talon master;
	private R_Talon followerA;
	private R_Talon followerB;
	private double gearRatio = 1.0; //TODO
	private DigitalInput limitSwitch;
	private DoubleSolenoid shifter;
	private boolean isLowGear = true; 
	private boolean knowsZero = false;

	public R_Elevator(final int deviceIDMaster, final int deviceIDFollowerA, final int deviceIDFollowerB, final DoubleSolenoid shifter, final int limitSwitchID) {
		
		master = new R_Talon(deviceIDMaster, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, false);
		followerA = new R_Talon(deviceIDFollowerA, gearRatio, R_Talon.follower);
		followerB = new R_Talon(deviceIDFollowerB, gearRatio, R_Talon.follower);
		
		this.shifter = shifter;
		
		limitSwitch = new DigitalInput(limitSwitchID);
		
	}
	
	/**
	//
	**/
	public void init() {
		followerA.set(ControlMode.Follower, master.getDeviceID());
		followerB.set(ControlMode.Follower, master.getDeviceID());
	}
	
	/**
	//	This function sets the elevator to a certain revolution value using PID. 
	**/
	public void setRevs(double revs) {
		master.set(master.convert.from.REVS.afterGears(gearRatio, revs), false, true);
	}
	
	/**
	// This function sets the elevator to a certain inch value value using PID. 
	**/
	public void setInches(double inches) {
		master.set(master.convert.from.REVS.afterGears(gearRatio, inches/(24*.35)), false, true);
	}
	
	/**
	//
	**/
	public void increment(double revs) {
		master.set(getPosition() + revs, false, true);
	}
	
	
	
	/**
	// 
	**/
	public double getPosition() {
		return master.convert.from.REVS.afterGears(gearRatio, master.getSelectedSensorPosition(0));
	}
	
	/**
	//
	**/
	public void zero() {
		if(!limitSwitch.get()) {
			knowsZero = false;
			master.quickSet(-0.3);
		}else {
			master.quickSet(0);
			master.getSensorCollection().setQuadraturePosition(0, R_Talon.kTimeoutMS);
			knowsZero = true;
		}
	}
	
	/**
	// 
	**/
	public boolean knowsZero() {
		return knowsZero;
	}
	
	
	/**
	// 
	**/
	public void completeLoopUpdate() {
		master.completeLoopUpdate();
	}
	
	/**
	// 
	**/
	public void shiftLowGear() {
		shifter.set(DoubleSolenoid.Value.kReverse);
		isLowGear = true;
	}
	
	/**
	// 
	**/
	public void shiftHighGear() {
		shifter.set(DoubleSolenoid.Value.kForward);
		isLowGear = false;
	}
	
	/**
	//
	**/
	public boolean isLowGear() {
		return isLowGear;
	}	 

}
