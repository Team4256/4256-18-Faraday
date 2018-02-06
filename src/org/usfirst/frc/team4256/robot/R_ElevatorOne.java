package org.usfirst.frc.team4256.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_ElevatorOne {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 2.873*Math.PI;//inches
	private static final double maximumHeight = 48.0;//inches
	private R_Talon master;
	private VictorSPX followerA;
	private VictorSPX followerB;
	private DoubleSolenoid shifter;
	private DigitalInput limitSwitch;
	private boolean isLowGear = true;
	private boolean knowsZero = false;
	private int maximumEncoderValue;

	public R_ElevatorOne(final int masterID, final int followerAID, final int followerBID, final DoubleSolenoid shifter, final int limitSwitchPort) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, false);
		followerA = new VictorSPX(followerAID);
		followerB = new VictorSPX(followerBID);
		
		this.shifter = shifter;	
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * 
	**/
	public void init() {
		master.setNeutralMode(R_Talon.coast);//TODO which works better (brake or coast)?
		master.configForwardSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.init();
		followerA.follow(master);
		followerB.follow(master);
	}
	
	/**
	 * This function sets the elevator to a certain revolution value using PID.
	**/
	public void setRevs(final double revs) {
		master.quickSet(revs, false);
	}
	
	public double getRevs() {
		return master.getCurrentRevs();
	}
	
	/**
	 * This function sets the elevator to a certain inch value value using PID.
	**/
	public void setInches(final double inches) {
		setRevs(inchesToRevs(inches));
	}
	
	public double getInches() {
		return revsToInches(getRevs());
	}
	
	/**
	 * 
	**/
	public void increment(final double revs) {
		master.quickSet(master.getCurrentRevs() + revs, false);
	}
	
	/**
	 *
	**/
	public void zero() {
		if(!limitSwitch.get()) {
			master.overrideSoftLimitsEnable(true);
			knowsZero = false;
			increment(-0.3);
		}else {
			master.quickSet(0, false);
			master.setSelectedSensorPosition(0, 0, R_Talon.kTimeoutMS);
			master.configReverseSoftLimitThreshold(0, R_Talon.kTimeoutMS);//assuming negative motor voltage results in downward motion (might need to be reversed)
			master.configForwardSoftLimitThreshold(maximumEncoderValue, R_Talon.kTimeoutMS);
			master.overrideSoftLimitsEnable(false);
			
			knowsZero = true;
		}
	}
	
	/**
	 * 
	**/
	public boolean knowsZero() {
		return knowsZero;
	}
	
	
	/**
	 * 
	**/
	public void completeLoopUpdate() {
		master.completeLoopUpdate();
	}
	
	/**
	 * 
	**/
	public void shiftLowGear() {
		shifter.set(DoubleSolenoid.Value.kForward);
		isLowGear = true;
	}
	
	/**
	 * 
	**/
	public void shiftHighGear() {
		shifter.set(DoubleSolenoid.Value.kReverse);
		isLowGear = false;
	}
	
	/**
	 * 
	**/
	public boolean isLowGear() {
		return isLowGear;
	}
	
	private static double inchesToRevs(final double inches) {
		return inches/sprocketCircumference;
	}
	
	private static double revsToInches(final double revs) {
		return sprocketCircumference*revs;
	}

}
