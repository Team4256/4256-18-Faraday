package org.usfirst.frc.team4256.robot.Elevators;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class R_ElevatorOne {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 2.873*Math.PI;//inches
	public static final double maximumHeight = 40.0;//inches
	private R_Talon master;
	private VictorSPX followerA;
	private VictorSPX followerB;
	private DoubleSolenoid shifter;
	private DigitalInput sensor;
	private boolean isLowGear = true;
	private boolean knowsZero = false;
	private int maximumEncoderValue;

	public R_ElevatorOne(final int masterID, final int followerAID, final int followerBID, final DoubleSolenoid shifter, final int sensorID) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, false);
		followerA = new VictorSPX(followerAID);
		followerB = new VictorSPX(followerBID);
		sensor = new DigitalInput(sensorID);
		this.shifter = shifter;	
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * 
	**/
	public void init() {
		master.init();
		
		master.setNeutralMode(R_Talon.coast);//TODO which works better (brake or coast)?
		enableSoftLimits();

		followerA.follow(master);
		followerB.follow(master);
	}
	
	/**
	 * This function sets the elevator to a certain revolution value using PID.
	**/
	public void setRevs(final double revs) {
		master.quickSet(revs, false);
	}
	
	/**
	 * 
	**/
	public double getRevs() {
		return master.getCurrentRevs();
	}
	
	/**
	 * This function sets the elevator to a certain inch value value using PID.
	**/
	public void setInches(final double inches) {
		setRevs(inchesToRevs(validateInches(inches)));
	}
	
	/**
	 * 
	**/
	public double getInches() {
		return revsToInches(getRevs());
	}
	
	public double validateInches(final double inches) {
		if (inches > maximumHeight) {
			return maximumHeight;
		}else if (inches < 0.0) {
			return 0.0;
		}else {
			return inches;
		}
	}
	
	/**
	 * 
	**/
	public void increment(final double inches) {
		setInches(getInches() + inches);
	}
	
	/**
	 *
	**/
	public void findZero() {
		if(!sensor.get()) {//not at zero
			master.overrideSoftLimitsEnable(true);
			knowsZero = false;
			increment(-0.3);
		}else {//at zero
			setZero(0.0);
			master.overrideSoftLimitsEnable(false);
			
			knowsZero = true;
		}
	}
	
	public void setZero(final double offsetInchesFromCurrent) {
		master.setSelectedSensorPosition(0 + (int)master.convert.from.REVS.afterGears(inchesToRevs(offsetInchesFromCurrent)), 0, R_Talon.kTimeoutMS);
	}
	
	public void enableSoftLimits() {
		master.configForwardSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitThreshold(0, R_Talon.kTimeoutMS);//assuming negative motor voltage results in downward motion
		master.configForwardSoftLimitThreshold(maximumEncoderValue, R_Talon.kTimeoutMS);
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
	
	/**
	 * 
	**/
	private static double inchesToRevs(final double inches) {
		return inches/sprocketCircumference;
	}
	
	/**
	 * 
	**/
	private static double revsToInches(final double revs) {
		return sprocketCircumference*revs;
	}
}
