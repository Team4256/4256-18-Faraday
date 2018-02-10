package org.usfirst.frc.team4256.robot.Elevators;

import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;

import edu.wpi.first.wpilibj.DigitalInput;

public class R_ElevatorTwo {
	private static final double gearRatio = 1.0;
	private static final double sprocketCircumference = 1.29*Math.PI;//inches
	public static final double maximumHeight = 48.0;//inches
	private R_Talon master;
	private DigitalInput sensor;
	private boolean knowsZero = false;
	private int maximumEncoderValue;
	
	public R_ElevatorTwo(final int masterID, final int sensorID) {
		master = new R_Talon(masterID, gearRatio, R_Talon.position, R_Encoder.OEM_QUAD, false);
		sensor = new DigitalInput(sensorID);
		
		maximumEncoderValue = (int)master.convert.from.REVS.afterGears(inchesToRevs(maximumHeight));
	}
	
	/**
	 * 
	**/
	public void init() {
		master.setNeutralMode(R_Talon.coast);//TODO which works better?
		master.configForwardSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.configReverseSoftLimitEnable(true, R_Talon.kTimeoutMS);
		master.init();
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
		setRevs(inchesToRevs(inches));
	}
	
	/**
	 * 
	**/
	public double getInches() {
		return revsToInches(getRevs());
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
	public void zero() {
		if(!sensor.get()) {
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
