package com.cyborgcats.reusable.Talon;

import com.cyborgcats.reusable.V_Compass;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class R_Talon extends TalonSRX {
	public static final ControlMode current = ControlMode.Current;
	public static final ControlMode follower = ControlMode.Follower;
	public static final ControlMode percent = ControlMode.PercentOutput;
	public static final ControlMode position = ControlMode.Position;
	public static final ControlMode velocity = ControlMode.Velocity;
	public static final ControlMode disabled = ControlMode.Disabled;
	public static final NeutralMode brake = NeutralMode.Brake;
	public static final NeutralMode coast = NeutralMode.Coast;
	
	public static final int kTimeoutMS = 10;
	private ControlMode controlMode;
	private boolean updated = false;
	private double lastSetPoint = 0;
	private double lastLegalDirection = 1;
	public V_Compass compass;
	private double gearRatio;
	public Convert convert;
//	private ConvertTo convertTo;
//	private ConvertFrom convertFrom;
	//This constructor is intended for use with an encoder on a motor with limited motion.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final R_Encoder encoder, final boolean flippedSensor, final double protectedZoneStart, final double protectedZoneSize) {
		super(deviceID);
		this.gearRatio = gearRatio;
		if (getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
			switch(encoder) {
			case CTRE_MAG_ABSOLUTE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			case CTRE_MAG_RELATIVE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			default: break;
			}
		}else {
			configSelectedFeedbackSensor(encoder.type(), 0, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
		}
		setSensorPhase(flippedSensor);
		this.controlMode = controlMode;
		compass = new V_Compass(protectedZoneStart, protectedZoneSize);
		convert = new Convert(encoder.countsPerRev());
	}
	//This constructor is intended for use with an encoder on a motor which can spin freely.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final R_Encoder encoder, final boolean flippedSensor) {
		super(deviceID);
		this.gearRatio = gearRatio;
		if (getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
			switch(encoder) {
			case CTRE_MAG_ABSOLUTE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			case CTRE_MAG_RELATIVE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			default: break;
			}
		}else {
			configSelectedFeedbackSensor(encoder.type(), 0, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
		}
		setSensorPhase(flippedSensor);
		this.controlMode = controlMode;
		compass = new V_Compass(0, 0);
		convert = new Convert(encoder.countsPerRev());
	}
	//This constructor is intended for a motor without an encoder.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode) {
		super(deviceID);
		this.gearRatio = gearRatio;
		this.controlMode = controlMode;
		compass = new V_Compass(0, 0);
	}
	
	
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	 * It then gets enslaved to the motor at the specified ID.
	**/
	public void init(final int masterID, final float maxVolts) {
		//clearStickyFaults();TODO
		selectProfileSlot(0, 0);//first is motion profile slot (things like allowable error), second is PID slot ID
		configAllowableClosedloopError(0, 0, kTimeoutMS);//motion profile slot, allowable error, timeout ms
		configNominalOutputForward(0, kTimeoutMS);//minimum voltage draw
		configNominalOutputReverse(0, kTimeoutMS);
		configPeakOutputForward(Math.abs(maxVolts), kTimeoutMS);//maximum voltage draw
		configPeakOutputReverse(-Math.abs(maxVolts), kTimeoutMS);
		if (getControlMode() == follower) {
			quickSet(masterID);//TODO may need to be super set not quickSet
		}else {
			quickSet(0);//TODO may need to be super set not quickSet
		}
	}
	
	
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	**/
	public void init() {//TODO
		init(0, 1);
	}
	
	
	public double getCurrentRevs() {
		return convert.to.REVS.afterGears(gearRatio, getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
	}
	
	
	/**
	 * This function returns the current angle. If wraparound is true, the output will be between 0 and 359.999...
	**/
	public double getCurrentAngle(final boolean wraparound) {//ANGLE
		return wraparound ? V_Compass.validateAngle(convert.to.DEGREES.afterGears(gearRatio, getSelectedSensorPosition(0))) : convert.to.DEGREES.afterGears(gearRatio, getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
	}
	
	
	/**
	 * This function finds the shortest legal path from the current angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	 * If the current angle is inside the protected zone, the path goes through the previously breached border.
	**/
	public double wornPath(double endAngle) {//ANGLE
		endAngle = compass.legalizeAngle(endAngle + compass.getTareAngle());
		double startAngle = getCurrentAngle(true);
		double currentPathVector = V_Compass.path(startAngle, endAngle);
		boolean legal = compass.legalizeAngle(startAngle) == startAngle;
		if (legal) {
			currentPathVector = compass.legalPath(startAngle, endAngle);
			lastLegalDirection = Math.signum(currentPathVector);
		}else if (!legal && Math.signum(currentPathVector) != -lastLegalDirection) {
			currentPathVector = 360*Math.signum(-currentPathVector) + currentPathVector;
		}return currentPathVector;
	}
	
	
	/**
	 * This function sets the motor's output or target setpoint based on the current control mode.
	 * Current: Milliamperes
	 * Follower: ID
	 * PercentVbus: -1 to 1
	 * Position: Degrees
	 * Speed: RPM
	 * Voltage: -1 to 1 (gets scaled to -12 to 12)
	**/
	public void quickSet(final double value) {
		set(value, true, true);
	}
	
	public void set(double value, final boolean treatAsAngle, final boolean setupdated) {//CURRENT, ANGLE, SPEED
		if (setupdated) {lastSetPoint = value;}
		
		switch (getControlMode()) {
		case Current:super.set(controlMode, value);break;//just use the basic set function
		case PercentOutput:super.set(controlMode, value);break;//just use the basic set function
		case Velocity:super.set(controlMode, value);break;//just use the basic set function
		
		case Follower:
			if (!updated) {//updated is treated differently for follower than for others because it should only be messed with once
				super.set(controlMode, value);
			}updated = true;
			break;
		
		case Position:
			if (treatAsAngle) {
				super.set(controlMode, convert.from.DEGREES.afterGears(gearRatio, getCurrentAngle(false) + wornPath(value)));
			}else {
				//lastSetPoint = ConvertFrom.REVS.afterGears(gearRatio, value);
				lastSetPoint = value*360.0/gearRatio;//TODO not sure exactly why this is being modified here
				super.set(controlMode, value);
			}
			break;
		
		case Disabled:
			if (Math.abs(value) > 1) {value = Math.signum(value);}
			super.set(controlMode, value*12);
			break;
			
		default:break;
		}
		if (getControlMode() != follower) {updated = setupdated;}
	}
	
	
	/**
	 * Run this after all other commands in a system level loop to make sure the Talon receives a command.
	**/
	public void completeLoopUpdate() {
		if (!updated && getControlMode() != follower) {
			this.set(lastSetPoint, true, false);
		}else if (getControlMode() != follower) {
			updated = false;
		}
	}
	
	
	/**
	 * This function returns the PID error for the current control mode.
	 * Current: Milliamperes
	 * Position: Degrees
	 * Speed: RPM
	**/
	public double getCurrentError() {//CURRENT, ANGLE, SPEED
		switch (getControlMode()) {
		case Current:return getClosedLoopError(0);//arg in getSelectedSensorPosition is PID slot ID
		case Position:return convert.to.DEGREES.afterGears(gearRatio, getClosedLoopError(0));
		case Velocity:return getClosedLoopError(0)*600/(4096.0*gearRatio);//TODO add this into enums
		default:return -1;
		}
	}
}