package com.cyborgcats.reusable;//COMPLETE 2017

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class R_Talon extends TalonSRX {
	public static final FeedbackDevice absolute = FeedbackDevice.CTRE_MagEncoder_Absolute;
	public static final FeedbackDevice relative = FeedbackDevice.CTRE_MagEncoder_Relative;
	public static final ControlMode current = ControlMode.Current;
	public static final ControlMode follower = ControlMode.Follower;
	public static final ControlMode percent = ControlMode.PercentOutput;
	public static final ControlMode position = ControlMode.Position;
	public static final ControlMode velocity = ControlMode.Velocity;
	public static final ControlMode disabled = ControlMode.Disabled;
	private boolean updated = false;
	private double lastSetPoint = 0;
	private double lastLegalDirection = 1;
	public V_Compass compass;
	private double gearRatio;
	//This constructor is intended for use with an encoder on a motor with limited motion.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final boolean flipped, final FeedbackDevice deviceType, final double protectedZoneStart, final double protectedZoneSize) {
		super(deviceID);
		this.gearRatio = gearRatio;
		if (isSensorPresent(deviceType) == FeedbackDeviceStatus.FeedbackStatusPresent) {
			setFeedbackDevice(deviceType);
		}else {
			throw new IllegalStateException("A CANTalon could not find its integrated versaplanetary encoder.");
		}
		reverseOutput(flipped);
		changeControlMode(controlMode);
		compass = new V_Compass(protectedZoneStart, protectedZoneSize);
	}
	//This constructor is intended for use with an encoder on a motor which can spin freely.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final boolean flipped, final FeedbackDevice deviceType) {
		super(deviceID);
		this.gearRatio = gearRatio;
		if (isSensorPresent(deviceType) == FeedbackDeviceStatus.FeedbackStatusPresent) {
			setFeedbackDevice(deviceType);
		}else {
			throw new IllegalStateException("A CANTalon could not find its integrated versaplanetary encoder.");
		}
		reverseOutput(flipped);
		changeControlMode(controlMode);
		compass = new V_Compass(0, 0);
	}
	//This constructor is intended for a motor without an encoder.
	public R_Talon(final int deviceID, final double gearRatio, final ControlMode controlMode) {
		super(deviceID);
		this.gearRatio = gearRatio;
		changeControlMode(controlMode);
		compass = new V_Compass(0, 0);
	}
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	 * It then gets enslaved to the motor at the specified ID.
	**/
	public void init(final int masterID, final float maxVolts) {
		clearStickyFaults();
		setProfile(0);//choose between PID loop parameter stores
		setAllowableClosedLoopErr(0);
		configNominalOutputVoltage(+0f, -0f);//minimum voltage draw
		configPeakOutputVoltage(Math.abs(maxVolts), -Math.abs(maxVolts));//maximum voltage draw
		if (getControlMode() == follower) {
			set(masterID);
		}else {
			set(0);
		}
	}
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	**/
	public void init() {
		init(0, 12f);
	}
	/**
	 * This function returns the current angle. If wraparound is true, the output will be between 0 and 359.999...
	**/
	public double getCurrentAngle(final boolean wraparound) {//ANGLE
		if (getControlMode() != position) {return -1;}
		return wraparound ? V_Compass.validateAngle(getPosition()*360/gearRatio) : getPosition()*360/gearRatio;
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
	@Override
	public void set(final ControlMode controlMode, final double value) {
		set(value, true, true);
	}
	
	public void set(double value, final boolean treatAsAngle, final boolean setupdated) {//CURRENT, ANGLE, SPEED
		if (setupdated) {lastSetPoint = value;}
		switch (getControlMode()) {
		case Current:super.set(value);break;
		case Follower:
			if (!updated) {//updated is treated differently for follower than for others because it should only be messed with once
				super.set(value);
			}updated = true;
			break;
		case PercentOutput:super.set(value);break;
		case Position:
			if (treatAsAngle) {
				super.set((getCurrentAngle(false) + wornPath(value))*gearRatio/360);
			}else {
				lastSetPoint = value*360/gearRatio;
				super.set(value);
			}
			break;
		case Velocity:super.set(value);break;
		case Disabled:
			if (Math.abs(value) > 1) {value = Math.signum(value);}
			super.set(value*12);
			break;
		default:break;
		}
		if (getControlMode() != follower) {
			updated = setupdated;
		}
	}
	/**
	 * Run this after all other commands in a system level loop to make sure the Talon receives a command.
	**/
	public void completeLoopUpdate() {
		if (!updated && getControlMode() != follower) {
			set(lastSetPoint, true, false);
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
		case Current:return getError();
		case Position:return getError()*360/(4096*gearRatio);
		case Velocity:return getError()*600/(4096*gearRatio);
		default:return -1;
		}
	}
}