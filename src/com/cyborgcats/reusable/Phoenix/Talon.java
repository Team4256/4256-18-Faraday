package com.cyborgcats.reusable.Phoenix;

import com.cyborgcats.reusable.Compass;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Talon extends TalonSRX {
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
	private Double lastSetpoint = 0.0;
	private double lastLegalDirection = 1.0;
	public Compass compass;
	public Convert convert;
	private Logger logger;
	
	//This constructor is intended for use with an encoder on a motor with limited rotary motion. To limit linear motion, use built-in Talon commands.
	public Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final Encoder encoder, final boolean flippedSensor, final double protectedZoneStart, final double protectedZoneSize) {
		super(deviceID);
		if (getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
			switch(encoder) {
			case CTRE_MAG_ABSOLUTE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			case CTRE_MAG_RELATIVE: throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			default: break;
			}
		}else {
			configSelectedFeedbackSensor(encoder.type(), 0, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
			configSelectedFeedbackSensor(encoder.type(), 1, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
			configSelectedFeedbackSensor(encoder.type(), 2, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
		}
		setSensorPhase(flippedSensor);
		this.controlMode = controlMode;
		compass = new Compass(protectedZoneStart, protectedZoneSize);
		convert = new Convert(encoder.countsPerRev(), gearRatio);
		logger = Logger.getLogger("Talon " + Integer.toString(deviceID));
	}
	//This constructor is intended for use with an encoder on a motor which can spin freely.
	public Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final Encoder encoder, final boolean flippedSensor) {
		this(deviceID, gearRatio, controlMode, encoder, flippedSensor, 0.0, 0.0);
	}
	//This constructor is intended for a motor without an encoder.
	public Talon(final int deviceID, final ControlMode controlMode) {
		super(deviceID);
		this.controlMode = controlMode;
		compass = new Compass(0.0, 0.0);
		logger = Logger.getLogger("Talon " + Integer.toString(deviceID));
	}
	
	
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum percentages.
	 * If a follower, it then gets enslaved to the motor at the specified ID.
	**/
	public void init(final int masterID, final double maxPercent) {
		clearStickyFaults(kTimeoutMS);//TODO everywhere where we have kTimeoutMS, do error handling
		selectProfileSlot(0, 0);//first is motion profile slot (things like allowable error), second is PID slot ID
		configAllowableClosedloopError(0, 0, kTimeoutMS);//motion profile slot, allowable error, timeout ms
		
		configNominalOutputForward(0.0, kTimeoutMS);
		configNominalOutputReverse(0.0, kTimeoutMS);
		configPeakOutputForward(Math.abs(maxPercent), kTimeoutMS);
		configPeakOutputReverse(-Math.abs(maxPercent), kTimeoutMS);
		
		if (getControlMode() == follower) {
			quickSet(masterID, false);
		}else {
			quickSet(0.0, false);
		}
	}
	
	
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	**/
	public void init() {
		init(0, 1.0);
	}
	
	
	/**
	 * This function returns the current position in revolutions.
	 * talon.setSelectedSensorPosition() commands will be taken into account, and compass.getTareAngle() is ignored.
	**/
	public double getCurrentRevs() {
		return convert.to.REVS.afterGears(getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
	}
	
	
	/**
	 * This function returns the current position in degrees. If wraparound is true, the output will be between 0 and 359.999...
	 * Both the compass.getTareAngle() and any talon.setSelectedSensorPosition() commands will be taken into account. DON'T USE BOTH!
	**/
	public double getCurrentAngle(final boolean wraparound) {//ANGLE
		final double raw = convert.to.DEGREES.afterGears(getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
		return wraparound ? Compass.validate(raw - compass.getTareAngle()) : raw - compass.getTareAngle();
	}
	
	
	public double getCurrentRPM() {return convert.to.RPM.afterGears(getSelectedSensorVelocity(0));}
	
	public double getCurrentRPS() {return convert.to.RPS.afterGears(getSelectedSensorVelocity(0));}
	
	
	/**
	 * This function finds the shortest legal path from the current angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	 * If the current angle is inside the protected zone, the path goes through the previously breached border.
	**/
	public double wornPath(double target) {//ANGLE
		final double current = getCurrentAngle(true);
		double path = compass.legalPath(current, target);
		if (current == compass.legalize(current)) lastLegalDirection = Math.signum(path);
		else if (Math.signum(path) != -lastLegalDirection) path -= Math.copySign(360, path);
		
		return path;
	}
	
	
	/**
	 * This function sets the motor's output or target setpoint based on the control mode.
	 * Current: Milliamperes
	 * Follower: ID
	 * Percent: -1 to 1
	 * Position: if treatAsDegrees, then will not spin more than 360 degrees
	 * Speed: RPM
	**/
	public void quickSet(final double value, final boolean treatAsDegrees) {
		try {
			this.set(value, treatAsDegrees, true);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
	}


	public void set(final double value, final boolean treatAsDegrees, final boolean updateSetPoint) throws IllegalAccessException {
		double currentSetPoint = lastSetpoint;
		switch (controlMode) {
		case Current:
			currentSetPoint = setMilliAmps(value);break;
		case Follower:
			if (!updated) {//updated is treated differently for follower than for others because it should only be messed with once
				currentSetPoint = (double)setFollower((int)value);//casting back and forth from double to int is not the best, but necessary
			}break;
		case PercentOutput:
			currentSetPoint = setPercent(value);break;
		case Position:
			currentSetPoint = treatAsDegrees ? setDegrees(value) : setRevs(value);break;
		case Velocity:
			currentSetPoint = setRPM(value);break;
		case Disabled:break;
		default:throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + "'s mode is unimplemented.");
		}
		
		updated = true;
		if (updateSetPoint) lastSetpoint = currentSetPoint;
	}

	
	private double setMilliAmps(final double milliAmps) throws IllegalAccessException {
		if (controlMode == current) {
			super.set(controlMode, milliAmps);
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given amps in " + controlMode.name() + " mode.");
		}return milliAmps;
	}
	
	
	private int setFollower(final int masterID) throws IllegalAccessException {//Only works with other Talons. To follow Victors, use .follow() command.
		if (controlMode == follower) {
			super.set(controlMode, masterID);
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given master ID in " + controlMode.name() + " mode.");
		}return masterID;
	}
	
	
	private double setPercent(final double percentage) throws IllegalAccessException {
		if (controlMode == percent) {
			super.set(controlMode, percentage);
			logger.log(Level.FINE, Double.toString(percentage));
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given percentage in " + controlMode.name() + " mode.");
		}return percentage;
	}
	
	
	private double setDegrees(final double degrees) throws IllegalAccessException {
		if (controlMode == position) {
			final double encoderCounts = getSelectedSensorPosition(0) + convert.from.DEGREES.afterGears(wornPath(degrees));
			super.set(controlMode, encoderCounts);
			logger.log(Level.FINE, Double.toString(degrees));
			return encoderCounts;
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given degrees in " + controlMode.name() + " mode.");
		}
	}
	
	
	private double setRevs(final double revs) throws IllegalAccessException {
		if (controlMode == position) {
			final double encoderCounts = convert.from.REVS.afterGears(revs);
			super.set(controlMode, encoderCounts);
			logger.log(Level.FINE, Double.toString(revs));
			return encoderCounts;
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given revs in " + controlMode.name() + " mode.");
		}
	}
	
	
	private double setRPM(final double rpm) throws IllegalAccessException {
		if (controlMode == velocity) {
			final double encoderUnits = convert.from.RPM.afterGears(rpm);
			super.set(controlMode, encoderUnits);
			return encoderUnits;
		}else {
			throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given rpm in " + controlMode.name() + " mode.");
		}
	}
	
	
	public void enterNeutral() {
		neutralOutput();
		updated = true;
		lastSetpoint = null;
	}
	
	
	/**
	 * Run this after all other commands in a system level loop to make sure the Talon receives a command.
	**/
	public void completeLoopUpdate() {
		if (!updated) {
			if (lastSetpoint != null) super.set(controlMode, lastSetpoint);//send a command if there hasn't yet been one, using raw encoder units
			else neutralOutput();
		}
		
		if (getControlMode() != follower) {updated = false;}//loop is over, reset updated for use in next loop (followers excluded)
	}
	
	
	/**
	 * This function returns the PID error for the current control mode.
	 * Current: Milliamperes
	 * Position: neither degrees nor revs are wrapped around 360 or 1
	 * Speed: RPM
	**/
	public double getCurrentError(final boolean asDegrees) {
		switch (getControlMode()) {
		case Current:return getClosedLoopError(0);
		case Position:return asDegrees ? convert.to.DEGREES.afterGears(getClosedLoopError(0)) : convert.to.REVS.afterGears(getClosedLoopError(0));
		case Velocity:return convert.to.RPM.afterGears(getClosedLoopError(0));
		default:return 0.0;
		}
	}

	public void setParentLogger(final Logger logger) {this.logger = logger;}
}
