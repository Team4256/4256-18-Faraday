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
	public final Compass compass;
	public final Convert convert;
	public final boolean hasEncoder;
	
	private ControlMode controlMode;
	private boolean updated = false;
	private Double lastSetpoint = 0.0;
	private double lastLegalDirection = 1.0;
	private Logger logger;
	
	//This constructor is intended for use with an encoder on a motor with limited rotary motion. To limit linear motion, use built-in Talon commands.
	public Talon(final int deviceID, final double gearRatio, final ControlMode controlMode, final Encoder encoder, final boolean flippedSensor, final double protectedZoneStart, final double protectedZoneSize) {
		super(deviceID);
		if (getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
			switch(encoder) {
			case CTRE_MAG_ABSOLUTE:
				hasEncoder = false;
				throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			case CTRE_MAG_RELATIVE:
				hasEncoder = false;
				throw new IllegalStateException("Talon " + Integer.toString(deviceID) + " could not find its encoder.");
			default: hasEncoder = true; break;
			}
		}else {
			configSelectedFeedbackSensor(encoder.type(), 0, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
			configSelectedFeedbackSensor(encoder.type(), 1, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
			configSelectedFeedbackSensor(encoder.type(), 2, kTimeoutMS);//FeedbackDevice, PID slot ID, timeout milliseconds
			hasEncoder = true;
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
		hasEncoder = false;
		compass = new Compass(0.0, 0.0);
		convert = new Convert(0, 0.0);
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
		
		if (getControlMode() == follower) quickSet(masterID, false);
		else quickSet(0.0, false);
	}
	
	/**
	 * This function prepares a motor by setting the PID profile, the closed loop error, and the minimum and maximum voltages.
	**/
	public void init() {init(0, 1.0);}
	
	/**
	 * This function returns the current position in revolutions.
	 * talon.setSelectedSensorPosition() commands will be taken into account, and compass.getTareAngle() is ignored.
	**/
	public double getCurrentRevs() {
		if (hasEncoder) return convert.to.REVS.afterGears(getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
		else return 0.0;//TODO could throw an exception
	}
	
	/**
	 * Gets raw encoder counts from <code>getSelectedSensorPosition(0)</code><br>
	 * Accounts for gear ratio and tare angle, then returns the current position in degrees<br><br>
	 * CAUTION: <code>setSelectedSensorPosition(counts)</code> can easily conflict with the tare angle
	 * @param wraparound if true, result will be in the range [0, 360)
	 * @return current angle in degrees
	 * @see TalonSRX#getSelectedSensorPosition(int)
	 * @see Convert
	 */
	public double getCurrentAngle(final boolean wraparound) {//ANGLE//TODO same as getCurrentRevs todo
		if (hasEncoder) {
			final double raw = convert.to.DEGREES.afterGears(getSelectedSensorPosition(0));//arg in getSelectedSensorPosition is PID slot ID
			return wraparound ? Compass.validate(raw - compass.getTareAngle()) : raw - compass.getTareAngle();
		}else return 0.0;//TODO could throw an exception
	}
	
	/**
	 * Gets raw encoder units from <code>getSelectedSensorVelocity(0)</code><br>
	 * Accounts for gear ratio, then returns the current velocity in RPM
	 * @return velocity in RPM
	 * @see TalonSRX#getSelectedSensorVelocity(int)
	 * @see Convert
	 */
	public double getCurrentRPM() {
		if (hasEncoder) return convert.to.RPM.afterGears(getSelectedSensorVelocity(0));
		else return 0.0;//TODO could throw an exception
	}
	
	/**
	 * Gets raw encoder units from <code>getSelectedSensorVelocity(0)</code><br>
	 * Accounts for gear ratio, then returns the current velocity in RPS
	 * @return velocity in RPS
	 * @see TalonSRX#getSelectedSensorVelocity(int)
	 * @see Convert
	 */
	public double getCurrentRPS() {
		if (hasEncoder) return convert.to.RPS.afterGears(getSelectedSensorVelocity(0));
		else return 0.0;//TODO could throw an excpetion
	}
	
	/**
	 * Uses <code>compass.legalPath(start, end)</code> to find the most efficient arc from <code>getCurrentAngle()</code> to target<br>
	 * If the current angle is inside the protected zone, the arc will be forced to intersect the most recently breached border
	 * @param target angle, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of current, negative otherwise)
	 * @see Compass#legalPath(current, target)
	 */
	public double pathTo(double target) {//ANGLE
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
	/**
	 * Calls {@link #set} with the <code>updateSetpoint</code> boolean set to true, and handles any exceptions
	 * @param value units are dependent on <code>controlMode</code>:<br>
	 * -- Current: Milliamperes<br>
	 * -- Follower: ID<br>
	 * -- Percent: -1.0 to 1.0<br>
	 * -- Position: revs (impacted by <code>setSelectedSensorPosition(counts)</code>) or degrees (impacted by <code>setSelectedSensorPosition(counts)</code> and tare angle)<br>
	 * -- Speed: RPM<br>
	 * @param treatAsDegrees if true, input is treated as an angle and Talon will spin no more than 360 degrees
	 */
	public void quickSet(final double value, final boolean treatAsDegrees) {
		try {
			this.set(value, treatAsDegrees, true);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * 
	 * @param value
	 * @param treatAsDegrees
	 * @param updateSetpoint
	 * @throws IllegalAccessException
	 */
	public void set(final double value, final boolean treatAsDegrees, final boolean updateSetpoint) throws IllegalAccessException {
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
		if (updateSetpoint) lastSetpoint = currentSetPoint;
	}

	
	private double setMilliAmps(final double milliAmps) throws IllegalAccessException {
		if (controlMode == current) super.set(controlMode, milliAmps);
		else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given amps in " + controlMode.name() + " mode.");
		return milliAmps;
	}
	
	
	private int setFollower(final int masterID) throws IllegalAccessException {//Only works with other Talons. To follow Victors, use .follow() command.
		if (controlMode == follower) super.set(controlMode, masterID);
		else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given master ID in " + controlMode.name() + " mode.");
		return masterID;
	}
	
	
	private double setPercent(final double percentage) throws IllegalAccessException {
		if (controlMode == percent) {
			super.set(controlMode, percentage);
			logger.log(Level.FINE, Double.toString(percentage));
			return percentage;
		}else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given percentage in " + controlMode.name() + " mode.");
	}
	
	/*
	 * setDegrees differs from setRevs in more than just units: it wraps values around the motor's axis and finds the shortest path to its target.
	 * For example, if we set to 0, then 120, 240, 360, and back to 120, the motor will have spun CW 1.333 revs and CCW 0 revs.
	 * If this were done with setRevs (0 -> .333 -> .666 -> 1 -> .333), the motor will have spun CW 1 rev and CCW .666 revs.
	 */
	private double setDegrees(final double degrees) throws IllegalAccessException {
		if (controlMode == position) {
			final double encoderCounts = getSelectedSensorPosition(0) + convert.from.DEGREES.afterGears(pathTo(degrees));
			super.set(controlMode, encoderCounts);
			logger.log(Level.FINE, Double.toString(degrees));
			return encoderCounts;
		}else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given degrees in " + controlMode.name() + " mode.");
	}
	
	
	private double setRevs(final double revs) throws IllegalAccessException {
		if (controlMode == position) {
			final double encoderCounts = convert.from.REVS.afterGears(revs);
			super.set(controlMode, encoderCounts);
			logger.log(Level.FINE, Double.toString(revs));
			return encoderCounts;
		}else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given revs in " + controlMode.name() + " mode.");
	}
	
	
	private double setRPM(final double rpm) throws IllegalAccessException {
		if (controlMode == velocity) {
			final double encoderUnits = convert.from.RPM.afterGears(rpm);
			super.set(controlMode, encoderUnits);
			return encoderUnits;
		}else throw new IllegalAccessException("Talon " + Integer.toString(getDeviceID()) + " was given rpm in " + controlMode.name() + " mode.");
	}
	
	
	public void setNeutral() {
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
		
		if (getControlMode() != follower) updated = false;//loop is over, reset updated for use in next loop (followers excluded)
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
