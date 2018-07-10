package org.usfirst.frc.team4256.robot;

import java.util.logging.Logger;

import com.cyborgcats.reusable.Compass;
import com.cyborgcats.reusable.Phoenix.Encoder;
import com.cyborgcats.reusable.Phoenix.Talon;

import edu.wpi.first.wpilibj.DigitalInput;

public final class SwerveModule {
	public static final double rotatorGearRatio = 1.0;
	public static final double tractionGearRatio = 40.0/3.0;
	public static final double tractionWheelCircumference = 2.625*Math.PI;//inches
	private final Talon rotation;
	private final Talon traction;
	private final DigitalInput magnet;
	
	private double decapitated = 1.0;
	private double tractionDeltaPathLength = 0.0;
	private double tractionPreviousPathLength = 0.0;
	private boolean aligned = true;
	
	//This constructor is intended for use with the module which has an encoder on the traction motor.
	public SwerveModule(final int rotatorID, final boolean flippedSensor, final int tractionID, final boolean flippedSensorTraction, final int magnetID) {
		rotation = new Talon(rotatorID, rotatorGearRatio, Talon.position, Encoder.CTRE_MAG_ABSOLUTE, flippedSensor);
		traction = new Talon(tractionID, tractionGearRatio, Talon.percent, Encoder.RS7_QUAD, flippedSensorTraction);
		magnet = new DigitalInput(magnetID);
	}
	//This constructor is intended for all other modules.
	public SwerveModule(final int rotatorID, final boolean flippedSensor, final int tractionID, final int magnetID) {
		rotation = new Talon(rotatorID, rotatorGearRatio, Talon.position, Encoder.CTRE_MAG_ABSOLUTE, flippedSensor);
		traction = new Talon(tractionID, Talon.percent);
		magnet = new DigitalInput(magnetID);
	}
	
	
	/**
	 * This function prepares each motor individually, including setting PID values for the rotator.
	**/
	public void init(final boolean reversedTraction) {
		rotation.init();
		
		rotation.setNeutralMode(Talon.coast);
		rotation.config_kP(0, 15.0, Talon.kTimeoutMS);
		rotation.config_kI(0, 0.0, Talon.kTimeoutMS);
		rotation.config_kD(0, 2.0, Talon.kTimeoutMS);
		
		traction.init();
		
		traction.setInverted(reversedTraction);
		traction.setNeutralMode(Talon.coast);
		traction.configPeakOutputForward(.9166, 0);//%, delay to wait for error code
		traction.configPeakOutputReverse(-.9166, 0);
		traction.configContinuousCurrentLimit(40, Talon.kTimeoutMS);
		traction.configPeakCurrentLimit(45, Talon.kTimeoutMS);
		traction.configPeakCurrentDuration(250, Talon.kTimeoutMS);
	}
	
	public void autoMode(final boolean enable) {
		if (enable) traction.configOpenloopRamp(2.0, Talon.kTimeoutMS);
		else traction.configOpenloopRamp(1.0, Talon.kTimeoutMS);
	}
	
	
	/**
	 * This sets the tare angle. Positive means clockwise and negative means counter-clockwise.
	**/
	public void setTareAngle(final double tareAngle) {setTareAngle(tareAngle, false);}
	
	
	/**
	 * This sets the tare angle. Positive means clockwise and negative means counter-clockwise.
	 * If relativeReference is true, tareAngle will be incremented rather than set.
	**/
	public void setTareAngle(double tareAngle, final boolean relativeReference) {
		if (relativeReference) tareAngle += rotation.compass.getTareAngle();
		rotation.compass.setTareAngle(tareAngle);
	}
	
	
	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the robot in degrees.
	**/
	public void swivelTo(final double wheel_chassisAngle) {
		rotation.quickSet(decapitateAngle(wheel_chassisAngle), true);
	}
	
	
	/**
	 * Use wheel_fieldAngle to specify the wheel's orientation relative to the field in degrees.
	**/
	public void swivelWith(final double wheel_fieldAngle, final double chassis_fieldAngle) {
		swivelTo(convertToRobot(wheel_fieldAngle, chassis_fieldAngle));
	}
	
	
	/**
	 * This function sets the master and slave traction motors to the specified speed, from -1 to 1.
	 * It also makes sure that they turn in the correct direction, regardless of decapitated state.
	**/
	public void set(final double speed) {traction.quickSet(speed*decapitated, false);}
	
	public boolean magneticAlignment(final double offset) {
		if (magnet.get()) {
			aligned = false;
			rotation.quickSet(rotation.getCurrentRevs() + 0.05, false);
		}else if (!aligned) {
			setTareAngle(rotation.getCurrentAngle(true) + offset, true);
			decapitated = 1;
			traction.setInverted(true);
			aligned = true;
		}
		return aligned;
	}
	
	
	public void checkTractionEncoder() {
		if (traction.hasEncoder) {
			final double currentPathLength = tractionPathLength();
			tractionDeltaPathLength = currentPathLength - tractionPreviousPathLength;
			tractionPreviousPathLength = currentPathLength;
		}
	}
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the module.
	**/
	public void completeLoopUpdate() {
		rotation.completeLoopUpdate();
		traction.completeLoopUpdate();
	}
	
	
	/**
	 * Threshold should be specified in degrees. If the rotator is within that many degrees of its target, this function returns true.
	**/
	public boolean isThere(final double threshold) {return Math.abs(rotation.getCurrentError(true)) <= threshold;}
	
	
	/**
	 * This function makes sure the module rotates no more than 90 degrees from its current position.
	 * It should be used every time a new angle is being set to ensure quick rotation.
	**/
	public double decapitateAngle(final double endAngle) {
		decapitated = Math.abs(rotation.pathTo(endAngle)) > 90 ? -1 : 1;
		return decapitated == -1 ? Compass.validate(endAngle + 180) : Compass.validate(endAngle);
	}

	
	public double tractionSpeed() {
		if (traction.hasEncoder) return tractionWheelCircumference*traction.getCurrentRPS();
		else throw new IllegalStateException("Cannot get traction motor speed without an encoder!");
	}
	
	
	public double tractionPathLength() {
		if (traction.hasEncoder) return traction.getCurrentRevs()*tractionWheelCircumference/12.0;
		else throw new IllegalStateException("Cannot get path length without an encoder!");
	}
	
	
	public double deltaDistance() {return tractionDeltaPathLength;}
	public double deltaXDistance() {return tractionDeltaPathLength*Math.sin(convertToField(rotation.getCurrentAngle(true), Robot.gyroHeading)*Math.PI/180.0);}
	public double deltaYDistance() {return tractionDeltaPathLength*Math.cos(convertToField(rotation.getCurrentAngle(true), Robot.gyroHeading)*Math.PI/180.0);}
	
	public Talon rotationMotor() {return rotation;}
	public Talon tractionMotor() {return traction;}
	public double decapitated() {return decapitated;}
	

	public void setParentLogger(final Logger logger) {
		rotation.setParentLogger(logger);
		traction.setParentLogger(logger);
	}
	
	/**
	 * This function translates angles from the robot's perspective to the field's orientation.
	 * It requires an angle and input from the gyro.
	**/
	public static double convertToField(final double wheel_robotAngle, final double chassis_fieldAngle) {
		return Compass.validate(wheel_robotAngle + chassis_fieldAngle);
	}
	
	
	/**
	 * This function translates angles from the field's orientation to the robot's perspective.
	 * It requires an angle and input from the gyro.
	**/
	public static double convertToRobot(final double wheel_fieldAngle, final double chassis_fieldAngle) {
		return Compass.validate(wheel_fieldAngle - chassis_fieldAngle);
	}
}