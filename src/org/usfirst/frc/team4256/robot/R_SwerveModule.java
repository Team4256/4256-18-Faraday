package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.Talon.R_Encoder;
import com.cyborgcats.reusable.Talon.R_Talon;
import com.cyborgcats.reusable.V_Compass;

public class R_SwerveModule {
	public static final double rotatorGearRatio = 1.0;
	public static final double tractionGearRatio = 40.0/3.0;
	public static final double tractionWheelCircumference = 2.625*Math.PI;//inches
	private double decapitated = 1;
	private R_Talon rotation;
	private R_Talon traction;
	private boolean hasTractionSensor;
	private double tractionDeltaPathLength = 0.0;
	private double tractionPreviousPathLength = 0.0;
	
	//This constructor is intended for use with the module which has an encoder on the traction motor.
	public R_SwerveModule(final int rotatorID, final boolean flippedSensor, final int tractionID, final boolean flippedSensorTraction) {
		this.rotation = new R_Talon(rotatorID, rotatorGearRatio, R_Talon.position, R_Encoder.CTRE_MAG_ABSOLUTE, flippedSensor);
		this.traction = new R_Talon(tractionID, tractionGearRatio, R_Talon.percent, R_Encoder.RS7_QUAD, flippedSensorTraction);
		hasTractionSensor = true;
	}
	//This constructor is intended for all other modules.
	public R_SwerveModule(final int rotatorID, final boolean flippedSensor, final int tractionID) {
		this.rotation = new R_Talon(rotatorID, rotatorGearRatio, R_Talon.position, R_Encoder.CTRE_MAG_ABSOLUTE, flippedSensor);
		this.traction = new R_Talon(tractionID, tractionGearRatio, R_Talon.percent);
		hasTractionSensor = false;
	}
	
	
	/**
	 * This function prepares each motor individually, including setting PID values for the rotator.
	**/
	public void init(final boolean reversedTraction) {//TODO config ramp rates
		rotation.init();
		
		rotation.setNeutralMode(R_Talon.coast);
		rotation.config_kP(0, 5.0, R_Talon.kTimeoutMS);
		rotation.config_kI(0, 0.0, R_Talon.kTimeoutMS);
		rotation.config_kD(0, 1.5, R_Talon.kTimeoutMS);
		
		traction.init();
		traction.setInverted(reversedTraction);
		traction.setNeutralMode(R_Talon.coast);
		traction.configContinuousCurrentLimit(45, R_Talon.kTimeoutMS);
		traction.configPeakCurrentLimit(50, R_Talon.kTimeoutMS);
		traction.configPeakCurrentDuration(250, R_Talon.kTimeoutMS);
	}
	
	
	/**
	 * This sets the tare angle. Positive means clockwise and negative means counter-clockwise.
	 * Useful when correcting for loose mechanical tolerances.
	**/
	public void setTareAngle(final double tareAngle) {
		rotation.compass.setTareAngle(tareAngle);
	}
	
	/**
	 * This offsets the tare angle by the specified amount. Positive means clockwise and negative means counter-clockwise.
	 * Useful when correcting for loose mechanical tolerances.
	**/
	public void incrementTareAngle(final double incrementAngle) {
		setTareAngle(rotation.compass.getTareAngle() + incrementAngle);
	}
	
	
	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the robot in degrees.
	**/
	public void swivelTo(final double wheel_chassisAngle) {
		swivelTo(wheel_chassisAngle, false);
	}
	
	
	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the robot in degrees.
	 * If ignore is true, nothing will happen, which is useful for coasting based on variables outside this class's scope.
	**/
	public void swivelTo(final double wheel_chassisAngle, final boolean ignore) {
		if (!ignore) {rotation.quickSet(decapitateAngle(wheel_chassisAngle), true);}//if this doesn't run, complete loop update will eventually set it to be the last angle
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
	public void set(final double speed) {
		traction.quickSet(speed*decapitated, false);
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the module.
	**/
	public void completeLoopUpdate() {
		rotation.completeLoopUpdate();
		traction.completeLoopUpdate();
		
		if (hasTractionSensor) {
			final double currentPathLength = tractionPathLength();
			tractionDeltaPathLength = currentPathLength - tractionPreviousPathLength;
			tractionPreviousPathLength = currentPathLength;
		}
	}
	
	
	/**
	 * Threshold should be specified in degrees. If the rotator is within that many degrees of its target, this function returns true.
	**/
	public boolean isThere(final double threshold) {
		return Math.abs(rotation.getCurrentError(true)) <= threshold;
	}
	
	
	/**
	 * This function makes sure the module rotates no more than 90 degrees from its current position.
	 * It should be used every time a new angle is being set to ensure quick rotation.
	**/
	public double decapitateAngle(final double endAngle) {
		decapitated = Math.abs(rotation.wornPath(endAngle)) > 90 ? -1 : 1;
		return decapitated == -1 ? V_Compass.validateAngle(endAngle + 180) : V_Compass.validateAngle(endAngle);
	}

	
	public double tractionSpeed() {//TODO componetize into X, Y, field centric, and robot centric, max around 60
		if (hasTractionSensor) {
			return traction.getCurrentRPM()*60.0*tractionWheelCircumference/12.0;
		}else {
			throw new IllegalStateException("Cannot get traction motor speed without an encoder!");
		}
	}
	
	
	public double tractionPathLength() {
		if (hasTractionSensor) {
			return traction.getCurrentRevs()*tractionWheelCircumference;
		}else {
			return 0.0;
		}
	}
	
	
	public double deltaDistance() {
		return tractionDeltaPathLength;
	}
	
	
	/**
	 * This function translates angles from the robot's perspective to the field's orientation.
	 * It requires an angle and input from the gyro.
	**/
	public static double convertToField(final double wheel_robotAngle, final double chassis_fieldAngle) {
		return V_Compass.validateAngle(wheel_robotAngle + chassis_fieldAngle);
	}
	
	
	/**
	 * This function translates angles from the field's orientation to the robot's perspective.
	 * It requires an angle and input from the gyro.
	**/
	public static double convertToRobot(final double wheel_fieldAngle, final double chassis_fieldAngle) {
		return V_Compass.validateAngle(wheel_fieldAngle - chassis_fieldAngle);
	}
}