package org.usfirst.frc.team4256.robot;

import com.cyborgcats.reusable.Talon.R_Talon;
import com.cyborgcats.reusable.V_Compass;

public class R_SwerveModule {
	public static final double rotatorGearRatio = 1.0;
	public static final double tractionGearRatio = 15.6;
	private double decapitated = 1;
	private R_Talon rotation;
	private R_Talon traction;
	
	public R_SwerveModule(final int rotatorID, final boolean flippedSensor, final int tractionID) {
		this.rotation = new R_Talon(rotatorID, rotatorGearRatio, R_Talon.position, R_Talon.absolute, flippedSensor);
		this.traction = new R_Talon(tractionID, tractionGearRatio, R_Talon.percent);
	}
	
	
	/**
	 * This function prepares each motor individually, including setting PID values for the rotator.
	**/
	public void init(final boolean reversedMotor) {
		rotation.init();
		rotation.setInverted(reversedMotor);//TODO Temporary line, only here until electronics are correct
		/*
		 * Though reversedMotor is a hardware characteristic that will not change after construction, I remembered why we didn't
		 * put it in the constructor last year: we want to force electronics to be right in the first place, especially since
		 * flipping motor leads is such an easy thing to fix. It would be nice to use the same argument for flippedSensor (which is
		 * in the constructor) but changing that requires taking apart the whole encoder so we are more lenient.
		 */
		rotation.setNeutralMode(R_Talon.coast);
		rotation.config_kP(0, 6, R_Talon.kTimeoutMS);
		rotation.config_kI(0, 0, R_Talon.kTimeoutMS);
		rotation.config_kD(0, .6, R_Talon.kTimeoutMS);
		traction.init();
		traction.setNeutralMode(R_Talon.coast);
	}
	
	
	/**
	 * This offsets the tare angle by the specified amount. Positive means clockwise and negative means counter-clockwise.
	 * Useful when correcting for loose mechanical tolerances.
	**/
	public void setTareAngle(final double tareAngle) {
		rotation.compass.setTareAngle(rotation.compass.getTareAngle() + tareAngle);
	}
	
	
	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the robot in degrees.
	**/
	public void swivelTo(final double wheel_chassisAngle) {
		swivelTo(wheel_chassisAngle, false);
	}
	
	
	/**
	 * Use wheel_chassisAngle to specify the wheel's orientation relative to the robot in degrees.
	 * If ignore is true, nothing will happen, which is useful for coasting based on variables outside this class' scope.
	**/
	public void swivelTo(final double wheel_chassisAngle, final boolean ignore) {
		if (!ignore) {rotation.quickSet(decapitateAngle(wheel_chassisAngle));}//if this doesn't run, complete loop update will eventually set it to be the last angle
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
		traction.quickSet(speed*decapitated);
	}
	
	
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the module except for the traction slave.
	**/
	public void completeLoopUpdate() {
		rotation.completeLoopUpdate();
		traction.completeLoopUpdate();
	}
	
	
	/**
	 * Threshold should be specified in degrees. If the rotator is within that many degrees of its target, this function returns true.
	**/
	public boolean isThere(final double threshold) {
		return Math.abs(rotation.getCurrentError()) <= threshold;
	}
	
	
	/**
	 * This function makes sure the module rotates no more than 90 degrees from its current position.
	 * It should be used every time a new angle is being set to ensure quick rotation.
	**/
	public double decapitateAngle(final double endAngle) {
		decapitated = Math.abs(rotation.wornPath(endAngle)) > 90 ? -1 : 1;
		return decapitated == -1 ? V_Compass.validateAngle(endAngle + 180) : V_Compass.validateAngle(endAngle);
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