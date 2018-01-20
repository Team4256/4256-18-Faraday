package org.usfirst.frc.team4256.robot;//COMPLETE 2017

import com.cyborgcats.reusable.R_Talon;
import com.cyborgcats.reusable.V_Compass;

import edu.wpi.first.wpilibj.DigitalInput;

public class R_SwerveModule {
	public static final double rotatorGearRatio = 4.2;
	public static final double tractionGearRatio = 15.6;
	private boolean aligned = false;
	private boolean aligning = false;
	private double alignmentRevs = 0;
	private double decapitated = 1;
	private R_Talon rotator;
	private R_Talon tractionA;
	private R_Talon tractionB;
	public DigitalInput sensor;
	
	public R_SwerveModule(final int rotatorID, final boolean flipped, final int tractionAID, final int tractionBID, final int sensorID) {
		this.rotator = new R_Talon(rotatorID, rotatorGearRatio, R_Talon.position, flipped, R_Talon.absolute);
		this.tractionA = new R_Talon(tractionAID, tractionGearRatio, R_Talon.percent);
		this.sensor = new DigitalInput(sensorID);
	}
	/**
	 * This function prepares each motor individually, including setting PID values for the rotator and enslaving the second traction motor.
	**/
	public void init() {
		rotator.init();
		rotator.setNeutralMode(R_Talon.coast);
		rotator.config_kP(0, 6, R_Talon.kTimeoutMS);
		rotator.config_kI(0, 0, R_Talon.kTimeoutMS);
		rotator.config_kD(0, .6, R_Talon.kTimeoutMS);
		tractionA.init();
		tractionA.setNeutralMode(R_Talon.coast);
	}
	/**
	 * This function indicates whether the module has been aligned.
	**/
	public boolean isAligned() {
		return aligned;
	}
	/**
	 * This function indicates whether the module is aligning.
	**/
	public boolean isAligning() {
		return aligning;
	}
	/**
	 * Call this to align the module with its magnet.
	**/
//	public void align(final double increment) {
//		set(0);
//		if (sensor.get()) {
//			if (!aligning) {
//				aligned = false;
//				aligning = true;
//				alignmentRevs = rotator.getPosition();
//			}alignmentRevs += increment;
//			rotator.set(alignmentRevs, false, true);
//		}else {
//			aligning = false;
//			rotator.set(alignmentRevs, false, true);
//			rotator.compass.setTareAngle(alignmentRevs%rotatorGearRatio*360/rotatorGearRatio);
//			aligned = true;
//		}
//	}
	/**
	 * This offsets the tare angle by the specified amount. Positive means clockwise and negative means counter-clockwise.
	 * Useful when correcting for loose mechanical tolerances.
	**/
	public void setTareAngle(final double tareAngle) {
		rotator.compass.setTareAngle(rotator.compass.getTareAngle() + tareAngle);
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
		if (!ignore) {rotator.quickSet(decapitateAngle(wheel_chassisAngle));}//if this doesn't run, complete loop update will eventually set it to be the last angle
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
		tractionA.quickSet(speed*decapitated);
	}
	/**
	 * A shortcut to call completeLoopUpdate on all the Talons in the module except for the traction slave.
	**/
	public void completeLoopUpdate() {
		rotator.completeLoopUpdate();
		tractionA.completeLoopUpdate();
	}
	/**
	 * Threshold should be specified in degrees. If the rotator is within that many degrees of its target, this function returns true.
	**/
	public boolean isThere(final double threshold) {
		return Math.abs(rotator.getCurrentError()) <= threshold;
	}
	/**
	 * This function makes sure the module rotates no more than 90 degrees from its current position.
	 * It should be used every time a new angle is being set to ensure quick rotation.
	**/
	public double decapitateAngle(final double endAngle) {
		decapitated = Math.abs(rotator.wornPath(endAngle)) > 90 ? -1 : 1;
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