package com.cyborgcats.reusable;//COMPLETE 2017

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;

public class R_Gyro extends AHRS {
	private double lastMeasuredAngle = 0.0;
	private double lastLegalDirection = 1.0;
	public V_Compass compass;
	
	public R_Gyro(final byte updateHz, final double protectedZoneStart, final double protectedZoneSize) {
		super(I2C.Port.kOnboard, updateHz);
		reset();
		compass = new V_Compass(protectedZoneStart, protectedZoneSize);
	}
	/**
	 * Just calls the function of the same name from V_Compass.
	**/
	public void setTareAngle(double tareAngle, final boolean relativeReference) {
		if (relativeReference) {tareAngle += compass.getTareAngle();}
		compass.setTareAngle(tareAngle);
	}
	/**
	 * This function returns the current angle based on the tare angle.
	 * It will ignore changes to the tare angle until hardware calibration is complete.
	**/
	public double getCurrentAngle() {//TODO ignores tare angle, getFusedHeading seems laggy
		double currentAngle = V_Compass.validateAngle((double)getAngle());
		if (!isCalibrating()) {
			if (0 <= currentAngle && currentAngle <= compass.getTareAngle()) {
				currentAngle += 360 - compass.getTareAngle();
			}else {
				currentAngle -= compass.getTareAngle();
			}lastMeasuredAngle = V_Compass.validateAngle(currentAngle);
		}return lastMeasuredAngle;
	}
	/**
	 * This function finds the shortest legal path from the current angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	 * If the current angle is inside the protected zone, the path goes through the previously breached border.
	**/
	public double wornPath(double endAngle) {
		endAngle = compass.legalizeAngle(endAngle + compass.getTareAngle());
		final double currentAngle = getCurrentAngle();
		double currentPathVector = V_Compass.path(currentAngle, endAngle);
		boolean legal = compass.legalizeAngle(currentAngle) == currentAngle;
		if (legal) {
			currentPathVector = compass.legalPath(currentAngle, endAngle);
			lastLegalDirection = Math.signum(currentPathVector);
		}else if (!legal && Math.signum(currentPathVector) != -lastLegalDirection) {
			currentPathVector = 360*Math.signum(-currentPathVector) + currentPathVector;
		}return currentPathVector;
	}
	/**
	 * This function computes the magnitude of the sum of the world-based acceleration vectors.
	**/
	public double netAcceleration() {
		double xy = (double)(getWorldLinearAccelX()*getWorldLinearAccelX() + getWorldLinearAccelY()*getWorldLinearAccelY());
		return Math.sqrt(xy + (double)(getWorldLinearAccelZ()*getWorldLinearAccelZ()));
	}
}