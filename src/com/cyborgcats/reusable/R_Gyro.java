package com.cyborgcats.reusable;//COMPLETE 2017

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;

public class R_Gyro extends AHRS {
//	private double lastLegalDirection = 1.0;
	public V_Compass compass;
	
	public R_Gyro(final byte updateHz) {
		super(I2C.Port.kOnboard, updateHz);
		compass = new V_Compass(0.0, 0.0);
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
	**/
	public double getCurrentAngle() {
		return V_Compass.validate((double)getAngle() - compass.getTareAngle());
	}
	/**
	 * This function finds the shortest legal path from the current angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	 * If the current angle is inside the protected zone, the path goes through the previously breached border.
	**/
	public double wornPath(final double target) {
		final double current = getCurrentAngle();
		double path = compass.legalPath(current, target);
		//this code is only necessary if protectedZoneSize is nonzero
//		if (current == compass.legalize(current)) lastLegalDirection = Math.signum(path);
//		else if (Math.signum(path) != -lastLegalDirection) path -= Math.copySign(360, path);
		
		return path;
	}
	/**
	 * This function computes the magnitude of the sum of the world-based acceleration vectors.
	**/
	public double netAcceleration() {
		double xy = (double)(getWorldLinearAccelX()*getWorldLinearAccelX() + getWorldLinearAccelY()*getWorldLinearAccelY());
		return Math.sqrt(xy + (double)(getWorldLinearAccelZ()*getWorldLinearAccelZ()));
	}
}