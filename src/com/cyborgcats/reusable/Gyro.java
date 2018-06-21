package com.cyborgcats.reusable;//COMPLETE 2017

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;

public class Gyro extends AHRS {
//	private double lastLegalDirection = 1.0;
	public Compass compass;
	
	public Gyro(final byte updateHz) {
		super(I2C.Port.kOnboard, updateHz);
		compass = new Compass(0.0, 0.0);
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
		return Compass.validate((double)getAngle() - compass.getTareAngle());
	}
	/**
	 * This function finds the shortest legal path from the current angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	**/
	public double pathTo(final double target) {
		return compass.legalPath(getCurrentAngle(), target);
	}
	/**
	 * This function computes the magnitude of the sum of the world-based acceleration vectors.
	**/
	public double netAcceleration() {
		double xy = (double)(getWorldLinearAccelX()*getWorldLinearAccelX() + getWorldLinearAccelY()*getWorldLinearAccelY());
		return Math.sqrt(xy + (double)(getWorldLinearAccelZ()*getWorldLinearAccelZ()));
	}
}