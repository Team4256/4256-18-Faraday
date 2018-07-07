package com.cyborgcats.reusable;//COMPLETE 2017

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;

public class Gyro extends AHRS {
	public final Compass compass;
	
	public Gyro(final byte updateHz) {
		super(I2C.Port.kOnboard, updateHz);
		compass = new Compass(0.0, 0.0);
	}
	
	/**
	 * Tares the gyro's compass
	 * @param tareAngle the new tare angle; can be positive or negative
	 * @param relativeReference if true, tares relative to the current tare rather than 0
	 * @see Compass#setTareAngle(double)
	 */
	public void setTareAngle(double tareAngle, final boolean relativeReference) {
		if (relativeReference) {tareAngle += compass.getTareAngle();}
		compass.setTareAngle(tareAngle);
	}
	
	/**
	 * Cleans up and returns gyro input, accounting for the tare angle
	 * @return gyro heading in the range [0, 360)
	 */
	public double getCurrentAngle() {return Compass.validate((double)getAngle() - compass.getTareAngle());}
	
	/**
	 * Uses <code>compass.legalPath(start, end)</code> to find the most efficient arc from <code>getCurrentAngle()</code> to target
	 * @param target angle, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of current, negative otherwise)
	 * @see Compass#legalPath(current, target)
	 */
	public double pathTo(final double target) {return compass.legalPath(getCurrentAngle(), target);}
	
	/**
	 * @return magnitude of the acceleration vector
	 */
	public double netAcceleration() {
		double xy = (double)(getWorldLinearAccelX()*getWorldLinearAccelX() + getWorldLinearAccelY()*getWorldLinearAccelY());
		return Math.sqrt(xy + (double)(getWorldLinearAccelZ()*getWorldLinearAccelZ()));
	}
}