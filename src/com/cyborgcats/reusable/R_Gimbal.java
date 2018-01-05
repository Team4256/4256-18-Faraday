package com.cyborgcats.reusable;//COMPLETE 2015

import edu.wpi.first.wpilibj.Servo;

public class R_Gimbal {
	private double positionX;
	private double positionY;
	private int scalingFactor;
	private Servo x;
	private Servo y;
	
	public R_Gimbal(final int servoXPort, final int servoYPort, int scalingFactor) {
		this.x = new Servo(servoXPort);
		this.y = new Servo(servoYPort);
		this.scalingFactor = scalingFactor;
	}
	
	public void moveCamera(double axisX, double axisY) {
		setX(positionX + scalingFactor*axisX);
		setY(positionY + scalingFactor*axisY);
	}	
	
	public void setPosition(double x, double y) {
		setX(x);
		setY(y);
	}
	
	public void setX(double positionX) {
		this.positionX = positionX;
		x.setAngle(positionX);
	}
	
	public void setY(double positionY) {
		this.positionY = positionY;
		y.setAngle(positionY);
	}
}
