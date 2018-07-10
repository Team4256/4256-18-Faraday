package com.cyborgcats.reusable;//COMPLETE 2015

import edu.wpi.first.wpilibj.Servo;

public class Gimbal {
	private final R_Servo x, y;
	private final int K;
	
	public Gimbal(final int portX, final int portY, int scalingFactor) {
		this.x = new R_Servo(portX);
		this.y = new R_Servo(portY);
		this.K = scalingFactor;
	}
	
	public void set(final double degreesX, final double degreesY) {x.setAngle(degreesX);	y.setAngle(degreesY);}
	public void increment(final double amountX, final double amountY) {x.increment(amountX*K);	y.increment(amountY*K);}
	
	private static class R_Servo extends Servo {
		private double angle = 0.0;
		public R_Servo(int port) {super(port);}
		@Override
		public void setAngle(final double degrees) {super.setAngle(degrees);	angle = degrees;}
		public void increment(final double degrees) {setAngle(angle + degrees);}
		@Override
		public double getAngle() {return angle;}
	}
}
