package com.cyborgcats.reusable.Autonomous;

public class P_Bezier implements Path {
	private final double p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y, start;

	private double independentVariable = 0.0;
    private double currentX, currentY;

	public P_Bezier(final double p0x, final double p0y, final double p1x, final double p1y, final double p2x, final double p2y, final double p3x, final double p3y,
			final double start) {
		
		this.p0x = p0x;	this.p0y = p0y;
		this.p1x = p1x;	this.p1y = p1y;
		this.p2x = p2x;	this.p2y = p2y;
		this.p3x = p3x;	this.p3y = p3y;
		this.start = start;
		
        currentX = this.p0x;
        currentY = this.p0y;
	}
	
	public boolean increment(final double amount) {
		independentVariable += amount;
		if (independentVariable > 1.0) return false;

		//P(t) = (1-t^3)*P0 + 3*((1-t)^3)*t*P1 + 3*(1-t)*(t^2)*P2 + (t^3)*P3, 0 <= t <= 1
        currentX = Math.pow(1-independentVariable,3)*p0x + 3*Math.pow(1-independentVariable,2)*independentVariable*p1x + 3*(1-independentVariable)*Math.pow(independentVariable,2)*p2x + Math.pow(independentVariable,3)*p3x;
        currentY = Math.pow(1-independentVariable,3)*p0y + 3*Math.pow(1-independentVariable,2)*independentVariable*p1y + 3*(1-independentVariable)*Math.pow(independentVariable,2)*p2y + Math.pow(independentVariable,3)*p3y;

		return true;
	}
	
	public double getX() {return currentX;}
	public double getY() {return currentY;}
	
	public double getIndependentVariable() {return independentVariable + start;}
}
