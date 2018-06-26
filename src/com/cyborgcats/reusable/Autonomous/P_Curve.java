package com.cyborgcats.reusable.Autonomous;

public class P_Curve implements Path {
	private final Function x, y;
	private final double end;
	private double independentVariable;
	public P_Curve(final Function x, final Function y, final double start, final double end) {
		this.x = x;
		this.y = y;
		this.end = end;
		this.independentVariable = start;
	}
	
	public boolean increment(final double amount) {
		independentVariable += amount;
		return independentVariable < end;
	}
	
	public double getX() {return x.at(independentVariable);}
	public double getY() {return y.at(independentVariable);}
	
	public double getIndependentVariable() {return independentVariable;}

	public static interface Function {double at(final double independentVariable);}
}
