package org.usfirst.frc.team4256.robot.Autonomous;

public class P_Curve implements Path {
	private final Function x;
	private final Function y;
	private double independentVariable, end;
	public P_Curve(final Function x, final Function y, final double start, final double end) {
		this.x = x;
		this.y = y;
		this.independentVariable = start;
		this.end = end;
	}
	
	public boolean increment(final double amount) {
		independentVariable += amount;
		return independentVariable < end;
	}
	
	public double getX() {return x.at(independentVariable);}
	public double getY() {return y.at(independentVariable);}
	
	public double getIndependentVariable() {return independentVariable;}

	public interface Function {
		double at(final double independentVariable);
	}
}
