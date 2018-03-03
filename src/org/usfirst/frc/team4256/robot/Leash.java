package org.usfirst.frc.team4256.robot;

public class Leash {
	/*Define segments of the path using lambda functions for x and y, as well as a start and end value
	for what might be thought of as time. Though the syntax makes it look like x and y are generated
	from themselves, it is actually the time-like incrementer that gets passed in.*/
	Segment one = new Segment(x -> -11.0*(1.0 - Math.cos(x)),
							  y -> 5.0*Math.sin(y),
							  0.0, Math.PI/2.0);
	Segment two = new Segment(x -> -11.0 + x*0.0,
							  y -> 18.0*Math.sin(y - Math.PI/2.0) + 5.0,
							  Math.PI/2.0, Math.PI);
	
	//Create an array of segments; represents a full path.
	private final Segment[] path = new Segment[] {one, two};
	private int currentSegment = 0;
	private boolean doneGeneratingTargets = false;
	
	
	private final double desiredLength;
	private final double growthRate;
	/**
	 * Leash will try to keep the distance between desired and actual coordinates as close to desiredLength as possible.
	 * growthRate tells it how quickly to increment the time-like parameter that helps generate new X and Y values.
	**/
	public Leash(final double desiredLength, final double growthRate) {
		this.desiredLength = desiredLength;
		this.growthRate = growthRate;
	}
	
	public void init() {
		currentSegment = 0;
	}
	
	private void increment(final double amount) {
		if (!path[currentSegment].increment(amount)) {
			if (currentSegment + 1 < path.length) currentSegment++;
			else doneGeneratingTargets = true;
		}
	}
	
	private double getActualLength(final double currentX, final double currentY) {
		final double differenceX = path[currentSegment].getX() - currentX;
		final double differenceY = path[currentSegment].getY() - currentY;
		return Math.sqrt(differenceX*differenceX + differenceY*differenceY);
	}
	
	public void maintainLength(final double currentX, final double currentY) {
		while (!doneGeneratingTargets && getActualLength(currentX, currentY) < desiredLength) {
			increment(growthRate);
		}
	}
	
	public double getX() {return path[currentSegment].getX();}
	public double getY() {return path[currentSegment].getY();}
	public double getIndependentVariable() {return path[currentSegment].getIndependentVariable();}
	public boolean doneGeneratingTargets() {return doneGeneratingTargets;}
	
	
	
	private class Segment {
		private final Function x;
		private final Function y;
		private double independentVariable, end;
		public Segment(final Function x, final Function y, final double start, final double end) {
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
	}
	
	private interface Function {
		double at(final double independentVariable);
	}
}