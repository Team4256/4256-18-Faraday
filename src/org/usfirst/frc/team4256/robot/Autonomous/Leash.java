package org.usfirst.frc.team4256.robot.Autonomous;

public class Leash {
	private int currentSegment = 0;
	private boolean doneGeneratingTargets = false;
	
	//an array of segments represents a full path
	private final Segment[] path;
	private final double desiredLength;
	private final double growthRate;
	/**
	 * Leash will try to keep the distance between desired and actual coordinates as close to desiredLength as possible.
	 * growthRate tells it how quickly to increment the time-like parameter that helps generate new X and Y values.
	**/
	public Leash(final Segment[] path, final double desiredLength, final double growthRate) {
		this.path = path;
		this.desiredLength = desiredLength;
		this.growthRate = growthRate;
	}
	
	public void init() {
		currentSegment = 0;
		doneGeneratingTargets = false;
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
	
	
	
//	private class Segment {
//		private final Function x;
//		private final Function y;
//		private double independentVariable, end;
//		public Segment(final Function x, final Function y, final double start, final double end) {
//			this.x = x;
//			this.y = y;
//			this.independentVariable = start;
//			this.end = end;
//		}
//		
//		public boolean increment(final double amount) {
//			independentVariable += amount;
//			return independentVariable < end;
//		}
//		
//		public double getX() {return x.at(independentVariable);}
//		public double getY() {return y.at(independentVariable);}
//		
//		public double getIndependentVariable() {return independentVariable;}
//	}
//	
//	private interface Function {
//		double at(final double independentVariable);
//	}
}