package org.usfirst.frc.team4256.robot.Autonomous;

public class V_LeashBezier {
    /* Create cubic Bezier segments. Each segment starts at (p0x, p0y) and ends at (p3x, p3y).*/
    /* Therefore (p0x, p0y) should be the same as (p3x, p3y) for the previous segment. The    */
    /* nSteps parameter defines how far we move along the path during the increment call.     */
    /*                                                                                        */
    /* The coordinates below are in inches.                                                   */
    /*                                 p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y nSteps          */
    CubicBezier cb1 = new CubicBezier(   0,   0,  50,  90,   0,  60,  30,  90, 20 );
    CubicBezier cb2 = new CubicBezier(  30,  90, 105, 135,  60,  90, 105,  90, 20 );
    CubicBezier cb3 = new CubicBezier( 105,  90, 105, 200, 105, 155, 105, 190, 20 );
    CubicBezier cb4 = new CubicBezier( 105, 190,  65, 276, 105, 240,  65, 240, 20 );

    //Create an array of CubicBeziers; represents a full path.
    private final CubicBezier[] path = new CubicBezier[] {cb1, cb2, cb3, cb4};
    private int currentSegment = 0;
    private boolean doneGeneratingTargets = false;


    private final double desiredLength;
	/**
	 * Leash will try to keep the distance between desired and actual coordinates as close to desiredLength as possible.
	 * growthRate tells it how quickly to increment the time-like parameter that helps generate new X and Y values.
	**/
	public V_LeashBezier(final double desiredLength, final double growthRate) {
		this.desiredLength = desiredLength;
	}
	
	public void init() {
		currentSegment = 0;
		doneGeneratingTargets = false;
	}
	
	private void increment() {
		if (!path[currentSegment].increment()) {
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
			increment();
		}
	}
	
	public double getX() {return path[currentSegment].getX();}
	public double getY() {return path[currentSegment].getY();}
	//public double getIndependentVariable() {return path[currentSegment].getIndependentVariable();}TODO
	public boolean doneGeneratingTargets() {return doneGeneratingTargets;}
	
	
	private class CubicBezier {
		private final double p0x, p0y;
		private final double p1x, p1y;
		private final double p2x, p2y;
		private final double p3x, p3y;
	
	    private int currentStep;
	    private final int nSteps;
	
	    double currentX, currentY;
	
		public CubicBezier(final double p0x, final double p0y,
		                   final double p1x, final double p1y,
		                   final double p2x, final double p2y,
		                   final double p3x, final double p3y,
	                       final int nSteps) {
	
	                   
	                    /* The points in the example are in inches so they need to be converted to feet. */
			this.p0x = p0x/12.0;
			this.p0y = p0y/12.0;
			this.p1x = p1x/12.0;
			this.p1y = p1y/12.0;
			this.p2x = p2x/12.0;
			this.p2y = p2y/12.0;
			this.p3x = p3x/12.0;
			this.p3y = p3y/12.0;
	
			this.nSteps = nSteps;
	        this.currentStep = 0;
	
	        currentX = p0x;
	        currentY = p0y;
		}
		
		public boolean increment() {
			currentStep++;
			if ( currentStep > nSteps )
	            return false;
	
	        double param = ((double)currentStep / nSteps );
	
	        /* P(t) = (t^3)*P0 + 3*((1-t)^3)*t*P1 + 3*(1-t)*(t^2)*P2 + (t^3)*P3, 0 <= t <= 1 */
	        currentX = Math.pow(1-param,3)*p0x + 3*Math.pow(1-param,2)*param*p1x + 3*(1-param)*Math.pow(param,2)*p2x + Math.pow(param,3)*p3x;
	        currentY = Math.pow(1-param,3)*p0y + 3*Math.pow(1-param,2)*param*p1y + 3*(1-param)*Math.pow(param,2)*p2y + Math.pow(param,3)*p3y;
	
			return true;
		}
		
		public double getX() {
			return currentX;
		}
		
		public double getY() {
			return currentY;
		}
	}
}
