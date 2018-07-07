package com.cyborgcats.reusable;//COMPLETE 2016

public class Compass {
	private double tareAngle = 0.0;
	private final double protectedZoneStart;//Angles increase as the numbers on a clock increase. This value should be the first protected angle encountered by a minute hand which starts at 12:00.
	private final double protectedZoneSize;//This value should be the number of degrees the minute hand must travel before reaching the end of the protected section.
	
	public Compass(final double protectedZoneStart, final double protectedZoneSize) {
		this.protectedZoneStart = validate(protectedZoneStart);
		this.protectedZoneSize = Math.abs(protectedZoneSize)%360;
	}
	
	
	/**
	 * Tares the compass relative to the current 0
	 * @param tareAngle the new tare angle; can be positive or negative
	 */
	public void setTareAngle(final double tareAngle) {this.tareAngle = tareAngle;}
	
	
	/**
	 * @return the current tare angle relative to original 0, not relative to a previous tare
	 */
	public double getTareAngle() {return tareAngle;}
	
	
	/**
	 * Wraps values around a circle
	 * @param angle degrees; can be positive, negative, huge, or tiny
	 * @return the corresponding angle in the range [0, 360)
	 */
	public static double validate(final double angle) {
		final double temp = 360 - (Math.abs(angle)%360);
		if (angle < 0) return (temp < 360) ? temp : 0;
		else return (angle%360 < 360) ? angle%360 : 0;
	}
	
	
	/**
	 * Finds the measure of the minor arc between two points on a circle
	 * @param start the first point, designated in degrees
	 * @param end the second point, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of start, negative if it's counterclockwise of start)
	 */
	public static double path(final double start, final double end) {return Math.IEEEremainder(end - start, 360.0);}
	
	
	/**
	 * Calls <code>validate(angle)</code>. If the result is inside the protected zone,
	 * this method pushes it out to the nearest zone border.
	 * @param angle degrees; can be positive, negative, huge, or tiny
	 * @return the corresponding angle in the range [0, 360) \ (zone start, zone end)<br>
	 * this denotes angles in [0, 360) but outside (zone start, zone end)
	 * @see #validate(angle)
	 */
	public double legalize(double angle) {
		if (protectedZoneSize != 0) {
			final double protectedZoneEnd = protectedZoneStart + protectedZoneSize;
				  double fromStartingEdge = path(protectedZoneStart, angle);
			final double toEndingEdge = path(angle, protectedZoneEnd);
			
			if (fromStartingEdge < 0) {
				fromStartingEdge += 360;
				fromStartingEdge *= Math.signum(toEndingEdge);
			}
			if ((fromStartingEdge > 0) && (fromStartingEdge < protectedZoneSize)) angle = fromStartingEdge <= Math.abs(toEndingEdge) ? protectedZoneStart : protectedZoneEnd;
		}
		return validate(angle);
	}
	
	
	/**
	 * Uses <code>path(start, end)</code> to find the minor arc measure between start and the nearest zone border
	 * @param start degrees; can be positive, negative, huge, or tiny
	 * @return arc measure in degrees (positive if the arc is clockwise of start, negative if it's counterclockwise of start)
	 * @see #path(start, end)
	 */
	private double borderPath(final double start) {
		final double toStartingEdge = path(start, protectedZoneStart),
					 toEndingEdge = path(start, protectedZoneStart + protectedZoneSize);
		return Math.abs(toStartingEdge) <= Math.abs(toEndingEdge) ? toStartingEdge : toEndingEdge;
	}
	
	
	/**
	 * This function finds the shortest legal path from the start angle to the end angle and returns the size of that path in degrees.
	 * Positive means clockwise and negative means counter-clockwise.
	**/
	/**
	 * Uses <code>path(start, end)</code> to find the smallest arc between start and <code>legalize(end)</code> that
	 * doesn't intersect the protected zone. If start was in that zone, <code>borderPath(start)</code> is added to the result
	 * @param start the first point, designated in degrees
	 * @param end the second point, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of start, negative if it's counterclockwise of start)
	 * @see #path(start, end)
	 * @see #legalize(angle)
	 * @see #borderPath(angle)
	 */
	public double legalPath(final double start, final double end) {
		final double start_legal = legalize(start);
		final double path_escape = validate(start) == start_legal ? 0.0 : borderPath(start);//0 if start was already legal, otherwise borderPath()
		double path_main = path(start_legal, legalize(end));
		
		if (protectedZoneSize != 0) {
			double borderPath = borderPath(start_legal);//yes, this is intentionally start_legal not start
			
			//OPTION A -- condensed
			double comparator = borderPath == 0 ? 0 : 1;
			if (borderPath == 0) borderPath = path(start_legal, protectedZoneStart + protectedZoneSize/2.0);
			if (path_main/borderPath > comparator) path_main -= Math.copySign(360, path_main);
			
			//OPTION B -- readable
//			if (borderPath != 0) {
//				//equivalent to:  if (Math.abs(borderPath) < Math.abs(path_main) && Math.signum(path_main) == Math.signum(borderPath))
//				if (path_main/borderPath > 1) path_main -= Math.copySign(360, path_main);
//			}else {
//				//equivalent to:  if (Math.signum(path_main) == Math.signum(path(start_legal, protectedZoneStart + protectedZoneSize/2)))
//				if (path_main/path(start_legal, protectedZoneStart + protectedZoneSize/2) > 0) path_main -= Math.copySign(360, path_main);
//			}
		}
		return path_main + path_escape;
	}
	
	
	/**
	 * Calculates the standard deviation of an array of angles (can handle 360->0 boundary condition)
	 * @param angles an array of angles in degrees
	 * @return the standard deviation in degrees
	 */
	public static double stdd(final double[] angles) {
		double sin = 0.0,	cos = 0.0;
		for (double angle : angles) {sin += Math.sin(Math.toRadians(angle));cos += Math.cos(Math.toRadians(angle));}
		
		sin /= angles.length;
		cos /= angles.length;
		final double stdd = Math.sqrt(-Math.log(sin*sin + cos*cos));
		
		return Math.toDegrees(stdd);
	}
	
	
	/**
	 * @param x x coordinate
	 * @param y y coordinate
	 * @return the angle between the Y axis and (x, y) measured in degrees
	 */
	public static double convertToAngle(final double x, final double y) {return Math.toDegrees(Math.atan2(x, -y));}
}