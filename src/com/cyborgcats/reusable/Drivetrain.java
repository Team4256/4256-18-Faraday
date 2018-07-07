package com.cyborgcats.reusable;

/**
 * A set of common methods which can be implemented for both swerve and tank drive robots.<br>
 * Having them all under one interface makes it easier to write reusable autonomous code.
 */
public interface Drivetrain {
	public void init();
	
	/**
	 * Should control how fast the robot moves in the direction set by {@link #travelTowards(heading)}
	 * @param likely a positive percentage in the range [0, 1]
	 */
	public void setSpeed(final double speed);
	
	/**
	 * Should control how fast the robot spins; positive being clockwise and negative being counterclockwise<br>
	 * @param spin likely a percentage in the range [-1, 1]
	 */
	public void setSpin(final double spin);
	
	/**
	 * Should control the direction in which the robot moves.<br>
	 * For tank drive, {@link #face(heading, maximumOutput)} should be called to accomplish that.
	 * @param heading the desired direction of travel, designated in degrees
	 */
	public void travelTowards(final double heading);
	
	/**
	 * Should correct for positional error during autonomous using some combination of the other methods and {@link PID}<br>
	 * This can usually be accomplished using just {@link #travelTowards(heading)} and {@link #setSpeed(speed)}
	 * @param errorDirection the direction in which the robot should travel to decrease the error, designated in degrees
	 * @param errorMagnitude some indication of the size of the error that can be fed into PID
	 */
	public void correctFor(final double errorDirection, final double errorMagnitude);
	
	/**
	 * Should adjust the direction in which the front of the robot points using {@link PID}
	 * @param orientation which direction the front should face, designated in degrees
	 * @param maximumOutput the maximum value that can be passed into {@link #setSpin(spin)} to achieve that orientation
	 * @return error between desired orientation and actual orientation, designated in degrees
	 */
	public double face(final double orientation, double maximumOutput);
	
	/**
	 * If each of the above methods were to send commands to the motors right away, drivetrain behavior would be incredibly messy and limited.
	 * To achieve smoothness, information gathered from each method must be combined.
	 * For example, swerve must be given a heading from <code>travelTowards()</code>, a speed from <code>setSpeed()</code>, and a spin from <code>setSpin()</code> before holonomic computations can be performed.<br><br>
	 * 
	 * This method satisfies that requirement. It should be called at the end of each loop to coordinate the drivetrain's abilities, matching
	 * the caller's desired heading, speed, spin, etc. as closely as possible.
	 */
	public void completeLoopUpdate();
}
