package com.cyborgcats.reusable.Autonomous;

import java.util.Map;

import com.cyborgcats.reusable.Drivetrain;
import com.cyborgcats.reusable.Subsystem;

public abstract class Strategy {
	protected Leash leash;
	protected Events events;
	protected Odometer odometer;
	
	public void use(final Drivetrain drivetrain, final Map<String, Subsystem> subsystems) {
		events.check(leash.getIndependentVariable());
		events.execute(drivetrain, subsystems);
		
		stayOnPath(drivetrain);
  	}
	
	protected void stayOnPath(final Drivetrain drivetrain) {
		//run path processing only if odometer values are new
  		if (odometer.newX() || odometer.newY()) {
  			//get most recent odometer values
  			final double actualX = odometer.getX(true);
  			final double actualY = odometer.getY(true);
		
  			//ensure that the desired position stays a leash length away
  			leash.maintainLength(actualX, actualY);
		
  			//get desired position and compute error components
  			final double desiredX = leash.getX(),
  						 desiredY = leash.getY();
  			final double errorX = desiredX - actualX,
  						 errorY = desiredY - actualY;
		
  			//use error components to compute commands that swerve understands
  			final double errorDirection = Math.toDegrees(Math.atan2(errorX, errorY));
  			final double errorMagnitude = Math.hypot(errorX, errorY);
  			drivetrain.correctFor(errorDirection, errorMagnitude);
  		}
	}
	
	public abstract double initialX();
	public abstract double initialY();
}
