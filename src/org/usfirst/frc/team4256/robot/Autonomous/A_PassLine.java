package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_DriveTrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

import com.cyborgcats.reusable.V_PID;
import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.V_Leash;
import com.cyborgcats.reusable.Autonomous.V_Odometer;

public class A_PassLine implements Autonomous{
	public final StartingPosition startingPosition;
	private final V_Odometer odometer;
	
	public V_Leash leash;
	public double initOdometerPosX = 0.0;
	
	public A_PassLine(final int startingPosition, final V_Odometer odometer) {
		//{organize initialization data}
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT;break;
		default: this.startingPosition = StartingPosition.CENTER;break;
		}
		
		this.odometer = odometer;
		
		switch (this.startingPosition) {
		case LEFT:  useLeash_left();  break;
		case CENTER:useLeash_center();break;
		case RIGHT: useLeash_right(); break;
		default:    useLeash_center();break;
		}
	}
	
	public void run(final R_DriveTrain swerve, final R_Clamp clamp, final R_Combined elevator) {
		//run processing only if ZED values are new
  		if (odometer.newX() && odometer.newY()) {
  			//get most recent ZED values
			final double actualX = odometer.getX(),
						 actualY = odometer.getY();
			
			//ensure that the desired position stays a leash length away
			leash.maintainLength(actualX, actualY);
			
			//get desired position and compute error components
			final double desiredX = leash.getX(),
						 desiredY = leash.getY();
			final double errorX = desiredX - actualX,
						 errorY = desiredY - actualY;
			
			//use error components to compute commands that swerve understands
			final double errorDirection = Math.toDegrees(Math.atan2(errorX, errorY));
			final double errorMagnitude = Math.sqrt(errorX*errorX + errorY*errorY);
			double speed = V_PID.get("zed", errorMagnitude);
			if (Math.abs(speed) > 0.7) speed = 0.7*Math.signum(speed);
			
			swerve.holonomic_encoderAware(errorDirection, speed, 0.0/*spin*/);
  		}
	}
	
	public double initOdometerPosX() {return initOdometerPosX;}
	
	private void useLeash_left() {
		initOdometerPosX = leftStartX;
		
		final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to nearest switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	
	private void useLeash_center() {
		initOdometerPosX = centerStartX;
		
		final P_Bezier a = new P_Bezier(centerStartX, startY, 84, 132, 103, 85, switchX, switchY, 0.0);//get to right switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	
	private void useLeash_right() {
		initOdometerPosX = rightStartX;
		
		final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to nearest switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
}
