package org.usfirst.frc.team4256.robot.Autonomous;//COMPLETE MARCH, untested

import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.Leash;
import com.cyborgcats.reusable.Autonomous.Odometer;

public class S_PassLine extends Strategy2018 {
	
	public S_PassLine(final int startingPosition, final Odometer odometer) {
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
	
	private void useLeash_left() {
		final P_Bezier a = new P_Bezier(initialX(), initialY(), -110, 93, -93, 93, -switchX, switchY, 0.0);//get to nearest switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	
	private void useLeash_center() {
		final P_Bezier a = new P_Bezier(initialX(), initialY(), 84, 132, 103, 85, switchX, switchY, 0.0);//get to right switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
	
	private void useLeash_right() {
		final P_Bezier a = new P_Bezier(initialX(), initialY(), 116, 89, 99, 89, switchX, switchY, 0.0);//get to nearest switch
		P_Bezier[] path = new P_Bezier[] {a};
		leash = new Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
	}
}
