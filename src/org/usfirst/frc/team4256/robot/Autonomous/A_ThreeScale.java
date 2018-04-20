package org.usfirst.frc.team4256.robot.Autonomous;

import org.usfirst.frc.team4256.robot.Parameters;
import org.usfirst.frc.team4256.robot.R_Clamp;
import org.usfirst.frc.team4256.robot.R_Drivetrain;
import org.usfirst.frc.team4256.robot.Elevators.R_Combined;

import com.cyborgcats.reusable.V_PID;
import com.cyborgcats.reusable.Autonomous.P_Bezier;
import com.cyborgcats.reusable.Autonomous.V_Events;
import com.cyborgcats.reusable.Autonomous.V_Leash;
import com.cyborgcats.reusable.Autonomous.V_Odometer;

public class A_ThreeScale implements Autonomous{
	public final FieldPieceConfig switchTarget, scaleTarget;
	public final StartingPosition startingPosition;
	private final V_Odometer odometer;

	public V_Leash leash;
	public V_Events events;
	public double initOdometerPosX = 0.0;

	public A_ThreeScale(final int startingPosition, final String gameData, final V_Odometer odometer) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;  break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT; break;
		default:this.startingPosition = StartingPosition.CENTER;break;
		}

		this.odometer = odometer;

		switch (this.startingPosition) {
		case LEFT:  useLeash_left();
					useEvents_left();  break;
		case CENTER:useLeash_center();
					useEvents_center();break;
		case RIGHT: useLeash_right();
					useEvents_right(); break;
		default:    useLeash_center();
					useEvents_center();break;
		}
	}

	public void run(final R_Drivetrain swerve, final R_Clamp clamp, final R_Combined elevator) {
		events.check(leash.getIndependentVariable());
  		final double spin = events.execute(clamp, elevator, swerve.gyro);

		//run path processing only if ZED values are new
  		if (odometer.newX() && odometer.newY()) {
  			//get most recent ZED values
			final double actualX = odometer.getX(true),
						 actualY = odometer.getY(true);

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
			double speed = V_PID.get("zed", errorMagnitude);//DO NOT use I gain with this because errorMagnitude is always positive
			if (speed > 0.6) speed = 0.6;

			swerve.holonomic_encoderIgnorant(errorDirection, speed, spin);
  		}
	}

	public double initOdometerPosX() {return initOdometerPosX;}

	//------------------------------------------------------------------------------------------
	private void useLeash_left() {
		initOdometerPosX = leftStartX;
		
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final P_Bezier a = new P_Bezier(leftStartX, startY, -120, 215, -86, 242, -scaleX, scaleY, 0.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
				
				final P_Bezier[] path = new P_Bezier[] {a};
				leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final P_Bezier a = new P_Bezier(leftStartX, startY, leftStartX, 93, leftStartX, 93, leftStartX, switchY, 0.0);//drive forward
				leash = new V_Leash(new P_Bezier[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}
	
	private void useEvents_left() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final int[][] instructions = new int[][] {
				{4, 3, 0, 5},
				{3, 3, 0, 5},
				{4, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 3},
				{4, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 1},
				{1, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 0}
			};
			
			events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final int[][] instructions = new int[][] {
					{4, 3, 270, 3},
					{3, Parameters.ElevatorPresets.SWITCH.height(), 90, 3},
					{1, Parameters.ElevatorPresets.SWITCH.height(), 90, 3}
				};
				
				events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final int[][] instructions = new int[][] {{4, 3, 0, 5}};
				events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1});
			}
		}
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private V_Leash useLeash_center() {
		initOdometerPosX = centerStartX;
		
		P_Bezier bezier;

		if (switchTarget.equals(FieldPieceConfig.LEFT)) bezier = new P_Bezier(centerStartX, startY, -30, 82, -52, 60, -switchX, switchY, 0.0);//get to left switch
		else bezier = new P_Bezier(centerStartX, startY, 30, 82, 52, 60, switchX, switchY, 0.0);//get to right switch
		
		leash = new V_Leash(new P_Bezier[] {bezier}, /*leash length*/1.5, /*growth rate*/0.1);
		return leash;
	}

	private V_Events useEvents_center() {
		// at 1.0, reaches switch
		final int[][] instructions = new int[][] {
			{4, 3, 0, 7},
			{3, Parameters.ElevatorPresets.SWITCH.height(), 0, 7},
			{1, Parameters.ElevatorPresets.SWITCH.height(), 0, 7}
		};
		
		events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
		return events;
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private void useLeash_right() {
		initOdometerPosX = rightStartX;
		
		if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			final P_Bezier a = new P_Bezier(rightStartX, startY, 120, 215, 86, 242, scaleX, scaleY, 0.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch

				final P_Bezier[] path = new P_Bezier[] {a};
				leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
			}else {
				final P_Bezier a = new P_Bezier(rightStartX, startY, rightStartX, 89, rightStartX, 89, rightStartX, switchY, 0.0);//drive forward
				leash = new V_Leash(new P_Bezier[] {a}, /*leash length*/1.5, /*growth rate*/0.1);
			}
		}
	}

	private void useEvents_right() {
		if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			final int[][] instructions = new int[][] {
				{4, 3, 0, 5},
				{3, 3, 0, 5},
				{4, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 3},
				{4, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 1},
				{1, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0, 0}
			};
			
			events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1, 0.2, 0.6, 0.8, 1.0});
		}else {
			if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				final int[][] instructions = new int[][] {
					{4, 3, 270, 3},
					{3, Parameters.ElevatorPresets.SWITCH.height(), 270, 3},
					{1, Parameters.ElevatorPresets.SWITCH.height(), 270, 3}
				};
				
				events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1, 0.2, 0.9});
			}else {
				final int[][] instructions = new int[][] {{4, 3, 0, 5}};
				events = new V_Events(V_Events.getFromArray(instructions), new double[] {0.1});
			}
		}
	}
	//------------------------------------------------------------------------------------------
}
