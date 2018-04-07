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

public class A_OneSwitchOneScale implements Autonomous{
	public final FieldPieceConfig switchTarget;
	public final FieldPieceConfig scaleTarget;
	public final StartingPosition startingPosition;
	private final V_Odometer odometer;

	public V_Leash leash;
	public V_Events events;
	public double initOdometerPosX = 0.0;

	public A_OneSwitchOneScale(final int startingPosition, final String gameData, final V_Odometer odometer) {
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

			swerve.holonomic_encoderIgnorant(errorDirection, speed, spin);//TODO spin should get updated even outside the if statement
  		}
	}

	public double initOdometerPosX() {return initOdometerPosX;}

	//------------------------------------------------------------------------------------------
	private void useLeash_left() {
		initOdometerPosX = leftStartX;

		if (switchTarget.equals(FieldPieceConfig.LEFT) && scaleTarget.equals(FieldPieceConfig.LEFT)) {
			//{easy switch then easy scale}
			final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
			final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -91, 228, -cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(-cubeX, cubeY, -80, 252, -70, 270, -scaleX, scaleY, 2.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (switchTarget.equals(FieldPieceConfig.LEFT)) {
			//{easy switch then hard scale}
			final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
			final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -91, 228, -cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(-cubeX, cubeY, -30, 289, 94, 190, scaleX, scaleY, 2.0);//get to hard scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			//{easy scale then hard switch}
			final P_Bezier a = new P_Bezier(leftStartX, startY, -120, 215, -86, 242, -scaleX, scaleY, 0.0);//get to easy scale
			final P_Bezier b = new P_Bezier(-scaleX, scaleY, -91, 193, 50, 277, cubeX, cubeY, 1.0);//get to new cube/hard switch

			final P_Bezier[] path = new P_Bezier[] {a, b};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else {
			//{hard switch then hard scale}
			final P_Bezier a = new P_Bezier(leftStartX, startY, -112, 140, -166, 268, 22, 255, 0.0);
			final P_Bezier b = new P_Bezier(22, 255, 41, 251, 55, 235, cubeX, cubeY, 1.0);//get to new cube/hard switch
			final P_Bezier c = new P_Bezier(cubeX, cubeY, 82, 240, 73, 264, scaleX, scaleY, 2.0);//get to hard scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}
	}
	
	private void useEvents_left() {
		if (switchTarget.equals(FieldPieceConfig.LEFT) && scaleTarget.equals(FieldPieceConfig.LEFT)) {
			// at 1.0, reaches easy switch; at 2.0, reaches new cube; at 3.0, reaches easy scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 0},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 0},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 0}
			};
			
			
			final double[] triggers = new double[] {0.3, 0.5, 0.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else if (switchTarget.equals(FieldPieceConfig.LEFT)) {
			// at 1.0, reaches easy switch; at 2.0, reaches new cube; at 3.0, reaches hard scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 0},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 0},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 0}
			};
			

			final double[] triggers = new double[] {0.3, 0.5, 0.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			// at 1.0, reaches easy scale; at 2.0, reaches new cube/hard switch
			final int[][] instructions = new int[][] {
				{0, 3, 0},
				{3, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0},
				{1, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0},
				{2, Parameters.ElevatorPresets.FLOOR.height(), 180},
				{0, Parameters.ElevatorPresets.SWITCH.height(), 180},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 180}
				
			};
			

			final double[] triggers = new double[] {0.3, 0.5, 0.9, 1.2, 1.7, 1.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else {
			// at 1.0, almost to hard switch; at 2.0, reaches new cube/hard switch; at 3.0, reaches hard scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 180},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 180},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 180}
			};

			
			final double[] triggers = new double[] {0.3, 0.6, 1.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);
		}
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private void useLeash_center() {
		initOdometerPosX = centerStartX;

		if (switchTarget.equals(FieldPieceConfig.LEFT) && scaleTarget.equals(FieldPieceConfig.LEFT)) {
			//{left switch then left scale}
			final P_Bezier a = new P_Bezier(centerStartX, startY, -58, 130, -104, 42, -switchX, switchY, 0.0);//get to left switch
			final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -91, 228, -cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(-cubeX, cubeY, -80, 252, -70, 270, -scaleX, scaleY, 2.0);//get to easy left

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (switchTarget.equals(FieldPieceConfig.LEFT)) {
			//{left switch then right scale}
			final P_Bezier a = new P_Bezier(centerStartX, startY, -58, 130, -104, 42, -switchX, switchY, 0.0);//get to left switch
			final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -91, 228, -cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(-cubeX, cubeY, -30, 279, 94, 180, scaleX, scaleY, 2.0);//get to right scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
			//{right switch then left scale}
			final P_Bezier a = new P_Bezier(centerStartX, startY, 84, 132, 103, 85, switchX, switchY, 0.0);//get to right switch
			final P_Bezier b = new P_Bezier(switchX, switchY, 115, 202, 91, 228, cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(cubeX, cubeY, 30, 279, -94, 180, -scaleX, scaleY, 2.0);//get to left scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else {
			//{right switch then right scale}
			final P_Bezier a = new P_Bezier(centerStartX, startY, 84, 132, 103, 85, switchX, switchY, 0.0);//get to right switch
			final P_Bezier b = new P_Bezier(switchX, switchY, 115, 202, 91, 228, cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(cubeX, cubeY, 80, 252, 70, 270, scaleX, scaleY, 2.0);//get to right scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}
	}

	private void useEvents_center() {
		// at 1.0, reaches switch; at 2.0, reaches new cube; at 3.0, reaches scale//TODO this was just copy pasted
		final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
			{0, 3, 0},
			{3, Parameters.ElevatorPresets.SWITCH.height(), 0},
			{1, Parameters.ElevatorPresets.SWITCH.height(), 0}
		};
		
		
		final double[] triggers = new double[] {0.3, 0.5, 0.9};
		events = new V_Events(V_Events.getFromArray(instructions), triggers);
	}
	//------------------------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	private void useLeash_right() {
		initOdometerPosX = rightStartX;

		if (switchTarget.equals(FieldPieceConfig.RIGHT) && scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			//{easy switch then easy scale}
			final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch
			final P_Bezier b = new P_Bezier(switchX, switchY, 115, 202, 91, 228, cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(cubeX, cubeY, 80, 252, 70, 270, scaleX, scaleY, 2.0);//get to easy scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
			//{easy switch then hard scale}
			final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch
			final P_Bezier b = new P_Bezier(switchX, switchY, 115, 202, 91, 228, cubeX, cubeY, 1.0);//get to new cube
			final P_Bezier c = new P_Bezier(cubeX, cubeY, 30, 289, -94, 190, -scaleX, scaleY, 2.0);//get to hard scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);

		}else if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			//{easy scale then hard switch}
			final P_Bezier a = new P_Bezier(rightStartX, startY, 120, 215, 86, 242, scaleX, scaleY, 0.0);//get to easy scale
			final P_Bezier b = new P_Bezier(scaleX, scaleY, 91, 193, -50, 277, -cubeX, cubeY, 1.0);//get to new cube/hard switch

			final P_Bezier[] path = new P_Bezier[] {a, b};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}else {
			//{hard switch then hard scale}
			final P_Bezier a = new P_Bezier(rightStartX, startY, 112, 140, 166, 268, -22, 255, 0.0);
			final P_Bezier b = new P_Bezier(-22, 255, -41, 251, -55, 235, -cubeX, cubeY, 1.0);//get to new cube/hard switch
			final P_Bezier c = new P_Bezier(-cubeX, cubeY, -82, 240, -73, 264, -scaleX, scaleY, 2.0);//get to hard scale

			final P_Bezier[] path = new P_Bezier[] {a, b, c};
			leash = new V_Leash(path, /*leash length*/1.5, /*growth rate*/0.1);
		}
	}

	private void useEvents_right() {
		if (switchTarget.equals(FieldPieceConfig.RIGHT) && scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			// at 1.0, reaches easy switch; at 2.0, reaches new cube; at 3.0, reaches easy scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 270},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 270},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 270}
			};
			
			
			final double[] triggers = new double[] {0.3, 0.5, 0.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
			// at 1.0, reaches easy switch; at 2.0, reaches new cube; at 3.0, reaches hard scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 270},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 270},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 270}
			};
			
			
			final double[] triggers = new double[] {0.3, 0.5, 0.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
			// at 1.0, reaches easy scale; at 2.0, reaches new cube/hard switch
			final int[][] instructions = new int[][] {
				{0, 3, 0},
				{3, Parameters.ElevatorPresets.SCALE_HIGH.height(), 0},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 0},
				{2, Parameters.ElevatorPresets.FLOOR.height(), 180},
				{0, Parameters.ElevatorPresets.SWITCH.height(), 180},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 180}
			};
			
			
			final double[] triggers = new double[] {0.3, 0.5, 0.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);

		}else {
			// at 1.0, almost to hard switch; at 2.0, reaches new cube/hard switch; at 3.0, reaches hard scale
			final int[][] instructions = new int[][] {//TODO only takes care of stuff up until switch
				{0, 3, 180},
				{3, Parameters.ElevatorPresets.SWITCH.height(), 180},
				{1, Parameters.ElevatorPresets.SWITCH.height(), 180}
			};
			
			
			final double[] triggers = new double[] {0.3, 0.6, 1.9};
			events = new V_Events(V_Events.getFromArray(instructions), triggers);
		}
	}
	//------------------------------------------------------------------------------------------
	}
