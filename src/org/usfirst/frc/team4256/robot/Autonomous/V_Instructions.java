package org.usfirst.frc.team4256.robot.Autonomous;

public class V_Instructions {
	public static final double leftStartX = -112.33, centerStartX = 7.19, rightStartX = 115.0;
	public static final double switchX = 95.43, cubeX = 72.64, scaleX = 71.11;
	public static final double startY = 13.32, switchY = 166.6, cubeY = 214.35, scaleY = 296.13;
	
	public final double initOdometerPosX;
	
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceCharter {LEFT, RIGHT};
	
	public final FieldPieceCharter switchTarget;
	public final FieldPieceCharter scaleTarget;
	public final StartingPosition startingPosition;
	
	private V_Leash leash;
	
	public V_Instructions(final String gameData, final int startingPosition) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceCharter.LEFT : FieldPieceCharter.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceCharter.LEFT : FieldPieceCharter.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT;break;
		default: this.startingPosition = StartingPosition.CENTER;break;
		}
		
		switch (this.startingPosition) {//TODO for each starting pos, have one set of events for paths of 3 and one for paths of 2
		case LEFT:initOdometerPosX = leftStartX;
		
			if (switchTarget.equals(FieldPieceCharter.LEFT) && scaleTarget.equals(FieldPieceCharter.LEFT)) {
				//{easy switch then easy scale}
				final P_Bezier a = new P_Bezier(leftStartX, startY, -120, 108, -119, 167, -switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(-switchX, switchY, -117, 204, -88, 222, -cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(-cubeX, cubeY, -72.64, 224, -71.11, 286, -scaleX, scaleY, 2.0);//get to easy scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (switchTarget.equals(FieldPieceCharter.LEFT)) {
				//{easy switch then hard scale}
				final P_Bezier a = new P_Bezier(leftStartX, startY, -120, 108, -119, 167, -switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(-switchX, switchY, -117, 204, -88, 222, -cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(-cubeX, cubeY, -40, 278, 63, 228, scaleX, scaleY, 2.0);//get to hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (scaleTarget.equals(FieldPieceCharter.LEFT)) {
				//{easy scale then hard switch}
				final P_Bezier a = new P_Bezier(leftStartX, startY, -119, 206, -92, 238, -scaleX, scaleY, 0.0);//get to easy scale
				final P_Bezier b = new P_Bezier(-scaleX, scaleY, -5, 255, 79, 267, /*cubeX*/61.5, cubeY, 1.0);//get to new cube/hard switch
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else {
				//(hard switch then hard scale)TODO
			}
			
			break;
			
		case CENTER:initOdometerPosX = centerStartX;
		
			//easy switch then ____ scale
			//TODO this should be a 2 cube auto
			if (switchTarget.equals(FieldPieceCharter.RIGHT)) leash = bezier_center2switchRight();
			else leash = bezier_center2switchLeft();
			
			break;
		
		case RIGHT:initOdometerPosX = rightStartX;
		
			if (switchTarget.equals(FieldPieceCharter.RIGHT) && scaleTarget.equals(FieldPieceCharter.RIGHT)) {
				//{easy switch then easy scale}
				final P_Bezier a = new P_Bezier(rightStartX, startY, 93, 57, 132, 158, switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(switchX, switchY, 111, 195, 101, 220, cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(cubeX, cubeY, 72.64, 224, 71.11, 286, scaleX, scaleY, 2.0);//get to easy scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (switchTarget.equals(FieldPieceCharter.RIGHT)) {
				//{easy switch then hard scale}
				final P_Bezier a = new P_Bezier(rightStartX, startY, 93, 57, 132, 158, switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(switchX, switchY, 111, 195, 101, 220, cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(cubeX, cubeY, 40, 278, -63, 228, -scaleX, scaleY, 2.0);//get to hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (scaleTarget.equals(FieldPieceCharter.RIGHT)) {
				//{easy scale then hard switch}
				final P_Bezier a = new P_Bezier(rightStartX, startY, 124, 222, 77, 248, scaleX, scaleY, 0.0);//get to easy scale
				final P_Bezier b = new P_Bezier(scaleX, scaleY, 20, 259, -74, 260, /*-cubeX*/-61.5, cubeY, 1.0);//get to new cube/hard switch
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
			}else {
				//(hard switch then hard scale)TODO
			}
			
			break;
			
		default:initOdometerPosX = centerStartX;break;
		}
	}
	
	public V_Leash getLeash() {return leash;}
	

//	private static V_Leash bezier_sides2switchNearest() {
//		//							p0x   p0y    p1x   p1y   p2x  p2y   p3x   p3y  start
//		P_Bezier a = new P_Bezier(7.188, 13.32, 7.188, 40, 7.188, 90, 7.188, 138, 0.0);//inches
//		//Create an array of CubicBeziers; represents a full path.
//		P_Bezier[] path = new P_Bezier[] {a};
//		return new V_Leash(path, 3.0, 0.05);
//	}

	private static V_Leash bezier_sideRight2scaleRight() {
//		                           p0x     p0y   p1x   p1y  p2x  p2y    p3x     p3y   start
		P_Bezier a = new P_Bezier(67.188, 13.32, 91.8, 126, 108, 252, 129.336, 327.78, 0.0);
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideRight2scaleLeft() {
//                                 p0x     p0y   p1x  p1y  p2x  p2y    p3x    p3y    start
		P_Bezier a = new P_Bezier(67.188, 13.32, 84, 93.6, 99, 187.2, 67.188, 228.36, 0.0);
		P_Bezier b = new P_Bezier(67.188, 228.36, -75.36, 228.36, -121.8, 272.88, -129.336, 327.78, 1.0);
		P_Bezier[] path = new P_Bezier[] {a, b};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideLeft2scaleRight() {
//                                  p0x     p0y     p1x   p1y    p2x   p2y     p3x      p3y  start
		P_Bezier a = new P_Bezier(-91.812, 13.32, -117.6, 120, -124.8, 252, -129.336, 327.78, 0.0);
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideLeft2scaleLeft() {
//                                  p0x     p0y    p1x   p1y   p2x   p2y     p3x     p3y   start
		P_Bezier a = new P_Bezier(-91.812, 13.32, -108, 82.8, -120, 164.4, -91.812, 228.36, 0.0);
		P_Bezier b = new P_Bezier(-91.812, 228.36, 72, 228.36, 117.6, 265.2, 129.336, 327.78, 1.0);
		P_Bezier[] path = new P_Bezier[] {a, b};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sides2switchNearest() {
		//							p0x   p0y    p1x   p1y   p2x  p2y   p3x   p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 7.188, 40, 7.188, 90, 7.188, 138, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.05);
	}
	
//	private static V_Leash curve_center2scaleLeft() {
//		/*Define segments of the path using lambda functions for x and y, as well as a start and end value
//		for what might be thought of as time. Though the syntax makes it look like x and y are generated
//		from themselves, it is actually the time-like incrementer that gets passed in.*/
//		
//		P_Curve a = new P_Curve(x -> -4.0*(1.0 - Math.cos(2.0*x)),		y -> 7.5*Math.sin(y),	0.0, 1.0);
//		P_Curve b = new P_Curve(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,	y -> y*y + 5.311,	1.0, 3.8);
//		P_Curve c = new P_Curve(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,	y -> 5.0*Math.sin(y - 3.8) + 19.751,	3.8, 5.4);
//		//Create an array of segments; represents a full path.
//		P_Curve[] path = new P_Curve[] {a, b, c};
//		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
//	}
	
//	private static V_Leash curve_center2scaleRight() {
//		P_Curve a = new P_Curve(x -> 4.0*(1.0 - Math.cos(2.0*x)),		y -> 7.5*Math.sin(y),	0.0, 1.0);
//		P_Curve b = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> y*y + 5.311,	1.0, 3.8);
//		P_Curve c = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> 5.0*Math.sin(y - 3.8) + 19.751,	3.8, 5.4);
//		//Create an array of segments; represents a full path.
//		P_Curve[] path = new P_Curve[] {a, b, c};
//		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
//	}
	
	private static V_Leash bezier_center2scaleLeft() {
		/*Define cubic beziers for the path. Each one starts at (p0x, p0y) and ends at (p3x, p3y).
	    Therefore (p0x, p0y) should be the same as (p3x, p3y) of the previous segment. The
	    nSteps parameter defines how far we move along the path during the increment call.*/
	    //							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 0, 108, -138, 90, -138, 140, 0.0);//inches
		P_Bezier b = new P_Bezier(-138, 140, -139, 174, -138, 189, -138, 204, 1.0);
		P_Bezier c = new P_Bezier(-138, 204, -137, 252, -109, 272, -90, 279, 2.0);
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_center2scaleRight() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 12, 120, 138, 84, 138, 138, 0.0);//inches
		P_Bezier b = new P_Bezier(138, 138, 138, 156, 138, 192, 138, 210, 1.0);
		P_Bezier c = new P_Bezier(138, 210, 138, 228, 114, 264, 90, 279, 2.0);
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_center2switchLeft() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 6, 120, -132, 44, -108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_center2switchRight() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 8, 120, 132, 44, 108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
}
