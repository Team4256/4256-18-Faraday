package org.usfirst.frc.team4256.robot.Autonomous;

public class V_Instructions {
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceCharter {LEFT, RIGHT};
	
	public final FieldPieceCharter switchTarget;
	public final FieldPieceCharter scaleTarget;
	public final StartingPosition startingPosition;
	
	private V_Leash leash;
	
	public V_Instructions(final String gameData, final StartingPosition startingPosition) {
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceCharter.LEFT : FieldPieceCharter.RIGHT;
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceCharter.LEFT : FieldPieceCharter.RIGHT;
		this.startingPosition = startingPosition;
		
		switch (this.startingPosition) {
		case LEFT:
			/*When starting on the left with the clamp facing right, there are four possible one-cube actions.
			  1) If the left side of the switch is assigned to us, drive forward and place cube there (no rotation).
			  2) If the left side of the scale is assigned to us, drive forward and place cube there (no rotation).
			  3) If everything assigned to us is on the right, just drive forward.//TODO could we get all the way around?
			  4) If the left side of the switch AND scale are assigned to us, drive forward and place cube in switch (according to Sam)*/
			
			if (switchTarget.equals(FieldPieceCharter.LEFT)) leash = bezier_sides2switchNearest();
			else if (switchTarget.equals(FieldPieceCharter.RIGHT)) leash = bezier_sides2switchNearest();//TODO should go farther to get to scale
			else leash = bezier_sides2switchNearest();//still drive forward, but don't do any spitting, etc.
			
		case CENTER:
			if (switchTarget.equals(FieldPieceCharter.RIGHT)) leash = bezier_center2switchRight();
			else leash = bezier_center2switchLeft();
//			if (scaleRight) leashA = curve_origin2scaleRight();
//			else leash = curve_origin2scaleLeft();
		
		case RIGHT:
			/*When starting on the right with the clamp facing right, there are four possible one-cube actions.
			  1) If the right side of the switch is assigned to us, drive forward and place cube there (180 rotation).
			  2) If the right side of the scale is assigned to us, drive forward and place cube there (180 rotation).
			  3) If everything assigned to us is on the left, just drive forward.//TODO could we get all the way around?
			  4) If the right side of the switch AND scale are assigned to us, drive forward and place cube in switch (180 rotation)*/
			
			if (switchTarget.equals(FieldPieceCharter.RIGHT)) leash = bezier_sides2switchNearest();
			else if (switchTarget.equals(FieldPieceCharter.LEFT)) leash = bezier_sides2switchNearest();//TODO should go farther to get to scale
			else leash = bezier_sides2switchNearest();//still drive forward, but don't do any spitting, etc.
		}
	}
	
	public V_Leash getLeash() {return leash;}
	
	private static V_Leash bezier_sides2switchNearest() {
		//							p0x   p0y    p1x   p1y   p2x  p2y   p3x   p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 7.188, 40, 7.188, 90, 7.188, 138, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.05);
	}
	
	private static V_Leash curve_center2scaleLeft() {
		/*Define segments of the path using lambda functions for x and y, as well as a start and end value
		for what might be thought of as time. Though the syntax makes it look like x and y are generated
		from themselves, it is actually the time-like incrementer that gets passed in.*/
		
		P_Curve a = new P_Curve(x -> -4.0*(1.0 - Math.cos(2.0*x)),		y -> 7.5*Math.sin(y),	0.0, 1.0);
		P_Curve b = new P_Curve(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,	y -> y*y + 5.311,	1.0, 3.8);
		P_Curve c = new P_Curve(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,	y -> 5.0*Math.sin(y - 3.8) + 19.751,	3.8, 5.4);
		//Create an array of segments; represents a full path.
		P_Curve[] path = new P_Curve[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash curve_center2scaleRight() {
		P_Curve a = new P_Curve(x -> 4.0*(1.0 - Math.cos(2.0*x)),		y -> 7.5*Math.sin(y),	0.0, 1.0);
		P_Curve b = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> y*y + 5.311,	1.0, 3.8);
		P_Curve c = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> 5.0*Math.sin(y - 3.8) + 19.751,	3.8, 5.4);
		//Create an array of segments; represents a full path.
		P_Curve[] path = new P_Curve[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
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
