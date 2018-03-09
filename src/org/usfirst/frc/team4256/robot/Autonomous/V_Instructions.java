package org.usfirst.frc.team4256.robot.Autonomous;

public class V_Instructions {
	
	private V_Leash leashA;
	public final boolean switchRight;
	public final boolean scaleRight;
	
	public V_Instructions(final String gameData) {
		switchRight = gameData.charAt(0) == 'R';
		scaleRight = gameData.charAt(1) == 'R';
		
//		if (scaleRight) leashA = curve_origin2scaleRight();
//		else leashA = curve_origin2scaleLeft();
		
//		if (switchRight) leashA = bezier_origin2switchRight();
//		else leashA = bezier_origin2switchLeft();
		
		leashA = bezier_origin2line();
	}
	
	public V_Leash getLeash() {return leashA;}
	
	private static V_Leash bezier_origin2line() {
		//							p0x   p0y    p1x   p1y   p2x  p2y   p3x   p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 7.188, 40, 7.188, 90, 7.188, 138, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.05);
	}
	
	private static V_Leash curve_origin2scaleLeft() {
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
	
	private static V_Leash curve_origin2scaleRight() {
		P_Curve a = new P_Curve(x -> 4.0*(1.0 - Math.cos(2.0*x)),		y -> 7.5*Math.sin(y),	0.0, 1.0);
		P_Curve b = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> y*y + 5.311,	1.0, 3.8);
		P_Curve c = new P_Curve(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,	y -> 5.0*Math.sin(y - 3.8) + 19.751,	3.8, 5.4);
		//Create an array of segments; represents a full path.
		P_Curve[] path = new P_Curve[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_origin2scaleLeft() {
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
	
	private static V_Leash bezier_origin2scaleRight() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 12, 120, 138, 84, 138, 138, 0.0);//inches
		P_Bezier b = new P_Bezier(138, 138, 138, 156, 138, 192, 138, 210, 1.0);
		P_Bezier c = new P_Bezier(138, 210, 138, 228, 114, 264, 90, 279, 2.0);
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a, b, c};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_origin2switchLeft() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 6, 120, -132, 44, -108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_origin2switchRight() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 8, 120, 132, 44, 108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	
}
