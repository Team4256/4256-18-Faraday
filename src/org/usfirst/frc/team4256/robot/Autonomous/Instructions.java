package org.usfirst.frc.team4256.robot.Autonomous;

public class Instructions {
	
	private Leash leashA;
	
	public Instructions(final String gameData) {
		final boolean switchRight = gameData.charAt(0) == 'R';
		final boolean scaleRight = gameData.charAt(1) == 'R';
		
		if (scaleRight) {
			leashA = createLeash_origin2scaleRight();
		}else {
			leashA = createLeash_origin2scaleLeft();
		}
	}
	
	public Leash getLeash() {return leashA;}
	
	private static Leash createLeash_origin2scaleLeft() {
		/*Define segments of the path using lambda functions for x and y, as well as a start and end value
		for what might be thought of as time. Though the syntax makes it look like x and y are generated
		from themselves, it is actually the time-like incrementer that gets passed in.*/
		
		Segment a = new Segment(x -> -4.0*(1.0 - Math.cos(2.0*x)),
								y -> 7.5*Math.sin(y),
								0.0, 1.0);
		Segment b = new Segment(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,
								y -> y*y + 5.311,
								1.0, 3.8);
		Segment c = new Segment(x -> -2.0*(1.0 - Math.cos(x)) - 4.746,
								y -> 5.0*Math.sin(y - 3.8) + 19.751,
								3.8, 5.4);
		//Create an array of segments; represents a full path.
		Segment[] path = new Segment[] {a, b};
		return new Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static Leash createLeash_origin2scaleRight() {
		Segment a = new Segment(x -> 4.0*(1.0 - Math.cos(2.0*x)),
				  				y -> 7.5*Math.sin(y),
				  				0.0, 1.0);
		Segment b = new Segment(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,
				  				y -> y*y + 5.311,
				  				1.0, 3.8);
		Segment c = new Segment(x -> 2.0*(1.0 - Math.cos(x)) + 4.746,
								y -> 5.0*Math.sin(y - 3.8) + 19.751,
								3.8, 5.4);
		//Create an array of segments; represents a full path.
		Segment[] path = new Segment[] {a, b, c};
		return new Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
}
